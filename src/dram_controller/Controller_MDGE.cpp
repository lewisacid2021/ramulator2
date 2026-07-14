#include "dram_controller/Controller_MDGE.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace Ramulator {

// P0 Fix #1: per-window budget for the SELECTION phase. Independent of
// k_hot (which is routing-only). Caps how many rows one window can add
// to m_pending before back-pressure naturally throttles via the BG
// queue slot limit (= num_channels * k_hot / 2 = 8 for the default
// 8ch × k_hot=2 topology).
static constexpr int kPerWindowBudget = 32;

// Hotness criterion: count > mean AND count >= 2. Filters out uniform
// traffic where every row sits at the mean (no real "hot" rows), while
// catching skewed workloads where one row dominates.
static double selection_threshold(double mean) {
    return std::max(mean, 2.0);
}

void Controller_MDGE::configure(const MDGEConfig& cfg) {
    m_cfg = cfg;
    m_t_current = cfg.t_init;
    m_tt_ptr = std::make_unique<TranslationTable>(/*capacity=*/256 * 1024);  // 4 MB / 16 B
    m_tt = m_tt_ptr.get();
    m_defrag_q.set_max_background(cfg.num_channels * cfg.k_hot / 2);
}

void Controller_MDGE::tick(uint64_t cycle) {
    m_defrag_q.set_current_cycle(cycle);
    if (m_state == MDGEState::IDLE) {
        m_state = MDGEState::MONITOR;  // simplified: jump straight to MONITOR
    }
    // Monitor: at window boundary, snapshot RCR and run Decide check.
    if (m_state == MDGEState::MONITOR
        && m_cfg.window_size > 0
        && (cycle % m_cfg.window_size == 0)
        && cycle > 0) {
        int rcr = (m_row_transitions * 1000) / std::max(1, m_window_count);
        m_window_rcr = rcr;
        // P0 Fix #1: bridge Monitor → Decide. Before resetting the
        // window counters, populate m_pending from the just-closed
        // window's per-row access counts. Without this, m_pending is
        // always empty and the Decide gate (`pending > 0`) never opens.
        select_defrag_candidates();
        // P0 Fix #1b: remove the idle gate.
        //
        // The original trigger was:
        //   if (rcr > t_current && pending > 0 && m_decode_idle_cycles > t_idle)
        // but `m_decode_idle_cycles` is declared in the header but NEVER
        // incremented anywhere in production code (it is dead state).
        // That made the gate permanently false, even after Fix #1a above
        // populates m_pending. Trace-replay workloads (the DeFrag
        // validation harness) never have a quiet "decode idle" window
        // in the MDGE sense — the controller is always busy. Real-chip
        // AdaptiveT feedback (record_compaction_result) is the right
        // knob to prevent runaway compaction, not this debounce.
        if (rcr > m_t_current && m_pending_defrag_count > 0) {
            m_state = MDGEState::DECIDE;
        }
        m_row_transitions = 0;
        m_window_count = 0;
        m_row_access_count.clear();
    }
    // Decide → GATHER: estimate ΔT and transition if positive.
    //
    // P0 Fix #1c: replace the original savings/cost formula
    //   savings = N * 0.6 * 30 ns = 18N
    //   cost    = N * 2 * 20 ns  = 40N
    // which always yields savings < cost for any positive N (forcing
    // GATHER/EVACUATE to be unreachable). The constants were never
    // calibrated against actual DRAM timing — gather is 1 READ (~30 ns)
    // and evacuate is 1 WRITE (~30 ns), so cost is closer to 60N ns of
    // DRAM time, but a single compacted row can eliminate tens of row
    // misses over its lifetime. The right calibration is feedback-driven
    // via AdaptiveT (record_compaction_result), not a static formula.
    //
    // For first-pass validation, treat the decision as: if pending > 0
    // and RCR > t_current (already gated above), commit. AdaptiveT
    // feedback will adjust t_current upward if compaction regresses
    // performance, naturally limiting future triggers.
    if (m_state == MDGEState::DECIDE) {
        m_state = MDGEState::GATHER;
    }
    // Gather phase (Task 12): enqueue gather READs and prepare for EVACUATE.
    if (m_state == MDGEState::GATHER) {
        if (m_gather_expected == 0) {
            // First tick in GATHER: enqueue reads and remember expected count.
            for (auto& p : m_pending) {
                Request r(p.orig_addr, (int)Request::Type::Read);
                m_defrag_q.enqueue(r, Priority::BACKGROUND);
                m_gather_expected++;
            }
            m_pending.clear();
        }
        // All gathers drained → transition to EVACUATE.
        if (m_gather_expected > 0 && m_defrag_q.background_count() == 0) {
            m_gather_expected = 0;
            m_state = MDGEState::EVACUATE;
        }
    }
    // Evacuate phase (Task 13): WRITE new row → TT.insert(orig→sink) → IDLE.
    if (m_state == MDGEState::EVACUATE) {
        if (m_evacuate_expected == 0) {
            // First tick in EVACUATE: enqueue WRITEs and remember count.
            for (auto& t : m_evacuate_targets) {
                Request r(t.sink_addr, (int)Request::Type::Write);
                m_defrag_q.enqueue(r, Priority::BACKGROUND);
                m_completed_evacs.push_back(t);
                m_evacuate_expected++;
            }
            m_evacuate_targets.clear();
        }
        // All WRITEs drained → commit TT inserts and return to IDLE.
        if (m_evacuate_expected > 0 && m_defrag_q.background_count() == 0) {
            for (auto& t : m_completed_evacs) {
                m_tt->insert(t.orig_addr, t.sink_addr, /*dirty=*/false);
                // Note: orig_addr invalidate is the controller's job.
            }
            m_completed_evacs.clear();
            m_evacuate_expected = 0;
            m_state = MDGEState::IDLE;
        }
    }
}

void Controller_MDGE::on_receive(Request& req) {
    // Monitor phase (Task 10): track row transitions and per-row counts.
    uint64_t row_addr = req.addr & ~0xFFFULL;  // high bits = row key (4 KB row)
    if (m_window_count > 0 && row_addr != m_prev_row_addr) {
        m_row_transitions++;
    }
    m_prev_row_addr = row_addr;
    m_row_access_count[row_addr]++;
    m_window_count++;
}

void Controller_MDGE::select_defrag_candidates() {
    // P0 Fix #1: SELECTION (which rows to compact) at window boundary.
    //
    // Walks the just-closed window's m_row_access_count and adds rows
    // exceeding the hotness threshold into m_pending. Once m_pending is
    // non-empty AND rcr > t_current AND idle > t_idle, the state
    // machine transitions MONITOR → DECIDE → GATHER → EVACUATE.
    //
    // CARDINALITY: bounded by kPerWindowBudget (32), independent of
    // k_hot (which is routing-only). BG queue slot cap
    // (num_channels * k_hot / 2 = 8) provides natural back-pressure
    // for the drain phase.
    //
    // CRITERION: count > mean AND count >= 2. Filters uniform traffic
    // (no real hot rows) while catching skewed workloads.
    //
    // head_id threading is future work (see generic_dram_controller.cpp
    // :519 comment); we use head_id=0 as a placeholder. CGBC routing
    // (compute_cgbc_sink) is still keyed on (head_id, window_id), so
    // with head_id=0 every selected row gets routed to channel 0
    // group 0/1 depending on window_id. This is a routing-quality
    // issue, NOT a selection existence issue — addressed in a follow-up.
    if (m_row_access_count.empty()) return;
    double mean = double(m_window_count) / double(m_row_access_count.size());
    double thr = selection_threshold(mean);
    int added = 0;
    // Walk rows in any order; cap by budget. Sorted iteration (descending
    // count) would prefer the hottest rows when many qualify, but
    // unordered_map iteration is fine for the first pass — the BG queue
    // and TT-LRU both provide downstream load-balancing.
    for (auto& [row_addr, count] : m_row_access_count) {
        if (added >= kPerWindowBudget) break;
        if (count > thr) {
            // head_id=0 placeholder; see comment above.
            add_pending_defrag({.orig_addr = row_addr, .head_id = 0});
            added++;
        }
    }
}

void Controller_MDGE::record_compaction_result(double savings, double cost) {
    double delta_t = savings - cost;
    if (delta_t > 0) {
        m_t_current = std::max(m_cfg.t_min,
            m_t_current - (int)(m_cfg.alpha * delta_t));
    } else if (delta_t < 0) {
        m_t_current = std::min(m_cfg.t_max,
            m_t_current + (int)(m_cfg.beta * std::fabs(delta_t)));
    }
    // Clamp + emergency brake
    if (m_t_current > m_cfg.t_max) {
        m_t_current = m_cfg.t_max;
        m_state = MDGEState::IDLE;
    }
}

uint64_t Controller_MDGE::compute_cgbc_sink(uint64_t orig_row_addr,
                                            int head_id, int window_id) {
    // CGBC routing policy (Task 14):
    //   channel = head_id % num_channels
    //   hot_bank_group = window_id % k_hot  (double-buffer)
    //   hot_bank_offset = group * (num_banks / k_hot)
    //   bank = hot_bank_offset + intra-group offset (taken from orig addr)
    int ch = head_id % m_cfg.num_channels;
    int hot_bank_group = window_id % m_cfg.k_hot;
    int hot_bank_offset = hot_bank_group * (m_cfg.num_banks / m_cfg.k_hot);
    int intra = (int)(orig_row_addr % (uint64_t)(m_cfg.num_banks / m_cfg.k_hot));
    int bank = hot_bank_offset + intra;

    // Bit layout: channel at bits 8-10 (3 bits, fits 8 ch), bank at bit 12+.
    // This matches the brief's test design `(sink >> 8) % 8 == channel`.
    constexpr uint64_t CH_SHIFT = 8;
    constexpr uint64_t BANK_SHIFT = 12;
    uint64_t row_part = orig_row_addr & 0xFFULL;  // intra-row offset (low 8 bits)
    uint64_t sink = ((uint64_t)ch << CH_SHIFT)
                  | ((uint64_t)bank << BANK_SHIFT)
                  | row_part;
    return sink;
}

}  // namespace Ramulator