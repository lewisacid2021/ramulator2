#include "dram_controller/Controller_MDGE.h"
#include <algorithm>
#include <cmath>

namespace Ramulator {

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
        // Decide trigger: RCR > T AND pending > 0 AND idle > t_idle
        if (rcr > m_t_current && m_pending_defrag_count > 0
            && m_decode_idle_cycles > m_cfg.t_idle) {
            m_state = MDGEState::DECIDE;
        }
        m_row_transitions = 0;
        m_window_count = 0;
    }
    // Decide: estimate ΔT and transition to GATHER if positive.
    if (m_state == MDGEState::DECIDE) {
        // Simplified ΔT estimate: row_miss_reduction = 0.6, t_miss_penalty = 30 ns
        double savings = m_pending_defrag_count * 0.6 * 30.0;
        double cost = m_pending_defrag_count * 2 * 20.0;  // gather + evacuate
        if (savings - cost > 0) m_state = MDGEState::GATHER;
        else m_state = MDGEState::IDLE;
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