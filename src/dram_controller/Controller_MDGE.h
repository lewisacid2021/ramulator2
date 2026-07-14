#pragma once
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>
#include "dram_controller/RequestQueue_DeFrag.h"
#include "addr_mapper/TranslationTable.h"

namespace Ramulator {

enum class MDGEState { IDLE, MONITOR, DECIDE, GATHER, EVACUATE };

// Test/scheduling hook (Task 12): a row pending defrag.
struct PendingDefrag { uint64_t orig_addr; int head_id; };

// Test/scheduling hook (Task 13): orig → sink destination for evacuation.
struct EvacuateTarget { uint64_t orig_addr; uint64_t sink_addr; };

struct MDGEConfig {
    int window_size = 256;
    int t_min = 64, t_max = 1024, t_init = 256;
    float alpha = 0.2f, beta = 0.1f;
    int k_hot = 2;
    int num_channels = 8, num_banks = 16;
    int num_attention_heads = 28;
    int t_idle = 32;  // cycles of decode-idle before triggering Gather (Task 11)
};

class Controller_MDGE {
public:
    void configure(const MDGEConfig& cfg);
    void tick(uint64_t cycle);
    void on_receive(Request& req);
    MDGEState get_state() const { return m_state; }
    RequestQueue_DeFrag* defrag_queue() { return &m_defrag_q; }
    TranslationTable* translation_table() { return m_tt; }
    double get_window_rcr() const { return m_window_rcr; }
    int get_current_t() const { return m_t_current; }
    // Selection bridge: number of rows currently queued for compaction.
    // Exposed for tests + e2e gates. Production reads m_pending via
    // gather phase drain; tests need to assert it advances from 0.
    int get_pending_defrag_count() const { return m_pending_defrag_count; }
    void record_compaction_result(double savings, double cost);

    // P0 Fix #1: select_defrag_candidates()
    // Bridges Monitor → Decide. Iterates m_row_access_count from the
    // just-closed window and pushes rows exceeding the hotness threshold
    // into m_pending. Called at the window boundary inside tick().
    //
    // NOTE: this is SELECTION (which rows to defrag), not ROUTING. The
    // routing topology (k_hot bank groups per channel) is independent
    // and unchanged.
    void select_defrag_candidates();
    // CGBC sink routing (Task 14): pick channel/bank for an evacuation sink.
    // NOTE(brief-deviation): brief's verbatim code uses CH_SHIFT=16, but the
    // accompanying test asserts `(sink >> 8) % 8 == channel`, which requires
    // CH_SHIFT=8. The test design (channel at bits 8-10) is authoritative;
    // BANK_SHIFT=12 still matches the brief.
    uint64_t compute_cgbc_sink(uint64_t orig_row_addr, int head_id, int window_id);

    // Test hooks (Task 12): force state and add pending defrag entry.
    void force_state(MDGEState s) { m_state = s; }
    void add_pending_defrag(PendingDefrag p) {
        m_pending.push_back(p);
        m_pending_defrag_count++;
    }
    // Test hook (Task 13): add an evacuate destination.
    void add_evacuate_target(EvacuateTarget t) {
        m_evacuate_targets.push_back(t);
    }

private:
    MDGEConfig m_cfg;
    MDGEState m_state = MDGEState::IDLE;
    RequestQueue_DeFrag m_defrag_q;
    // NOTE(brief-deviation): TranslationTable has no default ctor (only the
    // explicit `TranslationTable(size_t)` form), but Controller_MDGE itself
    // needs a default ctor (brief test uses `Ramulator::Controller_MDGE mdge;`).
    // Allocate on the heap; configure() re-initializes with real capacity.
    std::unique_ptr<TranslationTable> m_tt_ptr;
    TranslationTable* m_tt = nullptr;  // convenience accessor
    int m_t_current;

    // Monitor phase (Task 10): per-window RCR + per-row access count
    int m_window_count = 0;
    uint64_t m_prev_row_addr = 0;
    int m_row_transitions = 0;
    int m_window_rcr = 0;
    std::unordered_map<uint64_t, int> m_row_access_count;
    // Forward-declared for Task 11 (Decide trigger conditions)
    int m_pending_defrag_count = 0;
    int m_decode_idle_cycles = 0;

    // Gather phase (Task 12): pending rows + expected gather counter
    std::vector<PendingDefrag> m_pending;
    int m_gather_expected = 0;  // expected drains before EVACUATE

    // Evacuate phase (Task 13): orig→sink targets and completed evacs
    std::vector<EvacuateTarget> m_evacuate_targets;
    std::vector<EvacuateTarget> m_completed_evacs;
    int m_evacuate_expected = 0;  // expected drains before IDLE
};

}  // namespace Ramulator