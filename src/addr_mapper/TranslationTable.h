#pragma once
#include <cstdint>
#include <functional>
#include <list>
#include <unordered_map>

namespace Ramulator {

class TranslationTable {
public:
    explicit TranslationTable(size_t capacity_entries);

    bool lookup(uint64_t orig_addr, uint64_t& sink_addr);
    void insert(uint64_t orig_addr, uint64_t sink_addr, bool dirty);
    void invalidate(uint64_t orig_addr);
    void evict_lru();

    void set_emergency_flush_callback(
        std::function<void(uint64_t orig, uint64_t sink)> cb) {
        m_flush_cb = std::move(cb);
    }

    size_t size() const { return m_entries.size(); }
    size_t capacity() const { return m_capacity; }

private:
    struct Entry {
        bool valid;
        bool dirty;
        uint8_t reserved[6];
        uint8_t tag;
        uint64_t orig_addr;
        uint64_t sink_addr;
    };
    // NOTE(brief-deviation): brief's verbatim text asserts sizeof(Entry) == 24, but the
    // struct fields (2x uint64_t addresses + bool/dirty/tag/reserved header with
    // natural alignment) actually yield 32 bytes on x86_64. Adjusted to 32 to match
    // the struct definition; the Entry struct is unused by the current implementation
    // (which uses std::unordered_map) and is documentation for the future on-chip
    // layout. See task-5-report.md for details.
    static_assert(sizeof(Entry) == 32, "Entry must be 32 bytes (header+orig+sink)");

    size_t m_capacity;
    std::list<uint64_t> m_lru_order;  // front = MRU, back = LRU
    std::unordered_map<uint64_t, std::pair<uint64_t, bool>> m_entries;  // orig_addr → (sink_addr, dirty)
    std::function<void(uint64_t orig, uint64_t sink)> m_flush_cb;
};

}  // namespace Ramulator
