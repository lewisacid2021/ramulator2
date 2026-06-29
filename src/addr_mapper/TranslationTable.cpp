#include "addr_mapper/TranslationTable.h"

namespace Ramulator {

TranslationTable::TranslationTable(size_t capacity_entries)
    : m_capacity(capacity_entries) {}

bool TranslationTable::lookup(uint64_t orig_addr, uint64_t& sink_addr) {
    auto it = m_entries.find(orig_addr);
    if (it == m_entries.end()) return false;
    sink_addr = it->second.first;
    // Move to MRU
    m_lru_order.remove(orig_addr);
    m_lru_order.push_front(orig_addr);
    return true;
}

void TranslationTable::insert(uint64_t orig_addr, uint64_t sink_addr, bool dirty) {
    if (m_entries.size() >= m_capacity) evict_lru();
    m_entries[orig_addr] = {sink_addr, dirty};
    m_lru_order.push_front(orig_addr);
}

void TranslationTable::invalidate(uint64_t orig_addr) {
    m_entries.erase(orig_addr);
    m_lru_order.remove(orig_addr);
}

void TranslationTable::evict_lru() {
    if (m_lru_order.empty()) return;
    uint64_t lru_orig = m_lru_order.back();
    auto it = m_entries.find(lru_orig);
    if (it != m_entries.end() && it->second.second /*dirty*/ && m_flush_cb) {
        m_flush_cb(lru_orig, it->second.first);
    }
    m_lru_order.pop_back();
    m_entries.erase(lru_orig);
}

}  // namespace Ramulator
