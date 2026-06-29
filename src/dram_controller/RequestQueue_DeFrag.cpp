#include "dram_controller/RequestQueue_DeFrag.h"

namespace Ramulator {

void RequestQueue_DeFrag::enqueue(const Request& req, Priority p) {
    if (p == Priority::FOREGROUND) {
        m_fg.push_back(req);
        return;
    }
    // BACKGROUND: respect slot limit (default = 8 for 8ch topology)
    if (m_bg.size() >= m_max_bg) return;  // slot full, silently drop
    m_bg.push_back({req, m_current_cycle});
}

bool RequestQueue_DeFrag::dequeue(Request& req) {
    if (!m_fg.empty()) { req = m_fg.front(); m_fg.pop_front(); return true; }
    if (!m_bg.empty()) { req = m_bg.front().first; m_bg.pop_front(); return true; }
    return false;
}

}  // namespace Ramulator
