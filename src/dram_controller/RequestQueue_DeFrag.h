#pragma once
#include <deque>
#include <limits>
#include <utility>
#include "base/request.h"

namespace Ramulator {

enum class Priority { FOREGROUND, BACKGROUND };

class RequestQueue_DeFrag {
public:
    void enqueue(const Request& req, Priority p);
    bool dequeue(Request& req);
    size_t size() const { return m_fg.size() + m_bg.size(); }
    size_t background_count() const { return m_bg.size(); }
    void clear() { m_fg.clear(); m_bg.clear(); }

    // Slot limit + starvation tracking (Task 8)
    void set_max_background(size_t n) { m_max_bg = n; }
    void set_current_cycle(uint64_t c) { m_current_cycle = c; }
    uint64_t max_wait_cycles() const {
        if (m_bg.empty()) return 0;
        return m_current_cycle - m_bg.front().second;
    }

private:
    std::deque<Request> m_fg;
    std::deque<std::pair<Request, uint64_t>> m_bg;  // (request, enqueue_cycle)
    size_t m_max_bg = std::numeric_limits<size_t>::max();
    uint64_t m_current_cycle = 0;
};

}  // namespace Ramulator
