#pragma once

#include <atomic>
#include <cstdint>

namespace staging_probe {
// One word captures availability and its generation. A quick transition out
// and back invalidates an in-flight probe just as a sustained transition does.
class Gate {
    static constexpr uint32_t menu = 1, peer = 2;
    std::atomic<uint32_t> state_{0};

    void update(uint32_t bit, bool set) {
        uint32_t before = state_.load();
        while (bool(before & bit) != set) {
            const uint32_t after = ((before + 4) & ~bit) | (set ? bit : 0);
            if (state_.compare_exchange_weak(before, after)) return;
        }
    }
public:
    void menuIdle(bool idle) { update(menu, idle); }
    void bleConnected(bool connected) { update(peer, connected); }
    bool menuIsIdle() const { return (state_.load() & menu) != 0; }
    uint32_t ticket() const {
        const auto value = state_.load();
        return (value & (menu | peer)) == menu ? value : 0;
    }
    bool current(uint32_t ticket) const { return ticket != 0 && ticket == state_.load(); }

    // Only the observer task calls open and close. A mode or peer callback
    // invalidates the ticket immediately, without touching the TLS socket.
    template <class Open, class Close>
    int connect(uint32_t ticket, Open open, Close close) const {
        if (!current(ticket)) return 0;
        const int result = open();
        if (current(ticket)) return result;
        close();
        return 0;
    }
};

constexpr uint32_t intervalMs = 30000;
inline bool due(uint32_t previous, uint32_t now) {
    return previous == 0 || now - previous >= intervalMs;
}
}
