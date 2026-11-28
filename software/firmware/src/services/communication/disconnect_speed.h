#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <optional>

namespace motion_link {

// start(), next() and cancel() are serialized with motion ownership and
// remote speed writes. The loop may read active() without that lock.
class DisconnectSpeed {
    std::atomic<bool> active_{false};
    uint32_t started_ = 0;
    float from_ = 0;

public:
    bool active() const { return active_.load(std::memory_order_relaxed); }
    void cancel() { active_.store(false, std::memory_order_relaxed); }

    // A true result requests an immediate zero for a stopped effective output.
    bool start(uint32_t now, float effective, std::optional<float> remote) {
        const float cap = remote.value_or(0);
        if (!(cap > 0 && effective > 0)) {
            cancel();
            return cap > 0;
        }
        // Another observer leaving cannot give an existing shutdown more time.
        if (active()) return false;
        started_ = now;
        from_ = std::min(effective, cap);
        active_.store(true, std::memory_order_relaxed);
        return false;
    }

    std::optional<float> next(uint32_t now, std::optional<float> remote) {
        if (!active()) return {};
        if (!(remote.value_or(0) > 0)) {
            cancel();
            return {};
        }
        const uint32_t elapsed = now - started_;
        if (elapsed < 1000) return {};
        if (elapsed >= 3000) {
            cancel();
            return 0;
        }
        const double progress = (elapsed - 1000) / 2000.;
        const double eased = .5 * (1 + std::sin(3.1415926 * (progress - .5)));
        const float value = std::min(*remote, static_cast<float>(std::floor(from_ * (1 - eased))));
        if (value == 0) cancel();
        return value;
    }
};
}  // namespace motion_link
