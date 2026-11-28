#pragma once

#include <cstdint>

namespace drawn_pattern {
// Only already prepared motor targets cross this boundary. The end phase of
// each row stays in the software FIFO until its individual acceptance.
struct PreparedPulses {
    // Bounded by one existing 16 ms service visit. Releasing the motor lock
    // at a fixed three rows can strand an otherwise ready FIFO at 1.7 ms.
    static constexpr unsigned capacity = 7;
    struct Target {
        int32_t position;
        uint16_t crossing;
        uint8_t followingHolds = 0;
        bool stationary = false;
    };
    Target targets[capacity]{};
    unsigned count = 0;

    void groupWaits(Target &first) {
        // Two adjacent stationary intervals can share one hardware WAIT.
        // Do not coalesce through a changing position, gain or law.
        for (unsigned i = count; i > 0; --i) {
            auto &row = i == 1 ? first : targets[i - 2];
            const auto &next = targets[i - 1];
            if (row.stationary && next.stationary && row.position == next.position)
                row.followingHolds = next.followingHolds ? 2 : 1;
        }
    }
};
}
