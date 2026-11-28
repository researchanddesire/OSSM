#pragma once

#include <cstdint>

namespace drawn_pattern {
// Only already prepared motor targets cross this boundary. The end phase of
// each row stays in the software FIFO until its individual acceptance.
struct PreparedPulses {
    // A visit starts with one hardware horizon of source rows. The projection
    // also carries one horizon of replacements for intervals that elapse
    // inside the guarded motor writes. This is RAM, not a motor queue.
    static constexpr unsigned horizonRows = 8;
    static constexpr unsigned capacity = 2 * horizonRows - 1;
    struct Target {
        int32_t position;
        uint16_t crossing;
        uint8_t followingHolds = 0;
        bool stationary = false;
    };
    Target targets[capacity]{};
    unsigned count = 0;
    // Index of the first row calculated with a pending positive Speed edit.
    // Keep its hardware acceptance independently timestampable.
    unsigned rateBoundary = capacity + 1;

    // Every control or gain-goal change retains its own accepted timestamp.
    // A pause or fade endpoint can also require the controller to replan
    // before offering the next row.
    uint16_t gainBoundaries = 0;
    unsigned initialRows = horizonRows;
    bool boundaryAt(unsigned row) const {
        return row == rateBoundary || ((gainBoundaries >> row) & 1u);
    }

    void groupWaits(Target &first) {
        // Two adjacent stationary intervals can share one hardware WAIT.
        // Do not coalesce through a changing position, gain or law.
        for (unsigned i = count; i > 0; --i) {
            if (boundaryAt(i)) continue;
            auto &row = i == 1 ? first : targets[i - 2];
            const auto &next = targets[i - 1];
            if (row.stationary && next.stationary && row.position == next.position)
                row.followingHolds = next.followingHolds ? 2 : 1;
        }
    }
};
}
