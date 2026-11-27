#pragma once

#include <cstddef>
#include <cstdint>

namespace drawn_pattern {

// Start and end are thousandths of the authored (unrotated) cycle.
struct Modifier {
    uint16_t start, end, holdMs;
    bool pause() const { return start == end; }
};
static_assert(sizeof(Modifier) == 6);

struct Modifiers {
    static constexpr size_t capacity = 128;
    uint16_t count = 0;
    Modifier entries[capacity]{};

    static bool valid(const Modifier *items, size_t size) {
        if (size > capacity || (size && !items)) return false;
        for (size_t i = 0; i < size; ++i) {
            const auto &current = items[i];
            if (current.start >= 1000 || current.end > 1000 || current.start > current.end) return false;
            if (current.pause() ? (!current.holdMs || current.holdMs > 10000) : current.holdMs != 0)
                return false;
            if (i) {
                const auto &previous = items[i - 1];
                if (current.start < previous.end ||
                    (current.start == previous.end && (current.pause() || previous.pause()))) return false;
            }
        }
        // A point at zero also occupies the end of a range ending at 100%.
        return size < 2 || !items[0].pause() || items[0].start != 0 || items[size - 1].end != 1000;
    }

    bool valid() const { return valid(entries, count); }
};

inline const Modifiers &noModifiers() {
    static const Modifiers empty{};
    return empty;
}

}  // namespace drawn_pattern
