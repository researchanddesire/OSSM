#pragma once

#include <atomic>
#include <cstdint>

// OneButton runs on the button task. Publish click boundaries so a slower
// screen can discard turns even if the whole click happened between reads.
class EncoderClick {
    std::atomic<uint32_t> sequence{0};
public:
    // Single writer: the task that calls OneButton::tick().
    void update(bool idle) {
        const auto previous = sequence.load();
        if (isIdle(previous) != idle) sequence.store(previous + 1);
    }

    uint32_t snapshot() const { return sequence.load(); }
    static bool isIdle(uint32_t value) { return (value & 1u) == 0; }
    bool unchangedAndIdle(uint32_t value) const {
        return isIdle(value) && value == snapshot();
    }
};
