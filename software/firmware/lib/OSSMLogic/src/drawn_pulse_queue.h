#pragma once

#include <atomic>
#include <cstdint>

#include "drawn_waveform.h"

namespace drawn_pattern {
enum class Frame { Accepted, Retry, Rejected };
struct FrameResult {
    Frame state;
    const char *reason = nullptr;
    int8_t driverResult = 0;
};

constexpr unsigned pulseQueueFrames = 6;

// FAS may retire a leading pause in its first hardware interrupt. Fill the
// stopped queue, start it once, and use append-only writes for every refill.
class PulseQueue {
    enum class Stage : uint8_t { Idle, Filling, Running };
    std::atomic<Stage> stage_{Stage::Idle};
    int32_t initialPosition_ = 0;
    bool direction_ = false;

public:
    bool owned() const { return stage_.load() != Stage::Idle; }
    bool filling() const { return stage_.load() == Stage::Filling; }
    int32_t initialPosition() const { return initialPosition_; }
    void reset() { stage_ = Stage::Idle; }

    template <class Command, class Driver>
    FrameResult submit(Driver &driver, int32_t target, uint32_t frameTicks) {
        const bool running = stage_.load() == Stage::Running;
        if (running && !driver.isQueueRunning()) return {Frame::Rejected, "pulse_underrun"};
        const auto count = driver.queueEntries();
        if (count >= pulseQueueFrames) return {Frame::Retry, "pulse_full"};
        const auto from = driver.getPositionAfterCommandsCompleted();
        PulseFrame frame;
        if (!pulseFrame(from, target, frameTicks, direction_, frame))
            return {Frame::Rejected, "pulse_range"};
        if (!owned()) {
            initialPosition_ = from;
            stage_ = Stage::Filling;
        }
        Command command{frame.ticks, frame.steps, frame.increasing};
        const auto added = static_cast<int8_t>(driver.addQueueEntry(&command, false));
        if (added > 0) return {Frame::Retry, "pulse_append", added};
        if (added < 0) return {Frame::Rejected, "pulse_append", added};
        if (driver.getPositionAfterCommandsCompleted() != target ||
            (!running && driver.queueEntries() <= count))
            return {Frame::Rejected, "pulse_discarded", added};
        direction_ = frame.increasing;
        if (running) {
            // An interrupt can drain the queue between the first check and
            // our append. The unstarted straggler is flushed during Stop.
            if (!driver.isQueueRunning()) return {Frame::Rejected, "pulse_underrun"};
        } else if (driver.queueEntries() >= pulseQueueFrames) {
            const auto started = static_cast<int8_t>(driver.addQueueEntry(nullptr, true));
            if (started != 0) return {Frame::Rejected, "pulse_start", started};
            stage_ = Stage::Running;
        }
        return {Frame::Accepted};
    }
};
}
