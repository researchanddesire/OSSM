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

constexpr unsigned pulseQueueFrames = 10;
constexpr uint32_t pulseStopDrainMs = 25;
constexpr unsigned queuedDurationMs = 16;
constexpr unsigned maxGroupMs = 2*pulseFrameMs;
// ticksInQueue omits the current entry. Allow a second active hardware entry
// and both direction guard pauses (up to 0.4 ms) in the existing stop budget.
static_assert(queuedDurationMs + 2*maxGroupMs < pulseStopDrainMs);
// Reserve two additional frame intervals inside the existing stop deadline
// for hardware buffering, direction setup and timer rounding.
static_assert((pulseQueueFrames + 2) * pulseFrameMs < pulseStopDrainMs);

// Raw pulses can leave FAS's last finite target behind the drained queue.
// Its idle forceStop flag would then consume the next move to that old target.
// Resetting an empty hardware queue also cancels an as-yet-unpublished ramp.
template <class Driver>
void stopPulseRamp(Driver &driver) {
    if (driver.isQueueRunning() || !driver.isQueueEmpty()) driver.forceStop();
    else driver.forceStopAndNewPosition(driver.getCurrentPosition());
}

// FAS may retire a leading pause in its first hardware interrupt. Fill the
// stopped queue, start it once, and use append-only writes for every refill.
class PulseQueue {
    enum class Stage : uint8_t { Idle, Filling, Running };
    std::atomic<Stage> stage_{Stage::Idle};
    int32_t initialPosition_ = 0;
    bool direction_ = false;
    int32_t pendingTarget_ = 0;
    uint32_t pendingOffset_ = 0;
    int32_t tickResidual_ = 0;

public:
    bool owned() const { return stage_.load() != Stage::Idle; }
    bool filling() const { return stage_.load() == Stage::Filling; }
    int32_t initialPosition() const { return initialPosition_; }
    void reset() { stage_ = Stage::Idle; }

    // A moving group's first electrical edge is immediate in moveTimed().
    // Start that group at the outgoing half-step crossing. Its duration runs
    // to the first crossing of the following frame. A hold uses the midpoint
    // of its frame. The one pending group is software only and drops on Stop.
    template <class Command, class Driver>
    FrameResult submit(Driver &driver, int32_t target, uint32_t frameTicks,
                       uint16_t firstCrossing, uint16_t minCommandTicks,
                       int8_t movedFromEmpty) {
        const bool first = !owned();
        const bool running = stage_.load() == Stage::Running;
        if (running && !driver.isQueueRunning()) return {Frame::Rejected, "pulse_underrun"};
        const auto count = driver.queueEntries();
        if (count >= pulseQueueFrames) return {Frame::Retry, "pulse_full"};
        const auto from = driver.getPositionAfterCommandsCompleted();
        const auto prior = first ? from : pendingTarget_;
        const int64_t delta = static_cast<int64_t>(prior)-from;
        const int64_t upcoming = static_cast<int64_t>(target)-prior;
        if (std::abs(delta)>255 || std::abs(upcoming)>255 || !frameTicks || frameTicks>32767)
            return {Frame::Rejected, "pulse_range"};
        const uint32_t offset = upcoming ? (uint32_t(firstCrossing)*frameTicks + 32767)/65535 : frameTicks/2;
        const int32_t requested = first ? static_cast<int32_t>(offset) :
            static_cast<int32_t>(frameTicks+offset-pendingOffset_)+tickResidual_;
        const int32_t steps = std::max<int32_t>(1, std::abs(delta));
        // FAS checks the minimum duration after dividing and truncating ticks
        // per edge. Return all quantization/clipping error to the next frame.
        const int32_t minimum = steps*std::max<int32_t>(driver.getMaxSpeedInTicks(),
            (minCommandTicks+steps-1)/steps);
        const uint32_t duration = std::max(requested,minimum);
        if (duration>2*frameTicks+255 || duration/steps>65535)
            return {Frame::Rejected, "pulse_range"};
        const uint32_t queuedLimit = frameTicks*queuedDurationMs/pulseFrameMs;
        if (count>=2 && driver.ticksInQueue()+duration>queuedLimit) {
            if (!running) {
                const auto started=startQueue(driver);
                if (started.state==Frame::Rejected) return started;
            }
            return {Frame::Retry, "pulse_full"};
        }
        if (first) {
            initialPosition_ = from;
            direction_ = false;
            stage_ = Stage::Filling;
        }
        uint32_t actual = duration;
        int8_t added;
        if (delta) {
            added = static_cast<int8_t>(driver.moveTimed(static_cast<int16_t>(delta),duration,&actual,false));
            // This positive code reports an accepted insertion into an empty
            // driver queue. A live queue must still pass the underrun check.
            if (added==movedFromEmpty) added=0;
        } else {
            // moveTimed(0,...) forces count_up=true and may toggle DIR during
            // a negative run. A direction-preserving timed hold avoids that.
            Command hold{static_cast<uint16_t>(duration),0,direction_};
            added = static_cast<int8_t>(driver.addQueueEntry(&hold,false));
        }
        if (added > 0) {
            if (first) reset();
            return {Frame::Retry, "pulse_append", added};
        }
        if (added < 0) return {Frame::Rejected, "pulse_append", added};
        if (driver.getPositionAfterCommandsCompleted() != prior ||
            (!running && driver.queueEntries() <= count))
            return {Frame::Rejected, "pulse_discarded", added};
        if (delta) direction_=delta>0;
        pendingTarget_=target;
        pendingOffset_=offset;
        tickResidual_=requested-static_cast<int32_t>(actual);
        if (running) {
            // An interrupt can drain the queue between the first check and
            // our append. The unstarted straggler is flushed during Stop.
            if (!driver.isQueueRunning()) return {Frame::Rejected, "pulse_underrun"};
        } else if (driver.queueEntries() >= pulseQueueFrames || driver.ticksInQueue() >= queuedLimit) {
            return startQueue(driver);
        }
        return {Frame::Accepted};
    }

private:
    template <class Driver>
    FrameResult startQueue(Driver &driver) {
        const auto started=static_cast<int8_t>(driver.addQueueEntry(nullptr,true));
        if (started) return {Frame::Rejected,"pulse_start",started};
        stage_=Stage::Running;
        return {Frame::Accepted};
    }
};
}
