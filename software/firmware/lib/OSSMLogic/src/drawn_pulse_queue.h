#pragma once

#include <atomic>
#include <cstdint>

#include "drawn_waveform.h"
#include "drawn_prepared_pulses.h"

namespace drawn_pattern {
enum class Frame { Accepted, Retry, Rejected };
enum class Park { Fill, Draining, Parked, Failed };
enum class FrameTiming : uint8_t { Unknown, EntryHold, Prefill, Started, Running, PrimeTail };
struct FrameResult {
    Frame state;
    const char *reason = nullptr;
    int8_t driverResult = 0;
    FrameTiming timing = FrameTiming::Unknown;
    uint32_t commandGateUs = 0;
    uint16_t queuedUs = UINT16_MAX; // FAS time behind its current command, when sampled.
    uint8_t frames = 1; // Exact source intervals accepted by this hardware command.
};

constexpr uint32_t frameStarvationMs = 200;

// FAS can emit the one-step re-prime immediately, then keep its finite
// ramp busy with the remainder of that step interval. Only an observed
// arrival at the issued target can extend the ordinary starvation budget.
class PulsePrimeWait {
    int32_t target_ = 0;
    uint32_t startedAt_ = 0, deadlineMs_ = frameStarvationMs;
public:
    void start(int32_t target, uint32_t now, uint32_t speedTicks,
               uint32_t ticksPerMs, uint32_t acceleration) {
        target_ = target;
        startedAt_ = now;
        deadlineMs_ = frameStarvationMs;
        if (speedTicks == 0 || ticksPerMs == 0 || acceleration == 0) return;
        // The drawn backend sets an integer speed of at least 1 Hz and
        // acceleration of at least 1 step/s². FAS's slowest acceleration
        // interval is 1/sqrt(2*a). Use 1/sqrt(a) to cover timer rounding.
        const uint32_t travelMs = speedTicks / ticksPerMs + (speedTicks % ticksPerMs != 0);
        const auto accelerationMs = static_cast<uint32_t>(std::ceil(1000.0 / std::sqrt(acceleration)));
        const auto tailMs = std::max(std::min<uint32_t>(travelMs, 1000), accelerationMs);
        deadlineMs_ = std::max(frameStarvationMs, tailMs + 20); // finite-ramp publication and service
    }
    FrameResult poll(uint32_t now, bool running, int32_t position) const {
        const auto elapsed = now - startedAt_;
        if (!running && position == target_) return {Frame::Accepted};
        if (elapsed >= deadlineMs_) return {Frame::Rejected, "prime_timeout"};
        if (!running && elapsed >= 20) return {Frame::Rejected, "prime_position"};
        const bool tail = running && position == target_;
        return {Frame::Retry, "pulse_priming", 0, tail ? FrameTiming::PrimeTail : FrameTiming::Unknown};
    }
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

// Refill toward 14 ms within the 16 ms hardware limit. A full-sized
// command may not fit before that threshold; wait normally on such retries.
constexpr uint16_t refillBelowUs = (queuedDurationMs - pulseFrameMs) * 1000;
class PulseRefillSchedule {
    uint8_t consecutive_ = 0;
public:
    bool immediate(bool lowAfterProgress) {
        if (lowAfterProgress && consecutive_ < 2) {
            ++consecutive_;
            return true;
        }
        consecutive_ = 0;
        return false;
    }
};

static_assert(PreparedPulses::capacity + 1 == queuedDurationMs / pulseFrameMs,
              "a prepared burst cannot exceed the existing service visit");

struct PulseBatch {
    struct Attempt {
        FrameResult result{Frame::Rejected, "guard"};
        uint32_t atUs = 0;
    };
    Attempt attempts[1 + PreparedPulses::capacity]{};
    unsigned count = 0;
    bool revoked = false, controlsChanged = false;

    void revoke(uint32_t nowUs) {
        revoked = true;
        if (!count || attempts[0].result.state != Frame::Accepted) {
            // No accepted prefix exists. A simultaneous full-queue Retry
            // must never defer a definitive guard or ownership revocation.
            count = 1;
            attempts[0] = {{Frame::Rejected, "guard"}, nowUs};
        }
    }

    template<class Write, class Allowed, class Clock>
    void submit(PreparedPulses::Target first, const PreparedPulses &next, Write write,
                Allowed allowed, Clock clock) {
        auto target = first;
        unsigned consumed = 0;
        for (;;) {
            auto &attempt = attempts[count++];
            attempt.result = write(target);
            attempt.atUs = clock();
            if (attempt.result.state != Frame::Accepted) return;
            consumed += attempt.result.frames;
            if (consumed > next.count || attempt.result.timing != FrameTiming::Running ||
                attempt.result.queuedUs >= refillBelowUs) return;
            if (!allowed()) {
                controlsChanged = true;
                return;
            }
            target = next.targets[consumed - 1];
        }
    }
};

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
    enum class Entry { Raw, Busy, Stationary, FirstMove };
    // A drained Running queue stays owned and must report its underrun,
    // even if the next target equals the current physical count.
    template <class Driver>
    Entry entryState(Driver &driver, int32_t target, bool completedPrime = false) const {
        if (owned()) return Entry::Raw;
        if (driver.isQueueRunning() || !driver.isQueueEmpty()) return Entry::Busy;
        if (completedPrime) return Entry::Raw;
        return driver.getCurrentPosition() == target ? Entry::Stationary : Entry::FirstMove;
    }
    bool filling() const { return stage_.load() == Stage::Filling; }
    int32_t initialPosition() const { return initialPosition_; }
    void reset() { stage_ = Stage::Idle; }

    // The controller has committed two identical terminal frames. Approve
    // the drain while the driver still runs, then verify its physical count
    // and empty queue before surrendering raw ownership.
    template <class Driver>
    Park park(Driver &driver, int32_t target, bool draining) {
        if (filling()) return Park::Fill;
        if (owned() && (pendingTarget_ != target || (!draining && !driver.isQueueRunning())))
            return Park::Failed;
        if (owned() && driver.getPositionAfterCommandsCompleted() != target) return Park::Failed;
        if (driver.isQueueRunning() || !driver.isQueueEmpty()) return owned() ? Park::Draining : Park::Failed;
        if (driver.getCurrentPosition() != target) return Park::Failed;
        reset();
        return Park::Parked;
    }

    // A moving group's first electrical edge is immediate in moveTimed().
    // Start that group at the outgoing half-step crossing. Its duration runs
    // to the first crossing of the following frame. A hold uses the midpoint
    // of its frame. The one pending group is software only and drops on Stop.
    struct MayExtend { bool operator()() const { return true; } };
    template <class Command, class Driver, class CanExtend = MayExtend>
    FrameResult submit(Driver &driver, int32_t target, uint32_t frameTicks,
                       uint16_t firstCrossing, uint16_t minCommandTicks,
                       int8_t movedFromEmpty, unsigned followingHolds = 0, CanExtend canExtend = {},
                       bool appendSeparateTail = true) {
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
        const auto speedTicks = driver.getMaxSpeedInTicks();
        const uint32_t minHold = std::max<uint32_t>(minCommandTicks, speedTicks);
        const uint32_t queuedLimit = frameTicks * queuedDurationMs / pulseFrameMs;
        const uint32_t queued = count >= 2 ? driver.ticksInQueue() : 0;
        // Two midpoint-to-midpoint waits have no STEP edge or DIR change
        // between them. One native wait emits the same next edge with half
        // the append and interrupt load. Prefer a single wait near capacity.
        const bool paired = followingHolds && running && delta == 0 && upcoming == 0 &&
                            pendingOffset_ == offset && !tickResidual_ &&
                            frameTicks >= minHold &&
                            queued + 2 * frameTicks <= queuedLimit;
        const int32_t requested = first ? static_cast<int32_t>(offset) :
            static_cast<int32_t>((paired ? 2 : 1) * frameTicks + offset - pendingOffset_) + tickResidual_;
        const int32_t steps = std::max<int32_t>(1, std::abs(delta));
        // FAS checks the minimum duration after dividing and truncating ticks
        // per edge. Return all quantization/clipping error to the next frame.
        const int32_t minimum = steps*std::max<int32_t>(speedTicks,
            (minCommandTicks+steps-1)/steps);
        const uint32_t duration = std::max(requested,minimum);
        if (duration>2*frameTicks+255 || duration/steps>65535)
            return {Frame::Rejected, "pulse_range"};
        if (count >= 2 && queued + duration > queuedLimit) {
            if (!running) {
                const auto started=startQueue(driver);
                if (started.state==Frame::Rejected) return started;
                return {Frame::Retry, "pulse_full", 0, FrameTiming::Started};
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
        }
        FrameResult result{Frame::Accepted, nullptr, 0, running ? FrameTiming::Running : FrameTiming::Prefill};
        if (!running && (driver.queueEntries() >= pulseQueueFrames || driver.ticksInQueue() >= queuedLimit)) {
            result = startQueue(driver);
        }
        if (result.state == Frame::Accepted) {
            result.frames = paired ? 2 : 1;
            if (appendSeparateTail && followingHolds && running && std::abs(delta) == 1 && !upcoming && !tickResidual_ &&
                frameTicks >= minHold &&
                driver.queueEntries() < pulseQueueFrames) {
                // The first command emitted the preceding row's single step.
                // Append its known stationary successor without a second
                // controller turn. Keep the original pulse width, time and
                // direction, even when its two intervals cannot fit one wait.
                const auto queuedTicks = driver.ticksInQueue();
                const unsigned available = (queuedLimit - std::min(queuedTicks, queuedLimit)) / frameTicks;
                const auto tailFrames = std::min({followingHolds, available, 2u});
                if (!tailFrames) {
                    result.queuedUs = static_cast<uint16_t>(queuedTicks * (1000 * pulseFrameMs) / frameTicks);
                    return result;
                }
                if (!canExtend()) return {Frame::Rejected, "guard"};
                if (!driver.isQueueRunning()) return {Frame::Rejected, "pulse_underrun"};
                Command hold{static_cast<uint16_t>(tailFrames * frameTicks), 0, direction_};
                const auto extra = static_cast<int8_t>(driver.addQueueEntry(&hold, false));
                if (!driver.isQueueRunning()) return {Frame::Rejected, "pulse_underrun", extra};
                if (extra < 0) return {Frame::Rejected, "pulse_append", extra};
                // A busy driver accepted only the first row. Its successor
                // is still at the software front on the next guarded turn.
                if (extra == 0) result.frames = static_cast<uint8_t>(1 + tailFrames);
                else result.driverResult = extra;
            }
            // Read after the append/start: the interrupt may have consumed
            // commands since the admission check. FAS excludes the active entry.
            const uint32_t queuedTicks = driver.ticksInQueue();
            result.queuedUs = static_cast<uint16_t>(queuedTicks * (1000 * pulseFrameMs) / frameTicks);
        }
        return result;
    }

private:
    template <class Driver>
    FrameResult startQueue(Driver &driver) {
        const auto started=static_cast<int8_t>(driver.addQueueEntry(nullptr,true));
        if (started) return {Frame::Rejected,"pulse_start",started};
        stage_=Stage::Running;
        return {Frame::Accepted,nullptr,0,FrameTiming::Started};
    }
};
}
