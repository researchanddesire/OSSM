#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include "drawn_frame_queue.h"
#include "drawn_pattern.h"
#include "drawn_waveform.h"

#ifdef ARDUINO
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

namespace streaming {
uint32_t patternServiceMicros();

namespace frame_plan {

struct Marking {
    drawn_pattern::Modifiers authored;
    drawn_pattern::MarkerIndex index;
    drawn_pattern::PauseBrakeBounds brakeBounds;
};

struct Inputs {
    drawn_pattern::PhaseLimits limits{};
    drawn_pattern::PauseBrakeBounds brake;
    drawn_pattern::VibrationBudget budget;
    double strength = 0, dt = 0;
    int32_t lower = 0, upper = 0;
    bool zero = false, serial = false, vibrationEnabled = false;
};

inline void calculate(const Inputs &in, const Cursor &begin, Frame &frame,
                      const drawn_pattern::Waveform &curve, const drawn_pattern::MarkerIndex *markers) {
    static const drawn_pattern::MarkerIndex empty;
    const auto &index = markers ? *markers : empty;
    const uint32_t start = patternServiceMicros();
    const auto &phase = begin.phase;
    const auto &vibration = begin.vibration;
    const bool withinEnvelope = phase.clock.secondsToGoal() == 0 &&
                                phase.clock.rate() <= in.limits.rate + 1e-12;
    const double amplitude = in.vibrationEnabled && (withinEnvelope || !vibration.gain.atZero())
                                 ? in.strength : 0;
    drawn_pattern::FrameBrake brake;
    if (index.holds) brake.bounds = &in.brake;
    const auto advance = [&](auto &p, auto &v, double seconds) {
        if (in.zero) p.brakeForUser(seconds, in.limits);
        else p.advance(seconds, in.limits, index, in.strength, &brake);
        if (index.hasVibration) v.advance(seconds, amplitude);
    };
    const auto target = [&](const auto &p, const auto &v) {
        if (v.gain.atZero()) return curve.nativeTarget(p.phase, in.lower, in.upper);
        const double y = curve.position(p.phase);
        return in.upper - static_cast<int32_t>(std::lround(
            (in.upper - in.lower) * y + v.offset(y, p.phase, index, in.budget)));
    };

    frame.phase = phase;
    frame.vibration = vibration;
    advance(frame.phase, frame.vibration, in.dt);
    frame.target = target(frame.phase, frame.vibration);
    frame.targetKnown = true;
    frame.from = begin.targetKnown ? begin.target : target(phase, vibration);
    frame.crossing = 32768;
    if (!in.serial && frame.from != frame.target) {
        if (!markers) {
            auto clock = phase.clock;
            clock.setGoal(in.limits.rate, in.limits.acceleration, in.limits.jerk);
            frame.crossing = curve.firstPlainStepCrossing(frame.from, frame.target, phase.phase,
                                                          clock, in.dt, in.lower, in.upper);
        } else {
            frame.crossing = drawn_pattern::firstStepCrossing(frame.from, frame.target, [&](double fraction) {
                auto p = phase;
                auto v = vibration;
                advance(p, v, in.dt * fraction);
                return target(p, v);
            });
        }
    }
    frame.valid = true;
    frame.offered = false;
    frame.planUs = patternServiceMicros() - start;
}

// The feeder may run on the opposite core immediately after the planner
// starts publishing. Keep these bounded copies preemption-safe on ESP32;
// no driver, allocation, notification or curve evaluation holds this lock.
#ifdef ARDUINO
class RingMutex {
public:
    void lock() { portENTER_CRITICAL(&mux_); }
    void unlock() { portEXIT_CRITICAL(&mux_); }
private:
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};
#else
using RingMutex = std::mutex;
#endif

// The planner can only publish RAM. The feeder retains exclusive ownership
// of the motor and of accepted cursor state. A generation change discards
// completed rows AND any calculation currently outside the short ring lock.
class RawFramePlanner {
public:
    static constexpr unsigned capacity = FrameQueue::capacity;
    static constexpr unsigned warmupRows = 12;
    // Only a positive Speed-to-Speed edit may finish this already-planned
    // prefix. At 2 ms/frame it gives the new law at most 16 ms of software
    // handoff. A second edit cannot extend the boundary.
    static constexpr unsigned retargetPrefix = FrameQueue::maxRetained;
    static constexpr unsigned reserveForAssist = 16;
    static constexpr unsigned stackBytes = 4096;
    const std::unique_ptr<Marking> marking;
    // The motor consumer alone tracks the applied law across a positive
    // edit. No static allocation or worker access is needed while idle.
    PatternSpeedStatus previousSpeed;
    drawn_pattern::VibrationBudget previousBudget;
    struct Counters {
        uint32_t produced = 0, discarded = 0, emptyTurns = 0, peakDepth = 0;
        uint32_t stackFree = 0, assistAttempts = 0, assistPublished = 0, assistMaxUs = 0;
    };

    RawFramePlanner(const drawn_pattern::Waveform &source, std::unique_ptr<Marking> markers)
        : marking(std::move(markers)), workerLane_(source), assistLane_(source) {}

    bool hasQueueStorage() const { return rows_.hasStorage(); }

    bool start() {
        if (!hasQueueStorage()) return false;
#ifdef ARDUINO
        ++references_;
        if (xTaskCreatePinnedToCore(workerMain, "drawnPlan", stackBytes, this, 8, &worker_, 0) != pdPASS) {
            --references_;
            return false;
        }
#else
        if (!workerAvailable) return false;
#endif
        return true;
    }

    // Called exactly once by the feeder, including after task creation fails.
    void close(Counters &out) {
        {
            std::lock_guard<RingMutex> guard(mux_);
            closed_ = true;
            out = counters_;
        }
        release();
    }

    // All configuration/consume calls are serialized by the controller.
    bool configured() const { return configured_; }
    void configure(const Inputs &in, const Cursor &accepted) {
        {
            std::lock_guard<RingMutex> guard(mux_);
            inputs_ = in;
            if (!retained_) {
                rows_.clear(accepted);
                tail_ = accepted;
            }
            configured_ = true;
        }
        wake();
    }

    void invalidate(bool warmup = false) {
        std::lock_guard<RingMutex> guard(mux_);
        ++generation_;
        counters_.discarded += rows_.size();
        rows_.clear(tail_);
        configured_ = false;
        retained_ = 0;
        positivePending_ = false;
        if (warmup) warmed_ = false;
    }

    // Keep the first few rows of the applied law. Their final cursor is
    // the sole origin for the new generation. Old work outside this
    // prefix, including any calculation on either CPU, cannot publish.
    void retarget(drawn_pattern::VibrationClock &boundaryVibration) {
        std::lock_guard<RingMutex> guard(mux_);
        ++generation_;
        const unsigned before = rows_.size();
        retained_ = rows_.truncate(positivePending_ ? retained_ : retargetPrefix, tail_);
        counters_.discarded += before - retained_;
        if (retained_) boundaryVibration = tail_.vibration;
        positivePending_ = true;
        configured_ = false;
    }

    // Only the serialized motor consumer reads or changes these two fields.
    bool retainedFront() const { return retained_ != 0; }
    bool positivePending() const { return positivePending_; }

    enum class Action { Produced, Wait, Exit };

    template<class AfterSnapshot>
    Action prepare(AfterSnapshot afterSnapshot) {
        return prepareRow(workerLane_, capacity - 1, false, afterSnapshot);
    }

    // Called with playback ownership still held. A helper holds a lifetime
    // reference, not a motor lease. No helper can submit to the peripheral.
    bool requestAssist(unsigned maximumReady) {
        std::lock_guard<RingMutex> guard(mux_);
        if (!configured_ || !warmed_ || closed_ || assisting_ || rows_.size() > maximumReady) return false;
        assisting_ = true;
        assistAtDepth_ = maximumReady;
        ++references_;
        return true;
    }

    // The feeder invokes this outside the controller mailbox. An explicit
    // Stop can close this session, even during a marked crossing solve.
    template<class AfterSnapshot>
    void assist(AfterSnapshot afterSnapshot) {
        prepareRow(assistLane_, assistAtDepth_, true, afterSnapshot);
        {
            std::lock_guard<RingMutex> guard(mux_);
            assisting_ = false;
        }
        release();
    }

    // Only this consumer can release a row. The producer never writes into
    // the front slot, even while the motor is retrying its previous submit.
    Frame *front() {
#ifndef ARDUINO
        if (pumpInline) while (prepare([] {}) == Action::Produced) {}
#endif
        std::lock_guard<RingMutex> guard(mux_);
        if (warmed_ && rows_.size()) return rows_.front();
        if (warmed_) ++counters_.emptyTurns;
        return nullptr;
    }

    void successors(drawn_pattern::PreparedPulses &out, unsigned maximum) {
        std::lock_guard<RingMutex> guard(mux_);
        if (warmed_ && !positivePending_) rows_.successors(out, maximum);
    }

    bool hasPreparedFrame() {
        std::lock_guard<RingMutex> guard(mux_);
        return warmed_ && rows_.size() != 0;
    }

    void pop() {
        bool refill;
        {
            std::lock_guard<RingMutex> guard(mux_);
            if (retained_) --retained_;
            else positivePending_ = false;
            const bool full = rows_.full();
            rows_.pop();
            refill = full && !rows_.full();
        }
        if (refill) wake();
    }

#ifndef ARDUINO
    // The deterministic host runner advances the same production planner on
    // demand. Threaded tests use the worker's prepare/release lifetime.
    inline static bool pumpInline = true, workerAvailable = true;
    void retainForTest() { ++references_; }
    void releaseForTest() { release(); }
#endif

private:
    RingMutex mux_;
    std::atomic<unsigned> references_{1};
    struct Lane {
        explicit Lane(const drawn_pattern::Waveform &source) : model(source) {}
        drawn_pattern::Waveform model; // polynomial and vibration caches are private per core
        Inputs inputs;
        Cursor start;
        Frame row;
        uint32_t generation = UINT32_MAX;
    };
    Lane workerLane_, assistLane_;
    Inputs inputs_;
    Cursor tail_;
    FrameQueue rows_;
    Counters counters_;
    unsigned assistAtDepth_ = 0, retained_ = 0;
    uint32_t generation_ = 0, tailOrdinal_ = 0;
    bool configured_ = false, warmed_ = false, closed_ = false, assisting_ = false, positivePending_ = false;

    template<class AfterSnapshot>
    Action prepareRow(Lane &lane, unsigned maximumReady, bool assist, AfterSnapshot afterSnapshot) {
        uint32_t ordinal;
        {
            std::lock_guard<RingMutex> guard(mux_);
            if (closed_) return Action::Exit;
            if (!configured_ || rows_.full() || rows_.size() > maximumReady) return Action::Wait;
            ordinal = tailOrdinal_;
            lane.start = tail_;
            if (lane.generation != generation_) {
                lane.generation = generation_;
                lane.inputs = inputs_;
            }
            if (assist) ++counters_.assistAttempts;
        }
        afterSnapshot();
        calculate(lane.inputs, lane.start, lane.row, lane.model, marking ? &marking->index : nullptr);
        {
            std::lock_guard<RingMutex> guard(mux_);
            if (closed_) return Action::Exit;
            if (assist) counters_.assistMaxUs = std::max(counters_.assistMaxUs, lane.row.planUs);
            if (lane.generation == generation_ && ordinal == tailOrdinal_) {
                // No other producer advanced this ordinal after our snapshot.
                rows_.push(lane.row, tail_);
                tail_ = lane.row;
                ++tailOrdinal_;
                ++counters_.produced;
                if (assist) ++counters_.assistPublished;
                counters_.peakDepth = std::max(counters_.peakDepth, rows_.size());
                if (rows_.size() >= warmupRows) warmed_ = true;
            } else if (lane.generation != generation_) {
                ++counters_.discarded;
            }
        }
        return Action::Produced;
    }
#ifdef ARDUINO
    TaskHandle_t worker_ = nullptr;
    static void workerMain(void *opaque) {
        auto *self = static_cast<RawFramePlanner *>(opaque);
        unsigned batch = 0;
        TickType_t measuredAt = xTaskGetTickCount() - pdMS_TO_TICKS(1000);
        for (;;) {
            const auto action = self->prepare([] {});
            if (action == Action::Exit) break;
            if (action == Action::Wait) {
                const auto now = xTaskGetTickCount();
                if (now - measuredAt >= pdMS_TO_TICKS(1000)) {
                    const auto free = uxTaskGetStackHighWaterMark(nullptr);
                    std::lock_guard<RingMutex> guard(self->mux_);
                    self->counters_.stackFree = free;
                    measuredAt = now;
                }
                // A finite wait also retires a worker closed while asleep.
                // Its handle is never notified after it was asked to exit.
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
                batch = 0;
            } else if (++batch == 6) {
                // Lower-priority networking and idle work still get a tick,
                // even with continuous control edits or CPU-heavy curves.
                vTaskDelay(1);
                batch = 0;
            }
        }
        self->release();
        vTaskDelete(nullptr);
    }
#endif
    void wake() {
#ifdef ARDUINO
        if (worker_) xTaskNotifyGive(worker_);
#endif
    }
    void release() {
        if (--references_ == 0) delete this;
    }
};
}
}
