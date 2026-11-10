#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

// One finite, explicitly reviewed characterization, not normal authorization.
// Runtime owns exact identity, known free corridor, wiring review, fixture/bus
// ownership, preverified UART disable, NVS sentinel and every physical effect.
// Nothing here drives a pin, writes UART, resets a fault, or starts homing.
// The caller serializes all methods; no blocking UART work belongs in that lock.
namespace serial_interlock_probe {

enum class Phase : uint8_t {
    Idle, Prepared, Moving, AwaitingGpioCut, Braking, ProvingGpio,
    AwaitingSerialDisable, ProvingFinalQuiet, Passed, Failed
};
enum class Action : uint8_t { None, AssertGpioHigh, SerialDisable };
enum class Failure : uint8_t {
    None, Prerequisites, AlreadyUsed, InvalidSample, StaleSample, SampleRate,
    Continuity, Sequence, Envelope, Current, Voltage, Alarm, Speed,
    WrongDirection, UnexpectedMotion, TargetTooClose, NoMotion,
    UnexpectedWrite, ActionOrder, GpioNotAsserted, GpioDidNotStop,
    SerialDisableUnverified, OverallTimeout, SafetyFault, Cancelled
};

struct Sample {
    uint32_t sequence = 0;
    uint32_t atMs = 0;
    uint32_t continuity = 0;
    int32_t encoderCounts = 0;
    float currentAmps = 0;
    float voltageVolts = 0;
    float observedHz = 0;
    uint16_t alarm = 0;
};
struct SerialReadback {
    uint32_t atMs = 0;
    uint16_t mode = 0;
    uint16_t output = 0;
};
struct Status {
    Phase phase = Phase::Idle;
    Failure failure = Failure::None;
    bool active = false;
    bool finished = false;
    bool passed = false;
    bool enabled = false;
    bool targetWritten = false;
    bool gpioStopProven = false;
    bool serialDisabled = false;
    bool finalQuiet = false;
    uint8_t positiveIntervals = 0;
    uint8_t gpioQuietSamples = 0;
    uint8_t finalQuietSamples = 0;
    int32_t originCounts = 0;
    int32_t targetCounts = 0;
    int32_t lastCounts = 0;
    uint32_t startedAtMs = 0;
    uint32_t gpioCutAtMs = 0;
};

class Gate {
public:
    // Raw AIM encoder: 32768/rev, 40 mm/rev. Round commanded distances
    // inward and eligibility distances outward to retain the stated limits.
    static constexpr int32_t targetDeltaCounts = 409;   // <= +0.5 mm
    static constexpr int32_t envelopeCounts = 614;      // <= +/-0.75 mm
    static constexpr int32_t cutProgressCounts = 82;    // >= +0.1 mm
    static constexpr int32_t minimumRemainingCounts = 205; // >= 0.25 mm
    static constexpr int32_t quietRangeCounts = 4;

    bool start(int32_t originCounts, uint32_t nowMs,
               bool reviewedPrerequisites) {
        if (used_) return false;  // cannot restart even a failed/ambiguous run
        used_ = true;
        state_.startedAtMs = nowMs;
        state_.originCounts = state_.lastCounts = originCounts;
        const int64_t target = static_cast<int64_t>(originCounts) + targetDeltaCounts;
        if (!reviewedPrerequisites ||
            target > std::numeric_limits<int32_t>::max()) {
            // No admitted probe and no motor authority: a failed identity/
            // ownership precondition must not cause even a UART stop write.
            state_.failure = Failure::Prerequisites;
            state_.phase = Phase::Failed;
            return false;
        }
        state_.targetCounts = static_cast<int32_t>(target);
        state_.phase = Phase::Prepared;
        return true;
    }

    // Runtime calls this only after the guarded enable write and readback.
    // It must have supplied fresh stationary telemetry after start().
    bool markEnabled(uint32_t nowMs) {
        tick(nowMs);
        if (state_.phase != Phase::Prepared || failed() || !haveSample_ ||
            !sampleFresh(nowMs) || distance(last_.encoderCounts, state_.originCounts) > quietRangeCounts) {
            fail(Failure::ActionOrder, nowMs);
            return false;
        }
        state_.enabled = true;
        state_.phase = Phase::Moving;
        return true;
    }

    // Call at the actual committed finite-target dispatch; do not transmit
    // if false. Replays, refreshes and replacement targets are forbidden.
    bool markTargetWritten(int32_t targetCounts, uint32_t nowMs) {
        tick(nowMs);
        if (state_.phase != Phase::Moving || failed() || !state_.enabled ||
            state_.targetWritten || targetCounts != state_.targetCounts) {
            fail(Failure::UnexpectedWrite, nowMs);
            return false;
        }
        state_.targetWritten = true;
        return true;
    }

    void observe(const Sample &sample, uint32_t nowMs) {
        tick(nowMs);
        if (!used_ || terminal()) return;
        // A monitor may have captured an older snapshot before start() while
        // the worker admitted and seeded this probe on another core. It is
        // outside this probe's observation interval and cannot advance proof.
        if (!notBefore(sample.atMs, state_.startedAtMs)) return;
        if (!validSample(sample, nowMs)) return;
        if (haveSample_) {
            if (sample.continuity != last_.continuity) {
                fail(Failure::Continuity, nowMs); return;
            }
            const uint32_t sequenceDelta = sample.sequence - last_.sequence;
            if (sequenceDelta == 0) {
                // Repeated polling of the same immutable physical sample is
                // harmless, but never counts toward motion or quiet proof.
                if (sample.atMs != last_.atMs || sample.encoderCounts != last_.encoderCounts ||
                    sample.currentAmps != last_.currentAmps || sample.alarm != last_.alarm)
                    fail(Failure::Sequence, nowMs);
                return;
            }
            if (sequenceDelta >= 0x80000000U) {
                fail(Failure::Sequence, nowMs); return;
            }
            const uint32_t gap = sample.atMs - last_.atMs;
            if (gap == 0 || gap > 100) {
                fail(Failure::StaleSample, nowMs); return;
            }
            const int64_t delta = static_cast<int64_t>(sample.encoderCounts) - last_.encoderCounts;
            if (std::abs(static_cast<double>(delta)) * 1000.0 > 1638.4 * gap) {
                fail(Failure::Speed, nowMs); return;
            }
            if (state_.phase == Phase::Moving && state_.targetWritten) {
                state_.positiveIntervals = delta > 0
                    ? static_cast<uint8_t>(state_.positiveIntervals < 255 ? state_.positiveIntervals + 1 : 255)
                    : 0;
            }
        }
        last_ = sample;
        haveSample_ = true;
        state_.lastCounts = sample.encoderCounts;
        const int64_t progress = static_cast<int64_t>(sample.encoderCounts) - state_.originCounts;
        const int64_t remaining = static_cast<int64_t>(state_.targetCounts) - sample.encoderCounts;

        if (state_.phase == Phase::Prepared ||
            (state_.phase == Phase::Moving && !state_.targetWritten)) {
            if (distance(sample.encoderCounts, state_.originCounts) > quietRangeCounts)
                fail(Failure::UnexpectedMotion, nowMs);
            return;
        }
        if (state_.phase == Phase::Moving) {
            if (progress < -quietRangeCounts) {
                fail(Failure::WrongDirection, nowMs); return;
            }
            if (remaining < minimumRemainingCounts) {
                fail(Failure::TargetTooClose, nowMs); return;
            }
            if (progress >= cutProgressCounts && state_.positiveIntervals >= 3) {
                state_.phase = Phase::AwaitingGpioCut;
                gpioRequestedAtMs_ = nowMs;
                gpioPending_ = true;
            }
            return;
        }
        if (state_.phase == Phase::AwaitingGpioCut || state_.phase == Phase::Braking ||
            state_.phase == Phase::ProvingGpio) {
            if (remaining < minimumRemainingCounts) {
                fail(Failure::TargetTooClose, nowMs); return;
            }
        }
        if (state_.phase == Phase::Braking && elapsed(nowMs, state_.gpioCutAtMs) >= 100) {
            state_.phase = Phase::ProvingGpio;
            gpioQuiet_.clear();
        }
        if (state_.phase == Phase::ProvingGpio &&
            notBefore(sample.atMs, state_.gpioCutAtMs + 100)) {
            const bool quiet = gpioQuiet_.add(sample);
            state_.gpioQuietSamples = gpioQuiet_.count;
            if (quiet) {
                state_.gpioStopProven = true;
                requestSerialDisable();
            }
        }
        if (state_.phase == Phase::ProvingFinalQuiet &&
            notBefore(sample.atMs, serialReadbackAtMs_)) {
            const bool quiet = finalQuiet_.add(sample);
            state_.finalQuietSamples = finalQuiet_.count;
            if (quiet) {
                state_.finalQuiet = true;
                state_.enabled = false;
                state_.phase = !failed() && state_.gpioStopProven && state_.serialDisabled
                    ? Phase::Passed : Phase::Failed;
            }
        }
    }

    void tick(uint32_t nowMs) {
        if (!used_ || terminal()) return;
        // An already captured monitor clock can precede a concurrent start.
        // It has no post-start elapsed time and must not underflow a deadline.
        if (!notBefore(nowMs, state_.startedAtMs)) return;
        if (elapsed(nowMs, state_.startedAtMs) >= 3000) {
            fail(Failure::OverallTimeout, nowMs);
            state_.phase = Phase::Failed;
            return;
        }
        if (haveSample_ && !sampleFresh(nowMs)) {
            fail(Failure::StaleSample, nowMs);
            return;
        }
        if ((state_.phase == Phase::Prepared || state_.phase == Phase::Moving) &&
            elapsed(nowMs, state_.startedAtMs) >= 1000) {
            fail(Failure::NoMotion, nowMs);
            return;
        }
        if (state_.phase == Phase::AwaitingGpioCut &&
            elapsed(nowMs, gpioRequestedAtMs_) > 10) {
            fail(Failure::GpioNotAsserted, nowMs);
            return;
        }
        if ((state_.phase == Phase::Braking || state_.phase == Phase::ProvingGpio) &&
            elapsed(nowMs, state_.gpioCutAtMs) > 250)
            fail(Failure::GpioDidNotStop, nowMs);
    }

    // Drain actions even after a timeout set phase Failed. Each action is
    // returned once, always GPIO first, then UART disable on every outcome.
    Action takeAction(uint32_t nowMs) {
        tick(nowMs);
        if (gpioPending_ && !gpioActionIssued_) {
            gpioActionIssued_ = true;
            return Action::AssertGpioHigh;
        }
        if (serialPending_ && gpioActionIssued_ && !serialActionIssued_) {
            serialActionIssued_ = true;
            serialDisableIssuedAtMs_ = nowMs;
            return Action::SerialDisable;
        }
        return Action::None;
    }

    // Timestamp after the physical GPIO write, not when its action was queued.
    bool markGpioAsserted(uint32_t nowMs) {
        tick(nowMs);
        if (!gpioActionIssued_ || gpioAsserted_) {
            fail(Failure::ActionOrder, nowMs); return false;
        }
        gpioAsserted_ = true;
        state_.gpioCutAtMs = nowMs;
        if (!failed() && state_.phase == Phase::AwaitingGpioCut) {
            state_.phase = Phase::Braking;
            gpioQuiet_.clear();
        }
        return true;
    }

    bool markSerialDisabled(const SerialReadback &reply, uint32_t nowMs) {
        tick(nowMs);
        if (!serialActionIssued_ || reply.mode != 1 || (reply.output & 1U) != 0 ||
            elapsed(nowMs, reply.atMs) > 100 || !notBefore(reply.atMs, serialDisableIssuedAtMs_) ||
            !haveSample_ || !sampleFresh(nowMs)) {
            fail(Failure::SerialDisableUnverified, nowMs); return false;
        }
        state_.serialDisabled = true;
        state_.enabled = false;
        serialReadbackAtMs_ = reply.atMs;
        finalQuiet_.clear();
        if (!terminal()) state_.phase = Phase::ProvingFinalQuiet;
        return true;
    }

    void abort(Failure reason, uint32_t nowMs) {
        if (!used_ || terminal()) return;
        fail(reason == Failure::None ? Failure::SafetyFault : reason, nowMs);
    }
    void unexpectedControlWrite(uint32_t nowMs) {
        abort(Failure::UnexpectedWrite, nowMs);
    }
    Status status() const {
        Status result = state_;
        result.finished = terminal();
        result.active = used_ && !result.finished;
        result.passed = state_.phase == Phase::Passed && !failed() &&
            state_.gpioStopProven && state_.serialDisabled && state_.finalQuiet;
        return result;
    }

private:
    struct Quiet {
        uint32_t startedAtMs = 0;
        int32_t lower = 0, upper = 0;
        uint8_t count = 0;
        void clear() { count = 0; }
        bool add(const Sample &sample) {
            if (count == 0) {
                startedAtMs = sample.atMs;
                lower = upper = sample.encoderCounts;
                count = 1;
            } else {
                if (sample.encoderCounts < lower) lower = sample.encoderCounts;
                if (sample.encoderCounts > upper) upper = sample.encoderCounts;
                if (static_cast<int64_t>(upper) - lower > quietRangeCounts) {
                    startedAtMs = sample.atMs;
                    lower = upper = sample.encoderCounts;
                    count = 1;
                } else if (count < 255) ++count;
            }
            return count >= 6 && static_cast<uint32_t>(sample.atMs - startedAtMs) >= 100;
        }
    };
    static uint32_t elapsed(uint32_t now, uint32_t then) { return now - then; }
    static bool notBefore(uint32_t value, uint32_t reference) {
        return static_cast<uint32_t>(value - reference) < 0x80000000U;
    }
    static int64_t distance(int32_t a, int32_t b) {
        const int64_t delta = static_cast<int64_t>(a) - b;
        return delta < 0 ? -delta : delta;
    }
    bool terminal() const { return state_.phase == Phase::Passed || state_.phase == Phase::Failed; }
    bool failed() const { return state_.failure != Failure::None; }
    bool sampleFresh(uint32_t nowMs) const { return elapsed(nowMs, last_.atMs) <= 100; }
    void requestSerialDisable() {
        serialPending_ = true;
        state_.phase = Phase::AwaitingSerialDisable;
    }
    void fail(Failure reason, uint32_t) {
        if (terminal()) return;
        const bool alreadyFailed = failed();
        const bool verifyingDisabled = state_.phase == Phase::ProvingFinalQuiet &&
            state_.serialDisabled;
        if (!failed()) state_.failure = reason;
        gpioPending_ = true;
        serialPending_ = true;
        state_.gpioStopProven = false;
        state_.finalQuiet = false;
        if (verifyingDisabled) {
            // Cancellation/fault cannot turn an already verified UART disable
            // into a new enable. Keep collecting final quiet, restarting its
            // evidence once for a new failure, not on every monitor tick.
            if (!alreadyFailed) {
                finalQuiet_.clear();
                state_.finalQuietSamples = 0;
            }
            return;
        }
        state_.phase = Phase::AwaitingSerialDisable;
    }
    bool validSample(const Sample &sample, uint32_t nowMs) {
        if (sample.sequence == 0 || sample.continuity == 0 ||
            !std::isfinite(sample.currentAmps) || !std::isfinite(sample.voltageVolts) ||
            !std::isfinite(sample.observedHz) || sample.currentAmps < 0) {
            fail(Failure::InvalidSample, nowMs); return false;
        }
        if (elapsed(nowMs, sample.atMs) > 100) {
            fail(Failure::StaleSample, nowMs); return false;
        }
        if (sample.observedHz < 45 || sample.observedHz > 65) {
            fail(Failure::SampleRate, nowMs); return false;
        }
        if (distance(sample.encoderCounts, state_.originCounts) > envelopeCounts) {
            fail(Failure::Envelope, nowMs); return false;
        }
        if (sample.currentAmps > 0.25f) {
            fail(Failure::Current, nowMs); return false;
        }
        if (sample.voltageVolts < 20 || sample.voltageVolts > 36) {
            fail(Failure::Voltage, nowMs); return false;
        }
        if (sample.alarm != 0 && sample.alarm != 0x10) {
            fail(Failure::Alarm, nowMs); return false;
        }
        return true;
    }
    Status state_;
    Sample last_;
    Quiet gpioQuiet_, finalQuiet_;
    bool used_ = false, haveSample_ = false;
    bool gpioPending_ = false, gpioActionIssued_ = false, gpioAsserted_ = false;
    bool serialPending_ = false, serialActionIssued_ = false;
    uint32_t gpioRequestedAtMs_ = 0;
    uint32_t serialDisableIssuedAtMs_ = 0, serialReadbackAtMs_ = 0;
};

}  // namespace serial_interlock_probe
