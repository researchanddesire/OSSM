#pragma once

#include "serial_interlock_probe.h"

// Separate from GPIO characterization: this proves one real UART-timer stop,
// then a clean new-epoch restoration. It never credits GPIO26 as a stop path.
// Runtime owns identity/corridor review, preflights, the crash sentinel, exact
// owner/TX provenance, and every physical effect. All methods are serialized.
namespace serial_timer_probe {
using Sample = serial_interlock_probe::Sample;

enum class Phase : uint8_t {
    Idle, Prepared, Moving, AwaitingIrq, AwaitingOff, PassiveSettling,
    AwaitingRestore, Restoring, ProvingRestoredQuiet, Passed, Failed
};
enum class Action : uint8_t { None, RequestTimerStop, RestoreTransport, EmergencyStop };
enum class Failure : uint8_t {
    None, Prerequisites, ActionOrder, InvalidSample, StaleSample, SampleRate,
    Continuity, Sequence, Envelope, Current, Voltage, Alarm, Speed,
    UnexpectedMotion, WrongDirection, TargetTooClose, NoMotion, UnexpectedWrite,
    IrqUnverified, OffUnverified, RestoreUnverified, OverallTimeout, SafetyFault,
    Cancelled
};
struct Status {
    Phase phase = Phase::Idle;
    Failure failure = Failure::None;
    bool active = false, finished = false, passed = false;
    bool enabled = false, targetWritten = false, irqStopProven = false;
    bool serialDisabled = false, offQuiet = false, restoreVerified = false;
    uint8_t positiveIntervals = 0, quietSamples = 0, restoredQuietSamples = 0;
    int32_t originCounts = 0, targetCounts = 0, lastCounts = 0;
    uint32_t startedAtMs = 0, irqAtMs = 0, offAtMs = 0, restoredContinuity = 0;
};
class Gate {
public:
    static constexpr int32_t targetDeltaCounts = 409;
    static constexpr int32_t envelopeCounts = 614;
    static constexpr int32_t cutProgressCounts = 82;
    static constexpr int32_t minimumRemainingCounts = 205;
    static constexpr int32_t quietRangeCounts = 4;

    bool start(int32_t origin, uint32_t atMs, bool reviewedPrerequisites) {
        if (used_) return false;
        used_ = true;
        state_.originCounts = state_.lastCounts = origin;
        state_.startedAtMs = atMs;
        const int64_t target = static_cast<int64_t>(origin) + targetDeltaCounts;
        if (!reviewedPrerequisites || target > INT32_MAX) {
            state_.failure = Failure::Prerequisites;
            state_.phase = Phase::Failed;
            return false; // no admitted authority: do not manufacture stop I/O
        }
        state_.targetCounts = static_cast<int32_t>(target);
        state_.phase = Phase::Prepared;
        return true;
    }
    bool markEnabled(uint32_t now) {
        tick(now);
        if (state_.phase != Phase::Prepared || !have_ || !fresh(now) ||
            distance(last_.encoderCounts, state_.originCounts) > quietRangeCounts)
            return fail(Failure::ActionOrder);
        state_.enabled = true;
        state_.phase = Phase::Moving;
        return true;
    }
    bool markTargetWritten(int32_t target, uint32_t now) {
        tick(now);
        if (state_.phase != Phase::Moving || !state_.enabled ||
            state_.targetWritten || target != state_.targetCounts)
            return fail(Failure::UnexpectedWrite);
        state_.targetWritten = true;
        return true;
    }
    void observe(const Sample &sample, uint32_t now) {
        tick(now);
        if (!used_ || terminal() || !notBefore(sample.atMs, state_.startedAtMs)) return;
        if (state_.phase == Phase::Restoring) return; // explicit off/quiet teardown only
        if (!accept(sample, now, state_.serialDisabled ? 30.0 : 2.0)) return;
        if (state_.phase == Phase::Prepared ||
            (state_.phase == Phase::Moving && !state_.targetWritten)) {
            if (distance(sample.encoderCounts, state_.originCounts) > quietRangeCounts)
                fail(Failure::UnexpectedMotion);
        } else if (state_.phase == Phase::Moving || state_.phase == Phase::AwaitingIrq ||
                   state_.phase == Phase::AwaitingOff) {
            if (static_cast<int64_t>(sample.encoderCounts) - state_.originCounts < -quietRangeCounts)
                fail(Failure::WrongDirection);
            else if (!clearOfTarget(sample)) fail(Failure::TargetTooClose);
            else if (state_.phase == Phase::Moving && state_.positiveIntervals >= 3 &&
                     static_cast<int64_t>(sample.encoderCounts) - state_.originCounts >= cutProgressCounts) {
                state_.phase = Phase::AwaitingIrq;
                triggerPending_ = true;
                triggerRequestedAt_ = now;
            }
        } else if (state_.phase == Phase::PassiveSettling) {
            state_.quietSamples = quiet_.add(sample);
            if (quiet_.ready(sample.atMs)) {
                state_.offQuiet = true;
                state_.phase = Phase::AwaitingRestore;
                restorePending_ = true;
            }
        } else if (state_.phase == Phase::AwaitingRestore) {
            // The emergency owner's longer quiet receipt may arrive later.
            // Our independent proof must still be current when restore starts.
            state_.quietSamples = quiet_.add(sample);
            state_.offQuiet = quiet_.ready(sample.atMs);
        } else if (state_.phase == Phase::ProvingRestoredQuiet) {
            state_.restoredQuietSamples = quiet_.add(sample);
            if (quiet_.ready(sample.atMs)) state_.phase = Phase::Passed;
        }
    }
    // Real IRQ/owner provenance must be verified by runtime, including exactly
    // one fixed off frame and no ordinary control writes after the checkpoint.
    bool markIrqStop(uint32_t irqAtMs, uint32_t now, bool actualIrqAndParkedOwner) {
        tick(now);
        if (state_.phase != Phase::AwaitingIrq || !triggerIssued_ ||
            !actualIrqAndParkedOwner || !notBefore(irqAtMs, triggerIssuedAt_) ||
            !notBefore(now, irqAtMs) || !have_ || !clearOfTarget(last_))
            return fail(Failure::IrqUnverified);
        state_.irqAtMs = irqAtMs;
        state_.irqStopProven = true;
        state_.phase = Phase::AwaitingOff;
        return true;
    }
    // The first coherent emergency-owner full frame must itself prove mode1,
    // output0, current epoch and target clearance. Only THEN is its data
    // classified as passive settling (30 mm/s ceiling), never powered2 mm/s.
    bool markOff(const Sample &coherent, uint16_t mode, uint16_t output,
                 uint32_t now, bool emergencyOwnerVerified) {
        tick(now);
        if (state_.phase != Phase::AwaitingOff || !state_.irqStopProven ||
            !emergencyOwnerVerified || mode != 1 || (output & 1U) != 0 ||
            !notBefore(coherent.atMs, state_.irqAtMs) || !clearOfTarget(coherent))
            return fail(Failure::OffUnverified);
        if (!accept(coherent, now, 30.0)) return false;
        // This is an explicit actuator-state boundary, not continuity repair.
        // It cannot hide powered samples: they must have been supplied before
        // the coherent verified-off frame, with the2 mm/s limit still active.
        state_.offAtMs = coherent.atMs;
        state_.serialDisabled = true;
        state_.enabled = false;
        state_.phase = Phase::PassiveSettling;
        quiet_.clear();
        state_.quietSamples = quiet_.add(coherent);
        return true;
    }
    bool markRestoreStarted(uint32_t now) {
        tick(now);
        if (state_.phase != Phase::AwaitingRestore || !restoreIssued_ || !state_.offQuiet)
            return fail(Failure::ActionOrder);
        preRestoreContinuity_ = last_.continuity;
        restoreStartedAt_ = now;
        state_.phase = Phase::Restoring;
        have_ = false; // planned, output-off driver teardown; no stale proof
        return true;
    }
    bool markRestored(const Sample &coherent, uint16_t mode, uint16_t output,
                      uint32_t now, bool qualifiedCleanRestore) {
        tick(now);
        if (state_.phase != Phase::Restoring || !qualifiedCleanRestore ||
            mode != 1 || (output & 1U) != 0 ||
            coherent.continuity == preRestoreContinuity_ ||
            !notBefore(coherent.atMs, restoreStartedAt_))
            return fail(Failure::RestoreUnverified);
        if (!valid(coherent, now)) return false;
        last_ = coherent;
        have_ = true;
        state_.lastCounts = coherent.encoderCounts;
        state_.restoredContinuity = coherent.continuity;
        state_.restoreVerified = true;
        state_.phase = Phase::ProvingRestoredQuiet;
        quiet_.clear();
        state_.restoredQuietSamples = quiet_.add(coherent);
        return true;
    }
    void tick(uint32_t now) {
        if (!used_ || terminal() || !notBefore(now, state_.startedAtMs)) return;
        if (now - state_.startedAtMs >= 3000) { fail(Failure::OverallTimeout); return; }
        if (have_ && !fresh(now)) { fail(Failure::StaleSample); return; }
        if ((state_.phase == Phase::Prepared || state_.phase == Phase::Moving) &&
            now - state_.startedAtMs >= 1000) { fail(Failure::NoMotion); return; }
        if ((state_.phase == Phase::AwaitingIrq || state_.phase == Phase::AwaitingOff) &&
            now - triggerRequestedAt_ > 100) fail(Failure::IrqUnverified);
    }
    Action takeAction(uint32_t now) {
        tick(now);
        // A failure always wins over a queued restore, including after timeout.
        if (emergencyPending_) { emergencyPending_ = false; return Action::EmergencyStop; }
        if (terminal()) return Action::None;
        if (triggerPending_) {
            triggerPending_ = false; triggerIssued_ = true; triggerIssuedAt_ = now;
            return Action::RequestTimerStop;
        }
        if (restorePending_) {
            restorePending_ = false; restoreIssued_ = true;
            return Action::RestoreTransport;
        }
        return Action::None;
    }
    void abort(Failure reason) {
        if (used_ && !terminal()) fail(reason == Failure::None ? Failure::SafetyFault : reason);
    }
    Status status() const {
        Status result = state_;
        result.finished = terminal();
        result.active = used_ && !result.finished;
        result.passed = state_.phase == Phase::Passed && state_.failure == Failure::None &&
            state_.irqStopProven && state_.serialDisabled && state_.offQuiet && state_.restoreVerified;
        return result;
    }
private:
    struct Quiet {
        uint32_t at = 0;
        int32_t low = 0, high = 0;
        uint8_t count = 0;
        void clear() { count = 0; }
        uint8_t add(const Sample &sample) {
            if (!count || distance(sample.encoderCounts, low) > quietRangeCounts ||
                distance(sample.encoderCounts, high) > quietRangeCounts) {
                at = sample.atMs; low = high = sample.encoderCounts; count = 1;
            } else {
                if (sample.encoderCounts < low) low = sample.encoderCounts;
                if (sample.encoderCounts > high) high = sample.encoderCounts;
                if (count < 255) ++count;
            }
            return count;
        }
        bool ready(uint32_t atMs) const { return count >= 6 && atMs - at >= 100; }
    };
    static int64_t distance(int32_t a, int32_t b) {
        const int64_t d = static_cast<int64_t>(a) - b; return d < 0 ? -d : d;
    }
    static bool notBefore(uint32_t a, uint32_t b) { return a - b < 0x80000000U; }
    bool terminal() const { return state_.phase == Phase::Passed || state_.phase == Phase::Failed; }
    bool fresh(uint32_t now) const { return now - last_.atMs <= 100; }
    bool clearOfTarget(const Sample &s) const {
        return static_cast<int64_t>(state_.targetCounts) - s.encoderCounts >= minimumRemainingCounts;
    }
    bool fail(Failure reason) {
        if (terminal()) return false;
        if (state_.failure == Failure::None) state_.failure = reason;
        state_.phase = Phase::Failed;
        emergencyPending_ = true;
        triggerPending_ = restorePending_ = false;
        return false;
    }
    bool valid(const Sample &s, uint32_t now) {
        if (!s.sequence || !s.continuity || !std::isfinite(s.currentAmps) ||
            !std::isfinite(s.voltageVolts) || !std::isfinite(s.observedHz) || s.currentAmps < 0)
            return fail(Failure::InvalidSample);
        if (now - s.atMs > 100) return fail(Failure::StaleSample);
        if (s.observedHz < 45 || s.observedHz > 65) return fail(Failure::SampleRate);
        if (distance(s.encoderCounts, state_.originCounts) > envelopeCounts) return fail(Failure::Envelope);
        if (s.currentAmps > .25f) return fail(Failure::Current);
        if (s.voltageVolts < 20 || s.voltageVolts > 36) return fail(Failure::Voltage);
        // No intentional contact is part of this finite commissioning probe.
        if (s.alarm != 0) return fail(Failure::Alarm);
        return true;
    }
    bool accept(const Sample &s, uint32_t now, double limitMmS) {
        if (!valid(s, now)) return false;
        if (have_) {
            if (s.continuity != last_.continuity) return fail(Failure::Continuity);
            const uint32_t deltaSequence = s.sequence - last_.sequence;
            if (!deltaSequence) {
                if (s.atMs != last_.atMs || s.encoderCounts != last_.encoderCounts ||
                    s.currentAmps != last_.currentAmps || s.voltageVolts != last_.voltageVolts ||
                    s.alarm != last_.alarm)
                    fail(Failure::Sequence);
                return false;
            }
            if (deltaSequence >= 0x80000000U) return fail(Failure::Sequence);
            const uint32_t gap = s.atMs - last_.atMs;
            if (!gap || gap > 100) return fail(Failure::StaleSample);
            const int64_t delta = static_cast<int64_t>(s.encoderCounts) - last_.encoderCounts;
            if (std::abs(static_cast<double>(delta)) * 1000 > limitMmS * 819.2 * gap)
                return fail(Failure::Speed);
            if (state_.phase == Phase::Moving && state_.targetWritten)
                state_.positiveIntervals = delta > 0
                    ? static_cast<uint8_t>(state_.positiveIntervals < 255 ? state_.positiveIntervals + 1 : 255) : 0;
        }
        last_ = s; have_ = true; state_.lastCounts = s.encoderCounts;
        return true;
    }
    Status state_;
    Sample last_;
    Quiet quiet_;
    bool used_ = false, have_ = false;
    bool triggerPending_ = false, triggerIssued_ = false, restorePending_ = false;
    bool restoreIssued_ = false, emergencyPending_ = false;
    uint32_t triggerRequestedAt_ = 0, triggerIssuedAt_ = 0;
    uint32_t restoreStartedAt_ = 0, preRestoreContinuity_ = 0;
};
} // namespace serial_timer_probe
