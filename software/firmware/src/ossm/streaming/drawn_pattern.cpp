#include "drawn_waveform.h"
#include "drawn_modifier_motion.h"
#include "drawn_phase_rate.h"
#include "drawn_pattern.h"
#include "drawn_frame_planner.h"

#include <atomic>
#ifdef ESP_PLATFORM
#include <esp_timer.h>
#else
#include <chrono>
#endif
#include <cstring>
#include <mutex>
#include <memory>
#include <new>

#include "drawn_pattern_motor.h"
#include "streaming_logic.h"
#include "../state/ble.h"
#include "../state/settings.h"
#include "../../services/communication/motion_link.h"

namespace streaming {
uint32_t patternServiceMicros() {
#ifdef ESP_PLATFORM
    // The IDF clock already runs in microseconds. Do not rebuild a POSIX
    // timespec and divide a 64-bit nanosecond count for each pulse receipt.
    return static_cast<uint32_t>(esp_timer_get_time());
#else
    return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
#endif
}

namespace {
enum class Stage { Idle, Waiting, Positioning, EntryBraking, Playing, Draining, Paused };
std::mutex lock;
// A same-owner Stop or edit waiting on the mailbox takes precedence over
// optional refill writes. The actual physical guard revokes independently.
std::atomic<unsigned> pendingControl{0};
struct ControlRequest {
    ControlRequest() { ++pendingControl; }
    ~ControlRequest() { --pendingControl; }
};
drawn_pattern::Waveform curve;
Stage stage = Stage::Idle;
bool ready = false, local = false, serial = false;
uint64_t owner = 0;
PatternStop lastStop;
PatternWork work;
PatternFeedGap feedGap;
uint32_t lastAppendUs = 0;

void recordEmptyFrame() {
    if (!feedGap.frame) return;
    const uint32_t offset = patternServiceMicros() - lastAppendUs;
    if (!feedGap.emptyReads) feedGap.firstEmptyUs = offset;
    ++feedGap.emptyReads;
    feedGap.lastEmptyUs = offset;
}

void recordPulseSubmit(const drawn_pattern::FrameResult &result, uint32_t frame, uint32_t atUs) {
    if (result.state == drawn_pattern::Frame::Accepted && result.queuedUs != UINT16_MAX) {
        const auto previousQueued = feedGap.queuedUs;
        const auto previousInterval = feedGap.frame ? atUs - lastAppendUs : 0;
        const auto pairs = feedGap.combinedHoldFrames + result.frames - 1;
        feedGap = {};
        feedGap.previousQueuedUs = previousQueued;
        feedGap.previousAppendIntervalUs = previousInterval;
        feedGap.combinedHoldFrames = pairs;
        feedGap.frame = frame;
        feedGap.queuedUs = result.queuedUs;
        if (result.driverResult > 0) {
            // The step was accepted but FAS refused its optional waiting tail.
            feedGap.otherRetries = 1;
            feedGap.lastOtherDriverCode = result.driverResult;
        }
        lastAppendUs = atUs;
    } else if (feedGap.frame && result.state == drawn_pattern::Frame::Retry) {
        if (result.reason && std::strcmp(result.reason, "pulse_full") == 0) {
            ++feedGap.fullRetries;
            feedGap.lastFullUs = atUs - lastAppendUs;
        } else {
            ++feedGap.otherRetries;
            feedGap.lastOtherDriverCode = result.driverResult;
            feedGap.lastOtherOffsetUs = atUs - lastAppendUs;
        }
    }
}
drawn_pattern::PulseRefillSchedule refillSchedule;
PatternSpeedStatus speedStatus;
double goalRate = 0, stampedGoal = -1;
uint32_t drainAt = 0, entryStopBudget = 0;
unsigned stationaryFrames = 0;

uint32_t lastServiceEnd = 0;
bool haveServiceEnd = false;
uint32_t startedAt = 0, renewedAt = 0, lastFrameAt = 0, lastProgressAt = 0, effectivePeriod = 0;
int32_t lower = 0, upper = 0, entry = 0;
float windowRail = 0;
using frame_plan::Marking;
using frame_plan::RawFramePlanner;
std::unique_ptr<Marking> serialMarkers;
Marking *activeMarkers = nullptr;
RawFramePlanner *rawPlanner = nullptr;
bool rawClockStarted = false, fillingRaw = false;
const drawn_pattern::MarkerIndex unmarked{};

drawn_pattern::MarkedPhase motionPhase;
drawn_pattern::VibrationClock vibration;
drawn_pattern::VibrationBudget vibrationBudget;
bool reserveVibration = false, reconfigureAfterFade = false;
double lastStrength = -1;
double lastSpeed = 0, lastAcceleration = 0;
double clockAcceleration = 0, clockJerk = 0;
// Serial accepts one ACK-gated 20 ms sample. STEP/DIR uses the private
// planner ring; only the feeder ever submits one of its completed rows.
std::unique_ptr<frame_plan::Frame> pending;
drawn_pattern::StationaryEntryClock entryClock;
void invalidateFrames(bool warmup = false) {
    entryClock.reset();
    if (rawPlanner) rawPlanner->invalidate(warmup);
    if (pending) pending->valid = false;
}
int32_t committedTarget = 0;
bool committedTargetValid = false;
bool softwareEntry = false;

constexpr unsigned rateHistorySize = 8;
constexpr uint32_t pulseLeadBoundMs = drawn_pattern::pulseStopDrainMs + drawn_pattern::pulseFrameMs;
PatternRateStamp rateHistory[rateHistorySize]{};
bool awaitingStart[rateHistorySize]{};
unsigned newestRateStamp = 0;
uint32_t rateSequence = 0, lastAppliedPeriod = 0, acceptedFrames = 0;
double lastStampedStrength = -1;

void observeQueueStart(uint32_t now) {
    for (unsigned i=0; i<rateHistorySize; ++i) {
        if (!awaitingStart[i]) continue;
        auto &stamp = rateHistory[i];
        const uint32_t waiting = now-stamp.atMs;
        if (waiting <= UINT16_MAX-pulseLeadBoundMs) {
            stamp.leadBoundMs = static_cast<uint16_t>(waiting+pulseLeadBoundMs);
            stamp.leadKnown = true;
        }
        awaitingStart[i] = false;
    }
}

void recordRateGoal(uint32_t now, drawn_pattern::FrameTiming timing, double strength, double nextGain, bool handoff,
                    const PatternSpeedStatus &applied) {
    using drawn_pattern::FrameTiming;
    newestRateStamp = (newestRateStamp+1)%rateHistorySize;
    if (++rateSequence==0) ++rateSequence;
    const bool running = timing==FrameTiming::Started || timing==FrameTiming::Running;
    auto &stamp = rateHistory[newestRateStamp];
    stamp = {rateSequence,
             now,
             applied.periodMs,
             static_cast<float>(motionPhase.phase),
             static_cast<float>(motionPhase.clock.rate()),
             static_cast<float>(motionPhase.clock.acceleration()),
             static_cast<uint16_t>(running ? pulseLeadBoundMs : 0),
             acceptedFrames == 0,
             serial,
             running || timing == FrameTiming::EntryHold,
             static_cast<float>(strength),
             static_cast<float>(vibration.gain.value()),
             static_cast<float>(nextGain),
             static_cast<float>(vibration.carrier),
             acceptedFrames, applied.phaseRateHz, handoff};
    stampedGoal = applied.phaseRateHz;
    lastStampedStrength = strength;
    awaitingStart[newestRateStamp] = timing==FrameTiming::Prefill;
    lastAppliedPeriod = applied.periodMs;
}

// These values live behind lock. A task from an older streaming session
// can neither submit a frame nor release a newer session's mailbox.
const char *stageName() {
    switch (stage) {
        case Stage::Waiting: return "waiting";
        case Stage::Positioning: return "positioning";
        case Stage::EntryBraking: return "positioning";
        case Stage::Draining:
        case Stage::Paused:
        case Stage::Playing: return "playing";
        default: return "idle";
    }
}
void stopLocked(const char *reason, uint32_t now, int8_t driverResult = 0) {
    if (stage == Stage::Idle) return; // preserve the first stop until the next accepted Play
    if (feedGap.frame) feedGap.elapsedUs = patternServiceMicros() - lastAppendUs;
    lastStop = {reason, stageName(), now, now - startedAt, driverResult,
                stage == Stage::Playing ? now - lastFrameAt : 0};
    if (stage != Stage::Waiting) pattern_motor::stop(owner);
    stage = Stage::Idle;
    activeMarkers = nullptr;
    serialMarkers.reset();
    if (rawPlanner) {
        frame_plan::RawFramePlanner::Counters counts;
        rawPlanner->close(counts);
        work.prepared = counts.produced;
        work.discarded = counts.discarded;
        work.emptyTurns = counts.emptyTurns;
        work.maxPrepared = counts.peakDepth;
        work.plannerStackFree = counts.stackFree;
        work.pauseComputed = counts.pauseComputed;
        work.pauseAdopted = counts.pauseAdopted;
        work.pauseChanges = counts.pauseChanges;
        work.pauseMaxUs = counts.pauseMaxUs;
        feedGap.assistAttempts = counts.assistAttempts;
        feedGap.assistPublished = counts.assistPublished;
        feedGap.assistMaxUs = counts.assistMaxUs;
        rawPlanner = nullptr;
    }
    pending.reset();
    for (auto &waiting : awaitingStart) waiting = false;
    effectivePeriod = 0;
    speedStatus = {};
}

bool confirmWindow(int32_t from, int32_t to, bool zeroSpeed, uint32_t now) {
    if (to <= from) {
        stopLocked("limits_zero", now);
        return false;
    }
    if (from == lower && to == upper) return true;
    // A still-waiting Play may adopt a valid window at zero Speed.
    // Once Speed is positive, a different window needs its own Play.
    if (stage == Stage::Waiting && zeroSpeed) {
        lower = from;
        upper = to;
        return true;
    }
    stopLocked("window_changed", now);
    return false;
}

void recordPlan(frame_plan::Frame &frame) {
    if (frame.offered) return;
    frame.offered = true;
    const auto planUs = frame.planUs;
    ++work.plans;
    work.planTotalUs += planUs;
    if (frame.from != frame.target) {
        ++work.movingPlans;
        work.movingPlanTotalUs += planUs;
        work.maxMovingPlanUs = std::max(work.maxMovingPlanUs, planUs);
    }
    if (planUs > 2000) ++work.planOver2ms;
    work.maxPlanUs = std::max(work.maxPlanUs, planUs);
}

bool mayContinueBurst() {
    return pendingControl.load() == 0 && (local || pattern_motor::now() - renewedAt < drawn_pattern::leaseMs);
}

void service(uint32_t now, unsigned pulseFramesPerTurn, unsigned allowance,
             bool &lowAfterBatch, RawFramePlanner *&assist) {
    if (!local && now - renewedAt >= drawn_pattern::leaseMs) return stopLocked("lease_expired", now);
    const uint32_t readStart = patternServiceMicros();
    auto caps = pattern_motor::read(owner, activeMarkers != nullptr);
    if (!activeMarkers) caps.strength = 0;
    if (caps.ready && (stage == Stage::Positioning || stage == Stage::EntryBraking ||
                       (stage == Stage::Waiting && caps.fraction > 0))) {
        pattern_motor::readFiniteState(caps, stage != Stage::Positioning);
    }
    work.lastService.readUs = patternServiceMicros() - readStart;
    const auto &markerIndex = activeMarkers ? activeMarkers->index : unmarked;
    if (!caps.ready) return stopLocked("guard", now);
    if (caps.serial != serial) return stopLocked("backend_changed", now);
    const bool zero = caps.fraction == 0;
    if (!confirmWindow(caps.from, caps.to, zero, now)) return;
    if (!(caps.fullSpeed > 0 && caps.stepsPerMm > 0 && caps.fraction >= 0 && caps.fraction <= 1 &&
          caps.acceleration >= 0 && (caps.acceleration >= 1 || zero))) {
        if (stage != Stage::Waiting) stopLocked("limits_zero", now);
        return;
    }
    const bool pendingPositive = rawPlanner && rawPlanner->positivePending();
    const bool limitsChanged = caps.speed != lastSpeed || caps.acceleration != lastAcceleration;
    const bool strengthChanged = caps.strength != lastStrength;
    // Only an ordinary positive Speed edit can retain an outgoing prefix.
    // Explicit Stop/revocation is checked outside and inside the motor
    // submit. Zero, acceleration or strength edits discard the prefix.
    const bool positiveEdit = rawPlanner && stage == Stage::Playing && rawClockStarted && !zero &&
        lastSpeed > 0 && caps.speed != lastSpeed && caps.acceleration == lastAcceleration && !strengthChanged;
    const bool superseded = pendingPositive && (limitsChanged || strengthChanged || zero);
    if (positiveEdit && !pendingPositive) {
        rawPlanner->previousSpeed = speedStatus;
        rawPlanner->previousBudget = vibrationBudget;
    } else if (superseded) {
        vibrationBudget = rawPlanner->previousBudget;
    }
    speedStatus.speedPct = 100. * caps.fraction;
    speedStatus.nominalMmS = caps.nominalSpeed * caps.fraction / caps.stepsPerMm;
    if (zero) {
        goalRate = 0;
        effectivePeriod = speedStatus.periodMs = 0;
        speedStatus.phaseRateHz = speedStatus.plannedPeakMmS = speedStatus.steadyJerkLimitMmS3 = 0;
        if (stage == Stage::Waiting) {
            windowRail = caps.rail;
            speedStatus.state = "waiting";
            return;
        }
        speedStatus.state = stage == Stage::Paused ? "paused" : "pausing";
    }
    if (stage == Stage::Positioning && (zero || (caps.speed < 1 && caps.moving))) {
        if (!pattern_motor::pauseEntry(owner)) return stopLocked("entry_guard", now);
        stage = Stage::EntryBraking;
        drainAt = now;
        entryStopBudget = static_cast<uint32_t>(std::min(double(UINT32_MAX),
            1000. + std::ceil(1000. * lastSpeed / std::max(1., lastAcceleration))));
    }
    if (stage == Stage::EntryBraking) {
        if (caps.moving) {
            if (now - drainAt >= entryStopBudget) stopLocked("entry_pause", now);
            return;
        }
        stage = Stage::Waiting;
        speedStatus.state = "waiting";
        if (zero || caps.speed < 1) return;
    }
    if (stage == Stage::Draining || (stage == Stage::Paused && !zero)) {
        const auto parked = pattern_motor::park(owner, committedTarget, true);
        if (parked != pattern_motor::Park::Parked) {
            if (parked != pattern_motor::Park::Draining || stage == Stage::Paused ||
                now - drainAt >= (serial ? drawn_pattern::frameStarvationMs : drawn_pattern::pulseStopDrainMs))
                stopLocked("park_position", now);
            return;
        }
        stage = Stage::Paused;
        speedStatus.state = "paused";
        invalidateFrames(true);
        rawClockStarted = fillingRaw = false;
        if (!zero) {
            stage = Stage::Playing;
            softwareEntry = !serial;
            lastProgressAt = lastFrameAt = now;
        }
    }
    if (stage == Stage::Paused) return;
    const bool requestedVibration = !zero && markerIndex.hasVibration && caps.strength > 0;
    const bool reserve = markerIndex.hasVibration;
    const bool startRawPause = zero && lastSpeed > 0 && rawClockStarted && rawPlanner &&
        committedTargetValid && !pendingPositive && !strengthChanged && caps.acceleration == lastAcceleration;
    if (zero && lastSpeed != 0) {
        if (!startRawPause) invalidateFrames();
        lastSpeed = 0;
    }
    const bool refreshVibration = !pendingPositive && (reserve != reserveVibration ||
        (reconfigureAfterFade && vibration.gain.atZero()));
    if (!zero && (stage == Stage::Waiting || refreshVibration || limitsChanged || superseded)) {
        auto boundaryVibration = vibration;
        if (positiveEdit) {
            if (!pattern_motor::pulseClockRunning()) return stopLocked("pulse_underrun", pattern_motor::now());
            rawPlanner->retarget(boundaryVibration);
        } else {
            invalidateFrames();
        }
        const bool nextReserve = markerIndex.hasVibration;
        committedTargetValid = false;
        const double span = static_cast<double>(caps.to) - caps.from;
        // A changing vibration budget fades out at its old safe amplitude.
        const bool fading = !boundaryVibration.gain.atZero() && !vibrationBudget.matches(caps.speed, caps.acceleration);
        const auto pace = curve.pace(span, caps.fullSpeed, caps.acceleration, caps.fraction,
                                     serial ? 20 : drawn_pattern::pulseFrameMs);
        if (!pace.periodMs) return stopLocked("period_limit", now);
        effectivePeriod = speedStatus.periodMs = pace.periodMs;
        goalRate = speedStatus.phaseRateHz = pace.rate;
        speedStatus.plannedPeakMmS = pace.peak / caps.stepsPerMm;
        speedStatus.steadyJerkLimitMmS3 = pace.shapeJerkLimit / caps.stepsPerMm;
        speedStatus.limit = pace.limit;
        if (std::strcmp(pace.limit, "speed") == 0 && caps.fullSpeed < caps.nominalSpeed)
            speedStatus.limit = "backend";
        clockAcceleration = pace.phaseAcceleration;
        clockJerk = pace.phaseJerk;
        if (nextReserve && !fading) {
            // A gain edit may finish its current ramp before fading to zero.
            // From the last frame that requested positive gain, the base can
            // accelerate for at most two gain ramps plus one sampled frame.
            const double fadeSeconds = 2 * drawn_pattern::StrengthRamp::seconds +
                (serial ? .020 : drawn_pattern::pulseFrameMs * .001);
            const double reachableRate = std::min(goalRate / caps.fraction,
                                                   goalRate + clockAcceleration * fadeSeconds);
            vibrationBudget = {span, caps.speed, caps.acceleration, caps.stepsPerMm,
                               {reachableRate, clockAcceleration, clockJerk},
                               pace.fullPathJerkLimit, caps.fullSpeed, goalRate,
                               curve.maximumSlope(span), curve.maximumCurvature(span), curve.maximumThird(span)};
        }
        if (markerIndex.holds)
            activeMarkers->brakeBounds.reset({goalRate, clockAcceleration, clockJerk});
        reserveVibration = nextReserve;
        reconfigureAfterFade = fading;
        lastSpeed = caps.speed;
        lastAcceleration = caps.acceleration;
        stationaryFrames = 0;
    }
    if (caps.strength != lastStrength) {
        invalidateFrames();
        committedTargetValid = false;
        lastStrength = caps.strength;
    }
    if (stage == Stage::Waiting) {
        if (caps.moving || caps.queued) return stopLocked("entry_busy", now);
        entry = curve.nativeTarget(0, lower, upper);
        motionPhase = {};
        // The finite approach API takes whole steps per second. At a
        // positive sub-Hz setting stay stationary until it can represent
        // the approach; a curve already at its anchor can run its true rate.
        if (caps.speed < 1 && caps.position != entry) return;
        if (!pattern_motor::position(owner, entry, caps, true)) return stopLocked("entry_command", now);
        stage = Stage::Positioning;
        lastFrameAt = now;
        return;
    }
    if (stage == Stage::Positioning) {
        // The finite backend retains its caps. Keep the safety/owner check
        // each tick but write the driver only for an actual live adjustment,
        // including one that arrives before the first pulse.
        if (limitsChanged && !pattern_motor::position(owner, entry, caps, false))
            return stopLocked("entry_guard", now);
        if (caps.moving) return;
        if (std::abs(static_cast<int64_t>(caps.position) - entry) > 1) {
            // A finite command was accepted, but the backend may not yet
            // report motion. Allow one backend update without resubmitting it.
            if (now - lastFrameAt < 20) return;
            return stopLocked("entry_position", now);
        }
        stage = Stage::Playing;
        lastProgressAt = lastFrameAt = now;
    }
    speedStatus.state = zero ? "pausing" : "running";
    if (now - lastProgressAt > drawn_pattern::frameStarvationMs) return stopLocked("scheduler_gap", now); // stop after starvation; never burst to catch up
    const uint32_t frameMs = serial ? 20 : drawn_pattern::pulseFrameMs;
    if (serial && now - lastFrameAt < frameMs) return;
    const auto inputs = [&]() {
        frame_plan::Inputs in;
        in.limits = {goalRate, clockAcceleration, clockJerk};
        in.budget = vibrationBudget;
        if (markerIndex.holds) in.brake = activeMarkers->brakeBounds;
        in.strength = caps.strength;
        in.dt = frameMs / 1000.;
        in.lower = lower;
        in.upper = upper;
        in.zero = zero;
        in.serial = serial;
        in.vibrationEnabled = requestedVibration && vibrationBudget.matches(caps.speed, caps.acceleration);
        return in;
    };
    const auto cursor = [&]() {
        return frame_plan::Cursor{motionPhase, vibration, committedTarget, committedTargetValid};
    };
    if (startRawPause) rawPlanner->pause(inputs(), cursor());
    else if (rawPlanner && !rawPlanner->configured()) rawPlanner->configure(inputs(), cursor());

    const unsigned firstOrdinal = acceptedFrames;
    for (unsigned i = 0; i < (serial ? 1u : pulseFramesPerTurn); ++i) {
        if (pendingControl.load()) return;
        const auto beforeWrite = pattern_motor::now();
        if (!local && beforeWrite - renewedAt >= drawn_pattern::leaseMs)
            return stopLocked("lease_expired", beforeWrite);
        const unsigned issued = acceptedFrames - firstOrdinal;
        if (issued >= allowance) return;
        const unsigned remaining = allowance - issued;
        auto *next = rawPlanner ? rawPlanner->front() : pending.get();
        if (!next) {
            recordEmptyFrame();
            // A new goal can temporarily empty the software ring. Never
            // restart a peripheral which actually ran out of timed pulses.
            if (rawClockStarted) {
                if (!pattern_motor::pulseClockRunning()) return stopLocked("pulse_underrun", pattern_motor::now());
                if (rawPlanner->requestAssist(0)) assist = rawPlanner;
                // A concurrent publication or this one-row assist makes a
                // freshly guarded next turn useful without extending the tail.
                lowAfterBatch = true;
            }
            return;
        }
        auto &frame = *next;
        if (!frame.valid) {
            const auto in = inputs();
            frame_plan::calculate(in, cursor(), frame, curve, activeMarkers ? &activeMarkers->index : nullptr);
        }
        if (softwareEntry && committedTargetValid && frame.target == committedTarget &&
            !entryClock.due(beforeWrite)) return;
        recordPlan(frame);
        const uint32_t submitStart = patternServiceMicros();
        // First establish that an actual raw command was accepted. Finite
        // priming and unowned stationary entry retain their own clocks.
        const bool prepared = (rawClockStarted || fillingRaw) && rawPlanner && !softwareEntry;
        drawn_pattern::PreparedPulses successors;
        drawn_pattern::PreparedPulses::Target first{frame.target, frame.crossing, 0, frame.from == frame.target};
        if (prepared && (rawClockStarted || remaining > 1)) {
            const bool endAtQuiet = !zero && reserveVibration && reconfigureAfterFade;
            successors.initialRows = remaining;
            const auto lookahead = remaining + (rawClockStarted ? drawn_pattern::PreparedPulses::horizonRows : 0);
            rawPlanner->successors(successors, lookahead - 1, endAtQuiet, zero);
            const auto &firstGoal = rawPlanner->retainedFront() ? rawPlanner->previousSpeed : speedStatus;
            if (stampedGoal != firstGoal.phaseRateHz || lastAppliedPeriod != firstGoal.periodMs ||
                (activeMarkers && (lastStampedStrength != caps.strength ||
                 frame.vibration.gain.goal() != vibration.gain.goal()))) {
                successors.gainBoundaries |= 1u;
            }
            successors.groupWaits(first);
        }

        drawn_pattern::PulseBatch batch;
        // An urgent first braking row can arrive before its successors. It
        // still rechecks the live controls inside the actual hardware gate.
        if (successors.count || (zero && rawClockStarted && prepared)) {
            pattern_motor::submitPrepared(batch, first, successors, owner, caps, mayContinueBurst);
        } else {
            batch.count = 1;
            batch.attempts[0].result = pattern_motor::submit(frame.target, owner, serial, frame.crossing, 0);
            batch.attempts[0].atUs = patternServiceMicros();
        }

        if (!batch.count) {
            const auto at = pattern_motor::now();
            if (!local && at - renewedAt >= drawn_pattern::leaseMs) stopLocked("lease_expired", at);
            // A wired change could have overtaken our input snapshot while
            // waiting for the motor gate. Sample it again promptly while
            // the peripheral is low; a waiting Stop still gets the mailbox.
            lowAfterBatch = batch.controlsChanged && rawClockStarted && !pendingControl.load() &&
                            work.lastQueuedUs < drawn_pattern::refillBelowUs;
            return;
        }
        auto result = batch.attempts[0].result;
        unsigned committed = 0;
        uint32_t since = submitStart;
        const auto &last = batch.attempts[batch.count - 1].result;
        for (unsigned n = 0; n < batch.count; ++n) {
            const auto &attempt = batch.attempts[n];
            const auto &row = attempt.result;
            const uint32_t submitUs = attempt.atUs - since;
            since = attempt.atUs;
            work.lastService.commandGateUs = std::max(work.lastService.commandGateUs, row.commandGateUs);
            ++work.submissions;
            work.submitTotalUs += submitUs;
            if (submitUs > 2000) ++work.submitOver2ms;
            work.maxSubmitUs = std::max(work.maxSubmitUs, submitUs);
            if (row.state == pattern_motor::Frame::Accepted) {
                committed += row.frames;
                result = row;
                if (row.timing == drawn_pattern::FrameTiming::Running)
                    work.minRunningUs = std::min(work.minRunningUs, row.queuedUs);
                if (row.queuedUs != UINT16_MAX) work.maxQueuedUs = std::max(work.maxQueuedUs, row.queuedUs);
            }
            if (!serial) recordPulseSubmit(row, acceptedFrames + committed, attempt.atUs);
        }
        if (committed) result.frames = static_cast<uint8_t>(committed);
        const bool started = result.timing == drawn_pattern::FrameTiming::Started ||
                             last.timing == drawn_pattern::FrameTiming::Started;
        if (started) {
            // Starting a full prefill can leave this prospective frame pending.
            rawClockStarted = true;
            fillingRaw = false;
        }
        if (result.state == pattern_motor::Frame::Rejected)
            return stopLocked(result.reason, pattern_motor::now(), result.driverResult);
        if (result.state == pattern_motor::Frame::Retry) {
            if (started) observeQueueStart(pattern_motor::now());
            ++work.retries;
            if (result.timing == drawn_pattern::FrameTiming::PrimeTail) lastProgressAt = now;
            if (now - lastProgressAt >= drawn_pattern::frameStarvationMs)
                return stopLocked(result.reason, now, result.driverResult);
            // The driver has declined an append for lack of space. Use that
            // slack to restore the software reserve before the feeder can
            // run short during a curve or a live speed handoff. The next submit rechecks every owner/guard/clock.
            if (rawClockStarted && rawPlanner && result.reason &&
                std::strcmp(result.reason, "pulse_full") == 0 && rawPlanner->requestAssist(RawFramePlanner::reserveForAssist))
                assist = rawPlanner;
            return;
        }
        if (result.queuedUs != UINT16_MAX) {
            work.lastQueuedUs = result.queuedUs;
            lowAfterBatch = (i + 1 == pulseFramesPerTurn || batch.count > 1 ||
                             acceptedFrames - firstOrdinal + committed == allowance) &&
                            result.queuedUs < drawn_pattern::refillBelowUs;
        }
        bool retainedGoal = false;
        const bool entryToRaw = softwareEntry &&
            (result.timing == drawn_pattern::FrameTiming::Prefill ||
             result.timing == drawn_pattern::FrameTiming::Started ||
             result.timing == drawn_pattern::FrameTiming::Running);
        unsigned attemptIndex = 0, nextAttemptRow = batch.attempts[0].result.frames;
        for (unsigned consumed = 0; consumed < result.frames; ++consumed) {
            if (consumed == nextAttemptRow) {
                ++attemptIndex;
                nextAttemptRow += batch.attempts[attemptIndex].result.frames;
            }
            if (consumed) recordPlan(*next);
            const auto &accepted = *next;
            const bool boundary = successors.count && successors.boundaryAt(consumed);
            retainedGoal = rawPlanner && rawPlanner->retainedFront();
            const auto &applied = retainedGoal ? rawPlanner->previousSpeed : speedStatus;
            if ((consumed == 0 || boundary) &&
                (entryToRaw || stampedGoal != applied.phaseRateHz || lastAppliedPeriod != applied.periodMs ||
                (activeMarkers && (lastStampedStrength != caps.strength ||
                                   vibration.gain.goal() != accepted.vibration.gain.goal())))) {
                // A single guarded refill can cross a live rate boundary.
                // Its exact logical ordinal, incoming cursor and acceptance
                // time remain distinct even if more rows then reach FAS.
                const auto at = boundary ? batch.attempts[attemptIndex].boundaryAtMs : pattern_motor::now();
                recordRateGoal(at, batch.attempts[attemptIndex].result.timing, caps.strength,
                               accepted.vibration.gain.goal(), entryToRaw, applied);
            }
            const bool finalRow = consumed + 1 == result.frames;
            if (finalRow || (successors.count && successors.boundaryAt(consumed + 1))) {
                // Only a law boundary and the final cursor need full state
                // copies. Ordinary steady bursts still copy it once.
                motionPhase = accepted.phase;
                vibration = accepted.vibration;
            }
            if (finalRow) {
                const bool settled = zero && accepted.phase.clock.secondsToGoal() == 0 &&
                    accepted.phase.clock.rate() == 0 && accepted.vibration.gain.atZero() && accepted.from == accepted.target;
                stationaryFrames = settled ? stationaryFrames + 1 : 0;
                committedTarget = accepted.target;
            }
            ++acceptedFrames;
            if (rawPlanner) next = rawPlanner->pop(!finalRow);
            else frame.valid = false;
        }
        if (started) observeQueueStart(pattern_motor::now());
        fillingRaw = !started && result.timing == drawn_pattern::FrameTiming::Prefill;
        if (entryToRaw) work.rawStartSequence = rateSequence;
        softwareEntry = result.timing == drawn_pattern::FrameTiming::EntryHold;
        if (softwareEntry) entryClock.accepted(pattern_motor::now());
        else entryClock.reset();
        if (result.timing == drawn_pattern::FrameTiming::Running) rawClockStarted = true;
        work.frames = acceptedFrames;
        committedTargetValid = true;
        lastProgressAt = lastFrameAt = now;
        if (batch.revoked) return stopLocked("guard", pattern_motor::now());
        if (last.state == pattern_motor::Frame::Rejected)
            return stopLocked(last.reason, pattern_motor::now(), last.driverResult);
        if (last.state == pattern_motor::Frame::Retry) {
            ++work.retries;
            lowAfterBatch = false;
            return;
        }
        if (batch.controlsChanged) {
            const auto at = pattern_motor::now();
            if (!local && at - renewedAt >= drawn_pattern::leaseMs) return stopLocked("lease_expired", at);
            lowAfterBatch = rawClockStarted && !pendingControl.load() &&
                            work.lastQueuedUs < drawn_pattern::refillBelowUs;
            return;
        }
        if (batch.count > 1) return; // release the mailbox after each bounded burst
        if (stationaryFrames >= 2) {
            // One frame is held in software lookahead. Two accepted identical
            // terminal targets put a stationary tail after the final movement.
            const auto parked = pattern_motor::park(owner, committedTarget, false);
            if (parked == pattern_motor::Park::Failed) return stopLocked("park_position", now);
            if (parked != pattern_motor::Park::Fill) {
                stage = parked == pattern_motor::Park::Parked ? Stage::Paused : Stage::Draining;
                speedStatus.state = stage == Stage::Paused ? "paused" : "pausing";
                invalidateFrames(true);
                rawClockStarted = fillingRaw = false;
                drainAt = now;
                return;
            }
        }
        if (rawPlanner && rawClockStarted && i + 1 == pulseFramesPerTurn &&
            result.timing == drawn_pattern::FrameTiming::Running &&
            result.queuedUs != UINT16_MAX && result.queuedUs >= drawn_pattern::refillBelowUs &&
            rawPlanner->requestAssist(RawFramePlanner::reserveForAssist)) {
            // Help while the peripheral has at least its refill watermark.
            // A low hardware reserve always gives the motor feeder priority.
            assist = rawPlanner;
        }
        // Recompute the base budget before accepting the next frame, even
        // when this call could otherwise fill several more queue entries.
        if (!retainedGoal && !zero && reserveVibration && vibration.gain.atZero() && reconfigureAfterFade)
            return;
    }
}
} // namespace

DrawnWindowEdit::DrawnWindowEdit() : guard_(lock, std::defer_lock) {
    ++pendingControl;
    guard_.lock();
}

DrawnWindowEdit::~DrawnWindowEdit() { --pendingControl; }

void DrawnWindowEdit::set(float stroke, float depth) {
    if (!std::isfinite(stroke) || !std::isfinite(depth) ||
        stroke < 0 || stroke > 100 || depth < 0 || depth > 100) return;
    if (settings.stroke == stroke && settings.depth == depth) return;
    if (stage != Stage::Idle) {
        const int32_t span = streaming_logic::calculateMaxStroke(stroke, depth, windowRail);
        const int32_t offset = streaming_logic::calculateDepthOffset(windowRail, span, depth);
        confirmWindow(-offset - span, -offset, settings.speed == 0, pattern_motor::now());
    }
    settings.stroke = stroke;
    settings.depth = depth;
}

bool patternOwnerCurrent(const PatternReferenceLost &event) {
    return pattern_motor::current(event.revision);
}

namespace {
PatternStart startStatusLocked(uint64_t revision, pattern_motor::State &caps) {
    if (!ready || owner != revision) return PatternStart::NotReady;
    caps = pattern_motor::read(revision);
    if (!caps.ready) return PatternStart::NotReady;
    if (stage != Stage::Idle) return PatternStart::Busy;
    pattern_motor::readFiniteState(caps, true);
    if (caps.moving || caps.queued) return PatternStart::Busy;
    if (caps.to <= caps.from) return PatternStart::NoTravel;
    return PatternStart::Accepted;
}
}  // namespace

PatternStart drawnPatternStartStatus(uint64_t revision) {
    std::lock_guard<std::mutex> guard(lock);
    pattern_motor::State caps;
    return startStatusLocked(revision, caps);
}

static PatternStart startPattern(const char *hex, size_t length, uint32_t periodMs, uint64_t revision,
                                 bool localControl, drawn_pattern::SampleFormat format,
                                 const drawn_pattern::Modifiers &modifiers, drawn_pattern::Modifiers *owned) {
    drawn_pattern::Waveform incoming;
    if (!modifiers.valid() || !incoming.decode(hex, length, periodMs, format)) return PatternStart::Invalid;
    std::lock_guard<std::mutex> guard(lock);
    pattern_motor::State caps;
    const auto status = startStatusLocked(revision, caps);
    if (status != PatternStart::Accepted) return status;
    std::unique_ptr<Marking> marks;
    if (modifiers.count) {
        marks.reset(new (std::nothrow) Marking);
        if (!marks) return PatternStart::MarkerMemory;
        if (marks->index.tryCompactFullPauseGrid(modifiers, incoming.authoredStart())) {
            if (owned) {
                // Release the decoded slot's pages before allocating the planner.
                drawn_pattern::Modifiers released;
                released.takeFrom(*owned);
            }
        } else {
            // A serial backend can still play the already-owned pages even
            // if a contiguous compact duration table did not fit.
            if (owned) marks->authored.takeFrom(*owned);
            else if (!marks->authored.assign(modifiers)) return PatternStart::MarkerMemory;
            if (!marks->index.reset(marks->authored, incoming.authoredStart(), &incoming)) return PatternStart::MarkerMemory;
        }
    }
    if (caps.serial) {
        pending.reset(new (std::nothrow) frame_plan::Frame);
        if (!pending) return PatternStart::QueueMemory;
        serialMarkers = std::move(marks);
        activeMarkers = serialMarkers.get();
    } else {
        auto *planner = new (std::nothrow) RawFramePlanner(incoming, std::move(marks));
        if (!planner) return PatternStart::PlannerMemory;
        if (!planner->start()) {
            const auto result = planner->hasQueueStorage() ? PatternStart::WorkerMemory : PatternStart::QueueMemory;
            RawFramePlanner::Counters unused;
            planner->close(unused);
            return result;
        }
        rawPlanner = planner;
        activeMarkers = planner->marking.get();
    }
    rawClockStarted = fillingRaw = false;
    curve = incoming;
    speedStatus = {};
    speedStatus.state = "waiting";
    speedStatus.speedPct = caps.fraction * 100.;
    speedStatus.nominalMmS = caps.stepsPerMm > 0 ? caps.nominalSpeed * caps.fraction / caps.stepsPerMm : 0;
    goalRate = 0;
    stampedGoal = -1;
    stationaryFrames = 0;
    softwareEntry = false;
    entryClock.reset();
    committedTargetValid = false;
    vibration = {};
    vibrationBudget = {};
    reserveVibration = reconfigureAfterFade = false;
    lastStrength = -1;
    local = localControl;
    serial = caps.serial;
    lower = caps.from;
    upper = caps.to;
    windowRail = caps.rail;
    lastStop = {};
    work = {};
    feedGap = {};
    refillSchedule = {};
    haveServiceEnd = false;
    startedAt = renewedAt = pattern_motor::now();
    lastAppliedPeriod = acceptedFrames = 0;
    stage = Stage::Waiting;
    return PatternStart::Accepted;
}

PatternStart startDrawnPattern(const char *hex, size_t length, uint32_t periodMs, uint64_t revision,
                               bool localControl, drawn_pattern::SampleFormat format,
                               const drawn_pattern::Modifiers &modifiers) {
    return startPattern(hex, length, periodMs, revision, localControl, format, modifiers, nullptr);
}

PatternStart startOwnedDrawnPattern(const char *hex, size_t length, uint32_t periodMs, uint64_t revision,
                                    bool localControl, drawn_pattern::SampleFormat format,
                                    std::unique_ptr<drawn_pattern::Modifiers> modifiers) {
    const auto &items = modifiers ? *modifiers : drawn_pattern::noModifiers();
    return startPattern(hex, length, periodMs, revision, localControl, format, items, modifiers.get());
}

bool stoppedPatternWork(PatternWork &out) {
    std::lock_guard<std::mutex> guard(lock);
    if (stage != Stage::Idle || !lastStop.reason) return false;
    out = work;
    return true;
}

bool stoppedPatternFeedGap(PatternFeedGap &out) {
    std::lock_guard<std::mutex> guard(lock);
    if (stage != Stage::Idle || !lastStop.reason || !feedGap.frame) return false;
    out = feedGap;
    return true;
}

bool renewDrawnPattern(uint64_t revision) {
    const uint32_t waiting = patternServiceMicros();
    std::lock_guard<std::mutex> guard(lock);
    if (stage != Stage::Idle)
        work.maxRenewLockUs = std::max(work.maxRenewLockUs, patternServiceMicros() - waiting);
    const auto now = pattern_motor::now();
    if (local || stage == Stage::Idle || owner != revision || !pattern_motor::current(revision) ||
        now - renewedAt >= drawn_pattern::leaseMs) return false;
    renewedAt = now;
    return true;
}

// Called only after RAD BLE validates the connection-bound control lease.
// Keep the speed write and decision about the current loop together so a wired
// start or disconnect cannot take ownership between those operations.
bool applyLeasedBleSpeed(motion_link::Owner &link, uint16_t connection,
                         float value, bool selectBleSource) {
    if (connection == motion_link::kNoConnection || !std::isfinite(value) ||
        value < 0 || value > 100) return false;
    ControlRequest request;
    std::lock_guard<std::mutex> guard(lock);
    if (local || link.owner() != connection)
        stopLocked("stop", pattern_motor::now());
    writeBleSpeed(value, selectBleSource);
    link.onMotionCommand(connection);
    return true;
}

bool applyLeasedBleSensation(motion_link::Owner &link, uint16_t connection, uint8_t value) {
    if (connection == motion_link::kNoConnection || value > 100) return false;
    ControlRequest request;
    std::lock_guard<std::mutex> guard(lock);
    if (local || link.owner() != connection) stopLocked("stop", pattern_motor::now());
    pattern_motor::setSensation(value);
    link.onMotionCommand(connection);
    return true;
}

void cancelDrawnPattern() {
    ControlRequest request;
    std::lock_guard<std::mutex> guard(lock);
    stopLocked("stop", pattern_motor::now());
}
void cancelRemoteDrawnPattern() {
    ControlRequest request;
    std::lock_guard<std::mutex> guard(lock);
    if (!local) stopLocked("remote_release", pattern_motor::now());
}
bool drawnPatternActive() {
    std::lock_guard<std::mutex> guard(lock);
    return stage != Stage::Idle;
}
bool drawnPatternRateStamp(uint32_t sequence, PatternRateStamp &out) {
    std::lock_guard<std::mutex> guard(lock);
    if (sequence==0) sequence=rateSequence;
    if (sequence==0) return false;
    for (const auto &stamp : rateHistory) {
        if (stamp.sequence==sequence) { out=stamp; return true; }
    }
    return false;
}
PatternStatus drawnPatternStatus() {
    std::lock_guard<std::mutex> guard(lock);
    const auto &applied = rawPlanner && rawPlanner->positivePending() ? rawPlanner->previousSpeed : speedStatus;
    return {stage != Stage::Idle, applied.periodMs, lastStop, applied};
}
void setDrawnPatternReady(uint64_t revision, bool enabled, const char *exitReason) {
    ControlRequest request;
    std::lock_guard<std::mutex> guard(lock);
    if (enabled && pattern_motor::current(revision)) {
        stopLocked(exitReason, pattern_motor::now());
        ready = true;
        owner = revision;
    } else if (owner == revision) {
        ready = false;
        stopLocked(exitReason, pattern_motor::now());
    }
}
namespace {
bool serviceTurn(uint64_t revision, uint32_t wakeUs, bool *refillNow, unsigned frameLimit,
                 bool continuing, unsigned &allowance, bool &recharge, bool &mayResumePause) {
    recharge = false;
    if (refillNow) *refillNow = false;
    const uint32_t callUs = patternServiceMicros();
    std::unique_lock<std::mutex> guard(lock);
    if (stage == Stage::Idle || owner != revision) return false;
    if (continuing && (stage != Stage::Playing || !rawClockStarted || !rawPlanner)) return true;
    const uint32_t acquiredUs = patternServiceMicros();
    if (!wakeUs) wakeUs = callUs;
    auto &turn = work.lastService;
    turn = {};
    turn.atMs = pattern_motor::now();
    turn.frame = acceptedFrames;
    if (feedGap.frame) ++feedGap.serviceTurns;
    turn.offUs = haveServiceEnd ? wakeUs - lastServiceEnd : 0;
    turn.outerUs = callUs - wakeUs;
    turn.mailboxUs = acquiredUs - callUs;
    bool lowAfterBatch = false;
    RawFramePlanner *assist = nullptr;
    service(turn.atMs, frameLimit, allowance, lowAfterBatch, assist);
    allowance -= std::min(allowance, acceptedFrames - turn.frame);
    const bool low = stage == Stage::Playing && lowAfterBatch;
    // Only a real append to the running peripheral permits another bounded
    // batch. A full driver or an empty planner cannot spin in this loop.
    recharge = allowance && low && rawClockStarted && acceptedFrames > turn.frame &&
               !assist && rawPlanner && rawPlanner->hasPreparedFrame();
    if (!recharge) {
        const bool immediate = refillSchedule.immediate(low);
        if (immediate) ++work.immediateTurns;
        else if (low) ++work.lowWaterYields;
        if (refillNow) *refillNow = immediate;
    }
    lastServiceEnd = patternServiceMicros();
    haveServiceEnd = true;
    turn.serviceUs = lastServiceEnd - acquiredUs;
    if (turn.intervalUs() >= work.longestService.intervalUs()) work.longestService = turn;
    // When a zero edit found no prepared braking row, the worker or the
    // assisting feeder can publish it while this mailbox is unlocked.
    // Return directly to a fresh guarded admission in the same task wake.
    // At most one empty attempt per visit gets this extra opportunity.
    const bool retryPause = mayResumePause && low && speedStatus.speedPct == 0 && rawPlanner &&
        (assist || (acceptedFrames == turn.frame && rawPlanner->hasPreparedFrame()));
    guard.unlock();
    if (assist) assist->assist([] {});
    if (retryPause) {
        mayResumePause = false;
        recharge = true;
    }
    return true;
}
} // namespace

bool serviceDrawnPattern(uint64_t revision, uint32_t wakeUs, bool *refillNow) {
    constexpr unsigned ordinaryBatch = 3;
    // The starting work allowance excludes timed intervals spent inside an
    // admitted batch. Hardware admission always uses the live FAS ticks.
    constexpr unsigned maximumBatch = drawn_pattern::queuedDurationMs / drawn_pattern::pulseFrameMs;
    bool recharge = false;
    unsigned allowance = maximumBatch;
    bool mayResumePause = true;
    const bool active = serviceTurn(revision, wakeUs, refillNow, ordinaryBatch, false, allowance, recharge, mayResumePause);
    // Refill below the watermark during steady motion as well as at a live
    // retarget. Yield after one bounded visit, at the 14 ms refill
    // watermark, at the first declined write or on new control intent.
    while (recharge)
        serviceTurn(revision, 0, refillNow, 1, true, allowance, recharge, mayResumePause);
    return active;
}
} // namespace streaming
