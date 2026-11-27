#include "drawn_waveform.h"
#include "drawn_modifier_motion.h"
#include "drawn_phase_rate.h"
#include "drawn_pattern.h"
#include "drawn_frame_planner.h"

#include <chrono>
#include <cstring>
#include <mutex>
#include <memory>
#include <new>

#include "drawn_pattern_motor.h"
#include "../state/ble.h"
#include "../../services/communication/motion_link.h"

namespace streaming {
uint32_t patternServiceMicros() {
    return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

namespace {
enum class Stage { Idle, Waiting, Positioning, EntryBraking, Playing, Draining, Paused };
std::mutex lock;
drawn_pattern::Waveform curve;
Stage stage = Stage::Idle;
bool ready = false, local = false, serial = false;
uint64_t owner = 0;
PatternStop lastStop;
PatternWork work;
drawn_pattern::PulseRefillSchedule refillSchedule;
PatternSpeedStatus speedStatus;
double goalRate = 0, stampedGoal = -1;
uint32_t drainAt = 0, entryStopBudget = 0;
unsigned stationaryFrames = 0;

uint32_t lastServiceEnd = 0;
bool haveServiceEnd = false;
uint32_t startedAt = 0, renewedAt = 0, lastFrameAt = 0, lastProgressAt = 0, effectivePeriod = 0;
int32_t lower = 0, upper = 0, entry = 0;
using frame_plan::Marking;
using frame_plan::RawFramePlanner;
std::unique_ptr<Marking> serialMarkers;
Marking *activeMarkers = nullptr;
RawFramePlanner *rawPlanner = nullptr;
bool rawClockStarted = false;
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
void invalidateFrames(bool warmup = false) {
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

void recordRateGoal(uint32_t now, drawn_pattern::FrameTiming timing, double strength, double nextGain, bool handoff) {
    using drawn_pattern::FrameTiming;
    newestRateStamp = (newestRateStamp+1)%rateHistorySize;
    if (++rateSequence==0) ++rateSequence;
    const bool running = timing==FrameTiming::Started || timing==FrameTiming::Running;
    auto &stamp = rateHistory[newestRateStamp];
    stamp = {rateSequence,
             now,
             effectivePeriod,
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
             acceptedFrames, goalRate, handoff};
    stampedGoal = goalRate;
    lastStampedStrength = strength;
    awaitingStart[newestRateStamp] = timing==FrameTiming::Prefill;
    lastAppliedPeriod = effectivePeriod;
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
        rawPlanner = nullptr;
    }
    pending.reset();
    for (auto &waiting : awaitingStart) waiting = false;
    effectivePeriod = 0;
    speedStatus = {};
}

void service(uint32_t now, bool &lowAfterBatch) {
    if (!local && now - renewedAt >= drawn_pattern::leaseMs) return stopLocked("lease_expired", now);
    const uint32_t readStart = patternServiceMicros();
    auto caps = pattern_motor::read(owner, activeMarkers != nullptr);
    if (caps.ready && (stage == Stage::Positioning || stage == Stage::EntryBraking ||
                       (stage == Stage::Waiting && caps.fraction > 0))) {
        pattern_motor::readFiniteState(caps, stage != Stage::Positioning);
    }
    work.lastService.readUs = patternServiceMicros() - readStart;
    const auto &markerIndex = activeMarkers ? activeMarkers->index : unmarked;
    if (!caps.ready) return stopLocked("guard", now);
    if (caps.serial != serial) return stopLocked("backend_changed", now);
    if (!(caps.fullSpeed > 0 && caps.stepsPerMm > 0 && caps.fraction >= 0 && caps.fraction <= 1 &&
          caps.acceleration >= 0 && (caps.acceleration >= 1 || caps.fraction == 0) && caps.to > caps.from)) {
        if (stage != Stage::Waiting) stopLocked("limits_zero", now);
        return;
    }
    const bool zero = caps.fraction == 0;
    speedStatus.speedPct = 100. * caps.fraction;
    speedStatus.nominalMmS = caps.nominalSpeed * caps.fraction / caps.stepsPerMm;
    if (stage != Stage::Waiting && (lower != caps.from || upper != caps.to))
        return stopLocked("window_changed", now);
    if (zero) {
        goalRate = 0;
        effectivePeriod = speedStatus.periodMs = 0;
        speedStatus.phaseRateHz = speedStatus.plannedPeakMmS = 0;
        if (stage == Stage::Waiting) { speedStatus.state = "waiting"; return; }
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
        rawClockStarted = false;
        if (!zero) {
            stage = Stage::Playing;
            softwareEntry = !serial;
            lastProgressAt = lastFrameAt = now;
        }
    }
    if (stage == Stage::Paused) return;
    const bool requestedVibration = !zero && markerIndex.hasVibration && caps.strength > 0;
    const bool reserve = requestedVibration || !vibration.gain.atZero();
    const bool limitsChanged = caps.speed != lastSpeed || caps.acceleration != lastAcceleration;
    if (zero && lastSpeed != 0) {
        invalidateFrames();
        lastSpeed = 0;
    }
    if (!zero && (stage == Stage::Waiting || reserve != reserveVibration ||
        (reconfigureAfterFade && vibration.gain.atZero()) || limitsChanged)) {
        invalidateFrames();
        committedTargetValid = false;
        const double span = static_cast<double>(caps.to) - caps.from;
        const double share = reserve ? drawn_pattern::VibrationBudget::baseShare : 1.;
        // A changing vibration budget fades out at its old safe amplitude.
        const bool fading = !vibration.gain.atZero() && !vibrationBudget.matches(caps.speed, caps.acceleration);
        const double v = fading ? std::min(caps.speed, vibrationBudget.speed) : caps.speed;
        const double a = fading ? std::min(caps.acceleration, vibrationBudget.acceleration) : caps.acceleration;
        const auto pace = curve.pace(span, caps.fullSpeed * share, a * share, v / caps.fullSpeed,
                                     serial ? 20 : drawn_pattern::pulseFrameMs);
        if (!pace.periodMs) return stopLocked("period_limit", now);
        effectivePeriod = speedStatus.periodMs = pace.periodMs;
        goalRate = speedStatus.phaseRateHz = pace.rate;
        speedStatus.plannedPeakMmS = pace.peak / caps.stepsPerMm;
        speedStatus.limit = pace.limit;
        if (std::strcmp(pace.limit, "speed") == 0) {
            if (reserve) speedStatus.limit = "vibration";
            else if (caps.fullSpeed < caps.nominalSpeed) speedStatus.limit = "backend";
        }
        clockAcceleration = pace.phaseAcceleration;
        clockJerk = pace.phaseJerk;
        if (reserve) {
            clockAcceleration = std::min(clockAcceleration, 2 * goalRate);
            clockJerk = std::min(clockJerk, 8 * goalRate);
            if (!fading) vibrationBudget = {span, v, a, caps.stepsPerMm, {goalRate, clockAcceleration, clockJerk}};
        }
        if (markerIndex.holds)
            activeMarkers->brakeBounds.reset({goalRate, clockAcceleration, clockJerk});
        reserveVibration = reserve;
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
        lower = caps.from;
        upper = caps.to;
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
    if (rawPlanner && !rawPlanner->configured()) rawPlanner->configure(inputs(), cursor());

    constexpr unsigned pulseFramesPerTurn = 3;
    for (unsigned i = 0; i < (serial ? 1u : pulseFramesPerTurn); ++i) {
        auto *next = rawPlanner ? rawPlanner->front() : pending.get();
        if (!next) {
            // A new goal can temporarily empty the software ring. Never
            // restart a peripheral which actually ran out of timed pulses.
            if (rawClockStarted && !pattern_motor::pulseClockRunning())
                stopLocked("pulse_underrun", pattern_motor::now());
            return;
        }
        auto &frame = *next;
        if (!frame.valid) {
            const auto in = inputs();
            frame_plan::calculate(in, cursor(), frame, curve, activeMarkers ? &activeMarkers->index : nullptr);
        }
        if (!frame.offered) {
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
        const uint32_t submitStart = patternServiceMicros();
        const auto result = pattern_motor::submit(frame.target, owner, serial, frame.crossing);
        const uint32_t submitUs = patternServiceMicros() - submitStart;
        work.lastService.commandGateUs = std::max(work.lastService.commandGateUs, result.commandGateUs);
        ++work.submissions;
        work.submitTotalUs += submitUs;
        if (submitUs > 2000) ++work.submitOver2ms;
        work.maxSubmitUs = std::max(work.maxSubmitUs, submitUs);
        if (result.timing == drawn_pattern::FrameTiming::Started)
            observeQueueStart(pattern_motor::now());
        if (result.state == pattern_motor::Frame::Rejected)
            return stopLocked(result.reason, pattern_motor::now(), result.driverResult);
        if (result.state == pattern_motor::Frame::Retry) {
            ++work.retries;
            if (result.timing == drawn_pattern::FrameTiming::PrimeTail) lastProgressAt = now;
            if (now - lastProgressAt >= drawn_pattern::frameStarvationMs)
                stopLocked(result.reason, now, result.driverResult);
            return;
        }
        if (result.queuedUs != UINT16_MAX) {
            work.lastQueuedUs = result.queuedUs;
            work.maxQueuedUs = std::max(work.maxQueuedUs, result.queuedUs);
            if (result.timing == drawn_pattern::FrameTiming::Running)
                work.minRunningUs = std::min(work.minRunningUs, result.queuedUs);
            lowAfterBatch = i + 1 == pulseFramesPerTurn && result.queuedUs < drawn_pattern::refillBelowUs;
        }
        const bool entryToRaw = softwareEntry &&
            (result.timing == drawn_pattern::FrameTiming::Prefill ||
             result.timing == drawn_pattern::FrameTiming::Started ||
             result.timing == drawn_pattern::FrameTiming::Running);
        if (entryToRaw || stampedGoal != goalRate || lastAppliedPeriod != effectivePeriod ||
            (activeMarkers &&
             (lastStampedStrength != caps.strength || vibration.gain.goal() != frame.vibration.gain.goal())))
            recordRateGoal(pattern_motor::now(), result.timing, caps.strength, frame.vibration.gain.goal(), entryToRaw);
        if (entryToRaw) work.rawStartSequence = rateSequence;
        softwareEntry = result.timing == drawn_pattern::FrameTiming::EntryHold;
        if (result.timing == drawn_pattern::FrameTiming::Started ||
            result.timing == drawn_pattern::FrameTiming::Running) rawClockStarted = true;
        const bool settled = zero && frame.phase.clock.secondsToGoal() == 0 &&
            frame.phase.clock.rate() == 0 && frame.vibration.gain.atZero() && frame.from == frame.target;
        stationaryFrames = settled ? stationaryFrames + 1 : 0;
        committedTarget = frame.target;
        work.frames = ++acceptedFrames;
        motionPhase = frame.phase;
        vibration = frame.vibration;
        committedTargetValid = true;
        if (rawPlanner) rawPlanner->pop();
        else frame.valid = false;
        lastProgressAt = lastFrameAt = now;
        if (stationaryFrames >= 2) {
            // One frame is held in software lookahead. Two accepted identical
            // terminal targets put a stationary tail after the final movement.
            const auto parked = pattern_motor::park(owner, committedTarget, false);
            if (parked == pattern_motor::Park::Failed) return stopLocked("park_position", now);
            if (parked != pattern_motor::Park::Fill) {
                stage = parked == pattern_motor::Park::Parked ? Stage::Paused : Stage::Draining;
                speedStatus.state = stage == Stage::Paused ? "paused" : "pausing";
                invalidateFrames(true);
                rawClockStarted = false;
                drainAt = now;
                return;
            }
        }
        // Recompute the base budget before accepting the next frame, even
        // when this call could otherwise fill several more queue entries.
        if (!zero && reserveVibration && vibration.gain.atZero() && (!requestedVibration || reconfigureAfterFade))
            return;
    }
}
} // namespace

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
    return PatternStart::Accepted;
}
}  // namespace

PatternStart drawnPatternStartStatus(uint64_t revision) {
    std::lock_guard<std::mutex> guard(lock);
    pattern_motor::State caps;
    return startStatusLocked(revision, caps);
}

PatternStart startDrawnPattern(const char *hex, size_t length, uint32_t periodMs, uint64_t revision,
                               bool localControl, drawn_pattern::SampleFormat format,
                               const drawn_pattern::Modifiers &modifiers) {
    drawn_pattern::Waveform incoming;
    if (!modifiers.valid() || !incoming.decode(hex, length, periodMs, format)) return PatternStart::Invalid;
    std::lock_guard<std::mutex> guard(lock);
    pattern_motor::State caps;
    const auto status = startStatusLocked(revision, caps);
    if (status != PatternStart::Accepted) return status;
    std::unique_ptr<Marking> marks;
    if (modifiers.count) {
        marks.reset(new (std::nothrow) Marking);
        if (!marks || !marks->authored.assign(modifiers) ||
            !marks->index.reset(marks->authored, incoming.authoredStart())) return PatternStart::Busy;
    }
    if (caps.serial) {
        pending.reset(new (std::nothrow) frame_plan::Frame);
        if (!pending) return PatternStart::Busy;
        serialMarkers = std::move(marks);
        activeMarkers = serialMarkers.get();
    } else {
        auto *planner = new (std::nothrow) RawFramePlanner(incoming, std::move(marks));
        if (!planner) return PatternStart::Busy;
        if (!planner->start()) {
            RawFramePlanner::Counters unused;
            planner->close(unused);
            return PatternStart::Busy;
        }
        rawPlanner = planner;
        activeMarkers = planner->marking.get();
    }
    rawClockStarted = false;
    curve = incoming;
    speedStatus = {};
    speedStatus.state = "waiting";
    speedStatus.speedPct = caps.fraction * 100.;
    speedStatus.nominalMmS = caps.stepsPerMm > 0 ? caps.nominalSpeed * caps.fraction / caps.stepsPerMm : 0;
    goalRate = 0;
    stampedGoal = -1;
    stationaryFrames = 0;
    softwareEntry = false;
    committedTargetValid = false;
    vibration = {};
    vibrationBudget = {};
    reserveVibration = reconfigureAfterFade = false;
    lastStrength = -1;
    local = localControl;
    serial = caps.serial;
    lastStop = {};
    work = {};
    refillSchedule = {};
    haveServiceEnd = false;
    startedAt = renewedAt = pattern_motor::now();
    lastAppliedPeriod = acceptedFrames = 0;
    stage = Stage::Waiting;
    return PatternStart::Accepted;
}

bool stoppedPatternWork(PatternWork &out) {
    std::lock_guard<std::mutex> guard(lock);
    if (stage != Stage::Idle || !lastStop.reason) return false;
    out = work;
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
    std::lock_guard<std::mutex> guard(lock);
    if (local || link.owner() != connection)
        stopLocked("stop", pattern_motor::now());
    writeBleSpeed(value, selectBleSource);
    link.onMotionCommand(connection);
    return true;
}

bool applyLeasedBleSensation(motion_link::Owner &link, uint16_t connection, uint8_t value) {
    if (connection == motion_link::kNoConnection || value > 100) return false;
    std::lock_guard<std::mutex> guard(lock);
    if (local || link.owner() != connection || !activeMarkers) stopLocked("stop", pattern_motor::now());
    pattern_motor::setSensation(value);
    link.onMotionCommand(connection);
    return true;
}

void cancelDrawnPattern() {
    std::lock_guard<std::mutex> guard(lock);
    stopLocked("stop", pattern_motor::now());
}
void cancelRemoteDrawnPattern() {
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
    return {stage != Stage::Idle, effectivePeriod, lastStop, speedStatus};
}
void setDrawnPatternReady(uint64_t revision, bool enabled, const char *exitReason) {
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
bool serviceDrawnPattern(uint64_t revision, uint32_t wakeUs, bool *refillNow) {
    if (refillNow) *refillNow = false;
    const uint32_t callUs = patternServiceMicros();
    std::lock_guard<std::mutex> guard(lock);
    if (stage == Stage::Idle || owner != revision) return false;
    const uint32_t acquiredUs = patternServiceMicros();
    if (!wakeUs) wakeUs = callUs;
    auto &turn = work.lastService;
    turn = {};
    turn.atMs = pattern_motor::now();
    turn.frame = acceptedFrames;
    turn.offUs = haveServiceEnd ? wakeUs - lastServiceEnd : 0;
    turn.outerUs = callUs - wakeUs;
    turn.mailboxUs = acquiredUs - callUs;
    bool lowAfterBatch = false;
    service(turn.atMs, lowAfterBatch);
    const bool low = stage == Stage::Playing && lowAfterBatch;
    const bool immediate = refillSchedule.immediate(low);
    if (immediate) ++work.immediateTurns;
    else if (low) ++work.lowWaterYields;
    if (refillNow) *refillNow = immediate;
    lastServiceEnd = patternServiceMicros();
    haveServiceEnd = true;
    turn.serviceUs = lastServiceEnd - acquiredUs;
    if (turn.intervalUs() >= work.longestService.intervalUs()) work.longestService = turn;
    return true;
}
} // namespace streaming
