#include "drawn_waveform.h"
#include "drawn_modifier_motion.h"
#include "drawn_phase_rate.h"
#include "drawn_pattern.h"

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
PatternSpeedStatus speedStatus;
double goalRate = 0, stampedGoal = -1;
uint32_t drainAt = 0, entryStopBudget = 0;
unsigned stationaryFrames = 0;

uint32_t lastServiceEnd = 0;
bool haveServiceEnd = false;
uint32_t startedAt = 0, renewedAt = 0, lastFrameAt = 0, lastProgressAt = 0, effectivePeriod = 0;
int32_t lower = 0, upper = 0, entry = 0;
struct Marking {
    drawn_pattern::Modifiers authored;
    drawn_pattern::MarkerIndex index;
    drawn_pattern::PauseBrakeBounds brakeBounds;
};
std::unique_ptr<Marking> activeMarkers;
const drawn_pattern::MarkerIndex unmarked{};

drawn_pattern::MarkedPhase motionPhase;
drawn_pattern::VibrationClock vibration;
drawn_pattern::VibrationBudget vibrationBudget;
bool reserveVibration = false, reconfigureAfterFade = false;
double lastStrength = -1;
double lastSpeed = 0, lastAcceleration = 0;
double clockAcceleration = 0, clockJerk = 0;
struct PlannedFrame {
    drawn_pattern::MarkedPhase phase;
    drawn_pattern::VibrationClock vibration;
    int32_t target = 0;
    uint16_t crossing = 32768;
    bool valid = false;
};
// The submitted state advances only after the backend accepts the frame.
// Keep one prospective frame during playback so queue pressure does not
// repeat the phase clock, polynomial and crossing work for the same input.
std::unique_ptr<PlannedFrame> pending;
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
    activeMarkers.reset();
    pending.reset();
    for (auto &waiting : awaitingStart) waiting = false;
    effectivePeriod = 0;
    speedStatus = {};
}

void service(uint32_t now) {
    if (!local && now - renewedAt >= drawn_pattern::leaseMs) return stopLocked("lease_expired", now);
    const uint32_t readStart = patternServiceMicros();
    const auto caps = pattern_motor::read(owner, activeMarkers != nullptr);
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
        pending->valid = false;
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
    if (zero && limitsChanged) {
        pending->valid = false;
        lastSpeed = 0;
    }
    if (!zero && (stage == Stage::Waiting || reserve != reserveVibration ||
        (reconfigureAfterFade && vibration.gain.atZero()) || limitsChanged)) {
        pending->valid = committedTargetValid = false;
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
        pending->valid = committedTargetValid = false;
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
    const double dt = frameMs / 1000.0;
    const drawn_pattern::PhaseLimits phaseLimits{goalRate, clockAcceleration, clockJerk};
    // Three 2 ms frames per turn rebuild the existing bounded hardware queue
    // faster than it drains. Release the lock and CPU between refill batches.
    constexpr unsigned pulseFramesPerTurn = 3;
    for (unsigned i = 0; i < (serial ? 1u : pulseFramesPerTurn); ++i) {
        auto &frame = *pending;
        if (!frame.valid) {
            const uint32_t planStart = patternServiceMicros();
            const bool withinNewEnvelope =
                motionPhase.clock.secondsToGoal() == 0 && motionPhase.clock.rate() <= phaseLimits.rate + 1e-12;
            const double amplitude = requestedVibration && vibrationBudget.matches(caps.speed, caps.acceleration) &&
                                             (withinNewEnvelope || !vibration.gain.atZero())
                                         ? caps.strength
                                         : 0;
            drawn_pattern::FrameBrake frameBrake;
            if (markerIndex.holds) frameBrake.bounds = &activeMarkers->brakeBounds;
            const auto advance = [&](auto &p, auto &v, double seconds) {
                if (zero) p.brakeForUser(seconds, phaseLimits);
                else p.advance(seconds, phaseLimits, markerIndex, caps.strength, &frameBrake);
                if (markerIndex.hasVibration) v.advance(seconds, amplitude);
            };
            const auto target = [&](const auto &p, const auto &v) {
                if (v.gain.atZero()) return curve.nativeTarget(p.phase, lower, upper);
                const double y = curve.position(p.phase);
                return upper - static_cast<int32_t>(std::lround(
                                   (upper - lower) * y + v.offset(y, p.phase, markerIndex, vibrationBudget)));
            };
            frame.phase = motionPhase;
            frame.vibration = vibration;
            advance(frame.phase, frame.vibration, dt);
            frame.target = target(frame.phase, frame.vibration);
            if (!committedTargetValid) {
                committedTarget = target(motionPhase, vibration);
                committedTargetValid = true;
            }
            frame.crossing = 32768;
            if (frame.target != committedTarget) {
                ++work.movingPlans;
                if (!serial && !activeMarkers) {
                    auto clock = motionPhase.clock;
                    clock.setGoal(phaseLimits.rate, phaseLimits.acceleration, phaseLimits.jerk);
                    frame.crossing = drawn_pattern::firstStepCrossing(committedTarget, frame.target, [&](double fraction) {
                        return curve.nativeTarget(motionPhase.phase + clock.distanceAt(dt * fraction), lower, upper);
                    });
                } else if (!serial) {
                    frame.crossing = drawn_pattern::firstStepCrossing(committedTarget, frame.target, [&](double fraction) {
                        auto p = motionPhase;
                        auto v = vibration;
                        advance(p, v, dt * fraction);
                        return target(p, v);
                    });
                }
            }
            frame.valid = true;
            const uint32_t planUs = patternServiceMicros() - planStart;
            ++work.plans;
            work.planTotalUs += planUs;
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
        frame.valid = false;
        const bool settled = zero && frame.phase.clock.secondsToGoal() == 0 &&
            frame.phase.clock.rate() == 0 && frame.vibration.gain.atZero() && committedTarget == frame.target;
        stationaryFrames = settled ? stationaryFrames + 1 : 0;
        committedTarget = frame.target;
        work.frames = ++acceptedFrames;
        motionPhase = frame.phase;
        vibration = frame.vibration;
        lastProgressAt = lastFrameAt = now;
        if (stationaryFrames >= 2) {
            // One frame is held in software lookahead. Two accepted identical
            // terminal targets put a stationary tail after the final movement.
            const auto parked = pattern_motor::park(owner, committedTarget, false);
            if (parked == pattern_motor::Park::Failed) return stopLocked("park_position", now);
            if (parked != pattern_motor::Park::Fill) {
                stage = parked == pattern_motor::Park::Parked ? Stage::Paused : Stage::Draining;
                speedStatus.state = stage == Stage::Paused ? "paused" : "pausing";
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
    if (stage != Stage::Idle || caps.moving || caps.queued) return PatternStart::Busy;
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
    std::unique_ptr<PlannedFrame> firstFrame(new (std::nothrow) PlannedFrame);
    if (!firstFrame) return PatternStart::Busy;
    curve = incoming;
    if (modifiers.count) {
        // The idle BLE/TLS paths retain no full playback-marker table.
        activeMarkers.reset(new (std::nothrow) Marking);
        if (!activeMarkers) return PatternStart::Busy;
        if (!activeMarkers->authored.assign(modifiers) ||
            !activeMarkers->index.reset(activeMarkers->authored, curve.authoredStart())) {
            activeMarkers.reset();
            return PatternStart::Busy;
        }
    }
    pending = std::move(firstFrame);
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
bool serviceDrawnPattern(uint64_t revision, uint32_t wakeUs) {
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
    service(turn.atMs);
    lastServiceEnd = patternServiceMicros();
    haveServiceEnd = true;
    turn.serviceUs = lastServiceEnd - acquiredUs;
    if (turn.intervalUs() >= work.longestService.intervalUs()) work.longestService = turn;
    return true;
}
} // namespace streaming
