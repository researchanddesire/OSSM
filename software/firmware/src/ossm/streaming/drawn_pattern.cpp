#include "drawn_waveform.h"
#include "drawn_modifier_motion.h"
#include "drawn_phase_rate.h"
#include "drawn_pattern.h"

#include <mutex>
#include <memory>
#include <new>

#include "drawn_pattern_motor.h"
#include "../state/ble.h"
#include "../../services/communication/motion_link.h"

namespace streaming {
namespace {
enum class Stage { Idle, Waiting, Positioning, Playing };
std::mutex lock;
drawn_pattern::Waveform curve;
Stage stage = Stage::Idle;
bool ready = false, local = false, serial = false;
uint64_t owner = 0;
PatternStop lastStop;
uint32_t startedAt = 0, renewedAt = 0, lastFrameAt = 0, lastProgressAt = 0, effectivePeriod = 0;
int32_t lower = 0, upper = 0, entry = 0;
struct Marking {
    drawn_pattern::Modifiers authored;
    drawn_pattern::MarkerIndex index;
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
uint16_t firstCrossing = 32768;
bool crossingCached = false;

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

void recordRateGoal(uint32_t now, drawn_pattern::FrameTiming timing, double strength, double nextGain) {
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
             lastAppliedPeriod == 0,
             serial,
             running || timing == FrameTiming::EntryHold,
             static_cast<float>(strength),
             static_cast<float>(vibration.gain.value()),
             static_cast<float>(nextGain),
             static_cast<float>(vibration.carrier),
             acceptedFrames};
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
        case Stage::Playing: return "playing";
        default: return "idle";
    }
}
void stopLocked(const char *reason, uint32_t now, int8_t driverResult = 0) {
    if (stage == Stage::Idle) return; // preserve the first stop until the next accepted Play
    lastStop = {reason, stageName(), now, now - startedAt, driverResult,
                stage == Stage::Playing ? now - lastFrameAt : 0};
    if (stage == Stage::Playing || stage == Stage::Positioning) pattern_motor::stop(owner);
    stage = Stage::Idle;
    activeMarkers.reset();
    for (auto &pending : awaitingStart) pending = false;
    effectivePeriod = 0;
}

void service(uint32_t now) {
    if (!local && now - renewedAt >= drawn_pattern::leaseMs) return stopLocked("lease_expired", now);
    const auto caps = pattern_motor::read(owner, activeMarkers != nullptr);
    const auto &markerIndex = activeMarkers ? activeMarkers->index : unmarked;
    if (!caps.ready) return stopLocked("guard", now);
    if (caps.serial != serial) return stopLocked("backend_changed", now);
    if (caps.speed < 1 || caps.acceleration < 1 || caps.to <= caps.from) {
        if (stage != Stage::Waiting) stopLocked("limits_zero", now);
        return;
    }
    if (stage != Stage::Waiting && (lower != caps.from || upper != caps.to))
        return stopLocked("window_changed", now);
    const bool requestedVibration = markerIndex.hasVibration && caps.strength > 0;
    const bool reserve = requestedVibration || !vibration.gain.atZero();
    if (stage == Stage::Waiting || reserve != reserveVibration ||
        (reconfigureAfterFade && vibration.gain.atZero()) || caps.speed != lastSpeed ||
        caps.acceleration != lastAcceleration) {
        crossingCached = false;
        const double span = static_cast<double>(caps.to) - caps.from;
        const double share = reserve ? drawn_pattern::VibrationBudget::baseShare : 1.;
        // Fade the old carrier before raising its base-phase ceiling. A lower
        // speed starts braking immediately, as with an ordinary live edit.
        const bool fading = !vibration.gain.atZero() && !vibrationBudget.matches(caps.speed, caps.acceleration);
        const double v = fading ? std::min(caps.speed, vibrationBudget.speed) : caps.speed;
        const double a = fading ? std::min(caps.acceleration, vibrationBudget.acceleration) : caps.acceleration;
        const uint32_t period = curve.safePeriodMs(span, v * share, a * share / 2);
        if (!period) return stopLocked("period_limit", now);
        effectivePeriod = period;
        const auto limits = curve.rateLimits(span, a * share);
        clockAcceleration = limits.acceleration;
        clockJerk = limits.jerk;
        if (reserve) {
            const double r = 1000. / period;
            clockAcceleration = std::min(clockAcceleration, 2 * r);
            clockJerk = std::min(clockJerk, 8 * r);
            if (!fading) vibrationBudget = {span, v, a, caps.stepsPerMm, {r, clockAcceleration, clockJerk}};
        }
        reserveVibration = reserve;
        // Re-evaluate once a changed physical cap's old carrier reaches zero.
        reconfigureAfterFade = fading;
        lastSpeed = caps.speed;
        lastAcceleration = caps.acceleration;
    }
    if (caps.strength != lastStrength) {
        crossingCached = false;
        lastStrength = caps.strength;
    }
    if (stage == Stage::Waiting) {
        if (caps.moving || caps.queued) return stopLocked("entry_busy", now);
        lower = caps.from;
        upper = caps.to;
        entry = curve.nativeTarget(0, lower, upper);
        motionPhase = {};
        if (!pattern_motor::position(owner, entry, caps, true)) return stopLocked("entry_command", now);
        stage = Stage::Positioning;
        lastFrameAt = now;
        return;
    }
    if (stage == Stage::Positioning) {
        if (caps.moving) {
            if (!pattern_motor::position(owner, entry, caps, false)) stopLocked("entry_guard", now);
            return;
        }
        if (std::abs(static_cast<int64_t>(caps.position) - entry) > 1) {
            // A finite command was accepted, but the backend may not yet
            // report motion. Allow one backend update without resubmitting it.
            if (now - lastFrameAt < 20) return;
            return stopLocked("entry_position", now);
        }
        stage = Stage::Playing;
        lastProgressAt = lastFrameAt = now;
    }
    if (now - lastProgressAt > drawn_pattern::frameStarvationMs) return stopLocked("scheduler_gap", now); // stop after starvation; never burst to catch up
    const uint32_t frameMs = serial ? 20 : drawn_pattern::pulseFrameMs;
    if (serial && now - lastFrameAt < frameMs) return;
    const double dt = frameMs / 1000.0;
    const drawn_pattern::PhaseLimits phaseLimits{1000. / effectivePeriod, clockAcceleration, clockJerk};
    for (unsigned i = 0; i < (serial ? 1u : drawn_pattern::pulseQueueFrames); ++i) {
        const bool withinNewEnvelope =
            motionPhase.clock.secondsToGoal() == 0 && motionPhase.clock.rate() <= phaseLimits.rate + 1e-12;
        const double amplitude = requestedVibration && vibrationBudget.matches(caps.speed, caps.acceleration) &&
                                         (withinNewEnvelope || !vibration.gain.atZero())
                                     ? caps.strength
                                     : 0;
        auto nextPhase = motionPhase;
        auto nextVibration = vibration;
        drawn_pattern::FrameBrake frameBrake;
        const auto advance = [&](auto &p, auto &v, double seconds) {
            p.advance(seconds, phaseLimits, markerIndex, caps.strength, &frameBrake);
            if (markerIndex.hasVibration) v.advance(seconds, amplitude);
        };
        const auto target = [&](const auto &p, const auto &v) {
            if (v.gain.atZero()) return curve.nativeTarget(p.phase, lower, upper);
            const double y = curve.position(p.phase);
            return upper - static_cast<int32_t>(std::lround(
                               (upper - lower) * y + v.offset(y, p.phase, markerIndex, vibrationBudget)));
        };
        advance(nextPhase, nextVibration, dt);
        const int32_t logical = target(nextPhase, nextVibration);
        if (!serial && !crossingCached) {
            firstCrossing =
                drawn_pattern::firstStepCrossing(target(motionPhase, vibration), logical, [&](double fraction) {
                    auto p = motionPhase;
                    auto v = vibration;
                    advance(p, v, dt * fraction);
                    return target(p, v);
                });
            crossingCached = true;
        }
        const auto result = pattern_motor::submit(logical, owner, serial, firstCrossing);
        if (result.timing == drawn_pattern::FrameTiming::Started)
            observeQueueStart(pattern_motor::now());
        if (result.state == pattern_motor::Frame::Rejected)
            return stopLocked(result.reason, pattern_motor::now(), result.driverResult);
        if (result.state == pattern_motor::Frame::Retry) {
            if (result.timing == drawn_pattern::FrameTiming::PrimeTail) lastProgressAt = now;
            if (now - lastProgressAt >= drawn_pattern::frameStarvationMs)
                stopLocked(result.reason, now, result.driverResult);
            return;
        }
        if (lastAppliedPeriod != effectivePeriod ||
            (activeMarkers &&
             (lastStampedStrength != caps.strength || vibration.gain.goal() != nextVibration.gain.goal())))
            recordRateGoal(pattern_motor::now(), result.timing, caps.strength, nextVibration.gain.goal());
        crossingCached = false;
        ++acceptedFrames;
        motionPhase = nextPhase;
        vibration = nextVibration;
        lastProgressAt = lastFrameAt = now;
        // Recompute the base budget before accepting the next frame, even
        // when this call could otherwise fill several more queue entries.
        if (reserveVibration && vibration.gain.atZero() && (!requestedVibration || reconfigureAfterFade))
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
    vibration = {};
    vibrationBudget = {};
    reserveVibration = reconfigureAfterFade = false;
    lastStrength = -1;
    local = localControl;
    serial = caps.serial;
    lastStop = {};
    startedAt = renewedAt = pattern_motor::now();
    lastAppliedPeriod = acceptedFrames = 0;
    stage = Stage::Waiting;
    return PatternStart::Accepted;
}

bool renewDrawnPattern(uint64_t revision) {
    std::lock_guard<std::mutex> guard(lock);
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
    if (local || link.owner() != connection || value == 0)
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
    return {stage != Stage::Idle, effectivePeriod, lastStop};
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
bool serviceDrawnPattern(uint64_t revision) {
    std::lock_guard<std::mutex> guard(lock);
    if (stage == Stage::Idle || owner != revision) return false;
    service(pattern_motor::now());
    return true;
}
} // namespace streaming
