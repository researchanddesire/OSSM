#include "drawn_waveform.h"
#include "drawn_pattern.h"

#include <mutex>

#include "drawn_pattern_motor.h"

namespace streaming {
namespace {
enum class Stage { Idle, Waiting, Positioning, Playing };
std::mutex lock;
drawn_pattern::Waveform curve;
Stage stage = Stage::Idle;
bool ready = false, local = false, serial = false;
uint64_t owner = 0;
PatternStop lastStop;
uint32_t startedAt = 0, renewedAt = 0, lastFrameAt = 0, effectivePeriod = 0;
int32_t lower = 0, upper = 0, entry = 0;
double phase = 0, cyclesPerSecond = 0;

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
    lastStop = {reason, stageName(), now, now - startedAt, driverResult};
    if (stage == Stage::Playing || stage == Stage::Positioning) pattern_motor::stop(owner);
    stage = Stage::Idle;
    effectivePeriod = 0;
}

int32_t target(double p) {
    return lower + static_cast<int32_t>(std::lround((upper - lower) * curve.position(p)));
}

void service(uint32_t now) {
    if (!local && now - renewedAt >= drawn_pattern::leaseMs) return stopLocked("lease_expired", now);
    const auto caps = pattern_motor::read(owner);
    if (!caps.ready) return stopLocked("guard", now);
    if (caps.serial != serial) return stopLocked("backend_changed", now);
    if (caps.speed < 1 || caps.acceleration < 1 || caps.to <= caps.from) {
        if (stage != Stage::Waiting) stopLocked("limits_zero", now);
        return;
    }
    if (stage != Stage::Waiting && (lower != caps.from || upper != caps.to))
        return stopLocked("window_changed", now);
    const uint32_t period = curve.safePeriodMs(caps.to - caps.from, caps.speed, caps.acceleration / 2);
    if (!period) return stopLocked("period_limit", now);
    effectivePeriod = period;
    if (stage == Stage::Waiting) {
        if (caps.moving || caps.queued) return stopLocked("entry_busy", now);
        lower = caps.from;
        upper = caps.to;
        entry = target(0);
        phase = cyclesPerSecond = 0;
        if (!pattern_motor::position(owner, entry, caps, true)) return stopLocked("entry_command", now);
        stage = Stage::Positioning;
        return;
    }
    if (stage == Stage::Positioning) {
        if (caps.moving) {
            if (!pattern_motor::position(owner, entry, caps, false)) stopLocked("entry_guard", now);
            return;
        }
        if (std::abs(static_cast<int64_t>(caps.position) - entry) > 1)
            return stopLocked("entry_position", now);
        stage = Stage::Playing;
        lastFrameAt = now;
    }
    if (now - lastFrameAt > 200) return stopLocked("scheduler_gap", now); // stop after starvation; never burst to catch up
    const uint32_t frameMs = serial ? 20 : drawn_pattern::pulseFrameMs;
    if (serial && now - lastFrameAt < frameMs) return;
    const double dt = frameMs / 1000.0;
    const double goal = 1000.0 / period;
    // Half the acceleration budget is reserved for changing phase rate,
    // including starting at rest and live speed changes.
    const double rateDelta = caps.acceleration * 0.5 * dt /
                             std::max(1.0, curve.maximumSlope(upper - lower));
    for (unsigned i = 0; i < (serial ? 1u : drawn_pattern::pulseQueueFrames); ++i) {
        const double rate = cyclesPerSecond + std::clamp(goal - cyclesPerSecond, -rateDelta, rateDelta);
        const double next = phase + 0.5 * (rate + cyclesPerSecond) * dt;
        const int32_t logical = target(next);
        const auto result = pattern_motor::submit(logical, owner, serial);
        if (result.state == pattern_motor::Frame::Rejected)
            return stopLocked(result.reason, now, result.driverResult);
        if (result.state == pattern_motor::Frame::Retry) {
            if (now - lastFrameAt >= 200) stopLocked(result.reason, now, result.driverResult);
            return;
        }
        cyclesPerSecond = rate;
        phase = next - std::floor(next);
        lastFrameAt = now;
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

PatternStart startDrawnPattern(const char *hex, size_t length, uint32_t periodMs,
                               uint64_t revision, bool localControl) {
    drawn_pattern::Waveform incoming;
    if (!incoming.decode(hex, length, periodMs)) return PatternStart::Invalid;
    std::lock_guard<std::mutex> guard(lock);
    pattern_motor::State caps;
    const auto status = startStatusLocked(revision, caps);
    if (status != PatternStart::Accepted) return status;
    curve = incoming;
    local = localControl;
    serial = caps.serial;
    lastStop = {};
    startedAt = renewedAt = pattern_motor::now();
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
