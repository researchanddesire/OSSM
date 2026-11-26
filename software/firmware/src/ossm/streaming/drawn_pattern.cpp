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
uint32_t renewedAt = 0, lastFrameAt = 0, effectivePeriod = 0;
int32_t lower = 0, upper = 0, entry = 0;
double phase = 0, cyclesPerSecond = 0;

// These values live behind lock. A task from an older streaming session
// can neither submit a frame nor release a newer session's mailbox.
void stopLocked() {
    if (stage == Stage::Playing || stage == Stage::Positioning) pattern_motor::stop(owner);
    stage = Stage::Idle;
    effectivePeriod = 0;
}

int32_t target(double p) {
    return lower + static_cast<int32_t>(std::lround((upper - lower) * curve.position(p)));
}

bool service(uint32_t now) {
    if (!local && now - renewedAt >= drawn_pattern::leaseMs) return false;
    const auto caps = pattern_motor::read(owner);
    if (!caps.ready || caps.serial != serial) return false;
    if (caps.speed < 1 || caps.acceleration < 1 || caps.to <= caps.from)
        return stage == Stage::Waiting;
    if (stage != Stage::Waiting && (lower != caps.from || upper != caps.to))
        return false; // never jump to a new stroke window mid-cycle
    const uint32_t period = curve.safePeriodMs(caps.to - caps.from, caps.speed, caps.acceleration / 2);
    if (!period) return false;
    effectivePeriod = period;
    if (stage == Stage::Waiting) {
        if (caps.moving || caps.queued) return false;
        lower = caps.from;
        upper = caps.to;
        entry = target(0);
        phase = cyclesPerSecond = 0;
        const bool ok = pattern_motor::position(owner, entry, caps, true);
        if (ok) stage = Stage::Positioning;
        return ok;
    }
    if (stage == Stage::Positioning) {
        if (caps.moving) return pattern_motor::position(owner, entry, caps, false);
        if (std::abs(static_cast<int64_t>(caps.position) - entry) > 1) return false;
        stage = Stage::Playing;
        lastFrameAt = now;
    }
    if (now - lastFrameAt > 200) return false; // stop after starvation; never burst to catch up
    const uint32_t frameMs = serial ? 20 : drawn_pattern::pulseFrameMs;
    if (serial && now - lastFrameAt < frameMs) return true;
    const double dt = frameMs / 1000.0;
    const double goal = 1000.0 / period;
    // Half the acceleration budget is reserved for changing phase rate,
    // including starting at rest and live speed changes.
    const double rateDelta = caps.acceleration * 0.5 * dt /
                             std::max(1.0, curve.maximumSlope(upper - lower));
    for (unsigned i = 0; i < (serial ? 1u : 6u); ++i) {
        const double rate = cyclesPerSecond + std::clamp(goal - cyclesPerSecond, -rateDelta, rateDelta);
        const double next = phase + 0.5 * (rate + cyclesPerSecond) * dt;
        const int32_t logical = target(next);
        const auto result = pattern_motor::submit(logical, owner, serial);
        if (result == pattern_motor::Frame::Rejected) return false;
        if (result == pattern_motor::Frame::Retry) return now - lastFrameAt < 200;
        cyclesPerSecond = rate;
        phase = next - std::floor(next);
        lastFrameAt = now;
    }
    return true;
}
} // namespace

bool patternOwnerCurrent(const PatternReferenceLost &event) {
    return pattern_motor::current(event.revision);
}

PatternStart startDrawnPattern(const char *hex, size_t length, uint32_t periodMs,
                               uint64_t revision, bool localControl) {
    drawn_pattern::Waveform incoming;
    if (!incoming.decode(hex, length, periodMs)) return PatternStart::Invalid;
    std::lock_guard<std::mutex> guard(lock);
    if (!ready || owner != revision) return PatternStart::NotReady;
    const auto caps = pattern_motor::read(revision);
    if (!caps.ready) return PatternStart::NotReady;
    if (stage != Stage::Idle || caps.moving || caps.queued) return PatternStart::Busy;
    curve = incoming;
    local = localControl;
    serial = caps.serial;
    renewedAt = pattern_motor::now();
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
    stopLocked();
}
void cancelRemoteDrawnPattern() {
    std::lock_guard<std::mutex> guard(lock);
    if (!local) stopLocked();
}
bool drawnPatternActive() {
    std::lock_guard<std::mutex> guard(lock);
    return stage != Stage::Idle;
}
uint32_t drawnPatternPeriodMs() {
    std::lock_guard<std::mutex> guard(lock);
    return effectivePeriod;
}
void setDrawnPatternReady(uint64_t revision, bool enabled) {
    std::lock_guard<std::mutex> guard(lock);
    if (enabled && pattern_motor::current(revision)) {
        stopLocked();
        ready = true;
        owner = revision;
    } else if (owner == revision) {
        ready = false;
        stopLocked();
    }
}
bool serviceDrawnPattern(uint64_t revision) {
    std::lock_guard<std::mutex> guard(lock);
    if (stage == Stage::Idle || owner != revision) return false;
    if (!service(pattern_motor::now())) stopLocked();
    return true;
}
} // namespace streaming
