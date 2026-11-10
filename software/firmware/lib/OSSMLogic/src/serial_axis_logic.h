#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace serial_axis {

constexpr double encoderCountsPerRevolution = 32768.0;

inline bool insideBounds(int32_t target, int32_t firstStop, int32_t secondStop,
                         double countsPerMm, double marginMm = 6.0) {
    if (!std::isfinite(countsPerMm) || countsPerMm <= 0 ||
        !std::isfinite(marginMm) || marginMm < 0) return false;
    const int32_t lower = firstStop < secondStop ? firstStop : secondStop;
    const int32_t upper = firstStop > secondStop ? firstStop : secondStop;
    return target >= lower + marginMm * countsPerMm &&
           target <= upper - marginMm * countsPerMm;
}

struct Sample {
    uint32_t sequence = 0;
    uint32_t atMs = 0;
    uint32_t continuity = 0;
    int32_t encoder = 0;
};

inline bool speedRegisters(uint32_t stepsPerSecond, int32_t stepsPerSecond2,
                           double stepsPerRevolution, uint16_t maximumRpm,
                           uint16_t &rpm, uint16_t &acceleration) {
    if (!std::isfinite(stepsPerRevolution) || stepsPerRevolution <= 0 ||
        stepsPerSecond == 0 || stepsPerSecond2 <= 0 || maximumRpm == 0) return false;
    const double speed = std::floor(stepsPerSecond * 60.0 / stepsPerRevolution);
    const double ramp = std::floor(stepsPerSecond2 * 60.0 / stepsPerRevolution);
    // A register value of60000 disables ramping; never select that behavior.
    // Reject sub-rpm requests rather than silently exceeding requested speed.
    if (speed < 1 || ramp < 1) return false;
    rpm = static_cast<uint16_t>(speed > maximumRpm ? maximumRpm : speed);
    acceleration = static_cast<uint16_t>(ramp > 59999 ? 59999 : ramp);
    return true;
}

class Position {
public:
    explicit Position(double stepsPerRevolution = 800.0,
                      double millimetersPerRevolution = 40.0)
        : countsPerStep_(encoderCountsPerRevolution / stepsPerRevolution),
          countsPerMm_(encoderCountsPerRevolution / millimetersPerRevolution) {}

    bool observe(Sample sample, uint32_t nowMs) {
        if (sample.sequence == 0 || sample.continuity == 0 ||
            static_cast<uint32_t>(nowMs - sample.atMs) > 100) return false;
        if (haveSample_ && sample.continuity == latest_.continuity) {
            const uint32_t delta = sample.sequence - latest_.sequence;
            if (delta == 0 || delta >= 0x80000000U) return fresh(nowMs);
        }
        const bool continuous = haveSample_ && sample.continuity == latest_.continuity;
        const uint32_t gap = sample.atMs - latest_.atMs;
        if (!continuous || gap == 0 || gap > 100) {
            quiet_ = false;
            quietAt_ = sample.atMs;
            quietMinimum_ = quietMaximum_ = sample.encoder;
            quietSamples_ = 1;
            if (haveSample_) anchored_ = false;
        } else {
            if (sample.encoder < quietMinimum_) quietMinimum_ = sample.encoder;
            if (sample.encoder > quietMaximum_) quietMaximum_ = sample.encoder;
            // A low velocity is not a stop: even1RPM moves ~14counts/frame.
            // Require a stable physical encoder range over unique frames.
            if (static_cast<int64_t>(quietMaximum_) - quietMinimum_ > 2) {
                quietAt_ = sample.atMs;
                quietMinimum_ = quietMaximum_ = sample.encoder;
                quietSamples_ = 1;
            } else if (quietSamples_ < 255) ++quietSamples_;
            quiet_ = quietSamples_ >= 2 &&
                     static_cast<uint32_t>(sample.atMs - quietAt_) >= 20;
        }
        latest_ = sample;
        haveSample_ = true;
        if (anchored_) {
            const double value = anchorLogical_ +
                (static_cast<int64_t>(sample.encoder) - anchorEncoder_) /
                    countsPerStep_ * sign_;
            if (value < std::numeric_limits<int32_t>::min() ||
                value > std::numeric_limits<int32_t>::max()) anchored_ = false;
            else logical_ = static_cast<int32_t>(std::llround(value));
        }
        if (active_) {
            const bool near = std::abs(static_cast<int64_t>(sample.encoder) - targetEncoder_) <=
                              countsPerMm_ * 0.05;
            if (acknowledged_ && near && quiet_) {
                if (arrivalSamples_ < 3) ++arrivalSamples_;
                if (arrivalSamples_ >= 2) active_ = false;
            } else arrivalSamples_ = 0;
        }
        return true;
    }

    bool fresh(uint32_t nowMs) const {
        return haveSample_ && static_cast<uint32_t>(nowMs - latest_.atMs) <= 100;
    }
    bool canReanchor(uint32_t nowMs) const { return fresh(nowMs) && quiet_ && !active_; }
    bool anchored() const { return anchored_; }
    bool rebase(int32_t logical, uint32_t nowMs) {
        if (!canReanchor(nowMs)) return false;
        anchorEncoder_ = latest_.encoder;
        anchorLogical_ = logical_ = logical;
        anchored_ = true;
        return true;
    }
    bool direction(bool highCountsUp, uint32_t nowMs) {
        const int8_t next = highCountsUp ? -1 : 1;
        if (next == sign_) return true;
        if (!canReanchor(nowMs) || !anchored_) return false;
        anchorEncoder_ = latest_.encoder;
        anchorLogical_ = logical_;
        sign_ = next;
        return true;
    }
    bool mirrorFrame(int32_t keepoutSteps, uint32_t nowMs) {
        if (!canReanchor(nowMs) || !anchored_) return false;
        const int64_t anchor = -(static_cast<int64_t>(anchorLogical_) + keepoutSteps);
        const int64_t logical = -(static_cast<int64_t>(logical_) + keepoutSteps);
        if (anchor < INT32_MIN || anchor > INT32_MAX ||
            logical < INT32_MIN || logical > INT32_MAX) return false;
        // Transform the existing anchor exactly. Re-anchoring at a rounded
        // measured step on every mode switch would accumulate encoder drift.
        anchorLogical_ = static_cast<int32_t>(anchor);
        logical_ = static_cast<int32_t>(logical);
        sign_ = -sign_;
        return true;
    }
    bool target(int32_t logical, int32_t &encoder) const {
        if (!anchored_) return false;
        const double value = anchorEncoder_ +
            (static_cast<int64_t>(logical) - anchorLogical_) * countsPerStep_ * sign_;
        // Crossing the signed absolute-register boundary has not been proven
        // on hardware. Reject it; do not wrap a target into the opposite end.
        if (!std::isfinite(value) || value < std::numeric_limits<int32_t>::min() ||
            value > std::numeric_limits<int32_t>::max()) return false;
        encoder = static_cast<int32_t>(std::llround(value));
        return true;
    }
    void submitted(uint32_t requestId, int32_t logical, int32_t encoder) {
        requestId_ = requestId;
        commandTarget_ = logical;
        targetEncoder_ = encoder;
        active_ = true;
        acknowledged_ = false;
        arrivalSamples_ = 0;
    }
    void acknowledge(uint32_t requestId, int32_t target) {
        if (active_ && requestId == requestId_ && target == targetEncoder_)
            acknowledged_ = true;
    }
    bool running(uint32_t nowMs) const {
        return !fresh(nowMs) || !anchored_ || active_ || !quiet_;
    }
    int32_t current() const { return logical_; }
    int32_t commandedTarget() const { return commandTarget_; }
    int32_t encoder() const { return latest_.encoder; }
    bool quiet(uint32_t nowMs) const { return fresh(nowMs) && quiet_; }
    void canceled() { active_ = false; acknowledged_ = false; arrivalSamples_ = 0; }
    void invalidate() { canceled(); anchored_ = false; }
private:
    double countsPerStep_;
    double countsPerMm_;
    Sample latest_{};
    bool haveSample_ = false;
    bool quiet_ = false;
    bool anchored_ = false;
    bool active_ = false;
    bool acknowledged_ = false;
    uint32_t quietAt_ = 0;
    int32_t quietMinimum_ = 0, quietMaximum_ = 0;
    uint8_t quietSamples_ = 0;
    uint8_t arrivalSamples_ = 0;
    int8_t sign_ = 1;  // Native coordinates agree with the AIM encoder sign.
    int32_t anchorEncoder_ = 0;
    int32_t anchorLogical_ = 0;
    int32_t logical_ = 0;
    int32_t commandTarget_ = 0;
    int32_t targetEncoder_ = 0;
    uint32_t requestId_ = 0;
};

// A local position planner, never an encoder/arrival estimate. Proposals are
// committed only after the UART owner acknowledges their absolute target.
class Trajectory {
public:
    struct Proposal { double position = 0; double velocity = 0; bool final = false; };
    void reset(double position) { position_ = target_ = position; velocity_ = 0; }
    void target(double position) { target_ = position; }
    void stop(double acceleration) {
        target_ = position_ + velocity_ * std::abs(velocity_) /
                             (2.0 * acceleration);
    }
    bool active() const { return std::abs(target_ - position_) > 0.00001 || std::abs(velocity_) > 0.00001; }
    double position() const { return position_; }
    double targetPosition() const { return target_; }
    bool propose(double measuredPosition, double maximumFollowingSteps,
                 double speed, double acceleration, uint32_t elapsedMs,
                 Proposal &next) const {
        if (!active() || !std::isfinite(measuredPosition) ||
            !std::isfinite(maximumFollowingSteps) || maximumFollowingSteps <= 0 ||
            !std::isfinite(speed) || speed <= 0 ||
            !std::isfinite(acceleration) || acceleration <= 0 || elapsedMs == 0 ||
            std::abs(position_ - measuredPosition) > maximumFollowingSteps) return false;
        // No accumulated catch-up displacement after scheduling/transport delay.
        const double dt = (elapsedMs > 20 ? 20 : elapsedMs) / 1000.0;
        const double distance = target_ - position_;
        const double direction = distance < 0 ? -1.0 : 1.0;
        const double brakingSpeed = std::sqrt(2.0 * acceleration * std::abs(distance));
        const double desired = direction * (brakingSpeed < speed ? brakingSpeed : speed);
        const double change = desired - velocity_;
        double velocity = velocity_ + (std::abs(change) <= acceleration * dt
            ? change : (change < 0 ? -1.0 : 1.0) * acceleration * dt);
        double position = position_ + (velocity_ + velocity) * 0.5 * dt;
        bool final = direction * (target_ - position) <= 0;
        if (final) { position = target_; velocity = 0; }
        // Bound each waypoint relative to the measured physical encoder, even
        // at high requested acceleration. Any clipping reduces actual speed.
        const double lower = measuredPosition - maximumFollowingSteps;
        const double upper = measuredPosition + maximumFollowingSteps;
        if (position < lower || position > upper) {
            position = position < lower ? lower : upper;
            velocity = (position - position_) / dt;
            final = false;
        }
        next = {position, velocity, final};
        return std::isfinite(position) && std::isfinite(velocity);
    }
    void commit(const Proposal &next) { position_ = next.position; velocity_ = next.velocity; }
private:
    double position_ = 0;
    double target_ = 0;
    double velocity_ = 0;
};

}  // namespace serial_axis
