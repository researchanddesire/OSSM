#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace drawn_pattern {

// Phase is the integral of the cycle rate. Each segment has constant phase
// jerk, so phase rate and phase acceleration remain continuous on retargeting.
class PhaseClock {
public:
    void setGoal(double goal, double maxAcceleration, double maxJerk) {
        if (goal == goal_ && maxAcceleration == accelerationLimit_ && maxJerk == jerkLimit_) return;
        size_ = index_ = 0;
        double r = rate_, a = acceleration_;
        // If the acceleration control itself changed, shed the acceleration
        // already in flight using its old stopping envelope. A speed-only
        // retarget can start immediately with the existing acceleration.
        if (a != 0 && jerkLimit_ > 0 &&
            (maxJerk != jerkLimit_ || std::abs(a) > maxAcceleration)) {
            const double duration = std::abs(a) / jerkLimit_;
            append(duration, -std::copysign(jerkLimit_, a));
            r += .5*a*duration;
            a = 0;
        }
        goal_ = goal;
        accelerationLimit_ = maxAcceleration;
        jerkLimit_ = maxJerk;
        const double stopRate = r + a * std::abs(a) / (2*maxJerk);
        const double sign = goal >= stopRate ? 1. : -1.;
        const double initial = sign*a;
        const double distance = sign*(goal-r);
        const double peak = std::min(maxAcceleration,
            std::sqrt(std::max(0., maxJerk*distance + .5*initial*initial)));
        // Integral of the two acceleration ramps is (peak² − initial²/2)/jerk.
        // A positive remainder is covered by a constant-acceleration segment.
        append((peak-initial)/maxJerk, sign*maxJerk);
        if (peak > 0) append((distance-(peak*peak-.5*initial*initial)/maxJerk)/peak,0);
        append(peak/maxJerk,-sign*maxJerk);
        if (size_ == 0) { rate_=goal_; acceleration_=0; }
    }

    double advance(double seconds) {
        double distance = 0;
        while (seconds > 0 && index_ < size_) {
            auto &segment = segments_[index_];
            const double t = std::min(seconds, segment.seconds);
            distance += t*(rate_ + t*(.5*acceleration_ + t*segment.jerk/6));
            rate_ += t*(acceleration_ + .5*t*segment.jerk);
            acceleration_ += t*segment.jerk;
            seconds -= t;
            segment.seconds -= t;
            if (segment.seconds == 0) ++index_;
        }
        if (index_ == size_) {
            rate_ = goal_;
            acceleration_ = 0;
            distance += rate_*seconds;
        }
        return distance;
    }
    // Crossing probes only need displacement. Within a single ramp segment
    // use the same integral as advance() without changing the clock.
    double distanceAt(double seconds) const {
        if (index_ == size_) return rate_ * seconds;
        const auto &s = segments_[index_];
        if (seconds <= s.seconds)
            return seconds * (rate_ + seconds * (.5 * acceleration_ + seconds * s.jerk / 6));
        auto clock = *this;
        return clock.advance(seconds);
    }
    double secondsToGoal() const {
        double time = 0;
        for (unsigned i = index_; i < size_; ++i) time += segments_[i].seconds;
        return time;
    }
    double rate() const { return rate_; }
    double acceleration() const { return acceleration_; }

private:
    struct Segment { double seconds, jerk; };
    Segment segments_[4]{};
    unsigned size_=0, index_=0;
    double rate_=0, acceleration_=0, goal_=-1, accelerationLimit_=0, jerkLimit_=0;
    void append(double seconds, double jerk) {
        if (seconds > 0) segments_[size_++]={seconds,jerk};
    }
};
// Locate the first change of the integer STEP position within the frame.
// The serializer groups subsequent same-direction edges until the next
// frame's first crossing. Eleven bisections resolve a 2 ms frame to 1 µs.
template <class PositionAt>
uint16_t firstStepCrossing(int32_t from, int32_t to, const PositionAt &positionAt) {
    if (from==to) return 32768;
    unsigned lo=0, hi=2048;
    while (hi-lo>1) {
        const auto middle=(lo+hi)/2;
        if (positionAt(middle/2048.)==from) lo=middle;
        else hi=middle;
    }
    return static_cast<uint16_t>(((lo+hi)*65535u+2048u)/4096u);
}
} // namespace drawn_pattern
