#pragma once

#include <algorithm>
#include <cmath>

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
} // namespace drawn_pattern
