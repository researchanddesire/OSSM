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
    // A single constant-jerk segment has linear acceleration. Its smallest
    // endpoint acceleration bounds the rate loss over the entire frame.
    bool forwardDuring(double seconds) const {
        if (seconds < 0 || rate_ < 0) return false;
        if (index_ == size_) return true;
        const auto &segment = segments_[index_];
        if (seconds > segment.seconds) return false;
        const double endAcceleration = acceleration_ + seconds * segment.jerk;
        const double minimumAcceleration = std::min({0., acceleration_, endAcceleration});
        return rate_ + seconds * minimumAcceleration >= 0;
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
// The STEP serializer uses the midpoint of an adjacent pair on the
// original 2,048-interval grid. Keep its rounding identical on every path.
constexpr unsigned crossingGrid = 2048;
inline uint16_t encodeStepCrossing(unsigned low) {
    return static_cast<uint16_t>(((2 * low + 1) * 65535u + crossingGrid) / (2 * crossingGrid));
}

template <class BeforeEdge>
unsigned bisectStepCrossing(unsigned low, unsigned high, const BeforeEdge &beforeEdge) {
    while (high - low > 1) {
        const unsigned middle = (low + high) / 2;
        if (beforeEdge(middle)) low = middle;
        else high = middle;
    }
    return low;
}

// Preserve the original search order for arbitrary, possibly reversing motion.
template <class PositionAt>
uint16_t firstStepCrossing(int32_t from, int32_t to, const PositionAt &positionAt) {
    if (from == to) return 32768;
    return encodeStepCrossing(bisectStepCrossing(0, crossingGrid, [&](unsigned index) {
        return positionAt(double(index) / crossingGrid) == from;
    }));
}

// On a monotone interval all samples before the first edge are one contiguous
// prefix. A prediction can shorten the search, but never determines the result.
template <class BeforeEdge>
unsigned predictedStepCrossing(unsigned guess, const BeforeEdge &beforeEdge) {
    unsigned low = 0, high = crossingGrid;
    if (guess == 0 || beforeEdge(guess)) {
        low = guess;
        if (low + 1 == high || !beforeEdge(low + 1)) return low;
        ++low;
        const unsigned nearby = std::min(high, low + 8);
        if (nearby < high) {
            if (beforeEdge(nearby)) low = nearby;
            else high = nearby;
        }
    } else {
        high = guess;
        const unsigned nearby = high > 8 ? high - 8 : 0;
        if (nearby > 0) {
            if (beforeEdge(nearby)) low = nearby;
            else high = nearby;
        }
    }
    return bisectStepCrossing(low, high, beforeEdge);
}
} // namespace drawn_pattern
