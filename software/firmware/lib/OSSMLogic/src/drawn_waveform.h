#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace drawn_pattern {

constexpr size_t minSamples = 16;
constexpr size_t maxSamples = 128;
constexpr uint32_t minPeriodMs = 500;
constexpr uint32_t maxPeriodMs = 60000;
constexpr uint32_t leaseMs = 5000;

// A periodic quintic passes through the submitted positions with continuous
// velocity and acceleration, including the loop seam and the ends of dwells.
class Waveform {
public:
    bool decode(const char *hex, size_t length, uint32_t period, bool q16 = false) {
        const size_t digits = q16 ? 4 : 2;
        if (!hex || length % digits || length < digits * minSamples ||
            length > 2 * maxSamples || period < minPeriodMs || period > maxPeriodMs)
            return false;
        for (size_t i = 0; i < length / 2; ++i) {
            const int high = nibble(hex[2*i]), low = nibble(hex[2*i+1]);
            if (high < 0 || low < 0) return false;
            samples_[i] = static_cast<uint8_t>(high * 16 + low);
        }
        q16_ = q16;
        count_ = length / digits;
        start_ = 0;
        for (size_t i = 1; i < count_; ++i) {
            if (sample(i) < sample(start_)) start_ = i;
        }
        periodMs_ = period;
        cachedIndex_ = maxSamples;
        slope_ = curvature_ = third_ = 0;
        for (size_t i = 0; i < count_; ++i) {
            double controls[6];
            controlPoints(i, controls);
            boundDerivatives(controls, 0);
        }
        return true;
    }

    double position(double phase) const {
        const double u = prepareCell(phase);
        double value = polynomial_[5];
        for (int i = 4; i >= 0; --i) value = value * u + polynomial_[i];
        return value / scale();
    }

    // Derivatives are with respect to cycles, in normalized extension units.
    double phaseDerivative(double phase, unsigned order) const {
        const double u = prepareCell(phase);
        const double *p = polynomial_;
        if (order == 1) return ((((5*p[5]*u+4*p[4])*u+3*p[3])*u+2*p[2])*u+p[1])*count_/scale();
        if (order == 2) return (((20*p[5]*u+12*p[4])*u+6*p[3])*u+2*p[2])*count_*count_/scale();
        if (order == 3) return ((60*p[5]*u+24*p[4])*u+6*p[3])*count_*count_*count_/scale();
        return position(phase);
    }

    int32_t nativeTarget(double phase, int32_t deepest, int32_t shallowest) const {
        const double span = static_cast<double>(shallowest) - deepest;
        return shallowest - static_cast<int32_t>(std::lround(span * position(phase)));
    }

    // acceleration is the share reserved for the position curve at steady rate.
    // The complementary half is available for live changes in phase rate.
    uint32_t safePeriodMs(double span, double speed, double acceleration) const {
        if (!(speed > 0 && acceleration > 0 && span >= 0)) return 0;
        const double velocityTime = maximumSlope(span) / speed;
        const double accelerationTime = std::sqrt(maximumCurvature(span) / acceleration);
        const double jerkTime = std::cbrt(maximumThird(span) / jerkBudget(acceleration));
        const double ms = std::max(double(periodMs_), 1000 * std::max({velocityTime, accelerationTime, jerkTime}));
        return std::isfinite(ms) && ms <= UINT32_MAX ? static_cast<uint32_t>(std::ceil(ms)) : 0;
    }

    struct RateLimits { double acceleration, jerk; };
    RateLimits rateLimits(double span, double acceleration) const {
        const double slope = std::max(1., maximumSlope(span));
        const double curvature = maximumCurvature(span);
        const double quarterJerk = jerkBudget(acceleration) / 4;
        const double fastest = safePeriodMs(span, HUGE_VAL, acceleration / 2);
        double phaseAcceleration = acceleration / (2*slope);
        // The third derivative of the shape times rate cubed uses half the jerk budget. The terms 3 X'' r a and
        // X' j each receive one quarter across the full positive speed range.
        if (curvature > 0 && fastest > 0)
            phaseAcceleration = std::min(phaseAcceleration, quarterJerk*fastest/(3000*curvature));
        return {phaseAcceleration, quarterJerk/slope};
    }

    double maximumSlope(double span) const { return slope_ * count_ / scale() * span; }
    double maximumCurvature(double span) const { return curvature_ * count_ * count_ / scale() * span; }
    double maximumThird(double span) const { return third_ * count_ * count_ * count_ / scale() * span; }

    // Caps the change in commanded acceleration to its configured range per
    // 100 ms. The steady shape receives half this jerk budget, the phase clock
    // receives the rest. Physical Stop and guard trips bypass the motion clock.
    static double jerkBudget(double acceleration) { return acceleration / .100; }

private:
    double prepareCell(double phase) const {
        const double index = (phase - std::floor(phase)) * count_;
        const auto whole = static_cast<size_t>(index);
        const size_t cell = (start_ + whole) % count_;
        if (cachedIndex_ != cell) {
            double b[6];
            controlPoints(cell, b);
            // Power coefficients from forward differences of Bezier controls.
            constexpr double binomial[] = {1, 5, 10, 10, 5, 1};
            for (unsigned k = 0; k <= 5; ++k) {
                polynomial_[k] = binomial[k] * b[0];
                for (unsigned j = 0; j < 5-k; ++j) b[j] = b[j+1] - b[j];
            }
            cachedIndex_ = cell;
        }
        return index - whole;
    }
    struct Knot { double y, m, a; };
    static int nibble(char c) {
        if (c >= '0' && c <= '9') return c-'0';
        if (c >= 'a' && c <= 'f') return c-'a'+10;
        if (c >= 'A' && c <= 'F') return c-'A'+10;
        return -1;
    }
    double scale() const { return q16_ ? 65535. : 255.; }
    int sample(size_t i) const {
        return q16_ ? (int(samples_[2*i]) << 8) | samples_[2*i+1] : samples_[i];
    }
    Knot knot(size_t i) const {
        const double y = sample(i);
        const int left = int(y) - sample((i + count_ - 1) % count_);
        const int right = sample((i + 1) % count_) - int(y);
        if (!left || !right) return {y, 0, 0}; // Both ends of a hold are stationary.
        // Local estimates of the minimum-integrated-jerk periodic cardinal
        // quintic derivatives. The four symmetric pairs reproduce polynomial
        // derivatives through degree five. No full-curve table is allocated.
        constexpr double first[] = {.92917162386602,-.32917162386602,.09345450546639,-.01279797313329};
        constexpr double second[] = {2.18754700527005,-.63736559872486,.21873561364671,-.03791907082444};
        double m = 0, a = 0;
        for (size_t j = 1; j <= 4; ++j) {
            const int before = sample((i + count_ - j) % count_);
            const int after = sample((i + j) % count_);
            m += first[j-1] * (after-before);
            a += second[j-1] * (after+before-2*y);
        }
        if ((left > 0) != (right > 0) || (m > 0) != (right > 0)) m = 0;
        // Order each end's first three controls, preserving a nonzero
        // acceleration at a turn when the adjacent data permit it.
        if (right > 0 || left < 0) a = std::max(a, -4 * std::abs(m));
        if (right < 0 || left > 0) a = std::min(a,  4 * std::abs(m));
        return {y, m, a};
    }
    static double cellScale(const Knot &l, const Knot &r) {
        const double offsets = std::abs(.4*l.m + .05*l.a) + std::abs(.4*r.m - .05*r.a);
        return offsets > 0 ? std::min(1., std::abs(r.y-l.y)/offsets) : 1.;
    }
    void controlPoints(size_t i, double (&b)[6]) const {
        const Knot l = knot(i), r = knot((i+1) % count_);
        // A shared scale at each knot preserves C2. The two inner controls
        // can use at most the interval's travel, so the entire control polygon
        // is monotone, and its Bezier derivative has the same sign everywhere.
        const double center = cellScale(l,r);
        const double left = std::min(center,cellScale(knot((i+count_-1)%count_),l));
        const double right = std::min(center,cellScale(r,knot((i+2)%count_)));
        b[0]=l.y; b[1]=l.y+left*l.m/5; b[2]=l.y+left*(.4*l.m+.05*l.a);
        b[5]=r.y; b[4]=r.y-right*r.m/5; b[3]=r.y+right*(-.4*r.m+.05*r.a);
    }
    void boundDerivatives(double (&b)[6], unsigned depth) {
        // Two exact half subdivisions tighten the convex-hull bounds on the
        // derivatives without solving a quartic or sampling a potentially
        // narrow extremum. Rescale subinterval derivatives to one sample cell.
        if (depth < 2) {
            double left[6], right[6];
            for (unsigned k=0;k<6;++k) {
                left[k]=b[0]; right[5-k]=b[5-k];
                for (unsigned j=0;j<5-k;++j) b[j]=.5*(b[j]+b[j+1]);
            }
            boundDerivatives(left,depth+1);
            boundDerivatives(right,depth+1);
            return;
        }
        for (unsigned j=0;j<5;++j) { b[j]=20*(b[j+1]-b[j]); slope_=std::max(slope_,std::abs(b[j])); }
        for (unsigned j=0;j<4;++j) { b[j]=16*(b[j+1]-b[j]); curvature_=std::max(curvature_,std::abs(b[j])); }
        for (unsigned j=0;j<3;++j) { b[j]=12*(b[j+1]-b[j]); third_=std::max(third_,std::abs(b[j])); }
    }

    uint8_t samples_[maxSamples]{};
    size_t count_=0, start_=0;
    uint32_t periodMs_=0;
    bool q16_=false;
    double slope_=0, curvature_=0, third_=0;
    mutable size_t cachedIndex_=maxSamples;
    mutable double polynomial_[6]{};
};

// A STEP/DIR frame carries a bounded count spread over its entire interval.
// An empty frame is a timed hold. Rounding absolute positions prevents drift.
constexpr uint32_t pulseFrameMs = 2;
struct PulseFrame { uint16_t ticks = 0; uint8_t steps = 0; bool increasing = false; };
inline bool pulseFrame(int32_t from, int32_t to, uint32_t ticksPerFrame, bool lastDirection,
                       PulseFrame &frame) {
    const int64_t delta = static_cast<int64_t>(to) - from;
    const uint64_t count = std::abs(delta);
    if (count > 255 || ticksPerFrame == 0 || ticksPerFrame > UINT16_MAX) return false;
    const uint32_t ticks = count ? (ticksPerFrame + count / 2) / count : ticksPerFrame;
    if (ticks == 0) return false;
    frame = {static_cast<uint16_t>(ticks), static_cast<uint8_t>(count),
             delta == 0 ? lastDirection : delta > 0};
    return true;
}

} // namespace drawn_pattern
