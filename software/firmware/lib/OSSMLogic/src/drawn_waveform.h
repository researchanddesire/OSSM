#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include "drawn_sample_format.h"

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
    bool decode(const char *hex, size_t length, uint32_t period, SampleFormat format = SampleFormat::Q8) {
        const bool runs = format == SampleFormat::RunsQ16;
        if (!runs && format != SampleFormat::Q8 && format != SampleFormat::Q16) return false;
        const size_t digits = runs ? 8 : format == SampleFormat::Q16 ? 4 : 2;
        if (!hex || length % digits || length < digits * (runs ? 1 : minSamples) ||
            length > 2 * maxSamples || period < minPeriodMs || period > maxPeriodMs)
            return false;
        for (size_t i = 0; i < length / 2; ++i) {
            const int high = nibble(hex[2*i]), low = nibble(hex[2*i+1]);
            if (high < 0 || low < 0) return false;
            samples_[i] = static_cast<uint8_t>(high * 16 + low);
        }
        format_ = format;
        runs_ = runs ? length / digits : 0;
        count_ = runs ? sparseSamples : length / digits;
        if (runs) {
            // Each end is inclusive, so the first run starts at zero and later
            // ones start at previousEnd + 1. Exactly 1024 knots must be covered.
            int previousEnd = -1, previousHeight = -1;
            for (size_t i = 0; i < runs_; ++i) {
                const int end = word(4*i), height = word(4*i+2);
                if (end <= previousEnd || end >= static_cast<int>(sparseSamples) ||
                    height == previousHeight) return false;
                previousEnd = end;
                previousHeight = height;
            }
            if (previousEnd != static_cast<int>(sparseSamples - 1)) return false;
        }
        start_ = 0;
        cachedIndex_ = count_;
        periodMs_ = period;
        slope_ = curvature_ = third_ = 0;
        // Within each run the height is constant. Only its last cell can
        // move; validate the bounds and first-minimum entry in O(run count).
        const size_t intervals = runs ? runs_ : count_;
        size_t first = 0;
        int minimum = sample(0);
        for (size_t i = 0; i < intervals; ++i) {
            const size_t last = runs ? word(4*i) : i;
            const int height = sample(first);
            if (height < minimum) { start_ = first; minimum = height; }
            if (height != sample((last + 1) % count_)) {
                double controls[6];
                controlPoints(last, controls);
                boundDerivatives(controls, 0);
            }
            first = last + 1;
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
            const int y = sample(cell);
            if (y == sample((cell + 1) % count_)) {
                polynomial_[0] = y;
                std::fill(polynomial_ + 1, polynomial_ + 6, 0.);
                cachedIndex_ = cell;
                return index - whole;
            }
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
    double scale() const { return format_ == SampleFormat::Q8 ? 255. : 65535.; }
    int word(size_t offset) const { return (int(samples_[offset]) << 8) | samples_[offset+1]; }
    int sample(size_t i) const {
        if (format_ == SampleFormat::Q8) return samples_[i];
        if (format_ == SampleFormat::Q16) return word(2*i);
        // Find the first inclusive run end covering i; at most five comparisons.
        size_t lo = 0, hi = runs_ - 1;
        while (lo < hi) {
            const size_t mid = (lo + hi) / 2;
            if (i <= static_cast<size_t>(word(4*mid))) hi = mid;
            else lo = mid + 1;
        }
        return word(4*lo+2);
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
    SampleFormat format_=SampleFormat::Q8;
    uint8_t runs_=0;
    double slope_=0, curvature_=0, third_=0;
    mutable size_t cachedIndex_=0;
    mutable double polynomial_[6]{};
};

// The continuous phase clock advances on this interval, independently of the
// sub-frame time at which the STEP peripheral emits an electrical edge.
constexpr uint32_t pulseFrameMs = 2;

} // namespace drawn_pattern
