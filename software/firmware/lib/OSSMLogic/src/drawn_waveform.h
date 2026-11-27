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
        double lowerPeaks[3]{};
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
                boundDerivatives(controls, lowerPeaks);
            }
            first = last + 1;
        }
        slopeWitness_ = lowerPeaks[0];
        return true;
    }

    double authoredStart() const { return double(start_) / count_; }

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

    struct Pace {
        uint32_t periodMs = 0;
        double rate = 0, peak = 0, phaseAcceleration = 0, phaseJerk = 0;
        const char *limit = "flat";
    };

    // Positive speed selects a fraction of the fastest shape-preserving
    // clock on this physical span. Reserve derivative headroom for live
    // changes in phase rate, acceleration and jerk.
    Pace pace(double span, double fullSpeed, double acceleration,
              double fraction, uint32_t frameMs) const {
        Pace out;
        if (!(span > 0 && fullSpeed > 0 && acceleration > 0 &&
              fraction > 0 && fraction <= 1 && frameMs > 0)) return out;
        const double slope = maximumSlope(span);
        const double curvature = maximumCurvature(span);
        const double third = maximumThird(span);
        const double jerk = jerkBudget(acceleration);
        double seconds = slope / fullSpeed;
        out.limit = "speed";
        const auto requireDuration = [&](double duration, const char *cause) {
            if (duration > seconds) { seconds = duration; out.limit = cause; }
        };
        requireDuration(std::sqrt(curvature / (.6 * acceleration)), "acceleration");
        requireDuration(std::cbrt(third / (.7 * jerk)), "jerk");
        // Cover each cell at the actual backend cadence. A spatially
        // constant path has no slope from which to recover its clock.
        requireDuration(double(count_) * frameMs / 1000., "sampling");
        if (slope == 0) {
            seconds = std::max(seconds, periodMs_ / 1000.);
            out.limit = "flat";
        }
        const double milliseconds = std::ceil(1000. * seconds / fraction);
        out.periodMs = static_cast<uint32_t>(std::min(milliseconds, double(UINT32_MAX)));
        out.rate = fraction / seconds;
        out.peak = slopeWitness_ * count_ / scale() * span * out.rate;
        const double stableSlope = std::max(1., slope);
        out.phaseAcceleration = .4 * acceleration / stableSlope;
        if (curvature > 0)
            out.phaseAcceleration = std::min(out.phaseAcceleration, .15 * jerk * seconds / (3 * curvature));
        out.phaseJerk = .15 * jerk / stableSlope;
        return out;
    }

    double maximumSlope(double span) const { return slope_ * count_ / scale() * span; }
    double maximumCurvature(double span) const { return curvature_ * count_ * count_ / scale() * span; }
    double maximumThird(double span) const { return third_ * count_ * count_ * count_ / scale() * span; }

    // Caps the change in commanded acceleration to its configured range per
    // 100 ms. The steady shape receives 70% of this jerk budget, and
    // the live phase clock reserves 15% for each mixed/control term. Physical Stop and guard trips bypass the motion clock.
    static double jerkBudget(double acceleration) { return acceleration / .100; }

private:
    double prepareCell(double phase) const {
        if (phase >= 1 || phase < 0) phase -= std::floor(phase);
        const double index = phase * count_;
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
    // Bernstein coefficients enclose the full polynomial. Bisect only
    // intervals whose upper bound can exceed an observed endpoint peak.
    // At the depth limit the enclosing bound remains conservative.
    template <size_t N>
    static void boundPeak(const double (&values)[N], double &lower, double &upper) {
        constexpr unsigned maximumDepth = 12;
        struct Branch { double b[N]; unsigned depth; };
        Branch saved[maximumDepth];
        unsigned count = 0, depth = 0;
        double cell[N];
        std::copy(values, values+N, cell);
        for (;;) {
            lower = std::max({lower, std::abs(cell[0]), std::abs(cell[N-1])});
            double hull = 0;
            for (double v : cell) hull = std::max(hull, std::abs(v));
            if (hull > lower * 1.00001 + 1.e-12 && depth < maximumDepth) {
                // Depth-first traversal needs at most twelve right halves
                // on the ESP32 callback stack. Reuse the current left half.
                auto &right = saved[count++];
                right.depth = ++depth;
                double work[N];
                std::copy(cell, cell+N, work);
                for (size_t k = 0; k < N; ++k) {
                    cell[k] = work[0];
                    right.b[N-1-k] = work[N-1-k];
                    for (size_t j = 0; j+1 < N-k; ++j) work[j] = .5 * (work[j] + work[j+1]);
                }
            } else {
                upper = std::max(upper, hull);
                if (!count) return;
                const auto &right = saved[--count];
                std::copy(right.b, right.b+N, cell);
                depth = right.depth;
            }
        }
    }
    void boundDerivatives(const double (&b)[6], double (&lower)[3]) {
        double v[5], a[4], j[3];
        for (size_t i=0; i<5; ++i) v[i] = 5 * (b[i+1] - b[i]);
        for (size_t i=0; i<4; ++i) a[i] = 4 * (v[i+1] - v[i]);
        for (size_t i=0; i<3; ++i) j[i] = 3 * (a[i+1] - a[i]);
        boundPeak(v, lower[0], slope_);
        boundPeak(a, lower[1], curvature_);
        boundPeak(j, lower[2], third_);
    }
    uint8_t samples_[maxSamples]{};
    size_t count_=0, start_=0;
    uint32_t periodMs_=0;
    SampleFormat format_=SampleFormat::Q8;
    uint8_t runs_=0;
    double slope_=0, slopeWitness_=0, curvature_=0, third_=0;
    mutable size_t cachedIndex_=0;
    mutable double polynomial_[6]{};
};

// The continuous phase clock advances on this interval, independently of the
// sub-frame time at which the STEP peripheral emits an electrical edge.
constexpr uint32_t pulseFrameMs = 2;

} // namespace drawn_pattern
