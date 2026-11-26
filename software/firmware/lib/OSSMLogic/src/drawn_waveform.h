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

// One periodic, uniformly sampled position curve. Monotone Hermite tangents
// preserve the range of every interval and share velocity at the loop seam.
class Waveform {
public:
    bool decode(const char *hex, size_t length, uint32_t period) {
        if (hex == nullptr || (length & 1) || length < 2 * minSamples ||
            length > 2 * maxSamples || period < minPeriodMs || period > maxPeriodMs)
            return false;
        count_ = length / 2;
        start_ = 0;
        for (size_t i = 0; i < count_; ++i) {
            const int high = nibble(hex[2 * i]);
            const int low = nibble(hex[2 * i + 1]);
            if (high < 0 || low < 0) return false;
            samples_[i] = high * 16 + low;
            if (samples_[i] < samples_[start_]) start_ = i;
        }
        periodMs_ = period;
        peakSlope_ = 0;
        peakCurvature_ = 0;
        for (size_t i = 0; i < count_; ++i) {
            const auto s = segment(i);
            const auto slope = [&](double u) { return (3 * s.a * u + 2 * s.b) * u + s.c; };
            double peak = std::max(std::abs(slope(0)), std::abs(slope(1)));
            if (s.a != 0) {
                const double vertex = -s.b / (3 * s.a);
                if (vertex > 0 && vertex < 1) peak = std::max(peak, std::abs(slope(vertex)));
            }
            peakSlope_ = std::max(peakSlope_, peak);
            peakCurvature_ = std::max(peakCurvature_,
                std::max(std::abs(2 * s.b), std::abs(6 * s.a + 2 * s.b)));
        }
        return true;
    }

    // Start a cycle at rest at its first minimum. At a plateau, both
    // adjacent tangents are zero. Time advances from queued motor frames.
    double position(double phase) const {
        const double index = (phase - std::floor(phase)) * count_;
        const size_t whole = static_cast<size_t>(index);
        const auto s = segment((start_ + whole) % count_);
        const double u = index - whole;
        return (((s.a * u + s.b) * u + s.c) * u + s.d) / 255.0;
    }

    // Authored position grows with physical extension. Native step counts
    // decrease from the shallow end toward the deep end of the travel window.
    int32_t nativeTarget(double phase, int32_t deepest, int32_t shallowest) const {
        const double travel = static_cast<double>(shallowest) - deepest;
        return shallowest - static_cast<int32_t>(std::lround(travel * position(phase)));
    }

    uint32_t safePeriodMs(double span, double speed, double acceleration) const {
        if (!(speed > 0 && acceleration > 0 && span >= 0)) return 0;
        const double velocityTime = span * peakSlope_ * count_ / (255.0 * speed);
        const double accelerationTime = count_ * std::sqrt(span * peakCurvature_ /
                                                            (255.0 * acceleration));
        const double ms = std::max(double(periodMs_), 1000 * std::max(velocityTime, accelerationTime));
        return std::isfinite(ms) && ms <= UINT32_MAX ? static_cast<uint32_t>(std::ceil(ms)) : 0;
    }

    double maximumSlope(double span) const { return span * peakSlope_ * count_ / 255.0; }

private:
    struct Cubic { double a, b, c, d; };
    static int nibble(char c) {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    }
    double tangent(size_t i) const {
        const int left = samples_[i] - samples_[(i + count_ - 1) % count_];
        const int right = samples_[(i + 1) % count_] - samples_[i];
        return left * right <= 0 ? 0.0 : 2.0 * left * right / (left + right);
    }
    Cubic segment(size_t i) const {
        const size_t next = (i + 1) % count_;
        const int delta = samples_[next] - samples_[i];
        const double left = tangent(i), right = tangent(next);
        return {left + right - 2 * delta, 3 * delta - 2 * left - right, left, double(samples_[i])};
    }
    uint8_t samples_[maxSamples]{};
    size_t count_ = 0, start_ = 0;
    uint32_t periodMs_ = 0;
    double peakSlope_ = 0, peakCurvature_ = 0;
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
