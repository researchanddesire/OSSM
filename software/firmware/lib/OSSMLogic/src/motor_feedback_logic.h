#pragma once

#include <cmath>
#include <cstdint>

// Coordinate conversion only. AIM absolute position is a wrapping signed
// 32-bit encoder counter, independent of the STEP/DIR electronic gear.
namespace motor_feedback_logic {

constexpr double encoderCountsPerRevolution = 32768.0;

inline int64_t encoderDelta(int32_t position, int32_t origin) {
    const uint32_t delta = static_cast<uint32_t>(position) -
                           static_cast<uint32_t>(origin);
    return delta <= 0x7fffffffU ? static_cast<int64_t>(delta)
                              : static_cast<int64_t>(delta) - 0x100000000LL;
}

// Learn the physical encoder polarity from a completed native-frame homing
// traverse. Tiny moves and inconsistent distances cannot establish polarity.
// int64 arithmetic also avoids overflow around either signed counter boundary.
inline int8_t learnNativePolarity(int32_t encoderStart, int32_t encoderEnd,
                                 int32_t commandStart, int32_t commandEnd,
                                 double motorStepsPerRevolution,
                                 double minimumTravelSteps) {
    if (!std::isfinite(motorStepsPerRevolution) ||
        motorStepsPerRevolution <= 0 ||
        !std::isfinite(minimumTravelSteps) || minimumTravelSteps <= 0)
        return 0;

    const int64_t encoderTravel = encoderDelta(encoderEnd, encoderStart);
    const int64_t commandTravel = static_cast<int64_t>(commandEnd) -
                                  static_cast<int64_t>(commandStart);
    const double commandDistance = std::abs(static_cast<double>(commandTravel));
    const double encoderDistance = std::abs(static_cast<double>(encoderTravel));
    const double expectedEncoderDistance =
        commandDistance * encoderCountsPerRevolution / motorStepsPerRevolution;
    if (commandDistance < minimumTravelSteps ||
        encoderDistance < minimumTravelSteps * encoderCountsPerRevolution /
                              motorStepsPerRevolution ||
        encoderDistance < expectedEncoderDistance * 0.5 ||
        encoderDistance > expectedEncoderDistance * 1.5)
        return 0;

    return (encoderTravel > 0) == (commandTravel > 0) ? 1 : -1;
}

inline double nativePositionMm(int32_t position, int32_t homeEncoder,
                               int8_t encoderToNativeSign,
                               double millimetersPerRevolution) {
    return static_cast<double>(encoderDelta(position, homeEncoder)) *
           millimetersPerRevolution / encoderCountsPerRevolution *
           encoderToNativeSign;
}

inline double positionInFrameMm(double nativeMm, bool strokeEngineFrame,
                                double keepoutMm) {
    return strokeEngineFrame ? -(nativeMm + keepoutMm) : nativeMm;
}

// Sample times and sequences belong to the physical encoder. A cached reply
// cannot establish elapsed time. Encoder travel is relative to the first count
// in a window so a signed counter rollover preserves small movements.
class StationaryEncoderWindow {
  public:
    void reset() {
        samples_ = 0;
        settled_ = false;
    }

    bool update(uint32_t sequence, uint32_t sampleAtMs, int32_t encoderCounts,
                double maximumSpanCounts, uint32_t dwellMs) {
        if (sequence == 0 || !std::isfinite(maximumSpanCounts) || maximumSpanCounts <= 0) {
            reset();
            return false;
        }
        if (samples_ && sequence == lastSequence_) return settled_;
        if (!samples_ || static_cast<uint32_t>(sampleAtMs - lastSampleAtMs_) > 100) {
            begin(sequence, sampleAtMs, encoderCounts);
            return false;
        }
        const int64_t delta = encoderDelta(encoderCounts, originEncoder_);
        minimum_ = delta < minimum_ ? delta : minimum_;
        maximum_ = delta > maximum_ ? delta : maximum_;
        if (static_cast<double>(maximum_ - minimum_) >= maximumSpanCounts) {
            begin(sequence, sampleAtMs, encoderCounts);
            return false;
        }
        lastSequence_ = sequence;
        lastSampleAtMs_ = sampleAtMs;
        if (samples_ < 3) ++samples_;
        settled_ = samples_ >= 3 && static_cast<uint32_t>(sampleAtMs - startedAtMs_) >= dwellMs;
        return settled_;
    }

  private:
    void begin(uint32_t sequence, uint32_t sampleAtMs, int32_t encoderCounts) {
        samples_ = 1;
        settled_ = false;
        lastSequence_ = sequence;
        lastSampleAtMs_ = startedAtMs_ = sampleAtMs;
        originEncoder_ = encoderCounts;
        minimum_ = maximum_ = 0;
    }

    uint8_t samples_ = 0;
    bool settled_ = false;
    uint32_t lastSequence_ = 0;
    uint32_t lastSampleAtMs_ = 0;
    uint32_t startedAtMs_ = 0;
    int32_t originEncoder_ = 0;
    int64_t minimum_ = 0;
    int64_t maximum_ = 0;
};

// A high-current sample during acceleration is not an endpoint. Require a
// stationary encoder window while current remains above the contact threshold.
class HatContactDetector {
  public:
    void reset() { window_.reset(); }

    bool update(uint32_t sequence, uint32_t sampleAtMs,
                int32_t encoderCounts, float currentAmps,
                float thresholdAmps, double countsPerMm) {
        if (!std::isfinite(currentAmps) || !std::isfinite(thresholdAmps) ||
            thresholdAmps < 0 || currentAmps <= thresholdAmps) {
            reset();
            return false;
        }
        return window_.update(sequence, sampleAtMs, encoderCounts, countsPerMm * 0.25, 30);
    }

  private:
    StationaryEncoderWindow window_;
};

} // namespace motor_feedback_logic
