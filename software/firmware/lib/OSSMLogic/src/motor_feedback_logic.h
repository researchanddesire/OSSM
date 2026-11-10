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

// A high-current sample during acceleration is not an endpoint. Require a
// short stationary dwell measured by independent, fresh encoder samples.
// The caller must check sample age before update; timestamps here belong to
// the physical samples, so repeated reads of a cached sample cannot add dwell.
class HatContactDetector {
  public:
    void reset() {
        candidate_ = false;
        contact_ = false;
        samples_ = 0;
        lastSequence_ = 0;
    }

    bool update(uint32_t sequence, uint32_t sampleAtMs,
                int32_t encoderCounts, float currentAmps,
                float thresholdAmps, double countsPerMm) {
        if (sequence == 0 || !std::isfinite(currentAmps) ||
            !std::isfinite(thresholdAmps) || thresholdAmps < 0 ||
            !std::isfinite(countsPerMm) || countsPerMm <= 0 ||
            currentAmps <= thresholdAmps) {
            reset();
            return false;
        }
        if (sequence == lastSequence_) return contact_;

        if (!candidate_ ||
            static_cast<uint32_t>(sampleAtMs - lastSampleAtMs_) > 100) {
            begin(sequence, sampleAtMs, encoderCounts);
            return false;
        }

        const int64_t position = encoderDelta(encoderCounts, originEncoder_);
        const int64_t newMinimum = position < minimum_ ? position : minimum_;
        const int64_t newMaximum = position > maximum_ ? position : maximum_;
        if (static_cast<double>(newMaximum - newMinimum) >=
            countsPerMm * 0.25) {
            // This sample is moving. It can be the first observation of a
            // subsequent stationary dwell, but never completes this one.
            begin(sequence, sampleAtMs, encoderCounts);
            return false;
        }

        minimum_ = newMinimum;
        maximum_ = newMaximum;
        lastSequence_ = sequence;
        lastSampleAtMs_ = sampleAtMs;
        if (samples_ < 3) ++samples_;
        contact_ = samples_ >= 3 &&
                   static_cast<uint32_t>(sampleAtMs - startedAtMs_) >= 30;
        return contact_;
    }

  private:
    void begin(uint32_t sequence, uint32_t sampleAtMs,
               int32_t encoderCounts) {
        candidate_ = true;
        contact_ = false;
        samples_ = 1;
        lastSequence_ = sequence;
        startedAtMs_ = sampleAtMs;
        lastSampleAtMs_ = sampleAtMs;
        originEncoder_ = encoderCounts;
        minimum_ = 0;
        maximum_ = 0;
    }

    bool candidate_ = false;
    bool contact_ = false;
    uint8_t samples_ = 0;
    uint32_t lastSequence_ = 0;
    uint32_t startedAtMs_ = 0;
    uint32_t lastSampleAtMs_ = 0;
    int32_t originEncoder_ = 0;
    int64_t minimum_ = 0;
    int64_t maximum_ = 0;
};

}  // namespace motor_feedback_logic
