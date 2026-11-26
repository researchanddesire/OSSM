#pragma once

#include <cmath>
#include <cstdint>
#include <algorithm>

// Pure logic extracted from src/ossm/homing/homing.cpp.
// No hardware dependencies — testable on native platform.

namespace homing_logic {

/// Check if measured current exceeds the sensorless homing threshold.
/// homing.cpp lines 98-99
inline bool isCurrentOverLimit(float currentReading, float offset,
                               float threshold) {
    return (currentReading - offset) > threshold;
}

enum class SeekResult { Seeking, Contact, TravelEnded, TimedOut };

// Legacy ADC homing only recognizes contact during a live seek. Current that
// rises after the last step cannot establish an end-stop position.
inline SeekResult evaluateSensorlessSeek(bool timedOut, bool running,
                                         float currentRise, float threshold) {
    if (timedOut) return SeekResult::TimedOut;
    if (!running) return SeekResult::TravelEnded;
    if (isCurrentOverLimit(currentRise, 0, threshold)) return SeekResult::Contact;
    return SeekResult::Seeking;
}

/// Calculate the measured stroke from the stepper's current position,
/// clamped to the maximum physical stroke.
/// homing.cpp lines 116-118
inline float calculateMeasuredStroke(int32_t currentPosition,
                                     float maxStrokeSteps) {
    return std::min(static_cast<float>(std::abs(currentPosition)),
                    maxStrokeSteps);
}

// The first pass returns along its measured travel to set up the second
// seek. After the final pass, zero is the retracted, backed-off reference.
// Negative logical positions extend from that reference into calibrated travel.
inline int32_t calculatePostHomingPosition(int16_t sign, float measuredStrokeSteps,
                                           float extensionMm, float stepsPerMm) {
    if (!std::isfinite(measuredStrokeSteps) || measuredStrokeSteps <= 0) return 0;
    if (sign < 0) return static_cast<int32_t>(measuredStrokeSteps);
    if (!std::isfinite(extensionMm) || extensionMm <= 0 ||
        !std::isfinite(stepsPerMm) || stepsPerMm <= 0) return 0;
    const float extensionSteps = std::min(measuredStrokeSteps, extensionMm * stepsPerMm);
    return -static_cast<int32_t>(extensionSteps);
}

/// Check if homing has exceeded the timeout.
/// homing.cpp line 81
inline bool isHomingTimedOut(uint32_t elapsedMs, uint32_t timeoutMs) {
    return elapsedMs > timeoutMs;
}

/// Check if the measured stroke is too short for safe operation.
/// homing.cpp lines 147-148 (homing::isStrokeTooShort)
inline bool isStrokeTooShortLogic(float measuredStrokeSteps,
                                  float minStrokeLengthMm) {
    return measuredStrokeSteps <= minStrokeLengthMm;
}

}  // namespace homing_logic
