#pragma once

#include <cmath>
#include <cstdint>
#include <algorithm>

// Shared by the firmware and native streaming tests.

namespace streaming_logic {

/// Compute max stroke in steps from user percentages and calibration.
inline int32_t calculateMaxStroke(float strokePct, float depthPct,
                                  float measuredStrokeSteps) {
    return std::abs(
        (strokePct / 100.0f) * (depthPct / 100.0f) * measuredStrokeSteps);
}

/// Compute depth offset in steps.
inline int32_t calculateDepthOffset(float measuredStrokeSteps,
                                    int32_t maxStroke, float depthPct) {
    return measuredStrokeSteps * (depthPct / 100.0f) - maxStroke;
}

/// Scale a BLE position percentage (0-100) into a native-frame stepper target
/// (services/stepper.h: extension is negative counts).
///
inline int32_t scaleStreamPosition(int posPercent, int32_t maxStroke,
                                   int32_t depth) {
    return -(1 - (static_cast<float>(posPercent) / 100.0f)) * maxStroke - depth;
}

/// Result of motion profile planning.
struct MotionProfile {
    uint32_t speed;
    uint32_t acceleration;
    int32_t distance;         // adjusted if physically impossible
    int32_t targetPosition;   // adjusted if distance was clamped
};

/// Plan a triangular/trapezoidal motion profile for a streaming move.
///
/// Given a distance to travel, available time, and machine limits,
/// computes the required speed and acceleration for a smooth move.
/// If the requested distance exceeds what's physically possible,
/// it clamps the distance and adjusts the target position.
inline MotionProfile planMotion(int32_t currentPosition,
                                int32_t targetPosition, float timeSeconds,
                                uint32_t maxSpeed, uint32_t maxAccel,
                                int32_t stepsPerMm = 1) {
    const int32_t requestedDistance = std::abs(targetPosition - currentPosition);
    if (requestedDistance <= 1 || timeSeconds <= 0.01f || maxSpeed == 0 || maxAccel == 0) {
        return {0, 0, 0, currentPosition};
    }

    // Clamp distance to what's physically achievable
    const float halfTime = timeSeconds / 2.0f;
    const int32_t maxDistance = std::min(maxAccel * halfTime * halfTime,
                                        maxSpeed * timeSeconds);
    int32_t distance = requestedDistance;
    if (requestedDistance > maxDistance) {
        // Leave the existing 2 mm margin on shortened moves. A move that
        // cannot cover the margin holds its position.
        distance = maxDistance - 2 * stepsPerMm;
        if (distance <= 1) return {0, 0, 0, currentPosition};
        targetPosition = currentPosition + (targetPosition > currentPosition ? distance : -distance);
    }

    // Triangular profile: speed = 2 * distance / time
    uint32_t requiredSpeed = (2.0f * distance) / timeSeconds;
    requiredSpeed = std::max(requiredSpeed, 100u);
    requiredSpeed = std::min(requiredSpeed, maxSpeed);

    // Proportion of move at max speed (trapezoid vs triangle)
    float vt = requiredSpeed * timeSeconds;
    float proportion = std::max(-((2.0f * distance - 2.0f * vt) / vt), 0.01f);

    // Acceleration to create the triangle/trapezoid
    uint32_t requiredAccel =
        requiredSpeed / (timeSeconds * proportion / 2.0f);
    requiredAccel = std::max(requiredAccel, 100u);
    requiredAccel = std::min(requiredAccel, maxAccel);

    return {requiredSpeed, requiredAccel, distance, targetPosition};
}

}  // namespace streaming_logic
