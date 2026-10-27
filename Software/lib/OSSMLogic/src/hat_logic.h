#pragma once

#include <cmath>
#include <cstdint>

// Pure math for the OSSM HAT reader in src/services/ossmHat.cpp.
// No hardware dependencies — testable on native platform.

namespace hat_logic {

/// Extend a free-running 16-bit hardware pulse counter into a 32-bit total.
/// Valid while fewer than 32768 pulses happen between two reads.
inline int32_t accumulatePulses(int32_t total, int16_t previous, int16_t now) {
    const auto delta = static_cast<int16_t>(static_cast<uint16_t>(now) -
                                            static_cast<uint16_t>(previous));
    return total + delta;
}

/// Servo encoder counts for a number of STEP pulses.
inline int32_t pulsesToCounts(int32_t pulses, double countsPerPulse) {
    return static_cast<int32_t>(std::lround(pulses * countsPerPulse));
}

/// STEP pulses for a number of servo encoder counts.
inline int32_t countsToPulses(int32_t counts, double countsPerPulse) {
    return static_cast<int32_t>(std::lround(counts / countsPerPulse));
}

/// One full telemetry read of the 57AIM: registers 0x0E..0x17 (YZ-AIM manual
/// section V.2).
struct ServoReading {
    uint16_t alarm = 0;            // 0 = none
    float currentAmps = 0;
    float speedRpm = 0;            // positive counts the encoder up
    float voltage = 0;
    float temperatureC = 0;
    float outputPercent = 0;       // drive output, -100..100
    int32_t positionCounts = 0;    // encoder, 32768 counts per revolution
};

inline ServoReading decodeTelemetry(const uint16_t r[10]) {
    ServoReading s;
    s.alarm = r[0];
    s.currentAmps = r[1] / 2000.0f;
    s.speedRpm = static_cast<int16_t>(r[2]) / 10.0f;
    s.voltage = r[3] / 327.0f;
    s.temperatureC = r[4];
    s.outputPercent = static_cast<int16_t>(r[5]) * 100.0f / 32768.0f;
    // r[6] save flag and r[7] address are settings, not readings.
    s.positionCounts =
        static_cast<int32_t>((static_cast<uint32_t>(r[9]) << 16) | r[8]);
    return s;
}

/// Anchor that maps the current pulse total onto the measured position: a
/// servo coming out of ENA holds where it already is.
inline int32_t anchorAt(int32_t measuredCounts, int32_t pulses,
                        double countsPerPulse) {
    return measuredCounts - pulsesToCounts(pulses, countsPerPulse);
}

/// How far the servo trails the pulses, in pulses.
inline int32_t followingErrorPulses(int32_t measuredCounts,
                                    int32_t anchorCounts, int32_t pulses,
                                    double countsPerPulse) {
    return countsToPulses(measuredCounts - anchorCounts, countsPerPulse) -
           pulses;
}

}  // namespace hat_logic
