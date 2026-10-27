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
