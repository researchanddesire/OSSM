#ifndef OSSM_SOFTWARE_OSSM_HAT_H
#define OSSM_SOFTWARE_OSSM_HAT_H

#include <cstdint>

/**
 * OSSM HAT: an RS-485 transceiver on the V2.x expansion header that talks to
 * the 57AIM servo (Alex Warner's design; framing in lib/AimFrame).
 *
 * Motion reaches the servo as STEP/DIR pulses with or without a HAT. The HAT
 * only reads the servo back (encoder position, current, voltage and alarm)
 * and never writes to it. The ESP32 pulse counter counts the same STEP
 * pulses so the encoder can be compared with the commanded position.
 */
namespace ossm_hat {

struct Telemetry {
    int32_t positionCounts = 0;  // servo encoder, 32768 counts per revolution
    float currentAmps = 0;
    float voltage = 0;
    uint16_t alarm = 0;
    uint32_t sampledMs = 0;      // millis() of the last telemetry reply
};

// Look for the servo and start reading. Returns true when the HAT answered.
bool begin();
// The HAT answered at boot; it keeps its expansion pins until reboot.
bool isPresent();
// Readings are live. Turns false for good once the servo stops answering.
bool isAnswering();
Telemetry telemetry();
// Measured minus commanded position, in STEP pulses counted the way the
// native stepper frame counts (see services/stepper.h).
int32_t followingErrorPulses();

}  // namespace ossm_hat

#endif  // OSSM_SOFTWARE_OSSM_HAT_H
