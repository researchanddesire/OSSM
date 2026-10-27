#ifndef OSSM_SOFTWARE_OSSM_HAT_H
#define OSSM_SOFTWARE_OSSM_HAT_H

#include <cstdint>

/**
 * OSSM HAT: an RS-485 transceiver on the V2.x expansion header that talks to
 * the 57AIM servo (Alex Warner's design; framing in lib/AimFrame).
 *
 * When the servo answers at boot, the HAT replaces the STEP/DIR wires. A
 * follower task reads the pulses the firmware already generates (counted in
 * ESP32 pulse-counter hardware) and the ENA pin, and sends the servo the same
 * position as an absolute RS-485 target. Every motion mode runs unchanged;
 * only the wire differs. The same task reads back the servo's encoder,
 * current, voltage and alarm.
 */
namespace ossm_hat {

struct Telemetry {
    int32_t positionCounts = 0;  // servo encoder, 32768 counts per revolution
    int32_t targetCounts = 0;    // last position target sent
    float currentAmps = 0;
    float voltage = 0;
    uint16_t alarm = 0;
    uint32_t sampledMs = 0;      // millis() of the last telemetry reply
};

// Look for the servo and start following. Returns true when the HAT answered.
bool begin();
bool isPresent();
Telemetry telemetry();
// Measured minus commanded position, in STEP pulses counted the way the
// native stepper frame counts (see services/stepper.h).
int32_t followingErrorPulses();

}  // namespace ossm_hat

#endif  // OSSM_SOFTWARE_OSSM_HAT_H
