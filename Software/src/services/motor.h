#ifndef OSSM_SOFTWARE_MOTOR_H
#define OSSM_SOFTWARE_MOTOR_H

/**
 * What the firmware asks about the motor, however it is wired.
 *
 * Motion is always planned by FastAccelStepper (services/stepper.h). Without
 * an OSSM HAT the plan reaches the servo as STEP/DIR pulses, position is the
 * commanded step count and current comes from the board's analog sensor.
 * With a HAT (services/ossmHat.h) the plan reaches the servo over RS-485 and
 * these readings come from the servo itself.
 */
namespace motor {

// Detect the HAT once at boot. Call after initStepper().
void begin();
bool hasHat();
// True for expansion pins the HAT is using.
bool reservesPin(int pin);

// Carriage position in the current stepper frame: measured with a HAT,
// commanded without one.
float getPositionMm();

// Motor load for stall detection: servo amps with a HAT, otherwise the
// analog sensor's percent above its idle reading. NAN when unavailable.
float getCurrent();
// getCurrent() value that means the carriage is pushing a hard stop.
float getHomingCurrentLimit();
// Learn the idle reading (analog sensor only; the servo reports amps).
void zeroCurrent();

// Servo supply volts with a HAT, NAN without one.
float getVoltage();

}  // namespace motor

#endif  // OSSM_SOFTWARE_MOTOR_H
