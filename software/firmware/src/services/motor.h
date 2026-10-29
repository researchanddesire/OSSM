#ifndef OSSM_SOFTWARE_MOTOR_H
#define OSSM_SOFTWARE_MOTOR_H

/**
 * What the firmware asks about the motor, however it is wired.
 *
 * The classic ESP32 HAT uses guarded RS-485 absolute targets; the ALT board
 * and boards without a HAT keep their STEP/DIR planner. A HAT supplies
 * measured motor readings, while the legacy board uses its pulse count and
 * analog current sensor.
 */
namespace motor {

// Detect the HAT once at boot. Call after initStepper().
void begin();
// The readings below come from the servo. Classic ESP32 HAT selection is
// latched: stale telemetry never silently selects board sensors. ALT retains
// its existing telemetry-only behavior and hands readings back after loss.
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

// Carriage speed the servo measures, mm/s in the current stepper frame, with
// a HAT; NAN without one.
float getSpeedMmPerSec();
// Servo temperature in degrees C, NAN without a HAT.
float getTemperatureC();
// Servo drive output, -100..100 %, NAN without a HAT.
float getOutputPercent();
// Servo alarm code (0 = none; YZ-AIM manual alarm table), -1 without a HAT.
int getAlarm();

}  // namespace motor

#endif  // OSSM_SOFTWARE_MOTOR_H
