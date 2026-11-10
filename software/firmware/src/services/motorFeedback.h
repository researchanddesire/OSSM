#pragma once

#include "homing_session.h"

// Position follows the current shared stepper coordinate frame. Fresh HAT
// encoder feedback is used after a physical homing traverse establishes its
// polarity and home reference; otherwise this is the command-counter estimate.
float getMotorPositionMm();
bool motorPositionUsesRs485();

// These return false when fresh physical measurements are unavailable. They
// never relabel the legacy ADC percentage/raw values as amps or volts.
bool readMotorCurrentAmps(float &amps);
bool readMotorVoltageVolts(float &volts);

// Call begin/end with the motor stopped, while the counter is in Native frame.
// Finish must run before resetting the stepper's counter at the end of a pass.
// A short first pass may not establish polarity; the full second pass can.
bool beginMotorFeedbackHomingPass(homing_session::Ticket ticket);
bool finishMotorFeedbackHomingPass(homing_session::Ticket ticket);

// Call after the stopped native counter has been reset to zero at home. Waits
// up to 300 ms for a new physical sample, without changing any motor register.
// A missing/stale sample invalidates the reference until another valid home.
bool rebaseMotorFeedbackAtHome(homing_session::Ticket ticket);
