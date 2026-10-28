#ifndef OSSM_SOFTWARE_BOARD_H
#define OSSM_SOFTWARE_BOARD_H

#include <Arduino.h>
#include <NimBLEDevice.h>

#include "constants/Pins.h"
#include "services/encoder.h"
#include "services/led.h"
#include "services/stepper.h"

extern bool USE_LATENCY_COMPENSATION;
extern bool USE_SPEED_KNOB_AS_LIMIT;
void initBoard();
// Speed knob position, 0..100 %. A board without a knob reads 0.
float readSpeedKnobPercent();

#endif  // OSSM_SOFTWARE_BOARD_H
