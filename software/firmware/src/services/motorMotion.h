#pragma once

#include "FastAccelStepper.h"
#include "MotionAxis.h"

MotionAxis *initializeMotorMotion(FastAccelStepper *pulseStepper);

// Raw pulse engine for the ALT telemetry reader's PCNT observer.
FastAccelStepper *getMotorPulseStepper();
