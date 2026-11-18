#pragma once

#include "FastAccelStepper.h"
#include "MotionAxis.h"

MotionAxis *initializeMotorMotion(FastAccelStepper *pulseStepper);

// True once the serial backend was selected. Unlike motorMotionUsesSerial()
// this never selects it, so the safety observer may call it at any time.
bool motorMotionSerialLatched();

// Raw pulse engine for the ALT telemetry reader's PCNT observer.
FastAccelStepper *getMotorPulseStepper();
