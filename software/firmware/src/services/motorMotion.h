#pragma once

#include "FastAccelStepper.h"
#include "MotionAxis.h"

MotionAxis *initializeMotorMotion(FastAccelStepper *pulseStepper);

// True once the serial backend was selected. Unlike motorMotionUsesSerial()
// this never selects it, so the safety observer may call it at any time.
bool motorMotionSerialLatched();

// Raw pulse engine for the ALT telemetry reader's PCNT observer.
FastAccelStepper *getMotorPulseStepper();

// Timed, already acceleration-limited points from an owned pattern curve.
// Retry never advances the player's phase. The serial backend enforces
// feedback, following distance, verified drive limits and absolute bounds.
enum class PatternFrameResult { Accepted, Retry, Rejected };
PatternFrameResult submitPatternFrame(int32_t target, uint64_t revision, bool serial);
void stopPatternMotion(uint64_t revision);
