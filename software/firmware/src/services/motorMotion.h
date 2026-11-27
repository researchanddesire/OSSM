#pragma once

#include "FastAccelStepper.h"
#include "MotionAxis.h"
#include "drawn_pulse_queue.h"

MotionAxis *initializeMotorMotion(FastAccelStepper *pulseStepper);

// True once the serial backend was selected. Unlike motorMotionUsesSerial()
// this never selects it, so the safety observer may call it at any time.
bool motorMotionSerialLatched();

// Raw pulse engine for the ALT telemetry reader's PCNT observer.
FastAccelStepper *getMotorPulseStepper();

// Timed, already acceleration-limited points from an owned pattern curve.
// Retry never advances the player's phase. The serial backend enforces
// feedback, following distance, verified drive limits and absolute bounds.
drawn_pattern::FrameResult submitPatternFrame(int32_t target, uint64_t revision, bool serial, uint16_t crossing);
drawn_pattern::Park parkPatternMotion(uint64_t revision, int32_t target, bool draining);
void stopPatternMotion(uint64_t revision);
