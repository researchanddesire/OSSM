#ifndef SOFTWARE_STEPPER_H
#define SOFTWARE_STEPPER_H

#include "FastAccelStepper.h"
#include "MotionAxis.h"
#include "StrokeEngine.h"
#include "constants/Pins.h"

extern FastAccelStepperEngine stepperEngine;
extern MotionAxis *stepper;
extern class StrokeEngine Stroker;

// ── Shared-counter coordinate frame ─────────────────────────────────────
// One position counter is shared by every mode, expressed in one of two
// frames:
//  • Native — homing / SimplePenetration / Streaming: counter 0 at the
//    homed rest position, extension = negative counts, DIR polarity normal.
//  • StrokeEngine — counter = extension − keepout (home rest reads
//    −keepout), extension = positive counts, DIR polarity inverted
//    (StrokeEngine::begin applies it, thisIsHome anchors it).
// The frames are exact mirror images: c_other = −(c + keepout_steps), the
// same formula in both directions. Homing re-establishes Native when it
// zeroes the counter; every mode entry must ensure the counter is in ITS
// frame before commanding motion.
enum class StepperFrame { Native, StrokeEngine };
extern StepperFrame stepperFrame;

// Translate the shared counter into `to` (exact math, no motion). No-op
// when already in that frame. Call only with the motor stopped.
void stepperTranslateFrame(StepperFrame to);

void initStepper();
bool enableMotorOutputs();

// Unset callbacks keep serial motion fail-closed. Serial output/mode changes
// remain owned by the safety service and the sole UART task.
void registerMotorSerialMotionAuthorization(uint64_t (*session)(),
                                           bool (*canMove)(uint64_t));
bool motorMotionUsesSerial();
int32_t motorMotionCommandedTarget();
bool mirrorMotorMotionFrame(int32_t keepoutSteps);

struct MotorMotionStatus {
    bool serial = false;
    bool frameReady = false;
    bool moving = false;
    bool pending = false;
    uint32_t requestedStepsPerSecond = 0;
    uint32_t requestedAcceleration = 0;
    uint16_t maximumSpeedRpm = 0;
    uint16_t accelerationRpmPerSecond = 0;
    int32_t commandTarget = 0;
    float followingLimitMm = 1.0f;
};
MotorMotionStatus getMotorMotionStatus();

#endif  // SOFTWARE_STEPPER_H
