#include "stepper.h"

#include "constants/Config.h"
#include "services/motorSafety.h"
#include "services/motorMotion.h"

FastAccelStepperEngine stepperEngine = FastAccelStepperEngine();
MotionAxis *stepper = nullptr;
class StrokeEngine Stroker;
StepperFrame stepperFrame = StepperFrame::Native;

void stepperTranslateFrame(StepperFrame to) {
    if (stepperFrame == to) return;
    if (stepper == nullptr) return;
    // Mirror map between the two frames. The keepout must match
    // strokingMachine.keepoutBoundary (6 mm) in stroke_engine.cpp.
    constexpr int32_t keepoutSteps =
        static_cast<int32_t>(6.0f * Config::Driver::stepsPerMM);
    if (motorMotionUsesSerial()) {
        if (mirrorMotorMotionFrame(keepoutSteps)) stepperFrame = to;
        return;
    }
    stepper->setCurrentPosition(
        -(stepper->getCurrentPosition() + keepoutSteps));
    stepperFrame = to;
}

void initStepper() {
    stepperEngine.init();
    auto *pulseStepper = stepperEngine.stepperConnectToPin(Pins::Driver::motorStepPin);
    stepper = pulseStepper == nullptr ? nullptr : initializeMotorMotion(pulseStepper);
    if (stepper) {
        stepper->setDirectionPin(Pins::Driver::motorDirectionPin, false);
#ifndef OSSM_ALT
        stepper->setEnablePin(Pins::Driver::motorEnablePin, true);
#endif
        stepper->setAutoEnable(false);
    }

#ifndef OSSM_ALT
    // Classic V2 has an ENA output; ALT has no physical enable wire.
    digitalWrite(Pins::Driver::motorEnablePin, HIGH);
    delay(600);
#endif
    // The motor stays disabled until the appropriate motion mode has passed
    // its safety gate. A reboot must not energize it during HAT discovery.
    Stroker.setOutputGuard(motorSafetyCanEnableOutputs, enableMotorOutputs);
}

bool enableMotorOutputs() {
    return enableMotorSafetyOutputs();
}
