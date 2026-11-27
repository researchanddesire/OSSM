#include "drawn_pattern_motor.h"
#include <cmath>

#include "constants/Config.h"
#include "ossm/state/calibration.h"
#include "ossm/play_controls/play_controls.h"
#include "ossm/state/settings.h"
#include "services/communication/queue.h"
#include "services/motorMotion.h"
#include "services/ossmHat.h"
#include "services/motorSafety.h"
#include "services/stepper.h"
#include "streaming_logic.h"

namespace streaming::pattern_motor {
uint32_t now() { return millis(); }
bool current(uint64_t revision) { return motorSafetyEventCurrent(revision); }

static bool withinPercent(float value) {
    return std::isfinite(value) && value >= 0 && value <= 100;
}

void setSensation(uint8_t value) { play_controls::setRemoteControl(PlayControls::SENSATION, value); }

State read(uint64_t revision, bool annotated) {
    State state;
    if (stepper == nullptr || !calibration.isHomed || stepperFrame != StepperFrame::Native ||
        !current(revision) || !motorSafetyCanEnableOutputs()) return state;
    const auto s = settings.speed, a = settings.sensation;
    const auto stroke = settings.stroke, depth = settings.depth;
    const auto rail = calibration.measuredStrokeSteps;
    if (!withinPercent(s) || !withinPercent(a) || !withinPercent(stroke) || !withinPercent(depth) ||
        !std::isfinite(rail) || rail <= 0 || rail > Config::Driver::maxStrokeSteps) return state;
    const auto span = streaming_logic::calculateMaxStroke(stroke, depth, rail);
    const auto offset = streaming_logic::calculateDepthOffset(rail, span, depth);
    state.from = -offset - span;
    state.to = -offset;
    state.speed = Config::Driver::maxSpeedMmPerSecond * Config::Driver::stepsPerMM * s / 100.0;
    state.strength = a / 100.0;
    state.stepsPerMm = Config::Driver::stepsPerMM;
    state.acceleration =
        Config::Driver::maxAcceleration * Config::Driver::stepsPerMM * (annotated ? .5 : state.strength);
    state.serial = motorMotionUsesSerial();
    if (state.serial) {
        const auto caps = getOssmHatPositionStatus();
        const double stepsPerRpm = Config::Driver::motorStepPerRevolution / 60.0;
        state.speed = std::min(state.speed, caps.maximumSpeedRpm * stepsPerRpm);
        state.acceleration = std::min(state.acceleration, caps.accelerationRpmPerSec * stepsPerRpm);
    }
    state.moving = stepper->isRunning();
    state.position = stepper->getCurrentPosition();
    state.queued = !targetQueue.empty();
    state.ready = true;
    return state;
}

bool position(uint64_t revision, int32_t target, const State &caps, bool starting) {
    return runMotorPatternCommand(revision, [&] {
        stepper->setSpeedInHz(static_cast<uint32_t>(caps.speed));
        stepper->setAcceleration(static_cast<int32_t>(caps.acceleration));
        if (starting) return stepper->moveToForOwner(target, revision, motorSafetyEventCurrent, false) == 0;
        stepper->applySpeedAcceleration();
        return true;
    });
}
FrameResult submit(int32_t target, uint64_t revision, bool serial, uint16_t crossing) {
    FrameResult result{Frame::Rejected, "guard"};
    if (!runMotorPatternCommand(revision, [&] {
        result = submitPatternFrame(target, revision, serial, crossing);
        return true;
    })) return {Frame::Rejected, "guard"};
    return result;
}
void stop(uint64_t revision) {
    stopPatternMotion(revision);
}
}
