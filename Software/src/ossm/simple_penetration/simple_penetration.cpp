#include "simple_penetration.h"
#include <new>

#include "simple_pen_logic.h"
#include "constants/Config.h"
#include "ossm/state/calibration.h"
#include "ossm/state/session.h"
#include "ossm/state/settings.h"
#include "ossm/state/state.h"
#include "services/communication/nimble.h"
#include "services/communication/queue.h"
#include "services/stepper.h"
#include "services/motorSafety.h"
#include "services/tasks.h"

namespace sml = boost::sml;
using namespace sml;

namespace simple_penetration {

static void startSimplePenetrationTask(void *pvParameters) {
    const uint64_t safetyRevision = *static_cast<uint64_t *>(pvParameters);
    delete static_cast<uint64_t *>(pvParameters);
    if (!motorSafetyEventCurrent(safetyRevision)) { vTaskDelete(nullptr); return; }
    if (!motorSafetyBeginNormalOperation(safetyRevision)) {
        stateMachine->process_event(MotorSafetyFault{safetyRevision});
        vTaskDelete(nullptr); return;
    }
    // Own the shared stepper config at mode entry: a preceding StrokeEngine
    // session leaves the shared DIR polarity inverted (StrokeEngine::begin)
    // and the counter in the StrokeEngine frame; only homing resets them.
    // This mode's targets are native-frame absolutes with normal polarity,
    // so reconcile both explicitly (exact math, no motion) — otherwise every
    // stroke runs physically reversed and/or displaced into a hard stop.
    stepperTranslateFrame(StepperFrame::Native);
    stepper->setDirectionPin(Pins::Driver::motorDirectionPin, false);
    if (!motorSafetyEventCurrent(safetyRevision) || !enableMotorOutputs()) {
        stateMachine->process_event(MotorSafetyFault{safetyRevision});
        vTaskDelete(nullptr);
        return;
    }

    int fullStrokeCount = 0;
    static int32_t targetPosition = 0;

    auto isInCorrectState = [safetyRevision]() {
        if (!motorSafetyEventCurrent(safetyRevision)) return false;
        // Add any states that you want to support here.
        return stateMachine->is("simplePenetration"_s) ||
               stateMachine->is("simplePenetration.idle"_s);
    };

    double lastSpeed = 0;

    bool stopped = false;

    while (isInCorrectState()) {
        if (!motorSafetyCanEnableOutputs()) break;
        auto speed = simple_pen_logic::calculateSpeed(
            settings.speed, Config::Driver::maxSpeedMmPerSecond, (1_mm));
        auto acceleration = simple_pen_logic::calculateAcceleration(
            settings.speed, Config::Driver::maxSpeedMmPerSecond,
            Config::Advanced::accelerationScaling, (1_mm));

        // Use the effective speed after PlayControls has applied either the
        // physical knob or the BLE override. Checking the raw knob here makes
        // a valid BLE speed command unable to start motion when the knob is at
        // zero, even when speedKnobAsLimit is disabled.
        bool isSpeedZero = simple_pen_logic::isInDeadZone(
            settings.speed, Config::Advanced::commandDeadZonePercentage);
        bool isSpeedChanged =
            !isSpeedZero && simple_pen_logic::isSpeedChangeSignificant(
                                lastSpeed, speed,
                                Config::Advanced::commandDeadZonePercentage);
        bool isAtTarget =
            abs(targetPosition - stepper->getCurrentPosition()) == 0;

        // If the speed is zero, then stop the stepper and wait for the next
        if (isSpeedZero) {
            stepper->stopMoveForOwner(safetyRevision, motorSafetyEventCurrent);
            stopped = true;
            vTaskDelay(100);
            continue;
        } else if (stopped) {
            stepper->moveToForOwner(targetPosition, safetyRevision, motorSafetyEventCurrent, false);
            stopped = false;
        }

        // If the speed is greater than the dead-zone, and the speed has changed
        // by more than the dead-zone, then update the stepper.
        // This must be done in the same task that the stepper is running in.
        if (isSpeedChanged) {
            lastSpeed = speed;
            stepper->setAcceleration(acceleration);
            stepper->setSpeedInHz(speed);
        }

        // If the stepper is not at the target, then wait for the next loop
        if (!isAtTarget) {
            vTaskDelay(1);
            // more than zero
            continue;
        }

        bool nextDirection = !calibration.isForward;
        calibration.isForward = nextDirection;

        targetPosition = simple_pen_logic::calculateTarget(
            calibration.isForward, settings.stroke,
            calibration.measuredStrokeSteps);

        ESP_LOGV("SimplePenetration", "target: %f,\tspeed: %f,\tacc: %f",
                 targetPosition, speed, acceleration);

        stepper->moveToForOwner(targetPosition, safetyRevision, motorSafetyEventCurrent, false);

        if (settings.speed > Config::Advanced::commandDeadZonePercentage &&
            settings.stroke >
                (long)Config::Advanced::commandDeadZonePercentage) {
            fullStrokeCount++;
            session.strokeCount = floor(fullStrokeCount / 2);

            // This calculation assumes that at the end of every stroke you have
            // a whole positive distance, equal to maximum target position.
            session.distanceMeters +=
                (((float)settings.stroke / 100.0) *
                 calibration.measuredStrokeSteps / (1_mm)) /
                1000.0;
        }

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}

void startSimplePenetration() {
    int stackSize = 10 * configMINIMAL_STACK_SIZE;
    auto *revision = new (std::nothrow) uint64_t(getMotorSafetyStatus().controlRevision);
    if (!revision) return;
    if (xTaskCreatePinnedToCore(startSimplePenetrationTask,
                            "startSimplePenetrationTask", stackSize, revision,
                            configMAX_PRIORITIES - 1,
                            &Tasks::runSimplePenetrationTaskH,
                            Tasks::operationTaskCore) != pdPASS) delete revision;
}

}  // namespace simple_penetration
