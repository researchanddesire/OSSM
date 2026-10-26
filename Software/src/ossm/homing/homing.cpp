#include "homing.h"

#include "Strings.h"
#include "homing_logic.h"
#include "motor_feedback_logic.h"
#include "constants/Config.h"
#include "constants/Pins.h"
#include "constants/UserConfig.h"
#include "ossm/Events.h"
#include "ossm/state/calibration.h"
#include "ossm/state/error.h"
#include "ossm/state/state.h"
#include "services/communication/mqtt.h"
#include "services/led.h"
#include "services/stepper.h"
#include "services/motorFeedback.h"
#include "services/ossmHat.h"
#include "services/tasks.h"
#include "utils/analog.h"

#include <esp_heap_caps.h>

namespace sml = boost::sml;
using namespace sml;

namespace homing {

namespace {

// The MQTT client holds a TLS session that competes with homing for internal
// RAM. It is paused for the whole homing run and resumed once the state
// machine has left homing by any route (done, error, emergency stop).
bool mqttPausedForHoming = false;
bool useHatForHoming = false;

void failHatHoming(const char *message) {
    stepper->forceStop();
    stepper->disableOutputs();
    calibration.isHomed = false;
    errorState.message = message;
    setHomingActive(false);
    stateMachine->process_event(Error{});
}

bool hatSampleReady(OssmHatTelemetry &sample) {
    return readOssmHatTelemetry(sample) &&
           static_cast<uint32_t>(millis() - sample.updatedAtMs) <= 100 &&
           sample.voltageVolts >= Config::Hat::minimumSupplyVolts &&
           (sample.alarm == 0 || sample.alarm == 0x10);
}

bool isInHomingState() {
    return stateMachine->is("homing"_s) ||
           stateMachine->is("homing.forward"_s) ||
           stateMachine->is("homing.backward"_s);
}

bool moveAndWait(int32_t target) {
    stepper->moveTo(target, false);
    while (stepper->isRunning()) {
        if (!isInHomingState()) {
            stepper->forceStop();
            stepper->disableOutputs();
            return false;
        }
        if (useHatForHoming) {
            OssmHatTelemetry sample;
            if (!hatSampleReady(sample)) {
                failHatHoming("OSSM HAT feedback lost. Check motor power and RS485.");
                return false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return true;
}

void pauseMqttForHoming() {
    if (mqttPausedForHoming || mqttClient == nullptr) return;
    esp_mqtt_client_stop(mqttClient);
    mqttPausedForHoming = true;
}

void resumeMqttAfterHoming() {
    if (!mqttPausedForHoming) return;
    mqttPausedForHoming = false;
    if (mqttClient != nullptr) esp_mqtt_client_start(mqttClient);
}

}  // namespace

void clearHoming() {
    ESP_LOGD("Homing", "Homing started");

    // Set homing active flag for LED indication
    setHomingActive(true);

    calibration.isForward = true;

    // A re-home can start after StrokeEngine changed the shared counter's
    // frame. Restore native coordinates before recording a feedback traverse.
    if (!stepper->isRunning()) stepperTranslateFrame(StepperFrame::Native);

    // Set acceleration and deceleration in steps/s^2
    stepper->setAcceleration(1000_mm);
    // Set speed in steps/s
    stepper->setSpeedInHz(25_mm);

    // Clear the stored values.
    calibration.measuredStrokeSteps = 0;

    // Recalibrate the current sensor offset.
    useHatForHoming = ossmHatDetected();
    if (!useHatForHoming) {
        calibration.currentSensorOffset = getAnalogAveragePercent(
            SampleOnPin{Pins::Driver::currentSensorPin, 1000});
    }
}

static void startHomingTask(void *pvParameters) {
    TickType_t xTaskStartTime = xTaskGetTickCount();

#ifdef AJ_DEVELOPMENT_HARDWARE
    stepper->setCurrentPosition(0);
    stepper->forceStopAndNewPosition(0);
    stepperFrame = StepperFrame::Native;  // counter re-zeroed at home rest
    stateMachine->process_event(Done{});
    vTaskDelete(nullptr);
    return;
#endif

    pauseMqttForHoming();

    if (useHatForHoming) {
        OssmHatTelemetry sample;
        // A previously detected HAT losing power/feedback must not silently
        // select the unrelated ADC threshold while the carriage is moving.
        if (!hatSampleReady(sample) || !beginMotorFeedbackHomingPass()) {
            failHatHoming("OSSM HAT feedback unavailable. Check motor power and RS485.");
            resumeMqttAfterHoming();
            vTaskDelete(nullptr);
            return;
        }
    }

    // Stroke Engine and Simple Penetration treat this differently.
    stepper->enableOutputs();
    stepper->setDirectionPin(Pins::Driver::motorDirectionPin, false);
    int16_t sign = stateMachine->is("homing.backward"_s) ? 1 : -1;

    int32_t targetPositionInSteps =
        round(sign * Config::Driver::maxStrokeSteps);

    ESP_LOGD("Homing", "Target position in steps: %d", targetPositionInSteps);
    stepper->moveTo(targetPositionInSteps, false);
    motor_feedback_logic::HatContactDetector hatContact;

    // run loop for 15second or until loop exits
    while (isInHomingState()) {
        TickType_t xCurrentTickCount = xTaskGetTickCount();
        // Calculate the time in ticks that the task has been running.
        TickType_t xTicksPassed = xCurrentTickCount - xTaskStartTime;

        // If you need the time in milliseconds, convert ticks to milliseconds.
        // 'portTICK_PERIOD_MS' is the number of milliseconds per tick.
        uint32_t msPassed = xTicksPassed * portTICK_PERIOD_MS;

        if (homing_logic::isHomingTimedOut(msPassed, 40000)) {
            ESP_LOGE("Homing", "Homing took too long. Check power and restart");
            errorState.message = ui::strings::homingTookTooLong;
            stepper->forceStop();
            stepper->disableOutputs();

            // Clear homing active flag for LED indication
            setHomingActive(false);

            stateMachine->process_event(Error{});
            break;
        }

        // measure the current analog value.
        float current;
        bool isCurrentOverLimit;
        if (useHatForHoming) {
            OssmHatTelemetry sample;
            if (!hatSampleReady(sample)) {
                failHatHoming("OSSM HAT feedback lost. Check motor power and RS485.");
                break;
            }
            current = sample.currentAmps;
            // Repeated reads of one cached reply are not independent samples.
            // Require sustained load with an encoder stall, not acceleration
            // current while the carriage is still travelling.
            isCurrentOverLimit = hatContact.update(
                sample.sequence, sample.updatedAtMs, sample.positionCounts,
                current, Config::Hat::sensorlessCurrentLimitAmps,
                32768.0 / (Config::Driver::pulleyToothCount *
                           Config::Driver::beltPitchMm));
        } else {
            current = getAnalogAveragePercent(
                          SampleOnPin{Pins::Driver::currentSensorPin, 50}) -
                      calibration.currentSensorOffset;
            isCurrentOverLimit = homing_logic::isCurrentOverLimit(
                current, 0, Config::Driver::sensorlessCurrentLimit);
        }

        ESP_LOGV("Homing", "Current: %f", current);
        if (!isCurrentOverLimit) {
            vTaskDelay(10);  // Increased from 1ms to 10ms to reduce CPU load
            continue;
        }

        ESP_LOGD("Homing", "Current over limit: %f", current);
        stepper->stopMove();

        stepper->setSpeedInHz(250_mm);
        // step away from the hard stop, with your hands in the air!
        int32_t currentPosition = stepper->getCurrentPosition();
        if (!moveAndWait(currentPosition - sign * Config::Driver::homingOffsetMn))
            break;

        // measure and save the current position
        calibration.measuredStrokeSteps =
            homing_logic::calculateMeasuredStroke(
                stepper->getCurrentPosition(),
                Config::Driver::maxStrokeSteps);

        if (useHatForHoming) finishMotorFeedbackHomingPass();

        stepper->setCurrentPosition(0);
        stepper->forceStopAndNewPosition(0);
        // The counter was just re-zeroed at the homed rest position: the
        // shared frame is Native again by definition.
        stepperFrame = StepperFrame::Native;

        if (useHatForHoming && !rebaseMotorFeedbackAtHome() &&
            stateMachine->is("homing.backward"_s)) {
            failHatHoming("OSSM HAT position reference failed. Re-home before use.");
            break;
        }

        int32_t goToPosition = homing_logic::calculatePostHomingPosition(
            sign, calibration.measuredStrokeSteps,
            UserConfig::afterHomingPosition);

        if (!moveAndWait(goToPosition)) break;

        // Clear homing active flag for LED indication
        setHomingActive(false);

        stateMachine->process_event(Done{});
        break;
    };

    // The forward pass hands over to the backward pass while still homing;
    // only the pass that leaves homing (or any error/stop exit) resumes MQTT.
    if (!isInHomingState()) {
        resumeMqttAfterHoming();
    }

    vTaskDelete(nullptr);
}

void startHoming() {
    int stackSize = 10 * configMINIMAL_STACK_SIZE;

    // The timeout, stall detection and Done event all live inside this task.
    // If it silently fails to start, the state machine waits in homing
    // forever, so retry briefly (internal RAM frees up as the radios settle)
    // and fall back to the existing homing error instead of hanging.
    BaseType_t created = pdFAIL;
    for (int attempt = 0; attempt < 20 && created != pdPASS; attempt++) {
        if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(250));
        created = xTaskCreatePinnedToCore(
            startHomingTask, "startHomingTask", stackSize, nullptr,
            configMAX_PRIORITIES - 1, &Tasks::runHomingTaskH,
            Tasks::operationTaskCore);
    }
    if (created == pdPASS) return;

    ESP_LOGE("Homing",
             "Could not create homing task: internal free=%u largest=%u",
             static_cast<unsigned>(heap_caps_get_free_size(
                 MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(
                 MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
    errorState.message = ui::strings::homingTookTooLong;
    setHomingActive(false);
    resumeMqttAfterHoming();
    stateMachine->process_event(Error{});
}

bool isStrokeTooShort() {
    if (calibration.measuredStrokeSteps > Config::Driver::minStrokeLengthMm) {
        return false;
    }
    errorState.message = ui::strings::strokeTooShort;
    return true;
}

}  // namespace homing
