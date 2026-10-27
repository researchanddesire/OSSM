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
#include "services/motorSafety.h"
#include "services/tasks.h"
#include "utils/analog.h"

#include <esp_heap_caps.h>
#include <new>
#include <atomic>

namespace sml = boost::sml;
using namespace sml;

namespace homing {

namespace {

struct HomingTaskContext {
    MotorHomingTicket ticket;
    uint8_t pass;
    bool useHat;
};

bool mqttPausedForHoming = false;
SemaphoreHandle_t mqttMutex = nullptr;
std::atomic_bool taskCreationFailed{false};

void failHatHoming(const HomingTaskContext &context, const char *message) {
    if (!commitMotorHomingState(context.ticket, [&] {
        digitalWrite(Pins::Driver::motorEnablePin, HIGH);
        calibration.isHomed = false;
        setHomingActive(false, false);
        return true;
    })) return;
    if (!runMotorHomingCommand(context.ticket, [] {
        stepper->forceStop();
        stepper->disableOutputs();
        digitalWrite(Pins::Driver::motorEnablePin, HIGH);
        return true;
    })) return;
    // SML validates the same ticket again while processing the event. Cleanup
    // from an obsolete task cannot fail or stop a newer home.
    stateMachine->process_event(HomingFailed{context.ticket, message});
}

bool hatSampleReady(OssmHatTelemetry &sample) {
    return readOssmHatTelemetry(sample) &&
           static_cast<uint32_t>(millis() - sample.updatedAtMs) <= 100 &&
           sample.voltageVolts >= Config::Hat::minimumSupplyVolts &&
           (sample.alarm == 0 || sample.alarm == 0x10);
}

bool submitMove(const HomingTaskContext &context, int32_t target, bool markMotion) {
    return runMotorHomingCommand(context.ticket, [&] {
        if (markMotion && !motorSafetyBeginHomingMotion(context.ticket)) return false;
        return stepper->moveTo(target, false) == 0;
    });
}

bool moveAndWait(const HomingTaskContext &context, int32_t target, bool markMotion = false) {
    if (!submitMove(context, target, markMotion)) return false;
    while (stepper->isRunning()) {
        if (!motorHomingTicketCurrent(context.ticket)) return false;
        if (context.useHat) {
            OssmHatTelemetry sample;
            if (!hatSampleReady(sample) || !motorSafetyCanEnableOutputs()) {
                failHatHoming(context, "OSSM HAT feedback lost. Check motor power and RS485.");
                return false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return motorHomingTicketCurrent(context.ticket);
}

bool pauseMqttForHoming(MotorHomingTicket ticket) {
    if (mqttMutex == nullptr) return false;
    while (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(20)) != pdTRUE)
        if (!motorHomingTicketCurrent(ticket)) return false;
    if (!motorHomingTicketCurrent(ticket)) {
        xSemaphoreGive(mqttMutex);
        return false;
    }
    if (!mqttPausedForHoming && mqttClient != nullptr) {
        esp_mqtt_client_stop(mqttClient);  // may block; never owns a motion lock
        mqttPausedForHoming = true;
    }
    xSemaphoreGive(mqttMutex);
    return motorHomingTicketCurrent(ticket);
}

void resumeMqttAfterHoming() {
    if (mqttMutex == nullptr) return;
    xSemaphoreTake(mqttMutex, portMAX_DELAY);
    // A new home can own the pause even while an older SDK stop call returns.
    if (!motorHomingSessionActive() && mqttPausedForHoming) {
        mqttPausedForHoming = false;
        if (mqttClient != nullptr) esp_mqtt_client_start(mqttClient);
    }
    xSemaphoreGive(mqttMutex);
}

bool prepareHoming(const HomingTaskContext &context) {
    if (!runMotorHomingCommand(context.ticket, [&] {
        if (!stepper->isRunning()) stepperTranslateFrame(StepperFrame::Native);
        stepper->setAcceleration(1000_mm);
        stepper->setSpeedInHz(25_mm);
        stepper->setDirectionPin(Pins::Driver::motorDirectionPin, false);
        return true;
    })) return false;
    float offset = 0;
    if (!context.useHat)
        offset = getAnalogAveragePercent(SampleOnPin{Pins::Driver::currentSensorPin, 1000});
    return commitMotorHomingState(context.ticket, [&] {
        setHomingActive(true, false);
        calibration.isHomed = false;
        calibration.isForward = true;  // preserve the legacy next-stroke direction
        calibration.measuredStrokeSteps = 0;
        if (!context.useHat) calibration.currentSensorOffset = offset;
        return true;
    });
}

void runHomingPass(const HomingTaskContext &context) {
    if (!motorHomingTicketCurrent(context.ticket)) return;
    if (!pauseMqttForHoming(context.ticket)) {
        failHatHoming(context, "Could not prepare homing network resources.");
        return;
    }
    if (!motorHomingTicketCurrent(context.ticket) || !prepareHoming(context)) return;
    if (!motorHomingTicketCurrent(context.ticket)) return;
#ifdef AJ_DEVELOPMENT_HARDWARE
    if (!runMotorHomingCommand(context.ticket, [] {
        stepper->forceStopAndNewPosition(0);
        stepperFrame = StepperFrame::Native;
        return true;
    })) return;
    stateMachine->process_event(HomingPassDone{context.ticket, context.pass});
    return;
#endif
    if (!motorSafetyBeginHomingPass(context.pass, context.ticket)) {
        failHatHoming(context, motorSafetyReason());
        return;
    }
    if (!motorHomingTicketCurrent(context.ticket)) return;
    if (context.useHat) {
        OssmHatTelemetry sample;
        if (!hatSampleReady(sample) || !beginMotorFeedbackHomingPass(context.ticket)) {
            failHatHoming(context, "OSSM HAT feedback unavailable. Check motor power and RS485.");
            return;
        }
    } else if (ossmHatDetected()) {
        failHatHoming(context, "OSSM HAT appeared during homing setup. Request a new home.");
        return;
    }
    if (!motorHomingTicketCurrent(context.ticket)) return;
    if (!enableMotorSafetyHomingOutputs(context.ticket)) {
        failHatHoming(context, motorSafetyReason());
        return;
    }
    const bool backward = context.pass == 2;
    const int16_t sign = backward ? 1 : -1;
    const int32_t target = round(sign * Config::Driver::maxStrokeSteps);
    if (!submitMove(context, target, true)) {
        failHatHoming(context, motorSafetyReason());
        return;
    }
    const TickType_t started = xTaskGetTickCount();
    motor_feedback_logic::HatContactDetector hatContact;
    while (motorHomingTicketCurrent(context.ticket)) {
        if (homing_logic::isHomingTimedOut(
                (xTaskGetTickCount() - started) * portTICK_PERIOD_MS, 40000)) {
            failHatHoming(context, ui::strings::homingTookTooLong);
            return;
        }
        float current;
        bool overLimit;
        if (context.useHat) {
            OssmHatTelemetry sample;
            if (!hatSampleReady(sample)) {
                failHatHoming(context, "OSSM HAT feedback lost. Check motor power and RS485.");
                return;
            }
            current = sample.currentAmps;
            overLimit = hatContact.update(sample.sequence, sample.updatedAtMs,
                sample.positionCounts, current, Config::Hat::sensorlessCurrentLimitAmps,
                32768.0 / (Config::Driver::pulleyToothCount * Config::Driver::beltPitchMm));
        } else {
            current = getAnalogAveragePercent(SampleOnPin{Pins::Driver::currentSensorPin, 50}) -
                calibration.currentSensorOffset;
            overLimit = homing_logic::isCurrentOverLimit(current, 0,
                Config::Driver::sensorlessCurrentLimit);
        }
        if (!motorHomingTicketCurrent(context.ticket)) return;
        if (!overLimit) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        int32_t retreat = 0;
        if (!runMotorHomingCommand(context.ticket, [&] {
            stepper->stopMove();
            stepper->setSpeedInHz(context.useHat ? 25_mm : 250_mm);
            retreat = stepper->getCurrentPosition() - sign * Config::Driver::homingOffsetMn;
            return true;
        })) return;
        if (!moveAndWait(context, retreat, true)) {
            failHatHoming(context, motorSafetyReason());
            return;
        }
        if (!commitMotorHomingState(context.ticket, [&] {
            calibration.measuredStrokeSteps = homing_logic::calculateMeasuredStroke(
                stepper->getCurrentPosition(), Config::Driver::maxStrokeSteps);
            return true;
        })) return;
        if (context.useHat) finishMotorFeedbackHomingPass(context.ticket);
        if (!motorSafetyEndHomingPass(context.ticket)) {
            failHatHoming(context, motorSafetyReason());
            return;
        }
        if (!runMotorHomingCommand(context.ticket, [] {
            stepper->forceStopAndNewPosition(0);
            stepperFrame = StepperFrame::Native;
            return true;
        })) return;
        if (context.useHat && !rebaseMotorFeedbackAtHome(context.ticket) && backward) {
            failHatHoming(context, "OSSM HAT position reference failed. Re-home before use.");
            return;
        }
        if (!motorHomingTicketCurrent(context.ticket)) return;
        if (backward && !motorSafetyBeginNormalOperation(context.ticket)) {
            failHatHoming(context, motorSafetyReason());
            return;
        }
        const int32_t rest = homing_logic::calculatePostHomingPosition(
            sign, calibration.measuredStrokeSteps, UserConfig::afterHomingPosition);
        if ((!context.useHat || backward) && !moveAndWait(context, rest)) {
            failHatHoming(context, motorSafetyReason());
            return;
        }
        if (!commitMotorHomingState(context.ticket, [] { setHomingActive(false, false); return true; })) return;
        stateMachine->process_event(HomingPassDone{context.ticket, context.pass});
        return;
    }
}

void startHomingTask(void *parameter) {
    const HomingTaskContext context = *static_cast<HomingTaskContext *>(parameter);
    delete static_cast<HomingTaskContext *>(parameter);
    runHomingPass(context);
    resumeMqttAfterHoming();
    vTaskDelete(nullptr);
}

void createHomingTask(MotorHomingTicket ticket, uint8_t pass) {
    if (!motorHomingTicketCurrent(ticket)) return;
    auto *context = new (std::nothrow) HomingTaskContext{ticket, pass, ossmHatDetected()};
    BaseType_t created = pdFAIL;
    for (int attempt = 0; context != nullptr && attempt < 20 && created != pdPASS; ++attempt) {
        if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(250));
        if (!motorHomingTicketCurrent(ticket)) break;
        created = xTaskCreatePinnedToCore(startHomingTask, "startHomingTask",
            10 * configMINIMAL_STACK_SIZE, context, configMAX_PRIORITIES - 1,
            &Tasks::runHomingTaskH, Tasks::operationTaskCore);
    }
    if (created == pdPASS) return;
    delete context;
    if (commitMotorHomingState(ticket, [] { taskCreationFailed.store(true); return true; })) {
        errorState.message = ui::strings::homingTookTooLong;
        const HomingTaskContext failed{ticket, pass, ossmHatDetected()};
        failHatHoming(failed, ui::strings::homingTookTooLong);
    }
}

}  // namespace

void startHoming() {
    // First instruction reserves this request. No asynchronous work may adopt
    // a later ticket after a stop, even if task creation or setup takes seconds.
    const auto ticket = beginMotorHomingSession();
    taskCreationFailed.store(false);
    if (mqttMutex == nullptr) mqttMutex = xSemaphoreCreateMutex();
    createHomingTask(ticket, 1);
}

void startSecondHoming(homing_session::Ticket ticket) {
    // SML executes this while the source state is still forward. Pass number
    // and authorization come from the completed request, never current UI.
    createHomingTask(ticket, 2);
}

bool hasTaskCreationFailed() { return taskCreationFailed.load(); }

bool isStrokeTooShort() {
    if (calibration.measuredStrokeSteps > Config::Driver::minStrokeLengthMm) {
        return false;
    }
    errorState.message = ui::strings::strokeTooShort;
    return true;
}

}  // namespace homing
