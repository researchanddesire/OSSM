#include "streaming.h"

#include <queue>
#include <chrono>
#include <new>
#include "streaming_logic.h"
#include "constants/Config.h"
#include "ossm/state/calibration.h"
#include "ossm/state/session.h"
#include "ossm/state/settings.h"
#include "ossm/state/state.h"
#include "services/board.h"
#include "services/communication/nimble.h"
#include "services/communication/queue.h"
#include "services/stepper.h"
#include "services/motorSafety.h"
#include "services/tasks.h"

namespace sml = boost::sml;
using namespace sml;

namespace streaming {

static void startStreamingTask(void *pvParameters) {
    const uint64_t safetyRevision = *static_cast<uint64_t *>(pvParameters);
    delete static_cast<uint64_t *>(pvParameters);
    if (!motorSafetyEventCurrent(safetyRevision)) { vTaskDelete(nullptr); return; }
    if (!motorSafetyBeginNormalOperation(safetyRevision)) {
        stateMachine->process_event(MotorSafetyFault{safetyRevision});
        vTaskDelete(nullptr); return;
    }
    // Own the shared stepper config at mode entry (see simple_penetration.cpp:
    // StrokeEngine leaves the shared DIR polarity inverted and the counter in
    // its own frame; this mode's targets are native-frame absolutes).
    stepperTranslateFrame(StepperFrame::Native);
    stepper->setDirectionPin(Pins::Driver::motorDirectionPin, false);
    if (!motorSafetyEventCurrent(safetyRevision) || !enableMotorOutputs()) {
        stateMachine->process_event(MotorSafetyFault{safetyRevision});
        vTaskDelete(nullptr);
        return;
    }

    auto isInCorrectState = [safetyRevision]() {
        if (!motorSafetyEventCurrent(safetyRevision)) return false;
        return stateMachine->is("streaming"_s) ||
               stateMachine->is("streaming.preflight"_s) ||
               stateMachine->is("streaming.idle"_s);
    };

    auto best = std::chrono::steady_clock::now();
    PositionTime lastPositionTime{};

    // Reset the queue to clear any existing commands
    targetQueue.clear();

    const uint32_t maxSpeed = Config::Driver::maxSpeedMmPerSecond * (1_mm);
    const uint32_t maxAccel = Config::Driver::maxAcceleration * (1_mm);

    // Set initial max speed and acceleration
    stepper->setSpeedInHz(maxSpeed);
    stepper->setAcceleration(maxAccel);

    while (isInCorrectState()) {
        if (!motorSafetyCanEnableOutputs()) break;
        uint16_t currentBuffer = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - best).count();
        // Wait for new command from BLE
        // Get next move
        PositionTime targetPositionTime;
        if (!targetQueue.peek(targetPositionTime)) {
            vTaskDelay(1);
            continue;
        }
        //Wait for previous command to finish if it isn't moving in the same direction.
        const int positionDelta = targetPositionTime.position - lastPositionTime.position;
        targetPositionTime.direction = (positionDelta > 0) - (positionDelta < 0);
        bool sameDirection = lastPositionTime.direction == targetPositionTime.direction;
        if (!sameDirection && stepper->isRunning()){
            vTaskDelay(1);
            continue;
        }
        targetQueue.pop(targetPositionTime);

        float timeSeconds = targetPositionTime.inTime / 1000.0f;

        // settime is when the message was received. If we trust the source we can reduce perceived lag by creating a buffer.
        if (USE_LATENCY_COMPENSATION){
            int16_t mincomp =  min(int(settings.buffer * 2),int(lastPositionTime.inTime));
            int16_t offset = mincomp - currentBuffer;
            int16_t lag = std::chrono::duration_cast<std::chrono::milliseconds>(targetPositionTime.setTime - best).count();
            if (lag < 0 || lag > mincomp * 10){
                best = targetPositionTime.setTime;
                lag = 0;
                offset = 0;
            }
            best += std::chrono::milliseconds(targetPositionTime.inTime);
            ESP_LOGD("Streaming", "Lagency Compensation - Lag: %d, Buffer: %d/%d", lag, currentBuffer, mincomp);
            if (offset < 0){
                // Shorten time up to 1/4 of the total time.
                offset = max(int16_t(targetPositionTime.inTime/-4), offset);
            }
            timeSeconds += offset/1000.0f;
        } else {
            best = std::chrono::steady_clock::now();
        }
        lastPositionTime = targetPositionTime;
        int32_t maxStroke = streaming_logic::calculateMaxStroke(
            settings.stroke, settings.depth,
            calibration.measuredStrokeSteps);
        int32_t depth = streaming_logic::calculateDepthOffset(
            calibration.measuredStrokeSteps, maxStroke, settings.depth);
        const uint32_t speedLimit = maxSpeed * (settings.speed / 100.0);
        const uint32_t accelLimit = maxAccel * (settings.sensation / 100.0);
        const int32_t requestedPosition = streaming_logic::scaleStreamPosition(
            targetPositionTime.position, maxStroke, depth);
        const int32_t currentPosition = stepper->getCurrentPosition();
        auto move = streaming_logic::planMotion(
            currentPosition, requestedPosition, timeSeconds, speedLimit, accelLimit, 1_mm);
        if (move.distance > 0) {
            if (stepper->isRunning()) {
                move.acceleration = max(stepper->getAcceleration(), move.acceleration);
            }
            stepper->setAcceleration(move.acceleration);
            stepper->setSpeedInHz(move.speed);
            stepper->moveToForOwner(move.targetPosition, safetyRevision, motorSafetyEventCurrent, false);

            ESP_LOGI("Streaming", "P(%d): %ld -> %ld = %ld, T: %.3f, S: %lu, A: %lu, Q: %zu",
                     targetPositionTime.position, static_cast<long>(currentPosition),
                     static_cast<long>(move.targetPosition), static_cast<long>(move.distance),
                     timeSeconds, static_cast<unsigned long>(move.speed),
                     static_cast<unsigned long>(move.acceleration), targetQueue.size());
        }
        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}

void startStreaming() {
    // Keep the current low-stack allocation and retry policy. The captured
    // revision stays fixed while task creation retries, so stop cannot be undone.
    const uint32_t stackSize = 5 * configMINIMAL_STACK_SIZE;
    auto *revision = new (std::nothrow) uint64_t(getMotorSafetyStatus().controlRevision);
    if (!revision) return;
    if (!Tasks::startOperation(startStreamingTask, "startStreamingTask",
                               stackSize, nullptr, revision)) {
        delete revision;
        return;
    }
}

}  // namespace streaming
