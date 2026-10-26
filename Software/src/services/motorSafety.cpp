#include "motorSafety.h"

#include <Arduino.h>
#include <Preferences.h>
#include <atomic>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <soc/gpio_struct.h>

#include "constants/Config.h"
#include "motor_guard_logic.h"
#include "ossm/Events.h"
#include "ossm/state/calibration.h"
#include "ossm/state/error.h"
#include "ossm/state/settings.h"
#include "ossm/state/state.h"
#include "services/led.h"
#include "services/ossmHat.h"
#include "services/stepper.h"

namespace {
motor_guard::Config guardConfig() {
    motor_guard::Config config;
    config.countsPerMm = 32768.0 /
        (Config::Driver::pulleyToothCount * Config::Driver::beltPitchMm);
    return config;
}
motor_guard::Guard guard(guardConfig());
portMUX_TYPE guardMux = portMUX_INITIALIZER_UNLOCKED;
MotorSafetyStatus published;
std::atomic_bool initialized{false};
std::atomic_bool rememberedHat{false};
std::atomic_bool savedFault{false};
std::atomic_bool localExclusive{false};
volatile bool deadlineArmed = false;
volatile bool deadlineTripped = false;
volatile uint32_t guardHeartbeatUs = 0;
hw_timer_t *deadlineTimer = nullptr;
TaskHandle_t guardTaskHandle = nullptr;
SemaphoreHandle_t persistenceMutex = nullptr;

// Hardware timer is independent of the UART reader and safety task. Even a
// stalled task cannot leave ENA asserted. GPIO register access is IRAM-safe.
void IRAM_ATTR deadlineInterrupt() {
    // Keep the ISR to native32-bit arithmetic.64-bit division may call a
    // flash-resident compiler helper while an NVS write has disabled cache.
    const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
    if (deadlineArmed && static_cast<uint32_t>(now - guardHeartbeatUs) > 34000) {
        GPIO.out_w1ts = 1UL << Pins::Driver::motorEnablePin;
        deadlineTripped = true;
    }
}

bool required() {
#ifdef OSSM_HAT_REQUIRE_SAFETY
    return true;
#else
    return rememberedHat || ossmHatDetected();
#endif
}

void disableNow() {
    // Disabling output is not physical removal of motor power.
    digitalWrite(Pins::Driver::motorEnablePin, HIGH);
    if (stepper != nullptr) {
        stepper->forceStop();
        stepper->disableOutputs();
    }
    digitalWrite(Pins::Driver::motorEnablePin, HIGH);
}

bool controlsReady() {
    const auto handback = getOssmHatExternalControlStatus();
    if (handback.requested && !handback.complete) return false;
    OssmHatControlSnapshot control;
    // In STEP/DIR mode ENA controls the driver. Register1 is the serial-mode
    // output setting and can remain0 after a verified handback to mode0.
    return readOssmHatControlSnapshot(control) && control.modbusEnable == 0;
}

void publishCoreLocked(uint32_t now) {
    published.armed = guard.armed();
    published.latched = guard.fault() != motor_guard::Fault::None;
    published.fault = static_cast<uint8_t>(guard.fault());
    published.boundsValid = guard.boundsValid();
    published.contacts = guard.contactCount();
    published.observedHz = guard.observedHz(now);
    published.lowerStopCounts = guard.lowerStopCounts();
    published.upperStopCounts = guard.upperStopCounts();
}

bool lockPersistence() {
    return persistenceMutex != nullptr &&
           xSemaphoreTake(persistenceMutex, pdMS_TO_TICKS(1000)) == pdTRUE;
}

bool liveArmed() {
    portENTER_CRITICAL(&guardMux);
    const bool armed = guard.armed();
    portEXIT_CRITICAL(&guardMux);
    return armed;
}

// All first-arm transitions hold persistenceMutex too, so a writer cannot
// start after observing disarmed and then overlap a newly armed session.
void persistFaultIfNeeded() {
    if (!lockPersistence()) return;
    portENTER_CRITICAL(&guardMux);
    const auto fault = guard.fault();
    const bool canWrite = !guard.armed() && fault != motor_guard::Fault::None;
    portEXIT_CRITICAL(&guardMux);
    if (canWrite && !savedFault.load()) {
        Preferences prefs;
        if (prefs.begin("motorGuard", false)) {
            const bool stored = prefs.putUChar("fault", static_cast<uint8_t>(fault)) != 0;
            if (stored) {
                prefs.putBool("armed", false);
                savedFault.store(true);
            }
            prefs.end();
        }
    }
    xSemaphoreGive(persistenceMutex);
}

bool stoppedForReview() {
    const uint32_t started = millis();
    uint32_t quietAt = 0;
    uint32_t firstSequence = 0;
    uint32_t epoch = 0;
    int32_t origin = 0;
    while (static_cast<uint32_t>(millis() - started) < 1000) {
        disableNow();
        const auto hat = getOssmHatStatus();
        OssmHatTelemetry sample;
        const uint32_t now = millis();
        if (hat.startupReady && readOssmHatTelemetry(sample) &&
            static_cast<uint32_t>(now - sample.updatedAtMs) <= 100 &&
            sample.currentAmps < 0.5f && sample.voltageVolts >= 20.0f &&
            sample.voltageVolts <= 36.0f &&
            (sample.alarm == 0 || sample.alarm == 0x10) &&
            digitalRead(Pins::Driver::motorEnablePin) == HIGH) {
            const int64_t delta = motor_feedback_logic::encoderDelta(
                sample.positionCounts, origin);
            if (quietAt == 0 || epoch != sample.continuity ||
                std::abs(static_cast<double>(delta)) > guardConfig().countsPerMm * 0.25) {
                quietAt = now;
                firstSequence = sample.sequence;
                epoch = sample.continuity;
                origin = sample.positionCounts;
            }
            if (now - quietAt >= 100 && sample.sequence != firstSequence) return true;
        } else quietAt = 0;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}

void monitor(void *) {
    uint32_t stationaryAt = 0;
    int32_t stationaryOrigin = 0;
    uint32_t stationaryEpoch = 0;
    uint32_t stationarySequence = 0;
    bool stopRequested = false;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        const uint32_t now = millis();
        guardHeartbeatUs = static_cast<uint32_t>(esp_timer_get_time());
        const bool needed = required();
        const auto hat = getOssmHatStatus();
        OssmHatTelemetry sample;
        const bool fresh = readOssmHatTelemetry(sample) &&
            static_cast<uint32_t>(now - sample.updatedAtMs) <= 100;
        const bool control = controlsReady();
        const bool exclusive = !localExclusive || !hat.passive;
        bool armed;
        bool fault;
        portENTER_CRITICAL(&guardMux);
        if (fresh) {
            guard.observe({sample.sequence, sample.updatedAtMs, sample.continuity,
                           sample.positionCounts, sample.currentAmps,
                           sample.voltageVolts, sample.alarm}, now);
        }
        if (deadlineTripped)
            guard.tripFault(motor_guard::Fault::ObserverDelayed);
        if (guard.armed() && (!control || !exclusive))
            guard.tripFault(motor_guard::Fault::StopPathUnverified);
        guard.tick(now);
        armed = guard.armed();
        fault = guard.fault() != motor_guard::Fault::None;
        portEXIT_CRITICAL(&guardMux);

        if (needed && fault) disableNow();
        else if (needed && !armed) {
            // A first-arm request can race this task between its two critical
            // sections. Recheck live state and only reassert the pin here;
            // a stale unarmed snapshot must not forceStop a new trajectory.
            portENTER_CRITICAL(&guardMux);
            if (!guard.armed() || guard.fault() != motor_guard::Fault::None)
                GPIO.out_w1ts = 1UL << Pins::Driver::motorEnablePin;
            portEXIT_CRITICAL(&guardMux);
        }

        // Startup proves the stop command's pin level AND observes the actual
        // stopped motor. It does not energize or change any drive register.
        bool stopped = false;
        const bool disabled = digitalRead(Pins::Driver::motorEnablePin) == HIGH;
        if (needed && !armed && !fault && disabled && fresh &&
            exclusive && sample.currentAmps < 0.5f &&
            sample.voltageVolts >= 20.0f && sample.voltageVolts <= 36.0f &&
            (sample.alarm == 0 || sample.alarm == 0x10)) {
            const auto delta = motor_feedback_logic::encoderDelta(
                sample.positionCounts, stationaryOrigin);
            if (stationaryAt == 0 || sample.continuity != stationaryEpoch ||
                std::abs(static_cast<double>(delta)) > guardConfig().countsPerMm * 0.25) {
                stationaryAt = now;
                stationaryOrigin = sample.positionCounts;
                stationaryEpoch = sample.continuity;
                stationarySequence = sample.sequence;
            }
            stopped = now - stationaryAt >= 100 && sample.sequence != stationarySequence;
        } else if (!armed) {
            stationaryAt = 0;
        }
        portENTER_CRITICAL(&guardMux);
        if (!guard.armed() && guard.fault() == motor_guard::Fault::None)
            guard.setStopPathVerified(stopped);
        published.required = needed;
        published.ready = !needed || (control && exclusive && guard.ready(now));
        publishCoreLocked(now);
        published.outputDisableVerified = disabled;
        published.stoppedVerified = stopped;
        published.activeMaster = !hat.passive;
        published.sampleAgeMs = hat.ageMs;
        deadlineArmed = needed && guard.armed() && !published.latched;
        portEXIT_CRITICAL(&guardMux);

        if (fault && !stopRequested) {
            disableNow();
            requestOssmHatEmergencyStop();
            stopRequested = true;
        }
        if (!fault) stopRequested = false;
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(5));
    }
}

void report(void *) {
    uint8_t reported = UINT8_MAX;
    for (;;) {
        const auto state = getMotorSafetyStatus();
        if (ossmHatDetected() && !rememberedHat.load() && lockPersistence()) {
            if (!liveArmed()) {
                Preferences prefs;
                if (prefs.begin("motorGuard", false)) {
                    rememberedHat.store(prefs.putBool("requireHat", true) != 0);
                    prefs.end();
                }
            }
            xSemaphoreGive(persistenceMutex);
        }
        if (state.latched) {
            disableNow();
            persistFaultIfNeeded();
            // A reviewed reset may have completed while waiting for storage.
            const auto currentState = getMotorSafetyStatus();
            if (currentState.latched && reported != currentState.fault) {
                reported = currentState.fault;
                calibration.isHomed = false;
                settings.speed = 0;
                settings.speedBLE = 0;
                errorState.message = motorSafetyReason();
                setHomingActive(false);
                setLEDStatusError();
                Serial.printf("RAD_GUARD fault=%u reason=%s; output disabled, power remains connected\n",
                              currentState.fault, motorSafetyReason());
                const auto detail = getMotorSafetyFaultInfo();
                if (detail.available)
                    Serial.printf("RAD_GUARD_SAMPLE seq=%u sampleAt=%u decisionAt=%u delta=%lld gapMs=%u instantMmS=%.2f windowMmS=%.2f windowMs=%u\n",
                                  detail.sequence, detail.sampleAtMs, detail.decisionAtMs,
                                  static_cast<long long>(detail.deltaCounts), detail.gapMs,
                                  detail.instantaneousMmS, detail.windowMmS, detail.windowMs);
                if (stateMachine != nullptr)
                    stateMachine->process_event(MotorSafetyFault{});
                else reported = UINT8_MAX;
            }
        } else reported = UINT8_MAX;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
}  // namespace

void initMotorSafety() {
    bool wasInitialized = false;
    if (!initialized.compare_exchange_strong(wasInitialized, true)) return;
    persistenceMutex = xSemaphoreCreateMutex();
#ifdef OSSM_HAT_REQUIRE_SAFETY
    localExclusive = true;
#endif
    Preferences prefs;
    uint8_t priorFault = 0;
    if (prefs.begin("motorGuard", true)) {
        rememberedHat = prefs.getBool("requireHat", false);
        priorFault = prefs.getUChar("fault", 0);
        if (!priorFault && prefs.getBool("armed", false))
            priorFault = static_cast<uint8_t>(motor_guard::Fault::ObserverDelayed);
        prefs.end();
    }
    if (priorFault) guard.tripFault(static_cast<motor_guard::Fault>(priorFault));
    disableNow();
    guardHeartbeatUs = static_cast<uint32_t>(esp_timer_get_time());
    deadlineTimer = timerBegin(3, 80, true);
    if (deadlineTimer != nullptr) {
        timerAttachInterrupt(deadlineTimer, &deadlineInterrupt, true);
        timerAlarmWrite(deadlineTimer, 5000, true);
        timerAlarmEnable(deadlineTimer);
    }
    const auto created = xTaskCreatePinnedToCore(
        monitor, "motorSafety", 4096, nullptr, configMAX_PRIORITIES - 1,
        &guardTaskHandle, 0);
    const auto reportCreated = xTaskCreatePinnedToCore(
        report, "motorSafetyReport", 4096, nullptr, 2, nullptr, 0);
    if (created != pdPASS || reportCreated != pdPASS || deadlineTimer == nullptr ||
        persistenceMutex == nullptr) {
        portENTER_CRITICAL(&guardMux);
        guard.tripFault(motor_guard::Fault::ObserverDelayed);
        portEXIT_CRITICAL(&guardMux);
        disableNow();
        Serial.println("RAD_GUARD initialization failed; motor output remains disabled");
    }
    const bool needed = required();
    portENTER_CRITICAL(&guardMux);
    published.required = needed;
    publishCoreLocked(millis());
    portEXIT_CRITICAL(&guardMux);
    // If the reporter could not start, persist the failure from setup while
    // outputs are still disabled instead of losing the latch on the next boot.
    if (reportCreated != pdPASS) persistFaultIfNeeded();
}

MotorSafetyStatus getMotorSafetyStatus() {
    portENTER_CRITICAL(&guardMux);
    const auto status = published;
    portEXIT_CRITICAL(&guardMux);
    return status;
}

MotorSafetyFaultInfo getMotorSafetyFaultInfo() {
    portENTER_CRITICAL(&guardMux);
    const auto &detail = guard.faultDiagnostics();
    const MotorSafetyFaultInfo result{
        detail.available, detail.decisionAtMs, detail.sampleAtMs,
        detail.sampleSequence, detail.sampleGapMs, detail.sampleAgeMs,
        detail.continuity, detail.positionCounts, detail.encoderDeltaCounts,
        static_cast<float>(detail.instantaneousMmS),
        static_cast<float>(detail.homingWindowMmS), detail.homingWindowMs,
        guard.fault() == motor_guard::Fault::HomingProgress,
        detail.homingMotionExpected, detail.homingProgressDirection,
        detail.homingProgressAgeMs, detail.homingLowCurrentGraceAgeMs};
    portEXIT_CRITICAL(&guardMux);
    return result;
}

const char *motorSafetyReason() {
    const auto status = getMotorSafetyStatus();
    if (status.latched) return motor_guard::reason(static_cast<motor_guard::Fault>(status.fault));
    if (!status.required) return "Legacy board without a detected HAT";
    if (!status.activeMaster && localExclusive) return "Another RS485 master owns the bus";
    if (!status.ready && !status.armed) return "Waiting for fresh 30 Hz motor feedback and verified stopped output";
    return "ESP32 motor safety guard is monitoring";
}

bool motorSafetyCanEnableOutputs() {
    const bool needed = required();
    const auto hat = getOssmHatStatus();
    const bool controls = controlsReady();
    portENTER_CRITICAL(&guardMux);
    const bool fault = guard.fault() != motor_guard::Fault::None;
    const bool armed = guard.armed();
    portEXIT_CRITICAL(&guardMux);
    if (fault || deadlineTripped) return false;
    if (!needed) return true;
    return initialized.load() && armed && deadlineArmed &&
           hat.ageMs <= 100 && hat.sampleRateHz >= 30 &&
           (!localExclusive.load() || !hat.passive) && controls;
}

bool enableMotorSafetyOutputs() {
    const bool needed = required();
    const auto hat = getOssmHatStatus();
    const bool control = controlsReady();
    portENTER_CRITICAL(&guardMux);
    const bool allowed = initialized.load() && stepper != nullptr &&
        guard.fault() == motor_guard::Fault::None && !deadlineTripped &&
        (!needed || (guard.armed() && deadlineArmed && hat.ageMs <= 100 &&
                     hat.sampleRateHz >= 30 && control &&
                     (!localExclusive.load() || !hat.passive)));
    // FastAccelStepper's enableOutputs is only GPIO writes for these native
    // pins. Check and assert under the same lock as fault/disarm transitions.
    bool enabled = allowed && stepper->enableOutputs();
    if (deadlineTripped) enabled = false;
    if (!enabled) digitalWrite(Pins::Driver::motorEnablePin, HIGH);
    enabled = enabled && digitalRead(Pins::Driver::motorEnablePin) == LOW;
    portEXIT_CRITICAL(&guardMux);
    return enabled;
}

bool motorSafetyBeginHomingPass(uint8_t pass) {
    if (!required()) return motorSafetyCanEnableOutputs();
    disableNow();
    // A requested new home may follow an operator-aborted first pass. A real
    // latched fault is never cleared by this path.
    portENTER_CRITICAL(&guardMux);
    if (pass == 1 && guard.fault() == motor_guard::Fault::None &&
        !guard.boundsValid() && guard.contactCount() != 0) {
        guard.resetAfterReview();
        deadlineArmed = false;
    }
    portEXIT_CRITICAL(&guardMux);
    const uint32_t started = millis();
    while (!getMotorSafetyStatus().ready && millis() - started < 3000) {
        if (getMotorSafetyStatus().latched) return false;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (!getMotorSafetyStatus().ready) return false;
    if (!lockPersistence()) return false;
    portENTER_CRITICAL(&guardMux);
    const bool alreadyArmed = guard.armed();
    const bool hasFault = guard.fault() != motor_guard::Fault::None;
    portEXIT_CRITICAL(&guardMux);
    if (hasFault) {
        xSemaphoreGive(persistenceMutex);
        return false;
    }
    if (!alreadyArmed) {
        Preferences prefs;
        bool stored = false;
        if (prefs.begin("motorGuard", false)) {
            stored = prefs.putBool("requireHat", true) != 0 &&
                     prefs.putBool("armed", true) != 0;
            prefs.end();
        }
        if (!stored) {
            xSemaphoreGive(persistenceMutex);
            return false;
        }
        rememberedHat.store(true);
    }
    // Hold the persistence mutex through this first-arm transition. Reporters
    // can neither start a flash write nor race the crash marker while armed.
    const uint32_t now = millis();
    portENTER_CRITICAL(&guardMux);
    const bool result = guard.arm(now) && guard.beginHomingPass(pass, 25.0f, now);
    if (result && !alreadyArmed)
        guardHeartbeatUs = static_cast<uint32_t>(esp_timer_get_time());
    deadlineArmed = result && guard.armed() && !deadlineTripped;
    publishCoreLocked(now);
    published.ready = guard.ready(now);
    portEXIT_CRITICAL(&guardMux);
    xSemaphoreGive(persistenceMutex);
    return result;
}

bool motorSafetyEndHomingPass() {
    if (!required()) return motorSafetyCanEnableOutputs();
    // The last physical sample can still average part of the retreat. Wait
    // for new stationary encoder/current evidence while the guard keeps its
    // independent5ms loop running; do not wait inside a critical section.
    const uint32_t started = millis();
    bool stopped = false;
    while (static_cast<uint32_t>(millis() - started) <= 200) {
        OssmHatTelemetry sample;
        const uint32_t now = millis();
        const bool fresh = readOssmHatTelemetry(sample) &&
            static_cast<uint32_t>(now - sample.updatedAtMs) <= 100;
        portENTER_CRITICAL(&guardMux);
        const bool fault = guard.fault() != motor_guard::Fault::None;
        stopped = fresh && guard.armed() &&
            std::abs(guard.speedMmS()) <= 2.0 && sample.currentAmps < 1.5f;
        portEXIT_CRITICAL(&guardMux);
        if (fault) return false;
        if (stopped) break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (!stopped) return false;
    portENTER_CRITICAL(&guardMux);
    const bool result = guard.endHomingPass(millis());
    publishCoreLocked(millis());
    portEXIT_CRITICAL(&guardMux);
    return result;
}

bool motorSafetyBeginHomingMotion() {
    if (!required()) return motorSafetyCanEnableOutputs();
    portENTER_CRITICAL(&guardMux);
    const bool result = guard.beginHomingMotion(millis());
    publishCoreLocked(millis());
    portEXIT_CRITICAL(&guardMux);
    return result;
}

bool motorSafetyBeginNormalOperation() {
    if (!required()) return motorSafetyCanEnableOutputs();
    portENTER_CRITICAL(&guardMux);
    const bool result = guard.beginNormalOperation(millis());
    publishCoreLocked(millis());
    portEXIT_CRITICAL(&guardMux);
    return result;
}

void disarmMotorSafety() {
    disableNow();
    portENTER_CRITICAL(&guardMux);
    guard.disarm();
    deadlineArmed = false;
    digitalWrite(Pins::Driver::motorEnablePin, HIGH);
    publishCoreLocked(millis());
    portEXIT_CRITICAL(&guardMux);
    if (!lockPersistence()) return;
    portENTER_CRITICAL(&guardMux);
    const bool mayClearMarker = !guard.armed() &&
        guard.fault() == motor_guard::Fault::None && !deadlineTripped;
    portEXIT_CRITICAL(&guardMux);
    if (mayClearMarker) {
        Preferences prefs;
        if (prefs.begin("motorGuard", false)) {
            prefs.putBool("armed", false);
            prefs.end();
        }
    }
    xSemaphoreGive(persistenceMutex);
}

bool resetMotorSafetyAfterReview() {
    disableNow();
    if (liveArmed() || !stoppedForReview() || !lockPersistence()) return false;
    // The first-arm path also owns this mutex, so it cannot arm while a
    // reviewed reset updates persistent state and clears the live latch.
    if (liveArmed()) {
        xSemaphoreGive(persistenceMutex);
        return false;
    }
    if (!clearOssmHatStopAfterReview()) {
        xSemaphoreGive(persistenceMutex);
        return false;
    }
    Preferences prefs;
    bool cleared = false;
    if (prefs.begin("motorGuard", false)) {
        // A power loss or failed write midway through reset leaves the armed
        // sentinel set, requiring review on boot rather than losing a fault.
        cleared = prefs.putBool("armed", true) != 0 &&
                  prefs.putUChar("fault", 0) != 0 &&
                  prefs.putBool("armed", false) != 0;
        prefs.end();
    }
    if (!cleared) {
        xSemaphoreGive(persistenceMutex);
        return false;
    }
    portENTER_CRITICAL(&guardMux);
    guard.resetAfterReview();
    deadlineTripped = false;
    deadlineArmed = false;
    publishCoreLocked(millis());
    published.ready = false;
    portEXIT_CRITICAL(&guardMux);
    savedFault.store(false);
    xSemaphoreGive(persistenceMutex);
    return true;
}

bool prepareMotorSafetyExternalControlAfterReview() {
    if (liveArmed() || !lockPersistence()) return false;
    portENTER_CRITICAL(&guardMux);
    const bool permitted = !guard.armed() &&
        guard.fault() == motor_guard::Fault::None && !deadlineTripped;
    portEXIT_CRITICAL(&guardMux);
    const bool result = permitted && stoppedForReview() &&
        requestOssmHatExternalControlAfterReview();
    xSemaphoreGive(persistenceMutex);
    return result;
}
