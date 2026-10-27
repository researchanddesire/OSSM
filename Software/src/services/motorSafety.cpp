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
#include "services/motorFeedback.h"
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
SemaphoreHandle_t homingCommandMutex = nullptr;
homing_session::Gate homingSession;
// Initial serial commissioning caps apply to every HAT operating mode. The
// local trajectory planner may request less, never more, without RAM re-setup.
constexpr uint16_t serialMaximumRpm = 37;  // 24.67 mm/s with the stock pulley
constexpr uint16_t serialAccelerationRpmPerSec = 1500;
uint64_t serialCounter = 0;
uint64_t safetyControlRevision = 0;  // protected by guardMux
uint64_t serialSession = 0;  // protected by guardMux, distinct from home ticket
std::atomic_bool serialHardwareStopVerified{false};
uint32_t serialProofContinuity = 0;  // per-boot proof, never stored in NVS
uint64_t serialProbeCutWrites = 0;
bool serialProbeAuditStarted = false;
std::atomic_bool serialPreparing{false};
std::atomic_bool serialEnabling{false};
serial_interlock_probe::Gate serialProbe;
std::atomic_bool serialProbeClaimed{false};
uint64_t serialProbeSession = 0;
uint32_t serialProbeContinuity = 0;
bool serialProbeReadbackDelivered = false;
TaskHandle_t serialProbeTask = nullptr;
std::atomic_bool serialProbeCancelled{false};

void invalidateSerialLocked() {
    serialSession = 0;
    serialPreparing = false;
    serialEnabling = false;
}

bool ticketCurrentLocked(MotorHomingTicket ticket) {
    return homingSession.accepts(ticket) && !deadlineTripped &&
           guard.fault() == motor_guard::Fault::None;
}

void revokeHomingLocked() {
    if (homingSession.active() || serialSession != 0) ++safetyControlRevision;
    if (homingSession.active()) homingSession.cancel();
    invalidateSerialLocked();
    serialHardwareStopVerified = false;
    GPIO.out_w1ts = 1UL << Pins::Driver::motorEnablePin;
}

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

bool enforceCurrentFaultStop() {
    // Never wait in the independent5ms observer. A reset clears the live fault
    // under this same mutex, so an obsolete snapshot cannot perform stop I/O
    // after reset has handed authority to a new intent.
    const bool haveMutex = homingCommandMutex != nullptr;
    if (haveMutex && xSemaphoreTake(homingCommandMutex, 0) != pdTRUE) return false;
    portENTER_CRITICAL(&guardMux);
    const bool fault = guard.fault() != motor_guard::Fault::None;
    if (fault) GPIO.out_w1ts = 1UL << Pins::Driver::motorEnablePin;
    portEXIT_CRITICAL(&guardMux);
    if (fault) {
        disableNow();
        requestOssmHatEmergencyStop();
    }
    // A missing mutex is an initialization failure: reset cannot clear its
    // latch, but boot must still be able to request the serial output stop.
    if (haveMutex) xSemaphoreGive(homingCommandMutex);
    return fault;
}

bool controlsReady() {
    OssmHatControlSnapshot control;
    if (!readOssmHatControlSnapshot(control)) return false;
    if (!ossmHatDetected()) return control.modbusEnable == 0;
    const auto position = getOssmHatPositionStatus();
    // A disabled, prepared hold is safe while an armed home waits for enable.
    // The UART owner separately authorizes and verifies the final enable.
    return control.modbusEnable == 1 && position.prepared && !position.faulted &&
           serialHardwareStopVerified.load() &&
           (((control.outputEnable & 1U) == 0 && !position.enabled) ||
            ((control.outputEnable & 1U) != 0 && position.enabled) ||
            serialEnabling.load());
}

bool serialOutputOff() {
    OssmHatControlSnapshot control;
    return readOssmHatControlSnapshot(control) && control.modbusEnable == 1 &&
           (control.outputEnable & 1U) == 0;
}

bool serialTransportAuthorized(uint64_t session, bool motion) {
    const auto hat = getOssmHatStatus();
    const auto position = getOssmHatPositionStatus();
    OssmHatTelemetry physical;
    const bool physicalFresh = readOssmHatTelemetry(physical) &&
        static_cast<uint32_t>(millis() - physical.updatedAtMs) <= 100;
    OssmHatControlSnapshot control;
    if (!readOssmHatControlSnapshot(control) || control.modbusEnable != 1 ||
        !hat.startupReady || hat.passive || hat.baudRate != 115200 ||
        hat.ageMs > 100 || hat.sampleRateHz < 30) return false;
    portENTER_CRITICAL(&guardMux);
    bool allowed = session != 0 && session == serialSession &&
        guard.fault() == motor_guard::Fault::None && !deadlineTripped;
    if (motion) {
        const auto probe = serialProbe.status();
        const bool probeCommand = serialProbeClaimed.load() &&
            !serialProbeCancelled.load() &&
            session == serialProbeSession && probe.active && physicalFresh &&
            physical.continuity == serialProbeContinuity &&
            (probe.phase == serial_interlock_probe::Phase::Prepared ||
             probe.phase == serial_interlock_probe::Phase::Moving);
        const bool enabling = (probeCommand &&
            probe.phase == serial_interlock_probe::Phase::Prepared) ||
            (!serialProbeClaimed.load() && serialEnabling.load());
        const bool outputAuthorized = enabling ||
            ((control.outputEnable & 1U) != 0 && position.enabled &&
             !position.faulted && position.session == session);
        allowed = allowed && deadlineArmed &&
            ((guard.armed() && serialHardwareStopVerified.load() && physicalFresh &&
              physical.continuity == serialProofContinuity &&
              !serialProbeClaimed.load()) || probeCommand) &&
            outputAuthorized && digitalRead(Pins::Driver::motorEnablePin) == LOW;
    } else {
        allowed = allowed && !guard.armed() &&
            digitalRead(Pins::Driver::motorEnablePin) == HIGH &&
            (control.outputEnable & 1U) == 0;
    }
    portEXIT_CRITICAL(&guardMux);
    return allowed;
}

struct SafetyControlOwner {
    uint64_t revision = 0;
    uint64_t session = 0;
    MotorHomingTicket ticket{};
    bool hasTicket = false;
    bool requireHealthy = true;
};

bool controlOwnerLocked(const SafetyControlOwner &owner) {
    return owner.revision == safetyControlRevision &&
        (owner.session == 0 || owner.session == serialSession) &&
        (!owner.requireHealthy ||
            (guard.fault() == motor_guard::Fault::None && !deadlineTripped)) &&
        (!owner.hasTicket || ticketCurrentLocked(owner.ticket));
}

bool controlOwnerCurrent(const SafetyControlOwner &owner) {
    portENTER_CRITICAL(&guardMux);
    const bool current = controlOwnerLocked(owner);
    portEXIT_CRITICAL(&guardMux);
    return current;
}

bool takeControlCommands(const SafetyControlOwner &owner) {
    if (homingCommandMutex == nullptr) return false;
    const uint32_t started = millis();
    while (controlOwnerCurrent(owner) && millis() - started < 1000) {
        if (xSemaphoreTake(homingCommandMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            if (controlOwnerCurrent(owner)) return true;
            xSemaphoreGive(homingCommandMutex);
            return false;
        }
    }
    return false;
}

bool stoppedForReview(const SafetyControlOwner &owner, OssmHatTelemetry *proof = nullptr);
bool stoppedForReview();

bool prepareSerialDisabled(const MotorHomingTicket *ticket,
                           uint16_t rpm = serialMaximumRpm,
                           uint16_t acceleration = serialAccelerationRpmPerSec,
                           bool probe = false,
                           const uint64_t *expectedRevision = nullptr) {
    portENTER_CRITICAL(&guardMux);
    SafetyControlOwner owner{expectedRevision ? *expectedRevision : safetyControlRevision, 0,
        ticket ? *ticket : MotorHomingTicket{}, ticket != nullptr};
    const bool admitted = controlOwnerLocked(owner) && !guard.armed();
    portEXIT_CRITICAL(&guardMux);
    if (!admitted) return false;

    // Stop was queued by the caller. Observe its output-off readback without
    // repeatedly flushing commands or changing a newer request's ownership.
    const uint32_t offStarted = millis();
    while (!serialOutputOff()) {
        if (!controlOwnerCurrent(owner) || millis() - offStarted >= 1000) return false;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    OssmHatTelemetry proof;
    if (!stoppedForReview(owner, &proof) || !takeControlCommands(owner)) return false;
    OssmHatTelemetry sample;
    const bool fresh = readOssmHatTelemetry(sample) &&
        static_cast<uint32_t>(millis() - sample.updatedAtMs) <= 100 &&
        sample.currentAmps >= 0 && sample.currentAmps < 0.5f &&
        sample.voltageVolts >= 20.0f && sample.voltageVolts <= 36.0f &&
        (sample.alarm == 0 || sample.alarm == 0x10) &&
        sample.continuity == proof.continuity &&
        std::abs(static_cast<double>(motor_feedback_logic::encoderDelta(
            sample.positionCounts, proof.positionCounts))) <= guardConfig().countsPerMm * 0.02;
    bool allocated = false;
    portENTER_CRITICAL(&guardMux);
    if (fresh && controlOwnerLocked(owner) && !guard.armed() &&
        digitalRead(Pins::Driver::motorEnablePin) == HIGH) {
        if (++serialCounter == 0) ++serialCounter;
        owner.session = serialSession = serialCounter;
        if (probe) serialProbeSession = serialSession;
        serialPreparing = true;
        allocated = true;
    }
    portEXIT_CRITICAL(&guardMux);
    // Driver methods invoke the external validator, so do not hold guardMux.
    // The command mutex serializes this bounded commit against stop flushing.
    const bool queued = allocated && clearOssmHatStopAfterReview() &&
        requestOssmHatPreparePositionControl(owner.session, sample.positionCounts,
                                           rpm, acceleration);
    xSemaphoreGive(homingCommandMutex);
    const uint32_t started = millis();
    while (queued && static_cast<uint32_t>(millis() - started) < 1500) {
        if (!controlOwnerCurrent(owner)) return false;
        const auto status = getOssmHatPositionStatus();
        if (status.session != owner.session || status.faulted) break;
        if (status.prepared) {
            portENTER_CRITICAL(&guardMux);
            const bool current = controlOwnerLocked(owner) && !guard.armed();
            if (current) serialPreparing = false;
            portEXIT_CRITICAL(&guardMux);
            return current;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (allocated && takeControlCommands(owner)) {
        portENTER_CRITICAL(&guardMux);
        const bool cleanup = controlOwnerLocked(owner) && !guard.armed();
        if (cleanup) invalidateSerialLocked();
        portEXIT_CRITICAL(&guardMux);
        if (cleanup) cancelOssmHatPositionControl();
        xSemaphoreGive(homingCommandMutex);
    }
    return false;
}

void publishCoreLocked(uint32_t now) {
    published.armed = guard.armed();
    published.latched = guard.fault() != motor_guard::Fault::None;
    published.fault = static_cast<uint8_t>(guard.fault());
    published.boundsValid = guard.boundsValid();
    published.contacts = guard.contactCount();
    published.controlRevision = safetyControlRevision;
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
    const bool armed = guard.armed() || serialProbeClaimed.load();
    portEXIT_CRITICAL(&guardMux);
    return armed;
}

// All first-arm transitions hold persistenceMutex too, so a writer cannot
// start after observing disarmed and then overlap a newly armed session.
void persistFaultIfNeeded() {
    if (!lockPersistence()) return;
    portENTER_CRITICAL(&guardMux);
    const auto fault = guard.fault();
    const bool canWrite = !guard.armed() && !serialProbeClaimed.load() &&
        fault != motor_guard::Fault::None;
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

bool stoppedForReview(const SafetyControlOwner &owner, OssmHatTelemetry *proof) {
    const uint32_t started = millis();
    uint32_t quietAt = 0;
    uint32_t firstSequence = 0;
    uint32_t epoch = 0;
    int32_t origin = 0;
    while (static_cast<uint32_t>(millis() - started) < 1000) {
        if (!controlOwnerCurrent(owner)) return false;
        const auto hat = getOssmHatStatus();
        OssmHatTelemetry sample;
        const uint32_t now = millis();
        if (hat.startupReady && readOssmHatTelemetry(sample) &&
            static_cast<uint32_t>(now - sample.updatedAtMs) <= 100 &&
            sample.currentAmps < 0.5f && sample.voltageVolts >= 20.0f &&
            sample.voltageVolts <= 36.0f &&
            (sample.alarm == 0 || sample.alarm == 0x10) &&
            digitalRead(Pins::Driver::motorEnablePin) == HIGH &&
            (!ossmHatDetected() || serialOutputOff())) {
            const int64_t delta = motor_feedback_logic::encoderDelta(
                sample.positionCounts, origin);
            if (quietAt == 0 || epoch != sample.continuity ||
                std::abs(static_cast<double>(delta)) > guardConfig().countsPerMm * 0.02) {
                quietAt = now;
                firstSequence = sample.sequence;
                epoch = sample.continuity;
                origin = sample.positionCounts;
            }
            if (now - quietAt >= 100 && sample.sequence != firstSequence) {
                if (!controlOwnerCurrent(owner)) return false;
                if (proof) *proof = sample;
                return true;
            }
        } else quietAt = 0;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}

bool stoppedForReview() {
    portENTER_CRITICAL(&guardMux);
    SafetyControlOwner owner{safetyControlRevision};
    // A reviewed fault reset must be able to prove a physical stop while the
    // fault/deadline latch is still set. This proof itself never clears either.
    owner.requireHealthy = false;
    portEXIT_CRITICAL(&guardMux);
    return stoppedForReview(owner);
}

void monitor(void *) {
    uint32_t stationaryAt = 0;
    int32_t stationaryOrigin = 0;
    uint32_t stationaryEpoch = 0;
    uint32_t stationarySequence = 0;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        guardHeartbeatUs = static_cast<uint32_t>(esp_timer_get_time());
        const bool needed = required();
        const auto hat = getOssmHatStatus();
        OssmHatTelemetry sample;
        const bool haveSample = readOssmHatTelemetry(sample);
        const bool control = controlsReady();
        OssmHatControlSnapshot serialControl;
        const bool serialMode = readOssmHatControlSnapshot(serialControl) &&
            serialControl.modbusEnable == 1;
        const auto txAudit = getOssmHatTxAudit();
        const auto stopSnapshot = getOssmHatEmergencyStopStatus();
        const uint32_t now = millis();
        const bool fresh = haveSample &&
            static_cast<uint32_t>(now - sample.updatedAtMs) <= 100;
        const bool freshSerialMode = serialMode &&
            static_cast<uint32_t>(now - serialControl.updatedAtMs) <= 100;
        const bool exclusive = !hat.passive;
        bool armed;
        bool fault;
        bool probeDisableDue = false;
        portENTER_CRITICAL(&guardMux);
        if (serialHardwareStopVerified.load() &&
            (!hat.detected || !fresh || hat.passive || hat.baudRate != 115200 ||
             !serialMode || sample.continuity != serialProofContinuity)) {
            serialHardwareStopVerified = false;
            if (guard.armed()) guard.tripFault(motor_guard::Fault::StopPathUnverified);
        }
        if (fresh) {
            guard.observe({sample.sequence, sample.updatedAtMs, sample.continuity,
                           sample.positionCounts, sample.currentAmps,
                           sample.voltageVolts, sample.alarm}, now);
        }
        if (deadlineTripped)
            guard.tripFault(motor_guard::Fault::ObserverDelayed);
        if (serialProbeClaimed.load()) {
            const auto before = serialProbe.status();
            if (before.active &&
                (hat.passive || !hat.startupReady || hat.baudRate != 115200 ||
                 !fresh || !freshSerialMode))
                serialProbe.abort(serial_interlock_probe::Failure::SafetyFault, now);
            const bool gpioProof = before.phase == serial_interlock_probe::Phase::Braking ||
                before.phase == serial_interlock_probe::Phase::ProvingGpio;
            const bool afterCut = gpioProof ||
                before.phase == serial_interlock_probe::Phase::AwaitingSerialDisable ||
                before.phase == serial_interlock_probe::Phase::ProvingFinalQuiet;
            if (afterCut && digitalRead(Pins::Driver::motorEnablePin) != HIGH)
                serialProbe.unexpectedControlWrite(now);
            if (gpioProof && serialProbeAuditStarted &&
                (txAudit.controlWrites != serialProbeCutWrites || stopSnapshot.requested))
                serialProbe.unexpectedControlWrite(now);
            if (fresh) serialProbe.observe({sample.sequence, sample.updatedAtMs,
                sample.continuity, sample.positionCounts, sample.currentAmps,
                sample.voltageVolts, hat.sampleRateHz, sample.alarm}, now);
            serialProbe.tick(now);
            if (guard.fault() != motor_guard::Fault::None)
                serialProbe.abort(serial_interlock_probe::Failure::SafetyFault, now);
            for (;;) {
                const auto action = serialProbe.takeAction(now);
                if (action == serial_interlock_probe::Action::None) break;
                if (action == serial_interlock_probe::Action::AssertGpioHigh) {
                    GPIO.out_w1ts = 1UL << Pins::Driver::motorEnablePin;
                    if (serialProbe.markGpioAsserted(now)) {
                        serialProbeCutWrites = txAudit.controlWrites;
                        serialProbeAuditStarted = true;
                    }
                } else probeDisableDue = true;
            }
        }
        if (guard.armed() && (!control || !exclusive))
            guard.tripFault(motor_guard::Fault::StopPathUnverified);
        guard.tick(now);
        armed = guard.armed();
        fault = guard.fault() != motor_guard::Fault::None;
        if (fault) revokeHomingLocked();
        portEXIT_CRITICAL(&guardMux);

        if (needed && fault) enforceCurrentFaultStop();
        else if (needed && !armed && !serialProbeClaimed.load()) {
            // A first-arm request can race this task between its two critical
            // sections. Recheck live state and only reassert the pin here;
            // a stale unarmed snapshot must not forceStop a new trajectory.
            portENTER_CRITICAL(&guardMux);
            if ((!guard.armed() && !serialProbeClaimed.load()) ||
                guard.fault() != motor_guard::Fault::None)
                GPIO.out_w1ts = 1UL << Pins::Driver::motorEnablePin;
            portEXIT_CRITICAL(&guardMux);
        }

        // Startup proves the stop command's pin level AND observes the actual
        // stopped motor. It does not energize or change any drive register.
        bool stopped = false;
        const bool gpioDisabled = digitalRead(Pins::Driver::motorEnablePin) == HIGH;
        const bool disabled = gpioDisabled &&
            (!hat.detected || serialOutputOff() || serialHardwareStopVerified.load());
        if (needed && !armed && !fault && disabled && fresh &&
            exclusive && sample.currentAmps < 0.5f &&
            sample.voltageVolts >= 20.0f && sample.voltageVolts <= 36.0f &&
            (sample.alarm == 0 || sample.alarm == 0x10)) {
            const auto delta = motor_feedback_logic::encoderDelta(
                sample.positionCounts, stationaryOrigin);
            if (stationaryAt == 0 || sample.continuity != stationaryEpoch ||
                std::abs(static_cast<double>(delta)) > guardConfig().countsPerMm * 0.02) {
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
        published.ready = !needed || (!serialProbeClaimed.load() &&
            control && exclusive && guard.ready(now));
        publishCoreLocked(now);
        published.outputDisableVerified = disabled;
        published.stoppedVerified = stopped;
        published.activeMaster = !hat.passive;
        published.sampleAgeMs = hat.ageMs;
        published.armed = guard.armed() || serialProbe.status().active;
        deadlineArmed = needed && published.armed && !published.latched;
        portEXIT_CRITICAL(&guardMux);

        if (probeDisableDue) cancelOssmHatPositionControl();
        const auto probeSnapshot = getMotorSerialInterlockProbeStatus();
        if (serialProbeClaimed.load() &&
            (probeSnapshot.phase == serial_interlock_probe::Phase::AwaitingSerialDisable ||
             probeSnapshot.phase == serial_interlock_probe::Phase::ProvingFinalQuiet)) {
            const auto stop = getOssmHatEmergencyStopStatus();
            if (stop.requested && stop.complete && stop.verified) {
                portENTER_CRITICAL(&guardMux);
                if (!serialProbeReadbackDelivered &&
                    serialProbe.markSerialDisabled({stop.completedAtMs,
                        stop.modbusEnable, stop.driveOutputEnable}, millis()))
                    serialProbeReadbackDelivered = true;
                portEXIT_CRITICAL(&guardMux);
            }
        }

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
            enforceCurrentFaultStop();
            persistFaultIfNeeded();
            // A reviewed reset may have completed while waiting for storage.
            const auto currentState = getMotorSafetyStatus();
            if (currentState.latched && reported != currentState.fault) {
                reported = currentState.fault;
                calibration.isHomed = false;
                settings.speed = 0;
                settings.speedBLE = 0;
                errorState.message = motorSafetyReason();
                setHomingActive(false, false);
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
                    stateMachine->process_event(MotorSafetyFault{currentState.controlRevision});
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
    homingCommandMutex = xSemaphoreCreateMutex();
    registerMotorSerialMotionAuthorization(motorSafetySerialSession,
                                           motorSafetySerialCanMove);
    setOssmHatPositionValidator(serialTransportAuthorized);
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
        persistenceMutex == nullptr || homingCommandMutex == nullptr) {
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

bool motorSafetyEventCurrent(uint64_t revision) {
    portENTER_CRITICAL(&guardMux);
    const bool current = revision == safetyControlRevision;
    portEXIT_CRITICAL(&guardMux);
    return current;
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
    if (ossmHatDetected() && !serialHardwareStopVerified.load())
        return "Serial motion needs a reviewed ENA stop test after each boot";
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
    if (fault || deadlineTripped || serialProbeClaimed.load()) return false;
    if (!needed) return true;
    return initialized.load() && armed && deadlineArmed &&
           hat.ageMs <= 100 && hat.sampleRateHz >= 30 &&
           (!localExclusive.load() || !hat.passive) && controls;
}

uint64_t motorSafetySerialSession() {
    const auto position = getOssmHatPositionStatus();
    portENTER_CRITICAL(&guardMux);
    const uint64_t session = !serialProbeClaimed.load() && serialHardwareStopVerified.load() &&
        guard.armed() && guard.fault() == motor_guard::Fault::None &&
        !deadlineTripped && deadlineArmed && position.enabled &&
        position.session == serialSession ? serialSession : 0;
    portEXIT_CRITICAL(&guardMux);
    return session;
}

bool motorSafetySerialCanMove(uint64_t session) {
    const auto position = getOssmHatPositionStatus();
    return position.enabled && position.session == session &&
        !position.faulted && serialTransportAuthorized(session, true);
}

MotorSerialSafetyStatus getMotorSerialSafetyStatus() {
    const auto position = getOssmHatPositionStatus();
    return {ossmHatDetected(), serialHardwareStopVerified.load(),
            position.prepared, position.enabled, serialPreparing.load(),
            serialMaximumRpm, serialAccelerationRpmPerSec};
}

static bool enableOutputsForTicket(const MotorHomingTicket *ticket) {
    const bool needed = required();
    const auto hat = getOssmHatStatus();
    const bool control = controlsReady();
    portENTER_CRITICAL(&guardMux);
    const bool allowed = !serialProbeClaimed.load() && (ticket ? ticketCurrentLocked(*ticket) :
        !homingSession.active()) && homingSession.commandsReady() &&
        initialized.load() && stepper != nullptr &&
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

static bool enableSerialAfterGpio(const MotorHomingTicket *ticket) {
    if (!ossmHatDetected()) return true;
    portENTER_CRITICAL(&guardMux);
    const SafetyControlOwner owner{safetyControlRevision, serialSession,
        ticket ? *ticket : MotorHomingTicket{}, ticket != nullptr};
    const bool allowed = owner.session != 0 && controlOwnerLocked(owner) &&
        serialHardwareStopVerified.load() && guard.armed();
    portEXIT_CRITICAL(&guardMux);
    if (!allowed) return false;
    const auto prior = getOssmHatPositionStatus();
    if (prior.session == owner.session && prior.enabled &&
        motorSafetySerialCanMove(owner.session) && controlOwnerCurrent(owner)) return true;
    if (!takeControlCommands(owner)) return false;
    portENTER_CRITICAL(&guardMux);
    const bool current = controlOwnerLocked(owner) && guard.armed() &&
        serialHardwareStopVerified.load();
    if (current) serialEnabling = true;
    portEXIT_CRITICAL(&guardMux);
    const bool queued = current && requestOssmHatEnablePositionControl(owner.session);
    xSemaphoreGive(homingCommandMutex);
    const uint32_t started = millis();
    while (queued && static_cast<uint32_t>(millis() - started) < 1000) {
        if (!controlOwnerCurrent(owner)) return false;
        const auto position = getOssmHatPositionStatus();
        if (position.session != owner.session || position.faulted) break;
        if (position.enabled && motorSafetySerialCanMove(owner.session)) {
            portENTER_CRITICAL(&guardMux);
            const bool completed = controlOwnerLocked(owner) && guard.armed();
            if (completed) serialEnabling = false;
            portEXIT_CRITICAL(&guardMux);
            return completed;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    // An obsolete waiter must never disarm/cancel a later serial epoch. A
    // still-owned failed enable latches a fault before its serialized stop.
    if (takeControlCommands(owner)) {
        portENTER_CRITICAL(&guardMux);
        const bool cleanup = controlOwnerLocked(owner);
        if (cleanup) {
            guard.tripFault(motor_guard::Fault::StopPathUnverified);
            revokeHomingLocked();
            deadlineArmed = false;
            publishCoreLocked(millis());
        }
        portEXIT_CRITICAL(&guardMux);
        if (cleanup) disableNow();
        xSemaphoreGive(homingCommandMutex);
    }
    return false;
}

bool enableMotorSafetyOutputs() {
    return enableOutputsForTicket(nullptr) && enableSerialAfterGpio(nullptr);
}

bool enableMotorSafetyHomingOutputs(MotorHomingTicket ticket) {
    // Never wait for UART ACK while holding the command/flush mutex.
    return runMotorHomingCommand(ticket, [&] { return enableOutputsForTicket(&ticket); }) &&
        enableSerialAfterGpio(&ticket);
}

bool motorHomingTicketCurrent(MotorHomingTicket ticket) {
    portENTER_CRITICAL(&guardMux);
    const bool current = ticketCurrentLocked(ticket);
    portEXIT_CRITICAL(&guardMux);
    return current;
}

bool motorHomingSessionActive() {
    portENTER_CRITICAL(&guardMux);
    const bool active = homingSession.active();
    portEXIT_CRITICAL(&guardMux);
    return active;
}

bool completeMotorHomingSession(MotorHomingTicket ticket) {
    portENTER_CRITICAL(&guardMux);
    const bool current = ticketCurrentLocked(ticket);
    if (current) {
        calibration.isHomed = true;
        calibration.justHomed = true;
        homingSession.cancel();
    }
    portEXIT_CRITICAL(&guardMux);
    return current;
}

MotorHomingTicket beginMotorHomingSession() {
    // Reserve intent before any mutex/task allocation can block. A Stop during
    // the following command flush invalidates this exact ticket permanently.
    portENTER_CRITICAL(&guardMux);
    ++safetyControlRevision;
    published.controlRevision = safetyControlRevision;
    if (serialProbeClaimed.load()) {
        serialProbeCancelled = true;
        serialProbe.abort(serial_interlock_probe::Failure::Cancelled, millis());
        GPIO.out_w1ts = 1UL << Pins::Driver::motorEnablePin;
        portEXIT_CRITICAL(&guardMux);
        return {};
    }
    guard.disarm();
    deadlineArmed = false;
    invalidateSerialLocked();
    const auto ticket = homingSession.begin();
    calibration.isHomed = false;
    setHomingActive(true, false);
    GPIO.out_w1ts = 1UL << Pins::Driver::motorEnablePin;
    portEXIT_CRITICAL(&guardMux);
    runMotorHomingCommand(ticket, [] { disableNow(); return true; });
    return ticket;
}

static void cancelAndFlushHoming(bool disarm) {
    portENTER_CRITICAL(&guardMux);
    ++safetyControlRevision;
    published.controlRevision = safetyControlRevision;
    if (serialProbeClaimed.load()) {
        serialProbeCancelled = true;
        serialProbe.abort(serial_interlock_probe::Failure::Cancelled, millis());
    }
    const bool abandonedHome = homingSession.active();
    homingSession.beginCancel();
    invalidateSerialLocked();
    if (abandonedHome || disarm || ossmHatDetected()) {
        guard.disarm();
        deadlineArmed = false;
        if (abandonedHome) {
            calibration.isHomed = false;
            setHomingActive(false, false);
        }
        publishCoreLocked(millis());
    }
    GPIO.out_w1ts = 1UL << Pins::Driver::motorEnablePin;
    portEXIT_CRITICAL(&guardMux);
    // This mutex never spans NVS, network shutdown, feedback waits or SML.
    // A submission already in progress completes with ENA HIGH, then is
    // flushed before this Stop acknowledges completion.
    if (homingCommandMutex != nullptr) xSemaphoreTake(homingCommandMutex, portMAX_DELAY);
    disableNow();
    portENTER_CRITICAL(&guardMux);
    homingSession.finishCancel();
    GPIO.out_w1ts = 1UL << Pins::Driver::motorEnablePin;
    portEXIT_CRITICAL(&guardMux);
    if (homingCommandMutex != nullptr) xSemaphoreGive(homingCommandMutex);
}

void cancelMotorHomingSession() { cancelAndFlushHoming(false); }

bool runMotorHomingCommand(MotorHomingTicket ticket, MotorHomingAction action, void *context) {
    if (homingCommandMutex == nullptr) return false;
    while (motorHomingTicketCurrent(ticket)) {
        if (xSemaphoreTake(homingCommandMutex, pdMS_TO_TICKS(20)) != pdTRUE) continue;
        portENTER_CRITICAL(&guardMux);
        const bool current = ticketCurrentLocked(ticket);
        const bool flushing = !homingSession.commandsReady();
        portEXIT_CRITICAL(&guardMux);
        if (!current) { xSemaphoreGive(homingCommandMutex); return false; }
        if (flushing) {
            xSemaphoreGive(homingCommandMutex);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        const bool result = action(context);
        const bool stillCurrent = motorHomingTicketCurrent(ticket);
        xSemaphoreGive(homingCommandMutex);
        return result && stillCurrent;
    }
    return false;
}

bool commitMotorHomingState(MotorHomingTicket ticket, MotorHomingAction action, void *context) {
    portENTER_CRITICAL(&guardMux);
    const bool result = ticketCurrentLocked(ticket) &&
        homingSession.commit(ticket, [&] { return action(context); });
    portEXIT_CRITICAL(&guardMux);
    return result;
}

bool motorSafetyBeginHomingPass(uint8_t pass, MotorHomingTicket ticket) {
    if (!motorHomingTicketCurrent(ticket)) return false;
    if (!required()) return motorHomingTicketCurrent(ticket) && motorSafetyCanEnableOutputs();
    if (pass == 1) {
        if (!runMotorHomingCommand(ticket, [] { disableNow(); return true; })) return false;
        if (ossmHatDetected() &&
            (!serialHardwareStopVerified.load() || !prepareSerialDisabled(&ticket)))
            return false;
    }
    // A requested new home may follow an operator-aborted first pass. A real
    // latched fault is never cleared by this path.
    portENTER_CRITICAL(&guardMux);
    if (ticketCurrentLocked(ticket) && pass == 1 &&
        guard.fault() == motor_guard::Fault::None) {
        guard.resetAfterReview();
        deadlineArmed = false;
        published.ready = false;
        publishCoreLocked(millis());
    }
    portEXIT_CRITICAL(&guardMux);
    const uint32_t started = millis();
    while (!getMotorSafetyStatus().ready && millis() - started < 3000) {
        if (!motorHomingTicketCurrent(ticket) || getMotorSafetyStatus().latched) return false;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (!motorHomingTicketCurrent(ticket) || !getMotorSafetyStatus().ready) return false;
    if (!lockPersistence()) return false;
    portENTER_CRITICAL(&guardMux);
    const bool alreadyArmed = guard.armed();
    const bool hasFault = !ticketCurrentLocked(ticket);
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
    const bool result = ticketCurrentLocked(ticket) &&
        guard.arm(now) && guard.beginHomingPass(pass, 25.0f, now);
    if (result && !alreadyArmed)
        guardHeartbeatUs = static_cast<uint32_t>(esp_timer_get_time());
    deadlineArmed = result && guard.armed() && !deadlineTripped;
    publishCoreLocked(now);
    published.ready = guard.ready(now);
    portEXIT_CRITICAL(&guardMux);
    xSemaphoreGive(persistenceMutex);
    return result;
}

bool motorSafetyEndHomingPass(MotorHomingTicket ticket) {
    if (!motorHomingTicketCurrent(ticket)) return false;
    if (!required()) return motorHomingTicketCurrent(ticket) && motorSafetyCanEnableOutputs();
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
        const bool fault = !ticketCurrentLocked(ticket);
        stopped = fresh && guard.armed() &&
            std::abs(guard.speedMmS()) <= 2.0 && sample.currentAmps < 1.5f;
        portEXIT_CRITICAL(&guardMux);
        if (fault) return false;
        if (stopped) break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (!stopped) return false;
    portENTER_CRITICAL(&guardMux);
    const bool result = ticketCurrentLocked(ticket) && guard.endHomingPass(millis());
    publishCoreLocked(millis());
    portEXIT_CRITICAL(&guardMux);
    return result;
}

bool motorSafetyBeginHomingMotion(MotorHomingTicket ticket) {
    if (!motorHomingTicketCurrent(ticket)) return false;
    if (!required()) return motorHomingTicketCurrent(ticket) && motorSafetyCanEnableOutputs();
    OssmHatTelemetry sample;
    if (!readOssmHatTelemetry(sample)) return false;
    const uint32_t now = millis();
    if (static_cast<uint32_t>(now - sample.updatedAtMs) > 100 ||
        !controlsReady()) return false;
    portENTER_CRITICAL(&guardMux);
    // The homing task can see a new contact frame before the independent
    // observer's next tick. Ingest that frame before checking its marker.
    // Older/duplicate snapshots do not refresh any guard evidence. Only the
    // independent task refreshes the hardware deadline heartbeat.
    if (ticketCurrentLocked(ticket))
        guard.observe({sample.sequence, sample.updatedAtMs, sample.continuity,
                       sample.positionCounts, sample.currentAmps,
                       sample.voltageVolts, sample.alarm}, now);
    const bool result = ticketCurrentLocked(ticket) && guard.beginHomingMotion(millis());
    publishCoreLocked(millis());
    portEXIT_CRITICAL(&guardMux);
    return result;
}

bool motorSafetyBeginNormalOperation(MotorHomingTicket ticket) {
    if (!motorHomingTicketCurrent(ticket)) return false;
    if (!required()) return motorHomingTicketCurrent(ticket) && motorSafetyCanEnableOutputs();
    portENTER_CRITICAL(&guardMux);
    const bool result = ticketCurrentLocked(ticket) && guard.beginNormalOperation(millis());
    publishCoreLocked(millis());
    portEXIT_CRITICAL(&guardMux);
    return result;
}

bool motorSafetyBeginNormalOperation(uint64_t expectedRevision) {
    if (!required()) return motorSafetyEventCurrent(expectedRevision) &&
                            motorSafetyCanEnableOutputs();
    const bool referenced = motorPositionUsesRs485();
    SafetyControlOwner owner{expectedRevision};
    portENTER_CRITICAL(&guardMux);
    const bool own = controlOwnerLocked(owner) && !homingSession.active() &&
                     !serialProbeClaimed.load();
    const bool calibrated = calibration.isHomed && guard.boundsValid() && referenced;
    if (own && !calibrated) calibration.isHomed = false;
    const bool allowed = own && calibrated && serialHardwareStopVerified.load();
    const bool armed = guard.armed();
    bool result = false;
    if (allowed && armed) {
        result = guard.beginNormalOperation(millis());
        publishCoreLocked(millis());
    }
    portEXIT_CRITICAL(&guardMux);
    if (!allowed || armed) return result;

    auto rearmCurrent = [&] {
        portENTER_CRITICAL(&guardMux);
        const bool owned = controlOwnerLocked(owner) &&
            !homingSession.active() && !serialProbeClaimed.load();
        if (owned && !guard.boundsValid()) calibration.isHomed = false;
        const bool current = owned &&
            calibration.isHomed && guard.boundsValid() &&
            serialHardwareStopVerified.load();
        portEXIT_CRITICAL(&guardMux);
        return current;
    };
    // Menu/normal Stop already requested output-off. The preparation helper
    // waits for readback and stationary feedback without reissuing that stop.
    if (!rearmCurrent() || !prepareSerialDisabled(nullptr, serialMaximumRpm,
            serialAccelerationRpmPerSec, false, &expectedRevision)) return false;
    const auto prepared = getOssmHatPositionStatus();
    portENTER_CRITICAL(&guardMux);
    const bool preparedCurrent = controlOwnerLocked(owner) &&
        !homingSession.active() && !serialProbeClaimed.load() &&
        prepared.prepared && !prepared.faulted && prepared.session == serialSession;
    if (preparedCurrent) owner.session = serialSession;
    portEXIT_CRITICAL(&guardMux);
    if (!preparedCurrent) return false;
    const uint32_t started = millis();
    while (!getMotorSafetyStatus().ready && millis() - started < 3000) {
        if (!rearmCurrent()) return false;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    const bool finalReference = motorPositionUsesRs485();
    if (!getMotorSafetyStatus().ready || !rearmCurrent() || !finalReference ||
        !lockPersistence()) {
        if (!finalReference) {
            portENTER_CRITICAL(&guardMux);
            if (controlOwnerLocked(owner)) calibration.isHomed = false;
            portEXIT_CRITICAL(&guardMux);
        }
        return false;
    }
    portENTER_CRITICAL(&guardMux);
    const bool current = controlOwnerLocked(owner) &&
        !homingSession.active() && !serialProbeClaimed.load() &&
        calibration.isHomed && guard.boundsValid() && serialHardwareStopVerified.load();
    const bool alreadyArmed = guard.armed();
    if (current && alreadyArmed) {
        result = guard.beginNormalOperation(millis());
        publishCoreLocked(millis());
    }
    portEXIT_CRITICAL(&guardMux);
    if (!current || alreadyArmed) {
        xSemaphoreGive(persistenceMutex);
        return result;
    }
    // The persistence mutex excludes other arm transitions. No flash write
    // is performed with an armed guard or while holding guard/command locks.
    Preferences prefs;
    bool stored = false;
    if (prefs.begin("motorGuard", false)) {
        stored = prefs.putBool("requireHat", true) != 0 &&
                 prefs.putBool("armed", true) != 0;
        prefs.end();
    }
    if (!stored) { xSemaphoreGive(persistenceMutex); return false; }
    rememberedHat.store(true);
    const bool stillReferenced = motorPositionUsesRs485();
    const uint32_t now = millis();
    portENTER_CRITICAL(&guardMux);
    if (controlOwnerLocked(owner) && !stillReferenced) calibration.isHomed = false;
    result = controlOwnerLocked(owner) && !homingSession.active() &&
        !serialProbeClaimed.load() && calibration.isHomed &&
        guard.boundsValid() && stillReferenced && serialHardwareStopVerified.load() &&
        guard.arm(now) && guard.beginNormalOperation(now);
    if (result) guardHeartbeatUs = static_cast<uint32_t>(esp_timer_get_time());
    deadlineArmed = result && guard.armed() && !deadlineTripped;
    publishCoreLocked(now);
    published.ready = guard.ready(now);
    portEXIT_CRITICAL(&guardMux);
    xSemaphoreGive(persistenceMutex);
    return result;
}

bool motorSafetyBeginNormalOperation() {
    if (!required()) return motorSafetyCanEnableOutputs();
    portENTER_CRITICAL(&guardMux);
    const bool result = !homingSession.active() && !deadlineTripped &&
        guard.beginNormalOperation(millis());
    publishCoreLocked(millis());
    portEXIT_CRITICAL(&guardMux);
    return result;
}

void disarmMotorSafety() {
    cancelAndFlushHoming(true);
    portENTER_CRITICAL(&guardMux);
    guard.disarm();
    deadlineArmed = false;
    digitalWrite(Pins::Driver::motorEnablePin, HIGH);
    publishCoreLocked(millis());
    const SafetyControlOwner owner{safetyControlRevision};
    portEXIT_CRITICAL(&guardMux);
    // The probe worker retains its own crash sentinel through UART off and
    // physical quiet. An emergency Stop must not clear it during cleanup.
    if (serialProbeClaimed.load()) return;
    if (required() && !stoppedForReview(owner)) return;
    if (!lockPersistence()) return;
    portENTER_CRITICAL(&guardMux);
    const bool mayClearMarker = controlOwnerLocked(owner) &&
        !serialProbeClaimed.load() && !guard.armed() &&
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
    cancelMotorHomingSession();
    portENTER_CRITICAL(&guardMux);
    SafetyControlOwner owner{safetyControlRevision};
    owner.requireHealthy = false;
    const auto originalFault = guard.fault();
    portEXIT_CRITICAL(&guardMux);
    if (liveArmed() || !stoppedForReview(owner) || !lockPersistence()) return false;
    // The first-arm path also owns this mutex, so it cannot arm while a
    // reviewed reset updates persistent state and clears the live latch.
    if (liveArmed() || !controlOwnerCurrent(owner)) {
        xSemaphoreGive(persistenceMutex);
        return false;
    }
    Preferences prefs;
    const bool opened = prefs.begin("motorGuard", false);
    // Keep the crash sentinel set until fault I/O is excluded and the runtime
    // latch has been cleared. NVS is never written under the command mutex.
    const bool staged = opened && prefs.putBool("armed", true) != 0;
    if (!staged || !takeControlCommands(owner)) {
        if (opened) prefs.end();
        xSemaphoreGive(persistenceMutex);
        return false;
    }
    const bool stopCleared = clearOssmHatStopAfterReview();
    portENTER_CRITICAL(&guardMux);
    const bool cleared = stopCleared && controlOwnerLocked(owner) &&
        !guard.armed() && !serialProbeClaimed.load();
    if (cleared) {
        guard.resetAfterReview();
        deadlineTripped = false;
        deadlineArmed = false;
        ++safetyControlRevision;
        publishCoreLocked(millis());
        published.ready = false;
        savedFault.store(false);
    }
    portEXIT_CRITICAL(&guardMux);
    xSemaphoreGive(homingCommandMutex);
    // First-arm remains excluded by persistenceMutex while the final durable
    // marker is cleared, after releasing the short command critical region.
    const bool stored = cleared && prefs.putUChar("fault", 0) != 0 &&
                        prefs.putBool("armed", false) != 0;
    prefs.end();
    if (cleared && !stored) {
        portENTER_CRITICAL(&guardMux);
        guard.tripFault(originalFault == motor_guard::Fault::None
            ? motor_guard::Fault::StopPathUnverified : originalFault);
        revokeHomingLocked();
        publishCoreLocked(millis());
        portEXIT_CRITICAL(&guardMux);
    }
    xSemaphoreGive(persistenceMutex);
    return stored;
}

bool prepareMotorSafetyExternalControlAfterReview() {
    cancelMotorHomingSession();
    if (liveArmed() || !lockPersistence()) return false;
    portENTER_CRITICAL(&guardMux);
    const bool permitted = !guard.armed() &&
        guard.fault() == motor_guard::Fault::None && !deadlineTripped;
    portEXIT_CRITICAL(&guardMux);
    xSemaphoreGive(persistenceMutex);
    return permitted && !serialProbeClaimed.load() && prepareSerialDisabled(nullptr);
}

serial_interlock_probe::Status getMotorSerialInterlockProbeStatus() {
    portENTER_CRITICAL(&guardMux);
    const auto result = serialProbe.status();
    portEXIT_CRITICAL(&guardMux);
    return result;
}

namespace {
void serialInterlockProbeWorker(void *) {
    bool started = false;
    bool success = false;
    bool markerStored = false;
    uint32_t initialContinuity = 0;
    auto cancelled = [] { return serialProbeCancelled.load(); };
    do {
        if (cancelled() || !serialOutputOff() || !stoppedForReview()) break;
        if (!lockPersistence()) break;
        portENTER_CRITICAL(&guardMux);
        const bool healthy = !guard.armed() && !deadlineTripped &&
            guard.fault() == motor_guard::Fault::None && !cancelled();
        portEXIT_CRITICAL(&guardMux);
        if (healthy) {
            Preferences prefs;
            if (prefs.begin("motorGuard", false)) {
                markerStored = prefs.putBool("requireHat", true) != 0 &&
                    prefs.putBool("armed", true) != 0;
                prefs.end();
                if (markerStored) rememberedHat.store(true);
            }
        }
        xSemaphoreGive(persistenceMutex);
        if (!markerStored || cancelled() ||
            !prepareSerialDisabled(nullptr, 1, 15, true)) break;
        OssmHatTelemetry initial;
        const auto hat = getOssmHatStatus();
        if (cancelled() || !readOssmHatTelemetry(initial)) break;
        const uint32_t now = millis();
        if (now - initial.updatedAtMs > 100 || !serialOutputOff()) break;
        portENTER_CRITICAL(&guardMux);
        started = serialProbe.start(initial.positionCounts, initial.updatedAtMs, !cancelled());
        if (started) {
            serialProbe.observe({initial.sequence, initial.updatedAtMs,
                initial.continuity, initial.positionCounts, initial.currentAmps,
                initial.voltageVolts, hat.sampleRateHz, initial.alarm}, millis());
            started = serialProbe.status().phase == serial_interlock_probe::Phase::Prepared;
        }
        if (started) {
            initialContinuity = initial.continuity;
            serialProbeContinuity = initial.continuity;
            // Establish GPIO LOW only while serial output is readback-disabled.
            // This makes a subsequent HIGH assertion a real tested transition.
            GPIO.out_w1tc = 1UL << Pins::Driver::motorEnablePin;
            guardHeartbeatUs = static_cast<uint32_t>(esp_timer_get_time());
            deadlineArmed = true;
        }
        const uint64_t session = serialProbeSession;
        const int32_t target = serialProbe.status().targetCounts;
        portEXIT_CRITICAL(&guardMux);
        if (!started || !requestOssmHatEnablePositionControl(session)) break;
        const uint32_t enableStarted = millis();
        bool enabled = false;
        while (!cancelled() && millis() - enableStarted < 500) {
            const auto position = getOssmHatPositionStatus();
            if (position.faulted || position.session != session) break;
            if (position.enabled) { enabled = true; break; }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        if (!enabled || cancelled()) break;
        portENTER_CRITICAL(&guardMux);
        const bool targetAllowed = serialProbe.markEnabled(millis()) &&
            serialProbe.markTargetWritten(target, millis());
        portEXIT_CRITICAL(&guardMux);
        if (!targetAllowed ||
            !queueOssmHatPositionTarget(session, 1, target, 1, 15, 100)) break;
        const uint32_t observationStarted = millis();
        while (millis() - observationStarted < 3500) {
            const auto probe = getMotorSerialInterlockProbeStatus();
            const auto position = getOssmHatPositionStatus();
            if (probe.finished) { success = probe.passed; break; }
            // Expected transport cancellation is the mandatory output-off
            // action. Keep the independent monitor alive through its fresh
            // readback and six-sample final quiet proof.
            if (probe.failure == serial_interlock_probe::Failure::None &&
                (cancelled() || (position.faulted &&
                probe.phase != serial_interlock_probe::Phase::AwaitingSerialDisable &&
                probe.phase != serial_interlock_probe::Phase::ProvingFinalQuiet))) {
                portENTER_CRITICAL(&guardMux);
                serialProbe.abort(cancelled() ? serial_interlock_probe::Failure::Cancelled
                    : serial_interlock_probe::Failure::SafetyFault, millis());
                portEXIT_CRITICAL(&guardMux);
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    } while (false);
    // Every exit cuts GPIO and requests serial output-off. There is no retry,
    // reverse/retreat, arbitrary target or automatic home after this probe.
    if (!success) {
        portENTER_CRITICAL(&guardMux);
        if (!started) serialProbe.start(0, millis(), false);
        serialProbe.abort(cancelled() ? serial_interlock_probe::Failure::Cancelled
                                     : serial_interlock_probe::Failure::SafetyFault,
                          millis());
        portEXIT_CRITICAL(&guardMux);
    }
    // A worker-side cleanup assertion can never be attributed as GPIO proof:
    // failure was latched first, or the independent monitor already completed
    // the entire successful proof including UART disable and physical quiet.
    GPIO.out_w1ts = 1UL << Pins::Driver::motorEnablePin;
    cancelOssmHatPositionControl();
    bool cleanupVerified = false;
    OssmHatTelemetry cleanupSample{};
    const uint32_t cleanupStarted = millis();
    while (millis() - cleanupStarted < 3500) {
        const auto probe = getMotorSerialInterlockProbeStatus();
        const auto stop = getOssmHatEmergencyStopStatus();
        if (probe.finished && stop.complete && stop.verified && stoppedForReview()) {
            const auto hat = getOssmHatStatus();
            if (readOssmHatTelemetry(cleanupSample) && serialOutputOff() &&
                static_cast<uint32_t>(millis() - cleanupSample.updatedAtMs) <= 100 &&
                hat.startupReady && !hat.passive && hat.baudRate == 115200 &&
                hat.sampleRateHz >= 45 && hat.sampleRateHz <= 65 &&
                cleanupSample.currentAmps >= 0 && cleanupSample.currentAmps <= 0.25f &&
                cleanupSample.voltageVolts >= 20 && cleanupSample.voltageVolts <= 36 &&
                (cleanupSample.alarm == 0 || cleanupSample.alarm == 0x10) &&
                digitalRead(Pins::Driver::motorEnablePin) == HIGH) {
                cleanupVerified = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Normal callbacks remain blocked by the claim through every stop,
    // read-only quiet check and persistent-marker transition. Do not call the
    // global disarm path here: it could cancel this probe or a newly admitted
    // Home after the claim was released.
    const auto completed = getMotorSerialInterlockProbeStatus();
    const bool cleanCancellation = cancelled() &&
        (completed.failure == serial_interlock_probe::Failure::Cancelled ||
         completed.failure == serial_interlock_probe::Failure::None ||
         (!started && !markerStored));
    success = success && completed.passed && cleanupVerified && !cancelled();
    portENTER_CRITICAL(&guardMux);
    invalidateSerialLocked();
    deadlineArmed = false;
    guard.disarm();
    serialHardwareStopVerified = false;
    serialProofContinuity = 0;
    if (!cleanupVerified || (!success && !cleanCancellation))
        guard.tripFault(motor_guard::Fault::StopPathUnverified);
    const bool mayClearMarker = markerStored && completed.finished &&
        !completed.active && cleanupVerified && (success || cleanCancellation) &&
        guard.fault() == motor_guard::Fault::None && !deadlineTripped;
    portEXIT_CRITICAL(&guardMux);

    bool markerCleared = false;
    if (mayClearMarker && lockPersistence()) {
        // The actual probe is finished and output-off plus fresh physical
        // stationarity have been independently verified. The claim still
        // blocks all normal activation while cache/flash is unavailable.
        const bool stillQuiet = stoppedForReview() && serialOutputOff();
        portENTER_CRITICAL(&guardMux);
        const bool stillPermitted = stillQuiet && !guard.armed() &&
            !serialProbe.status().active && !deadlineArmed &&
            guard.fault() == motor_guard::Fault::None && !deadlineTripped;
        portEXIT_CRITICAL(&guardMux);
        if (stillPermitted) {
            Preferences prefs;
            if (prefs.begin("motorGuard", false)) {
                markerCleared = prefs.putBool("armed", false) != 0 &&
                    !prefs.getBool("armed", true);
                prefs.end();
            }
        }
        xSemaphoreGive(persistenceMutex);
    }

    // Publish proof from fresh physical feedback, never from an old status
    // snapshot or a successful write acknowledgement alone.
    OssmHatTelemetry finalSample{};
    const auto finalHat = getOssmHatStatus();
    const bool finalFresh = cleanupVerified && markerCleared &&
        readOssmHatTelemetry(finalSample) && serialOutputOff() &&
        static_cast<uint32_t>(millis() - finalSample.updatedAtMs) <= 100 &&
        finalHat.startupReady && !finalHat.passive && finalHat.baudRate == 115200 &&
        finalHat.sampleRateHz >= 45 && finalHat.sampleRateHz <= 65 &&
        finalSample.continuity == initialContinuity && initialContinuity != 0 &&
        finalSample.currentAmps >= 0 && finalSample.currentAmps <= 0.25f &&
        finalSample.voltageVolts >= 20 && finalSample.voltageVolts <= 36 &&
        (finalSample.alarm == 0 || finalSample.alarm == 0x10) &&
        std::abs(static_cast<int64_t>(finalSample.positionCounts) -
                 cleanupSample.positionCounts) <= serial_interlock_probe::Gate::quietRangeCounts &&
        digitalRead(Pins::Driver::motorEnablePin) == HIGH;
    portENTER_CRITICAL(&guardMux);
    const bool proof = success && finalFresh && !cancelled() &&
        serialProbe.status().passed && guard.fault() == motor_guard::Fault::None &&
        !deadlineTripped;
    serialHardwareStopVerified = proof;
    serialProofContinuity = proof ? finalSample.continuity : 0;
    if ((!proof && !cleanCancellation) || !cleanupVerified ||
        (markerStored && mayClearMarker && !markerCleared))
        guard.tripFault(motor_guard::Fault::StopPathUnverified);
    serialProbeSession = 0;
    // A dead independent monitor cannot finish the gate. Keep the claim and
    // crash marker latched for review instead of opening a new activation.
    if (completed.finished) serialProbeClaimed = false;
    publishCoreLocked(millis());
    portEXIT_CRITICAL(&guardMux);
    serialProbeTask = nullptr;
    vTaskDelete(nullptr);
}
}  // namespace

bool requestMotorSerialInterlockProbe(bool reviewedPositiveCorridor) {
    if (!reviewedPositiveCorridor || !ossmHatDetected()) return false;
    const auto hat = getOssmHatStatus();
    if (!hat.startupReady || hat.passive || hat.baudRate != 115200 ||
        hat.ageMs > 100 || hat.sampleRateHz < 45 || hat.sampleRateHz > 65 ||
        !serialOutputOff()) return false;
    portENTER_CRITICAL(&guardMux);
    const bool allowed = !guard.armed() && !homingSession.active() &&
        !serialProbeClaimed.load() && !deadlineTripped &&
        guard.fault() == motor_guard::Fault::None &&
        serialProbe.status().phase == serial_interlock_probe::Phase::Idle;
    if (allowed) {
        serialProbeClaimed = true;
        ++safetyControlRevision;
        published.controlRevision = safetyControlRevision;
        serialProbeCancelled = false;
        serialProbeReadbackDelivered = false;
        serialHardwareStopVerified = false;
    }
    portEXIT_CRITICAL(&guardMux);
    if (!allowed) return false;
    const bool created = xTaskCreatePinnedToCore(serialInterlockProbeWorker,
        "serialStopProbe", 6144, nullptr, configMAX_PRIORITIES - 3,
        &serialProbeTask, 1) == pdPASS;
    if (!created) { serialProbeClaimed = false; return false; }
    return true;
}
