#include "motorSafety.h"

#include "services/ossmHat.h"

#if OSSM_HAT_SERIAL_BACKEND

#include <Arduino.h>
#include <Preferences.h>
#include <atomic>
#include <driver/gpio.h>
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
#include "services/ossmHatEmergency.h"
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
// High-level permission only. The independent UART module owns TG1/T1 and
// its separate output-may-be-on obligation survives software disarm.
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
std::atomic_bool serialGpioStopVerified{false};
std::atomic_bool serialTimerStopVerified{false};
uint32_t serialTimerProofEpoch = 0;
enum class TimerOperation : uint8_t { None, OffPreflight, CachePreflight, Finite };
std::atomic<TimerOperation> serialTimerOperation{TimerOperation::None};
serial_timer_probe::Gate serialTimerProbe;
uint32_t serialTimerInitialEpoch = 0, serialTimerInitialRawOffWrites = 0;
OssmHatTxAudit serialTimerCheckpointAudit;
bool serialTimerCheckpointRequested = false;
bool serialTimerRestorePending = false;
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

bool independentSerialStopVerified() {
    return serialGpioStopVerified.load() || serialTimerStopVerified.load();
}

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
    serialGpioStopVerified = false;
    if (guard.fault() != motor_guard::Fault::None || deadlineTripped) {
        serialTimerStopVerified = false;
        if (serialProbeClaimed.load() && serialTimerOperation.load() != TimerOperation::None)
            ossmHatEmergencyCancelProcedure();
        ossmHatEmergencyFault();
    }
    GPIO.out_w1ts = 1UL << Pins::Driver::motorEnablePin;
}


bool required() {
#ifdef OSSM_HAT_REQUIRE_SAFETY
    return true;
#else
    return rememberedHat || ossmHatDetected();
#endif
}

void disableNow() {
    if (ossmHatDetected()) ossmHatEmergencyStopRequested();
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
        if (required()) ossmHatEmergencyFault();
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
    portENTER_CRITICAL(&guardMux);
    const bool idle = !guard.armed() && !serialProbeClaimed.load();
    portEXIT_CRITICAL(&guardMux);
    // A successful restore intentionally cancels its old position session.
    // Fresh disabled idle can be ready for a NEW reviewed home; this does not
    // authorize enable/target writes or reuse the cancelled session.
    const bool quietIdle = idle && (control.outputEnable & 1U) == 0 && !position.enabled &&
        (!position.faulted || position.fault == OssmHatPositionFault::Cancelled);
    return control.modbusEnable == 1 && independentSerialStopVerified() &&
        (quietIdle || (position.prepared && !position.faulted &&
           (((control.outputEnable & 1U) == 0 && !position.enabled) ||
            ((control.outputEnable & 1U) != 0 && position.enabled) || serialEnabling.load())));
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
        const auto timer = serialTimerProbe.status();
        const auto timerOperation = serialTimerOperation.load();
        const bool gpioPhase = timerOperation == TimerOperation::None && probe.active &&
            (probe.phase == serial_interlock_probe::Phase::Prepared ||
             probe.phase == serial_interlock_probe::Phase::Moving);
        const bool timerPhase = timerOperation == TimerOperation::Finite && timer.active &&
            (timer.phase == serial_timer_probe::Phase::Prepared ||
             timer.phase == serial_timer_probe::Phase::Moving);
        const bool probeCommand = serialProbeClaimed.load() &&
            !serialProbeCancelled.load() && session == serialProbeSession &&
            (gpioPhase || timerPhase) && physicalFresh &&
            physical.continuity == serialProbeContinuity;
        const bool enabling = (probeCommand &&
            ((gpioPhase && probe.phase == serial_interlock_probe::Phase::Prepared) ||
             (timerPhase && timer.phase == serial_timer_probe::Phase::Prepared))) ||
            (!serialProbeClaimed.load() && serialEnabling.load());
        const bool outputAuthorized = enabling ||
            ((control.outputEnable & 1U) != 0 && position.enabled &&
             !position.faulted && position.session == session);
        allowed = allowed && deadlineArmed &&
            ((guard.armed() && independentSerialStopVerified() && physicalFresh &&
              physical.continuity == serialProofContinuity &&
              !serialProbeClaimed.load()) || probeCommand) &&
            outputAuthorized && digitalRead(Pins::Driver::motorEnablePin) == LOW;
    } else {
        const bool ownProbeSetup = serialProbeClaimed.load() &&
            !serialProbeCancelled.load() && serialPreparing.load() &&
            session == serialProbeSession &&
            (serialTimerOperation.load() == TimerOperation::None ||
             serialTimerOperation.load() == TimerOperation::Finite);
        allowed = allowed && !guard.armed() &&
            (!serialProbeClaimed.load() || ownProbeSetup) &&
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
bool freshStopForPersistence(const OssmHatTelemetry *quietProof = nullptr) {
    // Read-only electrical/freshness gate. The caller separately establishes
    // physical quiet; software disarm alone says nothing about serial output.
    const auto hat = getOssmHatStatus();
    if (getOssmHatEmergencyStatus().outputMayBeOn) return false;
    OssmHatTelemetry physical;
    OssmHatControlSnapshot controls;
    if (!readOssmHatTelemetry(physical) ||
        !readOssmHatControlSnapshot(controls)) return false;
    const uint32_t now = millis();
    if (!hat.detected || !hat.startupReady || hat.passive || hat.baudRate != 115200 ||
        !(hat.sampleRateHz >= 30) ||
        static_cast<uint32_t>(now - physical.updatedAtMs) > 100 ||
        static_cast<uint32_t>(now - controls.updatedAtMs) > 100 ||
        controls.continuity != physical.continuity || physical.continuity == 0 ||
        controls.modbusEnable != 1 || (controls.outputEnable & 1U) != 0 ||
        !(physical.currentAmps >= 0 && physical.currentAmps < 0.5f) ||
        !(physical.voltageVolts >= 20 && physical.voltageVolts <= 36) ||
        (physical.alarm != 0 && physical.alarm != 0x10) ||
        digitalRead(Pins::Driver::motorEnablePin) != HIGH) return false;
    return quietProof == nullptr ||
        (quietProof->continuity == physical.continuity &&
         static_cast<uint32_t>(now - quietProof->updatedAtMs) <= 100 &&
         std::abs(static_cast<int64_t>(physical.positionCounts) -
                  quietProof->positionCounts) <= serial_interlock_probe::Gate::quietRangeCounts);
}

void persistFaultIfNeeded() {
    if (savedFault.load()) return;
    portENTER_CRITICAL(&guardMux);
    SafetyControlOwner owner{safetyControlRevision};
    owner.requireHealthy = false;
    const auto fault = guard.fault();
    const bool needed = !guard.armed() && !serialProbeClaimed.load() &&
        !deadlineArmed && fault != motor_guard::Fault::None;
    portEXIT_CRITICAL(&guardMux);
    if (!needed || !freshStopForPersistence()) return;
    if (!lockPersistence()) return;
    // Keep activation/reset excluded, but never hold guardMux or the command
    // mutex during a physical quiet wait or NVS I/O. A changed intent cancels
    // the read-only proof and leaves the pre-arm crash marker untouched.
    OssmHatTelemetry quietProof;
    const bool stopped = stoppedForReview(owner, &quietProof) &&
        freshStopForPersistence(&quietProof);
    portENTER_CRITICAL(&guardMux);
    const bool canWrite = stopped && controlOwnerLocked(owner) &&
        guard.fault() == fault && !guard.armed() &&
        !serialProbeClaimed.load() && !deadlineArmed && !savedFault.load();
    portEXIT_CRITICAL(&guardMux);
    if (canWrite) {
        Preferences prefs;
        if (prefs.begin("motorGuard", false)) {
            const bool stored = prefs.putUChar("fault", static_cast<uint8_t>(fault)) != 0;
            // A Stop can invalidate ownership while flash is busy. It must
            // not let this older writer clear a newer intent's crash marker.
            const bool stillStopped = stored && freshStopForPersistence(&quietProof);
            portENTER_CRITICAL(&guardMux);
            const bool stillOwned = stillStopped && controlOwnerLocked(owner) &&
                guard.fault() == fault && !guard.armed() &&
                !serialProbeClaimed.load() && !deadlineArmed;
            portEXIT_CRITICAL(&guardMux);
            if (stillOwned) {
                const bool markerCleared = prefs.putBool("armed", false) != 0 &&
                    !prefs.getBool("armed", true);
                portENTER_CRITICAL(&guardMux);
                const bool persistedCurrent = markerCleared && controlOwnerLocked(owner) &&
                    guard.fault() == fault && !guard.armed() &&
                    !serialProbeClaimed.load() && !deadlineArmed;
                portEXIT_CRITICAL(&guardMux);
                if (persistedCurrent) savedFault.store(true);
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
        ossmHatEmergencyGuardHeartbeat();
        const bool needed = required();
        const auto hat = getOssmHatStatus();
        OssmHatTelemetry sample;
        const bool haveSample = readOssmHatTelemetry(sample);
        const bool control = controlsReady();
        OssmHatControlSnapshot serialControl;
        const bool haveSerialControl = readOssmHatControlSnapshot(serialControl);
        const bool serialMode = haveSerialControl && serialControl.modbusEnable == 1;
        const auto txAudit = getOssmHatTxAudit();
        const auto stopSnapshot = getOssmHatEmergencyStopStatus();
        const uint32_t emergencyBeforeMs = millis();
        const auto emergency = getOssmHatEmergencyStatus();
        const uint32_t emergencyEpochAfter = getOssmHatEmergencyStatus().epoch;
        const uint32_t emergencyAfterMs = millis();
        const uint32_t now = emergencyAfterMs;
        const uint32_t rawIrqAgeUs = emergency.nowUs - emergency.tripAtUs;
        const bool emergencyClockBound = emergencyAfterMs - emergencyBeforeMs <= 2 &&
            emergencyEpochAfter == emergency.epoch && rawIrqAgeUs <= 100000;
        // Raw TG1/T1 and millis have different epochs. Translate the age only;
        // floor the earliest event bound conservatively by at most1ms plus
        // the measured<=2ms status-read bracket. Never subtract raw epochs.
        const uint32_t irqAtMs = emergencyBeforeMs - (rawIrqAgeUs + 999U) / 1000U;
        const bool fresh = haveSample &&
            static_cast<uint32_t>(now - sample.updatedAtMs) <= 100;
        const bool freshSerialControl = haveSerialControl &&
            static_cast<uint32_t>(now - serialControl.updatedAtMs) <= 100;
        const bool freshSerialMode = serialMode && freshSerialControl;
        const bool exclusive = !hat.passive;
        bool armed;
        bool fault;
        bool probeDisableDue = false;
        bool timerTripDue = false, timerRestoreDue = false, timerEmergencyDue = false;
        portENTER_CRITICAL(&guardMux);
        const bool timerAuthorityLost = serialTimerStopVerified.load() &&
            (!emergency.available || !emergency.irqInstalled || emergency.isolated ||
             emergency.failed || !emergency.qualificationPassed ||
             emergency.epoch != serialTimerProofEpoch || emergency.restoredEpoch != emergency.epoch);
        if (independentSerialStopVerified() &&
            (!hat.detected || !fresh || hat.passive || hat.baudRate != 115200 ||
             !freshSerialMode || sample.continuity != serialProofContinuity || timerAuthorityLost)) {
            serialGpioStopVerified = false;
            serialTimerStopVerified = false;
            if (guard.armed()) guard.tripFault(motor_guard::Fault::StopPathUnverified);
        }
        const bool unexpectedEmergency = emergency.isolated &&
            emergency.cause != OssmHatEmergencyCause::Qualification &&
            emergency.cause != OssmHatEmergencyCause::OffOnlyPreflight &&
            emergency.cause != OssmHatEmergencyCause::CacheOffPreflight;
        if (unexpectedEmergency) deadlineTripped = true;
        if (emergency.failed) guard.tripFault(motor_guard::Fault::StopPathUnverified);
        if (fresh) {
            guard.observe({sample.sequence, sample.updatedAtMs, sample.continuity,
                           sample.positionCounts, sample.currentAmps,
                           sample.voltageVolts, sample.alarm}, now);
        }
        if (deadlineTripped)
            guard.tripFault(motor_guard::Fault::ObserverDelayed);
        const auto timerOperation = serialTimerOperation.load();
        if (serialProbeClaimed.load() &&
            (timerOperation == TimerOperation::OffPreflight || timerOperation == TimerOperation::CachePreflight)) {
            // These procedures never authorize enable. Planned driver teardown
            // may have no sample, but any fresh unhealthy/enabled frame fails.
            if (serialProbeCancelled.load() || emergency.failed ||
                (fresh && (!(sample.currentAmps >= 0 && sample.currentAmps <= .25f) ||
                           !(sample.voltageVolts >= 20 && sample.voltageVolts <= 36) ||
                           sample.alarm != 0)) ||
                (freshSerialControl && (serialControl.modbusEnable != 1 ||
                                       (serialControl.outputEnable & 1U) != 0)))
                guard.tripFault(motor_guard::Fault::StopPathUnverified);
        }
        if (serialProbeClaimed.load() && serialTimerOperation.load() == TimerOperation::None) {
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
        if (serialProbeClaimed.load() && serialTimerOperation.load() == TimerOperation::Finite) {
            auto timer = serialTimerProbe.status();
            const bool restoring = timer.phase == serial_timer_probe::Phase::Restoring;
            if (timer.active && fresh) {
                // Even the deliberately off/quiet teardown does not excuse a
                // received electrical/envelope fault. First new-epoch frames
                // may still be rebuilding rate; no such frame advances proof.
                if (!(sample.currentAmps >= 0 && sample.currentAmps <= .25f))
                    serialTimerProbe.abort(serial_timer_probe::Failure::Current);
                else if (!(sample.voltageVolts >= 20 && sample.voltageVolts <= 36))
                    serialTimerProbe.abort(serial_timer_probe::Failure::Voltage);
                else if (sample.alarm != 0)
                    serialTimerProbe.abort(serial_timer_probe::Failure::Alarm);
                else if (std::abs(static_cast<int64_t>(sample.positionCounts)-timer.originCounts) >
                         serial_timer_probe::Gate::envelopeCounts)
                    serialTimerProbe.abort(serial_timer_probe::Failure::Envelope);
                else if (restoring && std::abs(guard.speedMmS()) > 30)
                    serialTimerProbe.abort(serial_timer_probe::Failure::Speed);
            }
            if (timer.active && (serialProbeCancelled.load() ||
                guard.fault() != motor_guard::Fault::None || emergency.failed ||
                (restoring && (emergency.outputMayBeOn ||
                    (freshSerialControl && (serialControl.modbusEnable != 1 ||
                                           (serialControl.outputEnable & 1U) != 0)))) ||
                (!restoring && (hat.passive || !hat.startupReady || hat.baudRate != 115200 ||
                                !fresh || !freshSerialMode))))
                serialTimerProbe.abort(serialProbeCancelled.load() ? serial_timer_probe::Failure::Cancelled
                                                                 : serial_timer_probe::Failure::SafetyFault);
            if (serialTimerCheckpointRequested) {
                const bool measuring = timer.phase == serial_timer_probe::Phase::AwaitingIrq ||
                    timer.phase == serial_timer_probe::Phase::AwaitingOff ||
                    timer.phase == serial_timer_probe::Phase::PassiveSettling;
                const bool changed = txAudit.absoluteTargets != serialTimerCheckpointAudit.absoluteTargets ||
                    txAudit.outputEnables != serialTimerCheckpointAudit.outputEnables ||
                    txAudit.setupWrites != serialTimerCheckpointAudit.setupWrites ||
                    txAudit.modeWrites != serialTimerCheckpointAudit.modeWrites ||
                    (measuring && txAudit.controlWrites != serialTimerCheckpointAudit.controlWrites);
                if (changed) serialTimerProbe.abort(serial_timer_probe::Failure::UnexpectedWrite);
            }
            const bool actualIrq = emergency.cause == OssmHatEmergencyCause::Qualification &&
                emergency.epoch != serialTimerInitialEpoch && emergency.isolated &&
                emergency.ownerParked && emergency.transmitted && emergency.txDone &&
                static_cast<uint32_t>(emergency.rawOffWrites - serialTimerInitialRawOffWrites) == 1;
            if (timer.phase == serial_timer_probe::Phase::AwaitingIrq && actualIrq && emergencyClockBound)
                serialTimerProbe.markIrqStop(irqAtMs, now, true);
            timer = serialTimerProbe.status();
            const bool coherent = fresh && freshSerialMode &&
                serialControl.updatedAtMs == sample.updatedAtMs &&
                serialControl.continuity == sample.continuity &&
                (serialControl.outputEnable & 1U) == 0;
            if (timer.phase == serial_timer_probe::Phase::AwaitingOff && actualIrq &&
                emergency.echoVerified && emergency.offVerified && coherent &&
                emergency.telemetrySequence == sample.sequence &&
                emergency.telemetryContinuity == sample.continuity &&
                static_cast<uint32_t>(sample.updatedAtMs - emergency.offAtMs) < 0x80000000U) {
                serialTimerProbe.markOff({sample.sequence,sample.updatedAtMs,sample.continuity,
                    sample.positionCounts,sample.currentAmps,sample.voltageVolts,hat.sampleRateHz,sample.alarm},
                    serialControl.modbusEnable,serialControl.outputEnable,now,true);
            }
            timer = serialTimerProbe.status();
            if (timer.phase == serial_timer_probe::Phase::Restoring && emergency.restored &&
                emergency.qualificationPassed && !emergency.isolated && !emergency.restoring &&
                emergency.epoch != serialTimerInitialEpoch && emergency.restoredEpoch == emergency.epoch &&
                coherent && emergency.telemetrySequence == sample.sequence &&
                emergency.telemetryContinuity == sample.continuity) {
                serialTimerProbe.markRestored({sample.sequence,sample.updatedAtMs,sample.continuity,
                    sample.positionCounts,sample.currentAmps,sample.voltageVolts,hat.sampleRateHz,sample.alarm},
                    serialControl.modbusEnable,serialControl.outputEnable,now,true);
            }
            timer = serialTimerProbe.status();
            if (timer.serialDisabled && timer.active &&
                timer.phase != serial_timer_probe::Phase::Restoring &&
                ((freshSerialControl && (serialControl.modbusEnable != 1 ||
                                        (serialControl.outputEnable & 1U) != 0)) ||
                 digitalRead(Pins::Driver::motorEnablePin) != HIGH))
                serialTimerProbe.abort(serial_timer_probe::Failure::OffUnverified);
            const auto observePhase = serialTimerProbe.status().phase;
            const bool offPublicationPending = observePhase == serial_timer_probe::Phase::AwaitingOff && coherent;
            if (fresh && observePhase != serial_timer_probe::Phase::Restoring && !offPublicationPending &&
                (!serialTimerProbe.status().serialDisabled || coherent))
                serialTimerProbe.observe({sample.sequence,sample.updatedAtMs,sample.continuity,
                    sample.positionCounts,sample.currentAmps,sample.voltageVolts,hat.sampleRateHz,sample.alarm},now);
            serialTimerProbe.tick(now);
            for (;;) {
                const auto action = serialTimerProbe.takeAction(now);
                if (action == serial_timer_probe::Action::None) break;
                if (action == serial_timer_probe::Action::RequestTimerStop) {
                    serialTimerCheckpointAudit = txAudit;
                    serialTimerCheckpointRequested = true;
                    timerTripDue = true;
                } else if (action == serial_timer_probe::Action::RestoreTransport) {
                    serialTimerRestorePending = true;
                } else timerEmergencyDue = true;
            }
            if (serialTimerRestorePending && serialTimerProbe.status().offQuiet &&
                emergency.quietVerified && !emergency.outputMayBeOn && coherent && actualIrq) {
                timerRestoreDue = serialTimerProbe.markRestoreStarted(now);
                serialTimerRestorePending = false;
            }
            if (serialTimerProbe.status().failure != serial_timer_probe::Failure::None)
                guard.tripFault(motor_guard::Fault::StopPathUnverified);
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
            (!hat.detected || serialOutputOff() || serialGpioStopVerified.load());
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
        published.armed = guard.armed() || serialProbe.status().active || serialTimerProbe.status().active;
        deadlineArmed = needed && published.armed && !published.latched;
        portEXIT_CRITICAL(&guardMux);

        if (timerTripDue && !requestOssmHatTimerQualification()) {
            portENTER_CRITICAL(&guardMux);
            serialTimerProbe.abort(serial_timer_probe::Failure::IrqUnverified);
            portEXIT_CRITICAL(&guardMux);
            timerEmergencyDue = true;
        }
        if (timerRestoreDue && !requestOssmHatQualificationRestore()) {
            portENTER_CRITICAL(&guardMux);
            serialTimerProbe.abort(serial_timer_probe::Failure::RestoreUnverified);
            portEXIT_CRITICAL(&guardMux);
            timerEmergencyDue = true;
        }
        if (timerEmergencyDue) {
            ossmHatEmergencyCancelProcedure();
            ossmHatEmergencyFault();
            GPIO.out_w1ts = 1UL << Pins::Driver::motorEnablePin;
            requestOssmHatEmergencyStop();
        }
        if (probeDisableDue) cancelOssmHatPositionControl();
        const auto probeSnapshot = getMotorSerialInterlockProbeStatus();
        if (serialProbeClaimed.load() && serialTimerOperation.load() == TimerOperation::None &&
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
        if (state.latched) {
            enforceCurrentFaultStop();
            // Publish the visible warning before any potentially long stop
            // observation or storage wait. An ACK is not verified output-off.
            const auto currentState = getMotorSafetyStatus();
            if (currentState.latched && reported != currentState.fault) {
                reported = currentState.fault;
                calibration.isHomed = false;
                settings.speed = 0;
                settings.speedBLE = 0;
                errorState.message = motorSafetyReason();
                setHomingActive(false, false);
                setLEDStatusError();
                Serial.printf("RAD_GUARD fault=%u reason=%s; stop requested, motor power remains connected\n",
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
            persistFaultIfNeeded();
        } else {
            reported = UINT8_MAX;
            // Remembering a newly detected HAT is also an NVS write. Apply
            // the same read-only stop/quiet gate before touching storage.
            if (ossmHatDetected() && !rememberedHat.load() && freshStopForPersistence()) {
                portENTER_CRITICAL(&guardMux);
                const SafetyControlOwner owner{safetyControlRevision};
                portEXIT_CRITICAL(&guardMux);
                if (lockPersistence()) {
                    OssmHatTelemetry quietProof;
                    const bool stopped = stoppedForReview(owner, &quietProof) &&
                        freshStopForPersistence(&quietProof);
                    portENTER_CRITICAL(&guardMux);
                    const bool canWrite = stopped && controlOwnerLocked(owner) &&
                        !guard.armed() && !serialProbeClaimed.load() && !deadlineArmed;
                    portEXIT_CRITICAL(&guardMux);
                    if (canWrite) {
                        Preferences prefs;
                        if (prefs.begin("motorGuard", false)) {
                            rememberedHat.store(prefs.putBool("requireHat", true) != 0);
                            prefs.end();
                        }
                    }
                    xSemaphoreGive(persistenceMutex);
                }
            }
        }
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
    ossmHatEmergencyGuardHeartbeat();
    const auto created = xTaskCreatePinnedToCore(
        monitor, "motorSafety", 4096, nullptr, configMAX_PRIORITIES - 1,
        &guardTaskHandle, 0);
    const auto reportCreated = xTaskCreatePinnedToCore(
        report, "motorSafetyReport", 4096, nullptr, 2, nullptr, 0);
    if (created != pdPASS || reportCreated != pdPASS ||
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
    if (ossmHatDetected() && !independentSerialStopVerified())
        return "Serial motion needs reviewed UART-timer preflights and a finite stop qualification after each boot";
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
    const uint64_t session = !serialProbeClaimed.load() && independentSerialStopVerified() &&
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
    MotorSerialSafetyStatus result;
    result.selected = ossmHatDetected();
    portENTER_CRITICAL(&guardMux);
    const bool gpioProof = serialGpioStopVerified.load();
    const bool timerProof = serialTimerStopVerified.load();
    const uint32_t continuity = serialProofContinuity;
    const uint32_t timerEpoch = serialTimerProofEpoch;
    portEXIT_CRITICAL(&guardMux);
    const auto hat = getOssmHatStatus();
    const auto emergency = getOssmHatEmergencyStatus();
    OssmHatTelemetry sample;
    OssmHatControlSnapshot control;
    const bool current = readOssmHatTelemetry(sample) && readOssmHatControlSnapshot(control) &&
        sample.continuity == continuity && control.continuity == continuity && continuity != 0 &&
        millis()-sample.updatedAtMs <= 100 && millis()-control.updatedAtMs <= 100 &&
        hat.detected && hat.startupReady && !hat.passive && hat.baudRate == 115200 &&
        hat.sampleRateHz >= 30 && control.modbusEnable == 1 && emergency.irqInstalled;
    result.gpioStopVerified = gpioProof && current;
    result.uartTimerStopVerified = timerProof && current && emergency.available &&
        !emergency.isolated && !emergency.failed && emergency.qualificationPassed &&
        emergency.epoch == timerEpoch && emergency.restoredEpoch == timerEpoch;
    result.hardwareStopVerified = result.gpioStopVerified || result.uartTimerStopVerified;
    result.prepared = position.prepared;
    result.enabled = position.enabled;
    result.preparing = serialPreparing.load();
    result.maximumSpeedRpm = serialMaximumRpm;
    result.accelerationRpmPerSec = serialAccelerationRpmPerSec;
    return result;
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
        independentSerialStopVerified() && guard.armed();
    portEXIT_CRITICAL(&guardMux);
    if (!allowed) return false;
    const auto prior = getOssmHatPositionStatus();
    if (prior.session == owner.session && prior.enabled &&
        motorSafetySerialCanMove(owner.session) && controlOwnerCurrent(owner)) return true;
    if (!takeControlCommands(owner)) return false;
    portENTER_CRITICAL(&guardMux);
    const bool current = controlOwnerLocked(owner) && guard.armed() &&
        independentSerialStopVerified();
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
        serialTimerProbe.abort(serial_timer_probe::Failure::Cancelled);
        if (serialTimerOperation.load() != TimerOperation::None) ossmHatEmergencyCancelProcedure();
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
    if (ossmHatDetected()) ossmHatEmergencyStopRequested();
    portENTER_CRITICAL(&guardMux);
    ++safetyControlRevision;
    published.controlRevision = safetyControlRevision;
    if (serialProbeClaimed.load()) {
        serialProbeCancelled = true;
        serialProbe.abort(serial_interlock_probe::Failure::Cancelled, millis());
        serialTimerProbe.abort(serial_timer_probe::Failure::Cancelled);
        if (serialTimerOperation.load() != TimerOperation::None) ossmHatEmergencyCancelProcedure();
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
            (!independentSerialStopVerified() || !prepareSerialDisabled(&ticket)))
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
    const bool allowed = own && calibrated && independentSerialStopVerified();
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
            independentSerialStopVerified();
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
        calibration.isHomed && guard.boundsValid() && independentSerialStopVerified();
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
        guard.boundsValid() && stillReferenced && independentSerialStopVerified() &&
        guard.arm(now) && guard.beginNormalOperation(now);
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
    const bool needed = required();
    // Output-off stops the50ms pending-off deadline; its separate physical
    // quiet obligation may legitimately take500ms to complete. Wait outside
    // both command/guard locks, and abandon this owner on any newer intent.
    const uint32_t quietBegan = millis();
    while (needed && getOssmHatEmergencyStatus().outputMayBeOn &&
           controlOwnerCurrent(owner) && millis()-quietBegan < 1200)
        vTaskDelay(pdMS_TO_TICKS(5));
    OssmHatTelemetry quietProof;
    if (needed && (!stoppedForReview(owner,&quietProof) || !freshStopForPersistence(&quietProof))) return;
    if (!lockPersistence()) return;
    const bool stillStopped = !needed || freshStopForPersistence(&quietProof);
    portENTER_CRITICAL(&guardMux);
    const bool mayClearMarker = stillStopped && controlOwnerLocked(owner) &&
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
    const auto emergency = getOssmHatEmergencyStatus();
    if (emergency.isolated || emergency.failed || emergency.outputMayBeOn) return false;
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
    serialGpioStopVerified = false;
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
    serialGpioStopVerified = proof;
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
        serialGpioStopVerified = false;
    }
    portEXIT_CRITICAL(&guardMux);
    if (!allowed) return false;
    const bool created = xTaskCreatePinnedToCore(serialInterlockProbeWorker,
        "serialStopProbe", 6144, nullptr, configMAX_PRIORITIES - 3,
        &serialProbeTask, 1) == pdPASS;
    if (!created) { serialProbeClaimed = false; return false; }
    return true;
}


serial_timer_probe::Status getMotorSerialTimerProbeStatus() {
    portENTER_CRITICAL(&guardMux);
    const auto result = serialTimerProbe.status();
    portEXIT_CRITICAL(&guardMux);
    return result;
}

namespace {
bool timerOffSnapshot(OssmHatTelemetry &sample) {
    const auto hat = getOssmHatStatus();
    OssmHatControlSnapshot control;
    if (!readOssmHatTelemetry(sample) || !readOssmHatControlSnapshot(control)) return false;
    const uint32_t now = millis();
    return hat.detected && hat.startupReady && !hat.passive && hat.baudRate == 115200 &&
        hat.sampleRateHz >= 45 && hat.sampleRateHz <= 65 &&
        now - sample.updatedAtMs <= 100 && sample.continuity != 0 &&
        control.updatedAtMs == sample.updatedAtMs && control.continuity == sample.continuity &&
        control.modbusEnable == 1 && (control.outputEnable & 1U) == 0 &&
        sample.currentAmps >= 0 && sample.currentAmps <= .25f &&
        sample.voltageVolts >= 20 && sample.voltageVolts <= 36 &&
        sample.alarm == 0 &&
        digitalRead(Pins::Driver::motorEnablePin) == HIGH;
}

void serialTimerWorker(void *) {
    const auto operation = serialTimerOperation.load();
    const bool finite = operation == TimerOperation::Finite;
    portENTER_CRITICAL(&guardMux);
    const SafetyControlOwner owner{safetyControlRevision};
    portEXIT_CRITICAL(&guardMux);
    auto current = [&] { return !serialProbeCancelled.load() && controlOwnerCurrent(owner); };
    bool markerStored = false, success = false;
    do {
        OssmHatTelemetry before;
        if (!current() || !timerOffSnapshot(before) || !stoppedForReview(owner) ||
            !lockPersistence()) break;
        const bool stillSafe = current() && timerOffSnapshot(before) &&
            !getOssmHatEmergencyStatus().outputMayBeOn;
        if (stillSafe) {
            Preferences prefs;
            if (prefs.begin("motorGuard",false)) {
                markerStored = prefs.putBool("requireHat",true) != 0 &&
                               prefs.putBool("armed",true) != 0;
                prefs.end();
                if (markerStored) rememberedHat = true;
            }
        }
        xSemaphoreGive(persistenceMutex);
        if (!markerStored || !current()) break;
        const auto initialEmergency = getOssmHatEmergencyStatus();
        if (!finite) {
            const bool cache = operation == TimerOperation::CachePreflight;
            const auto expected = cache ? OssmHatEmergencyCause::CacheOffPreflight
                                        : OssmHatEmergencyCause::OffOnlyPreflight;
            if (!requestOssmHatEmergencyPreflight(cache)) break;
            const uint32_t began = millis();
            while (current() && millis()-began < 6000) {
                const auto emergency = getOssmHatEmergencyStatus();
                if (emergency.failed) break;
                OssmHatTelemetry after;
                const bool receipt = emergency.epoch != initialEmergency.epoch &&
                    emergency.cause == expected && emergency.restored &&
                    emergency.restoredEpoch == emergency.epoch && !emergency.restoring &&
                    !emergency.isolated && !emergency.outputMayBeOn && emergency.quietVerified &&
                    (cache ? (emergency.cacheOffPassed && emergency.cacheOffAtTrip && emergency.cacheOffAtTx)
                           : emergency.offOnlyPassed);
                if (receipt && timerOffSnapshot(after)) { success = true; break; }
                vTaskDelay(pdMS_TO_TICKS(5));
            }
            break;
        }
        if (!initialEmergency.offOnlyPassed || !initialEmergency.cacheOffPassed ||
            !prepareSerialDisabled(nullptr,1,15,true,&owner.revision) || !current()) break;
        OssmHatTelemetry initial;
        const auto hat = getOssmHatStatus();
        if (!timerOffSnapshot(initial)) break;
        portENTER_CRITICAL(&guardMux);
        bool started = controlOwnerLocked(owner) && !serialProbeCancelled.load() &&
            serialTimerProbe.start(initial.positionCounts,initial.updatedAtMs,true);
        if (started) {
            serialTimerProbe.observe({initial.sequence,initial.updatedAtMs,initial.continuity,
                initial.positionCounts,initial.currentAmps,initial.voltageVolts,hat.sampleRateHz,initial.alarm},millis());
            started = serialTimerProbe.status().phase == serial_timer_probe::Phase::Prepared;
        }
        if (started) {
            serialProbeContinuity = initial.continuity;
            serialTimerInitialEpoch = initialEmergency.epoch;
            serialTimerInitialRawOffWrites = initialEmergency.rawOffWrites;
            deadlineArmed = true;
        }
        const auto session = serialProbeSession;
        const auto target = serialTimerProbe.status().targetCounts;
        portEXIT_CRITICAL(&guardMux);
        // The independent obligation precedes every physical enable. Neither
        // GPIO nor the firmware's armed boolean is an actuator-lifetime proxy.
        if (!started || !current() || !ossmHatEmergencyMarkOutputPossible()) break;
        portENTER_CRITICAL(&guardMux);
        const bool enableAllowed = controlOwnerLocked(owner) && !serialProbeCancelled.load() &&
            serialTimerProbe.status().phase == serial_timer_probe::Phase::Prepared;
        if (enableAllowed) GPIO.out_w1tc = 1UL << Pins::Driver::motorEnablePin;
        portEXIT_CRITICAL(&guardMux);
        if (!enableAllowed || !requestOssmHatEnablePositionControl(session)) break;
        bool enabled = false;
        const uint32_t enableBegan = millis();
        while (current() && millis()-enableBegan < 500) {
            const auto position = getOssmHatPositionStatus();
            if (position.faulted || position.session != session) break;
            if (position.enabled) { enabled = true; break; }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        if (!enabled || !current()) break;
        portENTER_CRITICAL(&guardMux);
        const bool targetAllowed = controlOwnerLocked(owner) && !serialProbeCancelled.load() &&
            serialTimerProbe.markEnabled(millis()) && serialTimerProbe.markTargetWritten(target,millis());
        portEXIT_CRITICAL(&guardMux);
        if (!targetAllowed || !queueOssmHatPositionTarget(session,1,target,1,15,100)) break;
        const uint32_t began = millis();
        while (current() && millis()-began < 3500) {
            const auto gate = getMotorSerialTimerProbeStatus();
            if (gate.finished) { success = gate.passed; break; }
            const auto position = getOssmHatPositionStatus();
            // Parking deliberately revokes the old normal owner. Its Cancelled
            // status cannot abort the independent IRQ/off/quiet observation.
            if (position.faulted && (gate.phase == serial_timer_probe::Phase::Prepared ||
                                     gate.phase == serial_timer_probe::Phase::Moving)) break;
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    } while (false);

    if (!success || !current()) {
        success = false;
        portENTER_CRITICAL(&guardMux);
        if (finite) {
            if (serialTimerProbe.status().phase == serial_timer_probe::Phase::Idle)
                serialTimerProbe.start(0,millis(),false);
            serialTimerProbe.abort(serialProbeCancelled.load() ? serial_timer_probe::Failure::Cancelled
                                                            : serial_timer_probe::Failure::SafetyFault);
        }
        guard.tripFault(motor_guard::Fault::StopPathUnverified);
        invalidateSerialLocked();
        deadlineArmed = false;
        GPIO.out_w1ts = 1UL << Pins::Driver::motorEnablePin;
        publishCoreLocked(millis());
        portEXIT_CRITICAL(&guardMux);
        ossmHatEmergencyCancelProcedure();
        ossmHatEmergencyFault();
        requestOssmHatEmergencyStop();
    }
    GPIO.out_w1ts = 1UL << Pins::Driver::motorEnablePin;
    bool cleanupVerified = false;
    OssmHatTelemetry quietSample;
    const uint32_t cleanupBegan = millis();
    while (millis()-cleanupBegan < 4500) {
        const auto emergency = getOssmHatEmergencyStatus();
        if (!emergency.outputMayBeOn && (!emergency.isolated ||
            (emergency.offVerified && emergency.quietVerified)) &&
            timerOffSnapshot(quietSample) && stoppedForReview()) {
            cleanupVerified = timerOffSnapshot(quietSample);
            if (cleanupVerified) break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    portENTER_CRITICAL(&guardMux);
    invalidateSerialLocked();
    guard.disarm();
    deadlineArmed = false;
    serialGpioStopVerified = false;
    serialTimerStopVerified = false;
    serialProofContinuity = 0;
    const bool healthy = controlOwnerLocked(owner) && !serialProbeCancelled.load() &&
        guard.fault() == motor_guard::Fault::None && !deadlineTripped;
    portEXIT_CRITICAL(&guardMux);
    success = success && healthy && cleanupVerified;

    bool markerCleared = false;
    if (success && markerStored && lockPersistence()) {
        OssmHatTelemetry finalQuiet;
        const bool quiet = current() && !getOssmHatEmergencyStatus().outputMayBeOn &&
            stoppedForReview(owner,&finalQuiet) && freshStopForPersistence(&finalQuiet);
        if (quiet) {
            Preferences prefs;
            if (prefs.begin("motorGuard",false)) {
                markerCleared = prefs.putBool("armed",false) != 0 && !prefs.getBool("armed",true);
                prefs.end();
            }
        }
        xSemaphoreGive(persistenceMutex);
    }
    OssmHatTelemetry finalSample;
    const auto finalEmergency = getOssmHatEmergencyStatus();
    const bool finalFresh = success && markerCleared && timerOffSnapshot(finalSample) &&
        !finalEmergency.outputMayBeOn && !finalEmergency.failed && !finalEmergency.isolated &&
        finalEmergency.restored && finalEmergency.restoredEpoch == finalEmergency.epoch;
    portENTER_CRITICAL(&guardMux);
    const auto gate = serialTimerProbe.status();
    const bool publishedSuccess = finalFresh && controlOwnerLocked(owner) &&
        !serialProbeCancelled.load() && guard.fault() == motor_guard::Fault::None && !deadlineTripped;
    const bool proof = finite && publishedSuccess && gate.passed &&
        finalEmergency.qualificationPassed && finalSample.continuity == gate.restoredContinuity;
    serialTimerStopVerified = proof;
    if (proof) {
        serialProofContinuity = finalSample.continuity;
        serialTimerProofEpoch = finalEmergency.epoch;
    }
    if (!publishedSuccess || (finite && !proof)) guard.tripFault(motor_guard::Fault::StopPathUnverified);
    // An unverified stop retains both the claim and the durable crash sentinel.
    // A genuine fault never requests same-boot transport restoration.
    if (cleanupVerified && (!finite || gate.finished)) {
        serialProbeClaimed = false;
        serialTimerOperation = TimerOperation::None;
    }
    publishCoreLocked(millis());
    portEXIT_CRITICAL(&guardMux);
    if (!publishedSuccess || (finite && !proof)) {
        ossmHatEmergencyCancelProcedure();
        ossmHatEmergencyFault();
    }
    serialProbeTask = nullptr;
    vTaskDelete(nullptr);
}

bool requestTimerOperation(TimerOperation operation, bool reviewed) {
    if (!reviewed || !ossmHatDetected()) return false;
    const auto emergency = getOssmHatEmergencyStatus();
    const bool finite = operation == TimerOperation::Finite;
    if (!emergency.available || !emergency.irqInstalled || emergency.outputMayBeOn ||
        emergency.isolated || emergency.restoring || emergency.failed || !emergency.quietVerified ||
        (finite && (!emergency.offOnlyPassed || !emergency.cacheOffPassed || emergency.qualificationPassed)) ||
        (operation == TimerOperation::OffPreflight && emergency.offOnlyPassed) ||
        (operation == TimerOperation::CachePreflight && (!emergency.offOnlyPassed || emergency.cacheOffPassed)))
        return false;
    OssmHatTelemetry sample;
    const auto position = getOssmHatPositionStatus();
    if (position.pending || position.enabled || serialPreparing.load() || serialEnabling.load() ||
        !timerOffSnapshot(sample)) return false;
    portENTER_CRITICAL(&guardMux);
    const bool allowed = !guard.armed() && !homingSession.active() &&
        !serialProbeClaimed.load() && !deadlineTripped &&
        guard.fault() == motor_guard::Fault::None &&
        serialTimerProbe.status().phase == serial_timer_probe::Phase::Idle;
    if (allowed) {
        invalidateSerialLocked();
        serialProbeClaimed = true;
        serialTimerOperation = operation;
        serialProbeCancelled = false;
        serialTimerCheckpointRequested = false;
        serialTimerRestorePending = false;
        serialGpioStopVerified = false;
        serialTimerStopVerified = false;
        serialProofContinuity = 0;
        ++safetyControlRevision;
        published.controlRevision = safetyControlRevision;
    }
    portEXIT_CRITICAL(&guardMux);
    if (!allowed) return false;
    const bool created = xTaskCreatePinnedToCore(serialTimerWorker,"serialTimerTest",8192,nullptr,
        configMAX_PRIORITIES-3,&serialProbeTask,1) == pdPASS;
    if (!created) {
        portENTER_CRITICAL(&guardMux);
        serialProbeClaimed = false;
        serialTimerOperation = TimerOperation::None;
        portEXIT_CRITICAL(&guardMux);
    }
    return created;
}
} // namespace

bool requestMotorSerialTimerPreflight(bool reviewed, bool cacheOff) {
    return requestTimerOperation(cacheOff ? TimerOperation::CachePreflight : TimerOperation::OffPreflight,
                                 reviewed);
}
bool requestMotorSerialTimerProbe(bool reviewedPositiveCorridor) {
    return requestTimerOperation(TimerOperation::Finite,reviewedPositiveCorridor);
}

#else
#include <Arduino.h>
#include "services/led.h"
#include "ossm/state/calibration.h"
#include "ossm/state/state.h"
#include "services/stepper.h"

// ALT keeps its existing STEP/DIR drive and telemetry reader. The classic V2
// hardware guard is unavailable here; it neither claims a physical stop nor
// touches an absent ENA pin. Ticket ownership still prevents delayed Home
// tasks from submitting commands after Stop or a newer Home request.
namespace {
portMUX_TYPE legacyMux = portMUX_INITIALIZER_UNLOCKED;
homing_session::Gate legacyHoming;
SemaphoreHandle_t legacyCommandMutex = nullptr;
uint64_t legacyRevision = 0;
void cancelLegacyHoming() {
    portENTER_CRITICAL(&legacyMux);
    ++legacyRevision;
    const bool abandoned = legacyHoming.active();
    legacyHoming.beginCancel();
    if (abandoned) { calibration.isHomed = false; setHomingActive(false, false); }
    portEXIT_CRITICAL(&legacyMux);
    if (legacyCommandMutex != nullptr) xSemaphoreTake(legacyCommandMutex, portMAX_DELAY);
    if (stepper != nullptr) stepper->forceStop();
    portENTER_CRITICAL(&legacyMux);
    legacyHoming.finishCancel();
    portEXIT_CRITICAL(&legacyMux);
    if (legacyCommandMutex != nullptr) xSemaphoreGive(legacyCommandMutex);
}
}
void initMotorSafety() {
    if (legacyCommandMutex == nullptr) legacyCommandMutex = xSemaphoreCreateMutex();
}
MotorSafetyStatus getMotorSafetyStatus() {
    MotorSafetyStatus status;
    status.ready = legacyCommandMutex != nullptr && stepper != nullptr;
    portENTER_CRITICAL(&legacyMux);
    status.controlRevision = legacyRevision;
    portEXIT_CRITICAL(&legacyMux);
    return status;
}
bool motorSafetyEventCurrent(uint64_t revision) {
    portENTER_CRITICAL(&legacyMux);
    const bool current = revision == legacyRevision;
    portEXIT_CRITICAL(&legacyMux);
    return current;
}
MotorSafetyFaultInfo getMotorSafetyFaultInfo() { return {}; }
const char *motorSafetyReason() { return "legacy STEP/DIR (serial guard unavailable)"; }
bool motorSafetyCanEnableOutputs() { return legacyCommandMutex != nullptr && stepper != nullptr; }
bool enableMotorSafetyOutputs() {
    return motorSafetyCanEnableOutputs() && stepper->enableOutputs();
}
bool enableMotorSafetyHomingOutputs(MotorHomingTicket ticket) {
    return runMotorHomingCommand(ticket, [] { return enableMotorSafetyOutputs(); });
}
MotorHomingTicket beginMotorHomingSession() {
    portENTER_CRITICAL(&legacyMux);
    ++legacyRevision;
    const auto ticket = legacyHoming.begin();
    calibration.isHomed = false;
    setHomingActive(true, false);
    portEXIT_CRITICAL(&legacyMux);
    runMotorHomingCommand(ticket, [] {
        if (stepper != nullptr) stepper->forceStop();
        return true;
    });
    return ticket;
}
bool motorHomingTicketCurrent(MotorHomingTicket ticket) {
    portENTER_CRITICAL(&legacyMux);
    const bool current = legacyHoming.accepts(ticket);
    portEXIT_CRITICAL(&legacyMux);
    return current;
}
bool motorHomingSessionActive() {
    portENTER_CRITICAL(&legacyMux);
    const bool active = legacyHoming.active();
    portEXIT_CRITICAL(&legacyMux);
    return active;
}
bool completeMotorHomingSession(MotorHomingTicket ticket) {
    return commitMotorHomingState(ticket, [&] {
        calibration.isHomed = true;
        calibration.justHomed = true;
        legacyHoming.cancel();
        return true;
    });
}
void cancelMotorHomingSession() { cancelLegacyHoming(); }
bool runMotorHomingCommand(MotorHomingTicket ticket, MotorHomingAction action, void *context) {
    if (legacyCommandMutex == nullptr || action == nullptr) return false;
    while (motorHomingTicketCurrent(ticket)) {
        if (xSemaphoreTake(legacyCommandMutex, pdMS_TO_TICKS(20)) != pdTRUE) continue;
        portENTER_CRITICAL(&legacyMux);
        const bool current = legacyHoming.accepts(ticket);
        const bool ready = legacyHoming.commandsReady();
        portEXIT_CRITICAL(&legacyMux);
        if (!current) { xSemaphoreGive(legacyCommandMutex); return false; }
        if (!ready) {
            xSemaphoreGive(legacyCommandMutex);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        const bool result = action(context);
        const bool stillCurrent = motorHomingTicketCurrent(ticket);
        xSemaphoreGive(legacyCommandMutex);
        return result && stillCurrent;
    }
    return false;
}
bool commitMotorHomingState(MotorHomingTicket ticket, MotorHomingAction action, void *context) {
    portENTER_CRITICAL(&legacyMux);
    const bool result = action != nullptr && legacyHoming.commit(ticket, [&] { return action(context); });
    portEXIT_CRITICAL(&legacyMux);
    return result;
}
bool motorSafetyBeginHomingPass(uint8_t, MotorHomingTicket ticket) {
    return motorHomingTicketCurrent(ticket) && motorSafetyCanEnableOutputs();
}
bool motorSafetyBeginHomingMotion(MotorHomingTicket ticket) { return motorHomingTicketCurrent(ticket); }
bool motorSafetyEndHomingPass(MotorHomingTicket ticket) { return motorHomingTicketCurrent(ticket); }
bool motorSafetyBeginNormalOperation(MotorHomingTicket ticket) {
    return motorHomingTicketCurrent(ticket) && motorSafetyCanEnableOutputs();
}
bool motorSafetyBeginNormalOperation(uint64_t revision) {
    return motorSafetyEventCurrent(revision) && motorSafetyCanEnableOutputs();
}
bool motorSafetyBeginNormalOperation() { return motorSafetyCanEnableOutputs(); }
uint64_t motorSafetySerialSession() { return 0; }
bool motorSafetySerialCanMove(uint64_t) { return false; }
MotorSerialSafetyStatus getMotorSerialSafetyStatus() { return {}; }
bool requestMotorSerialInterlockProbe(bool) { return false; }
serial_interlock_probe::Status getMotorSerialInterlockProbeStatus() { return {}; }
bool requestMotorSerialTimerPreflight(bool, bool) { return false; }
bool requestMotorSerialTimerProbe(bool) { return false; }
serial_timer_probe::Status getMotorSerialTimerProbeStatus() { return {}; }
void disarmMotorSafety() { cancelLegacyHoming(); }
bool resetMotorSafetyAfterReview() { return false; }
bool prepareMotorSafetyExternalControlAfterReview() { return false; }
#endif
