#include "motorFeedback.h"

#include <Arduino.h>
#include <cmath>

#include "constants/Config.h"
#include "motor_feedback_logic.h"
#include "services/ossmHat.h"
#include "services/stepper.h"
#include "services/motorSafety.h"

namespace {

constexpr uint32_t stoppedSampleWaitMs = 300;
constexpr float keepoutMm = Config::Advanced::strokeZeroOffsetMm;
constexpr float millimetersPerRevolution =
    Config::Driver::pulleyToothCount * Config::Driver::beltPitchMm;

struct Reference {
    bool polarityKnown = false;
    int8_t encoderToNativeSign = 0;
    uint32_t polarityContinuity = 0;
    bool homeKnown = false;
    int32_t homeEncoder = 0;
    uint32_t homeContinuity = 0;
    bool passStarted = false;
    int32_t passEncoder = 0;
    int32_t passCommand = 0;
    uint32_t passContinuity = 0;
};

Reference reference;
portMUX_TYPE referenceMux = portMUX_INITIALIZER_UNLOCKED;

void invalidateHome() {
    portENTER_CRITICAL(&referenceMux);
    reference.homeKnown = false;
    portEXIT_CRITICAL(&referenceMux);
}

bool readFreshTelemetry(OssmHatTelemetry &sample) {
    if (ossmHatDetected() && readOssmHatTelemetry(sample)) return true;
    // A disconnected/restarted motor may have a different encoder origin.
    // Never revive its previous home anchor merely because replies resume.
    invalidateHome();
    return false;
}

bool captureStoppedSample(OssmHatTelemetry &sample, int32_t &command) {
    if (!ossmHatDetected() || stepper == nullptr || stepper->isRunning() ||
        stepperFrame != StepperFrame::Native)
        return false;

    command = stepper->getCurrentPosition();
    const uint32_t started = millis();
    while (static_cast<uint32_t>(millis() - started) < stoppedSampleWaitMs) {
        if (stepper->isRunning() || stepper->getCurrentPosition() != command ||
            stepperFrame != StepperFrame::Native)
            return false;
        if (readOssmHatTelemetry(sample)) {
            const uint32_t sinceStopped = sample.updatedAtMs - started;
            if (sinceStopped != 0 && sinceStopped < 0x80000000U) return true;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}

bool measuredPositionMm(float &position) {
    OssmHatTelemetry sample;
    if (!readFreshTelemetry(sample)) return false;

    portENTER_CRITICAL(&referenceMux);
    const Reference saved = reference;
    portEXIT_CRITICAL(&referenceMux);
    if (!saved.homeKnown || !saved.polarityKnown) return false;
    if (sample.continuity != saved.homeContinuity) {
        // The service observes every frame, including outages entirely
        // between two low-rate BLE/MQTT reads. Such an outage can reset the
        // motor's absolute counter, so the old anchor cannot be reused.
        invalidateHome();
        return false;
    }

    const double nativeMm = motor_feedback_logic::nativePositionMm(
        sample.positionCounts, saved.homeEncoder, saved.encoderToNativeSign,
        millimetersPerRevolution);
    position = static_cast<float>(motor_feedback_logic::positionInFrameMm(
        nativeMm, stepperFrame == StepperFrame::StrokeEngine, keepoutMm));
    return std::isfinite(position);
}

}  // namespace

float getMotorPositionMm() {
    float measured;
    if (measuredPositionMm(measured)) return measured;
    return stepper == nullptr ? 0.0f
                             : static_cast<float>(stepper->getCurrentPosition()) /
                                   Config::Driver::stepsPerMM;
}

bool motorPositionUsesRs485() {
    float measured;
    return measuredPositionMm(measured);
}

bool readMotorCurrentAmps(float &amps) {
    OssmHatTelemetry sample;
    if (!readFreshTelemetry(sample) || !std::isfinite(sample.currentAmps) ||
        sample.currentAmps < 0)
        return false;
    amps = sample.currentAmps;
    return true;
}

bool readMotorVoltageVolts(float &volts) {
    OssmHatTelemetry sample;
    if (!readFreshTelemetry(sample) || !std::isfinite(sample.voltageVolts) ||
        sample.voltageVolts < 0)
        return false;
    volts = sample.voltageVolts;
    return true;
}

bool beginMotorFeedbackHomingPass(homing_session::Ticket ticket) {
    if (!commitMotorHomingState(ticket, [] {
        portENTER_CRITICAL(&referenceMux);
        reference.homeKnown = false;
        reference.passStarted = false;
        portEXIT_CRITICAL(&referenceMux);
        return true;
    })) return false;
    OssmHatTelemetry sample;
    int32_t command;
    if (!captureStoppedSample(sample, command)) return false;
    // Capture waits for a new physical sample. Publish only if its original
    // request still owns homing after that wait (including Stop -> new Home).
    return commitMotorHomingState(ticket, [&] {
        portENTER_CRITICAL(&referenceMux);
        reference.passEncoder = sample.positionCounts;
        reference.passCommand = command;
        reference.passContinuity = sample.continuity;
        reference.passStarted = true;
        portEXIT_CRITICAL(&referenceMux);
        return true;
    });
}

bool finishMotorFeedbackHomingPass(homing_session::Ticket ticket) {
    Reference start;
    if (!commitMotorHomingState(ticket, [&] {
        portENTER_CRITICAL(&referenceMux);
        start = reference;
        reference.passStarted = false;
        portEXIT_CRITICAL(&referenceMux);
        return true;
    }) || !start.passStarted) return false;
    OssmHatTelemetry sample;
    int32_t command;
    if (!captureStoppedSample(sample, command)) return false;
    if (sample.continuity != start.passContinuity) {
        commitMotorHomingState(ticket, [] {
            portENTER_CRITICAL(&referenceMux);
            reference.polarityKnown = false;
            reference.homeKnown = false;
            portEXIT_CRITICAL(&referenceMux);
            return true;
        });
        return false;
    }
    const int8_t learned = motor_feedback_logic::learnNativePolarity(
        start.passEncoder, sample.positionCounts, start.passCommand, command,
        Config::Driver::motorStepPerRevolution,
        5.0 * Config::Driver::stepsPerMM);
    if (learned == 0) return false;
    return commitMotorHomingState(ticket, [&] {
        portENTER_CRITICAL(&referenceMux);
        const bool consistent = !reference.polarityKnown ||
            reference.encoderToNativeSign == learned;
        reference.polarityKnown = consistent;
        reference.encoderToNativeSign = consistent ? learned : 0;
        reference.polarityContinuity = consistent ? sample.continuity : 0;
        reference.homeKnown = false;
        portEXIT_CRITICAL(&referenceMux);
        return consistent;
    });
}

bool rebaseMotorFeedbackAtHome(homing_session::Ticket ticket) {
    if (!commitMotorHomingState(ticket, [] { invalidateHome(); return true; })) return false;
    OssmHatTelemetry sample;
    int32_t command;
    if (!captureStoppedSample(sample, command) || command != 0) return false;
    return commitMotorHomingState(ticket, [&] {
        portENTER_CRITICAL(&referenceMux);
        reference.homeEncoder = sample.positionCounts;
        reference.homeContinuity = sample.continuity;
        reference.homeKnown = reference.polarityKnown &&
            reference.polarityContinuity == sample.continuity;
        const bool valid = reference.homeKnown;
        portEXIT_CRITICAL(&referenceMux);
        return valid;
    });
}
