#pragma once

#include <cstdint>
#include <type_traits>
#include "homing_session.h"
#include "serial_interlock_probe.h"

using MotorHomingTicket = homing_session::Ticket;

MotorHomingTicket beginMotorHomingSession();
bool motorHomingTicketCurrent(MotorHomingTicket ticket);
bool motorHomingSessionActive();
bool completeMotorHomingSession(MotorHomingTicket ticket);
// Revokes first and asserts ENA HIGH immediately; flushes serialized stepper
// commands before returning. Does not clear faults or restore drive registers.
void cancelMotorHomingSession();
using MotorHomingAction = bool (*)(void *);
bool runMotorHomingCommand(MotorHomingTicket, MotorHomingAction, void *);
// Only bounded, nonblocking state publication belongs in this critical section.
bool commitMotorHomingState(MotorHomingTicket, MotorHomingAction, void *);
template <class Action>
bool runMotorHomingCommand(MotorHomingTicket ticket, Action &&action) {
    return runMotorHomingCommand(ticket, [](void *context) {
        return (*static_cast<typename std::remove_reference<Action>::type *>(context))();
    }, &action);
}
template <class Action>
bool commitMotorHomingState(MotorHomingTicket ticket, Action &&action) {
    return commitMotorHomingState(ticket, [](void *context) {
        return (*static_cast<typename std::remove_reference<Action>::type *>(context))();
    }, &action);
}

struct MotorSafetyStatus {
    bool required = false;
    bool ready = false;
    bool armed = false;
    bool latched = false;
    bool outputDisableVerified = false;
    bool stoppedVerified = false;
    bool boundsValid = false;
    bool activeMaster = false;
    uint8_t fault = 0;
    uint8_t contacts = 0;
    uint64_t controlRevision = 0;
    float observedHz = 0;
    uint32_t sampleAgeMs = UINT32_MAX;
    int32_t lowerStopCounts = 0;
    int32_t upperStopCounts = 0;
};

struct MotorSafetyFaultInfo {
    bool available = false;
    uint32_t decisionAtMs = 0;
    uint32_t sampleAtMs = 0;
    uint32_t sequence = 0;
    uint32_t gapMs = 0;
    uint32_t ageMs = 0;
    uint32_t continuity = 0;
    int32_t encoderCounts = 0;
    int64_t deltaCounts = 0;
    float instantaneousMmS = 0;
    float windowMmS = 0;
    uint32_t windowMs = 0;
    bool progressFault = false;
    bool motionExpected = false;
    int8_t direction = 0;
    uint32_t progressMs = 0;
    uint32_t graceMs = 0;
};

void initMotorSafety();
MotorSafetyStatus getMotorSafetyStatus();
bool motorSafetyEventCurrent(uint64_t revision);
MotorSafetyFaultInfo getMotorSafetyFaultInfo();
const char *motorSafetyReason();
bool motorSafetyCanEnableOutputs();
bool enableMotorSafetyOutputs();
bool enableMotorSafetyHomingOutputs(MotorHomingTicket ticket);
bool motorSafetyBeginHomingPass(uint8_t pass, MotorHomingTicket ticket);
bool motorSafetyBeginHomingMotion(MotorHomingTicket ticket);
bool motorSafetyEndHomingPass(MotorHomingTicket ticket);
bool motorSafetyBeginNormalOperation(MotorHomingTicket ticket);
bool motorSafetyBeginNormalOperation(uint64_t expectedRevision);
bool motorSafetyBeginNormalOperation();
// HAT motion uses a distinct, revocable serial session. An unset/unproven
// hardware stop never grants normal serial motion authority.
uint64_t motorSafetySerialSession();
bool motorSafetySerialCanMove(uint64_t session);
struct MotorSerialSafetyStatus {
    bool selected = false;
    bool hardwareStopVerified = false;
    bool prepared = false;
    bool enabled = false;
    bool preparing = false;
    uint16_t maximumSpeedRpm = 37;
    uint16_t accelerationRpmPerSec = 1500;
};
MotorSerialSafetyStatus getMotorSerialSafetyStatus();
// One-shot per boot, +0.5mm characterization into an explicitly reviewed
// free corridor. This never authorizes arbitrary motion or clears a fault.
bool requestMotorSerialInterlockProbe(bool reviewedPositiveCorridor);
serial_interlock_probe::Status getMotorSerialInterlockProbeStatus();
// OTA/maintenance only: stop and disarm, never enable or clear a fault.
void disarmMotorSafety();
// Explicit operator review only. Does not restore motor registers or arm.
bool resetMotorSafetyAfterReview();
bool prepareMotorSafetyExternalControlAfterReview();
