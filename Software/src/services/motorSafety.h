#pragma once

#include <cstdint>
#include <type_traits>
#include "homing_session.h"

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
MotorSafetyFaultInfo getMotorSafetyFaultInfo();
const char *motorSafetyReason();
bool motorSafetyCanEnableOutputs();
bool enableMotorSafetyOutputs();
bool enableMotorSafetyHomingOutputs(MotorHomingTicket ticket);
bool motorSafetyBeginHomingPass(uint8_t pass, MotorHomingTicket ticket);
bool motorSafetyBeginHomingMotion(MotorHomingTicket ticket);
bool motorSafetyEndHomingPass(MotorHomingTicket ticket);
bool motorSafetyBeginNormalOperation(MotorHomingTicket ticket);
bool motorSafetyBeginNormalOperation();
// OTA/maintenance only: stop and disarm, never enable or clear a fault.
void disarmMotorSafety();
// Explicit operator review only. Does not restore motor registers or arm.
bool resetMotorSafetyAfterReview();
bool prepareMotorSafetyExternalControlAfterReview();
