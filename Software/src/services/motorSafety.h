#pragma once

#include <cstdint>

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
};

void initMotorSafety();
MotorSafetyStatus getMotorSafetyStatus();
MotorSafetyFaultInfo getMotorSafetyFaultInfo();
const char *motorSafetyReason();
bool motorSafetyCanEnableOutputs();
bool enableMotorSafetyOutputs();
bool motorSafetyBeginHomingPass(uint8_t pass);
bool motorSafetyEndHomingPass();
bool motorSafetyBeginNormalOperation();
// OTA/maintenance only: stop and disarm, never enable or clear a fault.
void disarmMotorSafety();
// Explicit operator review only. Does not restore motor registers or arm.
bool resetMotorSafetyAfterReview();
bool prepareMotorSafetyExternalControlAfterReview();
