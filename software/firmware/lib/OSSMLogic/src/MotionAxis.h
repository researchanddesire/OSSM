#pragma once

#include <cstdint>

// Planner coordinates remain pulse-equivalent steps even for a serial servo.
// They are a software frame, not the motor's absolute encoder register.
class MotionAxis {
public:
    virtual ~MotionAxis() = default;
    virtual void setDirectionPin(uint8_t pin, bool highCountsUp = true) = 0;
    virtual bool directionPinHighCountsUp() = 0;
    virtual void setEnablePin(uint8_t pin, bool activeLow = true) = 0;
    virtual void setAutoEnable(bool enabled) = 0;
    // GPIO-only operations: the safety service may call these under its lock.
    // They do not establish serial-mode stop/enable authority.
    virtual bool enableOutputs() = 0;
    virtual bool disableOutputs() = 0;
    virtual int8_t setSpeedInHz(uint32_t speed) = 0;
    virtual int8_t setAcceleration(int32_t acceleration) = 0;
    virtual uint32_t getAcceleration() = 0;
    virtual uint32_t getSpeedInMilliHz() = 0;
    virtual void applySpeedAcceleration() = 0;
    virtual int8_t moveTo(int32_t position, bool blocking = false) = 0;
    virtual int8_t moveToForOwner(int32_t position, uint64_t revision,
                                 bool (*validator)(uint64_t), bool blocking = false) {
        return validator != nullptr && !validator(revision) ? -1 : moveTo(position, blocking);
    }
    virtual int8_t move(int32_t delta, bool blocking = false) = 0;
    virtual bool isRunning() = 0;
    virtual int32_t getCurrentPosition() = 0;
    virtual void setCurrentPosition(int32_t position) = 0;
    virtual void stopMove() = 0;
    virtual void stopMoveForOwner(uint64_t revision, bool (*validator)(uint64_t)) {
        if (validator == nullptr || validator(revision)) stopMove();
    }
    virtual void forceStop() = 0;
    virtual void forceStopAndNewPosition(int32_t position) = 0;
};
