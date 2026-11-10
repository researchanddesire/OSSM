#pragma once

#include "FastAccelStepper.h"
#include "MotionAxis.h"

// Preserve the vendored library's ordinary FastAccelStepper entry point.
class FastAccelStepperAxis final : public MotionAxis {
public:
    void attach(FastAccelStepper *servo) { servo_ = servo; }
    void setDirectionPin(uint8_t pin, bool high = true) override { servo_->setDirectionPin(pin, high); }
    bool directionPinHighCountsUp() override { return servo_->directionPinHighCountsUp(); }
    void setEnablePin(uint8_t pin, bool low = true) override { servo_->setEnablePin(pin, low); }
    void setAutoEnable(bool value) override { servo_->setAutoEnable(value); }
    bool enableOutputs() override { return servo_->enableOutputs(); }
    bool disableOutputs() override { return servo_->disableOutputs(); }
    int8_t setSpeedInHz(uint32_t value) override { return servo_->setSpeedInHz(value); }
    int8_t setAcceleration(int32_t value) override { return servo_->setAcceleration(value); }
    uint32_t getAcceleration() override { return servo_->getAcceleration(); }
    uint32_t getSpeedInMilliHz() override { return servo_->getSpeedInMilliHz(); }
    void applySpeedAcceleration() override { servo_->applySpeedAcceleration(); }
    int8_t moveTo(int32_t position, bool blocking = false) override { return servo_->moveTo(position, blocking); }
    int8_t move(int32_t delta, bool blocking = false) override { return servo_->move(delta, blocking); }
    bool isRunning() override { return servo_->isRunning(); }
    int32_t getCurrentPosition() override { return servo_->getCurrentPosition(); }
    void setCurrentPosition(int32_t position) override { servo_->setCurrentPosition(position); }
    void stopMove() override { servo_->stopMove(); }
    void forceStop() override { servo_->forceStop(); }
    void forceStopAndNewPosition(int32_t position) override { servo_->forceStopAndNewPosition(position); }
private:
    FastAccelStepper *servo_ = nullptr;
};
