#include "motorMotion.h"

#include <Arduino.h>
#include <atomic>
#include <limits>

#include "constants/Config.h"
#include "serial_axis_logic.h"
#include "services/motorSafety.h"
#include "services/ossmHat.h"
#include "services/stepper.h"

namespace {
using SessionCallback = uint64_t (*)();
using AuthorizationCallback = bool (*)(uint64_t);
std::atomic<SessionCallback> sessionCallback{nullptr};
std::atomic<AuthorizationCallback> authorizationCallback{nullptr};

uint64_t authorizedSession() {
    const auto session = sessionCallback.load();
    const auto allowed = authorizationCallback.load();
    const uint64_t value = session == nullptr ? 0 : session();
    return value != 0 && allowed != nullptr && allowed(value) ? value : 0;
}

class OssmMotionAxis final : public MotionAxis {
public:
    void begin(FastAccelStepper *pulse) {
        pulse_ = pulse;
        taskReady_ = xTaskCreatePinnedToCore(task, "serialMotion", 4096, this,
            configMAX_PRIORITIES - 3, nullptr, 1) == pdPASS;
    }
    bool serialSelected() {
        if (serial_.load()) return true;
        if (!ossmHatDetected()) return false;
        bool expected = false;
        if (serial_.compare_exchange_strong(expected, true)) {
            // A detected HAT never falls back to pulses, including after loss
            // of power/telemetry or failure of serial authorization.
            const int32_t logical = pulse_->getCurrentPosition();
            pulse_->forceStop();
            pulse_->disableOutputs();
            portENTER_CRITICAL(&mux_);
            seedLogical_ = logical;
            portEXIT_CRITICAL(&mux_);
            selectionReady_.store(true);
        }
        return true;
    }
    void setDirectionPin(uint8_t pin, bool high = true) override {
        if (!serialSelected()) { pulse_->setDirectionPin(pin, high); directionHigh_ = high; return; }
        refresh();
        portENTER_CRITICAL(&mux_);
        if (high != directionHigh_) {
            if (trajectory_.active() || pending_ || !position_.direction(high, millis())) frameValid_ = false;
            else { directionHigh_ = high; frameValid_ = true; }
        }
        portEXIT_CRITICAL(&mux_);
    }
    bool directionPinHighCountsUp() override {
        return serialSelected() ? directionHigh_ : pulse_->directionPinHighCountsUp();
    }
    void setEnablePin(uint8_t pin, bool low = true) override { pulse_->setEnablePin(pin, low); }
    void setAutoEnable(bool value) override { pulse_->setAutoEnable(serial_.load() ? false : value); }
    // These two methods deliberately contain GPIO operations only. The guard
    // owns mode1/output-bit proof; a GPIO level alone is never that proof.
    bool enableOutputs() override { return pulse_->enableOutputs(); }
    bool disableOutputs() override { return pulse_->disableOutputs(); }
    int8_t setSpeedInHz(uint32_t value) override {
        if (!serialSelected()) return pulse_->setSpeedInHz(value);
        portENTER_CRITICAL(&mux_); speed_ = value; portEXIT_CRITICAL(&mux_);
        return value == 0 ? -1 : 0;
    }
    int8_t setAcceleration(int32_t value) override {
        if (!serialSelected()) return pulse_->setAcceleration(value);
        portENTER_CRITICAL(&mux_); acceleration_ = value; portEXIT_CRITICAL(&mux_);
        return value <= 0 ? -1 : 0;
    }
    uint32_t getAcceleration() override {
        if (!serialSelected()) return pulse_->getAcceleration();
        portENTER_CRITICAL(&mux_); const auto value = acceleration_; portEXIT_CRITICAL(&mux_);
        return value > 0 ? value : 0;
    }
    uint32_t getSpeedInMilliHz() override {
        if (!serialSelected()) return pulse_->getSpeedInMilliHz();
        portENTER_CRITICAL(&mux_); const uint64_t value = uint64_t{speed_} * 1000; portEXIT_CRITICAL(&mux_);
        return value > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(value);
    }
    void applySpeedAcceleration() override { if (!serialSelected()) pulse_->applySpeedAcceleration(); }
    int8_t moveTo(int32_t logical, bool blocking = false) override {
        return moveToForOwner(logical, 0, nullptr, blocking);
    }
    int8_t moveToForOwner(int32_t logical, uint64_t ownerRevision,
                         bool (*ownerValidator)(uint64_t), bool blocking = false) override {
        if (ownerValidator != nullptr && !ownerValidator(ownerRevision)) return -1;
        if (!serialSelected()) return pulse_->moveTo(logical, blocking);
        refresh();
        const uint64_t session = authorizedSession();
        if (ownerValidator != nullptr && !ownerValidator(ownerRevision)) return -1;
        const auto control = getOssmHatPositionStatus();
        const auto safety = getMotorSafetyStatus();
        if (!taskReady_ || session == 0 || control.session != session ||
            !control.prepared || !control.enabled || control.faulted) return -1;
        portENTER_CRITICAL(&mux_);
        int32_t encoder = 0;
        const bool valid = frameValid_ && position_.fresh(millis()) &&
            position_.target(logical, encoder) && speed_ != 0 && acceleration_ > 0 &&
            (!safety.boundsValid || serial_axis::insideBounds(encoder,
                safety.lowerStopCounts, safety.upperStopCounts,
                32768.0 / (Config::Driver::pulleyToothCount * Config::Driver::beltPitchMm)));
        if (valid) {
            if (commandSession_ != session) {
                trajectory_.reset(position_.current());
                pending_ = false;
                position_.canceled();
                ++revision_;
            }
            commandSession_ = session;
            commandOwnerRevision_ = ownerRevision;
            commandOwnerValidator_ = ownerValidator;
            commandTarget_ = logical;
            trajectory_.target(logical);
        }
        portEXIT_CRITICAL(&mux_);
        if (!valid) return -1;
        if (blocking) {
            while (isRunning()) {
                if (authorizedSession() != session ||
                    (ownerValidator != nullptr && !ownerValidator(ownerRevision))) return -1;
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
        return 0;
    }
    int8_t move(int32_t delta, bool blocking = false) override {
        if (!serialSelected()) return pulse_->move(delta, blocking);
        const int64_t target = static_cast<int64_t>(getCurrentPosition()) + delta;
        if (target < INT32_MIN || target > INT32_MAX) return -1;
        return moveTo(static_cast<int32_t>(target), blocking);
    }
    bool isRunning() override {
        if (!serialSelected()) return pulse_->isRunning();
        refresh();
        portENTER_CRITICAL(&mux_);
        const bool running = trajectory_.active() || pending_ || position_.running(millis());
        portEXIT_CRITICAL(&mux_);
        return running;
    }
    int32_t getCurrentPosition() override {
        if (!serialSelected()) return pulse_->getCurrentPosition();
        refresh();
        portENTER_CRITICAL(&mux_);
        const int32_t logical = position_.anchored() ? position_.current() : seedLogical_;
        portEXIT_CRITICAL(&mux_);
        return logical;
    }
    void setCurrentPosition(int32_t logical) override {
        if (!serialSelected()) { pulse_->setCurrentPosition(logical); return; }
        refresh();
        portENTER_CRITICAL(&mux_);
        if (trajectory_.active() || pending_ || !position_.rebase(logical, millis())) frameValid_ = false;
        else { seedLogical_ = logical; trajectory_.reset(logical); frameValid_ = true; ++revision_; }
        portEXIT_CRITICAL(&mux_);
    }
    void stopMove() override {
        if (!serialSelected()) { pulse_->stopMove(); return; }
        const auto status = getOssmHatPositionStatus();
        const double cap = status.accelerationRpmPerSec * Config::Driver::motorStepPerRevolution / 60.0;
        portENTER_CRITICAL(&mux_);
        const double acceleration = acceleration_ < cap ? acceleration_ : cap;
        if (acceleration > 0) {
            trajectory_.stop(acceleration);
            const double target = std::round(trajectory_.targetPosition());
            if (target >= INT32_MIN && target <= INT32_MAX)
                commandTarget_ = static_cast<int32_t>(target);
        }
        portEXIT_CRITICAL(&mux_);
    }
    void stopMoveForOwner(uint64_t ownerRevision,
                          bool (*ownerValidator)(uint64_t)) override {
        if (ownerValidator != nullptr && !ownerValidator(ownerRevision)) return;
        if (!serialSelected()) { pulse_->stopMove(); return; }
        const uint64_t session = authorizedSession();
        const auto status = getOssmHatPositionStatus();
        if (session == 0 || (ownerValidator != nullptr && !ownerValidator(ownerRevision))) return;
        const double cap = status.accelerationRpmPerSec * Config::Driver::motorStepPerRevolution / 60.0;
        portENTER_CRITICAL(&mux_);
        if (session == commandSession_ && commandOwnerRevision_ == ownerRevision &&
            commandOwnerValidator_ == ownerValidator) {
            const double acceleration = acceleration_ < cap ? acceleration_ : cap;
            if (acceleration > 0) {
                trajectory_.stop(acceleration);
                const double target = std::round(trajectory_.targetPosition());
                if (target >= INT32_MIN && target <= INT32_MAX)
                    commandTarget_ = static_cast<int32_t>(target);
            }
        }
        portEXIT_CRITICAL(&mux_);
    }
    void forceStop() override {
        pulse_->forceStop();
        if (!serialSelected()) return;
        portENTER_CRITICAL(&mux_);
        trajectory_.reset(position_.current());
        position_.canceled(); pending_ = false; commandSession_ = 0; ++revision_;
        portEXIT_CRITICAL(&mux_);
        cancelOssmHatPositionControl();
    }
    void forceStopAndNewPosition(int32_t logical) override {
        if (!serialSelected()) { pulse_->forceStopAndNewPosition(logical); return; }
        // Homing calls this only after physical arrival and stop proof. Do
        // not disable an enabled serial session merely to rebase its frame.
        if (isRunning()) { forceStop(); return; }
        setCurrentPosition(logical);
    }
    int32_t commandedTarget() {
        portENTER_CRITICAL(&mux_); const auto value = commandTarget_; portEXIT_CRITICAL(&mux_);
        return value;
    }
    bool mirror(int32_t keepout) {
        if (!serialSelected()) return false;
        refresh();
        portENTER_CRITICAL(&mux_);
        const bool mirrored = !trajectory_.active() && !pending_ &&
            position_.mirrorFrame(keepout, millis());
        if (mirrored) {
            seedLogical_ = position_.current();
            directionHigh_ = !directionHigh_;
            trajectory_.reset(seedLogical_);
            frameValid_ = true;
            ++revision_;
        }
        portEXIT_CRITICAL(&mux_);
        return mirrored;
    }
    MotorMotionStatus status() {
        const auto control = getOssmHatPositionStatus();
        MotorMotionStatus result;
        result.serial = serialSelected();
        result.maximumSpeedRpm = control.maximumSpeedRpm;
        result.accelerationRpmPerSecond = control.accelerationRpmPerSec;
        portENTER_CRITICAL(&mux_);
        result.frameReady = frameValid_ && position_.anchored();
        result.moving = trajectory_.active() || position_.running(millis());
        result.pending = pending_;
        result.requestedStepsPerSecond = speed_;
        result.requestedAcceleration = acceleration_ > 0 ? acceleration_ : 0;
        result.commandTarget = commandTarget_;
        portEXIT_CRITICAL(&mux_);
        return result;
    }
private:
    static void task(void *context) {
        auto *axis = static_cast<OssmMotionAxis *>(context);
        TickType_t wake = xTaskGetTickCount();
        for (;;) { axis->tick(); vTaskDelayUntil(&wake, pdMS_TO_TICKS(20)); }
    }
    void refresh() {
        if (!selectionReady_.load()) return;
        OssmHatTelemetry sample;
        if (!readOssmHatTelemetry(sample)) return;
        const bool homing = motorHomingSessionActive();
        portENTER_CRITICAL(&mux_);
        position_.observe({sample.sequence, sample.updatedAtMs, sample.continuity,
                           sample.positionCounts}, millis());
        if (!position_.anchored() && homing && position_.quiet(millis()) &&
            !trajectory_.active() && !pending_) {
            position_.rebase(seedLogical_, millis());
            frameValid_ = position_.direction(directionHigh_, millis());
            trajectory_.reset(seedLogical_);
        }
        portEXIT_CRITICAL(&mux_);
    }
    void tick() {
        if (!serialSelected()) return;
        refresh();
        portENTER_CRITICAL(&mux_);
        const uint64_t actorRevision = commandOwnerRevision_;
        const auto actorValidator = commandOwnerValidator_;
        portEXIT_CRITICAL(&mux_);
        if (actorValidator != nullptr && !actorValidator(actorRevision)) return;
        const uint64_t session = authorizedSession();
        const auto status = getOssmHatPositionStatus();
        const auto safety = getMotorSafetyStatus();
        const uint32_t now = millis();
        serial_axis::Trajectory::Proposal proposal;
        int32_t logical = 0, encoder = 0;
        uint32_t request = 0, revision = 0;
        uint64_t queuedOwnerRevision = 0;
        bool (*queuedOwnerValidator)(uint64_t) = nullptr;
        bool queue = false;
        portENTER_CRITICAL(&mux_);
        if (session == 0 || session != commandSession_ || status.session != session ||
            !status.prepared || !status.enabled || status.faulted || !frameValid_ ||
            !position_.fresh(now)) {
            lastPlanAt_ = now;
            portEXIT_CRITICAL(&mux_);
            return;
        }
        if (pending_) {
            if (status.acknowledgedRequestId == pendingId_ &&
                status.acknowledgedTargetCounts == pendingEncoder_) {
                trajectory_.commit(pendingProposal_);
                if (pendingProposal_.final) {
                    position_.submitted(pendingId_, commandTarget_, pendingEncoder_);
                    position_.acknowledge(pendingId_, pendingEncoder_);
                }
                pending_ = false;
            } else {
                lastPlanAt_ = now;
                portEXIT_CRITICAL(&mux_);
                return;
            }
        }
        const double speedCap = status.maximumSpeedRpm * Config::Driver::motorStepPerRevolution / 60.0;
        const double accelerationCap = status.accelerationRpmPerSec * Config::Driver::motorStepPerRevolution / 60.0;
        const double speed = speed_ < speedCap ? speed_ : speedCap;
        const double acceleration = acceleration_ < accelerationCap ? acceleration_ : accelerationCap;
        const uint32_t elapsed = now - lastPlanAt_;
        // One millimeter is a conservative staging following-error envelope;
        // fixed verified session speed/ramp caps further limit each waypoint.
        if (elapsed >= 20 && trajectory_.propose(position_.current(),
                Config::Driver::stepsPerMM, speed, acceleration, elapsed, proposal)) {
            const double rounded = std::round(proposal.position);
            if (rounded >= INT32_MIN && rounded <= INT32_MAX) {
                logical = static_cast<int32_t>(rounded);
                queue = position_.target(logical, encoder);
                if (queue && safety.boundsValid) {
                    queue = serial_axis::insideBounds(encoder,
                        safety.lowerStopCounts, safety.upperStopCounts,
                        32768.0 / (Config::Driver::pulleyToothCount * Config::Driver::beltPitchMm));
                }
                if (queue) {
                    if (++nextRequest_ == 0) ++nextRequest_;
                    request = nextRequest_; revision = revision_;
                    queuedOwnerRevision = commandOwnerRevision_;
                    queuedOwnerValidator = commandOwnerValidator_;
                }
            }
        }
        lastPlanAt_ = now;
        portEXIT_CRITICAL(&mux_);
        if (!queue || (queuedOwnerValidator != nullptr &&
                !queuedOwnerValidator(queuedOwnerRevision)) ||
            !queueOssmHatPositionTarget(session, request, encoder,
                status.maximumSpeedRpm, status.accelerationRpmPerSec, 100)) return;
        portENTER_CRITICAL(&mux_);
        if (revision == revision_ && commandSession_ == session) {
            pending_ = true; pendingId_ = request; pendingEncoder_ = encoder;
            pendingProposal_ = proposal;
        }
        portEXIT_CRITICAL(&mux_);
    }
    FastAccelStepper *pulse_ = nullptr;
    std::atomic_bool serial_{false};
    std::atomic_bool selectionReady_{false};
    bool taskReady_ = false;
    bool directionHigh_ = false;
    bool frameValid_ = false;
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    serial_axis::Position position_{Config::Driver::motorStepPerRevolution,
        Config::Driver::pulleyToothCount * Config::Driver::beltPitchMm};
    serial_axis::Trajectory trajectory_;
    serial_axis::Trajectory::Proposal pendingProposal_;
    uint32_t speed_ = 0;
    int32_t acceleration_ = 0;
    int32_t seedLogical_ = 0;
    int32_t commandTarget_ = 0;
    uint64_t commandSession_ = 0;
    uint64_t commandOwnerRevision_ = 0;
    bool (*commandOwnerValidator_)(uint64_t) = nullptr;
    uint32_t nextRequest_ = 0, pendingId_ = 0, revision_ = 0, lastPlanAt_ = 0;
    int32_t pendingEncoder_ = 0;
    bool pending_ = false;
};
OssmMotionAxis axis;
}  // namespace

MotionAxis *initializeMotorMotion(FastAccelStepper *pulseStepper) {
    axis.begin(pulseStepper);
    return &axis;
}
void registerMotorSerialMotionAuthorization(uint64_t (*session)(), bool (*canMove)(uint64_t)) {
    authorizationCallback.store(canMove);
    sessionCallback.store(session);
}
bool motorMotionUsesSerial() { return axis.serialSelected(); }
int32_t motorMotionCommandedTarget() { return axis.commandedTarget(); }
bool mirrorMotorMotionFrame(int32_t keepoutSteps) { return axis.mirror(keepoutSteps); }
MotorMotionStatus getMotorMotionStatus() { return axis.status(); }
