#include "motorMotion.h"

#include <Arduino.h>
#include <atomic>
#include <limits>

#include "drawn_waveform.h"
#include "constants/Config.h"
#include "serial_axis_logic.h"
#include "ossm/state/calibration.h"
#include "services/motorSafety.h"
#include "services/ossmHat.h"
#include "services/stepper.h"

namespace {
static_assert(drawn_pattern::pulseQueueFrames + 2 < QUEUE_LEN,
              "Pattern lookahead must leave room for the FAS direction guard");
using SessionCallback = uint64_t (*)();
using AuthorizationCallback = bool (*)(uint64_t);
FastAccelStepper *pulseEngine = nullptr;
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
#if OSSM_HAT_SERIAL_BACKEND
        taskReady_ = xTaskCreatePinnedToCore(task, "serialMotion", 4096, this,
            configMAX_PRIORITIES - 3, nullptr, 1) == pdPASS;
#endif
    }
    bool serialSelected() {
#if !OSSM_HAT_SERIAL_BACKEND
        return false;
#else
        if (serial_.load()) return true;
        // A receive-only HAT shares its bus with another master and can never
        // send a target, so motion stays on STEP/DIR (shared_bus_logic.h).
        // The request that reveals the other master is always decoded before
        // its reply can mark the HAT detected.
        if (!ossmHatCanSelectSerial()) return false;
        bool expected = false;
        if (serial_.compare_exchange_strong(expected, true)) {
            // A HAT that owned its bus never falls back to pulses, including
            // after loss of power/telemetry, failed serial authorization or
            // another master appearing later.
            const int32_t logical = pulse_->getCurrentPosition();
            if (pulseQueue_.owned() || primingPulse_) stopPulsesNow();
            else drawn_pattern::stopPulseRamp(*pulse_);
            pulse_->disableOutputs();
            portENTER_CRITICAL(&mux_);
            seedLogical_ = logical;
            portEXIT_CRITICAL(&mux_);
            selectionReady_.store(true);
        }
        return true;
#endif
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
        if (!serialSelected()) {
            const bool moves = pulse_->getPositionAfterCommandsCompleted() != logical;
            const auto result = static_cast<int8_t>(pulse_->moveTo(logical, blocking));
            if (result == 0 && moves) needsPulseRamp_ = false;
            return result;
        }
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
            serialPatternFrame_ = false;
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
    drawn_pattern::FrameResult patternFrame(int32_t logical, uint64_t owner, bool serial, uint16_t crossing, bool consecutiveHolds) {
        using drawn_pattern::Frame;
        if (!motorSafetyEventCurrent(owner) || !motorSafetyCanEnableOutputs() ||
            serialSelected() != serial) return {Frame::Rejected, "guard"};
        if (!serial) {
            if (primingPulse_) {
                const bool running = pulse_->isRunning();
                const auto wait = primeWait_.poll(millis(), running, pulse_->getCurrentPosition());
                if (wait.state != Frame::Accepted) return wait;
                // The prime already emitted a real step. Keep its completion
                // pending until the first raw frame is successfully queued.
            }
            if (pulse_->isRampGeneratorActive()) return {Frame::Rejected, "pulse_ramp_active"};
            using Entry = drawn_pattern::PulseQueue::Entry;
            const auto entry = pulseQueue_.entryState(*pulse_, logical, primingPulse_);
            if (entry == Entry::Busy) return {Frame::Rejected, "entry_busy"};
            if (entry == Entry::Stationary) {
                const auto now = millis();
                if (now - lastPauseAt_ < drawn_pattern::pulseFrameMs)
                    return {Frame::Retry, "pulse_entry_hold"};
                lastPauseAt_ = now;
                return {Frame::Accepted, nullptr, 0, drawn_pattern::FrameTiming::EntryHold};
            }
            if (needsPulseRamp_) {
                // A force stop inhibits raw writes until a finite ramp has
                // actually run. Re-prime on the first changed target.
                const auto result = static_cast<int8_t>(pulse_->moveTo(logical));
                if (result != 0) return {Frame::Rejected, "pulse_prime", result};
                primeWait_.start(logical, millis(), pulse_->getSpeedInTicks(),
                                 TICKS_PER_S / 1000, pulse_->getAcceleration());
                needsPulseRamp_ = false;
                primingPulse_ = true;
                return {Frame::Retry, "pulse_priming"};
            }
            const auto result = pulseQueue_.submit<stepper_command_s>(*pulse_, logical,
                TICKS_PER_S / 1000 * drawn_pattern::pulseFrameMs, crossing, MIN_CMD_TICKS,
                static_cast<int8_t>(MOVE_TIMED_EMPTY), consecutiveHolds, [&] {
                    return motorSafetyEventCurrent(owner) && motorSafetyCanEnableOutputs() && !serialSelected();
                });
            if (result.state == Frame::Accepted) primingPulse_ = false;
            return result;
        }

        refresh();
        const auto session = authorizedSession();
        const auto status = getOssmHatPositionStatus();
        const auto safety = getMotorSafetyStatus();
        if (!taskReady_ || session == 0 || status.session != session ||
            !status.prepared || !status.enabled || status.faulted || !safety.boundsValid ||
            !motorSafetyEventCurrent(owner)) return {Frame::Rejected, "serial_guard"};
        int32_t encoder = 0;
        uint32_t request = 0;
        portENTER_CRITICAL(&mux_);
        const bool valid = frameValid_ && position_.fresh(millis()) &&
            position_.target(logical, encoder) &&
            serial_axis::insideBounds(encoder, safety.lowerStopCounts, safety.upperStopCounts,
                32768.0 / (Config::Driver::pulleyToothCount * Config::Driver::beltPitchMm));
        const bool available = !pending_ && !trajectory_.active() &&
            std::abs(static_cast<int64_t>(logical) - position_.current()) <= Config::Driver::stepsPerMM;
        if (valid && available) {
            if (++nextRequest_ == 0) ++nextRequest_;
            request = nextRequest_;
        }
        portEXIT_CRITICAL(&mux_);
        if (!valid) return {Frame::Rejected, "serial_bounds"};
        if (!available) return {Frame::Retry, "serial_pending"};
        if (!queueOssmHatPositionTarget(session, request, encoder, status.maximumSpeedRpm,
                                        status.accelerationRpmPerSec, 100)) return {Frame::Rejected, "serial_submit"};
        portENTER_CRITICAL(&mux_);
        commandSession_ = session;
        commandOwnerRevision_ = owner;
        commandOwnerValidator_ = motorSafetyEventCurrent;
        commandTarget_ = logical;
        serialPatternFrame_ = true;
        // The existing tick commits this frame only on its matching bus ACK.
        // Each frame is a sample of one time law. Never run a point-to-point
        // ramp between these samples. The last acknowledged frame is a hold
        // if the producer or link disappears.
        trajectory_.target(logical);
        pending_ = true;
        pendingId_ = request;
        pendingEncoder_ = encoder;
        pendingProposal_ = {double(logical), 0, true};
        portEXIT_CRITICAL(&mux_);
        return {Frame::Accepted};
    }
    drawn_pattern::Park parkPattern(uint64_t owner, int32_t target, bool draining) {
        using drawn_pattern::Park;
        if (!serialSelected()) {
            if (primingPulse_ || pulse_->isRampGeneratorActive()) return Park::Failed;
            return pulseQueue_.park(*pulse_, target, draining);
        }
        refresh();
        portENTER_CRITICAL(&mux_);
        const bool available = commandOwnerRevision_ == owner && frameValid_ &&
                               position_.fresh(millis()) && commandTarget_ == target;
        const bool acknowledged = !pending_ && !trajectory_.active();
        const bool settled = !position_.running(millis()) &&
                             std::abs(int64_t(position_.current()) - target) <= 1;
        portEXIT_CRITICAL(&mux_);
        if (!available) return Park::Failed;
        return acknowledged && settled ? Park::Parked : Park::Draining;
    }
    bool beginPatternStop(uint64_t owner) {
        if (!motorSafetyEventCurrent(owner)) return false;
        if (!serialSelected()) {
            // Stop planning the entry ramp. Timed frames have no ramp
            // generator; their nominal lookahead is at most 20 ms.
            if (pulseQueue_.filling()) {
                stopPulsesNow();
                return false;
            }
            pulse_->stopMove();
            return true;
        }
        portENTER_CRITICAL(&mux_);
        const bool hold = serialPatternFrame_ && owner == commandOwnerRevision_;
        if (hold) {
            // Allow an already submitted bounded sample to settle. Its ACK
            // must never trigger a move back to the previous sample.
            trajectory_.target(pending_ ? pendingProposal_.position : trajectory_.position());
            serialPatternFrame_ = false;
        }
        portEXIT_CRITICAL(&mux_);
        if (!hold) stopMoveForOwner(owner, motorSafetyEventCurrent);
        return false;
    }
    bool pulseRunning() { return pulse_->isRunning(); }
    void finishPatternStop() {
        if (pulse_->isRunning()) stopPulsesNow();
        else { pulseQueue_.reset(); primingPulse_ = false; }
    }
    int8_t move(int32_t delta, bool blocking = false) override {
        if (!serialSelected()) return static_cast<int8_t>(pulse_->move(delta, blocking));
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
        if (pulseQueue_.owned() || primingPulse_) stopPulsesNow();
        else { drawn_pattern::stopPulseRamp(*pulse_); needsPulseRamp_ = true; }
        if (!serialSelected()) return;
        portENTER_CRITICAL(&mux_);
        trajectory_.reset(position_.current());
        position_.canceled(); pending_ = false; commandSession_ = 0; ++revision_;
        portEXIT_CRITICAL(&mux_);
        cancelOssmHatPositionControl();
    }
    void forceStopAndNewPosition(int32_t logical) override {
        if (!serialSelected()) {
            pulse_->forceStopAndNewPosition(logical);
            needsPulseRamp_ = true;
            pulseQueue_.reset();
            primingPulse_ = false;
            return;
        }
        // Homing calls this only after physical arrival and stop proof. Do
        // not disable an enabled serial session merely to rebase its frame.
        if (isRunning()) { forceStop(); return; }
        setCurrentPosition(logical);
    }
    int32_t commandedTarget() {
        portENTER_CRITICAL(&mux_); const auto value = commandTarget_; portEXIT_CRITICAL(&mux_);
        return value;
    }
    bool serialLatched() const { return serial_.load(); }
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
    void stopPulsesNow() {
        // The ESP32 RMT counter can omit the currently executing command.
        // An emergency abort must flush right now; re-home before using its
        // estimate for another pattern. Normal stops drain to an exact count.
        const bool unstarted = pulseQueue_.filling();
        if (!unstarted && pulse_->isRunning()) calibration.isHomed = false;
        pulse_->forceStopAndNewPosition(unstarted ? pulseQueue_.initialPosition() :
                                                    pulse_->getCurrentPosition());
        needsPulseRamp_ = true;
        pulseQueue_.reset();
        primingPulse_ = false;
    }
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
                    position_.submitted(pendingId_, static_cast<int32_t>(std::round(pendingProposal_.position)), pendingEncoder_);
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
    drawn_pattern::PulseQueue pulseQueue_;
    drawn_pattern::PulsePrimeWait primeWait_;
    std::atomic_bool primingPulse_{false}, needsPulseRamp_{true};
    uint32_t lastPauseAt_ = 0;
    bool serialPatternFrame_ = false;
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
    pulseEngine = pulseStepper;
    axis.begin(pulseStepper);
    return &axis;
}
void registerMotorSerialMotionAuthorization(uint64_t (*session)(), bool (*canMove)(uint64_t)) {
    authorizationCallback.store(canMove);
    sessionCallback.store(session);
}
bool motorMotionUsesSerial() { return axis.serialSelected(); }
bool motorMotionSerialLatched() { return axis.serialLatched(); }
int32_t motorMotionCommandedTarget() { return axis.commandedTarget(); }
bool mirrorMotorMotionFrame(int32_t keepoutSteps) { return axis.mirror(keepoutSteps); }
MotorMotionStatus getMotorMotionStatus() { return axis.status(); }

FastAccelStepper *getMotorPulseStepper() { return pulseEngine; }


drawn_pattern::FrameResult submitPatternFrame(int32_t target, uint64_t revision, bool serial, uint16_t crossing, bool consecutiveHolds) {
    return axis.patternFrame(target, revision, serial, crossing, consecutiveHolds);
}
drawn_pattern::Park parkPatternMotion(uint64_t owner, int32_t target, bool draining) {
    return axis.parkPattern(owner, target, draining);
}
void stopPatternMotion(uint64_t owner) {
    bool drain = false;
    if (!runMotorPatternCommand(owner, [&] {
        drain = axis.beginPatternStop(owner);
        return true;
    }) || !drain) return;
    // The producer's mailbox stays locked. Leave the physical stop mutex
    // free so an emergency can revoke, cut output and flush immediately.
    const uint32_t start = millis();
    while (motorSafetyEventCurrent(owner) && axis.pulseRunning() && millis() - start < drawn_pattern::pulseStopDrainMs)
        vTaskDelay(1);
    runMotorPatternCommand(owner, [] { axis.finishPatternStop(); return true; });
}
