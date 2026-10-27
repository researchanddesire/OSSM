#pragma once

#include <cstdint>
#include <cstring>
#include "AimFrame.h"
#include "ossmHat.h"

// Protocol and state model derived from Alex Warner's faa79768 and the
// YZ-AIM v2.55 manual, pp11,14,15,17. No FC10 incremental command, encoder
// register rewrite, gearing, address, baud or EEPROM write is produced here.
namespace ossm_hat {

constexpr uint8_t positionMotorId = 1;
constexpr uint32_t encoderCountsPerRevolution = 32768;

inline bool fullControlPoll(bool positionRequested, bool controlsPolled,
                            uint32_t now, uint32_t lastControlPoll) {
    return positionRequested || !controlsPolled ||
        static_cast<uint32_t>(now - lastControlPoll) >= 1000;
}

inline bool deferredStopReady(bool pending, uint32_t now, uint32_t requestedAt,
                              bool passive, bool uartReady, bool locked, bool detected) {
    return pending && (passive || static_cast<uint32_t>(now - requestedAt) >= 10000 ||
                       (uartReady && locked && detected));
}

inline size_t buildSerialOutputDisable(uint8_t *frame) {
    // Never select mode1 here. A mode0->1 switch can expose a stale serial
    // target before output-disable and requires separate physical authority.
    return AimFrame::buildWriteSingle(frame, positionMotorId, 1, 0);
}

inline bool serialOutputDisabled(uint16_t mode, uint16_t output) {
    return mode == 1 && (output & 1U) == 0;
}

inline bool retrySerialStopNeeded(const OssmHatStopStatus &stop, uint32_t now,
                                  bool activeFreshSoleMaster) {
    return stop.requested && stop.complete && !stop.verified &&
        !stop.passiveBlocked && activeFreshSoleMaster &&
        static_cast<uint32_t>(now - stop.completedAtMs) >= 1000;
}

inline bool validPositionLimits(uint16_t rpm, uint16_t acceleration) {
    return rpm > 0 && rpm <= 3000 && acceleration > 0 && acceleration <= 60000;
}

enum class PositionActionKind : uint8_t {
    ReadInitial, Hold, Speed, Acceleration, VerifySetup, Enable, VerifyEnable, Target
};

struct PositionAction {
    PositionActionKind kind = PositionActionKind::ReadInitial;
    uint64_t session = 0;
    uint32_t revision = 0;
    uint32_t requestId = 0;
    int32_t targetCounts = 0;
    uint32_t queuedAtMs = 0;
    uint32_t maximumAgeMs = 0;
    uint8_t frame[8] = {};
    uint8_t function = 0;
    uint8_t frameLength = 0;
    uint8_t replyLength = 0;

    bool motion() const {
        return kind == PositionActionKind::Enable ||
               kind == PositionActionKind::VerifyEnable ||
               kind == PositionActionKind::Target;
    }
    bool readsControls() const {
        return function == AimFrame::fcReadHolding;
    }
};

// Retain the identity of an already-committed reply across cancellation. A
// late own FC7B acknowledgement must not be mistaken for a second bus master
// and prevent the priority output-disable transaction that follows it.
class LatePositionReply {
public:
    void expect(const PositionAction &action, uint32_t now) {
        action_ = action;
        started_ = now;
        used_ = 0;
        active_ = true;
    }
    void clear() { active_ = false; }
    bool push(uint8_t byte, uint32_t now) {
        if (!active_) return false;
        if (static_cast<uint32_t>(now - started_) > 100 || used_ == sizeof(bytes_)) {
            active_ = false;
            return false;
        }
        bytes_[used_++] = byte;
        const auto parsed = AimFrame::parseReply(bytes_, used_, positionMotorId,
                                                 action_.function, action_.replyLength);
        if (parsed.status != AimFrame::Status::Ok) return false;
        const uint8_t *frame = bytes_ + parsed.offset;
        const bool own = action_.function == AimFrame::fcWriteSingle ?
            std::memcmp(frame, action_.frame, 6) == 0 :
            action_.function == AimFrame::fcMoveAbsolute ?
            AimFrame::moveReplyValue(frame) == action_.targetCounts :
            action_.readsControls();
        active_ = false;
        return own;
    }
private:
    PositionAction action_;
    uint8_t bytes_[80] = {};
    uint32_t started_ = 0;
    size_t used_ = 0;
    bool active_ = false;
};

// The owner and public mailbox calls serialize this model with stateMux.
// take() commits one transaction. A cancellation after that point cannot
// retract bytes already in flight; stop verification is a separate operation.
class PositionQueue {
public:
    bool prepare(uint64_t session, int32_t hold, uint16_t rpm,
                 uint16_t acceleration, uint32_t now) {
        if (session == 0 || session == cancelledSession_ ||
            !validPositionLimits(rpm, acceleration) ||
            (state_.requested && !state_.faulted)) return false;
        ++revision_;
        state_ = {};
        state_.session = session;
        state_.requested = true;
        state_.pending = true;
        state_.targetCounts = hold;
        state_.maximumSpeedRpm = rpm;
        state_.accelerationRpmPerSec = acceleration;
        hold_ = hold;
        preparedAt_ = now;
        phase_ = Phase::Initial;
        inFlight_ = false;
        haveTarget_ = false;
        enableRequested_ = false;
        targetClock_ = false;
        return true;
    }

    bool enable(uint64_t session, uint32_t now) {
        if (!owns(session) || !state_.prepared || state_.enabled) return false;
        if (enableRequested_) return true;
        enableRequested_ = true;
        enableAt_ = now;
        refreshPending();
        return true;
    }

    bool queue(uint64_t session, uint32_t id, int32_t target, uint16_t rpm,
               uint16_t acceleration, uint32_t now, uint32_t maximumAge) {
        if (!owns(session) || !state_.enabled || id == 0 || maximumAge == 0 ||
            maximumAge > 100 || rpm != state_.maximumSpeedRpm ||
            acceleration != state_.accelerationRpmPerSec) return false;
        if (state_.queuedRequestId != 0) {
            const uint32_t delta = id - state_.queuedRequestId;
            if (delta == 0 || delta >= 0x80000000U) return false;
        }
        state_.queuedRequestId = id;
        state_.targetCounts = target;
        target_ = target;
        targetId_ = id;
        targetAt_ = now;
        targetAge_ = maximumAge;
        haveTarget_ = true;
        refreshPending();
        return true;
    }

    bool owns(uint64_t session) const {
        return session != 0 && state_.session == session &&
               state_.requested && !state_.faulted;
    }

    void cancel(OssmHatPositionFault reason = OssmHatPositionFault::Cancelled) {
        if (state_.faulted) reason = state_.fault;  // retain the first cause
        if (state_.session != 0) cancelledSession_ = state_.session;
        ++revision_;
        state_.prepared = false;
        state_.enabled = false;
        state_.pending = false;
        state_.faulted = state_.requested;
        state_.fault = reason;
        inFlight_ = false;
        haveTarget_ = false;
        enableRequested_ = false;
        phase_ = Phase::Idle;
    }

    bool take(uint32_t now, PositionAction &action) {
        if (!owns(state_.session) || inFlight_) return false;
        if ((!state_.prepared && static_cast<uint32_t>(now - preparedAt_) > 1000) ||
            (haveTarget_ && static_cast<uint32_t>(now - targetAt_) > targetAge_) ||
            (enableRequested_ && static_cast<uint32_t>(now - enableAt_) > 100)) {
            cancel(OssmHatPositionFault::Expired);
            return false;
        }
        action = {};
        action.session = state_.session;
        action.revision = revision_;
        action.queuedAtMs = preparedAt_;
        action.maximumAgeMs = 1000;
        switch (phase_) {
            case Phase::Initial: action.kind = PositionActionKind::ReadInitial; break;
            case Phase::Hold: action.kind = PositionActionKind::Hold; action.targetCounts = hold_; break;
            case Phase::Speed: action.kind = PositionActionKind::Speed; break;
            case Phase::Acceleration: action.kind = PositionActionKind::Acceleration; break;
            case Phase::Verify: action.kind = PositionActionKind::VerifySetup; break;
            case Phase::Prepared:
                if (!enableRequested_) return false;
                action.kind = PositionActionKind::Enable;
                action.targetCounts = hold_;
                action.queuedAtMs = enableAt_;
                action.maximumAgeMs = 100;
                break;
            case Phase::VerifyEnable:
                action.kind = PositionActionKind::VerifyEnable;
                action.targetCounts = hold_;
                action.queuedAtMs = enableAt_;
                action.maximumAgeMs = 100;
                break;
            case Phase::Active:
                if (!haveTarget_ || (targetClock_ &&
                    static_cast<uint32_t>(now - lastTargetAt_) < 20)) return false;
                lastTargetAt_ = now;
                targetClock_ = true;
                action.kind = PositionActionKind::Target;
                action.targetCounts = target_;
                action.requestId = targetId_;
                action.queuedAtMs = targetAt_;
                action.maximumAgeMs = targetAge_;
                break;
            default: return false;
        }
        switch (action.kind) {
            case PositionActionKind::Hold:
            case PositionActionKind::Target:
                action.function = AimFrame::fcMoveAbsolute;
                action.frameLength = AimFrame::buildMoveAbsolute(action.frame, positionMotorId,
                                                                 action.targetCounts);
                action.replyLength = 8;
                break;
            case PositionActionKind::Speed:
            case PositionActionKind::Acceleration:
            case PositionActionKind::Enable: {
                const uint16_t reg = action.kind == PositionActionKind::Speed ? 2 :
                    action.kind == PositionActionKind::Acceleration ? 3 : 1;
                const uint16_t value = reg == 2 ? state_.maximumSpeedRpm :
                    reg == 3 ? state_.accelerationRpmPerSec : 1;
                action.function = AimFrame::fcWriteSingle;
                action.frameLength = AimFrame::buildWriteSingle(action.frame, positionMotorId, reg, value);
                action.replyLength = 8;
                break;
            }
            default:
                action.function = AimFrame::fcReadHolding;
                action.frameLength = AimFrame::buildRead(action.frame, positionMotorId, 0, 4);
                action.replyLength = 13;
                break;
        }
        current_ = action;
        inFlight_ = true;
        refreshPending();
        return true;
    }

    bool current(const PositionAction &action) const {
        return inFlight_ && owns(action.session) && action.revision == revision_ &&
            action.kind == current_.kind && action.requestId == current_.requestId;
    }

    bool superseded(const PositionAction &action) const {
        return action.kind == PositionActionKind::Target &&
            (!haveTarget_ || action.requestId != targetId_);
    }

    void abandon(const PositionAction &action) {
        if (!current(action)) return;
        inFlight_ = false;
        refreshPending();
    }

    void recordVendorReply(const PositionAction &action, const uint8_t *reply,
                           size_t length, uint32_t now) {
        if (!current(action) || action.function != AimFrame::fcMoveAbsolute) return;
        const auto parsed = AimFrame::parseReply(reply, length, positionMotorId,
                                                 action.function, action.replyLength);
        if (parsed.status != AimFrame::Status::Ok) return;
        state_.vendorReplyAvailable = true;
        state_.vendorReplyValue = AimFrame::moveReplyValue(reply + parsed.offset);
        state_.vendorReplyAtMs = now;
    }

    bool complete(const PositionAction &action, const uint8_t *reply,
                  size_t length, uint32_t now) {
        if (!current(action)) return false;
        const auto parsed = AimFrame::parseReply(reply, length, positionMotorId,
                                                 action.function, action.replyLength);
        if (parsed.status != AimFrame::Status::Ok) {
            cancel(OssmHatPositionFault::Protocol);
            return false;
        }
        recordVendorReply(action, reply, length, now);
        const uint8_t *frame = reply + parsed.offset;
        if (action.function == AimFrame::fcWriteSingle &&
            std::memcmp(frame, action.frame, 6) != 0) {
            cancel(OssmHatPositionFault::Protocol);
            return false;
        }
        if (action.function == AimFrame::fcMoveAbsolute &&
            AimFrame::moveReplyValue(frame) != action.targetCounts) {
            cancel(OssmHatPositionFault::Protocol);
            return false;
        }
        if (action.readsControls()) {
            const uint16_t mode = AimFrame::be16(frame + 3);
            const uint16_t output = AimFrame::be16(frame + 5);
            const bool expectedEnabled = action.kind == PositionActionKind::VerifyEnable;
            if (mode != 1 || ((output & 1U) != 0) != expectedEnabled ||
                (action.kind != PositionActionKind::ReadInitial &&
                 (AimFrame::be16(frame + 7) != state_.maximumSpeedRpm ||
                  AimFrame::be16(frame + 9) != state_.accelerationRpmPerSec))) {
                cancel(OssmHatPositionFault::ControlMismatch);
                return false;
            }
        }
        inFlight_ = false;
        switch (action.kind) {
            case PositionActionKind::ReadInitial: phase_ = Phase::Hold; break;
            case PositionActionKind::Hold:
                state_.acknowledgedTargetCounts = action.targetCounts;
                phase_ = Phase::Speed;
                break;
            case PositionActionKind::Speed: phase_ = Phase::Acceleration; break;
            case PositionActionKind::Acceleration: phase_ = Phase::Verify; break;
            case PositionActionKind::VerifySetup:
                state_.prepared = true;
                phase_ = Phase::Prepared;
                break;
            case PositionActionKind::Enable: phase_ = Phase::VerifyEnable; break;
            case PositionActionKind::VerifyEnable:
                state_.enabled = true;
                enableRequested_ = false;
                phase_ = Phase::Active;
                break;
            case PositionActionKind::Target:
                state_.acknowledgedRequestId = action.requestId;
                state_.acknowledgedTargetCounts = action.targetCounts;
                if (haveTarget_ && targetId_ == action.requestId) haveTarget_ = false;
                break;
        }
        state_.acknowledgedAtMs = now;
        refreshPending();
        return true;
    }

    OssmHatPositionStatus status() const { return state_; }

private:
    enum class Phase : uint8_t { Idle, Initial, Hold, Speed, Acceleration,
                                Verify, Prepared, VerifyEnable, Active };
    void refreshPending() {
        state_.pending = !state_.faulted && state_.requested &&
            (!state_.prepared || enableRequested_ || haveTarget_ || inFlight_);
    }
    OssmHatPositionStatus state_;
    Phase phase_ = Phase::Idle;
    PositionAction current_;
    uint64_t cancelledSession_ = 0;
    uint32_t revision_ = 0;
    int32_t hold_ = 0;
    uint32_t preparedAt_ = 0;
    uint32_t enableAt_ = 0;
    bool inFlight_ = false;
    bool haveTarget_ = false;
    bool enableRequested_ = false;
    int32_t target_ = 0;
    uint32_t targetId_ = 0;
    uint32_t targetAt_ = 0;
    uint32_t targetAge_ = 0;
    uint32_t lastTargetAt_ = 0;
    bool targetClock_ = false;
};

}  // namespace ossm_hat
