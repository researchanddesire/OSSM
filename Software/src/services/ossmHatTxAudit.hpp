#pragma once

#include <cstddef>
#include <cstdint>
#include "ossmHat.h"

namespace ossm_hat {

// Call only after the UART has accepted bytes. The caller owns stateMux;
// this model performs no I/O, callbacks, allocations or frame transmission.
class TxAudit {
public:
    void accepted(const uint8_t *frame, size_t length, size_t written, uint32_t now) {
        if (frame == nullptr || length < 2 || written == 0 || frame[1] == 3) return;
        const uint8_t fc = frame[1];
        if (fc != 6 && fc != 0x7B && fc != 0x78 && fc != 0x10 && fc != 0x7A) return;
        ++state_.controlWrites;
        state_.lastFunction = fc;
        state_.lastRegister = UINT16_MAX;
        state_.lastValue = 0;
        state_.lastAtMs = now;
        state_.lastAcceptedBytes = written > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(written);
        state_.lastCompleteFrame = written == length;
        state_.lastType = OssmHatControlWriteType::Other;
        if (fc == 0x7B && length >= 6) {
            const uint32_t bits = (static_cast<uint32_t>(frame[2]) << 24) |
                (static_cast<uint32_t>(frame[3]) << 16) |
                (static_cast<uint32_t>(frame[4]) << 8) | frame[5];
            state_.lastType = OssmHatControlWriteType::AbsoluteTarget;
            state_.lastValue = static_cast<int32_t>(bits);
            ++state_.absoluteTargets;
        } else if (fc == 6 && length >= 6) {
            state_.lastRegister = (static_cast<uint16_t>(frame[2]) << 8) | frame[3];
            state_.lastValue = (static_cast<uint16_t>(frame[4]) << 8) | frame[5];
            switch (state_.lastRegister) {
                case 0:
                    state_.lastType = OssmHatControlWriteType::ModeWrite;
                    ++state_.modeWrites;
                    break;
                case 1:
                    if (state_.lastValue & 1U) {
                        state_.lastType = OssmHatControlWriteType::OutputEnable;
                        ++state_.outputEnables;
                    } else {
                        state_.lastType = OssmHatControlWriteType::OutputDisable;
                        ++state_.outputDisables;
                    }
                    break;
                case 2:
                    state_.lastType = OssmHatControlWriteType::SpeedSetup;
                    ++state_.setupWrites;
                    break;
                case 3:
                    state_.lastType = OssmHatControlWriteType::AccelerationSetup;
                    ++state_.setupWrites;
                    break;
                default: break;
            }
        }
    }
    OssmHatTxAudit snapshot() const { return state_; }
private:
    OssmHatTxAudit state_;
};

}  // namespace ossm_hat
