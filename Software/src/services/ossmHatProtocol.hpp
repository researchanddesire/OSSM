#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "AimFrame.h"
#include "ossmHat.h"

// Register map, CRC and leading-zero handling follow Alex Warner's
// feature/rs485-hat-aim57-test (KinkyMakers/OSSM-Hardware, faa79768).
// AimFrame is imported unchanged under Software/lib/AimFrame.
namespace ossm_hat {

constexpr uint8_t motorId = 1;
constexpr uint16_t telemetryStart = 0x0E;
constexpr uint16_t telemetryCount = 10;
constexpr uint32_t replyTimeoutMs = 100;

inline bool fresh(const OssmHatTelemetry &sample, uint32_t now) {
    return sample.sequence != 0 &&
           static_cast<uint32_t>(now - sample.updatedAtMs) <= ossmHatFreshnessMs;
}

// A byte-stream parser rather than an RTU master: the independent PC guard
// remains the sole transmitting master whenever present. Requests are matched
// to replies so a random FC03 response cannot be assigned the wrong registers.
class Decoder {
  public:
    enum Event : uint8_t { None = 0, ExternalRequest = 1, Telemetry = 2 };

    void reset() {
        used_ = 0;
        pending_ = false;
    }

    void expectRead(uint8_t id, uint16_t start, uint16_t count, uint32_t now) {
        pending_ = true;
        pendingId_ = id;
        start_ = start;
        count_ = count;
        requestedAt_ = now;
    }

    uint8_t push(uint8_t byte, uint32_t now) {
        if (used_ == sizeof(buffer_)) discard(1);
        buffer_[used_++] = byte;
        uint8_t events = None;
        while (used_ >= 2) {
            const uint8_t id = buffer_[0];
            const uint8_t fc = buffer_[1];
            if (id == 0 || id > 247) {
                discard(1);  // includes the HAT's known leading 0x00 glitch
                continue;
            }
            if (fc == AimFrame::fcReadHolding) {
                if (used_ < 3) break;
                // Only an outstanding matching request establishes a reply's
                // length. Unsolicited/noisy FC03 headers cannot hold the
                // parser waiting for an invented large byte count.
                const bool possibleReply = pending_ && id == pendingId_ &&
                    buffer_[2] == 2 * count_ &&
                    static_cast<uint32_t>(now - requestedAt_) <= replyTimeoutMs;
                const size_t replyLength = possibleReply ? 5 + buffer_[2] : 0;
                // Requests have eight bytes and an address/count pair. Check
                // these first; a reply's byte count is not a register address.
                if (used_ >= 8) {
                    const uint16_t reg = AimFrame::be16(buffer_ + 2);
                    const uint16_t count = AimFrame::be16(buffer_ + 4);
                    if (count > 0 && count <= AimFrame::maxReadCount &&
                        reg < 0x1A && reg + count <= 0x1A && crcOk(8)) {
                        expectRead(id, reg, count, now);
                        ++validFrames_;
                        events |= ExternalRequest;
                        discard(8);
                        continue;
                    }
                }
                if (possibleReply && used_ >= replyLength) {
                    if (crcOk(replyLength)) {
                        ++validFrames_;
                        if (pending_ && id == pendingId_ &&
                            buffer_[2] == 2 * count_ &&
                            static_cast<uint32_t>(now - requestedAt_) <= replyTimeoutMs) {
                            if (id == motorId && start_ <= telemetryStart &&
                                start_ + count_ >= telemetryStart + telemetryCount) {
                                decode(now);
                                events |= Telemetry;
                            }
                            pending_ = false;
                        }
                        discard(replyLength);
                        continue;
                    }
                    ++crcErrors_;
                    discard(1);
                    continue;
                }
                if (used_ < 8 || (possibleReply && used_ < replyLength)) break;
                ++crcErrors_;
                discard(1);
                continue;
            }
            // Observe the guard's emergency register writes as external bus
            // ownership too. The service itself never constructs these frames.
            if (fc == AimFrame::fcWriteSingle || fc == AimFrame::fcMove ||
                fc == AimFrame::fcMoveAbsolute) {
                if (used_ < 8) break;
                if (crcOk(8)) {
                    pending_ = false;
                    ++validFrames_;
                    events |= ExternalRequest;
                    discard(8);
                    continue;
                }
            } else if (fc == AimFrame::fcWriteMultiple) {
                if (used_ < 8) break;
                // A normal FC10 acknowledgement is eight bytes, regardless
                // of the variable-length request's byte-count field.
                if (used_ >= 8 && crcOk(8)) {
                    pending_ = false;
                    ++validFrames_;
                    events |= ExternalRequest;
                    discard(8);
                    continue;
                }
                const size_t length = 9 + buffer_[6];
                if (length <= sizeof(buffer_) && used_ < length) break;
                if (length <= sizeof(buffer_) && crcOk(length)) {
                    pending_ = false;
                    ++validFrames_;
                    events |= ExternalRequest;
                    discard(length);
                    continue;
                }
            } else if (fc == (AimFrame::fcReadHolding | 0x80)) {
                if (used_ < 5) break;
                if (crcOk(5)) {
                    pending_ = false;
                    ++validFrames_;
                    discard(5);
                    continue;
                }
            }
            discard(1);
        }
        return events;
    }

    const OssmHatTelemetry &sample() const { return sample_; }
    uint32_t validFrames() const { return validFrames_; }
    uint32_t crcErrors() const { return crcErrors_; }

  private:
    bool crcOk(size_t length) const {
        const uint16_t crc = AimFrame::crc16(buffer_, length - 2);
        return buffer_[length - 2] == (crc & 0xFF) &&
               buffer_[length - 1] == (crc >> 8);
    }

    void discard(size_t count) {
        used_ -= count;
        if (used_) std::memmove(buffer_, buffer_ + count, used_);
    }

    uint16_t word(uint16_t reg) const {
        return AimFrame::be16(buffer_ + 3 + 2 * (reg - start_));
    }

    void decode(uint32_t now) {
        if (sample_.sequence == 0 ||
            static_cast<uint32_t>(now - sample_.updatedAtMs) > ossmHatFreshnessMs) {
            ++sample_.continuity;
            if (sample_.continuity == 0) ++sample_.continuity;
        }
        const uint32_t raw = static_cast<uint32_t>(word(0x16)) |
                             (static_cast<uint32_t>(word(0x17)) << 16);
        sample_.positionCounts = static_cast<int32_t>(raw);
        sample_.alarm = word(0x0E);
        sample_.currentAmps = static_cast<float>(word(0x0F)) / 2000.0f;
        sample_.voltageVolts = static_cast<float>(word(0x11)) / 327.0f;
        sample_.updatedAtMs = now;
        ++sample_.sequence;
        if (sample_.sequence == 0) ++sample_.sequence;
    }

    uint8_t buffer_[96] = {};
    size_t used_ = 0;
    bool pending_ = false;
    uint8_t pendingId_ = 0;
    uint16_t start_ = 0;
    uint16_t count_ = 0;
    uint32_t requestedAt_ = 0;
    OssmHatTelemetry sample_;
    uint32_t validFrames_ = 0;
    uint32_t crcErrors_ = 0;
};

}  // namespace ossm_hat
