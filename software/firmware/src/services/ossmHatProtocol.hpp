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

inline bool controlsFresh(const OssmHatControlSnapshot &controls,
                          const OssmHatTelemetry &sample, uint32_t now) {
    return controls.valid && controls.continuity != 0 && fresh(sample, now) &&
           controls.continuity == sample.continuity &&
           static_cast<uint32_t>(now - controls.updatedAtMs) <= 2000;
}

class SampleRate {
  public:
    void accepted(uint32_t now) {
        if (count_ && static_cast<uint32_t>(now - last_) > ossmHatFreshnessMs) {
            count_ = 0;
            next_ = 0;
        }
        times_[next_] = now;
        next_ = (next_ + 1) % capacity;
        if (count_ < capacity) ++count_;
        last_ = now;
    }

    void read(uint32_t now, float &hz, uint32_t &windowMs, uint16_t &count) const {
        count = 0;
        uint32_t first = last_;
        for (uint16_t i = 0; i < count_; ++i) {
            const uint16_t index = (next_ + capacity - count_ + i) % capacity;
            if (static_cast<uint32_t>(now - times_[index]) <= 1000) {
                if (count == 0) first = times_[index];
                ++count;
            }
        }
        windowMs = count > 1 ? static_cast<uint32_t>(last_ - first) : 0;
        hz = windowMs ? static_cast<float>(count - 1) * 1000.0f / windowMs : 0;
    }

  private:
    static constexpr uint16_t capacity = 64;
    uint32_t times_[capacity] = {};
    uint16_t next_ = 0;
    uint16_t count_ = 0;
    uint32_t last_ = 0;
};

// A byte-stream parser rather than an RTU master: the independent PC guard
// remains the sole transmitting master whenever present. Requests are matched
// to replies so a random FC03 response cannot be assigned the wrong registers.
class Decoder {
  public:
    enum Event : uint8_t { None = 0, ExternalRequest = 1, Telemetry = 2, Controls = 4 };

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

    void invalidateControls() { controls_.valid = false; }

    // A transport teardown invalidates every previous coordinate reference,
    // even if its wall-clock gap was shorter than the ordinary freshness limit.
    void restartContinuity() {
        reset();
        sample_.sequence = 0;
        controls_.valid = false;
    }

    // A prioritized owner exchange uses the same read-back cache as regular
    // polling. Never retain a pre-stop mode0 snapshot after a mode1 takeover.
    void acceptControlReadback(uint16_t mode, uint16_t output, uint32_t now) {
        controls_.valid = true;
        controls_.modbusEnable = mode;
        controls_.outputEnable = output;
        controls_.updatedAtMs = now;
        controls_.continuity = sample_.continuity;
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
                            if (id == motorId && start_ == 0 && count_ >= 2) {
                                acceptControlReadback(word(0), word(1), now);
                                events |= Controls;
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
            // ownership too. Explicit owner exchanges validate their own replies
            // separately; all other matching frames establish external ownership.
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
    const OssmHatControlSnapshot &controls() const { return controls_; }
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
        sample_.speedRpm = static_cast<int16_t>(word(0x10)) / 10.0f;
        sample_.temperatureC = static_cast<float>(word(0x12));
        sample_.outputPercent = static_cast<int16_t>(word(0x13)) * (100.0f / 32768.0f);
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
    OssmHatControlSnapshot controls_;
    uint32_t validFrames_ = 0;
    uint32_t crcErrors_ = 0;
};

}  // namespace ossm_hat
