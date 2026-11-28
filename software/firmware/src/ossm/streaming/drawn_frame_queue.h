#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <new>
#include <type_traits>

#include "drawn_modifier_motion.h"
#include "drawn_prepared_pulses.h"

namespace streaming::frame_plan {

struct Cursor {
    drawn_pattern::MarkedPhase phase;
    drawn_pattern::VibrationClock vibration;
    int32_t target = 0;
    bool targetKnown = false;
};

struct Frame : Cursor {
    int32_t from = 0;
    uint32_t planUs = 0;
    uint16_t crossing = 32768;
    bool valid = false;
    bool offered = false;
};

// Software speculation only. Most 2 ms frames change just a few words of
// the previous clock. Store exactly those words, including their IEEE bits.
// All operations require the caller's ring lock. Only the feeder mutates
// the returned front (offered), which stays valid across pushes and retries.
class FrameQueue {
public:
    static constexpr unsigned capacity = 64;
    static constexpr unsigned maxRetained = 8;

    bool hasStorage() const { return static_cast<bool>(storage_); }
    unsigned size() const { return count_; }
    bool full() const {
        return count_ == capacity || storageWords - used_ < maxRecordWords;
    }

    void clear(const Cursor &accepted) {
        accepted_ = accepted;
        firstRecord_ = count_ = read_ = write_ = used_ = 0;
        decoded_ = false;
    }

    // previousTail is the exact predecessor used to calculate row. A word
    // mask and absolute replacements also encode a completely different
    // cursor. Reserve its worst-case record before accepting any row.
    bool push(const Frame &row, const Cursor &previousTail) {
        if (full()) return false;
        const Cursor &next = row;
        unsigned pos = write_;
        put(pos, static_cast<uint32_t>(row.from));
        put(pos, static_cast<uint32_t>(row.target));
        put(pos, row.planUs);
        put(pos, row.crossing | (uint32_t(row.valid) << 16) | (uint32_t(row.offered) << 17) |
                     (uint32_t(row.targetKnown) << 18) |
                     (uint32_t(row.vibration.gain.goal() == previousTail.vibration.gain.goal()) << 19));
        const unsigned maskStart = pos;
        pos = advance(pos, maskWords);
        unsigned words = headerWords;
        for (unsigned group = 0; group < maskWords; ++group) {
            uint32_t mask = 0;
            const unsigned first = group * 32;
            const unsigned width = std::min(32u, cursorWords - first);
            for (unsigned bit = 0; bit < width; ++bit) {
                const auto word = loadWord(next, first + bit);
                if (word != loadWord(previousTail, first + bit)) {
                    mask |= uint32_t{1} << bit;
                    put(pos, word);
                    ++words;
                }
            }
            storage_[advance(maskStart, group)] = mask;
        }
        lengths_[record(count_)] = static_cast<uint8_t>(words);
        used_ += words;
        write_ = pos;
        ++count_;
        return true;
    }

    // Read only the two-word motor projection of successive immutable
    // rows. A gain-goal change needs its own accepted-frame timestamp.
    void successors(drawn_pattern::PreparedPulses &out, unsigned maximum) const {
        unsigned pos = read_;
        const unsigned end = std::min({count_, maximum + 1, drawn_pattern::PreparedPulses::capacity + 1});
        for (unsigned i = 1; i < end; ++i) {
            pos = advance(pos, lengths_[record(i - 1)]);
            const auto flags = storage_[advance(pos, 3)];
            constexpr uint32_t eligible = (1u << 16) | (1u << 18) | (1u << 19);
            if ((flags & eligible) != eligible) break;
            auto &target = out.targets[out.count++];
            const uint32_t bits = storage_[advance(pos, 1)];
            std::memcpy(&target.position, &bits, sizeof(bits));
            target.crossing = static_cast<uint16_t>(flags);
            target.stationary = storage_[pos] == bits;
        }
    }

    Frame *front() {
        if (!count_) return nullptr;
        if (!decoded_) {
            static_cast<Cursor &>(front_) = accepted_;
            apply(read_, front_);
            const uint32_t from = storage_[read_];
            std::memcpy(&front_.from, &from, sizeof(from));
            front_.planUs = storage_[advance(read_, 2)];
            const auto flags = storage_[advance(read_, 3)];
            front_.crossing = static_cast<uint16_t>(flags);
            front_.valid = (flags & (1u << 16)) != 0;
            front_.offered = (flags & (1u << 17)) != 0;
            decoded_ = true;
        }
        return &front_;
    }

    void pop() {
        const auto *row = front();
        if (!row) return;
        accepted_ = *row;
        const unsigned length = lengths_[firstRecord_];
        used_ -= length;
        read_ = advance(read_, length);
        firstRecord_ = record(1);
        --count_;
        decoded_ = false;
    }

    // Return the exact boundary for retargeting. Decode at most 16 ms of
    // previously planned rows, using integer copies only. Discarded suffix
    // rows and in-flight results are invalidated by the caller's generation.
    unsigned truncate(unsigned retained, Cursor &newTail) {
        retained = std::min({retained, count_, maxRetained});
        newTail = accepted_;
        unsigned pos = read_;
        unsigned retainedWords = 0;
        for (unsigned i = 0; i < retained; ++i) {
            apply(pos, newTail);
            const unsigned length = lengths_[record(i)];
            retainedWords += length;
            pos = advance(pos, length);
        }
        count_ = retained;
        used_ = retainedWords;
        write_ = pos;
        if (!retained) decoded_ = false;
        return retained;
    }

private:
    static_assert(std::is_trivially_copyable<Cursor>::value, "cursor must be a value-only snapshot");
    static_assert(sizeof(drawn_pattern::MarkedPhase) % sizeof(uint32_t) == 0 &&
                  sizeof(drawn_pattern::VibrationClock) % sizeof(uint32_t) == 0, "encode complete clock words");
    static_assert(alignof(drawn_pattern::MarkedPhase) >= alignof(uint32_t) &&
                  alignof(drawn_pattern::VibrationClock) >= alignof(uint32_t), "word-aligned clock storage");
    static constexpr unsigned phaseWords = sizeof(drawn_pattern::MarkedPhase) / sizeof(uint32_t);
    static constexpr unsigned cursorWords = phaseWords + sizeof(drawn_pattern::VibrationClock) / sizeof(uint32_t);
    static constexpr unsigned maskWords = (cursorWords + 31) / 32;
    static constexpr unsigned headerWords = 4 + maskWords;
    static constexpr unsigned maxRecordWords = headerWords + cursorWords;
    static constexpr unsigned storageWords = 960;
    static_assert(maxRecordWords <= UINT8_MAX, "record length fits one byte");
    static_assert(storageWords / maxRecordWords >= 12, "even all-literal cursors preserve the old reserve");
    static_assert((capacity & (capacity - 1)) == 0, "record indices wrap at a power of two");

    Cursor accepted_;
    Frame front_;
    // Allocate before starting the worker. The unchanged 64-row ring and
    // controller can occupy separate heap blocks on a fragmented ESP32.
    std::unique_ptr<uint32_t[]> storage_{new (std::nothrow) uint32_t[storageWords]};
    uint8_t lengths_[capacity];
    unsigned firstRecord_ = 0, count_ = 0, read_ = 0, write_ = 0, used_ = 0;
    bool decoded_ = false;

    unsigned record(unsigned offset) const { return (firstRecord_ + offset) & (capacity - 1); }
    static unsigned advance(unsigned pos, unsigned words) {
        const unsigned end = pos + words;
        return end < storageWords ? end : end - storageWords;
    }
    void put(unsigned &pos, uint32_t word) {
        storage_[pos] = word;
        pos = advance(pos, 1);
    }
    // Address complete member objects separately. A Frame may reuse tail
    // padding in its Cursor base, so never patch the base representation.
    template<class Byte, class State>
    static Byte *clockWord(State &cursor, unsigned index) {
        Byte *word = index < phaseWords
                         ? reinterpret_cast<Byte *>(&cursor.phase) + index * sizeof(uint32_t)
                         : reinterpret_cast<Byte *>(&cursor.vibration) + (index - phaseWords) * sizeof(uint32_t);
        // The members and each four-byte offset are aligned. Tell Xtensa's
        // optimizer so memcpy emits one word load/store without type punning.
        return static_cast<Byte *>(__builtin_assume_aligned(word, alignof(uint32_t)));
    }
    static uint32_t loadWord(const Cursor &cursor, unsigned index) {
        uint32_t word;
        std::memcpy(&word, clockWord<const unsigned char>(cursor, index), sizeof(word));
        return word;
    }
    void apply(unsigned pos, Cursor &cursor) const {
        const uint32_t target = storage_[advance(pos, 1)];
        std::memcpy(&cursor.target, &target, sizeof(target));
        cursor.targetKnown = (storage_[advance(pos, 3)] & (1u << 18)) != 0;
        const unsigned masks = advance(pos, 4);
        pos = advance(pos, headerWords);
        for (unsigned group = 0; group < maskWords; ++group) {
            uint32_t mask = storage_[advance(masks, group)];
            while (mask) {
                const unsigned bit = static_cast<unsigned>(__builtin_ctz(mask));
                const unsigned index = group * 32 + bit;
                std::memcpy(clockWord<unsigned char>(cursor, index), &storage_[pos], sizeof(uint32_t));
                pos = advance(pos, 1);
                mask &= mask - 1;
            }
        }
    }
};

static_assert(sizeof(FrameQueue) <= 12 * sizeof(Frame) + 2048, "bounded software reserve growth");

} // namespace streaming::frame_plan
