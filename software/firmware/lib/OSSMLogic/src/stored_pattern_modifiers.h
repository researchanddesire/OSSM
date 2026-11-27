#pragma once

#include "drawn_modifiers.h"

namespace drawn_pattern {

// Sparse slots store the original three unsigned 16-bit V1 fields per marker.
// Dense slots store one word per authored tick, independent of in-memory tags.
class StoredModifiers {
    struct SparseRecord { uint16_t start, end, holdMs; };
    static_assert(sizeof(SparseRecord) == 6);
    static constexpr size_t denseBytes = Modifiers::capacity * sizeof(uint16_t);
    static constexpr uint16_t rangeFlag = 0x8000;
    static constexpr uint16_t lastRange = rangeFlag + Modifiers::capacity;
    static constexpr uint16_t firstEscape = maxPauseMs + 1;
    static constexpr uint16_t lastEscape = firstEscape + Modifiers::capacity;
    static_assert(lastRange < maxPauseMs && lastEscape < UINT16_MAX);

    static SparseRecord sparse(const Modifier &m) { return {m.start, m.end, m.holdMs}; }

    static uint16_t valueAt(const uint8_t *data, unsigned tick) {
        uint16_t value;
        std::memcpy(&value, data + tick * sizeof(value), sizeof(value));
        return value;
    }

public:
    enum class Version : uint8_t { V4, V5 };

    static Version versionFor(const Modifiers &items) {
        for (unsigned i = 0; i < items.count; ++i)
            if (items.entries[i].holdMs > originalMaxPauseMs) return Version::V5;
        return Version::V4;
    }
    static size_t bytes(size_t count) { return std::min(count * sizeof(SparseRecord), denseBytes); }

    static uint16_t denseWord(const Modifier &item) {
        if (item.isVibration()) return rangeFlag | item.end;
        const uint16_t ms = item.holdMs;
        if (ms >= rangeFlag && ms <= lastRange) return firstEscape + ms - rangeFlag;
        return ms;
    }
    // Zero is an empty tick. Each nonzero code has at most one valid meaning.
    // V5 escapes exactly those longer pauses that share V4's range codes.
    static bool decodeWord(uint16_t tick, uint16_t word, Version version, Modifier &out) {
        if (word > rangeFlag && word <= lastRange) {
            out = Modifier(TimeRangeEffect{TimeRangeEffect::Kind::Vibration, tick, uint16_t(word - rangeFlag)});
        } else {
            if (!word || word == rangeFlag || word > lastEscape) return false;
            const uint16_t ms = word >= firstEscape ? uint16_t(rangeFlag + word - firstEscape) : word;
            if (version == Version::V4 && ms > originalMaxPauseMs) return false;
            out = Modifier(InstantEffect{InstantEffect::Kind::Pause, tick, ms});
        }
        return out.valid();
    }

    static void encode(uint8_t *data, const Modifiers &items) {
        const size_t length = bytes(items.count);
        if (!length) return;
        if (length < denseBytes) {
            for (unsigned i = 0; i < items.count; ++i) {
                const auto record = sparse(items.entries[i]);
                std::memcpy(data + i * sizeof(record), &record, sizeof(record));
            }
            return;
        }
        std::memset(data, 0, length);
        for (unsigned i = 0; i < items.count; ++i) {
            const auto &item = items.entries[i];
            const auto value = denseWord(item);
            std::memcpy(data + item.start * sizeof(value), &value, sizeof(value));
        }
    }

    static bool matches(const uint8_t *data, const Modifiers &items) {
        const bool dense = bytes(items.count) == denseBytes;
        for (unsigned i = 0; i < items.count; ++i) {
            const auto &item = items.entries[i];
            if (dense) {
                if (valueAt(data, item.start) != denseWord(item)) return false;
            } else {
                const auto record = sparse(item);
                if (std::memcmp(data + i * sizeof(record), &record, sizeof(record))) return false;
            }
        }
        return true;
    }

    // Validate in place during a catalog read. Only the selected playback
    // or edit needs to expand the complete marker list.
    static bool decode(const uint8_t *data, unsigned count, Version version, Modifiers *out = nullptr) {
        if (count > Modifiers::capacity || (count && !data) || (out && !out->reserve(count))) return false;
        unsigned seen = 0;
        Modifier first{}, previous{};
        auto accept = [&](const Modifier &item) {
            if (seen >= count || !item.valid() || (seen && !Modifiers::ordered(previous, item))) return false;
            if (out) out->entries[seen] = item;
            if (!seen) first = item;
            previous = item;
            ++seen;
            return true;
        };
        if (bytes(count) < denseBytes) {
            for (unsigned i = 0; i < count; ++i) {
                SparseRecord record;
                std::memcpy(&record, data + i * sizeof(record), sizeof(record));
                if (version == Version::V4 && record.holdMs > originalMaxPauseMs) return false;
                if (!accept(Modifier::fromV1(record.start, record.end, record.holdMs))) return false;
            }
        } else {
            for (uint16_t tick = 0; tick < Modifiers::capacity; ++tick) {
                const uint16_t value = valueAt(data, tick);
                if (!value) continue;
                Modifier item;
                if (!decodeWord(tick, value, version, item) || !accept(item)) return false;
            }
        }
        if (seen != count || !Modifiers::validSeam(first, previous, seen)) return false;
        if (out) out->count = count;
        return true;
    }
};
}  // namespace drawn_pattern
