#pragma once

#include "drawn_modifiers.h"

namespace drawn_pattern {

// Sparse slots store six bytes per marker. Dense slots store one 16-bit value
// per authored tick. Even 1,000 different pauses fit in 2,000 bytes per slot.
class StoredModifiers {
    static constexpr size_t denseBytes = Modifiers::capacity * sizeof(uint16_t);
    static constexpr uint16_t rangeFlag = 0x8000;

    static uint16_t tag(const Modifier &item) {
        return item.pause() ? item.holdMs : uint16_t(rangeFlag | item.end);
    }
    static uint16_t valueAt(const uint8_t *data, unsigned tick) {
        uint16_t value;
        std::memcpy(&value, data + tick*sizeof(value), sizeof(value));
        return value;
    }
public:
    static size_t bytes(size_t count) { return std::min(count * sizeof(Modifier), denseBytes); }

    static void encode(uint8_t *data, const Modifiers &items) {
        const size_t length = bytes(items.count);
        if (!length) return;
        if (length < denseBytes) {
            std::memcpy(data, items.entries, length);
            return;
        }
        std::memset(data, 0, length);
        for (unsigned i = 0; i < items.count; ++i) {
            const auto &item = items.entries[i];
            const auto value = tag(item);
            std::memcpy(data + item.start*sizeof(value), &value, sizeof(value));
        }
    }

    static bool matches(const uint8_t *data, const Modifiers &items) {
        const size_t length = bytes(items.count);
        if (length < denseBytes) return !length || std::memcmp(data, items.entries, length) == 0;
        for (unsigned i = 0; i < items.count; ++i) {
            const auto &item = items.entries[i];
            if (valueAt(data, item.start) != tag(item)) return false;
        }
        return true;
    }

    // Validate directly from flash bytes. Loading the catalog never needs to
    // expand every slot into its maximum playable representation.
    static bool decode(const uint8_t *data, unsigned count, Modifiers *out = nullptr) {
        if (count > Modifiers::capacity || (out && !out->reserve(count))) return false;
        unsigned seen = 0;
        Modifier first{}, previous{};
        auto accept = [&](Modifier item) {
            if (seen >= count || !Modifiers::validItem(item) ||
                (seen && !Modifiers::ordered(previous,item))) return false;
            if (out) out->entries[seen] = item;
            if (!seen) first = item;
            previous = item;
            ++seen;
            return true;
        };
        if (bytes(count) < denseBytes) {
            for (unsigned i = 0; i < count; ++i) {
                Modifier item;
                std::memcpy(&item, data + i*sizeof(item), sizeof(item));
                if (!accept(item)) return false;
            }
        } else {
            for (uint16_t tick = 0; tick < Modifiers::capacity; ++tick) {
                const uint16_t value = valueAt(data, tick);
                if (!value) continue;
                const Modifier item = (value & rangeFlag)
                    ? Modifier{tick, uint16_t(value ^ rangeFlag), 0}
                    : Modifier{tick, tick, value};
                if (!accept(item)) return false;
            }
        }
        if (seen != count || !Modifiers::validSeam(first,previous,seen)) return false;
        if (out) out->count = count;
        return true;
    }
};
}  // namespace drawn_pattern
