#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include "drawn_waveform.h"

namespace streaming {

constexpr uint8_t patternSlotCount = 4;
constexpr int selectedPatternSlot = -1;
constexpr size_t maxPatternNameBytes = 24;

// On-flash ABI of the original curveV1 record. Keep this layout readable.
struct SavedPattern {
    uint32_t version = 1;
    uint32_t periodMs = 0;
    uint32_t length = 0;
    char samples[drawn_pattern::maxSamples * 2] = {};
};
static_assert(sizeof(SavedPattern) == 268);

inline bool validSavedPattern(const SavedPattern &value) {
    drawn_pattern::Waveform curve;
    return value.version == 1 && value.length <= sizeof(value.samples) &&
           curve.decode(value.samples, value.length, value.periodMs);
}

inline bool makeSavedPattern(SavedPattern &out, const char *samples, size_t length,
                             uint32_t periodMs) {
    if (!samples || length > sizeof(out.samples)) return false;
    out.periodMs = periodMs;
    out.length = length;
    std::memcpy(out.samples, samples, length);
    return validSavedPattern(out);
}

// Reject controls, malformed, overlong, surrogate and out-of-range UTF-8.
// The byte length comes from JSON, so embedded NULs cannot truncate a name.
inline bool validPatternName(const char *name, size_t length) {
    if (!name || !length || length > maxPatternNameBytes ||
        name[0] == ' ' || name[length - 1] == ' ') return false;
    const auto *s = reinterpret_cast<const uint8_t *>(name);
    for (size_t i = 0; i < length;) {
        const uint8_t lead = s[i++];
        if (lead < 0x20 || lead == 0x7f) return false;
        if (lead < 0x80) continue;
        if (lead < 0xc2 || lead > 0xf4) return false;
        const size_t count = lead < 0xe0 ? 1 : lead < 0xf0 ? 2 : 3;
        if (length - i < count) return false;
        const uint8_t first = s[i];
        if ((lead == 0xc2 && first < 0xa0) || (lead == 0xe0 && first < 0xa0) ||
            (lead == 0xed && first > 0x9f) || (lead == 0xf0 && first < 0x90) ||
            (lead == 0xf4 && first > 0x8f)) return false;
        for (size_t j = 0; j < count; ++j) {
            if ((s[i++] & 0xc0) != 0x80) return false;
        }
    }
    return true;
}

struct PatternStorageState {
    bool homing = false;
    bool pulsesRunning = false;
    bool armed = false;
    bool serialQuiet = true;
    bool hasFeedback = false;
    bool fresh = false;
    float speedRpm = 0;
    bool stopped() const {
        return !homing && !pulsesRunning && !armed && serialQuiet &&
               (!hasFeedback || (fresh && std::abs(speedRpm) <= 0.1f));
    }
};

struct PatternSlotInfo {
    uint32_t periodMs = 0;
    char name[maxPatternNameBytes + 1] = {};
};
struct PatternCatalog {
    uint8_t selected = 0;
    PatternSlotInfo slots[patternSlotCount]{};
};

class SavedPatternLibrary {
    struct Slot {
        SavedPattern value;
        char name[maxPatternNameBytes + 1]{};
    };
    struct Record {
        uint32_t version = 2;
        uint32_t selected = 0;
        Slot slots[patternSlotCount]{};
    };
    static_assert(sizeof(Record) == 1192);
    Record record_;
    static constexpr const char *key = "libraryV2";

    bool occupied(int index) const {
        return index >= 0 && index < patternSlotCount && record_.slots[index].value.length;
    }
    bool validRecord() const {
        if (record_.version != 2 || !occupied(record_.selected)) return false;
        for (const auto &slot : record_.slots) {
            if (!slot.value.length) continue;
            const size_t length = strnlen(slot.name, sizeof(slot.name));
            if (!validPatternName(slot.name, length) || !validSavedPattern(slot.value)) return false;
        }
        return true;
    }

public:
    // Both callbacks return true only when exactly size bytes were transferred.
    // Read directly into the static live record at boot. No library-sized stack copy.
    template <typename Read> void load(Read read) {
        if (read(key, &record_, sizeof(record_)) && validRecord()) return;
        record_.version = 2;
        record_.selected = 0;
        for (auto &slot : record_.slots) slot = Slot{};
        auto &legacy = record_.slots[0];
        if (read("curveV1", &legacy.value, sizeof(legacy.value)) && validSavedPattern(legacy.value)) {
            std::strcpy(legacy.name, "Saved pattern");
        } else {
            legacy.value.length = 0;
        }
    }
    bool any() const { return occupied(record_.selected); }
    bool get(SavedPattern &out, int index = selectedPatternSlot) const {
        if (index == selectedPatternSlot) index = record_.selected;
        if (!occupied(index)) return false;
        out = record_.slots[index].value;
        return true;
    }
    PatternCatalog catalog() const {
        PatternCatalog result{};
        result.selected = record_.selected;
        for (uint8_t i = 0; i < patternSlotCount; ++i) {
            if (!occupied(i)) continue;
            result.slots[i].periodMs = record_.slots[i].value.periodMs;
            std::memcpy(result.slots[i].name, record_.slots[i].name, sizeof(result.slots[i].name));
        }
        return result;
    }
    template <typename Write>
    bool save(int index, const char *name, size_t nameLength, const SavedPattern &value, Write write) {
        if (index == selectedPatternSlot) index = record_.selected;
        if (index < 0 || index >= patternSlotCount || !validSavedPattern(value)) return false;
        auto &slot = record_.slots[index];
        if (!name) {
            if (nameLength) return false;
            name = occupied(index) ? slot.name : "Saved pattern";
            nameLength = std::strlen(name);
        }
        if (!validPatternName(name, nameLength)) return false;
        if (slot.value.length == value.length && slot.value.periodMs == value.periodMs &&
            std::memcmp(slot.value.samples, value.samples, value.length) == 0 &&
            std::strlen(slot.name) == nameLength && std::memcmp(slot.name, name, nameLength) == 0)
            return true;
        const auto previous = slot;
        const auto previousSelected = record_.selected;
        if (!any()) record_.selected = index;
        slot.value = value;
        if (name != slot.name) {
            std::memset(slot.name, 0, sizeof(slot.name));
            std::memcpy(slot.name, name, nameLength);
        }
        if (write(key, &record_, sizeof(record_))) return true;
        slot = previous;
        record_.selected = previousSelected;
        return false;
    }
    template <typename Write> bool select(int index, Write write) {
        if (!occupied(index)) return false;
        if (record_.selected == static_cast<uint32_t>(index)) return true;
        const auto previous = record_.selected;
        record_.selected = index;
        if (write(key, &record_, sizeof(record_))) return true;
        record_.selected = previous;
        return false;
    }
};
}  // namespace streaming
