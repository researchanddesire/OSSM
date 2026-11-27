#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <new>
#include "drawn_waveform.h"

namespace streaming {

constexpr uint8_t patternSlotCount = 4;
constexpr int selectedPatternSlot = -1;
constexpr size_t maxPatternNameBytes = 24;

// Preserve the 268-byte curveV1 layout. Version 1 is Q8, version 2 is big-endian Q16.
struct SavedPattern {
    uint32_t version = 1;
    uint32_t periodMs = 0;
    uint32_t length = 0;
    char samples[drawn_pattern::maxSamples * 2] = {};
    bool highPrecision() const { return version == 2; }
};
static_assert(sizeof(SavedPattern) == 268);

inline bool validSavedPattern(const SavedPattern &value) {
    drawn_pattern::Waveform curve;
    return (value.version == 1 || value.version == 2) && value.length <= sizeof(value.samples) &&
           curve.decode(value.samples, value.length, value.periodMs, value.highPrecision());
}

inline bool makeSavedPattern(SavedPattern &out, const char *samples, size_t length,
                             uint32_t periodMs, bool highPrecision = false) {
    if (!samples || length > sizeof(out.samples)) return false;
    out.version = highPrecision ? 2 : 1;
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
        uint32_t version = 3;
        uint32_t selected = 0;
        Slot slots[patternSlotCount]{};
    };
    static_assert(sizeof(Record) == 1192);
    Record record_;
    static constexpr const char *key = "libraryV3";

    bool occupied(int index) const {
        return index >= 0 && index < patternSlotCount && record_.slots[index].value.length;
    }
    bool validRecord(uint32_t version) const {
        if (record_.version != version || !occupied(record_.selected)) return false;
        for (const auto &slot : record_.slots) {
            if (!slot.value.length) continue;
            const size_t length = strnlen(slot.name, sizeof(slot.name));
            if (version == 2 && slot.value.version != 1) return false;
            if (!validPatternName(slot.name, length) || !validSavedPattern(slot.value)) return false;
        }
        return true;
    }

public:
    // Both callbacks return true only when exactly size bytes were transferred.
    // The caller supplies the temporary record storage.
    template <typename Read> void load(Read read) {
        if (read(key, &record_, sizeof(record_)) && validRecord(3)) return;
        if (read("libraryV2", &record_, sizeof(record_)) && validRecord(2)) {
            record_.version = 3; // Persist only after a successful, deliberate change.
            return;
        }
        record_.version = 3;
        record_.selected = 0;
        for (auto &slot : record_.slots) slot = Slot{};
        auto &legacy = record_.slots[0];
        if (read("curveV1", &legacy.value, sizeof(legacy.value)) &&
            legacy.value.version == 1 && validSavedPattern(legacy.value)) {
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
        if (slot.value.version == value.version && slot.value.length == value.length &&
            slot.value.periodMs == value.periodMs &&
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
// Keep the slot index, not all four curves, resident between explicit reads
// and stopped edits. The time-critical player owns its decoded waveform.
class SavedPatternCache {
    PatternCatalog catalog_{};
public:
    const PatternCatalog &catalog() const { return catalog_; }
    bool any() const { return catalog_.slots[catalog_.selected].periodMs != 0; }

    template <typename Load, typename Action>
    bool access(Load load, Action action) {
        std::unique_ptr<SavedPatternLibrary> record(new (std::nothrow) SavedPatternLibrary);
        if (!record || !load(*record) || !action(*record)) return false;
        catalog_ = record->catalog();
        return true;
    }
};
static_assert(sizeof(SavedPatternCache) == sizeof(PatternCatalog));

}  // namespace streaming
