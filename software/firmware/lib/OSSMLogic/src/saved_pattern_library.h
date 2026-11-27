#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <new>
#include "drawn_waveform.h"
#include "drawn_modifiers.h"

namespace streaming {

constexpr uint8_t patternSlotCount = 4;
constexpr int selectedPatternSlot = -1;
constexpr size_t maxPatternNameBytes = 24;

// Preserve the 268-byte curveV1 layout. The format word distinguishes Q8, Q16 and sparse Q16.
struct SavedPattern {
    uint32_t version = 1;
    uint32_t periodMs = 0;
    uint32_t length = 0;
    char samples[drawn_pattern::maxSamples * 2] = {};
    drawn_pattern::SampleFormat format() const { return static_cast<drawn_pattern::SampleFormat>(version); }
};
static_assert(sizeof(SavedPattern) == 268);

inline bool validSavedPattern(const SavedPattern &value) {
    drawn_pattern::Waveform curve;
    return (value.version >= 1 && value.version <= 3) && value.length <= sizeof(value.samples) &&
           curve.decode(value.samples, value.length, value.periodMs, value.format());
}

inline bool makeSavedPattern(SavedPattern &out, const char *samples, size_t length,
                             uint32_t periodMs, drawn_pattern::SampleFormat format = drawn_pattern::SampleFormat::Q8) {
    if (!samples || length > sizeof(out.samples)) return false;
    out.version = static_cast<uint32_t>(format);
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
    uint16_t modifierCount = 0;
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
    using Modifier = drawn_pattern::Modifier;
    using Modifiers = drawn_pattern::Modifiers;
    uint16_t counts_[patternSlotCount]{};
    std::unique_ptr<Modifier[]> modifiers_[patternSlotCount];
    static constexpr size_t headerSize = sizeof(Record) + sizeof(counts_);
    static constexpr size_t maxSize = headerSize + patternSlotCount * Modifiers::capacity * sizeof(Modifier);

    size_t packedSize() const {
        size_t size = headerSize;
        for (const auto count : counts_) size += count * sizeof(Modifier);
        return size;
    }

    template <typename Write> bool persist(Write write) const {
        if (record_.version == 3) return write("libraryV3", &record_, sizeof(record_));
        const size_t size = packedSize();
        std::unique_ptr<uint8_t[]> bytes(new (std::nothrow) uint8_t[size]);
        if (!bytes) return false;
        std::memcpy(bytes.get(), &record_, sizeof(record_));
        std::memcpy(bytes.get() + sizeof(record_), counts_, sizeof(counts_));
        size_t offset = headerSize;
        for (unsigned i = 0; i < patternSlotCount; ++i) {
            const size_t length = counts_[i] * sizeof(Modifier);
            if (length) std::memcpy(bytes.get() + offset, modifiers_[i].get(), length);
            offset += length;
        }
        return write("libraryV4", bytes.get(), size);
    }

    template <typename Read> bool loadExtended(Read read, size_t size) {
        if (size < headerSize || size > maxSize) return false;
        std::unique_ptr<uint8_t[]> bytes(new (std::nothrow) uint8_t[size]);
        if (!bytes || !read("libraryV4", bytes.get(), size)) return false;
        std::memcpy(&record_, bytes.get(), sizeof(record_));
        std::memcpy(counts_, bytes.get() + sizeof(record_), sizeof(counts_));
        if (!validRecord(4) || packedSize() != size) return false;
        size_t offset = headerSize;
        for (unsigned i = 0; i < patternSlotCount; ++i) {
            const auto count = counts_[i];
            if (!count) continue;
            if (!occupied(i) || count > Modifiers::capacity) return false;
            modifiers_[i].reset(new (std::nothrow) Modifier[count]);
            if (!modifiers_[i]) return false;
            const size_t length = count * sizeof(Modifier);
            std::memcpy(modifiers_[i].get(), bytes.get() + offset, length);
            if (!Modifiers::valid(modifiers_[i].get(), count)) return false;
            offset += length;
        }
        return true;
    }

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
    // A null destination queries the blob length. Actual reads must match exactly.
    // If an extended record exists, an invalid or unreadable blob blocks edits to it.
    template <typename Read> bool load(Read read) {
        std::memset(counts_, 0, sizeof(counts_));
        for (auto &items : modifiers_) items.reset();
        const size_t extended = read("libraryV4", nullptr, 0);
        if (extended) return loadExtended(read, extended);
        if (read("libraryV3", &record_, sizeof(record_)) && validRecord(3)) return true;
        if (read("libraryV2", &record_, sizeof(record_)) && validRecord(2)) {
            record_.version = 3; // Persist only after a successful, deliberate change.
            return true;
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
        return true;
    }
    bool any() const { return occupied(record_.selected); }
    bool get(SavedPattern &out, int index = selectedPatternSlot, Modifiers *effects = nullptr) const {
        if (index == selectedPatternSlot) index = record_.selected;
        if (!occupied(index)) return false;
        out = record_.slots[index].value;
        if (effects) {
            effects->count = counts_[index];
            if (effects->count) std::memcpy(effects->entries, modifiers_[index].get(), effects->count * sizeof(Modifier));
        }
        return true;
    }
    PatternCatalog catalog() const {
        PatternCatalog result{};
        result.selected = record_.selected;
        for (uint8_t i = 0; i < patternSlotCount; ++i) {
            if (!occupied(i)) continue;
            result.slots[i].periodMs = record_.slots[i].value.periodMs;
            result.slots[i].modifierCount = counts_[i];
            std::memcpy(result.slots[i].name, record_.slots[i].name, sizeof(result.slots[i].name));
        }
        return result;
    }
    template <typename Write>
    bool save(int index, const char *name, size_t nameLength, const SavedPattern &value, Write write,
              const Modifiers &effects = drawn_pattern::noModifiers()) {
        if (index == selectedPatternSlot) index = record_.selected;
        if (index < 0 || index >= patternSlotCount || !validSavedPattern(value) || !effects.valid()) return false;
        auto &slot = record_.slots[index];
        if (!name) {
            if (nameLength) return false;
            name = occupied(index) ? slot.name : "Saved pattern";
            nameLength = std::strlen(name);
        }
        if (!validPatternName(name, nameLength)) return false;
        const size_t effectBytes = effects.count * sizeof(Modifier);
        const bool sameEffects = counts_[index] == effects.count &&
            (!effectBytes || std::memcmp(modifiers_[index].get(), effects.entries, effectBytes) == 0);
        if (sameEffects && slot.value.version == value.version && slot.value.length == value.length &&
            slot.value.periodMs == value.periodMs &&
            std::memcmp(slot.value.samples, value.samples, value.length) == 0 &&
            std::strlen(slot.name) == nameLength && std::memcmp(slot.name, name, nameLength) == 0)
            return true;
        std::unique_ptr<Modifier[]> replacement;
        if (effects.count && !sameEffects) {
            replacement.reset(new (std::nothrow) Modifier[effects.count]);
            if (!replacement) return false;
            std::memcpy(replacement.get(), effects.entries, effectBytes);
        }
        const auto oldVersion = record_.version;
        const auto oldCount = counts_[index];
        if (!sameEffects) {
            modifiers_[index].swap(replacement);
            counts_[index] = effects.count;
        }
        if (effects.count) record_.version = 4;
        const auto previous = slot;
        const auto previousSelected = record_.selected;
        if (!any()) record_.selected = index;
        slot.value = value;
        if (name != slot.name) {
            std::memset(slot.name, 0, sizeof(slot.name));
            std::memcpy(slot.name, name, nameLength);
        }
        if (persist(write)) return true;
        if (!sameEffects) modifiers_[index].swap(replacement);
        counts_[index] = oldCount;
        record_.version = oldVersion;
        slot = previous;
        record_.selected = previousSelected;
        return false;
    }
    template <typename Write> bool select(int index, Write write) {
        if (!occupied(index)) return false;
        if (record_.selected == static_cast<uint32_t>(index)) return true;
        const auto previous = record_.selected;
        record_.selected = index;
        if (persist(write)) return true;
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
