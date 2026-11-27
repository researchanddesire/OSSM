#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <new>
#include "drawn_waveform.h"
#include "stored_pattern_modifiers.h"

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
    bool selectedAvailable = false;
    PatternSlotInfo slots[patternSlotCount]{};
};

class SavedPatternLibrary {
    using Encoded = drawn_pattern::StoredModifiers;
    using Modifiers = drawn_pattern::Modifiers;
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
    struct SlotHeader {
        uint32_t magic;
        uint16_t count;
        uint16_t initial;
        Slot slot;
    };
    static_assert(sizeof(SlotHeader) == 304);
    static constexpr uint32_t slotMagic = 0x534c3400;
    static constexpr uint32_t selectionMagic = 0x53453400;
    static constexpr int allSlots = -2;
    static constexpr const char *key(unsigned i) {
        return i == 0 ? "slot4_0" : i == 1 ? "slot4_1" : i == 2 ? "slot4_2" : "slot4_3";
    }

    Record record_{};
    bool extended_ = false;
    bool selectionKnown_ = true;
    int initial_ = -1;
    uint8_t damaged_ = 0;
    uint16_t counts_[patternSlotCount]{};
    std::unique_ptr<uint8_t[]> blobs_[patternSlotCount];

    bool occupied(int index) const {
        return index >= 0 && index < patternSlotCount && record_.slots[index].value.length;
    }
    static bool validSlot(const Slot &slot) {
        return validPatternName(slot.name, strnlen(slot.name, sizeof(slot.name))) &&
               validSavedPattern(slot.value);
    }
    bool validRecord(uint32_t version) const {
        if (record_.version != version || !occupied(record_.selected)) return false;
        for (const auto &slot : record_.slots) {
            if (!slot.value.length) continue;
            if ((version == 2 && slot.value.version != 1) || !validSlot(slot)) return false;
        }
        return true;
    }
    const uint8_t *encoded(int slot) const {
        return blobs_[slot] ? blobs_[slot].get() + sizeof(SlotHeader) : nullptr;
    }

    template <typename Read>
    void loadLegacy(Read read) {
        if (read("libraryV3", &record_, sizeof(record_)) && validRecord(3)) return;
        if (read("libraryV2", &record_, sizeof(record_)) && validRecord(2)) {
            record_.version = 3;
            return;
        }
        record_.version = 3;
        record_.selected = 0;
        for (auto &slot : record_.slots) slot = Slot{};
        auto &legacy = record_.slots[0];
        if (read("curveV1", &legacy.value, sizeof(legacy.value)) && legacy.value.version == 1 &&
            validSavedPattern(legacy.value))
            std::strcpy(legacy.name, "Saved pattern");
        else
            legacy.value.length = 0;
    }

   public:
    // Boot and selection need only names/counts. A playback or edit retains
    // the requested slot's compact bytes; unrelated slots are verified one at a time.
    static constexpr int catalogOnly = -3;
    template <typename Read>
    bool load(Read read, int retain = allSlots) {
        extended_ = false;
        selectionKnown_ = true;
        initial_ = -1;
        damaged_ = 0;
        for (auto &blob : blobs_) blob.reset();
        std::memset(counts_, 0, sizeof(counts_));
        loadLegacy(read);
        const bool legacySelected = any();
        uint32_t choice = 0;
        const size_t selectionSize = read("selected4", nullptr, 0);
        if (selectionSize == sizeof(choice) && !read("selected4", &choice, sizeof(choice))) return false;
        const bool explicitSelection = selectionSize == sizeof(choice) && (choice & ~3u) == selectionMagic;
        extended_ = selectionSize != 0;
        bool ambiguousInitial = false;
        if (explicitSelection) record_.selected = choice & 3u;
        for (unsigned i = 0; i < patternSlotCount; ++i) {
            const size_t size = read(key(i), nullptr, 0);
            if (!size) continue;
            extended_ = true;
            // A present but corrupt override must never revive an old waveform
            // from the fallback snapshot. Independently addressed siblings survive.
            record_.slots[i].value.length = 0;
            damaged_ |= uint8_t(1u << i);
            if (size < sizeof(SlotHeader) ||
                size > sizeof(SlotHeader) + Encoded::bytes(Modifiers::capacity))
                continue;
            std::unique_ptr<uint8_t[]> data(new (std::nothrow) uint8_t[size]);
            if (!data || !read(key(i), data.get(), size)) return false;
            SlotHeader header;
            std::memcpy(&header, data.get(), sizeof(header));
            if (header.magic != (slotMagic | i) || header.count > Modifiers::capacity ||
                header.initial > 1 || size != sizeof(header) + Encoded::bytes(header.count) ||
                !validSlot(header.slot) || !Encoded::decode(data.get() + sizeof(header), header.count))
                continue;
            if (header.initial) {
                if (initial_ >= 0) ambiguousInitial = true;
                initial_ = i;
                if (!explicitSelection) {
                    if (retain == selectedPatternSlot && record_.selected != i)
                        blobs_[record_.selected].reset();
                    record_.selected = i;
                }
            }
            damaged_ &= uint8_t(~(1u << i));
            record_.slots[i] = header.slot;
            counts_[i] = header.count;
            if (header.count && (retain == allSlots || retain == int(i) ||
                                 (retain == selectedPatternSlot && record_.selected == i)))
                blobs_[i] = std::move(data);
        }
        // A dangling explicit selector means its selected slot is unavailable,
        // while a user can still explicitly select an intact slot.
        selectionKnown_ = explicitSelection || (!selectionSize && !ambiguousInitial &&
                                                (!extended_ || legacySelected || initial_ >= 0));
        return true;
    }

    bool any() const { return selectionKnown_ && occupied(record_.selected); }
    bool get(SavedPattern &out, int index = selectedPatternSlot, Modifiers *effects = nullptr) const {
        if (index == selectedPatternSlot) {
            if (!selectionKnown_) return false;
            index = record_.selected;
        }
        if (!occupied(index)) return false;
        if (effects && ((counts_[index] && !encoded(index)) ||
                        !Encoded::decode(encoded(index), counts_[index], effects)))
            return false;
        out = record_.slots[index].value;
        return true;
    }
    PatternCatalog catalog() const {
        PatternCatalog result{};
        result.selected = record_.selected;
        result.selectedAvailable = any();
        for (unsigned i = 0; i < patternSlotCount; ++i) {
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
        const bool namedRepair = index != selectedPatternSlot && name != nullptr;
        if (index == selectedPatternSlot) {
            if (!selectionKnown_) return false;
            index = record_.selected;
        }
        if (index < 0 || index >= patternSlotCount || ((damaged_ & (1u << index)) && !namedRepair) ||
            !validSavedPattern(value) || !effects.valid())
            return false;
        auto &slot = record_.slots[index];
        if (!name) {
            if (nameLength) return false;
            name = occupied(index) ? slot.name : "Saved pattern";
            nameLength = std::strlen(name);
        }
        if (!validPatternName(name, nameLength)) return false;
        if (effects.count == counts_[index] && (!effects.count || encoded(index)) &&
            Encoded::matches(encoded(index), effects) && slot.value.version == value.version &&
            slot.value.length == value.length && slot.value.periodMs == value.periodMs &&
            !std::memcmp(slot.value.samples, value.samples, value.length) &&
            std::strlen(slot.name) == nameLength && !std::memcmp(slot.name, name, nameLength))
            return true;

        Slot replacement{};
        replacement.value = value;
        std::memcpy(replacement.name, name, nameLength);
        if (!extended_ && !effects.count) {
            const Slot previous = slot;
            const auto selected = record_.selected;
            if (!any()) record_.selected = index;
            slot = replacement;
            if (write("libraryV3", &record_, sizeof(record_))) return true;
            slot = previous;
            record_.selected = selected;
            return false;
        }

        const bool first = !extended_ && !any();
        const size_t size = sizeof(SlotHeader) + Encoded::bytes(effects.count);
        std::unique_ptr<uint8_t[]> blob(new (std::nothrow) uint8_t[size]);
        if (!blob) return false;
        const SlotHeader header{slotMagic | uint32_t(index), effects.count,
                                uint16_t(first || initial_ == index), replacement};
        std::memcpy(blob.get(), &header, sizeof(header));
        Encoded::encode(blob.get() + sizeof(header), effects);
        if (!write(key(index), blob.get(), size)) return false;
        // First selection is part of the first slot itself: power loss leaves
        // either the entire prior state or the new slot with its true default.
        if (first) {
            initial_ = index;
            record_.selected = index;
        }
        damaged_ &= uint8_t(~(1u << index));
        extended_ = true;
        counts_[index] = effects.count;
        blobs_[index] = std::move(blob);
        slot = replacement;
        return true;
    }

    template <typename Write> bool select(int index, Write write) {
        if (!occupied(index)) return false;
        if (selectionKnown_ && record_.selected == uint32_t(index)) return true;
        if (extended_) {
            const uint32_t choice = selectionMagic | uint32_t(index);
            if (!write("selected4", &choice, sizeof(choice))) return false;
            record_.selected = index;
            selectionKnown_ = true;
            return true;
        }
        const auto previous = record_.selected;
        record_.selected = index;
        if (write("libraryV3", &record_, sizeof(record_))) return true;
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
    bool any() const { return catalog_.selectedAvailable; }

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
