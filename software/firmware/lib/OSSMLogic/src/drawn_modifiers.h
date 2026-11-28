#pragma once

#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <cstring>
#include <memory>
#include <new>
#include <utility>

namespace drawn_pattern {

constexpr uint16_t maxPauseMs = 60000;
constexpr uint16_t originalMaxPauseMs = 10000;

// The two families have independent, stable effect IDs. V1's transport
// represents only Vibration and Pause and infers these IDs from its geometry.
enum class EffectGeometry : uint8_t { TimeRange, Instant };
struct TimeRangeEffect {
    enum class Kind : uint16_t { Vibration = 1 };
    Kind kind;
    uint16_t start, end;
    bool valid() const {
        return kind == Kind::Vibration && start < end && end <= 1000;
    }
};
struct InstantEffect {
    enum class Kind : uint16_t { Pause = 1 };
    Kind kind;
    uint16_t at, maximumMs;
    bool valid() const {
        return kind == Kind::Pause && at < 1000 && maximumMs && maximumMs <= maxPauseMs;
    }
};

// Keep a thousand tagged records within the original six kilobytes. Bit fields
// are private to the in-memory representation; flash and wire use explicit words.
struct Modifier {
    uint16_t start : 10;
    uint16_t kindLow : 6;
    uint16_t end : 10;
    uint16_t kindHigh : 5;
    uint16_t instant : 1;
    uint16_t holdMs;

private:
    // Preserve 2,047 independently identifiable effects per geometry in two
    // phase words. Reject out-of-range raw values before narrowing any field.
    static uint16_t checkedId(uint16_t start, uint16_t end, uint16_t id) {
        return start < 1000 && end <= 1000 && id <= 2047 ? id : 0;
    }
    Modifier(uint16_t start, uint16_t end, uint16_t id, bool point, uint16_t ms)
        : start(start), kindLow(id & 63), end(end), kindHigh(id >> 6), instant(point), holdMs(ms) {}

public:
    Modifier() : Modifier(0, 0, 0, false, 0) {}
    explicit Modifier(TimeRangeEffect range)
        : Modifier(range.start, range.end, checkedId(range.start, range.end, uint16_t(range.kind)), false, 0) {}
    explicit Modifier(InstantEffect point)
        : Modifier(point.at, point.at, checkedId(point.at, point.at, uint16_t(point.kind)), true, point.maximumMs) {}

    // Decode the original untagged three-word record. Reject malformed range
    // durations before converting them to the tagged range representation.
    static Modifier fromV1(uint16_t start, uint16_t end, uint16_t ms) {
        if (start == end) return Modifier(InstantEffect{InstantEffect::Kind::Pause, start, ms});
        if (ms) return {};
        return Modifier(TimeRangeEffect{TimeRangeEffect::Kind::Vibration, start, end});
    }
    Modifier(uint16_t start, uint16_t end, uint16_t ms) : Modifier(fromV1(start, end, ms)) {}
    EffectGeometry geometry() const { return instant ? EffectGeometry::Instant : EffectGeometry::TimeRange; }
    uint16_t kindId() const { return (kindHigh << 6) | kindLow; }
    InstantEffect point() const { return {static_cast<InstantEffect::Kind>(kindId()), start, holdMs}; }
    TimeRangeEffect range() const { return {static_cast<TimeRangeEffect::Kind>(kindId()), start, end}; }
    bool isPause() const { return instant && point().kind == InstantEffect::Kind::Pause; }
    bool isVibration() const { return !instant && range().kind == TimeRangeEffect::Kind::Vibration; }
    bool valid() const {
        if (instant) return end == start && point().valid();
        return !holdMs && range().valid();
    }
};
static_assert(sizeof(Modifier) == 6);

struct Modifiers {
    // Non-overlap permits exactly one start at each of the 1,000 authored ticks.
    static constexpr size_t capacity = 1000;
    Modifiers() = default;
    Modifiers(const Modifiers &) = delete;
    Modifiers &operator=(const Modifiers &) = delete;

    class Entries {
        static constexpr size_t inlineCount = 4;
        static constexpr size_t pageSize = 64;
        static constexpr size_t pageCount = (capacity - inlineCount + pageSize - 1) / pageSize;
        struct Page { Modifier values[pageSize]; };
        struct Directory { std::unique_ptr<Page> pages[pageCount]; };

        Modifier small_[inlineCount]{};
        std::unique_ptr<Directory> directory_;
        size_t pages_ = 0;

    public:
        size_t allocated() const { return inlineCount + pages_ * pageSize; }

        // Pages stay at fixed addresses throughout upload and playback.
        // Growing a valid prefix never requires a second copy of that prefix.
        bool reserve(size_t required) {
            if (required > capacity) return false;
            if (required <= allocated()) return true;
            if (!directory_) {
                directory_.reset(new (std::nothrow) Directory);
                if (!directory_) return false;
            }
            while (required > allocated()) {
                auto &page = directory_->pages[pages_];
                page.reset(new (std::nothrow) Page);
                if (!page) return false;
                ++pages_;
            }
            return true;
        }

        const Modifier &operator[](size_t index) const {
            if (index < inlineCount) return small_[index];
            const size_t i = index - inlineCount;
            return directory_->pages[i / pageSize]->values[i % pageSize];
        }

        Modifier &operator[](size_t index) {
            return const_cast<Modifier &>(static_cast<const Entries &>(*this)[index]);
        }

        void takeFrom(Entries &other) {
            std::copy_n(other.small_, inlineCount, small_);
            directory_ = std::move(other.directory_);
            pages_ = std::exchange(other.pages_, 0);
        }
    };

    uint16_t count = 0;
    Entries entries;

    bool reserve(size_t required) {
        return count <= entries.allocated() && entries.reserve(required);
    }

    bool assign(const Modifiers &other) {
        if (!other.valid() || !reserve(other.count)) return false;
        if (&other != this) {
            for (unsigned i = 0; i < other.count; ++i) entries[i] = other.entries[i];
            count = other.count;
        }
        return true;
    }

    void takeFrom(Modifiers &other) {
        if (this == &other) return;
        entries.takeFrom(other.entries);
        count = std::exchange(other.count, 0);
    }

    static bool validItem(const Modifier &item) {
        return item.valid();
    }

    static bool ordered(const Modifier &left, const Modifier &right) {
        return left.end < right.start ||
               (left.end == right.start && left.geometry() == EffectGeometry::TimeRange &&
                right.geometry() == EffectGeometry::TimeRange);
    }

    static bool validSeam(const Modifier &first, const Modifier &last, size_t size) {
        return size < 2 || first.geometry() != EffectGeometry::Instant || first.start != 0 || last.end != 1000;
    }

    static bool valid(const Entries &items, size_t size) {
        if (size > capacity || size > items.allocated()) return false;
        for (size_t i = 0; i < size; ++i) {
            if (!validItem(items[i]) || (i && !ordered(items[i-1],items[i]))) return false;
        }
        return !size || validSeam(items[0],items[size-1],size);
    }

    bool valid() const { return valid(entries, count); }
};

inline const Modifiers &noModifiers() {
    static const Modifiers empty{};
    return empty;
}

}  // namespace drawn_pattern
