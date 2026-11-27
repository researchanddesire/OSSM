#pragma once

#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <cstring>
#include <memory>
#include <new>

namespace drawn_pattern {

// Start and end are thousandths of the authored (unrotated) cycle.
struct Modifier {
    uint16_t start, end, holdMs;
    bool pause() const { return start == end; }
};
static_assert(sizeof(Modifier) == 6);

struct Modifiers {
    // Non-overlap permits exactly one start at each of the 1,000 authored ticks.
    static constexpr size_t capacity = 1000;
    Modifiers() = default;
    Modifiers(const Modifiers &) = delete;
    Modifiers &operator=(const Modifiers &) = delete;

    uint16_t count = 0;
private:
    Modifier small_[4]{};
    size_t allocated_ = 4;
    std::unique_ptr<Modifier[]> storage_;
public:
    Modifier *entries = small_;

    bool reserve(size_t required) {
        if (count > allocated_ || required > capacity) return false;
        if (required <= allocated_) return true;
        const size_t size = std::min(capacity, std::max(required, allocated_ * 2));
        std::unique_ptr<Modifier[]> replacement(new (std::nothrow) Modifier[size]);
        if (!replacement) return false;
        std::copy_n(entries, count, replacement.get());
        storage_ = std::move(replacement);
        entries = storage_.get();
        allocated_ = size;
        return true;
    }

    bool assign(const Modifiers &other) {
        if (!other.valid() || !reserve(other.count)) return false;
        if (&other != this) std::copy_n(other.entries, other.count, entries);
        count = other.count;
        return true;
    }

    static bool validItem(const Modifier &item) {
        if (item.start >= 1000 || item.end > 1000 || item.start > item.end) return false;
        return item.pause() ? item.holdMs > 0 && item.holdMs <= 10000 : item.holdMs == 0;
    }

    static bool ordered(const Modifier &left, const Modifier &right) {
        return left.end < right.start ||
               (left.end == right.start && !left.pause() && !right.pause());
    }

    static bool validSeam(const Modifier &first, const Modifier &last, size_t size) {
        return size < 2 || !first.pause() || first.start != 0 || last.end != 1000;
    }

    static bool valid(const Modifier *items, size_t size) {
        if (size > capacity || (size && !items)) return false;
        for (size_t i = 0; i < size; ++i) {
            if (!validItem(items[i]) || (i && !ordered(items[i-1],items[i]))) return false;
        }
        return !size || validSeam(items[0],items[size-1],size);
    }

    bool valid() const { return count <= allocated_ && valid(entries, count); }
};

inline const Modifiers &noModifiers() {
    static const Modifiers empty{};
    return empty;
}

}  // namespace drawn_pattern
