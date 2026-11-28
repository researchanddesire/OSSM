#pragma once

#include <ArduinoJson.h>
#include <memory>
#include <new>
#include "drawn_modifiers.h"

namespace streaming {

// The BLE lease and random upload key bind every packet to one transfer.
// Chunks carry whole records so retries can be verified without reappending.
class PatternModifierUpload {
    using Modifier = drawn_pattern::Modifier;
    using Modifiers = drawn_pattern::Modifiers;
    uint32_t lease_ = 0, key_ = 0;
    uint16_t connection_ = UINT16_MAX;
    std::unique_ptr<Modifiers> items_;

    static bool readWord(const char *hex, uint16_t &word) {
        word = 0;
        for (unsigned i = 0; i < 4; ++i) {
            const char c = hex[i];
            unsigned digit;
            if (c >= '0' && c <= '9') digit = c - '0';
            else if (c >= 'a' && c <= 'f') digit = c - 'a' + 10;
            else return false;
            word = (word << 4) | digit;
        }
        return true;
    }

    bool matches(uint16_t connection, uint32_t lease, uint32_t key) const {
        return items_ && connection_ == connection && lease_ == lease && key_ == key;
    }
public:
    // A zero count leaves the acknowledged prefix intact. Report a transient
    // allocator failure separately from malformed bytes or an incorrect owner.
    unsigned append(uint16_t connection, uint32_t lease, JsonObjectConst args,
                    bool *outOfMemory = nullptr) {
        if (outOfMemory) *outOfMemory = false;
        if (connection == UINT16_MAX || !lease || args.size() != 3 ||
            !args["key"].is<uint32_t>() || !args["offset"].is<uint16_t>()) return 0;
        const auto key = args["key"].as<uint32_t>();
        const auto offset = args["offset"].as<unsigned>();
        const auto data = args["data"].as<JsonString>();
        if (!key || data.isNull() || !data.size() || data.size() % 12 || data.size() > 32*12) return 0;
        const unsigned end = offset + data.size()/12;
        if (end > Modifiers::capacity) return 0;
        const bool existing = matches(connection, lease, key);
        if (!existing && offset) return 0;
        const unsigned have = existing ? items_->count : 0;
        if (offset > have || (offset < have && end > have)) return 0;

        Modifier chunk[32];
        for (unsigned index = offset; index < end; ++index) {
            const auto *hex = data.c_str() + (index - offset) * 12;
            uint16_t start, finish, maximumMs;
            if (!readWord(hex, start) || !readWord(hex + 4, finish) ||
                !readWord(hex + 8, maximumMs)) return 0;
            auto &marker = chunk[index - offset];
            marker = Modifier::fromV1(start, finish, maximumMs);
            if (!marker.valid()) return 0;
            if (index < have) {
                const auto &old = items_->entries[index];
                if (old.start != marker.start || old.end != marker.end || old.holdMs != marker.holdMs) return 0;
            } else if (index) {
                const auto &previous = index == offset ? items_->entries[index - 1] : chunk[index - offset - 1];
                if (!Modifiers::ordered(previous, marker)) return 0;
            }
        }
        if (offset < have) return have;
        const auto &first = offset ? items_->entries[0] : chunk[0];
        if (!Modifiers::validSeam(first, chunk[end - offset - 1], end)) return 0;

        std::unique_ptr<Modifiers> replacement;
        if (!existing) replacement.reset(new (std::nothrow) Modifiers);
        auto *target = existing ? items_.get() : replacement.get();
        if (!target || !target->reserve(end)) {
            if (outOfMemory) *outOfMemory = true;
            return 0;
        }
        for (unsigned i = offset; i < end; ++i) target->entries[i] = chunk[i - offset];
        target->count = end;
        if (replacement) items_ = std::move(replacement);
        key_ = key;
        lease_ = lease;
        connection_ = connection;
        return end;
    }

    std::unique_ptr<Modifiers> take(uint16_t connection, uint32_t lease, JsonObjectConst args) {
        if (!args["modifierKey"].is<uint32_t>() || !args["modifierCount"].is<uint16_t>() ||
            !matches(connection, lease, args["modifierKey"].as<uint32_t>()) ||
            items_->count != args["modifierCount"].as<unsigned>()) return {};
        return std::move(items_);
    }

    void clear() { items_.reset(); }
};

}  // namespace streaming
