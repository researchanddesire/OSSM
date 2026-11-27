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
    // Returns the acknowledged prefix length or zero for an invalid chunk.
    unsigned append(uint16_t connection, uint32_t lease, JsonObjectConst args) {
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

        std::unique_ptr<Modifiers> replacement;
        if (!existing) {
            replacement.reset(new (std::nothrow) Modifiers);
            if (!replacement) return 0;
        }
        auto &target = existing ? *items_ : *replacement;
        for (unsigned index = offset; index < end; ++index) {
            const auto *hex = data.c_str() + (index - offset)*12;
            Modifier marker;
            if (!readWord(hex, marker.start) || !readWord(hex+4, marker.end) ||
                !readWord(hex+8, marker.holdMs)) return 0;
            if (index < have) {
                const auto &old = target.entries[index];
                if (old.start != marker.start || old.end != marker.end || old.holdMs != marker.holdMs) return 0;
            } else target.entries[index] = marker;
        }
        if (offset < have) return have;
        if (!Modifiers::valid(target.entries, end)) return 0;
        target.count = end;
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
