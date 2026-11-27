#pragma once

#include <ArduinoJson.h>
#include "saved_pattern_library.h"

namespace streaming {

// The largest named OSSM state is 27 bytes. The correlated envelope with a
// ten-digit request ID and both state names uses 162 of the 509 wire bytes.
inline void addModifierCapabilities(JsonDocument &result) {
    result["modifierVersion"] = 1;
    result["maxModifiers"] = drawn_pattern::Modifiers::capacity;
    result["maxPauseMs"] = drawn_pattern::maxPauseMs;
    if (result["lastStop"].is<JsonObjectConst>() && measureJson(result) > 347) {
        result.remove("modifierVersion");
        result.remove("maxModifiers");
        result.remove("maxPauseMs");
    }
}

// Use exclusive fields so firmware lacking a format can never play its bytes at
// a different resolution. An explicit null still counts as a supplied field.
inline bool readPatternSamples(JsonObjectConst args, SavedPattern &out) {
    const bool q8 = !args["samples"].isUnbound();
    const bool q16 = !args["samples16"].isUnbound();
    const bool runs = !args["samples16Runs"].isUnbound();
    if (int(q8) + int(q16) + int(runs) != 1 || !args["periodMs"].is<uint32_t>()) return false;
    const auto data = args[runs ? "samples16Runs" : q16 ? "samples16" : "samples"].as<JsonString>();
    using drawn_pattern::SampleFormat;
    const auto format = runs ? SampleFormat::RunsQ16 : q16 ? SampleFormat::Q16 : SampleFormat::Q8;
    return !data.isNull() && makeSavedPattern(out, data.c_str(), data.size(), args["periodMs"], format);
}

// No slot means the original whole-library representation. A requested empty
// slot still reports selection and capacity with an empty entries array.
inline bool writePatternCatalog(JsonDocument &result, JsonObjectConst request,
                                const PatternCatalog &catalog) {
    const auto args = request["args"];
    if (!args.isUnbound() && !args.is<JsonObjectConst>()) return false;
    const auto slotArg = args["slot"];
    const bool paged = !slotArg.isUnbound();
    if (args.size() != (paged ? 1u : 0u) ||
        (paged && (!slotArg.is<uint8_t>() || slotArg.as<unsigned>() >= patternSlotCount)))
        return false;
    result["version"] = 1;
    result["capacity"] = patternSlotCount;
    result["selected"] = catalog.selected;
    auto slots = result["slots"].to<JsonArray>();
    const unsigned first = paged ? slotArg.as<unsigned>() : 0;
    const unsigned end = paged ? first + 1 : patternSlotCount;
    for (unsigned i = first; i < end; ++i) {
        if (!catalog.slots[i].periodMs) continue;
        auto slot = slots.add<JsonObject>();
        slot["slot"] = i;
        slot["name"] = catalog.slots[i].name;
        slot["periodMs"] = catalog.slots[i].periodMs;
        if (catalog.slots[i].modifierCount) slot["modifierCount"] = catalog.slots[i].modifierCount;
    }
    return true;
}

}  // namespace streaming
