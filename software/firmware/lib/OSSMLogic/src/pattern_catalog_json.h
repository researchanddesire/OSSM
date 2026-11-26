#pragma once

#include <ArduinoJson.h>
#include "saved_pattern_library.h"

namespace streaming {

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
    }
    return true;
}

}  // namespace streaming
