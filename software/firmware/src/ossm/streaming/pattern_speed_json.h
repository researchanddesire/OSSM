#pragma once

#include <ArduinoJson.h>
#include <cmath>
#include "drawn_pattern.h"
#include "pattern_catalog_json.h"

namespace streaming {

inline double reportedPeak(double mmPerSecond) {
    return std::round(mmPerSecond * 10.) / 10.;
}

inline void writePatternSpeed(JsonDocument &result, const PatternSpeedStatus &speed) {
    result["version"] = 2;
    result["state"] = speed.state;
    result["speedPct"] = speed.speedPct;
    result["nominalMmS"] = reportedPeak(speed.nominalMmS);
    result["plannedPeakMmS"] = reportedPeak(speed.plannedPeakMmS);
    result["phaseRateHz"] = speed.phaseRateHz;
    if (speed.periodMs) result["periodMs"] = speed.periodMs;
    else result["periodMs"] = nullptr;
    result["limit"] = speed.limit;
}

// The heartbeat already carries periodMs. A positional tuple keeps both
// peak speeds in the same correlated GATT response as the ownership renewal.
inline void addPatternSpeed(JsonDocument &result, const PatternStatus &playback) {
    result["speedLaw"] = 2;
    if (!playback.active) return;
    const auto &speed = playback.speed;
    auto plan = result["speedPlan"].to<JsonArray>();
    plan.add(speed.speedPct);
    plan.add(reportedPeak(speed.nominalMmS));
    plan.add(reportedPeak(speed.plannedPeakMmS));
    plan.add(speed.state);
    plan.add(speed.limit);
}

inline void writePatternStatus(JsonDocument &status, const PatternStatus &playback,
                               bool hasSpeedKnob, bool stored) {
    status["version"] = 1;
    status["liveSpeed"] = true;
    status["interpolationVersion"] = 2;
    status["maxSampleBits"] = 16;
    status["maxSparseSamples"] = drawn_pattern::sparseSamples;
    status["libraryVersion"] = 1;
    status["libraryPageSize"] = 1;
    status["hasSpeedKnob"] = hasSpeedKnob;
    status["active"] = playback.active;
    status["periodMs"] = playback.periodMs;
    if (playback.lastStop.reason) {
        auto stop = status["lastStop"].to<JsonObject>();
        stop["reason"] = playback.lastStop.reason;
        stop["stage"] = playback.lastStop.stage;
        stop["atMs"] = playback.lastStop.atMs;
        stop["elapsedMs"] = playback.lastStop.elapsedMs;
        stop["driverResult"] = playback.lastStop.driverResult;
        stop["frameGapMs"] = playback.lastStop.frameGapMs;
    }
    status["stored"] = stored;
    addPatternSpeed(status, playback);
    addModifierCapabilities(status);
}
}
