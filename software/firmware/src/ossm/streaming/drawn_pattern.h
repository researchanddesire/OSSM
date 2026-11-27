#pragma once

#include <cstddef>
#include <cstdint>
#include "drawn_sample_format.h"
#include "drawn_modifiers.h"

namespace motion_link { class Owner; }

namespace streaming {
struct PatternReferenceLost { uint64_t revision; };
bool patternOwnerCurrent(const PatternReferenceLost &event);
enum class PatternStart { Accepted, Invalid, NotReady, Busy };
PatternStart drawnPatternStartStatus(uint64_t revision);
PatternStart startDrawnPattern(const char *hex, size_t length, uint32_t periodMs, uint64_t ownerRevision,
                               bool localControl = false,
                               drawn_pattern::SampleFormat format = drawn_pattern::SampleFormat::Q8,
                               const drawn_pattern::Modifiers &modifiers = drawn_pattern::noModifiers());
bool renewDrawnPattern(uint64_t ownerRevision);
bool applyLeasedBleSpeed(motion_link::Owner &link, uint16_t connection,
                         float value, bool selectBleSource);
bool applyLeasedBleSensation(motion_link::Owner &link, uint16_t connection, uint8_t value);
void cancelDrawnPattern();
void cancelRemoteDrawnPattern();
bool drawnPatternActive();
// Accepted rate goals on the monotonic device clock. The eight newest
// records survive Stop. Sequence zero reads the newest record.
struct PatternRateStamp {
    uint32_t sequence, atMs, periodMs;
    float phase, rateHz, phaseAcceleration;
    uint16_t leadBoundMs;
    bool first, serial, leadKnown;
    float requestedStrength, gain, targetGain, carrier;
    uint32_t frameIndex;
};
bool drawnPatternRateStamp(uint32_t sequence, PatternRateStamp &out);
struct PatternStop {
    const char *reason = nullptr;
    const char *stage = nullptr;
    uint32_t atMs = 0, elapsedMs = 0;
    int8_t driverResult = 0;
    uint32_t frameGapMs = 0;
};
struct PatternStatus {
    bool active;
    uint32_t periodMs;
    PatternStop lastStop;
};
PatternStatus drawnPatternStatus();

// Called only by the guarded streaming task.
void setDrawnPatternReady(uint64_t revision, bool ready, const char *exitReason = "mode_exit");
bool serviceDrawnPattern(uint64_t revision);
}
