#pragma once

#include <cstddef>
#include <cstdint>

namespace streaming {
struct PatternReferenceLost { uint64_t revision; };
bool patternOwnerCurrent(const PatternReferenceLost &event);
enum class PatternStart { Accepted, Invalid, NotReady, Busy };
PatternStart drawnPatternStartStatus(uint64_t revision);
PatternStart startDrawnPattern(const char *hex, size_t length, uint32_t periodMs, uint64_t ownerRevision,
                               bool localControl = false);
bool renewDrawnPattern(uint64_t ownerRevision);
void cancelDrawnPattern();
void cancelRemoteDrawnPattern();
bool drawnPatternActive();
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
