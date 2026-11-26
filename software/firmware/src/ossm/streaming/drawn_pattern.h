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
uint32_t drawnPatternPeriodMs();

// Called only by the guarded streaming task.
void setDrawnPatternReady(uint64_t revision, bool ready);
bool serviceDrawnPattern(uint64_t revision);
}
