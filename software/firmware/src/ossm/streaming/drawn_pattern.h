#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
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
PatternStart startOwnedDrawnPattern(const char *hex, size_t length, uint32_t periodMs, uint64_t ownerRevision,
                                    bool localControl, drawn_pattern::SampleFormat format,
                                    std::unique_ptr<drawn_pattern::Modifiers> modifiers);
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
    double goalRateHz;
    bool rawHandoff;
};
bool drawnPatternRateStamp(uint32_t sequence, PatternRateStamp &out);
struct PatternStop {
    const char *reason = nullptr;
    const char *stage = nullptr;
    uint32_t atMs = 0, elapsedMs = 0;
    int8_t driverResult = 0;
    uint32_t frameGapMs = 0;
};
struct PatternServiceTiming {
    uint32_t atMs = 0, frame = 0, offUs = 0, outerUs = 0, mailboxUs = 0;
    uint32_t readUs = 0, commandGateUs = 0, serviceUs = 0;
    uint64_t intervalUs() const { return uint64_t(offUs) + outerUs + mailboxUs + serviceUs; }
};
struct PatternWork {
    PatternServiceTiming lastService, longestService;
    uint32_t frames = 0, retries = 0;
    uint32_t plans = 0, movingPlans = 0, planOver2ms = 0;
    uint32_t submissions = 0, submitOver2ms = 0, rawStartSequence = 0;
    uint64_t planTotalUs = 0, submitTotalUs = 0;
    uint32_t maxPlanUs = 0, maxSubmitUs = 0, maxRenewLockUs = 0;
    uint64_t movingPlanTotalUs = 0;
    uint32_t maxMovingPlanUs = 0, immediateTurns = 0, lowWaterYields = 0;
    uint32_t prepared = 0, discarded = 0, emptyTurns = 0, maxPrepared = 0, plannerStackFree = 0;
    uint16_t lastQueuedUs = 0, minRunningUs = UINT16_MAX, maxQueuedUs = 0;
};
bool stoppedPatternWork(PatternWork &out);
// Events after the last accepted raw pulse group, available only after Stop.
// Offsets are microseconds from the guarded append returning to the feeder.
struct PatternFeedGap {
    uint32_t frame = 0, elapsedUs = 0, serviceTurns = 0;
    uint32_t emptyReads = 0, firstEmptyUs = 0, lastEmptyUs = 0;
    uint32_t fullRetries = 0, lastFullUs = 0, otherRetries = 0;
    // Lifetime totals for work shared by the two cores during this Play.
    uint32_t assistAttempts = 0, assistPublished = 0, assistMaxUs = 0;
    uint16_t queuedUs = 0, previousQueuedUs = 0;
    uint32_t previousAppendIntervalUs = 0, lastOtherOffsetUs = 0, combinedHoldFrames = 0;
    int8_t lastOtherDriverCode = 0;
};
bool stoppedPatternFeedGap(PatternFeedGap &out);
struct PatternSpeedStatus {
    const char *state = "idle", *limit = "flat";
    double speedPct = 0, nominalMmS = 0, plannedPeakMmS = 0, phaseRateHz = 0;
    uint32_t periodMs = 0;
};
struct PatternStatus {
    bool active;
    uint32_t periodMs;
    PatternStop lastStop;
    PatternSpeedStatus speed;
};
PatternStatus drawnPatternStatus();

// Called only by the guarded streaming task.
void setDrawnPatternReady(uint64_t revision, bool ready, const char *exitReason = "mode_exit");
uint32_t patternServiceMicros();
bool serviceDrawnPattern(uint64_t revision, uint32_t wakeUs = 0, bool *refillNow = nullptr);
}
