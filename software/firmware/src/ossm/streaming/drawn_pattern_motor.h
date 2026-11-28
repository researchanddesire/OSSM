#pragma once

#include <cstdint>
#include "drawn_pulse_queue.h"

namespace streaming { uint32_t patternServiceMicros(); }

namespace streaming::pattern_motor {
// A snapshot in native pulse-equivalent coordinates. The backend independently
// rechecks the owner under the same submission lock as a physical Stop.
struct State {
    bool ready = false;
    bool serial = false;
    bool moving = false;
    bool queued = false;
    int32_t position = 0;
    int32_t from = 0, to = 0;
    double speed = 0, acceleration = 0;
    double strength = 0, stepsPerMm = 0;
    double fullSpeed = 0, nominalSpeed = 0, fraction = 0;
    // Native input values for detecting wired control or calibration changes
    // between contiguous electrical writes under one motor command lock.
    float speedPercent = 0, sensationPercent = 0, strokePercent = 0, depthPercent = 0, rail = 0;
    bool sensationControlsMotion = false;
};
// The usual 50% acceleration setting is the fixed baseline for authored paths.
constexpr double baseAcceleration(double maximum) { return .5 * maximum; }
using drawn_pattern::Frame;
using drawn_pattern::FrameResult;
State read(uint64_t revision, bool annotated = false);
void readFiniteState(State &state, bool checkQueue);
void setSensation(uint8_t value);
uint32_t now();
bool current(uint64_t revision);
bool position(uint64_t revision, int32_t target, const State &limits, bool starting);
FrameResult submit(int32_t target, uint64_t revision, bool serial, uint16_t crossing, unsigned followingHolds);
#ifdef ARDUINO
void submitPrepared(drawn_pattern::PulseBatch &batch, drawn_pattern::PreparedPulses::Target first,
                    const drawn_pattern::PreparedPulses &next, uint64_t revision,
                    const State &caps, bool (*mayContinue)());
bool pulseClockRunning();
#else
inline void submitPrepared(drawn_pattern::PulseBatch &batch, drawn_pattern::PreparedPulses::Target first,
                           const drawn_pattern::PreparedPulses &next, uint64_t revision,
                           const State &, bool (*mayContinue)()) {
    if (!mayContinue()) {
        batch.controlsChanged = true;
        return;
    }
    batch.submit(first, next, [&](auto target) {
        return submit(target.position, revision, false, target.crossing, target.followingHolds);
    }, mayContinue, patternServiceMicros, now);
}
// The host backend sets this observation when replaying a peripheral stop.
inline bool observedPulseRunning = true;
inline bool pulseClockRunning() { return observedPulseRunning; }
#endif
bool pauseEntry(uint64_t revision);
using drawn_pattern::Park;
Park park(uint64_t revision, int32_t target, bool draining);
void stop(uint64_t revision);
}
