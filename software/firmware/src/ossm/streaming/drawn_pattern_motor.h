#pragma once

#include <cstdint>

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
};
enum class Frame { Accepted, Retry, Rejected };
State read(uint64_t revision);
uint32_t now();
bool current(uint64_t revision);
bool position(uint64_t revision, int32_t target, const State &limits, bool starting);
Frame submit(int32_t target, uint64_t revision, bool serial);
void stop(uint64_t revision);
}
