#ifndef OSSM_COMMUNICATION_QUEUE_H
#define OSSM_COMMUNICATION_QUEUE_H

#include <Arduino.h>
#include <chrono>

#include "utils/bounded_queue.h"

struct PositionTime {
    uint8_t position; // 0 - 100
    uint16_t inTime;     // in ms
    std::chrono::steady_clock::time_point setTime; //received timestamp
    int direction; //0:uncalculated, 1:out, -1:in
};

// Legacy BLE text commands (go:/set:/stream:), drained by the NimBLE loop.
// A flood beyond this is dropped rather than queued without limit.
using MessageQueue = BoundedQueue<String, 64>;
// Streaming targets, consumed by the streaming task. A full queue drops the
// oldest target: streaming wants the latest position.
using TargetQueue = BoundedQueue<PositionTime, 64>;

extern MessageQueue messageQueue;
extern TargetQueue targetQueue;

#endif  // OSSM_COMMUNICATION_QUEUE_H
