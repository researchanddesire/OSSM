#pragma once

#include <cstdint>

// Start policy for a background worker whose task stack may not fit in
// internal RAM when it is first asked for. Pure logic, testable on the native
// platform; the firmware supplies the timer, the task creation and the logs.
//
// The OSSM boot initializer asks for the pairing-status worker while it still
// holds its own large stack and MQTT is connecting, so that creation can
// fail. Every attempt therefore runs from a one-shot timer: the first once
// the initializer has exited, the rest after a doubling, capped delay, with
// no attempt limit, until the worker starts or its answer is already known.
namespace worker_start_retry {

enum class Decision : uint8_t {
    Create,     // try to create the worker now
    Defer,      // a foreground TLS job is using the heap: wait, allocate nothing
    NotNeeded,  // the worker's answer is already known (paired another way)
};

inline Decision decide(bool answerKnown, bool foregroundTlsActive) {
    if (answerKnown) return Decision::NotNeeded;
    if (foregroundTlsActive) return Decision::Defer;
    return Decision::Create;
}

// Delay before the next round after `waits` rounds that did not start the
// worker (failed or deferred; 0 is treated as 1): initialMs, then doubling,
// never above maxMs and never overflowing. A zero initialMs counts as 1 ms so
// a misconfiguration cannot turn the retry into a busy loop.
inline uint32_t retryDelayMs(uint32_t waits, uint32_t initialMs, uint32_t maxMs) {
    if (initialMs == 0) initialMs = 1;
    const uint32_t doublings = waits > 1 ? waits - 1 : 0;
    if (initialMs >= maxMs || doublings >= 32 || initialMs > (maxMs >> doublings)) {
        return maxMs;
    }
    return initialMs << doublings;
}

}  // namespace worker_start_retry
