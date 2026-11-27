#pragma once

#include <cstdint>

// Start policy for a background worker whose task stack may not fit in
// internal RAM when it is first asked for. Pure logic, testable on the native
// platform; the firmware supplies the timer, the task creation and the logs.
//
// The OSSM pairing-status check needs a 15 KB task stack for each round. At
// boot the initializer asks for it while still holding its own large stack
// and MQTT is connecting, and later rounds compete with MQTT and BLE, so a
// creation can fail. Every attempt therefore runs from a one-shot timer, and
// a failed or deferred one retries after a doubling, capped delay, with no
// attempt limit, until the round starts or its answer is already known.
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

// Attempts and back-off for one start. Not thread-safe: the caller keeps a
// single owner (an armed timer or the running worker, never both).
class Backoff {
   public:
    constexpr Backoff(uint32_t initialMs, uint32_t maxMs) noexcept
        : initialMs_(initialMs), maxMs_(maxMs) {}

    // Number (from 1) of the creation attempt about to be made.
    uint32_t beginAttempt() { return ++attempts_; }
    // After a failed or deferred round: the delay before the next one.
    uint32_t nextDelayMs() { return retryDelayMs(++waits_, initialMs_, maxMs_); }
    // The worker started: the next start begins again at attempt 1.
    void started() { attempts_ = 0; waits_ = 0; }

    uint32_t attempts() const { return attempts_; }

   private:
    uint32_t initialMs_;
    uint32_t maxMs_;
    uint32_t attempts_ = 0;
    uint32_t waits_ = 0;
};

}  // namespace worker_start_retry
