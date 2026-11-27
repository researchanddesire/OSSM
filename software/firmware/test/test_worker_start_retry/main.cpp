#include <unity.h>

#include <cstdint>
#include <vector>

#include "worker_start_retry.h"

using worker_start_retry::Decision;
using worker_start_retry::decide;
using worker_start_retry::retryDelayMs;

void setUp() {}
void tearDown() {}

namespace {
// The firmware's pairing-status schedule.
constexpr uint32_t firstDelayMs = 1000;
constexpr uint32_t initialRetryMs = 2000;
constexpr uint32_t maxRetryMs = 60000;

struct Round {
    bool paired;
    bool foregroundTls;
    bool stackFits;
};

struct Outcome {
    bool started = false;
    bool stopped = false;
    uint32_t attempts = 0;
    uint32_t startedAtMs = 0;
    std::vector<uint32_t> delays;
};

// Drives the same decisions as the firmware's timer callback: each round is
// one timer expiry, and a round that does not start the worker re-arms it.
Outcome run(const std::vector<Round> &rounds) {
    Outcome outcome;
    uint32_t now = firstDelayMs;
    uint32_t waits = 0;
    for (const Round &round : rounds) {
        const Decision decision = decide(round.paired, round.foregroundTls);
        if (decision == Decision::NotNeeded) {
            outcome.stopped = true;
            return outcome;
        }
        if (decision == Decision::Create) {
            outcome.attempts++;
            if (round.stackFits) {
                outcome.started = true;
                outcome.startedAtMs = now;
                return outcome;
            }
        }
        const uint32_t delay = retryDelayMs(++waits, initialRetryMs, maxRetryMs);
        outcome.delays.push_back(delay);
        now += delay;
    }
    return outcome;
}
}  // namespace

void test_unpaired_worker_is_created_when_no_foreground_tls_job_runs() {
    TEST_ASSERT_TRUE(decide(false, false) == Decision::Create);
}

void test_foreground_tls_job_defers_the_allocation() {
    // The pairing page and updates pause MQTT and need the heap themselves.
    TEST_ASSERT_TRUE(decide(false, true) == Decision::Defer);
}

void test_paired_device_needs_no_worker() {
    TEST_ASSERT_TRUE(decide(true, false) == Decision::NotNeeded);
    TEST_ASSERT_TRUE(decide(true, true) == Decision::NotNeeded);
}

void test_retry_delay_doubles_from_the_initial_delay() {
    TEST_ASSERT_EQUAL_UINT32(2000, retryDelayMs(1, initialRetryMs, maxRetryMs));
    TEST_ASSERT_EQUAL_UINT32(4000, retryDelayMs(2, initialRetryMs, maxRetryMs));
    TEST_ASSERT_EQUAL_UINT32(8000, retryDelayMs(3, initialRetryMs, maxRetryMs));
    TEST_ASSERT_EQUAL_UINT32(16000, retryDelayMs(4, initialRetryMs, maxRetryMs));
    TEST_ASSERT_EQUAL_UINT32(32000, retryDelayMs(5, initialRetryMs, maxRetryMs));
}

void test_retry_delay_is_capped_and_never_overflows() {
    TEST_ASSERT_EQUAL_UINT32(60000, retryDelayMs(6, initialRetryMs, maxRetryMs));
    TEST_ASSERT_EQUAL_UINT32(60000, retryDelayMs(33, initialRetryMs, maxRetryMs));
    TEST_ASSERT_EQUAL_UINT32(60000, retryDelayMs(1000, initialRetryMs, maxRetryMs));
    TEST_ASSERT_EQUAL_UINT32(60000, retryDelayMs(UINT32_MAX, initialRetryMs, maxRetryMs));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, retryDelayMs(40, 1, UINT32_MAX));
}

void test_retry_delay_edge_inputs() {
    TEST_ASSERT_EQUAL_UINT32(2000, retryDelayMs(0, initialRetryMs, maxRetryMs));
    TEST_ASSERT_EQUAL_UINT32(60000, retryDelayMs(1, 90000, maxRetryMs));
    TEST_ASSERT_EQUAL_UINT32(1, retryDelayMs(1, 0, maxRetryMs));
    TEST_ASSERT_EQUAL_UINT32(8, retryDelayMs(4, 0, maxRetryMs));
}

void test_first_attempt_that_fits_starts_the_worker() {
    const Outcome outcome = run({{false, false, true}});
    TEST_ASSERT_TRUE(outcome.started);
    TEST_ASSERT_EQUAL_UINT32(1, outcome.attempts);
    TEST_ASSERT_EQUAL_UINT32(firstDelayMs, outcome.startedAtMs);
    TEST_ASSERT_TRUE(outcome.delays.empty());
}

void test_transient_allocation_failure_recovers() {
    // The boot-time failure from the issue: two rounds without room, then it fits.
    const Outcome outcome =
        run({{false, false, false}, {false, false, false}, {false, false, true}});
    TEST_ASSERT_TRUE(outcome.started);
    TEST_ASSERT_EQUAL_UINT32(3, outcome.attempts);
    TEST_ASSERT_EQUAL_UINT32(firstDelayMs + 2000 + 4000, outcome.startedAtMs);
}

void test_deferral_backs_off_without_allocating() {
    const Outcome outcome =
        run({{false, true, true}, {false, true, true}, {false, false, true}});
    TEST_ASSERT_TRUE(outcome.started);
    TEST_ASSERT_EQUAL_UINT32(1, outcome.attempts);  // deferred rounds create nothing
    TEST_ASSERT_EQUAL_UINT32(firstDelayMs + 2000 + 4000, outcome.startedAtMs);
}

void test_pairing_elsewhere_stops_the_retries() {
    const Outcome outcome = run({{false, false, false}, {true, false, true}});
    TEST_ASSERT_FALSE(outcome.started);
    TEST_ASSERT_TRUE(outcome.stopped);
    TEST_ASSERT_EQUAL_UINT32(1, outcome.attempts);
}

void test_retries_never_give_up() {
    std::vector<Round> rounds(200, Round{false, false, false});
    rounds.push_back({false, false, true});
    const Outcome outcome = run(rounds);
    TEST_ASSERT_TRUE(outcome.started);
    TEST_ASSERT_EQUAL_UINT32(201, outcome.attempts);
    TEST_ASSERT_EQUAL_UINT32(maxRetryMs, outcome.delays.back());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_unpaired_worker_is_created_when_no_foreground_tls_job_runs);
    RUN_TEST(test_foreground_tls_job_defers_the_allocation);
    RUN_TEST(test_paired_device_needs_no_worker);
    RUN_TEST(test_retry_delay_doubles_from_the_initial_delay);
    RUN_TEST(test_retry_delay_is_capped_and_never_overflows);
    RUN_TEST(test_retry_delay_edge_inputs);
    RUN_TEST(test_first_attempt_that_fits_starts_the_worker);
    RUN_TEST(test_transient_allocation_failure_recovers);
    RUN_TEST(test_deferral_backs_off_without_allocating);
    RUN_TEST(test_pairing_elsewhere_stops_the_retries);
    RUN_TEST(test_retries_never_give_up);
    return UNITY_END();
}
