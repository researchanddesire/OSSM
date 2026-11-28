#include <unity.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "uart_drain.h"

using uart_drain::kChunkBytes;
using uart_drain::kMaxPassBytes;

void setUp() {}
void tearDown() {}

namespace {

// The ESP-IDF 4.4 UART receive path as the HAT reader sees it, on a 10 us
// clock: a 128-byte hardware FIFO, the reader's 512-byte ring buffer, and the
// interrupt's stash. A full ring stops receiving; the FIFO then overflows and
// is reset. After each read call the driver tries to move the stash into the
// ring and, when it does not fit, waits one RTOS tick (1 ms) first.
struct IdfUart {
    static constexpr size_t kFifo = 128;
    static constexpr size_t kRing = 512;
    uint64_t ticks10us = 0;
    size_t fifo = 0, ring = 0, stash = 0;
    bool full = false;
    double arrival = 0;     // bytes due from the bus, fractional
    double bytesPerMs = 3.4;  // the guard's ~98 Hz telemetry polls
    uint32_t calls = 0, waits = 0, lost = 0;

    void interrupt() {
        if (full || fifo == 0) return;
        if (kRing - ring >= fifo) {
            ring += fifo;
        } else {
            full = true;
            stash = fifo;
        }
        fifo = 0;
    }
    void advance(uint32_t units) {
        for (uint32_t i = 0; i < units; ++i) {
            ++ticks10us;
            arrival += bytesPerMs / 100.0;
            while (arrival >= 1.0) {
                arrival -= 1.0;
                if (fifo == kFifo) {
                    lost += fifo;  // overflow: the driver resets the FIFO
                    fifo = 0;
                }
                ++fifo;
            }
            interrupt();
        }
    }
    int available() const { return static_cast<int>(ring); }
    size_t read(uint8_t *, size_t capacity) {
        ++calls;
        advance(1);  // a call takes about 10 us on its own
        const size_t taken = capacity < ring ? capacity : ring;
        ring -= taken;
        if (full) {
            if (kRing - ring >= stash) {
                ring += stash;
                stash = 0;
                full = false;
                interrupt();
            } else {
                ++waits;
                advance(100);  // xRingbufferSend(..., 1 tick) times out
            }
        }
        return taken;
    }
    double ms() const { return ticks10us / 100.0; }
};

// The state the rig reached: a reader delay filled the ring, and the FIFO's
// first bytes sit in the stash.
IdfUart fullRing() {
    IdfUart uart;
    uart.ring = IdfUart::kRing - 20;
    uart.fifo = 40;
    uart.interrupt();
    TEST_ASSERT_TRUE(uart.full);
    return uart;
}

// The reader loop: one bounded pass, then publication and a 1 ms sleep.
// Returns the time of the first publication after the ring emptied, or a
// negative value if none came within `limitMs`.
double drainUntilCaughtUp(IdfUart &uart, size_t chunk, double limitMs) {
    uint8_t buffer[kChunkBytes];
    while (uart.ms() < limitMs) {
        const size_t taken = uart_drain::pass(
            [&] { return uart.available(); },
            [&](uint8_t *out, size_t capacity) {
                return uart.read(out, capacity < chunk ? capacity : chunk);
            },
            [](const uint8_t *, size_t) {}, buffer);
        if (!uart.full && uart.ring == 0 && taken < kMaxPassBytes) return uart.ms();
        uart.advance(100);
    }
    return -1;
}

// The reader before this change: one byte per call, until none is waiting.
double perByteUntilEmpty(IdfUart &uart, double limitMs) {
    uint8_t byte;
    while (uart.available() > 0) {
        uart.read(&byte, 1);
        if (uart.ms() >= limitMs) return -1;
    }
    return uart.ms();
}

}  // namespace

void test_one_byte_per_call_never_catches_up_with_a_full_ring() {
    IdfUart uart = fullRing();
    TEST_ASSERT_TRUE(perByteUntilEmpty(uart, 1000) < 0);
    // About one byte per tick: the drain loop never reaches its publication.
    TEST_ASSERT_GREATER_THAN_UINT32(800, uart.waits);
    TEST_ASSERT_GREATER_THAN_UINT32(0, uart.lost);
}

void test_whole_chunks_recover_a_full_ring_within_milliseconds() {
    IdfUart uart = fullRing();
    const double caughtUp = drainUntilCaughtUp(uart, kChunkBytes, 1000);
    TEST_ASSERT_TRUE(caughtUp >= 0);
    TEST_ASSERT_TRUE(caughtUp < 5);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(2, uart.waits);
}

// 115200 baud with no idle time at all, which Modbus never reaches: the ring
// may never be exactly empty, but every pass ends, so the reader publishes
// each loop, and the ring never fills again after the first recovery.
void test_chunks_keep_up_with_a_saturated_bus() {
    IdfUart uart = fullRing();
    uart.bytesPerMs = 11.5;
    uint8_t buffer[kChunkBytes];
    size_t passes = 0, largestAfterRecovery = 0;
    while (uart.ms() < 1000) {
        uart_drain::pass([&] { return uart.available(); },
                         [&](uint8_t *out, size_t capacity) { return uart.read(out, capacity); },
                         [](const uint8_t *, size_t) {}, buffer);
        ++passes;
        if (passes > 1 && uart.ring > largestAfterRecovery) largestAfterRecovery = uart.ring;
        uart.advance(100);
    }
    TEST_ASSERT_GREATER_THAN_UINT32(900, passes);
    TEST_ASSERT_FALSE(uart.full);
    TEST_ASSERT_LESS_THAN_UINT32(IdfUart::kFifo, largestAfterRecovery);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(2, uart.waits);
}

void test_a_pass_is_bounded_and_reads_at_most_one_fifo_per_call() {
    std::vector<size_t> requested;
    size_t waiting = 5000;
    uint8_t buffer[kChunkBytes];
    size_t consumed = 0;
    const size_t taken = uart_drain::pass(
        [&] { return static_cast<int>(waiting); },
        [&](uint8_t *, size_t capacity) {
            requested.push_back(capacity);
            waiting -= capacity;
            return capacity;
        },
        [&](const uint8_t *chunk, size_t count) {
            TEST_ASSERT_EQUAL_PTR(buffer, chunk);
            consumed += count;
        },
        buffer);
    TEST_ASSERT_EQUAL_UINT32(kMaxPassBytes, taken);
    TEST_ASSERT_EQUAL_UINT32(kMaxPassBytes, consumed);
    TEST_ASSERT_EQUAL_UINT32(kMaxPassBytes / kChunkBytes, requested.size());
    for (size_t size : requested) TEST_ASSERT_EQUAL_UINT32(kChunkBytes, size);
}

void test_a_pass_asks_only_for_what_is_waiting_and_respects_its_limit() {
    std::vector<size_t> requested;
    size_t waiting = 33;
    uint8_t buffer[kChunkBytes];
    const size_t taken = uart_drain::pass(
        [&] { return static_cast<int>(waiting); },
        [&](uint8_t *, size_t capacity) {
            requested.push_back(capacity);
            waiting -= capacity;
            return capacity;
        },
        [](const uint8_t *, size_t) {}, buffer, 20);
    TEST_ASSERT_EQUAL_UINT32(20, taken);
    TEST_ASSERT_EQUAL_UINT32(1, requested.size());
    TEST_ASSERT_EQUAL_UINT32(20, requested[0]);
}

void test_a_pass_stops_when_nothing_is_waiting_or_a_read_returns_nothing() {
    uint8_t buffer[kChunkBytes];
    int calls = 0;
    TEST_ASSERT_EQUAL_UINT32(0, uart_drain::pass([] { return 0; },
        [&](uint8_t *, size_t) { ++calls; return size_t{1}; },
        [](const uint8_t *, size_t) {}, buffer));
    TEST_ASSERT_EQUAL(0, calls);
    TEST_ASSERT_EQUAL_UINT32(0, uart_drain::pass([] { return -1; },
        [&](uint8_t *, size_t) { ++calls; return size_t{1}; },
        [](const uint8_t *, size_t) {}, buffer));
    TEST_ASSERT_EQUAL(0, calls);
    // The driver said bytes were waiting but returned none: no busy loop.
    TEST_ASSERT_EQUAL_UINT32(0, uart_drain::pass([] { return 10; },
        [&](uint8_t *, size_t) { ++calls; return size_t{0}; },
        [](const uint8_t *, size_t) {}, buffer));
    TEST_ASSERT_EQUAL(1, calls);
}

void test_a_read_that_reports_more_than_asked_is_clipped() {
    uint8_t buffer[kChunkBytes];
    size_t consumed = 0;
    int reads = 0;
    const size_t taken = uart_drain::pass([&] { return reads < 1 ? 10 : 0; },
        [&](uint8_t *, size_t) { ++reads; return size_t{500}; },
        [&](const uint8_t *, size_t count) { consumed += count; }, buffer);
    TEST_ASSERT_EQUAL_UINT32(10, taken);
    TEST_ASSERT_EQUAL_UINT32(10, consumed);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_one_byte_per_call_never_catches_up_with_a_full_ring);
    RUN_TEST(test_whole_chunks_recover_a_full_ring_within_milliseconds);
    RUN_TEST(test_chunks_keep_up_with_a_saturated_bus);
    RUN_TEST(test_a_pass_is_bounded_and_reads_at_most_one_fifo_per_call);
    RUN_TEST(test_a_pass_asks_only_for_what_is_waiting_and_respects_its_limit);
    RUN_TEST(test_a_pass_stops_when_nothing_is_waiting_or_a_read_returns_nothing);
    RUN_TEST(test_a_read_that_reports_more_than_asked_is_clipped);
    return UNITY_END();
}
