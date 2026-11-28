#pragma once

#include <cstddef>
#include <cstdint>

// How the OSSM HAT reader takes bytes from the ESP-IDF UART driver. Pure
// logic, testable on the native platform; the firmware supplies the driver.
//
// When the driver's receive ring buffer is full, its interrupt keeps what it
// could not store (up to one 128-byte hardware FIFO) in a stash and stops
// receiving. Every later read call tries to move that stash into the ring and
// waits up to one RTOS tick when it does not fit yet (uart_check_buf_full in
// driver/uart.c). A reader that takes one byte per call frees one byte and
// waits a tick each time: about one byte per millisecond, while the shared bus
// delivers about 3.4. Its drain loop then never empties, so it never reaches
// the publication the motor guard reads. On the Test Rig a reader delay of a
// few tens of milliseconds just after MQTT started filled the ring this way and
// held the guard's snapshot stale for 0.8 to 5 s, past its 250 ms limit.
//
// Taking at least one FIFO per call frees room for the whole stash at once, so
// a full ring costs at most one wait. One pass is bounded by the ring's size,
// so the reader always reaches its publication afterwards.
namespace uart_drain {

constexpr size_t kChunkBytes = 128;    // the ESP32 UART hardware FIFO
constexpr size_t kMaxPassBytes = 512;  // the reader's driver ring buffer

// One pass. `available()` returns the bytes waiting (<= 0 when none),
// `read(buffer, capacity)` copies up to capacity bytes and returns the count,
// and `consume(buffer, count)` handles each chunk. Returns the bytes taken.
template <class Available, class Read, class Consume>
size_t pass(Available &&available, Read &&read, Consume &&consume,
            uint8_t (&buffer)[kChunkBytes], size_t maxBytes = kMaxPassBytes) {
    size_t total = 0;
    while (total < maxBytes) {
        const long waiting = static_cast<long>(available());
        if (waiting <= 0) break;
        size_t want = static_cast<size_t>(waiting);
        if (want > kChunkBytes) want = kChunkBytes;
        if (want > maxBytes - total) want = maxBytes - total;
        const size_t got = static_cast<size_t>(read(buffer, want));
        if (got == 0) break;
        consume(static_cast<const uint8_t *>(buffer), got > want ? want : got);
        total += got > want ? want : got;
    }
    return total;
}

}  // namespace uart_drain
