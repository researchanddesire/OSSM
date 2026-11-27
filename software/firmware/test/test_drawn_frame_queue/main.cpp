#include <unity.h>
#include <array>
#include <deque>
#include "../../src/ossm/streaming/drawn_frame_queue.h"

using namespace streaming::frame_plan;
using namespace drawn_pattern;

namespace {
void equalCursor(const Cursor &expected, const Cursor &actual) {
    TEST_ASSERT_EQUAL_MEMORY(&expected.phase, &actual.phase, sizeof(MarkedPhase));
    TEST_ASSERT_EQUAL_MEMORY(&expected.vibration, &actual.vibration, sizeof(VibrationClock));
    TEST_ASSERT_EQUAL_INT32(expected.target, actual.target);
    TEST_ASSERT_EQUAL(expected.targetKnown, actual.targetKnown);
}

void equalFrame(const Frame &expected, const Frame *actual) {
    TEST_ASSERT_NOT_NULL(actual);
    equalCursor(expected, *actual);
    TEST_ASSERT_EQUAL_INT32(expected.from, actual->from);
    TEST_ASSERT_EQUAL_UINT32(expected.planUs, actual->planUs);
    TEST_ASSERT_EQUAL_UINT16(expected.crossing, actual->crossing);
    TEST_ASSERT_EQUAL(expected.valid, actual->valid);
    TEST_ASSERT_EQUAL(expected.offered, actual->offered);
}

Frame advanceFrame(const Cursor &before, const MarkerIndex &index, unsigned sequence) {
    Frame frame{};
    static_cast<Cursor &>(frame) = before;
    const PhaseLimits limits{.7, 1.3, 18.};
    // Exercise simultaneous phase, slew, dwell and carrier progress, through
    // the authored seam and alternately zero/nonzero live Sensation.
    const double strength = (sequence / 570) % 3 * .5;
    frame.phase.advance(.002, limits, index, strength);
    frame.vibration.advance(.002, strength);
    frame.from = before.target;
    frame.target = static_cast<int32_t>(frame.phase.phase * 10000) - 5000;
    frame.targetKnown = true;
    frame.planUs = sequence ? 73 * sequence : UINT32_MAX;
    frame.crossing = static_cast<uint16_t>(sequence * 1999);
    frame.valid = true;
    return frame;
}

void test_retries_wrap_marks_and_retained_boundary_match_uncompressed_rows() {
    Modifiers spec;
    spec.count = 2;
    spec.entries[0] = {120, 120, 250};
    spec.entries[1] = {180, 870, 0};
    MarkerIndex index;
    TEST_ASSERT_TRUE(index.reset(spec, .375));
    FrameQueue queue;
    std::deque<Frame> expected;
    Cursor accepted{}, tail{};
    queue.clear(accepted);
    unsigned produced = 0;
    unsigned peak = 0;
    for (unsigned iteration = 0; iteration < 2500; ++iteration) {
        while (!queue.full()) {
            auto next = advanceFrame(tail, index, produced++);
            TEST_ASSERT_TRUE(queue.push(next, tail));
            expected.push_back(next);
            tail = next;
        }
        peak = std::max(peak, queue.size());
        TEST_ASSERT_EQUAL_UINT(expected.size(), queue.size());
        auto *first = queue.front();
        equalFrame(expected.front(), first);
        first->offered = expected.front().offered = true;
        TEST_ASSERT_EQUAL_PTR(first, queue.front());
        if (iteration % 11 == 0) {
            const unsigned requested = (iteration / 11) % 12;
            const unsigned keep = std::min(requested, FrameQueue::maxRetained);
            Cursor boundary{};
            TEST_ASSERT_EQUAL_UINT(keep, queue.truncate(requested, boundary));
            while (expected.size() > keep) expected.pop_back();
            equalCursor(keep ? static_cast<const Cursor &>(expected.back()) : accepted, boundary);
            tail = boundary;
            if (!keep) {
                TEST_ASSERT_NULL(queue.front());
                continue;
            }
            TEST_ASSERT_EQUAL_PTR(first, queue.front());
            TEST_ASSERT_TRUE(first->offered);
        }
        const unsigned pops = std::min(3u, queue.size());
        for (unsigned k = 0; k < pops; ++k) {
            equalFrame(expected.front(), queue.front());
            accepted = expected.front();
            expected.pop_front();
            queue.pop();
        }
    }
    TEST_ASSERT_EQUAL_UINT(FrameQueue::capacity, peak);
    Cursor reset{};
    reset.target = INT32_MIN;
    queue.clear(reset);
    TEST_ASSERT_NULL(queue.front());
    Cursor boundary{};
    TEST_ASSERT_EQUAL_UINT(0, queue.truncate(8, boundary));
    equalCursor(reset, boundary);
}

// On the supported little-endian IEEE platforms, every word filled with
// zero or the integer 1 gives valid booleans, enum values and finite doubles.
// This forces all words of the cursor to change, independently of any ratio
// achieved by a particular artwork. The supplied cursors are never advanced.
Cursor literalCursor(uint32_t value) {
    std::array<uint32_t, sizeof(Cursor) / sizeof(uint32_t)> words{};
    words.fill(value);
    Cursor cursor;
    std::memcpy(static_cast<void *>(&cursor), words.data(), sizeof(cursor));
    return cursor;
}

void test_complete_literal_rows_and_full_queue_refusal_are_lossless() {
    FrameQueue queue;
    Cursor before = literalCursor(0);
    queue.clear(before);
    std::deque<Frame> expected;
    unsigned produced = 0;
    for (unsigned round = 0; round < 50; ++round) {
        while (!queue.full()) {
            Frame row{};
            static_cast<Cursor &>(row) = literalCursor((produced + 1) % 2);
            row.from = produced % 2 ? INT32_MIN : INT32_MAX;
            row.planUs = UINT32_MAX - produced;
            row.crossing = produced % 2 ? 0 : UINT16_MAX;
            row.valid = produced % 2;
            row.offered = !row.valid;
            TEST_ASSERT_TRUE(queue.push(row, before));
            expected.push_back(row);
            before = row;
            ++produced;
        }
        TEST_ASSERT_GREATER_OR_EQUAL_UINT(12, queue.size());
        TEST_ASSERT_LESS_THAN_UINT(FrameQueue::capacity, queue.size());
        TEST_ASSERT_FALSE(queue.push(Frame{}, before));
        TEST_ASSERT_EQUAL_UINT(expected.size(), queue.size());
        for (unsigned i = 0; i < 8; ++i) {
            equalFrame(expected.front(), queue.front());
            queue.pop();
            expected.pop_front();
        }
    }
    while (!expected.empty()) {
        equalFrame(expected.front(), queue.front());
        queue.pop();
        expected.pop_front();
    }
    queue.pop();
    TEST_ASSERT_NULL(queue.front());
}

void test_fully_populated_markers_survive_queue_clear_and_new_generation() {
    Modifiers spec;
    TEST_ASSERT_TRUE(spec.reserve(Modifiers::capacity));
    spec.count = Modifiers::capacity;
    for (uint16_t i = 0; i < spec.count; ++i) spec.entries[i] = {i, i, 1};
    MarkerIndex index;
    TEST_ASSERT_TRUE(index.reset(spec, 0));
    FrameQueue queue;
    Cursor before{};
    for (unsigned gen = 1; gen <= 8; ++gen) {
        queue.clear(before);
        std::deque<Frame> expected;
        while (!queue.full()) {
            const auto row = advanceFrame(before, index, 570 * gen + queue.size());
            TEST_ASSERT_TRUE(queue.push(row, before));
            before = row;
            expected.push_back(row);
        }
        while (!expected.empty()) {
            equalFrame(expected.front(), queue.front());
            before = expected.front();
            expected.pop_front();
            queue.pop();
        }
        TEST_ASSERT_NULL(queue.front());
    }
}
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_retries_wrap_marks_and_retained_boundary_match_uncompressed_rows);
    RUN_TEST(test_complete_literal_rows_and_full_queue_refusal_are_lossless);
    RUN_TEST(test_fully_populated_markers_survive_queue_clear_and_new_generation);
    return UNITY_END();
}
