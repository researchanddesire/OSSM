#include <unity.h>
#include <vector>
#include "../../src/ossm/streaming/drawn_pattern.cpp"

namespace io = streaming::pattern_motor;
using namespace streaming;
namespace {
io::State motor;
uint64_t revision;
uint32_t clockMs;
int stops;
std::vector<int32_t> frames;
io::FrameResult feedback;
const char samples[] = "00081b335a8ee4ffffc4a75d3c3c1603";

PatternStart start(bool local = false) { return startDrawnPattern(samples, 32, 2000, revision, local); }
void tick(uint32_t count = 1) {
    for (uint32_t i = 0; i < count; ++i) {
        ++clockMs;
        serviceDrawnPattern(revision);
    }
}
}
namespace streaming::pattern_motor {
State read(uint64_t ticket) {
    auto copy = motor;
    copy.ready = copy.ready && ticket == revision;
    return copy;
}
uint32_t now() { return clockMs; }
bool current(uint64_t ticket) { return ticket == revision; }
bool position(uint64_t ticket, int32_t target, const State &, bool) {
    if (!read(ticket).ready) return false;
    motor.position = target;
    return true;
}
FrameResult submit(int32_t target, uint64_t ticket, bool) {
    if (!read(ticket).ready) return {Frame::Rejected, "guard"};
    if (feedback.state == Frame::Accepted) frames.push_back(target);
    return feedback;
}
void stop(uint64_t ticket) { if (ticket == revision) ++stops; }
}

void setUp() {
    ++revision;
    motor = {true, false, false, false, -100, -2000, -1000, 0, 8000};
    clockMs = 0;
    frames.clear();
    stops = 0;
    feedback = {io::Frame::Accepted};
    setDrawnPatternReady(revision, true);
}
void tearDown() { setDrawnPatternReady(revision, false); }

void test_zero_speed_arms_but_expiry_never_restarts() {
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    tick(4999);
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_TRUE(frames.empty());
    tick();
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_FALSE(renewDrawnPattern(revision));
    motor.speed = 100;
    tick(25);
    TEST_ASSERT_TRUE(frames.empty());
}

void test_remote_renewal_zero_stop_and_no_automatic_restart() {
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Busy), int(start()));
    tick(4990);
    TEST_ASSERT_TRUE(renewDrawnPattern(revision));
    motor.speed = 240;
    tick(700);
    TEST_ASSERT_FALSE(frames.empty());
    for (int32_t v : frames) TEST_ASSERT_TRUE(v >= motor.from && v <= motor.to);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(2000, drawnPatternStatus().periodMs);
    motor.speed = 0;
    tick();
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_EQUAL_INT(1, stops);
    const auto emitted = frames.size();
    motor.speed = 100;
    tick(20);
    TEST_ASSERT_EQUAL_UINT(emitted, frames.size());
}

void test_local_recall_survives_unrelated_remote_lease_but_obeys_user_controls() {
    motor.speed = 130;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start(true)));
    tick(5300);
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_FALSE(renewDrawnPattern(revision));
    cancelRemoteDrawnPattern();
    TEST_ASSERT_TRUE(drawnPatternActive());
    ++motor.from;
    tick();
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_EQUAL_INT(1, stops);
}

void test_stored_lookup_preflight_rejects_active_legacy_or_drawn_motion() {
    TEST_ASSERT_EQUAL_INT(int(PatternStart::NotReady), int(drawnPatternStartStatus(revision - 1)));
    motor.queued = true;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Busy), int(drawnPatternStartStatus(revision)));
    motor.queued = false;
    motor.moving = true;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Busy), int(drawnPatternStartStatus(revision)));
    motor.moving = false;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(drawnPatternStartStatus(revision)));
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Busy), int(drawnPatternStartStatus(revision)));
}

void test_guard_rejection_and_stale_task_do_not_touch_a_new_owner() {
    motor.speed = 100;
    motor.moving = true;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Busy), int(start()));
    motor.moving = false;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    setDrawnPatternReady(revision - 1, false);
    TEST_ASSERT_FALSE(renewDrawnPattern(revision - 1));
    TEST_ASSERT_FALSE(serviceDrawnPattern(revision - 1));
    tick(100);
    TEST_ASSERT_TRUE(drawnPatternActive());
    motor.ready = false;
    tick();
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_EQUAL_INT(int(PatternStart::NotReady), int(start()));
}

void test_no_phase_catchup_after_backpressure_or_a_scheduling_gap() {
    motor.speed = 200;
    motor.serial = true;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    tick(30);
    TEST_ASSERT_EQUAL_UINT(1, frames.size());
    feedback = {io::Frame::Retry, "serial_pending"};
    tick(100);
    TEST_ASSERT_EQUAL_UINT(1, frames.size());
    feedback = {io::Frame::Accepted};
    tick();
    TEST_ASSERT_EQUAL_UINT(2, frames.size());
    TEST_ASSERT_TRUE(std::abs(frames[1] - frames[0]) <= 4);
    clockMs += 201;
    tick();
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_EQUAL_UINT(2, frames.size());
}

void test_asymmetric_extension_loop_enters_at_shallow_end_and_reaches_deep_end() {
    motor.serial = true;
    motor.from = -1997;  // deepest native target
    motor.to = -1000;    // shallowest native target
    motor.speed = 100000;
    motor.acceleration = 100000000;
    const char pattern[] = "000000ffffffffff8080808040404000";
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted),
        int(startDrawnPattern(pattern, sizeof(pattern) - 1, 2000, revision)));
    tick();
    TEST_ASSERT_EQUAL_INT32(-1000, motor.position);
    tick(2001);
    TEST_ASSERT_EQUAL_UINT32(100, frames.size());
    TEST_ASSERT_EQUAL_INT32(-1000, frames[0]);   // y = 0
    TEST_ASSERT_EQUAL_INT32(-1997, frames[25]);  // y = 255
    TEST_ASSERT_EQUAL_INT32(-1500, frames[55]);  // y = 128
    TEST_ASSERT_EQUAL_INT32(-1250, frames[83]);  // y = 64
    TEST_ASSERT_EQUAL_INT32(-1000, frames[99]);  // wraps to shallow
}

void test_first_stop_diagnostic_survives_idle_stops_and_clears_only_at_accepted_start() {
    motor.speed = 160;
    clockMs = 100;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    tick(20);
    feedback = {io::Frame::Rejected, "pulse_start", -2};
    tick();
    cancelDrawnPattern();
    cancelRemoteDrawnPattern();
    setDrawnPatternReady(revision, false);
    auto status = drawnPatternStatus();
    TEST_ASSERT_FALSE(status.active);
    TEST_ASSERT_EQUAL_STRING("pulse_start", status.lastStop.reason);
    TEST_ASSERT_EQUAL_STRING("playing", status.lastStop.stage);
    TEST_ASSERT_EQUAL_UINT32(121, status.lastStop.atMs);
    TEST_ASSERT_EQUAL_UINT32(21, status.lastStop.elapsedMs);
    TEST_ASSERT_EQUAL_INT(-2, status.lastStop.driverResult);
    TEST_ASSERT_EQUAL_UINT32(1, status.lastStop.frameGapMs);
    TEST_ASSERT_EQUAL_INT(int(PatternStart::NotReady), int(start()));
    TEST_ASSERT_EQUAL_STRING("pulse_start", drawnPatternStatus().lastStop.reason);
    setDrawnPatternReady(revision, true);
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    TEST_ASSERT_NULL(drawnPatternStatus().lastStop.reason);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_asymmetric_extension_loop_enters_at_shallow_end_and_reaches_deep_end);
    RUN_TEST(test_first_stop_diagnostic_survives_idle_stops_and_clears_only_at_accepted_start);
    RUN_TEST(test_zero_speed_arms_but_expiry_never_restarts);
    RUN_TEST(test_remote_renewal_zero_stop_and_no_automatic_restart);
    RUN_TEST(test_local_recall_survives_unrelated_remote_lease_but_obeys_user_controls);
    RUN_TEST(test_guard_rejection_and_stale_task_do_not_touch_a_new_owner);
    RUN_TEST(test_no_phase_catchup_after_backpressure_or_a_scheduling_gap);
    RUN_TEST(test_stored_lookup_preflight_rejects_active_legacy_or_drawn_motion);
    return UNITY_END();
}
