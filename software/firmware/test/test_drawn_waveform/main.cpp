#include <unity.h>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include "drawn_waveform.h"
#include "drawn_pulse_queue.h"

using drawn_pattern::Waveform;
void setUp() {}
void tearDown() {}

static std::string encode(const std::array<int, 16>& points) {
    char hex[33]{};
    for (size_t i = 0; i < points.size(); ++i)
        std::snprintf(hex + 2 * i, 3, "%02x", points[i]);
    return hex;
}
const std::array<int, 16> asymmetric{0, 8, 27, 51, 90, 142, 228, 255,
                                     255, 196, 167, 93, 60, 60, 22, 3};

void test_wire_bounds_are_checked_before_motion() {
    Waveform curve;
    auto hex = encode(asymmetric);
    TEST_ASSERT_TRUE(curve.decode(hex.data(), hex.size(), 500));
    TEST_ASSERT_TRUE(curve.decode(hex.data(), hex.size(), 60000));
    TEST_ASSERT_FALSE(curve.decode(hex.data(), hex.size(), 499));
    TEST_ASSERT_FALSE(curve.decode(hex.data(), hex.size(), 60001));
    TEST_ASSERT_FALSE(curve.decode(nullptr, 32, 500));
    TEST_ASSERT_FALSE(curve.decode(hex.data(), 30, 500));
    TEST_ASSERT_FALSE(curve.decode(hex.data(), 33, 500));
    hex[7] = '\0';
    TEST_ASSERT_FALSE(curve.decode(hex.data(), hex.size(), 500));
    const std::string largest(2 * drawn_pattern::maxSamples, 'F');
    TEST_ASSERT_TRUE(curve.decode(largest.data(), largest.size(), 500));
    TEST_ASSERT_FALSE(curve.decode(largest.data(), largest.size() + 2, 500));
}

void test_passes_through_all_samples_no_overshoot_and_continuous_velocity() {
    Waveform curve;
    const auto shape = encode(asymmetric);
    TEST_ASSERT_TRUE(curve.decode(shape.data(), shape.size(), 2000));
    constexpr double epsilon = .000001;
    for (size_t i = 0; i < asymmetric.size(); ++i) {
        const double phase = double(i) / asymmetric.size();
        const double p = curve.position(phase);
        TEST_ASSERT_FLOAT_WITHIN(.00001f, asymmetric[i] / 255.f, p);
        const double left = (p - curve.position(phase - epsilon)) / epsilon;
        const double right = (curve.position(phase + epsilon) - p) / epsilon;
        TEST_ASSERT_FLOAT_WITHIN(.002f, left, right);
        const int next = asymmetric[(i + 1) % asymmetric.size()];
        if (i == 4) TEST_ASSERT_GREATER_THAN_FLOAT(1, left);
        for (size_t k = 0; k < 100; ++k) {
            const double v = curve.position(phase + k / 1600.0) * 255.0;
            TEST_ASSERT_TRUE(v >= std::min(asymmetric[i], next) - .00001);
            TEST_ASSERT_TRUE(v <= std::max(asymmetric[i], next) + .00001);
        }
    }
}

void test_phase_starts_at_a_resting_extremum_and_loops_without_a_jump() {
    Waveform curve;
    auto rotated = asymmetric;
    std::rotate(rotated.begin(), rotated.begin() + 5, rotated.end());
    const auto shape = encode(rotated);
    TEST_ASSERT_TRUE(curve.decode(shape.data(), shape.size(), 2000));
    TEST_ASSERT_FLOAT_WITHIN(.00001, 0, curve.position(0));
    TEST_ASSERT_FLOAT_WITHIN(.0001, 0, curve.position(.000001) / .000001);
    for (size_t i = 0; i <= 32; ++i)
        TEST_ASSERT_FLOAT_WITHIN(.000001, curve.position(i / 32.), curve.position(1 + i / 32.));
}

void test_authored_extension_maps_every_asymmetric_sample_to_the_native_motor_frame() {
    Waveform curve;
    const auto shape = encode(asymmetric);
    TEST_ASSERT_TRUE(curve.decode(shape.data(), shape.size(), 2000));
    // A 255-step-wide window makes the expected extension in steps exact.
    for (size_t i = 0; i < asymmetric.size(); ++i) {
        const double phase = double(i) / asymmetric.size();
        TEST_ASSERT_EQUAL_INT32(-100 - asymmetric[i], curve.nativeTarget(phase, -355, -100));
    }
    TEST_ASSERT_EQUAL_INT32(-100, curve.nativeTarget(1, -355, -100));
}

void test_time_stretch_bounds_entire_curve_at_speed_and_acceleration_caps() {
    Waveform curve;
    const auto shape = encode(asymmetric);
    TEST_ASSERT_TRUE(curve.decode(shape.data(), shape.size(), 500));
    for (double acceleration : {50., 20000.}) {
        const double span = 2300, maxSpeed = 700;
        const double seconds = curve.safePeriodMs(span, maxSpeed, acceleration) / 1000.;
        TEST_ASSERT_GREATER_THAN_FLOAT(.5, seconds);
        constexpr double fraction = 1. / 32768;
        const double dt = seconds * fraction;
        double p = curve.position(-fraction) * span;
        double v = (p - curve.position(-2 * fraction) * span) / dt;
        for (size_t i = 0; i < 32768; ++i) {
            const double next = curve.position(i * fraction) * span;
            const double speed = (next - p) / dt;
            TEST_ASSERT_TRUE(std::abs(speed) <= maxSpeed + .001);
            TEST_ASSERT_TRUE(std::abs(speed - v) / dt <= acceleration + .001);
            p = next;
            v = speed;
        }
    }
    TEST_ASSERT_EQUAL_UINT32(0, curve.safePeriodMs(2300, 0, 100));
}

void test_step_frames_track_multiple_periods_without_waypoint_stops_or_drift() {
    Waveform curve;
    const auto shape = encode(asymmetric);
    TEST_ASSERT_TRUE(curve.decode(shape.data(), shape.size(), 4000));
    drawn_pattern::PulseFrame frame{};
    int32_t actual = -2200;
    uint64_t elapsedTicks = 0;
    int32_t beforeMiddle = actual;
    for (size_t n = 1; n <= 6000; ++n) {
        const auto target = -2200 + static_cast<int32_t>(std::lround(2000 * curve.position(n / 2000.0)));
        TEST_ASSERT_TRUE(drawn_pattern::pulseFrame(actual, target, 32000, frame.increasing, frame));
        actual += frame.increasing ? frame.steps : -int(frame.steps);
        elapsedTicks += uint32_t(frame.ticks) * std::max<int>(1, frame.steps);
        TEST_ASSERT_EQUAL_INT32(target, actual);
        if (n == 498) beforeMiddle = actual;
        if (n == 502) TEST_ASSERT_GREATER_THAN_INT32(8, actual - beforeMiddle);
    }
    TEST_ASSERT_EQUAL_INT32(-2200, actual);
    TEST_ASSERT_TRUE(std::abs(static_cast<int64_t>(elapsedTicks) - 192000000) < 16000);
    TEST_ASSERT_TRUE(drawn_pattern::pulseFrame(actual, actual, 32000, true, frame));
    TEST_ASSERT_EQUAL_UINT8(0, frame.steps);
    TEST_ASSERT_TRUE(frame.increasing);
    TEST_ASSERT_EQUAL_UINT16(32000, frame.ticks);
    TEST_ASSERT_FALSE(drawn_pattern::pulseFrame(INT32_MIN, INT32_MAX, 32000, false, frame));
}

namespace {
struct PulseCommand { uint16_t ticks; uint8_t steps; bool count_up; };
struct InterruptingQueue {
    unsigned count = 0, starts = 0;
    bool running = false, discarded = false, drainOnAppend = false;
    int8_t startResult = 0;
    int32_t tail = -896;
    unsigned queueEntries() const { return count; }
    int32_t getPositionAfterCommandsCompleted() const { return tail; }
    bool isQueueRunning() const { return running; }
    int addQueueEntry(const PulseCommand *command, bool start) {
        if (command && !discarded) {
            if (drainOnAppend) { count = 0; running = false; }
            ++count;
            tail += command->count_up ? command->steps : -int(command->steps);
        }
        if (start) {
            ++starts;
            if (startResult) return startResult;
            // The MCPWM compare interrupt may retire the first hold before
            // the producer runs again, even for a long programmed interval.
            if (count) --count;
            running = count > 0;
        }
        return 0;
    }
};
}

void test_raw_queue_prefills_leading_holds_and_starts_once_with_bounded_lookahead() {
    using namespace drawn_pattern;
    PulseQueue q;
    InterruptingQueue driver;
    for (unsigned i = 0; i < pulseQueueFrames; ++i) {
        const auto result = q.submit<PulseCommand>(driver, -896, 32000);
        TEST_ASSERT_EQUAL_INT(int(Frame::Accepted), int(result.state));
        TEST_ASSERT_EQUAL_UINT(i + 1 == pulseQueueFrames, driver.starts);
    }
    TEST_ASSERT_TRUE(driver.running);
    TEST_ASSERT_FALSE(q.filling());
    TEST_ASSERT_EQUAL_INT(int(Frame::Accepted), int(q.submit<PulseCommand>(driver, -895, 32000).state));
    TEST_ASSERT_EQUAL_INT(int(Frame::Retry), int(q.submit<PulseCommand>(driver, -894, 32000).state));
    TEST_ASSERT_EQUAL_UINT(pulseQueueFrames, driver.count);
    driver.count = 1;
    driver.drainOnAppend = true;
    const auto underrun = q.submit<PulseCommand>(driver, -894, 32000);
    TEST_ASSERT_EQUAL_STRING("pulse_underrun", underrun.reason);
    TEST_ASSERT_EQUAL_UINT(1, driver.starts);
    TEST_ASSERT_FALSE(driver.running);
    TEST_ASSERT_EQUAL_INT(int(Frame::Rejected), int(q.submit<PulseCommand>(driver, -894, 32000).state));
    TEST_ASSERT_EQUAL_UINT(1, driver.starts);
}

void test_unstarted_raw_queue_preserves_physical_position_and_reports_driver_errors() {
    using namespace drawn_pattern;
    PulseQueue q;
    InterruptingQueue driver;
    driver.discarded = true;
    const auto discarded = q.submit<PulseCommand>(driver, -896, 32000);
    TEST_ASSERT_EQUAL_STRING("pulse_discarded", discarded.reason);
    TEST_ASSERT_EQUAL_INT32(-896, q.initialPosition());
    TEST_ASSERT_TRUE(q.filling());
    TEST_ASSERT_FALSE(driver.running);
    q.reset();
    TEST_ASSERT_FALSE(q.owned());
    driver.discarded = false;
    driver.startResult = -2;
    for (unsigned i = 1; i < pulseQueueFrames; ++i)
        TEST_ASSERT_EQUAL_INT(int(Frame::Accepted), int(q.submit<PulseCommand>(driver, -895, 32000).state));
    const auto refused = q.submit<PulseCommand>(driver, -894, 32000);
    TEST_ASSERT_EQUAL_INT(int(Frame::Rejected), int(refused.state));
    TEST_ASSERT_EQUAL_STRING("pulse_start", refused.reason);
    TEST_ASSERT_EQUAL_INT(-2, refused.driverResult);
    TEST_ASSERT_TRUE(q.filling());
    TEST_ASSERT_FALSE(driver.running);
    TEST_ASSERT_EQUAL_INT32(-896, q.initialPosition());
    TEST_ASSERT_EQUAL_INT32(-894, driver.tail);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_authored_extension_maps_every_asymmetric_sample_to_the_native_motor_frame);
    RUN_TEST(test_raw_queue_prefills_leading_holds_and_starts_once_with_bounded_lookahead);
    RUN_TEST(test_unstarted_raw_queue_preserves_physical_position_and_reports_driver_errors);
    RUN_TEST(test_wire_bounds_are_checked_before_motion);
    RUN_TEST(test_passes_through_all_samples_no_overshoot_and_continuous_velocity);
    RUN_TEST(test_phase_starts_at_a_resting_extremum_and_loops_without_a_jump);
    RUN_TEST(test_time_stretch_bounds_entire_curve_at_speed_and_acceleration_caps);
    RUN_TEST(test_step_frames_track_multiple_periods_without_waypoint_stops_or_drift);
    return UNITY_END();
}
