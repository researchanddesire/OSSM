#include <unity.h>
#include <array>
#include <vector>
#include "drawn_modifier_motion.h"
#include <cmath>
#include <cstdio>
#include <string>
#include "drawn_waveform.h"
#include "drawn_phase_rate.h"
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

void test_stationary_entry_keeps_time_across_late_visits_and_discards_debt_on_reset() {
    using namespace drawn_pattern;
    StationaryEntryClock clock;
    const uint32_t start = UINT32_MAX - 4000;
    unsigned accepted = 0;
    // Irregular service turns span a millisecond rollover and a long lead-in.
    // The real feeder may submit up to three ordinary rows per visit.
    for (uint32_t elapsed = 0; elapsed <= 10179; elapsed += elapsed % 5 == 0 ? 3 : 2) {
        for (unsigned i = 0; i < 3 && clock.due(start + elapsed); ++i) {
            clock.accepted(start + elapsed);
            ++accepted;
        }
        TEST_ASSERT_EQUAL_UINT32(1 + elapsed / pulseFrameMs, accepted);
    }
    clock.reset();
    const auto resumed = start + 60000;
    TEST_ASSERT_TRUE(clock.due(resumed));
    clock.accepted(resumed);
    TEST_ASSERT_FALSE(clock.due(resumed));
    TEST_ASSERT_FALSE(clock.due(resumed + 1));
    TEST_ASSERT_TRUE(clock.due(resumed + pulseFrameMs));
}

void test_finite_prime_tail_uses_programmed_limits_and_verified_position() {
    using namespace drawn_pattern;
    PulsePrimeWait wait;
    const uint32_t start = UINT32_MAX - 300;
    wait.start(-1, start, 16000000, 16000, 50000); // 1 Hz, crosses millis rollover
    TEST_ASSERT_EQUAL_INT(int(FrameTiming::Unknown), int(wait.poll(start + 2, true, 0).timing));
    TEST_ASSERT_EQUAL_INT(int(FrameTiming::PrimeTail), int(wait.poll(start + 999, true, -1).timing));
    TEST_ASSERT_EQUAL_INT(int(Frame::Accepted), int(wait.poll(start + 1001, false, -1).state));
    TEST_ASSERT_EQUAL_STRING("prime_timeout", wait.poll(start + 1020, true, -1).reason);

    wait.start(1, 0, 80000, 16000, 50000); // 200 Hz, ordinary 200 ms deadline
    TEST_ASSERT_EQUAL_INT(int(Frame::Retry), int(wait.poll(2, false, 0).state));
    TEST_ASSERT_EQUAL_STRING("prime_position", wait.poll(20, false, 0).reason);
    TEST_ASSERT_EQUAL_STRING("prime_timeout", wait.poll(200, true, 1).reason);

    wait.start(1, 0, 80000, 16000, 1); // acceleration, rather than speed, constrains the step
    TEST_ASSERT_EQUAL_INT(int(FrameTiming::PrimeTail), int(wait.poll(710, true, 1).timing));
    TEST_ASSERT_EQUAL_STRING("prime_timeout", wait.poll(1020, true, 1).reason);
}

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
        const double seconds = 1. / curve.pace(span, maxSpeed, acceleration, 1., 2).rate;
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
    TEST_ASSERT_EQUAL_UINT32(0, curve.pace(2300, 0, 100, 1., 2).periodMs);
}

void test_step_frames_track_multiple_periods_without_waypoint_stops_or_drift() {
    Waveform curve;
    const auto shape = encode(asymmetric);
    TEST_ASSERT_TRUE(curve.decode(shape.data(), shape.size(), 4000));
    int32_t previous=curve.nativeTarget(0,-2200,-200);
    const auto initial=previous;
    int32_t beforeMiddle=0;
    int64_t traveled=0;
    for (size_t n=1;n<=6000;++n) {
        const auto target=curve.nativeTarget(n/2000.,-2200,-200);
        traveled+=int64_t(target)-previous;
        if (n==498) beforeMiddle=target;
        if (n==502) TEST_ASSERT_GREATER_THAN_INT32(8,beforeMiddle-target);
        if (n%2000==0) TEST_ASSERT_EQUAL_INT32(initial,target);
        previous=target;
    }
    TEST_ASSERT_EQUAL_INT32(0,traveled);
}

namespace {
struct PulseCommand { uint16_t ticks; uint8_t steps; bool count_up; };
struct InterruptingQueue {
    unsigned count = 0, starts = 0;
    bool running = false, discarded = false, drainOnAppend = false;
    int8_t startResult = 0;
    PulseCommand last{};
    int8_t refuseHold = 0;
    uint32_t queuedTicks = 0;
    std::vector<PulseCommand> journal;
    bool retireImmediately = false;
    unsigned moveTimedCalls = 0;
    int32_t tail = -896;
    unsigned queueEntries() const { return count; }
    bool isQueueEmpty() const { return count == 0; }
    int32_t getCurrentPosition() const { return -896; }
    int32_t getPositionAfterCommandsCompleted() const { return tail; }
    bool isQueueRunning() const { return running; }
    uint16_t getMaxSpeedInTicks() const { return 80; }
    uint32_t ticksInQueue() const { return queuedTicks; }
    int moveTimed(int16_t steps,uint32_t duration,uint32_t *actual,bool start) {
        ++moveTimedCalls;
        const bool wasEmpty=count==0;
        const auto n=static_cast<uint8_t>(std::abs(steps));
        PulseCommand cmd{static_cast<uint16_t>(duration/n),n,steps>0};
        *actual=cmd.ticks*n;
        const auto code=addQueueEntry(&cmd,start);
        return code ? code : (wasEmpty?7:0);
    }
    int addQueueEntry(const PulseCommand *command, bool start) {
        if (command && !command->steps && refuseHold) return refuseHold;
        if (command && !discarded) {
            journal.push_back(*command);
            if (drainOnAppend) { count = 0; running = false; }
            last=*command;
            if (!retireImmediately || !running) ++count;
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

#include "refill_driver.h"

void test_raw_watermark_recovers_between_measured_size_service_gaps() {
    // Rounded 2 / 4 mm/s brake periods observed on the same 12 mm Q8
    // calibration. The spacing of these stress intervals is synthetic.
    for (auto period : {19737, 9869}) {
        const auto fixedTick = replayDrain(period, 3, false);
        const auto refill = replayDrain(period, 3, true);
        TEST_ASSERT_TRUE(fixedTick.underrun);
        TEST_ASSERT_FALSE(refill.underrun);
        TEST_ASSERT_GREATER_OR_EQUAL_UINT(2300, refill.accepted);
        TEST_ASSERT_GREATER_THAN_UINT(0, refill.immediate);
        TEST_ASSERT_LESS_THAN_UINT(drawn_pattern::pulseStopDrainMs * 1000, refill.drainUs);
        // Two adjacent disruptions can exceed the actual reserve. Stop at
        // that first depletion even when spare software frames are available.
        TEST_ASSERT_TRUE(replayDrain(period, 1, true).underrun);
    }
}

void test_stationary_entry_requires_untouched_empty_driver_and_matching_position() {
    using namespace drawn_pattern;
    using Entry = PulseQueue::Entry;
    PulseQueue q;
    InterruptingQueue driver;
    TEST_ASSERT_EQUAL_INT(int(Entry::Stationary), int(q.entryState(driver, -896)));
    TEST_ASSERT_EQUAL_INT(int(Entry::FirstMove), int(q.entryState(driver, -895)));
    TEST_ASSERT_EQUAL_INT(int(Entry::Raw), int(q.entryState(driver, -896, true)));
    driver.running = true;
    TEST_ASSERT_EQUAL_INT(int(Entry::Busy), int(q.entryState(driver, -896)));
    driver.running = false;
    driver.count = 1;
    TEST_ASSERT_EQUAL_INT(int(Entry::Busy), int(q.entryState(driver, -896)));
    driver.count = 0;
    q.submit<PulseCommand>(driver, -895, 32000, 32768, 3200, 7);
    TEST_ASSERT_EQUAL_INT(int(Entry::Raw), int(q.entryState(driver, -896)));
    for (unsigned i = 1; i < pulseQueueFrames; ++i)
        q.submit<PulseCommand>(driver, -895, 32000, 32768, 3200, 7);
    driver.count = 0;
    driver.running = false;
    TEST_ASSERT_EQUAL_INT(int(Entry::Raw), int(q.entryState(driver, -896)));
    const auto result = q.submit<PulseCommand>(driver, -896, 32000, 32768, 3200, 7);
    TEST_ASSERT_EQUAL_STRING("pulse_underrun", result.reason);
}

void test_crossing_clock_projection_matches_full_clock_during_retargets() {
    drawn_pattern::PhaseClock clock;
    for (const double goal : {.02, .66, .03, .9, .01}) {
        clock.setGoal(goal, .6, 2.);
        for (int i = 0; i < 1000; ++i) {
            for (const double offset : {.0000005, .000625, .001003, .001999, .002}) {
                auto trial = clock;
                TEST_ASSERT_TRUE(trial.advance(offset) == clock.distanceAt(offset));
            }
            clock.advance(.002);
        }
    }
}

void test_deliberate_raw_park_requires_positive_tail_and_verified_empty_anchor() {
    using namespace drawn_pattern;
    PulseQueue q;
    InterruptingQueue driver;
    TEST_ASSERT_EQUAL_INT(int(Park::Parked), int(q.park(driver, -896, false)));
    for (unsigned i = 0; i < pulseQueueFrames; ++i) {
        q.submit<PulseCommand>(driver, -896, 32000, 32768, 3200, 7);
        if (i+1 < pulseQueueFrames)
            TEST_ASSERT_EQUAL_INT(int(Park::Fill), int(q.park(driver, -896, false)));
    }
    TEST_ASSERT_EQUAL_INT(int(Park::Failed), int(q.park(driver, -895, false)));
    TEST_ASSERT_EQUAL_INT(int(Park::Draining), int(q.park(driver, -896, false)));
    driver.running = false;
    driver.count = 0;
    TEST_ASSERT_EQUAL_INT(int(Park::Failed), int(q.park(driver, -896, false)));
    TEST_ASSERT_TRUE(q.owned());
    TEST_ASSERT_EQUAL_INT(int(Park::Parked), int(q.park(driver, -896, true)));
    TEST_ASSERT_FALSE(q.owned());
}

void test_speed_fraction_sets_true_peak_on_classic_rail_independent_of_source_duration() {
    std::string hex;
    constexpr double tau = 6.283185307179586;
    for (unsigned i = 0; i < 16; ++i) {
        char point[5];
        std::snprintf(point, sizeof(point), "%04x", unsigned(std::lround(32767.5*(1-std::cos(tau*i/16)))));
        hex += point;
    }
    Waveform shortSource, longSource;
    TEST_ASSERT_TRUE(shortSource.decode(hex.c_str(), hex.size(), 500, drawn_pattern::SampleFormat::Q16));
    TEST_ASSERT_TRUE(longSource.decode(hex.c_str(), hex.size(), 60000, drawn_pattern::SampleFormat::Q16));
    double observed = 0;
    for (unsigned i = 0; i < 32768; ++i)
        observed = std::max(observed, std::abs(longSource.phaseDerivative(i/32768., 1)));
    for (double fraction : {.00000000001, .01, .25, .5, 1.}) {
        auto plan = longSource.pace(3280, 20000, 500000, fraction, 2);
        TEST_ASSERT_TRUE(plan.rate > 0);
        TEST_ASSERT_EQUAL_DOUBLE(shortSource.pace(3280, 20000, 500000, fraction, 2).rate, plan.rate);
        TEST_ASSERT_EQUAL_STRING("speed", plan.limit);
        TEST_ASSERT_DOUBLE_WITHIN(fraction*.02, fraction*1000, observed*164*plan.rate);
        TEST_ASSERT_TRUE(observed <= longSource.maximumSlope(1) + 1.e-12);
        if (fraction < 1.e-9) TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, plan.periodMs);
    }
    auto stopped = longSource.pace(3280, 20000, 500000, 0, 2);
    TEST_ASSERT_EQUAL_UINT32(0, stopped.periodMs);
    TEST_ASSERT_EQUAL_DOUBLE(0, stopped.rate);
    const auto limited = longSource.pace(120, 20000, 2000, 1., 2);
    TEST_ASSERT_TRUE(limited.peak < 20000);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, limited.peak/4, longSource.pace(120, 20000, 2000, .25, 2).peak);
}

// This smooth 64-knot waveform contains a small higher-frequency component.
// Its slope and acceleration permit 1 m/s. A fixed 100 ms shape-jerk budget
// would stretch it to below 700 mm/s even on the unrestricted classic backend.
void test_speed_scales_authored_jerk_without_accelerating_the_control_ramp() {
    char hex[64 * 4 + 1];
    for (unsigned i = 0; i < 64; ++i) {
        const double angle = i * 2 * std::acos(-1.) / 64;
        const double position = .5 - .45 * std::cos(angle) - .006 * std::cos(7 * angle);
        std::snprintf(hex + 4 * i, 5, "%04x", static_cast<unsigned>(std::lround(65535 * position)));
    }
    Waveform curve;
    TEST_ASSERT_TRUE(curve.decode(hex, 256, 500, drawn_pattern::SampleFormat::Q16));
    constexpr double span = 164, v = 1000, a = 50000, dt = .002;
    const auto full = curve.pace(span, v, a, 1, 2);
    const auto quarter = curve.pace(span, v, a, .25, 2);
    const double oldPeak = curve.maximumSlope(span) * std::cbrt(.7 * a / (.1 * curve.maximumThird(span)));
    TEST_ASSERT_TRUE(oldPeak < 700);
    TEST_ASSERT_DOUBLE_WITHIN(.02, 1000., full.peak);
    TEST_ASSERT_DOUBLE_WITHIN(.02, 250., quarter.peak);
    TEST_ASSERT_DOUBLE_WITHIN(1.e-7, full.shapeJerkLimit, 64 * quarter.shapeJerkLimit);
    TEST_ASSERT_TRUE(curve.maximumCurvature(span) * full.rate * full.rate <= .6 * a);
    TEST_ASSERT_TRUE(curve.maximumThird(span) * std::pow(full.rate, 3) <= full.shapeJerkLimit * (1. + 1.e-12));
    TEST_ASSERT_TRUE(full.fullPathJerkLimit <= a / dt);
    TEST_ASSERT_EQUAL_DOUBLE(full.phaseAcceleration, quarter.phaseAcceleration);
    TEST_ASSERT_EQUAL_DOUBLE(full.phaseJerk, quarter.phaseJerk);
    TEST_ASSERT_EQUAL_DOUBLE(0, curve.pace(span, v, a, 0, 2).shapeJerkLimit);
}

void test_raw_queue_prefills_leading_holds_and_starts_once_with_bounded_lookahead() {
    using namespace drawn_pattern;
    PulseQueue q;
    InterruptingQueue driver;
    for (unsigned i = 0; i < pulseQueueFrames; ++i) {
        const auto result = q.submit<PulseCommand>(driver, -896, 32000, 32768, 3200, 7);
        TEST_ASSERT_EQUAL_INT(int(Frame::Accepted), int(result.state));
        TEST_ASSERT_EQUAL_UINT(i + 1 == pulseQueueFrames, driver.starts);
    }
    TEST_ASSERT_TRUE(driver.running);
    TEST_ASSERT_FALSE(q.filling());
    TEST_ASSERT_EQUAL_INT(int(Frame::Accepted), int(q.submit<PulseCommand>(driver, -895, 32000, 32768, 3200, 7).state));
    TEST_ASSERT_EQUAL_INT(int(Frame::Retry), int(q.submit<PulseCommand>(driver, -894, 32000, 32768, 3200, 7).state));
    TEST_ASSERT_EQUAL_UINT(pulseQueueFrames, driver.count);
    driver.count = 1;
    driver.drainOnAppend = true;
    const auto underrun = q.submit<PulseCommand>(driver, -894, 32000, 32768, 3200, 7);
    TEST_ASSERT_EQUAL_STRING("pulse_underrun", underrun.reason);
    TEST_ASSERT_EQUAL_UINT(1, driver.starts);
    TEST_ASSERT_FALSE(driver.running);
    TEST_ASSERT_EQUAL_INT(int(Frame::Rejected), int(q.submit<PulseCommand>(driver, -894, 32000, 32768, 3200, 7).state));
    TEST_ASSERT_EQUAL_UINT(1, driver.starts);
}

void test_unstarted_raw_queue_preserves_physical_position_and_reports_driver_errors() {
    using namespace drawn_pattern;
    PulseQueue q;
    InterruptingQueue driver;
    driver.discarded = true;
    const auto discarded = q.submit<PulseCommand>(driver, -896, 32000, 32768, 3200, 7);
    TEST_ASSERT_EQUAL_STRING("pulse_discarded", discarded.reason);
    TEST_ASSERT_EQUAL_INT32(-896, q.initialPosition());
    TEST_ASSERT_TRUE(q.filling());
    TEST_ASSERT_FALSE(driver.running);
    q.reset();
    TEST_ASSERT_FALSE(q.owned());
    driver.discarded = false;
    driver.startResult = -2;
    for (unsigned i = 1; i < pulseQueueFrames; ++i)
        TEST_ASSERT_EQUAL_INT(int(Frame::Accepted), int(q.submit<PulseCommand>(driver, -895, 32000, 32768, 3200, 7).state));
    const auto refused = q.submit<PulseCommand>(driver, -894, 32000, 32768, 3200, 7);
    TEST_ASSERT_EQUAL_INT(int(Frame::Rejected), int(refused.state));
    TEST_ASSERT_EQUAL_STRING("pulse_start", refused.reason);
    TEST_ASSERT_EQUAL_INT(-2, refused.driverResult);
    TEST_ASSERT_TRUE(q.filling());
    TEST_ASSERT_FALSE(driver.running);
    TEST_ASSERT_EQUAL_INT32(-896, q.initialPosition());
    TEST_ASSERT_EQUAL_INT32(-895, driver.tail); // Most recent sampled group remains software-only.
}

void test_sparse_wait_groups_preserve_each_real_edge_and_stop_on_a_drained_driver() {
    using namespace drawn_pattern;
    // Actual Q8 2 mm/s capture: native -143 -> -142 at crossing 31440,
    // twelve unchanged source positions, then another step at 43279.
    std::vector<uint32_t> referenceTimes;
    std::vector<int> referenceSigns;
    uint32_t referenceEnd = 0;
    size_t referenceCommands = 0;
    for (bool compact : {false, true}) {
        PulseQueue queue;
        InterruptingQueue driver;
        driver.tail = -143;
        while (!driver.running)
            queue.submit<PulseCommand>(driver, -143, 32000, 32768, 3200, 7);
        driver.retireImmediately = true;
        driver.count = 1;
        driver.journal.clear();
        std::array<int32_t, 40> target;
        std::array<uint16_t, 40> crossing;
        target.fill(-143);
        crossing.fill(32768);
        for (unsigned i = 3; i < target.size(); ++i) target[i] = i < 16 ? -142 : i < 24 ? -141 : -142;
        crossing[3] = 31440;
        crossing[16] = 43279;
        crossing[24] = 28900;
        unsigned pairs = 0;
        for (unsigned i = 0; i < target.size();) {
            unsigned following = 0;
            if (compact && (i == 0 || target[i] == target[i-1])) {
                while (following < 2 && i + following + 1 < target.size() &&
                       target[i + following + 1] == target[i]) ++following;
            }
            const auto count = driver.journal.size();
            auto result = queue.submit<PulseCommand>(driver, target[i], 32000, crossing[i], 3200, 7, following);
            TEST_ASSERT_EQUAL_INT(int(Frame::Accepted), int(result.state));
            if (compact && i == 4) {
                // Real edge and its first unchanged successor within the same motor call.
                TEST_ASSERT_EQUAL_UINT8(3, result.frames);
                TEST_ASSERT_EQUAL_UINT(count + 2, driver.journal.size());
                TEST_ASSERT_EQUAL_UINT8(1, driver.journal[count].steps);
                TEST_ASSERT_EQUAL_UINT16(32648, driver.journal[count].ticks);
                TEST_ASSERT_EQUAL_UINT16(64000, driver.journal[count + 1].ticks);
            }
            pairs += result.frames - 1;
            i += result.frames;
        }
        std::vector<uint32_t> edgeTimes;
        std::vector<int> edgeSigns;
        uint32_t elapsed = 0;
        for (const auto &cmd : driver.journal) {
            if (cmd.steps) { edgeTimes.push_back(elapsed); edgeSigns.push_back(cmd.count_up ? 1 : -1); }
            elapsed += cmd.ticks * std::max<int>(cmd.steps, 1);
        }
        if (compact) {
            TEST_ASSERT_GREATER_THAN_UINT(10, pairs);
            TEST_ASSERT_LESS_THAN_UINT(referenceCommands, driver.journal.size());
            TEST_ASSERT_TRUE(edgeTimes == referenceTimes);
            TEST_ASSERT_TRUE(edgeSigns == referenceSigns);
            TEST_ASSERT_EQUAL_UINT32(referenceEnd, elapsed);
        } else {
            referenceTimes = edgeTimes;
            referenceSigns = edgeSigns;
            referenceEnd = elapsed;
            referenceCommands = driver.journal.size();
        }
        driver.running = false;
        auto dead = queue.submit<PulseCommand>(driver, -142, 32000, 32768, 3200, 7, true);
        TEST_ASSERT_EQUAL_STRING("pulse_underrun", dead.reason);
        TEST_ASSERT_EQUAL_UINT(1, driver.starts);
    }
}

void test_sparse_wait_pair_obeys_available_ticks_and_partial_append_or_revocation() {
    using namespace drawn_pattern;
    for (int mode = 0; mode < 3; ++mode) {
        PulseQueue queue;
        InterruptingQueue driver;
        while (!driver.running)
            queue.submit<PulseCommand>(driver, -896, 32000, 32768, 3200, 7);
        driver.retireImmediately = true;
        driver.count = 2;
        // 12.5 ms exists already: a single 2 ms wait fits, a 4 ms wait cannot.
        driver.queuedTicks = 200000;
        auto single = queue.submit<PulseCommand>(driver, -896, 32000, 32768, 3200, 7, true);
        TEST_ASSERT_EQUAL_UINT8(1, single.frames);
        TEST_ASSERT_EQUAL_UINT16(32000, driver.last.ticks);
        driver.queuedTicks = 0;
        queue.submit<PulseCommand>(driver, -895, 32000, 31440, 3200, 7);
        if (mode == 1) driver.refuseHold = 4;
        const auto old = driver.journal.size();
        auto step = queue.submit<PulseCommand>(driver, -895, 32000, 32768, 3200, 7, true,
                                              [&] { return mode != 2; });
        TEST_ASSERT_EQUAL_UINT32(old + (mode == 0 ? 2 : 1), driver.journal.size());
        if (mode == 2) {
            TEST_ASSERT_EQUAL_INT(int(Frame::Rejected), int(step.state));
            TEST_ASSERT_EQUAL_STRING("guard", step.reason);
        } else {
            TEST_ASSERT_EQUAL_INT(int(Frame::Accepted), int(step.state));
            TEST_ASSERT_EQUAL_UINT8(mode == 0 ? 2 : 1, step.frames);
            TEST_ASSERT_EQUAL_INT8(mode == 0 ? 0 : 4, step.driverResult);
        }
        if (mode == 1) {
            driver.refuseHold = 0;
            const auto retry = queue.submit<PulseCommand>(driver,-895,32000,32768,3200,7);
            TEST_ASSERT_EQUAL_INT(int(Frame::Accepted), int(retry.state));
            TEST_ASSERT_EQUAL_UINT8(0,driver.last.steps);
            TEST_ASSERT_EQUAL_INT(-895,driver.tail);
        }
    }
}

void test_prepared_refill_limits_ownership_and_releases_on_the_first_decline() {
    using namespace drawn_pattern;
    PreparedPulses rows;
    rows.count = rows.capacity;
    for (unsigned i = 0; i < rows.count; ++i) rows.targets[i] = {int32_t(i + 1), uint16_t(5000 + i)};

    // Fully elapsed intervals can admit the entire projection. A waiting
    // Stop or user edit can arrive at any individual call.
    // Only already authorized writes may precede it. There is no spin on
    // a full motor, a stopped clock or the end of the prepared projection.
    for (unsigned permitted = 1; permitted <= rows.count + 1; ++permitted) {
        for (bool decline : {false, true}) {
            PulseBatch burst;
            unsigned writes = 0;
            uint32_t clock = 0;
            burst.submit({0,32768}, rows, [&](PreparedPulses::Target row) -> FrameResult {
                TEST_ASSERT_EQUAL_INT32(int32_t(writes), row.position);
                ++writes;
                clock += 2000; // one elapsed source frame per individual write
                if (decline && writes == permitted) return {Frame::Retry, "pulse_append", 4};
                return {Frame::Accepted, nullptr, 0, FrameTiming::Running, 0, 1698};
            }, [&] { return decline || writes < permitted; }, [&] { return clock; });
            TEST_ASSERT_EQUAL_UINT(permitted, writes);
            TEST_ASSERT_EQUAL_UINT(permitted, burst.count);
            TEST_ASSERT_EQUAL_UINT32(permitted * 2000, burst.attempts[burst.count - 1].atUs);
        }
    }
    unsigned writes = 0;
    PulseBatch highWater;
    highWater.submit({0,32768}, rows, [&](auto) -> FrameResult {
        ++writes;
        return {Frame::Accepted, nullptr, 0, FrameTiming::Running, 0, uint16_t(8000 + 2000 * writes)};
    }, [] { return true; }, [] { return 0; });
    TEST_ASSERT_EQUAL_UINT(3, writes);
}

void test_slow_guarded_burst_replaces_elapsed_rows_before_returning_with_low_reserve() {
    using namespace drawn_pattern;
    // 110432: the last eight electrical groups before ordinal 1998. FAS
    // reported 6578 us pending at the last append. Its next service detected
    // a drained queue 7995 us later. Earlier per-write timing is modeled.
    const unsigned counts[] = {3,2,3,2,2,3,2,3};
    const unsigned ticks[] = {13474,13422,13375,13313,13273,13213,13157,13099};
    PreparedPulses rows;
    rows.count = rows.capacity;
    for (unsigned i = 0; i < rows.count; ++i) rows.targets[i].position = i + 1;
    for (const unsigned futureCost : {0u, 1000u, 1500u, 3000u}) {
        PulseBatch burst;
        uint32_t nowUs = UINT32_MAX - 10000; // include rollover inside the visit
        int pendingTicks = 105250;
        for (unsigned i = 0; i < 8; ++i) pendingTicks += 16 * 2400 - counts[i] * ticks[i];
        unsigned accepted = 0;
        burst.submit({0, 32768}, rows, [&](auto row) -> FrameResult {
            TEST_ASSERT_EQUAL_INT32(accepted, row.position);
            const auto elapsed = accepted < 8 ? 2400u : futureCost;
            nowUs += elapsed;
            pendingTicks -= static_cast<int>(elapsed * 16);
            if (pendingTicks <= 0) return {Frame::Rejected, "pulse_underrun"};
            pendingTicks += accepted < 8 ? counts[accepted] * ticks[accepted] : 32000;
            ++accepted;
            if (accepted == 8) TEST_ASSERT_EQUAL_INT(105250, pendingTicks);
            TEST_ASSERT_LESS_OR_EQUAL_INT(16 * 16000, pendingTicks);
            return {Frame::Accepted, nullptr, 0, FrameTiming::Running, 0, uint16_t(pendingTicks / 16)};
        }, [] { return true; }, [&] { return nowUs; });
        TEST_ASSERT_GREATER_THAN_UINT(8, accepted);
        if (futureCost <= 1500) TEST_ASSERT_GREATER_THAN_INT(7995 * 16, pendingTicks);
        else TEST_ASSERT_EQUAL_INT(int(Frame::Rejected), int(burst.attempts[burst.count-1].result.state));
    }
}

void test_fast_visits_never_spend_future_time_or_pair_across_the_last_credit() {
    using namespace drawn_pattern;
    for (unsigned initial = 1; initial <= PreparedPulses::horizonRows; ++initial) {
        PreparedPulses rows;
        rows.count = rows.capacity;
        rows.initialRows = initial;
        PreparedPulses::Target first{0,32768,0,true};
        for (auto &row : rows.targets) row = first;
        rows.groupWaits(first);
        unsigned accepted = 0;
        PulseBatch batch;
        batch.submit(first, rows, [&](auto target) -> FrameResult {
            const uint8_t n = target.followingHolds ? 2 : 1;
            accepted += n;
            return {Frame::Accepted,nullptr,0,FrameTiming::Running,0,6000,n};
        }, [] { return true; }, [] { return 123456u; });
        TEST_ASSERT_EQUAL_UINT(initial, accepted);
    }
}

void test_ready_wait_pairs_reach_a_moving_successor_once_under_the_same_authority() {
    using namespace drawn_pattern;
    PreparedPulses::Target first{-9, 32768, 0, true};
    PreparedPulses rows{{{-9,32768,0,true}, {-9,32768,0,true}, {-10,21000}, {-10,32768,0,true}}, 4};
    rows.groupWaits(first);
    TEST_ASSERT_EQUAL_UINT8(2, first.followingHolds);
    TEST_ASSERT_EQUAL_UINT8(1, rows.targets[0].followingHolds);
    TEST_ASSERT_EQUAL_UINT8(0, rows.targets[1].followingHolds);
    PulseBatch burst;
    std::vector<int32_t> submitted;
    std::vector<uint16_t> crossings;
    unsigned guardChecks = 0;
    burst.submit(first, rows, [&](PreparedPulses::Target row) -> FrameResult {
        submitted.push_back(row.position);
        crossings.push_back(row.crossing);
        return {Frame::Accepted, nullptr, 0, FrameTiming::Running, 0, 3600,
                uint8_t(row.followingHolds ? 2 : 1)};
    }, [&] { ++guardChecks; return true; }, [] { return 0; });
    TEST_ASSERT_EQUAL_UINT(3, guardChecks);
    TEST_ASSERT_TRUE(submitted == std::vector<int32_t>({-9,-9,-10,-10}));
    TEST_ASSERT_TRUE(crossings == std::vector<uint16_t>({32768,32768,21000,32768}));
}

void test_positive_rate_boundary_is_timed_at_its_own_accepted_write() {
    using namespace drawn_pattern;
    // Eight queued holds may belong to two different Speed settings.
    // In particular the first new row can be the first or the last row.
    static uint32_t clockMs;
    for (unsigned seam = 0; seam <= PreparedPulses::capacity; ++seam) {
        for (unsigned mode = 0; mode < 3; ++mode) {
            PreparedPulses::Target first{-142,32768,0,true};
            PreparedPulses rows;
            rows.count = rows.capacity;
            rows.rateBoundary = seam;
            for (auto &row : rows.targets) row = first;
            rows.groupWaits(first);
            PulseBatch burst;
            unsigned accepted = 0;
            uint32_t expectedTime = 0;
            clockMs = 100;
            // Mode 1 declines the first new row, mode 2 revokes authority
            // after the retained prefix. Neither may publish a speed stamp.
            if (mode != 2 || seam != 0) {
                burst.submit(first, rows, [&](PreparedPulses::Target row) -> FrameResult {
                    clockMs += 2;
                    if (mode == 1 && accepted == seam) return {Frame::Retry, "pulse_append", 4};
                    const uint8_t count = row.followingHolds ? 2 : 1;
                    if (accepted < seam) TEST_ASSERT_LESS_OR_EQUAL_UINT(seam, accepted + count);
                    if (accepted == seam) expectedTime = clockMs;
                    accepted += count;
                    return {Frame::Accepted, nullptr, 0, FrameTiming::Running, 0, 4000, count};
                }, [&] { return mode != 2 || accepted != seam; },
                [] { return clockMs * 1000; }, [] { return clockMs; });
            }
            TEST_ASSERT_EQUAL_UINT(mode == 0 ? rows.count + 1 : seam, accepted);
            uint32_t actualTime = 0;
            for (unsigned i = 0; i < burst.count; ++i) actualTime += burst.attempts[i].boundaryAtMs;
            TEST_ASSERT_EQUAL_UINT32(expectedTime, actualTime);
            TEST_ASSERT_EQUAL(mode == 0, expectedTime != 0);
        }
    }
}

void test_prefill_ends_on_start_and_each_gain_change_keeps_its_timestamp() {
    using namespace drawn_pattern;
    static uint32_t clockMs;
    for (const bool starting : {false, true}) {
        PreparedPulses::Target first{7,32768,0,true};
        PreparedPulses rows;
        rows.count = rows.horizonRows - 1;
        rows.rateBoundary = 3;
        rows.gainBoundaries = (1u << 1) | (1u << 5);
        for (auto &row : rows.targets) row = first;
        rows.groupWaits(first);
        unsigned ordinal = 0;
        PulseBatch batch;
        clockMs = 0;
        batch.submit(first, rows, [&](auto row) -> FrameResult {
            ++clockMs;
            const uint8_t count = row.followingHolds ? 2 : 1;
            if (count > 1) TEST_ASSERT_FALSE(rows.boundaryAt(ordinal + 1));
            ordinal += count;
            const auto timing = !starting ? FrameTiming::Running :
                                ordinal < 5 ? FrameTiming::Prefill : FrameTiming::Started;
            return {Frame::Accepted,nullptr,0,timing,0,6000,count};
        }, [] { return true; }, [] { return clockMs * 1000; }, [] { return clockMs; });
        TEST_ASSERT_EQUAL_UINT(starting ? 5 : 8, ordinal);
        unsigned n = 0;
        for (unsigned i = 0; i < batch.count; ++i) {
            const auto &a = batch.attempts[i];
            TEST_ASSERT_EQUAL_UINT32(rows.boundaryAt(n) ? i+1 : 0, a.boundaryAtMs);
            n += a.result.frames;
        }
    }
}

void test_motor_revocation_ends_a_retry_immediately_and_retains_an_accepted_prefix() {
    using namespace drawn_pattern;
    PulseBatch batch;
    batch.count = 1;
    batch.attempts[0] = {{Frame::Retry, "pulse_full"}, 100};
    batch.revoke(105);
    TEST_ASSERT_TRUE(batch.revoked);
    TEST_ASSERT_EQUAL_INT(int(Frame::Rejected), int(batch.attempts[0].result.state));
    TEST_ASSERT_EQUAL_STRING("guard", batch.attempts[0].result.reason);
    TEST_ASSERT_EQUAL_UINT32(105, batch.attempts[0].atUs);

    PulseBatch prefix;
    prefix.count = 2;
    prefix.attempts[0] = {{Frame::Accepted, nullptr, 0, FrameTiming::Running, 0, 1698}, 100};
    prefix.attempts[1] = {{Frame::Retry, "pulse_append", 4}, 103};
    prefix.revoke(105);
    TEST_ASSERT_TRUE(prefix.revoked);
    TEST_ASSERT_EQUAL_UINT(2, prefix.count);
    TEST_ASSERT_EQUAL_INT(int(Frame::Accepted), int(prefix.attempts[0].result.state));
    TEST_ASSERT_EQUAL_INT8(4, prefix.attempts[1].result.driverResult);
}

void test_prepared_pulse_burst_preserves_real_moving_edges_and_partial_acceptance() {
    using namespace drawn_pattern;
    // Captured 30.60 mm Q8 brake: accepted row 729 generated a 27,180-tick
    // wait, followed by a negative STEP and another wait in rows 730/731.
    const PreparedPulses::Target prefix[] = {
        {-96,32768}, {-97,27664}, {-97,32768}, {-98,24976}, {-98,32768}
    };
    PreparedPulses next{{{-99,32768}, {-100,21424}}, 2};
    for (unsigned mode = 0; mode < 5; ++mode) {
        PulseQueue queue;
        InterruptingQueue driver;
        driver.tail = -96;
        while (!driver.running) queue.submit<PulseCommand>(driver, -96, 32000, 32768, 3200, 7);
        driver.retireImmediately = true;
        driver.count = 1;
        auto write = [&](PreparedPulses::Target row) {
            return queue.submit<PulseCommand>(driver, row.position, 32000, row.crossing, 3200, 7);
        };
        for (auto row : prefix) TEST_ASSERT_EQUAL_INT(int(Frame::Accepted), int(write(row).state));
        TEST_ASSERT_EQUAL_UINT16(35805, driver.last.ticks);
        const auto prior = driver.journal.size();
        uint32_t receipt = 1000;
        PulseBatch batch;
        unsigned checks = 0;
        batch.submit({-99,22896}, next, write, [&] {
            ++checks;
            if (mode == 1) return false; // pending control, owner lease or wired edit
            if (mode == 2 && checks == 2) driver.refuseHold = 4;
            if (mode == 3) driver.running = false;
            if (mode == 4) { driver.count = 2; driver.queuedTicks = 15 * 16000; }
            return true;
        }, [&] { return ++receipt; });
        const unsigned accepted = mode == 0 ? 3 : mode == 2 ? 2 : 1;
        TEST_ASSERT_EQUAL_UINT(prior + accepted, driver.journal.size());
        TEST_ASSERT_EQUAL_UINT16(27180, driver.journal[prior].ticks);
        if (accepted > 1) {
            TEST_ASSERT_EQUAL_UINT16(36820, driver.journal[prior + 1].ticks);
            TEST_ASSERT_EQUAL_UINT8(1, driver.journal[prior + 1].steps);
            TEST_ASSERT_FALSE(driver.journal[prior + 1].count_up);
        }
        if (accepted > 2) {
            TEST_ASSERT_EQUAL_UINT16(26461, driver.journal[prior + 2].ticks);
            TEST_ASSERT_EQUAL_UINT8(0, driver.journal[prior + 2].steps);
            // Even with its executing pulse about to expire, both additional
            // commands cover the measured 4,989 us between feeder visits.
            TEST_ASSERT_GREATER_THAN_UINT32(4989 * 16, 27180 + 36820 + 26461);
        }
        const auto &last = batch.attempts[batch.count - 1].result;
        if (mode == 2) {
            TEST_ASSERT_EQUAL_INT(int(Frame::Retry), int(last.state));
            TEST_ASSERT_EQUAL_INT(4, last.driverResult);
            driver.refuseHold = 0;
            TEST_ASSERT_EQUAL_INT(int(Frame::Accepted), int(write(next.targets[1]).state));
            TEST_ASSERT_EQUAL_UINT8(0, driver.last.steps);
        } else if (mode == 3) {
            TEST_ASSERT_EQUAL_INT(int(Frame::Rejected), int(last.state));
            TEST_ASSERT_EQUAL_STRING("pulse_underrun", last.reason);
            TEST_ASSERT_EQUAL_UINT(1, driver.starts);
        } else if (mode == 4) {
            TEST_ASSERT_EQUAL_INT(int(Frame::Retry), int(last.state));
            TEST_ASSERT_EQUAL_STRING("pulse_full", last.reason);
        } else {
            TEST_ASSERT_EQUAL_INT(int(Frame::Accepted), int(last.state));
        }
    }
}

void test_timed_edges_follow_subframe_crossings_in_both_directions_and_floor_ticks_do_not_accumulate() {
    using namespace drawn_pattern;
    constexpr int frameTicks=32000;
    for (const int sign : {-1,1}) {
        InterruptingQueue driver;
        driver.tail=0;
        driver.retireImmediately=true;
        PulseQueue q;
        // Twenty-three hundred steps/s: 4.6 steps per frame, with crossings
        // between period boundaries. The first 2ms is a true stationary hold.
        const double speed=4.6;
        uint64_t elapsed=0;
        double maxError=0;
        int pulses=0;
        for (unsigned i=0;i<405;++i) {
            const auto position=[&](double u) {
                return sign*static_cast<int32_t>(std::lround(speed*std::clamp(double(i)+u-1.,0.,400.)));
            };
            const int32_t from=position(0), next=position(1);
            const uint16_t crossing=firstStepCrossing(from,next,position);
            const auto f=q.submit<PulseCommand>(driver,next,frameTicks,crossing,3200,7);
            TEST_ASSERT_EQUAL_INT(int(Frame::Accepted),int(f.state));
            const auto cmd=driver.last;
            for (unsigned k=0;k<cmd.steps;++k) {
                const double measured=elapsed+k*cmd.ticks;
                const double ideal=frameTicks*(1+(pulses+.5)/speed);
                // The final group is followed by a hold; its inner pulses
                // may have unequal physical and analytical intervals.
                if (pulses<1836) maxError=std::max(maxError,std::abs(measured-ideal));
                TEST_ASSERT_EQUAL(sign,cmd.count_up?1:-1);
                ++pulses;
            }
            if (cmd.steps==0 && pulses>0) TEST_ASSERT_EQUAL(sign,cmd.count_up?1:-1);
            elapsed+=uint32_t(cmd.ticks)*std::max(1,int(cmd.steps));
        }
        TEST_ASSERT_EQUAL_UINT(1840,pulses);
        TEST_ASSERT_EQUAL_INT32(sign*1840,driver.tail);
        TEST_ASSERT_GREATER_THAN_UINT(300,driver.moveTimedCalls);
        TEST_ASSERT_LESS_THAN_FLOAT(30.,maxError); // <2 microseconds including integer ticks
    }
}

void test_crossing_prediction_only_changes_how_the_original_grid_is_searched() {
    using namespace drawn_pattern;
    for (unsigned boundary = 1; boundary <= crossingGrid; ++boundary) {
        const unsigned expected = boundary - 1;
        const auto beforeEdge = [boundary](unsigned index) { return index < boundary; };
        for (unsigned guess : {0u, expected, expected / 2, crossingGrid - 1,
                               std::min(crossingGrid - 1, expected + 8)}) {
            TEST_ASSERT_EQUAL_UINT32(expected, predictedStepCrossing(guess, beforeEdge));
        }
        unsigned calls = 0;
        const auto low = predictedStepCrossing(expected, [&](unsigned index) {
            ++calls;
            return beforeEdge(index);
        });
        TEST_ASSERT_EQUAL_UINT32(expected, low);
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(2, calls);
        TEST_ASSERT_EQUAL_UINT16(((2 * expected + 1) * 65535u + 2048u) / 4096u,
                                 encodeStepCrossing(low));
    }
}

void test_plain_step_crossing_preserves_launch_changes_holds_reversals_and_seams() {
    using namespace drawn_pattern;
    Waveform curve;
    const auto hex = encode(asymmetric);
    TEST_ASSERT_TRUE(curve.decode(hex.data(), hex.size(), 2000));
    constexpr double dt = .002;
    constexpr int shallow = -71, deep = -3351;
    unsigned moving = 0, stationary = 0, dense = 0, seams = 0;
    PhaseClock clock;
    double phase = .99;
    for (double goal : {.006, 1.9, 0., 5.4, .024, 2.7}) {
        clock.setGoal(goal, 13., 91.);
        for (unsigned i = 0; i < 1700; ++i) {
            const auto at = [&](double f) {
                return curve.nativeTarget(phase + clock.distanceAt(dt * f), deep, shallow);
            };
            const int32_t from = at(0), to = at(1);
            TEST_ASSERT_EQUAL_UINT16(firstStepCrossing(from, to, at),
                                     curve.firstPlainStepCrossing(from, to, phase, clock, dt, deep, shallow));
            if (from != to) ++moving;
            else ++stationary;
            if (std::abs(to - from) > 2) ++dense;
            phase += clock.advance(dt);
            if (phase >= 1) {
                ++seams;
                phase -= std::floor(phase);
            }
        }
    }
    TEST_ASSERT_GREATER_THAN_UINT32(1000, moving);
    TEST_ASSERT_GREATER_THAN_UINT32(500, dense);
    TEST_ASSERT_GREATER_THAN_UINT32(500, stationary);
    TEST_ASSERT_GREATER_THAN_UINT32(10, seams);

    // The full frame crosses several reversals. A different bisection order
    // could choose a different crossing, so retain the reference order here.
    clock.setGoal(1., 10., 100.);
    clock.advance(clock.secondsToGoal() + 1.);
    phase = .90;
    const auto acrossSeam = [&](double fraction) {
        return curve.nativeTarget(phase + clock.distanceAt(.8 * fraction), deep, shallow);
    };
    const int32_t start = acrossSeam(0), end = acrossSeam(1);
    TEST_ASSERT_EQUAL_UINT16(firstStepCrossing(start, end, acrossSeam),
                             curve.firstPlainStepCrossing(start, end, phase, clock, .8, deep, shallow));

    // The arbitrary-position search is also used for active vibration. A
    // moving carrier can first cross in the opposite direction to the net
    // frame displacement. Lock down the reference binary decision order.
    const auto carrierTurn = [](double fraction) {
        return int32_t(std::lround(fraction - 8 * fraction * (1 - fraction)));
    };
    const unsigned firstChanged = unsigned(std::ceil(crossingGrid * (7. - std::sqrt(33.)) / 16.));
    TEST_ASSERT_EQUAL_UINT16(encodeStepCrossing(firstChanged - 1),
                             firstStepCrossing(carrierTurn(0), carrierTurn(1), carrierTurn));
}

void test_periodic_quintic_preserves_acceleration_holds_and_bounds_the_three_derivatives() {
    Waveform curve;
    const auto hex = encode(asymmetric);
    TEST_ASSERT_TRUE(curve.decode(hex.data(),hex.size(),500));
    const double epsilon = 1.e-8;
    for (unsigned k=0;k<16;++k) {
        const double f=k/16.;
        const double jump=std::abs(curve.phaseDerivative(f-epsilon,2)-curve.phaseDerivative(f+epsilon,2));
        TEST_ASSERT_TRUE(jump <= 2.01*epsilon*curve.maximumThird(1) + 1.e-7);
        if (asymmetric[k]==asymmetric[(k+1)%16]) {
            for (unsigned order=1;order<=3;++order)
                TEST_ASSERT_FLOAT_WITHIN(1.e-7,0,curve.phaseDerivative(f+1./32,order));
        }
    }
    for (unsigned i=0;i<8192;++i) {
        const double f=i/8192.;
        TEST_ASSERT_TRUE(std::abs(curve.phaseDerivative(f,1)) <= curve.maximumSlope(1)+1.e-8);
        TEST_ASSERT_TRUE(std::abs(curve.phaseDerivative(f,2)) <= curve.maximumCurvature(1)+1.e-8);
        TEST_ASSERT_TRUE(std::abs(curve.phaseDerivative(f,3)) <= curve.maximumThird(1)+1.e-7);
    }
}

void test_high_precision_sine_tracks_the_authored_third_derivative() {
    constexpr unsigned n=32;
    constexpr double tau=6.283185307179586;
    char q8[2*n+1]{},q16[4*n+1]{};
    for (unsigned i=0;i<n;++i) {
        const double y=(1-std::cos(tau*i/n))*.5;
        std::snprintf(q8+2*i,3,"%02x",static_cast<unsigned>(std::lround(y*255)));
        std::snprintf(q16+4*i,5,"%04x",static_cast<unsigned>(std::lround(y*65535)));
    }
    Waveform fine,coarse;
    TEST_ASSERT_TRUE(fine.decode(q16,4*n,4000,drawn_pattern::SampleFormat::Q16));
    TEST_ASSERT_TRUE(coarse.decode(q8,2*n,4000));
    TEST_ASSERT_FALSE(coarse.decode(q8,2*n-2,4000,drawn_pattern::SampleFormat::Q16));
    double errFine=0,errCoarse=0;
    for (unsigned i=0;i<8192;++i) {
        const double f=(i+.5)/8192;
        const double authored=-.5*tau*tau*tau*std::sin(tau*f);
        errFine+=std::pow(fine.phaseDerivative(f,3)-authored,2);
        errCoarse+=std::pow(coarse.phaseDerivative(f,3)-authored,2);
    }
    TEST_ASSERT_LESS_THAN_FLOAT(8.,std::sqrt(errFine/8192));
    TEST_ASSERT_TRUE(errFine < .01*errCoarse);
}

void test_retargetable_rate_uses_continuous_acceleration_and_bounded_jerk() {
    drawn_pattern::PhaseClock timer;
    constexpr double dt=.002, aLimit=1.25, jLimit=4.5;
    double phase=0;
    for (unsigned i=0;i<5000;++i) {
        const double goal=i<80?.8:i<110?.1:i<160?.9:i<210?.02:i<310?.85:.4;
        const double beforeRate=timer.rate(),beforeAcceleration=timer.acceleration();
        timer.setGoal(goal,aLimit,jLimit);
        TEST_ASSERT_FLOAT_WITHIN(1.e-12,beforeAcceleration,timer.acceleration());
        const double step=timer.advance(dt);phase+=step;
        TEST_ASSERT_TRUE(step>=0 && step<=.9*dt+1.e-12);
        TEST_ASSERT_TRUE(timer.rate()>=0 && timer.rate()<=.9+1.e-12);
        TEST_ASSERT_TRUE(std::abs(timer.acceleration())<=aLimit+1.e-10);
        TEST_ASSERT_TRUE(std::abs(timer.acceleration()-beforeAcceleration)<=jLimit*dt+1.e-10);
        TEST_ASSERT_TRUE(std::abs(timer.rate()-beforeRate)<=aLimit*dt+1.e-10);
    }
    TEST_ASSERT_FLOAT_WITHIN(1.e-9,.4,timer.rate());
    TEST_ASSERT_FLOAT_WITHIN(1.e-9,0,timer.acceleration());
    TEST_ASSERT_GREATER_THAN_FLOAT(3.,phase);
    // A rate of one is reached from rest after two seconds, with one cycle
    // elapsed. The triangular acceleration has slope +1 then -1.
    drawn_pattern::PhaseClock triangle;
    triangle.setGoal(1,1,1);
    TEST_ASSERT_FLOAT_WITHIN(1.e-10,1./6,triangle.advance(1));
    TEST_ASSERT_FLOAT_WITHIN(1.e-10,.5,triangle.rate());
    TEST_ASSERT_FLOAT_WITHIN(1.e-10,5./6,triangle.advance(1));
    TEST_ASSERT_FLOAT_WITHIN(1.e-10,1,triangle.rate());
    TEST_ASSERT_FLOAT_WITHIN(1.e-10,0,triangle.acceleration());
}

void test_time_domain_derivatives_remain_bounded_during_repeated_live_retargets() {
    Waveform curve;
    const auto hex = encode(asymmetric);
    TEST_ASSERT_TRUE(curve.decode(hex.data(),hex.size(),500));
    drawn_pattern::PhaseClock clock;
    constexpr double span=1800, acceleration=100000, dt=.0005;
    const auto limits=curve.pace(span,20000,acceleration,1.,2);
    double phase=0, previousAcceleration=0, previousPhaseAcceleration=0;
    for (unsigned frame=0;frame<60000;++frame) {
        constexpr double speeds[] = {650., 20000., 1400., 0., 20000.};
        const double speed = speeds[(frame/3100) % 5];
        clock.setGoal(curve.pace(span,20000,acceleration,speed/20000.,2).rate,
                      limits.phaseAcceleration,limits.phaseJerk);
        phase+=clock.advance(dt);
        const double r=clock.rate(), a=clock.acceleration();
        const double positionAcceleration=span*(curve.phaseDerivative(phase,2)*r*r+
                                               curve.phaseDerivative(phase,1)*a);
        TEST_ASSERT_TRUE(std::abs(positionAcceleration)<=acceleration);
        TEST_ASSERT_TRUE(std::abs(positionAcceleration-previousAcceleration)<=
                         dt*limits.fullPathJerkLimit+1e-7);
        // Separate the source's own cubic-in-rate jerk from the two terms
        // introduced by changing Speed. The latter stay comfort-bounded
        // even at the start of a rapid downshift from the full phase rate.
        const double beta = (a - previousPhaseAcceleration) / dt;
        const double controlJerk = span * (3 * curve.phaseDerivative(phase, 2) * r * a +
                                                 curve.phaseDerivative(phase, 1) * beta);
        TEST_ASSERT_TRUE(std::abs(controlJerk) <= .3 * acceleration / .100 + 1.e-6);
        const double fraction = r / limits.rate;
        const double shapeLimit = limits.shapeJerkLimit * fraction * fraction * fraction;
        const double sourceJerk = span * curve.phaseDerivative(phase, 3) * r * r * r;
        TEST_ASSERT_TRUE(std::abs(sourceJerk) <= shapeLimit + 1.e-7);
        previousPhaseAcceleration = a;
        previousAcceleration=positionAcceleration;
    }
}

void test_hold_lands_once_per_cycle_at_the_original_phase_and_scales_stationary_time() {
    using namespace drawn_pattern;
    Modifiers m;
    m.count = 1;
    m.entries[0] = {400, 400, 333};
    MarkerIndex index;
    index.reset(m, .25);
    const PhaseLimits limits{1.2, 4., 21.};
    for (double level : {0., .5, 1.}) {
        MarkedPhase motion;
        PhaseClock plain;
        double plainPhase = 0;
        std::vector<unsigned> holds;
        bool onPoint = false;
        double lastRate = 0, lastAccel = 0;
        for (unsigned frame = 0; frame < 11000; ++frame) {
            const auto previous = motion.phase;
            motion.advance(.001, limits, index, level);
            plain.setGoal(limits.rate, limits.acceleration, limits.jerk);
            plainPhase += plain.advance(.001);
            plainPhase -= std::floor(plainPhase);
            if (level == 0) TEST_ASSERT_EQUAL_DOUBLE(plainPhase, motion.phase);
            double increment = motion.phase - previous;
            if (increment < -.5) increment += 1.;
            TEST_ASSERT_TRUE(increment >= -1e-8 && increment <= limits.rate * .001 + 1e-8);
            const double accel = motion.clock.acceleration();
            TEST_ASSERT_TRUE(std::abs(accel) <= 4.00001 && std::abs(accel - lastAccel) <= .0210001);
            TEST_ASSERT_TRUE(std::abs(motion.clock.rate() - lastRate) <= .0040001);
            const bool parked = motion.clock.rate() == 0 && std::abs(motion.phase - .15) < 1e-8;
            if (parked && !onPoint) holds.push_back(0);
            if (parked) ++holds.back();
            onPoint = parked;
            lastRate = motion.clock.rate();
            lastAccel = accel;
        }
        if (!level) {
            TEST_ASSERT_TRUE(holds.empty());
            continue;
        }
        TEST_ASSERT_GREATER_OR_EQUAL_UINT32(4, holds.size());
        for (size_t i = 0; i + 1 < holds.size(); ++i) TEST_ASSERT_DOUBLE_WITHIN(1., 333. * level, holds[i]);
    }
}

void test_close_points_start_at_seam_and_late_activation_does_not_snap() {
    using namespace drawn_pattern;
    Modifiers list;
    list.count = 3;
    list.entries[0] = {0, 0, 25};
    list.entries[1] = {1, 1, 33};
    list.entries[2] = {999, 999, 22};
    MarkerIndex index;
    index.reset(list, 0);
    MarkedPhase m;
    PhaseLimits limits{1., 2., 12.};
    m.advance(.020, limits, index, 1);
    TEST_ASSERT_EQUAL_DOUBLE(0., m.phase);
    m.advance(.020, limits, index, 1);
    TEST_ASSERT_GREATER_THAN_DOUBLE(0., m.phase);
    for (unsigned k = 0; k < 1800; ++k) m.advance(.001, limits, index, 0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-8, 1., m.clock.rate());
    unsigned acquired = 0;
    for (unsigned k = 0; k < 9000; ++k) {
        double old = m.phase;
        m.advance(.002, limits, index, 1);
        double d = m.phase - old;
        if (d < -.5) d += 1;
        TEST_ASSERT_TRUE(d >= -1e-8 && d <= .0020001);
        if (m.clock.rate() == 0 && std::abs(m.phase - .999) < 1e-8) ++acquired;
    }
    TEST_ASSERT_GREATER_THAN_UINT32(20, acquired);
}

void test_rumble_wall_clock_linearity_region_bias_limits_and_live_derivatives() {
    using namespace drawn_pattern;
    Modifiers m;
    m.count = 1;
    m.entries[0] = {250, 750, 0};
    MarkerIndex index;
    Waveform flat;
    TEST_ASSERT_TRUE(flat.decode("80808080808080808080808080808080", 32, 2000));
    TEST_ASSERT_TRUE(index.reset(m, 0, &flat));
    VibrationBudget budget{2000, 8000, 200000, 20, {.2, .4, 1.6}, 2000000, 8000};
    VibrationClock half, full;
    half.advance(1, .5);
    full.advance(1, 1);
    const double amplitude = budget.amplitude(.5, {});
    TEST_ASSERT_GREATER_THAN_DOUBLE(1., amplitude);
    for (unsigned k = 0; k < 40; ++k) {
        for (double y : {0., .5, 1.}) {
            const double a = full.offset(y, .5, index, budget), b = half.offset(y, .5, index, budget);
            TEST_ASSERT_DOUBLE_WITHIN(1e-9, a / 2, b);
            TEST_ASSERT_DOUBLE_WITHIN(
                1e-9, amplitude * (std::sin(2 * 3.14159265358979323846 * 6 * k * .005) + 1 - 2 * y), a);
            TEST_ASSERT_TRUE(y * 2000 + a >= -1e-10 && y * 2000 + a <= 2000 + 1e-10);
            TEST_ASSERT_EQUAL_DOUBLE(0, full.offset(y, .1, index, budget));
            TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0, full.offset(y, .25, index, budget));
        }
        half.advance(.005, .5);
        full.advance(.005, 1);
    }
    // Finite differences on a smooth advancing base cover both envelope edges
    // and an in-flight live gain change. Use a much finer clock than the ESP.
    constexpr double dt = .0002;
    VibrationClock v;
    double lastX = 0, lastV = 0, lastA = 0, maxV = 0, maxA = 0, maxJ = 0;
    for (unsigned k = 0; k < 42000; ++k) {
        double t = dt * k, p = t * .2;
        double y = .5 + .25 * std::sin(6.283185307179586 * p);
        const double x = v.offset(y, p, index, budget);
        const double speed = (x - lastX) / dt, accel = (speed - lastV) / dt, jerk = (accel - lastA) / dt;
        if (k > 3) {
            maxV = std::max(maxV, std::abs(speed));
            maxA = std::max(maxA, std::abs(accel));
            maxJ = std::max(maxJ, std::abs(jerk));
        }
        lastX = x;
        lastV = speed;
        lastA = accel;
        v.advance(dt, t < 2 ? 1 : t < 3.1 ? .5 : t < 4 ? 0 : 1);
    }
    TEST_ASSERT_GREATER_THAN_DOUBLE(20., maxV);
    TEST_ASSERT_TRUE(maxV <= .25 * budget.speed && maxA <= .25 * budget.acceleration &&
                     maxJ <= 2.5 * budget.acceleration);
    TEST_ASSERT_TRUE(budget.amplitude(.001, {}) < budget.amplitude(.1, {}));
}

void test_original_phase_across_q8_q16_and_sparse_inputs() {
    using namespace drawn_pattern;
    const char *wire[] = {"80808080000000008080808080808080",
                          "8080808080808080000000000000000080808080808080808080808080808080",
                          "00ff808001ff000003ff8080"};
    const SampleFormat formats[] = {SampleFormat::Q8, SampleFormat::Q16, SampleFormat::RunsQ16};
    Modifiers items;
    items.count = 2;
    items.entries[0] = {400, 400, 40};
    items.entries[1] = {600, 900, 0};
    for (unsigned i = 0; i < 3; ++i) {
        Waveform curve;
        TEST_ASSERT_TRUE(curve.decode(wire[i], std::char_traits<char>::length(wire[i]), 2000, formats[i]));
        TEST_ASSERT_EQUAL_DOUBLE(.25, curve.authoredStart());
        MarkerIndex index;
        index.reset(items, curve.authoredStart());
        TEST_ASSERT_FALSE(index.range(.2));
        TEST_ASSERT_TRUE(index.range(.45) == &items.entries[1]);
        uint16_t dwell = 0;
        TEST_ASSERT_DOUBLE_WITHIN(1e-12, .15, index.upcoming(0, dwell));
        TEST_ASSERT_EQUAL_UINT16(40, dwell);
        TEST_ASSERT_DOUBLE_WITHIN(1e-12, 1.15, index.upcoming(.151, dwell));
    }
}

void test_ms_hold_and_queue_fraction_keep_the_same_park_across_serial_and_step_durations() {
    using namespace drawn_pattern;
    Modifiers items;
    items.count = 1;
    items.entries[0] = {270, 270, 1};
    MarkerIndex index;
    index.reset(items, 0);
    const PhaseLimits limits{.4, 1.2, 8.0};
    MarkedPhase whole, halves;
    for (unsigned i = 0; i < 12000; ++i) {
        const double dt = i % 2 ? .020 : .002;
        whole.advance(dt, limits, index, .5);
        halves.advance(dt * .37, limits, index, .5);
        halves.advance(dt * .63, limits, index, .5);
        double difference = whole.phase - halves.phase;
        difference -= std::round(difference);
        TEST_ASSERT_DOUBLE_WITHIN(.00000007, 0., difference);
        TEST_ASSERT_DOUBLE_WITHIN(.000005, whole.clock.rate(), halves.clock.rate());
    }
}

void test_live_sensation_changes_the_duration_of_the_current_pause() {
    using namespace drawn_pattern;
    Modifiers m;
    m.count = 1;
    m.entries[0] = {0, 0, 1000};
    MarkerIndex index;
    index.reset(m, 0);
    const PhaseLimits limits{.5, 2, 10};
    MarkedPhase motion;
    motion.advance(.3, limits, index, .5);
    TEST_ASSERT_EQUAL_DOUBLE(0, motion.phase);
    motion.advance(.4, limits, index, 1);
    TEST_ASSERT_EQUAL_DOUBLE(0, motion.phase);
    motion.advance(.001, limits, index, .25);
    TEST_ASSERT_GREATER_THAN_DOUBLE(0, motion.phase);
    TEST_ASSERT_GREATER_THAN_DOUBLE(0, motion.clock.rate());
    // The restarted half-cycle cannot reacquire the same point.
    motion.advance(.3, limits, index, 1);
    TEST_ASSERT_GREATER_THAN_DOUBLE(.01, motion.phase);
    motion = {};
    motion.advance(.1, limits, index, 1);
    motion.advance(.002, limits, index, 0);
    TEST_ASSERT_GREATER_THAN_DOUBLE(0, motion.phase);
}

void test_long_stationary_intervals_use_real_time_at_full_half_and_live_strength() {
    using namespace drawn_pattern;
    Modifiers items;
    items.count = 1;
    MarkerIndex index;
    const PhaseLimits limits{.5, 2, 10};
    for (uint16_t maximum : {30000, 60000}) {
        items.entries[0] = Modifier(InstantEffect{InstantEffect::Kind::Pause, 0, maximum});
        TEST_ASSERT_TRUE(index.reset(items, 0));
        for (double strength : {.5, 1.}) {
            for (unsigned frameMs : {2, 20}) {
                MarkedPhase motion;
                const unsigned duration = unsigned(maximum * strength);
                // The stationary interval begins on entry at phase zero.
                for (unsigned ms = 0; ms < duration; ms += frameMs)
                    motion.advance(frameMs * .001, limits, index, strength);
                TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0, motion.phase);
                motion.advance(.001, limits, index, strength);
                TEST_ASSERT_GREATER_THAN_DOUBLE(0, motion.phase);
            }
        }
    }
    items.entries[0] = {0, 0, 30000};
    MarkedPhase motion;
    motion.advance(12., limits, index, .5);
    motion.advance(8., limits, index, 1.);
    TEST_ASSERT_EQUAL_DOUBLE(0, motion.phase);
    motion.advance(.002, limits, index, .25);
    TEST_ASSERT_GREATER_THAN_DOUBLE(0, motion.phase);
    motion = {};
    motion.advance(.002, limits, index, 0.);
    TEST_ASSERT_GREATER_THAN_DOUBLE(0, motion.phase);
}

void test_compact_pause_grid_matches_every_authored_boundary_and_wrap() {
    using namespace drawn_pattern;
    Modifiers marks;
    TEST_ASSERT_TRUE(marks.reserve(Modifiers::capacity));
    for (uint16_t i = 0; i < Modifiers::capacity; ++i)
        marks.entries[marks.count++] = {i, i, static_cast<uint16_t>(1 + (i * 97u) % 60000u)};
    marks.entries[0] = {0, 0, 60000};
    marks.entries[999] = {999, 999, 32768};
    for (double offset : {0., .3125, .999}) {
        MarkerIndex compact, sparse;
        TEST_ASSERT_TRUE(sparse.reset(marks, offset));
        TEST_ASSERT_TRUE(compact.tryCompactFullPauseGrid(marks, offset));
        for (int i = -1000; i <= 2000; ++i) {
            for (double d : {-1e-12, -1e-14, 0., 1e-14, 1e-12, .000499}) {
                const double phase = i / 1000.0 + d - offset;
                uint16_t originalDuration, compactDuration;
                TEST_ASSERT_EQUAL_DOUBLE(sparse.upcoming(phase, originalDuration),
                                         compact.upcoming(phase, compactDuration));
                TEST_ASSERT_EQUAL_UINT16(originalDuration, compactDuration);
            }
        }
        TEST_ASSERT_NULL(compact.range(.25));
    }
    --marks.count;
    MarkerIndex incomplete;
    TEST_ASSERT_FALSE(incomplete.tryCompactFullPauseGrid(marks, 0));
}

void test_cached_pause_horizon_matches_exact_braking_on_both_frame_clocks() {
    using namespace drawn_pattern;
    Modifiers items;
    items.count = 3;
    items.entries[0] = {125, 125, 37};
    items.entries[1] = {201, 625, 0};
    items.entries[2] = {875, 875, 800};
    MarkerIndex index;
    TEST_ASSERT_TRUE(index.reset(items, .0625));
    for (const double dt : {.002, .020}) {
        MarkedPhase exact, fast;
        PauseBrakeBounds bounds;
        PhaseLimits limits{.0833, .1666, .6664};
        double strength = .5;
        bounds.reset(limits);
        for (unsigned frame = 0; frame < 20000; ++frame) {
            if (frame % 431 == 240) {
                const double rate = (.2 + frame % 8) / 12;
                limits = {rate, 2 * rate, 8 * rate};
                bounds.reset(limits);
            }
            if (frame % 517 == 117) strength = (frame % 5) * .25;
            FrameBrake reference, optimized;
            optimized.bounds = &bounds;
            exact.advance(dt, limits, index, strength, &reference);
            fast.advance(dt, limits, index, strength, &optimized);
            TEST_ASSERT_EQUAL_DOUBLE(exact.phase, fast.phase);
            TEST_ASSERT_EQUAL_DOUBLE(exact.clock.rate(), fast.clock.rate());
            TEST_ASSERT_EQUAL_DOUBLE(exact.clock.acceleration(), fast.clock.acceleration());
        }
    }
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_crossing_prediction_only_changes_how_the_original_grid_is_searched);
    RUN_TEST(test_plain_step_crossing_preserves_launch_changes_holds_reversals_and_seams);
    RUN_TEST(test_raw_watermark_recovers_between_measured_size_service_gaps);
    RUN_TEST(test_speed_fraction_sets_true_peak_on_classic_rail_independent_of_source_duration);
    RUN_TEST(test_speed_scales_authored_jerk_without_accelerating_the_control_ramp);
    RUN_TEST(test_deliberate_raw_park_requires_positive_tail_and_verified_empty_anchor);
    RUN_TEST(test_stationary_entry_requires_untouched_empty_driver_and_matching_position);
    RUN_TEST(test_crossing_clock_projection_matches_full_clock_during_retargets);
    RUN_TEST(test_compact_pause_grid_matches_every_authored_boundary_and_wrap);
    RUN_TEST(test_cached_pause_horizon_matches_exact_braking_on_both_frame_clocks);
    RUN_TEST(test_long_stationary_intervals_use_real_time_at_full_half_and_live_strength);
    RUN_TEST(test_live_sensation_changes_the_duration_of_the_current_pause);
    RUN_TEST(test_original_phase_across_q8_q16_and_sparse_inputs);
    RUN_TEST(test_ms_hold_and_queue_fraction_keep_the_same_park_across_serial_and_step_durations);
    RUN_TEST(test_hold_lands_once_per_cycle_at_the_original_phase_and_scales_stationary_time);
    RUN_TEST(test_close_points_start_at_seam_and_late_activation_does_not_snap);
    RUN_TEST(test_rumble_wall_clock_linearity_region_bias_limits_and_live_derivatives);
    RUN_TEST(test_sparse_wait_groups_preserve_each_real_edge_and_stop_on_a_drained_driver);
    RUN_TEST(test_sparse_wait_pair_obeys_available_ticks_and_partial_append_or_revocation);
    RUN_TEST(test_prepared_refill_limits_ownership_and_releases_on_the_first_decline);
    RUN_TEST(test_slow_guarded_burst_replaces_elapsed_rows_before_returning_with_low_reserve);
    RUN_TEST(test_fast_visits_never_spend_future_time_or_pair_across_the_last_credit);
    RUN_TEST(test_ready_wait_pairs_reach_a_moving_successor_once_under_the_same_authority);
    RUN_TEST(test_positive_rate_boundary_is_timed_at_its_own_accepted_write);
    RUN_TEST(test_prefill_ends_on_start_and_each_gain_change_keeps_its_timestamp);
    RUN_TEST(test_motor_revocation_ends_a_retry_immediately_and_retains_an_accepted_prefix);
    RUN_TEST(test_prepared_pulse_burst_preserves_real_moving_edges_and_partial_acceptance);
    RUN_TEST(test_timed_edges_follow_subframe_crossings_in_both_directions_and_floor_ticks_do_not_accumulate);
    RUN_TEST(test_time_domain_derivatives_remain_bounded_during_repeated_live_retargets);
    RUN_TEST(test_periodic_quintic_preserves_acceleration_holds_and_bounds_the_three_derivatives);
    RUN_TEST(test_high_precision_sine_tracks_the_authored_third_derivative);
    RUN_TEST(test_retargetable_rate_uses_continuous_acceleration_and_bounded_jerk);
    RUN_TEST(test_authored_extension_maps_every_asymmetric_sample_to_the_native_motor_frame);
    RUN_TEST(test_raw_queue_prefills_leading_holds_and_starts_once_with_bounded_lookahead);
    RUN_TEST(test_unstarted_raw_queue_preserves_physical_position_and_reports_driver_errors);
    RUN_TEST(test_finite_prime_tail_uses_programmed_limits_and_verified_position);
    RUN_TEST(test_stationary_entry_keeps_time_across_late_visits_and_discards_debt_on_reset);
    RUN_TEST(test_wire_bounds_are_checked_before_motion);
    RUN_TEST(test_passes_through_all_samples_no_overshoot_and_continuous_velocity);
    RUN_TEST(test_phase_starts_at_a_resting_extremum_and_loops_without_a_jump);
    RUN_TEST(test_time_stretch_bounds_entire_curve_at_speed_and_acceleration_caps);
    RUN_TEST(test_step_frames_track_multiple_periods_without_waypoint_stops_or_drift);
    return UNITY_END();
}
