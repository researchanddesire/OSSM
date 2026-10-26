#include <unity.h>

#include <climits>
#include <initializer_list>

#include "motor_guard_logic.h"

void setUp() {}
void tearDown() {}

using motor_guard::Fault;

namespace {

motor_guard::Config testConfig() {
    motor_guard::Config config;
    config.countsPerMm = 1000.0;
    return config;
}

struct Bench {
    motor_guard::Guard guard{testConfig()};
    uint32_t now = 0;
    uint32_t sequence = 0;
    uint32_t continuity = 1;
    int32_t position = 0;
    float current = 0.05f;
    float voltage = 24.0f;
    uint16_t alarm = 0;

    void sample(uint32_t elapsed = 20) {
        const uint32_t target = now + elapsed;
        for (uint32_t passed = 5; passed < elapsed; passed += 5)
            guard.tick(now + passed);
        now = target;
        guard.observe({++sequence, now, continuity, position,
                       current, voltage, alarm}, now);
        guard.tick(now);
    }

    void warmup() {
        guard.setStopPathVerified(true);
        for (int i = 0; i < 31; ++i) sample();
    }

    void arm() {
        warmup();
        TEST_ASSERT_TRUE(guard.ready(now));
        TEST_ASSERT_TRUE(guard.arm(now));
    }

    void travel(int32_t target) {
        while (position != target && guard.fault() == Fault::None) {
            const int64_t delta = static_cast<int64_t>(target) - position;
            position += delta > 500 ? 500 : delta < -500 ? -500 : delta;
            sample();
        }
        // A second stationary physical sample establishes that the retreat
        // has finished; the command counter is deliberately absent.
        sample();
    }

    void contact() {
        current = 2.0f;
        sample();
        TEST_ASSERT_TRUE(guard.contactActive());
    }

    void releaseAndRetreat(int32_t target) {
        current = 0.05f;
        travel(target);
    }

    void home() {
        arm();
        TEST_ASSERT_TRUE(guard.beginHomingPass(1, 25.0f, now));
        TEST_ASSERT_TRUE(guard.beginHomingMotion(now));
        travel(-50000);
        contact();
        TEST_ASSERT_TRUE(guard.beginHomingMotion(now));
        releaseAndRetreat(-40000);
        TEST_ASSERT_TRUE(guard.endHomingPass(now));
        TEST_ASSERT_TRUE(guard.beginHomingPass(2, 25.0f, now));
        TEST_ASSERT_TRUE(guard.beginHomingMotion(now));
        travel(50000);
        contact();
        TEST_ASSERT_TRUE(guard.beginHomingMotion(now));
        releaseAndRetreat(40000);
        TEST_ASSERT_TRUE(guard.endHomingPass(now));
        TEST_ASSERT_TRUE(guard.boundsValid());
        TEST_ASSERT_TRUE(guard.beginNormalOperation(now));
    }
};

void assertFault(Fault expected, const motor_guard::Guard &guard) {
    TEST_ASSERT_EQUAL(static_cast<int>(expected), static_cast<int>(guard.fault()));
    TEST_ASSERT_FALSE(guard.motionAllowed());
}

}  // namespace

void test_discovery_blocks_arm_without_latching() {
    Bench bench;
    TEST_ASSERT_FALSE(bench.guard.arm(0));
    TEST_ASSERT_EQUAL(static_cast<int>(Fault::None), static_cast<int>(bench.guard.fault()));
    bench.warmup();
    bench.guard.setStopPathVerified(false);
    TEST_ASSERT_FALSE(bench.guard.arm(bench.now));
    bench.guard.setStopPathVerified(true);
    bench.voltage = 19.0f;
    bench.sample();
    TEST_ASSERT_FALSE(bench.guard.arm(bench.now));
    TEST_ASSERT_EQUAL(static_cast<int>(Fault::None), static_cast<int>(bench.guard.fault()));
}

void test_rate_uses_distinct_physical_samples() {
    Bench bench;
    bench.guard.setStopPathVerified(true);
    for (uint32_t time = 0; time <= 600; time += 5) {
        bench.guard.observe({1, time, 1, 0, 0.05f, 24.0f, 0}, time);
        bench.guard.tick(time);
    }
    TEST_ASSERT_FALSE(bench.guard.ready(600));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, bench.guard.observedHz(600));
    Bench healthy;
    healthy.warmup();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.0f, healthy.guard.observedHz(healthy.now));
    TEST_ASSERT_TRUE(healthy.guard.arm(healthy.now));
}

void test_stale_sample_latches_and_cannot_auto_resume() {
    Bench bench;
    bench.arm();
    for (uint32_t elapsed = 5; elapsed <= 105; elapsed += 5)
        bench.guard.tick(bench.now + elapsed);
    assertFault(Fault::TelemetryStale, bench.guard);
    bench.now += 105;
    bench.sample();
    TEST_ASSERT_FALSE(bench.guard.arm(bench.now));
    assertFault(Fault::TelemetryStale, bench.guard);
    bench.guard.resetAfterReview();
    TEST_ASSERT_FALSE(bench.guard.arm(bench.now));
    bench.warmup();
    TEST_ASSERT_TRUE(bench.guard.arm(bench.now));
}

void test_rate_drop_latches_before_a_new_arm() {
    Bench bench;
    bench.arm();
    for (int i = 0; i < 30 && bench.guard.fault() == Fault::None; ++i)
        bench.sample(50);  // 20 Hz with a task that still ticks every 5 ms
    assertFault(Fault::TelemetryRate, bench.guard);
}

void test_guard_task_deadline_is_independent_of_uart_freshness() {
    Bench bench;
    bench.arm();
    bench.guard.tick(bench.now + 35);
    assertFault(Fault::ObserverDelayed, bench.guard);
}

void test_encoder_continuity_change_stops_an_armed_session() {
    Bench bench;
    bench.arm();
    ++bench.continuity;
    bench.sample();
    assertFault(Fault::ContinuityChanged, bench.guard);
}

void test_fatal_alarm_voltage_and_overcurrent_have_immediate_stops() {
    Bench alarm;
    alarm.arm();
    alarm.alarm = 0x10;
    alarm.sample();
    TEST_ASSERT_TRUE(alarm.guard.motionAllowed());
    alarm.alarm = 0x14;
    alarm.sample();
    assertFault(Fault::Alarm, alarm.guard);

    for (float value : {19.99f, 36.01f}) {
        Bench supply;
        supply.arm();
        supply.voltage = value;
        supply.sample();
        assertFault(Fault::SupplyVoltage, supply.guard);
    }
    Bench current;
    current.arm();
    current.current = 2.2f;
    current.sample();
    TEST_ASSERT_TRUE(current.guard.motionAllowed());
    current.current = 2.21f;
    current.sample();
    assertFault(Fault::OverCurrent, current.guard);
}

void test_invalid_data_and_external_stop_proof_faults_latch() {
    Bench invalid;
    invalid.arm();
    invalid.current = NAN;
    invalid.sample();
    assertFault(Fault::InvalidTelemetry, invalid.guard);

    Bench proof;
    proof.arm();
    proof.guard.setStopPathVerified(false);
    assertFault(Fault::StopPathUnverified, proof.guard);
    proof.guard.tripFault(Fault::Alarm);
    assertFault(Fault::StopPathUnverified, proof.guard);  // first reason retained
}

void test_motion_without_a_homing_phase_is_blocked() {
    Bench bench;
    bench.arm();
    bench.position += 500;
    bench.sample();
    assertFault(Fault::UncalibratedMotion, bench.guard);
}

void test_homing_uses_measured_speed_and_sequential_contact_stages() {
    Bench tooFast;
    tooFast.arm();
    TEST_ASSERT_TRUE(tooFast.guard.beginHomingPass(1, 25.0f, tooFast.now));
    for (int i = 0; i < 3 && tooFast.guard.fault() == Fault::None; ++i) {
        tooFast.position += 1000;  // sustained50mm/s despite declared25
        tooFast.sample();
    }
    assertFault(Fault::HomingSpeed, tooFast.guard);

    Bench badSequence;
    badSequence.arm();
    TEST_ASSERT_FALSE(badSequence.guard.beginHomingPass(2, 25.0f, badSequence.now));
    assertFault(Fault::HomingSequence, badSequence.guard);
}

void test_reply_completion_jitter_does_not_invent_homing_overspeed() {
    Bench bench;
    bench.arm();
    TEST_ASSERT_TRUE(bench.guard.beginHomingPass(1, 25.0f, bench.now));
    bool sawInflatedInstantaneousSpeed = false;
    for (int i = 0; i < 80; ++i) {
        // The motor's acquisition cadence is20ms and actual travel0.5mm.
        // Alternating reply lengths add/remove4ms of completion latency.
        bench.position += 500;
        bench.sample((i % 2) == 0 ? 24 : 16);
        sawInflatedInstantaneousSpeed |= bench.guard.speedMmS() > 30.0;
        TEST_ASSERT_EQUAL(static_cast<int>(Fault::None), static_cast<int>(bench.guard.fault()));
        TEST_ASSERT_TRUE(bench.guard.homingSpeedWindowMs() >= 60);
        TEST_ASSERT_TRUE(std::abs(bench.guard.homingSpeedMmS()) <= 30.0);
    }
    TEST_ASSERT_TRUE(sawInflatedInstantaneousSpeed);  //31.25mm/s artifact
    TEST_ASSERT_TRUE(bench.guard.motionAllowed());
}

void test_true_windowed_overspeed_still_latches_and_preserves_evidence() {
    Bench bench;
    bench.arm();
    TEST_ASSERT_TRUE(bench.guard.beginHomingPass(1, 25.0f, bench.now));
    const uint32_t started = bench.now;
    for (int i = 0; i < 5 && bench.guard.fault() == Fault::None; ++i) {
        bench.position += 700;  //35mm/s, still above unchanged30mm/s limit
        bench.sample();
    }
    assertFault(Fault::HomingSpeed, bench.guard);
    TEST_ASSERT_TRUE(bench.now - started <= 80);
    const auto evidence = bench.guard.faultDiagnostics();
    TEST_ASSERT_TRUE(evidence.available);
    TEST_ASSERT_EQUAL_UINT32(bench.now, evidence.sampleAtMs);
    TEST_ASSERT_EQUAL_UINT32(bench.sequence, evidence.sampleSequence);
    TEST_ASSERT_EQUAL_UINT32(20, evidence.sampleGapMs);
    TEST_ASSERT_EQUAL_INT64(700, evidence.encoderDeltaCounts);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 35.0, evidence.instantaneousMmS);
    TEST_ASSERT_TRUE(evidence.homingWindowMmS > 30.0);
    TEST_ASSERT_TRUE(evidence.homingWindowMs >= 60);
    bench.guard.tick(bench.now + 500);
    bench.guard.tripFault(Fault::Alarm);
    TEST_ASSERT_EQUAL_UINT32(evidence.sampleSequence,
                            bench.guard.faultDiagnostics().sampleSequence);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, evidence.homingWindowMmS,
                             bench.guard.faultDiagnostics().homingWindowMmS);
    bench.guard.resetAfterReview();
    TEST_ASSERT_FALSE(bench.guard.faultDiagnostics().available);
}

void test_exactly_two_deliberate_contacts_establish_physical_bounds() {
    Bench bench;
    bench.home();
    TEST_ASSERT_EQUAL_UINT8(2, bench.guard.contactCount());
    TEST_ASSERT_EQUAL_INT32(-50000, bench.guard.lowerStopCounts());
    TEST_ASSERT_EQUAL_INT32(50000, bench.guard.upperStopCounts());
    TEST_ASSERT_TRUE(bench.guard.motionAllowed());
    bench.travel(0);
    TEST_ASSERT_TRUE(bench.guard.motionAllowed());
}

void test_long_or_repeated_homing_contacts_fault() {
    Bench longContact;
    longContact.arm();
    TEST_ASSERT_TRUE(longContact.guard.beginHomingPass(1, 25.0f, longContact.now));
    longContact.contact();
    for (int i = 0; i < 11 && longContact.guard.fault() == Fault::None; ++i)
        longContact.sample();
    assertFault(Fault::ContactTooLong, longContact.guard);

    Bench repeated;
    repeated.arm();
    TEST_ASSERT_TRUE(repeated.guard.beginHomingPass(1, 25.0f, repeated.now));
    repeated.contact();
    repeated.current = 0.05f;
    repeated.sample();
    repeated.current = 2.0f;
    repeated.sample();
    assertFault(Fault::RepeatedContact, repeated.guard);
}

void test_normal_stationary_load_is_not_a_third_allowed_contact() {
    Bench bench;
    bench.home();
    bench.current = 2.0f;
    bench.sample();
    for (int i = 0; i < 3 && bench.guard.fault() == Fault::None; ++i)
        bench.sample();
    assertFault(Fault::UnexpectedLoad, bench.guard);
    TEST_ASSERT_EQUAL_UINT8(2, bench.guard.contactCount());
}

void test_small_encoder_oscillation_cannot_hide_loaded_contact() {
    Bench normal;
    normal.home();
    normal.current = 2.0f;
    for (int i = 0; i < 5 && normal.guard.fault() == Fault::None; ++i) {
        normal.position = (i % 2) ? 40000 : 40050;
        normal.sample();  // +/-2.5 mm/s derivative but only0.05mm range
    }
    assertFault(Fault::UnexpectedLoad, normal.guard);

    Bench homing;
    homing.arm();
    TEST_ASSERT_TRUE(homing.guard.beginHomingPass(1, 25.0f, homing.now));
    homing.current = 2.0f;
    for (int i = 0; i < 12 && homing.guard.fault() == Fault::None; ++i) {
        homing.position = (i % 2) ? 0 : 50;
        homing.sample();
    }
    assertFault(Fault::ContactTooLong, homing.guard);
}

void test_measured_bounds_include_margin_and_reaction_projection() {
    Bench margin;
    margin.home();
    // Approach slowly enough to retain the predictive reaction reserve.
    while (margin.position < 43000 && margin.guard.fault() == Fault::None) {
        margin.position += 100;
        margin.sample();
    }
    margin.sample();  // 7 mm from upper physical stop, at rest
    TEST_ASSERT_TRUE(margin.guard.motionAllowed());
    margin.position = 45000;  // only 5 mm reserve
    margin.sample();
    assertFault(Fault::TravelBounds, margin.guard);

    Bench projected;
    projected.home();
    projected.travel(0);
    projected.position = 10000;  // measured 500 mm/s, projected +62.5 mm
    projected.sample();
    assertFault(Fault::TravelBounds, projected.guard);
}

void test_epoch_and_guard_time_rollover_are_safe() {
    Bench bench;
    bench.now = UINT32_MAX - 700;
    bench.arm();
    TEST_ASSERT_TRUE(bench.guard.beginHomingPass(1, 25.0f, bench.now));
    for (int i = 0; i < 10; ++i) bench.sample();
    TEST_ASSERT_TRUE(bench.guard.motionAllowed());

    Bench encoder;
    encoder.position = INT32_MAX - 100;
    encoder.arm();
    TEST_ASSERT_TRUE(encoder.guard.beginHomingPass(1, 25.0f, encoder.now));
    encoder.position = INT32_MIN + 99;  // +200 counts across wrap: 10 mm/s
    encoder.sample();
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 10.0, encoder.guard.speedMmS());
    TEST_ASSERT_TRUE(encoder.guard.motionAllowed());
}

void test_unarmed_feedback_gap_discards_old_physical_bounds() {
    Bench bench;
    bench.home();
    bench.guard.disarm();
    bench.sample(120);  // longer than guard freshness, same driver epoch
    TEST_ASSERT_FALSE(bench.guard.boundsValid());
    bench.warmup();
    TEST_ASSERT_TRUE(bench.guard.arm(bench.now));
    TEST_ASSERT_FALSE(bench.guard.beginNormalOperation(bench.now));
    assertFault(Fault::UncalibratedMotion, bench.guard);
}

void test_low_current_homing_start_stall_is_bounded_to100ms() {
    Bench bench;
    bench.arm();
    TEST_ASSERT_TRUE(bench.guard.beginHomingPass(1, 25.0f, bench.now));
    // A stationary reference capture is not a motion command.
    for (int i = 0; i < 12; ++i) bench.sample();
    TEST_ASSERT_TRUE(bench.guard.motionAllowed());
    TEST_ASSERT_TRUE(bench.guard.beginHomingMotion(bench.now));
    const uint32_t started = bench.now;
    for (int i = 0; i < 6 && bench.guard.fault() == Fault::None; ++i)
        bench.sample();
    assertFault(Fault::HomingProgress, bench.guard);
    TEST_ASSERT_EQUAL_UINT32(100, bench.now - started);
    TEST_ASSERT_EQUAL_UINT8(0, bench.guard.contactCount());
    const auto evidence = bench.guard.faultDiagnostics();
    TEST_ASSERT_TRUE(evidence.homingMotionExpected);
    TEST_ASSERT_EQUAL_UINT32(100, evidence.homingProgressAgeMs);
}

void test_low_current_stall_after_real_motion_uses_last_new_extent() {
    Bench bench;
    bench.arm();
    TEST_ASSERT_TRUE(bench.guard.beginHomingPass(1, 25.0f, bench.now));
    TEST_ASSERT_TRUE(bench.guard.beginHomingMotion(bench.now));
    bench.travel(10000);
    const uint32_t lastAdvance = bench.now - 20;
    for (int i = 0; i < 6 && bench.guard.fault() == Fault::None; ++i)
        bench.sample();
    assertFault(Fault::HomingProgress, bench.guard);
    TEST_ASSERT_EQUAL_UINT32(100, bench.now - lastAdvance);
    TEST_ASSERT_EQUAL_INT8(1, bench.guard.faultDiagnostics().homingProgressDirection);
}

void test_encoder_oscillation_cannot_refresh_directional_progress() {
    Bench bench;
    bench.arm();
    TEST_ASSERT_TRUE(bench.guard.beginHomingPass(1, 25.0f, bench.now));
    TEST_ASSERT_TRUE(bench.guard.beginHomingMotion(bench.now));
    const uint32_t started = bench.now;
    for (int i = 0; i < 8 && bench.guard.fault() == Fault::None; ++i) {
        bench.position = i % 2 ? -300 : 300;
        bench.sample();
    }
    assertFault(Fault::HomingProgress, bench.guard);
    // The first0.3mm advance counts once; revisiting it cannot extend time.
    TEST_ASSERT_EQUAL_UINT32(120, bench.now - started);
    TEST_ASSERT_EQUAL_UINT32(100, bench.guard.faultDiagnostics().homingProgressAgeMs);
}

void test_repeated_motion_marker_cannot_extend_an_unqualified_stall() {
    Bench bench;
    bench.arm();
    TEST_ASSERT_TRUE(bench.guard.beginHomingPass(1, 25.0f, bench.now));
    TEST_ASSERT_TRUE(bench.guard.beginHomingMotion(bench.now));
    bench.sample();
    TEST_ASSERT_FALSE(bench.guard.beginHomingMotion(bench.now));
    assertFault(Fault::HomingSequence, bench.guard);
}

void test_qualified_retreat_has_bounded_grace_for_virtual_overtravel() {
    Bench bench;
    bench.arm();
    TEST_ASSERT_TRUE(bench.guard.beginHomingPass(1, 25.0f, bench.now));
    TEST_ASSERT_TRUE(bench.guard.beginHomingMotion(bench.now));
    bench.travel(-5000);
    bench.contact();
    for (int i = 0; i < 3; ++i) bench.sample();
    // The load can fall before the next encoder frame shows physical retreat.
    TEST_ASSERT_TRUE(bench.guard.beginHomingMotion(bench.now));
    bench.current = 0.05f;
    bench.sample();
    bench.sample();
    TEST_ASSERT_TRUE(bench.guard.motionAllowed());
    bench.position += 500;
    bench.sample();
    bench.travel(5000);
    TEST_ASSERT_TRUE(bench.guard.endHomingPass(bench.now));
    TEST_ASSERT_FALSE(bench.guard.homingMotionExpected());
    for (int i = 0; i < 15; ++i) bench.sample();
    TEST_ASSERT_TRUE(bench.guard.motionAllowed());
}

void test_stalled_retreat_cannot_restart_absolute200ms_progress_bound() {
    Bench bench;
    bench.arm();
    TEST_ASSERT_TRUE(bench.guard.beginHomingPass(1, 25.0f, bench.now));
    TEST_ASSERT_TRUE(bench.guard.beginHomingMotion(bench.now));
    bench.travel(-5000);
    const uint32_t lastAdvance = bench.now - 20;
    bench.contact();
    for (int i = 0; i < 4; ++i) bench.sample();
    TEST_ASSERT_TRUE(bench.guard.beginHomingMotion(bench.now));
    bench.current = 0.05f;
    for (int i = 0; i < 6 && bench.guard.fault() == Fault::None; ++i)
        bench.sample();
    assertFault(Fault::HomingProgress, bench.guard);
    TEST_ASSERT_EQUAL_UINT32(200, bench.now - lastAdvance);
    const auto evidence = bench.guard.faultDiagnostics();
    TEST_ASSERT_EQUAL_UINT32(200, evidence.homingProgressAgeMs);
    TEST_ASSERT_TRUE(evidence.homingLowCurrentGraceAgeMs < 100);
}

void test_load_drop_without_retreat_marker_does_not_reset_progress_clock() {
    Bench bench;
    bench.arm();
    TEST_ASSERT_TRUE(bench.guard.beginHomingPass(1, 25.0f, bench.now));
    TEST_ASSERT_TRUE(bench.guard.beginHomingMotion(bench.now));
    bench.travel(-5000);
    bench.contact();
    for (int i = 0; i < 4; ++i) bench.sample();
    TEST_ASSERT_TRUE(bench.guard.motionAllowed());
    bench.current = 0.05f;
    bench.sample();
    assertFault(Fault::HomingProgress, bench.guard);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_discovery_blocks_arm_without_latching);
    RUN_TEST(test_rate_uses_distinct_physical_samples);
    RUN_TEST(test_stale_sample_latches_and_cannot_auto_resume);
    RUN_TEST(test_rate_drop_latches_before_a_new_arm);
    RUN_TEST(test_guard_task_deadline_is_independent_of_uart_freshness);
    RUN_TEST(test_encoder_continuity_change_stops_an_armed_session);
    RUN_TEST(test_fatal_alarm_voltage_and_overcurrent_have_immediate_stops);
    RUN_TEST(test_invalid_data_and_external_stop_proof_faults_latch);
    RUN_TEST(test_motion_without_a_homing_phase_is_blocked);
    RUN_TEST(test_homing_uses_measured_speed_and_sequential_contact_stages);
    RUN_TEST(test_reply_completion_jitter_does_not_invent_homing_overspeed);
    RUN_TEST(test_true_windowed_overspeed_still_latches_and_preserves_evidence);
    RUN_TEST(test_exactly_two_deliberate_contacts_establish_physical_bounds);
    RUN_TEST(test_long_or_repeated_homing_contacts_fault);
    RUN_TEST(test_normal_stationary_load_is_not_a_third_allowed_contact);
    RUN_TEST(test_small_encoder_oscillation_cannot_hide_loaded_contact);
    RUN_TEST(test_measured_bounds_include_margin_and_reaction_projection);
    RUN_TEST(test_epoch_and_guard_time_rollover_are_safe);
    RUN_TEST(test_unarmed_feedback_gap_discards_old_physical_bounds);
    RUN_TEST(test_low_current_homing_start_stall_is_bounded_to100ms);
    RUN_TEST(test_low_current_stall_after_real_motion_uses_last_new_extent);
    RUN_TEST(test_encoder_oscillation_cannot_refresh_directional_progress);
    RUN_TEST(test_repeated_motion_marker_cannot_extend_an_unqualified_stall);
    RUN_TEST(test_qualified_retreat_has_bounded_grace_for_virtual_overtravel);
    RUN_TEST(test_stalled_retreat_cannot_restart_absolute200ms_progress_bound);
    RUN_TEST(test_load_drop_without_retreat_marker_does_not_reset_progress_clock);
    return UNITY_END();
}
