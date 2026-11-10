#include <unity.h>
#include <limits>
#include "serial_interlock_probe.h"

using namespace serial_interlock_probe;
void setUp() {}
void tearDown() {}

struct Run {
    Gate gate;
    uint32_t now = 0, sequence = 0;
    int32_t origin = -100000;
    Sample last;
    explicit Run(uint32_t start = 0) : now(start) {
        TEST_ASSERT_TRUE(gate.start(origin, now, true));
    }
    void sample(int32_t offset, uint32_t interval = 20) {
        now += interval;
        last = {++sequence, now, 1, origin + offset, 0.004f, 23.4f, 50.0f, 0};
        gate.observe(last, now);
    }
    void enable() {
        sample(0);
        TEST_ASSERT_TRUE(gate.markEnabled(now));
        TEST_ASSERT_TRUE(gate.markTargetWritten(gate.status().targetCounts, now));
    }
    void cut() {
        enable();
        for (int32_t offset = 11; offset <= 88; offset += 11) sample(offset);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::AwaitingGpioCut), static_cast<int>(gate.status().phase));
        TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::AssertGpioHigh), static_cast<int>(gate.takeAction(now)));
        TEST_ASSERT_TRUE(gate.markGpioAsserted(now));
    }
    void proveGpio() {
        cut();
        for (unsigned i = 0; i < 10; ++i) sample(88);
        TEST_ASSERT_TRUE(gate.status().gpioStopProven);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::SerialDisable), static_cast<int>(gate.takeAction(now)));
    }
    void disabled() {
        proveGpio();
        now += 10;
        TEST_ASSERT_TRUE(gate.markSerialDisabled({now, 1, 0}, now));
    }
    void pass() {
        disabled();
        sample(88, 10);
        for (unsigned i = 0; i < 5; ++i) sample(88);
        TEST_ASSERT_TRUE(gate.status().passed);
    }
};

void assert_cleanup(Gate &gate, uint32_t now) {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::AssertGpioHigh), static_cast<int>(gate.takeAction(now)));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::SerialDisable), static_cast<int>(gate.takeAction(now)));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::None), static_cast<int>(gate.takeAction(now)));
    TEST_ASSERT_FALSE(gate.status().passed);
}

void test_success_requires_both_independent_gpio_and_final_uart_quiet_proofs() {
    Run run; run.pass();
    const auto state = run.gate.status();
    TEST_ASSERT_TRUE(state.gpioStopProven);
    TEST_ASSERT_TRUE(state.serialDisabled);
    TEST_ASSERT_TRUE(state.finalQuiet);
    TEST_ASSERT_TRUE(state.finished);
    TEST_ASSERT_FALSE(state.active);
    TEST_ASSERT_FALSE(state.enabled);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::None), static_cast<int>(run.gate.takeAction(run.now)));
}

void test_unreviewed_or_overflowing_start_never_grants_motor_authority() {
    Gate gate;
    TEST_ASSERT_FALSE(gate.start(0, 0, false));
    TEST_ASSERT_FALSE(gate.markEnabled(0));
    TEST_ASSERT_FALSE(gate.status().passed);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::None), static_cast<int>(gate.takeAction(0)));
    Gate overflow;
    TEST_ASSERT_FALSE(overflow.start(std::numeric_limits<int32_t>::max() - 100, 0, true));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::None), static_cast<int>(overflow.takeAction(0)));
}

void test_probe_is_one_shot_even_after_a_success() {
    Run run; run.pass();
    TEST_ASSERT_FALSE(run.gate.start(100, run.now, true));
    TEST_ASSERT_FALSE(run.gate.markTargetWritten(100, run.now));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::None), static_cast<int>(run.gate.takeAction(run.now)));
}

void test_enable_without_fresh_sample_aborts_before_any_target() {
    Run run;
    TEST_ASSERT_FALSE(run.gate.markEnabled(run.now));
    assert_cleanup(run.gate, run.now);
}

void test_monitor_snapshot_before_start_is_ignored_after_newer_seed_even_across_wrap() {
    const uint32_t starts[] = {1000U, std::numeric_limits<uint32_t>::max() - 10U};
    for (const auto start : starts) {
        Run run(start);
        run.sequence = 10;
        run.sample(0);
        auto old = run.last;
        old.sequence = 10;
        old.atMs = start - 20U;
        run.gate.observe(old, start - 1U);
        run.gate.tick(start - 1U);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::None), static_cast<int>(run.gate.takeAction(start - 1U)));
        run.gate.observe(old, run.now);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::None), static_cast<int>(run.gate.status().failure));
        TEST_ASSERT_EQUAL_UINT8(0, run.gate.status().positiveIntervals);
        TEST_ASSERT_TRUE(run.gate.markEnabled(run.now));
        TEST_ASSERT_TRUE(run.gate.markTargetWritten(run.gate.status().targetCounts, run.now));
        run.sample(11);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::None), static_cast<int>(run.gate.status().failure));
        TEST_ASSERT_EQUAL_UINT8(1, run.gate.status().positiveIntervals);
    }
    Run noSeed(1000);
    const Sample old = {10, 980, 1, noSeed.origin, 0.004f, 23.4f, 50.0f, 0};
    noSeed.gate.observe(old, noSeed.now);
    TEST_ASSERT_FALSE(noSeed.gate.markEnabled(noSeed.now));
    TEST_ASSERT_FALSE(noSeed.gate.status().passed);
}

void test_no_motion_or_same_current_target_never_proves_interlock() {
    Run run; run.enable();
    for (unsigned i = 0; i < 50; ++i) run.sample(0);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::NoMotion), static_cast<int>(run.gate.status().failure));
    assert_cleanup(run.gate, run.now);
    Run wrong; wrong.sample(0); TEST_ASSERT_TRUE(wrong.gate.markEnabled(wrong.now));
    TEST_ASSERT_FALSE(wrong.gate.markTargetWritten(wrong.origin, wrong.now));
    assert_cleanup(wrong.gate, wrong.now);
}

void test_gpio_cut_needs_three_distinct_positive_intervals() {
    Run run; run.enable();
    run.sample(30); run.sample(60);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::None), static_cast<int>(run.gate.takeAction(run.now)));
    run.sample(90);
    TEST_ASSERT_EQUAL_UINT8(3, run.gate.status().positiveIntervals);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::AssertGpioHigh), static_cast<int>(run.gate.takeAction(run.now)));
}

void test_target_too_close_cannot_be_mistaken_for_gpio_stopping() {
    Run run; run.enable();
    run.sample(110, 100); run.sample(220, 100);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::TargetTooClose), static_cast<int>(run.gate.status().failure));
    assert_cleanup(run.gate, run.now);
}

void test_replayed_and_changed_duplicate_samples_abort() {
    Run run; run.enable(); run.sample(11);
    auto old = run.last; --old.sequence;
    run.gate.observe(old, run.now);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::Sequence), static_cast<int>(run.gate.status().failure));
    assert_cleanup(run.gate, run.now);
    Run changed; changed.enable();
    auto bad = changed.last; ++bad.encoderCounts;
    changed.gate.observe(bad, changed.now);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::Sequence), static_cast<int>(changed.gate.status().failure));
}

void test_duplicate_polling_never_counts_as_six_quiet_samples() {
    Run run; run.cut();
    run.sample(88, 100);
    TEST_ASSERT_EQUAL_UINT8(1, run.gate.status().gpioQuietSamples);
    for (unsigned i = 0; i < 4; ++i) {
        run.now += 20; run.gate.observe(run.last, run.now);
    }
    TEST_ASSERT_EQUAL_UINT8(1, run.gate.status().gpioQuietSamples);
    TEST_ASSERT_FALSE(run.gate.status().gpioStopProven);
}

void test_braking_grace_does_not_supply_quiet_evidence() {
    Run run; run.cut();
    for (unsigned i = 0; i < 4; ++i) run.sample(88);
    TEST_ASSERT_EQUAL_UINT8(0, run.gate.status().gpioQuietSamples);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::Braking), static_cast<int>(run.gate.status().phase));
    run.sample(88);
    TEST_ASSERT_EQUAL_UINT8(1, run.gate.status().gpioQuietSamples);
}

void test_six_frames_with_less_than_one_hundred_ms_never_prove_stop() {
    Run run; run.cut(); run.sample(88, 100);
    for (unsigned i = 0; i < 5; ++i) run.sample(88, 10);
    TEST_ASSERT_EQUAL_UINT8(6, run.gate.status().gpioQuietSamples);
    TEST_ASSERT_FALSE(run.gate.status().gpioStopProven);
}

void test_one_rpm_continued_motion_fails_tight_quiet_proof() {
    Run run; run.cut();
    for (int32_t offset = 99; offset <= 198; offset += 11) run.sample(offset);
    TEST_ASSERT_FALSE(run.gate.status().gpioStopProven);
    run.sample(209);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::TargetTooClose), static_cast<int>(run.gate.status().failure));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::SerialDisable), static_cast<int>(run.gate.takeAction(run.now)));
}

void test_gpio_measurement_deadline_always_requests_serial_disable() {
    Run run; run.cut();
    for (unsigned i = 0; i < 12; ++i) run.sample(88 + (i % 2 ? 10 : 0));
    run.now += 11; run.gate.tick(run.now);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::GpioDidNotStop), static_cast<int>(run.gate.status().failure));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::SerialDisable), static_cast<int>(run.gate.takeAction(run.now)));
}

void test_unacknowledged_gpio_action_cannot_produce_a_proof() {
    Run run; run.enable();
    for (int32_t offset = 11; offset <= 88; offset += 11) run.sample(offset);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::AssertGpioHigh), static_cast<int>(run.gate.takeAction(run.now)));
    run.now += 11; run.gate.tick(run.now);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::GpioNotAsserted), static_cast<int>(run.gate.status().failure));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::SerialDisable), static_cast<int>(run.gate.takeAction(run.now)));
}

void test_repeated_target_or_other_control_write_invalidates_measurement() {
    Run run; run.cut();
    TEST_ASSERT_FALSE(run.gate.markTargetWritten(run.gate.status().targetCounts, run.now));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::UnexpectedWrite), static_cast<int>(run.gate.status().failure));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::SerialDisable), static_cast<int>(run.gate.takeAction(run.now)));
    Run other; other.cut(); other.gate.unexpectedControlWrite(other.now);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::UnexpectedWrite), static_cast<int>(other.gate.status().failure));
}

void test_overcurrent_envelope_reverse_and_speed_each_request_cleanup() {
    for (unsigned fault = 0; fault < 4; ++fault) {
        Run run; run.enable();
        auto sample = run.last; sample.atMs += 20; ++sample.sequence;
        const Failure expected[] = {Failure::Current, Failure::Envelope, Failure::WrongDirection, Failure::Speed};
        if (fault == 0) sample.currentAmps = 0.251f;
        if (fault == 1) sample.encoderCounts += 615;
        if (fault == 2) sample.encoderCounts -= 5;
        if (fault == 3) sample.encoderCounts += 34;
        run.gate.observe(sample, sample.atMs);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(expected[fault]), static_cast<int>(run.gate.status().failure));
        assert_cleanup(run.gate, sample.atMs);
    }
}

void test_stale_rate_continuity_nan_and_alarm_each_abort() {
    const Failure expected[] = {Failure::StaleSample, Failure::SampleRate, Failure::Continuity,
                                Failure::InvalidSample, Failure::Alarm};
    for (unsigned fault = 0; fault < 5; ++fault) {
        Run run; run.enable();
        auto sample = run.last; sample.atMs += 20; ++sample.sequence;
        uint32_t now = sample.atMs;
        if (fault == 0) now += 101;
        if (fault == 1) sample.observedHz = 30;
        if (fault == 2) ++sample.continuity;
        if (fault == 3) sample.currentAmps = std::numeric_limits<float>::quiet_NaN();
        if (fault == 4) sample.alarm = 0x14;
        run.gate.observe(sample, now);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(expected[fault]), static_cast<int>(run.gate.status().failure));
        assert_cleanup(run.gate, now);
    }
}

void test_old_mode_zero_or_enabled_readback_can_never_finish_passed() {
    for (unsigned fault = 0; fault < 3; ++fault) {
        Run run; run.proveGpio();
        const uint32_t old = run.now - 20; run.now += 10;
        SerialReadback reply{fault == 0 ? old : run.now,
                             static_cast<uint16_t>(fault == 1 ? 0 : 1),
                             static_cast<uint16_t>(fault == 2 ? 1 : 0)};
        TEST_ASSERT_FALSE(run.gate.markSerialDisabled(reply, run.now));
        TEST_ASSERT_FALSE(run.gate.status().passed);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::SerialDisableUnverified), static_cast<int>(run.gate.status().failure));
    }
}

void test_serial_readback_alone_is_not_final_stationarity() {
    Run run; run.disabled();
    TEST_ASSERT_TRUE(run.gate.status().serialDisabled);
    TEST_ASSERT_FALSE(run.gate.status().passed);
    run.sample(88, 10);
    for (unsigned i = 0; i < 4; ++i) run.sample(88);
    TEST_ASSERT_EQUAL_UINT8(5, run.gate.status().finalQuietSamples);
    TEST_ASSERT_FALSE(run.gate.status().passed);
    run.sample(88);
    TEST_ASSERT_TRUE(run.gate.status().passed);
}

void test_safety_abort_cannot_be_upgraded_after_successful_cleanup() {
    Run run; run.cut(); run.gate.abort(Failure::SafetyFault, run.now);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::SerialDisable), static_cast<int>(run.gate.takeAction(run.now)));
    run.now += 10; TEST_ASSERT_TRUE(run.gate.markSerialDisabled({run.now, 1, 0}, run.now));
    run.sample(88, 10);
    for (unsigned i = 0; i < 5; ++i) run.sample(88);
    TEST_ASSERT_TRUE(run.gate.status().finished);
    TEST_ASSERT_TRUE(run.gate.status().finalQuiet);
    TEST_ASSERT_FALSE(run.gate.status().passed);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::SafetyFault), static_cast<int>(run.gate.status().failure));
}

void test_repeated_abort_after_uart_disable_finishes_quiet_without_passing() {
    Run run; run.disabled();
    run.gate.abort(Failure::Cancelled, run.now);
    for (unsigned i = 0; i < 6; ++i) {
        run.gate.abort(Failure::Cancelled, run.now);
        run.sample(88);
    }
    TEST_ASSERT_TRUE(run.gate.status().finished);
    TEST_ASSERT_TRUE(run.gate.status().serialDisabled);
    TEST_ASSERT_TRUE(run.gate.status().finalQuiet);
    TEST_ASSERT_FALSE(run.gate.status().passed);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::Cancelled), static_cast<int>(run.gate.status().failure));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::None), static_cast<int>(run.gate.takeAction(run.now)));
}

void test_overall_timeout_keeps_cleanup_actions_drainable() {
    Run run; run.enable();
    run.now = 3000; run.gate.tick(run.now);
    TEST_ASSERT_TRUE(run.gate.status().finished);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::OverallTimeout), static_cast<int>(run.gate.status().failure));
    assert_cleanup(run.gate, run.now);
}

void test_millisecond_counter_wrap_preserves_physical_proof_timing() {
    Run run(std::numeric_limits<uint32_t>::max() - 200);
    run.pass();
    TEST_ASSERT_TRUE(run.gate.status().passed);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_success_requires_both_independent_gpio_and_final_uart_quiet_proofs);
    RUN_TEST(test_unreviewed_or_overflowing_start_never_grants_motor_authority);
    RUN_TEST(test_probe_is_one_shot_even_after_a_success);
    RUN_TEST(test_enable_without_fresh_sample_aborts_before_any_target);
    RUN_TEST(test_monitor_snapshot_before_start_is_ignored_after_newer_seed_even_across_wrap);
    RUN_TEST(test_no_motion_or_same_current_target_never_proves_interlock);
    RUN_TEST(test_gpio_cut_needs_three_distinct_positive_intervals);
    RUN_TEST(test_target_too_close_cannot_be_mistaken_for_gpio_stopping);
    RUN_TEST(test_replayed_and_changed_duplicate_samples_abort);
    RUN_TEST(test_duplicate_polling_never_counts_as_six_quiet_samples);
    RUN_TEST(test_braking_grace_does_not_supply_quiet_evidence);
    RUN_TEST(test_six_frames_with_less_than_one_hundred_ms_never_prove_stop);
    RUN_TEST(test_one_rpm_continued_motion_fails_tight_quiet_proof);
    RUN_TEST(test_gpio_measurement_deadline_always_requests_serial_disable);
    RUN_TEST(test_unacknowledged_gpio_action_cannot_produce_a_proof);
    RUN_TEST(test_repeated_target_or_other_control_write_invalidates_measurement);
    RUN_TEST(test_overcurrent_envelope_reverse_and_speed_each_request_cleanup);
    RUN_TEST(test_stale_rate_continuity_nan_and_alarm_each_abort);
    RUN_TEST(test_old_mode_zero_or_enabled_readback_can_never_finish_passed);
    RUN_TEST(test_serial_readback_alone_is_not_final_stationarity);
    RUN_TEST(test_safety_abort_cannot_be_upgraded_after_successful_cleanup);
    RUN_TEST(test_repeated_abort_after_uart_disable_finishes_quiet_without_passing);
    RUN_TEST(test_overall_timeout_keeps_cleanup_actions_drainable);
    RUN_TEST(test_millisecond_counter_wrap_preserves_physical_proof_timing);
    return UNITY_END();
}
