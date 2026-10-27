#include <unity.h>

#include "serial_axis_logic.h"

void setUp() {}
void tearDown() {}

namespace {
struct Axis {
    serial_axis::Position position;
    uint32_t sequence = 0, now = 0, continuity = 1;
    int32_t encoder = 1000;
    void sample(uint32_t elapsed = 20) {
        now += elapsed;
        TEST_ASSERT_TRUE(position.observe({++sequence, now, continuity, encoder}, now));
    }
    void anchor(int32_t logical = 0) {
        sample(); sample();
        TEST_ASSERT_TRUE(position.rebase(logical, now));
    }
};
}

void test_absolute_targets_use_encoder_units_and_keep_command_separate() {
    Axis axis;
    axis.anchor();
    int32_t target;
    TEST_ASSERT_TRUE(axis.position.target(20, target));
    TEST_ASSERT_EQUAL_INT32(1819, target); // 1mm, not twenty encoder counts.
    axis.position.submitted(7, 20, target);
    TEST_ASSERT_EQUAL_INT32(0, axis.position.current());
    TEST_ASSERT_EQUAL_INT32(20, axis.position.commandedTarget());
    TEST_ASSERT_TRUE(axis.position.running(axis.now));
    axis.position.acknowledge(7, target);
    TEST_ASSERT_TRUE(axis.position.running(axis.now)); // ACK is not arrival.
    axis.encoder = target;
    axis.sample(); axis.sample(); axis.sample();
    TEST_ASSERT_EQUAL_INT32(20, axis.position.current());
    TEST_ASSERT_FALSE(axis.position.running(axis.now));
}

void test_arrival_requires_ack_and_unique_stopped_physical_samples() {
    Axis axis;
    axis.anchor();
    int32_t target;
    TEST_ASSERT_TRUE(axis.position.target(100, target));
    axis.position.submitted(9, 100, target);
    axis.encoder = target;
    axis.sample(); axis.sample();
    TEST_ASSERT_TRUE(axis.position.running(axis.now));
    axis.position.acknowledge(8, target); // Wrong request cannot certify arrival.
    axis.sample();
    TEST_ASSERT_TRUE(axis.position.running(axis.now));
    axis.position.acknowledge(9, target);
    for (int i = 0; i < 10; ++i)
        axis.position.observe({axis.sequence, axis.now, axis.continuity, target}, axis.now);
    TEST_ASSERT_TRUE(axis.position.running(axis.now));
    axis.sample();
    TEST_ASSERT_TRUE(axis.position.running(axis.now));
    axis.sample();
    TEST_ASSERT_FALSE(axis.position.running(axis.now));
    TEST_ASSERT_TRUE(axis.position.running(axis.now + 101));
}

void test_rebase_and_direction_change_do_not_move_the_encoder_origin() {
    Axis axis;
    axis.encoder = 2000;
    axis.anchor();
    TEST_ASSERT_TRUE(axis.position.rebase(-120, axis.now));
    TEST_ASSERT_TRUE(axis.position.direction(true, axis.now));
    int32_t target;
    TEST_ASSERT_TRUE(axis.position.target(80, target));
    TEST_ASSERT_EQUAL_INT32(-6192, target);
    axis.encoder = target;
    axis.sample(); axis.sample();
    TEST_ASSERT_EQUAL_INT32(80, axis.position.current());
    TEST_ASSERT_TRUE(axis.position.rebase(-200, axis.now));
    TEST_ASSERT_TRUE(axis.position.direction(false, axis.now));
    TEST_ASSERT_TRUE(axis.position.target(0, target));
    TEST_ASSERT_EQUAL_INT32(2000, target);
    TEST_ASSERT_EQUAL_INT32(-200, axis.position.current());
}

void test_active_or_moving_axis_cannot_be_rebased() {
    Axis axis;
    axis.anchor();
    axis.position.submitted(1, 100, 5096);
    TEST_ASSERT_FALSE(axis.position.rebase(0, axis.now));
    TEST_ASSERT_FALSE(axis.position.direction(true, axis.now));
    axis.position.canceled();
    axis.encoder += 100;
    axis.sample();
    TEST_ASSERT_TRUE(axis.position.running(axis.now));
    TEST_ASSERT_FALSE(axis.position.rebase(0, axis.now));
    axis.sample();
    TEST_ASSERT_TRUE(axis.position.rebase(0, axis.now));
}

void test_mode_frame_mirrors_preserve_anchor_without_rounding_drift() {
    Axis axis;
    axis.encoder = 2000;
    axis.anchor();
    axis.encoder += 13; // Fractional pulse-equivalent step at physical rest.
    axis.sample(); axis.sample();
    for (int i = 0; i < 100; ++i) {
        TEST_ASSERT_TRUE(axis.position.mirrorFrame(120, axis.now));
        TEST_ASSERT_TRUE(axis.position.mirrorFrame(120, axis.now));
        int32_t target;
        TEST_ASSERT_TRUE(axis.position.target(0, target));
        TEST_ASSERT_EQUAL_INT32(2000, target);
    }
}

void test_one_rpm_motion_is_not_reported_as_stopped_or_arrived() {
    Axis axis;
    axis.anchor();
    int32_t target;
    TEST_ASSERT_TRUE(axis.position.target(20, target));
    axis.position.submitted(1, 20, target);
    axis.position.acknowledge(1, target);
    axis.encoder = target - 35;
    for (int i = 0; i < 5; ++i) {
        axis.encoder += 14; // ~1RPM, within the former2mm/s quiet threshold.
        axis.sample();
        TEST_ASSERT_TRUE(axis.position.running(axis.now));
        TEST_ASSERT_FALSE(axis.position.rebase(0, axis.now));
    }
}

void test_stale_epoch_and_old_snapshots_cannot_restore_a_reference() {
    Axis axis;
    axis.anchor();
    axis.position.observe({axis.sequence - 1, axis.now - 20,
                           axis.continuity, INT32_MAX}, axis.now);
    TEST_ASSERT_EQUAL_INT32(0, axis.position.current());
    TEST_ASSERT_TRUE(axis.position.anchored());
    TEST_ASSERT_FALSE(axis.position.observe({axis.sequence + 1, axis.now,
                                            axis.continuity, axis.encoder}, axis.now + 101));
    ++axis.continuity;
    axis.sample();
    TEST_ASSERT_FALSE(axis.position.anchored());
    TEST_ASSERT_TRUE(axis.position.running(axis.now));
    axis.sample();
    TEST_ASSERT_FALSE(axis.position.anchored()); // No automatic rebase.
    TEST_ASSERT_TRUE(axis.position.rebase(0, axis.now)); // Explicit new home.
}

void test_absolute_target_overflow_is_rejected_instead_of_wrapped() {
    Axis axis;
    axis.encoder = INT32_MAX - 100;
    axis.anchor();
    int32_t target;
    TEST_ASSERT_FALSE(axis.position.target(100, target));
    TEST_ASSERT_TRUE(axis.position.target(-100, target));
    TEST_ASSERT_EQUAL_INT32(INT32_MAX - 4196, target);
}

void test_serial_speed_units_never_select_no_ramp_or_round_speed_up() {
    uint16_t rpm, acceleration;
    TEST_ASSERT_TRUE(serial_axis::speedRegisters(500, 20000, 800, 1500, rpm, acceleration));
    TEST_ASSERT_EQUAL_UINT16(37, rpm);
    TEST_ASSERT_EQUAL_UINT16(1500, acceleration);
    TEST_ASSERT_TRUE(serial_axis::speedRegisters(20000, 1000000, 800, 1500, rpm, acceleration));
    TEST_ASSERT_EQUAL_UINT16(1500, rpm);
    TEST_ASSERT_EQUAL_UINT16(59999, acceleration);
    TEST_ASSERT_FALSE(serial_axis::speedRegisters(1, 20000, 800, 1500, rpm, acceleration));
    TEST_ASSERT_FALSE(serial_axis::speedRegisters(500, 0, 800, 1500, rpm, acceleration));
}

void test_encoder_targets_respect_both_physical_bounds_and_margin() {
    TEST_ASSERT_TRUE(serial_axis::insideBounds(6000, 0, 100000, 1000));
    TEST_ASSERT_TRUE(serial_axis::insideBounds(94000, 100000, 0, 1000));
    TEST_ASSERT_FALSE(serial_axis::insideBounds(5999, 0, 100000, 1000));
    TEST_ASSERT_FALSE(serial_axis::insideBounds(94001, 0, 100000, 1000));
    TEST_ASSERT_FALSE(serial_axis::insideBounds(5000, 0, 10000, 1000));
    TEST_ASSERT_FALSE(serial_axis::insideBounds(INT32_MAX, INT32_MAX - 10000,
                                               INT32_MAX, 1000));
}

void test_trajectory_advances_only_when_transport_acknowledges_proposal() {
    serial_axis::Trajectory trajectory;
    trajectory.reset(0);
    trajectory.target(100);
    serial_axis::Trajectory::Proposal first, retry;
    TEST_ASSERT_TRUE(trajectory.propose(0, 20, 100, 1000, 20, first));
    TEST_ASSERT_DOUBLE_WITHIN(0.000001, 0, trajectory.position());
    TEST_ASSERT_TRUE(trajectory.propose(0, 20, 100, 1000, 20, retry));
    TEST_ASSERT_DOUBLE_WITHIN(0.000001, first.position, retry.position);
    trajectory.commit(first);
    TEST_ASSERT_DOUBLE_WITHIN(0.000001, first.position, trajectory.position());
}

void test_trajectory_has_no_catchup_jump_after_scheduler_or_uart_delay() {
    serial_axis::Trajectory trajectory;
    trajectory.reset(0);
    trajectory.target(1000);
    serial_axis::Trajectory::Proposal normal, delayed;
    TEST_ASSERT_TRUE(trajectory.propose(0, 20, 100, 1000, 20, normal));
    TEST_ASSERT_TRUE(trajectory.propose(0, 20, 100, 1000, 5000, delayed));
    TEST_ASSERT_DOUBLE_WITHIN(0.000001, normal.position, delayed.position);
    TEST_ASSERT_DOUBLE_WITHIN(0.000001, normal.velocity, delayed.velocity);
}

void test_following_error_blocks_advance_and_each_waypoint_is_bounded() {
    serial_axis::Trajectory trajectory;
    trajectory.reset(0);
    trajectory.target(10000);
    serial_axis::Trajectory::Proposal proposal;
    TEST_ASSERT_FALSE(trajectory.propose(-21, 20, 10000, 1000000, 20, proposal));
    TEST_ASSERT_DOUBLE_WITHIN(0.000001, 0, trajectory.position());
    TEST_ASSERT_TRUE(trajectory.propose(0, 20, 10000, 1000000, 20, proposal));
    TEST_ASSERT_TRUE(proposal.position <= 20);
    TEST_ASSERT_TRUE(proposal.position >= -20);
}

void test_trajectory_reaches_final_target_with_physical_following_and_retargets() {
    serial_axis::Trajectory trajectory;
    trajectory.reset(0);
    trajectory.target(100);
    double physical = 0;
    for (int i = 0; i < 1000 && trajectory.active(); ++i) {
        serial_axis::Trajectory::Proposal proposal;
        TEST_ASSERT_TRUE(trajectory.propose(physical, 20, 100, 1000, 20, proposal));
        trajectory.commit(proposal);
        physical = proposal.position;
    }
    TEST_ASSERT_FALSE(trajectory.active());
    TEST_ASSERT_DOUBLE_WITHIN(0.000001, 100, trajectory.position());
    trajectory.target(-100);
    for (int i = 0; i < 1000 && trajectory.active(); ++i) {
        serial_axis::Trajectory::Proposal proposal;
        TEST_ASSERT_TRUE(trajectory.propose(physical, 20, 100, 1000, 20, proposal));
        trajectory.commit(proposal);
        physical = proposal.position;
    }
    TEST_ASSERT_FALSE(trajectory.active());
    TEST_ASSERT_DOUBLE_WITHIN(0.000001, -100, trajectory.position());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_absolute_targets_use_encoder_units_and_keep_command_separate);
    RUN_TEST(test_arrival_requires_ack_and_unique_stopped_physical_samples);
    RUN_TEST(test_rebase_and_direction_change_do_not_move_the_encoder_origin);
    RUN_TEST(test_active_or_moving_axis_cannot_be_rebased);
    RUN_TEST(test_mode_frame_mirrors_preserve_anchor_without_rounding_drift);
    RUN_TEST(test_one_rpm_motion_is_not_reported_as_stopped_or_arrived);
    RUN_TEST(test_stale_epoch_and_old_snapshots_cannot_restore_a_reference);
    RUN_TEST(test_absolute_target_overflow_is_rejected_instead_of_wrapped);
    RUN_TEST(test_serial_speed_units_never_select_no_ramp_or_round_speed_up);
    RUN_TEST(test_encoder_targets_respect_both_physical_bounds_and_margin);
    RUN_TEST(test_trajectory_advances_only_when_transport_acknowledges_proposal);
    RUN_TEST(test_trajectory_has_no_catchup_jump_after_scheduler_or_uart_delay);
    RUN_TEST(test_following_error_blocks_advance_and_each_waypoint_is_bounded);
    RUN_TEST(test_trajectory_reaches_final_target_with_physical_following_and_retargets);
    return UNITY_END();
}
