#include <unity.h>
#include "homing_session.h"

using homing_session::Gate;
using homing_session::Ticket;

void setUp() {}
void tearDown() {}

// These schedules model the blocking boundaries that caused the physical
// late-start incident. The final commit uses the same gate as production's
// locked arm/state transitions; foreground checks are intentionally omitted.
struct Effects {
    unsigned arms = 0;
    unsigned enables = 0;
    unsigned moves = 0;
    unsigned publications = 0;
    bool persistedArm = false;
};

void test_stop_while_task_creation_waits_does_not_adopt_a_new_ticket() {
    Gate gate;
    Effects effects;
    const Ticket createdIntent = gate.begin();
    gate.cancel();  // task allocation retry is still asleep
    const Ticket newHome = gate.begin();
    TEST_ASSERT_FALSE(gate.commit(createdIntent, [&] { ++effects.arms; return true; }));
    TEST_ASSERT_EQUAL_UINT(0, effects.arms);
    TEST_ASSERT_TRUE(gate.accepts(newHome));
}

void test_stop_during_mqtt_shutdown_blocks_late_arm_enable_and_move() {
    Gate gate;
    Effects effects;
    const auto task = gate.begin();
    gate.cancel();  // SDK shutdown returns after Stop was acknowledged
    TEST_ASSERT_FALSE(gate.commit(task, [&] { ++effects.arms; return true; }));
    TEST_ASSERT_FALSE(gate.commit(task, [&] { ++effects.enables; return true; }));
    TEST_ASSERT_FALSE(gate.commit(task, [&] { ++effects.moves; return true; }));
    TEST_ASSERT_EQUAL_UINT(0, effects.arms + effects.enables + effects.moves);
}

void test_stop_during_nvs_write_blocks_the_final_arm_transition() {
    Gate gate;
    Effects effects;
    const auto task = gate.begin();
    TEST_ASSERT_TRUE(gate.accepts(task));  // optimistic preflight succeeded
    effects.persistedArm = true;  // crash marker completed, potentially slowly
    gate.cancel();
    TEST_ASSERT_FALSE(gate.commit(task, [&] { ++effects.arms; return true; }));
    TEST_ASSERT_TRUE(effects.persistedArm);
    TEST_ASSERT_EQUAL_UINT(0, effects.arms);
}

void test_stop_between_arm_enable_and_move_requires_each_gate() {
    for (unsigned boundary = 0; boundary < 2; ++boundary) {
        Gate gate;
        Effects effects;
        const auto task = gate.begin();
        TEST_ASSERT_TRUE(gate.commit(task, [&] { ++effects.arms; return true; }));
        if (boundary == 0) gate.cancel();
        gate.commit(task, [&] { ++effects.enables; return true; });
        if (boundary == 1) gate.cancel();
        TEST_ASSERT_FALSE(gate.commit(task, [&] { ++effects.moves; return true; }));
        TEST_ASSERT_EQUAL_UINT(boundary, effects.enables);
        TEST_ASSERT_EQUAL_UINT(0, effects.moves);
    }
}

void test_legacy_adc_wait_cannot_bypass_cancellation() {
    Gate gate;
    Effects effects;
    const auto legacyTask = gate.begin();
    gate.cancel();  // no HAT: the ADC offset average was in progress
    TEST_ASSERT_FALSE(gate.commit(legacyTask, [&] { ++effects.publications; return true; }));
    TEST_ASSERT_FALSE(gate.commit(legacyTask, [&] { ++effects.moves; return true; }));
    TEST_ASSERT_EQUAL_UINT(0, effects.publications + effects.moves);
}

void test_new_home_waits_for_all_previous_command_flushes() {
    Gate gate;
    const auto oldTask = gate.begin();
    gate.beginCancel();  // ENA is already HIGH; a move submission is returning
    const auto newTask = gate.begin();
    TEST_ASSERT_FALSE(gate.accepts(oldTask));
    TEST_ASSERT_TRUE(gate.accepts(newTask));
    TEST_ASSERT_FALSE(gate.commandsReady());
    gate.beginCancel();  // a second Stop must not release the first flush
    const auto newestTask = gate.begin();
    gate.finishCancel();
    TEST_ASSERT_FALSE(gate.commandsReady());
    gate.finishCancel();
    TEST_ASSERT_TRUE(gate.commandsReady());
    TEST_ASSERT_FALSE(gate.accepts(newTask));
    TEST_ASSERT_TRUE(gate.accepts(newestTask));
}

void test_old_pass_completion_and_capture_cannot_publish_into_a_new_home() {
    Gate gate;
    Effects effects;
    const auto passOne = gate.begin();
    gate.cancel();
    const auto nextHome = gate.begin();
    // Both pass2 admission and delayed feedback publication carry passOne.
    TEST_ASSERT_FALSE(gate.commit(passOne, [&] { ++effects.arms; return true; }));
    TEST_ASSERT_FALSE(gate.commit(passOne, [&] { ++effects.publications; return true; }));
    TEST_ASSERT_EQUAL_UINT(0, effects.arms + effects.publications);
    TEST_ASSERT_TRUE(gate.accepts(nextHome));
}

void test_fault_reset_never_revives_the_pre_fault_ticket() {
    Gate gate;
    const auto beforeFault = gate.begin();
    gate.cancel();  // fault observer
    gate.beginCancel();  // reviewed reset invalidates again before its waits
    gate.finishCancel();
    TEST_ASSERT_FALSE(gate.accepts(beforeFault));
    TEST_ASSERT_FALSE(gate.active());
    const auto explicitlyRequestedHome = gate.begin();
    TEST_ASSERT_TRUE(gate.accepts(explicitlyRequestedHome));
    TEST_ASSERT_FALSE(gate.accepts(beforeFault));
}

void test_both_valid_passes_share_intent_and_completion_closes_it() {
    Gate gate;
    Effects effects;
    const auto firstPass = gate.begin();
    const auto secondPass = firstPass;  // a continuation never mints a ticket
    TEST_ASSERT_TRUE(gate.commit(firstPass, [&] { ++effects.moves; return true; }));
    TEST_ASSERT_TRUE(gate.commit(secondPass, [&] { ++effects.moves; return true; }));
    TEST_ASSERT_TRUE(gate.commit(secondPass, [&] {
        ++effects.publications;
        gate.cancel();  // atomic successful home publication and session close
        return true;
    }));
    TEST_ASSERT_EQUAL_UINT(2, effects.moves);
    TEST_ASSERT_EQUAL_UINT(1, effects.publications);
    TEST_ASSERT_FALSE(gate.accepts(firstPass));
    TEST_ASSERT_FALSE(gate.active());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_stop_while_task_creation_waits_does_not_adopt_a_new_ticket);
    RUN_TEST(test_stop_during_mqtt_shutdown_blocks_late_arm_enable_and_move);
    RUN_TEST(test_stop_during_nvs_write_blocks_the_final_arm_transition);
    RUN_TEST(test_stop_between_arm_enable_and_move_requires_each_gate);
    RUN_TEST(test_legacy_adc_wait_cannot_bypass_cancellation);
    RUN_TEST(test_new_home_waits_for_all_previous_command_flushes);
    RUN_TEST(test_old_pass_completion_and_capture_cannot_publish_into_a_new_home);
    RUN_TEST(test_fault_reset_never_revives_the_pre_fault_ticket);
    RUN_TEST(test_both_valid_passes_share_intent_and_completion_closes_it);
    return UNITY_END();
}
