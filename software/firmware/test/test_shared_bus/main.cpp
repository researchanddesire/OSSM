#include <unity.h>
#include "shared_bus_logic.h"

using shared_bus::Interlock;
using shared_bus::Observation;

void setUp(void) {}
void tearDown(void) {}

namespace {

// The Test Rig arrangement: the PC guard polls the drive and the HAT listens.
Observation guardedStepDir() {
    Observation seen;
    seen.hatDetected = true;
    seen.receiveOnly = true;
    seen.telemetryFresh = true;
    seen.controlsFresh = true;
    seen.modbusEnable = 0;
    return seen;
}

}  // namespace

void test_board_without_hat_is_not_shared(void) {
    Observation seen;
    TEST_ASSERT_FALSE(shared_bus::active(seen));
    TEST_ASSERT_EQUAL(static_cast<int>(Interlock::NotShared),
                      static_cast<int>(shared_bus::interlock(seen)));
    TEST_ASSERT_FALSE(shared_bus::pulsesAllowed(seen));
}

void test_hat_that_owns_the_bus_is_not_shared(void) {
    auto seen = guardedStepDir();
    seen.receiveOnly = false;
    TEST_ASSERT_FALSE(shared_bus::active(seen));
    TEST_ASSERT_FALSE(shared_bus::pulsesAllowed(seen));
}

void test_other_master_before_detection_is_not_shared(void) {
    auto seen = guardedStepDir();
    seen.hatDetected = false;
    TEST_ASSERT_FALSE(shared_bus::active(seen));
    TEST_ASSERT_FALSE(shared_bus::pulsesAllowed(seen));
}

void test_guarded_step_dir_drive_allows_pulses(void) {
    const auto seen = guardedStepDir();
    TEST_ASSERT_TRUE(shared_bus::active(seen));
    TEST_ASSERT_EQUAL(static_cast<int>(Interlock::Ready),
                      static_cast<int>(shared_bus::interlock(seen)));
    TEST_ASSERT_TRUE(shared_bus::pulsesAllowed(seen));
}

void test_selected_serial_backend_never_returns_to_pulses(void) {
    auto seen = guardedStepDir();
    seen.serialSelected = true;
    TEST_ASSERT_FALSE(shared_bus::active(seen));
    TEST_ASSERT_FALSE(shared_bus::pulsesAllowed(seen));
}

void test_silent_telemetry_blocks_pulses(void) {
    auto seen = guardedStepDir();
    seen.telemetryFresh = false;
    TEST_ASSERT_TRUE(shared_bus::active(seen));
    TEST_ASSERT_EQUAL(static_cast<int>(Interlock::MonitorSilent),
                      static_cast<int>(shared_bus::interlock(seen)));
    TEST_ASSERT_FALSE(shared_bus::pulsesAllowed(seen));
}

void test_missing_control_reads_block_pulses(void) {
    // Telemetry alone omits register 0 and cannot show the control mode.
    auto seen = guardedStepDir();
    seen.controlsFresh = false;
    TEST_ASSERT_EQUAL(static_cast<int>(Interlock::MonitorSilent),
                      static_cast<int>(shared_bus::interlock(seen)));
    TEST_ASSERT_FALSE(shared_bus::pulsesAllowed(seen));
}

void test_stale_controls_are_not_trusted_for_their_value(void) {
    auto seen = guardedStepDir();
    seen.controlsFresh = false;
    seen.modbusEnable = 1;
    TEST_ASSERT_EQUAL(static_cast<int>(Interlock::MonitorSilent),
                      static_cast<int>(shared_bus::interlock(seen)));
}

void test_drive_in_serial_control_blocks_pulses(void) {
    // The PC guard stops the motor by taking it out of STEP/DIR control.
    auto seen = guardedStepDir();
    seen.modbusEnable = 1;
    TEST_ASSERT_EQUAL(static_cast<int>(Interlock::DriveNotStepDir),
                      static_cast<int>(shared_bus::interlock(seen)));
    TEST_ASSERT_FALSE(shared_bus::pulsesAllowed(seen));
}

void test_unread_control_register_blocks_pulses(void) {
    auto seen = guardedStepDir();
    seen.modbusEnable = UINT16_MAX;
    TEST_ASSERT_FALSE(shared_bus::pulsesAllowed(seen));
}

void test_every_shared_state_has_a_reason(void) {
    TEST_ASSERT_NOT_EQUAL(0, shared_bus::reason(Interlock::Ready)[0]);
    TEST_ASSERT_NOT_EQUAL(0, shared_bus::reason(Interlock::MonitorSilent)[0]);
    TEST_ASSERT_NOT_EQUAL(0, shared_bus::reason(Interlock::DriveNotStepDir)[0]);
    TEST_ASSERT_EQUAL(0, shared_bus::reason(Interlock::NotShared)[0]);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_board_without_hat_is_not_shared);
    RUN_TEST(test_hat_that_owns_the_bus_is_not_shared);
    RUN_TEST(test_other_master_before_detection_is_not_shared);
    RUN_TEST(test_guarded_step_dir_drive_allows_pulses);
    RUN_TEST(test_selected_serial_backend_never_returns_to_pulses);
    RUN_TEST(test_silent_telemetry_blocks_pulses);
    RUN_TEST(test_missing_control_reads_block_pulses);
    RUN_TEST(test_stale_controls_are_not_trusted_for_their_value);
    RUN_TEST(test_drive_in_serial_control_blocks_pulses);
    RUN_TEST(test_unread_control_register_blocks_pulses);
    RUN_TEST(test_every_shared_state_has_a_reason);
    return UNITY_END();
}
