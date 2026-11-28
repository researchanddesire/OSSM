#include <unity.h>

#include <string>

#include "services/communication/motion_link.h"
#include "services/communication/disconnect_speed.h"

using motion_link::isMotionCommand;
using motion_link::kNoConnection;
using motion_link::Owner;

void setUp() {}
void tearDown() {}

void test_motion_commands() {
    const char *motion[] = {"go:strokeEngine", "go:simplePenetration",
                            "go:streaming",    "set:speed:10",
                            "set:pattern:2",   "set:stroke:0",
                            "stream:50:100"};
    for (const char *command : motion) {
        TEST_ASSERT_TRUE_MESSAGE(isMotionCommand(command), command);
    }
    const char *notMotion[] = {"go:menu",  "go:restart", "go:update",
                               "go:pairing", "set:wifi:a|b", "", "garbage"};
    for (const char *command : notMotion) {
        TEST_ASSERT_FALSE_MESSAGE(isMotionCommand(command), command);
    }
}

void test_last_link_leaving_always_stops() {
    Owner owner;
    TEST_ASSERT_TRUE(owner.onDisconnect(3, 0));  // nobody ever commanded
    owner.onMotionCommand(1);
    TEST_ASSERT_TRUE(owner.onDisconnect(2, 0));
    TEST_ASSERT_EQUAL(kNoConnection, owner.owner());
}

void test_radr_dropping_while_app_stays_stops_motion() {
    // P5 on the Test Rig: RADR (1) drives, the RAD App (2) watches.
    Owner owner;
    owner.onMotionCommand(1);
    TEST_ASSERT_TRUE(owner.onDisconnect(1, 1));
    TEST_ASSERT_EQUAL(kNoConnection, owner.owner());
    // The app leaving later stops again (harmless: already stopped).
    TEST_ASSERT_TRUE(owner.onDisconnect(2, 0));
}

void test_watcher_leaving_does_not_stop_the_driver() {
    Owner owner;
    owner.onMotionCommand(1);
    TEST_ASSERT_FALSE(owner.onDisconnect(2, 1));
    TEST_ASSERT_EQUAL(1, owner.owner());
}

void test_latest_commander_owns_motion() {
    Owner owner;
    owner.onMotionCommand(1);
    owner.onMotionCommand(2);  // the app took over
    TEST_ASSERT_FALSE(owner.onDisconnect(1, 1));
    TEST_ASSERT_TRUE(owner.onDisconnect(2, 0));
}

void test_unattributed_motion_clears_ownership() {
    // RAD BLE requests do not say which link sent them: a RADR dropping
    // after the app drove motion over RAD BLE must not stop the app.
    Owner owner;
    owner.onMotionCommand(1);
    owner.onMotionCommand(kNoConnection);
    TEST_ASSERT_FALSE(owner.onDisconnect(1, 1));
    TEST_ASSERT_TRUE(owner.onDisconnect(2, 0));
}

void test_reconnected_handle_is_a_new_link() {
    // NimBLE reuses connection handles: after the owner left, a new link on
    // the same handle owns nothing until it commands motion.
    Owner owner;
    owner.onMotionCommand(1);
    TEST_ASSERT_TRUE(owner.onDisconnect(1, 1));
    TEST_ASSERT_FALSE(owner.onDisconnect(1, 1));
}

void test_disconnect_preserves_explicit_zero_despite_old_effective_speed() {
    motion_link::DisconnectSpeed ramp;
    const uint32_t dropped = 5000;
    // Real reported state at a stopped owner disconnect: BLE 0, knob 0,
    // effective 82. An old ramp from 82 would have written 25 by 2,252 ms.
    TEST_ASSERT_FALSE(ramp.start(dropped, 82, 0));
    std::optional<float> remote = 0;
    for (uint32_t ms = dropped; ms < dropped + 13000; ++ms) {
        if (auto value = ramp.next(ms, remote)) remote = *value;
        TEST_ASSERT_EQUAL_FLOAT(0, *remote);
    }
    TEST_ASSERT_FALSE(ramp.active());
    TEST_ASSERT_FALSE(ramp.start(dropped, 82, std::nullopt));
    TEST_ASSERT_FALSE(ramp.next(dropped + 2100, std::nullopt));
    const auto stopped = ramp.start(dropped, 0, 82);
    TEST_ASSERT_TRUE(stopped.has_value());
    TEST_ASSERT_EQUAL_FLOAT(0, *stopped);
}

void test_positive_shutdown_survives_observers_until_new_drive_or_zero() {
    motion_link::DisconnectSpeed ramp;
    Owner owner;
    owner.onMotionCommand(7);
    TEST_ASSERT_TRUE(owner.onDisconnect(7, 1));
    const uint32_t start = UINT32_MAX - 1200;
    std::optional<float> remote = 82;
    TEST_ASSERT_FALSE(ramp.start(start, 82, remote));
    TEST_ASSERT_FALSE(ramp.next(start + 999, remote));
    // Passive connect, reads and an unrelated observer disconnect have no
    // ownership. A last observer leaving cannot restart the original wait.
    TEST_ASSERT_FALSE(owner.onDisconnect(9, 1));
    TEST_ASSERT_FALSE(ramp.start(start + 1400, 82, remote));
    for (unsigned ms = 1000; ms <= 3000; ms += 50) {
        if (auto value = ramp.next(start + ms, remote)) {
            TEST_ASSERT_TRUE(*value <= *remote && *value >= 0);
            remote = *value;
        }
        if (ms == 2250) TEST_ASSERT_EQUAL_FLOAT(25, *remote);
    }
    TEST_ASSERT_EQUAL_FLOAT(0, *remote);
    TEST_ASSERT_FALSE(ramp.active());
    ramp.start(100, 82, 35);
    const auto lower = ramp.next(2100, 35);
    TEST_ASSERT_TRUE(lower && *lower <= 17);
    TEST_ASSERT_FALSE(ramp.next(2200, 0));
    TEST_ASSERT_FALSE(ramp.active());
    ramp.start(100, 35, 35);
    owner.onMotionCommand(9);
    ramp.cancel();
    TEST_ASSERT_FALSE(ramp.next(2400, 65));
    TEST_ASSERT_EQUAL_UINT(9, owner.owner());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_motion_commands);
    RUN_TEST(test_disconnect_preserves_explicit_zero_despite_old_effective_speed);
    RUN_TEST(test_positive_shutdown_survives_observers_until_new_drive_or_zero);
    RUN_TEST(test_last_link_leaving_always_stops);
    RUN_TEST(test_radr_dropping_while_app_stays_stops_motion);
    RUN_TEST(test_watcher_leaving_does_not_stop_the_driver);
    RUN_TEST(test_latest_commander_owns_motion);
    RUN_TEST(test_unattributed_motion_clears_ownership);
    RUN_TEST(test_reconnected_handle_is_a_new_link);
    return UNITY_END();
}
