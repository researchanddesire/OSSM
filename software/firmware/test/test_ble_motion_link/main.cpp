#include <unity.h>

#include <string>

#include "services/communication/motion_link.h"

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

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_motion_commands);
    RUN_TEST(test_last_link_leaving_always_stops);
    RUN_TEST(test_radr_dropping_while_app_stays_stops_motion);
    RUN_TEST(test_watcher_leaving_does_not_stop_the_driver);
    RUN_TEST(test_latest_commander_owns_motion);
    RUN_TEST(test_unattributed_motion_clears_ownership);
    RUN_TEST(test_reconnected_handle_is_a_new_link);
    return UNITY_END();
}
