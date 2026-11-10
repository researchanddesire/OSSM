#include <unity.h>

#include "motion_owner.h"

void setUp() {}
void tearDown() {}

namespace {
uint64_t currentRevision = 0;
bool current(uint64_t revision) { return revision == currentRevision; }
}

void test_unconfigured_standalone_motion_remains_compatible() {
    motion_owner::Gate gate;
    const auto owner = gate.snapshot();
    TEST_ASSERT_TRUE(gate.matches(owner));
    TEST_ASSERT_TRUE(motion_owner::authorized(owner));
}

void test_revoked_worker_cannot_borrow_a_new_revision() {
    motion_owner::Gate gate;
    currentRevision = 7;
    gate.bind(7, current);
    const auto old = gate.snapshot();
    TEST_ASSERT_TRUE(motion_owner::authorized(old));
    currentRevision = 8; // Stop revokes before any new session can enable.
    TEST_ASSERT_FALSE(motion_owner::authorized(old));
    gate.bind(8, current);
    TEST_ASSERT_FALSE(gate.matches(old));
    TEST_ASSERT_TRUE(motion_owner::authorized(gate.snapshot()));
    TEST_ASSERT_FALSE(motion_owner::authorized(old));
}

void test_rebinding_invalidates_already_computed_profiles() {
    motion_owner::Gate gate;
    currentRevision = 0; // Epoch0 is a valid initial epoch, never a wildcard.
    gate.bind(0, current);
    const auto computed = gate.snapshot();
    gate.bind(0, current);
    TEST_ASSERT_FALSE(gate.matches(computed));
    TEST_ASSERT_TRUE(gate.matches(gate.snapshot()));
    currentRevision = 1;
    TEST_ASSERT_FALSE(motion_owner::authorized(computed));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_unconfigured_standalone_motion_remains_compatible);
    RUN_TEST(test_revoked_worker_cannot_borrow_a_new_revision);
    RUN_TEST(test_rebinding_invalidates_already_computed_profiles);
    return UNITY_END();
}
