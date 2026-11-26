#include <unity.h>
#include "encoder_click.h"

void setUp() {}
void tearDown() {}

void test_click_invalidates_prepress_and_held_reads() {
    EncoderClick click;
    const auto before = click.snapshot();
    TEST_ASSERT_TRUE(click.unchangedAndIdle(before));
    click.update(false);
    const auto held = click.snapshot();
    TEST_ASSERT_FALSE(click.unchangedAndIdle(before));
    TEST_ASSERT_FALSE(click.unchangedAndIdle(held));
    click.update(false);
    TEST_ASSERT_EQUAL_UINT32(held, click.snapshot());
    click.update(true);
    TEST_ASSERT_FALSE(click.unchangedAndIdle(held));
    TEST_ASSERT_TRUE(click.unchangedAndIdle(click.snapshot()));
}

void test_completed_click_between_screen_polls_is_visible() {
    EncoderClick click;
    const auto before = click.snapshot();
    click.update(false);
    click.update(true);
    TEST_ASSERT_FALSE(click.unchangedAndIdle(before));
    const auto released = click.snapshot();
    click.update(true);
    TEST_ASSERT_TRUE(click.unchangedAndIdle(released));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_click_invalidates_prepress_and_held_reads);
    RUN_TEST(test_completed_click_between_screen_polls_is_visible);
    return UNITY_END();
}
