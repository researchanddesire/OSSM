#include <unity.h>

#include "motor_feedback_logic.h"
#include "position_telemetry_logic.h"

using position_telemetry_logic::extensionMm;

void setUp() {}
void tearDown() {}

void test_home_reads_zero_extension_in_both_frames() {
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 0.0f, extensionMm(0.0f, false, 6.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 0.0f, extensionMm(-6.0f, true, 6.0f));
}

void test_extension_is_positive_in_both_frames() {
    // 100 mm out: Native counts it negative, StrokeEngine as 100 - keepout.
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 100.0f,
                             extensionMm(-100.0f, false, 6.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 100.0f,
                             extensionMm(94.0f, true, 6.0f));
}

void test_extension_is_unchanged_by_a_frame_translation() {
    // stepperTranslateFrame maps a counter c to -(c + keepout), both ways.
    const float keepout = 6.0f;
    const float nativePositions[] = {0.0f, -0.05f, -37.5f, -250.0f, 4.0f};
    for (const float native : nativePositions) {
        const float strokeEngine = -(native + keepout);
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, extensionMm(native, false, keepout),
                                 extensionMm(strokeEngine, true, keepout));
    }
}

void test_extension_reverses_the_feedback_frame_conversion() {
    // Measured feedback reaches getMotorPositionMm() through
    // positionInFrameMm, whose native input counts extension negative.
    const bool frames[] = {false, true};
    for (const bool strokeEngineFrame : frames) {
        const float frameMm = static_cast<float>(
            motor_feedback_logic::positionInFrameMm(-80.0, strokeEngineFrame,
                                                    6.0));
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, 80.0f,
                                 extensionMm(frameMm, strokeEngineFrame, 6.0f));
    }
}

void test_positions_behind_home_read_negative() {
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, -2.0f, extensionMm(2.0f, false, 6.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, -2.0f, extensionMm(-8.0f, true, 6.0f));
}

void test_keepout_applies_only_to_the_stroke_engine_frame() {
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 10.0f, extensionMm(-10.0f, false, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 10.0f, extensionMm(-10.0f, false, 6.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 10.0f, extensionMm(10.0f, true, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 16.0f, extensionMm(10.0f, true, 6.0f));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_home_reads_zero_extension_in_both_frames);
    RUN_TEST(test_extension_is_positive_in_both_frames);
    RUN_TEST(test_extension_is_unchanged_by_a_frame_translation);
    RUN_TEST(test_extension_reverses_the_feedback_frame_conversion);
    RUN_TEST(test_positions_behind_home_read_negative);
    RUN_TEST(test_keepout_applies_only_to_the_stroke_engine_frame);
    return UNITY_END();
}
