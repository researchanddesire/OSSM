#include <unity.h>

#include <climits>

#include "motor_feedback_logic.h"

void setUp() {}
void tearDown() {}

void test_encoder_wrap_in_both_directions() {
    TEST_ASSERT_EQUAL_INT64(16, motor_feedback_logic::encoderDelta(
                                   INT32_MIN + 7, INT32_MAX - 8));
    TEST_ASSERT_EQUAL_INT64(-16, motor_feedback_logic::encoderDelta(
                                    INT32_MAX - 8, INT32_MIN + 7));
    TEST_ASSERT_EQUAL_INT64(-150000, motor_feedback_logic::encoderDelta(
                                        -100000, 50000));
}

void test_encoder_units_are_independent_of_electronic_gear() {
    TEST_ASSERT_DOUBLE_WITHIN(0.00001, 40.0,
        motor_feedback_logic::nativePositionMm(42768, 10000, 1, 40.0));
    TEST_ASSERT_DOUBLE_WITHIN(0.00001, -10.0,
        motor_feedback_logic::nativePositionMm(18192, 10000, -1, 40.0));
    TEST_ASSERT_DOUBLE_WITHIN(0.00001, 80.0,
        motor_feedback_logic::nativePositionMm(42768, 10000, 1, 80.0));
}

void test_learned_polarity_handles_both_driver_directions() {
    TEST_ASSERT_EQUAL_INT8(1, motor_feedback_logic::learnNativePolarity(
        10000, 18192, 0, 200, 800.0, 100.0));
    TEST_ASSERT_EQUAL_INT8(-1, motor_feedback_logic::learnNativePolarity(
        10000, 18192, 0, -200, 800.0, 100.0));
    TEST_ASSERT_EQUAL_INT8(-1, motor_feedback_logic::learnNativePolarity(
        10000, 1808, 200, 400, 800.0, 100.0));
}

void test_tiny_or_inconsistent_homing_travel_cannot_set_polarity() {
    TEST_ASSERT_EQUAL_INT8(0, motor_feedback_logic::learnNativePolarity(
        0, 2048, 0, 50, 800.0, 100.0));
    TEST_ASSERT_EQUAL_INT8(0, motor_feedback_logic::learnNativePolarity(
        0, 10, 0, 200, 800.0, 100.0));
    TEST_ASSERT_EQUAL_INT8(0, motor_feedback_logic::learnNativePolarity(
        0, 32768, 0, 200, 800.0, 100.0));
    TEST_ASSERT_EQUAL_INT8(0, motor_feedback_logic::learnNativePolarity(
        0, 8192, 0, 200, 0.0, 100.0));
}

void test_polarity_learning_handles_encoder_wrap() {
    TEST_ASSERT_EQUAL_INT8(-1, motor_feedback_logic::learnNativePolarity(
        INT32_MAX - 4095, INT32_MIN + 4096, 0, -200, 800.0, 100.0));
}

void test_native_and_stroke_engine_frames_keep_the_same_physical_home() {
    TEST_ASSERT_DOUBLE_WITHIN(0.00001, 0.0,
        motor_feedback_logic::positionInFrameMm(0.0, false, 6.0));
    TEST_ASSERT_DOUBLE_WITHIN(0.00001, -6.0,
        motor_feedback_logic::positionInFrameMm(0.0, true, 6.0));
    TEST_ASSERT_DOUBLE_WITHIN(0.00001, 94.0,
        motor_feedback_logic::positionInFrameMm(-100.0, true, 6.0));
    TEST_ASSERT_DOUBLE_WITHIN(0.00001, -100.0,
        motor_feedback_logic::positionInFrameMm(94.0, true, 6.0));
}

void test_hat_contact_requires_distinct_stationary_samples_and_dwell() {
    motor_feedback_logic::HatContactDetector detector;
    TEST_ASSERT_FALSE(detector.update(1, 100, 10000, 1.8f, 1.5f, 819.2));
    TEST_ASSERT_FALSE(detector.update(2, 110, 10003, 1.8f, 1.5f, 819.2));
    TEST_ASSERT_FALSE(detector.update(3, 120, 10002, 1.8f, 1.5f, 819.2));
    TEST_ASSERT_TRUE(detector.update(4, 130, 10001, 1.8f, 1.5f, 819.2));
}

void test_cached_samples_cannot_build_a_contact() {
    motor_feedback_logic::HatContactDetector detector;
    TEST_ASSERT_FALSE(detector.update(1, 100, 0, 2.0f, 1.5f, 819.2));
    TEST_ASSERT_FALSE(detector.update(1, 150, 0, 2.0f, 1.5f, 819.2));
    TEST_ASSERT_FALSE(detector.update(1, 190, 0, 2.0f, 1.5f, 819.2));
    TEST_ASSERT_FALSE(detector.update(2, 195, 0, 2.0f, 1.5f, 819.2));
    TEST_ASSERT_TRUE(detector.update(3, 200, 0, 2.0f, 1.5f, 819.2));
}

void test_movement_and_low_current_restart_contact_dwell() {
    motor_feedback_logic::HatContactDetector detector;
    TEST_ASSERT_FALSE(detector.update(1, 100, 0, 2.0f, 1.5f, 800.0));
    TEST_ASSERT_FALSE(detector.update(2, 120, 100, 2.0f, 1.5f, 800.0));
    // Total variation reaches exactly 0.25 mm: reject the moving dwell.
    TEST_ASSERT_FALSE(detector.update(3, 140, 200, 2.0f, 1.5f, 800.0));
    TEST_ASSERT_FALSE(detector.update(4, 160, 200, 2.0f, 1.5f, 800.0));
    TEST_ASSERT_FALSE(detector.update(5, 180, 200, 1.0f, 1.5f, 800.0));
    TEST_ASSERT_FALSE(detector.update(6, 200, 200, 2.0f, 1.5f, 800.0));
    TEST_ASSERT_FALSE(detector.update(7, 220, 200, 2.0f, 1.5f, 800.0));
    TEST_ASSERT_TRUE(detector.update(8, 240, 200, 2.0f, 1.5f, 800.0));
}

void test_large_sample_gap_cannot_bridge_contact_dwell() {
    motor_feedback_logic::HatContactDetector detector;
    TEST_ASSERT_FALSE(detector.update(1, 100, 0, 2.0f, 1.5f, 819.2));
    TEST_ASSERT_FALSE(detector.update(2, 120, 0, 2.0f, 1.5f, 819.2));
    TEST_ASSERT_FALSE(detector.update(3, 221, 0, 2.0f, 1.5f, 819.2));
    TEST_ASSERT_FALSE(detector.update(4, 241, 0, 2.0f, 1.5f, 819.2));
    TEST_ASSERT_TRUE(detector.update(5, 261, 0, 2.0f, 1.5f, 819.2));
    TEST_ASSERT_FALSE(detector.update(6, 280, 0, 1.5f, 1.5f, 819.2));
}

void test_contact_dwell_handles_encoder_and_millis_wrap() {
    motor_feedback_logic::HatContactDetector detector;
    TEST_ASSERT_FALSE(detector.update(1, UINT32_MAX - 19,
                                     INT32_MAX - 3, 2.0f, 1.5f, 819.2));
    TEST_ASSERT_FALSE(detector.update(2, 0, INT32_MIN + 2,
                                     2.0f, 1.5f, 819.2));
    TEST_ASSERT_TRUE(detector.update(3, 20, INT32_MIN + 4,
                                    2.0f, 1.5f, 819.2));
    detector.reset();
    TEST_ASSERT_FALSE(detector.update(4, 40, INT32_MIN + 4,
                                     2.0f, 1.5f, 819.2));
}

void test_invalid_contact_inputs_cannot_latch() {
    motor_feedback_logic::HatContactDetector detector;
    TEST_ASSERT_FALSE(detector.update(0, 100, 0, 2.0f, 1.5f, 819.2));
    TEST_ASSERT_FALSE(detector.update(1, 120, 0, NAN, 1.5f, 819.2));
    TEST_ASSERT_FALSE(detector.update(2, 140, 0, 2.0f, -1.0f, 819.2));
    TEST_ASSERT_FALSE(detector.update(3, 160, 0, 2.0f, 1.5f, 0.0));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_encoder_wrap_in_both_directions);
    RUN_TEST(test_encoder_units_are_independent_of_electronic_gear);
    RUN_TEST(test_learned_polarity_handles_both_driver_directions);
    RUN_TEST(test_tiny_or_inconsistent_homing_travel_cannot_set_polarity);
    RUN_TEST(test_polarity_learning_handles_encoder_wrap);
    RUN_TEST(test_native_and_stroke_engine_frames_keep_the_same_physical_home);
    RUN_TEST(test_hat_contact_requires_distinct_stationary_samples_and_dwell);
    RUN_TEST(test_cached_samples_cannot_build_a_contact);
    RUN_TEST(test_movement_and_low_current_restart_contact_dwell);
    RUN_TEST(test_large_sample_gap_cannot_bridge_contact_dwell);
    RUN_TEST(test_contact_dwell_handles_encoder_and_millis_wrap);
    RUN_TEST(test_invalid_contact_inputs_cannot_latch);
    return UNITY_END();
}
