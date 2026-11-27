#include <unity.h>
#include "homing_logic.h"

void setUp(void) {}
void tearDown(void) {}

// ─── isCurrentOverLimit ───

void test_isCurrentOverLimit_above_threshold(void) {
    // reading=5.0, offset=1.0, threshold=3.0 → (5-1)=4 > 3 → true
    TEST_ASSERT_TRUE(homing_logic::isCurrentOverLimit(5.0f, 1.0f, 3.0f));
}

void test_isCurrentOverLimit_below_threshold(void) {
    // reading=3.0, offset=1.0, threshold=3.0 → (3-1)=2 > 3 → false
    TEST_ASSERT_FALSE(homing_logic::isCurrentOverLimit(3.0f, 1.0f, 3.0f));
}

void test_isCurrentOverLimit_exactly_at_threshold(void) {
    // reading=4.5, offset=1.0, threshold=3.5 → (4.5-1)=3.5 > 3.5 → false (not strictly greater)
    TEST_ASSERT_FALSE(homing_logic::isCurrentOverLimit(4.5f, 1.0f, 3.5f));
}

// ─── calculateMeasuredStroke ───

void test_calculateMeasuredStroke_positive_position(void) {
    // position=5000, max=10000 → 5000
    float result = homing_logic::calculateMeasuredStroke(5000, 10000.0f);
    TEST_ASSERT_EQUAL_FLOAT(5000.0f, result);
}

void test_calculateMeasuredStroke_negative_position(void) {
    // position=-5000, max=10000 → abs → 5000
    float result = homing_logic::calculateMeasuredStroke(-5000, 10000.0f);
    TEST_ASSERT_EQUAL_FLOAT(5000.0f, result);
}

void test_calculateMeasuredStroke_clamped_to_max(void) {
    // position=15000, max=10000 → clamped to 10000
    float result = homing_logic::calculateMeasuredStroke(15000, 10000.0f);
    TEST_ASSERT_EQUAL_FLOAT(10000.0f, result);
}

// ─── calculatePostHomingPosition ───

void test_final_home_park_uses_millimetres_from_retracted_reference(void) {
    TEST_ASSERT_EQUAL_INT32(-100, homing_logic::calculatePostHomingPosition(1, 5000, 12.5f, 8));
    TEST_ASSERT_EQUAL_INT32(-50, homing_logic::calculatePostHomingPosition(1, 5000, 0.5f, 100));
    TEST_ASSERT_EQUAL_INT32(0, homing_logic::calculatePostHomingPosition(1, 5000, 0.0f, 100));
}

void test_final_home_park_stays_in_calibrated_travel(void) {
    TEST_ASSERT_EQUAL_INT32(-800, homing_logic::calculatePostHomingPosition(1, 800, 100, 40));
    TEST_ASSERT_EQUAL_INT32(0, homing_logic::calculatePostHomingPosition(1, 5000, -1, 40));
    TEST_ASSERT_EQUAL_INT32(0, homing_logic::calculatePostHomingPosition(1, 5000, NAN, 40));
    TEST_ASSERT_EQUAL_INT32(0, homing_logic::calculatePostHomingPosition(1, NAN, 50, 40));
    TEST_ASSERT_EQUAL_INT32(0, homing_logic::calculatePostHomingPosition(1, 5000, 50, 0));
}

void test_first_pass_uses_measured_return_independent_of_park_setting(void) {
    TEST_ASSERT_EQUAL_INT32(5000, homing_logic::calculatePostHomingPosition(-1, 5000, 12.5f, 8));
}

// ─── isHomingTimedOut ───

void test_isHomingTimedOut_not_timed_out(void) {
    // 30000ms, timeout=40000 → false
    TEST_ASSERT_FALSE(homing_logic::isHomingTimedOut(30000, 40000));
}

void test_isHomingTimedOut_timed_out(void) {
    // 41000ms, timeout=40000 → true
    TEST_ASSERT_TRUE(homing_logic::isHomingTimedOut(41000, 40000));
}

// ─── isStrokeTooShortLogic ───

void test_isStrokeTooShort_above_min(void) {
    // measured=100, min=50 → false (100 > 50)
    TEST_ASSERT_FALSE(homing_logic::isStrokeTooShortLogic(100.0f, 50.0f));
}

void test_isStrokeTooShort_below_min(void) {
    // measured=30, min=50 → true (30 <= 50)
    TEST_ASSERT_TRUE(homing_logic::isStrokeTooShortLogic(30.0f, 50.0f));
}

void test_isStrokeTooShort_equal_to_min(void) {
    // measured=50, min=50 → true (50 <= 50, equal)
    TEST_ASSERT_TRUE(homing_logic::isStrokeTooShortLogic(50.0f, 50.0f));
}

void test_sensorless_seek_requires_contact_before_travel_ends(void) {
    using namespace homing_logic;
    // Current rise and default threshold from the reported missed end stop.
    TEST_ASSERT_TRUE(evaluateSensorlessSeek(false, true, 4.07f, 6) == SeekResult::Seeking);
    TEST_ASSERT_TRUE(evaluateSensorlessSeek(false, false, 4.07f, 6) == SeekResult::TravelEnded);
    TEST_ASSERT_TRUE(evaluateSensorlessSeek(false, false, 7, 6) == SeekResult::TravelEnded);
    TEST_ASSERT_TRUE(evaluateSensorlessSeek(false, true, 7, 6) == SeekResult::Contact);
    TEST_ASSERT_TRUE(evaluateSensorlessSeek(true, true, 7, 6) == SeekResult::TimedOut);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_isCurrentOverLimit_above_threshold);
    RUN_TEST(test_isCurrentOverLimit_below_threshold);
    RUN_TEST(test_isCurrentOverLimit_exactly_at_threshold);

    RUN_TEST(test_calculateMeasuredStroke_positive_position);
    RUN_TEST(test_calculateMeasuredStroke_negative_position);
    RUN_TEST(test_calculateMeasuredStroke_clamped_to_max);

    RUN_TEST(test_final_home_park_uses_millimetres_from_retracted_reference);
    RUN_TEST(test_final_home_park_stays_in_calibrated_travel);
    RUN_TEST(test_first_pass_uses_measured_return_independent_of_park_setting);

    RUN_TEST(test_sensorless_seek_requires_contact_before_travel_ends);
    RUN_TEST(test_isHomingTimedOut_not_timed_out);
    RUN_TEST(test_isHomingTimedOut_timed_out);

    RUN_TEST(test_isStrokeTooShort_above_min);
    RUN_TEST(test_isStrokeTooShort_below_min);
    RUN_TEST(test_isStrokeTooShort_equal_to_min);

    return UNITY_END();
}
