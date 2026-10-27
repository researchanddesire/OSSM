#include <unity.h>
#include "hat_logic.h"

void setUp(void) {}
void tearDown(void) {}

// 57AIM30: 32768 encoder counts per revolution, 800 STEP pulses per revolution.
static constexpr double kCountsPerPulse = 32768.0 / 800.0;

// ─── accumulatePulses ───

void test_accumulatePulses_counts_up_and_down(void) {
    TEST_ASSERT_EQUAL_INT32(25, hat_logic::accumulatePulses(0, 100, 125));
    TEST_ASSERT_EQUAL_INT32(-75, hat_logic::accumulatePulses(25, 125, 25));
}

void test_accumulatePulses_survives_16bit_wrap(void) {
    // 32760 → -32766 is +10 pulses through the int16 wrap.
    TEST_ASSERT_EQUAL_INT32(40010,
                            hat_logic::accumulatePulses(40000, 32760, -32766));
    // and back down through it.
    TEST_ASSERT_EQUAL_INT32(40000,
                            hat_logic::accumulatePulses(40010, -32766, 32760));
}

// ─── unit conversion ───

void test_pulsesToCounts_one_revolution(void) {
    TEST_ASSERT_EQUAL_INT32(32768, hat_logic::pulsesToCounts(800, kCountsPerPulse));
    TEST_ASSERT_EQUAL_INT32(-32768,
                            hat_logic::pulsesToCounts(-800, kCountsPerPulse));
}

void test_pulsesToCounts_reversed_servo(void) {
    TEST_ASSERT_EQUAL_INT32(-32768,
                            hat_logic::pulsesToCounts(800, -kCountsPerPulse));
}

void test_countsToPulses_round_trip(void) {
    const int32_t samples[] = {0, 1, -1, 7, 799, -12345, 20000};
    for (int32_t pulses : samples) {
        const int32_t counts = hat_logic::pulsesToCounts(pulses, kCountsPerPulse);
        TEST_ASSERT_EQUAL_INT32(pulses,
                                hat_logic::countsToPulses(counts, kCountsPerPulse));
    }
}

// ─── anchor / following error ───

void test_anchorAt_holds_measured_position(void) {
    // Whatever the pulse total, the anchored spot has no following error.
    const int32_t anchor = hat_logic::anchorAt(17026, -4321, kCountsPerPulse);
    TEST_ASSERT_EQUAL_INT32(
        0, hat_logic::followingErrorPulses(17026, anchor, -4321, kCountsPerPulse));
}

void test_followingErrorPulses_tracks_pulses_from_anchor(void) {
    const int32_t anchor = hat_logic::anchorAt(1000, 0, kCountsPerPulse);
    // 20 pulses = 1 mm = 819.2 counts on a 40 mm pulley.
    TEST_ASSERT_EQUAL_INT32(
        0, hat_logic::followingErrorPulses(1819, anchor, 20, kCountsPerPulse));
    TEST_ASSERT_EQUAL_INT32(
        0, hat_logic::followingErrorPulses(181, anchor, -20, kCountsPerPulse));
}

void test_followingErrorPulses(void) {
    const int32_t anchor = hat_logic::anchorAt(0, 0, kCountsPerPulse);
    // Commanded 200 pulses out; the servo has reached 150.
    const int32_t measured = hat_logic::pulsesToCounts(150, kCountsPerPulse);
    TEST_ASSERT_EQUAL_INT32(
        -50, hat_logic::followingErrorPulses(measured, anchor, 200, kCountsPerPulse));
    TEST_ASSERT_EQUAL_INT32(
        0, hat_logic::followingErrorPulses(measured, anchor, 150, kCountsPerPulse));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_accumulatePulses_counts_up_and_down);
    RUN_TEST(test_accumulatePulses_survives_16bit_wrap);

    RUN_TEST(test_pulsesToCounts_one_revolution);
    RUN_TEST(test_pulsesToCounts_reversed_servo);
    RUN_TEST(test_countsToPulses_round_trip);

    RUN_TEST(test_anchorAt_holds_measured_position);
    RUN_TEST(test_followingErrorPulses_tracks_pulses_from_anchor);
    RUN_TEST(test_followingErrorPulses);

    return UNITY_END();
}
