#include <unity.h>
#include "serial_timer_probe.h"

using namespace serial_timer_probe;
void setUp() {}
void tearDown() {}
namespace {
Sample sample(uint32_t sequence, uint32_t at, int32_t progress = 0, uint32_t continuity = 1) {
    return {sequence, at, continuity, -148491 + progress, .004f, 23.35f, 50, 0};
}
void moving(Gate &gate) {
    TEST_ASSERT_TRUE(gate.start(-148491, 1000, true));
    gate.observe(sample(1,1000),1000);
    TEST_ASSERT_TRUE(gate.markEnabled(1001));
    TEST_ASSERT_TRUE(gate.markTargetWritten(-148082,1001));
}
void requested(Gate &gate) {
    moving(gate);
    for (uint32_t n=1;n<=5;++n) gate.observe(sample(n+1,1000+20*n,20*n),1000+20*n);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::RequestTimerStop),static_cast<int>(gate.takeAction(1100)));
}
void off(Gate &gate) {
    requested(gate);
    TEST_ASSERT_TRUE(gate.markIrqStop(1105,1105,true));
    TEST_ASSERT_TRUE(gate.markOff(sample(7,1120,120),1,0,1120,true));
}
void quietOff(Gate &gate) {
    off(gate);
    // Real passive recoil exceeds powered2 mm/s, stays below30 mm/s and
    // remains inside the same finite envelope. It is not powered-motion proof.
    for (uint32_t n=1;n<=6;++n) gate.observe(sample(n+7,1120+20*n,60),1120+20*n);
    TEST_ASSERT_TRUE(gate.status().offQuiet);
}
void restoring(Gate &gate) {
    quietOff(gate);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::RestoreTransport),static_cast<int>(gate.takeAction(1240)));
    TEST_ASSERT_TRUE(gate.markRestoreStarted(1241));
}
void success() {
    Gate gate; restoring(gate);
    TEST_ASSERT_TRUE(gate.markRestored(sample(1,1300,60,3),1,0,1300,true));
    for (uint32_t n=1;n<=5;++n) gate.observe(sample(n+1,1300+20*n,60,3),1300+20*n);
    TEST_ASSERT_TRUE(gate.status().passed);
    TEST_ASSERT_EQUAL_UINT32(3,gate.status().restoredContinuity);
    TEST_ASSERT_FALSE(gate.start(-148491,1500,true));
}
void prerequisites_do_not_stop_an_unowned_device() {
    Gate gate; TEST_ASSERT_FALSE(gate.start(-148491,1000,false));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::None),static_cast<int>(gate.takeAction(1001)));
}
void old_inflight_snapshot_is_ignored() {
    Gate gate; gate.start(-148491,1000,true); gate.observe(sample(99,999,600),1000);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::None),static_cast<int>(gate.status().failure));
    TEST_ASSERT_FALSE(gate.markEnabled(1000));
}
void stationary_or_duplicate_frames_never_request_irq() {
    Gate gate; moving(gate);
    for (uint32_t n=1;n<=8;++n) gate.observe(sample(n+1,1000+20*n),1000+20*n);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::None),static_cast<int>(gate.takeAction(1160)));
    for (uint32_t n=0;n<8;++n) gate.observe(sample(9,1160),1160);
    TEST_ASSERT_FALSE(gate.status().irqStopProven);
}
void too_few_positive_intervals_do_not_qualify() {
    Gate gate; moving(gate);
    gate.observe(sample(2,1060,90),1060);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::None),static_cast<int>(gate.takeAction(1060)));
}
void false_or_unparked_irq_never_passes() {
    Gate gate; requested(gate); TEST_ASSERT_FALSE(gate.markIrqStop(1105,1105,false));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::IrqUnverified),static_cast<int>(gate.status().failure));
}
void mode_output_and_owner_proof_are_mandatory() {
    for (int n=0;n<3;++n) {
        Gate gate; requested(gate); gate.markIrqStop(1105,1105,true);
        TEST_ASSERT_FALSE(gate.markOff(sample(1,1120,120,2),n==0?0:1,n==1?1:0,1120,n!=2));
        TEST_ASSERT_FALSE(gate.status().passed);
    }
}
void clearance_is_required_through_first_off() {
    Gate gate; requested(gate); gate.markIrqStop(1105,1105,true);
    TEST_ASSERT_FALSE(gate.markOff(sample(1,1120,205,2),1,0,1120,true));
}
void powered_speed_still_has_two_mm_s_limit() {
    Gate gate; moving(gate); gate.observe(sample(2,1020,34),1020);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::Speed),static_cast<int>(gate.status().failure));
}
void passive_speed_still_has_thirty_mm_s_limit() {
    Gate gate; off(gate); gate.observe(sample(8,1140,-500),1140);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::Speed),static_cast<int>(gate.status().failure));
}
void first_off_cannot_hide_excessive_passive_speed() {
    Gate gate; requested(gate); gate.markIrqStop(1105,1105,true);
    TEST_ASSERT_FALSE(gate.markOff(sample(7,1110,-500),1,0,1110,true));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::Speed),static_cast<int>(gate.status().failure));
}
void all_electrical_and_envelope_limits_survive_off() {
    for (int n=0;n<5;++n) {
        Gate gate; off(gate); auto s=sample(8,1140,120);
        if(n==0)s.currentAmps=.251f;
        if(n==1)s.voltageVolts=19.9f;
        if(n==2)s.alarm=1;
        if(n==3)s.encoderCounts=-148491+615;
        if(n==4)s.observedHz=29;
        gate.observe(s,1140); TEST_ASSERT_FALSE(gate.status().active);
        TEST_ASSERT_FALSE(gate.status().passed);
    }
}
void stale_and_duplicate_frames_never_advance_quiet() {
    Gate gate; off(gate);
    for(int n=0;n<8;++n)gate.observe(sample(7,1120,120),1120);
    TEST_ASSERT_EQUAL_UINT8(1,gate.status().quietSamples);
    gate.tick(1221); TEST_ASSERT_FALSE(gate.status().passed);
}
void output_off_does_not_itself_grant_proof_or_restore() {
    Gate gate; off(gate);
    TEST_ASSERT_FALSE(gate.status().passed);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::None),static_cast<int>(gate.takeAction(1121)));
}
void restore_requires_new_continuity_and_output_off() {
    for(int n=0;n<3;++n) {
        Gate gate; restoring(gate);
        TEST_ASSERT_FALSE(gate.markRestored(sample(1,1300,60,n==0?1:3),1,n==1?1:0,1300,n!=2));
        TEST_ASSERT_FALSE(gate.status().passed);
    }
}
void new_epoch_needs_its_own_six_quiet_samples() {
    Gate gate; restoring(gate); gate.markRestored(sample(1,1300,60,3),1,0,1300,true);
    for(uint32_t n=1;n<=4;++n)gate.observe(sample(n+1,1300+20*n,60,3),1300+20*n);
    TEST_ASSERT_FALSE(gate.status().passed);
}
void changed_continuity_cannot_be_silently_reseeded() {
    Gate gate; off(gate); gate.observe(sample(2,1140,120,3),1140);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::Continuity),static_cast<int>(gate.status().failure));
}
void cancellation_wins_over_queued_restore() {
    Gate gate; quietOff(gate); gate.abort(Failure::Cancelled);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::EmergencyStop),static_cast<int>(gate.takeAction(1241)));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::None),static_cast<int>(gate.takeAction(1242)));
    TEST_ASSERT_FALSE(gate.status().passed);
}
void overall_deadline_survives_planned_restore() {
    Gate gate; restoring(gate); gate.tick(4000);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::OverallTimeout),static_cast<int>(gate.status().failure));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::EmergencyStop),static_cast<int>(gate.takeAction(4000)));
}
void irq_deadline_does_not_wait_for_target_arrival() {
    Gate gate; requested(gate);
    gate.observe(sample(7,1180,120),1180); gate.tick(1201);
    TEST_ASSERT_FALSE(gate.status().passed);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::EmergencyStop),static_cast<int>(gate.takeAction(1201)));
}
void diagnostic_rate_change_is_not_changed_frame_payload() {
    Gate gate; off(gate);
    auto duplicate=sample(7,1120,120); duplicate.observedHz=49.8f;
    gate.observe(duplicate,1121);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::None),static_cast<int>(gate.status().failure));
    TEST_ASSERT_EQUAL_UINT8(1,gate.status().quietSamples);
}
void first_off_cannot_repair_changed_continuity() {
    Gate gate; requested(gate); gate.markIrqStop(1105,1105,true);
    TEST_ASSERT_FALSE(gate.markOff(sample(7,1120,120,2),1,0,1120,true));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::Continuity),static_cast<int>(gate.status().failure));
}
void waiting_for_driver_quiet_keeps_independent_proof_current() {
    Gate gate; quietOff(gate);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Action::RestoreTransport),static_cast<int>(gate.takeAction(1240)));
    gate.observe(sample(14,1260,70),1260);
    TEST_ASSERT_FALSE(gate.status().offQuiet);
    for(uint32_t n=1;n<=5;++n)gate.observe(sample(14+n,1260+20*n,70),1260+20*n);
    TEST_ASSERT_TRUE(gate.status().offQuiet);
    TEST_ASSERT_TRUE(gate.markRestoreStarted(1361));
}
void nonzero_alarm_never_qualifies_a_nonhoming_stop_test() {
    for(int phase=0;phase<3;++phase) {
        Gate gate;
        if(phase==0)moving(gate);
        else if(phase==1)off(gate);
        else restoring(gate);
        auto warning=phase==0?sample(2,1020):sample(8,1140,120);
        warning.alarm=0x10;
        if(phase==2) {
            warning=sample(1,1300,60,3); warning.alarm=0x10;
            TEST_ASSERT_FALSE(gate.markRestored(warning,1,0,1300,true));
        } else gate.observe(warning,warning.atMs);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(Failure::Alarm),static_cast<int>(gate.status().failure));
        TEST_ASSERT_FALSE(gate.status().passed);
    }
}
} // namespace
int main() {
    UNITY_BEGIN();
    RUN_TEST(success);
    RUN_TEST(prerequisites_do_not_stop_an_unowned_device);
    RUN_TEST(old_inflight_snapshot_is_ignored);
    RUN_TEST(stationary_or_duplicate_frames_never_request_irq);
    RUN_TEST(too_few_positive_intervals_do_not_qualify);
    RUN_TEST(false_or_unparked_irq_never_passes);
    RUN_TEST(mode_output_and_owner_proof_are_mandatory);
    RUN_TEST(clearance_is_required_through_first_off);
    RUN_TEST(powered_speed_still_has_two_mm_s_limit);
    RUN_TEST(passive_speed_still_has_thirty_mm_s_limit);
    RUN_TEST(first_off_cannot_hide_excessive_passive_speed);
    RUN_TEST(all_electrical_and_envelope_limits_survive_off);
    RUN_TEST(stale_and_duplicate_frames_never_advance_quiet);
    RUN_TEST(output_off_does_not_itself_grant_proof_or_restore);
    RUN_TEST(restore_requires_new_continuity_and_output_off);
    RUN_TEST(new_epoch_needs_its_own_six_quiet_samples);
    RUN_TEST(changed_continuity_cannot_be_silently_reseeded);
    RUN_TEST(cancellation_wins_over_queued_restore);
    RUN_TEST(overall_deadline_survives_planned_restore);
    RUN_TEST(irq_deadline_does_not_wait_for_target_arrival);
    RUN_TEST(diagnostic_rate_change_is_not_changed_frame_payload);
    RUN_TEST(first_off_cannot_repair_changed_continuity);
    RUN_TEST(waiting_for_driver_quiet_keeps_independent_proof_current);
    RUN_TEST(nonzero_alarm_never_qualifies_a_nonhoming_stop_test);
    return UNITY_END();
}
