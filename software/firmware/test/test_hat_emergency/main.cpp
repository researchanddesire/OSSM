#include <unity.h>
#include <initializer_list>
#include "../../src/services/ossmHatEmergencyLogic.hpp"

using namespace ossm_hat_emergency;
void setUp() {}
void tearDown() {}

Inputs live(uint32_t now) {
    Inputs in;
    in.outputPossible=true; in.guardAt=in.telemetryAt=now;
    in.rxIdle=in.txIdle=true;
    return in;
}

void guard_stall_isolates_without_waiting_for_owner() {
    Deadline d; auto in=live(1000);
    TEST_ASSERT_EQUAL(NoAction,d.tick(35000,in));
    TEST_ASSERT_EQUAL(Isolate,d.tick(35001,in));
    TEST_ASSERT_EQUAL(OssmHatEmergencyCause::GuardDeadline,d.cause);
}
void live_guard_cannot_mask_missing_physical_telemetry() {
    Deadline d; auto in=live(1);
    in.guardAt=80002;
    TEST_ASSERT_EQUAL(Isolate,d.tick(80002,in));
    TEST_ASSERT_EQUAL(OssmHatEmergencyCause::TelemetryDeadline,d.cause);
}
void stop_obligation_survives_observer_liveness() {
    Deadline d; auto in=live(60000);
    in.stopPending=true; in.stopAt=9999;
    TEST_ASSERT_EQUAL(Isolate,d.tick(60000,in));
    TEST_ASSERT_EQUAL(OssmHatEmergencyCause::StopTimeout,d.cause);
}
void unenergized_idle_does_not_transmit_on_stale_heartbeats() {
    Deadline d; Inputs in;
    TEST_ASSERT_EQUAL(NoAction,d.tick(5000000,in));
}
void qualification_uses_real_delayed_deadline_and_rtu_quiet() {
    Deadline d; Inputs in; in.rxIdle=in.txIdle=true;
    TEST_ASSERT_TRUE(d.schedule(OssmHatEmergencyCause::Qualification,100));
    TEST_ASSERT_EQUAL(NoAction,d.tick(3099,in));
    TEST_ASSERT_EQUAL(Isolate,d.tick(3100,in));
    in.lastRxAt=5000;
    TEST_ASSERT_EQUAL(NoAction,d.tick(7999,in));
    TEST_ASSERT_EQUAL(EnableDirection,d.tick(8000,in));
    TEST_ASSERT_EQUAL(NoAction,d.tick(8999,in));
    TEST_ASSERT_EQUAL(WriteOff,d.tick(9000,in));
    TEST_ASSERT_FALSE(d.sent(9100,false));
    TEST_ASSERT_TRUE(d.sent(9800,true));
    TEST_ASSERT_TRUE(d.echoed());
    TEST_ASSERT_TRUE(d.canRestore(true,true,true,false));
}
void busy_receiver_never_gets_a_speculative_stop_frame() {
    Deadline d; Inputs in; in.txIdle=true;
    d.schedule(OssmHatEmergencyCause::OffOnlyPreflight,0);
    TEST_ASSERT_EQUAL(Isolate,d.tick(3000,in));
    TEST_ASSERT_EQUAL(NoAction,d.tick(53000,in));
    TEST_ASSERT_EQUAL(DisableDirection,d.tick(53001,in));
    TEST_ASSERT_EQUAL(Phase::Failed,d.phase);
}
void transmit_timeout_releases_direction_and_prevents_restore() {
    Deadline d; auto in=live(0); in.fault=true;
    d.tick(1,in); d.tick(3001,in); d.tick(4001,in);
    TEST_ASSERT_EQUAL(DisableDirection,d.tick(7002,in));
    TEST_ASSERT_FALSE(d.canRestore(true,true,true,false));
}
void lost_echo_is_failure_not_verified_stop() {
    Deadline d; Inputs in; in.rxIdle=in.txIdle=true;
    d.schedule(OssmHatEmergencyCause::OffOnlyPreflight,0);
    d.tick(3000,in); d.tick(6000,in); d.tick(7000,in);
    d.sent(7700,true);
    TEST_ASSERT_EQUAL(DisableDirection,d.tick(107701,in));
    TEST_ASSERT_FALSE(d.echoed());
}
void real_fault_vetoes_qualification_restore_permanently() {
    Deadline d; Inputs in; in.rxIdle=in.txIdle=true;
    d.schedule(OssmHatEmergencyCause::Qualification,0);
    d.tick(3000,in); d.tick(6000,in); d.tick(7000,in);
    d.sent(7700,true); d.echoed();
    in.fault=true; d.tick(8000,in); in.fault=false;
    TEST_ASSERT_FALSE(d.canRestore(true,true,true,false));
}
void ordinary_fault_is_never_a_qualification_restore() {
    Deadline d; auto in=live(0); in.fault=true;
    d.tick(1,in); d.tick(3001,in); d.tick(4001,in);
    d.sent(4800,true); d.echoed();
    TEST_ASSERT_FALSE(d.canRestore(true,true,true,false));
}
void new_fault_has_priority_over_scheduled_test() {
    Deadline d; auto in=live(0); in.fault=true;
    d.schedule(OssmHatEmergencyCause::Qualification,0);
    d.tick(3000,in);
    TEST_ASSERT_EQUAL(OssmHatEmergencyCause::ExplicitFault,d.cause);
}
void unsigned_clock_wrap_keeps_the_same_deadline() {
    Deadline d; auto in=live(UINT32_MAX-10000);
    TEST_ASSERT_EQUAL(Isolate,d.tick(25000,in));
}
void restore_requires_all_independent_proofs() {
    Deadline d; Inputs in; in.rxIdle=in.txIdle=true;
    d.schedule(OssmHatEmergencyCause::OffOnlyPreflight,0);
    d.tick(3000,in); d.tick(6000,in); d.tick(7000,in); d.sent(7800,true); d.echoed();
    TEST_ASSERT_FALSE(d.canRestore(false,true,true,false));
    TEST_ASSERT_FALSE(d.canRestore(true,false,true,false));
    TEST_ASSERT_FALSE(d.canRestore(true,true,false,false));
    TEST_ASSERT_FALSE(d.canRestore(true,true,true,true));
    TEST_ASSERT_TRUE(d.canRestore(true,true,true,false));
}
void duplicates_cannot_fabricate_quiet_or_fifty_hertz() {
    QuietProof q;
    for (unsigned i=0;i<100;++i) TEST_ASSERT_FALSE(q.accept(1,1,i*20,100,true));
}
void actual_fifty_hertz_quiet_requires_half_second() {
    QuietProof q;
    for (unsigned i=0;i<25;++i) TEST_ASSERT_FALSE(q.accept(i+1,1,i*20,100,true));
    TEST_ASSERT_TRUE(q.accept(26,1,500,100,true));
}
void slow_receipts_do_not_pass_a_configured_fifty_hertz_claim() {
    QuietProof q;
    for (unsigned i=0;i<60;++i) TEST_ASSERT_FALSE(q.accept(i+1,1,i*30,100,true));
}
void movement_or_continuity_change_restarts_quiet_window() {
    QuietProof q;
    for (unsigned i=0;i<25;++i) q.accept(i+1,1,i*20,100,true);
    TEST_ASSERT_FALSE(q.accept(26,1,500,106,true));
    for (unsigned i=0;i<24;++i) TEST_ASSERT_FALSE(q.accept(i+27,1,520+i*20,106,true));
    TEST_ASSERT_FALSE(q.accept(51,2,1000,106,true));
}
void output_enabled_or_alarm_resets_stopped_proof() {
    QuietProof q;
    for (unsigned i=0;i<25;++i) q.accept(i+1,1,i*20,100,true);
    TEST_ASSERT_FALSE(q.accept(26,1,500,100,false));
    TEST_ASSERT_FALSE(q.accept(27,1,520,100,true));
}
void retained_old_echo_never_authorizes_queries_during_a_new_stop() {
    for (const auto phase : {Phase::Ready,Phase::Quiet,Phase::Setup,Phase::Transmit,Phase::Echo})
        TEST_ASSERT_FALSE(verificationReadAllowed(phase,true,true,true,false,false));
    TEST_ASSERT_TRUE(verificationReadAllowed(Phase::Verify,true,true,true,false,false));
    TEST_ASSERT_FALSE(verificationReadAllowed(Phase::Verify,true,true,true,true,false));
    TEST_ASSERT_FALSE(verificationReadAllowed(Phase::Verify,true,true,true,false,true));
}
int main() {
    UNITY_BEGIN();
    RUN_TEST(guard_stall_isolates_without_waiting_for_owner);
    RUN_TEST(live_guard_cannot_mask_missing_physical_telemetry);
    RUN_TEST(stop_obligation_survives_observer_liveness);
    RUN_TEST(unenergized_idle_does_not_transmit_on_stale_heartbeats);
    RUN_TEST(qualification_uses_real_delayed_deadline_and_rtu_quiet);
    RUN_TEST(busy_receiver_never_gets_a_speculative_stop_frame);
    RUN_TEST(transmit_timeout_releases_direction_and_prevents_restore);
    RUN_TEST(lost_echo_is_failure_not_verified_stop);
    RUN_TEST(real_fault_vetoes_qualification_restore_permanently);
    RUN_TEST(ordinary_fault_is_never_a_qualification_restore);
    RUN_TEST(new_fault_has_priority_over_scheduled_test);
    RUN_TEST(unsigned_clock_wrap_keeps_the_same_deadline);
    RUN_TEST(restore_requires_all_independent_proofs);
    RUN_TEST(duplicates_cannot_fabricate_quiet_or_fifty_hertz);
    RUN_TEST(actual_fifty_hertz_quiet_requires_half_second);
    RUN_TEST(slow_receipts_do_not_pass_a_configured_fifty_hertz_claim);
    RUN_TEST(movement_or_continuity_change_restarts_quiet_window);
    RUN_TEST(output_enabled_or_alarm_resets_stopped_proof);
    RUN_TEST(retained_old_echo_never_authorizes_queries_during_a_new_stop);
    return UNITY_END();
}
