#include <ArduinoFake.h>
#ifdef abs
#undef abs // ArduinoFake's macro must not rewrite std::abs in pure headers.
#endif
#include <ArduinoJson.h>
#include <unity.h>
#include <climits>
#include <string>
#include "services/serialStopDiagnostics.h"

void setUp() {}
void tearDown() {}

static void envelope(JsonDocument &d) {
    d["v"]=1; d["id"]=UINT32_MAX; d["stage"]="completed";
    d["ok"]=true; d["code"]="ok";
    // Longer than current OSSM state names; includes the actual library's
    // correlated before/after fields rather than budgeting only the result.
    d["stateBefore"]=std::string(40,'b');
    d["stateAfter"]=std::string(40,'a');
}
static OssmHatEmergencyStatus maximumEmergency() {
    OssmHatEmergencyStatus s;
    s.cause=static_cast<OssmHatEmergencyCause>(UINT32_MAX);
    s.epoch=s.restoredEpoch=s.tripAtUs=s.isolatedAtUs=s.transmittedAtUs=UINT32_MAX;
    s.txDoneAtUs=s.nowUs=s.capturedAtMs=s.echoAtMs=s.offAtMs=s.quietAtMs=UINT32_MAX;
    s.lastTelemetryAtMs=s.telemetrySequence=s.telemetryContinuity=UINT32_MAX;
    s.rawOffWrites=s.rxOverflows=UINT32_MAX;
    return s; // false takes more bytes than true
}
static serial_timer_probe::Status maximumProbe() {
    serial_timer_probe::Status s;
    s.phase=static_cast<serial_timer_probe::Phase>(255);
    s.failure=static_cast<serial_timer_probe::Failure>(255);
    s.positiveIntervals=s.quietSamples=s.restoredQuietSamples=255;
    s.originCounts=s.targetCounts=s.lastCounts=INT32_MIN;
    s.startedAtMs=s.irqAtMs=s.offAtMs=s.restoredContinuity=UINT32_MAX;
    return s;
}
static void emergency_pages_fit_complete_att_envelope() {
    const auto s=maximumEmergency();
    for (int p=0;p<5;++p) {
        JsonDocument d; envelope(d);
        JsonObject result=d["result"].to<JsonObject>();
        TEST_ASSERT_TRUE(serial_stop_diagnostics::emergencyPage(result,p,s));
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(509,measureJson(d));
        TEST_ASSERT_EQUAL_UINT32(UINT32_MAX,result["epoch"].as<uint32_t>());
        TEST_ASSERT_EQUAL_INT(p,result["page"].as<int>());
    }
}
static void probe_pages_fit_complete_att_envelope() {
    const auto s=maximumProbe();
    for (int p=0;p<3;++p) {
        JsonDocument d; envelope(d);
        JsonObject result=d["result"].to<JsonObject>();
        TEST_ASSERT_TRUE(serial_stop_diagnostics::timerProbePage(result,p,s));
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(509,measureJson(d));
        TEST_ASSERT_EQUAL_UINT32(UINT32_MAX,result["startedAtMs"].as<uint32_t>());
        TEST_ASSERT_EQUAL_INT(p,result["page"].as<int>());
    }
}
static void pages_keep_clock_pair_and_result_kind_separate() {
    auto e=maximumEmergency(); e.qualificationPassed=true;
    JsonDocument page;
    TEST_ASSERT_TRUE(serial_stop_diagnostics::emergencyPage(page,2,e));
    TEST_ASSERT_EQUAL_UINT32(e.nowUs,page["nowUs"].as<uint32_t>());
    TEST_ASSERT_EQUAL_UINT32(e.capturedAtMs,page["capturedAtMs"].as<uint32_t>());
    TEST_ASSERT_EQUAL_UINT32(e.tripAtUs,page["tripAtUs"].as<uint32_t>());
    TEST_ASSERT_TRUE(page["gpioStopProven"].isNull());
    JsonDocument summary;
    TEST_ASSERT_TRUE(serial_stop_diagnostics::emergencyPage(summary,0,e));
    TEST_ASSERT_TRUE(summary["qualificationPassed"].as<bool>());
    TEST_ASSERT_TRUE(summary["gpioStopProven"].isNull());
}
static void unknown_pages_fail_without_partial_evidence() {
    JsonDocument d;
    TEST_ASSERT_FALSE(serial_stop_diagnostics::emergencyPage(d,-1,maximumEmergency()));
    TEST_ASSERT_FALSE(serial_stop_diagnostics::emergencyPage(d,5,maximumEmergency()));
    TEST_ASSERT_FALSE(serial_stop_diagnostics::timerProbePage(d,3,maximumProbe()));
    TEST_ASSERT_EQUAL_UINT32(0,d.size());
}
int main() {
    UNITY_BEGIN();
    RUN_TEST(emergency_pages_fit_complete_att_envelope);
    RUN_TEST(probe_pages_fit_complete_att_envelope);
    RUN_TEST(pages_keep_clock_pair_and_result_kind_separate);
    RUN_TEST(unknown_pages_fail_without_partial_evidence);
    return UNITY_END();
}
