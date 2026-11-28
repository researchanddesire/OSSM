#include <ArduinoJson.h>
#include <unity.h>

#include <cstdint>
#include <cstring>
#include <string>

#include "feedback_timing.h"

using namespace feedback_timing;

void setUp() {}
void tearDown() {}

namespace {

// A healthy shared bus: the other master polls every 10 ms and each reply is
// accepted and published at once; the reader loops every millisecond.
struct Bus {
    Recorder recorder;
    uint32_t now = 1000;
    uint32_t sampleAt = 0;

    void readerIteration(bool bytes, bool request, bool frame) {
        recorder.mark(Stage::ReaderLoop, now);
        recorder.point(ReaderPoint::Reading);
        if (bytes) {
            recorder.mark(Stage::UartBytes, now);
            recorder.drained(33);
        }
        if (request) recorder.mark(Stage::RequestSeen, now);
        if (frame) {
            recorder.mark(Stage::FrameAccepted, now);
            recorder.point(ReaderPoint::Publishing);
            recorder.mark(Stage::Published, now);
            sampleAt = now;
        }
        recorder.point(ReaderPoint::Sleeping);
    }

    // Advance `ms` milliseconds of normal traffic, with a guard tick every 5 ms.
    void healthy(uint32_t ms) {
        for (uint32_t i = 0; i < ms; ++i) {
            ++now;
            const bool cycle = now % 10 == 0;
            readerIteration(cycle, cycle, cycle);
            if (now % 5 == 0) guardTick(Core::Reader);
        }
    }

    bool guardTick(Core core, const char *task = nullptr) {
        const bool open = recorder.observe(now, true, sampleAt);
        if (open) recorder.core(core, task);
        return open;
    }

    // `ms` milliseconds in which the reader never runs, its core held by `core`.
    void readerStopped(uint32_t ms, Core core, const char *task = nullptr) {
        for (uint32_t i = 0; i < ms; ++i) {
            ++now;
            if (now % 5 == 0) guardTick(core, task);
        }
    }
};

Stall latest(const Recorder &recorder) { return recorder.snapshot().latest; }

}  // namespace

void test_mark_tracks_last_time_and_longest_gap() {
    Recorder recorder;
    recorder.mark(Stage::UartBytes, 100);
    recorder.mark(Stage::UartBytes, 110);
    recorder.mark(Stage::UartBytes, 150);
    recorder.mark(Stage::UartBytes, 155);
    const auto snapshot = recorder.snapshot();
    const auto &gap = snapshot.stages[static_cast<size_t>(Stage::UartBytes)];
    TEST_ASSERT_EQUAL_UINT32(155, gap.lastAtMs);
    TEST_ASSERT_EQUAL_UINT32(40, gap.longestGapMs);
    TEST_ASSERT_EQUAL_UINT32(150, gap.longestGapEndMs);
    TEST_ASSERT_EQUAL_UINT32(0, snapshot.stages[static_cast<size_t>(Stage::ReaderLoop)].lastAtMs);
}

void test_a_stamp_at_time_zero_still_counts_as_happened() {
    Recorder recorder;
    recorder.mark(Stage::ReaderLoop, 0);
    TEST_ASSERT_NOT_EQUAL(0, recorder.stageAt(Stage::ReaderLoop));
}

void test_age_clamps_a_stamp_made_after_the_observer_read_its_clock() {
    TEST_ASSERT_EQUAL_UINT32(0, ageMs(100, 101));
    TEST_ASSERT_EQUAL_UINT32(kUnknownAge, ageMs(100, 0));
    TEST_ASSERT_EQUAL_UINT32(30, ageMs(5, 0xFFFFFFF0u - 9));  // across the wrap
}

void test_healthy_traffic_opens_no_stall() {
    Bus bus;
    bus.healthy(5000);
    const auto snapshot = bus.recorder.snapshot();
    TEST_ASSERT_FALSE(snapshot.latest.valid);
    TEST_ASSERT_EQUAL_UINT32(0, snapshot.stalls);
    TEST_ASSERT_EQUAL_UINT32(0, bus.recorder.reports());
    TEST_ASSERT_EQUAL_UINT32(10, snapshot.stages[static_cast<size_t>(Stage::FrameAccepted)].longestGapMs);
    TEST_ASSERT_EQUAL_UINT32(5, snapshot.stages[static_cast<size_t>(Stage::GuardDecision)].longestGapMs);
}

void test_no_stall_before_the_first_sample_or_without_feedback() {
    Recorder recorder;
    TEST_ASSERT_FALSE(recorder.observe(10000, true, 0));
    TEST_ASSERT_FALSE(recorder.observe(10000, false, 100));
    TEST_ASSERT_FALSE(recorder.snapshot().latest.valid);
}

void test_stall_opens_only_beyond_the_watch_limit() {
    Bus bus;
    bus.healthy(1000);
    const uint32_t last = bus.sampleAt;
    bus.now = last + kStallWatchMs;
    TEST_ASSERT_FALSE(bus.recorder.observe(bus.now, true, last));
    bus.now = last + kStallWatchMs + 1;
    TEST_ASSERT_TRUE(bus.recorder.observe(bus.now, true, last));
    const auto stall = latest(bus.recorder);
    TEST_ASSERT_TRUE(stall.valid);
    TEST_ASSERT_TRUE(stall.open);
    TEST_ASSERT_EQUAL_UINT32(last, stall.lastSampleAtMs);
    TEST_ASSERT_EQUAL_UINT32(bus.now, stall.detectedAtMs);
    TEST_ASSERT_EQUAL_UINT32(1, bus.recorder.snapshot().stalls);
}

void test_reader_preempted_names_the_task_holding_its_core() {
    Bus bus;
    bus.healthy(1000);
    bus.readerStopped(140, Core::Other, "mqtt_task");
    TEST_ASSERT_TRUE(latest(bus.recorder).open);
    bus.healthy(30);
    const auto stall = latest(bus.recorder);
    TEST_ASSERT_FALSE(stall.open);
    TEST_ASSERT_EQUAL(Finding::ReaderPreempted, classify(stall));
    TEST_ASSERT_EQUAL_STRING("mqtt_task", stall.otherTask);
    TEST_ASSERT_GREATER_THAN_UINT32(100, stall.stageAgeMs[0]);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(140, stall.readerGapMs);
    TEST_ASSERT_GREATER_THAN_UINT16(0, stall.otherSamples);
    TEST_ASSERT_EQUAL_UINT16(0, stall.idleSamples);
    TEST_ASSERT_EQUAL_UINT32(1, bus.recorder.reports());
}

void test_reader_blocked_when_its_core_sat_idle() {
    Bus bus;
    bus.healthy(1000);
    bus.recorder.point(ReaderPoint::StopService);
    bus.readerStopped(150, Core::Idle);
    bus.healthy(30);
    const auto stall = latest(bus.recorder);
    TEST_ASSERT_EQUAL(Finding::ReaderBlocked, classify(stall));
    TEST_ASSERT_EQUAL(ReaderPoint::StopService, stall.point);
    TEST_ASSERT_EQUAL_STRING("", stall.otherTask);
}

void test_reader_busy_when_it_held_its_core_without_finishing() {
    Bus bus;
    bus.healthy(1000);
    bus.recorder.point(ReaderPoint::Reading);
    bus.readerStopped(150, Core::Reader);
    bus.healthy(30);
    const auto stall = latest(bus.recorder);
    TEST_ASSERT_EQUAL(Finding::ReaderBusy, classify(stall));
    TEST_ASSERT_EQUAL(ReaderPoint::Reading, stall.point);
}

void test_the_place_a_stopped_reader_was_stuck_is_kept_after_it_resumes() {
    Bus bus;
    bus.healthy(1000);
    bus.recorder.point(ReaderPoint::Reconnecting);
    bus.readerStopped(120, Core::Idle);
    // The reader resumes and moves on, but no frame has been accepted yet.
    for (int i = 0; i < 4; ++i) {
        ++bus.now;
        bus.readerIteration(false, false, false);
    }
    bus.guardTick(Core::Reader);
    TEST_ASSERT_EQUAL(ReaderPoint::Reconnecting, latest(bus.recorder).point);
}

void test_running_reader_without_uart_bytes() {
    Bus bus;
    bus.healthy(1000);
    for (uint32_t i = 0; i < 150; ++i) {
        ++bus.now;
        bus.readerIteration(false, false, false);
        if (bus.now % 5 == 0) bus.guardTick(Core::Idle);
    }
    bus.healthy(30);
    const auto stall = latest(bus.recorder);
    TEST_ASSERT_EQUAL(Finding::NoUartBytes, classify(stall));
    TEST_ASSERT_LESS_THAN_UINT32(kStageStaleMs, stall.readerGapMs);
}

void test_bytes_without_requests_and_requests_without_replies() {
    {
        Bus bus;
        bus.healthy(1000);
        for (uint32_t i = 0; i < 150; ++i) {
            ++bus.now;
            bus.readerIteration(bus.now % 10 == 0, false, false);
            if (bus.now % 5 == 0) bus.guardTick(Core::Reader);
        }
        bus.healthy(30);
        TEST_ASSERT_EQUAL(Finding::NoRequests, classify(latest(bus.recorder)));
    }
    {
        Bus bus;
        bus.healthy(1000);
        for (uint32_t i = 0; i < 150; ++i) {
            ++bus.now;
            const bool cycle = bus.now % 10 == 0;
            bus.readerIteration(cycle, cycle, false);
            if (bus.now % 5 == 0) bus.guardTick(Core::Reader);
        }
        bus.healthy(30);
        TEST_ASSERT_EQUAL(Finding::NoReplies, classify(latest(bus.recorder)));
    }
}

void test_accepted_frames_that_never_reach_the_guard() {
    Bus bus;
    bus.healthy(1000);
    const uint32_t published = bus.sampleAt;
    for (uint32_t i = 0; i < 150; ++i) {
        ++bus.now;
        bus.recorder.mark(Stage::ReaderLoop, bus.now);
        if (bus.now % 10 == 0) {
            bus.recorder.mark(Stage::UartBytes, bus.now);
            bus.recorder.mark(Stage::RequestSeen, bus.now);
            bus.recorder.mark(Stage::FrameAccepted, bus.now);
        }
        if (bus.now % 5 == 0) {
            if (bus.recorder.observe(bus.now, true, published)) bus.recorder.core(Core::Reader, nullptr);
        }
    }
    TEST_ASSERT_EQUAL(Finding::NotPublished, classify(latest(bus.recorder)));
}

void test_only_the_guard_view_aged() {
    Recorder recorder;
    const uint32_t now = 5000;
    for (size_t stage = 0; stage < kPipelineStages; ++stage)
        recorder.mark(static_cast<Stage>(stage), now - 2);
    TEST_ASSERT_TRUE(recorder.observe(now, true, now - 101));
    TEST_ASSERT_EQUAL(Finding::GuardOnly, classify(latest(recorder)));
}

void test_backlog_records_the_largest_drain_during_the_stall() {
    Bus bus;
    bus.healthy(1000);
    bus.readerStopped(120, Core::Other, "wifi");
    ++bus.now;
    bus.recorder.mark(Stage::ReaderLoop, bus.now);
    bus.recorder.drained(512);
    bus.guardTick(Core::Reader);
    bus.healthy(30);
    const auto snapshot = bus.recorder.snapshot();
    TEST_ASSERT_EQUAL_UINT16(512, snapshot.latest.backlogBytes);
    TEST_ASSERT_EQUAL_UINT16(512, snapshot.longestBacklogBytes);
    bus.recorder.drained(70000);
    TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, bus.recorder.snapshot().longestBacklogBytes);
}

// The shape of the cold-boot refusals: frames every ~10 ms, then the guard
// sees a 250 ms old snapshot and refuses while the stall is still open.
void test_refusal_keeps_collecting_until_its_stall_closes_then_freezes() {
    Bus bus;
    bus.healthy(1000);
    const uint32_t last = bus.sampleAt;
    bus.readerStopped(251, Core::Other, "pairingStatus");
    bus.recorder.refused(bus.now, 2);
    TEST_ASSERT_EQUAL_UINT32(1, bus.recorder.reports());
    auto snapshot = bus.recorder.snapshot();
    TEST_ASSERT_TRUE(snapshot.refusal.refused);
    TEST_ASSERT_TRUE(snapshot.refusal.open);
    TEST_ASSERT_EQUAL_UINT8(2, snapshot.refusal.fault);
    TEST_ASSERT_EQUAL_UINT32(last, snapshot.refusal.lastSampleAtMs);

    bus.readerStopped(20, Core::Other, "pairingStatus");
    bus.healthy(30);
    snapshot = bus.recorder.snapshot();
    TEST_ASSERT_FALSE(snapshot.refusal.open);
    TEST_ASSERT_GREATER_THAN_UINT32(last + 270, snapshot.refusal.endedAtMs);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(270, snapshot.refusal.readerGapMs);
    TEST_ASSERT_EQUAL(Finding::ReaderPreempted, classify(snapshot.refusal));
    TEST_ASSERT_EQUAL_UINT32(2, bus.recorder.reports());
    const uint32_t endedAt = snapshot.refusal.endedAtMs;

    // A later stall replaces the latest record only.
    bus.readerStopped(130, Core::Idle);
    bus.healthy(30);
    bus.recorder.refused(bus.now, 3);  // a second refusal is not kept
    snapshot = bus.recorder.snapshot();
    TEST_ASSERT_EQUAL(Finding::ReaderBlocked, classify(snapshot.latest));
    TEST_ASSERT_EQUAL(Finding::ReaderPreempted, classify(snapshot.refusal));
    TEST_ASSERT_EQUAL_UINT32(endedAt, snapshot.refusal.endedAtMs);
    TEST_ASSERT_EQUAL_UINT8(2, snapshot.refusal.fault);
    TEST_ASSERT_EQUAL_UINT32(2, snapshot.stalls);
}

void test_refusal_without_an_open_stall_is_recorded_without_stage_evidence() {
    Bus bus;
    bus.healthy(1000);
    bus.recorder.refused(bus.now, 16);
    const auto snapshot = bus.recorder.snapshot();
    TEST_ASSERT_TRUE(snapshot.refusal.valid);
    TEST_ASSERT_TRUE(snapshot.refusal.refused);
    TEST_ASSERT_FALSE(snapshot.refusal.open);
    TEST_ASSERT_EQUAL_UINT8(16, snapshot.refusal.fault);
    TEST_ASSERT_EQUAL(Finding::None, classify(snapshot.refusal));
    TEST_ASSERT_FALSE(snapshot.latest.valid);
}

void test_milestones_keep_their_latest_time() {
    Recorder recorder;
    recorder.milestone(Milestone::MqttStarted, 20528);
    recorder.milestone(Milestone::MqttStarted, 30000);
    recorder.milestone(Milestone::HomingDone, 0);
    const auto snapshot = recorder.snapshot();
    TEST_ASSERT_EQUAL_UINT32(30000, snapshot.milestones[static_cast<size_t>(Milestone::MqttStarted)]);
    TEST_ASSERT_EQUAL_UINT32(1, snapshot.milestones[static_cast<size_t>(Milestone::HomingDone)]);
    TEST_ASSERT_EQUAL_UINT32(0, snapshot.milestones[static_cast<size_t>(Milestone::BleConnected)]);
}

void test_report_is_compact_valid_json() {
    Bus bus;
    bus.healthy(20000);
    bus.recorder.milestone(Milestone::HomingDone, 18523);
    bus.recorder.milestone(Milestone::PairingRound, 21200);
    bus.readerStopped(260, Core::Other, "pairingStatus");
    bus.recorder.refused(bus.now, 2);
    bus.healthy(30);
    const auto snapshot = bus.recorder.snapshot();
    char out[kReportBytes];
    const size_t length = format(out, sizeof(out), "refusal", bus.now, snapshot, snapshot.refusal);
    TEST_ASSERT_GREATER_THAN(0, length);
    TEST_ASSERT_EQUAL(std::strlen(out), length);
    JsonDocument doc;
    TEST_ASSERT_EQUAL(DeserializationError::Ok, deserializeJson(doc, out).code());
    TEST_ASSERT_EQUAL(1, doc["v"].as<int>());
    TEST_ASSERT_EQUAL_STRING("refusal", doc["event"].as<const char *>());
    TEST_ASSERT_EQUAL_STRING("reader-preempted", doc["stall"]["finding"].as<const char *>());
    TEST_ASSERT_EQUAL_STRING("pairingStatus", doc["stall"]["core"]["task"].as<const char *>());
    TEST_ASSERT_EQUAL(2, doc["stall"]["fault"].as<int>());
    TEST_ASSERT_FALSE(doc["stall"]["open"].as<bool>());
    TEST_ASSERT_GREATER_THAN(100, doc["stall"]["age"]["loop"].as<int>());
    TEST_ASSERT_EQUAL(18523, doc["last"]["homed"].as<long>());
    TEST_ASSERT_EQUAL(21200, doc["last"]["pairing"].as<long>());
    TEST_ASSERT_EQUAL(0, doc["last"]["ble"].as<long>());
    TEST_ASSERT_TRUE(doc["longest"]["loop"].is<JsonArray>());
    TEST_ASSERT_GREATER_OR_EQUAL(260, doc["longest"]["publish"][0].as<long>());
}

void test_report_without_a_stall_and_with_unknown_ages() {
    Recorder recorder;
    const auto snapshot = recorder.snapshot();
    char out[kReportBytes];
    TEST_ASSERT_GREATER_THAN(0, format(out, sizeof(out), "query", 5, snapshot, snapshot.latest));
    JsonDocument doc;
    TEST_ASSERT_EQUAL(DeserializationError::Ok, deserializeJson(doc, out).code());
    TEST_ASSERT_TRUE(doc["stall"].isNull());

    Recorder unseen;
    TEST_ASSERT_TRUE(unseen.observe(5000, true, 100));  // no reader stage ever stamped
    const auto open = unseen.snapshot();
    TEST_ASSERT_GREATER_THAN(0, format(out, sizeof(out), "query", 5000, open, open.latest));
    TEST_ASSERT_EQUAL(DeserializationError::Ok, deserializeJson(doc, out).code());
    TEST_ASSERT_EQUAL(-1, doc["stall"]["age"]["loop"].as<int>());
    TEST_ASSERT_TRUE(doc["stall"]["open"].as<bool>());
}

void test_report_worst_case_fits_its_buffer_and_never_overflows() {
    Recorder recorder;
    for (size_t stage = 0; stage < kStageCount; ++stage) {
        recorder.mark(static_cast<Stage>(stage), 1);
        recorder.mark(static_cast<Stage>(stage), 0x7FFFFFFFu);
    }
    for (size_t m = 0; m < kMilestoneCount; ++m)
        recorder.milestone(static_cast<Milestone>(m), UINT32_MAX);
    recorder.drained(UINT16_MAX);
    const uint32_t now = 0xFFFFFFF0u;
    TEST_ASSERT_TRUE(recorder.observe(now, true, now - 0x70000000u));
    for (int i = 0; i < 70000; ++i) recorder.core(Core::Other, "0123456789abcdefXYZ");
    recorder.refused(UINT32_MAX, 255);
    const auto snapshot = recorder.snapshot();
    char out[kReportBytes];
    const size_t length = format(out, sizeof(out), "refusal", UINT32_MAX, snapshot, snapshot.refusal);
    TEST_ASSERT_GREATER_THAN(0, length);
    TEST_ASSERT_LESS_THAN(sizeof(out), length);
    TEST_ASSERT_EQUAL_STRING("0123456789abcde", snapshot.refusal.otherTask);
    JsonDocument doc;
    TEST_ASSERT_EQUAL(DeserializationError::Ok, deserializeJson(doc, out).code());

    char small[64];
    std::memset(small, 'x', sizeof(small));
    TEST_ASSERT_EQUAL(0, format(small, sizeof(small), "refusal", 1, snapshot, snapshot.refusal));
    TEST_ASSERT_EQUAL_CHAR('\0', small[0]);
    TEST_ASSERT_EQUAL(0, format(nullptr, 10, "query", 1, snapshot, snapshot.latest));
}

void test_task_names_are_made_json_safe() {
    Bus bus;
    bus.healthy(1000);
    bus.readerStopped(120, Core::Other, "a\"b\\c\nd");
    const auto snapshot = bus.recorder.snapshot();
    char out[kReportBytes];
    TEST_ASSERT_GREATER_THAN(0, format(out, sizeof(out), "stall", bus.now, snapshot, snapshot.latest));
    JsonDocument doc;
    TEST_ASSERT_EQUAL(DeserializationError::Ok, deserializeJson(doc, out).code());
    TEST_ASSERT_EQUAL_STRING("abcd", doc["stall"]["core"]["task"].as<const char *>());
}

void test_finding_and_point_names_are_stable() {
    TEST_ASSERT_EQUAL_STRING("none", name(Finding::None));
    TEST_ASSERT_EQUAL_STRING("reader-preempted", name(Finding::ReaderPreempted));
    TEST_ASSERT_EQUAL_STRING("reader-blocked", name(Finding::ReaderBlocked));
    TEST_ASSERT_EQUAL_STRING("reader-busy", name(Finding::ReaderBusy));
    TEST_ASSERT_EQUAL_STRING("no-uart-bytes", name(Finding::NoUartBytes));
    TEST_ASSERT_EQUAL_STRING("no-requests", name(Finding::NoRequests));
    TEST_ASSERT_EQUAL_STRING("no-replies", name(Finding::NoReplies));
    TEST_ASSERT_EQUAL_STRING("not-published", name(Finding::NotPublished));
    TEST_ASSERT_EQUAL_STRING("guard-only", name(Finding::GuardOnly));
    TEST_ASSERT_EQUAL_STRING("sleeping", name(ReaderPoint::Sleeping));
    TEST_ASSERT_EQUAL_STRING("emergency", name(ReaderPoint::Emergency));
    TEST_ASSERT_EQUAL_STRING("stop", name(ReaderPoint::StopService));
    TEST_ASSERT_EQUAL_STRING("reconnecting", name(ReaderPoint::Reconnecting));
    TEST_ASSERT_EQUAL_STRING("polling", name(ReaderPoint::Polling));
    TEST_ASSERT_EQUAL_STRING("unknown", name(static_cast<ReaderPoint>(200)));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_mark_tracks_last_time_and_longest_gap);
    RUN_TEST(test_a_stamp_at_time_zero_still_counts_as_happened);
    RUN_TEST(test_age_clamps_a_stamp_made_after_the_observer_read_its_clock);
    RUN_TEST(test_healthy_traffic_opens_no_stall);
    RUN_TEST(test_no_stall_before_the_first_sample_or_without_feedback);
    RUN_TEST(test_stall_opens_only_beyond_the_watch_limit);
    RUN_TEST(test_reader_preempted_names_the_task_holding_its_core);
    RUN_TEST(test_reader_blocked_when_its_core_sat_idle);
    RUN_TEST(test_reader_busy_when_it_held_its_core_without_finishing);
    RUN_TEST(test_the_place_a_stopped_reader_was_stuck_is_kept_after_it_resumes);
    RUN_TEST(test_running_reader_without_uart_bytes);
    RUN_TEST(test_bytes_without_requests_and_requests_without_replies);
    RUN_TEST(test_accepted_frames_that_never_reach_the_guard);
    RUN_TEST(test_only_the_guard_view_aged);
    RUN_TEST(test_backlog_records_the_largest_drain_during_the_stall);
    RUN_TEST(test_refusal_keeps_collecting_until_its_stall_closes_then_freezes);
    RUN_TEST(test_refusal_without_an_open_stall_is_recorded_without_stage_evidence);
    RUN_TEST(test_milestones_keep_their_latest_time);
    RUN_TEST(test_report_is_compact_valid_json);
    RUN_TEST(test_report_without_a_stall_and_with_unknown_ages);
    RUN_TEST(test_report_worst_case_fits_its_buffer_and_never_overflows);
    RUN_TEST(test_task_names_are_made_json_safe);
    RUN_TEST(test_finding_and_point_names_are_stable);
    return UNITY_END();
}
