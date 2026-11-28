#include "feedbackTiming.h"

#if defined(ARDUINO) && !defined(PIO_UNIT_TESTING)
#include <Arduino.h>
#include <atomic>
#include <cstdlib>
#include <cstring>
#include <new>

namespace {
using namespace feedback_timing;

Recorder recorder;
// Guards the stall records only; stage stamps are lock-free atomics.
portMUX_TYPE recordMux = portMUX_INITIALIZER_UNLOCKED;
std::atomic<TaskHandle_t> readerTask{nullptr};
std::atomic<int> readerCore{-1};

// Report task state (one caller at a time: the motor safety reporter).
uint32_t printedReports = 0;
uint32_t printedRefusalAt = 0, printedRefusalEnd = 0;
uint32_t printedStallAt = 0;
uint32_t lastStallPrintMs = 0;
constexpr uint32_t kStallPrintSpacingMs = 1000;
constexpr char kPrefix[] = "RAD_FEEDBACK ";

Snapshot snapshotLocked() {
    portENTER_CRITICAL(&recordMux);
    const Snapshot snapshot = recorder.snapshot();
    portEXIT_CRITICAL(&recordMux);
    return snapshot;
}

// The report can be longer than a small task stack allows, so it borrows a
// short-lived heap block. If none is free the line is skipped, never blocked.
void print(const char *event, const Snapshot &snapshot, const Stall &stall) {
    constexpr size_t prefixBytes = sizeof(kPrefix) - 1;
    auto *line = static_cast<char *>(malloc(prefixBytes + kReportBytes + 1));
    if (line == nullptr) return;
    std::memcpy(line, kPrefix, prefixBytes);
    const size_t length = format(line + prefixBytes, kReportBytes, event, millis(), snapshot, stall);
    if (length != 0) {
        line[prefixBytes + length] = '\n';
        // One write keeps concurrent task logs from splitting the record.
        Serial.write(reinterpret_cast<const uint8_t *>(line), prefixBytes + length + 1);
    }
    free(line);
}
}  // namespace

void feedbackTimingReaderStarted() {
    readerTask.store(xTaskGetCurrentTaskHandle());
    readerCore.store(xPortGetCoreID());
}

void feedbackTimingMark(Stage stage, uint32_t atMs) { recorder.mark(stage, atMs); }
void feedbackTimingPoint(ReaderPoint point) { recorder.point(point); }
void feedbackTimingDrained(uint32_t bytes) { recorder.drained(bytes); }
void feedbackTimingMilestone(Milestone milestone) { recorder.milestone(milestone, millis()); }

void feedbackTimingObserve(uint32_t nowMs, bool feedbackExpected, uint32_t sampleAtMs) {
    portENTER_CRITICAL(&recordMux);
    const bool open = recorder.observe(nowMs, feedbackExpected, sampleAtMs);
    portEXIT_CRITICAL(&recordMux);
    const TaskHandle_t reader = readerTask.load();
    const int core = readerCore.load();
    if (!open || reader == nullptr || core < 0 || core >= portNUM_PROCESSORS ||
        core == xPortGetCoreID())
        return;
    // What the reader's core runs right now. A task handle read from another
    // core may be deleted a moment later: copy only its fixed-size name.
    const TaskHandle_t current = xTaskGetCurrentTaskHandleForCPU(core);
    Core who = Core::Other;
    char name[kTaskNameBytes] = {};
    if (current == reader) {
        who = Core::Reader;
    } else if (current == xTaskGetIdleTaskHandleForCPU(core)) {
        who = Core::Idle;
    } else if (current != nullptr) {
        std::strncpy(name, pcTaskGetName(current), sizeof(name) - 1);
    }
    portENTER_CRITICAL(&recordMux);
    recorder.core(who, who == Core::Other ? name : nullptr);
    portEXIT_CRITICAL(&recordMux);
}

void feedbackTimingRefused(uint32_t nowMs, uint8_t fault) {
    portENTER_CRITICAL(&recordMux);
    recorder.refused(nowMs, fault);
    portEXIT_CRITICAL(&recordMux);
}

void feedbackTimingReportDue() {
    const uint32_t reports = recorder.reports();
    if (reports == printedReports) return;
    const Snapshot snapshot = snapshotLocked();
    const Stall &refusal = snapshot.refusal;
    const Stall &latest = snapshot.latest;
    bool done = true;
    if (refusal.refused &&
        (refusal.refusedAtMs != printedRefusalAt || refusal.endedAtMs != printedRefusalEnd)) {
        print("refusal", snapshot, refusal);
        printedRefusalAt = refusal.refusedAtMs;
        printedRefusalEnd = refusal.endedAtMs;
    }
    // A refused stall is already reported above, open and again when closed.
    const bool sameAsRefusal = latest.refused && latest.detectedAtMs == refusal.detectedAtMs;
    if (latest.valid && !latest.open && !sameAsRefusal && latest.detectedAtMs != printedStallAt) {
        const uint32_t now = millis();
        if (lastStallPrintMs != 0 && now - lastStallPrintMs < kStallPrintSpacingMs) {
            done = false;  // print it on a later call; never flood the console
        } else {
            print("stall", snapshot, latest);
            printedStallAt = latest.detectedAtMs;
            lastStallPrintMs = now;
        }
    }
    if (done) printedReports = reports;
}

void feedbackTimingReportNow() {
    const Snapshot snapshot = snapshotLocked();
    print("query", snapshot, snapshot.latest);
    if (snapshot.refusal.valid) print("refusal", snapshot, snapshot.refusal);
}

#else
void feedbackTimingReaderStarted() {}
void feedbackTimingMark(feedback_timing::Stage, uint32_t) {}
void feedbackTimingPoint(feedback_timing::ReaderPoint) {}
void feedbackTimingDrained(uint32_t) {}
void feedbackTimingMilestone(feedback_timing::Milestone) {}
void feedbackTimingObserve(uint32_t, bool, uint32_t) {}
void feedbackTimingRefused(uint32_t, uint8_t) {}
void feedbackTimingReportDue() {}
void feedbackTimingReportNow() {}
#endif
