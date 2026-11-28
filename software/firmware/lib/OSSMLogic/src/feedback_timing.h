#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

// Timing evidence for the OSSM HAT's RS-485 shaft feedback, all on the
// controller's millisecond clock (millis()). Pure logic, testable on the
// native platform; the firmware supplies the clock, the tasks and the output.
//
// A sample travels: the HAT reader task runs (ReaderLoop), takes bytes from
// the UART driver (UartBytes), decodes the other master's request
// (RequestSeen) and the servo's CRC-valid reply (FrameAccepted), and copies
// the sample to the snapshot the motor guard reads (Published). The guard
// task then decides whether that snapshot is fresh (GuardDecision).
//
// The guard task calls observe() on every tick. Once its snapshot is older
// than kStallWatchMs it opens a stall record: the age of every stage at that
// moment, the longest reader-loop gap while the stall lasts, where the reader
// last said it was, and what the reader's core was running. The most upstream
// stage that had stopped is the delayed one.
//
// Diagnostic only. Nothing here changes a freshness limit, a stop or a
// motion decision; the guard's own thresholds are unchanged.
namespace feedback_timing {

enum class Stage : uint8_t {
    ReaderLoop,
    UartBytes,
    RequestSeen,
    FrameAccepted,
    Published,
    GuardDecision,
    Count,
};
constexpr size_t kStageCount = static_cast<size_t>(Stage::Count);
// Stages whose age is captured when a stall opens (all but the guard's own).
constexpr size_t kPipelineStages = static_cast<size_t>(Stage::GuardDecision);

// Where the reader task last said it was. Read from the other core while the
// reader has stopped iterating, it names the call it is stuck in.
enum class ReaderPoint : uint8_t {
    Unknown,
    Sleeping,      // vTaskDelay at the end of an iteration
    Emergency,     // the emergency-stop service
    StopService,   // a requested serial stop
    Reading,       // taking bytes from the UART driver and decoding them
    Reconnecting,  // timeouts, baud search and UART reconfiguration
    Polling,       // the bus master's own requests (never on a shared bus)
    Publishing,    // copying the sample to the guard's snapshot
    Count,
};

// Last time each of these happened, for correlation with a stall.
enum class Milestone : uint8_t {
    HomingDone,
    MqttStarted,
    MqttConnected,
    PairingRound,
    BleConnected,
    Count,
};
constexpr size_t kMilestoneCount = static_cast<size_t>(Milestone::Count);

// What the reader's core was running when the guard task looked.
enum class Core : uint8_t { Reader, Idle, Other };

// The stage that stopped first, judged from a closed or refused stall.
enum class Finding : uint8_t {
    None,
    ReaderPreempted,  // the reader did not run; another task held its core
    ReaderBlocked,    // the reader did not run; its core was idle (it waited)
    ReaderBusy,       // the reader ran but did not finish an iteration
    NoUartBytes,      // the reader ran; the UART driver delivered nothing
    NoRequests,       // bytes arrived; no request frame decoded
    NoReplies,        // requests decoded; no reply accepted
    NotPublished,     // replies accepted; the guard's snapshot not updated
    GuardOnly,        // every reader stage was fresh; only the guard's view aged
};

// Opens a stall record. Below the guard's own limits: evidence, not a fault.
constexpr uint32_t kStallWatchMs = 100;
// A stage that has not happened for this long counts as stopped. The reader
// loops every millisecond and the other master polls about every 10 ms.
constexpr uint32_t kStageStaleMs = 50;
constexpr uint32_t kUnknownAge = UINT32_MAX;
constexpr size_t kTaskNameBytes = 16;
// Enough for format() at its largest (every field at its maximum width).
constexpr size_t kReportBytes = 832;

struct StageGap {
    uint32_t lastAtMs = 0;
    uint32_t longestGapMs = 0;
    uint32_t longestGapEndMs = 0;
};

struct Stall {
    bool valid = false;
    bool open = false;
    bool refused = false;
    uint8_t fault = 0;
    uint32_t lastSampleAtMs = 0;  // acceptance time of the last sample before it
    uint32_t detectedAtMs = 0;
    uint32_t endedAtMs = 0;       // acceptance time of the first sample after it
    uint32_t refusedAtMs = 0;
    uint32_t stageAgeMs[kPipelineStages] = {};  // when it opened
    uint32_t readerGapMs = 0;     // longest reader-loop gap while open
    uint16_t backlogBytes = 0;    // most bytes the reader took in one iteration
    uint16_t samples = 0;         // guard ticks while open
    uint16_t readerSamples = 0;
    uint16_t idleSamples = 0;
    uint16_t otherSamples = 0;
    ReaderPoint point = ReaderPoint::Unknown;  // where a stopped reader was
    char otherTask[kTaskNameBytes] = {};       // last other task on its core
};

struct Snapshot {
    StageGap stages[kStageCount];
    uint32_t milestones[kMilestoneCount] = {};
    uint32_t stalls = 0;
    uint16_t longestBacklogBytes = 0;
    Stall latest;
    Stall refusal;
};

inline uint32_t ageMs(uint32_t nowMs, uint32_t atMs) {
    if (atMs == 0) return kUnknownAge;
    // Another core can stamp a stage just after the observer read its clock.
    const int32_t age = static_cast<int32_t>(nowMs - atMs);
    return age < 0 ? 0 : static_cast<uint32_t>(age);
}

inline Finding classify(const Stall &stall) {
    // A refusal with no stall open has no stage evidence.
    if (!stall.valid || stall.detectedAtMs == 0) return Finding::None;
    const uint32_t reader = stall.stageAgeMs[0] > stall.readerGapMs
        ? stall.stageAgeMs[0] : stall.readerGapMs;
    if (reader != kUnknownAge && reader >= kStageStaleMs) {
        if (stall.otherSamples > 0 && stall.otherSamples >= stall.idleSamples &&
            stall.otherSamples >= stall.readerSamples)
            return Finding::ReaderPreempted;
        if (stall.idleSamples > 0 && stall.idleSamples >= stall.readerSamples)
            return Finding::ReaderBlocked;
        return Finding::ReaderBusy;
    }
    const Finding later[] = {Finding::NoUartBytes, Finding::NoRequests,
                             Finding::NoReplies, Finding::NotPublished};
    for (size_t stage = 1; stage < kPipelineStages; ++stage) {
        if (stall.stageAgeMs[stage] >= kStageStaleMs) return later[stage - 1];
    }
    return Finding::GuardOnly;
}

inline const char *name(Finding finding) {
    switch (finding) {
        case Finding::None: return "none";
        case Finding::ReaderPreempted: return "reader-preempted";
        case Finding::ReaderBlocked: return "reader-blocked";
        case Finding::ReaderBusy: return "reader-busy";
        case Finding::NoUartBytes: return "no-uart-bytes";
        case Finding::NoRequests: return "no-requests";
        case Finding::NoReplies: return "no-replies";
        case Finding::NotPublished: return "not-published";
        case Finding::GuardOnly: return "guard-only";
    }
    return "unknown";
}

inline const char *name(ReaderPoint point) {
    switch (point) {
        case ReaderPoint::Unknown: return "unknown";
        case ReaderPoint::Sleeping: return "sleeping";
        case ReaderPoint::Emergency: return "emergency";
        case ReaderPoint::StopService: return "stop";
        case ReaderPoint::Reading: return "reading";
        case ReaderPoint::Reconnecting: return "reconnecting";
        case ReaderPoint::Polling: return "polling";
        case ReaderPoint::Publishing: return "publishing";
        case ReaderPoint::Count: break;
    }
    return "unknown";
}

// Lock-free on the reader side: each stage has one writer (the reader task,
// or the guard task for GuardDecision), and every other reader tolerates a
// torn pair of values. The stall records belong to the guard task; the
// firmware holds one short lock around observe/refused/core and snapshot.
class Recorder {
  public:
    // ---- any single writer per stage ----
    void mark(Stage stage, uint32_t atMs) {
        auto &slot = stages_[static_cast<size_t>(stage)];
        const uint32_t last = slot.lastAtMs.load(std::memory_order_relaxed);
        if (last != 0) {
            const uint32_t gap = atMs - last;
            if (gap < 0x80000000u && gap > slot.longestGapMs.load(std::memory_order_relaxed)) {
                slot.longestGapMs.store(gap, std::memory_order_relaxed);
                slot.longestGapEndMs.store(atMs, std::memory_order_relaxed);
            }
        }
        // Never 0: that means "not yet".
        slot.lastAtMs.store(atMs ? atMs : 1, std::memory_order_relaxed);
    }

    void point(ReaderPoint where) {
        point_.store(static_cast<uint8_t>(where), std::memory_order_relaxed);
    }

    // Bytes the reader took from the driver in one iteration.
    void drained(uint32_t bytes) {
        const uint16_t clipped = bytes > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(bytes);
        lastBacklog_.store(clipped, std::memory_order_relaxed);
        if (clipped > longestBacklog_.load(std::memory_order_relaxed))
            longestBacklog_.store(clipped, std::memory_order_relaxed);
    }

    void milestone(Milestone which, uint32_t atMs) {
        milestones_[static_cast<size_t>(which)].store(atMs ? atMs : 1, std::memory_order_relaxed);
    }

    uint32_t stageAt(Stage stage) const {
        return stages_[static_cast<size_t>(stage)].lastAtMs.load(std::memory_order_relaxed);
    }

    // ---- guard task only ----
    // One guard tick. `sampleAtMs` is the acceptance time of the snapshot the
    // guard just read (0 before the first sample). Returns true while a stall
    // is open: the caller should then report what the reader's core runs.
    bool observe(uint32_t nowMs, bool feedbackExpected, uint32_t sampleAtMs) {
        mark(Stage::GuardDecision, nowMs);
        if (latest_.open) {
            if (sampleAtMs != 0 && sampleAtMs != latest_.lastSampleAtMs) {
                close(sampleAtMs);
                return false;
            }
            update(nowMs);
            return true;
        }
        if (!feedbackExpected || sampleAtMs == 0 || ageMs(nowMs, sampleAtMs) <= kStallWatchMs)
            return false;
        open(nowMs, sampleAtMs);
        return true;
    }

    // What the reader's core was running on this tick of an open stall.
    void core(Core who, const char *taskName) {
        if (!latest_.open) return;
        if (latest_.samples < UINT16_MAX) ++latest_.samples;
        uint16_t &count = who == Core::Reader ? latest_.readerSamples
                        : who == Core::Idle   ? latest_.idleSamples
                                              : latest_.otherSamples;
        if (count < UINT16_MAX) ++count;
        if (who == Core::Other && taskName != nullptr) {
            std::strncpy(latest_.otherTask, taskName, kTaskNameBytes - 1);
            latest_.otherTask[kTaskNameBytes - 1] = '\0';
        }
        mirror();
    }

    // The guard refused motion. Only the first refusal per boot is kept.
    void refused(uint32_t nowMs, uint8_t fault) {
        if (refusal_.refused) return;
        if (latest_.open) {
            latest_.refused = true;
            latest_.refusedAtMs = nowMs;
            latest_.fault = fault;
            refusal_ = latest_;
            trackingRefusal_ = true;
        } else {
            refusal_ = Stall{};
            refusal_.valid = true;
            refusal_.refused = true;
            refusal_.refusedAtMs = nowMs;
            refusal_.fault = fault;
        }
        ++reports_;
    }

    // Changes whenever a stall closes or a refusal is recorded.
    uint32_t reports() const { return reports_.load(std::memory_order_relaxed); }

    Snapshot snapshot() const {
        Snapshot out;
        for (size_t i = 0; i < kStageCount; ++i) {
            out.stages[i].lastAtMs = stages_[i].lastAtMs.load(std::memory_order_relaxed);
            out.stages[i].longestGapMs = stages_[i].longestGapMs.load(std::memory_order_relaxed);
            out.stages[i].longestGapEndMs = stages_[i].longestGapEndMs.load(std::memory_order_relaxed);
        }
        for (size_t i = 0; i < kMilestoneCount; ++i)
            out.milestones[i] = milestones_[i].load(std::memory_order_relaxed);
        out.stalls = stalls_;
        out.longestBacklogBytes = longestBacklog_.load(std::memory_order_relaxed);
        out.latest = latest_;
        out.refusal = refusal_;
        return out;
    }

  private:
    struct AtomicGap {
        std::atomic<uint32_t> lastAtMs{0};
        std::atomic<uint32_t> longestGapMs{0};
        std::atomic<uint32_t> longestGapEndMs{0};
    };

    void open(uint32_t nowMs, uint32_t sampleAtMs) {
        latest_ = Stall{};
        latest_.valid = true;
        latest_.open = true;
        latest_.lastSampleAtMs = sampleAtMs;
        latest_.detectedAtMs = nowMs;
        for (size_t i = 0; i < kPipelineStages; ++i)
            latest_.stageAgeMs[i] = ageMs(nowMs, stages_[i].lastAtMs.load(std::memory_order_relaxed));
        latest_.point = static_cast<ReaderPoint>(point_.load(std::memory_order_relaxed));
        lastBacklog_.store(0, std::memory_order_relaxed);
        if (stalls_ < UINT32_MAX) ++stalls_;
        update(nowMs);
    }

    void update(uint32_t nowMs) {
        const uint32_t reader = ageMs(nowMs, stageAt(Stage::ReaderLoop));
        if (reader != kUnknownAge && reader > latest_.readerGapMs) latest_.readerGapMs = reader;
        // Keep the place a stopped reader is stuck in, not where it resumed.
        if (reader != kUnknownAge && reader >= kStageStaleMs)
            latest_.point = static_cast<ReaderPoint>(point_.load(std::memory_order_relaxed));
        const uint16_t backlog = lastBacklog_.load(std::memory_order_relaxed);
        if (backlog > latest_.backlogBytes) latest_.backlogBytes = backlog;
        mirror();
    }

    void close(uint32_t sampleAtMs) {
        const uint16_t backlog = lastBacklog_.load(std::memory_order_relaxed);
        if (backlog > latest_.backlogBytes) latest_.backlogBytes = backlog;
        latest_.open = false;
        latest_.endedAtMs = sampleAtMs;
        mirror();
        trackingRefusal_ = false;
        ++reports_;
    }

    // The refused stall keeps collecting evidence until it closes.
    void mirror() {
        if (trackingRefusal_) refusal_ = latest_;
    }

    AtomicGap stages_[kStageCount];
    std::atomic<uint32_t> milestones_[kMilestoneCount] = {};
    std::atomic<uint8_t> point_{0};
    std::atomic<uint16_t> lastBacklog_{0};
    std::atomic<uint16_t> longestBacklog_{0};
    Stall latest_;
    Stall refusal_;
    bool trackingRefusal_ = false;
    uint32_t stalls_ = 0;
    std::atomic<uint32_t> reports_{0};
};

namespace detail {
struct Writer {
    char *out;
    size_t capacity;
    size_t used = 0;
    bool overflow = false;

    void raw(const char *text) {
        const size_t length = std::strlen(text);
        if (overflow || length >= capacity - used) {
            overflow = true;
            return;
        }
        std::memcpy(out + used, text, length + 1);
        used += length;
    }

    void ch(char c) {
        const char text[2] = {c, '\0'};
        raw(text);
    }

    template <class First, class... Rest>
    void put(const char *format, First first, Rest... rest) {
        if (overflow) return;
        const int written = std::snprintf(out + used, capacity - used, format, first, rest...);
        if (written < 0 || static_cast<size_t>(written) >= capacity - used) {
            overflow = true;
            return;
        }
        used += static_cast<size_t>(written);
    }
};

inline long long printableAge(uint32_t age) {
    return age == kUnknownAge ? -1LL : static_cast<long long>(age);
}

inline void putStall(Writer &w, const Stall &s) {
    w.put("{\"finding\":\"%s\",\"open\":%s,\"from\":%lu,\"detected\":%lu,\"ended\":%lu",
          name(classify(s)), s.open ? "true" : "false",
          static_cast<unsigned long>(s.lastSampleAtMs), static_cast<unsigned long>(s.detectedAtMs),
          static_cast<unsigned long>(s.endedAtMs));
    if (s.refused)
        w.put(",\"refused\":%lu,\"fault\":%u", static_cast<unsigned long>(s.refusedAtMs),
              static_cast<unsigned>(s.fault));
    w.put(",\"age\":{\"loop\":%lld,\"uart\":%lld,\"request\":%lld,\"frame\":%lld,\"publish\":%lld}",
          printableAge(s.stageAgeMs[0]), printableAge(s.stageAgeMs[1]),
          printableAge(s.stageAgeMs[2]), printableAge(s.stageAgeMs[3]),
          printableAge(s.stageAgeMs[4]));
    w.put(",\"readerGap\":%lu,\"point\":\"%s\",\"backlog\":%u",
          static_cast<unsigned long>(s.readerGapMs), name(s.point),
          static_cast<unsigned>(s.backlogBytes));
    w.put(",\"core\":{\"samples\":%u,\"reader\":%u,\"idle\":%u,\"other\":%u,\"task\":\"",
          static_cast<unsigned>(s.samples), static_cast<unsigned>(s.readerSamples),
          static_cast<unsigned>(s.idleSamples), static_cast<unsigned>(s.otherSamples));
    // Task names are fixed firmware identifiers; drop anything JSON-unsafe.
    for (size_t i = 0; i < kTaskNameBytes && s.otherTask[i]; ++i) {
        const char c = s.otherTask[i];
        if (c >= 0x20 && c < 0x7f && c != '"' && c != '\\') w.ch(c);
    }
    w.raw("\"}}");
}
}  // namespace detail

// One compact JSON object (no prefix, no newline). Returns the length, or 0
// if it did not fit. `stall` is the record this report is about.
inline size_t format(char *out, size_t capacity, const char *event, uint32_t nowMs,
                     const Snapshot &snapshot, const Stall &stall) {
    if (out == nullptr || capacity == 0) return 0;
    detail::Writer w{out, capacity};
    w.put("{\"v\":1,\"event\":\"%s\",\"now\":%lu,\"stalls\":%lu,\"stall\":", event,
          static_cast<unsigned long>(nowMs), static_cast<unsigned long>(snapshot.stalls));
    if (stall.valid) detail::putStall(w, stall);
    else w.raw("null");
    static const char *const stageNames[kStageCount] = {"loop", "uart", "request", "frame",
                                                        "publish", "guard"};
    w.raw(",\"longest\":{");
    for (size_t i = 0; i < kStageCount; ++i)
        w.put("%s\"%s\":[%lu,%lu]", i ? "," : "", stageNames[i],
              static_cast<unsigned long>(snapshot.stages[i].longestGapMs),
              static_cast<unsigned long>(snapshot.stages[i].longestGapEndMs));
    w.put("},\"backlog\":%u", static_cast<unsigned>(snapshot.longestBacklogBytes));
    static const char *const milestoneNames[kMilestoneCount] = {"homed", "mqttStart", "mqttUp",
                                                                "pairing", "ble"};
    w.raw(",\"last\":{");
    for (size_t i = 0; i < kMilestoneCount; ++i)
        w.put("%s\"%s\":%lu", i ? "," : "", milestoneNames[i],
              static_cast<unsigned long>(snapshot.milestones[i]));
    w.raw("}}");
    if (w.overflow) {
        out[0] = '\0';
        return 0;
    }
    return w.used;
}

}  // namespace feedback_timing
