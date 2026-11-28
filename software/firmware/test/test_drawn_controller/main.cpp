#include <unity.h>
#include <vector>
#include "../../src/ossm/state/ble.cpp"
#include "../../src/ossm/state/settings.cpp"
#include "../../src/services/communication/leased_speed.h"
#include "../../src/ossm/streaming/drawn_pattern.cpp"

namespace io = streaming::pattern_motor;
using namespace streaming;
namespace {
io::State motor;
uint64_t revision;
uint32_t clockMs;
int stops;
bool deferEntry;
int entryPauses;
io::Park parkState;
unsigned parkPolls;
int entrySubmissions;
int entryUpdates;
unsigned finiteReads, legacyQueueReads;
double entrySpeed, entryAcceleration;
bool timedPulses = false, simulateReserve = false;
bool compactHolds = false;
size_t stopAtFrame = 0;
size_t guardTripAfter = SIZE_MAX;
size_t expireAfter = SIZE_MAX;
unsigned timedWriteCostMs = 0;
std::vector<uint32_t> acceptedAtMs;
uint32_t reserveThrough = 0;
uint32_t nextPulseAt = 0;
std::vector<int32_t> frames;
io::FrameResult feedback;
const char samples[] = "00081b335a8ee4ffffc4a75d3c3c1603";

PatternStart start(bool local = false) { return startDrawnPattern(samples, 32, 2000, revision, local); }
void tick(uint32_t count = 1) {
    for (uint32_t i = 0; i < count; ++i) {
        ++clockMs;
        if (simulateReserve) io::observedPulseRunning = clockMs < reserveThrough;
        serviceDrawnPattern(revision);
    }
}
}
namespace streaming::pattern_motor {
void setSensation(uint8_t value) {
    settings.sensation = value;
    motor.strength = value * .01;
}
State read(uint64_t ticket, bool) {
    auto copy = motor;
    copy.moving = copy.queued = false;
    copy.position = 0;
    copy.ready = copy.ready && ticket == revision && frames.size() < guardTripAfter;
    copy.fullSpeed = std::max(20000., copy.speed);
    copy.nominalSpeed = copy.fullSpeed;
    copy.fraction = copy.speed / copy.fullSpeed;
    copy.stepsPerMm = 20;
    return copy;
}
void readFiniteState(State &out, bool checkQueue) {
    ++finiteReads;
    out.moving = motor.moving;
    if (!out.moving) out.position = motor.position;
    if (checkQueue) {
        ++legacyQueueReads;
        out.queued = motor.queued;
    }
}
uint32_t now() { return clockMs; }
bool current(uint64_t ticket) { return ticket == revision; }
bool position(uint64_t ticket, int32_t target, const State &caps, bool starting) {
    if (!read(ticket).ready) return false;
    if (starting) ++entrySubmissions;
    else ++entryUpdates;
    entrySpeed = caps.speed;
    entryAcceleration = caps.acceleration;
    if (!deferEntry) motor.position = target;
    return true;
}
FrameResult submit(int32_t target, uint64_t ticket, bool, uint16_t crossing, unsigned followingHolds) {
    if (stopAtFrame) {
        if (!read(ticket).ready) return {Frame::Rejected, "guard"};
        if (frames.size() == stopAtFrame) return {Frame::Retry, "pulse_full"};
        const auto extra = compactHolds ? std::min<size_t>(followingHolds, stopAtFrame - frames.size() - 1) : 0;
        const auto count = static_cast<uint8_t>(1 + extra);
        frames.insert(frames.end(), count, target);
        return {Frame::Accepted, nullptr, 0, drawn_pattern::FrameTiming::Running, 0, 16000, count};
    }
    if (!read(ticket).ready) return {Frame::Rejected, "guard"};
    if (simulateReserve) {
        if (!observedPulseRunning) return {Frame::Rejected, "pulse_underrun"};
        if (reserveThrough + 2 > clockMs + 16) return {Frame::Retry, "pulse_full"};
        const unsigned rows = compactHolds && followingHolds && reserveThrough + 4 <= clockMs + 16 ? 2 : 1;
        reserveThrough += rows * 2;
        frames.insert(frames.end(), rows, target);
        if (timedWriteCostMs) {
            clockMs += timedWriteCostMs;
            acceptedAtMs.push_back(clockMs);
        }
        const auto queuedUs = static_cast<uint16_t>((reserveThrough - clockMs) * 1000);
        if (frames.size() == expireAfter) clockMs += drawn_pattern::leaseMs;
        return {Frame::Accepted, nullptr, 0, drawn_pattern::FrameTiming::Running, 0,
                queuedUs, static_cast<uint8_t>(rows)};
    }
    if (timedPulses && clockMs < nextPulseAt) return {Frame::Retry, "pulse_full"};
    if (feedback.state == Frame::Accepted) {
        frames.push_back(target);
        nextPulseAt = clockMs + drawn_pattern::pulseFrameMs;
    }
    return feedback;
}
bool pauseEntry(uint64_t ticket) { ++entryPauses; return ticket == revision; }
Park park(uint64_t ticket, int32_t target, bool) {
    if (ticket != revision || !motor.ready) return Park::Failed;
    ++parkPolls;
    if (parkState == Park::Parked) motor.position = target;
    return parkState;
}
void stop(uint64_t ticket) { if (ticket == revision) ++stops; }
}

void setUp() {
    ++revision;
    motor = {true, false, false, false, -100, -2000, -1000, 0, 8000};
    clockMs = 0;
    frames.clear();
    stops = 0;
    deferEntry = false;
    parkPolls = 0;
    entryPauses = 0;
    parkState = io::Park::Parked;
    entrySubmissions = 0;
    entryUpdates = 0;
    finiteReads = legacyQueueReads = 0;
    frame_plan::RawFramePlanner::pumpInline = true;
    frame_plan::RawFramePlanner::workerAvailable = true;
    io::observedPulseRunning = true;
    timedPulses = simulateReserve = compactHolds = false;
    stopAtFrame = 0;
    guardTripAfter = expireAfter = SIZE_MAX;
    timedWriteCostMs = 0;
    acceptedAtMs.clear();
    nextPulseAt = 0;
    settings = {};
    bleState = {};
    feedback = {io::Frame::Accepted};
    setDrawnPatternReady(revision, true);
}
void tearDown() { setDrawnPatternReady(revision, false); }

void test_worker_allocation_failure_leaves_motor_and_serial_available() {
    using Planner = frame_plan::RawFramePlanner;
    Planner::workerAvailable = false;
    motor.speed = 250;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::WorkerMemory), int(start()));
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_EQUAL_UINT(0, entrySubmissions);
    TEST_ASSERT_EQUAL_UINT(0, stops);
    motor.serial = true;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    tick(25);
    TEST_ASSERT_FALSE(frames.empty());
    cancelDrawnPattern();
    Planner::workerAvailable = true;
    motor.serial = false;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
}

void test_retired_planner_keeps_its_own_source_and_cannot_publish_after_stop() {
    using Planner = frame_plan::RawFramePlanner;
    Planner::pumpInline = false;
    motor.speed = 500;
    motor.strength = .5;
    drawn_pattern::Modifiers marked;
    marked.entries[marked.count++] = {150, 420, 0};
    marked.entries[marked.count++] = {530, 530, 30000};
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(startDrawnPattern(samples, 32, 2000, revision,
                              false, drawn_pattern::SampleFormat::Q8, marked)));
    tick(2);
    auto *old = rawPlanner;
    old->retainForTest(); // the active worker's reference
    const auto result = old->prepare([&] {
        cancelDrawnPattern();
        TEST_ASSERT_EQUAL_STRING("stop", drawnPatternStatus().lastStop.reason);
        TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
        tick(2);
    });
    TEST_ASSERT_EQUAL_INT(int(Planner::Action::Exit), int(result));
    old->releaseForTest();
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_TRUE(frames.empty());
    for (unsigned i = 0; i < Planner::capacity; ++i) rawPlanner->prepare([] {});
    tick();
    TEST_ASSERT_EQUAL_UINT(3, frames.size());
}

void test_worker_retargets_from_last_accepted_frame_with_continuous_slider_changes() {
    using Planner = frame_plan::RawFramePlanner;
    Planner::pumpInline = false;
    motor.speed = 200;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    tick(2);
    auto produce = [] { return rawPlanner->prepare([] {}); };
    for (unsigned i = 1; i < Planner::warmupRows; ++i) produce();
    tick();
    TEST_ASSERT_TRUE(frames.empty()); // finite entry done; software rows cannot start the clock yet
    produce();
    feedback = {io::Frame::Accepted, nullptr, 0, drawn_pattern::FrameTiming::Running, 0, 6400};
    tick(2); // low hardware reserve drains precisely the precomputed warmup rows
    TEST_ASSERT_EQUAL_UINT(Planner::warmupRows, frames.size());
    // On a new positive setting and then zero, even an almost-completed
    // calculation from the prior setting cannot extend the submitted path.
    for (unsigned edit = 1; edit <= 64; ++edit) {
        const auto before = frames.size();
        const auto phase = motionPhase.phase;
        rawPlanner->prepare([&] {
            motor.speed = edit == 64 ? 0 : 200 + 20 * edit;
            tick();
            TEST_ASSERT_LESS_OR_EQUAL_UINT(Planner::retargetPrefix, frames.size() - before);
            if (frames.size() == before) TEST_ASSERT_EQUAL_DOUBLE(phase, motionPhase.phase);
        });
        tick(3);
        TEST_ASSERT_GREATER_THAN_UINT(before, frames.size()); // no full warmup after an edit
        TEST_ASSERT_EQUAL_DOUBLE(edit == 64 ? 0 : drawnPatternStatus().speed.phaseRateHz, stampedGoal);
    }
    io::observedPulseRunning = false;
    motor.speed = 900; // retire a possible one-row assist from the previous goal
    tick();
    TEST_ASSERT_EQUAL_STRING("pulse_underrun", drawnPatternStatus().lastStop.reason);
    PatternWork stats;
    TEST_ASSERT_TRUE(stoppedPatternWork(stats));
    TEST_ASSERT_EQUAL_UINT(Planner::warmupRows, stats.maxPrepared);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT(64, stats.discarded);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT(64, stats.emptyTurns);
}

void test_live_clock_survives_observed_22ms_without_background_rows_and_stops_on_real_exhaustion() {
    using Planner = frame_plan::RawFramePlanner;
    Planner::pumpInline = false;
    motor.speed = 1600;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    tick(2);
    for (unsigned i = 0; i < Planner::warmupRows; ++i) rawPlanner->prepare([] {});
    feedback = {io::Frame::Accepted, nullptr, 0, drawn_pattern::FrameTiming::Running, 0, 6400};
    tick(2);
    TEST_ASSERT_EQUAL_UINT(Planner::warmupRows, frames.size());
    // The 16 ms observed electrical reserve is all the hardware can use.
    // Suspend the background planner for the failed 22 ms, then longer.
    simulateReserve = true;
    reserveThrough = clockMs + 16;
    tick(22);
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_GREATER_OR_EQUAL_UINT(10, frames.size() - Planner::warmupRows);
    tick(60);
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_LESS_OR_EQUAL_UINT(16, reserveThrough - clockMs);
    const auto accepted = frames.size();
    clockMs = reserveThrough;
    tick(); // now suspend BOTH service paths past electrical exhaustion
    TEST_ASSERT_EQUAL_STRING("pulse_underrun", drawnPatternStatus().lastStop.reason);
    TEST_ASSERT_EQUAL_UINT(accepted, frames.size());
    PatternFeedGap gap;
    TEST_ASSERT_TRUE(stoppedPatternFeedGap(gap));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT(35, gap.assistPublished);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT(gap.assistPublished, gap.assistAttempts);
}

void test_two_planners_publish_one_ordinal_and_an_explicit_stop_during_assist_retains_nothing() {
    using Planner = frame_plan::RawFramePlanner;
    Planner::pumpInline = false;
    motor.speed = 1000;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    tick(2);
    for (unsigned i = 0; i < Planner::warmupRows; ++i) rawPlanner->prepare([] {});
    feedback = {io::Frame::Accepted, nullptr, 0, drawn_pattern::FrameTiming::Running, 0, 6400};
    tick(2);
    auto *old = rawPlanner;
    TEST_ASSERT_TRUE(old->requestAssist(0));
    old->assist([&] { old->prepare([] {}); }); // CPU0 wins this ordinal
    TEST_ASSERT_NOT_NULL(old->front());
    old->pop();
    TEST_ASSERT_NULL(old->front());
    old->prepare([&] {
        TEST_ASSERT_TRUE(old->requestAssist(0));
        old->assist([] {}); // CPU1 wins the next ordinal
    });
    TEST_ASSERT_NOT_NULL(old->front());
    old->pop();
    TEST_ASSERT_NULL(old->front());
    const auto accepted = frames.size();
    TEST_ASSERT_TRUE(old->requestAssist(0));
    old->assist([&] {
        cancelDrawnPattern();
        TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
        tick(2);
    });
    TEST_ASSERT_EQUAL_UINT(accepted, frames.size());
    TEST_ASSERT_NULL(rawPlanner->front()); // old in-flight row never entered replacement
}

void test_live_positive_retarget_recharges_low_peripheral_and_stamps_only_latest_boundary() {
    using Planner = frame_plan::RawFramePlanner;
    motor.speed = 5000;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    feedback = {io::Frame::Accepted, nullptr, 0, drawn_pattern::FrameTiming::Running, 0, 16000};
    tick(3);
    Planner::pumpInline = false;
    const auto before = frames.size();
    PatternRateStamp original{};
    TEST_ASSERT_TRUE(drawnPatternRateStamp(0, original));
    const auto initialRate = drawnPatternStatus().speed.phaseRateHz;
    motor.speed = 14000;
    simulateReserve = true;
    reserveThrough = clockMs + 7; // 6 ms before an append, 8 ms after one
    tick();
    TEST_ASSERT_EQUAL_UINT(before + 4, frames.size());
    TEST_ASSERT_EQUAL_UINT32(14, reserveThrough - clockMs);
    TEST_ASSERT_EQUAL_DOUBLE(initialRate, drawnPatternStatus().speed.phaseRateHz);
    // The physical failure followed one new-goal append by 13.064 ms.
    // Here that turn had an existing valid prefix, despite a stopped worker.
    clockMs += 12;
    motor.speed = 20000; // replace the pending edit; never extend its boundary
    tick();
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_EQUAL_UINT(before + Planner::retargetPrefix, frames.size());
    for (unsigned i = 0; i < 20; ++i) rawPlanner->prepare([] {});
    tick();
    TEST_ASSERT_GREATER_THAN_UINT(before + Planner::retargetPrefix, frames.size());
    PatternRateStamp changed{};
    TEST_ASSERT_TRUE(drawnPatternRateStamp(0, changed));
    TEST_ASSERT_EQUAL_UINT32(original.sequence + 1, changed.sequence);
    TEST_ASSERT_EQUAL_UINT32(before + Planner::retargetPrefix, changed.frameIndex);
    TEST_ASSERT_EQUAL_DOUBLE(100,drawnPatternStatus().speed.speedPct);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(16, reserveThrough - clockMs);
}

void test_steady_raw_recharges_with_ready_frames_without_a_live_edit() {
    motor.speed = 5000;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    feedback = {io::Frame::Accepted, nullptr, 0, drawn_pattern::FrameTiming::Running, 0, 16000};
    tick(3);
    TEST_ASSERT_FALSE(rawPlanner->positivePending());

    const auto before = frames.size();
    simulateReserve = true;
    reserveThrough = clockMs + 3; // 2 ms left on the next service
    tick();
    TEST_ASSERT_EQUAL_UINT(6, frames.size() - before);
    TEST_ASSERT_EQUAL_UINT32(14, reserveThrough - clockMs);

    clockMs += 6; // the measured no-append interval was 6.556 ms
    tick();       // one millisecond host tick, peripheral still running
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(14, reserveThrough - clockMs);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(16, reserveThrough - clockMs);

    const auto last = frames.size();
    clockMs = reserveThrough; // a genuine exhaustion still stops before another pulse
    tick();
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_EQUAL_UINT(last, frames.size());
    TEST_ASSERT_EQUAL_STRING("pulse_underrun", drawnPatternStatus().lastStop.reason);
}

void test_saved_marked_speed_change_refills_through_the_first_fading_gain_command() {
    using namespace drawn_pattern;
    // Actual 23.95 mm saved Q16 trial, before the vibration window.
    // The 1% path was still active when a 16% Speed edit began its fade.
    constexpr char wire[] =
        "00000285097b1400212f302640004fda5ed06c0076847d7b8000800080008000"
        "8000800080007fff8000800080007fff80008000800080008000800080008000"
        "80008000800080008000800080008000800080008000857f93ffa87fbfffd77f"
        "ebfffa7ffffffc44f1d1e1c5cd3fb55d9b3d800064c24aa232c01e3a0e2e03bb";
    Modifiers marks;
    marks.entries[marks.count++] = {200,625,0};
    marks.entries[marks.count++] = {875,875,30000};
    motor.from = -479;
    motor.to = 0;
    motor.speed = 200;
    motor.strength = 1;
    motor.acceleration = 500000;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(startDrawnPattern(
        wire, sizeof(wire)-1, 12000, revision, true, SampleFormat::Q16, marks)));
    compactHolds = true;
    stopAtFrame = 443;
    while (frames.size() < stopAtFrame && clockMs < 3000) tick();
    TEST_ASSERT_EQUAL_UINT(443, frames.size());
    TEST_ASSERT_EQUAL_DOUBLE(1, vibration.gain.value());
    motor.speed = 3200;
    stopAtFrame = 447;
    while (frames.size() < stopAtFrame && clockMs < 3000) tick();
    TEST_ASSERT_EQUAL_UINT(447, frames.size());
    TEST_ASSERT_TRUE(reconfigureAfterFade);
    auto expectedPhase = motionPhase;
    const PhaseLimits original{goalRate, clockAcceleration, clockJerk};
    for (unsigned i=447; i<451; ++i)
        expectedPhase.advance(.002, original, activeMarkers->index, motor.strength);
    stopAtFrame = 0;
    simulateReserve = true;
    timedWriteCostMs = 1;
    reserveThrough = clockMs + 7;
    tick();
    TEST_ASSERT_GREATER_THAN_UINT(451, frames.size());
    TEST_ASSERT_GREATER_OR_EQUAL_UINT(14, reserveThrough - clockMs);
    PatternRateStamp stamp{};
    TEST_ASSERT_TRUE(drawnPatternRateStamp(0, stamp));
    TEST_ASSERT_EQUAL_UINT32(451, stamp.frameIndex);
    TEST_ASSERT_EQUAL_DOUBLE(1, stamp.gain);
    TEST_ASSERT_EQUAL_DOUBLE(0, stamp.targetGain);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, expectedPhase.phase, stamp.phase);
    // Actual 8.649 ms without another write exhausted the old 6 ms queue.
    timedWriteCostMs = 0;
    clockMs += 8;
    tick();
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_LESS_OR_EQUAL_UINT(16, reserveThrough - clockMs);
}

void test_real_slow_to_fast_boundary_commits_its_actual_frame_clock_during_refill() {
    // Actual Q8, 30.6 mm travel, 5% Sensation and 1 -> 18% Speed.
    // The retained 1% law ends before row 406. At this phase the next
    // dozen targets can be identical, although their clocks differ.
    const char brake[] = "000103060b11181f28313b46515c6874808b97a3aeb9c4ced7e0e7eef4f9fcfe"
                         "fffefcf9f4eee7e0d7cec4b9aea3978b8074685c51463b31281f18110b060301";
    motor.from = -612;
    motor.to = 0;
    motor.speed = 200;
    motor.acceleration = 50000;
    motor.strength = .05;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted),
                          int(startDrawnPattern(brake, sizeof(brake)-1, 18850, revision, true)));
    stopAtFrame = 398;
    while (frames.size() < stopAtFrame && clockMs < 3000) tick();
    TEST_ASSERT_EQUAL_UINT(398, frames.size());
    PatternRateStamp old{};
    TEST_ASSERT_TRUE(drawnPatternRateStamp(0, old));
    auto expected = motionPhase;
    const drawn_pattern::PhaseLimits oldLimits{goalRate, clockAcceleration, clockJerk};
    for (unsigned i = 0; i < 8; ++i) expected.advance(.002, oldLimits, unmarked, motor.strength);
    motor.speed = 3600;
    stopAtFrame = 402;
    while (frames.size() < stopAtFrame && clockMs < 3000) tick();
    TEST_ASSERT_EQUAL_UINT(402, frames.size());
    PatternRateStamp pendingStamp{};
    TEST_ASSERT_TRUE(drawnPatternRateStamp(0, pendingStamp));
    TEST_ASSERT_EQUAL_UINT32(old.sequence, pendingStamp.sequence);

    stopAtFrame = 0;
    simulateReserve = true;
    timedWriteCostMs = 1;
    reserveThrough = clockMs + 5;
    tick();
    TEST_ASSERT_EQUAL_UINT(410, frames.size());
    PatternRateStamp changed{};
    TEST_ASSERT_TRUE(drawnPatternRateStamp(0, changed));
    TEST_ASSERT_EQUAL_UINT32(old.sequence + 1, changed.sequence);
    TEST_ASSERT_EQUAL_UINT32(406, changed.frameIndex);
    TEST_ASSERT_EQUAL_UINT32(acceptedAtMs[4], changed.atMs);
    TEST_ASSERT_LESS_THAN_UINT32(acceptedAtMs.back(), changed.atMs);
    TEST_ASSERT_FLOAT_WITHIN(1.e-7f, expected.phase, changed.phase);
    TEST_ASSERT_FLOAT_WITHIN(1.e-7f, expected.clock.rate(), changed.rateHz);
    // Reproduce the recorded 6.701ms interruption after healthy refilling.
    timedWriteCostMs = 0;
    clockMs += 6;
    tick();
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(16, reserveThrough - clockMs);
}

void test_steady_recharge_rechecks_guard_before_its_next_prepared_row() {
    motor.speed = 5000;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    feedback = {io::Frame::Accepted, nullptr, 0, drawn_pattern::FrameTiming::Running, 0, 16000};
    tick(3);
    guardTripAfter = frames.size() + 4;
    simulateReserve = true;
    reserveThrough = clockMs + 3;
    tick();
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_EQUAL_UINT(guardTripAfter, frames.size());
    TEST_ASSERT_EQUAL_STRING("guard", drawnPatternStatus().lastStop.reason);
}

void test_owner_lease_expiring_at_a_ready_pulse_boundary_stops_at_last_accepted_row() {
    motor.speed = 5000;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    feedback = {io::Frame::Accepted, nullptr, 0, drawn_pattern::FrameTiming::Running, 0, 16000};
    tick(3);
    TEST_ASSERT_TRUE(renewDrawnPattern(revision));
    expireAfter = frames.size() + 1;
    simulateReserve = true;
    reserveThrough = clockMs + 3;
    tick();
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_EQUAL_UINT(expireAfter, frames.size());
    TEST_ASSERT_EQUAL_STRING("lease_expired", drawnPatternStatus().lastStop.reason);
    PatternWork stopped;
    TEST_ASSERT_TRUE(stoppedPatternWork(stopped));
    TEST_ASSERT_EQUAL_UINT(frames.size(), stopped.frames);
}

void test_guard_is_rechecked_before_extra_handoff_recharge_row() {
    motor.speed = 5000;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    feedback = {io::Frame::Accepted, nullptr, 0, drawn_pattern::FrameTiming::Running, 0, 16000};
    tick(3);
    guardTripAfter = frames.size() + 3;
    simulateReserve = true;
    reserveThrough = clockMs + 7;
    motor.speed = 20000;
    tick();
    TEST_ASSERT_EQUAL_UINT(guardTripAfter, frames.size());
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_EQUAL_STRING("guard", drawnPatternStatus().lastStop.reason);
}

void test_urgent_zero_and_explicit_stop_cancel_a_pending_positive_handoff() {
    using drawn_pattern::FrameTiming;
    motor.speed = 4000;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    feedback = {io::Frame::Accepted, nullptr, 0, FrameTiming::Running, 0, 16000};
    tick(3);
    motor.speed = 20000;
    feedback = {io::Frame::Retry,"pulse_full",0,FrameTiming::Running};
    tick();
    TEST_ASSERT_TRUE(rawPlanner->positivePending());
    const auto boundary = frames.size();
    motor.speed = 0;
    feedback = {io::Frame::Accepted,nullptr,0,FrameTiming::Running};
    tick();
    PatternRateStamp zero{};
    TEST_ASSERT_TRUE(drawnPatternRateStamp(0, zero));
    TEST_ASSERT_EQUAL_UINT(boundary, zero.frameIndex);
    TEST_ASSERT_EQUAL_DOUBLE(0, zero.goalRateHz);
    TEST_ASSERT_FALSE(rawPlanner->positivePending());
    motor.speed = 4000;
    tick();
    motor.speed = 20000;
    feedback = {io::Frame::Retry,"pulse_full",0,FrameTiming::Running};
    tick();
    TEST_ASSERT_TRUE(rawPlanner->positivePending());
    const auto accepted = frames.size();
    cancelDrawnPattern();
    feedback = {io::Frame::Accepted,nullptr,0,FrameTiming::Running};
    tick(20);
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_EQUAL_UINT(accepted, frames.size());
    TEST_ASSERT_EQUAL_STRING("stop", drawnPatternStatus().lastStop.reason);
}

void test_rejected_next_row_does_not_hide_a_started_prefill() {
    using Planner = frame_plan::RawFramePlanner;
    Planner::pumpInline = false;
    motor.speed = 500;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    tick(2);
    for (unsigned i = 0; i < Planner::capacity; ++i) rawPlanner->prepare([] {});
    feedback = {io::Frame::Accepted, nullptr, 0, drawn_pattern::FrameTiming::Prefill};
    tick(2);
    feedback = {io::Frame::Retry, "pulse_full", 0, drawn_pattern::FrameTiming::Started};
    tick();
    const auto accepted = frames.size();
    motor.speed = 700; // invalidates only future RAM rows
    io::observedPulseRunning = false; // prefilled driver exhausted before a new row
    tick();
    TEST_ASSERT_EQUAL_STRING("pulse_underrun", drawnPatternStatus().lastStop.reason);
    TEST_ASSERT_EQUAL_UINT(accepted, frames.size());
}

void test_raw_low_water_reenters_with_unlocked_owner_and_bounded_cpu_turns() {
    motor.speed = 240;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    tick(); // guarded finite move
    feedback = {io::Frame::Accepted, nullptr, 0, drawn_pattern::FrameTiming::Running, 0, 6400};
    bool again = false;
    auto turn = [&]() { return serviceDrawnPattern(revision, 0, &again); };
    TEST_ASSERT_TRUE(turn());
    TEST_ASSERT_TRUE(again);
    const auto entryReads = finiteReads;
    const auto queueReads = legacyQueueReads;
    TEST_ASSERT_TRUE(renewDrawnPattern(revision));
    TEST_ASSERT_TRUE(turn());
    TEST_ASSERT_TRUE(again);
    TEST_ASSERT_TRUE(turn());
    TEST_ASSERT_FALSE(again); // lower-priority tasks get a blocking tick
    TEST_ASSERT_EQUAL_UINT(entryReads, finiteReads);
    TEST_ASSERT_EQUAL_UINT(queueReads, legacyQueueReads);
    feedback.queuedUs = drawn_pattern::refillBelowUs;
    TEST_ASSERT_TRUE(turn());
    TEST_ASSERT_FALSE(again);
    feedback.queuedUs = 1300;
    TEST_ASSERT_TRUE(turn());
    TEST_ASSERT_TRUE(again);
    feedback = {io::Frame::Retry, "pulse_full"};
    TEST_ASSERT_TRUE(turn());
    TEST_ASSERT_FALSE(again); // no spin when the hardware cannot accept a frame
    cancelRemoteDrawnPattern();
    const auto atStop = frames.size();
    TEST_ASSERT_FALSE(turn());
    TEST_ASSERT_FALSE(again);
    TEST_ASSERT_EQUAL_UINT(atStop, frames.size());
    PatternWork stopped;
    TEST_ASSERT_TRUE(stoppedPatternWork(stopped));
    TEST_ASSERT_EQUAL_UINT(3, stopped.immediateTurns);
    TEST_ASSERT_EQUAL_UINT(1, stopped.lowWaterYields);
    TEST_ASSERT_EQUAL_UINT(1300, stopped.minRunningUs);
    TEST_ASSERT_EQUAL_UINT(drawn_pattern::refillBelowUs, stopped.maxQueuedUs);
}

void test_accepted_entry_waits_for_backend_without_reissuing_move() {
    deferEntry = true;
    motor.speed = 240;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    tick(4);
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_TRUE(frames.empty());
    motor.moving = true;
    tick(30);
    motor.position = motor.to;
    motor.moving = false;
    tick();
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_FALSE(frames.empty());
    TEST_ASSERT_EQUAL_INT(1, entrySubmissions);
}

void test_unstarted_entry_fails_at_deadline_and_guard_remains_immediate() {
    deferEntry = true;
    motor.speed = 240;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    tick(20);
    TEST_ASSERT_TRUE(drawnPatternActive());
    tick();
    TEST_ASSERT_EQUAL_STRING("entry_position", drawnPatternStatus().lastStop.reason);
    TEST_ASSERT_TRUE(frames.empty());
    TEST_ASSERT_EQUAL_INT(1, entrySubmissions);
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    tick(2);
    motor.ready = false;
    tick();
    TEST_ASSERT_EQUAL_STRING("guard", drawnPatternStatus().lastStop.reason);
    TEST_ASSERT_EQUAL_INT(2, stops);
}

void test_finite_entry_applies_only_changed_caps_and_stops_before_playback() {
    deferEntry = true;
    char flat[] = "00000000000000000000000000000000";
    for (bool serial : {false, true}) {
        motor.serial = serial;
        motor.speed = 200;
        motor.acceleration = 8000;
        motor.ready = true;
        motor.moving = false;
        TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted),
                              int(startDrawnPattern(flat, sizeof(flat) - 1, 2000, revision)));
        tick();
        const int initialUpdates = entryUpdates;
        motor.speed = 160;
        tick(); // accepted finite move, first pulse still pending
        TEST_ASSERT_EQUAL_INT(initialUpdates + 1, entryUpdates);
        TEST_ASSERT_EQUAL_FLOAT(160, entrySpeed);
        motor.moving = true;
        tick(300); // spans the real shared HAT's freshness deadline
        TEST_ASSERT_EQUAL_INT(initialUpdates + 1, entryUpdates);
        motor.acceleration = 4000;
        tick(2);
        TEST_ASSERT_EQUAL_INT(initialUpdates + 2, entryUpdates);
        TEST_ASSERT_EQUAL_FLOAT(4000, entryAcceleration);
        motor.ready = false;
        tick();
        TEST_ASSERT_EQUAL_STRING("guard", drawnPatternStatus().lastStop.reason);
        TEST_ASSERT_EQUAL_STRING("positioning", drawnPatternStatus().lastStop.stage);
        TEST_ASSERT_TRUE(frames.empty());

        motor.ready = true;
        motor.moving = false;
        TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
        tick();
        motor.moving = true;
        tick(300);
        cancelDrawnPattern();
        TEST_ASSERT_EQUAL_STRING("stop", drawnPatternStatus().lastStop.reason);
        TEST_ASSERT_FALSE(drawnPatternActive());
        TEST_ASSERT_TRUE(frames.empty());
    }
    TEST_ASSERT_EQUAL_INT(4, entrySubmissions);
    TEST_ASSERT_EQUAL_INT(4, stops);
}

void test_zero_speed_arms_but_expiry_never_restarts() {
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    tick(4999);
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_TRUE(frames.empty());
    tick();
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_FALSE(renewDrawnPattern(revision));
    motor.speed = 100;
    tick(25);
    TEST_ASSERT_TRUE(frames.empty());
}

void test_remote_renewal_zero_pause_and_no_restart_after_stop() {
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Busy), int(start()));
    tick(4990);
    TEST_ASSERT_TRUE(renewDrawnPattern(revision));
    motor.speed = 240;
    tick(700);
    TEST_ASSERT_FALSE(frames.empty());
    for (int32_t v : frames) TEST_ASSERT_TRUE(v >= motor.from && v <= motor.to);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(2000, drawnPatternStatus().periodMs);
    motor.speed = 0;
    tick();
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_EQUAL_UINT32(0, drawnPatternStatus().speed.periodMs);
    tick(1000);
    TEST_ASSERT_EQUAL_STRING("paused", drawnPatternStatus().speed.state);
    cancelDrawnPattern();
    TEST_ASSERT_EQUAL_INT(1, stops);
    const auto emitted = frames.size();
    motor.speed = 100;
    tick(20);
    TEST_ASSERT_EQUAL_UINT(emitted, frames.size());
}

void test_local_recall_survives_unrelated_remote_lease_but_obeys_user_controls() {
    motor.speed = 130;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start(true)));
    tick(5300);
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_FALSE(renewDrawnPattern(revision));
    cancelRemoteDrawnPattern();
    TEST_ASSERT_TRUE(drawnPatternActive());
    ++motor.from;
    tick();
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_EQUAL_INT(1, stops);
}

void test_stored_lookup_preflight_rejects_active_legacy_or_drawn_motion() {
    TEST_ASSERT_EQUAL_INT(int(PatternStart::NotReady), int(drawnPatternStartStatus(revision - 1)));
    motor.queued = true;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Busy), int(drawnPatternStartStatus(revision)));
    motor.queued = false;
    motor.moving = true;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Busy), int(drawnPatternStartStatus(revision)));
    motor.moving = false;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(drawnPatternStartStatus(revision)));
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Busy), int(drawnPatternStartStatus(revision)));
}

void test_guard_rejection_and_stale_task_do_not_touch_a_new_owner() {
    motor.speed = 100;
    motor.moving = true;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Busy), int(start()));
    motor.moving = false;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    setDrawnPatternReady(revision - 1, false);
    TEST_ASSERT_FALSE(renewDrawnPattern(revision - 1));
    TEST_ASSERT_FALSE(serviceDrawnPattern(revision - 1));
    tick(100);
    TEST_ASSERT_TRUE(drawnPatternActive());
    motor.ready = false;
    tick();
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_EQUAL_INT(int(PatternStart::NotReady), int(start()));
}

void test_no_phase_catchup_after_backpressure_or_a_scheduling_gap() {
    motor.speed = 200;
    motor.serial = true;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    tick(30);
    TEST_ASSERT_EQUAL_UINT(1, frames.size());
    feedback = {io::Frame::Retry, "serial_pending"};
    tick(100);
    TEST_ASSERT_EQUAL_UINT(1, frames.size());
    feedback = {io::Frame::Accepted};
    tick();
    TEST_ASSERT_EQUAL_UINT(2, frames.size());
    TEST_ASSERT_TRUE(std::abs(frames[1] - frames[0]) <= 4);
    clockMs += 201;
    tick();
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_EQUAL_UINT(2, frames.size());
}

void test_verified_prime_tail_freezes_phase_and_preserves_raw_starvation_deadline() {
    motor.speed = 2;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start(true)));
    feedback = {io::Frame::Retry, "pulse_priming", 0, drawn_pattern::FrameTiming::PrimeTail};
    tick(500);
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_TRUE(frames.empty());
    feedback = {io::Frame::Accepted};
    tick();
    TEST_ASSERT_EQUAL_INT(motor.to, frames.front());
    feedback = {io::Frame::Retry, "pulse_full"};
    tick(199);
    TEST_ASSERT_TRUE(drawnPatternActive());
    tick();
    TEST_ASSERT_EQUAL_STRING("pulse_full", drawnPatternStatus().lastStop.reason);
    TEST_ASSERT_EQUAL_UINT32(200, drawnPatternStatus().lastStop.frameGapMs);
}

void test_prime_tail_still_obeys_stop_zero_guard_lease_owner_and_service_loss() {
    motor.speed = 2;
    feedback = {io::Frame::Retry, "pulse_priming", 0, drawn_pattern::FrameTiming::PrimeTail};
    auto begin = [] {
        TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
        tick(260);
        TEST_ASSERT_TRUE(drawnPatternActive());
    };
    begin(); cancelDrawnPattern();
    TEST_ASSERT_EQUAL_STRING("stop", drawnPatternStatus().lastStop.reason);
    begin(); motor.speed = 0; tick();
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_EQUAL_STRING("pausing", drawnPatternStatus().speed.state);
    cancelDrawnPattern();
    motor.speed = 2;
    begin(); motor.ready = false; tick();
    TEST_ASSERT_EQUAL_STRING("guard", drawnPatternStatus().lastStop.reason);
    motor.ready = true;
    begin(); tick(drawn_pattern::leaseMs - 260);
    TEST_ASSERT_EQUAL_STRING("lease_expired", drawnPatternStatus().lastStop.reason);
    begin(); setDrawnPatternReady(revision, false, "mode_exit");
    TEST_ASSERT_EQUAL_STRING("mode_exit", drawnPatternStatus().lastStop.reason);
    setDrawnPatternReady(revision, true);
    begin(); clockMs += 200; tick();
    TEST_ASSERT_EQUAL_STRING("scheduler_gap", drawnPatternStatus().lastStop.reason);
}

void test_asymmetric_extension_loop_enters_at_shallow_end_and_reaches_deep_end() {
    motor.serial = true;
    motor.from = -1997;  // deepest native target
    motor.to = -1000;    // shallowest native target
    motor.speed = 100000;
    motor.acceleration = 100000000;
    const char pattern[] = "000000ffffffffff8080808040404000";
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted),
        int(startDrawnPattern(pattern, sizeof(pattern) - 1, 2000, revision)));
    tick();
    TEST_ASSERT_EQUAL_INT32(-1000, motor.position);
    tick(2001);
    TEST_ASSERT_EQUAL_UINT32(100, frames.size());
    TEST_ASSERT_EQUAL_INT32(-1000, frames[0]);   // y = 0
    TEST_ASSERT_TRUE(std::find(frames.begin(), frames.end(), -1997) != frames.end());
    TEST_ASSERT_TRUE(std::find(frames.begin(), frames.end(), -1500) != frames.end());
    TEST_ASSERT_TRUE(std::find(frames.begin(), frames.end(), -1250) != frames.end());
    TEST_ASSERT_TRUE(std::find(frames.begin()+1, frames.end(), -1000) != frames.end());
}

void test_first_stop_diagnostic_survives_idle_stops_and_clears_only_at_accepted_start() {
    motor.speed = 160;
    clockMs = 100;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    tick(20);
    feedback = {io::Frame::Rejected, "pulse_start", -2};
    tick();
    cancelDrawnPattern();
    cancelRemoteDrawnPattern();
    setDrawnPatternReady(revision, false);
    auto status = drawnPatternStatus();
    TEST_ASSERT_FALSE(status.active);
    TEST_ASSERT_EQUAL_STRING("pulse_start", status.lastStop.reason);
    TEST_ASSERT_EQUAL_STRING("playing", status.lastStop.stage);
    TEST_ASSERT_EQUAL_UINT32(121, status.lastStop.atMs);
    TEST_ASSERT_EQUAL_UINT32(21, status.lastStop.elapsedMs);
    TEST_ASSERT_EQUAL_INT(-2, status.lastStop.driverResult);
    TEST_ASSERT_EQUAL_UINT32(1, status.lastStop.frameGapMs);
    TEST_ASSERT_EQUAL_INT(int(PatternStart::NotReady), int(start()));
    TEST_ASSERT_EQUAL_STRING("pulse_start", drawnPatternStatus().lastStop.reason);
    setDrawnPatternReady(revision, true);
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    TEST_ASSERT_NULL(drawnPatternStatus().lastStop.reason);
}

// Exercise the same JSON dispatcher the RAD setting.write handler uses after
// lease validation, the actual owner/mailbox and the production speed setter.
motion_link::SpeedWrite writeSpeed(motion_link::Owner &link, uint16_t connection,
                                  const char *path, double value) {
    JsonDocument args;
    if (value == std::floor(value)) args["value"] = static_cast<int>(value);
    else args["value"] = value;
    return motion_link::dispatchLeasedSpeed(path, args["value"], connection,
        [&](uint16_t source, float speed, bool select) {
            return applyLeasedBleSpeed(link, source, speed, select);
        });
}

void test_only_leased_current_owner_changes_live_speed_and_zero_retains_ownership() {
    using motion_link::SpeedWrite;
    motion_link::Owner link;
    link.onMotionCommand(7);
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    TEST_ASSERT_EQUAL_INT(int(SpeedWrite::Invalid), int(writeSpeed(link, 7, "motion.speed", 32.5)));
    TEST_ASSERT_EQUAL_INT(int(SpeedWrite::Invalid), int(writeSpeed(link, 7, "motion.speed", 101)));
    TEST_ASSERT_EQUAL_INT(int(SpeedWrite::Disconnected),
                          int(writeSpeed(link, motion_link::kNoConnection, "motion.speed", 10)));
    TEST_ASSERT_FALSE(settings.speedBLE.has_value());
    TEST_ASSERT_EQUAL_INT(int(SpeedWrite::Applied), int(writeSpeed(link, 7, "motion.speedBle", 25.5)));
    TEST_ASSERT_FLOAT_WITHIN(.001f, 25.5f, settings.speedBLE.value());
    TEST_ASSERT_FALSE(bleState.lastSpeedCommandWasFromBLE);
    TEST_ASSERT_EQUAL_INT(int(SpeedWrite::Applied), int(writeSpeed(link, 7, "motion.speed", 50)));
    TEST_ASSERT_TRUE(bleState.lastSpeedCommandWasFromBLE);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 50, settings.speedBLE.value());
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, settings.speed); // effective value remains owned by knob arbitration
    motor.speed = 200;
    tick(50);
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_EQUAL_INT(int(SpeedWrite::Applied), int(writeSpeed(link, 7, "motion.speed", 0)));
    motor.speed = 0; // effective arbitrated input
    tick(500);
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_EQUAL_STRING("paused", drawnPatternStatus().speed.state);
    TEST_ASSERT_EQUAL_INT(0, stops);
    writeSpeed(link, 7, "motion.speed", 100);
    motor.speed = 20000;
    tick();
    TEST_ASSERT_TRUE(drawnPatternActive());
    writeSpeed(link, 8, "motion.speed", 75);
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_EQUAL_UINT16(8, link.owner());
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start(true)));
    writeSpeed(link, 8, "motion.speed", 75); // a wired loop has no BLE owner
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    TEST_ASSERT_TRUE(link.takeCommand(8, "set:speed:75", cancelDrawnPattern));
    TEST_ASSERT_FALSE(drawnPatternActive()); // unleased legacy text on the same link
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    tick(4900);
    writeSpeed(link, 8, "motion.speed", 50);
    tick(100);
    TEST_ASSERT_FALSE(drawnPatternActive()); // a setting write does not renew the five-second lease
    TEST_ASSERT_EQUAL_STRING("lease_expired", drawnPatternStatus().lastStop.reason);
}

void test_full_stroke_high_speed_and_live_changes_preserve_phase_and_limits() {
    motion_link::Owner link;
    link.onMotionCommand(8);
    motor.from = -10000; // maximum configured 500 mm, 20 steps/mm
    motor.to = 0;
    motor.acceleration = 1000000; // classic 50,000 mm/s²
    timedPulses = true;
    const char smooth[] = "000a254f7fb0daf5fff5dab07f4f250a";
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted),
        int(startDrawnPattern(smooth, sizeof(smooth)-1, 500, revision)));
    drawn_pattern::Waveform wave;
    TEST_ASSERT_TRUE(wave.decode(smooth, sizeof(smooth)-1, 500));
    unsigned peaks = 0;
    int32_t previous = motor.to;
    for (int percent : {15, 100, 35, 80, 25, 100, 25}) {
        TEST_ASSERT_EQUAL_INT(int(motion_link::SpeedWrite::Applied),
                              int(writeSpeed(link, 8, "motion.speed", percent)));
        motor.speed = 200.0 * settings.speedBLE.value(); // 20kHz at 100%
        const uint32_t period = wave.pace(10000, 20000, motor.acceleration, motor.speed/20000, 2).periodMs;
        tick(2);
        TEST_ASSERT_EQUAL_UINT32(period, drawnPatternStatus().periodMs);
        const auto stepCount = frames.size();
        for (uint32_t elapsed = 0; elapsed < period + 250; ++elapsed) {
            if (elapsed % 1000 == 0) TEST_ASSERT_TRUE(renewDrawnPattern(revision));
            tick();
        }
        TEST_ASSERT_TRUE(drawnPatternActive());
        const auto first = frames.begin() + stepCount;
        TEST_ASSERT_LESS_OR_EQUAL_INT32(motor.from+2, *std::min_element(first, frames.end()));
        TEST_ASSERT_GREATER_OR_EQUAL_INT32(motor.to-2, *std::max_element(first, frames.end()));
    }
    for (const int32_t target : frames) {
        TEST_ASSERT_TRUE(target >= motor.from && target <= motor.to);
        const auto steps = static_cast<unsigned>(std::abs(target-previous));
        TEST_ASSERT_LESS_OR_EQUAL_UINT(40, steps); // includes live changes, no phase jumps
        peaks = std::max(peaks, steps);
        previous = target;
    }
    TEST_ASSERT_EQUAL_UINT(40, peaks);
    TEST_ASSERT_EQUAL_INT(0, stops);
}

void test_rate_stamp_history_uses_accepted_frames_and_preserves_real_prefill_time() {
    using drawn_pattern::FrameTiming;
    motor.speed=20;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted),int(start()));
    feedback={io::Frame::Accepted,nullptr,0,FrameTiming::Prefill};
    tick(2);
    PatternRateStamp first{};
    TEST_ASSERT_TRUE(drawnPatternRateStamp(0,first));
    TEST_ASSERT_TRUE(first.first);
    TEST_ASSERT_FALSE(first.leadKnown);
    TEST_ASSERT_FLOAT_WITHIN(.000001,0.,first.phase);
    clockMs+=37;
    feedback={io::Frame::Retry,"pulse_full",0,FrameTiming::Started};
    tick();
    PatternRateStamp revised{};
    TEST_ASSERT_TRUE(drawnPatternRateStamp(first.sequence,revised));
    TEST_ASSERT_TRUE(revised.leadKnown);
    TEST_ASSERT_EQUAL_UINT32(first.atMs,revised.atMs);
    TEST_ASSERT_EQUAL_UINT32(clockMs-first.atMs+27,revised.leadBoundMs);
    const auto boundary = frames.size() + frame_plan::RawFramePlanner::retargetPrefix;
    motor.speed=120;
    tick();
    TEST_ASSERT_TRUE(drawnPatternRateStamp(0,revised));
    TEST_ASSERT_EQUAL_UINT32(first.sequence,revised.sequence);
    feedback={io::Frame::Accepted,nullptr,0,FrameTiming::Running};
    tick(3);
    TEST_ASSERT_TRUE(drawnPatternRateStamp(0,revised));
    TEST_ASSERT_EQUAL_UINT32(first.sequence+1,revised.sequence);
    TEST_ASSERT_EQUAL_UINT32(boundary,revised.frameIndex);
    TEST_ASSERT_NOT_EQUAL(first.periodMs,revised.periodMs);
    TEST_ASSERT_FALSE(revised.first);
    TEST_ASSERT_TRUE(revised.leadKnown);
    TEST_ASSERT_EQUAL_UINT16(27,revised.leadBoundMs);
    const auto change=revised.sequence;
    cancelDrawnPattern();
    TEST_ASSERT_TRUE(drawnPatternRateStamp(change,revised));
    TEST_ASSERT_TRUE(drawnPatternRateStamp(first.sequence,revised));
    // A stop before hardware start cannot turn this Play's unknown queue
    // latency into a known value when some later Play starts.
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted),int(start()));
    feedback={io::Frame::Accepted,nullptr,0,FrameTiming::Prefill};
    tick(2);
    TEST_ASSERT_TRUE(drawnPatternRateStamp(0,revised));
    const auto abandoned=revised.sequence;
    TEST_ASSERT_TRUE(revised.first);
    cancelDrawnPattern();
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted),int(start()));
    feedback={io::Frame::Accepted,nullptr,0,FrameTiming::Started};
    tick(2);
    TEST_ASSERT_TRUE(drawnPatternRateStamp(abandoned,revised));
    TEST_ASSERT_FALSE(revised.leadKnown);
    // Eight retained events, including the latest, span multiple Plays.
    for (int i=0;i<9;++i) { motor.speed=30+3*i; tick(4); }
    TEST_ASSERT_FALSE(drawnPatternRateStamp(first.sequence,revised));
    TEST_ASSERT_TRUE(drawnPatternRateStamp(0,revised));
    const auto latest=revised.sequence;
    for (uint32_t seq=latest-7;seq<=latest;++seq)
        TEST_ASSERT_TRUE(drawnPatternRateStamp(seq,revised));
}

void test_modifier_zero_reproduces_each_original_target_and_strength_writes_obey_ownership() {
    using namespace drawn_pattern;
    motor.speed = 320;
    motor.stepsPerMm = 20;
    timedPulses = true;
    const auto plain = start(true);
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(plain));
    tick(3500);
    const auto reference = frames;
    cancelDrawnPattern();
    frames.clear();
    nextPulseAt = 0;
    Modifiers m;
    m.count = 2;
    m.entries[0] = {200, 400, 0};
    m.entries[1] = {700, 700, 250};
    const auto launch = [&](bool local = false) {
        return startDrawnPattern(samples, 32, 2000, revision, local, SampleFormat::Q8, m);
    };
    motion_link::Owner link;
    link.onMotionCommand(7);
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(launch()));
    tick(3500);
    TEST_ASSERT_TRUE(reference == frames);
    PatternRateStamp before{}, after{};
    TEST_ASSERT_TRUE(drawnPatternRateStamp(0, before));
    TEST_ASSERT_TRUE(applyLeasedBleSensation(link, 7, 50));
    tick(8);
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_TRUE(drawnPatternRateStamp(0, after));
    TEST_ASSERT_TRUE(after.sequence > before.sequence);
    TEST_ASSERT_EQUAL_UINT32(0, before.frameIndex);
    TEST_ASSERT_TRUE(after.frameIndex > before.frameIndex);
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, std::fmod(after.frameIndex * .012, 1.), after.carrier);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, .5, after.requestedStrength);
    TEST_ASSERT_TRUE(after.carrier >= 0 && after.carrier < 1);
    TEST_ASSERT_FALSE(applyLeasedBleSensation(link, motion_link::kNoConnection, 80));
    TEST_ASSERT_FALSE(applyLeasedBleSensation(link, 7, 180));
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_TRUE(applyLeasedBleSensation(link, 7, 0));
    tick(4);
    TEST_ASSERT_TRUE(drawnPatternActive());
    TEST_ASSERT_TRUE(applyLeasedBleSensation(link, 8, 80));
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_EQUAL_INT(8, link.owner());
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(launch(true)));
    TEST_ASSERT_TRUE(applyLeasedBleSensation(link, 8, 20));
    TEST_ASSERT_FALSE(drawnPatternActive());
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    TEST_ASSERT_TRUE(applyLeasedBleSensation(link, 8, 20));
    TEST_ASSERT_FALSE(drawnPatternActive());
}

void test_marked_plateau_streams_six_hz_through_both_backends_and_hard_stop() {
    using namespace drawn_pattern;
    const char flat[] = "80808080808080808080808080808080";
    Modifiers m;
    m.count = 1;
    m.entries[0] = {0, 1000, 0};
    motor.speed = 20000;
    motor.acceleration = 100000;
    motor.stepsPerMm = 20;
    motor.strength = 1;
    for (bool serial : {false, true}) {
        motor.serial = serial;
        timedPulses = !serial;
        nextPulseAt = 0;
        frames.clear();
        motor.ready = true;
        TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted),
                              int(startDrawnPattern(flat, 32, 4000, revision, true, SampleFormat::Q8, m)));
        tick(4500);
        TEST_ASSERT_TRUE(drawnPatternActive());
        const int32_t centre = motor.to - std::lround((motor.to - motor.from) * 128. / 255.);
        bool above = false, below = false;
        unsigned crossings = 0;
        for (size_t i = 1; i < frames.size(); ++i) {
            TEST_ASSERT_TRUE(frames[i] >= motor.from && frames[i] <= motor.to);
            above |= frames[i] > centre + 2;
            below |= frames[i] < centre - 2;
            if (frames[i] > centre && frames[i - 1] <= centre) ++crossings;
        }
        TEST_ASSERT_TRUE(above && below);
        TEST_ASSERT_GREATER_THAN_UINT32(12, crossings);
        motor.ready = false;
        tick(1);
        TEST_ASSERT_FALSE(drawnPatternActive());
        TEST_ASSERT_EQUAL_STRING("guard", drawnPatternStatus().lastStop.reason);
    }
}
void test_high_ordinal_markers_survive_controller_copy_and_give_exact_pause() {
    using namespace drawn_pattern;
    Modifiers list;
    TEST_ASSERT_TRUE(list.reserve(500));
    list.count = 500;
    for (uint16_t i = 0; i < 498; ++i) list.entries[i] = {i, i, 20};
    list.entries[498] = {600, 700, 0};
    list.entries[499] = {900, 900, 1234};
    TEST_ASSERT_TRUE(list.valid());
    motor.strength = 1;
    motor.speed = 2000;
    motor.stepsPerMm = 20;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted),
                          int(startDrawnPattern(samples, 32, 2000, revision, true, SampleFormat::Q8, list)));
    // The mailbox owns this data independently of the upload's lifetime.
    list.entries[499] = {910, 910, 999};
    uint16_t duration = 0;
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, .9, activeMarkers->index.upcoming(.8, duration));
    TEST_ASSERT_EQUAL_UINT16(1234, duration);
    TEST_ASSERT_TRUE(activeMarkers->index.range(.65) != nullptr);
    tick(25);
    TEST_ASSERT_TRUE(drawnPatternActive());
    cancelDrawnPattern();
    TEST_ASSERT_FALSE(activeMarkers);
}

void test_full_pause_grid_releases_decoded_pages_and_survives_zero_on_both_backends() {
    using namespace drawn_pattern;
    for (bool serial : {false, true}) {
        motor.serial = serial;
        motor.speed = 0;
        motor.stepsPerMm = 20;
        motor.strength = 1.;
        auto effects = std::make_unique<Modifiers>();
        TEST_ASSERT_TRUE(effects->reserve(1000));
        effects->count = 1000;
        for (uint16_t i = 0; i < 1000; ++i) effects->entries[i] = {i, i, uint16_t(i + 1)};

        TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted),
                              int(startOwnedDrawnPattern(samples, 32, 2000, revision, true,
                                                         SampleFormat::Q8, std::move(effects))));
        TEST_ASSERT_NULL(effects.get());
        TEST_ASSERT_EQUAL_UINT(0, activeMarkers->authored.count);
        TEST_ASSERT_EQUAL_UINT(4, activeMarkers->authored.entries.allocated());
        uint16_t duration;
        TEST_ASSERT_DOUBLE_WITHIN(1e-10, .999, activeMarkers->index.upcoming(.9985, duration));
        TEST_ASSERT_EQUAL_UINT16(1000, duration);
        TEST_ASSERT_DOUBLE_WITHIN(1e-10, 1., activeMarkers->index.upcoming(.9995, duration));
        TEST_ASSERT_EQUAL_UINT16(1, duration);
        const auto sent = frames.size();
        const auto entries = entrySubmissions;
        tick(6000);
        TEST_ASSERT_EQUAL_UINT(sent, frames.size());
        TEST_ASSERT_EQUAL_INT(entries, entrySubmissions);
        motor.speed = 450;
        tick(100);
        TEST_ASSERT_GREATER_THAN_UINT(sent, frames.size());
        TEST_ASSERT_TRUE(drawnPatternActive());
        cancelDrawnPattern();
        TEST_ASSERT_FALSE(drawnPatternActive());
    }
}

void test_remote_long_hold_keeps_lease_stop_and_guard_live_on_both_backends() {
    using namespace drawn_pattern;
    Modifiers markers;
    markers.count = 1;
    markers.entries[0] = {0, 0, 60000};
    motor.speed = 450;
    motor.stepsPerMm = 20;
    motor.strength = 1.;
    for (bool serial : {false, true}) {
        timedPulses = !serial;
        motor.serial = serial;
        auto play = [&]() {
            nextPulseAt = 0;
            frames.clear();
            TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(startDrawnPattern(
                samples, 32, 2000, revision, false, SampleFormat::Q8, markers)));
        };
        play();
        for (unsigned seconds = 0; seconds < 32; seconds += 4) {
            tick(4000);
            TEST_ASSERT_TRUE(renewDrawnPattern(revision));
        }
        TEST_ASSERT_TRUE(drawnPatternActive());
        TEST_ASSERT_FALSE(frames.empty());
        TEST_ASSERT_TRUE(std::all_of(frames.begin(), frames.end(), [&](int32_t v) { return v == motor.to; }));
        cancelDrawnPattern();
        TEST_ASSERT_EQUAL_STRING("stop", drawnPatternStatus().lastStop.reason);
        play();
        tick(leaseMs);
        TEST_ASSERT_FALSE(drawnPatternActive());
        TEST_ASSERT_EQUAL_STRING("lease_expired", drawnPatternStatus().lastStop.reason);
        play();
        tick(100);
        motor.ready = false;
        tick();
        TEST_ASSERT_EQUAL_STRING("guard", drawnPatternStatus().lastStop.reason);
        motor.ready = true;
    }
}

void test_refill_turn_is_bounded_and_retains_work_only_after_stop() {
    motor.speed = 500;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    tick();  // accepted finite entry
    PatternWork summary;
    TEST_ASSERT_FALSE(stoppedPatternWork(summary));
    tick();
    TEST_ASSERT_EQUAL_UINT(3, frames.size());
    tick();
    TEST_ASSERT_EQUAL_UINT(6, frames.size());
    feedback = {io::Frame::Retry, "pulse_full"};
    tick(4);
    TEST_ASSERT_TRUE(renewDrawnPattern(revision));
    cancelDrawnPattern();
    TEST_ASSERT_TRUE(stoppedPatternWork(summary));
    TEST_ASSERT_EQUAL_UINT(6, summary.frames);
    TEST_ASSERT_EQUAL_UINT(4, summary.retries);
    TEST_ASSERT_EQUAL_UINT(7, summary.plans);
    TEST_ASSERT_EQUAL_UINT(10, summary.submissions);
    TEST_ASSERT_LESS_OR_EQUAL_UINT(summary.plans, summary.movingPlans);
    TEST_ASSERT_EQUAL_STRING("stop", drawnPatternStatus().lastStop.reason);
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    TEST_ASSERT_FALSE(stoppedPatternWork(summary));
    cancelDrawnPattern();
    TEST_ASSERT_TRUE(stoppedPatternWork(summary));
    TEST_ASSERT_EQUAL_UINT(0, summary.frames);
    TEST_ASSERT_EQUAL_UINT(0, summary.retries);
}

void test_raw_queue_acquisition_restamps_delayed_stationary_entry() {
    using drawn_pattern::FrameTiming;
    motor.speed = 500;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start()));
    feedback = {io::Frame::Accepted, nullptr, 0, FrameTiming::EntryHold};
    tick(3);
    PatternRateStamp entry, raw;
    TEST_ASSERT_TRUE(drawnPatternRateStamp(0, entry));
    TEST_ASSERT_TRUE(entry.first && entry.leadKnown);
    TEST_ASSERT_EQUAL_UINT16(0, entry.leadBoundMs);
    const auto entryFrames = frames.size();
    clockMs += 22;
    feedback = {io::Frame::Accepted, nullptr, 0, FrameTiming::Prefill};
    tick();
    TEST_ASSERT_TRUE(drawnPatternRateStamp(0, raw));
    TEST_ASSERT_EQUAL_UINT32(entry.sequence + 1, raw.sequence);
    TEST_ASSERT_EQUAL_UINT32(entryFrames, raw.frameIndex);
    TEST_ASSERT_EQUAL_UINT32(entry.periodMs, raw.periodMs);
    TEST_ASSERT_FALSE(raw.first || raw.leadKnown);
    const auto submittedAt = raw.atMs;
    clockMs += 4;
    feedback = {io::Frame::Retry, "pulse_full", 0, FrameTiming::Started};
    tick();
    TEST_ASSERT_TRUE(drawnPatternRateStamp(0, raw));
    TEST_ASSERT_TRUE(raw.leadKnown);
    TEST_ASSERT_EQUAL_UINT32(clockMs - submittedAt + 27, raw.leadBoundMs);
    cancelDrawnPattern();
    PatternWork summary;
    TEST_ASSERT_TRUE(stoppedPatternWork(summary));
    TEST_ASSERT_EQUAL_UINT32(raw.sequence, summary.rawStartSequence);
}

void test_retry_reuses_frame_but_replans_for_live_inputs_and_safety() {
    motor.speed = 500;
    motor.strength = .5;
    drawn_pattern::Modifiers marked;
    marked.entries[marked.count++] = {200, 625, 0};
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(startDrawnPattern(samples, 32, 2000, revision,
                               false, drawn_pattern::SampleFormat::Q8, marked)));
    tick();
    feedback = {io::Frame::Retry, "pulse_full"};
    tick(8);
    motor.strength = 1;
    tick(2);
    motor.speed = 1000;
    tick(2);
    feedback = {io::Frame::Accepted};
    tick();
    motor.ready = false;
    tick();
    PatternWork summary;
    TEST_ASSERT_TRUE(stoppedPatternWork(summary));
    TEST_ASSERT_EQUAL_STRING("guard", drawnPatternStatus().lastStop.reason);
    TEST_ASSERT_EQUAL_UINT(3, summary.frames);
    TEST_ASSERT_EQUAL_UINT(12, summary.retries);
    TEST_ASSERT_EQUAL_UINT(15, summary.submissions);
    TEST_ASSERT_EQUAL_UINT(5, summary.plans);
}

void test_indefinite_zero_parks_and_resumes_same_phase_on_both_backends() {
    using namespace drawn_pattern;
    Modifiers modifiers;
    modifiers.entries[modifiers.count++] = {100, 650, 0};
    modifiers.entries[modifiers.count++] = {875, 875, 30000};
    for (bool serial : {false, true}) {
        motor.serial = serial;
        motor.acceleration = 500000;
        motor.strength = .5;
        motor.speed = 0;
        nextPulseAt = 0;
        timedPulses = !serial;
        const auto previousEntries = entrySubmissions;
        TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(startDrawnPattern(
            samples, 32, 12000, revision, false, SampleFormat::Q8, modifiers)));
        tick(200);
        TEST_ASSERT_EQUAL_INT(previousEntries, entrySubmissions);
        TEST_ASSERT_EQUAL_STRING("waiting", drawnPatternStatus().speed.state);
        motor.speed = 5000;
        tick(800);
        TEST_ASSERT_TRUE(drawnPatternActive());
        TEST_ASSERT_EQUAL_INT(previousEntries+1, entrySubmissions);
        motor.speed = 0;
        tick();
        for (unsigned i=0; i<2000 && std::string(drawnPatternStatus().speed.state)!="paused"; ++i) tick();
        const auto quiet = drawnPatternStatus();
        TEST_ASSERT_TRUE(quiet.active);
        TEST_ASSERT_EQUAL_STRING("paused", quiet.speed.state);
        TEST_ASSERT_EQUAL_UINT32(0, quiet.periodMs);
        TEST_ASSERT_EQUAL_DOUBLE(0, quiet.speed.phaseRateHz);
        const auto at = motionPhase.phase;
        const auto carrier = vibration.carrier;
        TEST_ASSERT_TRUE(vibration.gain.atZero());
        const auto count = frames.size();
        for (unsigned j=0;j<8;++j) {
            TEST_ASSERT_TRUE(renewDrawnPattern(revision));
            tick(4000);
        }
        TEST_ASSERT_EQUAL_UINT(count, frames.size());
        TEST_ASSERT_EQUAL_DOUBLE(at, motionPhase.phase);
        TEST_ASSERT_EQUAL_DOUBLE(carrier, vibration.carrier);
        TEST_ASSERT_FALSE(stoppedPatternWork(work));
        motor.speed = 10000;
        tick(serial ? 22 : 3);
        TEST_ASSERT_EQUAL_STRING("running", drawnPatternStatus().speed.state);
        TEST_ASSERT_TRUE(drawnPatternActive());
        TEST_ASSERT_EQUAL_INT(previousEntries+1, entrySubmissions);
        TEST_ASSERT_DOUBLE_WITHIN(.002, at, motionPhase.phase);
        cancelRemoteDrawnPattern();
        TEST_ASSERT_FALSE(drawnPatternActive());
        const auto last = frames.size();
        tick(60);
        TEST_ASSERT_EQUAL_UINT(last, frames.size());
    }
}

void test_operator_pause_preserves_remaining_authored_hold() {
    drawn_pattern::Modifiers marks;
    marks.entries[marks.count++] = {0, 0, 30000};
    drawn_pattern::MarkerIndex index;
    TEST_ASSERT_TRUE(index.reset(marks, 0));
    drawn_pattern::MarkedPhase clock;
    const drawn_pattern::PhaseLimits limits{.5, 1, 4};
    for (int i = 0; i < 2500; ++i) clock.advance(.002, limits, index, .5);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0, clock.phase);
    for (int i = 0; i < 60000; ++i) clock.brakeForUser(.002, limits);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0, clock.phase);
    for (int i = 0; i < 4999; ++i) clock.advance(.002, limits, index, .5);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0, clock.phase);
    for (int i = 0; i < 5; ++i) clock.advance(.002, limits, index, .5);
    TEST_ASSERT_GREATER_THAN_DOUBLE(0, clock.phase);
}

void test_paired_holds_commit_every_cursor_and_stop_at_the_same_phase() {
    const char curve[] = "0003091625364f688398b1c9dae9f6fcfffcf6e9dac9b19883684f3625160903";
    std::vector<int32_t> plain;
    double finalPhase = 0;
    for (bool compact : {false,true}) {
        frames.clear();
        motor.from = -240;
        motor.to = motor.position = 0;
        motor.strength = .05;
        motor.acceleration = 50000;
        motor.speed = 1600;
        compactHolds = compact;
        stopAtFrame = 7350;
        TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted),
                int(startDrawnPattern(curve, sizeof(curve)-1, 12000, revision, true)));
        for (unsigned t = 0; frames.size() < stopAtFrame && t < 10000; ++t) tick();
        TEST_ASSERT_EQUAL_UINT32(stopAtFrame, frames.size());
        const double phase = motionPhase.phase;
        cancelDrawnPattern();
        PatternFeedGap gap;
        PatternWork costs;
        TEST_ASSERT_TRUE(stoppedPatternFeedGap(gap));
        TEST_ASSERT_TRUE(stoppedPatternWork(costs));
        TEST_ASSERT_EQUAL_UINT32(stopAtFrame, gap.frame);
        TEST_ASSERT_EQUAL_UINT32(stopAtFrame, costs.frames);
        if (compact) {
            TEST_ASSERT_TRUE(frames == plain);
            TEST_ASSERT_EQUAL_DOUBLE(finalPhase,phase);
            TEST_ASSERT_GREATER_THAN_UINT32(1500,gap.combinedHoldFrames);
            TEST_ASSERT_LESS_THAN_UINT32(stopAtFrame-1000,costs.submissions);
        } else {
            plain = frames;
            finalPhase = phase;
            TEST_ASSERT_EQUAL_UINT32(0,gap.combinedHoldFrames);
        }
        tick(10);
        TEST_ASSERT_EQUAL_UINT32(stopAtFrame,frames.size());
    }
}

void test_feed_gap_retains_driver_retry_after_a_single_accepted_edge() {
    using drawn_pattern::FrameTiming;
    feedGap = {};
    recordPulseSubmit({io::Frame::Accepted, nullptr, 0, FrameTiming::Running, 0, 6000, 2}, 7295, 10000);
    recordPulseSubmit({io::Frame::Accepted, nullptr, 0, FrameTiming::Running, 0, 0}, 7296, 14000);
    recordPulseSubmit({io::Frame::Retry, "pulse_append", 4}, 7297, 16450);
    TEST_ASSERT_EQUAL_UINT32(7296, feedGap.frame);
    TEST_ASSERT_EQUAL_UINT32(6000, feedGap.previousQueuedUs);
    TEST_ASSERT_EQUAL_UINT32(4000, feedGap.previousAppendIntervalUs);
    TEST_ASSERT_EQUAL_UINT32(1, feedGap.otherRetries);
    TEST_ASSERT_EQUAL_UINT32(2450, feedGap.lastOtherOffsetUs);
    TEST_ASSERT_EQUAL_INT8(4, feedGap.lastOtherDriverCode);
    TEST_ASSERT_EQUAL_UINT32(1, feedGap.combinedHoldFrames);
    recordPulseSubmit({io::Frame::Accepted, nullptr, 4, FrameTiming::Running, 0, 2000}, 7297, 17000);
    TEST_ASSERT_EQUAL_UINT32(1, feedGap.otherRetries);
    TEST_ASSERT_EQUAL_INT8(4, feedGap.lastOtherDriverCode);
    TEST_ASSERT_EQUAL_UINT32(0, feedGap.lastOtherOffsetUs);
}

void test_stopped_service_cost_retains_failing_turn_and_longest_interval() {
    motor.speed = 200;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start(true)));
    tick(8);
    feedback = {io::Frame::Rejected, "pulse_underrun", 0, drawn_pattern::FrameTiming::Running, 3900};
    const auto frame = frames.size();
    tick();
    PatternWork retained;
    TEST_ASSERT_TRUE(stoppedPatternWork(retained));
    TEST_ASSERT_EQUAL_UINT32(frame, retained.lastService.frame);
    TEST_ASSERT_EQUAL_UINT32(clockMs, retained.lastService.atMs);
    TEST_ASSERT_EQUAL_UINT32(3900, retained.lastService.commandGateUs);
    TEST_ASSERT_TRUE(retained.longestService.intervalUs() >= retained.lastService.intervalUs());
    tick(2);
    PatternWork later;
    TEST_ASSERT_TRUE(stoppedPatternWork(later));
    TEST_ASSERT_EQUAL_UINT32(retained.lastService.atMs, later.lastService.atMs);
}

void test_pause_drain_deadline_and_resume_position_fail_closed() {
    motor.speed = 4000;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start(true)));
    tick(100);
    motor.speed = 0;
    parkState = io::Park::Draining;
    for (unsigned i=0; i<2000 && parkPolls==0; ++i) tick();
    TEST_ASSERT_GREATER_THAN_UINT(0, parkPolls);
    tick(25);
    TEST_ASSERT_EQUAL_STRING("park_position",drawnPatternStatus().lastStop.reason);
    motor.speed = 4000;
    parkState = io::Park::Parked;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted), int(start(true)));
    tick(100);
    motor.speed=0;
    tick(1000);
    TEST_ASSERT_EQUAL_STRING("paused",drawnPatternStatus().speed.state);
    motor.acceleration=0;
    tick(200);
    TEST_ASSERT_TRUE(drawnPatternActive());
    motor.acceleration=8000;
    motor.speed=2000;
    parkState=io::Park::Failed;
    tick();
    TEST_ASSERT_EQUAL_STRING("park_position",drawnPatternStatus().lastStop.reason);
}

void test_paused_remote_lease_and_finite_entry_remain_guarded() {
    deferEntry=true;
    motor.speed=.001;
    TEST_ASSERT_EQUAL_INT(int(PatternStart::Accepted),int(start()));
    tick(40);
    TEST_ASSERT_EQUAL_INT(0, entrySubmissions);
    motor.speed=500;
    tick();
    motor.moving=true;
    tick(2);
    motor.speed=0;
    tick(10);
    TEST_ASSERT_EQUAL_INT(1,entryPauses);
    TEST_ASSERT_EQUAL_INT(1,entrySubmissions);
    motor.moving=false;
    tick(100);
    TEST_ASSERT_EQUAL_STRING("waiting",drawnPatternStatus().speed.state);
    deferEntry=false;
    motor.speed=500;
    tick(50);
    TEST_ASSERT_EQUAL_INT(2,entrySubmissions);
    motor.speed=0;
    tick(1200);
    TEST_ASSERT_EQUAL_STRING("paused",drawnPatternStatus().speed.state);
    TEST_ASSERT_TRUE(renewDrawnPattern(revision));
    tick(drawn_pattern::leaseMs);
    TEST_ASSERT_EQUAL_STRING("lease_expired",drawnPatternStatus().lastStop.reason);
    motor.speed=20000;
    tick();
    TEST_ASSERT_FALSE(drawnPatternActive());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_worker_allocation_failure_leaves_motor_and_serial_available);
    RUN_TEST(test_retired_planner_keeps_its_own_source_and_cannot_publish_after_stop);
    RUN_TEST(test_worker_retargets_from_last_accepted_frame_with_continuous_slider_changes);
    RUN_TEST(test_live_clock_survives_observed_22ms_without_background_rows_and_stops_on_real_exhaustion);
    RUN_TEST(test_two_planners_publish_one_ordinal_and_an_explicit_stop_during_assist_retains_nothing);
    RUN_TEST(test_live_positive_retarget_recharges_low_peripheral_and_stamps_only_latest_boundary);
    RUN_TEST(test_steady_raw_recharges_with_ready_frames_without_a_live_edit);
    RUN_TEST(test_saved_marked_speed_change_refills_through_the_first_fading_gain_command);
    RUN_TEST(test_real_slow_to_fast_boundary_commits_its_actual_frame_clock_during_refill);
    RUN_TEST(test_steady_recharge_rechecks_guard_before_its_next_prepared_row);
    RUN_TEST(test_guard_is_rechecked_before_extra_handoff_recharge_row);
    RUN_TEST(test_owner_lease_expiring_at_a_ready_pulse_boundary_stops_at_last_accepted_row);
    RUN_TEST(test_urgent_zero_and_explicit_stop_cancel_a_pending_positive_handoff);
    RUN_TEST(test_rejected_next_row_does_not_hide_a_started_prefill);
    RUN_TEST(test_raw_low_water_reenters_with_unlocked_owner_and_bounded_cpu_turns);
    RUN_TEST(test_operator_pause_preserves_remaining_authored_hold);
    RUN_TEST(test_paired_holds_commit_every_cursor_and_stop_at_the_same_phase);
    RUN_TEST(test_feed_gap_retains_driver_retry_after_a_single_accepted_edge);
    RUN_TEST(test_stopped_service_cost_retains_failing_turn_and_longest_interval);
    RUN_TEST(test_paused_remote_lease_and_finite_entry_remain_guarded);
    RUN_TEST(test_pause_drain_deadline_and_resume_position_fail_closed);
    RUN_TEST(test_indefinite_zero_parks_and_resumes_same_phase_on_both_backends);
    RUN_TEST(test_raw_queue_acquisition_restamps_delayed_stationary_entry);
    RUN_TEST(test_retry_reuses_frame_but_replans_for_live_inputs_and_safety);
    RUN_TEST(test_refill_turn_is_bounded_and_retains_work_only_after_stop);
    RUN_TEST(test_remote_long_hold_keeps_lease_stop_and_guard_live_on_both_backends);
    RUN_TEST(test_full_pause_grid_releases_decoded_pages_and_survives_zero_on_both_backends);
    RUN_TEST(test_high_ordinal_markers_survive_controller_copy_and_give_exact_pause);
    RUN_TEST(test_modifier_zero_reproduces_each_original_target_and_strength_writes_obey_ownership);
    RUN_TEST(test_marked_plateau_streams_six_hz_through_both_backends_and_hard_stop);
    RUN_TEST(test_rate_stamp_history_uses_accepted_frames_and_preserves_real_prefill_time);
    RUN_TEST(test_accepted_entry_waits_for_backend_without_reissuing_move);
    RUN_TEST(test_finite_entry_applies_only_changed_caps_and_stops_before_playback);
    RUN_TEST(test_unstarted_entry_fails_at_deadline_and_guard_remains_immediate);
    RUN_TEST(test_only_leased_current_owner_changes_live_speed_and_zero_retains_ownership);
    RUN_TEST(test_full_stroke_high_speed_and_live_changes_preserve_phase_and_limits);
    RUN_TEST(test_verified_prime_tail_freezes_phase_and_preserves_raw_starvation_deadline);
    RUN_TEST(test_prime_tail_still_obeys_stop_zero_guard_lease_owner_and_service_loss);
    RUN_TEST(test_asymmetric_extension_loop_enters_at_shallow_end_and_reaches_deep_end);
    RUN_TEST(test_first_stop_diagnostic_survives_idle_stops_and_clears_only_at_accepted_start);
    RUN_TEST(test_zero_speed_arms_but_expiry_never_restarts);
    RUN_TEST(test_remote_renewal_zero_pause_and_no_restart_after_stop);
    RUN_TEST(test_local_recall_survives_unrelated_remote_lease_but_obeys_user_controls);
    RUN_TEST(test_guard_rejection_and_stale_task_do_not_touch_a_new_owner);
    RUN_TEST(test_no_phase_catchup_after_backpressure_or_a_scheduling_gap);
    RUN_TEST(test_stored_lookup_preflight_rejects_active_legacy_or_drawn_motion);
    return UNITY_END();
}
