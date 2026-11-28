#pragma once

#include <deque>
#include <numeric>

// Consume each submitted group on a clock independent of the producer. As in
// FAS, ticksInQueue excludes the command currently processed by the interrupt.
struct DrainDriver {
    std::deque<uint32_t> durations;
    uint32_t activeTicks = 0, maxDrainTicks = 0;
    int32_t tail = 0;
    bool running = false;
    size_t queueEntries() const { return durations.size(); }
    bool isQueueRunning() const { return running; }
    uint16_t getMaxSpeedInTicks() const { return 80; }
    int32_t getPositionAfterCommandsCompleted() const { return tail; }
    uint32_t ticksInQueue() const {
        return durations.empty() ? 0 : std::accumulate(durations.begin()+1, durations.end(), 0u);
    }
    void advanceUs(uint32_t us) {
        uint32_t time = us*16;
        while (running && time >= activeTicks) {
            time -= activeTicks;
            durations.pop_front();
            running = !durations.empty();
            if (running) activeTicks = durations.front();
        }
        if (running) activeTicks -= time;
    }
    int addQueueEntry(const PulseCommand *cmd, bool start) {
        if (cmd) {
            durations.push_back(cmd->ticks * std::max(1, int(cmd->steps)));
            tail += cmd->count_up ? cmd->steps : -int(cmd->steps);
        }
        if (start) {
            running = true;
            activeTicks = durations.front();
        }
        const uint32_t drain = ticksInQueue() + (running ? activeTicks : durations.front());
        maxDrainTicks = std::max(maxDrainTicks, drain);
        return 0;
    }
    int moveTimed(int16_t steps, uint32_t duration, uint32_t *actual, bool start) {
        const auto n = static_cast<uint8_t>(std::abs(steps));
        PulseCommand cmd{static_cast<uint16_t>(duration/n), n, steps>0};
        *actual = cmd.ticks*n;
        return addQueueEntry(&cmd, start);
    }
};
struct RefillResult {
    unsigned accepted = 0, immediate = 0;
    uint32_t drainUs = 0;
    bool underrun = false;
};

RefillResult replayDrain(int periodMs, unsigned spacing, bool adaptive) {
    using namespace drawn_pattern;
    const char shape[] = "000103060b11181f28313b46515c6874808b97a3aeb9c4ced7e0e7eef4f9fcfefffefcf9f4eee7e0d7cec4b9aea3978b8074685c51463b31281f18110b060301";
    Waveform curve;
    curve.decode(shape, sizeof(shape)-1, 18850);
    PulseQueue queue;
    PulseRefillSchedule schedule;
    DrainDriver driver;
    RefillResult result{};
    bool pending = false;
    for (unsigned turn = 0; turn < 5000 && result.accepted < 2300; ++turn) {
        // Two service disruptions bounded by the observed controller intervals.
        // These are stress inputs. The recording retained aggregate costs.
        unsigned readUs = 350;
        if (turn == 300) readUs = 12883;
        if (turn == 300 + spacing) readUs = 9000;
        driver.advanceUs(readUs);
        bool low = false;
        for (unsigned i = 0; i != 3; ++i) {
            // Measured means were 535 us per submission and 362 us per
            // newly planned frame. A retry reuses its pending calculation.
            driver.advanceUs(pending ? 535 : 535 + 362);
            const double phase = (result.accepted+1)*2./periodMs;
            const int32_t target = curve.nativeTarget(phase,-240,0);
            const int32_t prior = curve.nativeTarget(phase-2./periodMs,-240,0);
            const uint16_t crossing = prior == target ? 32768 :
                firstStepCrossing(prior, target, [&](double f) {
                    return curve.nativeTarget(phase+(f-1)*2./periodMs,-240,0);
                });
            const auto frame=queue.submit<PulseCommand>(driver,target,32000,crossing,3200,7);
            if (frame.state == Frame::Rejected) {
                result.underrun = std::strcmp(frame.reason,"pulse_underrun")==0;
                result.drainUs=driver.maxDrainTicks/16;
                return result;
            }
            pending = frame.state == Frame::Retry;
            if (pending) break;
            ++result.accepted;
            low = i == 2 && frame.queuedUs < refillBelowUs;
        }
        if (adaptive && schedule.immediate(low)) ++result.immediate;
        else driver.advanceUs(1000);
    }
    result.drainUs = driver.maxDrainTicks/16;
    return result;
}
