// Use the installed FastAccelStepper PC test port for the ramp planner and
// emulate only the queue position normally advanced by the hardware.
#include <cassert>
#include <cstdint>
#include <cstdio>

#include "drawn_pulse_queue.h"
#include "log2/Log2Representation.cpp"
#include "fas_ramp/RampControl.cpp"
#include "fas_ramp/RampCalculator.cpp"
#include "fas_ramp/RampGenerator.cpp"

void noInterrupts() {}
void interrupts() {}
void inject_fill_interrupt(int) {}

struct PulseDriver {
    RampGenerator ramp;
    queue_end_s queue{};
    bool movingStop = false;
    bool hardwareActive = false;

    bool isRunning() { return ramp.isRampGeneratorActive(); }
    bool isQueueRunning() { return hardwareActive; }
    bool isQueueEmpty() { return !hardwareActive; }
    int32_t getCurrentPosition() { return queue.pos; }
    void forceStop() { movingStop = true; ramp.forceStop(); }
    void forceStopAndNewPosition(int32_t position) {
        assert(!hardwareActive);
        ramp.stopRamp();
        queue.pos = position;
        ramp.setTargetPosition(position);
    }
    unsigned nextCommand() {
        NextCommand entry{};
        ramp.getNextCommand(&queue, &entry);
        const auto &pulse = entry.command;
        queue.pos += pulse.count_up ? pulse.steps : -pulse.steps;
        if (pulse.steps) queue.count_up = pulse.count_up;
        ramp.afterCommandEnqueued(&entry);
        hardwareActive = pulse.ticks != 0;
        return pulse.steps;
    }
    unsigned drain() {
        unsigned steps = 0;
        unsigned frames = 0;
        while (isRunning()) {
            assert(++frames < 100000);
            steps += nextCommand();
        }
        hardwareActive = false;
        return steps;
    }
};

int main() {
    for (const int direction : {-1, 1}) {
        PulseDriver driver;
        driver.ramp.init();
        driver.ramp.setSpeedInTicks(16000000ul / 200);
        driver.ramp.setAcceleration(50000);
        const int entry = direction * 1762;
        assert(driver.ramp.moveTo(entry, &driver.queue) == MOVE_OK);
        assert(driver.drain() == 1762);
        driver.queue.pos += direction * 162; // raw pulses after finite entry
        drawn_pattern::stopPulseRamp(driver);
        assert(!driver.movingStop);
        assert(driver.ramp.moveTo(entry, &driver.queue) == MOVE_OK);
        assert(driver.drain() == 162 && driver.queue.pos == entry);
        assert(driver.ramp.moveTo(0, &driver.queue) == MOVE_OK);
        drawn_pattern::stopPulseRamp(driver);
        assert(!driver.movingStop);
        assert(driver.drain() == 0 && driver.queue.pos == entry);
        assert(driver.ramp.moveTo(0, &driver.queue) == MOVE_OK);
        driver.nextCommand();
        const int positionAtStop = driver.queue.pos;
        drawn_pattern::stopPulseRamp(driver);
        assert(driver.movingStop);
        assert(driver.drain() == 0 && driver.queue.pos == positionAtStop);
    }
    std::fprintf(stderr, "Idle replay and active-stop checks passed in both directions.\n");
}
