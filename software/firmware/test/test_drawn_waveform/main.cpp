#include <unity.h>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include "drawn_waveform.h"
#include "drawn_phase_rate.h"
#include "drawn_pulse_queue.h"

using drawn_pattern::Waveform;
void setUp() {}
void tearDown() {}

static std::string encode(const std::array<int, 16>& points) {
    char hex[33]{};
    for (size_t i = 0; i < points.size(); ++i)
        std::snprintf(hex + 2 * i, 3, "%02x", points[i]);
    return hex;
}
const std::array<int, 16> asymmetric{0, 8, 27, 51, 90, 142, 228, 255,
                                     255, 196, 167, 93, 60, 60, 22, 3};

void test_wire_bounds_are_checked_before_motion() {
    Waveform curve;
    auto hex = encode(asymmetric);
    TEST_ASSERT_TRUE(curve.decode(hex.data(), hex.size(), 500));
    TEST_ASSERT_TRUE(curve.decode(hex.data(), hex.size(), 60000));
    TEST_ASSERT_FALSE(curve.decode(hex.data(), hex.size(), 499));
    TEST_ASSERT_FALSE(curve.decode(hex.data(), hex.size(), 60001));
    TEST_ASSERT_FALSE(curve.decode(nullptr, 32, 500));
    TEST_ASSERT_FALSE(curve.decode(hex.data(), 30, 500));
    TEST_ASSERT_FALSE(curve.decode(hex.data(), 33, 500));
    hex[7] = '\0';
    TEST_ASSERT_FALSE(curve.decode(hex.data(), hex.size(), 500));
    const std::string largest(2 * drawn_pattern::maxSamples, 'F');
    TEST_ASSERT_TRUE(curve.decode(largest.data(), largest.size(), 500));
    TEST_ASSERT_FALSE(curve.decode(largest.data(), largest.size() + 2, 500));
}

void test_passes_through_all_samples_no_overshoot_and_continuous_velocity() {
    Waveform curve;
    const auto shape = encode(asymmetric);
    TEST_ASSERT_TRUE(curve.decode(shape.data(), shape.size(), 2000));
    constexpr double epsilon = .000001;
    for (size_t i = 0; i < asymmetric.size(); ++i) {
        const double phase = double(i) / asymmetric.size();
        const double p = curve.position(phase);
        TEST_ASSERT_FLOAT_WITHIN(.00001f, asymmetric[i] / 255.f, p);
        const double left = (p - curve.position(phase - epsilon)) / epsilon;
        const double right = (curve.position(phase + epsilon) - p) / epsilon;
        TEST_ASSERT_FLOAT_WITHIN(.002f, left, right);
        const int next = asymmetric[(i + 1) % asymmetric.size()];
        if (i == 4) TEST_ASSERT_GREATER_THAN_FLOAT(1, left);
        for (size_t k = 0; k < 100; ++k) {
            const double v = curve.position(phase + k / 1600.0) * 255.0;
            TEST_ASSERT_TRUE(v >= std::min(asymmetric[i], next) - .00001);
            TEST_ASSERT_TRUE(v <= std::max(asymmetric[i], next) + .00001);
        }
    }
}

void test_phase_starts_at_a_resting_extremum_and_loops_without_a_jump() {
    Waveform curve;
    auto rotated = asymmetric;
    std::rotate(rotated.begin(), rotated.begin() + 5, rotated.end());
    const auto shape = encode(rotated);
    TEST_ASSERT_TRUE(curve.decode(shape.data(), shape.size(), 2000));
    TEST_ASSERT_FLOAT_WITHIN(.00001, 0, curve.position(0));
    TEST_ASSERT_FLOAT_WITHIN(.0001, 0, curve.position(.000001) / .000001);
    for (size_t i = 0; i <= 32; ++i)
        TEST_ASSERT_FLOAT_WITHIN(.000001, curve.position(i / 32.), curve.position(1 + i / 32.));
}

void test_authored_extension_maps_every_asymmetric_sample_to_the_native_motor_frame() {
    Waveform curve;
    const auto shape = encode(asymmetric);
    TEST_ASSERT_TRUE(curve.decode(shape.data(), shape.size(), 2000));
    // A 255-step-wide window makes the expected extension in steps exact.
    for (size_t i = 0; i < asymmetric.size(); ++i) {
        const double phase = double(i) / asymmetric.size();
        TEST_ASSERT_EQUAL_INT32(-100 - asymmetric[i], curve.nativeTarget(phase, -355, -100));
    }
    TEST_ASSERT_EQUAL_INT32(-100, curve.nativeTarget(1, -355, -100));
}

void test_time_stretch_bounds_entire_curve_at_speed_and_acceleration_caps() {
    Waveform curve;
    const auto shape = encode(asymmetric);
    TEST_ASSERT_TRUE(curve.decode(shape.data(), shape.size(), 500));
    for (double acceleration : {50., 20000.}) {
        const double span = 2300, maxSpeed = 700;
        const double seconds = curve.safePeriodMs(span, maxSpeed, acceleration) / 1000.;
        TEST_ASSERT_GREATER_THAN_FLOAT(.5, seconds);
        constexpr double fraction = 1. / 32768;
        const double dt = seconds * fraction;
        double p = curve.position(-fraction) * span;
        double v = (p - curve.position(-2 * fraction) * span) / dt;
        for (size_t i = 0; i < 32768; ++i) {
            const double next = curve.position(i * fraction) * span;
            const double speed = (next - p) / dt;
            TEST_ASSERT_TRUE(std::abs(speed) <= maxSpeed + .001);
            TEST_ASSERT_TRUE(std::abs(speed - v) / dt <= acceleration + .001);
            p = next;
            v = speed;
        }
    }
    TEST_ASSERT_EQUAL_UINT32(0, curve.safePeriodMs(2300, 0, 100));
}

void test_step_frames_track_multiple_periods_without_waypoint_stops_or_drift() {
    Waveform curve;
    const auto shape = encode(asymmetric);
    TEST_ASSERT_TRUE(curve.decode(shape.data(), shape.size(), 4000));
    int32_t previous=curve.nativeTarget(0,-2200,-200);
    const auto initial=previous;
    int32_t beforeMiddle=0;
    int64_t traveled=0;
    for (size_t n=1;n<=6000;++n) {
        const auto target=curve.nativeTarget(n/2000.,-2200,-200);
        traveled+=int64_t(target)-previous;
        if (n==498) beforeMiddle=target;
        if (n==502) TEST_ASSERT_GREATER_THAN_INT32(8,beforeMiddle-target);
        if (n%2000==0) TEST_ASSERT_EQUAL_INT32(initial,target);
        previous=target;
    }
    TEST_ASSERT_EQUAL_INT32(0,traveled);
}

namespace {
struct PulseCommand { uint16_t ticks; uint8_t steps; bool count_up; };
struct InterruptingQueue {
    unsigned count = 0, starts = 0;
    bool running = false, discarded = false, drainOnAppend = false;
    int8_t startResult = 0;
    PulseCommand last{};
    bool retireImmediately = false;
    unsigned moveTimedCalls = 0;
    int32_t tail = -896;
    unsigned queueEntries() const { return count; }
    int32_t getPositionAfterCommandsCompleted() const { return tail; }
    bool isQueueRunning() const { return running; }
    uint16_t getMaxSpeedInTicks() const { return 80; }
    uint32_t ticksInQueue() const { return 0; }
    int moveTimed(int16_t steps,uint32_t duration,uint32_t *actual,bool start) {
        ++moveTimedCalls;
        const bool wasEmpty=count==0;
        const auto n=static_cast<uint8_t>(std::abs(steps));
        PulseCommand cmd{static_cast<uint16_t>(duration/n),n,steps>0};
        *actual=cmd.ticks*n;
        const auto code=addQueueEntry(&cmd,start);
        return code ? code : (wasEmpty?7:0);
    }
    int addQueueEntry(const PulseCommand *command, bool start) {
        if (command && !discarded) {
            if (drainOnAppend) { count = 0; running = false; }
            last=*command;
            if (!retireImmediately || !running) ++count;
            tail += command->count_up ? command->steps : -int(command->steps);
        }
        if (start) {
            ++starts;
            if (startResult) return startResult;
            // The MCPWM compare interrupt may retire the first hold before
            // the producer runs again, even for a long programmed interval.
            if (count) --count;
            running = count > 0;
        }
        return 0;
    }
};
}

void test_raw_queue_prefills_leading_holds_and_starts_once_with_bounded_lookahead() {
    using namespace drawn_pattern;
    PulseQueue q;
    InterruptingQueue driver;
    for (unsigned i = 0; i < pulseQueueFrames; ++i) {
        const auto result = q.submit<PulseCommand>(driver, -896, 32000, 32768, 3200, 7);
        TEST_ASSERT_EQUAL_INT(int(Frame::Accepted), int(result.state));
        TEST_ASSERT_EQUAL_UINT(i + 1 == pulseQueueFrames, driver.starts);
    }
    TEST_ASSERT_TRUE(driver.running);
    TEST_ASSERT_FALSE(q.filling());
    TEST_ASSERT_EQUAL_INT(int(Frame::Accepted), int(q.submit<PulseCommand>(driver, -895, 32000, 32768, 3200, 7).state));
    TEST_ASSERT_EQUAL_INT(int(Frame::Retry), int(q.submit<PulseCommand>(driver, -894, 32000, 32768, 3200, 7).state));
    TEST_ASSERT_EQUAL_UINT(pulseQueueFrames, driver.count);
    driver.count = 1;
    driver.drainOnAppend = true;
    const auto underrun = q.submit<PulseCommand>(driver, -894, 32000, 32768, 3200, 7);
    TEST_ASSERT_EQUAL_STRING("pulse_underrun", underrun.reason);
    TEST_ASSERT_EQUAL_UINT(1, driver.starts);
    TEST_ASSERT_FALSE(driver.running);
    TEST_ASSERT_EQUAL_INT(int(Frame::Rejected), int(q.submit<PulseCommand>(driver, -894, 32000, 32768, 3200, 7).state));
    TEST_ASSERT_EQUAL_UINT(1, driver.starts);
}

void test_unstarted_raw_queue_preserves_physical_position_and_reports_driver_errors() {
    using namespace drawn_pattern;
    PulseQueue q;
    InterruptingQueue driver;
    driver.discarded = true;
    const auto discarded = q.submit<PulseCommand>(driver, -896, 32000, 32768, 3200, 7);
    TEST_ASSERT_EQUAL_STRING("pulse_discarded", discarded.reason);
    TEST_ASSERT_EQUAL_INT32(-896, q.initialPosition());
    TEST_ASSERT_TRUE(q.filling());
    TEST_ASSERT_FALSE(driver.running);
    q.reset();
    TEST_ASSERT_FALSE(q.owned());
    driver.discarded = false;
    driver.startResult = -2;
    for (unsigned i = 1; i < pulseQueueFrames; ++i)
        TEST_ASSERT_EQUAL_INT(int(Frame::Accepted), int(q.submit<PulseCommand>(driver, -895, 32000, 32768, 3200, 7).state));
    const auto refused = q.submit<PulseCommand>(driver, -894, 32000, 32768, 3200, 7);
    TEST_ASSERT_EQUAL_INT(int(Frame::Rejected), int(refused.state));
    TEST_ASSERT_EQUAL_STRING("pulse_start", refused.reason);
    TEST_ASSERT_EQUAL_INT(-2, refused.driverResult);
    TEST_ASSERT_TRUE(q.filling());
    TEST_ASSERT_FALSE(driver.running);
    TEST_ASSERT_EQUAL_INT32(-896, q.initialPosition());
    TEST_ASSERT_EQUAL_INT32(-895, driver.tail); // Most recent sampled group remains software-only.
}

void test_timed_edges_follow_subframe_crossings_in_both_directions_and_floor_ticks_do_not_accumulate() {
    using namespace drawn_pattern;
    constexpr int frameTicks=32000;
    for (const int sign : {-1,1}) {
        InterruptingQueue driver;
        driver.tail=0;
        driver.retireImmediately=true;
        PulseQueue q;
        // Twenty-three hundred steps/s: 4.6 steps per frame, with crossings
        // between period boundaries. The first 2ms is a true stationary hold.
        const double speed=4.6;
        uint64_t elapsed=0;
        double maxError=0;
        int pulses=0;
        for (unsigned i=0;i<405;++i) {
            const auto position=[&](double u) {
                return sign*static_cast<int32_t>(std::lround(speed*std::clamp(double(i)+u-1.,0.,400.)));
            };
            const int32_t from=position(0), next=position(1);
            const uint16_t crossing=firstStepCrossing(from,next,position);
            const auto f=q.submit<PulseCommand>(driver,next,frameTicks,crossing,3200,7);
            TEST_ASSERT_EQUAL_INT(int(Frame::Accepted),int(f.state));
            const auto cmd=driver.last;
            for (unsigned k=0;k<cmd.steps;++k) {
                const double measured=elapsed+k*cmd.ticks;
                const double ideal=frameTicks*(1+(pulses+.5)/speed);
                // The final group is followed by a hold; its inner pulses
                // may have unequal physical and analytical intervals.
                if (pulses<1836) maxError=std::max(maxError,std::abs(measured-ideal));
                TEST_ASSERT_EQUAL(sign,cmd.count_up?1:-1);
                ++pulses;
            }
            if (cmd.steps==0 && pulses>0) TEST_ASSERT_EQUAL(sign,cmd.count_up?1:-1);
            elapsed+=uint32_t(cmd.ticks)*std::max(1,int(cmd.steps));
        }
        TEST_ASSERT_EQUAL_UINT(1840,pulses);
        TEST_ASSERT_EQUAL_INT32(sign*1840,driver.tail);
        TEST_ASSERT_GREATER_THAN_UINT(300,driver.moveTimedCalls);
        TEST_ASSERT_LESS_THAN_FLOAT(30.,maxError); // <2 microseconds including integer ticks
    }
}

void test_periodic_quintic_preserves_acceleration_holds_and_bounds_the_three_derivatives() {
    Waveform curve;
    const auto hex = encode(asymmetric);
    TEST_ASSERT_TRUE(curve.decode(hex.data(),hex.size(),500));
    const double epsilon = 1.e-8;
    for (unsigned k=0;k<16;++k) {
        const double f=k/16.;
        const double jump=std::abs(curve.phaseDerivative(f-epsilon,2)-curve.phaseDerivative(f+epsilon,2));
        TEST_ASSERT_TRUE(jump <= 2.01*epsilon*curve.maximumThird(1) + 1.e-7);
        if (asymmetric[k]==asymmetric[(k+1)%16]) {
            for (unsigned order=1;order<=3;++order)
                TEST_ASSERT_FLOAT_WITHIN(1.e-7,0,curve.phaseDerivative(f+1./32,order));
        }
    }
    for (unsigned i=0;i<8192;++i) {
        const double f=i/8192.;
        TEST_ASSERT_TRUE(std::abs(curve.phaseDerivative(f,1)) <= curve.maximumSlope(1)+1.e-8);
        TEST_ASSERT_TRUE(std::abs(curve.phaseDerivative(f,2)) <= curve.maximumCurvature(1)+1.e-8);
        TEST_ASSERT_TRUE(std::abs(curve.phaseDerivative(f,3)) <= curve.maximumThird(1)+1.e-7);
    }
}

void test_high_precision_sine_tracks_the_authored_third_derivative() {
    constexpr unsigned n=32;
    constexpr double tau=6.283185307179586;
    char q8[2*n+1]{},q16[4*n+1]{};
    for (unsigned i=0;i<n;++i) {
        const double y=(1-std::cos(tau*i/n))*.5;
        std::snprintf(q8+2*i,3,"%02x",static_cast<unsigned>(std::lround(y*255)));
        std::snprintf(q16+4*i,5,"%04x",static_cast<unsigned>(std::lround(y*65535)));
    }
    Waveform fine,coarse;
    TEST_ASSERT_TRUE(fine.decode(q16,4*n,4000,drawn_pattern::SampleFormat::Q16));
    TEST_ASSERT_TRUE(coarse.decode(q8,2*n,4000));
    TEST_ASSERT_FALSE(coarse.decode(q8,2*n-2,4000,drawn_pattern::SampleFormat::Q16));
    double errFine=0,errCoarse=0;
    for (unsigned i=0;i<8192;++i) {
        const double f=(i+.5)/8192;
        const double authored=-.5*tau*tau*tau*std::sin(tau*f);
        errFine+=std::pow(fine.phaseDerivative(f,3)-authored,2);
        errCoarse+=std::pow(coarse.phaseDerivative(f,3)-authored,2);
    }
    TEST_ASSERT_LESS_THAN_FLOAT(8.,std::sqrt(errFine/8192));
    TEST_ASSERT_TRUE(errFine < .01*errCoarse);
}

void test_retargetable_rate_uses_continuous_acceleration_and_bounded_jerk() {
    drawn_pattern::PhaseClock timer;
    constexpr double dt=.002, aLimit=1.25, jLimit=4.5;
    double phase=0;
    for (unsigned i=0;i<5000;++i) {
        const double goal=i<80?.8:i<110?.1:i<160?.9:i<210?.02:i<310?.85:.4;
        const double beforeRate=timer.rate(),beforeAcceleration=timer.acceleration();
        timer.setGoal(goal,aLimit,jLimit);
        TEST_ASSERT_FLOAT_WITHIN(1.e-12,beforeAcceleration,timer.acceleration());
        const double step=timer.advance(dt);phase+=step;
        TEST_ASSERT_TRUE(step>=0 && step<=.9*dt+1.e-12);
        TEST_ASSERT_TRUE(timer.rate()>=0 && timer.rate()<=.9+1.e-12);
        TEST_ASSERT_TRUE(std::abs(timer.acceleration())<=aLimit+1.e-10);
        TEST_ASSERT_TRUE(std::abs(timer.acceleration()-beforeAcceleration)<=jLimit*dt+1.e-10);
        TEST_ASSERT_TRUE(std::abs(timer.rate()-beforeRate)<=aLimit*dt+1.e-10);
    }
    TEST_ASSERT_FLOAT_WITHIN(1.e-9,.4,timer.rate());
    TEST_ASSERT_FLOAT_WITHIN(1.e-9,0,timer.acceleration());
    TEST_ASSERT_GREATER_THAN_FLOAT(3.,phase);
    // A rate of one is reached from rest after two seconds, with one cycle
    // elapsed. The triangular acceleration has slope +1 then -1.
    drawn_pattern::PhaseClock triangle;
    triangle.setGoal(1,1,1);
    TEST_ASSERT_FLOAT_WITHIN(1.e-10,1./6,triangle.advance(1));
    TEST_ASSERT_FLOAT_WITHIN(1.e-10,.5,triangle.rate());
    TEST_ASSERT_FLOAT_WITHIN(1.e-10,5./6,triangle.advance(1));
    TEST_ASSERT_FLOAT_WITHIN(1.e-10,1,triangle.rate());
    TEST_ASSERT_FLOAT_WITHIN(1.e-10,0,triangle.acceleration());
}

void test_time_domain_derivatives_remain_bounded_during_repeated_live_retargets() {
    Waveform curve;
    const auto hex = encode(asymmetric);
    TEST_ASSERT_TRUE(curve.decode(hex.data(),hex.size(),500));
    drawn_pattern::PhaseClock clock;
    constexpr double span=1800, acceleration=100000, dt=.0005;
    const auto limits=curve.rateLimits(span,acceleration);
    double phase=0, previousAcceleration=0;
    for (unsigned frame=0;frame<60000;++frame) {
        const double speed=(frame/3100)%3==0?650.:((frame/3100)%3==1?20000.:1400.);
        clock.setGoal(1000./curve.safePeriodMs(span,speed,acceleration/2),
                      limits.acceleration,limits.jerk);
        phase+=clock.advance(dt);
        const double r=clock.rate(), a=clock.acceleration();
        const double positionAcceleration=span*(curve.phaseDerivative(phase,2)*r*r+
                                               curve.phaseDerivative(phase,1)*a);
        TEST_ASSERT_TRUE(std::abs(positionAcceleration)<=acceleration);
        TEST_ASSERT_TRUE(std::abs(positionAcceleration-previousAcceleration)<=
                         dt*curve.jerkBudget(acceleration)+1e-7);
        previousAcceleration=positionAcceleration;
    }
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_timed_edges_follow_subframe_crossings_in_both_directions_and_floor_ticks_do_not_accumulate);
    RUN_TEST(test_time_domain_derivatives_remain_bounded_during_repeated_live_retargets);
    RUN_TEST(test_periodic_quintic_preserves_acceleration_holds_and_bounds_the_three_derivatives);
    RUN_TEST(test_high_precision_sine_tracks_the_authored_third_derivative);
    RUN_TEST(test_retargetable_rate_uses_continuous_acceleration_and_bounded_jerk);
    RUN_TEST(test_authored_extension_maps_every_asymmetric_sample_to_the_native_motor_frame);
    RUN_TEST(test_raw_queue_prefills_leading_holds_and_starts_once_with_bounded_lookahead);
    RUN_TEST(test_unstarted_raw_queue_preserves_physical_position_and_reports_driver_errors);
    RUN_TEST(test_wire_bounds_are_checked_before_motion);
    RUN_TEST(test_passes_through_all_samples_no_overshoot_and_continuous_velocity);
    RUN_TEST(test_phase_starts_at_a_resting_extremum_and_loops_without_a_jump);
    RUN_TEST(test_time_stretch_bounds_entire_curve_at_speed_and_acceleration_caps);
    RUN_TEST(test_step_frames_track_multiple_periods_without_waypoint_stops_or_drift);
    return UNITY_END();
}
