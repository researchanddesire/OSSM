#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <new>

#include "drawn_modifiers.h"
#include "drawn_phase_rate.h"

namespace drawn_pattern {

    // This index is built once at Play. Frame trials and STEP edge searches
    // copy only the clock/cursor, never the authored marker list.
    class MarkerIndex {
      public:
        bool reset(const Modifiers &items, double firstMinimum) {
            markers = &items;
            offset = firstMinimum;
            holds = 0;
            hasVibration = false;
            for (unsigned i = 0; i < items.count; ++i) {
                if (items.entries[i].isPause()) ++holds;
                if (items.entries[i].isVibration()) hasVibration = true;
            }
            extraPauses.reset(holds > inlineCapacity ? new (std::nothrow)
                                                           uint16_t[holds]
                                                     : nullptr);
            if (holds > inlineCapacity && !extraPauses) return false;
            uint16_t *out = extraPauses ? extraPauses.get() : inlinePauses;
            for (unsigned i = 0; i < items.count; ++i)
                if (items.entries[i].isPause()) *out++ = static_cast<uint16_t>(i);
            return true;
        }

        const Modifier *range(double phase) const {
            if (!hasVibration) return nullptr;
            const double p = authored(phase) * 1000;
            unsigned lo = 0, hi = markers->count;
            while (lo < hi) {
                const unsigned k = (lo + hi) / 2;
                if (markers->entries[k].start <= p)
                    lo = k + 1;
                else
                    hi = k;
            }
            if (!lo || !markers->entries[lo - 1].isVibration() || p >= markers->entries[lo - 1].end) return nullptr;
            return &markers->entries[lo - 1];
        }

        double authored(double phase) const {
            const double p = phase + offset;
            return p - std::floor(p);
        }

        // First pause at/after a threshold in unwrapped playback coordinates.
        double upcoming(double threshold, uint16_t &duration) const {
            const uint16_t *pauses =
                extraPauses ? extraPauses.get() : inlinePauses;
            const double t = threshold + offset;
            double cycle = std::floor(t);
            const double at = (t - cycle) * 1000;
            unsigned lo = 0, hi = holds;
            while (lo < hi) {
                const unsigned k = (lo + hi) / 2;
                if (markers->entries[pauses[k]].start < at - 1e-10)
                    lo = k + 1;
                else
                    hi = k;
            }
            if (lo == holds) {
                lo = 0;
                ++cycle;
            }
            const auto &m = markers->entries[pauses[lo]];
            duration = m.holdMs;
            return cycle + m.start / 1000.0 - offset;
        }

        bool hasVibration = false;
        unsigned holds = 0;

      private:
        const Modifiers *markers = nullptr;
        double offset = 0;
        static constexpr unsigned inlineCapacity = 4;
        uint16_t inlinePauses[inlineCapacity]{};
        std::unique_ptr<uint16_t[]> extraPauses;
    };

    struct PhaseLimits {
        double rate, acceleration, jerk;
    };

    // A peripheral frame samples eleven possible edge times. Solve the
    // braking decision once on that frame, then reuse it for those previews.
    struct FrameBrake {
        bool known = false;
        double when = HUGE_VAL, target = 0;
        uint16_t holdMs = 0;
    };

    // Braking uses the original bounded-jerk phase clock; its analytical stop
    // distance places the flat interval exactly on the artist's chosen
    // coordinate.
    class MarkedPhase {
      public:
        double phase = 0;
        PhaseClock clock;
        void advance(double dt, const PhaseLimits &limits,
                     const MarkerIndex &index, double strength,
                     FrameBrake *frame = nullptr) {
            if (!index.holds || strength <= 0) {
                if (state != Free) skipThrough = target;
                state = Free;
                clock.setGoal(limits.rate, limits.acceleration, limits.jerk);
                phase += clock.advance(dt);
            } else {
                advanceWithPauses(dt, limits, index, strength, frame);
            }
            const double turns = std::floor(phase);
            phase -= turns;
            target -= turns;
            skipThrough = std::max(-2., skipThrough - turns);
        }

      private:
        enum State { Free, Braking, Holding } state = Free;
        double target = 0, skipThrough = -2, holdElapsed = 0;
        uint16_t maximumHoldMs = 0;

        static double stopDistance(PhaseClock clock,
                                   const PhaseLimits &limits) {
            clock.setGoal(0, limits.acceleration, limits.jerk);
            return clock.advance(clock.secondsToGoal() + 1e-12);
        }

        FrameBrake planBrake(double dt, const PhaseLimits &limits,
                             const MarkerIndex &index) const {
            FrameBrake result;
            result.known = true;
            const double front = phase + stopDistance(clock, limits);
            result.target = index.upcoming(
                std::max(skipThrough + 1e-8, front - 1e-10), result.holdMs);
            PhaseClock end = clock;
            const double distance = end.advance(dt);
            if (phase + distance + stopDistance(end, limits) <
                result.target - 1e-12)
                return result;
            double low = 0, high = dt;
            if (front < result.target - 1e-12) {
                for (unsigned k = 0; k < 23; ++k) {
                    const double middle = (low + high) * .5;
                    PhaseClock trial = clock;
                    const double here = phase + trial.advance(middle);
                    if (here + stopDistance(trial, limits) < result.target)
                        low = middle;
                    else
                        high = middle;
                }
            } else
                high = 0;
            result.when = high;
            return result;
        }

        void advanceWithPauses(double dt, const PhaseLimits &limits,
                               const MarkerIndex &index, double strength,
                               FrameBrake *frame) {
            while (dt > 1e-12) {
                if (state == Holding) {
                    const double remaining =
                        maximumHoldMs * .001 * strength - holdElapsed;
                    const double t = std::min(dt, std::max(0., remaining));
                    holdElapsed += t;
                    dt -= t;
                    if (remaining <= t + 1e-12) {
                        skipThrough = target;
                        state = Free;
                    }
                    continue;
                }
                if (state == Braking) {
                    const double t = clock.secondsToGoal();
                    if (t > dt) {
                        phase += clock.advance(dt);
                        return;
                    }
                    clock.advance(t + 1e-12);
                    phase = target;
                    dt = std::max(0., dt - t);
                    state = Holding;
                    continue;
                }
                clock.setGoal(limits.rate, limits.acceleration, limits.jerk);
                const FrameBrake decision = frame && frame->known
                                                ? *frame
                                                : planBrake(dt, limits, index);
                if (frame) {
                    *frame = decision;
                    frame = nullptr;
                }
                if (decision.when > dt) {
                    phase += clock.advance(dt);
                    return;
                }
                phase += clock.advance(decision.when);
                dt -= decision.when;
                clock.setGoal(0, limits.acceleration, limits.jerk);
                target = decision.target;
                maximumHoldMs = decision.holdMs;
                holdElapsed = 0;
                state = Braking;
            }
        }
    };

    // Quintic gain transitions have zero first and second derivatives at either
    // end. Rapid edits coalesce into the most recent target at the next join.
    class StrengthRamp {
      public:
        static constexpr double seconds = .4;
        double value() const {
            const double u = elapsed / seconds;
            return from + (to - from) * u * u * u * (10 + u * (-15 + 6 * u));
        }
        double goal() const { return to; }
        bool atZero() const { return elapsed == seconds && to == 0; }
        void advance(double dt, double requested) {
            while (dt > 0) {
                if (elapsed == seconds) {
                    if (to == requested) return;
                    from = to;
                    to = requested;
                    elapsed = 0;
                }
                const double d = std::min(dt, seconds - elapsed);
                elapsed = std::min(seconds, elapsed + d);
                dt -= d;
            }
        }

      private:
        double from = 0, to = 0, elapsed = seconds;
    };

    // The base path receives 75% of each physical derivative budget when rumble
    // is armed. The remaining quarter covers the complete derivative of the
    // added waveform, including its spatial window, live strength and endpoint
    // bias.
    struct VibrationBudget {
        static constexpr double baseShare = .75;
        static constexpr double hz = 6;
        double span = 0, speed = 0, acceleration = 0, nominalSteps = 0;
        PhaseLimits phase{};
        bool matches(double v, double a) const {
            return v == speed && a == acceleration;
        }

        double amplitude(double width) const {
            if (width == cachedWidth) return cachedAmplitude;
            if (!(width > 0 && span > 0 && speed > 0 && acceleration > 0))
                return 0;
            constexpr double w = 2 * 3.14159265358979323846 * hz;
            const double r = phase.rate / width, a = phase.acceleration / width,
                         j = phase.jerk / width;
            // First three absolute derivatives of 64 u³(1-u)³ are ≤4, 24 and
            // 384.
            const double e1 = 4 * r, e2 = 24 * r * r + 4 * a,
                         e3 = 384 * r * r * r + 72 * r * a + 4 * j;
            constexpr double t = StrengthRamp::seconds;
            constexpr double g1 = 1.875 / t, g2 = 5.78 / (t * t),
                             g3 = 60 / (t * t * t);
            const double h1 = e1 + g1, h2 = e2 + 2 * e1 * g1 + g2;
            const double h3 = e3 + 3 * e2 * g1 + 3 * e1 * g2 + g3;
            const double jerk = acceleration / .1;
            const double p1 = w + 2 * baseShare * speed / span;
            const double p2 = w * w + 2 * baseShare * acceleration / span;
            const double p3 = w * w * w + 2 * baseShare * jerk / span;
            constexpr double share = 1 - baseShare;
            cachedWidth = width;
            cachedAmplitude = std::min(
                {span * .01, nominalSteps, share * speed / (p1 + 2 * h1),
                 share * acceleration / (p2 + 2 * p1 * h1 + 2 * h2),
                 share * jerk / (p3 + 3 * p2 * h1 + 3 * p1 * h2 + 2 * h3)});
            return cachedAmplitude;
        }
        // Recreated with the budget whenever governing limits change.
        mutable double cachedWidth = -1, cachedAmplitude = 0;
    };

    struct VibrationClock {
        StrengthRamp gain;
        double carrier = 0;
        void advance(double dt, double requested) {
            gain.advance(dt, requested);
            carrier += dt * VibrationBudget::hz;
            carrier -= std::floor(carrier);
        }
        double offset(double y, double playbackPhase, const MarkerIndex &index,
                      const VibrationBudget &budget) const {
            const double level = gain.value();
            if (!(level > 0)) return 0;
            const auto *region = index.range(playbackPhase);
            if (!region) return 0;
            const double width = (region->end - region->start) * .001;
            const double u =
                (index.authored(playbackPhase) - region->start * .001) / width;
            const double bell = 4 * u * (1 - u);
            const double wave =
                std::sin(2 * 3.14159265358979323846 * carrier) + 1 - 2 * y;
            // y + a*(sin + 1−2y) is a convex blend of y and (1+sin)/2;
            // 0≤a≤.01 throughout. Bounds do not require clipping the position.
            return level * budget.amplitude(width) * bell * bell * bell * wave;
        }
    };

}  // namespace drawn_pattern
