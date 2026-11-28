#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <new>

#include "drawn_modifiers.h"
#include "drawn_phase_rate.h"
#include "drawn_waveform.h"

namespace drawn_pattern {

    // This index is built once at Play. Frame trials and STEP edge searches
    // copy only the clock/cursor, never the authored marker list.
    class MarkerIndex {
      public:
        // A full pause grid has a point at every authored tick. Its tick is its
        // array index, leaving only the 16-bit duration to own during playback.
        bool tryCompactFullPauseGrid(const Modifiers &items, double firstMinimum) {
            if (items.count != Modifiers::capacity) return false;
            for (unsigned i = 0; i < items.count; ++i) {
                const auto &m = items.entries[i];
                if (!m.isPause() || m.start != i) return false;
            }
            std::unique_ptr<uint16_t[]> durations(new (std::nothrow) uint16_t[Modifiers::capacity]);
            if (!durations) return false;
            for (unsigned i = 0; i < items.count; ++i) durations[i] = items.entries[i].holdMs;
            pauseData = std::move(durations);
            holds = items.count;
            offset = firstMinimum;
            hasVibration = false;
            markers = nullptr;
            return true;
        }

        bool reset(const Modifiers &items, double firstMinimum, const Waveform *curve = nullptr) {
            markers = &items;
            offset = firstMinimum;
            holds = 0;
            hasVibration = false;
            std::fill_n(use, roomBins, Waveform::DerivativeUse{});
            for (unsigned i = 0; i < items.count; ++i) {
                const auto &item = items.entries[i];
                if (item.isPause()) ++holds;
                if (item.isVibration()) {
                    hasVibration = true;
                    auto &bin = use[item.start * roomBins / 1000];
                    const auto bounds = curve ? curve->vibrationDerivatives(item.start, item.end)
                                              : Waveform::DerivativeUse{255, 255, 255};
                    bin.slope = std::max(bin.slope, bounds.slope);
                    bin.curvature = std::max(bin.curvature, bounds.curvature);
                    bin.third = std::max(bin.third, bounds.third);
                }
            }
            if (hasVibration) {
                for (unsigned i = 0; i < subdivisions; ++i)
                    spatialUse[i] = curve ? curve->vibrationDerivatives(i * bandWidth, (i + 1) * bandWidth)
                                          : Waveform::DerivativeUse{255, 255, 255};
            }
            // If every marker is a pause its lookup index is its own ordinal.
            // A fully occupied pause grid needs no second 2,000-byte allocation.
            if (holds == items.count) {
                pauseData.reset();
                return true;
            }
            pauseData.reset(holds > inlineCapacity ? new (std::nothrow)
                                                           uint16_t[holds]
                                                     : nullptr);
            if (holds > inlineCapacity && !pauseData) return false;
            uint16_t *out = pauseData ? pauseData.get() : inlinePauses;
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
            const double t = threshold + offset;
            double cycle = std::floor(t);
            const double at = (t - cycle) * 1000;
            if (!markers) {
                unsigned tick = static_cast<unsigned>(std::ceil(at - 1e-10));
                if (tick == Modifiers::capacity) {
                    tick = 0;
                    ++cycle;
                }
                duration = pauseData[tick];
                return cycle + tick / 1000.0 - offset;
            }
            const uint16_t *pauses = nullptr;
            if (holds != markers->count) pauses = pauseData ? pauseData.get() : inlinePauses;
            unsigned lo = 0, hi = holds;
            while (lo < hi) {
                const unsigned k = (lo + hi) / 2;
                if (markers->entries[pauses ? pauses[k] : k].start < at - 1e-10)
                    lo = k + 1;
                else
                    hi = k;
            }
            if (lo == holds) {
                lo = 0;
                ++cycle;
            }
            const auto &m = markers->entries[pauses ? pauses[lo] : lo];
            duration = m.holdMs;
            return cycle + m.start / 1000.0 - offset;
        }

        const Waveform::DerivativeUse &vibrationUse(unsigned start) const { return use[start * roomBins / 1000]; }

        static constexpr unsigned subdivisions = 8;
        static constexpr unsigned bandWidth = 1000 / subdivisions;
        const Waveform::DerivativeUse &bandUse(unsigned band) const { return spatialUse[band]; }
        bool hasVibration = false;
        unsigned holds = 0;

      private:
        // Nearby ranges share the most conservative allowance. This keeps
        // storage bounded even for a thousand markers.
        static constexpr unsigned roomBins = 32;
        Waveform::DerivativeUse use[roomBins]{};
        Waveform::DerivativeUse spatialUse[subdivisions]{};
        const Modifiers *markers = nullptr;
        double offset = 0;
        static constexpr unsigned inlineCapacity = 4;
        uint16_t inlinePauses[inlineCapacity]{};
        std::unique_ptr<uint16_t[]> pauseData;

    };

    struct PhaseLimits {
        double rate, acceleration, jerk;
    };

    // Reused only while the clock limits remain unchanged. At cruise the
    // exact stopping distance is identical on every frame. During a forward
    // ramp the upper bound rules out distant pauses before solving the exact
    // brake. The slow path remains authoritative near every marker.
    struct PauseBrakeBounds {
        double cruise = 0, far = 0;
        static constexpr double horizon = .020;  // longest serial or STEP frame

        void reset(const PhaseLimits &limits) {
            PhaseClock atCruise;
            atCruise.setGoal(limits.rate, limits.acceleration, limits.jerk);
            atCruise.advance(atCruise.secondsToGoal() + 1e-12);
            atCruise.setGoal(0, limits.acceleration, limits.jerk);
            cruise = atCruise.advance(atCruise.secondsToGoal() + 1e-12);

            // The two jerk ramps of a stop from |a| <= A take <= 3A/J.
            // A constant-acceleration segment exists only at peak A and
            // lasts <= R/A + A/(2J). Account for motion during this frame.
            const double a = limits.acceleration;
            const double r = limits.rate + a * horizon;
            const double duration = r / a + 4 * a / limits.jerk;
            far = r * horizon + (r + a * duration) * duration;
        }
    };

    // A peripheral frame samples eleven possible edge times. Solve the
    // braking decision once on that frame, then reuse it for those previews.
    struct FrameBrake {
        bool known = false;
        double when = HUGE_VAL, target = 0;
        uint16_t holdMs = 0;
        const PauseBrakeBounds *bounds = nullptr;
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
            wrap();
        }

        // A user pause freezes authored dwell time. An already committed
        // approach to an authored pause still lands exactly on its point.
        void brakeForUser(double dt, const PhaseLimits &limits) {
            if (state == Holding) return;
            if (state == Braking) {
                const double remaining = clock.secondsToGoal();
                if (remaining <= dt) {
                    clock.advance(remaining + 1e-12);
                    phase = target;
                    state = Holding;
                    wrap();
                    return;
                }
            } else {
                clock.setGoal(0, limits.acceleration, limits.jerk);
            }
            phase += clock.advance(dt);
            wrap();
        }
      private:
        enum State { Free, Braking, Holding } state = Free;
        double target = 0, skipThrough = -2, holdElapsed = 0;
        uint16_t maximumHoldMs = 0;

        void wrap() {
            if (phase >= 1 || phase < 0) {
                const double turns = std::floor(phase);
                phase -= turns;
                target -= turns;
                skipThrough = std::max(-2., skipThrough - turns);
            }
        }

        static double stopDistance(PhaseClock clock,
                                   const PhaseLimits &limits) {
            clock.setGoal(0, limits.acceleration, limits.jerk);
            return clock.advance(clock.secondsToGoal() + 1e-12);
        }

        FrameBrake planBrake(double dt, const PhaseLimits &limits,
                             const MarkerIndex &index, const PauseBrakeBounds *bounds) const {
            FrameBrake result;
            result.known = true;
            const bool cruising = bounds && clock.secondsToGoal() == 0;
            if (bounds && !cruising && dt <= PauseBrakeBounds::horizon &&
                clock.rate() >= 0 && clock.rate() <= limits.rate &&
                clock.acceleration() >= 0 && clock.acceleration() <= limits.acceleration) {
                // With nonnegative initial rate/acceleration the earliest
                // prospective stop starts at or forward of the current phase.
                const double next = index.upcoming(std::max(skipThrough + 1e-8, phase - 1e-10), result.holdMs);
                if (next > phase + bounds->far + 1e-10) return result;
            }
            const double front = phase + (cruising ? bounds->cruise : stopDistance(clock, limits));
            result.target = index.upcoming(
                std::max(skipThrough + 1e-8, front - 1e-10), result.holdMs);
            if (cruising) {
                if (phase + limits.rate * dt + bounds->cruise < result.target - 1e-12) return result;
            } else {
                PhaseClock end = clock;
                const double distance = end.advance(dt);
                if (phase + distance + stopDistance(end, limits) < result.target - 1e-12) return result;
            }
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
                                                : planBrake(dt, limits, index, frame ? frame->bounds : nullptr);
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
        static constexpr double seconds = .25;
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

    // Limit only the added carrier, including its spatial and gain windows.
    struct VibrationBudget {
        static constexpr double hz = 6;
        double span = 0, speed = 0, acceleration = 0, nominalSteps = 0;
        PhaseLimits phase{};
        double basePathJerk = 0, maximumSpeed = 0, cruiseRate = 0;
        double slope = 0, curvature = 0, third = 0;
        bool matches(double v, double a) const {
            return v == speed && a == acceleration;
        }

        struct AmplitudeCache {
            double width = -1, amplitude = 0;
            Waveform::DerivativeUse use{};
        };
        double amplitude(double width, Waveform::DerivativeUse use, bool spatial = false) const {
            auto &cache = spatial ? bandCache : rangeCache;
            if (width == cache.width && use.slope == cache.use.slope &&
                use.curvature == cache.use.curvature && use.third == cache.use.third) return cache.amplitude;
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
            const double d1 = slope * (use.slope / 255.);
            const double d2 = curvature * (use.curvature / 255.);
            const double d3 = third * (use.third / 255.);
            const double rate = phase.rate, rate2 = rate * rate;
            const double pathSpeed = d1 * rate;
            const double pathAcceleration = d2 * rate2 + d1 * phase.acceleration;
            const double pathJerk = d3 * rate2 * rate +
                3 * d2 * rate * phase.acceleration + d1 * phase.jerk;
            // The endpoint bias moves with the drawing. Charge its actual
            // derivative bounds and reserve the remaining physical capacity.
            const double p1 = w + 2 * pathSpeed / span;
            const double p2 = w * w + 2 * pathAcceleration / span;
            const double p3 = w * w * w + 2 * pathJerk / span;
            const double v = std::max(0., std::min({.25 * speed, maximumSpeed - pathSpeed,
                                                   speed - d1 * cruiseRate}));
            const double aRoom = std::min(.25 * acceleration, std::max(0., acceleration - pathAcceleration));
            const double jRoom = std::min(.5 * jerk, std::max(0., basePathJerk - pathJerk));
            cache.use = use;
            cache.width = width;
            cache.amplitude = std::min(
                {span * .01, nominalSteps, v / (p1 + 2 * h1),
                 aRoom / (p2 + 2 * p1 * h1 + 2 * h2),
                 jRoom / (p3 + 3 * p2 * h1 + 3 * p1 * h2 + 2 * h3)});
            return cache.amplitude;
        }
        struct Window { double start, width, height; };
        Window window(const Modifier &region, double authored, const MarkerIndex &index) const {
            double start = region.start * .001;
            double width = (region.end - region.start) * .001;
            double height = amplitude(width, index.vibrationUse(region.start));
            if (height < 1.e-8) {
                // A fast passage in one cell must not mute every slower
                // passage in a wide marking. Each disjoint sub-window joins
                // at zero displacement, velocity and acceleration.
                const unsigned band = static_cast<unsigned>(authored * MarkerIndex::subdivisions);
                const unsigned left = std::max<unsigned>(region.start, band * MarkerIndex::bandWidth);
                const unsigned right = std::min<unsigned>(region.end, (band + 1) * MarkerIndex::bandWidth);
                start = left * .001;
                width = (right - left) * .001;
                height = amplitude(width, index.bandUse(band), true);
            }
            return {start, width, height};
        }
        // Recreated with the budget whenever governing limits change.
        mutable AmplitudeCache rangeCache{}, bandCache{};
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
            const double authored = index.authored(playbackPhase);
            const auto window = budget.window(*region, authored, index);
            if (window.height == 0) return 0;
            const double u = (authored - window.start) / window.width;
            const double bell = 4 * u * (1 - u);
            const double wave =
                std::sin(2 * 3.14159265358979323846 * carrier) + 1 - 2 * y;
            // y + a*(sin + 1−2y) is a convex blend of y and (1+sin)/2;
            // 0≤a≤.01 throughout. Bounds do not require clipping the position.
            return level * window.height * bell * bell * bell * wave;
        }
    };

}  // namespace drawn_pattern
