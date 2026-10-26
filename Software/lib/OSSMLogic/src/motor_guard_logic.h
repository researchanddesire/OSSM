#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "motor_feedback_logic.h"

// Pure safety decisions. The caller owns the independent task, UART snapshot,
// verified hardware stop, persistent fault latch and all output-enable gates.
// Every position used here comes from the physical encoder, never STEP/DIR's
// commanded position counter.
namespace motor_guard {

enum class Fault : uint8_t {
    None,
    InvalidTelemetry,
    TelemetryStale,
    TelemetryRate,
    ObserverDelayed,
    ContinuityChanged,
    Alarm,
    SupplyVoltage,
    OverCurrent,
    UnexpectedLoad,
    HomingSpeed,
    ContactTooLong,
    RepeatedContact,
    HomingSequence,
    UncalibratedMotion,
    TravelBounds,
    StopPathUnverified,
};

inline const char *reason(Fault fault) {
    switch (fault) {
        case Fault::None: return "No motor safety fault";
        case Fault::InvalidTelemetry: return "Invalid physical motor telemetry";
        case Fault::TelemetryStale: return "RS485 motor telemetry is stale";
        case Fault::TelemetryRate: return "RS485 motor telemetry is below 30 Hz";
        case Fault::ObserverDelayed: return "Independent motor safety task missed its deadline";
        case Fault::ContinuityChanged: return "Motor encoder continuity changed; re-home after review";
        case Fault::Alarm: return "Motor reported a fatal alarm";
        case Fault::SupplyVoltage: return "Motor supply is outside 20..36 V";
        case Fault::OverCurrent: return "Motor current exceeds the configured safety limit";
        case Fault::UnexpectedLoad: return "Unexpected loaded stationary motor";
        case Fault::HomingSpeed: return "Measured homing speed exceeds 30 mm/s";
        case Fault::ContactTooLong: return "Continuous homing load exceeds 200 ms";
        case Fault::RepeatedContact: return "Unexpected repeated hard-stop contact";
        case Fault::HomingSequence: return "Invalid deliberate homing contact sequence";
        case Fault::UncalibratedMotion: return "Motion requested without measured physical bounds";
        case Fault::TravelBounds: return "Physical encoder approaches a measured travel boundary";
        case Fault::StopPathUnverified: return "Motor output-disable stop path is not verified";
    }
    return "Unknown motor safety fault";
}

struct Config {
    double countsPerMm = 32768.0 / 40.0;
    float minimumVoltage = 20.0f;
    float maximumVoltage = 36.0f;
    // Conservative initial bench policy, not a statement of motor rated current.
    float maximumCurrent = 2.2f;
    float loadedCurrent = 1.5f;
    float minimumRateHz = 30.0f;
    uint32_t freshnessMs = 100;
    uint32_t rateWindowMs = 1000;
    uint32_t rateWarmupMs = 500;
    uint32_t maximumTickGapMs = 34;
    float maximumHomingSpeedMmS = 30.0f;
    float stationarySpeedMmS = 2.0f;
    uint32_t unexpectedLoadMs = 50;
    uint32_t maximumContactMs = 200;
    float safeMarginMm = 6.0f;
    float minimumTravelMm = 50.0f;
    // Additional allowance beyond the maximum telemetry age, before output
    // disable arrests travel. This is conservative policy, not a measured
    // motor stopping time; validate hardware dynamics before increasing speed.
    uint32_t stopAllowanceMs = 25;
};

struct Sample {
    uint32_t sequence = 0;
    uint32_t atMs = 0;
    uint32_t continuity = 0;
    int32_t positionCounts = 0;
    float currentAmps = 0;
    float voltageVolts = 0;
    uint16_t alarm = 0;
};

class Guard {
  public:
    explicit Guard(const Config &config = Config{}) : config_(config) {}

    void setStopPathVerified(bool verified) {
        stopPathVerified_ = verified;
        if (!verified && armed_) trip(Fault::StopPathUnverified);
    }

    // Call for each snapshot; duplicate sequence values cannot inflate rate
    // or advance contact evidence. The service must call tick at least 30 Hz.
    void observe(const Sample &sample, uint32_t nowMs) {
        if (fault_ != Fault::None) return;
        if (!valid(sample)) {
            if (armed_) trip(Fault::InvalidTelemetry);
            else clearObservation();
            return;
        }
        if (static_cast<uint32_t>(nowMs - sample.atMs) > config_.freshnessMs) {
            if (armed_) trip(Fault::TelemetryStale);
            return;
        }
        if (haveSample_ && sample.sequence == latest_.sequence) return;

        const bool first = !haveSample_;
        const bool changed = !first && sample.continuity != latest_.continuity;
        const uint32_t elapsed = first ? 0 : sample.atMs - latest_.atMs;
        if (armed_ && changed) {
            trip(Fault::ContinuityChanged);
            return;
        }
        if (armed_ && (elapsed == 0 || elapsed > config_.freshnessMs)) {
            trip(Fault::TelemetryStale);
            return;
        }
        if (first || changed || elapsed == 0 || elapsed > config_.freshnessMs) {
            clearObservation();
            if (changed) clearBounds();
            unwrappedCounts_ = 0;
            speedMmS_ = 0;
        } else {
            const int64_t delta = motor_feedback_logic::encoderDelta(
                sample.positionCounts, latest_.positionCounts);
            unwrappedCounts_ += delta;
            speedMmS_ = static_cast<double>(delta) * 1000.0 /
                        config_.countsPerMm / elapsed;
        }

        latest_ = sample;
        haveSample_ = true;
        appendTime(sample.atMs);
        if (!armed_) return;
        if (!checkElectrical()) return;

        if (mode_ == Mode::AwaitingHoming &&
            std::abs(speedMmS_) > config_.stationarySpeedMmS) {
            trip(Fault::UncalibratedMotion);
            return;
        }
        if ((mode_ == Mode::Homing || mode_ == Mode::HomingReposition) &&
            std::abs(speedMmS_) > config_.maximumHomingSpeedMmS) {
            trip(Fault::HomingSpeed);
            return;
        }

        const bool loaded = latest_.currentAmps > config_.loadedCurrent;
        if (!loaded) {
            loadStarted_ = false;
            contactActive_ = false;
        } else {
            if (!loadStarted_) {
                loadStarted_ = true;
                loadStartedAt_ = sample.atMs;
                stationaryWindowAt_ = sample.atMs;
                stationaryMinimum_ = unwrappedCounts_;
                stationaryMaximum_ = unwrappedCounts_;
            } else {
                if (unwrappedCounts_ < stationaryMinimum_)
                    stationaryMinimum_ = unwrappedCounts_;
                if (unwrappedCounts_ > stationaryMaximum_)
                    stationaryMaximum_ = unwrappedCounts_;
                if (static_cast<double>(stationaryMaximum_ - stationaryMinimum_) >=
                    0.25 * config_.countsPerMm) {
                    stationaryWindowAt_ = sample.atMs;
                    stationaryMinimum_ = unwrappedCounts_;
                    stationaryMaximum_ = unwrappedCounts_;
                }
            }
            const bool stationary =
                std::abs(speedMmS_) <= config_.stationarySpeedMmS ||
                static_cast<uint32_t>(sample.atMs - stationaryWindowAt_) >= 30;
            if (mode_ == Mode::Homing && stationary && !contactActive_) {
                if (passContactSeen_ || contacts_ >= 2) {
                    trip(Fault::RepeatedContact);
                    return;
                }
                contactActive_ = true;
                passContactSeen_ = true;
                contactStartedAt_ = loadStartedAt_;
                stopUnwrapped_[contacts_] = unwrappedCounts_;
                stopRaw_[contacts_] = sample.positionCounts;
                ++contacts_;
            }
        }
        checkLoad(nowMs);
        if (fault_ == Fault::None && mode_ == Mode::Normal) checkBounds();
    }

    Fault tick(uint32_t nowMs) {
        if (fault_ != Fault::None) return fault_;
        if (!armed_) {
            lastTickAt_ = nowMs;
            return fault_;
        }
        if (static_cast<uint32_t>(nowMs - lastTickAt_) >
            config_.maximumTickGapMs) {
            trip(Fault::ObserverDelayed);
            return fault_;
        }
        lastTickAt_ = nowMs;
        if (!haveSample_ ||
            static_cast<uint32_t>(nowMs - latest_.atMs) > config_.freshnessMs) {
            trip(Fault::TelemetryStale);
            return fault_;
        }
        const Rate rate = rateAt(nowMs);
        if (rate.span < config_.rateWarmupMs ||
            rate.hz < config_.minimumRateHz) {
            trip(Fault::TelemetryRate);
            return fault_;
        }
        if (!checkElectrical()) return fault_;
        checkLoad(nowMs);
        if (fault_ == Fault::None && mode_ == Mode::Normal) checkBounds();
        return fault_;
    }

    // Discovery/low supply/slow telemetry block arming without latching a
    // fault while outputs remain disabled. Once armed, they latch immediately.
    bool ready(uint32_t nowMs) const {
        const Rate rate = rateAt(nowMs);
        return fault_ == Fault::None && stopPathVerified_ && haveSample_ &&
               valid(latest_) && electricalValid() &&
               static_cast<uint32_t>(nowMs - latest_.atMs) <= config_.freshnessMs &&
               rate.span >= config_.rateWarmupMs &&
               rate.hz >= config_.minimumRateHz &&
               latest_.currentAmps <= config_.loadedCurrent &&
               std::abs(speedMmS_) <= config_.stationarySpeedMmS;
    }

    bool arm(uint32_t nowMs) {
        if (armed_) return tick(nowMs) == Fault::None;
        if (!ready(nowMs)) return false;
        armed_ = true;
        lastTickAt_ = nowMs;
        mode_ = boundsValid_ ? Mode::Normal : Mode::AwaitingHoming;
        loadStarted_ = false;
        if (mode_ == Mode::Normal) checkBounds();
        return fault_ == Fault::None;
    }

    void disarm() { armed_ = false; }

    // Hardware watchdogs and the UART ownership/control-register preflight
    // can report faults observed outside this pure telemetry core. The owning
    // service must serialize this with observe/tick and retain the first fault.
    void tripFault(Fault fault) {
        if (fault != Fault::None) trip(fault);
    }

    // This must be called only by an explicit reviewed reset action. It never
    // arms the motor; a new stop verification, rate warmup and home are needed.
    void resetAfterReview() {
        armed_ = false;
        fault_ = Fault::None;
        stopPathVerified_ = false;
        clearObservation();
        clearBounds();
        mode_ = Mode::AwaitingHoming;
    }

    bool beginHomingPass(uint8_t pass, float declaredSpeedMmS,
                         uint32_t nowMs) {
        if (!armed_ || fault_ != Fault::None) return false;
        if (tick(nowMs) != Fault::None) return false;
        if (!std::isfinite(declaredSpeedMmS) || declaredSpeedMmS <= 0 ||
            declaredSpeedMmS > config_.maximumHomingSpeedMmS ||
            std::abs(speedMmS_) > config_.stationarySpeedMmS ||
            latest_.currentAmps > config_.loadedCurrent) {
            trip(Fault::HomingSequence);
            return false;
        }
        if (pass == 1 && completedPasses_ == 2 && mode_ == Mode::Normal)
            clearBounds();  // explicit new physical homing sequence
        if (mode_ == Mode::Homing || pass < 1 || pass > 2 ||
            pass != completedPasses_ + 1 || contacts_ != completedPasses_) {
            trip(Fault::HomingSequence);
            return false;
        }
        activePass_ = pass;
        passContactSeen_ = false;
        contactActive_ = false;
        loadStarted_ = false;
        mode_ = Mode::Homing;
        return true;
    }

    // End after contact has released and the low-speed retreat has completed.
    // Between passes only low-speed homing reposition is permitted. The second
    // end establishes physical bounds from the two recorded encoder endpoints.
    bool endHomingPass(uint32_t nowMs) {
        if (!armed_ || fault_ != Fault::None) return false;
        if (tick(nowMs) != Fault::None) return false;
        if (mode_ != Mode::Homing || !passContactSeen_ || contactActive_ ||
            contacts_ != activePass_ ||
            latest_.currentAmps > config_.loadedCurrent ||
            std::abs(speedMmS_) > config_.stationarySpeedMmS) {
            trip(Fault::HomingSequence);
            return false;
        }
        completedPasses_ = activePass_;
        activePass_ = 0;
        mode_ = Mode::HomingReposition;
        if (completedPasses_ == 2) {
            lowerStop_ = stopUnwrapped_[0] < stopUnwrapped_[1]
                             ? stopUnwrapped_[0] : stopUnwrapped_[1];
            upperStop_ = stopUnwrapped_[0] > stopUnwrapped_[1]
                             ? stopUnwrapped_[0] : stopUnwrapped_[1];
            const double travelMm =
                static_cast<double>(upperStop_ - lowerStop_) / config_.countsPerMm;
            if (travelMm < config_.minimumTravelMm ||
                travelMm <= 2.0 * config_.safeMarginMm) {
                trip(Fault::HomingSequence);
                return false;
            }
            boundsValid_ = true;
        }
        return true;
    }

    bool beginNormalOperation(uint32_t nowMs) {
        if (!armed_ || fault_ != Fault::None) return false;
        if (tick(nowMs) != Fault::None) return false;
        if (!boundsValid_ || completedPasses_ != 2 || activePass_ != 0) {
            trip(Fault::UncalibratedMotion);
            return false;
        }
        mode_ = Mode::Normal;
        checkBounds();
        return fault_ == Fault::None;
    }

    Fault fault() const { return fault_; }
    bool armed() const { return armed_; }
    bool motionAllowed() const { return armed_ && fault_ == Fault::None; }
    float observedHz(uint32_t nowMs) const { return rateAt(nowMs).hz; }
    uint8_t contactCount() const { return contacts_; }
    bool boundsValid() const { return boundsValid_; }
    bool contactActive() const { return contactActive_; }
    double speedMmS() const { return speedMmS_; }
    int32_t lowerStopCounts() const {
        return stopUnwrapped_[0] < stopUnwrapped_[1] ? stopRaw_[0] : stopRaw_[1];
    }
    int32_t upperStopCounts() const {
        return stopUnwrapped_[0] > stopUnwrapped_[1] ? stopRaw_[0] : stopRaw_[1];
    }

  private:
    enum class Mode : uint8_t { AwaitingHoming, Homing, HomingReposition, Normal };
    struct Rate { float hz = 0; uint32_t span = 0; };
    static constexpr size_t rateCapacity = 128;

    bool valid(const Sample &sample) const {
        return sample.sequence != 0 && sample.continuity != 0 &&
               std::isfinite(config_.countsPerMm) && config_.countsPerMm > 0 &&
               std::isfinite(sample.currentAmps) && sample.currentAmps >= 0 &&
               std::isfinite(sample.voltageVolts) && sample.voltageVolts >= 0;
    }

    bool electricalValid() const {
        return (latest_.alarm == 0 || latest_.alarm == 0x10) &&
               latest_.voltageVolts >= config_.minimumVoltage &&
               latest_.voltageVolts <= config_.maximumVoltage &&
               latest_.currentAmps <= config_.maximumCurrent;
    }

    bool checkElectrical() {
        if (latest_.alarm != 0 && latest_.alarm != 0x10) trip(Fault::Alarm);
        else if (latest_.voltageVolts < config_.minimumVoltage ||
                 latest_.voltageVolts > config_.maximumVoltage)
            trip(Fault::SupplyVoltage);
        else if (latest_.currentAmps > config_.maximumCurrent)
            trip(Fault::OverCurrent);
        return fault_ == Fault::None;
    }

    void checkLoad(uint32_t nowMs) {
        if (!loadStarted_ || latest_.currentAmps <= config_.loadedCurrent)
            return;
        if (mode_ == Mode::Homing &&
            static_cast<uint32_t>(nowMs - loadStartedAt_) >=
                config_.maximumContactMs) {
            // Bound continuous homing load even if encoder oscillation at a
            // physical obstruction resembles motion in a single derivative.
            trip(Fault::ContactTooLong);
        } else if (contactActive_) {
            if (static_cast<uint32_t>(nowMs - contactStartedAt_) >=
                config_.maximumContactMs)
                trip(Fault::ContactTooLong);
        } else if (mode_ != Mode::Homing &&
                   ((std::abs(speedMmS_) <= config_.stationarySpeedMmS &&
                     static_cast<uint32_t>(nowMs - loadStartedAt_) >=
                         config_.unexpectedLoadMs) ||
                    static_cast<uint32_t>(nowMs - stationaryWindowAt_) >=
                        config_.unexpectedLoadMs)) {
            trip(Fault::UnexpectedLoad);
        }
    }

    void checkBounds() {
        if (!boundsValid_) {
            trip(Fault::UncalibratedMotion);
            return;
        }
        const double low = lowerStop_ + config_.safeMarginMm * config_.countsPerMm;
        const double high = upperStop_ - config_.safeMarginMm * config_.countsPerMm;
        const double projected = unwrappedCounts_ +
            speedMmS_ * config_.countsPerMm *
                (config_.freshnessMs + config_.stopAllowanceMs) / 1000.0;
        if (unwrappedCounts_ < low || unwrappedCounts_ > high ||
            projected < low || projected > high)
            trip(Fault::TravelBounds);
    }

    void appendTime(uint32_t atMs) {
        if (timeCount_ == rateCapacity) {
            timeStart_ = (timeStart_ + 1) % rateCapacity;
            --timeCount_;
        }
        times_[(timeStart_ + timeCount_) % rateCapacity] = atMs;
        ++timeCount_;
    }

    Rate rateAt(uint32_t nowMs) const {
        if (!haveSample_ || timeCount_ < 2 ||
            static_cast<uint32_t>(nowMs - latest_.atMs) > config_.freshnessMs)
            return {};
        size_t first = 0;
        while (first < timeCount_ &&
               static_cast<uint32_t>(nowMs - times_[(timeStart_ + first) % rateCapacity]) >
                   config_.rateWindowMs)
            ++first;
        const size_t count = timeCount_ - first;
        if (count < 2) return {};
        const uint32_t span = latest_.atMs - times_[(timeStart_ + first) % rateCapacity];
        return span ? Rate{static_cast<float>((count - 1) * 1000.0 / span), span}
                    : Rate{};
    }

    void clearObservation() {
        haveSample_ = false;
        timeStart_ = 0;
        timeCount_ = 0;
        loadStarted_ = false;
        contactActive_ = false;
        // Resetting the unwrapped observation origin invalidates every bound
        // measured in that origin, even if a short outage did not advance the
        // UART service's (longer) continuity epoch.
        clearBounds();
    }

    void clearBounds() {
        boundsValid_ = false;
        contacts_ = 0;
        completedPasses_ = 0;
        activePass_ = 0;
        passContactSeen_ = false;
        contactActive_ = false;
        loadStarted_ = false;
    }

    void trip(Fault fault) {
        if (fault_ == Fault::None) fault_ = fault;
        armed_ = false;
        stopPathVerified_ = false;
    }

    Config config_;
    Fault fault_ = Fault::None;
    Mode mode_ = Mode::AwaitingHoming;
    bool armed_ = false;
    bool stopPathVerified_ = false;
    bool haveSample_ = false;
    Sample latest_;
    uint32_t lastTickAt_ = 0;
    uint32_t times_[rateCapacity] = {};
    size_t timeStart_ = 0;
    size_t timeCount_ = 0;
    int64_t unwrappedCounts_ = 0;
    double speedMmS_ = 0;
    bool loadStarted_ = false;
    uint32_t loadStartedAt_ = 0;
    uint32_t stationaryWindowAt_ = 0;
    int64_t stationaryMinimum_ = 0;
    int64_t stationaryMaximum_ = 0;
    bool contactActive_ = false;
    uint32_t contactStartedAt_ = 0;
    bool passContactSeen_ = false;
    uint8_t contacts_ = 0;
    uint8_t completedPasses_ = 0;
    uint8_t activePass_ = 0;
    int64_t stopUnwrapped_[2] = {};
    int32_t stopRaw_[2] = {};
    bool boundsValid_ = false;
    int64_t lowerStop_ = 0;
    int64_t upperStop_ = 0;
};

}  // namespace motor_guard
