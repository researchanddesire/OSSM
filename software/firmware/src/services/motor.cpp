#include "motor.h"
#include "services/ossmHat.h"

#if OSSM_HAT_SERIAL_BACKEND
#include <Arduino.h>
#include <cmath>
#include "constants/Config.h"
#include "constants/Pins.h"
#include "ossm/state/calibration.h"
#include "services/motorFeedback.h"
#include "services/stepper.h"
#include "utils/analog.h"

namespace motor {
namespace {
bool readFreshHat(OssmHatTelemetry &sample) {
    return readOssmHatTelemetry(sample) &&
           static_cast<uint32_t>(millis() - sample.updatedAtMs) <= 100;
}
}

void begin() { initOssmHat(); }
bool hasHat() { return ossmHatDetected(); }
bool reservesPin(int pin) { return ossmHatOwnsPin(pin); }
float getPositionMm() { return getMotorPositionMm(); }

float getCurrent() {
    if (hasHat()) {
        OssmHatTelemetry sample;
        return readFreshHat(sample) ? sample.currentAmps : NAN;
    }
    return getAnalogAveragePercent(SampleOnPin{Pins::Driver::currentSensorPin, 50}) -
           calibration.currentSensorOffset;
}
float getHomingCurrentLimit() {
    return hasHat() ? Config::Hat::sensorlessCurrentLimitAmps
                    : Config::Driver::sensorlessCurrentLimit;
}
void zeroCurrent() {
    if (!hasHat()) calibration.currentSensorOffset = getAnalogAveragePercent(
        SampleOnPin{Pins::Driver::currentSensorPin, 1000});
}
float getVoltage() {
    OssmHatTelemetry sample;
    return hasHat() && readFreshHat(sample) ? sample.voltageVolts : NAN;
}
float getSpeedMmPerSec() {
    OssmHatTelemetry sample;
    if (stepper == nullptr || !hasHat() || !readFreshHat(sample)) return NAN;
    constexpr float travel = Config::Driver::pulleyToothCount * Config::Driver::beltPitchMm;
    const float native = sample.speedRpm * travel / 60.0f;
    return stepper->directionPinHighCountsUp() ? -native : native;
}
float getTemperatureC() {
    OssmHatTelemetry sample;
    return hasHat() && readFreshHat(sample) ? sample.temperatureC : NAN;
}
float getOutputPercent() {
    OssmHatTelemetry sample;
    return hasHat() && readFreshHat(sample) ? sample.outputPercent : NAN;
}
int getAlarm() {
    OssmHatTelemetry sample;
    return hasHat() && readFreshHat(sample) ? sample.alarm : -1;
}
}  // namespace motor

#else
#include "motor.h"

#include <cmath>

#include "constants/Config.h"
#include "constants/Pins.h"
#include "ossm/state/calibration.h"
#include "services/ossmHat.h"
#include "services/stepper.h"
#include "utils/analog.h"

namespace motor {

namespace {

// HAT readings older than this are not trusted.
constexpr uint32_t freshHatMs = 100;

bool readFreshHat(ossm_hat::Telemetry &out) {
    out = ossm_hat::telemetry();
    return millis() - out.sampledMs <= freshHatMs;
}

}  // namespace

void begin() { ossm_hat::begin(); }

bool hasHat() { return ossm_hat::isAnswering(); }

bool reservesPin(int pin) {
    return ossm_hat::isPresent() &&
           (pin == Pins::Hat::de || pin == Pins::Hat::tx || pin == Pins::Hat::rx);
}

float getPositionMm() {
    if (stepper == nullptr) return 0.0f;
    int32_t steps = stepper->getCurrentPosition();
    if (hasHat()) {
        // The servo's lag is counted in the native frame; StrokeEngine's
        // frame counts the other way (services/stepper.h).
        const int32_t lag = ossm_hat::followingErrorPulses();
        steps += stepper->directionPinHighCountsUp() ? -lag : lag;
    }
    return static_cast<float>(steps) / static_cast<float>(1_mm);
}

float getCurrent() {
    if (hasHat()) {
        ossm_hat::Telemetry hat;
        return readFreshHat(hat) ? hat.currentAmps : NAN;
    }
    return getAnalogAveragePercent(
               SampleOnPin{Pins::Driver::currentSensorPin, 50}) -
           calibration.currentSensorOffset;
}

float getHomingCurrentLimit() {
    return hasHat() ? Config::Hat::homingCurrentAmps
                    : Config::Driver::sensorlessCurrentLimit;
}

// Also with a HAT: the analog sensor takes over if the HAT stops answering.
void zeroCurrent() {
    calibration.currentSensorOffset = getAnalogAveragePercent(
        SampleOnPin{Pins::Driver::currentSensorPin, 1000});
}

float getVoltage() {
    ossm_hat::Telemetry hat;
    return hasHat() && readFreshHat(hat) ? hat.voltage : NAN;
}

float getSpeedMmPerSec() {
    ossm_hat::Telemetry hat;
    if (stepper == nullptr || !hasHat() || !readFreshHat(hat)) return NAN;
    constexpr float mmPerRevolution =
        Config::Driver::pulleyToothCount * Config::Driver::beltPitchMm;
    // The servo counts the native frame's way; StrokeEngine's counts the
    // other way (services/stepper.h).
    const float native = hat.speedRpm * mmPerRevolution / 60.0f;
    return stepper->directionPinHighCountsUp() ? -native : native;
}

float getTemperatureC() {
    ossm_hat::Telemetry hat;
    return hasHat() && readFreshHat(hat) ? hat.temperatureC : NAN;
}

float getOutputPercent() {
    ossm_hat::Telemetry hat;
    return hasHat() && readFreshHat(hat) ? hat.outputPercent : NAN;
}

int getAlarm() {
    ossm_hat::Telemetry hat;
    return hasHat() && readFreshHat(hat) ? hat.alarm : -1;
}

}  // namespace motor

#endif  // OSSM_HAT_SERIAL_BACKEND
