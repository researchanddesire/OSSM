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

}  // namespace motor
