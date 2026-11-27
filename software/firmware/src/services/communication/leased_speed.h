#pragma once

#include <ArduinoJson.h>
#include <cmath>
#include <cstring>

namespace motion_link {
enum class SpeedWrite { Unhandled, Invalid, Disconnected, Applied };

// Dispatch only after the shared server validates the setting.write lease.
template<class Apply>
SpeedWrite dispatchLeasedSpeed(const char *path, JsonVariantConst value,
                               uint16_t connection, Apply apply) {
    const bool selectSource = std::strcmp(path, "motion.speed") == 0;
    if (!selectSource && std::strcmp(path, "motion.speedBle") != 0) return SpeedWrite::Unhandled;
    if ((selectSource && !value.is<int>()) || !value.is<float>()) return SpeedWrite::Invalid;
    const float speed = value.as<float>();
    if (!std::isfinite(speed) || speed < 0 || speed > 100) return SpeedWrite::Invalid;
    return apply(connection, speed, selectSource) ? SpeedWrite::Applied : SpeedWrite::Disconnected;
}
}
