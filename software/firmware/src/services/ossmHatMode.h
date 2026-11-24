#pragma once

#include <cstddef>
#include <cstdio>
#include <cstring>

// Serial-only OSSM HAT setting (`hat off` / `hat auto`), stored in NVS and
// read once at boot. `off` boots as an OSSM_HAT_DISABLED image does: no HAT
// reader task, the RS-485 transmitter held off, discovery complete with
// nothing detected, and homing on the board's analog current sensor. There is
// no BLE or app setting.
namespace ossm_hat_mode {

enum class Mode : unsigned char { Auto, Off };

inline const char *name(Mode mode) { return mode == Mode::Off ? "off" : "auto"; }

// Only an exact stored "off" disables the HAT. A missing or damaged value is
// the default, so storage trouble can never leave the HAT off by accident.
inline Mode fromStored(const char *value) {
    return value != nullptr && std::strcmp(value, "off") == 0 ? Mode::Off : Mode::Auto;
}

// A complete console line that selects a mode. Any other line is not one.
inline bool parseCommand(const char *line, Mode &mode) {
    if (line == nullptr) return false;
    if (std::strcmp(line, "hat off") == 0) {
        mode = Mode::Off;
        return true;
    }
    if (std::strcmp(line, "hat auto") == 0) {
        mode = Mode::Auto;
        return true;
    }
    return false;
}

// RAD_HAT {"mode":"off","applies":"next_boot"}. When the setting could not be
// stored, mode is the unchanged stored value and "error" says why.
inline size_t formatReply(char *output, size_t capacity, Mode mode,
                          const char *error = nullptr) {
    const int size = error == nullptr
        ? std::snprintf(output, capacity,
                        "RAD_HAT {\"mode\":\"%s\",\"applies\":\"next_boot\"}\n",
                        name(mode))
        : std::snprintf(output, capacity,
                        "RAD_HAT {\"mode\":\"%s\",\"applies\":\"next_boot\","
                        "\"error\":\"%s\"}\n",
                        name(mode), error);
    return size > 0 && static_cast<size_t>(size) < capacity ? static_cast<size_t>(size) : 0;
}

}  // namespace ossm_hat_mode

// The mode this boot runs with. The first call reads NVS; later changes to the
// stored setting apply only from the next boot.
ossm_hat_mode::Mode ossmHatBootMode();
inline bool ossmHatOff() { return ossmHatBootMode() == ossm_hat_mode::Mode::Off; }
// The mode the next boot will use.
ossm_hat_mode::Mode ossmHatStoredMode();
// Stores the mode for the next boot. False when NVS could not be written.
bool storeOssmHatMode(ossm_hat_mode::Mode mode);
