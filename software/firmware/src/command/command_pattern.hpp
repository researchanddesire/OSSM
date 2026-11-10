#ifndef OSSM_COMMAND_PATTERN_HPP
#define OSSM_COMMAND_PATTERN_HPP

#include <string>

// Single source of truth for the BLE command grammar. The NimBLE write
// callback validates against this and the native tests exercise it, so the
// accepted set cannot drift from commandFromString(). It is checked by hand:
// std::regex allocates on every match, and a failed allocation aborts the
// firmware when internal RAM runs low.
//
//   go:(simplePenetration|strokeEngine|streaming|menu|restart|update|pairing)
//   set:(speed|stroke|depth|sensation|buffer|pattern):<digits>
//   set:wifi:<ssid without |>|<password>
//   stream:<digits>:<digits>
namespace ble_command {

inline bool startsWith(const std::string& text, const char* prefix) {
    return text.rfind(prefix, 0) == 0;
}

inline bool isDigits(const std::string& text, size_t from, size_t to) {
    if (from >= to) return false;
    for (size_t i = from; i < to; i++) {
        if (text[i] < '0' || text[i] > '9') return false;
    }
    return true;
}

}  // namespace ble_command

inline bool isValidBleCommand(const std::string& command) {
    using namespace ble_command;
    static const char* const goTargets[] = {
        "simplePenetration", "strokeEngine", "streaming", "menu",
        "restart",           "update",       "pairing"};
    static const char* const settings[] = {"speed",     "stroke", "depth",
                                           "sensation", "buffer", "pattern"};

    if (startsWith(command, "go:")) {
        for (const char* target : goTargets) {
            if (command.compare(3, std::string::npos, target) == 0) return true;
        }
        return false;
    }
    if (startsWith(command, "stream:")) {
        const size_t colon = command.find(':', 7);
        return colon != std::string::npos && isDigits(command, 7, colon) &&
               isDigits(command, colon + 1, command.size());
    }
    if (startsWith(command, "set:wifi:")) {
        const size_t bar = command.find('|', 9);
        return bar != std::string::npos && bar > 9 && bar + 1 < command.size() &&
               command.find_first_of("\r\n", bar + 1) == std::string::npos;
    }
    if (startsWith(command, "set:")) {
        const size_t colon = command.find(':', 4);
        if (colon == std::string::npos ||
            !isDigits(command, colon + 1, command.size())) {
            return false;
        }
        for (const char* setting : settings) {
            if (command.compare(4, colon - 4, setting) == 0) return true;
        }
    }
    return false;
}

#endif  // OSSM_COMMAND_PATTERN_HPP
