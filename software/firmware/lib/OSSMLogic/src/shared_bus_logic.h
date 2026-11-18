#pragma once

#include <cstdint>

// Pure decision logic for an OSSM HAT that shares its RS-485 bus with another
// Modbus master, such as the Test Rig's independent PC guard.
// No hardware dependencies — testable on native platform.
//
// RTU has no arbitration, so the HAT stays receive-only on such a bus until
// reboot. It can then never send a serial target or a serial stop. Motion
// stays on STEP/DIR, as on a board without a HAT, and the other master is the
// independent monitor. STEP pulses are valid only while that master keeps
// reporting the drive in STEP/DIR control (register 0 == 0): a drive switched
// to serial control counts pulses without executing them and runs them when
// control is handed back.

namespace shared_bus {

struct Observation {
    bool hatDetected = false;
    // Another master was observed. Latched until reboot.
    bool receiveOnly = false;
    // The serial backend was selected while the HAT still owned the bus. A
    // board that selected it never returns to pulses.
    bool serialSelected = false;
    // A coherent position/current/voltage/alarm sample from the other
    // master's reads.
    bool telemetryFresh = false;
    // A register 0/1 snapshot from the same telemetry continuity.
    bool controlsFresh = false;
    // Register 0: 0 = STEP/DIR control, 1 = serial control.
    uint16_t modbusEnable = UINT16_MAX;
};

enum class Interlock : uint8_t {
    NotShared,        // no HAT, or the HAT owns the bus: other rules apply
    Ready,            // the other master reports STEP/DIR control
    MonitorSilent,    // its telemetry or control reads stopped
    DriveNotStepDir,  // it took the drive out of STEP/DIR control
};

inline bool active(const Observation &seen) {
    return seen.hatDetected && seen.receiveOnly && !seen.serialSelected;
}

inline Interlock interlock(const Observation &seen) {
    if (!active(seen)) return Interlock::NotShared;
    if (!seen.telemetryFresh || !seen.controlsFresh) return Interlock::MonitorSilent;
    return seen.modbusEnable == 0 ? Interlock::Ready : Interlock::DriveNotStepDir;
}

/// STEP pulses and the ENA output may be used only in this state.
inline bool pulsesAllowed(const Observation &seen) {
    return interlock(seen) == Interlock::Ready;
}

inline const char *reason(Interlock state) {
    switch (state) {
        case Interlock::Ready:
            return "Another RS485 master guards the motor; moving on STEP/DIR";
        case Interlock::MonitorSilent:
            return "Shared RS485 bus: waiting for the other master's telemetry and control reads";
        case Interlock::DriveNotStepDir:
            return "Shared RS485 bus: the other master holds the drive out of STEP/DIR control";
        case Interlock::NotShared:
            break;
    }
    return "";
}

}  // namespace shared_bus
