#pragma once

#include <cstdint>

// A single CRC-checked AIM read of 0x0E..0x17 (or a larger encompassing
// block). Encoder counts are 32768/revolution; STEP/DIR gear is unrelated.
struct OssmHatTelemetry {
    int32_t positionCounts = 0;
    float currentAmps = 0;
    float voltageVolts = 0;
    uint16_t alarm = 0;
    uint32_t updatedAtMs = 0;
    uint32_t sequence = 0;
    // Changes after any telemetry gap longer than freshness. Position
    // references must bind to this epoch even if no getter observed the gap.
    uint32_t continuity = 0;
};

struct OssmHatStatus {
    // Detection is latched after a complete valid sample. A disconnected HAT
    // must never silently select the legacy sensors during a motion session.
    bool detected = false;
    bool fresh = false;
    bool passive = false;
    bool discoveryComplete = false;
    uint32_t baudRate = 0;
    uint32_t ageMs = UINT32_MAX;
    uint32_t validFrames = 0;
    uint32_t crcErrors = 0;
};

constexpr uint32_t ossmHatFreshnessMs = 250;

// Starts a background UART2 reader. No register writes, motor moves or drive
// configuration are sent. Call after the board's expansion-pin setup.
void initOssmHat();
bool ossmHatDetected();
bool readOssmHatTelemetry(OssmHatTelemetry &out);
OssmHatStatus getOssmHatStatus();

// TX/RX/DE remain reserved while discovery/reconnect is running, including
// when no motor answers. IO15 is not connected to the V2.3 HAT and stays free.
bool ossmHatOwnsPin(int pin);
