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
    // Measured accepted coherent samples over the last second. Readiness
    // requires at least a half-second observation, not an instantaneous rate.
    float sampleRateHz = 0;
    uint32_t sampleWindowMs = 0;
    uint16_t sampleCount = 0;
    bool startupReady = false;
};

struct OssmHatStopStatus {
    bool requested = false;
    bool complete = false;
    bool verified = false;
    bool passiveBlocked = false;
    uint16_t modbusEnable = UINT16_MAX;
    uint16_t driveOutputEnable = UINT16_MAX;
    uint32_t requestedAtMs = 0;
    uint32_t completedAtMs = 0;
};

struct OssmHatControlSnapshot {
    bool valid = false;
    uint16_t modbusEnable = UINT16_MAX;
    uint16_t outputEnable = UINT16_MAX;
    uint32_t updatedAtMs = 0;
    uint32_t continuity = 0;
};

struct OssmHatExternalControlStatus {
    bool requested = false;
    bool complete = false;
    bool verified = false;
    bool preempted = false;
    bool passiveBlocked = false;
    uint16_t modbusEnable = UINT16_MAX;
    uint16_t outputEnable = UINT16_MAX;
    uint32_t requestedAtMs = 0;
    uint32_t completedAtMs = 0;
};

constexpr uint32_t ossmHatFreshnessMs = 250;

// Starts a background UART2 reader. Ordinary discovery/telemetry is read-only;
// only an explicit emergency-stop request writes the output-disable registers.
// No moves or EEPROM/configuration writes are sent. Call after pin setup.
void initOssmHat();
bool ossmHatDetected();
bool readOssmHatTelemetry(OssmHatTelemetry &out);
OssmHatStatus getOssmHatStatus();
// Read-only register 0/1 snapshot, refreshed once a second by the active UART
// owner. False after 2s, on a telemetry gap, or before controls were observed.
bool readOssmHatControlSnapshot(OssmHatControlSnapshot &out);

// Priority request serviced by the UART owner, never by a second bus client.
// The caller must immediately disable GPIO ENA/pulses before requesting this.
// It latches Modbus control=1/output=0 and verifies both registers. It never
// restores control or physical power and refuses TX on a passive/shared bus.
// Idempotent until an explicit reviewed clear: repeated enforcement preserves
// the first result instead of replacing it or re-queueing bus commands.
void requestOssmHatEmergencyStop();
OssmHatStopStatus getOssmHatEmergencyStopStatus();

// Explicit review/recovery only. The caller additionally proves the guard is
// unarmed and the physical encoder was stationary for >=100ms. This API checks
// ENA high, fresh healthy low-current feedback/control data and sole UART
// ownership. It queues ONLY register0=0 and read-back; it never enables ENA,
// writes register1/targets/settings, or arms a motion mode.
bool requestOssmHatExternalControlAfterReview();
OssmHatExternalControlStatus getOssmHatExternalControlStatus();

// Explicit review only: clear the stop-command latch while ENA is high and no
// owner command is pending/in flight. No TX, drive restore or automatic arm.
bool clearOssmHatStopAfterReview();

// TX/RX/DE remain reserved while discovery/reconnect is running, including
// when no motor answers. IO15 is not connected to the V2.3 HAT and stays free.
bool ossmHatOwnsPin(int pin);
