#pragma once

#include <cstdint>
#include "ossmHatEmergency.h"

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
    uint16_t attempts = 0;
    bool hadFailure = false;
    uint32_t firstFailureAtMs = 0;
    uint16_t firstFailureMode = UINT16_MAX;
    uint16_t firstFailureOutput = UINT16_MAX;
};

struct OssmHatControlSnapshot {
    bool valid = false;
    uint16_t modbusEnable = UINT16_MAX;
    uint16_t outputEnable = UINT16_MAX;
    uint32_t updatedAtMs = 0;
    uint32_t continuity = 0;
};

constexpr uint32_t ossmHatFreshnessMs = 250;

// Starts the sole background UART2 owner. Discovery/telemetry is read-only;
// explicit guarded APIs separately queue RAM motion setup/absolute targets or
// priority output disable. No EEPROM writes are sent. Call after pin setup.
void initOssmHat();
bool ossmHatDetected();
bool readOssmHatTelemetry(OssmHatTelemetry &out);
OssmHatStatus getOssmHatStatus();
// Read-only register 0/1 snapshot, refreshed once a second by the active UART
// owner. False after 2s, on a telemetry gap, or before controls were observed.
bool readOssmHatControlSnapshot(OssmHatControlSnapshot &out);

// Priority request serviced by the UART owner, never by a second bus client.
// The caller must immediately disable GPIO ENA/pulses before requesting this.
// It writes ONLY register1=0 and reads registers0/1 back. It never changes
// control mode: mode0 remains mode0. Serial stop is verified only for mode1
// with output bit0 off. It never restores control or physical power and
// refuses TX on a passive/shared bus.
// Idempotent until an explicit reviewed clear: repeated enforcement preserves
// the first result instead of replacing it or re-queueing bus commands.
// After an unverified stop, the owner may reassert ONLY output-off/readback
// at>=1s intervals on a restored fresh sole-master link. First failure/attempt
// diagnostics remain available; no motion or mode-selection retry occurs.
void requestOssmHatEmergencyStop();
OssmHatStopStatus getOssmHatEmergencyStopStatus();

// Explicit review only: clear the stop-command latch while ENA is high and no
// owner command is pending/in flight. No TX, drive restore or automatic arm.
bool clearOssmHatStopAfterReview();

// TX/RX/DE remain reserved while discovery/reconnect is running, including
// when no motor answers. IO15 is not connected to the V2.3 HAT and stays free.
bool ossmHatOwnsPin(int pin);

enum class OssmHatPositionFault : uint8_t {
    None, Cancelled, InvalidParameters, SessionRejected, Expired,
    TelemetryLost, SharedBus, UnsupportedBaud, ControlMismatch, Protocol
};

struct OssmHatPositionStatus {
    uint64_t session = 0;
    bool requested = false;
    bool prepared = false;
    bool enabled = false;
    bool pending = false;
    bool faulted = false;
    OssmHatPositionFault fault = OssmHatPositionFault::None;
    uint32_t queuedRequestId = 0;
    uint32_t acknowledgedRequestId = 0;
    int32_t targetCounts = 0;
    // Intended target of the correlated request, not the FC7B response word.
    int32_t acknowledgedTargetCounts = 0;
    uint16_t maximumSpeedRpm = 0;
    uint16_t accelerationRpmPerSec = 0;
    uint32_t acknowledgedAtMs = 0;
    // Raw low-word-first FC7B response value; the immediate current-position
    // word need not equal the accepted target. Diagnostic only: it never
    // refreshes coherent FC03 telemetry or proves physical arrival.
    bool vendorReplyAvailable = false;
    int32_t vendorReplyValue = 0;
    uint32_t vendorReplyAtMs = 0;
};

// The guard owns authorization. Invoked outside the HAT state lock before
// every committed setup/enable/position TX; zero is never a valid session.
// A true motion argument additionally requires live arm/enable authorization.
using OssmHatPositionValidator = bool (*)(uint64_t session, bool motion);
void setOssmHatPositionValidator(OssmHatPositionValidator validator);

// Software transport only. Preparation requires already-verified serial
// mode1/output0, exclusive115200 UART and healthy coherent >=30Hz telemetry.
// It seeds an absolute encoder hold and verifies RAM speed/acceleration;
// output remains disabled until a separate explicit guarded enable request.
// No motor encoder re-zero, STEP/DIR gear, baud or EEPROM writes are made.
bool requestOssmHatPreparePositionControl(uint64_t session, int32_t holdCounts,
                                        uint16_t maximumSpeedRpm,
                                        uint16_t accelerationRpmPerSec);
bool requestOssmHatEnablePositionControl(uint64_t session);

// A single coalescing mailbox: newer targets replace unsent targets, with at
// most one ordinary absolute target transaction per20ms. Positions
// are signed raw encoder counts (32768/rev), never planner steps. FC0x7B is
// absolute; FC0x10@0x0C is incremental and is deliberately not used for moves.
// Limits must equal the disabled/readback-verified session limits; trajectory
// speed/acceleration changes belong to the local planner, not live RAM writes.
// A positive transport ACK is not physical arrival; use fresh telemetry.
bool queueOssmHatPositionTarget(uint64_t session, uint32_t requestId,
                               int32_t targetCounts, uint16_t maximumSpeedRpm,
                               uint16_t accelerationRpmPerSec,
                               uint32_t maximumAgeMs = 100);
OssmHatPositionStatus getOssmHatPositionStatus();

// Revokes all queued intent immediately and queues the existing priority
// register1=0 stop/readback (no mode change). A frame committed before cancellation can finish;
// only verified serial stop is a serial-motion stop acknowledgement. GPIO ENA
// or pulse cessation alone is not proof that a serial move has stopped.
void cancelOssmHatPositionControl();

enum class OssmHatControlWriteType : uint8_t {
    None, AbsoluteTarget, OutputEnable, OutputDisable, SpeedSetup,
    AccelerationSetup, ModeWrite, Other
};

struct OssmHatTxAudit {
    uint64_t controlWrites = 0;
    uint64_t absoluteTargets = 0;
    uint64_t outputEnables = 0;
    uint64_t outputDisables = 0;
    uint64_t setupWrites = 0;
    uint64_t modeWrites = 0;
    OssmHatControlWriteType lastType = OssmHatControlWriteType::None;
    uint8_t lastFunction = 0;
    uint16_t lastRegister = UINT16_MAX;  // FC7B has no register address
    int32_t lastValue = 0;  // request target is int32 MSB-first; FC06 is uint16
    uint32_t lastAtMs = 0;
    uint16_t lastAcceptedBytes = 0;
    bool lastCompleteFrame = false;
};

// Actual UART writes accepted by the sole owner, before ACK/readback. Reads,
// queued intents and rejected/cancelled requests do not count. A partial
// control frame counts conservatively and is labeled incomplete. No reset API.
OssmHatTxAudit getOssmHatTxAudit();
