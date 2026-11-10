#pragma once

#include <cstdint>

// A second, CPU-local stop path. Raw level3 IRAM interrupts bypass the
// ordinary UART2 task/driver. Its deadline assumes bounded masking of level3
// interrupts and working CPU/APB/UART/RS485/motor hardware. It is not a power
// disconnect and provides no dead-CPU, spinlock or broken-bus guarantee.
enum class OssmHatEmergencyCause : uint32_t {
    None, GuardDeadline, TelemetryDeadline, StopTimeout, ExplicitFault,
    OffOnlyPreflight, CacheOffPreflight, Qualification
};

struct OssmHatEmergencyStatus {
    bool available = false;
    bool irqInstalled = false;
    bool outputMayBeOn = false;
    bool isolated = false;
    bool ownerParked = false;
    bool transmitted = false;
    bool txDone = false;
    bool echoVerified = false;
    bool offVerified = false;
    bool quietVerified = false;
    bool failed = false;
    bool restoring = false;
    bool restored = false;
    bool offOnlyPassed = false;
    bool cacheOffPassed = false;
    bool qualificationPassed = false;
    bool cacheOffAtTrip = false;
    bool cacheOffAtTx = false;
    OssmHatEmergencyCause cause = OssmHatEmergencyCause::None;
    uint32_t epoch = 0;
    uint32_t restoredEpoch = 0;
    // Dedicated TG1T1 microseconds, not esp_timer or uptime. Subtract only
    // timestamps in this same modulo32-bit domain.
    uint32_t nowUs = 0;
    uint32_t capturedAtMs = 0; // boot millis paired with nowUs outside IRQ
    uint32_t tripAtUs = 0;
    uint32_t isolatedAtUs = 0;
    uint32_t transmittedAtUs = 0;
    uint32_t txDoneAtUs = 0;
    uint32_t echoAtMs = 0;
    uint32_t offAtMs = 0;
    uint32_t quietAtMs = 0;
    uint32_t lastTelemetryAtMs = 0;
    uint32_t telemetrySequence = 0;
    uint32_t telemetryContinuity = 0;
    uint32_t rawOffWrites = 0;
    uint32_t rxOverflows = 0;
};

// Guard APIs: native32-bit publications, no waiting or driver/mux changes.
// Heartbeat does not clear a stop obligation. The obligation is set before
// ANY output-enable write and survives disarm/fault until fresh off+quiet.
void ossmHatEmergencyGuardHeartbeat();
bool ossmHatEmergencyMarkOutputPossible();
void ossmHatEmergencyStopRequested();
void ossmHatEmergencyFault();
// Unconditional sticky veto for a claimed reviewed procedure, including the
// interval BEFORE it reaches the UART-owner checkpoint. Publish this during
// cancellation/first-fault revocation before waiting for a command mutex.
void ossmHatEmergencyCancelProcedure();
OssmHatEmergencyStatus getOssmHatEmergencyStatus();

// Explicit reviewed, asynchronous procedures, serviced by the sole owner.
// Preflights require fresh mode1/off/quiet and perform ONLY output-off plus
// read-only verification; the cache variant really disables flash caches for
// a bounded IRAM window. Their receipts are published only after successful
// same-boot restoration and new-epoch fresh telemetry. No motion is started.
bool requestOssmHatEmergencyPreflight(bool cacheOff);

// Called only by the bounded finite qualification's guard after its positive
// progress/corridor checks. The owner parks at a lock-free, command-free
// checkpoint; the real hardware timer then takes TX22 AND DE2 away from it.
bool requestOssmHatTimerQualification();

// Only successful explicit qualification/preflight may restore in this boot.
// The guard must independently accept its finite qualification result before
// requesting restoration. Actual faults cannot use this API. No enable,
// target, mode change, EEPROM write or automatic arm is performed.
bool requestOssmHatQualificationRestore();
