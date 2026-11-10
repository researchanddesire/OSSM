#pragma once
#include "ossmHatEmergency.h"
#include "serial_timer_probe.h"

// Paged so the result and the pinned RadBLE correlation/state envelope fit
// its 509-byte limit. All pages retain their procedure identity; a reader
// must bracket terminal evidence with matching summary pages.
namespace serial_stop_diagnostics {
template <class Document>
bool emergencyPage(Document &doc, int page, const OssmHatEmergencyStatus &s) {
    if (page < 0 || page >= 5) return false;
    doc["path"] = "motor.serialEmergency";
    doc["page"] = page;
    doc["pages"] = 5;
    doc["epoch"] = s.epoch;
    doc["restoredEpoch"] = s.restoredEpoch;
    doc["cause"] = static_cast<uint32_t>(s.cause);
    if (page == 0) {
        doc["available"] = s.available;
        doc["irqInstalled"] = s.irqInstalled;
        doc["isolated"] = s.isolated;
        doc["failed"] = s.failed;
        doc["restored"] = s.restored;
        doc["offOnlyPassed"] = s.offOnlyPassed;
        doc["cacheOffPassed"] = s.cacheOffPassed;
        doc["qualificationPassed"] = s.qualificationPassed;
    } else if (page == 1) {
        doc["outputMayBeOn"] = s.outputMayBeOn;
        doc["ownerParked"] = s.ownerParked;
        doc["transmitted"] = s.transmitted;
        doc["txDone"] = s.txDone;
        doc["echoVerified"] = s.echoVerified;
        doc["offVerified"] = s.offVerified;
        doc["quietVerified"] = s.quietVerified;
        doc["restoring"] = s.restoring;
        doc["cacheOffAtTrip"] = s.cacheOffAtTrip;
        doc["cacheOffAtTx"] = s.cacheOffAtTx;
    } else if (page == 2) {
        doc["tripAtUs"] = s.tripAtUs;
        doc["isolatedAtUs"] = s.isolatedAtUs;
        doc["transmittedAtUs"] = s.transmittedAtUs;
        doc["txDoneAtUs"] = s.txDoneAtUs;
        doc["nowUs"] = s.nowUs;
        doc["capturedAtMs"] = s.capturedAtMs;
    } else if (page == 3) {
        doc["echoAtMs"] = s.echoAtMs;
        doc["offAtMs"] = s.offAtMs;
        doc["quietAtMs"] = s.quietAtMs;
        doc["lastTelemetryAtMs"] = s.lastTelemetryAtMs;
    } else {
        doc["telemetrySequence"] = s.telemetrySequence;
        doc["telemetryContinuity"] = s.telemetryContinuity;
        doc["rawOffWrites"] = s.rawOffWrites;
        doc["rxOverflows"] = s.rxOverflows;
    }
    return true;
}

template <class Document>
bool timerProbePage(Document &doc, int page, const serial_timer_probe::Status &s) {
    if (page < 0 || page >= 3) return false;
    doc["path"] = "motor.serialTimerProbe";
    doc["page"] = page;
    doc["pages"] = 3;
    doc["phase"] = static_cast<uint8_t>(s.phase);
    doc["failure"] = static_cast<uint8_t>(s.failure);
    doc["startedAtMs"] = s.startedAtMs;
    if (page == 0) {
        doc["active"] = s.active;
        doc["finished"] = s.finished;
        doc["passed"] = s.passed;
        doc["enabled"] = s.enabled;
        doc["targetWritten"] = s.targetWritten;
        doc["irqStopProven"] = s.irqStopProven;
        doc["serialDisabled"] = s.serialDisabled;
        doc["offQuiet"] = s.offQuiet;
        doc["restoreVerified"] = s.restoreVerified;
    } else if (page == 1) {
        doc["positiveIntervals"] = s.positiveIntervals;
        doc["quietSamples"] = s.quietSamples;
        doc["restoredQuietSamples"] = s.restoredQuietSamples;
        doc["originCounts"] = s.originCounts;
        doc["targetCounts"] = s.targetCounts;
        doc["lastCounts"] = s.lastCounts;
    } else {
        doc["irqAtMs"] = s.irqAtMs;
        doc["offAtMs"] = s.offAtMs;
        doc["restoredContinuity"] = s.restoredContinuity;
    }
    return true;
}
} // namespace serial_stop_diagnostics
