#pragma once
#include "ossmHat.h"
#include "ossmHatEmergency.h"
#include <cstddef>

// Sole UART-owner adapter. These are not user command APIs.
namespace ossm_hat_emergency {
bool installHardware();
void ownerLease(bool exclusive115200);
bool ordinaryTxAllowed();
bool configurationAllowed();
bool restoreVetoed();
bool protocolFailed();
bool beginConfiguration();
void endConfiguration();
bool beginTest(OssmHatEmergencyCause cause);
void ownerParked(bool parked);
void cacheOffWindow();
size_t receive(uint8_t *out, size_t capacity);
bool readFullRegisters();
void acknowledgeEcho(uint32_t nowMs);
void observe(const OssmHatTelemetry &, const OssmHatControlSnapshot &);
void failed();
bool beginRestore();
bool resetUnderConstantFence();
bool finishRestore();
} // namespace ossm_hat_emergency
