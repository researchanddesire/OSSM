#include "ossmHealth.h"

#if defined(ARDUINO) && !defined(PIO_UNIT_TESTING)
#include <Arduino.h>
#include <WiFi.h>
#include <atomic>
#include <esp_system.h>
#include <esp_timer.h>
#include <serialIdentityBuild.h>

#include "FirmwareProvenance.h"
#include "ossm/state/calibration.h"
#include "ossm/state/state.h"
#include "services/ossmHat.h"
#include "services/ossmHatMode.h"
#include "services/ossmHealthProtocol.h"
#include "services/serialIdentity.h"

#ifndef FIRMWARE_BUILD_SHA
#define FIRMWARE_BUILD_SHA "unknown"
#endif
#ifndef FIRMWARE_TRACK
#define FIRMWARE_TRACK "development"
#endif

namespace sml = boost::sml;
using namespace sml;

namespace {
// Zero until the boot record is written; then random for this boot.
std::atomic<uint32_t> bootId{0};
std::atomic<uint32_t> menuProgressMs{0};
uint32_t lastHeartbeatMs = 0;

unsigned long long uptimeMs() {
    return static_cast<unsigned long long>(esp_timer_get_time() / 1000);
}

// One buffered write per record, so concurrent task output cannot split it.
// Serial is the console UART on every OSSM build, independent of log levels.
void emit(const char *frame, size_t size) {
    if (size) Serial.write(reinterpret_cast<const uint8_t *>(frame), size);
}

bool hatDetected() {
#if OSSM_HAT_SERIAL_BACKEND
    return ossmHatDetected();
#else
    return ossm_hat::isPresent();
#endif
}

void wifiConnected(WiFiEvent_t, WiFiEventInfo_t info) {
    char ip[16];
    snprintf(ip, sizeof(ip), IPSTR, IP2STR(&info.got_ip.ip_info.ip));
    char frame[160];
    emit(frame, ossm_health::formatWifiConnected(frame, sizeof(frame), bootId.load(),
                                                 uptimeMs(), ip));
}
}  // namespace

void ossmHealthBoot() {
    if (bootId.load() != 0) return;
    uint32_t id = esp_random();
    if (id == 0) id = 1;
    char deviceId[13];
    serialIdentityDeviceId(deviceId);
    // The same running-image digest the Lockbox reports.
    const auto imageHash = firmware::provenance::runningImageSha256();
    ossm_health::Boot boot;
    boot.bootId = id;
    boot.uptimeMs = uptimeMs();
    boot.buildSha = FIRMWARE_BUILD_SHA;
    boot.track = FIRMWARE_TRACK;
    boot.variant = ossm_health::variantForFlash(RAD_ID_FLASH_BYTES);
    boot.deviceId = deviceId;
    boot.imageSha256 = imageHash.c_str();
    boot.resetReason = static_cast<int>(esp_reset_reason());
    boot.hatMode = ossm_hat_mode::name(ossmHatBootMode());
    char frame[448];
    const size_t size = ossm_health::formatBoot(frame, sizeof(frame), boot);
    lastHeartbeatMs = millis();
    emit(frame, size);
    bootId.store(id);
    // Every got-IP, including a reconnection. Registered before the
    // communication task starts Wi-Fi.
    WiFi.onEvent(wifiConnected, ARDUINO_EVENT_WIFI_STA_GOT_IP);
}

void ossmHealthMqttConnected(const char *broker) {
    char frame[224];
    emit(frame, ossm_health::formatMqttConnected(frame, sizeof(frame), bootId.load(),
                                                 uptimeMs(), broker ? broker : ""));
}

void ossmHealthMenuProgress() {
    menuProgressMs.store(millis(), std::memory_order_relaxed);
}

void ossmHealthHeartbeat() {
    const uint32_t id = bootId.load();
    if (id == 0) return;
    if (!ossm_health::heartbeatDue(lastHeartbeatMs, millis())) return;
    // Read progress before the clock: the menu task can advance it meanwhile,
    // and the other order could report a wrapped, enormous age.
    const uint32_t menuMs = menuProgressMs.load(std::memory_order_relaxed);
    const uint32_t now = millis();
    lastHeartbeatMs = now;
    ossm_health::Heartbeat beat;
    beat.bootId = id;
    beat.uptimeMs = uptimeMs();
    beat.mainMenu = menuMs != 0 && stateMachine != nullptr && stateMachine->is("menu.idle"_s);
    beat.menuAgeMs = now - menuMs;
    beat.hat = hatDetected();
    beat.homed = calibration.isHomed;
    char frame[224];
    emit(frame, ossm_health::formatHeartbeat(frame, sizeof(frame), beat));
}
#else
void ossmHealthBoot() {}
void ossmHealthMqttConnected(const char *) {}
void ossmHealthMenuProgress() {}
void ossmHealthHeartbeat() {}
#endif
