#include "serialIdentity.h"
#if defined(ARDUINO) && !defined(PIO_UNIT_TESTING)
#include <Arduino.h>
#include <atomic>
#include <cmath>
#include "ossm/streaming/drawn_pattern.h"
#include "constants/Version.h"
#include "serialIdentityProtocol.h"
#include "services/ossmHat.h"
#include "services/motorFeedback.h"
#include "services/motorSafety.h"
#include <ArduinoJson.h>
#include <serialIdentityBuild.h>
#if ARDUINO_USB_CDC_ON_BOOT && !ARDUINO_USB_MODE
#include <USB.h>
#endif
#ifndef FIRMWARE_BUILD_SHA
#define FIRMWARE_BUILD_SHA "unknown"
#endif
#ifndef FIRMWARE_TRACK
#define FIRMWARE_TRACK "development"
#endif

namespace {
TaskHandle_t identityTask = nullptr;
#ifdef VERSIONDEV
std::atomic<void (*)(const char*)> developmentCommand{nullptr};
std::atomic<void (*)()> developmentPoll{nullptr};
#endif

void announce() {
    const uint64_t mac = ESP.getEfuseMac();
    char deviceId[13];
    snprintf(deviceId, sizeof(deviceId), "%02X%02X%02X%02X%02X%02X",
             unsigned(mac & 255), unsigned((mac >> 8) & 255), unsigned((mac >> 16) & 255),
             unsigned((mac >> 24) & 255), unsigned((mac >> 32) & 255), unsigned((mac >> 40) & 255));
    const serialIdentity::Info info{RAD_ID_PRODUCT, RAD_ID_MODEL, VERSION, FIRMWARE_TRACK,
        FIRMWARE_BUILD_SHA, deviceId, RAD_ID_FLASH_BYTES, RAD_ID_PSRAM_BYTES,
        ESP.getFlashChipSize(), ESP.getPsramSize()};
    char frame[640];
    const size_t size = serialIdentity::format(frame, sizeof(frame), info);
    if (size) Serial.write(reinterpret_cast<const uint8_t*>(frame), size);
}

void announceHat() {
    const auto status = getOssmHatStatus();
    JsonDocument doc;
    doc["detected"] = status.detected;
    doc["fresh"] = status.fresh;
    doc["passive"] = status.passive;
    doc["baudRate"] = status.baudRate;
    doc["ageMs"] = status.ageMs;
    doc["sampleRateHz"] = status.sampleRateHz;
    doc["positionSource"] = motorPositionUsesRs485() ? "rs485" : "stepper";
    OssmHatTelemetry sample;
    if (readOssmHatTelemetry(sample)) {
        doc["encoderCounts"] = sample.positionCounts;
        doc["currentAmps"] = sample.currentAmps;
        doc["voltageVolts"] = sample.voltageVolts;
        doc["alarm"] = sample.alarm;
    }
    const auto safety = getMotorSafetyStatus();
    doc["guardReady"] = safety.ready;
    doc["guardArmed"] = safety.armed;
    doc["guardLatched"] = safety.latched;
    doc["guardFault"] = safety.fault;
    doc["outputDisabled"] = safety.outputDisableVerified;
    doc["externalGuard"] = getMotorSerialSafetyStatus().externalGuard;
    char frame[640] = "RAD_HAT ";
    const size_t size = serializeJson(doc, frame + 8, sizeof(frame) - 10);
    frame[size + 8] = '\n';
    Serial.write(reinterpret_cast<const uint8_t *>(frame), size + 9);
}

#if OSSM_HAT_SERIAL_BACKEND
void announceDriveConfig() {
    OssmHatDriveConfig config;
    if (!readOssmHatDriveConfig(config)) {
        Serial.print("RAD_DRIVE_CONFIG unavailable\n");
        return;
    }
    char hex[sizeof(config.frame) * 2 + 1];
    constexpr char digits[] = "0123456789abcdef";
    for (size_t i = 0; i < sizeof(config.frame); ++i) {
        hex[2*i] = digits[config.frame[i] >> 4];
        hex[2*i+1] = digits[config.frame[i] & 15];
    }
    hex[sizeof(hex)-1] = '\0';
    Serial.printf("RAD_DRIVE_CONFIG %lu %s\n", static_cast<unsigned long>(config.observedAtMs), hex);
}
#endif

void announcePatternRate(uint32_t sequence) {
    streaming::PatternRateStamp stamp{};
    if (!streaming::drawnPatternRateStamp(sequence, stamp)) {
        Serial.printf("RAD_PATTERN_RATE unavailable %lu\n", static_cast<unsigned long>(sequence));
        return;
    }
    Serial.printf("RAD_PATTERN_RATE %lu %lld %u\n", static_cast<unsigned long>(stamp.sequence),
                  std::llround(stamp.goalRateHz * 1.e9), stamp.rawHandoff);
}
void announcePatternTime(uint32_t sequence) {
    streaming::PatternRateStamp stamp{};
    if (!streaming::drawnPatternRateStamp(sequence, stamp)) {
        Serial.printf("RAD_PATTERN_TIME unavailable %lu\n", static_cast<unsigned long>(sequence));
        return;
    }
    Serial.printf("RAD_PATTERN_TIME %lu %lu %lu %lld %lld %lld %u %u %u %u\n",
                  static_cast<unsigned long>(stamp.sequence), static_cast<unsigned long>(stamp.atMs),
                  static_cast<unsigned long>(stamp.periodMs), std::llround(stamp.phase * 1e9),
                  std::llround(stamp.rateHz * 1e6), std::llround(stamp.phaseAcceleration * 1e6),
                  unsigned(stamp.leadBoundMs), unsigned(stamp.first), unsigned(stamp.serial), unsigned(stamp.leadKnown));
}

void announcePatternFeel(uint32_t sequence) {
    streaming::PatternRateStamp stamp{};
    if (!streaming::drawnPatternRateStamp(sequence, stamp)) {
        Serial.printf("RAD_PATTERN_FEEL unavailable %lu\n", static_cast<unsigned long>(sequence));
        return;
    }
    Serial.printf("RAD_PATTERN_FEEL %lu %lu %lld %lld %lld %lld %lu\n",
                  static_cast<unsigned long>(stamp.sequence), static_cast<unsigned long>(stamp.atMs),
                  std::llround(stamp.requestedStrength * 1e5), std::llround(stamp.gain * 1e6),
                  std::llround(stamp.targetGain * 1e6), std::llround(stamp.carrier * 1e9),
                  static_cast<unsigned long>(stamp.frameIndex));
}

void console(void*) {
    // Never wait for a USB host or block application setup on a serial cable.
    vTaskDelay(pdMS_TO_TICKS(1000));
    announce();
    uint32_t lastAnnouncement = millis();
    serialIdentity::CommandBuffer parser;
    for (;;) {
        // Bound serial flood work so application tasks still get scheduled.
        for (int count = 0; count < 128 && Serial.available(); ++count) {
            const char* command = parser.push(Serial.read());
            if (!command) continue;
            uint32_t argument;
            if (strcmp(command, "info") == 0 || strcmp(command, "identity") == 0) {
                if (millis() - lastAnnouncement >= 250) {
                    announce();
                    lastAnnouncement = millis();
                }
            }
            else if (serialIdentity::unsignedArgument(command, "clock", argument)) {
                const auto sampledAt = millis();
                Serial.printf("RAD_CLOCK %lu %lu\n", static_cast<unsigned long>(argument),
                              static_cast<unsigned long>(sampledAt));
            }
            else if (serialIdentity::unsignedArgument(command, "patterntime", argument)) announcePatternTime(argument);
            else if (serialIdentity::unsignedArgument(command, "patternrate", argument)) announcePatternRate(argument);
            else if (serialIdentity::unsignedArgument(command, "patternfeel", argument))
                announcePatternFeel(argument);
            else if (strcmp(command, "patternwork") == 0) {
                streaming::PatternWork work;
                if (streaming::stoppedPatternWork(work)) {
                    Serial.printf("RAD_PATTERN_WORK %lu %lu %lu %lu %lu\n",
                                  static_cast<unsigned long>(work.frames), static_cast<unsigned long>(work.retries),
                                  static_cast<unsigned long>(work.maxPlanUs), static_cast<unsigned long>(work.maxSubmitUs),
                                  static_cast<unsigned long>(work.maxRenewLockUs));
                } else {
                    Serial.println("RAD_PATTERN_WORK unavailable");
                }
            }
            else if (serialIdentity::unsignedArgument(command, "patternsched", argument)) {
                streaming::PatternWork work;
                if (argument <= 1 && streaming::stoppedPatternWork(work)) {
                    const auto &turn = argument == 0 ? work.lastService : work.longestService;
                    Serial.printf("RAD_PATTERN_SCHED %u %u %u %u %u %u %u %u %u\n",
                                  unsigned(argument), unsigned(turn.atMs), unsigned(turn.frame),
                                  unsigned(turn.offUs), unsigned(turn.outerUs), unsigned(turn.mailboxUs),
                                  unsigned(turn.readUs), unsigned(turn.commandGateUs), unsigned(turn.serviceUs));
                } else {
                    Serial.println("RAD_PATTERN_SCHED unavailable");
                }
            }
            else if (strcmp(command, "patterncost") == 0) {
                streaming::PatternWork work;
                if (streaming::stoppedPatternWork(work)) {
                    Serial.printf("RAD_PATTERN_COST %lu %lu %llu %lu %lu %llu %lu %lu\n",
                                  static_cast<unsigned long>(work.plans), static_cast<unsigned long>(work.movingPlans),
                                  static_cast<unsigned long long>(work.planTotalUs), static_cast<unsigned long>(work.planOver2ms),
                                  static_cast<unsigned long>(work.submissions), static_cast<unsigned long long>(work.submitTotalUs),
                                  static_cast<unsigned long>(work.submitOver2ms), static_cast<unsigned long>(work.rawStartSequence));
                } else {
                    Serial.println("RAD_PATTERN_COST unavailable");
                }
            }
            else if (strcmp(command, "hat") == 0) announceHat();
#if OSSM_HAT_SERIAL_BACKEND
            else if (strcmp(command, "driveconfig") == 0) announceDriveConfig();
#endif
#ifdef VERSIONDEV
            else if (const auto handler = developmentCommand.load()) handler(command);
#endif
        }
#ifdef VERSIONDEV
        if (const auto poll = developmentPoll.load()) poll();
#endif
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
}

void configureSerialIdentityUsb() {
#if ARDUINO_USB_CDC_ON_BOOT && !ARDUINO_USB_MODE
    // Only TinyUSB descriptors are programmable. Keep VID/PID, serial number,
    // and the existing USB transport unchanged for reconnects and flashing.
    USB.productName(RAD_ID_MODEL);
    USB.manufacturerName("Research + Desire");
#elif ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE && !defined(VERSIONDEV) && !defined(RAD_HIL_VARIANT)
    // Production HWCDC also needs room for the complete identity record.
    Serial.setTxBufferSize(1024);
    Serial.setTxTimeoutMs(1000);
#endif
}

void startSerialIdentity() {
    if (identityTask) return;
#ifdef VERSIONDEV
    constexpr uint32_t stackBytes = 6144; // Includes development screen encoding.
#else
    constexpr uint32_t stackBytes = 3072;
#endif
    if (xTaskCreate(console, "serialIdentity", stackBytes, nullptr, 1, &identityTask) != pdPASS)
        Serial.println("RAD_ID_ERROR console_task");
}

#ifdef VERSIONDEV
void setSerialDevelopmentHandler(void (*command)(const char*), void (*poll)()) {
    developmentPoll.store(poll);
    developmentCommand.store(command);
}
#endif
#else
void configureSerialIdentityUsb() {}
void startSerialIdentity() {}
#ifdef VERSIONDEV
void setSerialDevelopmentHandler(void (*)(const char*), void (*)()) {}
#endif
#endif
