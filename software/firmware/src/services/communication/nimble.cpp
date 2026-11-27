#include "nimble.h"

#include <ArduinoJson.h>
#include <constants/LogTags.h>
#include <services/board.h>
#include <services/tasks.h>

#include <atomic>
#include <mutex>
#include <queue>

#include "command.hpp"
#include "command/commands.hpp"
#include "config.hpp"
#include "gpio.hpp"
#include "motion_link.h"
#include "services/motorSafety.h"
#include "services/radHil.h"
#include "ossm/OSSM.h"
#include "pairing.hpp"
#include "patterns.hpp"
#include "rad_ble.h"
#include "rename.hpp"
#include "services/led.h"
#include "state.hpp"
#include "wifi.hpp"

// Define the global variables
NimBLEServer* pServer = nullptr;

NimBLECharacteristic* pStateCharacteristic = nullptr;
NimBLECharacteristic* pSpeedKnobConfigCharacteristic = nullptr;
NimBLECharacteristic* pLatencyCompensationConfigCharacteristic = nullptr;
NimBLECharacteristic* pCommandCharacteristic = nullptr;

// Set by the BLE host task (disconnects), read by nimbleLoop.
static std::atomic<long> lostConnectionTime{0};
static std::atomic<int> speedOnLostConnection{0};
static const unsigned long RAMP_DURATION_MS =
    2000;  // Duration for speed ramp to zero

// Host task (legacy writes, disconnects) and RAD BLE task (motion requests).
static std::mutex motionOwnerLock;
static motion_link::Owner motionOwner;

void noteBleMotionCommand(uint16_t connHandle, const std::string& command) {
    std::lock_guard<std::mutex> guard(motionOwnerLock);
    if (!motionOwner.takeCommand(connHandle, command, streaming::cancelDrawnPattern)) return;
    // Someone is driving again: the disconnect ramp must not fight them.
    lostConnectionTime = 0;
}

streaming::PatternStart startBlePattern(uint16_t connection, const char *samples,
                                       size_t length, uint32_t periodMs) {
    std::lock_guard<std::mutex> guard(motionOwnerLock);
    // A request already in the server task may race a disconnect.
    if (!pServer || connection == motion_link::kNoConnection ||
        ble_gap_conn_find(connection, nullptr) != 0)
        return streaming::PatternStart::NotReady;
    const auto result = streaming::startDrawnPattern(samples, length, periodMs,
                                  getMotorSafetyStatus().controlRevision);
    if (result == streaming::PatternStart::Accepted) {
        motionOwner.onMotionCommand(connection);
        lostConnectionTime = 0;
    }
    return result;
}

bool setLeasedBleSpeed(uint16_t connection, float value, bool selectBleSource) {
    std::lock_guard<std::mutex> guard(motionOwnerLock);
    if (!pServer || connection == motion_link::kNoConnection ||
        ble_gap_conn_find(connection, nullptr) != 0) return false;
    if (!streaming::applyLeasedBleSpeed(motionOwner, connection, value, selectBleSource)) return false;
    lostConnectionTime = 0;
    return true;
}

bool renewBlePattern(uint16_t connection) {
    std::lock_guard<std::mutex> guard(motionOwnerLock);
    return connection != motion_link::kNoConnection && motionOwner.owner() == connection &&
           streaming::renewDrawnPattern(getMotorSafetyStatus().controlRevision);
}

void restartAdvertisingWithCurrentName() {
    const std::string deviceName = getDeviceName();
    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
    advertising->stop();
    advertising->clearData();
    advertising->enableScanResponse(true);
    advertising->setName(deviceName);
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->addServiceUUID(DEVICE_INFO_SERVICE_UUID);
#ifdef PRETEND_TO_BE_FLESHY_THRUST_SYNC
    advertising->addServiceUUID("0000ffe0-0000-1000-8000-00805f9b34fb");
#endif
    NimBLEDevice::setDeviceName(deviceName);
    ESP_LOGI(NIMBLE_TAG, "Advertising as: %s", deviceName.c_str());
    advertising->start();
}

double easeInOutSine(double t) {
    return 0.5 * (1 + sin(3.1415926 * (t - 0.5)));
}

class StateCallbacks : public NimBLECharacteristicCallbacks {
    // A long read arrives as several ATT requests, each of which calls
    // onRead. Rebuilding the value on every one of them hands the reader
    // chunks from different versions (seen as corrupted JSON on the RADR).
    // Keep the value stable for the duration of one long read.
    uint32_t lastBuildMs = 0;

    void onRead(NimBLECharacteristic* characteristic, NimBLEConnInfo&) override {
        if (!ossm) return;
        const uint32_t now = millis();
        if (lastBuildMs != 0 && now - lastBuildMs < 300) return;
        lastBuildMs = now;
        // RADR needs full legacy state on reads; retain shared BLE metadata.
        JsonDocument value, current;
        const auto stored = characteristic->getValue();
        deserializeJson(value, stored.data(), stored.size());
        if (deserializeJson(current, ossm->getCurrentState())) return;
        for (JsonPairConst field : current.as<JsonObjectConst>())
            value[field.key()] = field.value();
        String payload;
        serializeJson(value, payload);
        characteristic->setValue(payload);
    }
    void onSubscribe(NimBLECharacteristic*, NimBLEConnInfo& connection,
                     uint16_t value) override {
        radBleServer.onStateSubscribe(connection.getConnHandle(), value);
    }
} stateCallbacks;

/** Handler class for server actions */
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        radHilBleConnected(true);
        ESP_LOGI(NIMBLE_TAG, "Client connected: %s",
                 connInfo.getAddress().toString().c_str());
        ESP_LOGI(NIMBLE_TAG, "Connection count: %d",
                 pServer->getConnectedCount());

        // Set BLE connection status to true
        if (ossm) {
            ossm->setBLEConnectionStatus(true);
        }
        radBleServer.onConnect(connInfo.getConnHandle());

        lostConnectionTime = 0;
    }

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo,
                      int reason) override {
        ESP_LOGI(NIMBLE_TAG, "Client disconnected: %s, reason: %d",
                 connInfo.getAddress().toString().c_str(), reason);
        ESP_LOGI(NIMBLE_TAG, "Connection count: %d",
                 pServer->getConnectedCount());

        const int remaining = pServer->getConnectedCount();
        radHilBleConnected(remaining != 0);
        bool stopMotion;
        {
            std::lock_guard<std::mutex> guard(motionOwnerLock);
            stopMotion = motionOwner.onDisconnect(connInfo.getConnHandle(),
                                                  remaining);
            if (stopMotion) streaming::cancelRemoteDrawnPattern();
        }

        // Set BLE connection status to false when no connections remain
        if (ossm && remaining == 0) {
            ossm->setBLEConnectionStatus(false);
        }
        // Stop when the last link went, or when the link that last drove
        // motion went while another central (RAD App, dashboard) stays.
        if (ossm && stopMotion) {
            if (remaining > 0)
                ESP_LOGW(NIMBLE_TAG,
                         "Motion owner disconnected (%d links remain); stopping",
                         remaining);
            ossm->ble_click("go:menu");
            // Capture current speed when connection is lost
            speedOnLostConnection = ossm->getSpeed();
            ESP_LOGI(NIMBLE_TAG, "Speed on disconnect: %d",
                     speedOnLostConnection.load());
            lostConnectionTime = millis();
        }
        radBleServer.onDisconnect(connInfo.getConnHandle());

        // Restart advertising when client disconnects
        if (remaining == 0) {
            ESP_LOGI(NIMBLE_TAG,
                     "No connections remaining, restarting advertising");
            restartAdvertisingWithCurrentName();
        }
    }

    void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override {
        ESP_LOGD(NIMBLE_TAG, "MTU changed to: %d for connection: %s", MTU,
                 connInfo.getAddress().toString().c_str());
    }
} serverCallbacks;

#ifdef PRETEND_TO_BE_FLESHY_THRUST_SYNC
class FTSCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic,
                 NimBLEConnInfo& connInfo) override {
        std::string value = pCharacteristic->getValue();

        // Expected format: [position, timeHigh, timeLow]
        // position: uint8 (0-180), convert to 100
        // time: uint16 big-endian (MSB first)
        if (value.length() >= 3) {
            uint8_t position = static_cast<uint8_t>(value[0]/1.8);
            uint16_t time = (static_cast<uint8_t>(value[1]) << 8) |
                            static_cast<uint8_t>(value[2]);

            ESP_LOGI("NIMBLE", "FTS Command - Position: %d, Time: %d ms", position, time);
            noteBleMotionCommand(connInfo.getConnHandle(), "stream:");
            targetQueue.push({position, time, std::chrono::steady_clock::now()},
                             true);

        } else {
            ESP_LOGW("NIMBLE", "FTS write - Invalid data length: %d bytes",
                     value.length());
        }
    }

    void onRead(NimBLECharacteristic* pCharacteristic,
                NimBLEConnInfo& connInfo) override {
        Serial.println("FTS read callback");
        std::string value = pCharacteristic->getValue();
        String ftsValue = String(value.c_str());
        pCharacteristic->setValue(ftsValue);
        // Print everything that comes to this
        Serial.print("FTS read: ");
        Serial.println(ftsValue);
        ESP_LOGD(NIMBLE_TAG, "FTS read: %s", ftsValue.c_str());
    }

    /** Peer subscribed to notifications/indications */
    void onSubscribe(NimBLECharacteristic* pCharacteristic,
                     NimBLEConnInfo& connInfo, uint16_t subValue) override {
        std::string str = "Client ID: ";
        str += connInfo.getConnHandle();
        str += " Address: ";
        str += connInfo.getAddress().toString();
        if (subValue == 0) {
            str += " Unsubscribed to ";
        } else if (subValue == 1) {
            str += " Subscribed to notifications for ";
        } else if (subValue == 2) {
            str += " Subscribed to indications for ";
        } else if (subValue == 3) {
            str += " Subscribed to notifications and indications for ";
        }
        str += std::string(pCharacteristic->getUUID());
        ESP_LOGV("NIMBLE", "%s", str.c_str());
    }

} ftsCallbacks;
#endif

// One step of the 2 s ease-out of the BLE speed after a motion link dropped
// (the first second waits). Ends by setting speed 0.
static void stepLostConnectionRamp() {
    const long lost = lostConnectionTime.load();
    if (lost <= 0 || ossm == nullptr) return;
    const int fromSpeed = speedOnLostConnection.load();
    // Skip ramp-down if speed was already zero when connection was lost
    if (fromSpeed <= 0) {
        lostConnectionTime = 0;
        return;
    }

    const unsigned long elapsed = millis() - (unsigned long)lost;
    // Wait 1 second before starting easing
    if (elapsed < 1000) return;

    if (elapsed > 1000 + RAMP_DURATION_MS) {
        ESP_LOGI(NIMBLE_TAG, "Speed ramp duration exceeded, setting speed to 0");
        lostConnectionTime = 0;
        speedOnLostConnection = 0;
        ossm->ble_click("set:speed:0");
        return;
    }

    // Calculate easing factor (0 to 1) over ramp duration
    const double progress =
        constrain((elapsed - 1000) / (double)RAMP_DURATION_MS, 0.0, 1.0);
    const double t = easeInOutSine(progress);

    // Ramp from current speed to zero
    const int targetSpeed = (int)(fromSpeed * (1.0 - t));
    ESP_LOGI(NIMBLE_TAG, "Target speed: %d (from %d, progress: %.2f)",
             targetSpeed, fromSpeed, progress);
    ossm->ble_click("set:speed:" + String(targetSpeed));

    // Stop processing when easing is complete
    if (t >= 1) lostConnectionTime = 0;
}

void nimbleLoop(void* pvParameters) {
    NimBLEServer* pServer = (NimBLEServer*)pvParameters;
    /** Loop here and send notifications to connected peers */

    String lastState = "";
    int lastConnCount = 0;
    int lastMessageTime = 0;
    uint32_t lastRampStepMs = 0;
    while (true) {
        // Ramp the BLE speed down after the link that drove motion dropped.
        // This also runs while other centrals stay connected (see
        // ServerCallbacks::onDisconnect); a new motion command cancels it.
        const bool ramping = lostConnectionTime.load() > 0;
        if (ramping && millis() - lastRampStepMs >= 50) {
            lastRampStepMs = millis();
            stepLostConnectionRamp();
        }

        // Check if we should be advertising (no connections)
        if (pServer->getConnectedCount() == 0) {
            // If not advertising and no connections, restart advertising
            if (!pServer->getAdvertising()) {
                ESP_LOGI(NIMBLE_TAG,
                         "No connections and not advertising, restarting "
                         "advertising");
                restartAdvertisingWithCurrentName();
            }

            vTaskDelay(pdMS_TO_TICKS(lostConnectionTime.load() > 0 ? 50 : 200));
            continue;
        }

        int currentConnCount = pServer->getConnectedCount();

        // Clear last state when connection count changes
        if (currentConnCount != lastConnCount) {
            lastState = "";
            lastConnCount = currentConnCount;
        }

        NimBLEService* pSvc = pServer->getServiceByUUID(SERVICE_UUID);
        if (!pSvc) {
            lastState = "";
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        NimBLECharacteristic* pChr =
            pSvc->getCharacteristic(CHARACTERISTIC_STATE_UUID);
        if (!pChr) {
            lastState = "";
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // mannage message queue
        String cmd;
        while (messageQueue.pop(cmd)) {
            ossm->ble_click(cmd);

            // Trigger LED communication pulse for command processing
            pulseForCommunication();

            vTaskDelay(1);
        }

        int currentTime = millis();
        String fingerprint = ossm->getStateFingerprint();
        bool stateChanged = fingerprint != lastState;
        bool timeElapsed = (currentTime - lastMessageTime) > 1000;

        if (!stateChanged && !timeElapsed) {
            vTaskDelay(1);
            continue;
        }
        lastMessageTime = currentTime;
        lastState = fingerprint;

        String currentState = ossm->getCurrentState();
        if (stateChanged) {
            ESP_LOGD(NIMBLE_TAG, "State changed to: %s", currentState.c_str());
            radBleServer.publishStateNotification(currentState);
        }

        // Trigger LED communication pulse for state update
        pulseForCommunication();

        lastState = fingerprint;
        vTaskDelay(1);
    }
}

void initNimble() {
    /** Initialize NimBLE and set the device name */
    NimBLEDevice::init(getDeviceName());
    NimBLEDevice::setMTU(512);

    NimBLEDevice::setSecurityAuth(false, false, false);
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(&serverCallbacks);

    // Create Service
    NimBLEService* pService = pServer->createService(SERVICE_UUID);

    pCommandCharacteristic =
        initCommandCharacteristic(pService, NimBLEUUID(CHARACTERISTIC_UUID));

    pSpeedKnobConfigCharacteristic = initSpeedKnobConfigCharacteristic(
        pService, NimBLEUUID(CHARACTERISTIC_SPEED_KNOB_CONFIG_UUID));

    pLatencyCompensationConfigCharacteristic = initLatencyCompensationConfigCharacteristic(
        pService, NimBLEUUID(CHARACTERISTIC_LATENCY_COMPENSATION_CONFIG_UUID));

    initWiFiConfigCharacteristic(pService,
                                 NimBLEUUID(CHARACTERISTIC_WIFI_CONFIG_UUID));

    initRenameConfigCharacteristic(pService,
                                 NimBLEUUID(CHARACTERISTIC_RENAME_CONFIG_UUID));

    pStateCharacteristic = initStateCharacteristic(
        pService, NimBLEUUID(CHARACTERISTIC_STATE_UUID));
    pStateCharacteristic->setCallbacks(&stateCallbacks);

    initPatternsCharacteristic(pService,
                               NimBLEUUID(CHARACTERISTIC_PATTERNS_UUID));
    initPatternDataCharacteristic(
        pService, NimBLEUUID(CHARACTERISTIC_GET_PATTERN_DATA_UUID));

    // GPIO write/read characteristic
    initGPIOCharacteristic(pService, NimBLEUUID(CHARACTERISTIC_GPIO_UUID));

    // Pairing characteristic — BLE one-click pairing from the dashboard
    initPairingCharacteristic(pService,
                              NimBLEUUID(CHARACTERISTIC_PAIRING_UUID));

    // Add Device Information Service
    NimBLEService* pDeviceInfoService =
        pServer->createService(DEVICE_INFO_SERVICE_UUID);

    // Add Manufacturer Name characteristic
    NimBLECharacteristic* pManufacturerName =
        pDeviceInfoService->createCharacteristic(MANUFACTURER_NAME_UUID,
                                                 NIMBLE_PROPERTY::READ);
    static const char manufacturer[] PROGMEM = "Research And Desire";
    pManufacturerName->setValue(String(FPSTR(manufacturer)));

    // Add System ID characteristic
    NimBLECharacteristic* pSystemId = pDeviceInfoService->createCharacteristic(
        SYSTEM_ID_UUID, NIMBLE_PROPERTY::READ);
    uint8_t systemId[] = {0x88, 0x1A, 0x14, 0xFF, 0xFE, 0x34, 0x29, 0x63};
    pSystemId->setValue(systemId, 8);

    if (!initRadBle(pServer)) {
        ESP_LOGE(NIMBLE_TAG, "Universal RAD BLE service failed to initialize");
    }

    // NimBLE 2 starts every registered service when advertising begins.

#ifdef PRETEND_TO_BE_FLESHY_THRUST_SYNC
    // if this is true, then we'll start a service for the FTS
    NimBLEService* pFTS = pServer->createService(
        NimBLEUUID("0000ffe0-0000-1000-8000-00805f9b34fb"));
    NimBLECharacteristic* pFTSCharacteristic = pFTS->createCharacteristic(
        NimBLEUUID("0000ffe1-0000-1000-8000-00805f9b34fb"),
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE |
            NIMBLE_PROPERTY::WRITE_NR);
    pFTSCharacteristic->setCallbacks(&ftsCallbacks);
    pFTS->start();
#endif

    // // Configure advertising parameters for better reliability
    // pAdvertising->setMinInterval(0x20);  // 20ms minimum interval
    // pAdvertising->setMaxInterval(0x40);  // 40ms maximum interval

    restartAdvertisingWithCurrentName();

    xTaskCreatePinnedToCore(
        nimbleLoop, "nimbleLoop", 5 * configMINIMAL_STACK_SIZE, pServer,
        configMAX_PRIORITIES - 1, nullptr, Tasks::stepperCore);
}
