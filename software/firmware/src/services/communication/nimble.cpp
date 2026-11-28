#include "nimble.h"

#include <ArduinoJson.h>
#include <constants/LogTags.h>
#include <services/board.h>
#include <services/tasks.h>

#include <mutex>
#include <queue>

#include "command.hpp"
#include "command/commands.hpp"
#include "config.hpp"
#include "gpio.hpp"
#include "motion_link.h"
#include "disconnect_speed.h"
#include "ossm/state/ble.h"
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

// Host task, queued legacy dispatch and RAD BLE actions share this ownership boundary.
static std::mutex motionOwnerLock;
static motion_link::Owner motionOwner;
static motion_link::DisconnectSpeed disconnectSpeed;

static bool clearQueuedSpeeds() {
    bool lastWasZero = false;
    messageQueue.removeIf([&](const String &command) {
        if (!command.startsWith("set:speed:")) return false;
        lastWasZero = commandFromString(command).value == 0;
        return true;
    });
    return lastWasZero;
}

static void noteCommandLocked(uint16_t connection, const std::string &command) {
    if (!motionOwner.takeCommand(connection, command, streaming::cancelDrawnPattern)) return;
    disconnectSpeed.cancel();
}

void noteBleMotionCommand(uint16_t connection, const std::string &command) {
    std::lock_guard<std::mutex> guard(motionOwnerLock);
    if (motionOwner.owner() != connection && motion_link::isMotionCommand(command)) clearQueuedSpeeds();
    noteCommandLocked(connection, command);
}

bool enqueueLegacyBleCommand(uint16_t connection, const std::string &command) {
    std::lock_guard<std::mutex> guard(motionOwnerLock);
    // A rejected overflow does not take ownership or alter queued speed.
    if (command != "go:menu" && messageQueue.size() >= MessageQueue::capacity) return false;
    if (motionOwner.owner() != connection && motion_link::isMotionCommand(command)) clearQueuedSpeeds();
    const String value(command.c_str());
    if (!messageQueue.push(value)) {
        if (command != "go:menu") return false;
        messageQueue.replaceAll(value);
    }
    noteCommandLocked(connection, command);
    return true;
}

static bool runNextLegacyCommand() {
    String command;
    {
        std::lock_guard<std::mutex> guard(motionOwnerLock);
        if (!messageQueue.pop(command)) return false;
        if (command.startsWith("set:speed:")) {
            // Same validated legacy parser and setter as OSSM::ble_click;
            // never run an SML action while holding this ownership lock.
            const auto parsed = commandFromString(command);
            if (parsed.command == Commands::setSpeed) writeBleSpeed(parsed.value, true);
            return true;
        }
    }
    ossm->ble_click(command);
    return true;
}

streaming::PatternStart startBlePattern(uint16_t connection, const char *samples,
                                       size_t length, uint32_t periodMs, drawn_pattern::SampleFormat format,
                                       std::unique_ptr<drawn_pattern::Modifiers> modifiers) {
    std::lock_guard<std::mutex> guard(motionOwnerLock);
    // A request already in the server task may race a disconnect.
    if (!pServer || connection == motion_link::kNoConnection ||
        ble_gap_conn_find(connection, nullptr) != 0)
        return streaming::PatternStart::NotReady;
    const auto result = streaming::startOwnedDrawnPattern(samples, length, periodMs,
                                  getMotorSafetyStatus().controlRevision, false, format, std::move(modifiers));
    if (result == streaming::PatternStart::Accepted) {
        motionOwner.onMotionCommand(connection);
        clearQueuedSpeeds();
        disconnectSpeed.cancel();
    }
    return result;
}

bool setLeasedBleSpeed(uint16_t connection, float value, bool selectBleSource) {
    std::lock_guard<std::mutex> guard(motionOwnerLock);
    if (!pServer || connection == motion_link::kNoConnection ||
        ble_gap_conn_find(connection, nullptr) != 0) return false;
    if (!streaming::applyLeasedBleSpeed(motionOwner, connection, value, selectBleSource)) return false;
    clearQueuedSpeeds();
    disconnectSpeed.cancel();
    return true;
}

bool setLeasedBleSensation(uint16_t connection, uint8_t value) {
    std::lock_guard<std::mutex> guard(motionOwnerLock);
    if (!pServer || connection == motion_link::kNoConnection ||
        ble_gap_conn_find(connection, nullptr) != 0) return false;
    const bool changesOwner = motionOwner.owner() != connection;
    if (!streaming::applyLeasedBleSensation(motionOwner, connection, value)) return false;
    if (changesOwner) clearQueuedSpeeds();
    disconnectSpeed.cancel();
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
        // Reconnection by itself does not claim motion or end a shutdown.
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
            if (stopMotion) {
                const bool pendingZero = clearQueuedSpeeds();
                if (pendingZero) writeBleSpeed(0, true);
                streaming::cancelRemoteDrawnPattern();
                if (ossm && disconnectSpeed.start(millis(), ossm->getSpeed(), settings.speedBLE)) {
                    writeBleSpeed(0, true);
                }
            }
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
    std::lock_guard<std::mutex> guard(motionOwnerLock);
    const auto next = disconnectSpeed.next(millis(), settings.speedBLE);
    if (next) writeBleSpeed(*next, true);
}

void nimbleLoop(void* pvParameters) {
    NimBLEServer* pServer = (NimBLEServer*)pvParameters;
    /** Loop here and send notifications to connected peers */

    String lastState, fingerprint;
    int lastConnCount = 0;
    uint32_t lastMessageTime = 0, lastStatusCheck = 0;
    uint32_t lastRampStepMs = 0;
    while (true) {
        // Ramp the BLE speed down after the link that drove motion dropped.
        // This also runs while other centrals stay connected (see
        // ServerCallbacks::onDisconnect); a new motion command cancels it.
        const bool ramping = disconnectSpeed.active();
        if (ramping && millis() - lastRampStepMs >= 50) {
            lastRampStepMs = millis();
            stepLostConnectionRamp();
        }

        // Check if we should be advertising (no connections)
        const int currentConnCount = pServer->getConnectedCount();
        if (currentConnCount == 0) {
            lastConnCount = 0;
            // If not advertising and no connections, restart advertising
            if (!pServer->getAdvertising()) {
                ESP_LOGI(NIMBLE_TAG,
                         "No connections and not advertising, restarting "
                         "advertising");
                restartAdvertisingWithCurrentName();
            }

            vTaskDelay(pdMS_TO_TICKS(disconnectSpeed.active() ? 50 : 200));
            continue;
        }

        // Clear last state when connection count changes
        if (currentConnCount != lastConnCount) {
            lastState = "";
            lastConnCount = currentConnCount;
        }

        // Cached when the service is created, before this task is started.
        if (!pStateCharacteristic) {
            lastState = "";
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Commands and disconnect handling retain their per-tick service.
        bool handledCommand = false;
        while (runNextLegacyCommand()) {
            handledCommand = true;

            // Trigger LED communication pulse for command processing
            pulseForCommunication();

            vTaskDelay(1);
        }

        const uint32_t currentTime = millis();
        if (!handledCommand && lastState.length() && currentTime - lastStatusCheck < 20) {
            vTaskDelay(1);
            continue;
        }
        lastStatusCheck = currentTime;
        ossm->writeStateFingerprint(fingerprint);
        const bool stateChanged = fingerprint != lastState;
        if (stateChanged) {
            lastState = fingerprint;
            String currentState = ossm->getCurrentState();
            ESP_LOGD(NIMBLE_TAG, "State changed to: %s", currentState.c_str());
            radBleServer.publishStateNotification(currentState);
        }
        if (stateChanged || currentTime - lastMessageTime > 1000) {
            lastMessageTime = currentTime;
            pulseForCommunication();
        }
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
