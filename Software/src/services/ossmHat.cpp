#include "ossmHat.h"

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_log.h>

#include "constants/Pins.h"
#include "ossmHatProtocol.hpp"

namespace {

constexpr const char *tag = "OSSM_HAT";
constexpr uint32_t bauds[] = {115200, 19200, 9600, 38400};
constexpr size_t baudCount = sizeof(bauds) / sizeof(bauds[0]);
constexpr uint32_t listenSlotMs = 700;
constexpr uint32_t quietBeforeProbeMs = 1000;
constexpr uint32_t probeSlotMs = 1500;
constexpr uint32_t pollIntervalMs = 20;
constexpr uint32_t reconnectAfterMs = 2000;

portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
OssmHatStatus publishedStatus;
OssmHatTelemetry publishedSample;
bool initialized = false;

bool startUart(uint32_t baud) {
    Serial2.end();
    pinMode(Pins::Hat::direction, OUTPUT);
    digitalWrite(Pins::Hat::direction, LOW);
    Serial2.setRxBufferSize(512);
    Serial2.begin(baud, SERIAL_8N1, Pins::Hat::rx, Pins::Hat::tx);
    // Deliver bytes during long 9600-baud replies as well as at RX idle;
    // otherwise a whole slow reply could arrive after the resync timeout.
    Serial2.setRxFIFOFull(1);
    Serial2.setRxTimeout(2);
    const bool pins = Serial2.setPins(Pins::Hat::rx, Pins::Hat::tx, -1,
                                      Pins::Hat::direction);
    const bool mode = Serial2.setMode(UART_MODE_RS485_HALF_DUPLEX);
    gpio_pullup_en(static_cast<gpio_num_t>(Pins::Hat::rx));
    if (!pins || !mode) {
        Serial2.end();
        digitalWrite(Pins::Hat::direction, LOW);
        ESP_LOGE(tag, "UART2 RS485 setup failed");
        return false;
    }
    return true;
}

void reader(void *) {
    ossm_hat::Decoder decoder;
    OssmHatStatus status;
#ifdef OSSM_HAT_LISTEN_ONLY
    // Required on a shared safety bus: RTU has no multi-master arbitration.
    // This mode cannot transmit, even before the PC's first request is heard.
    status.passive = true;
#endif
    size_t baudIndex = 0;
    bool listeningSweep = true;
    bool lockedBaud = false;
    bool attempted = false;
    bool awaiting = false;
    uint32_t slotAt = millis();
    uint32_t lastByteAt = slotAt;
    uint32_t lastFrameAt = slotAt;
    uint32_t lastPollAt = slotAt;
    uint32_t requestAt = slotAt;
    status.baudRate = bauds[baudIndex];
    bool uartReady = startUart(status.baudRate);

    auto advanceBaud = [&]() {
        baudIndex = (baudIndex + 1) % baudCount;
        if (baudIndex == 0) {
            if (listeningSweep) {
                listeningSweep = false;
                if (status.passive) status.discoveryComplete = true;
            } else {
                status.discoveryComplete = true;
                listeningSweep = true;
            }
        }
        status.baudRate = bauds[baudIndex];
        decoder.reset();
        uartReady = startUart(status.baudRate);
        slotAt = millis();
        lastByteAt = slotAt;
        awaiting = false;
        attempted = false;
    };

    ESP_LOGI(tag, "Discovering AIM telemetry on TX=%d RX=%d DE=%d%s",
             Pins::Hat::tx, Pins::Hat::rx, Pins::Hat::direction,
             status.passive ? " (receive only)" : "");

    for (;;) {
        uint32_t now = millis();
        // A truncated/noisy frame cannot poison the next transaction forever.
        // The AIM's normal request/reply turnaround is far below 20 ms.
        if (static_cast<uint32_t>(now - lastByteAt) > 20 &&
            !awaiting && Serial2.available()) {
            decoder.reset();
        }
        while (uartReady && Serial2.available()) {
            const int byte = Serial2.read();
            if (byte < 0) break;
            now = millis();
            lastByteAt = now;
            const uint8_t events = decoder.push(static_cast<uint8_t>(byte), now);
            if (events & ossm_hat::Decoder::ExternalRequest) {
                if (!status.passive) {
                    ESP_LOGI(tag, "External RS485 master detected; receive only until reboot");
                }
                status.passive = true;
                lockedBaud = true;
                lastFrameAt = now;
            }
            if (events & ossm_hat::Decoder::Telemetry) {
                if (!status.detected) {
                    ESP_LOGI(tag, "HAT AIM telemetry detected at %lu baud (%s)",
                             static_cast<unsigned long>(status.baudRate),
                             status.passive ? "passive" : "polled");
                }
                status.detected = true;
                status.discoveryComplete = true;
                lockedBaud = true;
                lastFrameAt = now;
                awaiting = false;
            }
        }

        now = millis();
        if (awaiting && static_cast<uint32_t>(now - requestAt) >=
                            ossm_hat::replyTimeoutMs) {
            awaiting = false;
            decoder.reset();
        }

        if (lockedBaud && static_cast<uint32_t>(now - lastFrameAt) >=
                              reconnectAfterMs) {
            // Keep a detected HAT latched across disconnects. Consumers see
            // stale telemetry, while the background reader searches again.
            lockedBaud = false;
            listeningSweep = true;
            baudIndex = baudCount - 1;
            advanceBaud();
        } else if (!lockedBaud) {
            const uint32_t slotAge = now - slotAt;
            if ((listeningSweep || status.passive) && slotAge >= listenSlotMs) {
                advanceBaud();
            } else if (!listeningSweep && !status.passive) {
                if (slotAge >= probeSlotMs ||
                    (attempted && !awaiting)) {
                    advanceBaud();
                }
            }
        }

        now = millis();
        // Read-only standalone polling is allowed only after a complete
        // receive-only baud sweep and a long quiet bus. Once any external
        // master is observed this branch is permanently unavailable.
        const bool standalonePoll = uartReady && !status.passive &&
            !listeningSweep && !awaiting &&
            (lockedBaud || !attempted) &&
            static_cast<uint32_t>(now - lastPollAt) >= pollIntervalMs &&
            static_cast<uint32_t>(now - lastByteAt) >=
                (lockedBaud ? 3U : quietBeforeProbeMs);
        if (standalonePoll) {
            uint8_t request[8];
            const size_t length = AimFrame::buildRead(
                request, ossm_hat::motorId, ossm_hat::telemetryStart,
                ossm_hat::telemetryCount);
            decoder.reset();
            decoder.expectRead(ossm_hat::motorId, ossm_hat::telemetryStart,
                               ossm_hat::telemetryCount, now);
            awaiting = true;
            attempted = true;
            requestAt = now;
            lastPollAt = now;
            Serial2.write(request, length);
            Serial2.flush();
        }

        status.fresh = ossm_hat::fresh(decoder.sample(), millis());
        status.ageMs = decoder.sample().sequence
                           ? static_cast<uint32_t>(millis() - decoder.sample().updatedAtMs)
                           : UINT32_MAX;
        status.validFrames = decoder.validFrames();
        status.crcErrors = decoder.crcErrors();
        portENTER_CRITICAL(&stateMux);
        publishedSample = decoder.sample();
        publishedStatus = status;
        portEXIT_CRITICAL(&stateMux);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

}  // namespace

void initOssmHat() {
    if (initialized) return;
    if (Pins::Hat::tx == Pins::Remote::displayClock ||
        Pins::Hat::rx == Pins::Remote::displayClock) {
        ESP_LOGW(tag, "HAT UART conflicts with this development display pinout");
        portENTER_CRITICAL(&stateMux);
        publishedStatus.discoveryComplete = true;
        portEXIT_CRITICAL(&stateMux);
        return;
    }
    // Hold the transmitter off from the first initialization instruction.
    pinMode(Pins::Hat::direction, OUTPUT);
    digitalWrite(Pins::Hat::direction, LOW);
    initialized = true;
    if (xTaskCreatePinnedToCore(reader, "ossmHat", 4096, nullptr, 2, nullptr, 1) != pdPASS) {
        ESP_LOGE(tag, "Cannot create HAT reader task");
        portENTER_CRITICAL(&stateMux);
        publishedStatus.discoveryComplete = true;
        portEXIT_CRITICAL(&stateMux);
    }
}

bool ossmHatDetected() {
    portENTER_CRITICAL(&stateMux);
    const bool detected = publishedStatus.detected;
    portEXIT_CRITICAL(&stateMux);
    return detected;
}

bool readOssmHatTelemetry(OssmHatTelemetry &out) {
    portENTER_CRITICAL(&stateMux);
    out = publishedSample;
    portEXIT_CRITICAL(&stateMux);
    return ossm_hat::fresh(out, millis());
}

OssmHatStatus getOssmHatStatus() {
    portENTER_CRITICAL(&stateMux);
    OssmHatStatus status = publishedStatus;
    const OssmHatTelemetry sample = publishedSample;
    portEXIT_CRITICAL(&stateMux);
    status.fresh = ossm_hat::fresh(sample, millis());
    status.ageMs = sample.sequence
                       ? static_cast<uint32_t>(millis() - sample.updatedAtMs)
                       : UINT32_MAX;
    return status;
}

bool ossmHatOwnsPin(int pin) {
    return initialized && (pin == Pins::Hat::tx || pin == Pins::Hat::rx ||
                           pin == Pins::Hat::direction);
}
