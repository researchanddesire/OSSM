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
OssmHatControlSnapshot publishedControls;
OssmHatStopStatus publishedStop;
OssmHatExternalControlStatus publishedExternalControl;
bool pendingStop = false;
bool stopInFlight = false;
bool pendingExternalControl = false;
bool externalControlInFlight = false;
bool initialized = false;

bool handbackConditions(bool requireControlSnapshot) {
    if (digitalRead(Pins::Driver::motorEnablePin) != HIGH) return false;
    portENTER_CRITICAL(&stateMux);
    const auto sample = publishedSample;
    const auto controls = publishedControls;
    const bool allowed = initialized && !publishedStatus.passive &&
                         !pendingStop && !publishedStop.requested;
    portEXIT_CRITICAL(&stateMux);
    const uint32_t now = millis();
    return allowed && ossm_hat::fresh(sample, now) &&
           static_cast<uint32_t>(now - sample.updatedAtMs) <= 100 &&
           sample.currentAmps >= 0 && sample.currentAmps < 0.5f &&
           (sample.alarm == 0 || sample.alarm == 0x10) &&
           (!requireControlSnapshot || ossm_hat::controlsFresh(controls, sample, now));
}

// Used only by the UART owner for a latched emergency stop. Ordinary
// telemetry never writes a register. All exchanges are bounded and validated;
// GPIO ENA/pulses must already be disabled by the independent guard.
bool stopExchange(const uint8_t *request, size_t length, uint8_t function,
                  size_t replyLength, ossm_hat::Decoder &observer,
                  bool &externalMaster, uint16_t *controlWords = nullptr,
                  bool reviewedHandback = false, bool *preempted = nullptr) {
    const auto interrupted = [&]() {
        if (!reviewedHandback || handbackConditions(false)) return false;
        if (preempted) *preempted = true;
        return true;
    };
    uint32_t quietAt = micros();
    const uint32_t quietStarted = millis();
    while (static_cast<uint32_t>(micros() - quietAt) < 3000) {
        if (interrupted()) return false;
        while (Serial2.available()) {
            const int byte = Serial2.read();
            if (byte >= 0 &&
                (observer.push(static_cast<uint8_t>(byte), millis()) &
                 ossm_hat::Decoder::ExternalRequest)) {
                // A valid request outside an expected response means another
                // master (or an ambiguous late echo). GPIO is already off;
                // refuse to compete for the bus, even for a secondary stop.
                externalMaster = true;
                return false;
            }
            quietAt = micros();
        }
        if (static_cast<uint32_t>(millis() - quietStarted) >= 50) return false;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (interrupted()) return false;
    Serial2.write(request, length);
    Serial2.flush();
    uint8_t response[80] = {};
    size_t received = 0;
    const uint32_t started = millis();
    while (static_cast<uint32_t>(millis() - started) < ossm_hat::replyTimeoutMs) {
        if (interrupted()) return false;
        while (Serial2.available() && received < sizeof(response)) {
            const int byte = Serial2.read();
            if (byte < 0) break;
            response[received++] = static_cast<uint8_t>(byte);
            const uint8_t observed = observer.push(static_cast<uint8_t>(byte), millis());
            const auto partial = AimFrame::parseReply(
                response, received, ossm_hat::motorId, function, replyLength);
            // FC06 replies echo their request. Only the exact CRC-validated
            // echo currently expected by this exchange is our own reply.
            const bool ownEcho = function == AimFrame::fcWriteSingle &&
                partial.status == AimFrame::Status::Ok &&
                std::memcmp(response + partial.offset, request, 6) == 0;
            if ((observed & ossm_hat::Decoder::ExternalRequest) && !ownEcho) {
                externalMaster = true;
                return false;
            }
            if (partial.status == AimFrame::Status::Ok) break;
        }
        const auto parsed = AimFrame::parseReply(
            response, received, ossm_hat::motorId, function, replyLength);
        if (parsed.status == AimFrame::Status::Ok) {
            const uint8_t *frame = response + parsed.offset;
            if (function == AimFrame::fcWriteSingle)
                return std::memcmp(frame, request, 6) == 0;
            if (controlWords) {
                controlWords[0] = AimFrame::be16(frame + 3);
                controlWords[1] = AimFrame::be16(frame + 5);
            }
            return true;
        }
        if ((parsed.status != AimFrame::Status::Incomplete && received >= 8) ||
            received == sizeof(response))
            return false;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

OssmHatStopStatus stopAndVerify(OssmHatStopStatus result, bool &externalMaster) {
    // Keep ownership parsing across all exchanges so a fragmented external
    // request cannot disappear at a transaction boundary.
    ossm_hat::Decoder observer;
    uint8_t request[8];
    AimFrame::buildWriteSingle(request, ossm_hat::motorId, 0x00, 1);
    stopExchange(request, sizeof(request), AimFrame::fcWriteSingle, 8,
                  observer, externalMaster);
    // Attempt output disable even if the first acknowledgement was lost.
    if (!externalMaster) {
        AimFrame::buildWriteSingle(request, ossm_hat::motorId, 0x01, 0);
        stopExchange(request, sizeof(request), AimFrame::fcWriteSingle, 8,
                      observer, externalMaster);
    }
    uint16_t words[2] = {UINT16_MAX, UINT16_MAX};
    bool readBack = false;
    if (!externalMaster) {
        AimFrame::buildRead(request, ossm_hat::motorId, 0, 2);
        readBack = stopExchange(request, sizeof(request),
                                AimFrame::fcReadHolding, 9, observer,
                                externalMaster, words);
    }
    result.modbusEnable = words[0];
    result.driveOutputEnable = words[1];
    // Register1 also carries status bits. As in the reviewed host takeover,
    // bit0 is the serial-mode output enable; unrelated status bits may remain.
    result.verified = readBack && words[0] == 1 && (words[1] & 1U) == 0;
    result.passiveBlocked = externalMaster;
    result.complete = true;
    result.completedAtMs = millis();
    ESP_LOGE(tag, "Guard RS485 output disable %s (control=%u output=%u)",
             result.verified ? "verified" : "UNVERIFIED", words[0], words[1]);
    return result;
}

OssmHatExternalControlStatus handbackAndVerify(
    OssmHatExternalControlStatus result, bool &externalMaster) {
    ossm_hat::Decoder observer;
    uint8_t request[8];
    AimFrame::buildWriteSingle(request, ossm_hat::motorId, 0x00, 0);
    stopExchange(request, sizeof(request), AimFrame::fcWriteSingle, 8,
                  observer, externalMaster, nullptr, true, &result.preempted);
    uint16_t words[2] = {UINT16_MAX, UINT16_MAX};
    bool readBack = false;
    if (!externalMaster && !result.preempted && handbackConditions(false)) {
        AimFrame::buildRead(request, ossm_hat::motorId, 0, 2);
        readBack = stopExchange(request, sizeof(request), AimFrame::fcReadHolding,
                                9, observer, externalMaster, words, true,
                                &result.preempted);
    }
    result.modbusEnable = words[0];
    result.outputEnable = words[1];
    result.verified = readBack && words[0] == 0 && handbackConditions(false);
    result.passiveBlocked = externalMaster;
    result.complete = true;
    result.completedAtMs = millis();
    ESP_LOGW(tag, "Reviewed STEP/DIR handback %s (control=%u output=%u); ENA remains disabled",
             result.verified ? "verified" : "UNVERIFIED", words[0], words[1]);
    return result;
}

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
    ossm_hat::SampleRate sampleRate;
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
    uint32_t lastControlPollAt = 0;
    bool controlsPolled = false;
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
        controlsPolled = false;
    };

    ESP_LOGI(tag, "Discovering AIM telemetry on TX=%d RX=%d DE=%d%s",
             Pins::Hat::tx, Pins::Hat::rx, Pins::Hat::direction,
             status.passive ? " (receive only)" : "");

    for (;;) {
        uint32_t now = millis();
        portENTER_CRITICAL(&stateMux);
        const bool performStop = pendingStop;
        const OssmHatStopStatus stopRequest = publishedStop;
        pendingStop = false;
        if (performStop) stopInFlight = true;
        portEXIT_CRITICAL(&stateMux);
        if (performStop) {
            decoder.invalidateControls();
            OssmHatStopStatus stopped = stopRequest;
            if (status.passive || !uartReady || !status.detected) {
                stopped.passiveBlocked = status.passive;
                stopped.complete = true;
                stopped.completedAtMs = millis();
                ESP_LOGE(tag, "Guard RS485 stop unavailable%s; GPIO stop must remain latched",
                         status.passive ? " on passive/shared bus" : "");
            } else {
                bool externalMaster = false;
                stopped = stopAndVerify(stopped, externalMaster);
                if (externalMaster) {
                    status.passive = true;
                    ESP_LOGE(tag, "External/ambiguous RS485 master during stop; receive only");
                }
            }
            if (stopped.modbusEnable != UINT16_MAX &&
                stopped.driveOutputEnable != UINT16_MAX)
                decoder.acceptControlReadback(stopped.modbusEnable,
                                              stopped.driveOutputEnable,
                                              stopped.completedAtMs);
            portENTER_CRITICAL(&stateMux);
            publishedStop = stopped;
            publishedControls = decoder.controls();
            stopInFlight = false;
            portEXIT_CRITICAL(&stateMux);
            decoder.reset();
            awaiting = false;
            lastByteAt = millis();
        }

        portENTER_CRITICAL(&stateMux);
        const bool performHandback = pendingExternalControl &&
            !pendingStop && !publishedStop.requested;
        const auto handbackRequest = publishedExternalControl;
        if (performHandback) {
            pendingExternalControl = false;
            externalControlInFlight = true;
        }
        portEXIT_CRITICAL(&stateMux);
        if (performHandback) {
            auto handback = handbackRequest;
            if (!uartReady || status.passive || !handbackConditions(true)) {
                handback.complete = true;
                handback.preempted = true;
                handback.passiveBlocked = status.passive;
                handback.completedAtMs = millis();
            } else {
                decoder.invalidateControls();
                portENTER_CRITICAL(&stateMux);
                publishedControls.valid = false;
                portEXIT_CRITICAL(&stateMux);
                bool externalMaster = false;
                handback = handbackAndVerify(handback, externalMaster);
                if (externalMaster) status.passive = true;
                if (handback.modbusEnable != UINT16_MAX &&
                    handback.outputEnable != UINT16_MAX)
                    decoder.acceptControlReadback(handback.modbusEnable,
                                                  handback.outputEnable,
                                                  handback.completedAtMs);
            }
            portENTER_CRITICAL(&stateMux);
            if (publishedStop.requested) {
                handback.verified = false;
                handback.preempted = true;
                decoder.invalidateControls();
            }
            publishedExternalControl = handback;
            publishedControls = decoder.controls();
            externalControlInFlight = false;
            portEXIT_CRITICAL(&stateMux);
            decoder.reset();
            awaiting = false;
            lastByteAt = millis();
            // An emergency queued during handback must run before another
            // ordinary poll or owner command can be issued.
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
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
                sampleRate.accepted(now);
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
            const bool includeControls = !controlsPolled ||
                static_cast<uint32_t>(now - lastControlPollAt) >= 1000;
            const uint16_t firstRegister = includeControls ? 0 : ossm_hat::telemetryStart;
            const uint16_t registerCount = includeControls ? 26 : ossm_hat::telemetryCount;
            const size_t length = AimFrame::buildRead(
                request, ossm_hat::motorId, firstRegister, registerCount);
            decoder.reset();
            decoder.expectRead(ossm_hat::motorId, firstRegister, registerCount, now);
            if (includeControls) {
                controlsPolled = true;
                lastControlPollAt = now;
            }
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
        sampleRate.read(millis(), status.sampleRateHz, status.sampleWindowMs,
                        status.sampleCount);
        status.startupReady = status.fresh && status.sampleRateHz >= 30.0f &&
                              status.sampleWindowMs >= 500;
        portENTER_CRITICAL(&stateMux);
        publishedSample = decoder.sample();
        publishedControls = decoder.controls();
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
    if (xTaskCreatePinnedToCore(reader, "ossmHat", 4096, nullptr,
                              configMAX_PRIORITIES - 3, nullptr, 1) != pdPASS) {
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
    status.startupReady = status.fresh && status.sampleRateHz >= 30.0f &&
                          status.sampleWindowMs >= 500;
    return status;
}

bool readOssmHatControlSnapshot(OssmHatControlSnapshot &out) {
    portENTER_CRITICAL(&stateMux);
    out = publishedControls;
    const OssmHatTelemetry sample = publishedSample;
    portEXIT_CRITICAL(&stateMux);
    const uint32_t now = millis();
    out.valid = ossm_hat::controlsFresh(out, sample, now);
    return out.valid;
}

void requestOssmHatEmergencyStop() {
    OssmHatStopStatus requested;
    requested.requested = true;
    requested.requestedAtMs = millis();
    portENTER_CRITICAL(&stateMux);
    // Until an explicit reviewed clear, repeated high-rate enforcement must
    // neither queue another exchange nor erase a verified completion result.
    if (publishedStop.requested) {
        portEXIT_CRITICAL(&stateMux);
        return;
    }
    publishedStop = requested;
    publishedControls.valid = false;
    pendingStop = true;
    if (pendingExternalControl) {
        pendingExternalControl = false;
        publishedExternalControl.complete = true;
        publishedExternalControl.verified = false;
        publishedExternalControl.preempted = true;
        publishedExternalControl.completedAtMs = requested.requestedAtMs;
    }
    portEXIT_CRITICAL(&stateMux);
}

OssmHatStopStatus getOssmHatEmergencyStopStatus() {
    portENTER_CRITICAL(&stateMux);
    const OssmHatStopStatus status = publishedStop;
    portEXIT_CRITICAL(&stateMux);
    return status;
}

bool requestOssmHatExternalControlAfterReview() {
    if (!handbackConditions(true)) return false;
    OssmHatExternalControlStatus request;
    request.requested = true;
    request.requestedAtMs = millis();
    portENTER_CRITICAL(&stateMux);
    if (pendingStop || publishedStop.requested || stopInFlight ||
        publishedStatus.passive) {
        portEXIT_CRITICAL(&stateMux);
        return false;
    }
    if (pendingExternalControl || externalControlInFlight) {
        portEXIT_CRITICAL(&stateMux);
        return true;
    }
    publishedExternalControl = request;
    pendingExternalControl = true;
    portEXIT_CRITICAL(&stateMux);
    return true;
}

OssmHatExternalControlStatus getOssmHatExternalControlStatus() {
    portENTER_CRITICAL(&stateMux);
    const auto status = publishedExternalControl;
    portEXIT_CRITICAL(&stateMux);
    return status;
}

bool clearOssmHatStopAfterReview() {
    if (digitalRead(Pins::Driver::motorEnablePin) != HIGH) return false;
    portENTER_CRITICAL(&stateMux);
    if (pendingStop || stopInFlight || pendingExternalControl ||
        externalControlInFlight) {
        portEXIT_CRITICAL(&stateMux);
        return false;
    }
    publishedStop = {};
    publishedExternalControl = {};
    portEXIT_CRITICAL(&stateMux);
    return true;
}

bool ossmHatOwnsPin(int pin) {
    return initialized && (pin == Pins::Hat::tx || pin == Pins::Hat::rx ||
                           pin == Pins::Hat::direction);
}
