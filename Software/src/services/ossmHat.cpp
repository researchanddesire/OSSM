#include "ossmHat.h"

#include <Arduino.h>
#include <cmath>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <hal/uart_ll.h>
#include <soc/uart_struct.h>

#include "constants/Pins.h"
#include "ossmHatProtocol.hpp"
#include "ossmHatPositionProtocol.hpp"
#include "ossmHatTxAudit.hpp"
#include "ossmHatEmergencyInternal.h"
#include "ossmHatEmergencyLogic.hpp"

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
bool pendingStop = false;
bool stopInFlight = false;
bool initialized = false;
ossm_hat::PositionQueue positionQueue;
OssmHatPositionValidator positionValidator = nullptr;
ossm_hat::LatePositionReply latePositionReply;
ossm_hat::TxAudit txAudit;
OssmHatEmergencyCause requestedProcedure = OssmHatEmergencyCause::None;
bool procedureBusy = false;
bool restoreRequested = false;

size_t ownerWrite(const uint8_t *frame, size_t length) {
    if (!ossm_hat_emergency::ordinaryTxAllowed()) return 0;
    const size_t accepted = Serial2.write(frame, length);
    const uint32_t at = millis();
    portENTER_CRITICAL(&stateMux);
    txAudit.accepted(frame, length, accepted, at);
    portEXIT_CRITICAL(&stateMux);
    return accepted;
}

// Shared snapshot checks are performed before calling the external validator;
// no HAT lock is held while the guard/facade inspects its own authorization.
bool positionConditions(uint64_t session, bool motion, bool requireEnabled,
                        const int32_t *hold = nullptr) {
    if (session == 0) return false;
    portENTER_CRITICAL(&stateMux);
    const auto sample = publishedSample;
    const auto controls = publishedControls;
    const auto status = publishedStatus;
    const auto validator = positionValidator;
    const bool permitted = initialized && !pendingStop && !publishedStop.requested &&
        !stopInFlight && !procedureBusy && ossm_hat_emergency::ordinaryTxAllowed();
    portEXIT_CRITICAL(&stateMux);
    const uint32_t now = millis();
    if (!permitted || validator == nullptr || status.passive ||
        status.baudRate != 115200 || status.sampleRateHz < 30 ||
        status.sampleWindowMs < 500 || !ossm_hat::fresh(sample, now) ||
        static_cast<uint32_t>(now - sample.updatedAtMs) > 100 ||
        !ossm_hat::controlsFresh(controls, sample, now) || controls.modbusEnable != 1 ||
        !std::isfinite(sample.currentAmps) || sample.currentAmps < 0 ||
        !std::isfinite(sample.voltageVolts) ||
        sample.voltageVolts < 20 || sample.voltageVolts > 36 ||
        (sample.alarm != 0 && sample.alarm != 0x10)) return false;
    if (requireEnabled && (controls.outputEnable & 1U) == 0) return false;
    if (!motion && ((controls.outputEnable & 1U) != 0 || sample.currentAmps >= 0.5f))
        return false;
    if (hold != nullptr) {
        const int64_t delta = static_cast<int64_t>(*hold) - sample.positionCounts;
        // The caller proves physical stationarity; additionally refuse an old
        // hold target more than0.25mm from the current physical encoder.
        if (delta < -204 || delta > 204) return false;
    }
    return validator(session, motion);
}

bool positionActionPermitted(const ossm_hat::PositionAction &action) {
    portENTER_CRITICAL(&stateMux);
    const bool current = positionQueue.current(action);
    portEXIT_CRITICAL(&stateMux);
    if (!current || static_cast<uint32_t>(millis() - action.queuedAtMs) > action.maximumAgeMs)
        return false;
    const int32_t *hold = (action.kind == ossm_hat::PositionActionKind::Hold ||
        action.kind == ossm_hat::PositionActionKind::Enable ||
        action.kind == ossm_hat::PositionActionKind::VerifyEnable) ?
        &action.targetCounts : nullptr;
    return positionConditions(action.session, action.motion(),
        action.kind == ossm_hat::PositionActionKind::Target, hold);
}

// At most one committed owner exchange. It checks epoch/stop authorization
// throughout every wait and immediately before TX. No retry can repeat a
// cancelled or expired command. A frame committed before Stop can finish;
// the priority stop/readback, never a GPIO level, acknowledges serial stop.
bool positionExchange(const ossm_hat::PositionAction &action,
                      ossm_hat::Decoder &observer, bool &externalMaster,
                      uint8_t *response, size_t &received, bool &transmitted) {
    uint32_t quietAt = micros();
    const uint32_t quietStarted = millis();
    while (static_cast<uint32_t>(micros() - quietAt) < 1500) {
        if (!positionActionPermitted(action)) return false;
        while (Serial2.available()) {
            const int byte = Serial2.read();
            const bool lateOwn = byte >= 0 && latePositionReply.push(
                static_cast<uint8_t>(byte), millis());
            if (byte >= 0 && (observer.push(static_cast<uint8_t>(byte), millis()) &
                             ossm_hat::Decoder::ExternalRequest) && !lateOwn) {
                externalMaster = true;
                return false;
            }
            quietAt = micros();
        }
        if (static_cast<uint32_t>(millis() - quietStarted) >= 10) return false;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (!positionActionPermitted(action)) return false;
    portENTER_CRITICAL(&stateMux);
    const bool obsolete = !positionQueue.current(action) || positionQueue.superseded(action);
    portEXIT_CRITICAL(&stateMux);
    if (obsolete) return false;
    if (action.kind == ossm_hat::PositionActionKind::Enable &&
        !ossmHatEmergencyMarkOutputPossible()) return false;
    if (action.readsControls()) observer.expectRead(ossm_hat::motorId, 0, 4, millis());
    transmitted = true;
    const size_t accepted = ownerWrite(action.frame, action.frameLength);
    if (!latePositionReply.expect(action, accepted, millis())) {
        // Any attempted short write faults this intent and queues priority
        // disable. It must never claim a response window or replay motion.
        Serial2.flush();
        return false;
    }
    Serial2.flush();
    received = 0;
    const uint32_t started = millis();
    while (static_cast<uint32_t>(millis() - started) < 20) {
        if (!positionActionPermitted(action)) return false;
        while (Serial2.available() && received < 80) {
            const int byte = Serial2.read();
            if (byte < 0) break;
            response[received++] = static_cast<uint8_t>(byte);
            latePositionReply.push(static_cast<uint8_t>(byte), millis());
            const auto parsed = AimFrame::parseReply(response, received, ossm_hat::motorId,
                                                     action.function, action.replyLength);
            const auto events = observer.push(static_cast<uint8_t>(byte), millis());
            if (parsed.status == AimFrame::Status::Ok &&
                action.function == AimFrame::fcMoveAbsolute) {
                portENTER_CRITICAL(&stateMux);
                positionQueue.recordVendorReply(action, response, received, millis());
                portEXIT_CRITICAL(&stateMux);
            }
            // FC06 echoes and FC7B responses otherwise look like external
            // requests to the passive parser. Only this expected CRC-valid
            // response is consumed as the current owner's acknowledgement.
            const bool ownReply = ossm_hat::ownedPositionReply(action, response, received);
            if ((events & ossm_hat::Decoder::ExternalRequest) && !ownReply) {
                externalMaster = true;
                return false;
            }
            if (parsed.status == AimFrame::Status::Ok) {
                latePositionReply.clear();
                return true;
            }
            if (parsed.status == AimFrame::Status::Exception ||
                parsed.status == AimFrame::Status::BadCrc) return false;
        }
        if (received == 80) return false;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

// Used only by the UART owner for a latched emergency stop. Ordinary
// telemetry never writes a register. All exchanges are bounded and validated;
// GPIO ENA/pulses must already be disabled by the independent guard.
bool stopExchange(const uint8_t *request, size_t length, uint8_t function,
                  size_t replyLength, ossm_hat::Decoder &observer,
                  bool &externalMaster, uint16_t *controlWords = nullptr) {
    uint32_t quietAt = micros();
    const uint32_t quietStarted = millis();
    while (static_cast<uint32_t>(micros() - quietAt) < 3000) {
        while (Serial2.available()) {
            const int byte = Serial2.read();
            const bool lateOwn = byte >= 0 && latePositionReply.push(
                static_cast<uint8_t>(byte), millis());
            if (byte >= 0 &&
                (observer.push(static_cast<uint8_t>(byte), millis()) &
                 ossm_hat::Decoder::ExternalRequest) && !lateOwn) {
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
    ownerWrite(request, length);
    Serial2.flush();
    uint8_t response[80] = {};
    size_t received = 0;
    const uint32_t started = millis();
    while (static_cast<uint32_t>(millis() - started) < ossm_hat::replyTimeoutMs) {
        while (Serial2.available() && received < sizeof(response)) {
            const int byte = Serial2.read();
            if (byte < 0) break;
            response[received++] = static_cast<uint8_t>(byte);
            const bool lateOwn = latePositionReply.push(static_cast<uint8_t>(byte), millis());
            const uint8_t observed = observer.push(static_cast<uint8_t>(byte), millis());
            const auto partial = AimFrame::parseReply(
                response, received, ossm_hat::motorId, function, replyLength);
            // FC06 replies echo their request. Only the exact CRC-validated
            // echo currently expected by this exchange is our own reply.
            const bool ownEcho = function == AimFrame::fcWriteSingle &&
                partial.status == AimFrame::Status::Ok &&
                std::memcmp(response + partial.offset, request, 6) == 0;
            if ((observed & ossm_hat::Decoder::ExternalRequest) && !ownEcho && !lateOwn) {
                externalMaster = true;
                return false;
            }
            if (lateOwn && partial.status != AimFrame::Status::Ok) {
                // The preempted transaction's late frame is complete. Start
                // the current stop's reply buffer after it, not before it.
                received = 0;
                continue;
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
        if (!ossm_hat::stopReplyCanContinue(parsed.status, received,
                                           latePositionReply, millis()) ||
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
    // Output-off is safe to request without changing control modes. Never
    // select mode1 as a stop: that could expose an old serial target/output
    // setting on a mode0 device before the following disable arrived.
    ossm_hat::buildSerialOutputDisable(request);
    stopExchange(request, sizeof(request), AimFrame::fcWriteSingle, 8,
                  observer, externalMaster);
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
    result.verified = readBack && ossm_hat::serialOutputDisabled(words[0], words[1]);
    result.passiveBlocked = externalMaster;
    result.complete = true;
    result.completedAtMs = millis();
    ESP_LOGE(tag, "Guard RS485 output disable %s (control=%u output=%u)",
             result.verified ? "verified" : "UNVERIFIED", words[0], words[1]);
    return result;
}

bool startUart(uint32_t baud) {
    // Atomic admission, not a check followed by an unprotected remap. Enable
    // admission and reconfiguration cannot both acquire this route state.
    if (!ossm_hat_emergency::beginConfiguration()) return false;
    Serial2.end();
    pinMode(Pins::Hat::direction, OUTPUT);
    digitalWrite(Pins::Hat::direction, LOW);
    Serial2.setRxBufferSize(512);
    Serial2.begin(baud, SERIAL_8N1, Pins::Hat::rx, Pins::Hat::tx);
    // Deliver bytes during long 9600-baud replies as well as at RX idle;
    // otherwise a whole slow reply could arrive after the resync timeout.
    Serial2.setRxFIFOFull(1);
    Serial2.setRxTimeout(2);
    // Establish inactive RTS before attaching the physical DE pad.
    const bool mode = Serial2.setMode(UART_MODE_RS485_HALF_DUPLEX);
    const bool pins = Serial2.setPins(Pins::Hat::rx, Pins::Hat::tx, -1,
                                      Pins::Hat::direction);
    gpio_pullup_en(static_cast<gpio_num_t>(Pins::Hat::rx));
    if (!pins || !mode) {
        Serial2.end();
        digitalWrite(Pins::Hat::direction, LOW);
        ESP_LOGE(tag, "UART2 RS485 setup failed");
        ossm_hat_emergency::endConfiguration();
        return false;
    }
    ossm_hat_emergency::endConfiguration();
    return true;
}

void reader(void *) {
    // Install both raw IRQs on this pinned core, with UART1 clean before the
    // ordinary UART2 driver is initialized. No UART1 default pins are attached.
    ossm_hat_emergency::installHardware();
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

    OssmHatEmergencyCause procedure = OssmHatEmergencyCause::None;
    bool sawIsolation = false;
    bool restoredTransport = false;
    bool backupAwaiting = false;
    uint32_t procedureAt = 0, backupRequestAt = 0, backupPollAt = 0;
    uint8_t echoWindow[8] = {};
    size_t echoUsed = 0;

    auto publishDecoder = [&]() {
        const uint32_t at = millis();
        status.fresh = ossm_hat::fresh(decoder.sample(), at);
        status.ageMs = decoder.sample().sequence ? at-decoder.sample().updatedAtMs:UINT32_MAX;
        status.validFrames=decoder.validFrames(); status.crcErrors=decoder.crcErrors();
        sampleRate.read(at,status.sampleRateHz,status.sampleWindowMs,status.sampleCount);
        status.startupReady=status.fresh && status.sampleRateHz>=30 && status.sampleWindowMs>=500;
        portENTER_CRITICAL(&stateMux);
        publishedSample=decoder.sample(); publishedControls=decoder.controls(); publishedStatus=status;
        portEXIT_CRITICAL(&stateMux);
        ossm_hat_emergency::ownerLease(lockedBaud && !status.passive && status.baudRate==115200);
        ossm_hat_emergency::observe(decoder.sample(),decoder.controls());
    };

    auto emergencyService = [&]() -> bool {
        auto emergency=getOssmHatEmergencyStatus();
        portENTER_CRITICAL(&stateMux);
        const auto requested=requestedProcedure;
        const bool restore=restoreRequested;
        const bool noCommand=!positionQueue.status().pending && !stopInFlight && !pendingStop;
        portEXIT_CRITICAL(&stateMux);

        if (procedure==OssmHatEmergencyCause::None && requested!=OssmHatEmergencyCause::None &&
            !awaiting && noCommand && !emergency.isolated) {
            procedure=requested; procedureAt=millis();
            // This acknowledged checkpoint is after all ordinary exchanges
            // returned, outside every driver/state lock. No normal command
            // may be admitted while procedureBusy is set.
            ossm_hat_emergency::ownerParked(true);
            if (!ossm_hat_emergency::beginTest(procedure)) ossm_hat_emergency::failed();
            else if (procedure==OssmHatEmergencyCause::CacheOffPreflight)
                ossm_hat_emergency::cacheOffWindow();
            emergency=getOssmHatEmergencyStatus();
        }

        if (emergency.isolated) {
            if (!sawIsolation) {
                sawIsolation=true; restoredTransport=false;
                decoder.reset(); awaiting=false; backupAwaiting=false; echoUsed=0;
                ossm_hat_emergency::ownerParked(true);
                latePositionReply.clear();
                if (!ossm_hat_emergency::expected(emergency.cause)) {
                    portENTER_CRITICAL(&stateMux);
                    positionQueue.cancel(OssmHatPositionFault::Cancelled);
                    pendingStop=false;
                    portEXIT_CRITICAL(&stateMux);
                }
            }
            uint8_t bytes[128];
            const size_t count=ossm_hat_emergency::receive(bytes,sizeof(bytes));
            for (size_t i=0;i<count;++i) {
                if (!emergency.echoVerified) {
                    if (echoUsed==sizeof(echoWindow)) {
                        std::memmove(echoWindow,echoWindow+1,sizeof(echoWindow)-1);
                        --echoUsed;
                    }
                    echoWindow[echoUsed++]=bytes[i];
                    static const uint8_t expectedEcho[8]={1,6,0,1,0,0,0xD8,0x0A};
                    if (emergency.txDone && echoUsed==sizeof(expectedEcho) &&
                        std::memcmp(echoWindow,expectedEcho,sizeof(expectedEcho))==0) {
                        ossm_hat_emergency::acknowledgeEcho(millis());
                        emergency=getOssmHatEmergencyStatus();
                        decoder.reset();
                    }
                } else {
                    const uint8_t events=decoder.push(bytes[i],millis());
                    if (events&ossm_hat::Decoder::ExternalRequest) {
                        status.passive=true; ossm_hat_emergency::failed();
                    }
                    if (events&ossm_hat::Decoder::Telemetry) {
                        sampleRate.accepted(millis()); backupAwaiting=false;
                        lastFrameAt=millis();
                    }
                }
            }
            emergency=getOssmHatEmergencyStatus();
            if (emergency.rxOverflows || ossm_hat_emergency::protocolFailed()) {
                ossm_hat_emergency::failed();
            } else if (emergency.echoVerified) {
                if (backupAwaiting && millis()-backupRequestAt>=100) {
                    backupAwaiting=false; decoder.reset(); ossm_hat_emergency::failed();
                }
                if (!backupAwaiting && millis()-backupPollAt>=20 &&
                    ossm_hat_emergency::readFullRegisters()) {
                    decoder.reset(); decoder.expectRead(1,0,26,millis());
                    backupAwaiting=true; backupRequestAt=backupPollAt=millis();
                }
            }
            publishDecoder();
            emergency=getOssmHatEmergencyStatus();
            if (emergency.offVerified) {
                portENTER_CRITICAL(&stateMux);
                pendingStop=false; stopInFlight=false;
                publishedStop.requested=publishedStop.complete=publishedStop.verified=true;
                publishedStop.modbusEnable=1; publishedStop.driveOutputEnable=0;
                publishedStop.completedAtMs=emergency.offAtMs;
                portEXIT_CRITICAL(&stateMux);
            }
            const bool preflight=procedure==OssmHatEmergencyCause::OffOnlyPreflight ||
                procedure==OssmHatEmergencyCause::CacheOffPreflight;
            if ((restore || preflight) && !backupAwaiting && emergency.quietVerified &&
                !emergency.failed && !status.passive) {
                if (!ossm_hat_emergency::beginRestore()) {
                    ossm_hat_emergency::failed(); return true;
                }
                // Every old session, queued target and retained reply loses
                // authority before deleting any driver state. GPIO DE is
                // already held constant LOW through detach/reset/reattach.
                portENTER_CRITICAL(&stateMux);
                positionQueue.cancel(OssmHatPositionFault::Cancelled);
                latePositionReply.clear(); pendingStop=false; stopInFlight=false;
                publishedControls.valid=false; publishedSample.sequence=0;
                portEXIT_CRITICAL(&stateMux);
                const uint32_t drainAt=millis();
                while (!uart_ll_is_tx_idle(&UART2) && millis()-drainAt<20)
                    vTaskDelay(pdMS_TO_TICKS(1));
                if (!uart_ll_is_tx_idle(&UART2)) {
                    ossm_hat_emergency::failed(); return true;
                }
                Serial2.end(); // parked owner, no outstanding HAL users
                if (!ossm_hat_emergency::resetUnderConstantFence() ||
                    !startUart(115200) || !ossm_hat_emergency::finishRestore()) {
                    ossm_hat_emergency::failed(); return true;
                }
                decoder.restartContinuity(); sampleRate=ossm_hat::SampleRate{};
                controlsPolled=false; awaiting=false; lastByteAt=lastPollAt=millis();
                status.baudRate=115200; uartReady=lockedBaud=true;
                restoredTransport=true;
            }
            return true;
        }
        if (restoredTransport && emergency.restored) {
            portENTER_CRITICAL(&stateMux);
            requestedProcedure=OssmHatEmergencyCause::None;
            procedureBusy=false; restoreRequested=false;
            publishedStop={};
            portEXIT_CRITICAL(&stateMux);
            procedure=OssmHatEmergencyCause::None; sawIsolation=false; restoredTransport=false;
        }
        if (emergency.failed) return true;
        if (procedure!=OssmHatEmergencyCause::None && !restoredTransport) {
            if (millis()-procedureAt>2000) ossm_hat_emergency::failed();
            return true;
        }
        return false;
    };

    for (;;) {
        if (emergencyService()) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }
        uint32_t now = millis();
        portENTER_CRITICAL(&stateMux);
        const OssmHatStopStatus stopRequest = publishedStop;
        const bool performStop = ossm_hat::deferredStopReady(pendingStop, now,
            stopRequest.requestedAtMs, status.passive, uartReady, lockedBaud, status.detected);
        if (performStop) { pendingStop = false; stopInFlight = true; }
        portEXIT_CRITICAL(&stateMux);
        if (performStop) {
            decoder.invalidateControls();
            OssmHatStopStatus stopped = stopRequest;
            if (stopped.attempts != UINT16_MAX) ++stopped.attempts;
            if (status.passive || !uartReady || !lockedBaud || !status.detected) {
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
            if (!stopped.verified && !stopped.hadFailure) {
                stopped.hadFailure = true;
                stopped.firstFailureAtMs = stopped.completedAtMs;
                stopped.firstFailureMode = stopped.modbusEnable;
                stopped.firstFailureOutput = stopped.driveOutputEnable;
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
            const bool lateOwn = latePositionReply.push(static_cast<uint8_t>(byte), now);
            const uint8_t events = decoder.push(static_cast<uint8_t>(byte), now);
            if ((events & ossm_hat::Decoder::ExternalRequest) && !lateOwn) {
                if (!status.passive) {
                    ESP_LOGI(tag, "External RS485 master detected; receive only until reboot");
                }
                status.passive = true;
                lockedBaud = true;
                lastFrameAt = now;
                portENTER_CRITICAL(&stateMux);
                const auto motion = positionQueue.status();
                const bool hadPositionOwner = motion.requested && !motion.faulted;
                if (hadPositionOwner) positionQueue.cancel(OssmHatPositionFault::SharedBus);
                portEXIT_CRITICAL(&stateMux);
                if (hadPositionOwner) requestOssmHatEmergencyStop();
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
        // A boot stop can fail before power/link is restored. Only the safe
        // output-off/readback sequence may reassert automatically, at most
        // once per second on a fresh sole-master link. Retain its first
        // failure and original request; no position/enable retry is allowed.
        const bool retryLinkReady = uartReady && lockedBaud && status.detected &&
            !status.passive && !awaiting && ossm_hat::fresh(decoder.sample(), now) &&
            static_cast<uint32_t>(now - decoder.sample().updatedAtMs) <= 100;
        portENTER_CRITICAL(&stateMux);
        if (!pendingStop && !stopInFlight &&
            ossm_hat::retrySerialStopNeeded(publishedStop, now, retryLinkReady)) {
            pendingStop = true;
            publishedStop.complete = false;
        }
        portEXIT_CRITICAL(&stateMux);
        if (awaiting && static_cast<uint32_t>(now - requestAt) >=
                            ossm_hat::replyTimeoutMs) {
            awaiting = false;
            decoder.reset();
        }

        if (lockedBaud && ossm_hat_emergency::configurationAllowed() &&
            static_cast<uint32_t>(now - lastFrameAt) >=
                              reconnectAfterMs) {
            // Keep a detected HAT latched across disconnects. Consumers see
            // stale telemetry, while the background reader searches again.
            lockedBaud = false;
            listeningSweep = true;
            baudIndex = baudCount - 1;
            advanceBaud();
        } else if (!lockedBaud && ossm_hat_emergency::configurationAllowed()) {
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
        // Full physical telemetry has priority over every ordinary position
        // transaction. Setup/enable are split into individual operations so
        // they cannot monopolize the bus or invent a configured sample rate.
        if (uartReady && lockedBaud && !status.passive && !awaiting && !procedureBusy &&
            static_cast<uint32_t>(now - lastPollAt) < pollIntervalMs) {
            ossm_hat::PositionAction action;
            portENTER_CRITICAL(&stateMux);
            const bool taken = positionQueue.take(now, action);
            const bool expired = positionQueue.status().faulted &&
                positionQueue.status().fault == OssmHatPositionFault::Expired;
            portEXIT_CRITICAL(&stateMux);
            if (taken) {
                uint8_t response[80] = {};
                size_t received = 0;
                bool externalMaster = false;
                bool transmitted = false;
                const bool success = positionExchange(action, decoder, externalMaster,
                                                       response, received, transmitted);
                portENTER_CRITICAL(&stateMux);
                if (positionQueue.current(action)) {
                    if (success) positionQueue.complete(action, response, received, millis());
                    else if (!transmitted && positionQueue.superseded(action)) positionQueue.abandon(action);
                    else positionQueue.cancel(externalMaster ? OssmHatPositionFault::SharedBus :
                                               OssmHatPositionFault::Protocol);
                }
                const bool failed = positionQueue.status().faulted;
                portEXIT_CRITICAL(&stateMux);
                if (externalMaster) status.passive = true;
                if (failed) requestOssmHatEmergencyStop();
                lastByteAt = millis();
                decoder.reset();
            } else if (expired) requestOssmHatEmergencyStop();
        }

        now = millis();
        // Read-only standalone polling is allowed only after a complete
        // receive-only baud sweep and a long quiet bus. Once any external
        // master is observed this branch is permanently unavailable.
        portENTER_CRITICAL(&stateMux);
        const bool serviceableStop = pendingStop && lockedBaud && status.detected;
        portEXIT_CRITICAL(&stateMux);
        const bool standalonePoll = uartReady && !status.passive && !serviceableStop &&
            !listeningSweep && !awaiting &&
            (lockedBaud || !attempted) &&
            static_cast<uint32_t>(now - lastPollAt) >= pollIntervalMs &&
            static_cast<uint32_t>(now - lastByteAt) >=
                (lockedBaud ? 3U : quietBeforeProbeMs);
        if (standalonePoll) {
            uint8_t request[8];
            portENTER_CRITICAL(&stateMux);
            const bool positionRequested = positionQueue.status().requested;
            portEXIT_CRITICAL(&stateMux);
            // Once a serial position session has been requested, retain full
            // controls in every existing telemetry transaction, even after
            // cancellation/fault while final disabled quiet is being proven.
            const bool includeControls = getOssmHatEmergencyStatus().available ||
                ossm_hat::fullControlPoll(positionRequested, controlsPolled, now, lastControlPollAt);
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
            ownerWrite(request, length);
            Serial2.flush();
        }

        publishDecoder();
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
    ossmHatEmergencyStopRequested();
    OssmHatStopStatus requested;
    requested.requested = true;
    requested.requestedAtMs = millis();
    portENTER_CRITICAL(&stateMux);
    positionQueue.cancel();
    // Until an explicit reviewed clear, repeated high-rate enforcement must
    // neither queue another exchange nor erase a verified completion result.
    // The owner alone may safely reassert an unverified output-off after
    // link restoration, retaining original/first-failure diagnostics.
    if (publishedStop.requested) {
        portEXIT_CRITICAL(&stateMux);
        return;
    }
    publishedStop = requested;
    publishedControls.valid = false;
    pendingStop = true;
    portEXIT_CRITICAL(&stateMux);
}

OssmHatStopStatus getOssmHatEmergencyStopStatus() {
    portENTER_CRITICAL(&stateMux);
    const OssmHatStopStatus status = publishedStop;
    portEXIT_CRITICAL(&stateMux);
    return status;
}

bool clearOssmHatStopAfterReview() {
    if (digitalRead(Pins::Driver::motorEnablePin) != HIGH) return false;
    portENTER_CRITICAL(&stateMux);
    if (pendingStop || stopInFlight || (positionQueue.status().requested &&
                                   !positionQueue.status().faulted)) {
        portEXIT_CRITICAL(&stateMux);
        return false;
    }
    publishedStop = {};
    portEXIT_CRITICAL(&stateMux);
    return true;
}

bool ossmHatOwnsPin(int pin) {
    return initialized && (pin == Pins::Hat::tx || pin == Pins::Hat::rx ||
                           pin == Pins::Hat::direction);
}

void setOssmHatPositionValidator(OssmHatPositionValidator validator) {
    portENTER_CRITICAL(&stateMux);
    positionValidator = validator;
    portEXIT_CRITICAL(&stateMux);
    if (validator == nullptr) cancelOssmHatPositionControl();
}

bool requestOssmHatPreparePositionControl(uint64_t session, int32_t holdCounts,
                                        uint16_t rpm, uint16_t acceleration) {
    if (!ossm_hat::validPositionLimits(rpm, acceleration) ||
        !positionConditions(session, false, false, &holdCounts)) return false;
    portENTER_CRITICAL(&stateMux);
    const bool accepted = !publishedStop.requested && !publishedStatus.passive &&
        positionQueue.prepare(session, holdCounts, rpm, acceleration, millis());
    portEXIT_CRITICAL(&stateMux);
    return accepted;
}

bool requestOssmHatEnablePositionControl(uint64_t session) {
    const auto prepared = getOssmHatPositionStatus();
    if (prepared.session != session || !prepared.prepared ||
        !positionConditions(session, true, false, &prepared.acknowledgedTargetCounts)) return false;
    portENTER_CRITICAL(&stateMux);
    const bool accepted = !publishedStop.requested && positionQueue.enable(session, millis());
    portEXIT_CRITICAL(&stateMux);
    return accepted;
}

bool queueOssmHatPositionTarget(uint64_t session, uint32_t requestId,
                               int32_t targetCounts, uint16_t rpm,
                               uint16_t acceleration, uint32_t maximumAgeMs) {
    if (!positionConditions(session, true, true)) return false;
    portENTER_CRITICAL(&stateMux);
    const bool accepted = !publishedStop.requested && positionQueue.queue(
        session, requestId, targetCounts, rpm, acceleration, millis(), maximumAgeMs);
    portEXIT_CRITICAL(&stateMux);
    return accepted;
}

OssmHatPositionStatus getOssmHatPositionStatus() {
    portENTER_CRITICAL(&stateMux);
    const auto status = positionQueue.status();
    portEXIT_CRITICAL(&stateMux);
    return status;
}

void cancelOssmHatPositionControl() {
    requestOssmHatEmergencyStop();
}

OssmHatTxAudit getOssmHatTxAudit() {
    portENTER_CRITICAL(&stateMux);
    const auto audit = txAudit.snapshot();
    portEXIT_CRITICAL(&stateMux);
    return audit;
}

namespace {
bool queueEmergencyProcedure(OssmHatEmergencyCause cause) {
    const auto raw=getOssmHatEmergencyStatus();
    if (!raw.available || !raw.irqInstalled || raw.isolated || raw.failed || raw.restoring ||
        ossm_hat_emergency::restoreVetoed())
        return false;
    const bool qualification=cause==OssmHatEmergencyCause::Qualification;
    if (qualification ? (!raw.offOnlyPassed || !raw.cacheOffPassed ||
                         raw.qualificationPassed || !raw.outputMayBeOn)
                      : (raw.outputMayBeOn || !raw.quietVerified)) return false;
    if (cause==OssmHatEmergencyCause::OffOnlyPreflight && raw.offOnlyPassed) return false;
    if (cause==OssmHatEmergencyCause::CacheOffPreflight &&
        (!raw.offOnlyPassed || raw.cacheOffPassed)) return false;
    const uint32_t now=millis();
    portENTER_CRITICAL(&stateMux);
    const bool accepted=initialized && !procedureBusy && !pendingStop && !stopInFlight &&
        !positionQueue.status().pending && !publishedStatus.passive &&
        publishedStatus.startupReady && publishedStatus.baudRate==115200 &&
        publishedSample.sequence && now-publishedSample.updatedAtMs<=40 &&
        publishedControls.valid && publishedControls.continuity==publishedSample.continuity &&
        publishedControls.updatedAtMs==publishedSample.updatedAtMs &&
        publishedControls.modbusEnable==1 &&
        (((publishedControls.outputEnable&1)!=0)==qualification) &&
        (publishedSample.alarm==0 || publishedSample.alarm==0x10) && publishedSample.currentAmps>=0 &&
        publishedSample.currentAmps<0.5f;
    if (accepted) { requestedProcedure=cause; procedureBusy=true; restoreRequested=false; }
    portEXIT_CRITICAL(&stateMux);
    return accepted;
}
}

bool requestOssmHatEmergencyPreflight(bool cacheOff) {
    if (digitalRead(Pins::Driver::motorEnablePin)!=HIGH) return false;
    return queueEmergencyProcedure(cacheOff ? OssmHatEmergencyCause::CacheOffPreflight:
                                            OssmHatEmergencyCause::OffOnlyPreflight);
}

bool requestOssmHatTimerQualification() {
    return queueEmergencyProcedure(OssmHatEmergencyCause::Qualification);
}

bool requestOssmHatQualificationRestore() {
    const auto raw=getOssmHatEmergencyStatus();
    if (raw.cause!=OssmHatEmergencyCause::Qualification || !raw.isolated ||
        !raw.ownerParked || !raw.echoVerified || !raw.offVerified ||
        !raw.quietVerified || raw.failed || raw.outputMayBeOn) return false;
    portENTER_CRITICAL(&stateMux);
    const bool accepted=procedureBusy && requestedProcedure==OssmHatEmergencyCause::Qualification;
    if (accepted) restoreRequested=true;
    portEXIT_CRITICAL(&stateMux);
    return accepted;
}
