#include "ossmHat.h"

#include <Arduino.h>
#include <rom/ets_sys.h>

#include <algorithm>

#include "AimFrame.h"
#include "constants/Config.h"
#include "constants/Pins.h"
#include "hat_logic.h"
#include "services/stepper.h"
#include "services/tasks.h"

namespace ossm_hat {

namespace {

// 57AIM registers (YZ-AIM manual section V.2).
constexpr uint16_t serialControlRegister = 0x00;  // 1 = STEP/DIR ignored
constexpr uint16_t telemetryRegister = 0x0E;      // alarm, current, speed, voltage, ...
constexpr uint16_t telemetryCount = 10;           // ... through position at 0x16/0x17

constexpr uint8_t servoId = 1;
constexpr uint8_t pulseCounterUnit = 7;  // FastAccelStepper leaves 6 and 7 free
// Every read takes the full telemetry, which is back ~4.6 ms after the request
// at 115200 baud, so reads start every 5 ms on RTOS ticks: 200 a second. The
// servo also ignores a request that starts less than ~3.7 ms after the
// previous one (Test Rig).
static_assert(configTICK_RATE_HZ == 1000, "read slots are in 1 ms ticks");
constexpr TickType_t readSlot = 5;
constexpr uint32_t minSpacingUs = 3900;
// Consecutive failed reads (each waits a reply timeout) before the servo
// counts as gone. Counting failures, not time, ignores CPU stalls.
constexpr uint32_t maxFailures = 20;
constexpr uint32_t readerStack = 3072;

HardwareSerial &uart = Serial2;
bool present = false;
volatile bool answering = false;
portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
Telemetry latest;
int32_t anchorCounts = 0;   // encoder count that pulse total 0 maps onto
int32_t samplePulses = 0;   // pulse total when `latest` was read
int32_t pulses = 0;         // STEP pulses since the counter was attached
int16_t lastCount = 0;

// ── One request on the bus at a time ────────────────────────────────────
uint8_t sent[8];
size_t sentLength = 0;
size_t replyLength = 0;
uint8_t rx[AimFrame::maxFrame + 8];
uint32_t replyTimeoutMs = 10;
uint32_t lastStartUs = 0;

void prepareRead(uint16_t address, uint16_t count) {
    sentLength = AimFrame::buildRead(sent, servoId, address, count);
    replyLength = AimFrame::replyLength(AimFrame::fcReadHolding, count);
}

// Send `sent` and block until its reply is in rx (true) or it timed out.
bool exchange() {
    const uint32_t since = micros() - lastStartUs;
    if (since < minSpacingUs) ets_delay_us(minSpacingUs - since);
    lastStartUs = micros();
    while (uart.available()) uart.read();
    uart.write(sent, sentLength);
    size_t received = 0;
    for (;;) {
        const auto reply =
            AimFrame::parseReply(rx, received, servoId, sent[1], replyLength);
        if (reply.status == AimFrame::Status::Ok) {
            if (reply.offset > 0) memmove(rx, rx + reply.offset, replyLength);
            return true;
        }
        const uint32_t waitedMs = (micros() - lastStartUs) / 1000;
        if (reply.status != AimFrame::Status::Incomplete ||
            waitedMs >= replyTimeoutMs || received >= sizeof(rx)) {
            return false;
        }
        // Sleeps until bytes land (a stray 0x00 before the reply costs a pass).
        uart.setTimeout(replyTimeoutMs - waitedMs);
        received += uart.readBytes(
            rx + received, std::max<size_t>(1, replyLength - std::min(received, replyLength)));
    }
}

// Extend the 16-bit hardware count into the running pulse total.
int32_t countPulses() {
    const int16_t count = stepper->readPulseCounter();
    portENTER_CRITICAL(&lock);
    pulses = hat_logic::accumulatePulses(pulses, lastCount, count);
    const int32_t total = pulses;
    portEXIT_CRITICAL(&lock);
    lastCount = count;
    return total;
}

// Store the telemetry reply in rx, read while the pulse total was `atPulses`.
void storeReply(int32_t atPulses) {
    uint16_t r[telemetryCount];
    for (uint16_t i = 0; i < telemetryCount; i++) {
        r[i] = AimFrame::be16(rx + 3 + 2 * i);
    }
    const auto reading = hat_logic::decodeTelemetry(r);
    // ENA is active low (services/stepper.cpp). While the servo is off it
    // ignores pulses, so it comes back on holding wherever it now is: map
    // that spot onto the pulse total until it does.
    const bool outputOff = digitalRead(Pins::Driver::motorEnablePin) == HIGH;
    const uint32_t now = millis();
    portENTER_CRITICAL(&lock);
    static_cast<hat_logic::ServoReading &>(latest) = reading;
    latest.sampledMs = now;
    samplePulses = atPulses;
    if (outputOff) {
        anchorCounts = hat_logic::anchorAt(reading.positionCounts, atPulses,
                                           Config::Hat::countsPerPulse);
    }
    portEXIT_CRITICAL(&lock);
}

// One read, paired with the pulse total halfway through it (about when the
// servo samples its encoder). Needs the pulse counter attached.
bool read() {
    prepareRead(telemetryRegister, telemetryCount);
    const int32_t before = countPulses();
    if (!exchange()) return false;
    const int32_t after = countPulses();
    storeReply(before + (after - before) / 2);
    return true;
}

bool openAt(uint32_t baud) {
    uart.end();
    uart.setRxBufferSize(256);
    uart.begin(baud, SERIAL_8N1, Pins::Hat::rx, Pins::Hat::tx);
    // The UART drives DE from its RTS pin: no software direction switching.
    replyTimeoutMs = baud >= 57600 ? 10 : 50;
    return uart.setPins(Pins::Hat::rx, Pins::Hat::tx, -1, Pins::Hat::de) &&
           uart.setMode(UART_MODE_RS485_HALF_DUPLEX);
}

// ── Reader task: one read per slot, only when a HAT answered ────────────
void readLoop(void *) {
    TickType_t wake = xTaskGetTickCount();
    uint32_t failures = 0;
    for (;;) {
        if (read()) {
            failures = 0;
        } else if (++failures >= maxFailures) {
            // Motion never depended on the HAT: only the readings go.
            answering = false;
            ESP_LOGE("HAT", "Servo stopped answering on RS-485; using board readings");
            vTaskDelete(nullptr);
        }
        // Next slot; after a timeout start afresh rather than catch up.
        wake += readSlot;
        const TickType_t now = xTaskGetTickCount();
        if (static_cast<int32_t>(wake - now) <= 0) wake = now + 1;
        vTaskDelay(wake - now);
    }
}

}  // namespace

bool begin() {
#if defined(OSSM_HAT_DISABLED) || defined(AJ_DEVELOPMENT_HARDWARE)
    // Build without HAT support (or a board that uses IO22 for its display).
    return false;
#else
    if (stepper == nullptr) return false;
    // A motor powered up with the board may need a moment before it answers.
    prepareRead(telemetryRegister, telemetryCount);
    for (int attempt = 0; attempt < 3 && !present; attempt++) {
        for (uint32_t baud : {115200u, 19200u}) {
            if (openAt(baud) && exchange()) {
                present = true;
                ESP_LOGI("HAT", "OSSM HAT found at %u baud", baud);
                break;
            }
        }
    }
    // The pulse counter must attach in the native frame (initStepper).
    if (present && !stepper->attachToPulseCounter(pulseCounterUnit, 0, 0)) {
        ESP_LOGE("HAT", "OSSM HAT answered but no pulse counter is free");
        present = false;
    }
    if (!present) {
        uart.end();
        ESP_LOGI("HAT", "No OSSM HAT: STEP/DIR only");
        return false;
    }
    // The firmware never writes to the servo. One left in serial control
    // (a power cycle clears it) would not follow the pulses.
    prepareRead(serialControlRegister, 1);
    if (exchange() && AimFrame::be16(rx + 3) != 0) {
        ESP_LOGE("HAT", "Servo is in RS-485 control and ignores STEP/DIR; "
                        "power-cycle the motor");
    }
    lastCount = stepper->readPulseCounter();
    read();  // anchor the encoder at boot, before any motion
    portENTER_CRITICAL(&lock);
    anchorCounts = hat_logic::anchorAt(latest.positionCounts, samplePulses,
                                       Config::Hat::countsPerPulse);
    portEXIT_CRITICAL(&lock);
    // Beside the motion tasks, at their priority: position matters as much.
    // The reader mostly sleeps on the UART. Its stack is allocated here, so
    // boards without a HAT pay nothing.
    answering = xTaskCreatePinnedToCore(readLoop, "ossmHat", readerStack,
                                        nullptr, configMAX_PRIORITIES - 1,
                                        nullptr, Tasks::operationTaskCore) ==
                pdPASS;
    if (!answering) ESP_LOGE("HAT", "Could not start the OSSM HAT reader");
    return true;
#endif
}

bool isPresent() { return present; }

bool isAnswering() { return answering; }

Telemetry telemetry() {
    portENTER_CRITICAL(&lock);
    const Telemetry copy = latest;
    portEXIT_CRITICAL(&lock);
    return copy;
}

int32_t followingErrorPulses() {
    portENTER_CRITICAL(&lock);
    const int32_t error = hat_logic::followingErrorPulses(
        latest.positionCounts, anchorCounts, samplePulses,
        Config::Hat::countsPerPulse);
    portEXIT_CRITICAL(&lock);
    return error;
}

}  // namespace ossm_hat
