#include "ossmHat.h"

#include <Arduino.h>
#include <esp_timer.h>

#include "AimFrame.h"
#include "constants/Config.h"
#include "constants/Pins.h"
#include "hat_logic.h"
#include "services/stepper.h"

namespace ossm_hat {

namespace {

// 57AIM registers (YZ-AIM manual section V.2).
constexpr uint16_t serialControlRegister = 0x00;  // 1 = STEP/DIR ignored
constexpr uint16_t telemetryRegister = 0x0E;      // alarm, current, speed, voltage, ...
constexpr uint16_t telemetryCount = 10;           // ... through position at 0x16/0x17

constexpr uint8_t servoId = 1;
constexpr uint8_t pulseCounterUnit = 7;  // FastAccelStepper leaves 6 and 7 free
// Consecutive failed reads (each waits a reply timeout) before the servo
// counts as gone. Counting failures, not time, ignores CPU stalls.
constexpr uint32_t maxFailures = 20;
// The servo ignores a request that starts within ~1.2 ms of its own reply
// (Alex Warner's bench measurement).
constexpr uint32_t turnaroundUs = 1500;

HardwareSerial &uart = Serial2;
bool present = false;
volatile bool answering = false;
portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
Telemetry latest;
int32_t anchorCounts = 0;  // encoder count that pulse total 0 maps onto
int32_t pulses = 0;        // STEP pulses since the counter was attached
int16_t lastCount = 0;

// ── One request on the bus at a time ────────────────────────────────────
uint8_t sent[8];
size_t sentLength = 0;
size_t replyLength = 0;
uint8_t rx[AimFrame::maxFrame + 8];
size_t received = 0;
uint32_t sentMs = 0;
uint32_t replyTimeoutMs = 10;
uint32_t lastReplyUs = 0;

void transmit() {
    while (uart.available()) uart.read();
    uart.write(sent, sentLength);  // fits the TX FIFO: returns at once
    received = 0;
    sentMs = millis();
}

// Collect the reply: Incomplete while waiting, then Ok or a failure.
AimFrame::Status poll() {
    while (uart.available() && received < sizeof(rx)) rx[received++] = uart.read();
    const auto reply =
        AimFrame::parseReply(rx, received, servoId, sent[1], replyLength);
    auto status = reply.status;
    if (status == AimFrame::Status::Incomplete) {
        if (millis() - sentMs < replyTimeoutMs) return status;
        status = AimFrame::Status::BadReply;  // timed out
    }
    if (status == AimFrame::Status::Ok && reply.offset > 0) {
        memmove(rx, rx + reply.offset, replyLength);
    }
    lastReplyUs = micros();
    return status;
}

bool busReady() { return micros() - lastReplyUs >= turnaroundUs; }

void prepareRead(uint16_t address, uint16_t count) {
    sentLength = AimFrame::buildRead(sent, servoId, address, count);
    replyLength = AimFrame::replyLength(AimFrame::fcReadHolding, count);
}

// Blocking request, for setup before the reader starts.
bool transact() {
    while (!busReady()) {
    }
    transmit();
    AimFrame::Status status;
    while ((status = poll()) == AimFrame::Status::Incomplete) vTaskDelay(1);
    return status == AimFrame::Status::Ok;
}

// Extend the 16-bit hardware count into the running pulse total.
void countPulses() {
    const int16_t count = stepper->readPulseCounter();
    portENTER_CRITICAL(&lock);
    pulses = hat_logic::accumulatePulses(pulses, lastCount, count);
    portEXIT_CRITICAL(&lock);
    lastCount = count;
}

// Decode the telemetry reply in rx (registers 0x0E..0x17).
void storeTelemetry() {
    uint16_t r[telemetryCount];
    for (uint16_t i = 0; i < telemetryCount; i++) {
        r[i] = AimFrame::be16(rx + 3 + 2 * i);
    }
    const uint32_t now = millis();
    // ENA is active low (services/stepper.cpp). While the servo is off it
    // ignores pulses, so it comes back on holding wherever it now is: map
    // that spot onto the pulse total until it does.
    const bool outputOff = digitalRead(Pins::Driver::motorEnablePin) == HIGH;
    portENTER_CRITICAL(&lock);
    latest.alarm = r[0];
    latest.currentAmps = r[1] / 2000.0f;
    latest.voltage = r[3] / 327.0f;
    latest.positionCounts =
        static_cast<int32_t>((static_cast<uint32_t>(r[9]) << 16) | r[8]);
    latest.sampledMs = now;
    if (outputOff) {
        anchorCounts = hat_logic::anchorAt(latest.positionCounts, pulses,
                                           Config::Hat::countsPerPulse);
    }
    portEXIT_CRITICAL(&lock);
}

bool readTelemetry() {
    prepareRead(telemetryRegister, telemetryCount);
    if (!transact()) return false;
    storeTelemetry();
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

// ── Reader: a non-blocking step every millisecond (no task stack) ───────
esp_timer_handle_t timer = nullptr;
bool waiting = false;
uint32_t failures = 0;

void step(void *) {
    countPulses();
    if (waiting) {
        const auto status = poll();
        if (status == AimFrame::Status::Incomplete) return;
        waiting = false;
        if (status == AimFrame::Status::Ok) {
            storeTelemetry();
            failures = 0;
        } else if (++failures >= maxFailures) {
            // Motion never depended on the HAT: only the readings go.
            answering = false;
            esp_timer_stop(timer);
            ESP_LOGE("HAT", "Servo stopped answering on RS-485; using board readings");
            return;
        }
    }
    if (!busReady()) return;
    transmit();
    waiting = true;
}

}  // namespace

bool begin() {
#if defined(OSSM_HAT_DISABLED) || defined(AJ_DEVELOPMENT_HARDWARE)
    // Build without HAT support (or a board that uses IO22 for its display).
    return false;
#else
    if (stepper == nullptr) return false;
    // A motor powered up with the board may need a moment before it answers.
    for (int attempt = 0; attempt < 3 && !present; attempt++) {
        for (uint32_t baud : {115200u, 19200u}) {
            if (openAt(baud) && readTelemetry()) {
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
    if (transact() && AimFrame::be16(rx + 3) != 0) {
        ESP_LOGE("HAT", "Servo is in RS-485 control and ignores STEP/DIR; "
                        "power-cycle the motor");
    }
    lastCount = stepper->readPulseCounter();
    readTelemetry();  // anchor the encoder at boot, before any motion
    portENTER_CRITICAL(&lock);
    anchorCounts = hat_logic::anchorAt(latest.positionCounts, pulses,
                                       Config::Hat::countsPerPulse);
    portEXIT_CRITICAL(&lock);
    answering = true;  // the reader repeats the telemetry request in `sent`
    esp_timer_create_args_t args = {};
    args.callback = step;
    args.name = "ossmHat";
    esp_timer_create(&args, &timer);
    esp_timer_start_periodic(timer, 1000);
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
        latest.positionCounts, anchorCounts, pulses,
        Config::Hat::countsPerPulse);
    portEXIT_CRITICAL(&lock);
    return error;
}

}  // namespace ossm_hat
