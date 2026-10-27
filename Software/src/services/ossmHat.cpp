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
constexpr uint16_t serialControlRegister = 0x00;  // 1 = RS-485 targets, STEP/DIR ignored
constexpr uint16_t outputEnableRegister = 0x01;
constexpr uint16_t accelerationRegister = 0x03;   // >= 60000 = no internal ramp
constexpr uint16_t telemetryRegister = 0x0E;      // alarm, current, speed, voltage, ...
constexpr uint16_t telemetryCount = 10;           // ... through position at 0x16/0x17

constexpr uint8_t servoId = 1;
constexpr uint8_t pulseCounterUnit = 7;  // FastAccelStepper leaves 6 and 7 free
constexpr uint32_t telemetryEvery = 4;   // every 4th request reads telemetry
constexpr uint32_t linkTimeoutMs = 150;
// The servo ignores a request that starts within ~1.2 ms of its own reply
// (Alex Warner's bench measurement).
constexpr uint32_t turnaroundUs = 1500;

HardwareSerial &uart = Serial2;
bool present = false;
portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
Telemetry latest;
int32_t anchorCounts = 0;  // encoder count that pulse total 0 maps onto
int32_t pulses = 0;        // STEP pulses since the counter was attached

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
    // The servo echoes a write it accepted.
    if (status == AimFrame::Status::Ok && sent[1] == AimFrame::fcWriteSingle &&
        memcmp(sent, rx + reply.offset, 6) != 0) {
        status = AimFrame::Status::BadReply;
    }
    if (status == AimFrame::Status::Ok && reply.offset > 0) {
        memmove(rx, rx + reply.offset, replyLength);
    }
    lastReplyUs = micros();
    return status;
}

bool busReady() { return micros() - lastReplyUs >= turnaroundUs; }

// Each request builder prepares `sent`; transmit() puts it on the bus.
void prepareRead() {
    sentLength =
        AimFrame::buildRead(sent, servoId, telemetryRegister, telemetryCount);
    replyLength = AimFrame::replyLength(AimFrame::fcReadHolding, telemetryCount);
}

void prepareWrite(uint16_t address, uint16_t value) {
    sentLength = AimFrame::buildWriteSingle(sent, servoId, address, value);
    replyLength = 8;
}

void prepareTarget(int32_t target) {
    sentLength = AimFrame::buildMoveAbsolute(sent, servoId, target);
    replyLength = 8;
}

// Decode the telemetry reply in rx (registers 0x0E..0x17).
void storeTelemetry() {
    uint16_t r[telemetryCount];
    for (uint16_t i = 0; i < telemetryCount; i++) {
        r[i] = AimFrame::be16(rx + 3 + 2 * i);
    }
    const uint32_t now = millis();
    portENTER_CRITICAL(&lock);
    latest.alarm = r[0];
    latest.currentAmps = r[1] / 2000.0f;
    latest.voltage = r[3] / 327.0f;
    latest.positionCounts =
        static_cast<int32_t>((static_cast<uint32_t>(r[9]) << 16) | r[8]);
    latest.sampledMs = now;
    portEXIT_CRITICAL(&lock);
}

// Blocking request, for setup before the follower starts.
bool transact() {
    while (!busReady()) {
    }
    transmit();
    AimFrame::Status status;
    while ((status = poll()) == AimFrame::Status::Incomplete) vTaskDelay(1);
    return status == AimFrame::Status::Ok;
}

bool readTelemetry() {
    prepareRead();
    if (!transact()) return false;
    storeTelemetry();
    return true;
}

bool writeRegister(uint16_t address, uint16_t value) {
    prepareWrite(address, value);
    return transact();
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

// Switch the servo to RS-485 targets, aimed at where it is, output off until
// the follower sees ENA asking for it.
bool takeControl() {
    if (!readTelemetry() || !writeRegister(serialControlRegister, 1)) return false;
    prepareTarget(telemetry().positionCounts);
    return transact() && writeRegister(outputEnableRegister, 0) &&
           writeRegister(accelerationRegister, 60000);
}

// ── Follower: a non-blocking step every millisecond (no task stack) ─────
enum class Request : uint8_t { None, Telemetry, Target, Hold, Enable, Disable };
enum class Output : uint8_t { Off, Holding, On };

esp_timer_handle_t timer = nullptr;
Request inFlight = Request::None;
Output output = Output::Off;
uint32_t requests = 0;
uint32_t lastOkMs = 0;
int16_t lastCount = 0;
int32_t sentTarget = 0;

// A servo that stops answering cannot be steered: stop like an emergency stop.
void stopFollowing() {
    esp_timer_stop(timer);
    ESP_LOGE("HAT", "Servo stopped answering on RS-485; stopping motion");
    stepper->forceStop();
    stepper->disableOutputs();
    prepareWrite(outputEnableRegister, 0);  // best effort
    transmit();
}

void onReply(Request request) {
    switch (request) {
        case Request::Telemetry:
            storeTelemetry();
            break;
        case Request::Target:
            portENTER_CRITICAL(&lock);
            latest.targetCounts = sentTarget;
            portEXIT_CRITICAL(&lock);
            break;
        case Request::Hold:
            output = Output::Holding;
            break;
        case Request::Enable:
            output = Output::On;
            break;
        case Request::Disable:
            output = Output::Off;
            break;
        case Request::None:
            break;
    }
}

// Mirror ENA and the STEP/DIR pulse position onto the servo.
Request prepareNext() {
    const int16_t count = stepper->readPulseCounter();
    portENTER_CRITICAL(&lock);
    pulses = hat_logic::accumulatePulses(pulses, lastCount, count);
    const int32_t target = hat_logic::targetCounts(anchorCounts, pulses,
                                                   Config::Hat::countsPerPulse);
    const Telemetry now = latest;
    portEXIT_CRITICAL(&lock);
    lastCount = count;

    // ENA is active low (services/stepper.cpp).
    const bool wantOn = digitalRead(Pins::Driver::motorEnablePin) == LOW;
    const bool fresh = millis() - now.sampledMs < 20;
    if (!wantOn && output != Output::Off) {
        prepareWrite(outputEnableRegister, 0);
        return Request::Disable;
    }
    if (wantOn && output == Output::Off && fresh) {
        // Like a servo coming out of ENA: hold where it is, then follow.
        portENTER_CRITICAL(&lock);
        anchorCounts = hat_logic::anchorAt(now.positionCounts, pulses,
                                           Config::Hat::countsPerPulse);
        portEXIT_CRITICAL(&lock);
        prepareTarget(now.positionCounts);
        return Request::Hold;
    }
    if (wantOn && output == Output::Holding) {
        prepareWrite(outputEnableRegister, 1);
        return Request::Enable;
    }
    if (output == Output::Off || ++requests % telemetryEvery == 0) {
        prepareRead();
        return Request::Telemetry;
    }
    sentTarget = target;
    prepareTarget(target);
    return Request::Target;
}

void step(void *) {
    if (inFlight != Request::None) {
        const auto status = poll();
        if (status == AimFrame::Status::Incomplete) return;
        if (status == AimFrame::Status::Ok) {
            onReply(inFlight);
            lastOkMs = millis();
        }
        inFlight = Request::None;
    }
    if (millis() - lastOkMs > linkTimeoutMs) {
        stopFollowing();
        return;
    }
    if (!busReady()) return;
    inFlight = prepareNext();
    transmit();
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
    if (present &&
        !(stepper->attachToPulseCounter(pulseCounterUnit, 0, 0) &&
          takeControl())) {
        ESP_LOGE("HAT", "OSSM HAT answered but could not take control");
        present = false;
    }
    if (!present) {
        uart.end();
        ESP_LOGI("HAT", "No OSSM HAT: driving the motor with STEP/DIR");
        return false;
    }
    lastCount = stepper->readPulseCounter();
    lastOkMs = millis();
    esp_timer_create_args_t args = {};
    args.callback = step;
    args.name = "ossmHat";
    esp_timer_create(&args, &timer);
    esp_timer_start_periodic(timer, 1000);
    return true;
#endif
}

bool isPresent() { return present; }

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
