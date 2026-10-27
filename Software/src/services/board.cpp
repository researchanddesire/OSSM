#include "services/serialIdentity.h"
#include "board.h"

#include "services/motor.h"
#include "utils/analog.h"

bool USE_LATENCY_COMPENSATION = false;
// Without a knob, a BLE speed is absolute rather than capped by the knob.
bool USE_SPEED_KNOB_AS_LIMIT = Pins::Remote::hasSpeedKnob;

float readSpeedKnobPercent() {
    if (!Pins::Remote::hasSpeedKnob) return 0.0f;
    return getAnalogAveragePercent(SampleOnPin{Pins::Remote::speedPotPin, 50});
}

void initBoard() {
    configureSerialIdentityUsb();
    Serial.begin(115200);
    startSerialIdentity();

    pinMode(Pins::Remote::encoderSwitch,
            INPUT_PULLDOWN);  // Rotary Encoder Pushbutton

    pinMode(Pins::Driver::motorEnablePin, OUTPUT);
    pinMode(Pins::Wifi::resetPin, INPUT_PULLDOWN);
    if (Pins::Remote::hasSpeedKnob) {
        pinMode(Pins::Remote::speedPotPin, INPUT);
        adcAttachPin(Pins::Remote::speedPotPin);
    }

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);  // allows us to read almost full 3.3V range
    initStepper();
    motor::begin();
    // A HAT owns IO2/IO22/IO33; otherwise the expansion pins are outputs.
    if (!motor::hasHat()) {
        pinMode(Pins::GPIO::pin1, OUTPUT);
        pinMode(Pins::GPIO::pin2, OUTPUT);
        pinMode(Pins::GPIO::pin3, OUTPUT);
        pinMode(Pins::GPIO::pin4, OUTPUT);
    }
    initEncoder();
    initLED();
}
