#include "services/serialIdentity.h"
#include "board.h"

#include "services/motor.h"
#include "services/ossmHat.h"
#include "services/motorSafety.h"
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

#if OSSM_HAT_SERIAL_BACKEND
    digitalWrite(Pins::Hat::direction, LOW);
    pinMode(Pins::Hat::direction, OUTPUT);
    pinMode(Pins::Hat::rx, INPUT);
    pinMode(Pins::Hat::tx, INPUT);
    digitalWrite(Pins::Driver::motorEnablePin, HIGH);
#endif
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
    initMotorSafety();
    // A HAT owns IO2/IO22/IO33; otherwise the expansion pins are outputs.
    const int expansionPins[] = {Pins::GPIO::pin1, Pins::GPIO::pin2,
                                 Pins::GPIO::pin3, Pins::GPIO::pin4};
    for (int pin : expansionPins) {
        if (!motor::reservesPin(pin)) pinMode(pin, OUTPUT);
    }
    initEncoder();
    initLED();
}
