#include "services/serialIdentity.h"
#include "board.h"
#include "services/ossmHat.h"
#include "services/motorSafety.h"

bool USE_LATENCY_COMPENSATION = false;
bool USE_SPEED_KNOB_AS_LIMIT = true;

void initBoard() {
    configureSerialIdentityUsb();
    Serial.begin(115200);
    startSerialIdentity();

    pinMode(Pins::Remote::encoderSwitch,
            INPUT_PULLDOWN);  // Rotary Encoder Pushbutton

    // The V2.3 HAT shares J9 with expansion GPIO. Never drive its RX line,
    // and keep DE low until the UART has taken ownership.
    digitalWrite(Pins::GPIO::pin1, LOW);
    pinMode(Pins::GPIO::pin1, OUTPUT);
    pinMode(Pins::GPIO::pin2, OUTPUT);
    pinMode(Pins::GPIO::pin3, INPUT);
    pinMode(Pins::GPIO::pin4, INPUT);

    digitalWrite(Pins::Driver::motorEnablePin, HIGH);
    pinMode(Pins::Driver::motorEnablePin, OUTPUT);
    digitalWrite(Pins::Driver::motorEnablePin, HIGH);
    pinMode(Pins::Wifi::resetPin, INPUT_PULLDOWN);
    pinMode(Pins::Remote::speedPotPin, INPUT);
    adcAttachPin(Pins::Remote::speedPotPin);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);  // allows us to read almost full 3.3V range
    initStepper();
    initOssmHat();
    initMotorSafety();
    initEncoder();
    initLED();
}
