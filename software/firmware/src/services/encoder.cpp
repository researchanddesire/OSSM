#include "encoder.h"

EncoderClick encoderClick;

// Define the global encoder instance
AiEsp32RotaryEncoder encoder(
    Pins::Remote::encoderA,
    Pins::Remote::encoderB,
    Pins::Remote::encoderSwitch,
    Pins::Remote::encoderPower,
    Pins::Remote::encoderStepsPerNotch
);

void IRAM_ATTR readEncoderISR() {
    encoder.readEncoder_ISR();
}

void initEncoder() {
    // The driver's default offset is a whole notch on the two-step remote.
    // A programmatic selection must read back exactly, including menu wrap points.
    encoder.correctionOffset = 0;
    encoder.begin();
    encoder.setup(readEncoderISR);
    encoder.setBoundaries(0, 99, false);
    encoder.setAcceleration(0);
    encoder.disableAcceleration();
}
