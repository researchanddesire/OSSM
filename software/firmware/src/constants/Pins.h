#ifndef OSSM_SOFTWARE_PINS_CPP
#define OSSM_SOFTWARE_PINS_CPP

/**
 * Pin definitions for the OSSM
 *
 * These are all the hardware pins used by the OSSM.
 * These include pin definitions for Drivers, Buttons and Remotes
 *
 * To reference a pin, import this file and use the Pins namespace as follows:
 *
 * ```cpp
 * #include "constants/pins.cpp"
 *
 * Pins::button_in
 * Pins::Driver::motorStepPin
 * Pins::Homing::stopPin
 *
 * ```
 */

#ifdef OSSM_ALT
/**
 * OSSM ALT Edition v3.4 (github.com/jollydodo/OSSM-ALT-Edition): an ESP32-S3
 * board with the servo's STEP/DIR and an SP3485 RS-485 transceiver on board.
 * Taken from its KiCad netlist (OSSM_ALT_SERVO_V3_4). It has no enable line,
 * no limit/stop switches and no wired remote: those inputs sit on unconnected
 * GPIOs held low, and the speed knob reads 0.
 */
namespace Pins {

    namespace Display {
        constexpr int oledReset = -1;
        constexpr int ledPin = 38;  // WS2812 status LED
    }

    namespace Driver {
        constexpr int currentSensorPin = 6;    // CSENSE
        constexpr int motorStepPin = 4;        // IO_PUL
        constexpr int motorDirectionPin = 1;   // IO_DIR
        constexpr int motorEnablePin = 7;      // not connected: no ENA line
        constexpr int stopPin = 47;            // not connected
        constexpr int limitSwitchPin = 48;     // not connected
    }

    namespace Wifi {
        constexpr int resetPin = 2;  // not connected
    }

    // IO header.
    namespace GPIO {
        constexpr int pin1 = 13;
        constexpr int pin2 = 14;
        constexpr int pin3 = 17;
        constexpr int pin4 = 21;
    }

    // On-board RS-485 to the servo, used like an OSSM HAT.
    namespace Hat {
        constexpr int de = 11;  // RE/DE (high = transmit)
        constexpr int tx = 10;  // DI
        constexpr int rx = 12;  // RO
    }

    namespace Remote {
        constexpr bool hasSpeedKnob = false;
        constexpr int speedPotPin = 5;     // not connected
        constexpr int encoderSwitch = 18;  // not connected
        constexpr int encoderA = 15;       // not connected
        constexpr int encoderB = 16;       // not connected
        constexpr int encoderPower = -1;
        constexpr int displayData = 8;     // I2C header SDA
        constexpr int displayClock = 9;    // I2C header SCL
        constexpr int encoderStepsPerNotch = 2;
    }
}
#else
namespace Pins {

    namespace Display {
        // This pin is used by u8g2 to reset the display.
        // Use -1 if this is shared with the board reset pin.
        constexpr int oledReset = -1;

        // Pin used by RGB LED
        constexpr int ledPin = 25;
    }

    namespace Driver {
        constexpr int currentSensorPin = 36;

        // Pin that pulses on servo/stepper steps - likely labelled PUL on
        // drivers.
        constexpr int motorStepPin = 14;
        // Pin connected to driver/servo step direction - likely labelled DIR on
        // drivers. N.b. to iHSV57 users - DIP switch #5 can be flipped to
        // invert motor direction entirely
        constexpr int motorDirectionPin = 27;
        // Pin for motor enable - likely labelled ENA on drivers.
        constexpr int motorEnablePin = 26;

        // define the IO pin the emergency stop switch is connected to
        constexpr int stopPin = 19;
        // define the IO pin where the limit(homingStart) switch(es) are
        // connected to (switches in series in normally open setup) Switches
        // wired from IO pin to ground.
        constexpr int limitSwitchPin = 12;
    }

    namespace Wifi {
        // Pin for Wi-Fi reset button (optional)
        constexpr int resetPin = 23;
    }

    namespace GPIO {
        constexpr int pin1 = 2;
        constexpr int pin2 = 15;
        constexpr int pin3 = 22;
        constexpr int pin4 = 33;
    }

    // OSSM HAT on the expansion header (J9): RS-485 to the servo.
    namespace Hat {
        constexpr int de = GPIO::pin1;  // IO2, driver enable (high = transmit)
        constexpr int tx = GPIO::pin3;  // IO22
        constexpr int rx = GPIO::pin4;  // IO33
    }

    /** These are configured for the OSSM Remote - which has a screen, a
     * potentiometer and an encoder which clicks*/
    namespace Remote {

        constexpr bool hasSpeedKnob = true;
        constexpr int speedPotPin = 34;

        // This switch occurs when you press the right button in.
        // With the current state of the code this will send out a "ButtonPress"
        // event automatically.
        constexpr int encoderSwitch = 35;

        // The rotary encoder requires at least two pins to function.
        constexpr int encoderA = 18;
        constexpr int encoderB = 5;
        constexpr int encoderPower =
            -1; /* Put -1 of Rotary encoder Vcc is connected directly to 3,3V;
                   else you can use declared output pin for powering rotary
                   encoder */

#ifdef AJ_DEVELOPMENT_HARDWARE
        constexpr int displayData = 21;
        constexpr int displayClock = 22;
#else
        constexpr int displayData = 21;
        constexpr int displayClock = 19;
#endif
        constexpr int encoderStepsPerNotch = 2;
    }
}
#endif  // OSSM_ALT

#endif  // OSSM_SOFTWARE_PINS_CPP
