#ifndef OSSM_GPIO_HPP
#define OSSM_GPIO_HPP

#include "Arduino.h"
#include "NimBLECharacteristic.h"
#include "NimBLEService.h"
#include "NimBLEUUID.h"
#include "constants/LogTags.h"
#include "constants/Pins.h"
#include "services/motor.h"

// Simple helper to map 1-4 to actual board pins
static int mapGpioIndexToPin(int index) {
    switch (index) {
        case 1:
            return Pins::GPIO::pin1;
        case 2:
            return Pins::GPIO::pin2;
        case 3:
            return Pins::GPIO::pin3;
        case 4:
            return Pins::GPIO::pin4;
        default:
            return -1;
    }
}

class GPIOCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic,
                 NimBLEConnInfo& /*connInfo*/) override {
        std::string raw = pCharacteristic->getValue();
        String input = String(raw.c_str());
        input.trim();

        // "<index>:<low|high|0|1>", checked by hand: std::regex allocates and
        // aborts the firmware when internal RAM runs low.
        const int colon = input.indexOf(':');
        String indexText = colon > 0 ? input.substring(0, colon) : String();
        indexText.trim();
        String levelText = colon > 0 ? input.substring(colon + 1) : String();
        levelText.toLowerCase();
        bool wellFormed = indexText.length() > 0 &&
                          (levelText == "low" || levelText == "high" ||
                           levelText == "0" || levelText == "1");
        for (unsigned int i = 0; wellFormed && i < indexText.length(); i++) {
            wellFormed = isDigit(indexText[i]);
        }

        if (!wellFormed) {
            static const char err[] PROGMEM = "error:invalid_format";
            ESP_LOGW(NIMBLE_TAG, "GPIO write invalid format: %s", raw.c_str());
            pCharacteristic->setValue(String(FPSTR(err)));
            return;
        }

        int index = indexText.toInt();
        String stateStr = levelText;

        int targetPin = mapGpioIndexToPin(index);
        if (targetPin < 0 || motor::reservesPin(targetPin)) {
            static const char err[] PROGMEM = "error:pin_out_of_range";
            ESP_LOGW(NIMBLE_TAG, "GPIO write out of range index: %d", index);
            pCharacteristic->setValue(String(FPSTR(err)));
            return;
        }
        if (motor::reservesPin(targetPin)) {
            pCharacteristic->setValue("error:pin_reserved_for_rs485");
            return;
        }

        int level = (stateStr == "high" || stateStr == "1") ? HIGH : LOW;
        digitalWrite(targetPin, level);

        // ESP_LOGD(NIMBLE_TAG, "GPIO set pin%d (GPIO %d) to %s", index,
        // targetPin,
        //          level == HIGH ? "HIGH" : "LOW");

        // Respond with normalized acknowledgement
        String ack =
            String("ok:") + index + ":" + (level == HIGH ? "high" : "low");
        pCharacteristic->setValue(ack);
    }

    void onRead(NimBLECharacteristic* pCharacteristic,
                NimBLEConnInfo& /*connInfo*/) override {
        // Return simple capabilities hint
        pCharacteristic->setValue(motor::reservesPin(Pins::GPIO::pin1)
                                      ? "pins:[2];reserved:rs485"
                                      : "pins:[1,2,3,4]");
    }
} inline gpioCallbacks;

inline NimBLECharacteristic* initGPIOCharacteristic(NimBLEService* pService,
                                             NimBLEUUID uuid) {
    NimBLECharacteristic* pChar = pService->createCharacteristic(
        uuid, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ);
    pChar->setCallbacks(&gpioCallbacks);
    // Seed with capabilities text on init
    static const char caps[] PROGMEM = "pins:[1,2,3,4]";
    pChar->setValue(String(FPSTR(caps)));
    return pChar;
}

#endif  // OSSM_GPIO_HPP
