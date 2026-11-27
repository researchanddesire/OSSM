#include "ble.h"

#include "settings.h"

BLEState bleState;

void writeBleSpeed(float value, bool selectBleSource) {
    settings.speedBLE = value;
    if (selectBleSource) bleState.lastSpeedCommandWasFromBLE = true;
}
