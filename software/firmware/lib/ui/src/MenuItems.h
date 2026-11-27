#ifndef UI_MENU_ITEMS_H
#define UI_MENU_ITEMS_H

#include "Strings.h"

enum Menu {
    SimplePenetration,
    StrokeEngine,
    Streaming,
    Pairing,
    UpdateOSSM,
    WiFiSetup,
    Help,
    Restart,
    SavedPatterns,
    NUM_OPTIONS
};

static const char* menuStrings[Menu::NUM_OPTIONS] = {
    ui::strings::simplePenetration, ui::strings::strokeEngine,
    ui::strings::streaming,         ui::strings::pairing,
    ui::strings::update,            ui::strings::wifiSetup,
    ui::strings::helpTitle,         ui::strings::restart,
    ui::strings::savedPatterns,
};

#endif  // UI_MENU_ITEMS_H
