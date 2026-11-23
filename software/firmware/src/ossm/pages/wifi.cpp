#include "wifi.h"

#include <WiFi.h>
#include <esp_heap_caps.h>
#include <esp_log.h>

#include "Strings.h"
#include "ossm/Events.h"
#include "ossm/state/state.h"
#include "components/HeaderBar.h"
#include "services/display.h"
#include "services/tasks.h"
#include "services/wm.h"
#include "ui.h"

namespace sml = boost::sml;
using namespace sml;

namespace pages {

namespace {

constexpr int setupNetworkAttempts = 3;

bool setupNetworkRunning() {
    return (WiFi.getMode() & WIFI_MODE_AP) != 0 &&
           WiFi.softAPIP() != IPAddress(0, 0, 0, 0);
}

void logInternalRam(const char *what) {
    ESP_LOGW("WiFi", "%s: internal free=%u largest=%u", what,
             static_cast<unsigned>(
                 heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(
                 MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
}

void serviceSetupPortal(void *) {
    auto isInCorrectState = []() {
        return stateMachine->is("wifi"_s) || stateMachine->is("wifi.idle"_s);
    };

    while (isInCorrectState()) {
        wm.process();

        if (WiFiClass::status() == WL_CONNECTED) {
            stateMachine->process_event(Done{});
            break;
        }

        vTaskDelay(50);
    }

    // Only stop portal if user left the screen, not on successful connection
    if (WiFiClass::status() != WL_CONNECTED) {
        wm.stopConfigPortal();
    }
    Tasks::wmTaskH = nullptr;
    vTaskDelete(nullptr);
}

// WiFiManager reports success even when the access point failed to start
// (it ignores startAP()'s result), and the portal task used to be created
// once with no retry. Under internal-RAM pressure either step could fail and
// leave the screen asking for "OSSM Setup" with nothing broadcasting.
void startSetupPortal() {
    for (int attempt = 0; attempt < setupNetworkAttempts; attempt++) {
        if (attempt > 0) {
            logInternalRam("OSSM Setup network did not start; retrying");
            wm.stopConfigPortal();
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        wm.startConfigPortal(ui::strings::ossmSetup);
        if (setupNetworkRunning()) break;
    }
    if (!setupNetworkRunning()) {
        logInternalRam("OSSM Setup network failed to start");
    }

    if (!Tasks::startOperation(serviceSetupPortal, "wmProcessTask",
                               4 * configMINIMAL_STACK_SIZE, &Tasks::wmTaskH)) {
        // Without the service task the setup page can never load.
        wm.stopConfigPortal();
    }
}

}  // namespace

void drawWiFi() {
    if (xSemaphoreTake(displayMutex, 200) == pdTRUE) {
        bool isConnected = WiFiClass::status() == WL_CONNECTED;

        if (!isConnected) {
            showHeaderIcons = false;
            ui::drawTextPage(display.getU8g2(),
                             ui::pages::wifiDisconnectedPage);
        } else {
            showHeaderIcons = true;
            ui::drawTextPage(display.getU8g2(),
                             ui::pages::wifiConnectedPage);
        }
        refreshPage(true, true);
        xSemaphoreGive(displayMutex);
    }

    // Only start portal if not already connected
    if (WiFiClass::status() != WL_CONNECTED) {
        wm.setConfigPortalBlocking(false);
        wm.setCleanConnect(true);
        wm.setConnectTimeout(30);
        wm.setConnectRetries(5);
        startSetupPortal();
    }
}

}  // namespace pages
