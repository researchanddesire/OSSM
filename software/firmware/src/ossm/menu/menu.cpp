#include "menu.h"

#include <atomic>
#include <cstdio>
#include <WiFi.h>

#include "constants/Menu.h"
#include "ossm/state/menu.h"
#include "ossm/state/state.h"
#include "ossm/streaming/saved_pattern.h"
#include "services/display.h"
#include "services/encoder.h"
#include "services/radHil.h"
#include "services/tasks.h"
#include "ui.h"
#include "components/HeaderBar.h"

namespace sml = boost::sml;
using namespace sml;

namespace menu {
namespace {
constexpr int clicksPerRow = 3;
constexpr int back = -1;
std::atomic<int> highlightedSlot{back};
std::atomic<bool> writeFailed{false};

bool ownsMenu(bool chooser, TaskHandle_t task) {
    if (task != Tasks::drawMenuTaskH) return false;
    return chooser ? stateMachine->is("savedPatterns.idle"_s)
                   : stateMachine->is("menu"_s) || stateMachine->is("menu.idle"_s);
}

void drawMenuTask(void *context) {
    const bool chooser = context != nullptr;
    const auto task = xTaskGetCurrentTaskHandle();
    if (!ownsMenu(chooser, task)) {
        vTaskDelete(nullptr);
        return;
    }

    // The catalog can only change in the main menu. Keep a small name snapshot
    // for the chooser, with one explicit Back row and no waveform copies.
    char labels[streaming::patternSlotCount][streaming::maxPatternNameBytes + 5]{};
    const char *rows[streaming::patternSlotCount + 1]{};
    int slotForRow[streaming::patternSlotCount + 1]{};
    int row = menuState.currentOption;
    int count = Menu::NUM_OPTIONS;
    if (chooser) {
        const auto catalog = streaming::getPatternCatalog();
        count = 0;
        row = 0;
        for (int slot = 0; slot < streaming::patternSlotCount; ++slot) {
            if (!catalog.slots[slot].periodMs) continue;
            const bool selected = slot == catalog.selected;
            std::snprintf(labels[count], sizeof(labels[count]), "%c%c %s", selected ? '*' : ' ',
                          '1' + slot, catalog.slots[slot].name);
            if (selected) row = count;
            rows[count] = labels[count];
            slotForRow[count++] = slot;
        }
        rows[count] = count ? "Back" : "No saved patterns";
        slotForRow[count++] = back;
    }
    if (!ownsMenu(chooser, task)) {
        vTaskDelete(nullptr);
        return;
    }
    encoder.setBoundaries(0, clicksPerRow * count - 1, true);
    encoder.setAcceleration(0);
    encoder.setEncoderValue(row * clicksPerRow);
    showHeaderIcons = true;
    bool redraw = true;
    bool showedFailure = false;
    uint32_t lastClick = encoderClick.snapshot();
    wl_status_t wifi = WL_IDLE_STATUS;

    while (ownsMenu(chooser, task)) {
        if (!chooser) radHilProgress(stateMachine->is("menu.idle"_s));
        const auto click = encoderClick.snapshot();
        const auto currentWifi = WiFiClass::status();
        redraw |= wifi != currentWifi;
        wifi = currentWifi;
        if (encoderClick.unchangedAndIdle(click) && ownsMenu(chooser, task)) {
            if (click != lastClick) {
                encoder.setEncoderValue(row * clicksPerRow);
                lastClick = click;
            }
            const int candidate = encoder.readEncoder() / clicksPerRow;
            if (candidate >= 0 && candidate < count && candidate != row &&
                encoderClick.unchangedAndIdle(click) && ownsMenu(chooser, task)) {
                row = candidate;
                redraw = true;
                if (!chooser) menuState.currentOption = static_cast<Menu>(row);
                writeFailed = false;
            }
        }
        const bool failed = chooser && writeFailed.load();
        redraw |= failed != showedFailure;
        if (redraw && xSemaphoreTake(displayMutex, 100) == pdTRUE) {
            if (ownsMenu(chooser, task)) {
                if (chooser) highlightedSlot = slotForRow[row];
                const char *const *items = chooser ? rows : menuStrings;
                const char *retry[] = {"Save failed - retry"};
                showedFailure = failed;
                ui::MenuData data{failed ? retry : items, failed ? 1 : count, failed ? 0 : row};
                ui::drawMenu(display.getU8g2(), data);
                refreshPage(true, true);
                redraw = false;
            }
            xSemaphoreGive(displayMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelete(nullptr);
}
}  // namespace

void drawMenu() {
    xTaskCreate(drawMenuTask, "drawMenuTask", 5 * configMINIMAL_STACK_SIZE, nullptr,
                1, &Tasks::drawMenuTaskH);
}

void drawSavedPatterns() {
    highlightedSlot = back;
    writeFailed = false;
    xTaskCreate(drawMenuTask, "savedPatterns", 5 * configMINIMAL_STACK_SIZE, &highlightedSlot,
                1, &Tasks::drawMenuTaskH);
}

bool confirmSavedPattern() {
    const int slot = highlightedSlot.load();
    if (slot == back) return true;
    bool stored = false;
    streaming::selectPatternInMenu({slot, stored});
    writeFailed = !stored;
    if (stored) menuState.currentOption = Menu::Streaming;
    return stored;
}
}  // namespace menu
