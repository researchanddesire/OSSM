#include "pattern_controls.h"

#include "Strings.h"
#include "constants/Pins.h"
#include "services/board.h"
#include "ossm/state/settings.h"
#include "ossm/state/state.h"
#include "services/display.h"
#include "services/encoder.h"
#include "services/tasks.h"
#include "structs/SettingPercents.h"
#include "ui.h"
#include "utils/analog.h"
#include "utils/format.h"
#include "components/HeaderBar.h"

namespace sml = boost::sml;
using namespace sml;

namespace pattern_controls {

static constexpr int numberOfDescriptions =
    sizeof(ui::strings::strokeEngineDescriptions) /
    sizeof(ui::strings::strokeEngineDescriptions[0]);
static constexpr int numberOfPatterns =
    sizeof(ui::strings::strokeEngineNames) / sizeof(ui::strings::strokeEngineNames[0]);

static void drawPatternControlsTask(void *pvParameters) {
    const TaskHandle_t screenTask = xTaskGetCurrentTaskHandle();
    auto isInCorrectState = [screenTask]() {
        return screenTask == Tasks::drawPatternControlsTaskH &&
               stateMachine->is("strokeEngine.pattern"_s);
    };

    if (!isInCorrectState()) {
        vTaskDelete(nullptr);
        return;
    }

    uint32_t lastClick = encoderClick.snapshot();
    int nextPattern = constrain(static_cast<int>(settings.pattern), 0, numberOfPatterns - 1);
    bool shouldUpdateDisplay = true;

    encoder.setAcceleration(10);
    encoder.setBoundaries(0, numberOfPatterns * 3 - 1, true);
    encoder.setEncoderValue(nextPattern * 3);

    showHeaderIcons = true;

    while (isInCorrectState()) {
        float speed;
        float speedKnob = readSpeedKnobPercent();
        settings.speedKnob = speedKnob;
        if (USE_SPEED_KNOB_AS_LIMIT || !settings.speedBLE.has_value()) {
            speed = speedKnob * (settings.speedBLE.value_or(100)) / 100;
        } else {
            speedKnob = settings.speedBLE.value_or(100);
            speed = settings.speedBLE.value_or(100);
        }

        if (speed != settings.speed) {
            shouldUpdateDisplay = true;
            settings.speed = speed;
        }

        const uint32_t click = encoderClick.snapshot();
        if (!EncoderClick::isIdle(click) || !isInCorrectState()) {
            vTaskDelay(100);
            continue;
        }

        const StrokePatterns selected = settings.pattern;
        if (lastClick != click || nextPattern != static_cast<int>(selected)) {
            // A complete click or remote selection can fall between UI polls.
            nextPattern = constrain(static_cast<int>(selected), 0, numberOfPatterns - 1);
            encoder.setEncoderValue(nextPattern * 3);
            shouldUpdateDisplay = true;
            lastClick = click;
        }
        const int encoderIndex = static_cast<int>(encoder.readEncoder() / 3);
        const int encoderPattern = constrain(encoderIndex, 0, numberOfPatterns - 1);
        if (!encoderClick.unchangedAndIdle(click) || !isInCorrectState() ||
            settings.pattern != selected) {
            vTaskDelay(100);
            continue;
        }

        nextPattern = encoderPattern;
        shouldUpdateDisplay |= nextPattern != static_cast<int>(selected);
        if (!shouldUpdateDisplay) {
            vTaskDelay(100);
            continue;
        }
        settings.pattern = static_cast<StrokePatterns>(nextPattern);
        const char *patternName = ui::strings::strokeEngineNames[nextPattern];
        const char *patternDescription = nextPattern < numberOfDescriptions
            ? ui::strings::strokeEngineDescriptions[nextPattern] : ui::strings::noDescription;

        if (xSemaphoreTake(displayMutex, 100) == pdTRUE) {
            ui::TextPage page;
            page.title = patternName;
            page.body = patternDescription;
            page.scrollPercent =
                ui::scrollPercent(nextPattern, numberOfPatterns);
            ui::drawTextPage(display.getU8g2(), page);
            refreshPage(true, true);
            xSemaphoreGive(displayMutex);
            shouldUpdateDisplay = false;
        }

        vTaskDelay(200);
    }

    vTaskDelete(nullptr);
};

void drawPatternControls() {
    int stackSize = 3 * configMINIMAL_STACK_SIZE;
    xTaskCreate(drawPatternControlsTask, "drawPatternControlsTask", stackSize,
                nullptr, 1, &Tasks::drawPatternControlsTaskH);
}

}  // namespace pattern_controls
