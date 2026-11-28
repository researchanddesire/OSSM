#include "play_controls.h"
#include "ossm/streaming/drawn_pattern.h"
#include "ossm/streaming/saved_pattern.h"

#include <mutex>
#include <optional>

#include "Strings.h"
#include "constants/Menu.h"
#include "constants/Pins.h"
#include "services/board.h"
#include "ossm/state/ble.h"
#include "ossm/state/menu.h"
#include "ossm/state/session.h"
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

namespace play_controls {

static ui::PlayControl toUiPlayControl(PlayControls pc) {
    switch (pc) {
        case PlayControls::STROKE:    return ui::PlayControl::STROKE;
        case PlayControls::SENSATION: return ui::PlayControl::SENSATION;
        case PlayControls::DEPTH:     return ui::PlayControl::DEPTH;
        case PlayControls::BUFFER:    return ui::PlayControl::BUFFER;
    }
    return ui::PlayControl::STROKE;
}

static float &controlSetting(PlayControls pc) {
    switch (pc) {
        case PlayControls::STROKE:    return settings.stroke;
        case PlayControls::SENSATION: return settings.sensation;
        case PlayControls::DEPTH:     return settings.depth;
        case PlayControls::BUFFER:    return settings.buffer;
    }
    return settings.stroke;
}

// The BLE task updates the selected setting immediately. Only its screen task
// repositions the physical encoder, so a remote depth write cannot select a
// pattern or a menu item when a different screen owns the knob.
static std::mutex remoteControlMutex;
static uint32_t remoteControlRevision = 0;

static bool isWindowControl(PlayControls control) {
    return control == PlayControls::STROKE || control == PlayControls::DEPTH;
}

static void setWindow(streaming::DrawnWindowEdit &edit, PlayControls control, float value) {
    edit.set(control == PlayControls::STROKE ? value : settings.stroke,
             control == PlayControls::DEPTH ? value : settings.depth);
}

void setRemoteControl(PlayControls control, float value) {
    std::optional<streaming::DrawnWindowEdit> edit;
    if (isWindowControl(control)) edit.emplace();
    std::lock_guard<std::mutex> lock(remoteControlMutex);
    session.playControl = control;
    if (edit) setWindow(*edit, control, value);
    else controlSetting(control) = value;
    ++remoteControlRevision;
}

static void drawPlayControlsTask(void *pvParameters) {
    const TaskHandle_t screenTask = xTaskGetCurrentTaskHandle();
    auto isInCorrectState = [screenTask]() {
        if (screenTask != Tasks::drawPlayControlsTaskH) return false;
        return stateMachine->is("simplePenetration"_s) ||
               stateMachine->is("simplePenetration.idle"_s) ||
               stateMachine->is("strokeEngine"_s) ||
               stateMachine->is("strokeEngine.idle"_s) ||
               stateMachine->is("streaming"_s) ||
               stateMachine->is("streaming.idle"_s);
    };

    if (!isInCorrectState()) {
        vTaskDelete(nullptr);
        return;
    }

    encoder.setAcceleration(10);
    encoder.setBoundaries(0, 100, false);
    uint32_t lastRemoteControl;
    {
        std::lock_guard<std::mutex> lock(remoteControlMutex);
        encoder.setEncoderValue(controlSetting(session.playControl));
        lastRemoteControl = remoteControlRevision;
    }

    SettingPercents next = {0, 0, 0, 0, 0};
    unsigned long displayLastUpdated = 0;

    bool isStrokeEngine = stateMachine->is("strokeEngine"_s) ||
                          stateMachine->is("strokeEngine.idle"_s);

    bool isStreaming = stateMachine->is("streaming"_s) ||
                       stateMachine->is("streaming.idle"_s);

    bool shouldUpdateDisplay = false;
    uint32_t lastClick = encoderClick.snapshot();

    showHeaderIcons = true;
    vTaskDelay(100);

    while (isInCorrectState()) {
        shouldUpdateDisplay = false;
        const uint32_t click = encoderClick.snapshot();

#ifdef AJ_DEVELOPMENT_HARDWARE
        next.speedKnob = 0;
#else
        next.speedKnob = readSpeedKnobPercent();
#endif
        if (abs(next.speedKnob - settings.speedKnob) > 2 &&
            next.speedKnob < settings.speed) {
            ::resetLastSpeedCommandWasFromBLE();
        }

        next.speed = next.speedKnob;
        if (settings.speedBLE.has_value()) {
            if (USE_SPEED_KNOB_AS_LIMIT) {
                next.speed = next.speedKnob * settings.speedBLE.value() / 100;
            } else if (::wasLastSpeedCommandFromBLE()) {
                next.speed = settings.speedBLE.value();
            }
        }

        if (next.speed != settings.speed) {
            shouldUpdateDisplay = true;
            settings.speed = next.speed;
        }

        settings.speedKnob = next.speedKnob;

        // A click can turn the knob and transfer the encoder to another screen
        // while the speed-pot read is in progress. Restore the current setting
        // after a click, and commit the sample only while its control owns it.
        PlayControls loopControl;
        std::optional<float> windowValue;
        {
            std::lock_guard<std::mutex> lock(remoteControlMutex);
            loopControl = session.playControl;
            if (encoderClick.unchangedAndIdle(click) && isInCorrectState()) {
                float &value = controlSetting(loopControl);
                if (lastClick != click || lastRemoteControl != remoteControlRevision) {
                    encoder.setEncoderValue(value);
                    lastClick = click;
                    lastRemoteControl = remoteControlRevision;
                    shouldUpdateDisplay = true;
                }
                const auto encoderValue = encoder.readEncoder();
                if (encoderClick.unchangedAndIdle(click) && isInCorrectState() &&
                    session.playControl == loopControl) {
                    shouldUpdateDisplay |= value != encoderValue;
                    if (isWindowControl(loopControl)) {
                        if (value != encoderValue) windowValue = encoderValue;
                    } else {
                        value = encoderValue;
                    }
                }
            }
        }
        if (windowValue) {
            // A BLE write or click may win while acquiring the mailbox.
            // Revalidate the sample under the knob lock in that same order.
            streaming::DrawnWindowEdit edit;
            std::lock_guard<std::mutex> lock(remoteControlMutex);
            if (remoteControlRevision == lastRemoteControl && session.playControl == loopControl &&
                encoderClick.unchangedAndIdle(click) && isInCorrectState())
                setWindow(edit, loopControl, *windowValue);
        }

        shouldUpdateDisplay =
            shouldUpdateDisplay || millis() - displayLastUpdated > 1000;

        if (!shouldUpdateDisplay) {
            vTaskDelay(100);
            continue;
        }

        displayLastUpdated = millis();

        if (xSemaphoreTake(displayMutex, 100) == pdTRUE) {
            const char *headerText;
            if (isStrokeEngine) {
                headerText =
                    ui::strings::strokeEngineNames[(int)settings.pattern];
            } else if (isStreaming) {
                headerText = ui::strings::streaming;
                if (streaming::drawnPatternActive())
                    headerText = "Pattern: 2x stop";
                else if (streaming::hasSavedPattern())
                    headerText = "Saved: 2x play";
            } else {
                headerText = ui::strings::simplePenetration;
            }

            static String distStr;
            static String timeStr;
            if (!isStrokeEngine && !isStreaming) {
                distStr = formatDistance(session.distanceMeters);
            }
            if (!isStreaming) {
                timeStr = formatTime(displayLastUpdated - session.startTime);
            }

            ui::PlayControlsData data{};
            data.speed = next.speed;
            data.stroke = settings.stroke;
            data.sensation = settings.sensation;
            data.depth = settings.depth;
            data.buffer = settings.buffer;
            data.activeControl = toUiPlayControl(loopControl);
            data.strokeCount = session.strokeCount;
            data.distanceMeters = session.distanceMeters;
            data.elapsedMs = displayLastUpdated - session.startTime;
            data.pattern = (int)settings.pattern;
            data.isStrokeEngine = isStrokeEngine;
            data.isStreaming = isStreaming;
            data.headerText = headerText;
            data.speedLabel = ui::strings::speed;
            data.strokeLabel = ui::strings::stroke;
            data.distanceStr =
                (!isStrokeEngine && !isStreaming) ? distStr.c_str() : nullptr;
            data.timeStr = !isStreaming ? timeStr.c_str() : nullptr;

            ui::drawPlayControls(display.getU8g2(), data);
            refreshPage(true, true);
            xSemaphoreGive(displayMutex);
        }

        vTaskDelay(200);
    }

    vTaskDelete(nullptr);
};

void drawPlayControls() {
    int stackSize = 3 * configMINIMAL_STACK_SIZE;
    xTaskCreate(drawPlayControlsTask, "drawPlayControlsTask", stackSize,
                nullptr, 1, &Tasks::drawPlayControlsTaskH);
}

}  // namespace play_controls
