#include "rad_ble.h"

#include <Preferences.h>
#include <WiFi.h>

#include "FirmwareProvenance.h"
#include "constants/Pins.h"
#include "constants/UserConfig.h"
#include "constants/Version.h"
#include "ossm/Events.h"
#include "ossm/OSSM.h"
#include "ossm/state/actions.h"
#include "ossm/state/calibration.h"
#include "ossm/state/motion.h"
#include "ossm/state/settings.h"
#include "ossm/state/state.h"
#include "services/encoder.h"
#include "services/board.h"
#include "services/communication/nimble.h"
#include "services/led.h"
#include "services/stepper.h"
#include "services/motorFeedback.h"
#include "services/ossmHat.h"
#include "services/motorSafety.h"

#ifndef FIRMWARE_BUILD_SHA
#define FIRMWARE_BUILD_SHA "unknown"
#endif

namespace {

constexpr uint16_t R = radble::RESOURCE_READABLE |
                       radble::RESOURCE_AVAILABLE;
constexpr uint16_t RW = R | radble::RESOURCE_WRITABLE |
                        radble::RESOURCE_LEASE_REQUIRED;
constexpr uint16_t RWS = RW | radble::RESOURCE_SAFETY_CRITICAL;
constexpr uint16_t RS = R | radble::RESOURCE_STREAMABLE;
constexpr uint16_t RWT = RW | radble::RESOURCE_STREAMABLE;
constexpr uint16_t RWST = RWS | radble::RESOURCE_STREAMABLE;
constexpr uint16_t RWP = RW | radble::RESOURCE_PERSISTENT;

const radble::Resource RESOURCES[] = {
    {"essential", "essential.live", "essential", "object", "", RS,
     "{\"characteristic\":\"2010\",\"changeDriven\":true,\"maxRateHz\":4}"},
    {"enter", "button.enter", "button", "bool", "", R,
     "{\"events\":[\"click\",\"double\",\"long\"]}"},
    {"encoder", "encoder.main", "encoder", "int", "ticks", RWT,
     "{\"min\":0,\"max\":100}"},
    {"ble_connections", "connectivity.bleConnections", "connectivity", "int",
     "connections", RS, "{\"min\":0}"},
    {"speed_knob", "analog.speedKnob", "analog", "int", "raw", RS, ""},
    {"speed_knob_percent", "analog.speedKnobPercent", "analog", "float",
     "percent", RS, "{\"min\":0,\"max\":100}"},
    {"motor_current", "analog.motorCurrent", "analog", "int", "raw", RS,
     ""},
    {"motor_current_filtered", "analog.motorCurrentFiltered", "analog",
     "float", "raw", RS, ""},
    {"motor_current_amps", "motor.currentAmps", "motor", "float", "A", RS, ""},
    {"motor_voltage_volts", "motor.voltageVolts", "motor", "float", "V", RS, ""},
    {"ossm_hat", "motor.hat", "motor", "object", "", RS, ""},
    {"motor_guard", "motor.guard", "motor", "object", "", RS, ""},
    {"motor_guard_control", "motor.guardControl", "motor", "object", "", RS, ""},
    {"motor_guard_fault", "motor.guardFault", "motor", "object", "", RS, ""},
    {"guard_reviewed_reset", "target.guardReset", "target", "event", "", RWS,
     "{\"requiresReviewed\":true}"},
    {"guard_external_control", "target.guardPrepare", "target", "event", "", RWS,
     "{\"requiresReviewed\":true,\"enablesOutput\":false}"},
    {"emergency_stop", "button.emergencyStop", "button", "bool", "", RS,
     ""},
    {"limit", "button.limit", "button", "bool", "", RS, ""},
    {"gpio1", "analog.expansion1", "analog", "int", "raw", RWT, ""},
    {"gpio2", "analog.expansion2", "analog", "int", "raw", RWT, ""},
    {"gpio3", "analog.expansion3", "analog", "int", "raw", RWT, ""},
    {"gpio4", "analog.expansion4", "analog", "int", "raw", RWT, ""},
    {"position", "motion.position", "motion", "float", "mm", RWST,
     "{\"min\":0,\"max\":100}"},
    {"homed", "motion.homed", "motion", "bool", "", RS, ""},
    {"speed", "motion.speed", "setting", "int", "percent", RWST,
     "{\"min\":0,\"max\":100}"},
    {"stroke", "motion.stroke", "setting", "int", "percent", RWT,
     "{\"min\":0,\"max\":100}"},
    {"depth", "motion.depth", "setting", "int", "percent", RWT,
     "{\"min\":0,\"max\":100}"},
    {"sensation", "motion.sensation", "setting", "int", "percent", RWT,
     "{\"min\":0,\"max\":100}"},
    {"buffer", "motion.buffer", "setting", "int", "percent", RWT,
     "{\"min\":0,\"max\":100}"},
    {"pattern", "motion.pattern", "setting", "int", "index", RWT,
     "{\"min\":0,\"max\":6}"},
    {"speed_ble", "motion.speedBle", "setting", "float", "percent", RW,
     "{\"min\":0,\"max\":100}"},
    {"speed_limit", "setting.speedKnobAsLimit", "setting", "bool", "", RW,
     ""},
    {"latency", "setting.latencyCompensation", "setting", "bool", "", RW,
     ""},
    {"metric", "setting.displayMetric", "setting", "bool", "", RW, ""},
    {"home_position", "setting.afterHomingPosition", "setting", "float", "mm",
     RW, "{\"min\":0,\"max\":100}"},
    {"mqtt_rate", "setting.mqttPublishFrequency", "setting", "float", "Hz",
     RW, "{\"min\":1,\"max\":100}"},
    {"device_name", "device.name", "setting", "string", "", RWP,
     "{\"maxBytes\":24,\"emptyResets\":true}"},
    {"firmware_provenance", "device.firmwareProvenance", "setting", "object",
     "", R, ""},
    {"calibration_offset", "motion.currentOffset", "motion", "float", "raw",
     R, ""},
    {"calibration_stroke", "motion.measuredStroke", "motion", "float", "steps",
     R, ""},
    {"target_position", "motion.targetPosition", "motion", "float", "percent",
     RS, ""},
    {"target_time", "motion.targetTime", "motion", "int", "ms", RS, ""},
    {"session_strokes", "session.strokeCount", "motion", "int", "strokes", RS,
     ""},
    {"session_distance", "session.distance", "motion", "float", "m", RS,
     ""},
    {"led", "indicator.status", "indicator", "object", "rgb", RW, ""},
    {"wifi", "connectivity.wifi", "connectivity", "object", "", RS, ""},
    {"menu", "target.menu", "target", "event", "", RW, ""},
    {"simple", "target.simplePenetration", "target", "event", "", RWS,
     ""},
    {"stroke_engine", "target.strokeEngine", "target", "event", "", RWS,
     ""},
    {"streaming", "target.streaming", "target", "event", "", RWS, ""},
    {"pairing", "target.pairing", "target", "event", "", RW, ""},
    {"wifi_setup", "target.wifi", "target", "event", "", RW, ""},
    {"update", "target.update", "target", "event", "", RW, ""},
    {"help", "target.help", "target", "event", "", RW, ""},
    {"restart", "target.restart", "target", "event", "", RW, ""},
    {"home", "target.home", "target", "event", "", RWS, ""},
    {"emergency", "target.emergencyStop", "target", "event", "", RWS, ""},
    {"return_to_menu", "event.returnToMenu", "event", "event", "", RW,
     ""},
    {"done", "event.done", "event", "event", "", RW, ""},
    {"error", "event.error", "event", "event", "", RW, ""},
    {"home_event", "event.home", "event", "event", "", RWS, ""},
    {"emergency_event", "event.emergencyStop", "event", "event", "", RWS,
     ""},
    {"update_unavailable", "event.updateUnavailable", "event", "event", "",
     RW, ""},
};

String currentStateName() {
    JsonDocument current;
    deserializeJson(current, ossm->getCurrentState());
    return current["state"] | "";
}

bool returnToMenuSafely() {
    if (currentStateName().startsWith("menu")) return true;
    return stateMachine->process_event(ReturnToMenu{}) &&
           currentStateName().startsWith("menu");
}

bool selectMenuTarget(Menu target) {
    if (!returnToMenuSafely()) return false;
    menuState.currentOption = target;
    return stateMachine->process_event(ButtonPress{});
}

void requestRestartTask(void*) {
    vTaskDelay(pdMS_TO_TICKS(500));
    selectMenuTarget(Menu::Restart);
    vTaskDelete(nullptr);
}

void requestNetworkOtaTask(void*) {
    vTaskDelay(pdMS_TO_TICKS(500));
    if (stateMachine != nullptr) {
        stateMachine->process_event(ReturnToMenu{});
        vTaskDelay(pdMS_TO_TICKS(20));
        menuState.currentOption = Menu::UpdateOSSM;
        stateMachine->process_event(ButtonPress{});
    }
    vTaskDelete(nullptr);
}

radble::Result settingValue(const String& path) {
    JsonDocument document;
    if (path == "motion.speed")
        document["value"] = settings.speed;
    else if (path == "motion.stroke")
        document["value"] = settings.stroke;
    else if (path == "motion.depth")
        document["value"] = settings.depth;
    else if (path == "motion.sensation")
        document["value"] = settings.sensation;
    else if (path == "motion.buffer")
        document["value"] = settings.buffer;
    else if (path == "motion.pattern")
        document["value"] = static_cast<int>(settings.pattern);
    else if (path == "motion.speedBle") {
        document["available"] = settings.speedBLE.has_value();
        if (settings.speedBLE.has_value()) document["value"] = *settings.speedBLE;
    } else if (path == "setting.speedKnobAsLimit")
        document["value"] = USE_SPEED_KNOB_AS_LIMIT;
    else if (path == "setting.latencyCompensation")
        document["value"] = USE_LATENCY_COMPENSATION;
    else if (path == "setting.displayMetric")
        document["value"] = UserConfig::displayMetric;
    else if (path == "setting.afterHomingPosition")
        document["value"] = UserConfig::afterHomingPosition;
    else if (path == "setting.mqttPublishFrequency")
        document["value"] = UserConfig::mqttPublishFrequencyHz;
    else if (path == "device.name") {
        Preferences preferences;
        preferences.begin("UserConfig", true);
        document["value"] = preferences.getString("DeviceName", "OSSM");
        preferences.end();
    } else if (path == "device.firmwareProvenance") {
        const auto snapshot = firmware::provenance::runningSnapshot(
            FIRMWARE_TRACK, "ossm", VERSION, FIRMWARE_BUILD_SHA);
        document["origin"] = snapshot.origin;
        document["keyId"] = snapshot.keyId;
        document["provenanceId"] = snapshot.provenanceId;
        document["imageSha256"] = snapshot.imageSha256;
        document["compactJws"] = snapshot.token;
    }
    else
        return radble::Result::failure("unknown_path", "Unknown setting path");
    document["path"] = path;
    String output;
    serializeJson(document, output);
    return radble::Result::success(output);
}

radble::Result handleCommand(JsonObjectConst request, void*) {
    if (ossm == nullptr || stateMachine == nullptr)
        return radble::Result::failure("not_ready", "OSSM is still starting");

    const String operation = request["op"] | "";
    const String path = request["path"] | "";
    const JsonObjectConst args = request["args"].as<JsonObjectConst>();

    if (operation == "ota.start" && String(args["transport"] | "") == "wifi") {
        if (WiFi.status() != WL_CONNECTED)
            return radble::Result::failure("network_failed", "Wi-Fi is disconnected");
        JsonDocument current;
        deserializeJson(current, ossm->getCurrentState());
        const String state = current["state"] | "";
        const bool canReturnToMenu =
            state == "menu.idle" || state.startsWith("simplePenetration") ||
            state.startsWith("strokeEngine") || state.startsWith("streaming");
        if (!canReturnToMenu)
            return radble::Result::failure(
                "invalid_state", "OSSM is not ready to enter network OTA");
        if (xTaskCreate(requestNetworkOtaTask, "rad-net-ota", 2048, nullptr, 1,
                        nullptr) != pdPASS)
            return radble::Result::failure("busy", "Could not schedule network OTA");
        return radble::Result::success(R"({"transport":"wifi","requested":true})");
    }

    if (operation == "setting.read") return settingValue(path);

    if (operation == "sensor.read") {
        JsonDocument document;
        document["path"] = path;
        if (path.startsWith("analog.expansion")) {
            const int index = path.substring(strlen("analog.expansion")).toInt();
            const int pins[] = {Pins::GPIO::pin1, Pins::GPIO::pin2,
                                Pins::GPIO::pin3, Pins::GPIO::pin4};
            if (index >= 1 && index <= 4 && ossmHatOwnsPin(pins[index - 1]))
                return radble::Result::failure("pin_reserved", "GPIO is reserved for RS485");
        }
        if (path == "analog.speedKnob")
            document["value"] = analogRead(Pins::Remote::speedPotPin);
        else if (path == "analog.speedKnobPercent")
            document["value"] =
                analogRead(Pins::Remote::speedPotPin) * 100.0f / 4095.0f;
        else if (path == "analog.motorCurrent")
            document["value"] = analogRead(Pins::Driver::currentSensorPin);
        else if (path == "analog.motorCurrentFiltered")
            document["value"] =
                analogRead(Pins::Driver::currentSensorPin) -
                calibration.currentSensorOffset;
        else if (path == "motor.currentAmps" || path == "motor.voltageVolts") {
            float value;
            const bool valid = path == "motor.currentAmps"
                                   ? readMotorCurrentAmps(value)
                                   : readMotorVoltageVolts(value);
            if (!valid)
                return radble::Result::failure("unavailable", "Fresh RS485 motor telemetry is unavailable");
            document["value"] = value;
            document["source"] = "rs485";
        }
        else if (path == "motor.hat") {
            const auto hat = getOssmHatStatus();
            document["detected"] = hat.detected;
            document["fresh"] = hat.fresh;
            document["passive"] = hat.passive;
            document["baudRate"] = hat.baudRate;
            document["ageMs"] = hat.ageMs;
            document["sampleRateHz"] = hat.sampleRateHz;
            document["positionSource"] = motorPositionUsesRs485() ? "rs485" : "stepper";
            OssmHatTelemetry sample;
            if (readOssmHatTelemetry(sample)) {
                document["encoderCounts"] = sample.positionCounts;
                document["currentAmps"] = sample.currentAmps;
                document["voltageVolts"] = sample.voltageVolts;
                document["alarm"] = sample.alarm;
            }
        }
        else if (path == "motor.guard") {
            const auto safety = getMotorSafetyStatus();
            document["required"] = safety.required;
            document["ready"] = safety.ready;
            document["armed"] = safety.armed;
            document["latched"] = safety.latched;
            document["fault"] = safety.fault;
            document["outputDisabled"] = safety.outputDisableVerified;
            document["stoppedVerified"] = safety.stoppedVerified;
            document["boundsValid"] = safety.boundsValid;
            document["activeMaster"] = safety.activeMaster;
            document["contacts"] = safety.contacts;
            document["observedHz"] = safety.observedHz;
            document["sampleAgeMs"] = safety.sampleAgeMs;
        } else if (path == "motor.guardFault") {
            const auto detail = getMotorSafetyFaultInfo();
            document["available"] = detail.available;
            if (detail.available) {
                document["decisionAtMs"] = detail.decisionAtMs;
                document["sampleAtMs"] = detail.sampleAtMs;
                document["sequence"] = detail.sequence;
                document["gapMs"] = detail.gapMs;
                document["ageMs"] = detail.ageMs;
                document["continuity"] = detail.continuity;
                document["encoderCounts"] = detail.encoderCounts;
                document["deltaCounts"] = detail.deltaCounts;
                if (detail.progressFault) {
                    document["motionExpected"] = detail.motionExpected;
                    document["dir"] = detail.direction;
                    document["progressMs"] = detail.progressMs;
                    document["graceMs"] = detail.graceMs;
                } else {
                    document["instantMmS"] = detail.instantaneousMmS;
                    document["windowMmS"] = detail.windowMmS;
                    document["windowMs"] = detail.windowMs;
                }
            }
        } else if (path == "motor.guardControl") {
            const auto safety = getMotorSafetyStatus();
            document["reason"] = motorSafetyReason();
            document["lowerStopCounts"] = safety.lowerStopCounts;
            document["upperStopCounts"] = safety.upperStopCounts;
            document["rs485StopVerified"] = getOssmHatEmergencyStopStatus().verified;
            const auto external = getOssmHatExternalControlStatus();
            document["prepareRequested"] = external.requested;
            document["prepareComplete"] = external.complete;
            document["prepareVerified"] = external.verified;
            OssmHatControlSnapshot controls;
            if (readOssmHatControlSnapshot(controls)) {
                document["modbusEnable"] = controls.modbusEnable;
                document["serialOutputSetting"] = controls.outputEnable;
            }
        }
        else if (path == "motion.homed")
            document["value"] = calibration.isHomed;
        else if (path == "motion.position")
            document["value"] = getMotorPositionMm();
        else if (path == "motion.currentOffset")
            document["value"] = calibration.currentSensorOffset;
        else if (path == "motion.measuredStroke")
            document["value"] = calibration.measuredStrokeSteps;
        else if (path == "motion.targetPosition")
            document["value"] = motion.targetPosition;
        else if (path == "motion.targetTime")
            document["value"] = motion.targetTime;
        else if (path == "session.strokeCount")
            document["value"] = session.strokeCount;
        else if (path == "session.distance")
            document["value"] = session.distanceMeters;
        else if (path == "button.emergencyStop")
            document["value"] = digitalRead(Pins::Driver::stopPin) == LOW;
        else if (path == "button.limit")
            document["value"] = digitalRead(Pins::Driver::limitSwitchPin) == LOW;
        else if (path == "analog.expansion1")
            document["value"] = analogRead(Pins::GPIO::pin1);
        else if (path == "analog.expansion2")
            document["value"] = analogRead(Pins::GPIO::pin2);
        else if (path == "analog.expansion3")
            document["value"] = analogRead(Pins::GPIO::pin3);
        else if (path == "analog.expansion4")
            document["value"] = analogRead(Pins::GPIO::pin4);
        else if (path == "button.enter")
            document["value"] =
                digitalRead(Pins::Remote::encoderSwitch) == LOW;
        else if (path == "encoder.main")
            document["value"] = encoder.readEncoder();
        else if (path == "connectivity.bleConnections")
            document["value"] = pServer == nullptr ? 0 : pServer->getConnectedCount();
        else if (path == "connectivity.wifi") {
            const bool connected = WiFi.status() == WL_CONNECTED;
            document["connected"] = connected;
            if (connected) {
                document["rssi"] = WiFi.RSSI();
                document["ip"] = WiFi.localIP().toString();
            }
        }
        else
            return radble::Result::failure("unknown_path", "Unknown sensor path");
        String output;
        serializeJson(document, output);
        return radble::Result::success(output);
    }

    if (operation == "setting.write") {
        if (path.startsWith("analog.expansion")) {
            const int index = path.substring(strlen("analog.expansion")).toInt();
            const int pins[] = {Pins::GPIO::pin1, Pins::GPIO::pin2,
                                Pins::GPIO::pin3, Pins::GPIO::pin4};
            if (index < 1 || index > 4)
                return radble::Result::failure("unknown_path",
                                               "Unknown expansion GPIO");
            if (ossmHatOwnsPin(pins[index - 1]))
                return radble::Result::failure("pin_reserved", "GPIO is reserved for RS485");
            const String mode = args["mode"] | "output";
            if (mode == "input") {
                pinMode(pins[index - 1], INPUT);
            } else if (mode == "inputPullup") {
                pinMode(pins[index - 1], INPUT_PULLUP);
            } else if (mode == "output") {
                if (!args["value"].is<bool>() && !args["value"].is<int>())
                    return radble::Result::failure(
                        "invalid_value", "GPIO output value must be boolean");
                pinMode(pins[index - 1], OUTPUT);
                digitalWrite(pins[index - 1], (args["value"] | 0) ? HIGH : LOW);
            } else {
                return radble::Result::failure("invalid_value",
                                               "GPIO mode is invalid");
            }
            return radble::Result::success(
                "{\"mode\":\"" + mode + "\",\"value\":" +
                String(digitalRead(pins[index - 1])) + "}");
        }
        if (path == "setting.speedKnobAsLimit") {
            if (!args["value"].is<bool>())
                return radble::Result::failure("invalid_value",
                                               "A boolean value is required");
            USE_SPEED_KNOB_AS_LIMIT = args["value"] | false;
            return settingValue(path);
        }
        if (path == "setting.latencyCompensation") {
            if (!args["value"].is<bool>())
                return radble::Result::failure("invalid_value",
                                               "A boolean value is required");
            USE_LATENCY_COMPENSATION = args["value"] | false;
            return settingValue(path);
        }
        if (path == "setting.displayMetric") {
            if (!args["value"].is<bool>())
                return radble::Result::failure("invalid_value",
                                               "A boolean value is required");
            UserConfig::displayMetric = args["value"] | true;
            return settingValue(path);
        }
        if (path == "setting.afterHomingPosition") {
            if (!args["value"].is<float>())
                return radble::Result::failure("invalid_value",
                                               "A numeric home position is required");
            const float value = args["value"] | -1.0f;
            if (value < 0.0f || value > 100.0f)
                return radble::Result::failure("invalid_value",
                                               "Home position must be 0..100 mm");
            UserConfig::afterHomingPosition = value;
            return settingValue(path);
        }
        if (path == "setting.mqttPublishFrequency") {
            if (!args["value"].is<float>())
                return radble::Result::failure("invalid_value",
                                               "A numeric MQTT frequency is required");
            const float value = args["value"] | 0.0f;
            if (value < 1.0f || value > 100.0f)
                return radble::Result::failure("invalid_value",
                                               "MQTT frequency must be 1..100 Hz");
            UserConfig::mqttPublishFrequencyHz = value;
            return settingValue(path);
        }
        if (path == "motion.speedBle") {
            if (!args["value"].is<float>())
                return radble::Result::failure("invalid_value",
                                               "A numeric BLE speed is required");
            const float value = args["value"] | -1.0f;
            if (value < 0.0f || value > 100.0f)
                return radble::Result::failure("invalid_value",
                                               "BLE speed must be 0..100");
            settings.speedBLE = value;
            return settingValue(path);
        }
        if (path == "device.name") {
            if (!args["value"].is<const char*>())
                return radble::Result::failure("invalid_value",
                                               "A device name string is required");
            String value = args["value"] | "";
            value.trim();
            if (value.isEmpty() || value.length() > 8)
                return radble::Result::failure("invalid_value",
                                               "Device name must be 1..8 characters");
            Preferences preferences;
            preferences.begin("UserConfig", false);
            const bool stored = preferences.putString("DeviceName", value) > 0;
            preferences.end();
            if (!stored)
                return radble::Result::failure("storage_failed",
                                               "Could not store device name");
            return settingValue(path);
        }
        if (!args["value"].is<int>())
            return radble::Result::failure("invalid_value", "Integer value required");
        int value = args["value"].as<int>();
        String command;
        if (path == "motion.pattern") {
            if (value < 0 || value > 6)
                return radble::Result::failure("invalid_value", "Pattern must be 0..6");
            command = "set:pattern:" + String(value);
        } else {
            if (value < 0 || value > 100)
                return radble::Result::failure("invalid_value", "Value must be 0..100");
            if (path == "motion.speed") command = "set:speed:" + String(value);
            else if (path == "motion.stroke") command = "set:stroke:" + String(value);
            else if (path == "motion.depth") command = "set:depth:" + String(value);
            else if (path == "motion.sensation") command = "set:sensation:" + String(value);
            else if (path == "motion.buffer") command = "set:buffer:" + String(value);
            else return radble::Result::failure("unknown_path", "Unknown setting path");
        }
        ossm->ble_click(command);
        Serial.printf("[RAD BLE][OSSM] applied %s\n", command.c_str());
        return settingValue(path);
    }

    if (operation == "input.emit" || operation == "event.emit") {
        const String event = args["event"] | "click";
        bool handled = false;
        if (path == "button.enter") {
            if (event == "click")
                handled = stateMachine->process_event(ButtonPress{});
            else if (event == "double")
                handled = stateMachine->process_event(DoublePress{});
            else if (event == "long")
                handled = stateMachine->process_event(LongPress{});
            else
                return radble::Result::failure("invalid_value",
                                               "Unknown button event");
        } else if (operation == "event.emit" && path == "event.returnToMenu")
            handled = stateMachine->process_event(ReturnToMenu{});
        else if (operation == "event.emit" && path == "event.done")
            handled = stateMachine->process_event(Done{});
        else if (operation == "event.emit" && path == "event.error")
            handled = stateMachine->process_event(Error{});
        else if (operation == "event.emit" && path == "event.home")
            handled = stateMachine->process_event(Home{});
        else if (operation == "event.emit" && path == "event.emergencyStop")
            handled = stateMachine->process_event(EmergencyStop{});
        else if (operation == "event.emit" && path == "event.updateUnavailable")
            handled = stateMachine->process_event(UpdateUnavailable{});
        else
            return radble::Result::failure("unknown_path", "Unknown event path");
        if (!handled)
            return radble::Result::failure(
                "guard_rejected", "The current state rejected this event");
        Serial.printf("[RAD BLE][OSSM] input %s %s\n", path.c_str(), event.c_str());
        return radble::Result::success();
    }

    if (operation == "target.set") {
        const String state = currentStateName();
        bool handled = false;
        bool deferred = false;
        if (path == "target.menu") {
            handled = returnToMenuSafely();
        } else if (path == "target.simplePenetration" ||
                   path == "target.strokeEngine" ||
                   path == "target.streaming") {
            const Menu target = path == "target.simplePenetration"
                                    ? Menu::SimplePenetration
                                : path == "target.strokeEngine"
                                    ? Menu::StrokeEngine
                                    : Menu::Streaming;
            const String targetState = path.substring(strlen("target."));
            if (state.startsWith(targetState)) {
                handled = true;
            } else if (state.startsWith("homing")) {
                menuState.currentOption = target;
                deferred = true;
                handled = true;
            } else {
                handled = selectMenuTarget(target);
            }
        } else if (path == "target.pairing") {
            handled = selectMenuTarget(Menu::Pairing);
        } else if (path == "target.wifi") {
            handled = selectMenuTarget(Menu::WiFiSetup);
        } else if (path == "target.update") {
            handled = selectMenuTarget(Menu::UpdateOSSM);
        } else if (path == "target.help") {
            handled = selectMenuTarget(Menu::Help);
        } else if (path == "target.restart") {
            // Restart is a recovery action and must remain available when
            // homing cannot complete because motor power is absent.
            handled = xTaskCreate(requestRestartTask, "rad-restart", 2048,
                                  nullptr, 1, nullptr) == pdPASS;
        } else if (path == "target.home") {
            handled = stateMachine->process_event(Home{});
        } else if (path == "target.guardReset") {
            if (!(args["reviewed"] | false))
                return radble::Result::failure("review_required", "Review the latched fault before resetting");
            if (!resetMotorSafetyAfterReview())
                return radble::Result::failure("not_ready", "Fresh stopped motor feedback is required");
            calibration.isHomed = false;
            stateMachine->process_event(MotorSafetyReviewed{});
            handled = true;
        } else if (path == "target.guardPrepare") {
            if (!(args["reviewed"] | false))
                return radble::Result::failure("review_required", "Explicit stopped-motor review is required");
            if (!prepareMotorSafetyExternalControlAfterReview())
                return radble::Result::failure("not_ready", "Unarmed, stationary, healthy motor feedback is required");
            return radble::Result::success(R"({"pending":true,"enablesOutput":false})");
        } else if (path == "target.emergencyStop") {
            // Always apply the stop, including when already in a menu.
            ossmEmergencyStop();
            disarmMotorSafety();
            calibration.isHomed = false;
            stateMachine->process_event(EmergencyStop{});
            // Apply the stop even if a latched error keeps its current UI.
            handled = true;
        } else if (path == "motion.position") {
            if (!args["value"].is<int>() ||
                (!args["durationMs"].isNull() &&
                 !args["durationMs"].is<int>()))
                return radble::Result::failure(
                    "invalid_value", "Position and duration must be integers");
            const int position = args["value"] | -1;
            const int duration = args["durationMs"] | 0;
            if (position < 0 || position > 100 || duration < 0 || duration > 60000)
                return radble::Result::failure("invalid_value", "Invalid position target");
            if (!state.startsWith("streaming"))
                return radble::Result::failure(
                    "invalid_state", "Position targets require streaming mode");
            ossm->ble_click("stream:" + String(position) + ":" + String(duration));
            handled = true;
        } else return radble::Result::failure("unknown_path", "Unknown target path");
        if (!handled)
            return radble::Result::failure(
                "guard_rejected", "The current state rejected this target");
        Serial.printf("[RAD BLE][OSSM] target %s\n", path.c_str());
        return radble::Result::success(deferred ? R"({"deferred":true})" : "{}");
    }

    if (operation == "encoder.set" || operation == "encoder.delta") {
        if (path != "encoder.main")
            return radble::Result::failure("unknown_path", "Unknown encoder path");
        if ((operation == "encoder.delta" && !args["delta"].is<int>()) ||
            (operation == "encoder.set" && !args["value"].is<int>()))
            return radble::Result::failure("invalid_value",
                                           "An integer encoder value is required");
        const int value = operation == "encoder.delta"
                              ? encoder.readEncoder() + (args["delta"] | 0)
                              : (args["value"] | -1);
        if (value < 0 || value > 100)
            return radble::Result::failure("invalid_value", "Encoder value must be 0..100");
        encoder.setEncoderValue(value);
        Serial.printf("[RAD BLE][OSSM] encoder=%d\n", value);
        return radble::Result::success("{\"value\":" + String(value) + "}");
    }

    if (operation == "indicator.set") {
        if (path != "indicator.status")
            return radble::Result::failure("unknown_path", "Unknown indicator path");
        if ((!args["r"].isNull() && !args["r"].is<int>()) ||
            (!args["g"].isNull() && !args["g"].is<int>()) ||
            (!args["b"].isNull() && !args["b"].is<int>()))
            return radble::Result::failure("invalid_value",
                                           "RGB values must be integers");
        const int red = args["r"] | 0;
        const int green = args["g"] | 0;
        const int blue = args["b"] | 0;
        if (red < 0 || red > 255 || green < 0 || green > 255 || blue < 0 ||
            blue > 255)
            return radble::Result::failure("invalid_value",
                                           "RGB values must be 0..255");
        setLEDColor(red, green, blue);
        Serial.printf("[RAD BLE][OSSM] indicator rgb(%d,%d,%d)\n", red, green, blue);
        return radble::Result::success();
    }

    return radble::Result::failure("unsupported", "Operation is not supported");
}

String snapshot(radble::Surface surface, void*) {
    if (ossm == nullptr) return R"({"state":"starting"})";
    JsonDocument document;
    switch (surface) {
        case radble::Surface::State: {
            // Keep the unsolicited state heartbeat and state.read response below
            // the smallest ATT payload observed on supported centrals. Detailed
            // motion values remain available through the motion.* resources.
            JsonDocument current;
            if (deserializeJson(current, ossm->getCurrentState()))
                return R"({"state":"unknown"})";
            document["state"] = current["state"] | "unknown";
            break;
        }
        case radble::Surface::Essential: {
            JsonDocument current;
            if (deserializeJson(current, ossm->getCurrentState()))
                document["state"] = "unknown";
            else
                document["state"] = current["state"] | "unknown";
            float volts;
            if (readMotorVoltageVolts(volts))
                document["powered"] = volts >= 20.0f;
            else if (ossmHatDetected())
                document["powered"] = nullptr;
            else
                document["powered"] = true;
            document["batteryPercent"] = nullptr;
            document["charging"] = nullptr;
            document["positionMm"] = getMotorPositionMm();
            document["sessionDistanceMeters"] = session.distanceMeters;
            document["sessionStrokeCount"] = session.strokeCount;
            break;
        }
        case radble::Surface::Button:
            document.add(JsonObject());
            document[0]["id"] = "enter";
            document[0]["pressed"] =
                digitalRead(Pins::Remote::encoderSwitch) == LOW;
            document.add(JsonObject());
            document[1]["id"] = "emergency_stop";
            document[1]["pressed"] = digitalRead(Pins::Driver::stopPin) == LOW;
            document.add(JsonObject());
            document[2]["id"] = "limit";
            document[2]["pressed"] =
                digitalRead(Pins::Driver::limitSwitchPin) == LOW;
            break;
        case radble::Surface::Encoder:
            document["id"] = "encoder";
            document["value"] = encoder.readEncoder();
            break;
        case radble::Surface::Analog:
            document["speedKnob"] = analogRead(Pins::Remote::speedPotPin);
            document["speedKnobPercent"] =
                analogRead(Pins::Remote::speedPotPin) * 100.0f / 4095.0f;
            document["motorCurrent"] =
                analogRead(Pins::Driver::currentSensorPin);
            document["motorCurrentFiltered"] =
                analogRead(Pins::Driver::currentSensorPin) -
                calibration.currentSensorOffset;
            if (!ossmHatOwnsPin(Pins::GPIO::pin1))
                document["expansion1"] = analogRead(Pins::GPIO::pin1);
            else document["expansion1"] = nullptr;
            document["expansion2"] = analogRead(Pins::GPIO::pin2);
            if (!ossmHatOwnsPin(Pins::GPIO::pin3))
                document["expansion3"] = analogRead(Pins::GPIO::pin3);
            else document["expansion3"] = nullptr;
            if (!ossmHatOwnsPin(Pins::GPIO::pin4))
                document["expansion4"] = analogRead(Pins::GPIO::pin4);
            else document["expansion4"] = nullptr;
            {
                float value;
                if (readMotorCurrentAmps(value)) document["motorCurrentAmps"] = value;
                else document["motorCurrentAmps"] = nullptr;
                if (readMotorVoltageVolts(value)) document["motorVoltageVolts"] = value;
                else document["motorVoltageVolts"] = nullptr;
                document["hatDetected"] = ossmHatDetected();
            }
            break;
        case radble::Surface::Motion:
            document["homed"] = calibration.isHomed;
            document["positionMm"] = getMotorPositionMm();
            document["positionSource"] = motorPositionUsesRs485() ? "rs485" : "stepper";
            document["speed"] = settings.speed;
            document["stroke"] = settings.stroke;
            document["depth"] = settings.depth;
            document["sensation"] = settings.sensation;
            document["buffer"] = settings.buffer;
            document["pattern"] = static_cast<int>(settings.pattern);
            document["targetPosition"] = motion.targetPosition;
            document["targetTimeMs"] = motion.targetTime;
            document["strokeCount"] = session.strokeCount;
            document["distanceMeters"] = session.distanceMeters;
            break;
        case radble::Surface::Connectivity:
            document["wifi"] = WiFi.status() == WL_CONNECTED;
            document["rssi"] = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
            document["ip"] = WiFi.localIP().toString();
            document["ble"] = ossm->hasActiveBLE();
            break;
        case radble::Surface::Indicator:
            document["id"] = "led";
            document["r"] = leds[0].r;
            document["g"] = leds[0].g;
            document["b"] = leds[0].b;
            break;
        default:
            return "{}";
    }
    String output;
    serializeJson(document, output);
    return output;
}

radble::Result prepareOta(bool starting, const char*, void*) {
    if (!starting) return radble::Result::success();
    disarmMotorSafety();
    if (stepper != nullptr) ossmEmergencyStop();
    settings.speed = 0;
    if (stateMachine != nullptr) stateMachine->process_event(ReturnToMenu{});
    Serial.println("[RAD BLE][OSSM] direct OTA safety stop applied");
    return radble::Result::success();
}

void releaseDiagnosticOutputs(void*) { setLEDOff(); }

}  // namespace

radble::Server radBleServer;

bool initRadBle(NimBLEServer* server) {
    const radble::Config config = {
        .identity = {
            .deviceType = "OSSM",
            .deviceName = "OSSM",
            .serviceUuid = radble::OSSM_SERVICE_UUID,
            .firmwareVersion = VERSION,
            .build = FIRMWARE_BUILD_SHA,
            .partitionLayout = "ossm-ota-16mb-v1",
        },
        .capabilities = radble::CAP_BUTTON | radble::CAP_ENCODER |
                        radble::CAP_ANALOG | radble::CAP_MOTION |
                        radble::CAP_CONNECTIVITY | radble::CAP_INDICATOR |
                        radble::CAP_SENSOR_STREAM,
        .channels = radble::CHANNEL_SENSOR_STREAM |
                    radble::CHANNEL_APPLICATION_OTA,
        .resources = RESOURCES,
        .resourceCount = sizeof(RESOURCES) / sizeof(RESOURCES[0]),
        .callbacks = {
            .commandHandler = handleCommand,
            .snapshotHandler = snapshot,
            .otaDataHandler = nullptr,
            .otaSafetyHandler = prepareOta,
            .leaseReleaseHandler = releaseDiagnosticOutputs,
            .streamSafetyHandler = nullptr,
            .readSnapshotHandler = [](radble::Surface, void*) {
                return ossm == nullptr ? String(R"({"state":"starting"})")
                                       : ossm->getCurrentState();
            },
        },
        .context = nullptr,
    };
    return radBleServer.begin(server, config);
}
