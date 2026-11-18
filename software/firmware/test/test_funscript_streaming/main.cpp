#include <array>
#include <filesystem>
#include <fstream>
#include <string>

#include "streaming_logic.h"

// ArduinoFake defines macros such as round and abs. Load the standard
// library headers first so these macros do not affect their declarations.
#include <ArduinoFake.h>
#include <ArduinoJson.h>
#include <unity.h>

#define ESP_LOGI(tag, format, ...)
#include "command/command_pattern.hpp"
#include "command/commands.hpp"

void setUp() {}
void tearDown() {}

void test_player_waypoints_follow_the_script_within_motion_limits() {
    const auto fixture = std::filesystem::path(__FILE__).parent_path() /
                         "fixtures/bench.funscript";
    std::ifstream input(fixture);
    TEST_ASSERT_TRUE(input.is_open());
    JsonDocument script;
    TEST_ASSERT_FALSE(deserializeJson(script, input));
    const auto actions = script["actions"].as<JsonArrayConst>();

    struct ExpectedMove {
        const char* command;
        int32_t targetSteps;
    };
    const std::array<ExpectedMove, 5> expected{{
        {"stream:35:1500", -825},
        {"stream:65:1400", -675},
        {"stream:65:850", -675},
        {"stream:20:50", -675},
        {"stream:50:1700", -750},
    }};
    TEST_ASSERT_EQUAL_UINT(expected.size() + 1, actions.size());
    TEST_ASSERT_TRUE(isValidBleCommand("go:streaming"));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Commands::goToStreaming),
                          static_cast<int>(commandFromString("go:streaming").command));

    constexpr int32_t measuredStroke = 2000;
    constexpr uint32_t speedLimit = 300;
    constexpr uint32_t accelerationLimit = 1200;
    constexpr int32_t stepsPerMm = 20;
    const auto maxStroke = streaming_logic::calculateMaxStroke(50, 50, measuredStroke);
    const auto depth = streaming_logic::calculateDepthOffset(measuredStroke, maxStroke, 50);
    int32_t position = streaming_logic::scaleStreamPosition(actions[0]["pos"].as<int>(),
                                                           maxStroke, depth);
    for (size_t i = 0; i < expected.size(); ++i) {
        // In full-detail playback the web player sends the next position with
        // the difference between its timestamp and the current action's timestamp.
        const auto duration = actions[i + 1]["at"].as<int>() - actions[i]["at"].as<int>();
        const auto waypoint = actions[i + 1]["pos"].as<int>();
        const auto text = "stream:" + std::to_string(waypoint) + ":" + std::to_string(duration);
        TEST_ASSERT_EQUAL_STRING(expected[i].command, text.c_str());
        TEST_ASSERT_TRUE(isValidBleCommand(text));
        const auto command = commandFromString(text.c_str());
        TEST_ASSERT_EQUAL_INT(static_cast<int>(Commands::streamPosition),
                              static_cast<int>(command.command));
        TEST_ASSERT_EQUAL_INT(waypoint, command.value);
        TEST_ASSERT_EQUAL_INT(duration, command.time);

        const auto requested = streaming_logic::scaleStreamPosition(command.value,
                                                                    maxStroke, depth);
        const auto move = streaming_logic::planMotion(position, requested, command.time / 1000.0f,
                                                     speedLimit, accelerationLimit, stepsPerMm);
        TEST_ASSERT_EQUAL_INT32(expected[i].targetSteps, move.targetPosition);
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(speedLimit, move.speed);
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(accelerationLimit, move.acceleration);
        if (position == move.targetPosition) {
            TEST_ASSERT_EQUAL_UINT32(0, move.speed);
        }
        position = move.targetPosition;
    }
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_player_waypoints_follow_the_script_within_motion_limits);
    return UNITY_END();
}
