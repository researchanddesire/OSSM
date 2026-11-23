#include <ArduinoFake.h>
#include <unity.h>

#include <string>

// Stub ESP-IDF logging macros (not available on native platform)
#define ESP_LOGI(tag, fmt, ...)
#define ESP_LOGW(tag, fmt, ...)
#define ESP_LOGE(tag, fmt, ...)

#include "command/command_pattern.hpp"
#include "command/commands.hpp"

// ---------------------------------------------------------------------------
// commandFromString tests
// ---------------------------------------------------------------------------

void test_commandFromString_goStrokeEngine() {
    auto result = commandFromString(String("go:strokeEngine"));
    TEST_ASSERT_EQUAL(Commands::goToStrokeEngine, result.command);
    TEST_ASSERT_EQUAL(0, result.value);
    TEST_ASSERT_EQUAL(0, result.time);
}

void test_commandFromString_goSimplePenetration() {
    auto result = commandFromString(String("go:simplePenetration"));
    TEST_ASSERT_EQUAL(Commands::goToSimplePenetration, result.command);
    TEST_ASSERT_EQUAL(0, result.value);
    TEST_ASSERT_EQUAL(0, result.time);
}

void test_commandFromString_goStreaming() {
    auto result = commandFromString(String("go:streaming"));
    TEST_ASSERT_EQUAL(Commands::goToStreaming, result.command);
}

void test_commandFromString_goMenu() {
    auto result = commandFromString(String("go:menu"));
    TEST_ASSERT_EQUAL(Commands::goToMenu, result.command);
}

void test_commandFromString_goRestart() {
    auto result = commandFromString(String("go:restart"));
    TEST_ASSERT_EQUAL(Commands::goToRestart, result.command);
    TEST_ASSERT_EQUAL(0, result.value);
    TEST_ASSERT_EQUAL(0, result.time);
}

void test_commandFromString_goUpdate() {
    auto result = commandFromString(String("go:update"));
    TEST_ASSERT_EQUAL(Commands::goToUpdate, result.command);
    TEST_ASSERT_EQUAL(0, result.value);
    TEST_ASSERT_EQUAL(0, result.time);
}

void test_commandFromString_goPairing() {
    auto result = commandFromString(String("go:pairing"));
    TEST_ASSERT_EQUAL(Commands::goToPairing, result.command);
    TEST_ASSERT_EQUAL(0, result.value);
    TEST_ASSERT_EQUAL(0, result.time);
}

// ---------------------------------------------------------------------------
// BLE command grammar (what the write callback accepts before queueing)
// ---------------------------------------------------------------------------

void test_grammar_accepts_every_go_command() {
    const char* accepted[] = {"go:simplePenetration", "go:strokeEngine",
                              "go:streaming",         "go:menu",
                              "go:restart",           "go:update",
                              "go:pairing"};
    for (const char* cmd : accepted) {
        TEST_ASSERT_TRUE_MESSAGE(isValidBleCommand(cmd), cmd);
    }
}

void test_grammar_accepts_set_stream_and_wifi() {
    TEST_ASSERT_TRUE(isValidBleCommand("set:speed:50"));
    TEST_ASSERT_TRUE(isValidBleCommand("set:pattern:3"));
    TEST_ASSERT_TRUE(isValidBleCommand("stream:40:250"));
    TEST_ASSERT_TRUE(isValidBleCommand("set:wifi:MySSID|MyPass"));
}

void test_grammar_rejects_junk() {
    const char* rejected[] = {"go:unknown", "go:", "update", "go:update ",
                              "set:speed:abc", "set:speed:", "stream:40",
                              "restart", ""};
    for (const char* cmd : rejected) {
        TEST_ASSERT_FALSE_MESSAGE(isValidBleCommand(cmd), cmd);
    }
}

// Every go:* the grammar accepts must map to a dedicated command, never the
// goToMenu fallback, otherwise the two tables have drifted.
void test_grammar_and_commandFromString_agree() {
    const char* accepted[] = {"go:strokeEngine", "go:streaming", "go:restart",
                              "go:update",       "go:pairing"};
    for (const char* cmd : accepted) {
        TEST_ASSERT_TRUE_MESSAGE(isValidBleCommand(cmd), cmd);
        TEST_ASSERT_NOT_EQUAL_MESSAGE(Commands::goToMenu,
                                      commandFromString(String(cmd)).command,
                                      cmd);
    }
}

void test_commandFromString_goUnknown_defaultsToMenu() {
    auto result = commandFromString(String("go:unknown"));
    TEST_ASSERT_EQUAL(Commands::goToMenu, result.command);
}

void test_commandFromString_garbage_returnsIgnore() {
    auto result = commandFromString(String("garbage"));
    TEST_ASSERT_EQUAL(Commands::ignore, result.command);
}

// ---------------------------------------------------------------------------
// setCommandValue tests
// ---------------------------------------------------------------------------

void test_setCommandValue_speed50() {
    auto result = setCommandValue(String("set:speed:50"));
    TEST_ASSERT_EQUAL(Commands::setSpeed, result.command);
    TEST_ASSERT_EQUAL(50, result.value);
    TEST_ASSERT_EQUAL(0, result.time);
}

void test_setCommandValue_depth0_boundary() {
    auto result = setCommandValue(String("set:depth:0"));
    TEST_ASSERT_EQUAL(Commands::setDepth, result.command);
    TEST_ASSERT_EQUAL(0, result.value);
}

void test_setCommandValue_depth100_boundary() {
    auto result = setCommandValue(String("set:depth:100"));
    TEST_ASSERT_EQUAL(Commands::setDepth, result.command);
    TEST_ASSERT_EQUAL(100, result.value);
}

void test_setCommandValue_speedNegative_returnsIgnore() {
    auto result = setCommandValue(String("set:speed:-1"));
    TEST_ASSERT_EQUAL(Commands::ignore, result.command);
}

void test_setCommandValue_speed101_returnsIgnore() {
    auto result = setCommandValue(String("set:speed:101"));
    TEST_ASSERT_EQUAL(Commands::ignore, result.command);
}

void test_setCommandValue_speedAbc_returnsIgnore() {
    auto result = setCommandValue(String("set:speed:abc"));
    TEST_ASSERT_EQUAL(Commands::ignore, result.command);
}

void test_setCommandValue_unknownParam_returnsIgnore() {
    auto result = setCommandValue(String("set:unknown:50"));
    TEST_ASSERT_EQUAL(Commands::ignore, result.command);
}

void test_setCommandValue_malformedSingleColon_returnsIgnore() {
    auto result = setCommandValue(String("set:speed"));
    TEST_ASSERT_EQUAL(Commands::ignore, result.command);
}

// ---------------------------------------------------------------------------
// streamCommandValue tests
// ---------------------------------------------------------------------------

void test_streamCommandValue_valid() {
    auto result = streamCommandValue(String("stream:50:200"));
    TEST_ASSERT_EQUAL(Commands::streamPosition, result.command);
    TEST_ASSERT_EQUAL(50, result.value);
    TEST_ASSERT_EQUAL(200, result.time);
}

void test_streamCommandValue_zeroBoundary() {
    auto result = streamCommandValue(String("stream:0:0"));
    TEST_ASSERT_EQUAL(Commands::streamPosition, result.command);
    TEST_ASSERT_EQUAL(0, result.value);
    TEST_ASSERT_EQUAL(0, result.time);
}

void test_streamCommandValue_pos101_returnsIgnore() {
    auto result = streamCommandValue(String("stream:101:100"));
    TEST_ASSERT_EQUAL(Commands::ignore, result.command);
}

void test_streamCommandValue_posNegative_returnsIgnore() {
    auto result = streamCommandValue(String("stream:-1:100"));
    TEST_ASSERT_EQUAL(Commands::ignore, result.command);
}

void test_streamCommandValue_malformedSingleColon_returnsIgnore() {
    auto result = streamCommandValue(String("stream:50"));
    TEST_ASSERT_EQUAL(Commands::ignore, result.command);
}

// ---------------------------------------------------------------------------
// parseWiFiCommand tests
// ---------------------------------------------------------------------------

void test_parseWiFiCommand_valid() {
    auto result = parseWiFiCommand(String("set:wifi:MySSID|MyPass"));
    TEST_ASSERT_EQUAL_STRING("MySSID", result.ssid.c_str());
    TEST_ASSERT_EQUAL_STRING("MyPass", result.password.c_str());
}

void test_parseWiFiCommand_noPipe_returnsEmpty() {
    auto result = parseWiFiCommand(String("set:wifi:NoPipe"));
    TEST_ASSERT_EQUAL_STRING("", result.ssid.c_str());
    TEST_ASSERT_EQUAL_STRING("", result.password.c_str());
}

void test_parseWiFiCommand_wrongPrefix_returnsEmpty() {
    auto result = parseWiFiCommand(String("not:wifi"));
    TEST_ASSERT_EQUAL_STRING("", result.ssid.c_str());
    TEST_ASSERT_EQUAL_STRING("", result.password.c_str());
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Malformed, out-of-range and oversized input
// ---------------------------------------------------------------------------

void test_stream_time_is_bounded() {
    // The streaming queue stores the time in 16 bits: 70000 ms used to
    // wrap to 4464 ms. Longer moves than motion.position allows are refused.
    auto atLimit = streamCommandValue(String("stream:50:60000"));
    TEST_ASSERT_EQUAL(Commands::streamPosition, atLimit.command);
    TEST_ASSERT_EQUAL(60000, atLimit.time);
    TEST_ASSERT_EQUAL(Commands::ignore,
                      streamCommandValue(String("stream:50:60001")).command);
    TEST_ASSERT_EQUAL(Commands::ignore,
                      streamCommandValue(String("stream:50:70000")).command);
    TEST_ASSERT_EQUAL(Commands::ignore,
                      streamCommandValue(String("stream:50:99999999999")).command);
}

void test_stream_rejects_non_numbers() {
    const char* rejected[] = {"stream:abc:100", "stream:50:abc", "stream:50:",
                              "stream::100",    "stream:5a:100", "stream:50:1e3",
                              "stream:+5:100",  "stream:50:-0", "stream:1:2:3",
                              "stream:4294967346:10"};
    for (const char* cmd : rejected) {
        TEST_ASSERT_EQUAL_MESSAGE(Commands::ignore,
                                  commandFromString(String(cmd)).command, cmd);
    }
}

void test_stream_accepts_leading_zeros() {
    auto result = commandFromString(String("stream:050:0100"));
    TEST_ASSERT_EQUAL(Commands::streamPosition, result.command);
    TEST_ASSERT_EQUAL(50, result.value);
    TEST_ASSERT_EQUAL(100, result.time);
}

void test_set_rejects_padded_signed_and_overflowing_values() {
    const char* rejected[] = {"set:speed:050", "set:speed:+5", "set:speed: 5",
                              "set:speed:5 ",  "set:speed:4294967346",
                              "set:speed:1e2", "set:speed:5.5", "set:speed::5",
                              "set::5",        "set:speed"};
    for (const char* cmd : rejected) {
        TEST_ASSERT_EQUAL_MESSAGE(Commands::ignore,
                                  commandFromString(String(cmd)).command, cmd);
    }
}

void test_oversized_commands_are_rejected() {
    const std::string padding(2000, '9');
    const std::string commands[] = {"set:speed:" + padding,
                                    "stream:" + padding + ":" + padding,
                                    "go:" + padding, padding};
    for (const std::string& cmd : commands) {
        TEST_ASSERT_FALSE(isValidBleCommand(cmd) &&
                          commandFromString(String(cmd.c_str())).command !=
                              Commands::ignore);
        TEST_ASSERT_NOT_EQUAL(Commands::setSpeed,
                              commandFromString(String(cmd.c_str())).command);
        TEST_ASSERT_NOT_EQUAL(Commands::streamPosition,
                              commandFromString(String(cmd.c_str())).command);
    }
}

void test_grammar_rejects_control_characters_and_case() {
    const char* rejected[] = {"go:menu\n", "go:menu\r", "GO:menu", "Go:menu",
                              "go:Menu",   " go:menu",  "set:SPEED:5",
                              "stream:5:5\n"};
    for (const char* cmd : rejected) {
        TEST_ASSERT_FALSE_MESSAGE(isValidBleCommand(cmd), cmd);
    }
}

void test_every_valid_set_value_round_trips() {
    const char* settings[] = {"speed", "stroke", "depth", "sensation", "buffer",
                              "pattern"};
    for (const char* setting : settings) {
        for (int value = 0; value <= 100; value++) {
            const std::string cmd =
                std::string("set:") + setting + ":" + std::to_string(value);
            TEST_ASSERT_TRUE_MESSAGE(isValidBleCommand(cmd), cmd.c_str());
            const auto result = commandFromString(String(cmd.c_str()));
            TEST_ASSERT_NOT_EQUAL_MESSAGE(Commands::ignore, result.command,
                                          cmd.c_str());
            TEST_ASSERT_EQUAL(value, result.value);
        }
    }
}

int main(int argc, char** argv) {
    UNITY_BEGIN();

    // commandFromString
    RUN_TEST(test_commandFromString_goStrokeEngine);
    RUN_TEST(test_commandFromString_goSimplePenetration);
    RUN_TEST(test_commandFromString_goStreaming);
    RUN_TEST(test_commandFromString_goMenu);
    RUN_TEST(test_commandFromString_goRestart);
    RUN_TEST(test_commandFromString_goUpdate);
    RUN_TEST(test_commandFromString_goPairing);
    RUN_TEST(test_grammar_accepts_every_go_command);
    RUN_TEST(test_grammar_accepts_set_stream_and_wifi);
    RUN_TEST(test_grammar_rejects_junk);
    RUN_TEST(test_grammar_and_commandFromString_agree);
    RUN_TEST(test_commandFromString_goUnknown_defaultsToMenu);
    RUN_TEST(test_commandFromString_garbage_returnsIgnore);

    // setCommandValue
    RUN_TEST(test_setCommandValue_speed50);
    RUN_TEST(test_setCommandValue_depth0_boundary);
    RUN_TEST(test_setCommandValue_depth100_boundary);
    RUN_TEST(test_setCommandValue_speedNegative_returnsIgnore);
    RUN_TEST(test_setCommandValue_speed101_returnsIgnore);
    RUN_TEST(test_setCommandValue_speedAbc_returnsIgnore);
    RUN_TEST(test_setCommandValue_unknownParam_returnsIgnore);
    RUN_TEST(test_setCommandValue_malformedSingleColon_returnsIgnore);

    // streamCommandValue
    RUN_TEST(test_streamCommandValue_valid);
    RUN_TEST(test_streamCommandValue_zeroBoundary);
    RUN_TEST(test_streamCommandValue_pos101_returnsIgnore);
    RUN_TEST(test_streamCommandValue_posNegative_returnsIgnore);
    RUN_TEST(test_streamCommandValue_malformedSingleColon_returnsIgnore);

    // parseWiFiCommand
    RUN_TEST(test_parseWiFiCommand_valid);
    RUN_TEST(test_parseWiFiCommand_noPipe_returnsEmpty);
    RUN_TEST(test_parseWiFiCommand_wrongPrefix_returnsEmpty);

    // malformed, out-of-range and oversized input
    RUN_TEST(test_stream_time_is_bounded);
    RUN_TEST(test_stream_rejects_non_numbers);
    RUN_TEST(test_stream_accepts_leading_zeros);
    RUN_TEST(test_set_rejects_padded_signed_and_overflowing_values);
    RUN_TEST(test_oversized_commands_are_rejected);
    RUN_TEST(test_grammar_rejects_control_characters_and_case);
    RUN_TEST(test_every_valid_set_value_round_trips);

    return UNITY_END();
}
