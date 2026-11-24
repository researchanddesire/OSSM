#include <unity.h>

#include <ArduinoJson.h>
#include <cstring>

#include "../../src/services/ossmHatMode.h"
#include "../../src/services/serialIdentityProtocol.h"

void setUp() {}
void tearDown() {}

using ossm_hat_mode::Mode;

namespace {

const char *feed(serialIdentity::CommandBuffer &parser, const char *bytes) {
    const char *line = nullptr;
    for (const char *c = bytes; *c; ++c) {
        if (const char *complete = parser.push(*c)) line = complete;
    }
    return line;
}

}  // namespace

void test_default_is_auto_and_only_exact_off_disables() {
    TEST_ASSERT_TRUE(ossm_hat_mode::fromStored(nullptr) == Mode::Auto);
    TEST_ASSERT_TRUE(ossm_hat_mode::fromStored("") == Mode::Auto);
    TEST_ASSERT_TRUE(ossm_hat_mode::fromStored("auto") == Mode::Auto);
    TEST_ASSERT_TRUE(ossm_hat_mode::fromStored("off") == Mode::Off);
    TEST_ASSERT_TRUE(ossm_hat_mode::fromStored("OFF") == Mode::Auto);
    TEST_ASSERT_TRUE(ossm_hat_mode::fromStored("off ") == Mode::Auto);
    TEST_ASSERT_TRUE(ossm_hat_mode::fromStored("of") == Mode::Auto);
}

void test_names_round_trip_through_storage() {
    TEST_ASSERT_EQUAL_STRING("off", ossm_hat_mode::name(Mode::Off));
    TEST_ASSERT_EQUAL_STRING("auto", ossm_hat_mode::name(Mode::Auto));
    TEST_ASSERT_TRUE(ossm_hat_mode::fromStored(ossm_hat_mode::name(Mode::Off)) == Mode::Off);
    TEST_ASSERT_TRUE(ossm_hat_mode::fromStored(ossm_hat_mode::name(Mode::Auto)) == Mode::Auto);
}

void test_only_the_two_commands_select_a_mode() {
    Mode mode = Mode::Auto;
    TEST_ASSERT_TRUE(ossm_hat_mode::parseCommand("hat off", mode));
    TEST_ASSERT_TRUE(mode == Mode::Off);
    TEST_ASSERT_TRUE(ossm_hat_mode::parseCommand("hat auto", mode));
    TEST_ASSERT_TRUE(mode == Mode::Auto);
    mode = Mode::Off;
    for (const char *other : {"hat", "info", "hat on", "hat  off", "hat off now", "HAT OFF",
                              " hat off", "hat Auto", ""}) {
        TEST_ASSERT_FALSE_MESSAGE(ossm_hat_mode::parseCommand(other, mode), other);
    }
    TEST_ASSERT_FALSE(ossm_hat_mode::parseCommand(nullptr, mode));
    TEST_ASSERT_TRUE(mode == Mode::Off);
}

void test_console_lines_reach_the_parser_with_crlf() {
    serialIdentity::CommandBuffer parser;
    Mode mode = Mode::Auto;
    const char *line = feed(parser, "\nhat off\r\n");
    TEST_ASSERT_NOT_NULL(line);
    TEST_ASSERT_TRUE(ossm_hat_mode::parseCommand(line, mode));
    TEST_ASSERT_TRUE(mode == Mode::Off);
    line = feed(parser, "hat auto\n");
    TEST_ASSERT_TRUE(ossm_hat_mode::parseCommand(line, mode));
    TEST_ASSERT_TRUE(mode == Mode::Auto);
    // A corrupted line is discarded whole, never read as its "hat off" suffix.
    TEST_ASSERT_NULL(feed(parser, "\x01hat off\n"));
}

void test_reply_is_one_exact_json_line() {
    char frame[96];
    size_t size = ossm_hat_mode::formatReply(frame, sizeof(frame), Mode::Off);
    TEST_ASSERT_EQUAL_STRING("RAD_HAT {\"mode\":\"off\",\"applies\":\"next_boot\"}\n", frame);
    TEST_ASSERT_EQUAL(std::strlen(frame), size);
    size = ossm_hat_mode::formatReply(frame, sizeof(frame), Mode::Auto);
    TEST_ASSERT_EQUAL_STRING("RAD_HAT {\"mode\":\"auto\",\"applies\":\"next_boot\"}\n", frame);
    TEST_ASSERT_EQUAL(std::strlen(frame), size);
}

void test_refusal_reports_the_unchanged_mode_and_reason() {
    char frame[96];
    const size_t size =
        ossm_hat_mode::formatReply(frame, sizeof(frame), Mode::Auto, "motor_armed");
    TEST_ASSERT_TRUE(size > 0);
    TEST_ASSERT_EQUAL('\n', frame[size - 1]);
    TEST_ASSERT_EQUAL(0, std::strncmp(frame, "RAD_HAT ", 8));
    JsonDocument doc;
    TEST_ASSERT_FALSE(deserializeJson(doc, frame + 8));
    TEST_ASSERT_EQUAL_STRING("auto", doc["mode"]);
    TEST_ASSERT_EQUAL_STRING("next_boot", doc["applies"]);
    TEST_ASSERT_EQUAL_STRING("motor_armed", doc["error"]);
}

void test_reply_never_truncates() {
    char small[20];
    TEST_ASSERT_EQUAL(0, ossm_hat_mode::formatReply(small, sizeof(small), Mode::Off));
    char exact[48];
    // 45 characters need room for the terminator as well.
    TEST_ASSERT_EQUAL(0, ossm_hat_mode::formatReply(exact, 45, Mode::Off));
    TEST_ASSERT_EQUAL(45, ossm_hat_mode::formatReply(exact, 46, Mode::Off));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_default_is_auto_and_only_exact_off_disables);
    RUN_TEST(test_names_round_trip_through_storage);
    RUN_TEST(test_only_the_two_commands_select_a_mode);
    RUN_TEST(test_console_lines_reach_the_parser_with_crlf);
    RUN_TEST(test_reply_is_one_exact_json_line);
    RUN_TEST(test_refusal_reports_the_unchanged_mode_and_reason);
    RUN_TEST(test_reply_never_truncates);
    return UNITY_END();
}
