#include <unity.h>

#include <ArduinoJson.h>
#include <climits>
#include <cstring>
#include <set>
#include <string>

#include "../../src/services/ossmHealthProtocol.h"

void setUp() {}
void tearDown() {}

namespace {

constexpr const char *prefix = "OSSM_HEALTH ";

// Each record is "\nOSSM_HEALTH <json>\n": a fresh line holding one object.
JsonDocument parseRecord(const char *frame, size_t size) {
    TEST_ASSERT_TRUE(size > 0);
    TEST_ASSERT_EQUAL(std::strlen(frame), size);
    TEST_ASSERT_EQUAL('\n', frame[0]);
    TEST_ASSERT_EQUAL('\n', frame[size - 1]);
    const std::string line(frame + 1, size - 2);
    TEST_ASSERT_TRUE(line.find('\n') == std::string::npos);
    TEST_ASSERT_EQUAL(0, line.compare(0, std::strlen(prefix), prefix));
    JsonDocument doc;
    TEST_ASSERT_FALSE(deserializeJson(doc, line.substr(std::strlen(prefix))));
    TEST_ASSERT_TRUE(doc.is<JsonObject>());
    return doc;
}

void assertKeys(const JsonDocument &doc, std::set<std::string> expected) {
    std::set<std::string> actual;
    for (JsonPairConst pair : doc.as<JsonObjectConst>()) actual.insert(pair.key().c_str());
    TEST_ASSERT_TRUE(actual == expected);
}

ossm_health::Boot sampleBoot() {
    ossm_health::Boot boot;
    boot.bootId = UINT32_MAX;
    boot.uptimeMs = 1234;
    boot.buildSha = "0123456789abcdef0123456789abcdef01234567";
    boot.track = "main";
    boot.variant = "16mb";
    boot.deviceId = "A1B2C3D4E5F6";
    boot.imageSha256 = "00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff";
    boot.resetReason = 1;
    boot.hatMode = "off";
    return boot;
}

}  // namespace

void test_boot_record_has_exact_fields_and_types() {
    char frame[448];
    const auto doc = parseRecord(frame, ossm_health::formatBoot(frame, sizeof(frame), sampleBoot()));
    assertKeys(doc, {"event", "boot_id", "uptime_ms", "build_sha", "track", "variant",
                     "device_id", "image_sha256", "reset_reason", "hat_mode"});
    TEST_ASSERT_EQUAL_STRING("boot", doc["event"]);
    TEST_ASSERT_TRUE(doc["boot_id"].is<uint32_t>());
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, doc["boot_id"].as<uint32_t>());
    TEST_ASSERT_TRUE(doc["uptime_ms"].is<uint64_t>());
    TEST_ASSERT_EQUAL_STRING("0123456789abcdef0123456789abcdef01234567", doc["build_sha"]);
    TEST_ASSERT_EQUAL_STRING("main", doc["track"]);
    TEST_ASSERT_EQUAL_STRING("16mb", doc["variant"]);
    TEST_ASSERT_EQUAL_STRING("A1B2C3D4E5F6", doc["device_id"]);
    TEST_ASSERT_EQUAL(64, std::strlen(doc["image_sha256"].as<const char *>()));
    TEST_ASSERT_TRUE(doc["reset_reason"].is<int>());
    TEST_ASSERT_EQUAL(1, doc["reset_reason"].as<int>());
    TEST_ASSERT_EQUAL_STRING("off", doc["hat_mode"]);
}

void test_boot_record_fits_its_buffer_with_long_values() {
    auto boot = sampleBoot();
    boot.uptimeMs = ULLONG_MAX;
    boot.track = "development";
    boot.variant = "unknown";
    boot.resetReason = INT_MIN;
    char frame[448];
    const size_t size = ossm_health::formatBoot(frame, sizeof(frame), boot);
    parseRecord(frame, size);
    TEST_ASSERT_TRUE(size < 400);
}

void test_wifi_record() {
    char frame[160];
    const auto doc = parseRecord(frame, ossm_health::formatWifiConnected(
        frame, sizeof(frame), UINT32_MAX, ULLONG_MAX, "255.255.255.255"));
    assertKeys(doc, {"event", "boot_id", "uptime_ms", "ip"});
    TEST_ASSERT_EQUAL_STRING("wifi_connected", doc["event"]);
    TEST_ASSERT_EQUAL_STRING("255.255.255.255", doc["ip"]);
}

void test_mqtt_record_reports_the_configured_broker() {
    char frame[224];
    const auto doc = parseRecord(frame, ossm_health::formatMqttConnected(
        frame, sizeof(frame), 7, 8, "mqtts://te9acc16.ala.us-east-1.emqxsl.com"));
    assertKeys(doc, {"event", "boot_id", "uptime_ms", "broker"});
    TEST_ASSERT_EQUAL_STRING("mqtt_connected", doc["event"]);
    TEST_ASSERT_EQUAL_STRING("mqtts://te9acc16.ala.us-east-1.emqxsl.com", doc["broker"]);
    TEST_ASSERT_EQUAL_UINT32(7, doc["boot_id"].as<uint32_t>());
}

void test_heartbeat_record_has_exact_fields_and_bools() {
    ossm_health::Heartbeat beat;
    beat.bootId = 42;
    beat.uptimeMs = ULLONG_MAX;
    beat.mainMenu = true;
    beat.menuAgeMs = UINT32_MAX;
    beat.hat = false;
    beat.homed = true;
    char frame[224];
    const auto doc = parseRecord(frame, ossm_health::formatHeartbeat(frame, sizeof(frame), beat));
    assertKeys(doc, {"event", "boot_id", "uptime_ms", "main_menu", "menu_age_ms", "hat", "homed"});
    TEST_ASSERT_EQUAL_STRING("heartbeat", doc["event"]);
    TEST_ASSERT_TRUE(doc["main_menu"].is<bool>());
    TEST_ASSERT_TRUE(doc["main_menu"].as<bool>());
    TEST_ASSERT_TRUE(doc["hat"].is<bool>());
    TEST_ASSERT_FALSE(doc["hat"].as<bool>());
    TEST_ASSERT_TRUE(doc["homed"].as<bool>());
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, doc["menu_age_ms"].as<uint32_t>());
}

void test_records_never_truncate() {
    char small[32];
    TEST_ASSERT_EQUAL(0, ossm_health::formatBoot(small, sizeof(small), sampleBoot()));
    TEST_ASSERT_EQUAL(0, ossm_health::formatWifiConnected(small, sizeof(small), 1, 2, "10.0.0.1"));
    TEST_ASSERT_EQUAL(0, ossm_health::formatMqttConnected(small, sizeof(small), 1, 2, "mqtts://x"));
    TEST_ASSERT_EQUAL(0, ossm_health::formatHeartbeat(small, sizeof(small), {}));
}

void test_heartbeat_is_at_most_every_five_seconds_across_wrap() {
    TEST_ASSERT_FALSE(ossm_health::heartbeatDue(1000, 1000));
    TEST_ASSERT_FALSE(ossm_health::heartbeatDue(1000, 5999));
    TEST_ASSERT_TRUE(ossm_health::heartbeatDue(1000, 6000));
    TEST_ASSERT_FALSE(ossm_health::heartbeatDue(UINT32_MAX - 1000, 3000));
    TEST_ASSERT_TRUE(ossm_health::heartbeatDue(UINT32_MAX - 1000, 4000));
}

void test_variant_follows_the_flash_profile() {
    TEST_ASSERT_EQUAL_STRING("4mb", ossm_health::variantForFlash(4u * 1024 * 1024));
    TEST_ASSERT_EQUAL_STRING("16mb", ossm_health::variantForFlash(16u * 1024 * 1024));
    TEST_ASSERT_EQUAL_STRING("8mb", ossm_health::variantForFlash(8u * 1024 * 1024));
    TEST_ASSERT_EQUAL_STRING("unknown", ossm_health::variantForFlash(0));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_boot_record_has_exact_fields_and_types);
    RUN_TEST(test_boot_record_fits_its_buffer_with_long_values);
    RUN_TEST(test_wifi_record);
    RUN_TEST(test_mqtt_record_reports_the_configured_broker);
    RUN_TEST(test_heartbeat_record_has_exact_fields_and_bools);
    RUN_TEST(test_records_never_truncate);
    RUN_TEST(test_heartbeat_is_at_most_every_five_seconds_across_wrap);
    RUN_TEST(test_variant_follows_the_flash_profile);
    return UNITY_END();
}
