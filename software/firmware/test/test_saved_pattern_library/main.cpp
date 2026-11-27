#include <unity.h>
#include <map>
#include <string>
#include <vector>
#include "saved_pattern_library.h"
#include "pattern_catalog_json.h"

using namespace streaming;
namespace {
std::map<std::string, std::vector<uint8_t>> flash;
bool failWrite;
int writes;
const char samples[] = "00081b335a8ee4ffffc4a75d3c3c1603";

bool readRecord(const char *key, void *out, size_t size) {
    const auto it = flash.find(key);
    if (it == flash.end() || it->second.size() != size) return false;
    std::memcpy(out, it->second.data(), size);
    return true;
}
bool writeRecord(const char *key, const void *ptr, size_t size) {
    ++writes;
    if (failWrite) return false;
    const auto *bytes = static_cast<const uint8_t *>(ptr);
    flash[key] = std::vector<uint8_t>(bytes, bytes + size);
    return true;
}
SavedPattern curve(uint32_t period) {
    SavedPattern result;
    makeSavedPattern(result, samples, sizeof(samples) - 1, period);
    return result;
}
}
void setUp() { flash.clear(); writes = 0; failWrite = false; }
void tearDown() {}

void test_legacy_blob_loads_without_writing_or_starting_and_survives_upgrade() {
    const auto old = curve(1800);
    writeRecord("curveV1", &old, sizeof(old));
    SavedPatternLibrary lib;
    lib.load(readRecord);
    SavedPattern actual;
    TEST_ASSERT_TRUE(lib.get(actual));
    TEST_ASSERT_EQUAL_MEMORY(&old, &actual, sizeof(actual));
    TEST_ASSERT_EQUAL_STRING("Saved pattern", lib.catalog().slots[0].name);
    TEST_ASSERT_EQUAL_INT(1, writes);
    TEST_ASSERT_TRUE(lib.save(2, "Second", 6, curve(2500), writeRecord));
    SavedPatternLibrary reboot;
    reboot.load(readRecord);
    TEST_ASSERT_TRUE(reboot.get(actual));
    TEST_ASSERT_EQUAL_UINT32(1800, actual.periodMs);
    TEST_ASSERT_TRUE(reboot.get(actual, 2));
    TEST_ASSERT_EQUAL_UINT32(2500, actual.periodMs);
    TEST_ASSERT_TRUE(flash.count("curveV1") == 1);
}

void test_first_slot_default_named_select_and_v1_aliases_survive_reboot() {
    SavedPatternLibrary lib;
    TEST_ASSERT_FALSE(lib.any());
    TEST_ASSERT_FALSE(lib.select(3, writeRecord));
    TEST_ASSERT_TRUE(lib.save(3, "On remote", 9, curve(4000), writeRecord));
    TEST_ASSERT_EQUAL_UINT8(3, lib.catalog().selected);
    TEST_ASSERT_TRUE(lib.save(0, "Other", 5, curve(6000), writeRecord));
    TEST_ASSERT_EQUAL_UINT8(3, lib.catalog().selected);
    TEST_ASSERT_TRUE(lib.select(0, writeRecord));
    TEST_ASSERT_TRUE(lib.save(selectedPatternSlot, nullptr, 0, curve(2500), writeRecord));
    SavedPatternLibrary reboot;
    reboot.load(readRecord);
    SavedPattern actual;
    TEST_ASSERT_TRUE(reboot.get(actual));
    TEST_ASSERT_EQUAL_UINT32(2500, actual.periodMs);
    TEST_ASSERT_EQUAL_STRING("Other", reboot.catalog().slots[0].name);
    TEST_ASSERT_TRUE(reboot.get(actual, 3));
    TEST_ASSERT_EQUAL_UINT32(4000, actual.periodMs);
}

void test_failed_or_repeated_transactions_preserve_selection_and_slot() {
    SavedPatternLibrary lib;
    failWrite = true;
    TEST_ASSERT_FALSE(lib.save(3, "A", 1, curve(1500), writeRecord));
    TEST_ASSERT_FALSE(lib.any());
    failWrite = false;
    TEST_ASSERT_TRUE(lib.save(2, "A", 1, curve(1500), writeRecord));
    TEST_ASSERT_TRUE(lib.save(1, "B", 1, curve(2000), writeRecord));
    const int committed = writes;
    TEST_ASSERT_TRUE(lib.select(2, writeRecord));
    TEST_ASSERT_TRUE(lib.save(2, "A", 1, curve(1500), writeRecord));
    TEST_ASSERT_EQUAL_INT(committed, writes);
    failWrite = true;
    TEST_ASSERT_FALSE(lib.select(1, writeRecord));
    TEST_ASSERT_FALSE(lib.save(2, "changed", 7, curve(2500), writeRecord));
    TEST_ASSERT_EQUAL_UINT8(2, lib.catalog().selected);
    TEST_ASSERT_EQUAL_STRING("A", lib.catalog().slots[2].name);
    TEST_ASSERT_EQUAL_UINT32(1500, lib.catalog().slots[2].periodMs);
    SavedPatternLibrary reboot;
    reboot.load(readRecord);
    TEST_ASSERT_EQUAL_UINT8(2, reboot.catalog().selected);
    TEST_ASSERT_EQUAL_UINT32(1500, reboot.catalog().slots[2].periodMs);
}

void test_bounded_names_slots_and_corrupt_flash() {
    SavedPatternLibrary lib;
    const auto value = curve(2000);
    TEST_ASSERT_TRUE(lib.save(3, u8"Nuancé", 7, value, writeRecord));
    TEST_ASSERT_TRUE(validPatternName("abcdefghijklmnopqrstuvwx", 24));
    TEST_ASSERT_FALSE(validPatternName("abcdefghijklmnopqrstuvwxy", 25));
    TEST_ASSERT_FALSE(validPatternName("A\0B", 3));
    TEST_ASSERT_FALSE(validPatternName("\xC0\xAF", 2));
    TEST_ASSERT_FALSE(validPatternName("\xED\xA0\x80", 3));
    TEST_ASSERT_FALSE(validPatternName("\xC2\x80", 2));
    TEST_ASSERT_FALSE(validPatternName(" \n", 2));
    TEST_ASSERT_FALSE(validPatternName("broken\xF0", 7));
    TEST_ASSERT_FALSE(lib.save(4, "bad slot", 8, value, writeRecord));
    TEST_ASSERT_FALSE(lib.save(-2, "bad slot", 8, value, writeRecord));
    TEST_ASSERT_FALSE(lib.save(1, "bad\x7F", 4, value, writeRecord));
    TEST_ASSERT_FALSE(lib.save(1, "OK", 2, curve(0), writeRecord));
    TEST_ASSERT_FALSE(lib.select(0, writeRecord));
    TEST_ASSERT_EQUAL_INT(1, writes);
    flash["libraryV3"][0] ^= 0xff;
    lib.load(readRecord);
    TEST_ASSERT_FALSE(lib.any());
    flash["curveV1"] = {0,1,2};
    lib.load(readRecord);
    TEST_ASSERT_FALSE(lib.any());
}

void test_q16_upgrade_preserves_all_prior_slot_bytes_and_selected_name() {
    SavedPatternLibrary old;
    auto original = curve(7251);
    TEST_ASSERT_TRUE(old.save(2, "Donor", 5, original, writeRecord));
    TEST_ASSERT_TRUE(old.save(0, "Second", 6, curve(4000), writeRecord));
    // Historical libraryV2 layout differs only in the enclosing version word.
    auto previous = flash.at("libraryV3");
    uint32_t oldVersion = 2;
    std::memcpy(previous.data(), &oldVersion, sizeof(oldVersion));
    flash.clear();
    flash["libraryV2"] = previous;
    const int before = writes;
    SavedPatternLibrary upgraded;
    upgraded.load(readRecord);
    SavedPattern loaded;
    TEST_ASSERT_TRUE(upgraded.get(loaded));
    TEST_ASSERT_EQUAL_MEMORY(&original, &loaded, sizeof(loaded));
    TEST_ASSERT_EQUAL_INT(2, upgraded.catalog().selected);
    TEST_ASSERT_EQUAL_INT(before, writes);

    const std::string hex = std::string(64, '1');
    SavedPattern ordinary, precise;
    TEST_ASSERT_TRUE(makeSavedPattern(ordinary, hex.data(), hex.size(), 2000));
    TEST_ASSERT_TRUE(makeSavedPattern(precise, hex.data(), hex.size(), 2000, true));
    TEST_ASSERT_FALSE(ordinary.highPrecision());
    TEST_ASSERT_TRUE(precise.highPrecision());
    TEST_ASSERT_TRUE(upgraded.save(1, "Wave", 4, ordinary, writeRecord));
    const int beforePrecision = writes;
    TEST_ASSERT_TRUE(upgraded.save(1, "Wave", 4, precise, writeRecord));
    TEST_ASSERT_EQUAL_INT(beforePrecision + 1, writes); // Same hex, different positions.
    TEST_ASSERT_TRUE(upgraded.select(1, writeRecord));
    TEST_ASSERT_TRUE(flash.at("libraryV2") == previous);
    SavedPatternLibrary rebooted;
    rebooted.load(readRecord);
    TEST_ASSERT_TRUE(rebooted.get(loaded));
    TEST_ASSERT_EQUAL_MEMORY(&precise, &loaded, sizeof(loaded));
    TEST_ASSERT_EQUAL_STRING("Second", rebooted.catalog().slots[0].name);
    TEST_ASSERT_EQUAL_STRING("Donor", rebooted.catalog().slots[2].name);
    TEST_ASSERT_FALSE(makeSavedPattern(loaded, hex.data(), 62, 2000, true));
    TEST_ASSERT_FALSE(makeSavedPattern(loaded, hex.data(), 32, 2000, true));
}

void test_storage_requires_stationary_current_feedback_when_available() {
    PatternStorageState state;
    TEST_ASSERT_TRUE(state.stopped());  // Legacy pulse-only board.
    state.hasFeedback = true;
    TEST_ASSERT_FALSE(state.stopped());
    state.fresh = true;
    TEST_ASSERT_TRUE(state.stopped());
    state.speedRpm = -1;
    TEST_ASSERT_FALSE(state.stopped());
    state.speedRpm = 0;
    state.homing = true;
    TEST_ASSERT_FALSE(state.stopped());
    state.homing = false;
    state.armed = true;
    TEST_ASSERT_FALSE(state.stopped());
    state.armed = false;
    state.pulsesRunning = true;
    TEST_ASSERT_FALSE(state.stopped());
    state.pulsesRunning = false;
    state.serialQuiet = false;
    TEST_ASSERT_FALSE(state.stopped());
}

void test_catalog_optional_slot_preserves_full_and_empty_entries() {
    PatternCatalog catalog;
    catalog.selected = 3;
    catalog.slots[0] = {500, "First"};
    catalog.slots[3] = {60000, "Last"};
    for (const char *query : {"{}", "{\"args\":{}}"}) {
        JsonDocument request, result;
        TEST_ASSERT_FALSE(deserializeJson(request, query));
        TEST_ASSERT_TRUE(writePatternCatalog(result, request.as<JsonObjectConst>(), catalog));
        TEST_ASSERT_EQUAL_UINT(2, result["slots"].size());
        TEST_ASSERT_EQUAL_STRING("First", result["slots"][0]["name"].as<const char *>());
        TEST_ASSERT_EQUAL_UINT(3, result["slots"][1]["slot"].as<unsigned>());
    }
    for (unsigned i = 0; i < patternSlotCount; ++i) {
        JsonDocument request, result;
        request["args"]["slot"] = i;
        TEST_ASSERT_TRUE(writePatternCatalog(result, request.as<JsonObjectConst>(), catalog));
        TEST_ASSERT_EQUAL_UINT(1, result["version"].as<unsigned>());
        TEST_ASSERT_EQUAL_UINT(4, result["capacity"].as<unsigned>());
        TEST_ASSERT_EQUAL_UINT(3, result["selected"].as<unsigned>());
        const bool occupied = i == 0 || i == 3;
        TEST_ASSERT_EQUAL_UINT(occupied ? 1 : 0, result["slots"].size());
        if (occupied) TEST_ASSERT_EQUAL_UINT(i, result["slots"][0]["slot"].as<unsigned>());
    }
}

void test_catalog_rejects_invalid_page_without_returning_other_entries() {
    PatternCatalog catalog;
    for (const char *args : {"null", "[]", "1", "{\"slot\":null}", "{\"slot\":true}",
                             "{\"slot\":-1}", "{\"slot\":4}", "{\"slot\":0.5}",
                             "{\"slot\":\"1\"}", "{\"page\":1}", "{\"slot\":1,\"extra\":0}"}) {
        JsonDocument request, result;
        TEST_ASSERT_FALSE(deserializeJson(request, std::string("{\"args\":") + args + "}"));
        TEST_ASSERT_FALSE(writePatternCatalog(result, request.as<JsonObjectConst>(), catalog));
        TEST_ASSERT_TRUE(result.isNull());
    }
}

void test_catalog_escaped_maximum_names_fit_correlated_ble_response_when_paged() {
    PatternCatalog catalog;
    for (auto &slot : catalog.slots) {
        std::memset(slot.name, '"', maxPatternNameBytes);
        slot.periodMs = 60000;
    }
    JsonDocument request;
    for (unsigned i = 0; i < patternSlotCount; ++i) {
        JsonDocument result;
        request["args"]["slot"] = i;
        TEST_ASSERT_TRUE(writePatternCatalog(result, request.as<JsonObjectConst>(), catalog));
        std::string json;
        serializeJson(result, json);
        TEST_ASSERT_EQUAL_UINT(135, json.size());
        // Worst-case request ID and two generous 64-byte state names in the
        // pinned 509-byte RAD BLE terminal response envelope.
        JsonDocument envelope;
        envelope["v"] = 1;
        envelope["id"] = UINT32_MAX;
        envelope["stage"] = "completed";
        envelope["ok"] = true;
        envelope["code"] = "ok";
        envelope["stateBefore"] = std::string(64, 's');
        envelope["stateAfter"] = std::string(64, 's');
        envelope["result"] = result.as<JsonObjectConst>();
        TEST_ASSERT_EQUAL_UINT(371, measureJson(envelope));
        TEST_ASSERT_LESS_OR_EQUAL_UINT(509, measureJson(envelope));
    }
}

void test_index_uses_storage_only_for_explicit_requests_and_failed_io_preserves_catalog() {
    SavedPatternCache cache;
    int reads = 0;
    auto load = [&](SavedPatternLibrary &record) {
        ++reads;
        record.load(readRecord);
        return true;
    };
    const auto value = curve(4000);
    TEST_ASSERT_TRUE(cache.access(load, [&](SavedPatternLibrary &record) {
        return record.save(2, "Sine", 4, value, writeRecord);
    }));
    TEST_ASSERT_EQUAL_UINT(1, reads);
    TEST_ASSERT_TRUE(cache.any());
    TEST_ASSERT_EQUAL_UINT(2, cache.catalog().selected);
    TEST_ASSERT_EQUAL_STRING("Sine", cache.catalog().slots[2].name);
    TEST_ASSERT_EQUAL_UINT(1, reads);
    bool attempted = false;
    auto unavailable = [](SavedPatternLibrary &) { return false; };
    TEST_ASSERT_FALSE(cache.access(unavailable, [&](SavedPatternLibrary &) { attempted = true; return true; }));
    TEST_ASSERT_FALSE(attempted);
    failWrite = true;
    TEST_ASSERT_FALSE(cache.access(load, [&](SavedPatternLibrary &record) {
        return record.save(1, "Other", 5, value, writeRecord);
    }));
    TEST_ASSERT_EQUAL_UINT(0, cache.catalog().slots[1].periodMs);
    TEST_ASSERT_EQUAL_UINT(2, cache.catalog().selected);
    SavedPattern restored;
    TEST_ASSERT_TRUE(cache.access(load, [&](const SavedPatternLibrary &record) { return record.get(restored); }));
    TEST_ASSERT_EQUAL_MEMORY(value.samples, restored.samples, value.length);
    TEST_ASSERT_EQUAL_UINT(3, reads);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_legacy_blob_loads_without_writing_or_starting_and_survives_upgrade);
    RUN_TEST(test_first_slot_default_named_select_and_v1_aliases_survive_reboot);
    RUN_TEST(test_failed_or_repeated_transactions_preserve_selection_and_slot);
    RUN_TEST(test_bounded_names_slots_and_corrupt_flash);
    RUN_TEST(test_q16_upgrade_preserves_all_prior_slot_bytes_and_selected_name);
    RUN_TEST(test_storage_requires_stationary_current_feedback_when_available);
    RUN_TEST(test_catalog_optional_slot_preserves_full_and_empty_entries);
    RUN_TEST(test_catalog_rejects_invalid_page_without_returning_other_entries);
    RUN_TEST(test_catalog_escaped_maximum_names_fit_correlated_ble_response_when_paged);
    RUN_TEST(test_index_uses_storage_only_for_explicit_requests_and_failed_io_preserves_catalog);
    return UNITY_END();
}
