#include <unity.h>
#include <map>
#include <string>
#include <vector>
#include "saved_pattern_library.h"

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
    flash["libraryV2"][0] ^= 0xff;
    lib.load(readRecord);
    TEST_ASSERT_FALSE(lib.any());
    flash["curveV1"] = {0,1,2};
    lib.load(readRecord);
    TEST_ASSERT_FALSE(lib.any());
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

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_legacy_blob_loads_without_writing_or_starting_and_survives_upgrade);
    RUN_TEST(test_first_slot_default_named_select_and_v1_aliases_survive_reboot);
    RUN_TEST(test_failed_or_repeated_transactions_preserve_selection_and_slot);
    RUN_TEST(test_bounded_names_slots_and_corrupt_flash);
    RUN_TEST(test_storage_requires_stationary_current_feedback_when_available);
    return UNITY_END();
}
