#include <unity.h>
#include <map>
#include <string>
#include <vector>
#include "saved_pattern_library.h"
#include "pattern_catalog_json.h"
#include "pattern_modifier_upload.h"

using namespace streaming;
namespace {
std::map<std::string, std::vector<uint8_t>> flash;
bool failWrite;
int writes;
const char samples[] = "00081b335a8ee4ffffc4a75d3c3c1603";

size_t readRecord(const char *key, void *out, size_t size) {
    const auto it = flash.find(key);
    if (it == flash.end()) return 0;
    if (!out) return it->second.size();
    if (it->second.size() != size) return 0;
    std::memcpy(out, it->second.data(), size);
    return size;
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
    TEST_ASSERT_TRUE(makeSavedPattern(precise, hex.data(), hex.size(), 2000, drawn_pattern::SampleFormat::Q16));
    TEST_ASSERT_TRUE(ordinary.format() == drawn_pattern::SampleFormat::Q8);
    TEST_ASSERT_TRUE(precise.format() == drawn_pattern::SampleFormat::Q16);
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
    TEST_ASSERT_FALSE(makeSavedPattern(loaded, hex.data(), 62, 2000, drawn_pattern::SampleFormat::Q16));
    TEST_ASSERT_FALSE(makeSavedPattern(loaded, hex.data(), 32, 2000, drawn_pattern::SampleFormat::Q16));
}

void test_sparse_samples_cover_one_cycle_and_survive_named_save_and_reboot() {
    const char *hex = "020066660201680c02026c660203728c02047999020580a6020686cc02078b26"
                      "02088ccc02098b26020a86cc020b80a6020c7999020d728c020e6c66020f680c03ff6666";
    JsonDocument args;
    args["periodMs"] = 60000;
    args["samples16Runs"] = hex;
    SavedPattern value;
    TEST_ASSERT_TRUE(readPatternSamples(args.as<JsonObjectConst>(), value));
    TEST_ASSERT_EQUAL_UINT32(3, value.version);
    SavedPatternLibrary library;
    TEST_ASSERT_TRUE(library.save(3, "Excursion", 9, value, writeRecord));
    SavedPatternLibrary reboot;
    reboot.load(readRecord);
    SavedPattern recovered;
    TEST_ASSERT_TRUE(reboot.get(recovered));
    TEST_ASSERT_EQUAL_MEMORY(&value, &recovered, sizeof(value));
    drawn_pattern::Waveform waveform;
    TEST_ASSERT_TRUE(waveform.decode(recovered.samples, recovered.length, recovered.periodMs, recovered.format()));
    for (unsigned i = 0; i <= 8192; ++i) {
        const double phase = i / 8192.;
        const double u = std::max(0., 1. - std::abs((phase - 65./128) * 128));
        const double source = .4 + .15*u*u*(3-2*u);
        TEST_ASSERT_TRUE(std::abs(source - waveform.position(phase)) <= .00020);
        if (phase < .5 || phase >= 66./128) {
            TEST_ASSERT_TRUE(std::abs(waveform.position(phase)-.4) < 1e-12);
            TEST_ASSERT_TRUE(waveform.phaseDerivative(phase, 1) == 0);
            TEST_ASSERT_TRUE(waveform.phaseDerivative(phase, 2) == 0);
        }
    }
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(60000, waveform.safePeriodMs(100, 1000, 10000));
}

void test_invalid_numeric_and_overlapping_formats_are_rejected_at_the_ble_boundary() {
    JsonDocument args;
    args["periodMs"] = 7251;
    SavedPattern value;
    auto parse = [&] { return readPatternSamples(args.as<JsonObjectConst>(), value); };
    TEST_ASSERT_FALSE(parse());
    args["samples"] = samples;
    TEST_ASSERT_TRUE(parse());
    args["samples16Runs"] = nullptr;
    TEST_ASSERT_FALSE(parse());
    args.remove("samples");
    for (const char *invalid : {"", "03ff", "03ffzzzz", "04000000", "03fe1234",
                               "000600010006000203ffffff", "000600010001000203ffffff",
                               "0007000103ff0001", "03ff123400000000"}) {
        args["samples16Runs"] = invalid;
        TEST_ASSERT_FALSE(parse());
    }
    args["samples16Runs"] = "0000ffff03ff0000"; // one-position run is legal
    TEST_ASSERT_TRUE(parse());
    args["samples16"] = std::string(64, 'a');
    TEST_ASSERT_FALSE(parse());
    args.remove("samples16Runs");
    TEST_ASSERT_TRUE(parse());
    args["periodMs"] = -1;
    TEST_ASSERT_FALSE(parse());
    args["periodMs"] = 7251;
    args["samples16"] = 42;
    TEST_ASSERT_FALSE(parse());
    args.remove("samples16");
    std::string runs;
    char entry[9];
    for (unsigned i = 0; i < 32; ++i) {
        snprintf(entry, sizeof(entry), "%04x%04x", (i+1)*32-1, i % 2 ? 0 : 65535);
        runs += entry;
    }
    args["samples16Runs"] = runs;
    TEST_ASSERT_TRUE(parse());
    // Offset 128 was the old invalid cache sentinel. Entry rotates to knot 32;
    // make the first lookup land on knot 128, whose height is the maximum.
    drawn_pattern::Waveform waveform;
    TEST_ASSERT_TRUE(waveform.decode(value.samples, value.length, 7251, value.format()));
    TEST_ASSERT_TRUE(std::abs(waveform.position(96./1024) - 1.) < 1e-12);
    args["samples16Runs"] = runs + "03ff0000";
    TEST_ASSERT_FALSE(parse());
    value.version = 259; // low byte alone must never authorize a corrupt record.
    TEST_ASSERT_FALSE(validSavedPattern(value));
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

void test_modifier_chunks_are_bound_to_one_lease_and_atomic_across_retries() {
    PatternModifierUpload upload;
    JsonDocument doc;
    auto args = doc.to<JsonObject>();
    args["key"] = 7;
    args["offset"] = 0;
    args["data"] = "006400c80000"; // 10–20% vibration
    TEST_ASSERT_EQUAL_UINT(1, upload.append(3, 44, args));
    TEST_ASSERT_EQUAL_UINT(1, upload.append(3, 44, args));
    args["offset"] = 1;
    args["data"] = "0096009603e8"; // pause inside vibration
    TEST_ASSERT_EQUAL_UINT(0, upload.append(3, 44, args));
    args["data"] = "00c800c803e8"; // pause on vibration boundary
    TEST_ASSERT_EQUAL_UINT(0, upload.append(3, 44, args));
    args["data"] = "00c8012c00000190019003e8";
    TEST_ASSERT_EQUAL_UINT(0, upload.append(3, 45, args));
    TEST_ASSERT_EQUAL_UINT(0, upload.append(4, 44, args));
    TEST_ASSERT_EQUAL_UINT(3, upload.append(3, 44, args));
    TEST_ASSERT_EQUAL_UINT(3, upload.append(3, 44, args));
    args["modifierKey"] = 7;
    args["modifierCount"] = 2;
    TEST_ASSERT_FALSE(bool(upload.take(3, 44, args)));
    args["modifierCount"] = 3;
    TEST_ASSERT_FALSE(bool(upload.take(3, 45, args)));
    const auto complete = upload.take(3, 44, args);
    TEST_ASSERT_TRUE(bool(complete));
    TEST_ASSERT_EQUAL_UINT(1000, complete->entries[2].holdMs);
    TEST_ASSERT_FALSE(bool(upload.take(3, 44, args)));
}

void test_modifier_upload_accepts_capacity_and_rejects_bad_or_conflicting_input() {
    PatternModifierUpload upload;
    JsonDocument doc;
    auto args = doc.to<JsonObject>();
    args["key"] = 99;
    char hex[13];
    for (unsigned offset = 0; offset < 128; offset += 32) {
        std::string data;
        for (unsigned i = offset; i < offset + 32; ++i) {
            snprintf(hex, sizeof(hex), "%04x%04x0001", i, i);
            data += hex;
        }
        args["offset"] = offset;
        args["data"] = data;
        TEST_ASSERT_EQUAL_UINT(offset + 32, upload.append(1, 12, args));
    }
    args["offset"] = 128;
    args["data"] = "008000800001";
    TEST_ASSERT_EQUAL_UINT(0, upload.append(1, 12, args));
    args["offset"] = 0;
    args["key"] = 10;
    for (const auto bad : {"", "000000000000", "00g000080000", "000100000005",
                           "000000000002000203e80000", "03e803e80001",
                           "006400642711", "000000010001", "0000002a0000002900290001"}) {
        args["data"] = bad;
        TEST_ASSERT_EQUAL_UINT(0, upload.append(1, 12, args));
    }
    args["modifierKey"] = 99;
    args["modifierCount"] = 128;
    TEST_ASSERT_TRUE(bool(upload.take(1, 12, args)));
}

void test_modified_slots_round_trip_and_keep_original_on_failure() {
    using drawn_pattern::Modifiers;
    SavedPatternLibrary library;
    const auto waveform = curve(3000);
    TEST_ASSERT_TRUE(library.save(0, "Base", 4, waveform, writeRecord));
    const auto legacy = flash.at("libraryV3");
    Modifiers effects;
    effects.count = 2;
    effects.entries[0] = {250, 500, 0};
    effects.entries[1] = {750, 750, 2000};
    TEST_ASSERT_TRUE(library.save(1, "With effects", 12, waveform, writeRecord, effects));
    TEST_ASSERT_TRUE(library.select(1, writeRecord));
    TEST_ASSERT_EQUAL_UINT(2, library.catalog().slots[1].modifierCount);
    TEST_ASSERT_TRUE(legacy == flash.at("libraryV3"));
    TEST_ASSERT_EQUAL_UINT(1192 + 8 + 12, flash.at("libraryV4").size());
    SavedPatternLibrary restored;
    TEST_ASSERT_TRUE(restored.load(readRecord));
    SavedPattern saved;
    Modifiers retrieved;
    TEST_ASSERT_TRUE(restored.get(saved, selectedPatternSlot, &retrieved));
    TEST_ASSERT_EQUAL_MEMORY(effects.entries, retrieved.entries, 2*sizeof(drawn_pattern::Modifier));
    const int committed = writes;
    TEST_ASSERT_TRUE(restored.save(1, nullptr, 0, waveform, writeRecord, effects));
    TEST_ASSERT_EQUAL_INT(committed, writes);
    failWrite = true;
    TEST_ASSERT_FALSE(restored.save(1, nullptr, 0, waveform, writeRecord));
    TEST_ASSERT_EQUAL_UINT(2, restored.catalog().slots[1].modifierCount);
    failWrite = false;
    TEST_ASSERT_TRUE(restored.save(1, nullptr, 0, waveform, writeRecord));
    TEST_ASSERT_TRUE(restored.load(readRecord));
    TEST_ASSERT_TRUE(restored.get(saved, 1, &retrieved));
    TEST_ASSERT_EQUAL_UINT(0, retrieved.count);
    TEST_ASSERT_EQUAL_UINT(0, restored.catalog().slots[0].modifierCount);
    flash["libraryV4"].pop_back();
    TEST_ASSERT_FALSE(restored.load(readRecord)); // A partial V4 never silently revives stale V3.
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_modifier_chunks_are_bound_to_one_lease_and_atomic_across_retries);
    RUN_TEST(test_modifier_upload_accepts_capacity_and_rejects_bad_or_conflicting_input);
    RUN_TEST(test_modified_slots_round_trip_and_keep_original_on_failure);
    RUN_TEST(test_legacy_blob_loads_without_writing_or_starting_and_survives_upgrade);
    RUN_TEST(test_first_slot_default_named_select_and_v1_aliases_survive_reboot);
    RUN_TEST(test_failed_or_repeated_transactions_preserve_selection_and_slot);
    RUN_TEST(test_bounded_names_slots_and_corrupt_flash);
    RUN_TEST(test_q16_upgrade_preserves_all_prior_slot_bytes_and_selected_name);
    RUN_TEST(test_sparse_samples_cover_one_cycle_and_survive_named_save_and_reboot);
    RUN_TEST(test_invalid_numeric_and_overlapping_formats_are_rejected_at_the_ble_boundary);
    RUN_TEST(test_storage_requires_stationary_current_feedback_when_available);
    RUN_TEST(test_catalog_optional_slot_preserves_full_and_empty_entries);
    RUN_TEST(test_catalog_rejects_invalid_page_without_returning_other_entries);
    RUN_TEST(test_catalog_escaped_maximum_names_fit_correlated_ble_response_when_paged);
    RUN_TEST(test_index_uses_storage_only_for_explicit_requests_and_failed_io_preserves_catalog);
    return UNITY_END();
}
