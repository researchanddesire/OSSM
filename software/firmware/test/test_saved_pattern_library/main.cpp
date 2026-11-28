#include <unity.h>
#include <map>
#include <string>
#include <vector>
#include "saved_pattern_library.h"
#include "pattern_catalog_json.h"
#include "../../src/ossm/streaming/pattern_speed_json.h"
#include "pattern_modifier_upload.h"

namespace {
bool refuseNextMarkerPage = false;
}

// Inject failure only into the next 64-record marker page, leaving ArduinoJson
// and the rest of the native record/storage implementation unchanged.
void *operator new(std::size_t bytes, const std::nothrow_t &) noexcept {
    if (refuseNextMarkerPage && bytes == 64 * sizeof(drawn_pattern::Modifier)) {
        refuseNextMarkerPage = false;
        return nullptr;
    }
    return ::operator new(bytes);
}

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
    TEST_ASSERT_EQUAL_UINT32(60000, recovered.periodMs);
    const auto full = waveform.pace(100, 1000, 10000, 1., 2);
    const auto quarter = waveform.pace(100, 1000, 10000, .25, 2);
    TEST_ASSERT_TRUE(full.rate > 0 && full.peak <= 1000);
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, full.rate * .25, quarter.rate);
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
    for (unsigned offset = 0; offset < 1000; offset += 32) {
        std::string data;
        for (unsigned i = offset; i < std::min(1000u, offset + 32); ++i) {
            snprintf(hex, sizeof(hex), "%04x%04x0001", i, i);
            data += hex;
        }
        args["offset"] = offset;
        args["data"] = data;
        TEST_ASSERT_EQUAL_UINT(std::min(1000u, offset + 32), upload.append(1, 12, args));
    }
    args["offset"] = 1000;
    args["data"] = "008000800001";
    TEST_ASSERT_EQUAL_UINT(0, upload.append(1, 12, args));
    args["offset"] = 0;
    args["key"] = 10;
    for (const auto bad : {"", "000000000000", "00g000080000", "000100000005",
                           "000000000002000203e80000", "03e803e80001",
                           "00640064ea61", "00640064ffff", "ffff03e80000", "000000010001", "0000002a0000002900290001"}) {
        args["data"] = bad;
        TEST_ASSERT_EQUAL_UINT(0, upload.append(1, 12, args));
    }
    args["modifierKey"] = 99;
    args["modifierCount"] = 1000;
    TEST_ASSERT_TRUE(bool(upload.take(1, 12, args)));
}

void test_dense_24_record_packets_retry_allocation_and_move_without_copying_pages() {
    using namespace drawn_pattern;
    PatternModifierUpload upload;
    JsonDocument doc;
    const auto args = doc.to<JsonObject>();
    args["key"] = 42;
    bool outOfMemory = false;
    const uint16_t special[] = {1, 30000, 32767, 32768, 60000};
    for (unsigned offset = 0; offset < 1000; offset += 24) {
        std::string data;
        const auto end = std::min(offset + 24, 1000u);
        for (unsigned i = offset; i < end; ++i) {
            char hex[13];
            const unsigned ms = i < 5 ? special[i] : 1 + i * 97 % 60000;
            snprintf(hex, sizeof(hex), "%04x%04x%04x", i, i, ms);
            data += hex;
        }
        args["offset"] = offset;
        args["data"] = data;
        if (offset == 768) {
            refuseNextMarkerPage = true;
            TEST_ASSERT_EQUAL_UINT(0, upload.append(1, 12, args, &outOfMemory));
            TEST_ASSERT_TRUE(outOfMemory);
            TEST_ASSERT_FALSE(refuseNextMarkerPage);
            args["data"] = "030003000000";
            TEST_ASSERT_EQUAL_UINT(0, upload.append(1, 12, args, &outOfMemory));
            TEST_ASSERT_FALSE(outOfMemory);
            args["data"] = data;
        }
        TEST_ASSERT_EQUAL_UINT(end, upload.append(1, 12, args, &outOfMemory));
        TEST_ASSERT_FALSE(outOfMemory);
        TEST_ASSERT_EQUAL_UINT(end, upload.append(1, 12, args));
    }
    args["modifierKey"] = 42;
    args["modifierCount"] = 1000;
    auto uploaded = upload.take(1, 12, args);
    TEST_ASSERT_NOT_NULL(uploaded.get());
    const auto *page = &uploaded->entries[768];
    Modifiers owned;
    owned.takeFrom(*uploaded);
    TEST_ASSERT_EQUAL_PTR(page, &owned.entries[768]);
    TEST_ASSERT_EQUAL_UINT(1000, owned.count);
    TEST_ASSERT_EQUAL_UINT(0, uploaded->count);
    TEST_ASSERT_TRUE(uploaded->reserve(5));
    TEST_ASSERT_EQUAL_UINT(60000, owned.entries[4].holdMs);
    TEST_ASSERT_EQUAL_UINT(1 + 999 * 97 % 60000, owned.entries[999].holdMs);
    TEST_ASSERT_TRUE(owned.valid());
}

void test_v1_upload_supports_typed_ranges_and_long_pauses_with_stable_ids() {
    using namespace drawn_pattern;
    PatternModifierUpload upload;
    JsonDocument doc;
    auto args = doc.to<JsonObject>();
    args["key"] = 12;
    args["offset"] = 0;
    args["data"] = "0000000a0000001400147530001e001eea60";
    TEST_ASSERT_EQUAL_UINT(3, upload.append(1, 2, args));
    args["modifierKey"] = 12;
    args["modifierCount"] = 3;
    auto items = upload.take(1, 2, args);
    TEST_ASSERT_NOT_NULL(items.get());
    TEST_ASSERT_TRUE(items->entries[0].geometry() == EffectGeometry::TimeRange);
    TEST_ASSERT_TRUE(items->entries[0].range().kind == TimeRangeEffect::Kind::Vibration);
    TEST_ASSERT_TRUE(items->entries[1].geometry() == EffectGeometry::Instant);
    TEST_ASSERT_TRUE(items->entries[1].point().kind == InstantEffect::Kind::Pause);
    TEST_ASSERT_EQUAL_UINT16(30000, items->entries[1].point().maximumMs);
    TEST_ASSERT_EQUAL_UINT16(60000, items->entries[2].point().maximumMs);
    for (uint16_t id : {33, 65, 257, 1025, 2047, 2049}) {
        const Modifier range(TimeRangeEffect{static_cast<TimeRangeEffect::Kind>(id), 0, 2});
        const Modifier point(InstantEffect{static_cast<InstantEffect::Kind>(id), 0, 30000});
        TEST_ASSERT_EQUAL_UINT16(id <= 2047 ? id : 0, uint16_t(range.range().kind));
        TEST_ASSERT_EQUAL_UINT16(id <= 2047 ? id : 0, uint16_t(point.point().kind));
        TEST_ASSERT_FALSE(range.valid());
        TEST_ASSERT_FALSE(range.isVibration());
        TEST_ASSERT_FALSE(point.valid());
        TEST_ASSERT_FALSE(point.isPause());
    }
    TEST_ASSERT_FALSE(Modifier::fromV1(1, 1024 + 2, 0).valid());
}

void test_every_dense_word_has_one_meaning_and_v4_rejects_long_pauses() {
    using namespace drawn_pattern;
    using V = StoredModifiers::Version;
    unsigned pauses = 0, ranges = 0, legacy = 0;
    Modifier item;
    for (uint32_t word = 0; word <= UINT16_MAX; ++word) {
        if (StoredModifiers::decodeWord(0, word, V::V4, item)) ++legacy;
        if (!StoredModifiers::decodeWord(0, word, V::V5, item)) continue;
        TEST_ASSERT_EQUAL_UINT16(word, StoredModifiers::denseWord(item));
        if (item.isVibration()) ++ranges;
        if (item.isPause()) ++pauses;
    }
    TEST_ASSERT_EQUAL_UINT(1000, ranges);
    TEST_ASSERT_EQUAL_UINT(60000, pauses);
    TEST_ASSERT_EQUAL_UINT(11000, legacy);
    for (uint32_t ms = 1; ms <= 60000; ++ms) {
        Modifier authored{250, 250, uint16_t(ms)};
        TEST_ASSERT_TRUE(StoredModifiers::decodeWord(250, StoredModifiers::denseWord(authored), V::V5, item));
        if (!item.isPause() || item.holdMs != ms) TEST_FAIL_MESSAGE("dense pause duration changed");
    }
    for (uint16_t word : {0, 0x8000, 61002, 65535})
        TEST_ASSERT_FALSE(StoredModifiers::decodeWord(0, word, V::V5, item));
    TEST_ASSERT_FALSE(StoredModifiers::decodeWord(600, 0x8000 | 600, V::V5, item));
}

void test_sparse_and_dense_reboots_keep_range_tags_and_long_pause_boundaries() {
    using namespace drawn_pattern;
    const uint16_t ms[] = {1, 2400, 10000, 10001, 30000, 32767, 32768, 33768, 33769, 60000};
    SavedPatternLibrary library;
    const auto pattern = curve(2200);
    Modifiers markers, loaded;
    SavedPattern saved;
    TEST_ASSERT_TRUE(markers.reserve(1000));
    unsigned slot = 0;
    for (uint16_t count : {333, 334, 1000}) {
        markers.count = count;
        for (uint16_t i = 0; i < count; ++i)
            markers.entries[i] = {i, i, ms[i % 10]};
        // At 1,000 occupied starts there is no free tick between a range end
        // and adjacent pauses. Exercise a genuine range in the two mixed slots.
        if (count < 1000) markers.entries[count - 1] = {uint16_t(count), 999, 0};
        TEST_ASSERT_TRUE(library.save(slot, "Mixed", 5, pattern, writeRecord, markers));
        const auto key = std::string("slot4_") + std::to_string(slot);
        auto &blob = flash.at(key);
        TEST_ASSERT_EQUAL_UINT(304 + std::min<unsigned>(6 * count, 2000), blob.size());
        TEST_ASSERT_EQUAL_UINT8(0x35, blob[1]);
        TEST_ASSERT_TRUE(library.load(readRecord));
        TEST_ASSERT_TRUE(library.get(saved, slot, &loaded));
        for (unsigned i = 0; i < count; ++i)
            TEST_ASSERT_EQUAL_MEMORY(&markers.entries[i], &loaded.entries[i], sizeof(Modifier));
        // The exact same bytes under an old V4 header are malformed.
        blob[1] = 0x34;
        TEST_ASSERT_TRUE(library.load(readRecord));
        TEST_ASSERT_FALSE(library.get(saved, slot, &loaded));
        blob[1] = 0x35;
        TEST_ASSERT_TRUE(library.load(readRecord));
        ++slot;
    }
    markers.count = 2;
    markers.entries[0] = {0, 10, 0};
    markers.entries[1] = {200, 200, 2400};
    TEST_ASSERT_TRUE(library.save(3, "Short", 5, pattern, writeRecord, markers));
    const auto &blob = flash.at("slot4_3");
    const uint16_t legacyWords[] = {0, 10, 0, 200, 200, 2400};
    TEST_ASSERT_EQUAL_UINT8(0x34, blob[1]);
    TEST_ASSERT_EQUAL_MEMORY(legacyWords, blob.data() + 304, sizeof(legacyWords));
}

void test_capabilities_do_not_displace_stop_diagnostics_from_a_full_reply() {
    JsonDocument reply;
    addModifierCapabilities(reply);
    TEST_ASSERT_EQUAL_UINT(1, reply["modifierVersion"].as<unsigned>());
    TEST_ASSERT_EQUAL_UINT(1000, reply["maxModifiers"].as<unsigned>());
    TEST_ASSERT_EQUAL_UINT(60000, reply["maxPauseMs"].as<unsigned>());
    reply["lastStop"]["reason"] = "scheduler_gap";
    reply["diagnostic"] = std::string(279,'x');
    addModifierCapabilities(reply);
    TEST_ASSERT_TRUE(reply["maxModifiers"].isUnbound());
    TEST_ASSERT_TRUE(reply["maxPauseMs"].isUnbound());
    TEST_ASSERT_EQUAL_STRING("scheduler_gap", reply["lastStop"]["reason"].as<const char*>());
    TEST_ASSERT_LESS_OR_EQUAL_UINT(347, measureJson(reply));
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
    TEST_ASSERT_EQUAL_UINT(304 + 12, flash.at("slot4_1").size());
    TEST_ASSERT_EQUAL_UINT(4, flash.at("selected4").size());
    SavedPatternLibrary restored;
    TEST_ASSERT_TRUE(restored.load(readRecord));
    SavedPattern saved;
    Modifiers retrieved;
    TEST_ASSERT_TRUE(restored.get(saved, selectedPatternSlot, &retrieved));
    for (unsigned i = 0; i < effects.count; ++i)
        TEST_ASSERT_EQUAL_MEMORY(&effects.entries[i], &retrieved.entries[i], sizeof(drawn_pattern::Modifier));
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
    flash["slot4_1"].pop_back();
    TEST_ASSERT_TRUE(restored.load(readRecord));
    TEST_ASSERT_FALSE(restored.get(saved, 1, &retrieved));
    TEST_ASSERT_FALSE(restored.get(saved, selectedPatternSlot, &retrieved));
    TEST_ASSERT_TRUE(restored.get(saved, 0, &retrieved));
    TEST_ASSERT_FALSE(restored.save(1, nullptr, 0, waveform, writeRecord));
    TEST_ASSERT_FALSE(restored.save(selectedPatternSlot, nullptr, 0, waveform, writeRecord));
    const auto neighbor = flash.at("libraryV3");
    TEST_ASSERT_TRUE(restored.save(1, "Repaired", 8, waveform, writeRecord));
    TEST_ASSERT_TRUE(restored.load(readRecord));
    TEST_ASSERT_TRUE(restored.get(saved, selectedPatternSlot, &retrieved));
    TEST_ASSERT_EQUAL_STRING("Repaired", restored.catalog().slots[1].name);
    TEST_ASSERT_TRUE(neighbor == flash.at("libraryV3"));
}

void test_all_four_slots_store_and_update_every_tick_independently() {
    drawn_pattern::Modifiers all;
    TEST_ASSERT_TRUE(all.reserve(1000));
    all.count = 1000;
    SavedPatternLibrary library;
    const auto waveform = curve(5000);
    for (unsigned slot = 0; slot < patternSlotCount; ++slot) {
        for (uint16_t tick = 0; tick < all.count; ++tick) {
            all.entries[tick] = slot % 2
                ? drawn_pattern::Modifier{tick, uint16_t(tick+1), 0}
                : drawn_pattern::Modifier{tick, tick, uint16_t(tick+1+slot)};
        }
        TEST_ASSERT_TRUE(library.save(slot, "Full grid", 9, waveform, writeRecord, all));
    }
    for (const auto *key : {"slot4_0", "slot4_1", "slot4_2", "slot4_3"})
        TEST_ASSERT_EQUAL_UINT(2304, flash.at(key).size());
    const auto beforeSelect = flash;
    TEST_ASSERT_TRUE(library.select(3, writeRecord));
    for (const auto &item : beforeSelect) TEST_ASSERT_TRUE(item.second == flash.at(item.first));
    TEST_ASSERT_EQUAL_UINT(4, flash.at("selected4").size());
    SavedPatternLibrary reboot;
    TEST_ASSERT_TRUE(reboot.load(readRecord));
    SavedPattern restored;
    drawn_pattern::Modifiers read;
    for (unsigned slot = 0; slot < patternSlotCount; ++slot) {
        TEST_ASSERT_TRUE(reboot.get(restored, slot, &read));
        TEST_ASSERT_EQUAL_UINT(1000, read.count);
        for (uint16_t tick = 0; tick < read.count; ++tick) {
            TEST_ASSERT_EQUAL_UINT(tick, read.entries[tick].start);
            TEST_ASSERT_EQUAL_UINT(tick+(slot%2), read.entries[tick].end);
            TEST_ASSERT_EQUAL_UINT(slot%2 ? 0 : tick+1+slot, read.entries[tick].holdMs);
        }
    }
    // Keep the size and count syntactically plausible but remove one cell.
    flash["slot4_0"][304] = flash["slot4_0"][305] = 0;
    TEST_ASSERT_TRUE(reboot.load(readRecord));
    TEST_ASSERT_FALSE(reboot.get(restored, 0, &read));
    TEST_ASSERT_TRUE(reboot.get(restored, 3, &read));
    TEST_ASSERT_EQUAL_UINT(1000, read.count);
}

void test_first_modified_save_and_selection_are_single_atomic_writes() {
    drawn_pattern::Modifiers effects;
    effects.count = 1;
    effects.entries[0] = {200, 200, 300};
    SavedPatternLibrary lib;
    const auto s = curve(2100);
    failWrite = true;
    TEST_ASSERT_FALSE(lib.save(3, "First", 5, s, writeRecord, effects));
    TEST_ASSERT_TRUE(flash.empty());
    TEST_ASSERT_FALSE(lib.any());
    failWrite = false;
    TEST_ASSERT_TRUE(lib.save(3, "First", 5, s, writeRecord, effects));
    TEST_ASSERT_EQUAL_UINT(1, flash.size());
    TEST_ASSERT_EQUAL_UINT(1, flash.count("slot4_3"));
    TEST_ASSERT_TRUE(lib.save(0, "Second", 6, s, writeRecord, effects));
    SavedPatternLibrary reboot;
    TEST_ASSERT_TRUE(reboot.load(readRecord, selectedPatternSlot));
    TEST_ASSERT_EQUAL_UINT(3, reboot.catalog().selected);
    SavedPattern value;
    drawn_pattern::Modifiers restored;
    TEST_ASSERT_TRUE(reboot.get(value, selectedPatternSlot, &restored));
    TEST_ASSERT_EQUAL_UINT(1, restored.count);
    TEST_ASSERT_FALSE(reboot.get(value, 0, &restored));  // Only the selected payload was retained.
    const auto before = flash;
    failWrite = true;
    TEST_ASSERT_FALSE(reboot.select(0, writeRecord));
    TEST_ASSERT_TRUE(before == flash);
    failWrite = false;
    TEST_ASSERT_TRUE(reboot.select(0, writeRecord));
    TEST_ASSERT_TRUE(reboot.load(readRecord, 0));
    TEST_ASSERT_TRUE(reboot.get(value, 0, &restored));
    TEST_ASSERT_EQUAL_UINT(0, reboot.catalog().selected);
}

void test_corrupt_implicit_first_slot_cannot_redirect_wired_recall_to_its_neighbor() {
    drawn_pattern::Modifiers effects;
    effects.count = 1;
    effects.entries[0] = {200, 300, 0};
    SavedPatternLibrary lib;
    auto value = curve(3500);
    TEST_ASSERT_TRUE(lib.save(3, "Original", 8, value, writeRecord, effects));
    TEST_ASSERT_TRUE(lib.save(0, "Neighbor", 8, value, writeRecord, effects));
    flash["slot4_3"][0] ^= 0xff;
    TEST_ASSERT_TRUE(lib.load(readRecord));
    TEST_ASSERT_FALSE(lib.any());
    TEST_ASSERT_FALSE(lib.catalog().selectedAvailable);
    TEST_ASSERT_FALSE(lib.get(value, selectedPatternSlot));
    TEST_ASSERT_TRUE(lib.get(value, 0));
    TEST_ASSERT_TRUE(lib.select(0, writeRecord));
    TEST_ASSERT_TRUE(lib.load(readRecord));
    TEST_ASSERT_TRUE(lib.any());
    TEST_ASSERT_TRUE(lib.get(value, selectedPatternSlot));
}

void test_invalid_selector_preserves_explicit_slots_and_requires_deliberate_reselection() {
    drawn_pattern::Modifiers effects;
    effects.count = 1;
    effects.entries[0] = {400, 400, 99};
    SavedPatternLibrary lib;
    const auto value = curve(3000);
    TEST_ASSERT_TRUE(lib.save(2, "Earlier", 7, value, writeRecord));
    TEST_ASSERT_TRUE(lib.save(0, "Marked", 6, value, writeRecord, effects));
    TEST_ASSERT_TRUE(lib.select(0, writeRecord));
    flash["selected4"] = {0x7f, 0};
    const auto slot = flash.at("slot4_0");
    SavedPattern read;
    TEST_ASSERT_TRUE(lib.load(readRecord));
    TEST_ASSERT_FALSE(lib.get(read));
    TEST_ASSERT_TRUE(lib.get(read, 2));
    TEST_ASSERT_TRUE(lib.select(2, writeRecord));
    TEST_ASSERT_TRUE(lib.load(readRecord));
    TEST_ASSERT_TRUE(lib.get(read));
    TEST_ASSERT_EQUAL_UINT(2, lib.catalog().selected);
    TEST_ASSERT_TRUE(slot == flash.at("slot4_0"));
}

void test_live_speed_telemetry_fits_one_heartbeat_with_legacy_fields() {
    PatternStatus playback{};
    playback.active = true;
    playback.periodMs = UINT32_MAX;
    playback.speed.state = "running";
    playback.speed.limit = "acceleration";
    playback.speed.speedPct = 99.987655;
    playback.speed.nominalMmS = 999.87655;
    playback.speed.plannedPeakMmS = 123.45678;
    playback.speed.periodMs = playback.periodMs;
    playback.speed.phaseRateHz = 1000.0 / UINT32_MAX;
    JsonDocument status;
    writePatternStatus(status, playback, false, false);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(347, measureJson(status));
    TEST_ASSERT_EQUAL_INT(1, status["version"].as<int>());
    TEST_ASSERT_EQUAL_INT(2, status["speedLaw"].as<int>());
    TEST_ASSERT_TRUE(status["hasSpeedKnob"].is<bool>());
    TEST_ASSERT_TRUE(status["stored"].is<bool>());
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, status["periodMs"].as<uint32_t>());
    const auto plan = status["speedPlan"].as<JsonArrayConst>();
    TEST_ASSERT_EQUAL_UINT32(5, plan.size());
    TEST_ASSERT_FLOAT_WITHIN(.00001, playback.speed.speedPct, plan[0].as<double>());
    TEST_ASSERT_FLOAT_WITHIN(.00001, 999.9, plan[1].as<double>());
    TEST_ASSERT_FLOAT_WITHIN(.00001, 123.5, plan[2].as<double>());
    TEST_ASSERT_EQUAL_STRING("running", plan[3].as<const char *>());
    TEST_ASSERT_EQUAL_STRING("acceleration", plan[4].as<const char *>());
}

void test_paused_speed_is_indefinite_and_long_stop_keeps_its_diagnostics() {
    PatternStatus playback{};
    playback.active = true;
    playback.speed.state = "paused";
    JsonDocument heartbeat;
    writePatternStatus(heartbeat, playback, true, true);
    TEST_ASSERT_TRUE(heartbeat["active"].as<bool>());
    TEST_ASSERT_EQUAL_UINT32(0, heartbeat["periodMs"].as<uint32_t>());
    TEST_ASSERT_EQUAL_STRING("paused", heartbeat["speedPlan"][3].as<const char *>());

    JsonDocument detail;
    writePatternSpeed(detail, playback.speed);
    TEST_ASSERT_TRUE(detail["periodMs"].is<JsonVariantConst>());
    TEST_ASSERT_TRUE(detail["periodMs"].isNull());
    TEST_ASSERT_EQUAL_FLOAT(0, detail["phaseRateHz"].as<float>());
    TEST_ASSERT_EQUAL_STRING("paused", detail["state"].as<const char *>());

    playback.active = false;
    playback.lastStop = {"pulse_underrun","positioning",UINT32_MAX,UINT32_MAX,INT8_MIN,UINT32_MAX};
    JsonDocument stopped;
    writePatternStatus(stopped, playback, false, false);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(347, measureJson(stopped));
    TEST_ASSERT_FALSE(stopped["speedPlan"].is<JsonArrayConst>());
    TEST_ASSERT_EQUAL_STRING("pulse_underrun", stopped["lastStop"]["reason"].as<const char *>());
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, stopped["lastStop"]["frameGapMs"].as<uint32_t>());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_live_speed_telemetry_fits_one_heartbeat_with_legacy_fields);
    RUN_TEST(test_paused_speed_is_indefinite_and_long_stop_keeps_its_diagnostics);
    RUN_TEST(test_every_dense_word_has_one_meaning_and_v4_rejects_long_pauses);
    RUN_TEST(test_sparse_and_dense_reboots_keep_range_tags_and_long_pause_boundaries);
    RUN_TEST(test_v1_upload_supports_typed_ranges_and_long_pauses_with_stable_ids);
    RUN_TEST(test_invalid_selector_preserves_explicit_slots_and_requires_deliberate_reselection);
    RUN_TEST(test_first_modified_save_and_selection_are_single_atomic_writes);
    RUN_TEST(test_corrupt_implicit_first_slot_cannot_redirect_wired_recall_to_its_neighbor);
    RUN_TEST(test_all_four_slots_store_and_update_every_tick_independently);
    RUN_TEST(test_modifier_chunks_are_bound_to_one_lease_and_atomic_across_retries);
    RUN_TEST(test_dense_24_record_packets_retry_allocation_and_move_without_copying_pages);
    RUN_TEST(test_modifier_upload_accepts_capacity_and_rejects_bad_or_conflicting_input);
    RUN_TEST(test_capabilities_do_not_displace_stop_diagnostics_from_a_full_reply);
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
