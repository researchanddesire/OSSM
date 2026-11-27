#include "saved_pattern.h"

#include <mutex>
#include <Preferences.h>
#include <nvs.h>

#include "drawn_pattern.h"
#include "services/motorSafety.h"
#include "services/stepper.h"

namespace streaming {
namespace {
std::mutex savedLock;
SavedPatternCache library;
constexpr char storageNamespace[] = "drawnPattern";

// Both mutation entry points are state-machine menu actions. The state
// machine holds its mode-transition lock through the atomic NVS write.
bool stopped() {
    return !getMotorSafetyStatus().armed && !(stepper && stepper->isRunning());
}
// Missing/invalid records retain the existing empty/legacy fallback. An I/O
// failure aborts the transaction, so a partial read cannot replace valid NVS.
bool loadRecord(SavedPatternLibrary &record) {
    nvs_handle_t handle;
    const auto opened = nvs_open(storageNamespace, NVS_READONLY, &handle);
    if (opened == ESP_ERR_NVS_NOT_FOUND) return true; // New namespace.
    if (opened != ESP_OK) return false;
    bool readable = true;
    record.load([&](const char *key, void *data, size_t size) {
        size_t actual = size;
        const auto error = nvs_get_blob(handle, key, data, &actual);
        if (error == ESP_OK) return actual == size;
        if (error != ESP_ERR_NVS_NOT_FOUND && error != ESP_ERR_NVS_INVALID_LENGTH)
            readable = false;
        return false;
    });
    nvs_close(handle);
    return readable;
}
bool writeRecord(const char *key, const void *data, size_t size) {
    return runStoppedMotorStorage([&] {
        Preferences nvs;
        return nvs.begin(storageNamespace, false) && nvs.putBytes(key, data, size) == size;
    });
}
}

void loadSavedPattern() {
    std::lock_guard<std::mutex> lock(savedLock);
    library.access(loadRecord, [](const SavedPatternLibrary &) { return true; });
}
bool getSavedPattern(SavedPattern &out, int slot) {
    std::lock_guard<std::mutex> lock(savedLock);
    return library.access(loadRecord, [&](const SavedPatternLibrary &record) { return record.get(out, slot); });
}
bool hasSavedPattern() {
    std::lock_guard<std::mutex> lock(savedLock);
    return library.any();
}
PatternCatalog getPatternCatalog() {
    std::lock_guard<std::mutex> lock(savedLock);
    return library.catalog();
}
void storePatternInMenu(const SavePattern &event) {
    if (!stopped()) return;
    std::lock_guard<std::mutex> lock(savedLock);
    event.written = library.access(loadRecord, [&](SavedPatternLibrary &record) {
        return record.save(event.slot, event.name, event.nameLength, event.value, writeRecord);
    });
}
void selectPatternInMenu(const SelectPattern &event) {
    if (!stopped()) return;
    std::lock_guard<std::mutex> lock(savedLock);
    event.written = library.access(loadRecord, [&](SavedPatternLibrary &record) {
        return record.select(event.slot, writeRecord);
    });
}

void toggleWiredPattern() {
    if (drawnPatternActive()) {
        cancelDrawnPattern();
        return;
    }
    const auto revision = getMotorSafetyStatus().controlRevision;
    if (drawnPatternStartStatus(revision) != PatternStart::Accepted) return;
    SavedPattern value;
    if (getSavedPattern(value)) {
        startDrawnPattern(value.samples, value.length, value.periodMs, revision, true, value.format());
    }
}
}  // namespace streaming
