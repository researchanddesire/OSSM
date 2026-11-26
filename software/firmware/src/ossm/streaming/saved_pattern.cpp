#include "saved_pattern.h"

#include <mutex>
#include <Preferences.h>

#include "drawn_pattern.h"
#include "services/motorSafety.h"
#include "services/stepper.h"

namespace streaming {
namespace {
std::mutex savedLock;
SavedPatternLibrary library;
constexpr char storageNamespace[] = "drawnPattern";

// Both mutation entry points are state-machine menu actions. The state
// machine holds its mode-transition lock through the atomic NVS write.
bool stopped() {
    return !getMotorSafetyStatus().armed && !(stepper && stepper->isRunning());
}
bool writeRecord(const char *key, const void *data, size_t size) {
    return runStoppedMotorStorage([&] {
        Preferences nvs;
        return nvs.begin(storageNamespace, false) && nvs.putBytes(key, data, size) == size;
    });
}
}

void loadSavedPattern() {
    Preferences nvs;
    std::lock_guard<std::mutex> lock(savedLock);
    if (!nvs.begin(storageNamespace, true)) return;
    library.load([&](const char *key, void *data, size_t size) {
        return nvs.getBytesLength(key) == size && nvs.getBytes(key, data, size) == size;
    });
}
bool getSavedPattern(SavedPattern &out, int slot) {
    std::lock_guard<std::mutex> lock(savedLock);
    return library.get(out, slot);
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
    event.written = library.save(event.slot, event.name, event.nameLength, event.value, writeRecord);
}
void selectPatternInMenu(const SelectPattern &event) {
    if (!stopped()) return;
    std::lock_guard<std::mutex> lock(savedLock);
    event.written = library.select(event.slot, writeRecord);
}

void toggleWiredPattern() {
    if (drawnPatternActive()) {
        cancelDrawnPattern();
        return;
    }
    SavedPattern value;
    if (getSavedPattern(value)) {
        startDrawnPattern(value.samples, value.length, value.periodMs,
                          getMotorSafetyStatus().controlRevision, true);
    }
}
}  // namespace streaming
