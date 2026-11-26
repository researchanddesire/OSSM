#include "saved_pattern.h"

#include <cstring>
#include <mutex>
#include <Preferences.h>

#include "drawn_pattern.h"
#include "services/motorSafety.h"
#include "services/stepper.h"

namespace streaming {
namespace {
std::mutex savedLock;
SavedPattern saved;
constexpr char storageNamespace[] = "drawnPattern";
constexpr char storageKey[] = "curveV1";

bool valid(const SavedPattern &value) {
    drawn_pattern::Waveform curve;
    return value.version == 1 && value.length <= sizeof(value.samples) &&
           curve.decode(value.samples, value.length, value.periodMs);
}
}

bool makeSavedPattern(SavedPattern &out, const char *samples, size_t length,
                      uint32_t periodMs) {
    if (!samples || length > sizeof(out.samples)) return false;
    out.periodMs = periodMs;
    out.length = length;
    std::memcpy(out.samples, samples, length);
    return valid(out);
}

void loadSavedPattern() {
    Preferences nvs;
    if (!nvs.begin(storageNamespace, true)) return;
    SavedPattern value;
    if (nvs.getBytesLength(storageKey) == sizeof(value) &&
        nvs.getBytes(storageKey, &value, sizeof(value)) == sizeof(value) &&
        valid(value)) {
        std::lock_guard<std::mutex> lock(savedLock);
        saved = value;
    }
}

bool getSavedPattern(SavedPattern &out) {
    std::lock_guard<std::mutex> lock(savedLock);
    if (!saved.length) return false;
    out = saved;
    return true;
}

bool hasSavedPattern() {
    std::lock_guard<std::mutex> lock(savedLock);
    return saved.length != 0;
}

// An internal menu event holds the same lock as mode entry and homing through
// the flash commit. No motor operation can start during an NVS cache stall.
void storePatternInMenu(const SavePattern &event) {
    if (getMotorSafetyStatus().armed || (stepper && stepper->isRunning())) return;
    std::lock_guard<std::mutex> lock(savedLock);
    const auto &value = event.value;
    if (saved.length == value.length && saved.periodMs == value.periodMs &&
        std::memcmp(saved.samples, value.samples, value.length) == 0) {
        event.written = true;
        return;
    }
    Preferences nvs;
    if (!nvs.begin(storageNamespace, false) ||
        nvs.putBytes(storageKey, &value, sizeof(value)) != sizeof(value)) return;
    saved = value;
    event.written = true;
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
