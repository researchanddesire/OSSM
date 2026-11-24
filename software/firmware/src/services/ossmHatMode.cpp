#include "ossmHatMode.h"

#if defined(ARDUINO) && !defined(PIO_UNIT_TESTING)
#include <atomic>
#include <nvs.h>

namespace {
using ossm_hat_mode::Mode;

constexpr const char *storageNamespace = "ossmHat";
constexpr const char *storageKey = "mode";
constexpr int unknown = -1;

// Fixed by the first read in a boot. A concurrent first read sees the same
// flash value, so either reader may publish it.
std::atomic<int> bootMode{unknown};
// What NVS holds for the next boot. Unknown after a read error, so the next
// command always writes.
std::atomic<int> storedMode{unknown};

int readStored() {
    nvs_handle_t handle;
    const esp_err_t opened = nvs_open(storageNamespace, NVS_READONLY, &handle);
    if (opened == ESP_ERR_NVS_NOT_FOUND) {
        storedMode.store(static_cast<int>(Mode::Auto));
        return static_cast<int>(Mode::Auto);
    }
    if (opened != ESP_OK) return static_cast<int>(Mode::Auto);
    char value[8] = {};
    size_t length = sizeof(value);
    const esp_err_t read = nvs_get_str(handle, storageKey, value, &length);
    nvs_close(handle);
    const Mode mode = read == ESP_OK ? ossm_hat_mode::fromStored(value) : Mode::Auto;
    if (read == ESP_OK || read == ESP_ERR_NVS_NOT_FOUND)
        storedMode.store(static_cast<int>(mode));
    return static_cast<int>(mode);
}
}  // namespace

Mode ossmHatBootMode() {
    int mode = bootMode.load();
    if (mode == unknown) {
        int expected = unknown;
        bootMode.compare_exchange_strong(expected, readStored());
        mode = bootMode.load();
    }
    return static_cast<Mode>(mode);
}

Mode ossmHatStoredMode() {
    ossmHatBootMode();
    const int stored = storedMode.load();
    return stored == unknown ? ossmHatBootMode() : static_cast<Mode>(stored);
}

bool storeOssmHatMode(Mode mode) {
    // Fix this boot's mode before the stored setting changes.
    ossmHatBootMode();
    // Repeated commands do not rewrite flash.
    if (storedMode.load() == static_cast<int>(mode)) return true;
    nvs_handle_t handle;
    if (nvs_open(storageNamespace, NVS_READWRITE, &handle) != ESP_OK) return false;
    const bool stored = nvs_set_str(handle, storageKey, ossm_hat_mode::name(mode)) == ESP_OK &&
                        nvs_commit(handle) == ESP_OK;
    nvs_close(handle);
    storedMode.store(stored ? static_cast<int>(mode) : unknown);
    return stored;
}
#endif
