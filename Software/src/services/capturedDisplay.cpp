#ifdef VERSIONDEV
#include "display.h"
#include "screenCaptureProtocol.h"

namespace {
constexpr size_t kBufferBytes = 128 * 64 / 8;
void* copyScreen(const char*& error) {
    auto* snapshot = static_cast<uint8_t*>(malloc(kBufferBytes));
    if (!snapshot) { error = "no_memory"; return nullptr; }
    if (!displayMutex || xSemaphoreTake(displayMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        free(snapshot);
        error = "busy";
        return nullptr;
    }
    memcpy(snapshot, display.getBufferPtr(), kBufferBytes);
    xSemaphoreGive(displayMutex);
    return snapshot;
}
uint16_t pixel(const void* snapshot, int x, int y) {
    // U8g2 stores vertical groups of eight pixels, in controller orientation.
    // Undo the configured rotation to return the same logical view as the UI.
    const auto* bytes = static_cast<const uint8_t*>(snapshot);
    return (bytes[(y / 8) * 128 + x] & (1u << (y % 8))) ? 0xffff : 0;
}
const screenCapture::Source source{ "OSSM", 128, 64, copyScreen, pixel };
}
void startScreenCaptureConsole() { screenCapture::start(source); }
#endif
