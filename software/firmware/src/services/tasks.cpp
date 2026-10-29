#include "tasks.h"

#include <esp_heap_caps.h>
#include <esp_log.h>

namespace Tasks {
    TaskHandle_t drawHelloTaskH = nullptr;
    TaskHandle_t drawMenuTaskH = nullptr;
    TaskHandle_t drawPlayControlsTaskH = nullptr;
    TaskHandle_t drawPatternControlsTaskH = nullptr;
    TaskHandle_t wmTaskH = nullptr;
    TaskHandle_t drawPreflightTaskH = nullptr;

    TaskHandle_t runHomingTaskH = nullptr;
    TaskHandle_t runSimplePenetrationTaskH = nullptr;
    TaskHandle_t runStrokeEngineTaskH = nullptr;
}

bool Tasks::startOperation(TaskFunction_t task, const char *name,
                           uint32_t stackSize, TaskHandle_t *handle, void *context) {
    for (int attempt = 0; attempt < 20; attempt++) {
        if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(250));
        if (xTaskCreatePinnedToCore(task, name, stackSize, context,
                                    configMAX_PRIORITIES - 1, handle,
                                    operationTaskCore) == pdPASS) {
            return true;
        }
    }
    ESP_LOGE("Tasks", "Could not create %s: internal free=%u largest=%u", name,
             static_cast<unsigned>(heap_caps_get_free_size(
                 MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(
                 MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
    return false;
}
