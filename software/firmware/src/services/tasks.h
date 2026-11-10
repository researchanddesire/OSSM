#ifndef OSSM_SOFTWARE_TASKS_H
#define OSSM_SOFTWARE_TASKS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace Tasks {
    // Declare variables as extern
    extern TaskHandle_t drawHelloTaskH;
    extern TaskHandle_t drawMenuTaskH;
    extern TaskHandle_t drawPlayControlsTaskH;
    extern TaskHandle_t drawPatternControlsTaskH;
    extern TaskHandle_t wmTaskH;
    extern TaskHandle_t drawPreflightTaskH;

    extern TaskHandle_t runHomingTaskH;
    extern TaskHandle_t runSimplePenetrationTaskH;
    extern TaskHandle_t runStrokeEngineTaskH;

    // Constants can stay in the header
    constexpr int stepperCore = 1;
    constexpr int operationTaskCore = 0;

    // Start a motion mode's task. Internal RAM fragments while TLS sessions
    // come and go, so retry briefly before giving up; logs and returns false
    // if the task never starts.
    bool startOperation(TaskFunction_t task, const char *name,
                        uint32_t stackSize, TaskHandle_t *handle = nullptr,
                        void *context = nullptr);
}

#endif  // OSSM_SOFTWARE_TASKS_H
