#include "state.h"

#include "../Events.h"

StateLogger stateLogger;

// Static pointer to hold the state machine instance
sml::sm<OSSMStateMachine, sml::thread_safe<ESP32RecursiveMutex>,
        sml::logger<StateLogger>> *stateMachine = nullptr;

void initStateMachine() {
    if (stateMachine == nullptr) {
        stateMachine = new sml::sm<OSSMStateMachine,
                                   sml::thread_safe<ESP32RecursiveMutex>,
                                   sml::logger<StateLogger>>(stateLogger);

#ifdef OSSM_HAT_INSPECT_BOOT
        // Bench validation starts with outputs disabled and BLE diagnostics
        // available. Homing requires an explicit target.home command.
        stateMachine->process_event(InspectBoot{});
#else
        stateMachine->process_event(Done{});
#endif
    }
}
