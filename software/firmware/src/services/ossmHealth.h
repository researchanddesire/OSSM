#pragma once

#include <cstdint>

// Always-on production health record (OSSM_HEALTH, see ossmHealthProtocol.h),
// modelled on the Lockbox's LKBX_HEALTH. No build flag, no task of its own and
// no lasting allocation. These hooks observe normal application work; they
// never advance the state machine, move the motor, change settings or keep the
// display awake. The staging-only RAD_HEALTH observer is separate.
void ossmHealthBoot();
void ossmHealthMqttConnected(const char *broker);
// The main menu task's loop progressed while the menu was idle.
void ossmHealthMenuProgress();
// Call from a task that already runs; reports at most every 5 s.
void ossmHealthHeartbeat();
