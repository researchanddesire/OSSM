#ifndef OSSM_HOMING_HOMING_H
#define OSSM_HOMING_HOMING_H

#include "homing_session.h"

namespace homing {

/**
 * Start the homing task
 * Runs the sensorless homing procedure in a FreeRTOS task
 */
void startHoming();
bool hasTaskCreationFailed();
void startSecondHoming(homing_session::Ticket ticket);

/**
 * Check if the measured stroke is too short
 * @return true if stroke is below minimum threshold
 */
bool isStrokeTooShort();

}  // namespace homing

#endif  // OSSM_HOMING_HOMING_H
