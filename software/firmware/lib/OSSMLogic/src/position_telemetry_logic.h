#pragma once

// Telemetry conversion only. The shared position counter changes frame with
// the operating mode (services/stepper.h), so its sign and origin are not
// stable across modes. This reports one scale for every mode: it undoes
// motor_feedback_logic::positionInFrameMm and counts extension positive.
namespace position_telemetry_logic {

// Extension from the homed rest position in mm: 0 at home, growing toward the
// far end of the measured travel. `frameMm` is the counter in its current
// frame: Native counts extension negative from 0, StrokeEngine reads
// extension minus the keepout.
inline float extensionMm(float frameMm, bool strokeEngineFrame,
                         float keepoutMm) {
    return strokeEngineFrame ? frameMm + keepoutMm : -frameMm;
}

}  // namespace position_telemetry_logic
