# Drawn patterns

In the Dashboard pattern editor, draw or edit one cycle, preview the loop and save it to your library. Connect to an OSSM with pattern support over Web Bluetooth. Prepare the OSSM and complete its homing and Streaming preflight. Select Play to start. The speed knob, stroke, depth and sensation controls govern the physical motion. At zero speed the loop waits for the first deliberate increase. Once moving, reducing speed to zero stops it. Changing stroke or depth to a different travel window requires pressing Play again. A cycle that exceeds the velocity or acceleration limit takes longer to run.

**Offline replay.** Return to the menu and select Save to OSSM in the Dashboard. The OSSM keeps four named slots across power cycles. Saving replaces the chosen slot. Select **Saved patterns** in the wired menu, turn the encoder to a name, then click to choose it. An asterisk marks the current selection. A long press or Back leaves the selector. Choosing a name returns to the menu with Streaming highlighted and the motor stopped. With the wired remote, select Streaming, complete its normal preflight and double-click the encoder to play the saved curve. Double-click again to stop. A long press returns to the menu. Booting or reconnecting leaves playback stopped. The ALT controller lacks the wired encoder and speed knob. Use the Dashboard’s saved-pattern playback with an explicit software speed on that controller.

The optional BLE setting `setting.afterHomingPosition` sets the parking distance in millimetres from the retracted, backed-off reference after homing. Its range is 0–100 mm. The controller limits the target to the measured usable stroke. The default of 0 mm parks at the retracted reference. The setting is held in memory until reboot.

## BLE protocol

Probe the OSSM with `sensor.read` at `pattern.play`. Version 1 returns `version`, `hasSpeedKnob`, `active`, `stored`, and `periodMs`. The effective period becomes available after the controller evaluates the curve against the calibrated travel and current motion limits.

| Operation | Arguments | Behavior |
| --- | --- | --- |
| `pattern.play` | `samples`, `periodMs` | Start a live loop from stationary, homed Streaming. |
| `pattern.play` | `{}` | Play the pattern stored on the OSSM over the same live connection. |
| `pattern.keepalive` | `{}` | Renew the active loop. Send once per second. Five seconds without renewal stops it. |
| `pattern.stop` | `{}` | Cancel the loop. To leave Streaming or abort homing, use `target.set` at `target.menu`. |
| `pattern.save` | `samples`, `periodMs` | Replace the selected pattern while stopped at the menu. |

All five operations require the connection's RAD BLE control lease. Samples are 16–128 unsigned byte positions encoded as hexadecimal pairs. Each sample occupies an equal fraction of the requested 500–60,000 ms cycle. The last interval joins the last sample to the first. A monotone cubic Hermite interpolant shares the velocity on both sides of every sample and bounds each segment by its two endpoint positions. Playback starts at a minimum to enter the cycle from rest. The device sets the travel window and stretches time to respect its velocity and acceleration limits.

The pulse backend queues timed STEP/DIR pulses. The serial backend sends bounded absolute targets through the existing acknowledged drive interface. The Dashboard needs a BLE connection for a live session. The wired remote can initiate offline playback of the saved curve.

Historical prototypes and design discussions are available in the [original pattern branch](https://github.com/KinkyMakers/OSSM-hardware/pull/261), the [Bézier proof of concept](https://github.com/KinkyMakers/OSSM-hardware/pull/208) and the [streaming planner](https://github.com/KinkyMakers/OSSM-hardware/pull/319).

## Named slots

When the pattern status advertises `libraryVersion: 1`, read `sensor.read` at `pattern.library`. It returns `version`, `capacity` (4), `selected` (0–3, initially 0), and an array `slots` of occupied entries with `slot`, `name` and `periodMs`. Names use 1–24 UTF-8 bytes. Trim surrounding spaces and omit control characters.

| Operation | Arguments | Behavior |
| --- | --- | --- |
| `pattern.save` | `slot`, `name`, `samples`, `periodMs` | Save to that slot while stopped at the main menu. The first occupied slot becomes the default. |
| `pattern.select` | `slot` | Make an occupied slot the wired default. Requires the stopped main menu. |
| `pattern.play` | `slot` | Start the specified occupied slot through the normal guarded live connection. Its selection for the wired remote stays as set. |

The original commands still target the selected slot. The first boot after upgrade reads an existing one-slot record as slot zero with the name “Saved pattern”. A new save commits the waveform and catalog together. Selecting or saving a pattern never starts the motor.
