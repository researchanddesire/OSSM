# Drawn patterns

In the Dashboard pattern editor, draw or edit one cycle, preview the loop and save it to your library. Connect to an OSSM with pattern support over Web Bluetooth. Prepare the OSSM and complete its homing and Streaming preflight. Select Play to start. The speed knob, stroke, depth and sensation controls govern the physical motion. At zero speed the loop waits for the first deliberate increase. Once moving, reducing speed to zero stops it. Changing stroke or depth to a different travel window requires pressing Play again. A cycle that exceeds the velocity, acceleration or jerk limit takes longer to run.

**Offline replay.** Return to the menu and select Save to OSSM in the Dashboard. The OSSM keeps four named slots across power cycles. Saving replaces the chosen slot. Select **Saved patterns** in the wired menu, turn the encoder to a name, then click to choose it. An asterisk marks the current selection. A long press or Back leaves the selector. Choosing a name returns to the menu with Streaming highlighted and the motor stopped. With the wired remote, select Streaming, complete its normal preflight and double-click the encoder to play the saved curve. Double-click again to stop. A long press returns to the menu. Booting or reconnecting leaves playback stopped. The ALT controller lacks the wired encoder and speed knob. Use the Dashboard’s saved-pattern playback with an explicit software speed on that controller.

The optional BLE setting `setting.afterHomingPosition` sets the parking distance in millimetres from the retracted, backed-off reference after homing. Its range is 0–100 mm. The controller limits the target to the measured usable stroke. The default of 0 mm parks at the retracted reference. The setting is held in memory until reboot.

## BLE protocol

Probe the OSSM with `sensor.read` at `pattern.play`. Version 1 returns `version`, `hasSpeedKnob`, `active`, `stored`, and `periodMs`. The effective period becomes available after the controller evaluates the curve against the calibrated travel and current motion limits.

When status advertises `liveSpeed: true`, the controlling RAD BLE connection can change speed during a live loop. Send `setting.write` to `motion.speed` with an integer percentage from 1 to 100. The existing knob arbitration still determines the effective speed. `motion.speedBle` also accepts fractions and changes the configured BLE percentage without selecting it as the speed source. A zero write stops playback and a subsequent increase requires a new Play. Legacy text motion commands and commands that take over a wired or differently owned loop stop that loop.

Speed is a physical velocity ceiling. The cycle stretches when its path would exceed the velocity, acceleration or jerk ceiling and returns toward the authored period when the limits permit it. Two speed settings can therefore yield the same period. A positive change preserves phase and ramps the rate of future frames. Previously queued STEP/DIR frames cover up to 20 ms and knob arbitration runs on the play-controls task.

After playback ends, the optional `lastStop` object reports the first stop reason and the stage (`waiting`, `positioning` or `playing`). `atMs` is uptime in milliseconds, `elapsedMs` is milliseconds since the accepted Play request, `frameGapMs` gives the age of the last accepted playing frame, and `driverResult` is the signed backend result when applicable. An explicit stop reports `stop`. A missed hardware refill reports `pulse_underrun`. Reading status, issuing another Stop or returning to the menu preserves the record. The next accepted Play clears it.

| Operation | Arguments | Behavior |
| --- | --- | --- |
| `pattern.play` | `samples`, `periodMs` | Start a live loop from stationary, homed Streaming. |
| `pattern.play` | `{}` | Play the pattern stored on the OSSM over the same live connection. |
| `pattern.keepalive` | `{}` | Renew the active loop. Send once per second. Five seconds without renewal stops it. |
| `pattern.stop` | `{}` | Cancel the loop. To leave Streaming or abort homing, use `target.set` at `target.menu`. |
| `pattern.save` | `samples`, `periodMs` | Replace the selected pattern while stopped at the menu. |

All five operations require the connection's RAD BLE control lease. The original `samples` field contains 16–128 unsigned byte positions encoded as hexadecimal pairs. A byte of 0 is the shallowest extension in the configured travel window. A byte of 255 is the deepest. For example, a cycle starting at 0, reaching 255, then falling to 64 starts shallow, fully extends within that window, then retracts to about one-quarter extension. The established `stream:<percent>:<time>` waypoint interface uses the opposite percentage convention. Each sample occupies an equal fraction of the requested 500–60,000 ms cycle. The last interval joins the last sample to the first. All supported interpolants keep each segment between its endpoints. The original version 1 firmware shares velocity at each knot. Version 2 also shares acceleration. Playback starts at a minimum to enter the cycle from rest. The device sets the travel window and stretches time to respect its motion limits.

For STEP/DIR, the firmware locates the first half-step crossing within each 2 ms phase frame. FastAccelStepper distributes the signed pulses until the next crossing and the controller carries timer rounding forward. A pause preserves direction. The queue preloads up to ten pulse groups and aims for 16 ms of driver-reported time, about 20 ms including the active group. An exhausted queue stops playback. A normal Stop drains the finite pulse tail for up to 25 ms. Physical revocation flushes the pulse queue immediately. The serial backend sends bounded absolute targets every 20 ms through the existing acknowledged drive interface. At 20 pulses/mm, one commanded step spans 0.05 mm. These timings describe commands. Actual drive response requires physical measurement. The Dashboard needs a BLE connection for a live session. The wired remote can initiate offline playback of the saved curve.

Historical prototypes and design discussions are available in the [original pattern branch](https://github.com/KinkyMakers/OSSM-hardware/pull/261), the [Bézier proof of concept](https://github.com/KinkyMakers/OSSM-hardware/pull/208) and the [streaming planner](https://github.com/KinkyMakers/OSSM-hardware/pull/319).

## Named slots

When the pattern status advertises `libraryVersion: 1`, read `sensor.read` at `pattern.library`. It returns `version`, `capacity` (4), `selected` (0–3, initially 0), and an array `slots` of occupied entries with `slot`, `name` and `periodMs`. When the status also reports `libraryPageSize: 1`, send `{"slot":0}` through `{"slot":3}` as the read arguments. Each response uses the same fields and contains only the requested entry. An empty slot returns an empty array. Reading one slot per request keeps even fully escaped names within the BLE response limit. Names use 1–24 UTF-8 bytes. Trim surrounding spaces and omit control characters.

| Operation | Arguments | Behavior |
| --- | --- | --- |
| `pattern.save` | `slot`, `name`, `samples`, `periodMs` | Save to that slot while stopped at the main menu. The first occupied slot becomes the default. |
| `pattern.select` | `slot` | Make an occupied slot the wired default. Requires the stopped main menu. |
| `pattern.play` | `slot` | Start the specified occupied slot through the normal guarded live connection. Its selection for the wired remote stays as set. |

The original commands still target the selected slot. The first boot after upgrade reads an existing one-slot record as slot zero with the name “Saved pattern”. A new save commits the waveform and catalog together. Selecting or saving a pattern never starts the motor.

### Curve precision and firmware compatibility

The pattern status field `interpolationVersion: 2` selects periodic quintic interpolation with shared velocity and acceleration at each knot. A missing field selects the original harmonic cubic interpolation. Both start at the first minimum, pass through the transferred positions, preserve holds and repeat around the seam. The Dashboard uses the connected firmware's interpolation for the dashed device preview.

A device reporting `maxSampleBits: 16` accepts `samples16` in place of `samples` on play and save. Supply exactly one of these fields. Each Q16 sample is four big-endian hex digits from `0000` to `ffff`. A packet holds 16–64 Q16 positions or 16–128 legacy Q8 positions. The Dashboard prefers 64 Q16 positions and tests whether 128 Q8 positions capture a narrow feature that falls between them. Older devices receive the original Q8 commands. Their firmware rejects a Q16-only command.

When both `maxSparseSamples: 1024` and 16-bit support are advertised, an additional exclusive field, `samples16Runs`, can preserve short features among long holds. Each eight-character entry has a four-digit inclusive end index, then a four-digit Q16 position, both unsigned big-endian hex. Starting at index zero, hold that height through the end index. Start the next run at the following index. Include one to 32 runs with strictly increasing ends and distinct adjacent heights. The final end must be `03ff` (index 1023). For example, `01ff000003ffffff` describes 512 minimum samples followed by 512 maximum samples. The same periodic quintic reconstructs between knots. The Dashboard tests the reconstructed curve against the editable source and enables transfer only inside its 1% stroke tolerance.

Existing single-slot and four-slot records load at their original precision. An unannotated library stores ordinary edits and the selected slot atomically in `libraryV3` until its first annotated save. Upgrading leaves the previous `libraryV2` or `curveV1` snapshot intact. Downgrading therefore restores that snapshot, including any previous slot contents replaced since the upgrade. Check the displayed name and slot after changing firmware. Playback still requires a deliberate Play or wired double-click.

## Measuring the timing of a speed change

The read-only USB command `clock <nonce>` echoes the unsigned decimal nonce
and the device uptime in milliseconds as `RAD_CLOCK <nonce> <uptime>`. A
host can bracket this response with its own send and receive times while the
shaft is stopped to register the two clocks.

`patterntime 0` reads the newest accepted playback rate change.
`patterntime <sequence>` reads one of the last eight changes by sequence.
The response is `RAD_PATTERN_TIME sequence uptimeMs periodMs phaseNanoCycles rateMicroHz phaseAccelerationMicroHzPerSec leadBoundMs first serial leadKnown`.
An unavailable sequence returns `RAD_PATTERN_TIME unavailable <sequence>`.
The three flags are decimal zero or one. Uptime and phase refer to the
producer accepting the first frame at that governed rate. The final
flag marks a known upper bound on the remaining queue delay. Serial-path
shaft arrival requires independent measurement. The first flag identifies
the first rate in a new Play. Read the records after Stop to measure live
adjustments without issuing USB commands during playback.

## Experimental sensation modifiers

A drawing can contain vibration ranges and pause points. Their locations are fractions of the drawing's original cycle, so starting playback at the first minimum keeps every marker attached to its drawn position.

- A vibration range adds a 6 Hz oscillation. It fades in and out inside the marked interval. Sensation controls its amplitude. Full strength uses up to 1% of the configured stroke or 1 mm, whichever is smaller. Available speed, acceleration, jerk and the width of the range can reduce the physical amplitude further.
- A pause point holds the drawn position for a fraction of its configured duration. For a two-second pause, 0%, 50% and 100% sensation request zero, one and two seconds of hold. Braking and resuming take additional time. While the shaft is parked, changing Sensation adjusts the total hold against the time already spent stationary. Raising it can extend the hold to its authored maximum. Lowering it below the elapsed time releases the hold. Returning Sensation to zero resumes playback. Raising it after the point has been passed, or too late to brake smoothly, applies on the next pass.
- At zero strength the drawing plays through. For marked drawings the firmware uses the base acceleration associated with the normal 50% setting at every strength. Travel, speed and backend limits still apply. The reported base period excludes time spent approaching, holding and leaving a pause.
- A vibration range may touch another range. A pause must have its own point, including at the loop seam. A range lies within a single cycle. Place one range on either side of the seam to mark both ends.

A supporting device reports `modifierVersion: 1` and `maxModifiers: 1000` in pattern status. Long stop diagnostics can fill the BLE reply. If those fields are missing, read the same capabilities with `sensor.read` at `pattern.modifiers`. On the same control lease, `setting.write` at `motion.sensation` accepts an integer from 0 through 100 while a marked pattern is playing. Physical controls can also set Sensation. A saved slot with markers reports its `modifierCount` in the pattern catalog.

### Transferring markers

Send `pattern.modifiers` with a nonzero random unsigned 32-bit `key`, zero-based record `offset`, and `data`. Encode each record in twelve lowercase hexadecimal characters, four per unsigned field in this order.

| Field | Vibration | Pause |
| --- | --- | --- |
| Start | 0–999 thousandths of the cycle | 0–999 thousandths of the cycle |
| End | Greater than start, up to 1000 | Same as start |
| Hold | 0 | Maximum hold in milliseconds, 1–10000 |

Sort by start. Send at most 32 whole records per request, reducing chunk size as needed for the BLE packet budget. Start at offset zero. The reply's `count` acknowledges the number of records received. A retry of an identical chunk returns the current count. A gap, overlap, invalid field or conflicting retry fails.

Commit on the same connection and lease with `pattern.playModified` or `pattern.saveModified`. Supply the usual sample/period and optional save destination, plus `modifierKey` and the exact `modifierCount`. A commit consumes the upload. For a new attempt after a failed commit, start a fresh transfer. Stop and lease release discard uncommitted markers. A live upload alone cannot move the shaft or change a saved slot.

Each of the 1,000 distinct start positions can carry a marker. The device allocates transfer and playback memory as needed, and stores a dense slot in at most 2,000 bytes of modifier data. After the first annotated save, each changed slot stores its waveform, name and markers in one atomic NVS blob (`slot4_0` through `slot4_3`). A full slot occupies 2,304 bytes. On an empty device the first blob also records the initial selection. Later explicit selections write one four-byte `selected4` value. Earlier library snapshots and untouched slots remain available. Downgrading exposes the older snapshot. If the shared NVS partition lacks room for an atomic change, the device reports a save failure. A malformed slot remains unavailable until an explicitly named save replaces that slot. Other readable slots remain available by number. An invalid selection requires a deliberate selection before wired recall. Existing drawings and saves on earlier firmware continue using the original commands. After upgrading, both Dashboard slot playback and the wired controller replay the saved markers.

With the shaft stopped, USB `patternfeel <sequence>` reads the sensation associated with an accepted frame on the same sequence as `patterntime`. Zero requests the newest record. The fields following `RAD_PATTERN_FEEL` are sequence, uptime in milliseconds, requested strength in thousandths of a percent, current gain in millionths, target gain in millionths, carrier phase in billionths of a cycle, and accepted frame index since Play. The record captures changes in commanded timing or effect strength. Both readouts survive Stop.
