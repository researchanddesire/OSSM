# OSSM V2.3 RS-485 HAT motion and feedback

The firmware discovers a live 57AIM30 feedback path at startup, then reads motor
position, current, supply voltage and alarm through UART2. A detected HAT selects
absolute RS485 position commands for homing, Simple Penetration, Streaming and
StrokeEngine. Boards without a detected/remembered HAT retain STEP/DIR. Discovery
is read-only; motion requires an explicitly armed session and a proven independent
GPIO or UART timer stop. A detected HAT never silently falls back to pulses after a fault.

This serial motion backend applies to classic ESP32 V2 boards. The ESP32-S3
OSSM ALT build retains its existing read-only HAT telemetry and STEP/DIR motion;
the classic ESP32 emergency UART/timer peripheral code does not run on the ALT.
The shared `motor::` API, smaller motion task stacks, start retries and negative
native Streaming coordinates are preserved from the newer firmware.

The supported serial admission state is already-configured Modbus mode1 with
output bit0 off, exclusive 115200-baud ownership and fresh feedback. Firmware
refuses arbitrary mode0-to-mode1 transitions because a stale serial target could
become active. It does not write gearing, EEPROM, baud or the absolute encoder.
No firmware command in this feature removes physical motor power.

The pinout, AIM register scaling and leading-zero reply workaround come from
[Alex Warner's HAT work](https://github.com/KinkyMakers/OSSM-Hardware/tree/faa79768fccbdbec5b236f6c96457b45fcaca74c/Hardware/Servo%20Tools/Gold%20Motor/RS485-Hat-Test).
`lib/AimFrame` preserves his framing source and its provenance/license. The
standalone motion test and historical hardware trees are not imported.

## Wiring and detection

| V2.3 J9 signal | ESP32 GPIO | HAT function |
| --- | --- | --- |
| IO2 | 2 | DE/~RE, LOW receives |
| IO15 | 15 | Unused; remains available as expansion 2 |
| IO22 | 22 | UART TX |
| IO33 | 33 | UART RX |

The HAT also needs 3V3 and ground. Its motor connector requires the isolated
RS-485 interface's 5V supply, A, B and ground, plus the motor's separate supply;
see Alex's wiring reference. A powered ESP32 alone does not prove motor power.

There is no dedicated HAT presence pin. `ossmHatDetected()` becomes true only
after a CRC-valid request/reply supplies the complete alarm/current/voltage/
position register block from motor address 1. It remains true after an outage;
`readOssmHatTelemetry()` separately rejects samples older than 250 ms. An absent,
unpowered, incorrectly wired or differently addressed motor cannot be detected.

Discovery listens at 115200, 19200, 9600 and 38400 baud before attempting any
read-only FC03 probe. Successful operation locks the baud. Reconnection searches
again without changing motor baud or identity. The main state machine waits up
to ten seconds for initial discovery before selecting its sensor backend.

GPIO2/22/33 stay reserved during discovery and reconnection, even when no HAT
answers. Both BLE GPIO interfaces reject writes to these pins, and ADC surface
reads skip them because `analogRead()` can change UART pin modes. The alternate
`AJ_DEVELOPMENT_HARDWARE` display uses GPIO22; HAT initialization is disabled for
that conflicting pinout.

## Shared PC safety bus

Modbus RTU does not arbitrate between masters. **Build with
`OSSM_HAT_LISTEN_ONLY` whenever the independent PC safety monitor is attached.**
The PR HIL workflow opts in with `RAD_HIL_HAT_LISTEN_ONLY=1`. That build never
transmits on the HAT: it correlates the PC monitor's FC03 requests and replies,
including its full block at `0x0E`, count 12. A position-only PC stream cannot
make current and voltage appear fresh.

Normal staging and production builds poll the full telemetry block at `0x0E`,
count 10, plus control/telemetry at register 0, count 26, once a second, with
a nominal 20 ms interval subject to transaction time. They listen first, require
a quiet bus before probing, and latch receive-only mode after observing another
master. Listen-first discovery is not an arbitration guarantee: use the explicit
listen-only build when sharing any bus with another master.

Once serial position preparation is requested, every existing telemetry poll
includes register0/count26, through cancellation and final quiet. This supplies
fresh control-mode evidence during the GPIO stop test without adding a second
polling transaction or a second UART owner.

The explicit `hat-validation-16mb` profile uses the ESP32 guard as the primary safety
monitor instead: pause the PC RTU master before using it. That image requires a
working HAT and boots into a stopped inspection menu. It cannot fall back to ADC
homing, and it does not auto-home after flashing. Existing installed PC-based HIL
gates remain separate; a manual onboard-guard run must not be relabeled as a
passing PC-guard run.

A passive onboard observer also needs fresh control registers: the PC master must
read register 0 with count 26 at least once a second, alongside its regular telemetry
polls. A telemetry-only `0x0E` block omits the STEP/DIR control mode and cannot prove
that it is safe to arm. Serial motion additionally requires exclusive UART
ownership; the HAT cannot send targets while the PC is acting as an RTU master.

## Serial position commands and commissioning

`MotionAxis` adapts the existing motion modes to the HAT or FastAccelStepper.
Logical coordinates remain pulse-equivalent steps. The serial adapter anchors
them to fresh absolute encoder counts, converts direction/frame changes locally,
and never re-zeros the motor. A bounded local acceleration planner submits at
most one waypoint per20ms, requires the preceding request's correlated ACK before
advancing, and stops advancing beyond a1mm following envelope. The immediate
FC7B position word need not equal the requested target. Arrival requires fresh,
quiet FC03 encoder samples; an ACK alone is never physical arrival.

The vendor absolute command is FC0x7B (MSB-first request, low-word-first reply).
FC0x10 at register0x0C is incremental and is not used for moves. RAM speed/ramp
limits are written and read back only while serial output is disabled. The initial
normal cap is37RPM (24.67mm/s with the stock40mm/revolution pulley), with a
1500RPM/s motor ramp cap; the local trajectory may request less. Ordinary targets
must use the same verified limits. Enable is a separate final guarded write.
Stop cancels pending work, asserts GPIO26 HIGH as a best effort, writes only register1=0 and
verifies mode1/output0 plus physical stationarity. It never changes control mode.

GPIO26 drives the PCB's ENA transistor; it is not a motor-power switch. Its ability
to stop a serial move must be characterized on the actual connected harness.
The tested harness did not establish this GPIO stop authority in serial mode.
Each boot starts with normal serial motion blocked until a separate stop path
is qualified. The explicit BLE
`target.guardProbe` commissioning request requires a lease and both
`reviewed:true` and `positiveCorridorClear:true`. The operator must verify an
unoccupied positive encoder direction before requesting it. It is one-shot per
boot and is never automatically run at startup.

The independent safety task supervises this single target of at most+0.5mm at
1RPM/15RPM/s, bounded to0.25A,2mm/s, a0.75mm envelope and3seconds. After measured
positive progress, it asserts ENA HIGH while at least0.25mm remains to the target.
It then requires six unique quiet samples over100ms, after100ms braking, with
no intervening UART control write and ENA continuously HIGH. UART output-off is
mandatory within250ms of the GPIO cut, followed by a fresh readback and separate
physical quiet proof. Any failure blocks ordinary serial operation and latches
the warning; there is no reverse, retry or automatic homing. Success grants only
a volatile proof tied to the current telemetry continuity. Fault, mode change,
bus contention, lost feedback or reboot revokes it. A normal home remains a
separate explicit action with exactly two allowed contacts.

### UART timer stop qualification

The alternate stop uses an unused UART1 and raw level3 IRAM interrupts on core1.
It isolates both TX22 and DE2 from the ordinary UART2 owner, waits for a quiet
RTU bus, and sends only the fixed register1=0 frame. Actual shift-register idle
ends DE assertion. The stop obligation begins before output enable and survives
software disarm until fresh output-off and physical quiet are verified.
Guard silence beyond34ms, coherent telemetry silence beyond80ms, a pending
off request beyond50ms, or a fault can trigger the alternate path. No task lock,
normal UART driver or flash code is required to transmit that frame.

Before any enable, reviewed `target.guardTimerPreflight` requests with explicit
`cacheOff:false` and then `cacheOff:true` must each pass. They never enable the
drive. The second uses the pinned SDK's shared flash-operation lock and real
cache-disabled interval; it performs no erase or write. Both require an actual
interrupt transmission, echo, coherent output-off readback and500ms of measured
quiet at an observed rate of at least45Hz. GPIO DE LOW/TX HIGH fencing remains
through full UART2 teardown and reset. Every queued target and old reply loses
authority before clean transport restoration with a new telemetry continuity.

The separate one-shot `target.guardTimerProbe` then requires `reviewed:true` and
`positiveCorridorClear:true`. It uses one+0.5mm target at1RPM/15RPM/s, a0.25A
limit, a0.75mm envelope and3second deadline. It requires measured positive
progress, a real timer-originated stop while at least0.25mm remains, and fresh
physical quiet before and after restoration. Powered samples are limited to
2mm/s. Only after the first coherent output-off sample may passive settling use
the separate30mm/s ceiling. All nonzero alarms block these non-homing tests,
including the battery warning otherwise permitted by the homing policy.

Success grants a volatile UART timer proof bound to the restored emergency epoch
and new telemetry continuity. It does not change `gpioStopVerified`. A fault or
reboot revokes authority. Genuine fault takeovers never automatically restore
UART2; only a successful reviewed qualification can do so. Homing and normal
motion remain separate requests after qualification.

This mechanism assumes bounded level3 interrupt masking and a functioning CPU,
APB clock, UART, bus and motor controller. It cannot cover a permanently masked
interrupt, failed processor or broken physical bus. A transmitted stop without
fresh readback and physical quiet is never reported as a verified stop.

## Independent onboard motor guard

A high-priority FreeRTOS task runs every 5 ms, independently of the UART reader and
motion tasks. It accepts only coherent physical encoder/current/voltage/alarm
samples. Before arming, it requires at least 30 Hz observed over 500 ms, samples no
older than 100 ms, healthy 20..36 V supply/alarm data, GPIO26 HIGH readback, and at
least 100 ms of measured stationary position with current below0.5A. HAT motion
requires mode1 and a per-boot GPIO or UART timer stop proof, followed by a disabled,
verified hold before enabling serial output. Register readback and actual GPIO
stop authority are distinct checks.

Every firmware output-enable path, including StrokeEngine, goes through the
guard. Fault/disarm and output enabling share a lock. A separate hardware timer
asserts ENA HIGH and invokes the qualified UART fallback if the safety task
misses its34ms deadline. Serial disable is credited only to a proven GPIO path
or actual fresh register1=0 readback; GPIO HIGH alone is insufficient. The sole
ordinary UART owner also requests and verifies mode1/output bit0=0 when it owns
the bus; it refuses to compete with another master.

The initial conservative policy bounds current to 2.2 A (not a motor rating), loaded contact to 200 ms, and actual homing speed
to 30 mm/s. Only two explicitly sequenced homing contacts are allowed. Homing and
retreat use 25 mm/s. After the two contacts it learns physical encoder boundaries;
normal operation enforces 6 mm margins plus conservative velocity projection over
the telemetry/stop allowance. Unexpected stationary load trips within 50 ms.
High-speed stopping dynamics require separate physical validation; this policy
can conservatively reject fast travel before a nominal trajectory reaches an end.

The homing speed limit uses an encoder window spanning at least 60 ms and three
distinct samples. Individual UART completion intervals vary with reply length
and scheduler timing; they must not turn a steady 25 mm/s traverse into a false
overspeed. Instantaneous estimates remain conservative inputs to bounds and
stopped/contact checks. `motor.guardFault` and USB fault logs freeze the last
accepted sample, interval and both speed estimates at a live fault. A fault
restored from NVS can lack that in-memory sample detail.

Each commanded homing approach and retreat also starts an encoder-progress
watchdog. Under-threshold current cannot hide a stalled axis: absent directional
progress of 0.25 mm trips within 100 ms. Only one contact-qualified retreat marker
can grant a new 100 ms grace period, with an absolute 200 ms no-progress cap.
Loaded contact retains its separate 200 ms limit. Oscillation cannot renew the
progress clock; stopped-reference capture starts before this motion clock.
The HAT's second pass starts directly from the first retreated zero, so the
whole uncalibrated traverse is watched. The final parking move uses the normal
guard with both measured boundaries. Concurrent cached telemetry snapshots are
ordered by sequence; an older snapshot cannot rewind or refresh the guard.

Faults persist in NVS, including an interrupted armed run. A remembered HAT makes
later boot failures fail closed rather than selecting ADC homing. Faults show on
the display/LED, USB and BLE. A reviewed reset requires fresh stopped-motor proof
and never arms automatically. `target.guardPrepare` prepares the serial hold and
RAM limits while outputs stay disabled; it is distinct from clearing a fault
or enabling motion. None of these operations claims physical power removal.

## Runtime helpers and units

- `getMotorPositionMm()` uses fresh measured feedback after homing establishes
  encoder polarity and a stopped home reference. Otherwise it returns the
  existing command-counter estimate; `motorPositionUsesRs485()` exposes the
  choice. The serial motion adapter also uses measured position for completion;
  its requested/acknowledged targets remain separate from physical position.
- `readMotorCurrentAmps(float&)` and `readMotorVoltageVolts(float&)` return false
  when fresh physical measurements are unavailable. They never invent a voltage
  from an ADC current input or relabel ADC values as amps.
- AIM position is 32768 encoder counts per revolution, independent of the
  configured 800 STEP/DIR pulses per revolution. Conversion uses the configured
  pulley travel, learns direction from a homing traverse, and preserves Native
  and StrokeEngine coordinate frames without changing the motor's absolute
  encoder. An outage invalidates the reference even if it occurred between
  telemetry consumers' reads; another home is needed to restore measured position.

Homing selects RS-485 current when the HAT is detected. Its separate
`Config::Hat::sensorlessCurrentLimitAmps` is initially 1.5 A; the legacy threshold
is six ADC percentage points and is not interchangeable. Contact requires three
distinct loaded samples over at least 30 ms with less than 0.25 mm encoder travel.
Homing requires samples no older than 100 ms, supply at least 20 V, and no fatal
alarm (AIM battery warning `0x10` is permitted). Lost feedback stops pulses and
disables output, including during retreat and final reposition. The independent
guard still enforces the two deliberate contacts and their current/duration limits.

**The amp threshold, stop dynamics and physical homing behavior require validation
on the actual fixture before merge.** Native tests establish parsing and math,
not safe physical contact. A previously detected/required HAT losing feedback
latches the motion interlock rather than resuming from a position estimate.

Each homing request owns a generation ticket captured before task creation or
blocking preparation. Stop/disarm cancels that ticket and asserts disabled output
before flushing queued homing commands. Delayed work must still own its ticket
when arming, enabling, submitting a move, publishing a reference or completing a
pass. The second pass inherits the same ticket. A completed home returns to the
menu; selecting an operating mode requires a new user action. This cancellation
contract covers homing. Normal-mode task creation captures the control revision;
stale startup/fault events cannot adopt a newer Home request's revision.

## Inspecting the running ESP32

The existing USB input reader accepts the read-only `hat` command on every track.
Its `RAD_HAT` JSON record reports detection, freshness, passive mode, baud, sample
age, position source and fresh measurements. It does not reset or move the motor.

RAD BLE exposes read-only `motor.guard`, `motor.guardControl`, `motor.hat`, `motor.currentAmps` (A) and
`motor.voltageVolts` (V) resources. The Analog surface also includes explicitly
named amp/volt fields, and Motion reports `positionSource`. Existing
`analog.motorCurrent` and `analog.motorCurrentFiltered` retain their ADC units.
Absent/stale amp or volt reads return `unavailable`; detailed HAT diagnostics omit
stale measurements. The MQTT state schema is unchanged and uses the position
helper for its existing numeric position field.

`motor.drive` reports the selected backend and trajectory state;
`motor.serial` separates queued/acknowledged targets and raw vendor reply words;
`motor.serialTx` counts actual accepted UART control writes;
`motor.driveSafety` distinguishes GPIO and UART timer proofs and verified limits;
`motor.serialProbe` reports the GPIO commissioning result. The separate
`motor.serialTimerProbe` and `motor.serialEmergency` use3 and5 read-only pages,
respectively, selected by integer `page`. Match their procedure identities and
bracket terminal evidence with unchanged summary pages. Raw microsecond timer
values have their own clock domain; `nowUs`/`capturedAtMs` provide a paired capture.
None of these
read-only resources arms or resets anything.

## Verification

Run native suites `test_ossm_hat`, `test_motor_feedback`, `test_motor_guard`,
`test_homing_logic`, `test_homing_session`, `test_ossm_hat_position`,
`test_serial_axis`, `test_serial_interlock_probe`, `test_hat_emergency`,
`test_serial_timer_probe`, `test_serial_stop_diagnostics` and `test_state_json`.
They cover independent fixed protocol vectors, leading zeroes,
CRC errors, incomplete and mismatched replies, other masters, noise recovery,
freshness/continuity, encoder rollover, polarity, frame conversion and contact
qualification. Build both staging and production hardware variants through the
normal PR workflow. Required Test Rig evidence must identify the exact enrolled
board, show the PC guard remaining healthy, and compare the ESP32's `hat`/BLE readings
with independently observed position/current/voltage/alarm while recording the motor and
remote. Record absence/disconnect behavior separately; do not claim a passing
motion demonstration if any safety preflight is unavailable.

An explicitly authorized manual run using the onboard-guard inspection profile
with the PC master paused is supplemental evidence. Label its firmware commit,
guard mode, actual result and recording timebase separately; it cannot satisfy
the required Test Rig checks. A failed or incomplete home is not a normal-operation
demonstration. Keep a fault latched and halt further motion when contact or stop
behavior needs review.
