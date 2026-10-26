# OSSM V2.3 RS-485 HAT feedback

The firmware discovers a live 57AIM30 feedback path at startup, then reads motor
position, current, supply voltage and alarm through UART2. STEP/DIR remains the
motion command interface. Detection and telemetry are read-only. The onboard
safety guard can latch Modbus control with output disabled on a fault and verify
the registers. An explicit reviewed handback can return register 0 to STEP/DIR
mode while GPIO ENA remains disabled. Neither path writes targets, gearing,
EEPROM or the absolute encoder, or removes physical motor power.

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

The explicit `hat-validation-16mb` profile uses the ESP32 guard as the primary safety
monitor instead: pause the PC RTU master before using it. That image requires a
working HAT and boots into a stopped inspection menu. It cannot fall back to ADC
homing, and it does not auto-home after flashing. Existing installed PC-based HIL
gates remain separate; a manual onboard-guard run must not be relabeled as a
passing PC-guard run.

A passive onboard guard also needs fresh control registers: the PC master must
read register 0 with count 26 at least once a second, alongside its regular telemetry
polls. A telemetry-only `0x0E` block omits the STEP/DIR control mode and cannot prove
that it is safe to arm. The guard blocks motion until those reads are available.

## Independent onboard motor guard

A high-priority FreeRTOS task runs every 5 ms, independently of the UART reader and
motion tasks. It accepts only coherent physical encoder/current/voltage/alarm
samples. Before arming, it requires at least 30 Hz observed over 500 ms, samples no
older than 100 ms, healthy 20..36 V supply/alarm data, GPIO26 HIGH readback, and at
least 100 ms of measured stationary position with current below 0.5 A. STEP/DIR mode
must be read back from the motor. The serial output-enable register is not used
as GPIO ENA's status; it can legitimately remain0 in external-control mode.

Every firmware output-enable path, including StrokeEngine, goes through the
guard. Fault/disarm and output enabling share a lock. A separate hardware timer
forces ENA HIGH if the safety task misses its 34 ms deadline. The first stop does
not wait for UART, UI or BLE: pulses stop and output is disabled immediately.
The sole UART owner subsequently requests and verifies register 0 = 1/output bit 0 = 0
when it owns the bus; it refuses to compete with another master.

The initial conservative policy bounds current to 2.2 A (not a motor rating), loaded contact to 200 ms, and actual homing speed
to 30 mm/s. Only two explicitly sequenced homing contacts are allowed. Homing and
retreat use 25 mm/s. After the two contacts it learns physical encoder boundaries;
normal operation enforces 6 mm margins plus conservative velocity projection over
the telemetry/stop allowance. Unexpected stationary load trips within 50 ms.
High-speed stopping dynamics require separate physical validation; this policy
can conservatively reject fast travel before a nominal trajectory reaches an end.

Faults persist in NVS, including an interrupted armed run. A remembered HAT makes
later boot failures fail closed rather than selecting ADC homing. Faults show on
the display/LED, USB and BLE. A reviewed reset requires fresh stopped-motor proof
and never arms automatically. `target.guardPrepare` queues only the reviewed
mode 0 handback while outputs stay disabled; it is distinct from clearing a fault
or enabling motion. None of these operations claims physical power removal.

## Runtime helpers and units

- `getMotorPositionMm()` uses fresh measured feedback after homing establishes
  encoder polarity and a stopped home reference. Otherwise it returns the
  existing command-counter estimate; `motorPositionUsesRs485()` exposes the
  choice. Planner/completion comparisons continue to use exact command steps.
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

## Verification

Run native suites `test_ossm_hat`, `test_motor_feedback`, `test_motor_guard`, `test_homing_logic` and
`test_state_json`. They cover independent fixed protocol vectors, leading zeroes,
CRC errors, incomplete and mismatched replies, other masters, noise recovery,
freshness/continuity, encoder rollover, polarity, frame conversion and contact
qualification. Build both staging and production hardware variants through the
normal PR workflow. Physical evidence must identify the exact enrolled board,
show the PC guard remaining healthy, and compare the ESP32's `hat`/BLE readings
with independently observed position/current/voltage/alarm while recording the motor and
remote. Record absence/disconnect behavior separately; do not claim a passing
motion demonstration if any safety preflight is unavailable.
