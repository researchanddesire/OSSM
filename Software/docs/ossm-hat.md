# OSSM V2.3 RS-485 HAT feedback

The firmware discovers a live 57AIM30 feedback path at startup, then reads motor
position, current, supply voltage and alarm through UART2. STEP/DIR remains the
motion command interface. Detection and telemetry never write a motor register,
change electronic gearing, enable output, save parameters or zero the encoder.

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
The existing staging/HIL profiles set this flag automatically. That build never
transmits on the HAT: it correlates the PC monitor's FC03 requests and replies,
including its full block at `0x0E`, count 12. A position-only PC stream cannot
make current and voltage appear fresh.

Standalone production builds poll a single full block at `0x0E`, count 10, with
a nominal 20 ms interval subject to transaction time. They listen first, require
a quiet bus before probing, and latch receive-only mode after observing another
master. Listen-first discovery is not an arbitration guarantee: use the explicit
listen-only build when sharing any bus with another master.

The PC guard remains independent and mandatory for bench motion: fresh full
telemetry at 30 Hz or more, a verified stop path and a valid fixture are required
before flashing or enabling motion. The firmware HAT service does not replace
that monitor or remove physical motor power.

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

**The amp threshold and physical homing behavior require validation on the actual
fixture before merge.** Native tests establish parsing and math, not safe physical
contact. Outside homing, unavailable HAT readings are reported explicitly and
position can fall back to its labeled command estimate; this change does not add
an independent firmware motion interlock to every play mode.

## Inspecting the running ESP32

The existing USB input reader accepts the read-only `hat` command on every track.
Its `RAD_HAT` JSON record reports detection, freshness, passive mode, baud, sample
age, position source and fresh measurements. It does not reset or move the motor.

RAD BLE exposes read-only `motor.hat`, `motor.currentAmps` (A) and
`motor.voltageVolts` (V) resources. The Analog surface also includes explicitly
named amp/volt fields, and Motion reports `positionSource`. Existing
`analog.motorCurrent` and `analog.motorCurrentFiltered` retain their ADC units.
Absent/stale amp or volt reads return `unavailable`; detailed HAT diagnostics omit
stale measurements. The MQTT state schema is unchanged and uses the position
helper for its existing numeric position field.

## Verification

Run native suites `test_ossm_hat`, `test_motor_feedback`, `test_homing_logic` and
`test_state_json`. They cover independent fixed protocol vectors, leading zeroes,
CRC errors, incomplete and mismatched replies, other masters, noise recovery,
freshness/continuity, encoder rollover, polarity, frame conversion and contact
qualification. Build both staging and production hardware variants through the
normal PR workflow. Physical evidence must identify the exact enrolled board,
show the PC guard remaining healthy, and compare the ESP32's `hat`/BLE readings
with independent PC position/current/voltage/alarm while recording the motor and
remote. Record absence/disconnect behavior separately; do not claim a passing
motion demonstration if any safety preflight is unavailable.
