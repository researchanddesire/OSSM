# OSSM HAT

The OSSM HAT is an RS-485 transceiver on the V2.x expansion header (Alex
Warner's design) that talks Modbus RTU to the 57AIM servo. Any V2.x board runs
the same firmware with or without one.

## How it fits

Motion is always planned by FastAccelStepper. Nothing in homing, Simple
Penetration, Streaming or StrokeEngine knows whether a HAT exists.

- **Without a HAT** the plan reaches the servo as STEP/DIR pulses. Position is
  the commanded step count; homing current comes from the board's analog sensor.
- **With a HAT** the ESP32's pulse-counter hardware counts the same STEP/DIR
  pulses, and the follower in `src/services/ossmHat.cpp` sends the servo that
  position as an absolute RS-485 target (FC 0x7B). It mirrors ENA onto the
  servo's output enable and reads back encoder position, current, voltage and
  alarm. The servo ignores STEP/DIR while it takes RS-485 targets.

`src/services/motor.h` is the only thing the rest of the firmware asks:
`getPositionMm()`, `getCurrent()` / `getHomingCurrentLimit()`, `getVoltage()`,
`hasHat()`. Each returns the HAT's measurement when there is one and the
board's own reading otherwise.

## Wiring and detection

| V2.x J9 | ESP32 | HAT |
| --- | --- | --- |
| IO2 | GPIO2 | DE/~RE (UART RTS, RS-485 half duplex) |
| IO22 | GPIO22 | UART TX |
| IO33 | GPIO33 | UART RX |

At boot `motor::begin()` reads the servo's telemetry registers at 115200, then
19200 baud. If the servo answers, the HAT owns those three pins (BLE GPIO and
analog reads skip them); otherwise they stay expansion outputs as before. Build
with `-D OSSM_HAT_DISABLED` to ignore a HAT.

Taking control writes register 0x00 = 1 (RS-485 targets), aims the servo at
its measured position, turns output off and sets 0x03 = 60000 (no internal
ramp: targets already follow the planned trajectory). Output turns on when ENA
asks for it, holding wherever the servo is. Nothing is saved to the servo's
EEPROM. If the servo stops answering for 150 ms the follower stops the motion
like an emergency stop.

The follower is an `esp_timer` step every millisecond with no task of its own:
the firmware's mode tasks need large contiguous internal RAM, and a dedicated
follower task left too little for them to start.

## Tuning

`Config::Hat::countsPerPulse` maps STEP pulses to encoder counts (32768 per
revolution over 800 pulses; sign measured on the Test Rig 57AIM30 with DIR
polarity 1). `Config::Hat::homingCurrentAmps` is the servo current that ends a
homing pass.

Framing and CRC come from Alex Warner's `lib/AimFrame` (see its
`PROVENANCE.md`). His bench measurement that the servo ignores a request
starting within ~1.2 ms of its reply sets the follower's 1.5 ms turnaround.
