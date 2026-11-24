# Serial device identity

Every firmware track prints its target and version at boot and answers `info`
or `identity` followed by a newline on the normal USB/UART port at 115200 baud.
For example, `LKBX-N16R2 1.21.59`. The following `RAD_ID` JSON record has schema 1,
product, model, semantic version, track, build SHA and factory MAC (`device_id`).
It separates target_flash_bytes/target_psram_bytes from measured flash_bytes/
psram_bytes. A 4 MB Trainer image on a 16 MB board therefore reports DTT-N4 while
retaining the true physical capacity in its record. PSRAM bytes are the amount
initialized and available to the application. No Wi-Fi, pairing or account data
is exposed. Identity is a discovery hint, never authorization to flash or move.

Open the serial port, then send `\ninfo\n` to terminate any incomplete old
command. Retry once per second while booting; malformed and oversized commands
are ignored and replies are rate-limited to four per second. There is one input
reader for identity and the existing development-only screen commands. Identity
does not wait for a host, change saved settings or control actuators.

Lab reads identity after its existing ROM hardware inspection, verifies the MAC
and measured flash against that inspection, and labels the board with the
reported model/version. Old firmware without this protocol remains usable.
Firmware selection and flash compatibility still require verified hardware.

For TinyUSB CDC builds, the USB product descriptor uses the model name without
changing VID/PID or the serial number. Espressif's hardware USB Serial/JTAG name
is fixed; CP210x UART bridge strings belong to the separate bridge and require
host-side vendor configuration, not ESP firmware. Do not switch USB transports
or rewrite identities merely to change a display name.

## OSSM HAT setting

The OSSM also accepts `hat off` and `hat auto` on the same port. The setting is
stored on the device and applies from the next boot; `auto` is the default. The
reply is one line, for example `RAD_HAT {"mode":"off","applies":"next_boot"}`.
If the setting cannot be stored, the reply adds `"error"` (`motor_armed` while
the onboard motor guard is armed, or `storage`) and `mode` is the unchanged
stored value. The `hat` status record reports the `mode` in effect for this
boot.

With `off`, the OSSM boots as if no HAT were fitted: it never starts the HAT
reader, holds the RS-485 transmitter off so it never sends on the bus, and homes
on the board's own current sensor. The HAT pins stay reserved. `hat auto`
restores HAT detection on the next boot. There is no BLE or app equivalent.

## OSSM health record

Every OSSM build, production included, prints `OSSM_HEALTH` followed by one JSON
object per line, for release hardware validation. It carries no tokens, Wi-Fi
passwords or MQTT credentials.

- `boot`, once per boot: `boot_id` (random per boot), `uptime_ms`, `build_sha`,
  `track`, `variant` (`4mb` for V1, `16mb` for V2), `device_id` (as in
  `RAD_ID`), `image_sha256` (the running image), `reset_reason` and `hat_mode`.
- `wifi_connected`, on every Wi-Fi connection, with the `ip` address.
- `mqtt_connected`, on every MQTT connection, with the configured `broker`.
- `heartbeat`, every 5 seconds: `main_menu` (at the idle main menu, with its
  task still running), `menu_age_ms` (since the menu task last ran), `hat` (HAT
  detected) and `homed`.
