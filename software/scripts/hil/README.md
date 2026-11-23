# PR firmware and shared bench validation

A pull request into `staging` or `main` that changes the firmware directory, or
this validation tooling, builds the GitHub PR merge commit and runs it on the
shared Test Rig. `RELEASE_BUILD_SHA` is `github.sha`, never the PR head SHA.
Production profiles compile; only staging artifacts reach fixtures. Other pull
requests skip the build and the rig, and `Build validation` and `Hardware
validation` pass without work. Each repository's `FIRMWARE_PATHS` in
`pr-firmware.yml` and `pr-target-policy.yml` defines its firmware directory.

| Repositories | Staging and production suffixes | Physical coverage |
| --- | --- | --- |
| lkbx | r2, r8 | Both 16 MB ESP32-S3 boards, verified R2/R8 PSRAM |
| dtt | v1, v2 | 4 MB and 16 MB profiles; actual chip capacities recorded separately |
| OSSM | 4mb, 16mb | One ESP32, selected by measured capacity |
| radr | existing staging/production aliases | Enrolled 16 MB R8 remote, application and LittleFS |

OSSM retains its original development/staging/production aliases. Its restored
Software tree comes from this repository's main firmware; staging hardware and
assembly documentation remain separate. The Trainer keeps its SDK security
patches and both deployed partition tables.

OSSM's 4 MB profiles use the standard 4 MB min_spiffs layout and size optimization;
16 MB profiles and the existing aliases retain main's custom `min_spiffs.csv`.
Measured capacity alone does not authorize a partition migration. A 16 MB fixture
still carrying the older 4 MB table fails preflight until a separately reviewed,
storage-preserving migration has been completed. Never relabel it as a 4 MB board.

## Interfaces and trust boundary

`catalog.py` is the reviewed policy for products, physical variants, partition
layouts and writable regions. `bundle.py` emits schema 2 `manifest.json` plus
per-variant images. It records repository, merge SHA, run ID/attempt, product,
environment, chip, flash bytes, PSRAM, offsets, byte lengths, SHA-256 checksums,
and the application digest reported at boot. All variants must be present.
Bootloader, partition table, boot-selection image, application, ELF, and RADR's
LittleFS are included. ELF is retained for debugging and never executed/flashed.

Bundles are **data only**. The Windows worker never checks out PR source or runs
downloaded Python, PowerShell, or other artifact files. It calls the installed
`C:\rad-hil\harness\launch.py` with isolated Python. The host's catalogue and
validator, not a manifest supplied by a PR, decide writable offsets and limits.
The manifest must match the caller's repository, merge SHA, run ID and attempt.
Artifact names include the attempt. Use **Re-run all jobs** to create a fresh
bundle; rerunning only a failed hardware job must not reuse an older attempt's images.

The fixture is schema 1 JSON, stored only at `C:\rad-hil\fixture.json`. Its
`products` object has `lockbox` (r2/r8), `dtt` (v1/v2), `ossm` (exactly one of
4mb/16mb), and `radr` (r8). Each device contains:

```json
{
  "product": "dtt",
  "enrolled": false,
  "device_id": "REPLACE_WITH_VERIFIED_FACTORY_MAC",
  "chip": "esp32",
  "flash_bytes": 4194304,
  "psram": null,
  "usb_vid": 4292,
  "usb_pid": 60000,
  "usb_location": "REPLACE_WITH_VERIFIED_USB_LOCATION"
}
```

Supply exactly one of `usb_serial` or `usb_location`; never use a transient COM
number as identity. Set `enrolled` only after physically identifying the product,
reading its factory MAC/chip/flash, and verifying PSRAM where relevant. The owner
has authorized a 16 MB Trainer to exercise the 4 MB V1 image/partition profile.
That V1 fixture must retain `flash_bytes: 16777216` and explicitly set
`allow_larger_flash: true`; boot evidence still verifies the actual 16 MB chip.
This is profile coverage, not evidence of testing a physically 4 MB chip. The
exception is refused for every other product/variant or capacity. Missing, duplicate, ambiguous,
or incompatible fixtures fail before writes. Every write rechecks chip, MAC,
flash and PSRAM and reads the installed partition table. A different table
requires a separately reviewed storage-preserving migration; no automatic erase
or repartition fallback is permitted. Approved writes exclude NVS, calibration,
identity, coredump and the inactive application slot. RADR writes its filesystem
alongside its matching application.

`RAD_HEALTH` serial JSON schema 1 is the health interface; product control/BLE
protocols are unchanged. Boot evidence binds the runtime image digest and build
SHA to product/variant, factory MAC, physical flash and staging track. The normal
menu/state task supplies progress, independently of the heartbeat task. Lockbox
readiness also retains the existing requirement for a live staging MQTT connection. Health
requires an idle application, a usable IPv4 address on `IoT_PHB`, and a successful
certificate-verified HTTPS request to staging. Readiness must arrive within 180
seconds, followed by 180 continuous seconds measured by both host time and device
uptime. A heartbeat gap over 20 seconds, stalled progress, network loss, reset,
panic, watchdog, brownout or serial error fails permanently. There is no crash
retry or restarted observation window. Keep Lockboxes unlocked and all devices
idle before starting. No test issues motion/lock commands.

## Provisioning and evidence

Use the existing BLE provisioning service on products that expose it. Obtain the
bench password from the operator's local secret store, never source/build flags,
workflow inputs, artifacts or command-line logs. BLE-disabled Trainers use saved
Wi-Fi settings or their existing setup portal. Verify saved connectivity before
starting the run. Flashing preserves NVS. Do not enable BLE on the public Trainer
to make provisioning easier.

Firmware artifacts expire after 14 days. Evidence expires after 30 days and is
uploaded on success and failure: redacted serial/flash logs, per-device JSON,
aggregate JSON, JUnit and workflow diagnostics. Raw serial output can contain
credentials and stays off uploaded logs. Download/setup failure also produces a
failing JUnit report. Current-run evidence replaces Trainer `test/report.json`
freshness; do not commit generated reports.

### Video evidence and the PR comment

The harness records the camera that sees the product for the whole run: identity
check, flashing, reset and observation. Each device is cropped and rotated from
that stream into `results/video/<variant>.mp4` with an elapsed-time clock, next to
`manifest.json` (camera, views and a timeline of harness events). The PR job
`Hardware evidence` turns every recording into an inline GIF and a web MP4,
stores them in the private `hil-evidence` bucket of Supabase project
`acjajruwevyyatztbkdf` (through `SUPABASE_ACCESS_TOKEN`), and comments on the
PR with 1-year signed links, on pass and fail. Video never changes a result; a
missing camera is reported in the comment.

The camera map is host-only, like the fixture: `C:\rad-hil\cameras.json` names
the ffmpeg binary (`C:\rad-hil\tools\ffmpeg-7.1`), each Brio 100 by serial and
DirectShow path, and per-product views as a normalized `[x, y, width, height]`
crop plus a clockwise rotation. The Brios look down on the bench, so screens need
rotation:

| Camera | Serial | Views |
| --- | --- | --- |
| 1 | 2548APP4WY98 | DTT V1 (right, 180°) and V2 (left, 180°) |
| 2 | 2606APJMNM48 | OSSM motor and rail, before/after stills |
| 3 | 2613APHG9948 | Lockbox R2 (left, 180°) and R8 (right, 180°) |
| 4 | 2613APNG9C58 | RADR (left, 90°) and OSSM screen (right, 270°) |

The USB topology supports one streaming camera at a time, so a product's
devices share one camera. The recorder uses the bench camera service on
127.0.0.1:8765 when it is running and reads DirectShow directly otherwise. An
open Windows Camera app blocks every camera; close it before rig runs.

### OSSM RS-485 interlock

No OSSM image is written unless the independent RS-485 guard
(`C:\rad-hil\rs485-guard`) protects the 57AIM30 motor: its `status.json` must
be under 5 s old, stream bench telemetry at 30 Hz or more, hold a verified stop
path, report no fault, and be in `collision-only` mode with a verified handback,
which lets the firmware home within the guard's hard-stop contact limits. The
harness watches the guard throughout the run and fails it on any guard fault,
latch or stale status. The guard's before/after state is
`results/rs485-guard.json`, and the comment shows motor stills from camera 2.

The enrolled OSSM carries an RS-485 HAT on the motor bus the guard polls. The
guard's polling keeps that HAT receive-only, so the firmware homes on STEP/DIR
and treats the guard as its independent monitor. Twice a second the guard reads
the full block from register 0; the firmware lowers ENA only while those reads
show the drive in STEP/DIR control. See
[Shared PC safety bus](../../firmware/docs/ossm-hat.md#shared-pc-safety-bus).

After verifying the running image, the harness reads the OSSM's onboard guard
using the read-only USB `hat` command. A reported latch fails immediately with
its fault and bus mode in the evidence. The usual readiness and soak checks
still apply.

### Shared queue and fixture ownership

The dashboard orders PR and release jobs together. Critical jobs can move ahead
of queued jobs. A started run keeps its fixture until cleanup and no queue
reorder interrupts a flash. After queue cutover, the installed launcher
checks a fresh host receipt for the exact repository, run, attempt and SHA.
Manual bench sessions still yield to a reviewed PR. The release queue uses
the same fixture locks as the PR harness and protects a started release run
from watchdog preemption.

- **Fixture.** `rig_lock` records its holder in `<lock>.holder`. A pull-request
  run that finds a fixture taken leaves `<lock>.preempt` and waits up to five
  minutes. The rig watchdog (`C:\rad-hil\watchdog`, scheduled task
  `RAD Rig Watchdog`) terminates a preemptible manual holder. PR and release jobs mark
  their owned fixture as protected. Manual sessions must hold the fixture
  through `rig_lock` and expect to be stopped.
- **Guard.** When the OSSM guard is missing, latched or not handed back, the run
  first holds every OSSM in its ROM loader, so none sends pulses, then leaves
  `locks\ossm-guard.request`. The watchdog starts a missing guard. The guard
  checks that the motor is idle, healthy and between its measured stops, moves
  it slowly to mid-rail, hands it back and clears its latch. It refuses, and the
  run fails before any write, if those checks fail.
- **Record.** The fault that was cleared stays in the guard's `events.jsonl` and
  `latch.cleared.json`, in `results/rs485-guard.json`, and in the "Before
  flashing" line of the PR comment.

The firmware's own stop latch on a guarded bus is not stored, so the reset that
follows flashing clears it.

### OSSM Funscript playback

The required PR build runs `test_funscript_streaming` for firmware and validation
changes. It opens the standard JSON
[`bench.funscript`](../../firmware/test/test_funscript_streaming/fixtures/bench.funscript),
forms the R+D Funscript Player's `stream:<next position>:<timestamp difference in ms>`
commands, and passes them through the firmware's BLE validator, command parser and
motion planner. The fixture includes a reversal, a repeated position, and a move
too short to execute at the test speed. Run it locally with
`pio test -d software/firmware -e test -f test_funscript_streaming`.

For a powered playback check, use the same fixture in the R+D Funscript Player
with a local media clip longer than six seconds. Use the unloaded, identified
OSSM on the Test Rig and take the OSSM fixture lock. Keep the independent RS-485
guard active with a fresh stationary baseline, verified stop and measured interior
travel bounds before connecting. Turn off **Simplified** and **Reverse**, start
at low speed, and constrain depth and stroke to the verified range. Compare the
player's transmitted commands with the independently measured rail position.
Check that it moves in both directions within the range, holds at the repeat,
never moves opposite the short target, and returns to rest after disconnect.
Keep the regular unattended HIL run at idle.

The guard's `status.json` carries the last three seconds of encoder positions at
50 Hz in `trace`; merge successive snapshots for the independent record, and
read the file at most eight times a second. On hardware the 50 ms waypoint is
reached rather than held when the firmware's latency buffer lengthens it; it
must still never move the rail away from its target.

## Installation and runner migration

1. Review and commit the `hardware-run.yml` wrapper first. Callers and the runner
   group use its full immutable commit SHA from `reviewed-workflows.json`.
   Push that commit with the implementation branch before running its PR. This
   removes the bootstrap dependency on merging a new wrapper onto main first.
   Review wrapper updates and explicitly update the organization allowlist;
   changing a PR workflow alone never authorizes new host code. Follow repository
   manual-commit rules unless the owner explicitly authorizes committing/merging.
2. Run `install.ps1` from the reviewed checkout as the installation owner (or a host administrator). It installs
   the harness outside runner work directories, pins dependencies, records source
   hashes and prevents the runner service account from modifying installed code.
   Fixture JSON and provisioning secrets stay outside source and artifacts.
3. Create one organization runner group named `rad-firmware-validation`, with
   selected access to these six repositories, public repositories permitted,
   `restricted_to_workflows=true`, and exactly the six reviewed wrapper paths.
   `rollout.py prepare` creates reviewable REST payloads; it does not activate rules.
   See [GitHub runner group API](https://docs.github.com/en/rest/actions/self-hosted-runner-groups).
4. Drain and stop the existing Lockbox worker before moving its registration to
   the organization. Keep one worker only and retain `lkbx-rig-01`; add `rad-hil`.
   Register it in this group and run the service as the existing dedicated account.
   Never run it as administrator. Grant write access only to runner work/temp and
   lock directories; grant read/execute access to the harness and fixture.
5. Hardware jobs do not cancel in-progress flashing. One worker serializes jobs
   across repositories; OS locks additionally cover shared product families and
   the legacy Lockbox `C:\lkbx-hil\fixture.lock`, and a pull-request run takes
   them from a bench session (see above). Within a job both required
   variants flash and observe in parallel. A killed flash is a failure and needs
   explicit recovery; it never becomes a passing run on retry.
6. Run real PRs against both persistent branches for each product, plus a
   docs-only PR, which must skip the build and the rig yet pass both checks. Check complete device coverage, runtime merge SHA/image digest,
   180-second observation, network continuity, and the Lockbox migration regression.
   Fork builds remain on hosted runners; move reviewed changes to an internal
   branch before hardware validation can pass.
7. Only after those runs pass, activate the branch ruleset requiring
   `PR target policy`, `Build validation` and `Hardware validation`, integration
   ID 15368 (GitHub Actions), for main and staging. Do not require up-to-date
   branches: every firmware merge adds a version-bump commit, which would force
   another rig run on every open PR. Never bypass failed or skipped devices.

## Verification

Run `python -m unittest discover -s software/scripts/hil -p 'test_*.py'` from the checkout.
Tests exercise corrupted/stale bundles, wrong repository/chip/flash, overflowing
regions, missing variants/fixtures, cross-process locks, malformed serial records,
boot loops, stalled tasks, heartbeat gaps and network loss. These synthetic tests
do not replace full real PR checks or physical failure-injection verification.
