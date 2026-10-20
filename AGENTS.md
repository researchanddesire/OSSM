# AGENTS.md

## Repository Role

- `researchanddesire/OSSM` is the canonical OSSM repository: firmware in
  `Software/`, hardware in `hardware/`, and assembly documentation in
  `assembly-docs/`. It is the only source of official OSSM firmware releases,
  including dashboard updates and the documentation web flasher.
- `KinkyMakers/OSSM-hardware` is the community repository. Do not publish
  releases, add workflows, or enable Actions or GitHub Pages there. Bring
  community firmware changes in by merging its `main` into a branch started
  from `origin/staging`, keeping this repository's hardware and documentation
  layout; do not import its historical CAD, `Printed Parts`, or
  `Documentation` trees.
- Do not use GitHub Pages for any OSSM site, firmware, or documentation.

## Branch and Release Policy

- Persistent branches are lowercase `staging` and `main`. Fetch before branching.
- The firmware directory is `Software/`. `FIRMWARE_PATHS` in `pr-firmware.yml` and `pr-target-policy.yml` and the `publish_firmware.yml` push paths define it; keep them in sync.
- Firmware changes enter `staging` first: branch from `origin/staging` and open the PR into `staging`. Promote with a same-repository `staging` → `main` PR.
- A production fix may take firmware to `main` directly from a `fix/<issue>` branch (an owner prefix such as `AJ/fix/<issue>` also works) started from `origin/main`.
- Changes outside the firmware directory (documentation, hardware, BOMs, workflows) may open their PR straight into `main`; they need no build or hardware checks. `PR target policy` enforces these rules.
- After every merge into `main` that is not a promotion, `hotfix-sync.yml` merges `main` back into `staging` directly (main wins documentation, staging keeps its own firmware version) and opens a `main` → `staging` PR only when a conflict needs a person.
- Squash feature and fix PRs into one focused commit with a concise, imperative title. Merge `staging` → `main` promotions and `main` → `staging` sync PRs with a merge commit; never squash or rebase them.
- Use matching branch names and linked pull requests for changes spanning RAD App or another firmware repository.
- Staging firmware reports the `staging` track and checks `https://staging.researchanddesire.com`. Main firmware reports `main` and checks `https://dashboard.researchanddesire.com`.
- Firmware is update-eligible only after its immutable Supabase artifacts and all required validation records verify. Missing hardware runners leave a release in `validating`; never bypass a gate.

## Firmware Releases

- `publish_firmware.yml` runs after a reviewed merge to `staging` or `main`.
  It allocates the next version through the dashboard, builds and tests the
  firmware, and publishes immutable `validating` releases: V2 (16 MB) on both
  tracks and V1 (4 MB, from the pinned Bluetooth-capable legacy source) on
  `main`. `finalize_release.yml` tags a production release only after its
  persisted gates pass.
- Releases reach Supabase only through the dashboard firmware API. Do not add
  GitHub Pages, S3, release-asset, or storage-alias publishing, and do not
  publish or attach release artifacts by hand.
- `FIRMWARE_PUBLISH_TOKEN` and `FIRMWARE_PROVENANCE_SIGNING_KEY_PEM` exist only
  as secrets of the `firmware-staging` (branch `staging`) and
  `firmware-production` (branch `main`) environments. Git writes use the
  `rad-version-control` app token from `vars.RAD_VERSION_CONTROL_APP_ID` and
  the organization secret `RAD_VERSION_CONTROL_PRIVATE_KEY`; do not add deploy
  keys or personal access tokens.

## Supabase

- The only Supabase project agents and workflows address is `acjajruwevyyatztbkdf`, through the organization secret `SUPABASE_ACCESS_TOKEN`. Name it explicitly in every remote operation; never infer a project from a URL, environment variable, cached CLI link, neighboring repository or prior conversation.
- Firmware releases reach it only through the dashboard firmware API. PR hardware evidence videos go to its private `hil-evidence` bucket.

## Documentation Sync

- The rad-app documentation hub imports `assembly-docs/` and
  `hardware/bom.csv` from `main` only. Make documentation changes on `main`;
  `staging` receives them through the `main` to `staging` merge. Never
  reconcile documentation on `staging` or rewrite `main` history.
- Every documentation or BOM change on `main` reaches the live site within
  minutes, deletions included. Ask AJ before removing or replacing written
  documentation.
- Keep `.github/workflows/trigger-assembly-docs.yml` as the main-only hub
  notifier, and keep `hardware/bom.csv` valid; the hub stops on an invalid CSV.

## Hardware and Assembly Content

- Keep mechanical designs in `hardware/cad/`, PCB projects in `hardware/pcb/`,
  and WireViz sources with their image dependencies in `hardware/cables/`.
- Preserve the harness pinout, wire specifications, and images unless the user
  requests a design change. Keep source provenance and the hardware license.
- Assembly documentation uses MDX and `meta.json` under `assembly-docs/`.
  Describe only designs that are actually present; do not invent build steps,
  a product BOM, or validation results.
- `hardware/bom.csv` is human-owned. Keep detailed cable components in
  the generated cable BOM and use a single harness assembly in a product BOM.
- Exclude generated diagrams, render outputs, credentials, editor caches, and
  local environment files from Git.
- After changing cable tooling or sources, render with
  `bash scripts/render-cables.sh` and verify the outputs and source dependencies.

## PR hardware validation

- PRs into `staging` or `main` that change the firmware directory, or the validation tooling (`scripts/hil/`, `pr-firmware.yml`, `hardware-run.yml`, `.github/scripts/hil_evidence.py`), build every hardware variant and run the exact staging bundle for the PR merge SHA on the shared Test Rig. Other PRs skip both; `Build validation` and `Hardware validation` then pass without work.
- `PR target policy`, `Build validation` and `Hardware validation` are required on `staging` and `main`.
- The rig records the product's camera for the whole run; `Hardware evidence` comments on the PR with each device's video and result, pass or fail. Video never decides a result.
- Merging a firmware change into `staging` or `main` allocates the next version and publishes a signed candidate through `publish_firmware.yml`.
- OSSM runs require the independent RS-485 guard on the rig to protect the motor; the harness refuses to flash otherwise (scripts/hil/README.md).
- Firmware bundles and hardware evidence are Actions artifacts, never tracked files.
- The reviewed harness and fixture inventory live outside the runner work directory.
- Never infer product identity or flash capacity from a COM port or advertising name.
- Follow scripts/hil/README.md for enrollment, shared-device locks, and rollout.
- Keep Wi-Fi credentials, fixture identities, and raw device logs outside the repository.
- A missing device or incomplete three-minute observation must fail validation.

## Development display capture

- Development builds expose the read-only USB `screen` command; staging and production omit it.
- Follow docs/development-screen-capture.md; use the display mutex for snapshots and keep serial transmission outside it.
- Preserve the capture mirror when adding TFT bulk drawing paths. Keep capture files outside Git.

## Serial identity

- All firmware tracks expose the read-only boot/`info` identity described in docs/serial-identity.md.
- Keep one serial input reader; development screen commands register with it.
- Report target memory and measured memory separately; a label never authorizes flashing or motion.

## Development Safety

- Preserve the dedicated TLS/update task and MQTT pause behavior. Keep update protocol, checksum, reboot, and rollback behavior covered by native tests.
- Never commit credentials, local build products, or generated secrets.
