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

- Persistent branches are lowercase `staging` and `main`.
- Fetch before branching. Start feature and release work from `origin/staging`, then open the pull request back into `staging`.
- Only urgent production bug fixes may target `main` directly. Start them from `origin/main`, open the pull request into `main`, and make the standalone word `hotfix` the first word of the PR title (case-insensitive).
- Promote normal releases with a same-repository `staging` to `main` pull request only after every required build, unit, integration, artifact-verification, and OSSM hardware validation succeeds.
- Immediately merge every main hotfix back into `staging` with a `main` to `staging` pull request.
- Squash feature and hotfix pull requests into one focused commit with a concise, imperative title. Merge long-lived branch synchronization pull requests (`staging` to `main` and `main` to `staging`) with a merge commit; never squash or rebase them.
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
- Supabase project refs behind the dashboard: production/`main`
  `acjajruwevyyatztbkdf`, staging `meuaxbjzqrszxdvmacug`. This repository's
  workflows and agents never call Supabase directly.

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

- Every PR into staging or main builds all hardware variants, including docs-only changes.
- Install the exact staging bundle for the PR merge SHA; production still builds.
- Require Build validation and Hardware validation in addition to existing checks.
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
