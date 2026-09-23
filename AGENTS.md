# OSSM

This repository is being rebuilt from scratch. The initial hardware source is
Lucy Chapar's OSSM motor control harness and its six reference images.

- The user-authorized PR hardware validation work restores firmware and its
  build/release tooling from this repository's main branch into Software/.
  Preserve the staging rebuild's hardware and assembly documentation. Do not
  import historical CAD, hardware, or documentation as part of firmware work.
- Keep mechanical designs in `hardware/cad/`, PCB projects in `hardware/pcb/`,
  and WireViz sources with their image dependencies in `hardware/cables/`.
- Preserve the harness pinout, wire specifications, and images unless the user
  requests a design change. Keep source provenance and the hardware license.
- Assembly documentation uses MDX and `meta.json` under `assembly-docs/`.
  Describe only designs that are actually present; do not invent build steps,
  a product BOM, or validation results.
- A future `hardware/bom.csv` is human-owned. Keep detailed cable components in
  the generated cable BOM and use a single harness assembly in a product BOM.
- Exclude generated diagrams, render outputs, credentials, editor caches, and
  local environment files from Git.
- Fetch before branching. Prepare changes from `origin/staging` and target
  `staging`. Production promotion uses a reviewed `staging` to `main` change.
- Do not automatically commit, publish, or attach release artifacts.
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
