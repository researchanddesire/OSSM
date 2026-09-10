# OSSM

This repository is being rebuilt from scratch. The initial hardware source is
Lucy Chapar's OSSM motor control harness and its six reference images.

- Do not import designs, firmware, or documentation from the historical
  KinkyMakers/OSSM-hardware repository. Add new sources intentionally as part of
  the rebuild.
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
