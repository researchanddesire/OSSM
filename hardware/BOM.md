# Bill of Materials

Edit `hardware/bom.csv` using the canonical 12-column RAD BOM format. This CSV is
the source of truth for OHAI’s BOM. Assembly documentation reaches OHAI through
reviewed sync PRs on the matching staging or main branch.

Keep harnesses as top-level assemblies; detailed cable parts stay WireViz-owned.
Rows are sorted by Category and numbered sequentially. Preserve quantities
and units when reordering. Notes identify the actuator,
extrusion stand or electronics assembly.

`assembly-docs/_bom/bom.csv`, `source.json` and `bom.json` are generated snapshots.
Change the hardware CSV rather than those copies; regenerate them together and
pin the commit containing the CSV. The source-aware hub importer automates this
step once deployed.

Use repository source paths for CAD and harness files. Never link to Google Drive files.

## Stands

The OSSM ships with one of two PitClamp stands. Both use the
[lower PitClamp pivot](cad/stand/lower-pivot/README.md) parts.

- **1-arm stand** (single hinge): one 18 in arm on the base rail. One active
  lower PitClamp locks the arm angle; a passive pivot plate holds the other side.
- **2-arm stand** (double hinge): two 18 in arms in series. Each joint has two
  active lower PitClamps, one normal and one mirrored, so four in total. There
  is no passive pivot.

The pivot-plate stand (two printed pivot plates with adjustable M6 handles) is
discontinued. Its files stay in [`cad/stand/`](cad/stand/README.md) for
existing stands.

`bom.csv` lists the 1-arm stand. Rows that change for the 2-arm stand give its
quantity in `Notes`; rows marked "2-arm stand only" are left out of a 1-arm
build, and rows marked "1-arm stand only" are left out of a 2-arm build.

### Stand parts by build

Quantities are for the stand alone. They leave out the PitClamp Mini that
holds the actuator and the four T-nuts for the Mini and the PCB box.

| Part | 1-arm | 2-arm |
| --- | --- | --- |
| 12 in extrusion (crossfeet) | 2 | 2 |
| 18 in extrusion (arm) | 1 | 2 |
| 19.5 in extrusion (base rail) | 1 | 1 |
| Base bracket | 4 | 4 |
| Printed foot and grippy foot | 4 + 4 | 4 + 4 |
| End cap | 6 | to confirm |
| Active lower clamp | 1 | 2 |
| Active lower clamp, mirrored | – | 2 |
| Passive pivot | 1 | – |
| Mounting ring | 1 | 2 |
| Mounting ring, mirrored | – | 2 |
| Upper jaw, handle and both dogbones | 1 set | 2 sets |
| Upper jaw, handle and both dogbones, mirrored | – | 2 sets |
| M6 × 12 screw (feet and brackets) | 12 | 12 |
| M6 T-nut | 19 | 28 |
| M6 × 25 countersunk screw (active lowers) | 2 | 8 |
| M6 × 25 socket-head screw (mounting rings) | 2 | 8 |
| M6 × 20 socket-head screw (passive pivot: 2 base bolts, 1 axle) | 3 | – |
| 18 mm M6 steel washer (passive pivot) | 2 | – |
| M6 plain washer (brackets and rings) | 10 | 16 |
| M4 × 16 screw and M4 nut (handle pivot) | 1 + 1 | 4 + 4 |
| M4 × 25 screw and M4 nut (dogbone links) | 2 + 2 | 8 + 8 |
| M5 × 25 screw and M5 nut (hinge) | 1 + 1 | 4 + 4 |

### Open questions

The 2-arm quantities come from the CAD layout; the README in
`cad/stand/lower-pivot/` notes that physical fit and loading are untested.
These points need a decision before the counts are final:

- **Screw lengths.** The lower-pivot parts call for M6 × 25 countersunk screws
  on the active lowers, M6 × 20 on the passive pivot and an M4 × 16 handle
  pivot. Stand Build Guide 1.0 uses M6 × 30 socket-head screws with washers on
  the lower clamp, M6 × 25 on the pivot plate and its axle, and an M4 × 12
  handle pin.
- **Hinge.** The guide's hinge is M5 × 25. The actuator PitClamp moved to an
  M6 × 25 hinge with a 6 mm bore; check the lower-pivot uppers.
- **Screws per lower clamp.** Two, as in the guide; confirm against the part.
- **End caps** for the 2-arm stand's second arm.
- **Packaging** quantities for the new hardware.
