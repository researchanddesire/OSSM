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
holds the actuator and the four T-nuts for the Mini and the PCB box. Where the
sources disagreed, these follow the CAD: the Onshape lower clamp
"Lower V1.1 - SideMount Reinforced 3030" and the two stand assemblies in
[`cad/stand/lower-pivot/`](cad/stand/lower-pivot/README.md).

| Part | 1-arm | 2-arm |
| --- | --- | --- |
| 11.75 in extrusion (crossfeet) | 2 | 2 |
| 18 in extrusion (arm) | 1 | 2 |
| 19.5 in extrusion (base rail) | 1 | 1 |
| Base bracket | 4 | 4 |
| Printed foot and grippy foot | 4 + 4 | 4 + 4 |
| End cap | 6 | 8 |
| Active lower clamp | 1 | 2 |
| Active lower clamp, mirrored | – | 2 |
| Passive pivot | 1 | – |
| Mounting ring | 1 | 2 |
| Mounting ring, mirrored | – | 2 |
| Upper jaw and handle | 1 + 1 | 2 + 2 |
| Upper jaw and handle, mirrored | – | 2 + 2 |
| Dogbone links, hex and round (all clamps use the normal links) | 1 + 1 | 4 + 4 |
| M6 × 12 screw (brackets and feet) | 12 | 12 |
| M6 × 30 socket-head screw and M6 washer (lower clamps, 2 per clamp) | 2 + 2 | 8 + 8 |
| M6 × 25 socket-head screw (mounting rings, 2 per ring) | 2 | 8 |
| M6 × 20 socket-head screw (passive pivot: 2 base bolts, 1 axle) | 3 | – |
| 18 mm M6 steel washer (passive pivot) | 2 | – |
| M6 plain washer (brackets) | 8 | 8 |
| M6 T-nut | 19 | 28 |
| M5 × 25 screw and M5 nut (hinge) | 1 + 1 | 4 + 4 |
| M4 × 16 screw and M4 nut (handle pivot) | 1 + 1 | 4 + 4 |
| M4 × 25 screw and M4 nut (dogbone links) | 2 + 2 | 8 + 8 |

### Notes

- **Lower clamp.** The shipping part is the Onshape "Lower V1.1 - SideMount
  Reinforced 3030": three Ø6.2 mm holes on a 30 mm pitch. The outer two
  (60 mm apart) are counterbored and take the M6 × 30 screws; the middle hole
  stays empty. `lower-pivot/active-lower.step` is the earlier keyed base with
  two countersunk holes 46 mm apart. The mirrored clamp for the 2-arm stand
  is not in Onshape yet.
- **Hinge.** The lower clamp's hinge bore is 5.2 mm with an M5 hex nut trap,
  so the hinge is M5 × 25 + M5 nut.
- **Not yet tested:** physical fit and loading of the 2-arm stand (see the
  README in `cad/stand/lower-pivot/`), and packaging for its extra hardware.
