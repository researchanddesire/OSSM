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
[lower PitClamp pivot](cad/stand/lower-pivot/README.md) parts, and every hinge
is built the same way: one active lower PitClamp locks the angle on one side
of the arm and a passive pivot plate carries the other side.

- **1-arm stand** (single hinge): one 18 in arm on the base rail.
- **2-arm stand** (double hinge): two 18 in arms in series, so two active
  lower PitClamps and two passive pivots. No mirrored parts are needed.

The pivot-plate stand (two printed pivot plates with adjustable M6 handles) is
discontinued. Its files stay in [`cad/stand/`](cad/stand/README.md) for
existing stands.

`bom.csv` lists the 1-arm stand. Rows that change for the 2-arm stand give its
quantity in `Notes`.

### No stand-only fasteners

The stands use only fastener sizes the OSSM already ships: M6 × 12, M6 × 16,
M6 × 25, M6 washers, M6 T-nuts, M5 × 20, M5 nuts, M4 × 25 and M4 nuts. Keep it
that way: change a quantity rather than add a size.

### Stand parts by build

Quantities are for the stand alone. They leave out the PitClamp Mini that
holds the actuator and the four T-nuts for the Mini and the PCB box.

| Part | 1-arm | 2-arm |
| --- | --- | --- |
| 11.75 in extrusion (crossfeet) | 2 | 2 |
| 18 in extrusion (arm) | 1 | 2 |
| 19.5 in extrusion (base rail) | 1 | 1 |
| Base bracket | 4 | 4 |
| Printed foot and grippy foot | 4 + 4 | 4 + 4 |
| End cap | 6 | 8 |
| Active lower clamp (keyed base) | 1 | 2 |
| Mounting ring | 1 | 2 |
| Upper + handle, printed in place | 1 | 2 |
| Dogbone links, hex and round | 1 + 1 | 2 + 2 |
| Passive pivot | 1 | 2 |
| M6 × 12 screw (brackets and feet) | 12 | 12 |
| M6 × 25 screw (per clamp: 2 lower clamp base, 2 mounting ring) | 4 | 8 |
| M6 × 16 screw (per passive pivot: 2 base bolts, 1 axle) | 3 | 6 |
| M6 plain washer (brackets 8, passive pivot base bolts 2 per pivot) | 10 | 12 |
| M6 T-nut | 19 | 26 |
| M5 × 20 screw and M5 nut (hinge) | 1 + 1 | 2 + 2 |
| M4 × 25 screw and M4 nut (dogbone links) | 2 + 2 | 4 + 4 |

### Notes

Screw lengths follow the measured CAD. On the 3030 profile the slot floor is
9.3 mm below the face, the lips are 2 mm thick and the T-nut is 6 mm thick, so
a screw has to reach between 2 and 9.3 mm past the face.

- **Lower clamp base.** The keyed base in `lower-pivot/active-lower.step`, the
  Onshape part "Stand Pivot Clamp": 30 mm thick with a 4 mm key that sits in
  the T-slot, and two Ø6.4 mm holes 46 mm apart. Each hole has a Ø12.5 mm
  counterbore 11 mm deep that ends in a 90° seat. An M6 × 25 socket-head screw
  seats on it and reaches 7.3 mm past the face. No washer.
- **Mounting ring.** 2 × M6 × 25 on a flat seat with 17 mm of ring under the
  head: 8 mm past the face.
- **Hinge.** M5 × 20 with an M5 nut in the hex trap, as on the PitClamp Mini.
  The head runs in the Ø8.9 mm bore of the outer cheek and stops on the eye of
  the upper; 3 mm of thread reaches the nut.
- **Upper and handle.** The print-in-place part the PitClamp Mini uses
  (`cad/stand/pitclamp-upper-handle-print-in-place-v1.1.step`). It needs no
  pivot screw.
- **Passive pivot.** Two base bolts, M6 × 16 with an M6 washer on the
  counterbore floor (5.4 mm past the face), and an M6 × 16 axle into a T-nut
  in the arm (7 mm). The axle seat in `lower-pivot/passive-pivot.step` is
  2.2 mm deeper than in the earlier file to suit that screw.
- **Not yet tested:** nothing here has been printed or loaded with this
  hardware. To check on the first build: socket heads on the 90° seats of the
  lower clamp base, the hinge with 3 mm of thread in its nut, and standard
  washers in place of 18 mm ones on the passive pivot. The two assembly STEPs
  and `ossm-assembly.FCStd` still show the earlier hardware and, for the
  2-arm stand, two active clamps per joint.
