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
| M6 plain washer (brackets 8, passive pivot 3 per pivot) | 11 | 14 |
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
  head: 8 mm past the face. Each seat is a Ø13 mm counterbore (Ø18.5 mm in
  earlier files), which fits the socket head and would take a standard M6
  washer. The ring has no centre hole (the Ø24 mm hole of earlier files is
  filled) and a third key in the arm's slot between the two screws, so its
  T-nuts stay in place too.
- **Hinge.** M5 × 20 with an M5 nut in the hex trap, as on the PitClamp Mini.
  The head runs in the Ø8.9 mm bore of the outer cheek and stops on the eye of
  the upper; 4 mm of thread reaches the nut. The trap in the lower clamp base
  is 5 mm deep with a flat floor, so a 4.7 mm nut sits 0.3 mm below the
  mounting face and the base seats flat on the rail. It was 4 mm deep with a
  chamfered floor, which left the nut proud. The wall between the trap and
  the slot for the upper's eye is 1.9 mm, as on the PitClamp Mini lower.
- **Upper and handle.** The print-in-place part the PitClamp Mini uses
  (`cad/stand/pitclamp-upper-handle-print-in-place-v1.1.step`). It needs no
  pivot screw.
- **Passive pivot.** Two base bolts into the rail and an axle into a T-nut
  in the arm, all three M6 × 16 on a standard M6 washer (ISO 7089, 12 mm
  across). Each sits in the same counterbore, Ø13 mm and 8 mm deep with a
  flat floor: the head ends 0.4 mm below the face and the screw reaches
  6.4 mm past the mounting face. Three keys sit in the rail's slot and leave
  a 17 mm pocket around each base bolt, as on the lower clamp base, so the
  T-nuts stay where they are put.
- **Tested so far:** the first test print (2026-11-12) confirmed the lower
  clamp base with this hardware: socket heads on the 90° seats, M5 × 20 and
  the M5 nut in the 5 mm trap. The passive pivot's washer counterbores and
  third key and the ring without its centre hole come from that print and
  are in the second test print; they are not printed yet. Nothing has been
  load tested. The two assembly STEPs
  show this hardware; `ossm-assembly.FCStd` still shows the earlier screws
  and, for the 2-arm stand, two active clamps per joint.
