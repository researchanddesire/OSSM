# Lower Pitclamp pivot

Designed by **Alex - R+D** for OSSM stands using 3030 extrusion.

Both configurations are included in the [OSSM assembly](../../ossm-assembly.FCStd).

## How a hinge is built

Every hinge has **one active lower Pitclamp** on one side of the arm and
**one passive pivot** on the other. The active clamp locks the angle; the
passive pivot carries the other side on an axle screw.

Printed parts per hinge:

- [Active lower clamp](active-lower.step): the keyed base. Its 4 mm key sits
  in the T-slot.
- [Mounting ring](double-arm/mounting-ring.step), fixed to the arm.
- [Upper + handle, print in place](../pitclamp-upper-handle-print-in-place-v1.1.step),
  the same part as on the Pitclamp Mini.
- [Hex dogbone](double-arm/dogbone-nuts.step) and
  [round dogbone](double-arm/dogbone-bolts.step).
- [Passive pivot](passive-pivot.step), including its blue inlays.

Hardware per hinge, all of it sizes the OSSM already ships:

| Where | Hardware |
| --- | --- |
| Lower clamp base to the rail | 2 × M6 × 25 socket-head, 2 T-nuts, no washers |
| Mounting ring to the arm | 2 × M6 × 25 socket-head, 2 T-nuts |
| Clamp hinge | M5 × 20 socket-head, M5 nut in the hex trap |
| Dogbone links | 2 × M4 × 25 socket-head, 2 M4 nuts |
| Passive pivot to the rail | 2 × M6 × 16 socket-head, 2 M6 washers, 2 T-nuts |
| Passive pivot axle into the arm | M6 × 16 socket-head, 1 M6 washer, 1 T-nut |

The passive pivot's three screws each sit on a standard M6 washer (12 mm
across) in the same counterbore: Ø13 mm, 8 mm deep, flat floor. Head and
washer end 0.4 mm below the face, and an M6 × 16 reaches 6.4 mm past the
mounting face. Earlier files had Ø19 mm seats for an 18 mm washer on the base
bolts, which left the head proud, and an Ø11 mm conical seat on the axle that
took no washer. The passive pivot also has a third T-slot key between its base
bolts. The keys leave a 17 mm pocket around each bolt, as on the lower clamp,
so the T-nuts stay in place while the plate goes on.

The mounting ring has no centre hole: the Ø24 mm hole of earlier files is
filled.

Both changes follow the first test print (2026-11-12) and are not printed
yet. [`passive-pivot.3mf`](passive-pivot.3mf) still holds the plate of that
first print; it is regenerated once the second test print passes.

The active lower clamp's hinge nut trap is **5 mm deep with a flat floor**.
It was 4 mm deep with a 0.8 mm chamfer at the floor, which held the M5 nut
proud of the mounting face, against the rail and in the path of the second
arm. Now a 4.7 mm nut sits 0.3 mm below the face, and the M5 × 20 has 4 mm
of thread in it. The wall left between the trap and the slot for the upper's
eye is 1.9 mm, the same as on the Pitclamp Mini lower. The copies in
[`double-arm/`](double-arm/) carry the same trap.

Why each length fits is in [the BOM notes](../../../BOM.md#notes).

## Single hinge (1-arm stand)

One hinge on the base rail. [Complete frame assembly](active-passive-assembly.step).

## Double hinge (2-arm stand): two arms in series

One base rail, a first hinged arm and a second hinged arm: two hinges, so
**two active clamps and two passive pivots**. The second hinge mounts at the
far end of the first arm. No mirrored parts are needed.

The [complete frame assembly](double-arm-assembly.step) shows it with the
first arm upright and the second arm square to it. The second hinge is built
exactly like the first, carried from the base rail onto the first arm: its
clamp and passive pivot bolt to the face of the first arm that looks toward
the hinge end of the base, so they are on top when the first arm is folded
down onto the base and the stand can lie flat. That clamp is on the same side
of the stand as the first one. In use the second arm reaches out past the
hinge end of the base and the OSSM points away from the base. The second arm
sits in its mounting ring as the first does, 40 mm past the hinge, so its end
cap clears the first arm when the two are square.

The mirrored parts in [`double-arm/`](double-arm/) belong to the earlier
layout with two active clamps at each joint. They are kept for reference and
are not needed for either stand.

Both assembly STEPs were redrawn on 2026-11-11 with the hardware above and
the deeper nut trap. Their M6 × 16 and M5 × 20 screws, M5 nuts and washers
are plain solids, and the clasp is drawn in the upper's earlier, slightly
over-closed position.
[`ossm-assembly.FCStd`](../../ossm-assembly.FCStd) still shows the earlier
layout and screws.

## Printing and frame parts

The circular and semicircular accents are **blue, flush and 0.6 mm deep**.
The STEP files keep the base and blue inlays as separate bodies.

- [Lower clamps and mounting rings](lower-pitclamp-stand.3mf)
- [Upper clamps](pitclamp-uppers.3mf)
- [Handles and dogbones](pitclamp-latch-parts.3mf)
- [Passive pivot](passive-pivot.3mf)

The print projects keep **8 wall loops**. The lower-clamp project retains its
solid-infill reinforcement near the end of the latch. For the double-hinge
frame, print two of each hinge part.

Both frames use one **495.3 mm base rail** and two **304.8 mm crossfeet**.
Each arm is **457.2 mm** long: one for the single hinge, two for the double hinge.

## CAD source

The [lower Pitclamp project](https://github.com/researchanddesire/community-mods/tree/main/mods/ossm/lower-pitclamp)
and October 1, 2026 Onshape export `OSSM - Dual Stand Pivot Clamp.step` supply
the pivot parts. The upper clamps have their raised Pitclamp lettering removed,
and the mirrored OSSM lettering reads normally. The latches are seated on their
lower bolts.

Frame parts come from the [OSSM - Extrusion Base Onshape assembly](https://cad.onshape.com/documents/d520ea9a8cadb4ae8681f59b/w/00b211a6fa3b76c59ef28f4e/e/a97eb2dc016eb5c9594d0915),
exported October 30, 2026. Extrusion lengths above are measured from its geometry.
The assemblies are CAD references; physical fit and loading are untested,
and so is the hardware listed above.

Hardware and supporting files use the repository's **CERN-OHL-S v2** license.
