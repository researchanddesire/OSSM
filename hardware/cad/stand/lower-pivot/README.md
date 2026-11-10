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
| Passive pivot axle into the arm | M6 × 16 socket-head, 1 T-nut |

The passive pivot's axle seat is 2.2 mm deeper than in the first release of
this file, so that an M6 × 16 reaches 7 mm into the arm's slot. Why each
length fits is in [the BOM notes](../../../BOM.md#notes).

## Single hinge (1-arm stand)

One hinge on the base rail. [Complete frame assembly](active-passive-assembly.step).

## Double hinge (2-arm stand): two arms in series

One base rail, a first hinged arm and a second hinged arm: two hinges, so
**two active clamps and two passive pivots**. The second hinge mounts at the
far end of the first arm. No mirrored parts are needed.

The [complete frame assembly](double-arm-assembly.step) and the mirrored
parts in [`double-arm/`](double-arm/) show the earlier layout with two active
clamps (one normal, one mirrored) at each joint. They are kept for reference
until the assembly is redrawn. Both assembly STEPs also still model the
earlier screws.

## Printing and frame parts

The circular and semicircular accents are **blue, flush and 0.6 mm deep**.
The STEP files keep the base and blue inlays as separate bodies.

- [Lower clamps and mounting rings](lower-pitclamp-stand.3mf)
- [Upper clamps](pitclamp-uppers.3mf)
- [Handles and dogbones](pitclamp-latch-parts.3mf)
- [Passive pivot](passive-pivot.3mf)

The print projects keep **8 wall loops**. The lower-clamp project retains its
solid-infill reinforcement near the end of the latch. `passive-pivot.3mf`
still holds the passive pivot with the earlier, shallower axle seat; print
from the STEP until it is re-saved. For the double-hinge frame, print two of
each hinge part.

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
