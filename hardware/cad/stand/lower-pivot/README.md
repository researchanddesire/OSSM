# Lower Pitclamp pivot

Designed by **Alex - R+D** for OSSM stands using 3030 extrusion.

Both configurations are included in the [OSSM assembly](../../ossm-assembly.FCStd).

## Single hinge: active + passive

- [Complete frame assembly](active-passive-assembly.step)
- [Active lower clamp](active-lower.step)
- [Passive pivot](passive-pivot.step), including its blue inlays
- **2 × 18 mm steel washers** for the passive pivot

The other Pitclamp parts use standard geometry with the same blue trim:
[upper](double-arm/upper.step), [handle](double-arm/handle.step),
[hex dogbone](double-arm/dogbone-nuts.step) and
[round dogbone](double-arm/dogbone-bolts.step).

## Double hinge: two arms in series

The [complete frame assembly](double-arm-assembly.step) has one base rail,
a first hinged arm, and a second hinged arm. **Each joint uses two active
Pitclamps**, one on each side of the extrusion: four clamps total. The second
pair mounts at the far end of the first arm, facing toward the first pair.

Print **two of each component in both columns**: one normal and one mirrored
set per joint.

| Part | Normal | Mirrored |
| --- | --- | --- |
| Lower clamp | [STEP](double-arm/active-lower.step) | [STEP](double-arm/mirrored-active-lower.step) |
| Upper | [STEP](double-arm/upper.step) | [STEP](double-arm/mirrored-upper.step) |
| Handle | [STEP](double-arm/handle.step) | [STEP](double-arm/mirrored-handle.step) |
| Mounting ring | [STEP](double-arm/mounting-ring.step) | [STEP](double-arm/mirrored-mounting-ring.step) |
| Dogbone nuts | [STEP](double-arm/dogbone-nuts.step) | [STEP](double-arm/mirrored-dogbone-nuts.step) |
| Dogbone bolts | [STEP](double-arm/dogbone-bolts.step) | [STEP](double-arm/mirrored-dogbone-bolts.step) |

Use standard Pitclamp hardware, including an **M4 × 16 socket-head screw and
M4 nut** for each handle pivot (four on the double-hinge frame).

## Printing and frame parts

The circular and semicircular accents are **blue, flush and 0.6 mm deep**.
The STEP files keep the base and blue inlays as separate bodies.

- [Lower clamps and mounting rings](lower-pitclamp-stand.3mf)
- [Upper clamps](pitclamp-uppers.3mf)
- [Handles and dogbones](pitclamp-latch-parts.3mf)
- [Passive pivot](passive-pivot.3mf)

The print projects keep **8 wall loops**. The lower-clamp project retains its
solid-infill reinforcement near the end of the latch. For the double-hinge
frame, print **four lowers and four mounting rings**, plus the quantities in
the table above; adjust the project quantities accordingly.

Both frames use one **495.3 mm base rail** and two **304.8 mm crossfeet**.
Each arm is **457.2 mm** long: one for the single hinge, two for the double hinge.

Active lowers use **M6 × 25 countersunk screws**; mounting rings use
**M6 × 25 socket-head screws**. The single frame's passive plate uses
**M6 × 20 socket-head screws**, with its two 18 mm washers under the base bolts.

## CAD source

The [lower Pitclamp project](https://github.com/researchanddesire/community-mods/tree/main/mods/ossm/lower-pitclamp)
and October 1, 2026 Onshape export `OSSM - Dual Stand Pivot Clamp.step` supply
the pivot parts. The upper clamps have their raised Pitclamp lettering removed,
and the mirrored OSSM lettering reads normally. The latches are seated on their
lower bolts.

Frame parts come from the [OSSM - Extrusion Base Onshape assembly](https://cad.onshape.com/documents/d520ea9a8cadb4ae8681f59b/w/00b211a6fa3b76c59ef28f4e/e/a97eb2dc016eb5c9594d0915),
exported October 30, 2026. Extrusion lengths above are measured from its geometry.
The assemblies are CAD references; physical fit and loading are untested.

Hardware and supporting files use the repository's **CERN-OHL-S v2** license.
