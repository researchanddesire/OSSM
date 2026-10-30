# Lower Pitclamp pivot

Designed by **Alex - R+D** for OSSM stands using 3030 extrusion.

Compare both frames in the [FreeCAD model](lower-pivot-frames.FCStd).

## Active + passive

- [Complete frame assembly](active-passive-assembly.step)
- [Active lower clamp](active-lower.step)
- [Passive pivot](passive-pivot.step)
- [Bambu Studio print project for active lowers and mounting rings](lower-pitclamp-stand.3mf)
- **2 × 18 mm steel washers** for the passive pivot.

Print the passive pivot from its STEP file, including its three bodies.

All other Pitclamp parts are standard: [upper](../pitclamp-upper-v1.1.step),
[handle](../pitclamp-handle.step), [hex dogbone](../pitclamp-dogbone-hex.step)
and [round dogbone](../pitclamp-dogbone-round.step).

The supplied 3MF keeps **8 wall loops** and solid-infill reinforcement near
the end of the lower clamp latch. Select the quantities needed for your build.

## Double arm

Use an active Pitclamp on each side, with the opposite side mirrored.
The [complete frame assembly](double-arm-assembly.step) shows both latches engaged.

Print one of each component in the two columns:

| Part | Active side | Mirrored side |
| --- | --- | --- |
| Lower clamp | [STEP](double-arm/active-lower.step) | [STEP](double-arm/mirrored-active-lower.step) |
| Upper | [STEP](double-arm/upper.step) | [STEP](double-arm/mirrored-upper.step) |
| Handle | [STEP](double-arm/handle.step) | [STEP](double-arm/mirrored-handle.step) |
| Mounting ring | [STEP](double-arm/mounting-ring.step) | [STEP](double-arm/mirrored-mounting-ring.step) |
| Dogbone nuts | [STEP](double-arm/dogbone-nuts.step) | [STEP](double-arm/mirrored-dogbone-nuts.step) |
| Dogbone bolts | [STEP](double-arm/dogbone-bolts.step) | [STEP](double-arm/mirrored-dogbone-bolts.step) |

Use standard Pitclamp hardware on each active side, including an **M4 × 16
socket-head screw and M4 nut** for each handle pivot. Keep **8 wall loops** and
the solid latch-end reinforcement when preparing the mirrored lower clamp.

The single frame has one 495.3 mm base rail, one 457.2 mm arm and two
304.8 mm crossfeet. The double frame uses two base rails and two arms.

Active lowers use **M6 × 25 countersunk screws**; mounting rings use
**M6 × 25 socket-head screws**. The passive plate uses **M6 × 20 socket-head
screws**, with its two 18 mm washers under the base bolts.

Check that the clamps and pivots are secure before use.

## CAD source

The active/passive STEP and 3MF files come from the
[published lower Pitclamp project](https://github.com/researchanddesire/community-mods/tree/main/mods/ossm/lower-pitclamp).
The double-arm pivot is based on the October 1, 2026 Onshape export
`OSSM - Dual Stand Pivot Clamp.step`; its individual components are exported
from that assembly, with modeled metal fasteners omitted from the print parts.
The raised Pitclamp lettering has been removed from both upper clamps.
The mirrored OSSM lettering reads normally in the STEP and print files, and
the assembly includes both latch mechanisms with the lower bolts seated in
their hooks.

Frame components come from the [OSSM - Extrusion Base Onshape assembly](https://cad.onshape.com/documents/d520ea9a8cadb4ae8681f59b/w/00b211a6fa3b76c59ef28f4e/e/a97eb2dc016eb5c9594d0915),
exported on October 30, 2026. The double frame duplicates its base rail and
arm; the modeled extrusion lengths above are measured from the source geometry.

Hardware and supporting files use the repository's **CERN-OHL-S v2** license.
