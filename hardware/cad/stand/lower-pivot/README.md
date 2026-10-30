# Lower Pitclamp pivot

Designed by **Alex - R+D** for OSSM stands using 3030 extrusion.

## Active + passive

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
The [complete STEP assembly](double-arm-assembly.step) shows how they fit.

Print one of each component in the two columns:

| Part | Active side | Mirrored side |
| --- | --- | --- |
| Lower clamp | [STEP](double-arm/active-lower.step) | [STEP](double-arm/mirrored-active-lower.step) |
| Upper | [STEP](double-arm/upper.step) | [STEP](double-arm/mirrored-upper.step) |
| Handle | [STEP](double-arm/handle.step) | [STEP](double-arm/mirrored-handle.step) |
| Mounting ring | [STEP](double-arm/mounting-ring.step) | [STEP](double-arm/mirrored-mounting-ring.step) |
| Dogbone nuts | [STEP](double-arm/dogbone-nuts.step) | [STEP](double-arm/mirrored-dogbone-nuts.step) |
| Dogbone bolts | [STEP](double-arm/dogbone-bolts.step) | [STEP](double-arm/mirrored-dogbone-bolts.step) |

Use standard Pitclamp hardware on each active side. Keep **8 wall loops** and
the solid latch-end reinforcement when preparing the mirrored lower clamp.

Check that the clamps and pivots are secure before use.

## CAD source

The active/passive STEP and 3MF files come from the
[published lower Pitclamp project](https://github.com/researchanddesire/community-mods/tree/main/mods/ossm/lower-pitclamp).
The double-arm assembly is the October 1, 2026 Onshape export
`OSSM - Dual Stand Pivot Clamp.step`; its individual components are exported
from that assembly, with modeled metal fasteners omitted from the print parts.
The raised Pitclamp lettering has been removed from both upper clamps.
The mirrored OSSM lettering reads normally in the STEP and print files, and
the assembly includes aligned standard fasteners on both sides.

Hardware and supporting files use the repository's **CERN-OHL-S v2** license.
