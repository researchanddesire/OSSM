# OSSM CAD

STEP models for the parts in [the hardware BOM](../bom.csv).

- `actuator/`: body, belt clamps, tensioner, jam nut and 24 mm rail adapter.
- `stand/`: PCB enclosure, PitClamp, pivot plates, spacers, feet and end caps.
- [remote/](remote/README.md): wired remote body and knobs.
- `end-effectors/`: flange base and 50/63 mm rings.

STEP files are CAD interchange models. Most use millimetres; the PCB mount base
and lid use metres.
Pivot plate 1 is the right plate; plate 2 is the left plate.
The upper-and-handle BOM entry links to both component files.

Restored files come from the [released hardware inventory](https://github.com/researchanddesire/OSSM/tree/41d683fe104f4aef8f478f2ff61269a61e06a183).

The flange base restores the later 26 mm boss and 7 mm outer top fillet in the
original smooth STEP geometry. The shorter boss in the older STEP predates the
[May 2024 thread-engagement fix](https://github.com/researchanddesire/OSSM/commit/a8841fc6acc47bc4516f6be20ddfb93e506610cf).
Comparison against the later supplied mesh is within 0.036 mm; this verifies
geometry correspondence, not physical fit or strength.

Matching native CAD sources for the shipped Double Double and remote top cover
have not been recovered. They remain included in the product BOM; mesh
conversions are not included here.
