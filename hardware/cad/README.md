# OSSM CAD

STEP models for the parts in [the hardware BOM](../bom.csv).

- `actuator/`: body, belt clamps, tensioner, jam nut and 24 mm rail adapter.
- `stand/`: PCB enclosure, PitClamp, pivot plates, spacers, feet and end caps.
- [remote/](remote/README.md): wired remote body and knobs.
- `end-effectors/`: flange base and 50/63 mm rings.
- [jigs/](jigs/README.md): the actuator's two assembly jigs, a bearing
  alignment jig and a 3.8 mm pulley spacer, with their print projects.

STEP files are CAD interchange models. The pulley spacer has no STEP model
and is published as an STL. Most use millimetres; the PCB mount base
and lid use metres.
Pivot plate 1 is the right plate; plate 2 is the left plate.
The upper-and-handle BOM entry links to both component files.
