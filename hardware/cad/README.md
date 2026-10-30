# OSSM CAD

[Open the editable OSSM assembly](ossm-assembly.FCStd). It provides the stand and
lower-pivot foundation, with the single-arm frame shown by default. To view the
double-arm alternative, hide the single-arm group and show the double-arm group
in FreeCAD's model tree. Both configurations share the same origin.
The actuator is not included yet.

See the [lower-pivot parts and print files](stand/lower-pivot/README.md).

STEP component exports for the parts in [the hardware BOM](../bom.csv).

- `actuator/`: body, belt clamps, tensioner and jam nut.
- `stand/`: PCB enclosure, PitClamp components, pivot plates, handle spacer, feet and extrusion end caps.

Most STEP files use millimetres; the PCB mount base and lid use metres.
Pivot plate 1 is the right plate; plate 2 is the left plate.
The upper-and-handle BOM entry links to both component files.
