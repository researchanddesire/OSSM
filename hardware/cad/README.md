# OSSM CAD

STEP component exports for the parts in [the hardware BOM](../bom.csv).

- `actuator/`: body, belt clamps, tensioner and jam nut.
- `mounting/`: PCB enclosure and PitClamp components.
- `stand/`: pivot plates, handle spacer, feet and extrusion end caps.

Exports retain their original Fusion 360 geometry and embedded STEP units.
Most use millimetres; the PCB mount base and lid use metres.
Pivot plate 1 is the original right plate; plate 2 is the original left plate.
The upper-and-handle BOM entry links to both component files.
No STEP export for the T-nut support was present in the original source.

Copied unchanged from [OSSM-hardware](https://github.com/KinkyMakers/OSSM-hardware/tree/b7f01bf6df1be6f3ebf17dc0e31ed64ddf4c15b7).
`source.json` records the original paths and file hashes. Hardware remains under
the repository's CERN-OHL-S-2.0 license.
