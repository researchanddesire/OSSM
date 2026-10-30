# OSSM actuator

The [OSSM assembly](../ossm-assembly.FCStd) includes the visible actuator parts
from the [Onshape source](https://cad.onshape.com/documents/d520ea9a8cadb4ae8681f59b/w/00b211a6fa3b76c59ef28f4e/e/5f2c80d9d9c2433a7be7f789),
exported on 2026-10-30: the 57AIM motor, 350 mm rail and carriage, belt, pulley,
end fittings, body and visible fasteners. The double-arm actuator is an internal instance of the same editable parts.
Both stand configurations retain
their existing rail positions and blue lower-pivot inlays.

The [curved one-piece front](curved-front/) by Lucy Chapar replaces
`body-cover.step` and `body-middle.step`. Its native FreeCAD source and STEP
are included. Omit the two cover-to-middle joining screws and side-loaded nuts.
The separate stock cover exports remain available for other configurations.

The motor clamp uses the source's one-piece overcenter lever and integral
pivot loop. The upper jaw, dogbones and motor adapter remain separate parts.
Current clamp STEP files are in the [actuator variant](../stand/actuator-pitclamp/README.md).

[source.json](source.json) maps the Onshape exports to local files, records
which existing components were retained, and identifies omitted duplicate
motor bolts. STEP files use millimetres. `24mm-thread.step` is the male end
adapter; `24mm-jam-nut.step` is a separate part.

The source closed-clamp pose contains small rigid-body overlaps, including
118.239 mm³ between the motor adapter and upper jaw. They are retained from
Onshape; clamp clearance, the enclosure hinge hardware and physical fit remain open checks. See the
[development notes](../stand/pitclamp-enclosure/DEVELOPMENT.md).

The original lower and middle body STEP files retain source import defects.
The installed lower uses a geometry-preserving 0.001 mm tolerance repair; the
installed front uses the curved replacement. Details are in the manifest.

Hardware source follows [CERN-OHL-S-2.0](../../../LICENSES/CERN-OHL-S-2.0.txt).
