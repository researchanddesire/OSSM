# PitClamp PCB enclosure development

Development snapshot of an electronics enclosure integrated with the Standard
PitClamp Mini lower. This directory records the current CAD and printable
geometry so subsequent design changes can be reviewed in Git. It is a separate
variant; the shipped PitClamp and PCB enclosure files remain the BOM references.

## Files

| File | Purpose |
| --- | --- |
| [pitclamp-enclosure.FCStd](pitclamp-enclosure.FCStd) | Native FreeCAD document; start further geometry work here. |
| [print/pitclamp-enclosure.3mf](print/pitclamp-enclosure.3mf) | Verified print mesh package, with body and cover as separate objects. |
| [print/body.stl](print/body.stl), [print/cover.stl](print/cover.stl) | The same accepted print meshes as individual files. |
| [pitclamp-enclosure.step](pitclamp-enclosure.step) | Body and cover in assembly position for CAD interchange; see the STEP limitation below. |
| [m6x25-hinge-screw.step](m6x25-hinge-screw.step) | Standard ISO 4762 M6 × 25 screw used by the revised hinge. |
| [DEVELOPMENT.md](DEVELOPMENT.md) | Current design scope, digital checks, and remaining development work. |
| [source.json](source.json), [SHA256SUMS](SHA256SUMS) | Provenance, snapshot identities, and artifact checksums. |

All exported dimensions use millimeters. The current revision has a 6 mm
hinge passage, a 10.4 mm head counterbore, and a 10.2 mm across-flats M6 nut
trap. The original 1.9 mm supporting wall is retained. The assembly uses an
M6 × 25 screw and moves the M6 nut outward by 1 mm.

## Editing

The native document has `Enclosure`, `Cover`, and `ElectronicsReferences`
roots. The 23 colored electronics, motor, connector, and harness references are
embedded and hidden. Select a reference and press **Space** in FreeCAD to show
it when checking packaging. Hidden construction tools are not print parts.
The STEP and print exports contain only the enclosure and cover.

The enclosure's original `SurfaceContinuity` feature stores an edited solid. Its
dimension properties are descriptive metadata, and the original snapshot had no
Sketcher objects. A native chamfer and Boolean return features remain, but this
is not a fully constrained, end-to-end regenerating feature history. Three
new sketches and native pockets follow it: the 6 mm passage, head counterbore,
and M6 nut trap. `M6NutTrap` is the current body tip. The circle profiles are
dimensioned; the hexagon profile records six fixed segments. Earlier geometry
and the hidden references remain available in the native document.

## Representative stand assembly

The [OSSM assembly](../../ossm-assembly.FCStd) includes this enclosure and cover
on the final moving arm of both stand configurations, together with the
actuator. The single-arm housing extends toward the lower pivot, with the motor
ring on the opposite side of the rail. The double-arm housing extends toward
its arm pivot. All three M6 mounting stations align to the rail slot.

The actuator uses the curved one-piece front housing and motor-clamp parts
from the current Onshape source. The primary arm remains seated at 30.4°; the
complete upper assembly moves with its rail. Each configuration retains its
23 hidden electronics references for packaging review.

The standalone enclosure and body print mesh include the same M6 revision;
the cover is unchanged. See the [development notes](DEVELOPMENT.md) for the
remaining source clamp contacts and development work.

## Printing

Use the **3MF** or the individual STLs. The body has its rear face down; the
cover has its outside face down. The generic 3MF lays them apart with a 10 mm
gap and contains no printer, filament, or process settings. Select suitable
settings for the actual printer and material.

The cover counterbores use two orthogonal 0.2 mm bridge transitions. Preserve
that layer relationship when choosing the cover's layer height and inspecting
its toolpaths. This does not establish that the complete body needs no support.

The revised packaged meshes passed a Bambu Studio 02.08.02.61 check:
both objects were manifold, each had one connected part, and a diagnostic
P2S/0.4 mm/0.20 Standard/PLA slice completed successfully. Bambu still flagged
a body cantilever for support or orientation review. No physical print or
loaded-fit result is established by those digital checks.

### STEP limitation

The STEP reimports as two valid CAD solids. Earlier fine slicer tessellation
created open seams and intersecting triangles. In the investigated setup,
Bambu's STEP import settings were 0.001 mm linear and 0.05 rad angular
deflection. Attempts to heal only the STEP representation remained sensitive
to mesh resolution and were not adopted. A valid BRep is not proof of a clean
slicer mesh: use the supplied 3MF/STLs for printing until this route is
requalified for the revised STEP.

## Snapshot checks

From this directory, verify the tracked binary artifacts with:

```sh
shasum -a 256 -c SHA256SUMS
```

See [source.json](source.json) for the validation date and the exact recorded
mesh counts. Future geometry edits need fresh exports and slicer checks; the
recorded result applies only to the checksummed snapshot.

## Provenance and license

The initial enclosure was developed from the
[Standard PitClamp Mini lower v1.1](https://github.com/KinkyMakers/OSSM-hardware/blob/b7f01bf6df1be6f3ebf17dc0e31ed64ddf4c15b7/Printed%20Parts/Mounting/STEP/OSSM%20-%20Base%20-%20PitClamp%20Mini%20-%20Lower%20V1.1.stp).
The [OSSM PCB case](https://github.com/KinkyMakers/OSSM-hardware/blob/b7f01bf6df1be6f3ebf17dc0e31ed64ddf4c15b7/Printed%20Parts/Mounting/STEP/OSSM%20-%20PCB%20-%20Box.stp)
was used for the original mounting measurements. Those are pinned source
references, not a claim that the final development variant retains every
original interface unchanged. Embedded electronics remain reference geometry;
their presence does not prove actual component or cable fit.

Hardware and supporting source follow the repository's
[license notices](../../../../LICENSE) and
[CERN-OHL-S v2 license](../../../../LICENSES/CERN-OHL-S-2.0.txt).
