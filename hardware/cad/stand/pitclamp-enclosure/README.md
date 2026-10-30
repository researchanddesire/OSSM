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
| [DEVELOPMENT.md](DEVELOPMENT.md) | Current design scope, digital checks, and remaining development work. |
| [source.json](source.json), [SHA256SUMS](SHA256SUMS) | Provenance, snapshot identities, and artifact checksums. |

All exported dimensions use millimeters. The native model and exports were
copied without geometry changes from the current development workspace.

## Editing

The native document has `Enclosure`, `Cover`, and `ElectronicsReferences`
roots. The 23 colored electronics, motor, connector, and harness references are
embedded and hidden. Select a reference and press **Space** in FreeCAD to show
it when checking packaging. Hidden construction tools are not print parts.
The STEP and print exports contain only the enclosure and cover.

The enclosure's final `SurfaceContinuity` feature stores an edited solid. Its
dimension properties are descriptive metadata, and the current snapshot has no
Sketcher objects. A native chamfer and Boolean return features remain, but this
is not a fully constrained, end-to-end regenerating feature history. Continue
editing the saved native geometry; a complete portable rebuild script is not
included in this snapshot.

## Representative stand assembly

The [OSSM assembly](../../ossm-assembly.FCStd) includes this enclosure and cover
on the final moving arm of both the single-hinge and double-hinge stands. The
enclosure extends toward the arm pivot, with all three M6 mounting stations
aligned to the rail's T-slot. The standard Mini upper, raised handle, and both
dogbones are included as assembly context. The handle is not in a locked pose.

Each configuration retains its own 23 hidden colored electronics and motor
references. The nominal 57AIM ring is shown for context only. It clears the
enclosure and cover at this pose, but overlaps the upper collar by 22.156 mm³;
that source-interface fit remains unresolved. New clamp and enclosure fasteners
are not modeled, and the actuator is not included. These are representative
CAD placements, not proof of a complete
mechanism, physical fit, or load capacity. The standalone enclosure geometry
and print files are unchanged; see [the development notes](DEVELOPMENT.md).

## Printing

Use the **3MF** or the individual STLs. The body has its rear face down; the
cover has its outside face down. The generic 3MF lays them apart with a 10 mm
gap and contains no printer, filament, or process settings. Select suitable
settings for the actual printer and material.

The cover counterbores use two orthogonal 0.2 mm bridge transitions. Preserve
that layer relationship when choosing the cover's layer height and inspecting
its toolpaths. This does not establish that the complete body needs no support.

The exact packaged meshes passed the recorded Bambu Studio 02.07.01.57 check:
both objects were manifold, each had one connected part, and a diagnostic
P2S/0.4 mm/0.20 Standard/PLA slice completed successfully. Bambu still flagged
a body cantilever for support or orientation review. No physical print or
loaded-fit result is established by those digital checks.

### STEP limitation

The STEP reimports as two valid CAD solids, but fine slicer tessellation can
create open seams and intersecting triangles. In the investigated setup,
Bambu's STEP import settings were 0.001 mm linear and 0.05 rad angular
deflection. Attempts to heal only the STEP representation remained sensitive
to mesh resolution and were not adopted. A valid BRep is not proof of a clean
slicer mesh: use the supplied 3MF/STLs for printing until this is resolved.

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
