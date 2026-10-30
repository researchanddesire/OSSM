# Development status

Tracking snapshot added on 2026-10-30. The native geometry was saved on
2026-10-15; the accepted print package and slicer investigation were recorded
on 2026-10-16. This import records that existing work without changing its
geometry or treating it as a released replacement for the shipped parts.

## Current design

- Integrated PitClamp lower and electronics enclosure, with a separate 3 mm
  cover and a flat rear print plane.
- Filled, swept shoulder and matching 1.2 mm cover/shoulder chamfer; the shared
  large corner reaches R8.
- Continued hook-mouth and lower-edge bevels, refined hinge returns, and
  mounting supports that grow into the enclosure walls/roof.
- Four cover mounting holes with two 0.2 mm bridge transitions at each
  counterbore, plus self-tapping body pilots.
- M6 extrusion mounting access and a locally widened central counterbore mouth.
  The neighboring grip texture is relieved locally: the first flute is removed,
  the second is shortened, and the other 28 are retained. Physical holding force
  has not been measured.
- PCB supports, rest pads, cable relief, connector clearances, and 23 hidden
  colored electronics/harness/motor references retained for further review.

These features are present in the saved model. They are not a claim that every
surface join is tangent or curvature continuous, or that the industrial design
review is complete. Intentional chamfer creases, mating seams, screw seats, and
grip texture remain.

## Recorded digital checks

- The native body and cover were valid single solids; the STEP reimport
  contained the two corresponding solids.
- Accepted body STL: 45,456 triangles, one closed connected component.
- Accepted cover STL: 3,896 triangles, one closed connected component.
- Both STLs had zero open edges, nonmanifold edges/vertices, orientation
  defects, degenerate triangles, and reported self-intersections in the
  recorded independent mesh audits.
- Re-reading the archived 3MF preserved the accepted STL geometry exactly,
  apart from the rigid translations used for the bed layout.
- Bambu Studio reported both print objects manifold and completed the
  diagnostic slice. It retained a body cantilever warning.
- Modeled screw passages, regional pin/dogbone guards, rear backing, and
  sampled cover-removal positions were checked during development. Those
  regional checks do not prove full mechanism travel or physical fit.

The print artifacts and STEP are imported byte-for-byte. The PR copy of the
native document changes only its `License` and `LicenseURL` metadata from the
FreeCAD defaults to the repository's CERN-OHL-S v2 notice. Its geometry,
appearance, references, and all other document content remain identical; the
original workspace file is untouched. The [manifest](source.json) records both
native hashes and binds the recorded checks to this snapshot. This PR does
not claim that all geometry or physical checks were rerun on import.

## Stand assembly integration — 2026-10-30

The [OSSM assembly](../../ossm-assembly.FCStd) now includes one representative
motor-clamp/enclosure set on the final moving arm of each stand alternative.
The existing lower-pivot clamps, frame, and hardware are retained. The source
enclosure, cover, standard Mini upper, handle, dogbones, and nominal 57AIM ring
are placed without geometry changes. This assembly update does not alter the
standalone native enclosure, STEP, 3MF, or STLs recorded above.

The three enclosure mounting stations align to the rail slot and remain
33.782, 103.782, and 166.388 mm from its free end. The conservative body outline
stays 5 mm inside that end; the modeled body-to-tip-cap gap is 7.854 mm. The
body and cover were checked against the surrounding frame and end caps in both
placements, excluding the intentionally contacting mounting rail. These local
placement checks do not establish a complete interference-free mechanism.

The handle is raised rather than locked. Static checks found no positive
overlap among the enclosure, upper, handle, and dogbones, or between the cover
and those added parts. Correctly phased ring ribs clear the enclosure and cover;
the visible nominal 57AIM ring still overlaps the upper collar by 22.156 mm³.
It is a source reference only; that interface fit remains unresolved. Both
alternatives retain 23 hidden colored electronics and motor references for
packaging review. The actuator and new clamp/enclosure fasteners remain absent.
Full latch closure, hardware/tool access, mechanism travel, physical fit, and
load capacity remain open checks.

The saved assembly was reopened and checked: each alternative contains seven
valid single-solid parts in the added set, plus 23 hidden colored references.
All 511 original objects retain their geometry, placements, and appearance.
The document has no external links, opens with the single-arm alternative
visible, and was inspected in full-frame and close views for both alternatives.
These are digital assembly checks; the physical checks below remain open.

## Work to track

- [ ] Resolve fine STEP tessellation at the actual target-slicer settings.
      Require closed, consistently oriented, nonintersecting output after STEP
      import; checking only CAD-solid validity is insufficient.
- [ ] Review the remaining hook, hinge, M6 rim, standoff, and shared-corner
      transitions in close views before accepting further surface changes.
- [ ] Review the body cantilever warning and decide on support, orientation,
      or a local printable-geometry change using actual sliced toolpaths.
- [ ] Print body and cover; confirm the counterbore bridge layers, self-tapping
      screws, mounting-head access, and cover fit with the chosen material.
- [ ] Check the actual PCB, motor, connectors, barrel jack, and cable bundle
      against the housing and strain relief. Reference envelopes are not
      physical-fit proof.
- [ ] Resolve the nominal 57AIM ring's interference with the upper collar
      before fabricating the assembled motor-clamp variant.
- [ ] Verify complete upper-clamp/dogbone travel and extrusion mounting with
      real hardware, including fastener length and tool access.
- [ ] Measure loaded clamp retention and assess the locally relieved grip
      texture before adopting the variant for use.
- [ ] Improve the editable feature history or provide a portable, reproducible
      model-generation workflow before claiming parametric regeneration.

For each geometry revision, keep the native source and corresponding exports
together, update the checksum manifest, and record which CAD, mesh, slicer, and
physical checks actually passed. Keep failed experiments and older study
models out of this current-design directory.
