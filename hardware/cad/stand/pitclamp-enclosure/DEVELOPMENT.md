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

The [OSSM assembly](../../ossm-assembly.FCStd) includes the visible Onshape
actuator on the final moving arm of both stand alternatives. The PCB enclosure
and cover, lower-pivot clamps, frame hardware, blue inlays and stand placements
are retained. The primary arm is seated at 30.4°; the double-arm final rail is
at 60.8°. The single-arm enclosure still extends toward the lower pivot.

The current source upper jaw, one-piece lever with integral pivot loop,
dogbones and 57AIM motor adapter replace the earlier nominal clamp context.
The actuator uses the community one-piece curved front in place of the stock
front/middle bodies and their two joining M3 screws/nuts. Four exactly
coincident source motor bolts are omitted, leaving one bolt at each station.
The two hidden electronics groups retain 23 references each.

Registration matches the upper collar and hinge axes to the existing clamp;
the source motor axis is 0.0263 mm from the previous nominal motor axis.
The curved front clears the checked motor and adapter parts; its overlap with
the lower body is below 0.0002 mm³ numerical residue. Source rigid-body contacts
remain: adapter/upper jaw 118.239 mm³, adapter/motor 0.356 mm³ and
upper jaw/lever 0.342 mm³. The retained lower/adapter contact is 63.906 mm³;
the source M6 hinge screw intersects the enclosure lower by 94.085 mm³. Its
axis aligns, but the retained 5.2 mm passage and source M6 screw require a
separate fit decision. A tested M5 substitution did not seat its nut cleanly,
so the source hardware layout is retained without assuming threaded engagement. The adapter and jaw cylinder axes are about 0.299 mm
eccentric in the source closed-clamp pose. The adapter/upper contacts are inherited from Onshape; the enclosure interfaces
are recorded separately. They do not establish operating clearance. The earlier
22.156 mm³ nominal-ring check is superseded by this source configuration.

The assembly-derived lever and adapter may need a tolerance-only BRep fix on
import; original individual STEP exports are retained. Deep Boolean checks
also flag curve/surface warnings in the source lever, including the historical
version. No new print package or physical-fit result is established by this
integration. The standalone enclosure and print artifacts above are unchanged.

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
- [ ] Check the source closed-clamp adapter/jaw contacts and operating clearance
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
