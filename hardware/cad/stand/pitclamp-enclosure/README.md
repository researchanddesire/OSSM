# PitClamp PCB enclosure development

Development snapshot of an electronics enclosure integrated with the Standard
PitClamp Mini lower. This directory records the current CAD and printable
geometry so subsequent design changes can be reviewed in Git. It is a separate
variant; the shipped PitClamp and PCB enclosure files remain the BOM references.

## Files

| File | Purpose |
| --- | --- |
| [pitclamp-enclosure.FCStd](pitclamp-enclosure.FCStd) | Native FreeCAD document; start further geometry work here. |
| [print/pitclamp-enclosure.3mf](print/pitclamp-enclosure.3mf) | Print mesh package: body and cover, each with four material regions. |
| [Bambu P2S 0.6 mm project](print/pitclamp-enclosure-bambu-p2s-0p6.3mf) | Unsliced four-color Bambu project; P2S 0.6 mm High Flow, 0.2 mm normal and first layers. |
| [print/body.stl](print/body.stl), [print/cover.stl](print/cover.stl) | Individual print meshes; the cover STL combines its material regions into one physical solid. |
| [Black](print/body-black.stl), [cyan](print/body-cyan.stl), [white](print/body-white.stl), [pink](print/body-pink.stl) body STLs | Rear artwork and black body, with a shared origin; import as one multipart body object. |
| [Black](print/cover-black.stl), [cyan](print/cover-cyan.stl), [white](print/cover-white.stl), [pink](print/cover-pink.stl) cover STLs | Separate material regions with a shared origin; import together as parts of one cover object. |
| [pitclamp-enclosure.step](pitclamp-enclosure.step) | Body and cover in assembly position for CAD interchange; see the STEP limitation below. |
| [m6x25-hinge-screw.step](m6x25-hinge-screw.step) | Standard ISO 4762 M6 × 25 screw used by the revised hinge. |
| [artwork/rd-conformal-neo-memphis.svg](artwork/rd-conformal-neo-memphis.svg) | Editable radiating heart-and-gear artwork and source notes. |
| [DEVELOPMENT.md](DEVELOPMENT.md) | Current design scope, digital checks, and remaining development work. |
| [source.json](source.json), [SHA256SUMS](SHA256SUMS) | Provenance, snapshot identities, and artifact checksums. |

All exported dimensions use millimeters. The current revision has a 6 mm
hinge passage, a 10.4 mm head counterbore, and a 10.2 mm across-flats M6 nut
trap. The original 1.9 mm supporting wall is retained. The assembly uses an
M6 × 25 screw and moves the M6 nut outward by 1 mm.

## Editing

The native document has `EnclosureAssembly`, `CoverAssembly`,
`ElectronicsReferences`, and hidden cover-control and brand-artwork construction
groups. `EnclosureAssembly` contains the black `Enclosure` body and cyan,
white and pink rear inlays. `CoverAssembly` contains the black
`Cover` body, cyan button faces, white I/O marks, and pink AUX heart. The 23
colored electronics, motor, connector, and harness references are
embedded and hidden. Select a reference and press **Space** in FreeCAD to show
it when checking packaging. Hidden construction tools are not print parts.
The STEP and print exports contain only the enclosure and cover.

The enclosure's original `SurfaceContinuity` feature stores an edited solid. Its
dimension properties are descriptive metadata, and the original snapshot had no
Sketcher objects. A native chamfer and Boolean return features remain, but this
is not a fully constrained, end-to-end regenerating feature history. Three
new sketches and native pockets follow it: the 6 mm passage, head counterbore,
and M6 nut trap. `M6NutTrap` retains that physical body; the subsequent
`BodyBrandPockets` tip partitions its rear face for the flush color inlays. The circle profiles are
dimensioned; the hexagon profile records six fixed segments. Earlier geometry
and the hidden references remain available in the native document.

The cover's reset and AUX flexures come from the latest
[community motherboard cover](https://github.com/researchanddesire/community-mods/tree/aee6c9e4df47ea35281a559f2808685ae531cd69/mods/ossm/ossm-rs485-pcb-cover).
Their clearances, anchors, guards, AUX travel stop, cyan press faces, and the AUX tip's
0.3 mm extension are retained at native scale. Separate striker extensions
reach the PitClamp's tilted board; their ends follow its 8.33° contact planes.
White power-input, USB, wired-remote and reset symbols, plus the pink AUX heart,
reuse the source artwork. A new circle-M symbol marks the shared motor-harness
exit. The inlays are 0.6 mm deep and flush with the exterior.

The I/O and control icons read with native +Z up from the usual folded stand
view. Each icon rotates a quarter turn around its existing anchor; the physical
buttons stay in place. Nominal folded single-arm and double-arm views confirm
reading direction, without establishing folded-motion clearance.

The opposite rear exterior has a large pink heart over the center of the cyan
open-hardware gear, covering its circular aperture. The original gear path
remains underneath, including the unchanged angles of its tapered opening.
The body stays black; white is used only in selected geometric accents. Segmented concentric arcs
radiate from the gear across the full profile toward the hinge, with staggered
starts and varied palette colors, cropped at the perimeter.
The composition uses the cover's four colors.

The art field follows the complete usable rear profile with a nominal 2.5 mm
inward margin, including the hinge-side return, narrow tail and concave curves.
A 0.01 mm tolerance guard keeps the colored contours inside that boundary.
The editable SVG contains the same flattened material footprints as the native
profiles; preview clipping is not required for the production geometry.

The rear inlays are 0.6 mm deep and flush, leaving the original wall's 2.4 mm
black backing in the 3 mm rear floor. The filled body and cover retain their
previous physical geometry.

Named cover construction stages store the resulting OpenCascade solids;
they are geometry snapshots, not a fully regenerating Part Design history.
Exact source tools, constrained artwork sketches and registration data remain
in the hidden construction group. The original cover perimeter, screw-hole
bridge layers, enclosure and hinge geometry are retained. Switch force,
travel and printed flexure durability still need a test with the real PCB.

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

The standalone enclosure and body print mesh retain the same M6 revision;
both stand variants include the updated controls and artwork cover. See the [development notes](DEVELOPMENT.md) for the
remaining source clamp contacts and development work.

## Printing

Use the **3MF** or the individual STLs. The body has its rear face down; the
cover has its outside face down. The generic 3MF lays them apart with a 10 mm
gap and contains no printer, filament, or process settings. Select suitable
settings for the actual printer and material.

In Bambu Studio, the portable generic 3MF imports all eight parts on filament 1;
its standard display colors do not assign Bambu filaments. Assign the parts
manually, or open the separate Bambu project above. It explicitly assigns
black 1, cyan/blue 2, white 3 and pink 4 to both objects. These are project
filament indices, not AMS bay numbers; choose the actual AMS mapping before printing.

The Bambu project targets P2S 0.6 mm High Flow with normal and first-layer heights
overridden to 0.2 mm, while its underlying preset label still reads
`0.30mm Standard @BBL P2S 0.6 nozzle`. Explicit Studio process-difference markers
keep both 0.2 mm overrides when reopening the project. A fresh GUI reopen and
reslice retained 194 layers and nine color changes. The actual exported toolpaths
were checked at all 18 colored region/layer combinations and all four orthogonal
bridge pairs at Z2.2/2.4 mm. Physical print quality, controls and fit remain unvalidated.

Both objects use the same four material slots: black `#303842`, cyan
`#00D9E6`, white `#FAFAFA` and pink `#FF2E7A`. Assign the loaded filament
to each region in the slicer. Use compatible colors of the same material for
the bonded inlays. Both decorated faces are already placed down, so their
0.6 mm inlays occupy the first three 0.2 mm face layers. The unchanged fused
`body.stl` and `cover.stl` are the single-color alternatives.

The cover counterbores use two orthogonal 0.2 mm bridge transitions. Preserve
that layer relationship when choosing the cover's layer height and inspecting
its toolpaths. This does not establish that the complete body needs no support.

The current body and cover material meshes are checked for closed, consistently
oriented manifold geometry. Self-intersection diagnostics and their source-mesh
comparison are recorded separately in the manifest. Each fused
single-color STL has one connected component. The 3MF contains two objects
with four material parts each. A diagnostic P2S/0.4 mm/0.20 Standard/PLA slice completed
for the complete package and for the cover alone; the cover-only check had no
warnings. This geometry check used a single filament, so it does not validate
an AMS assignment or color-change schedule. Only the diagnostic copy was
arranged to that printer's bed; the tracked generic layout is retained.

Bambu still flags the body's existing cantilever for support or orientation
review. No physical print or loaded-fit result is established by these checks.

### STEP limitation

The STEP contains the body and cover as separate color material products. Earlier fine slicer tessellation
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
