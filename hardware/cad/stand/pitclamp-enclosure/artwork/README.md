# R+D conformal Neo Memphis artwork

`rd-conformal-neo-memphis.svg` is the editable production artwork. Preview body
outlines and clearance guides are excluded from the material regions. The
composition places the official R+D mark inside three parallel ribbons, with
repeated curves and consistent spacing. Cyan and pink sections belong to those
same ribbons, on a conformal white field. The supplied Neo Memphis reference
informs the geometry and restricted palette. The ribbon width is 2.5 mm, on a
4.5 mm pitch with 2 mm white gaps; complete turns and repeated stripe spacing
keep the pattern continuous around the mark.

The branding follows the [R+D homepage](https://www.researchanddesire.com/) and
[About Us](https://www.researchanddesire.com/pages/about-us). The archived
`rd-official-mark.svg` is the [official vector mark](https://dashboard.researchanddesire.com/safari-pinned-tab.svg),
SHA-256 `44e579ce48539a72a95f2da445e720bef7d37042b7395cd7691ddb4915fb4b88`.
Its original paths are retained; manufacturing contours receive only a microscopic
cusp cleanup, with the measured bound recorded in `source.json`. Exact source
curves are retained as hidden native reference geometry.

## Registration and printing

Artwork coordinates map directly to native X/Z: X is right and Z is up. Rotate
+90° about native X to place the planar profiles on rear Y0; no overall artwork
scale or translation is applied. The physical inlays extend inward through
Y0..0.6 mm. The outermost filled edges stay at least 5 mm inside the actual rear
perimeter and hole. A 0.01 mm guide allowance accounts for the imported edge
tolerances. The field terminates smoothly before the narrow clamp tail.

| Slot | Color | Use |
| --- | --- | --- |
| 1 | Black `#303842` | Body, R+D mark and geometric negative space |
| 2 | Cyan `#00D9E6` | Selected geometric accents and cover press faces |
| 3 | White `#FAFAFA` | Conformal rear field and cover I/O icons |
| 4 | Pink `#FF2E7A` | Selected rear accents and cover AUX heart |

The existing face-down orientation puts the decoration in the first three
0.2 mm face layers. Assign the actual loaded compatible filaments in the slicer.
Hardware and supporting source follow the repository's CERN-OHL-S v2 notices.
