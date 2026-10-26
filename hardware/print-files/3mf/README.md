# 3MF print projects

## Multicolor PCB cover

Open [pcb-mount-lid-multicolor.3mf](pcb-mount-lid-multicolor.3mf) as a project
in Bambu Studio. It contains one assembled cover with four named material
parts, positioned with the front face on the bed.

| Filament slot | Material region | Suggested color |
| --- | --- | --- |
| 1 | Clear light window, first 0.6 mm only | Clear |
| 2 | Cover and light-pipe socket | Black |
| 3 | Flush symbol inlays | White |
| 4 | Buttons, flexures, strikers, and fixed anchors | Cyan |

The four Ø3.4 mm mounting through-holes have 0.2 mm entry chamfers.
Screw heads sit above the cover.

The saved profile is Bambu Lab P2S, 0.4 mm nozzle, 0.20 mm layers. Select your
printer and actual filament profiles, check the slot assignments, and slice
before printing. The project contains no sliced toolpaths.

The light window keeps its 6 mm outer diameter and uses a nominal 1.7 mm bore
behind the clear face for a short clear-filament insert. Insert the segment
from the rear until it touches the window; verify the printed fit before
final assembly.

The editable [FreeCAD source](../../cad/stand/pcb-mount-lid-multicolor.FCStd)
and [STEP assembly](../../cad/stand/pcb-mount-lid-multicolor.step) are in
`hardware/cad/stand/`. Geometry and Bambu Studio import have been checked;
physical fit, button travel, and fatigue have not been validated.
