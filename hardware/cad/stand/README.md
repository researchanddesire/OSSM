# PitClamp upper and handle

The BOM's “PitClamp v1.1 Upper + Handle” entry contains one of each:

- [Upper v1.1](pitclamp-upper-v1.1.step)
- [Handle](pitclamp-handle.step)

## Multicolor PCB cover

This cover is remodeled from [pcb-mount-lid.step](pcb-mount-lid.step) with an
editable FreeCAD feature tree, printable symbols, and four material groups.

| File | Use |
| --- | --- |
| [pcb-mount-lid-multicolor.FCStd](pcb-mount-lid-multicolor.FCStd) | Editable FreeCAD source |
| [pcb-mount-lid-multicolor.step](pcb-mount-lid-multicolor.step) | STEP assembly with four named material groups and colors |
| [pcb-mount-lid-multicolor.3mf](../../print-files/3mf/pcb-mount-lid-multicolor.3mf) | Bambu Studio project with the four materials already assigned |

The native model's `PrintAssembly` contains the black cover, cyan buttons,
clear light window, and white symbol inlays. PCB reference geometry is excluded
from that assembly. Both cyan flexures extend 3 mm past their pivots into their
fixed anchors.

The motor-power opening is 13.35 × 12.50 mm with R1 corners. The central
four-pin J12 motor-control opening is 17 × 13 mm with R2 corners; its top edge
is 2 mm lower than the original opening. The remote/USB edge recess is 8 mm
deep. These dimensions are editable in `MotorTerminalDimensions`,
`StepperControlDimensions`, and `ConnectorRecessDimensions` respectively.

See the [3MF printing notes](../../print-files/3mf/README.md) for material
assignments and light-pipe assembly. Geometry and Bambu Studio import have
been checked; physical connector fit and button travel/fatigue still need
verification on hardware.
