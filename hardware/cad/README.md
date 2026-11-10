# OSSM CAD

[Open the editable OSSM assembly](ossm-assembly.FCStd). It includes the actuator
on both stand configurations. The single-arm stand is shown by default; hide
its group and show the double-arm group in FreeCAD to switch configurations.
Both use the same origin.

The actuator tree groups linear motion, belt drive, body, PitClamp, end fittings
and fasteners into folders. The motor, carriage and belt each use one multibody
model; the linear rail remains separate.

The primary arm is seated at 30.4° above the base. The double-arm stand retains
its second-joint bend, placing the final arm at 60.8°.

The actuator uses the [curved one-piece front body](actuator/README.md), replacing
the stock front and middle covers. The motor clamp uses the current Onshape
parts, including the one-piece lever with its integral pivot loop. The
[integrated PitClamp PCB enclosure](stand/pitclamp-enclosure/README.md) remains
mounted on the final arm of each stand.

See the [lower-pivot parts and print files](stand/lower-pivot/README.md).

- `actuator/`: actuator components, curved-front native source and export manifest.
- `stand/`: PitClamp parts, PCB enclosure, lower pivots, feet and end caps.

STEP exports use millimetres unless noted otherwise. The original PCB mount
base and lid use metres. Pivot plate 1 is the right plate; plate 2 is the left.
