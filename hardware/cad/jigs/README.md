# Actuator assembly jigs

Two printed jigs for building the actuator. Each one sets a position that
would otherwise be measured or judged by eye, and comes off again once its
part is fixed. The
[Actuator Build Guide](../../build-guides/actuator-1.1.pdf) shows them in
steps 2, 3, 7 and 8.

| Jig | Files | What it sets |
| --- | --- | --- |
| Bearing alignment jig | [STEP](bearing-alignment-jig.step) · [print project](bearing-alignment-jig.3mf) | The two idler bearing stacks on the body's lower part. |
| 3.8 mm pulley spacer | [STL](pulley-spacer-3.8mm.stl) · [print project](pulley-spacer-3.8mm.3mf) | How far the belt pulley goes onto the motor shaft. |

## Bearing alignment jig

A block with a cradle for each stack of three MR115 bearings and a handle on
its front. It sits in the body's lower part, in the seat where the linear
rail's carriage goes, so use it before the body goes on the rail.

1. Slide the jig into the seat, flat against the body.
2. Screw both bearing stacks in through its cradles until they are snug.
3. Pull the jig out by its handle and check that every bearing spins freely.

The STEP model shares the lower body's coordinates
([body-lower.step](../actuator/body-lower.step)), so the two line up when
imported together.

## 3.8 mm pulley spacer

A flat open-ended wrench, 3.8 mm thick, with an 8.5 mm slot for the motor's
8 mm shaft. It keeps the pulley from going too far onto the shaft.

1. Hang the spacer on the shaft, flat against the motor's front boss.
2. Slide the pulley on, hub first, until it touches the spacer.
3. Hold the pulley against the spacer and tighten both grub screws.
4. Lift the spacer off.

There is no STEP model of the spacer; the STL is the source.

## Printing

Both print in PLA at a 0.2 mm layer height with two walls and 15 % infill,
without supports. The print projects are Bambu Studio files: the bearing jig
for a Bambu Lab H2D, and a plate of fifteen spacers for a Bambu Lab A1. The
spacer's "3.8mm" lettering is a second colour in its print project; one
colour works as well.
