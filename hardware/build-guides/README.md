# OSSM build guides

Step-by-step PDF build guides for the OSSM. They are also listed on
[ohai.researchanddesire.com/ossm/build-guides](https://ohai.researchanddesire.com/ossm/build-guides).

Build in this order: the stand, the actuator, the wiring, then the wired remote.

## Current hardware

| File | Guide | Version on the cover |
| --- | --- | --- |
| [stand-1.0.pdf](stand-1.0.pdf) | OSSM Stand Build Guide, PitClamp stand | 1.0.14 |
| [actuator-1.1.pdf](actuator-1.1.pdf) | OSSM Actuator Build Guide, 24 mm Clamping Thread end effector | 1.1.11 |
| [wiring-1.0.pdf](wiring-1.0.pdf) | OSSM Wiring Guide, V2.3 board | 1.0.10 |
| [remote-1.0.pdf](remote-1.0.pdf) | OSSM Wired Remote Guide, Remote V2 | 1.0.8 |

## Earlier hardware

| File | Guide | Version on the cover |
| --- | --- | --- |
| [actuator-1.0.pdf](actuator-1.0.pdf) | OSSM Actuator Build Guide, 24 mm thread with a separate belt clamp | 1.0.16 |
| [stand-0.1.pdf](stand-0.1.pdf) | OSSM Stand Build Guide, pivot-plate stand with the PitClamp Mini V1.1 | 0.1.1 |
| [stand-0.0.pdf](stand-0.0.pdf) | OSSM Stand Build Guide, pivot-plate stand with the earlier PitClamp Mini kit | 0.0.5 |

## File names

A guide's version is `MAJOR.MINOR.PATCH`. The major is the hardware version, the
minor counts compatible part changes, and the patch counts documentation
corrections. Each file is named for its `MAJOR.MINOR` edition and holds that
edition's latest correction. A new correction replaces its file in place; a new
minor edition, such as actuator 1.2, gets a new file.

## Thumbnails

The docs page shows each guide as a thumbnail of the finished device, cropped
from the drawing on page one, in
`assembly-docs/assets/build-guides/<file>.webp`. After adding or replacing a
PDF, run `bash software/scripts/render-build-guide-covers.sh` and commit the
PDF and its thumbnail together. A new file also needs a thumbnail in the grid on
`assembly-docs/build-guides.mdx`. The `--check` option lists PDFs without a
thumbnail and thumbnails without a PDF.
