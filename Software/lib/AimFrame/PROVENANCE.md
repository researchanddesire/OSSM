Imported unchanged from Alex Warner's `feature/rs485-hat-aim57-test` branch in
[`KinkyMakers/OSSM-Hardware`](https://github.com/KinkyMakers/OSSM-Hardware/tree/faa79768fccbdbec5b236f6c96457b45fcaca74c/Hardware/Servo%20Tools/Gold%20Motor/RS485-Hat-Test/lib/AimFrame)
at commit `faa79768fccbdbec5b236f6c96457b45fcaca74c` (2026-10-23).

Original location: `Hardware/Servo Tools/Gold Motor/RS485-Hat-Test/lib/AimFrame/src/`.
The source directory declares no separate per-file license. Its repository's
CERN-OHL-S v2 license is preserved verbatim here as `LICENSE`.

The production HAT service uses the original read/CRC, FC06 output-control,
and FC7B absolute-position builders. Its single UART owner, external-master
policy and guarded/coalescing position transport are in `src/services/ossmHat.cpp`,
`ossmHatProtocol.hpp` and `ossmHatPositionProtocol.hpp` under the firmware source root.
FC10 at0x0C remains an incremental protocol example and is not used for moves.

The vendor FC7B reply's low-word-first position word is not a target echo:
an immediate current-position response can differ while a move is in progress.
The production owner correlates CRC, unit, function and frame length within
its bounded response window, retaining the raw word separately from the
intended request ID/target. The same rule recognizes a preempted owner's late
reply so it cannot falsely block priority output disable. FC06 still requires
an exact register/value echo. Completion and telemetry freshness come only
from the independent coherent FC03 encoder/current/voltage/alarm samples.
