Imported unchanged from Alex Warner's `feature/rs485-hat-aim57-test` branch in
[`KinkyMakers/OSSM-Hardware`](https://github.com/KinkyMakers/OSSM-Hardware/tree/faa79768fccbdbec5b236f6c96457b45fcaca74c/Hardware/Servo%20Tools/Gold%20Motor/RS485-Hat-Test/lib/AimFrame)
at commit `faa79768fccbdbec5b236f6c96457b45fcaca74c` (2026-10-23).

Original location: `Hardware/Servo Tools/Gold Motor/RS485-Hat-Test/lib/AimFrame/src/`.
The source directory declares no separate per-file license. Its repository's
CERN-OHL-S v2 license is preserved verbatim here as `LICENSE`.

The production HAT service uses only the read-frame and CRC helpers. Its
receive-only parser and shared-bus policy are in `src/services/ossmHatProtocol.hpp`
and `src/services/ossmHat.cpp` under the firmware source root.
