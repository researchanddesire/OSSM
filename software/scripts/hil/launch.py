"""Isolated PR entry point from the reviewed host installation."""
from pathlib import Path
import runpy
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run import main


if __name__ == '__main__':
    # Admission becomes mandatory only after all queue producers are deployed.
    # The separately installed guard survives a base harness update.
    gate = Path(__file__).resolve().parent.parent / "release-admission"
    if (gate / "enabled").is_file():
        guard = runpy.run_path(str(gate / "release_admission.py"))
        guard["check_from_argv"](gate / "admitted.json")
    raise SystemExit(main())
