# OSSM

A fresh home for the Research and Desire OSSM hardware designs.

The starting design is Lucy Chapar's motor control harness. Mechanical CAD and
PCB designs will be added as they are developed and reviewed. There is no
complete machine design or firmware release in this baseline.

## Repository layout

| Folder | Contents |
| --- | --- |
| [`hardware/cables/`](hardware/cables/) | Motor control harness source and reference images |
| [`hardware/cad/`](hardware/cad/) | New mechanical design sources and STEP exports |
| [`hardware/pcb/`](hardware/pcb/) | New PCB projects, schematics, and local libraries |
| [`assembly-docs/`](assembly-docs/) | Assembly documentation developed alongside these designs |

## Render the cable

Install Python 3 and Graphviz, then run:

```sh
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements-cables.txt
bash scripts/render-cables.sh
```

The command writes the diagram, PDF, HTML, and cable BOM to `cable-export/`.
Generated exports are excluded from Git. The electrical connections and wire
specifications remain in the WireViz source.

## License

Hardware designs use [CERN-OHL-S v2](LICENSES/CERN-OHL-S-2.0.txt).
The preserved harness and images retain their original provenance; see
[`hardware/cables/README.md`](hardware/cables/README.md).
