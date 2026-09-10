# OSSM motor control harness

[`OSSM-Motor-Control-Harness.yml`](OSSM-Motor-Control-Harness.yml) is the
preserved WireViz source authored by Lucy Chapar. The source and all six images
are carried forward unchanged.

The harness connects TB1 to J12 and two loose power ferrules:

| Connection | Wire |
| --- | --- |
| TB1 1–4 → J12 1–4 | Four 250 mm, 24 AWG signal wires |
| TB1 5 → GND ferrule | One 250 mm, 18 AWG black wire |
| TB1 6 → 24V ferrule | One 250 mm, 18 AWG red wire |

Pin numbering, connector orientation, colors, sleeve length, contacts, and
cable ties are specified in the YAML. Keep it as the design source of truth.

The YAML uses five images in `images/`. The sixth image,
[`OSSM_Harness_Installed.png`](images/OSSM_Harness_Installed.png), is the
installation reference photo.

Run `bash scripts/render-cables.sh` from the repository root to generate
diagrams and the detailed cable BOM under `cable-export/`.

## Provenance

- Author: Lucy Chapar.
- Initial cable: `ae60351d44b2895871b91dec157afa211080c5cd`.
- PNG image update: `80bc40ce4aed52b6f6f0d94e9feba8fc5539846c`.
- Move to `hardware/cables/`: `5176bf28707237948e1e99e8ec1ac4bc33a9fea3`.
- Preserved YAML SHA-256:
  `6e67f59054a1f7aefae94e73372c6fa4c623f2360d53002af9fd45154352d58a`.
- Hardware license: [CERN-OHL-S v2](../../LICENSES/CERN-OHL-S-2.0.txt).
