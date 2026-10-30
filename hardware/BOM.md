# Bill of Materials

Edit `hardware/bom.csv` using the canonical 13-column RAD BOM format. This CSV is
the source of truth for OHAI’s BOM. Assembly documentation reaches OHAI through
reviewed sync PRs on the matching staging or main branch.

Keep harnesses as top-level assemblies; detailed cable parts stay WireViz-owned.
Rows are sorted by Category and numbered sequentially. Preserve quantities
and units when reordering. Notes identify the actuator,
extrusion stand or electronics assembly.

`assembly-docs/_bom/bom.csv`, `source.json` and `bom.json` are generated snapshots.
Change the hardware CSV rather than those copies; regenerate them together and
pin the commit containing the CSV. The source-aware hub importer automates this
step once deployed.

Use `Design Source` for repository CAD, PCB and harness paths. Use `Purchase Source`
for vendor or webstore product pages, including kits where appropriate. Keep both
when a part has both a design and a purchase link. Never link to Google Drive files.
Mark optional parts and their build variant in `Notes`.
