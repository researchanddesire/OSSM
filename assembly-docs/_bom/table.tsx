import bom from "./bom.json";
import categoryMap from "./categories.json";

export function OssmBomTable() {
  const rows = [...bom.rows].sort(
    (a, b) => a[2].localeCompare(b[2]) || Number(a[0]) - Number(b[0]),
  );
  const categories: Record<
    string,
    { label: string; background: string; foreground: string }
  > = categoryMap;
  const fields = [
    "line_item",
    "part_name",
    "category",
    "description",
    "qty",
    "unit_of_measure",
    "manufacturer",
    "mfg_part_number",
    "vendor",
    "vendor_part_number",
    "design_source",
    "purchase_source",
    "notes",
  ];

  return (
    <section className="bom not-prose my-6">
      <p className="mb-3 text-sm">
        <a
          className="text-fd-primary underline underline-offset-4"
          href={`https://github.com/researchanddesire/${bom.repo}/blob/${bom.commit}/hardware/bom.csv`}
        >
          View source CSV on GitHub
        </a>
      </p>
      <div
        className="bom-table-frame"
        role="region"
        aria-label="OSSM Bill of Materials, sorted by category"
        tabIndex={0}
      >
        <table className="bom-table">
          <thead>
            <tr>
              {bom.columns.map((column) => (
                <th key={column} scope="col">
                  {column}
                </th>
              ))}
            </tr>
          </thead>
          <tbody>
            {rows.map((row) => (
              <tr key={row[0]}>
                {row.map((raw, column) => {
                  const value = raw;
                  const sourceUrl = value.startsWith("https://")
                    ? value
                    : column === 10
                      ? `https://github.com/researchanddesire/${bom.repo}/blob/${bom.commit}/hardware/${value}`
                      : undefined;
                  return (
                    <td key={column} className={`bom-col-${fields[column]}`}>
                      {column === 2 && categories[value] ? (
                        <span
                          className="bom-cat"
                          tabIndex={0}
                          role="img"
                          title={categories[value].label}
                          aria-label={`Category: ${categories[value].label}`}
                          data-label={categories[value].label}
                          style={{
                            background: categories[value].background,
                            color: categories[value].foreground,
                          }}
                        >
                          {value}
                        </span>
                      ) : [0, 4, 5].includes(column) ? (
                        value
                      ) : (
                        <span className="bom-clamp" title={value}>
                          {[10, 11].includes(column) &&
                          value !== "–" &&
                          sourceUrl ? (
                            <a href={sourceUrl}>
                              {column === 10 ? "Design" : "Purchase"}
                            </a>
                          ) : (
                            value
                          )}
                        </span>
                      )}
                    </td>
                  );
                })}
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </section>
  );
}
