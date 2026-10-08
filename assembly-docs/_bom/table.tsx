import bom from "./bom.json";
import categoryMap from "./categories.json";

// Sections are recorded in CSV Notes; ungrouped additions remain visible.
export function OssmBomTable({
  assembly,
}: {
  assembly: "actuator" | "stand" | "electronics" | "other";
}) {
  const groups = {
    actuator: { title: "Actuator", prefix: "Actuator." },
    stand: { title: "Extrusion stand", prefix: "Extrusion stand." },
    electronics: { title: "Electronics and cables", prefix: "Electronics." },
    other: { title: "Other parts", prefix: "" },
  };
  const group = groups[assembly];
  const rows = bom.rows.filter((row) =>
    assembly === "other"
      ? !Object.values(groups).some(
          ({ prefix }) => prefix && row[11].startsWith(prefix),
        )
      : row[11].startsWith(group.prefix),
  );
  if (!rows.length) return null;
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
    "source",
    "notes",
  ];

  return (
    <section className="bom not-prose my-6">
      {assembly === "other" ? (
        <h2 className="mb-4 text-xl font-semibold">Other parts</h2>
      ) : null}
      <div
        className="bom-table-frame"
        role="region"
        aria-label={`${group.title} Bill of Materials table`}
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
                  const value =
                    column === 11
                      ? raw.replace(
                          /^(Actuator|Extrusion stand|Electronics)\.\s*/,
                          "",
                        ) || "–"
                      : raw;
                  const sourceUrl = value.startsWith("https://")
                    ? value
                    : `https://github.com/researchanddesire/${bom.repo}/blob/${bom.commit}/hardware/${value}`;
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
                          {column === 10 && value !== "–" ? (
                            <a href={sourceUrl}>Source</a>
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
