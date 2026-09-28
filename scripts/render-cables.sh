#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cable_dir="$repo_root/hardware/cables"
output_dir="${1:-$repo_root/cable-export}"

for tool_name in wireviz dot; do
  if ! command -v "$tool_name" >/dev/null 2>&1; then
    echo "Missing $tool_name. See README.md for the cable rendering setup." >&2
    exit 1
  fi
done

mkdir -p "$output_dir"
output_dir=$(cd "$output_dir" && pwd)

shopt -s nullglob
harnesses=("$cable_dir"/*.yml "$cable_dir"/*.yaml)
if [ "${#harnesses[@]}" -eq 0 ]; then
  echo "No WireViz source files found in $cable_dir." >&2
  exit 1
fi

for harness in "${harnesses[@]}"; do
  filename=$(basename "$harness")
  basename="${filename%.*}"
  (
    cd "$cable_dir"
    wireviz -f ghpst -o "$output_dir" "$filename"
    dot -Tpdf "$output_dir/$basename.gv" -o "$output_dir/$basename.pdf"
  )
  for extension in svg png html bom.tsv pdf; do
    if [ ! -s "$output_dir/$basename.$extension" ]; then
      echo "Missing or empty output: $basename.$extension" >&2
      exit 1
    fi
  done
done

echo "Cable exports written to $output_dir"
