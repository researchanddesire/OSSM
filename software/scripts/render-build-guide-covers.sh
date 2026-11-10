#!/usr/bin/env bash
# Render page one of every build guide PDF as the cover thumbnail shown on
# the Build Guides docs page. Run after adding or replacing a guide PDF, then
# commit the PDF and its thumbnail together.
#
#   bash software/scripts/render-build-guide-covers.sh          # write thumbnails
#   bash software/scripts/render-build-guide-covers.sh --check  # verify only
#
# Needs pdftoppm (poppler) and cwebp (webp): `brew install poppler webp`.
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
pdf_dir="$repo_root/hardware/build-guides"
cover_dir="$repo_root/assembly-docs/assets/build-guides"
width=800
quality=82

shopt -s nullglob
pdfs=("$pdf_dir"/*.pdf)
if [ "${#pdfs[@]}" -eq 0 ]; then
  echo "No PDFs in $pdf_dir." >&2
  exit 1
fi

if [ "${1:-}" = "--check" ]; then
  status=0
  for pdf in "${pdfs[@]}"; do
    name=$(basename "$pdf" .pdf)
    cover="$cover_dir/$name.webp"
    if [ ! -f "$cover" ]; then
      echo "Missing cover: assembly-docs/assets/build-guides/$name.webp" >&2
      status=1
    fi
  done
  for cover in "$cover_dir"/*.webp; do
    name=$(basename "$cover" .webp)
    if [ ! -f "$pdf_dir/$name.pdf" ]; then
      echo "Cover without a PDF: assembly-docs/assets/build-guides/$name.webp" >&2
      status=1
    fi
  done
  [ "$status" -eq 0 ] && echo "Every build guide has a cover."
  exit "$status"
fi

for tool_name in pdftoppm cwebp; do
  if ! command -v "$tool_name" >/dev/null 2>&1; then
    echo "Missing $tool_name. Install it with: brew install poppler webp" >&2
    exit 1
  fi
done

mkdir -p "$cover_dir"
work_dir=$(mktemp -d)
trap 'rm -rf "$work_dir"' EXIT

for cover in "$cover_dir"/*.webp; do
  name=$(basename "$cover" .webp)
  if [ ! -f "$pdf_dir/$name.pdf" ]; then
    rm "$cover"
    echo "Removed $name.webp (no matching PDF)"
  fi
done

for pdf in "${pdfs[@]}"; do
  name=$(basename "$pdf" .pdf)
  pdftoppm -f 1 -l 1 -singlefile -png -scale-to-x "$width" -scale-to-y -1 \
    "$pdf" "$work_dir/$name"
  cwebp -quiet -q "$quality" -metadata none "$work_dir/$name.png" \
    -o "$cover_dir/$name.webp"
  echo "Rendered $name.webp"
done
