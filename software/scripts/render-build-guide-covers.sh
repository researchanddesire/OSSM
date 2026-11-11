#!/usr/bin/env bash
# Make the thumbnail shown for every build guide on the Build Guides docs
# page: the finished device as drawn on the guide's cover (page one), cropped
# away from the title, logo and support block so readers can see at a glance
# what each guide builds. Run after adding or replacing a guide PDF, then
# commit the PDF and its thumbnail together.
#
#   bash software/scripts/render-build-guide-covers.sh          # write thumbnails
#   bash software/scripts/render-build-guide-covers.sh --check  # verify only
#
# Needs pdftoppm (poppler), magick (ImageMagick 7) and cwebp (webp):
# `brew install poppler imagemagick webp`.
#
# The crop relies on the cover layout every guide shares: title block at the
# top, device drawing on a white background in the middle, and a rule above
# the support block (QR code, e-mail, link) near the bottom. The script keeps
# the band between the title block and the rule, trims the white around the
# drawing, and centres it on a square white canvas.
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
pdf_dir="$repo_root/hardware/build-guides"
cover_dir="$repo_root/assembly-docs/assets/build-guides"
size=800          # output thumbnail, square, in pixels
quality=82
band_top=19       # percent of page height: just below the title block
band_bottom=82    # percent of page height: just above the support-block rule
padding=8         # percent of white space added around the trimmed drawing

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
      echo "Missing thumbnail: assembly-docs/assets/build-guides/$name.webp" >&2
      status=1
    fi
  done
  for cover in "$cover_dir"/*.webp; do
    name=$(basename "$cover" .webp)
    if [ ! -f "$pdf_dir/$name.pdf" ]; then
      echo "Thumbnail without a PDF: assembly-docs/assets/build-guides/$name.webp" >&2
      status=1
    fi
  done
  [ "$status" -eq 0 ] && echo "Every build guide has a thumbnail."
  exit "$status"
fi

for tool_name in pdftoppm magick cwebp; do
  if ! command -v "$tool_name" >/dev/null 2>&1; then
    echo "Missing $tool_name. Install it with: brew install poppler imagemagick webp" >&2
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
  page="$work_dir/$name.png"
  pdftoppm -f 1 -l 1 -singlefile -png -r 200 "$pdf" "$work_dir/$name"

  read -r page_w page_h < <(magick identify -format '%w %h\n' "$page")
  top=$((page_h * band_top / 100))
  band_h=$((page_h * (band_bottom - band_top) / 100))

  # Keep the drawing band, then trim the white (and near-white JPEG noise)
  # around the device.
  magick "$page" -crop "${page_w}x${band_h}+0+${top}" +repage \
    -fuzz 8% -trim +repage "$work_dir/$name-object.png"
  read -r obj_w obj_h < <(magick identify -format '%w %h\n' "$work_dir/$name-object.png")
  if [ "$obj_w" -lt 50 ] || [ "$obj_h" -lt 50 ]; then
    echo "No drawing found on the cover of $name.pdf; check the cover layout." >&2
    exit 1
  fi

  # Centre it on a square white canvas with a little breathing room.
  side=$(( (obj_w > obj_h ? obj_w : obj_h) * (100 + 2 * padding) / 100 ))
  magick "$work_dir/$name-object.png" -background white -gravity center \
    -extent "${side}x${side}" -resize "${size}x${size}" \
    "$work_dir/$name-thumb.png"
  cwebp -quiet -q "$quality" -metadata none "$work_dir/$name-thumb.png" \
    -o "$cover_dir/$name.webp"
  echo "Rendered $name.webp"
done
