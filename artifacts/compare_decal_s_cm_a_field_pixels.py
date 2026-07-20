"""Pixel-exact §0d.0 comparison for the two S-CM.a field views."""

from __future__ import annotations

from array import array
import json
from pathlib import Path

import bpy


ROOT = Path(__file__).resolve().parents[1]
PAIRS = (
    (
        "walls.006",
        "decal_s_cm_a_before_walls006_miter_w32.png",
        "decal_s_cm_a_field_walls006_miter_w32.png",
    ),
    (
        "sagging_wall[e6,e7,e8]",
        "decal_s_cm_a_before_sagging_wall_miter_w32.png",
        "decal_s_cm_a_field_sagging_wall_miter_w32.png",
    ),
)


def _pixels(path):
    image = bpy.data.images.load(str(path), check_existing=False)
    try:
        values = array("f", [0.0]) * (len(image.pixels))
        image.pixels.foreach_get(values)
        return tuple(image.size), values
    finally:
        bpy.data.images.remove(image)


def main():
    records = []
    for object_name, before_name, after_name in PAIRS:
        before_path = ROOT / "artifacts" / before_name
        after_path = ROOT / "artifacts" / after_name
        before_size, before_pixels = _pixels(before_path)
        after_size, after_pixels = _pixels(after_path)
        changed_components = sum(
            first != second
            for first, second in zip(before_pixels, after_pixels)
        )
        records.append(
            {
                "object": object_name,
                "before": before_name,
                "after": after_name,
                "size_before": before_size,
                "size_after": after_size,
                "changed_components": changed_components,
                "pixel_equal": (
                    before_size == after_size
                    and len(before_pixels) == len(after_pixels)
                    and changed_components == 0
                ),
            }
        )
    report = {
        "schema": "cftuv.decal_s_cm_a_field_pixels.v1",
        "records": records,
        "all_pixel_equal": all(item["pixel_equal"] for item in records),
    }
    output = ROOT / "artifacts" / "decal_s_cm_a_field_pixels.json"
    output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print("CFTUV_S_CM_A_PIXELS=" + json.dumps(report, ensure_ascii=False))
    if not report["all_pixel_equal"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
