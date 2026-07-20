"""R1.8/RF28: field diagnosis of BEVEL arrangement identity.

The script is read-only: it opens the supplied .blend in background mode and
never saves it.  It reports every mesh that has at least two selected seam
edges so the user's wall-with-openings case is not guessed by object name.
"""

from __future__ import annotations

import json
import os
import sys
from dataclasses import replace
from pathlib import Path

import bpy


SCRIPT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(os.environ.get("CFTUV_SOURCE_ROOT", SCRIPT_ROOT)).resolve()
OUTPUT_JSON = Path(
    os.environ.get(
        "CFTUV_R18_REPORT",
        str(SCRIPT_ROOT / "artifacts" / "decal_r18_rc5_field.json"),
    )
)
for path in (REPO_ROOT, SCRIPT_ROOT / "artifacts"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

loaded_cftuv = sys.modules.get("cftuv")
if loaded_cftuv is not None:
    loaded_path = Path(getattr(loaded_cftuv, "__file__", "")).resolve()
    if REPO_ROOT not in loaded_path.parents:
        for module_name in tuple(sys.modules):
            if module_name == "cftuv" or module_name.startswith("cftuv."):
                del sys.modules[module_name]

from cftuv.decal_voronoi import serialize_network_faces  # noqa: E402
from cftuv.decals import (  # noqa: E402
    compile_manual_seam_decal_plan,
    evaluate_manual_seam_faces,
)
from cftuv.model import DecalSettings  # noqa: E402

import verify_decal_r13_rr9a as field  # noqa: E402


WIDTHS = (0.8, 3.2)


def _selected_seams(obj):
    return tuple(
        int(edge.index)
        for edge in obj.data.edges
        if edge.select and edge.use_seam
    )


def _segment_snapshot(faces):
    return serialize_network_faces(
        tuple(face for face in faces if str(face.component_kind) == "SEGMENT")
    )


def _evaluate(obj, selected):
    graph, selected, _all_seams = field._source_graph(obj, selected)
    base = DecalSettings(width_seam=0.8, offset=0.01)
    plan = compile_manual_seam_decal_plan(
        graph, base, selected, alpha_budget=max(WIDTHS)
    )
    widths = []
    for width in WIDTHS:
        by_mode = {}
        for join_mode in ("MITER", "BEVEL"):
            settings = replace(
                base,
                width_seam=width,
                corner_join_mode=join_mode,
            )
            result = evaluate_manual_seam_faces(
                obj, settings, plan, preview=True
            )
            faces = tuple(result.faces)
            by_mode[join_mode] = {
                "backend_summary": str(
                    getattr(result, "backend_summary", "reported-to-console")
                ),
                "face_count": len(faces),
                "segment_snapshot": _segment_snapshot(faces),
                "corner_pieces": [
                    {
                        "kind": str(face.component_kind),
                        "side": str(face.component_side),
                        "vertex_count": len(face.positions),
                        "keys": [repr(key) for key in face.vert_keys],
                    }
                    for face in faces
                    if str(face.component_kind) != "SEGMENT"
                ],
            }
        widths.append(
            {
                "width": width,
                "segment_equal": (
                    by_mode["MITER"]["segment_snapshot"]
                    == by_mode["BEVEL"]["segment_snapshot"]
                ),
                "miter": {
                    key: value
                    for key, value in by_mode["MITER"].items()
                    if key != "segment_snapshot"
                },
                "bevel": {
                    key: value
                    for key, value in by_mode["BEVEL"].items()
                    if key != "segment_snapshot"
                },
            }
        )
    return {
        "object": obj.name,
        "selected": list(selected),
        "widths": widths,
    }


def main():
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    candidates = []
    for obj in bpy.data.objects:
        if obj.type != "MESH":
            continue
        selected = _selected_seams(obj)
        if len(selected) < 2:
            continue
        try:
            candidates.append(_evaluate(obj, selected))
        except Exception as exc:  # named field evidence, never hidden
            candidates.append(
                {
                    "object": obj.name,
                    "selected": list(selected),
                    "error": repr(exc),
                }
            )
    payload = {
        "source_root": str(REPO_ROOT),
        "blend": str(bpy.data.filepath),
        "candidates": candidates,
    }
    OUTPUT_JSON.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print("CFTUV_R18_FIELD=" + json.dumps(payload, ensure_ascii=False))


if __name__ == "__main__":
    main()
