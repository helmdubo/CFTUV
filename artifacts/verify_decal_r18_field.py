"""R1.8 canonical six-object field gate, without saving the .blend."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import bpy


SCRIPT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(os.environ.get("CFTUV_SOURCE_ROOT", SCRIPT_ROOT)).resolve()
OUTPUT_JSON = Path(
    os.environ.get(
        "CFTUV_R18_FIELD_REPORT",
        str(SCRIPT_ROOT / "artifacts" / "decal_r18_field_acceptance.json"),
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

import verify_decal_r13_rr9a as field  # noqa: E402


def main():
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    report = {
        "schema": "cftuv.decal_r18.field.v1",
        "blender_version": bpy.app.version_string,
        "blend_file": bpy.data.filepath,
        "source_root": str(REPO_ROOT),
        "objects": [],
        "missing_objects": [],
        "blend_saved": False,
    }
    for object_name, requested_edges in field.FIELD_OBJECTS.items():
        obj = bpy.data.objects.get(object_name)
        if obj is None or obj.type != "MESH":
            report["missing_objects"].append(object_name)
            continue
        try:
            report["objects"].append(
                field._object_acceptance(obj, requested_edges)
            )
        except Exception as exc:
            report["objects"].append(
                {
                    "object": object_name,
                    "status": "SCRIPT_ERROR",
                    "error": repr(exc),
                }
            )
    OUTPUT_JSON.write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print("CFTUV_R18_FIELD=" + json.dumps(report, ensure_ascii=False))
    if report["missing_objects"] or any(
        record.get("status") == "SCRIPT_ERROR"
        for record in report["objects"]
    ):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
