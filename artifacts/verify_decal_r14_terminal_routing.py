"""RM9 этап 1: routing-отчёт торцов на шести полевых объектах."""

from __future__ import annotations

import json
import sys
from dataclasses import replace
from pathlib import Path

import bmesh
import bpy


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

loaded_cftuv = sys.modules.get("cftuv")
if loaded_cftuv is not None:
    loaded_path = Path(getattr(loaded_cftuv, "__file__", "")).resolve()
    if REPO_ROOT not in loaded_path.parents:
        for module_name in tuple(sys.modules):
            if module_name == "cftuv" or module_name.startswith("cftuv."):
                del sys.modules[module_name]

from cftuv.analysis import build_patch_graph  # noqa: E402
from cftuv.decals import (  # noqa: E402
    compile_manual_seam_decal_plan,
    evaluate_manual_seam_faces,
)
from cftuv.model import DecalSettings  # noqa: E402


FIELD_OBJECTS = {
    "rounded_wall.001": (55, 59, 62, 65, 68, 71, 74, 77, 80, 83),
    "rounded_wall_noise_top": "ALL_SEAMS",
    "sagging_wall": "ALL_SEAMS",
    "wall_noise_top": (7, 8, 9, 10, 11, 12, 13, 14, 15, 17),
    "half_sphere": (6, 11, 16, 21, 26),
    "building": (412, 504, 553),
}
OUTPUT_JSON = REPO_ROOT / "artifacts" / "decal_r14_terminal_routing.json"


def _source_graph(obj, requested_edges):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    selected = (
        tuple(sorted(edge.index for edge in bm.edges if edge.seam))
        if requested_edges == "ALL_SEAMS"
        else tuple(sorted(int(edge_id) for edge_id in requested_edges))
    )
    try:
        graph = build_patch_graph(
            bm, [face.index for face in bm.faces], obj
        )
    except ValueError as exc:
        bm.free()
        if "editmode" not in str(exc).lower():
            raise
        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.mode_set(mode="EDIT")
        edit_bm = bmesh.from_edit_mesh(obj.data)
        edit_bm.verts.ensure_lookup_table()
        edit_bm.edges.ensure_lookup_table()
        edit_bm.faces.ensure_lookup_table()
        if requested_edges == "ALL_SEAMS":
            selected = tuple(
                sorted(edge.index for edge in edit_bm.edges if edge.seam)
            )
        try:
            graph = build_patch_graph(
                edit_bm,
                [face.index for face in edit_bm.faces],
                obj,
            )
        finally:
            bpy.ops.object.mode_set(mode="OBJECT")
        return graph, selected
    bm.free()
    return graph, selected


def main():
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    report = {
        "schema": "cftuv.decal_r14.terminal-routing.v1",
        "blender_version": bpy.app.version_string,
        "blend_file": bpy.data.filepath,
        "blend_saved": False,
        "objects": [],
    }
    for object_name, requested_edges in FIELD_OBJECTS.items():
        obj = bpy.data.objects.get(object_name)
        if obj is None or obj.type != "MESH":
            report["objects"].append(
                {"object": object_name, "status": "MISSING"}
            )
            continue
        try:
            graph, selected = _source_graph(obj, requested_edges)
            settings = DecalSettings(width_seam=0.8, offset=0.01)
            plan = compile_manual_seam_decal_plan(
                graph,
                settings,
                selected,
            )
            width_evaluations = []
            for width in (0.4, 0.8):
                try:
                    evaluated = evaluate_manual_seam_faces(
                        obj,
                        replace(settings, width_seam=width),
                        plan,
                        preview=True,
                    )
                    width_evaluations.append(
                        {
                            "width": width,
                            "status": "UPDATED",
                            "face_count": len(evaluated.faces),
                        }
                    )
                except Exception as exc:
                    width_evaluations.append(
                        {
                            "width": width,
                            "status": "ERROR",
                            "error": repr(exc),
                        }
                    )
            report["objects"].append(
                {
                    "object": object_name,
                    "status": "COMPILED",
                    "selected_edge_indices": list(selected),
                    "saved_mesh_selected_edge_indices": [
                        edge.index for edge in obj.data.edges if edge.select
                    ],
                    "backend_summary": plan.backend_summary,
                    "width_evaluations": width_evaluations,
                    "terminal_routing": [
                        {
                            "component_index": record.component_index,
                            "patch_id": record.patch_id,
                            "spine_vertex_id": record.spine_vertex_id,
                            "spine_edge_id": record.spine_edge_id,
                            "source_face_ids": list(record.source_face_ids),
                            "choice": record.choice,
                            "edge_ids": list(record.edge_ids),
                            "plan_kind": record.plan_kind,
                            "route_id": record.route_id,
                            "backend": record.backend,
                            "backend_kind": record.backend_kind,
                            "line": record.report_line,
                        }
                        for record in plan.terminal_routing
                    ],
                }
            )
            if object_name == "sagging_wall":
                saved_selection = tuple(
                    sorted(edge.index for edge in obj.data.edges if edge.select)
                )
                if saved_selection and saved_selection != selected:
                    saved_plan = compile_manual_seam_decal_plan(
                        graph, settings, saved_selection
                    )
                    saved_widths = []
                    for width in (0.4, 0.8):
                        evaluated = evaluate_manual_seam_faces(
                            obj,
                            replace(settings, width_seam=width),
                            saved_plan,
                            preview=True,
                        )
                        saved_widths.append(
                            {
                                "width": width,
                                "status": "UPDATED",
                                "face_count": len(evaluated.faces),
                            }
                        )
                    report["objects"][-1]["saved_selection_scope"] = {
                        "selected_edge_indices": list(saved_selection),
                        "backend_summary": saved_plan.backend_summary,
                        "width_evaluations": saved_widths,
                        "terminal_routing": list(
                            saved_plan.terminal_routing_report
                        ),
                    }
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
    print("CFTUV_R14_TERMINAL_ROUTING=" + json.dumps(report, ensure_ascii=False))
    if any(
        record["status"] == "SCRIPT_ERROR" for record in report["objects"]
    ):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
