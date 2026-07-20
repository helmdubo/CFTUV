"""Blender field acceptance для R1.3 / RR9a без сохранения ``.blend``.

Собирает канонический PatchGraph для шести production-объектов, компилирует
strict SEAMS plan один раз и дважды читает его при разных width. Отчёт отдельно
публикует terminal pChain groups, выбранный guide и статичность rail-plan.
"""

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
from cftuv.decal_rails import compile_decal_rail_attempt  # noqa: E402
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
OUTPUT_JSON = REPO_ROOT / "artifacts" / "decal_r13_rr9a_field_acceptance.json"


def _source_graph(obj, requested_edges):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    all_seam_edges = tuple(sorted(edge.index for edge in bm.edges if edge.seam))
    if requested_edges == "ALL_SEAMS":
        selected = all_seam_edges
    else:
        selected = tuple(sorted(int(edge_id) for edge_id in requested_edges))
    try:
        graph = build_patch_graph(bm, [face.index for face in bm.faces], obj)
    except ValueError as exc:
        bm.free()
        if "editmode" not in str(exc).lower():
            raise
        # Patches with several loops use the isolated UV classifier and need
        # a real edit BMesh. The temporary mode switch lives only in this
        # unsaved headless process.
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
        else:
            selected = tuple(sorted(int(edge_id) for edge_id in requested_edges))
        try:
            graph = build_patch_graph(
                edit_bm,
                [face.index for face in edit_bm.faces],
                obj,
            )
        finally:
            bpy.ops.object.mode_set(mode="OBJECT")
        return graph, selected, selected if requested_edges == "ALL_SEAMS" else all_seam_edges
    bm.free()
    return graph, selected, all_seam_edges


def _failure_record(failure):
    return {
        "reason": str(getattr(failure, "reason", "UNKNOWN")),
        "edge_indices": list(getattr(failure, "edge_indices", ()) or ()),
        "vertex_indices": list(getattr(failure, "vertex_indices", ()) or ()),
        "face_indices": list(getattr(failure, "face_indices", ()) or ()),
        "details": [list(item) for item in getattr(failure, "details", ()) or ()],
    }


def _terminal_records(plan):
    rail_plan = plan.rail_plan
    if rail_plan is None:
        return []
    edge_by_id = {edge.edge_id: edge for edge in rail_plan.edges}
    vertex_by_id = {vertex.vertex_id: vertex for vertex in rail_plan.vertices}
    records = []
    for use in rail_plan.terminal_uses:
        pchain_candidates = tuple(
            edge_id
            for edge_id in use.candidate_edge_ids
            if edge_by_id[edge_id].is_pchain
        )
        if not pchain_candidates:
            continue
        spine_edge = edge_by_id[use.spine_edge_id]
        spine_other = next(
            vertex_id
            for vertex_id in spine_edge.vertex_ids
            if vertex_id != use.spine_vertex_id
        )
        origin = vertex_by_id[use.spine_vertex_id].position
        spine_vector = tuple(
            a - b
            for a, b in zip(vertex_by_id[spine_other].position, origin)
        )

        def measure(edge_id):
            edge = edge_by_id[edge_id]
            other = next(
                vertex_id
                for vertex_id in edge.vertex_ids
                if vertex_id != use.spine_vertex_id
            )
            candidate = tuple(
                a - b
                for a, b in zip(vertex_by_id[other].position, origin)
            )
            spine_length = sum(value * value for value in spine_vector) ** 0.5
            candidate_length = sum(value * value for value in candidate) ** 0.5
            return abs(
                sum(a * b for a, b in zip(spine_vector, candidate))
                / (spine_length * candidate_length)
            )

        records.append(
            {
                "spine_vertex_id": use.spine_vertex_id,
                "spine_edge_id": use.spine_edge_id,
                "source_face_ids": list(use.source_face_ids),
                "kind": use.kind.value,
                "pchain_candidate_edge_ids": list(pchain_candidates),
                "candidate_measures_abs_dot": {
                    str(edge_id): measure(edge_id)
                    for edge_id in pchain_candidates
                },
                "chosen_edge_id": use.route_edge_id,
                "route_id": use.route_id,
            }
        )
    return records


def _single_edge_rr9a_cases(graph, all_seam_edges):
    cases = []
    for edge_id in all_seam_edges:
        attempt = compile_decal_rail_attempt(
            graph,
            (edge_id,),
            alpha_budget=1.0,
        )
        if attempt.plan is None:
            continue
        edge_by_id = {edge.edge_id: edge for edge in attempt.plan.edges}
        for use in attempt.plan.terminal_uses:
            pchain_candidates = tuple(
                candidate_id
                for candidate_id in use.candidate_edge_ids
                if edge_by_id[candidate_id].is_pchain
            )
            if len(pchain_candidates) < 2:
                continue
            cases.append(
                {
                    "selected_edge_id": edge_id,
                    "spine_vertex_id": use.spine_vertex_id,
                    "source_face_ids": list(use.source_face_ids),
                    "candidate_edge_ids": list(pchain_candidates),
                    "kind": use.kind.value,
                    "chosen_edge_id": use.route_edge_id,
                    "route_id": use.route_id,
                }
            )
    return cases


def _object_acceptance(obj, requested_edges):
    graph, selected, all_seam_edges = _source_graph(obj, requested_edges)
    settings = DecalSettings(width_seam=0.8, offset=0.01)
    plan = compile_manual_seam_decal_plan(graph, settings, selected)
    record = {
        "object": obj.name,
        "selected_edge_count": len(selected),
        "selected_edge_indices": list(selected),
        "backend_summary": plan.backend_summary,
        "accounting_is_exact": plan.accounting_is_exact,
        "accepted_rail_planar": list(plan.accepted_rail_planar_edge_indices),
        "accepted_patch_voronoi": list(plan.accepted_patch_voronoi_edge_indices),
        "rejected": [
            {
                "edge_index": rejection.edge_index,
                "reason": rejection.reason,
            }
            for rejection in plan.rejected_edges
        ],
        "rail_compile_failures": [
            _failure_record(failure) for failure in plan.rail_compile_failures
        ],
        "rail_geometry_failures": [
            _failure_record(failure) for failure in plan.rail_geometry_failures
        ],
        "terminal_pchain_uses": _terminal_records(plan),
        "multi_candidate_terminal_count": 0,
        "rr9a_single_edge_cases": _single_edge_rr9a_cases(
            graph, all_seam_edges
        ),
        "width_evaluations": [],
    }
    record["multi_candidate_terminal_count"] = sum(
        len(use["pchain_candidate_edge_ids"]) >= 2
        for use in record["terminal_pchain_uses"]
    )
    compiled_plan_id = id(plan.rail_plan)
    for width in (0.4, 0.8):
        width_settings = replace(settings, width_seam=width)
        try:
            evaluated = evaluate_manual_seam_faces(
                obj,
                width_settings,
                plan,
                preview=True,
            )
            record["width_evaluations"].append(
                {
                    "width": width,
                    "status": "UPDATED",
                    "face_count": len(evaluated.faces),
                    "rail_plan_identity": id(plan.rail_plan),
                    "compile_static": id(plan.rail_plan) == compiled_plan_id,
                }
            )
        except Exception as exc:
            record["width_evaluations"].append(
                {
                    "width": width,
                    "status": "ERROR",
                    "error": repr(exc),
                    "rail_plan_identity": id(plan.rail_plan),
                    "compile_static": id(plan.rail_plan) == compiled_plan_id,
                }
            )
    return record


def main():
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    report = {
        "schema": "cftuv.decal_r13_rr9a.field.v1",
        "blender_version": bpy.app.version_string,
        "blend_file": bpy.data.filepath,
        "objects": [],
        "missing_objects": [],
        "blend_saved": False,
    }
    for object_name, requested_edges in FIELD_OBJECTS.items():
        obj = bpy.data.objects.get(object_name)
        if obj is None or obj.type != "MESH":
            report["missing_objects"].append(object_name)
            continue
        try:
            report["objects"].append(_object_acceptance(obj, requested_edges))
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
    print("CFTUV_R13_RR9A_FIELD=" + json.dumps(report, ensure_ascii=False))
    if report["missing_objects"] or any(
        record.get("status") == "SCRIPT_ERROR" for record in report["objects"]
    ):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
