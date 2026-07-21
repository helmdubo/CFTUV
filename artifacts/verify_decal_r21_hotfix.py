"""R2.1/RF29b/RF30: read-only field acceptance in Blender 4.3.

Исходный blend никогда не сохраняется. Harness компилирует один plan на
максимальную ширину каждого кейса и сравнивает preview/confirm по полной
serialization GeometryBatch.
"""

from __future__ import annotations

from dataclasses import replace
from hashlib import sha256
import json
import os
from pathlib import Path
import sys

import bmesh
import bpy


SCRIPT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(
    os.environ.get("CFTUV_SOURCE_ROOT", str(SCRIPT_ROOT))
).resolve()
OUTPUT_JSON = Path(
    os.environ.get(
        "CFTUV_R21_REPORT",
        str(SCRIPT_ROOT / "artifacts" / "decal_r21_hotfix_acceptance.json"),
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

from cftuv.analysis import build_analysis_bundle  # noqa: E402
from cftuv.decal_voronoi import serialize_network_faces  # noqa: E402
from cftuv.decals import (  # noqa: E402
    compile_manual_seam_decal_plan,
    evaluate_manual_seam_faces,
)
from cftuv.model import CornerJoinMode, DecalSettings  # noqa: E402


def _digest(faces):
    payload = json.dumps(
        serialize_network_faces(faces),
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return sha256(payload).hexdigest()


def _edge_adjacency(faces, source_edge_id):
    def point_key(point):
        return tuple(round(float(value), 5) for value in point)

    uses = {}
    for face in faces:
        count = len(face.positions)
        owner_edges = tuple(int(value) for value in face.provenance.source_edge_ids)
        for index, point in enumerate(face.positions):
            following = face.positions[(index + 1) % count]
            key = tuple(sorted((point_key(point), point_key(following))))
            uses.setdefault(key, []).append(
                {
                    "kind": str(face.component_kind),
                    "source_edges": list(owner_edges),
                }
            )
    records = []
    for key, edge_uses in sorted(uses.items()):
        if not any(
            int(source_edge_id) in use["source_edges"] for use in edge_uses
        ):
            continue
        records.append(
            {
                "points": [list(point) for point in key],
                "uses": edge_uses,
            }
        )
    suspect_curves = [
        record
        for record in records
        if any(use["kind"] == "BEVEL" for use in record["uses"])
    ]
    return {
        "source_edge": int(source_edge_id),
        "suspect_curve_witnesses": suspect_curves,
        "all_suspect_curves_have_two_material_owners": bool(suspect_curves)
        and all(len(record["uses"]) == 2 for record in suspect_curves),
        "single_use_edges": [
            record for record in records if len(record["uses"]) == 1
        ],
    }


def _source_graph(obj, requested_edges):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    all_seams = tuple(sorted(edge.index for edge in bm.edges if edge.seam))
    selected = (
        all_seams
        if requested_edges == "ALL_SEAMS"
        else tuple(sorted(int(edge_id) for edge_id in requested_edges))
    )
    try:
        bundle = build_analysis_bundle(
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
            bundle = build_analysis_bundle(
                edit_bm, [face.index for face in edit_bm.faces], obj
            )
        finally:
            bpy.ops.object.mode_set(mode="OBJECT")
        return bundle, selected
    bm.free()
    return bundle, selected


def _evaluate_case(object_name, selected, widths, join_mode):
    obj = bpy.data.objects[object_name]
    graph, selected = _source_graph(obj, selected)
    settings = DecalSettings(
        width_seam=float(widths[0]),
        offset=0.01,
        corner_join_mode=CornerJoinMode(join_mode),
    )
    plan = compile_manual_seam_decal_plan(
        graph,
        settings,
        selected,
        alpha_budget=max(float(width) for width in widths),
    )
    compiled_counts = {
        "surface_count": 0,
        "atom_count": 0,
        "point_atom_count": 0,
        "unbound_point_atom_count": 0,
        "corner_release_atom_count": 0,
    }
    for partition in plan.backend_partitions:
        for surface in getattr(partition.compiled_plan, "surfaces", ()):
            compiled_counts["surface_count"] += 1
            compiled_counts["atom_count"] += len(surface.atoms)
            point_atoms = tuple(
                atom for atom in surface.atoms if atom.cell_kind == "POINT"
            )
            compiled_counts["point_atom_count"] += len(point_atoms)
            compiled_counts["unbound_point_atom_count"] += sum(
                atom.corner_index < 0 for atom in point_atoms
            )
            releases = tuple(
                release
                for records in surface.corner_release_atoms.values()
                for release in records
            )
            compiled_counts["corner_release_atom_count"] += len(releases)
    records = []
    for width in widths:
        runtime = replace(settings, width_seam=float(width))
        evaluations = {}
        for preview in (True, False):
            key = "preview" if preview else "confirm"
            try:
                result = evaluate_manual_seam_faces(
                    obj, runtime, plan, preview=preview
                )
                evaluations[key] = {
                    "status": "OK",
                    "face_count": len(result.faces),
                    "digest": _digest(result.faces),
                    "policy_counts": [
                        list(item) for item in result.policy_counts
                    ],
                }
                if object_name == "walls.001" and float(width) == 3.2:
                    evaluations[key]["edge_94_adjacency"] = _edge_adjacency(
                        result.faces, 94
                    )
            except Exception as exc:
                evaluations[key] = {
                    "status": "ERROR",
                    "error": repr(exc),
                }
        records.append(
            {
                "width": float(width),
                "evaluations": evaluations,
                "preview_confirm_equal": (
                    evaluations["preview"].get("digest")
                    == evaluations["confirm"].get("digest")
                ),
            }
        )
    return {
        "object": object_name,
        "selected": list(selected),
        "join": join_mode,
        "compiled_counts": compiled_counts,
        "widths": records,
    }


def main():
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    blend_path = Path(bpy.data.filepath)
    report = {
        "schema": "cftuv.decal_r21_hotfix.v1",
        "source_commit": os.environ.get("CFTUV_SOURCE_COMMIT", "WORKTREE"),
        "source_root": str(REPO_ROOT),
        "blender_version": bpy.app.version_string,
        "blend_file": str(blend_path),
        "blend_sha256": sha256(blend_path.read_bytes()).hexdigest(),
        "blend_saved": False,
        "rf30": _evaluate_case(
            "walls.001",
            "ALL_SEAMS",
            (0.8, 1.6, 3.2),
            "BEVEL",
        ),
        "rf29b": _evaluate_case(
            "sagging_wall",
            (6, 7, 8),
            (7.8, 12.0, 18.0, 24.0, 30.0, 36.0),
            "MITER",
        ),
    }
    rf30_oracle = report["rf30"]["widths"][-1]["evaluations"][
        "preview"
    ]["edge_94_adjacency"]
    report["rf30"]["field_oracle"] = rf30_oracle
    report["rf30"]["diagnosis"] = (
        "MUTUAL_ARRIVAL_CONFIRMED"
        if rf30_oracle["all_suspect_curves_have_two_material_owners"]
        else "RF30_ONE_SIDED_COMPETITION_FOUND"
    )
    report["green"] = all(
        record["preview_confirm_equal"]
        and all(
            evaluation.get("status") == "OK"
            for evaluation in record["evaluations"].values()
        )
        for case in (report["rf30"], report["rf29b"])
        for record in case["widths"]
    ) and rf30_oracle["all_suspect_curves_have_two_material_owners"]
    OUTPUT_JSON.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("CFTUV_R21_HOTFIX=" + json.dumps(report, ensure_ascii=False))
    if not report["green"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
