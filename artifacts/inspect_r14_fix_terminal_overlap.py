"""RF19-fix: ищет положительную площадь наложения emitted decal faces."""

from __future__ import annotations

import json
import sys
from dataclasses import replace
from pathlib import Path

import bpy
from mathutils import Vector
from mathutils.geometry import tessellate_polygon


REPO_ROOT = Path(__file__).resolve().parents[1]
ARTIFACT_ROOT = REPO_ROOT / "artifacts"
for path in (REPO_ROOT, ARTIFACT_ROOT):
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

from cftuv import decal_voronoi  # noqa: E402
from cftuv.constants import DECAL_WELD_DISTANCE  # noqa: E402
from cftuv.decals import (  # noqa: E402
    compile_manual_seam_decal_plan,
    evaluate_manual_seam_faces,
)
from cftuv.model import DecalSettings  # noqa: E402


FIELD_CASES = {
    "rounded_wall.001": (55, 59, 62, 65, 68, 71, 74, 77, 80, 83),
    "sagging_wall": (3, 4, 5, 6, 7, 8, 9, 11),
}


def _face_triangles(face):
    polygon = [Vector(point) for point in face.positions]
    triangles = tessellate_polygon([polygon])
    if triangles and isinstance(triangles[0][0], int):
        return tuple(
            tuple(polygon[index] for index in triangle)
            for triangle in triangles
        )
    return tuple(
        tuple(point.copy() for point in triangle) for triangle in triangles
    )


def _triangle_overlap(first, second):
    normal = (first[1] - first[0]).cross(first[2] - first[0])
    if normal.length_squared == 0.0:
        return 0.0
    normal.normalize()
    other_normal = (second[1] - second[0]).cross(second[2] - second[0])
    if other_normal.length_squared == 0.0:
        return 0.0
    other_normal.normalize()
    if abs(normal.dot(other_normal)) < 1.0 - 1e-8:
        return 0.0
    if max(abs((point - first[0]).dot(normal)) for point in second) > DECAL_WELD_DISTANCE:
        return 0.0
    basis_u = (first[1] - first[0]).normalized()
    basis_v = normal.cross(basis_u).normalized()

    def project(point):
        delta = point - first[0]
        return (delta.dot(basis_u), delta.dot(basis_v))

    overlap = decal_voronoi._clip_to_convex(
        tuple(project(point) for point in first),
        tuple(project(point) for point in second),
    )
    return abs(decal_voronoi._polygon_area2(overlap)) if overlap else 0.0


def _overlap_records(faces):
    triangles = tuple(_face_triangles(face) for face in faces)
    area_floor = DECAL_WELD_DISTANCE * DECAL_WELD_DISTANCE
    records = []
    for first_index, first in enumerate(faces):
        for second_index in range(first_index + 1, len(faces)):
            second = faces[second_index]
            overlap = sum(
                _triangle_overlap(first_triangle, second_triangle)
                for first_triangle in triangles[first_index]
                for second_triangle in triangles[second_index]
            )
            if overlap <= area_floor:
                continue
            records.append(
                {
                    "faces": [first_index, second_index],
                    "surface_ids": [first.surface_id, second.surface_id],
                    "component_kinds": [
                        first.component_kind,
                        second.component_kind,
                    ],
                    "component_sides": [
                        first.component_side,
                        second.component_side,
                    ],
                    "overlap_area": overlap,
                }
            )
    return records


def main():
    payload = {"schema": "cftuv.decal_r14_fix.overlap.v1", "objects": []}
    for object_name, selected in FIELD_CASES.items():
        obj = bpy.data.objects[object_name]
        graph, selected, _all_seams = field._source_graph(obj, selected)
        settings = DecalSettings(width_seam=0.8, offset=0.01)
        plan = compile_manual_seam_decal_plan(graph, settings, selected)
        widths = []
        for width in (0.4, 0.8):
            evaluated = evaluate_manual_seam_faces(
                obj,
                replace(settings, width_seam=width),
                plan,
                preview=True,
            )
            overlaps = _overlap_records(evaluated.faces)
            terminal_cut_faces = [
                face
                for face in evaluated.faces
                if face.component_kind == "SEGMENT"
                and face.component_side.startswith("TERMINAL_")
            ]
            widths.append(
                {
                    "width": width,
                    "face_count": len(evaluated.faces),
                    "overlap_count": len(overlaps),
                    "overlaps": overlaps,
                    "terminal_cut_face_count": len(terminal_cut_faces),
                    "terminal_cut_faces": [
                        {
                            "surface_id": face.surface_id,
                            "component_side": face.component_side,
                            "positions": [list(point) for point in face.positions],
                            "u_fracs": list(face.u_fracs),
                            "v_lengths": list(face.v_lengths),
                        }
                        for face in terminal_cut_faces
                    ],
                }
            )
        payload["objects"].append(
            {
                "object": object_name,
                "selected_edge_indices": list(selected),
                "compile_plan_identity": id(plan.rail_plan),
                "widths": widths,
            }
        )
    output = ARTIFACT_ROOT / "decal_r14_fix_terminal_overlap.json"
    output.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    summary = {
        item["object"]: [
            {
                "width": width["width"],
                "faces": width["face_count"],
                "overlaps": width["overlap_count"],
                "terminal_cut_faces": width["terminal_cut_face_count"],
            }
            for width in item["widths"]
        ]
        for item in payload["objects"]
    }
    print("CFTUV_R14_FIX_OVERLAP=" + json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
