"""R1.6: provenance corner clamp и CAP на пользовательском sagging_wall.

Скрипт предназначен только для headless-чтения ``E:\\testscene.blend`` и
никогда не сохраняет сцену.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import bpy


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

from cftuv.decal_rail_geometry import (  # noqa: E402
    evaluate_planar_rail_geometry_plan,
)
from cftuv.decal_voronoi import (  # noqa: E402
    PatchVoronoiDiagnostics,
    _corner_offset_edge_relation,
    _classify_surface_corner_runtime,
    corner_runtime_settings_from_decal_settings,
    evaluate_patch_voronoi_plan,
)
from cftuv.decals import (  # noqa: E402
    compile_manual_seam_decal_plan,
    evaluate_manual_seam_faces,
)
from cftuv.model import DecalSettings  # noqa: E402


OBJECT_NAME = "sagging_wall"
SELECTED = (6, 7, 8)
WIDTHS = (0.8, 2.0, 3.2, 4.0)
OUTPUT_JSON = ARTIFACT_ROOT / "decal_r16_field_diagnostic.json"


def _path_reach(path):
    return max(
        (
            float(vertex.r)
            for piece in path.pieces
            for vertex in (piece.start, piece.end)
        ),
        default=0.0,
    )


def _key(value):
    if isinstance(value, tuple):
        return [_key(item) for item in value]
    return value


def _face_record(face):
    provenance = getattr(face, "rail_provenance", None)
    return {
        "kind": str(face.component_kind),
        "side": str(face.component_side),
        "surface_id": int(face.surface_id),
        "corner_partition_id": (
            None
            if provenance is None
            else provenance.corner_partition_id
        ),
        "channel_id": None if provenance is None else provenance.channel_id,
        "keys": [_key(key) for key in face.vert_keys],
        "positions": [list(map(float, point)) for point in face.positions],
        "u_fracs": list(map(float, face.u_fracs)),
        "v_lengths": list(map(float, face.v_lengths)),
    }


def _source_topology(obj):
    return {
        "vertices": {
            str(vertex.index): list(map(float, vertex.co))
            for vertex in obj.data.vertices
        },
        "edges": {
            str(edge.index): {
                "vertices": list(edge.vertices),
                "seam": bool(edge.use_seam),
                "faces": [
                    polygon.index
                    for polygon in obj.data.polygons
                    if tuple(sorted(edge.vertices)) in polygon.edge_keys
                ],
            }
            for edge in obj.data.edges
        },
        "faces": {
            str(polygon.index): {
                "vertices": list(polygon.vertices),
                "edges": list(polygon.edge_keys),
            }
            for polygon in obj.data.polygons
        },
    }


def main():
    obj = bpy.data.objects[OBJECT_NAME]
    graph, selected, _all_seams = field._source_graph(obj, SELECTED)
    settings = DecalSettings(width_seam=0.8, offset=0.01)
    plan = compile_manual_seam_decal_plan(
        graph,
        settings,
        selected,
        alpha_budget=4.0,
    )
    payload = {
        "object": OBJECT_NAME,
        "selected": list(selected),
        "backend_summary": str(plan.backend_summary),
        "terminal_routing": list(plan.terminal_routing_report),
        "rail_terminal_uses": [],
        "source_topology": _source_topology(obj),
        "partitions": [],
        "evaluations": [],
        "bevel_evaluations": [],
        "bevel_corner_table": [],
    }
    if plan.rail_plan is not None:
        edge_by_id = {edge.edge_id: edge for edge in plan.rail_plan.edges}
        route_by_id = {route.route_id: route for route in plan.rail_plan.routes}
        payload["rail_terminal_uses"] = [
            {
                "spine_vertex_id": terminal.spine_vertex_id,
                "spine_edge_id": terminal.spine_edge_id,
                "source_face_ids": list(terminal.source_face_ids),
                "choice": str(terminal.kind.value),
                "candidate_edge_ids": list(terminal.candidate_edge_ids),
                "candidate_facts": [
                    {
                        "edge_id": edge_id,
                        "vertices": list(edge_by_id[edge_id].vertex_ids),
                        "faces": list(edge_by_id[edge_id].face_indices),
                        "is_pchain": bool(edge_by_id[edge_id].is_pchain),
                        "is_fold": bool(edge_by_id[edge_id].is_fold),
                        "is_border": len(edge_by_id[edge_id].face_indices) == 1,
                    }
                    for edge_id in terminal.candidate_edge_ids
                ],
                "incident_facts": [
                    {
                        "edge_id": edge.edge_id,
                        "vertices": list(edge.vertex_ids),
                        "faces": list(edge.face_indices),
                        "is_pchain": bool(edge.is_pchain),
                        "is_spine": bool(edge.is_spine),
                        "is_fold": bool(edge.is_fold),
                        "is_border": len(edge.face_indices) == 1,
                    }
                    for edge in plan.rail_plan.edges
                    if terminal.spine_vertex_id in edge.vertex_ids
                ],
                "route_id": terminal.route_id,
                "route_edge_id": terminal.route_edge_id,
                "route_edges": (
                    []
                    if terminal.route_id is None
                    else [
                        segment.edge_id
                        for segment in route_by_id[terminal.route_id].segments
                    ]
                ),
                "route_distances": (
                    []
                    if terminal.route_id is None
                    else [
                        float(station.distance)
                        for station in route_by_id[terminal.route_id].stations
                    ]
                ),
            }
            for terminal in plan.rail_plan.terminal_uses
            if terminal.spine_edge_id in selected
        ]
    for backend in plan.backend_partitions:
        record = {
            "backend": str(backend.backend),
            "edges": list(backend.edge_indices),
        }
        geometry = backend.compiled_plan
        if backend.backend == "RAIL_PLANAR":
            paths = {path.path_id: path for path in geometry.boundary_paths}
            record["channels"] = [
                {
                    "channel_id": channel.channel_id,
                    "spine_edge_id": channel.spine_edge_id,
                    "side_sign": channel.side_sign,
                    "from_path_id": _key(channel.from_path_id),
                    "to_path_id": _key(channel.to_path_id),
                    "from_reach": _path_reach(paths[channel.from_path_id]),
                    "to_reach": _path_reach(paths[channel.to_path_id]),
                    "alpha_limit": float(channel.alpha_limit),
                }
                for channel in geometry.channels
            ]
            record["corner_partitions"] = [
                {
                    "partition_id": corner.partition_id,
                    "source_vertex_id": corner.source_vertex_id,
                    "side_sign": corner.side_sign,
                    "mode": corner.mode,
                    "previous_channel_id": corner.previous_channel_id,
                    "next_channel_id": corner.next_channel_id,
                    "previous_path_id": _key(corner.previous_path_id),
                    "next_path_id": _key(corner.next_path_id),
                    "previous_reach": _path_reach(paths[corner.previous_path_id]),
                    "next_reach": _path_reach(paths[corner.next_path_id]),
                    "alpha_limit": float(corner.alpha_limit),
                    "owner_face_ids": list(corner.owner_face_ids),
                }
                for corner in geometry.corner_partitions
            ]
            record["corner_cells"] = [
                {
                    "cell_id": cell.cell_id,
                    "partition_id": cell.partition_id,
                    "owner_face_id": cell.owner_face_id,
                    "boundary_path_ids": [_key(value) for value in cell.boundary_path_ids],
                    "vertices": [
                        {
                            "key": _key(vertex.key),
                            "r": float(vertex.r),
                            "path": _key(vertex.boundary_path_id),
                        }
                        for vertex in cell.vertices
                    ],
                }
                for cell in geometry.corner_cells
            ]
        payload["partitions"].append(record)

    for width in WIDTHS:
        width_faces = []
        cap_keep_counts = {}
        for backend in plan.backend_partitions:
            if backend.backend == "RAIL_PLANAR":
                width_faces.extend(
                    _face_record(face)
                    for face in evaluate_planar_rail_geometry_plan(
                        backend.compiled_plan,
                        width=width,
                        offset=settings.offset,
                        preview=True,
                    )
                )
            elif backend.backend == "PATCH_VORONOI":
                diagnostics = PatchVoronoiDiagnostics()
                evaluate_patch_voronoi_plan(
                    backend.compiled_plan,
                    width=width,
                    preview=True,
                    corner_settings=corner_runtime_settings_from_decal_settings(
                        settings
                    ),
                    diagnostics=diagnostics,
                    terminal_routing=plan.terminal_routing,
                    rail_plan=plan.rail_plan,
                )
                cap_keep_counts.update(diagnostics.cap_keep_counts)
        evaluated = evaluate_manual_seam_faces(
            obj,
            DecalSettings(width_seam=width, offset=settings.offset),
            plan,
            preview=True,
        )
        payload["evaluations"].append(
            {
                "width": width,
                "rail_faces": width_faces,
                "cap_keep_counts": cap_keep_counts,
                "all_face_count": len(evaluated.faces),
                "all_faces": [_face_record(face) for face in evaluated.faces],
                "component_kinds": {
                    kind: sum(
                        1
                        for face in evaluated.faces
                        if str(face.component_kind) == kind
                    )
                    for kind in sorted(
                        {str(face.component_kind) for face in evaluated.faces}
                    )
                },
            }
        )

    bevel_settings = DecalSettings(
        width_seam=0.8,
        offset=0.01,
        corner_join_mode="BEVEL",
    )
    bevel_plan = compile_manual_seam_decal_plan(
        graph,
        bevel_settings,
        selected,
        alpha_budget=4.0,
    )
    base_runtime = corner_runtime_settings_from_decal_settings(settings)
    bevel_runtime = corner_runtime_settings_from_decal_settings(bevel_settings)
    for backend in bevel_plan.backend_partitions:
        if backend.backend != "PATCH_VORONOI":
            continue
        for surface_id, surface in enumerate(backend.compiled_plan.surfaces):
            for corner in surface.corners:
                payload["bevel_corner_table"].append(
                    {
                        "surface_id": int(surface_id),
                        "patch_id": int(surface.patch_id),
                        "chart_id": int(surface.domain.chart_id),
                        "vertex_id": int(corner.vert_index),
                        "source_position": list(
                            map(float, obj.data.vertices[corner.vert_index].co)
                        ),
                        "chart_point": list(map(float, corner.point)),
                        "incident_edges": [
                            int(surface.sites[index].edge_index)
                            for index in corner.incident_sites
                        ],
                        "site_uv_signs": [
                            float(surface.sites[index].uv_sign)
                            for index in corner.incident_sites
                        ],
                        "site_arc_signs": [
                            float(surface.sites[index].arc_sign)
                            for index in corner.incident_sites
                        ],
                        "site_inward_normals": [
                            list(map(float, surface.sites[index].inward_normal))
                            for index in corner.incident_sites
                        ],
                        "turn_sign": float(corner.turn_sign),
                        "interior_angle": float(corner.interior_angle),
                        "extrusion_angle": float(corner.extrusion_angle),
                        "is_convex": bool(corner.is_convex),
                        "join_relation": (
                            _corner_offset_edge_relation(surface, corner)
                            if len(corner.incident_sites) == 2
                            else "NONE"
                        ),
                        "base_policy": _classify_surface_corner_runtime(
                            surface, corner, base_runtime
                        ).value,
                        "bevel_policy": _classify_surface_corner_runtime(
                            surface, corner, bevel_runtime
                        ).value,
                    }
                )
    for width in (0.8, 3.2):
        evaluated = evaluate_manual_seam_faces(
            obj,
            DecalSettings(
                width_seam=width,
                offset=bevel_settings.offset,
                corner_join_mode="BEVEL",
            ),
            bevel_plan,
            preview=True,
        )
        payload["bevel_evaluations"].append(
            {
                "width": width,
                "backend_summary": str(bevel_plan.backend_summary),
                "component_kinds": {
                    kind: sum(
                        1
                        for face in evaluated.faces
                        if str(face.component_kind) == kind
                    )
                    for kind in sorted(
                        {str(face.component_kind) for face in evaluated.faces}
                    )
                },
                "bevel_face_sizes": [
                    len(face.positions)
                    for face in evaluated.faces
                    if str(face.component_kind) == "BEVEL"
                ],
            }
        )

    OUTPUT_JSON.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print("CFTUV_R16_FIELD=" + json.dumps(payload, ensure_ascii=False))


if __name__ == "__main__":
    main()
