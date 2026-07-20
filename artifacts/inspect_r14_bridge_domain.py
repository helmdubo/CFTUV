"""Одноразовый факт-чек: rail guide против production CAP chart domain."""

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

from cftuv.analysis import build_patch_graph  # noqa: E402
from cftuv.decals import (  # noqa: E402
    compile_manual_seam_decal_plan,
    evaluate_manual_seam_faces,
)
from cftuv.model import DecalSettings  # noqa: E402


def _source_vertex_points(domain, vertex_id):
    if domain.kind == "PLANAR":
        return tuple(
            point
            for source_vertex_id, point in domain.planar_source_vertices
            if source_vertex_id == vertex_id
        )
    return tuple(
        triangle.chart_points[local_index]
        for triangle in domain.intrinsic_triangles
        for local_index, source_vertex_id in enumerate(
            triangle.source_vertex_ids
        )
        if source_vertex_id == vertex_id
    )


def main():
    obj = bpy.data.objects["rounded_wall.001"]
    selected = (55, 59, 62, 65, 68, 71, 74, 77, 80, 83)
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    graph = build_patch_graph(bm, [face.index for face in bm.faces], obj)
    bm.free()
    plan = compile_manual_seam_decal_plan(
        graph,
        DecalSettings(width_seam=0.8, offset=0.01),
        selected,
    )
    patch_plan = next(
        partition.compiled_plan
        for partition in plan.backend_partitions
        if partition.backend == "PATCH_VORONOI"
    )
    route_by_id = {
        route.route_id: route for route in plan.rail_plan.routes
    }
    edge_by_id = {edge.edge_id: edge for edge in plan.rail_plan.edges}
    records = []
    for use in plan.rail_plan.terminal_uses:
        if use.spine_edge_id not in selected or use.route_id is None:
            continue
        route = route_by_id[use.route_id]
        guide_edge = edge_by_id[use.route_edge_id]
        patch_ids = sorted(
            {
                graph.face_to_patch[face_id]
                for face_id in use.source_face_ids
                if face_id in graph.face_to_patch
            }
        )
        surfaces = []
        for surface in patch_plan.surfaces:
            if surface.patch_id not in patch_ids:
                continue
            cap_sites = [
                {
                    "site_index": site_index,
                    "edge_id": site.edge_index,
                    "point_a": site.point_a,
                    "point_b": site.point_b,
                }
                for site_index, site in enumerate(surface.sites)
                if site.edge_index == use.spine_edge_id
                and use.spine_vertex_id in (site.vert_a, site.vert_b)
            ]
            surfaces.append(
                {
                    "patch_id": surface.patch_id,
                    "chart_id": surface.domain.chart_id,
                    "kind": surface.domain.kind,
                    "source_faces": sorted(
                        {
                            triangle.source_face_id
                            for triangle in surface.domain.intrinsic_triangles
                            if triangle.source_face_id is not None
                        }
                    ),
                    "cap_sites": cap_sites,
                    "terminal_vertex_points": _source_vertex_points(
                        surface.domain, use.spine_vertex_id
                    ),
                    "guide_vertex_points": {
                        str(vertex_id): _source_vertex_points(
                            surface.domain, vertex_id
                        )
                        for vertex_id in guide_edge.vertex_ids
                    },
                }
            )
        records.append(
            {
                "terminal": [use.spine_vertex_id, use.spine_edge_id],
                "source_face_ids": use.source_face_ids,
                "patch_ids": patch_ids,
                "guide_edge_id": use.route_edge_id,
                "guide_vertex_ids": guide_edge.vertex_ids,
                "route_stations": [
                    {
                        "distance": station.distance,
                        "kind": station.kind.value,
                        "source_vertex_id": station.source_vertex_id,
                        "source_edge_id": station.source_edge_id,
                        "edge_parameter": station.edge_parameter,
                    }
                    for station in route.stations
                ],
                "route_segments": [
                    {
                        "edge_id": segment.edge_id,
                        "source_face_ids": segment.source_face_ids,
                    }
                    for segment in route.segments
                ],
                "surfaces": surfaces,
            }
        )
    evaluations = []
    for width in (0.4, 0.8):
        evaluated = evaluate_manual_seam_faces(
            obj,
            replace(
                DecalSettings(width_seam=0.8, offset=0.01),
                width_seam=width,
            ),
            plan,
            preview=True,
        )
        evaluations.append(
            {"width": width, "face_count": len(evaluated.faces)}
        )
    sagging = bpy.data.objects["sagging_wall"]
    sagging_bm = bmesh.new()
    sagging_bm.from_mesh(sagging.data)
    sagging_bm.verts.ensure_lookup_table()
    sagging_bm.edges.ensure_lookup_table()
    sagging_bm.faces.ensure_lookup_table()
    sagging_graph = build_patch_graph(
        sagging_bm,
        [face.index for face in sagging_bm.faces],
        sagging,
    )
    sagging_seams = tuple(
        edge.index for edge in sagging_bm.edges if edge.seam
    )
    sagging_saved_selection = tuple(
        edge.index for edge in sagging_bm.edges if edge.select
    )
    sagging_bm.free()
    sagging_single_edge_routes = []
    for edge_id in sagging_seams:
        single = compile_manual_seam_decal_plan(
            sagging_graph,
            DecalSettings(width_seam=0.8, offset=0.01),
            (edge_id,),
        )
        if single.terminal_routing:
            sagging_single_edge_routes.append(
                {
                    "selected_edge_id": edge_id,
                    "backend_summary": single.backend_summary,
                    "routing": list(single.terminal_routing_report),
                }
            )
    sagging_saved_plan = compile_manual_seam_decal_plan(
        sagging_graph,
        DecalSettings(width_seam=0.8, offset=0.01),
        sagging_saved_selection,
    )
    sagging_saved_evaluations = []
    for width in (0.4, 0.8):
        evaluated = evaluate_manual_seam_faces(
            sagging,
            DecalSettings(width_seam=width, offset=0.01),
            sagging_saved_plan,
            preview=True,
        )
        sagging_saved_evaluations.append(
            {"width": width, "face_count": len(evaluated.faces)}
        )
    payload = {
        "terminals": records,
        "evaluations": evaluations,
        "sagging_saved_selection": sagging_saved_selection,
        "sagging_saved_backend": sagging_saved_plan.backend_summary,
        "sagging_saved_terminal_routing": list(
            sagging_saved_plan.terminal_routing_report
        ),
        "sagging_saved_evaluations": sagging_saved_evaluations,
        "sagging_single_edge_routes": sagging_single_edge_routes,
    }
    output = REPO_ROOT / "artifacts" / "decal_r14_bridge_domain.json"
    output.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print("CFTUV_R14_BRIDGE_DOMAIN=" + json.dumps(payload, ensure_ascii=False))


if __name__ == "__main__":
    main()
