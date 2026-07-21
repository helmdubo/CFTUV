"""Blender 4.x field gate for TRANCHE S1 PatchSurfaceIR.

Run:
  blender --background --factory-startup --python \
    artifacts/verify_decal_s1_surface_ir_blender.py -- <output.json>
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import bmesh
import bpy
from mathutils import Vector


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from cftuv.analysis_surface import (  # noqa: E402
    build_patch_surface_ir,
    source_revision_from_bmesh,
    validate_analysis_bundle,
)
from cftuv.model import BoundaryChain, BoundaryLoop, PatchGraph, PatchNode  # noqa: E402
from cftuv.surface_ir import AnalysisBundle  # noqa: E402


def _point_in_polygon_xy(point, polygon):
    inside = False
    for first, second in zip(polygon, polygon[1:] + polygon[:1]):
        if (first[1] > point[1]) == (second[1] > point[1]):
            continue
        crossing_x = first[0] + (
            (point[1] - first[1])
            * (second[0] - first[0])
            / (second[1] - first[1])
        )
        if crossing_x >= point[0]:
            inside = not inside
    return inside


def _build_case(name, vertices, face_cycle):
    mesh = bpy.data.meshes.new(name + "Mesh")
    mesh.from_pydata(vertices, (), (face_cycle,))
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)

    bm = bmesh.new()
    bm.from_mesh(mesh)
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.normal_update()
    face = bm.faces[0]

    revision = source_revision_from_bmesh(bm, obj, (0,))
    graph = PatchGraph(source_revision=revision)
    node = PatchNode(patch_id=0, face_indices=[0])
    vertex_cycle = [int(loop.vert.index) for loop in face.loops]
    edge_cycle = [int(loop.edge.index) for loop in face.loops]
    coordinates = [loop.vert.co.copy() for loop in face.loops]
    node.boundary_loops = [
        BoundaryLoop(
            vert_indices=vertex_cycle,
            vert_cos=coordinates,
            edge_indices=edge_cycle,
            chains=[
                BoundaryChain(
                    vert_indices=vertex_cycle + [vertex_cycle[0]],
                    vert_cos=coordinates + [coordinates[0].copy()],
                    edge_indices=edge_cycle,
                    side_face_indices=[0] * len(edge_cycle),
                    side_face_normals=[face.normal.copy()] * len(edge_cycle),
                )
            ],
        )
    ]
    graph.add_node(node)
    surface = build_patch_surface_ir(bm, graph, revision)
    validate_analysis_bundle(AnalysisBundle(revision, graph, surface))

    blender_triangles = tuple(
        tuple(int(loop.vert.index) for loop in triangle)
        for triangle in bm.calc_loop_triangles()
    )
    emitted_triangles = tuple(
        triangle.vertex_ids for triangle in surface.triangles
    )
    assert emitted_triangles == blender_triangles

    points = {int(vertex.index): vertex.co.copy() for vertex in bm.verts}
    area = sum(
        (
            points[triangle[1]] - points[triangle[0]]
        ).cross(
            points[triangle[2]] - points[triangle[0]]
        ).length
        * 0.5
        for triangle in emitted_triangles
    )
    normals = tuple(triangle.triangle_normal for triangle in surface.triangles)
    result = {
        "name": name,
        "vertex_cycle": vertex_cycle,
        "triangle_vertex_ids": emitted_triangles,
        "triangle_area_3d": area,
        "triangle_normals": normals,
        "all_tessellation_diagonals_are_none": all(
            edge_id is None
            for triangle in surface.triangles
            for edge_id in triangle.physical_edge_ids
            if edge_id not in edge_cycle
        ),
    }
    bm.free()
    bpy.data.objects.remove(obj)
    bpy.data.meshes.remove(mesh)
    return result


def main():
    u_vertices = (
        (0.0, 0.0, 0.0),
        (3.0, 0.0, 0.0),
        (3.0, 3.0, 0.0),
        (2.0, 3.0, 0.0),
        (2.0, 1.0, 0.0),
        (1.0, 1.0, 0.0),
        (1.0, 3.0, 0.0),
        (0.0, 3.0, 0.0),
    )
    u_case = _build_case("concave_u_ngon", u_vertices, tuple(range(8)))
    reversed_case = _build_case(
        "reversed_concave_u_ngon",
        u_vertices,
        tuple(reversed(range(8))),
    )
    non_planar_case = _build_case(
        "non_planar_quad",
        (
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 0.4),
            (0.0, 1.0, 0.0),
        ),
        (0, 1, 2, 3),
    )

    u_polygon = tuple(u_vertices[index] for index in u_case["vertex_cycle"])
    u_case["all_triangle_centroids_inside_polygon"] = all(
        _point_in_polygon_xy(
            tuple(
                sum(u_vertices[vertex_id][axis] for vertex_id in triangle) / 3.0
                for axis in range(2)
            ),
            u_polygon,
        )
        for triangle in u_case["triangle_vertex_ids"]
    )
    assert abs(u_case["triangle_area_3d"] - 7.0) < 1e-9
    assert u_case["all_triangle_centroids_inside_polygon"]
    assert abs(reversed_case["triangle_area_3d"] - 7.0) < 1e-9
    assert all(normal[2] > 0.0 for normal in u_case["triangle_normals"])
    assert all(normal[2] < 0.0 for normal in reversed_case["triangle_normals"])
    assert len(set(non_planar_case["triangle_normals"])) == 2

    report = {
        "schema": "cftuv.decal_s1_surface_ir_blender.v1",
        "blender_version": bpy.app.version_string,
        "verdict": "PASS",
        "cases": (u_case, non_planar_case, reversed_case),
    }
    args = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    output = Path(args[0]) if args else REPO_ROOT / "artifacts" / "decal_s1_surface_ir_blender.json"
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print("CFTUV_S1_SURFACE_IR_PASS", output)


if __name__ == "__main__":
    main()
