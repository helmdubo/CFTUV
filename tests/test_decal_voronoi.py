from __future__ import annotations

import pytest
from mathutils import Vector

pytest.importorskip("pyvoronoi")

from cftuv.decal_voronoi import (  # noqa: E402
    compile_patch_voronoi_plan,
    evaluate_patch_voronoi_plan,
)
from cftuv.model import (  # noqa: E402
    BoundaryChain,
    BoundaryLoop,
    PatchGraph,
    PatchNode,
)


def _planar_two_site_graph():
    node = PatchNode(
        patch_id=0,
        face_indices=[0, 1],
        centroid=Vector((2.0, 1.0, 0.0)),
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
    )
    node.mesh_verts = [
        Vector((0.0, 0.0, 0.0)),
        Vector((4.0, 0.0, 0.0)),
        Vector((4.0, 2.0, 0.0)),
        Vector((0.0, 2.0, 0.0)),
    ]
    node.mesh_tris = [(0, 1, 2), (0, 2, 3)]
    left = BoundaryChain(
        vert_indices=[0, 3],
        vert_cos=[node.mesh_verts[0], node.mesh_verts[3]],
        edge_indices=[10],
    )
    right = BoundaryChain(
        vert_indices=[1, 2],
        vert_cos=[node.mesh_verts[1], node.mesh_verts[2]],
        edge_indices=[12],
    )
    node.boundary_loops = [BoundaryLoop(chains=[left, right])]
    graph = PatchGraph()
    graph.add_node(node)
    return graph


def _face_area_xy(face):
    area = 0.0
    points = face.positions
    for index, point in enumerate(points):
        other = points[(index + 1) % len(points)]
        area += point.x * other.y - other.x * point.y
    return abs(area) * 0.5


def test_patch_voronoi_front_rebuilds_and_never_leaves_patch():
    plan = compile_patch_voronoi_plan(
        _planar_two_site_graph(), [10, 12], offset=0.01
    )
    assert plan is not None
    assert len(plan.surfaces) == 1
    assert len(plan.surfaces[0].sites) == 2

    narrow = evaluate_patch_voronoi_plan(plan, width=0.5, preview=True)
    wide = evaluate_patch_voronoi_plan(plan, width=4.0, preview=True)
    assert narrow and wide
    assert sum(_face_area_xy(face) for face in narrow) == pytest.approx(
        1.0, abs=1e-5
    )
    assert sum(_face_area_xy(face) for face in wide) == pytest.approx(
        8.0, abs=1e-5
    )
    for face in narrow + wide:
        for point in face.positions:
            assert -1e-7 <= point.x <= 4.0 + 1e-7
            assert -1e-7 <= point.y <= 2.0 + 1e-7
            assert point.z == pytest.approx(0.01)


def test_patch_voronoi_rejects_non_planar_owner_patch():
    graph = _planar_two_site_graph()
    graph.nodes[0].mesh_verts[2] = Vector((4.0, 2.0, 0.2))
    assert compile_patch_voronoi_plan(graph, [10, 12], offset=0.01) is None
