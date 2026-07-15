from __future__ import annotations

import pytest
from mathutils import Vector

pytest.importorskip("pyvoronoi")

import cftuv.decal_voronoi as decal_voronoi  # noqa: E402
from cftuv.decal_voronoi import (  # noqa: E402
    _merge_polygon_fragments,
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


def _signed_face_area(face):
    area_vector = Vector((0.0, 0.0, 0.0))
    origin = face.positions[0]
    for index in range(1, len(face.positions) - 1):
        area_vector += (face.positions[index] - origin).cross(
            face.positions[index + 1] - origin
        )
    return area_vector.dot(face.surface_normal) * 0.5


def _orthogonal_corner_graph():
    graph = PatchGraph()
    patch_specs = (
        (
            0,
            Vector((0.0, 0.0, 1.0)),
            Vector((1.0, 0.0, 0.0)),
            Vector((0.0, 1.0, 0.0)),
            [
                Vector((0.0, 0.0, 0.0)),
                Vector((2.0, 0.0, 0.0)),
                Vector((2.0, 1.0, 0.0)),
                Vector((0.0, 1.0, 0.0)),
            ],
        ),
        (
            1,
            Vector((0.0, -1.0, 0.0)),
            Vector((1.0, 0.0, 0.0)),
            Vector((0.0, 0.0, 1.0)),
            [
                Vector((0.0, 0.0, 0.0)),
                Vector((2.0, 0.0, 0.0)),
                Vector((2.0, 0.0, 1.0)),
                Vector((0.0, 0.0, 1.0)),
            ],
        ),
    )
    for patch_id, normal, basis_u, basis_v, verts in patch_specs:
        node = PatchNode(
            patch_id=patch_id,
            face_indices=[patch_id],
            centroid=sum(verts, Vector((0.0, 0.0, 0.0))) / 4.0,
            normal=normal,
            basis_u=basis_u,
            basis_v=basis_v,
        )
        node.mesh_verts = verts
        node.mesh_tris = [(0, 1, 2), (0, 2, 3)]
        node.boundary_loops = [
            BoundaryLoop(
                chains=[
                    BoundaryChain(
                        vert_indices=[0, 1],
                        vert_cos=[verts[0], verts[1]],
                        edge_indices=[30],
                    )
                ]
            )
        ]
        graph.add_node(node)
    return graph


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

    # Две source triangles остаются вычислительной деталью: каждая
    # Voronoi-cell материализуется одним цельным quad, а не двумя tris.
    assert len(wide) == 2
    assert all(len(face.positions) == 4 for face in wide)


def test_patch_voronoi_fragment_union_removes_internal_triangulation():
    components = _merge_polygon_fragments(
        [
            [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)],
            [(0.0, 0.0), (2.0, 2.0), (0.0, 2.0)],
        ]
    )
    assert len(components) == 1
    assert len(components[0]) == 4
    assert abs(sum(
        point[0] * components[0][(index + 1) % 4][1]
        - components[0][(index + 1) % 4][0] * point[1]
        for index, point in enumerate(components[0])
    )) * 0.5 == pytest.approx(4.0)


def test_fragment_union_keeps_point_contact_as_separate_components():
    components = _merge_polygon_fragments(
        [
            [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)],
            [(0.0, 0.0), (-1.0, 0.0), (0.0, -1.0)],
        ]
    )
    assert len(components) == 2


def test_patch_domain_uses_boundary_loop_instead_of_owner_triangulation(
    monkeypatch,
):
    node = PatchNode(
        patch_id=0,
        face_indices=[0],
        centroid=Vector((1.0, 1.0, 0.0)),
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
    )
    node.mesh_verts = [
        Vector((0.0, 0.0, 0.0)),
        Vector((2.0, 0.0, 0.0)),
        Vector((2.0, 2.0, 0.0)),
        Vector((0.0, 2.0, 0.0)),
    ]
    # Намеренно неполная owner topology: старый backend потерял бы половину
    # patch. Boundary-domain должен полностью её игнорировать.
    node.mesh_tris = [(0, 1, 2)]
    node.boundary_loops = [
        BoundaryLoop(vert_cos=[point.copy() for point in node.mesh_verts])
    ]
    monkeypatch.setattr(
        decal_voronoi,
        "_tessellate_polygon",
        lambda _loops: [(0, 1, 2), (0, 2, 3)],
    )

    triangles = decal_voronoi._patch_domain_triangles(
        node,
        node.centroid,
        node.basis_u,
        node.basis_v,
    )
    assert len(triangles) == 2
    assert sum(
        abs(decal_voronoi._polygon_area2(tri))
        for tri in triangles
    ) == pytest.approx(4.0)


def test_patch_voronoi_rejects_non_planar_owner_patch():
    graph = _planar_two_site_graph()
    graph.nodes[0].mesh_verts[2] = Vector((4.0, 2.0, 0.2))
    assert compile_patch_voronoi_plan(graph, [10, 12], offset=0.01) is None


def test_preview_and_confirm_keep_identical_miter_geometry():
    graph = _planar_two_site_graph()
    graph.nodes[0].boundary_loops = [
        BoundaryLoop(
            chains=[
                BoundaryChain(
                    vert_indices=[4, 5],
                    vert_cos=[
                        Vector((1.0, 0.0, 0.0)),
                        Vector((3.0, 0.0, 0.0)),
                    ],
                    edge_indices=[20],
                )
            ]
        )
    ]
    plan = compile_patch_voronoi_plan(graph, [20], offset=0.01)
    preview = evaluate_patch_voronoi_plan(plan, width=1.0, preview=True)
    confirmed = evaluate_patch_voronoi_plan(plan, width=1.0, preview=False)

    def signature(faces):
        return [
            (
                tuple(face.vert_keys),
                tuple(
                    tuple(round(value, 9) for value in point)
                    for point in face.positions
                ),
            )
            for face in faces
        ]

    assert signature(preview) == signature(confirmed)


def test_narrow_corner_offset_cannot_invert_owner_wings():
    plan = compile_patch_voronoi_plan(
        _orthogonal_corner_graph(), [30], offset=0.2
    )
    faces = evaluate_patch_voronoi_plan(plan, width=0.01, preview=True)
    assert faces
    assert min(_signed_face_area(face) for face in faces) >= -1e-12
