from __future__ import annotations

import pytest
from math import cos, pi, sin
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


def _folded_turn_graph():
    """Два selected edges поворачивают через три owner patches."""

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
                Vector((2.0, 2.0, 0.0)),
                Vector((0.0, 2.0, 0.0)),
            ],
            (
                ([0, 1], [0, 1], [30]),
                ([0, 3], [0, 3], [31]),
            ),
        ),
        (
            1,
            Vector((0.0, -1.0, 0.0)),
            Vector((1.0, 0.0, 0.0)),
            Vector((0.0, 0.0, 1.0)),
            [
                Vector((0.0, 0.0, 0.0)),
                Vector((2.0, 0.0, 0.0)),
                Vector((2.0, 0.0, 2.0)),
                Vector((0.0, 0.0, 2.0)),
            ],
            (([0, 1], [0, 1], [30]),),
        ),
        (
            2,
            Vector((-1.0, 0.0, 0.0)),
            Vector((0.0, 1.0, 0.0)),
            Vector((0.0, 0.0, 1.0)),
            [
                Vector((0.0, 0.0, 0.0)),
                Vector((0.0, 2.0, 0.0)),
                Vector((0.0, 2.0, 2.0)),
                Vector((0.0, 0.0, 2.0)),
            ],
            (([0, 3], [0, 1], [31]),),
        ),
    )
    for patch_id, normal, basis_u, basis_v, verts, chains in patch_specs:
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
                        vert_indices=vert_indices,
                        vert_cos=[verts[index] for index in point_indices],
                        edge_indices=edge_indices,
                    )
                    for vert_indices, point_indices, edge_indices in chains
                ]
            )
        ]
        graph.add_node(node)
    return graph


def _acute_corner_graph():
    """Planar convex apex с intrinsic углом 30 градусов."""

    angle = pi / 12.0
    points = [
        Vector((0.0, 0.0, 0.0)),
        Vector((2.0 * cos(angle), -2.0 * sin(angle), 0.0)),
        Vector((2.0 * cos(angle), 2.0 * sin(angle), 0.0)),
    ]
    node = PatchNode(
        patch_id=0,
        face_indices=[0],
        centroid=sum(points, Vector((0.0, 0.0, 0.0))) / 3.0,
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
    )
    node.mesh_verts = points
    node.mesh_tris = [(0, 1, 2)]
    node.boundary_loops = [
        BoundaryLoop(
            chains=[
                BoundaryChain(
                    vert_indices=[0, 1],
                    vert_cos=[points[0], points[1]],
                    edge_indices=[40],
                ),
                BoundaryChain(
                    vert_indices=[2, 0],
                    vert_cos=[points[2], points[0]],
                    edge_indices=[41],
                ),
            ]
        )
    ]
    graph = PatchGraph()
    graph.add_node(node)
    return graph


def _acute_notch_graph():
    """Reflex patch corner 330°, extrusion wedge 30°."""

    shoulder = 2.0 * sin(pi / 12.0) / cos(pi / 12.0)
    points = [
        Vector((-3.0, -2.0, 0.0)),
        Vector((3.0, -2.0, 0.0)),
        Vector((3.0, 2.0, 0.0)),
        Vector((shoulder, 2.0, 0.0)),
        Vector((0.0, 0.0, 0.0)),
        Vector((-shoulder, 2.0, 0.0)),
        Vector((-3.0, 2.0, 0.0)),
    ]
    node = PatchNode(
        patch_id=0,
        face_indices=[0],
        centroid=Vector((0.0, 0.0, 0.0)),
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
    )
    node.mesh_verts = points
    node.mesh_tris = [
        (0, 1, 4),
        (1, 2, 3),
        (1, 3, 4),
        (0, 4, 5),
        (0, 5, 6),
    ]
    all_edges = list(range(50, 57))
    node.boundary_loops = [
        BoundaryLoop(
            vert_indices=list(range(7)),
            vert_cos=[point.copy() for point in points],
            edge_indices=all_edges,
            chains=[
                BoundaryChain(
                    vert_indices=[3, 4],
                    vert_cos=[points[3], points[4]],
                    edge_indices=[53],
                ),
                BoundaryChain(
                    vert_indices=[4, 5],
                    vert_cos=[points[4], points[5]],
                    edge_indices=[54],
                ),
            ],
        )
    ]
    graph = PatchGraph()
    graph.add_node(node)
    return graph


def _door_opening_graph():
    """Один planar patch с concave дверным проёмом без owner diagonals."""

    points = [
        Vector((-4.0, 0.0, 0.0)),
        Vector((-1.0, 0.0, 0.0)),
        Vector((-1.0, 3.0, 0.0)),
        Vector((1.0, 3.0, 0.0)),
        Vector((1.0, 0.0, 0.0)),
        Vector((4.0, 0.0, 0.0)),
        Vector((4.0, 4.0, 0.0)),
        Vector((-4.0, 4.0, 0.0)),
    ]
    node = PatchNode(
        patch_id=0,
        face_indices=[0],
        centroid=Vector((0.0, 2.0, 0.0)),
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
    )
    node.mesh_verts = points
    node.mesh_tris = [
        (0, 1, 7),
        (1, 2, 7),
        (2, 3, 7),
        (3, 6, 7),
        (3, 4, 6),
        (4, 5, 6),
    ]
    edge_indices = list(range(100, 108))
    chain = BoundaryChain(
        vert_indices=list(range(8)),
        vert_cos=[point.copy() for point in points],
        edge_indices=edge_indices,
        is_closed=True,
    )
    node.boundary_loops = [
        BoundaryLoop(
            vert_indices=list(range(8)),
            vert_cos=[point.copy() for point in points],
            edge_indices=edge_indices,
            chains=[chain],
        )
    ]
    graph = PatchGraph()
    graph.add_node(node)
    return graph, edge_indices


def _wide_t_junction_front_graph():
    """Точный planar front patch walls.006 вокруг широкого T-junction."""

    points = [
        Vector((-37.679363, 15.175671, 0.0)),
        Vector((-37.679363, 19.733892, 0.0)),
        Vector((-25.643166, 19.733900, 0.0)),
        Vector((-25.643166, 15.175671, 0.0)),
        Vector((-30.796854, 15.175671, 0.0)),
        Vector((-30.796854, 18.608587, 0.0)),
        Vector((-32.621742, 18.608587, 0.0)),
        Vector((-32.621742, 15.175671, 0.0)),
    ]
    node = PatchNode(
        patch_id=0,
        face_indices=[0],
        centroid=sum(points, Vector((0.0, 0.0, 0.0))) / len(points),
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
    )
    node.mesh_verts = points
    node.mesh_tris = [
        (0, 1, 6),
        (0, 6, 7),
        (1, 2, 5),
        (1, 5, 6),
        (2, 3, 4),
        (2, 4, 5),
    ]
    edge_indices = list(range(200, 208))
    chain = BoundaryChain(
        vert_indices=list(range(8)),
        vert_cos=[point.copy() for point in points],
        edge_indices=edge_indices,
        is_closed=True,
    )
    node.boundary_loops = [
        BoundaryLoop(
            vert_indices=list(range(8)),
            vert_cos=[point.copy() for point in points],
            edge_indices=edge_indices,
            chains=[chain],
        )
    ]
    graph = PatchGraph()
    graph.add_node(node)
    return graph, edge_indices


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


def test_door_opening_builds_realtime_endpoint_miters_with_shared_keys():
    graph, edge_indices = _door_opening_graph()
    plan = compile_patch_voronoi_plan(graph, edge_indices, offset=0.01)
    assert plan is not None
    surface = plan.surfaces[0]
    point_atoms = [atom for atom in surface.atoms if atom.cell_kind == "POINT"]
    assert len(point_atoms) == 2
    assert {
        surface.corners[atom.corner_index].vert_index for atom in point_atoms
    } == {2, 3}

    narrow = evaluate_patch_voronoi_plan(plan, width=0.5, preview=True)
    wide = evaluate_patch_voronoi_plan(plan, width=2.0, preview=True)
    assert narrow and wide
    assert [tuple(face.positions) for face in wide] != [
        tuple(face.positions) for face in narrow
    ]
    for faces in (narrow, wide):
        for corner_vertex in (2, 3):
            corner_key = ("pv-sv", corner_vertex)
            owners = [face for face in faces if corner_key in face.vert_keys]
            # Endpoint-cell и обе соседние segment-cells используют одну
            # identity до финального remove-doubles.
            assert len(owners) >= 3
        for face in faces:
            for point in face.positions:
                assert -4.0 - 1e-7 <= point.x <= 4.0 + 1e-7
                assert -1e-7 <= point.y <= 4.0 + 1e-7

    def signature(faces):
        return [
            (
                tuple(face.vert_keys),
                tuple(tuple(round(value, 9) for value in point) for point in face.positions),
            )
            for face in faces
        ]

    assert signature(wide) == signature(
        evaluate_patch_voronoi_plan(plan, width=2.0, preview=False)
    )


def test_corner_spec_classifies_intrinsic_convex_concave_and_acute():
    door_graph, door_edges = _door_opening_graph()
    door_plan = compile_patch_voronoi_plan(
        door_graph, door_edges, offset=0.01
    )
    door_corners = {
        corner.vert_index: corner for corner in door_plan.surfaces[0].corners
    }
    for vert_index in (2, 3):
        corner = door_corners[vert_index]
        assert not corner.is_convex
        assert corner.interior_angle == pytest.approx(1.5 * pi)
        assert corner.extrusion_angle == pytest.approx(0.5 * pi)
        assert corner.policy == decal_voronoi._CornerPolicy.KITE
        assert len(corner.ordered_sites) == 2
        assert corner.turn_sign in (-1.0, 1.0)

    folded_plan = compile_patch_voronoi_plan(
        _folded_turn_graph(), [30, 31], offset=0.01
    )
    folded_corner = next(
        corner
        for corner in folded_plan.surfaces[0].corners
        if corner.vert_index == 0
    )
    assert folded_corner.is_convex
    assert folded_corner.interior_angle == pytest.approx(0.5 * pi)
    assert folded_corner.extrusion_angle == pytest.approx(0.5 * pi)
    assert folded_corner.policy == decal_voronoi._CornerPolicy.MITER

    acute_plan = compile_patch_voronoi_plan(
        _acute_corner_graph(), [40, 41], offset=0.01
    )
    acute_corner = next(
        corner
        for corner in acute_plan.surfaces[0].corners
        if corner.vert_index == 0
    )
    assert acute_corner.is_convex
    assert acute_corner.interior_angle == pytest.approx(pi / 6.0)
    assert acute_corner.extrusion_angle == pytest.approx(pi / 6.0)
    assert acute_corner.policy == decal_voronoi._CornerPolicy.MITER

    notch_plan = compile_patch_voronoi_plan(
        _acute_notch_graph(), [53, 54], offset=0.01
    )
    notch_corner = next(
        corner
        for corner in notch_plan.surfaces[0].corners
        if corner.vert_index == 4
    )
    assert not notch_corner.is_convex
    assert notch_corner.interior_angle == pytest.approx(11.0 * pi / 6.0)
    assert notch_corner.extrusion_angle == pytest.approx(pi / 6.0)
    assert notch_corner.policy == decal_voronoi._CornerPolicy.ACUTE_SPLIT


def test_convex_corner_builds_explicit_realtime_kite():
    graph, edge_indices = _door_opening_graph()
    plan = compile_patch_voronoi_plan(graph, edge_indices, offset=0.01)
    surface = plan.surfaces[0]
    corner = next(
        corner for corner in surface.corners if corner.vert_index == 2
    )
    kite = decal_voronoi._kite_crop_polygon(surface, corner, alpha=0.5)
    assert len(kite) == 4
    assert corner.point in kite
    assert abs(decal_voronoi._polygon_area2(kite)) == pytest.approx(0.25)

    narrow = decal_voronoi._kite_crop_polygon(surface, corner, alpha=0.25)
    assert len(narrow) == 4
    assert abs(decal_voronoi._polygon_area2(narrow)) == pytest.approx(
        0.25 * abs(decal_voronoi._polygon_area2(kite))
    )


def test_acute_corner_splits_inner_outer_with_shared_mesh_edge_and_uv_seam():
    plan = compile_patch_voronoi_plan(
        _acute_notch_graph(), [53, 54], offset=0.01
    )
    surface = plan.surfaces[0]
    corner = next(
        corner for corner in surface.corners if corner.vert_index == 4
    )
    components = decal_voronoi._acute_crop_components(
        surface, corner, alpha=0.5
    )
    assert [component.side for component in components] == ["INNER", "OUTER"]
    assert all(len(component.points) == 3 for component in components)
    full_kite = decal_voronoi._kite_crop_polygon(
        surface, corner, alpha=0.5
    )
    assert sum(
        abs(decal_voronoi._polygon_area2(component.points))
        for component in components
    ) == pytest.approx(abs(decal_voronoi._polygon_area2(full_kite)))
    overlap = decal_voronoi._clip_to_convex(
        components[0].points, components[1].points
    )
    assert not overlap or abs(decal_voronoi._polygon_area2(overlap)) <= 1e-9

    faces = evaluate_patch_voronoi_plan(plan, width=1.0, preview=True)
    acute_faces = {
        face.component_side: face
        for face in faces
        if face.component_kind == "ACUTE_SPLIT"
    }
    assert set(acute_faces) == {"INNER", "OUTER"}
    shared_keys = set(acute_faces["INNER"].vert_keys) & set(
        acute_faces["OUTER"].vert_keys
    )
    assert len(shared_keys) == 2
    for key in shared_keys:
        inner_index = acute_faces["INNER"].vert_keys.index(key)
        outer_index = acute_faces["OUTER"].vert_keys.index(key)
        assert acute_faces["INNER"].v_lengths[inner_index] != pytest.approx(
            acute_faces["OUTER"].v_lengths[outer_index]
        )

    confirmed = evaluate_patch_voronoi_plan(plan, width=1.0, preview=False)
    assert [
        (face.component_kind, face.component_side, tuple(face.vert_keys))
        for face in faces
    ] == [
        (face.component_kind, face.component_side, tuple(face.vert_keys))
        for face in confirmed
    ]


def test_wide_t_junction_cells_remain_convex_and_non_overlapping():
    graph, edge_indices = _wide_t_junction_front_graph()
    plan = compile_patch_voronoi_plan(graph, edge_indices, offset=0.02)
    assert plan is not None

    for width in (1.0545852, 2.0, 4.0):
        faces = evaluate_patch_voronoi_plan(plan, width=width, preview=True)
        polygons = [
            [(point.x, point.y) for point in face.positions] for face in faces
        ]
        assert polygons
        assert all(
            decal_voronoi._polygon_is_simple(polygon)
            and decal_voronoi._polygon_is_convex(polygon)
            for polygon in polygons
        )
        for index, polygon in enumerate(polygons):
            for other in polygons[index + 1 :]:
                intersection = decal_voronoi._clip_to_convex(
                    polygon, other
                )
                if intersection:
                    assert abs(
                        decal_voronoi._polygon_area2(intersection)
                    ) <= 1e-5


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


def test_arrangement_splits_point_on_neighbor_edge_before_materialization():
    polygons, inserted = decal_voronoi._insert_surface_edge_stations(
        [
            [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)],
            [(0.0, 1.0), (1.0, 1.0), (1.0, 2.0), (0.0, 2.0)],
        ],
        tolerance=1e-7,
    )
    assert inserted == 1
    assert polygons[0] == [
        (0.0, 0.0),
        (2.0, 0.0),
        (2.0, 1.0),
        (1.0, 1.0),
        (0.0, 1.0),
    ]
    assert (1.0, 1.0) in polygons[1]


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


def test_folded_turn_builds_one_realtime_junction_sector():
    plan = compile_patch_voronoi_plan(
        _folded_turn_graph(), [30, 31], offset=0.01
    )
    faces = evaluate_patch_voronoi_plan(plan, width=0.5, preview=True)
    connectors = [
        face
        for face in faces
        if face.surface_id == -1 and face.vert_keys[0] == ("pv-sv", 0)
    ]
    assert len(connectors) == 1

    edge_uses = {}
    for face in faces:
        for index, key_a in enumerate(face.vert_keys):
            key_b = face.vert_keys[(index + 1) % len(face.vert_keys)]
            edge_key = tuple(sorted((key_a, key_b), key=repr))
            edge_uses[edge_key] = edge_uses.get(edge_key, 0) + 1
    open_rail_edges = [
        edge_key
        for edge_key, use_count in edge_uses.items()
        if use_count == 1
        and ("pv-sv", 0) in edge_key
        and not all(key[:1] == ("pv-sv",) for key in edge_key)
    ]
    assert not open_rail_edges

    confirmed = evaluate_patch_voronoi_plan(plan, width=0.5, preview=False)
    assert [
        (
            tuple(face.vert_keys),
            tuple(tuple(round(value, 9) for value in point) for point in face.positions),
        )
        for face in faces
    ] == [
        (
            tuple(face.vert_keys),
            tuple(tuple(round(value, 9) for value in point) for point in face.positions),
        )
        for face in confirmed
    ]
