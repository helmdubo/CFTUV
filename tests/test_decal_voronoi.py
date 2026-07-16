from __future__ import annotations

from copy import deepcopy
from types import SimpleNamespace

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


@pytest.mark.parametrize(
    "error_type",
    (
        decal_voronoi.pyvoronoi.FocusOnDirectixException,
        decal_voronoi.pyvoronoi.UnsolvableParabolaEquation,
    ),
)
def test_invalid_boost_parabola_falls_back_to_shared_chord(error_type):
    class BrokenDiagram:
        def DiscretizeCurvedEdge(self, *_args):
            raise error_type("invalid curved edge")

    edges = [
        SimpleNamespace(start=0, end=1, is_linear=False),
    ]
    vertices = [
        SimpleNamespace(X=1.25, Y=-2.0),
        SimpleNamespace(X=3.75, Y=4.0),
    ]

    assert decal_voronoi._edge_points(
        BrokenDiagram(), edges, vertices, 0, 0.5
    ) == [(1.25, -2.0), (3.75, 4.0)]


def test_unrelated_curve_backend_error_is_not_hidden():
    class BrokenDiagram:
        def DiscretizeCurvedEdge(self, *_args):
            raise RuntimeError("unexpected backend failure")

    edges = [SimpleNamespace(start=0, end=1, is_linear=False)]
    vertices = [
        SimpleNamespace(X=0.0, Y=0.0),
        SimpleNamespace(X=1.0, Y=1.0),
    ]

    with pytest.raises(RuntimeError, match="unexpected backend failure"):
        decal_voronoi._edge_points(
            BrokenDiagram(), edges, vertices, 0, 0.5
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


def test_quantized_duplicate_site_is_localized_as_compile_failure():
    graph = _planar_two_site_graph()
    chain = graph.nodes[0].boundary_loops[0].chains[0]
    chain.vert_cos[1] = Vector((0.0, 0.00005, 0.0))

    attempt = decal_voronoi.compile_patch_voronoi_attempt(
        graph, [10], offset=0.01, allow_partial=True
    )

    assert attempt.plan is None
    assert attempt.rejected_edge_indices == (10,)
    assert len(attempt.failures) == 1
    failure = attempt.failures[0]
    assert failure.reason == "QUANTIZED_DEGENERATE_SITE"
    assert failure.edge_indices == (10,)
    assert "raw_length=" in failure.details
    assert "quantized_length=0" in failure.details
    assert "quantum=" in failure.details


def test_corner_offset_lines_reject_zero_length_site():
    surface = SimpleNamespace(
        sites=(
            SimpleNamespace(
                segment_length=0.0,
                vert_a=1,
                point_a=(0.0, 0.0),
                point_b=(0.0, 0.0),
                inward_normal=(0.0, 1.0),
            ),
        )
    )
    corner = SimpleNamespace(
        vert_index=1,
        point=(0.0, 0.0),
        ordered_sites=(0,),
    )

    assert decal_voronoi._corner_offset_lines(surface, corner, 0.5) == []


def test_cell_triangulation_accepts_only_zero_area_collinear_remainder():
    # Реальная segment-cell walls.010: ненулевые ears покрывают весь contour,
    # после чего остаётся четыре коллинеарные stations. Это не compile failure.
    polygon = [
        (-2.508528, 4.30705),
        (-2.508529, 4.3071),
        (-76.00318, 4.3071),
        (-76.00318, 1.7071),
        (-2.508529, 1.7071),
        (-2.5088, 1.7428),
        (-2.5088, 2.9713),
        (-2.508528, 3.00705),
        (-2.5088, 3.0428),
        (-2.5088, 4.2713),
    ]

    triangles = decal_voronoi._triangulate_cell_polygon(polygon)

    assert triangles
    assert sum(
        decal_voronoi._polygon_area2(triangle) for triangle in triangles
    ) == pytest.approx(
        decal_voronoi._polygon_area2(polygon),
        abs=1e-7,
    )


def test_network_face_serializer_is_stable_across_face_order():
    first = decal_voronoi._NetworkFace(
        surface_id=2,
        surface_normal=Vector((0.0, 0.0, 1.0)),
        vert_keys=[("b", 2), ("a", 1), ("c", 3)],
        positions=[
            Vector((1.000000001, 0.0, 0.0)),
            Vector((0.0, 0.0, 0.0)),
            Vector((0.0, 1.0, 0.0)),
        ],
        u_fracs=[1.0, -1.0, 0.0],
        v_lengths=[2.0, 1.0, 3.0],
        component_kind="SEGMENT",
        component_side="",
    )
    second = decal_voronoi._NetworkFace(
        surface_id=1,
        surface_normal=Vector((0.0, 0.0, 1.0)),
        vert_keys=[("d", 4), ("e", 5), ("f", 6)],
        positions=[
            Vector((2.0, 0.0, 0.0)),
            Vector((3.0, 0.0, 0.0)),
            Vector((2.0, 1.0, 0.0)),
        ],
        u_fracs=[-1.0, 1.0, 0.0],
        v_lengths=[0.0, 0.0, 1.0],
        component_kind="KITE",
        component_side="OUTER",
    )

    forward = decal_voronoi.serialize_network_faces([first, second])
    reversed_faces = decal_voronoi.serialize_network_faces([second, first])

    assert forward == reversed_faces
    assert forward["topology_signature"] == {
        "vertex_count": 6,
        "edge_count": 6,
        "face_count": 2,
        "face_loops": ((3, 4, 5), (0, 2, 1)),
    }
    assert forward["faces"][1]["positions"][2] == (1.0, 0.0, 0.0)


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


def _single_patch_fold_graph():
    """Один topology patch с двумя реальными owner-плоскостями."""

    verts = [
        Vector((0.0, 0.0, 0.0)),
        Vector((2.0, 0.0, 0.0)),
        Vector((2.0, 2.0, 0.0)),
        Vector((0.0, 2.0, 0.0)),
        Vector((2.0, 0.0, 1.0)),
        Vector((0.0, 0.0, 1.0)),
    ]
    node = PatchNode(
        patch_id=7,
        face_indices=[0, 1, 2, 3],
        centroid=sum(verts, Vector((0.0, 0.0, 0.0))) / len(verts),
        normal=Vector((0.0, -1.0, 1.0)).normalized(),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
    )
    node.mesh_verts = verts
    node.mesh_tris = [
        (0, 1, 2),
        (0, 2, 3),
        (0, 4, 1),
        (0, 5, 4),
    ]
    front = BoundaryChain(
        vert_indices=[3, 2],
        vert_cos=[verts[3], verts[2]],
        edge_indices=[70],
        side_face_normals=[Vector((0.0, 0.0, 1.0))],
    )
    folded = BoundaryChain(
        vert_indices=[5, 4],
        vert_cos=[verts[5], verts[4]],
        edge_indices=[71],
        side_face_normals=[Vector((0.0, -1.0, 0.0))],
    )
    node.boundary_loops = [BoundaryLoop(chains=[front, folded])]
    graph = PatchGraph()
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


def test_width_drag_does_not_reconstruct_voronoi_diagram():
    graph, edge_indices = _door_opening_graph()
    diagnostics = decal_voronoi.PatchVoronoiDiagnostics()
    plan = compile_patch_voronoi_plan(
        graph,
        edge_indices,
        offset=0.01,
        diagnostics=diagnostics,
    )
    compile_construct_calls = diagnostics.construct_calls

    snapshots = []
    for width in (2.0, 3.0, 3.7076, 4.5):
        faces = evaluate_patch_voronoi_plan(
            plan,
            width=width,
            preview=True,
            diagnostics=diagnostics,
        )
        snapshots.append(decal_voronoi.serialize_network_faces(faces))

    assert compile_construct_calls > 0
    assert diagnostics.construct_calls == compile_construct_calls
    assert all(snapshot["faces"] for snapshot in snapshots)


def test_repeated_preview_and_confirm_serialization_is_deterministic():
    graph, edge_indices = _door_opening_graph()
    first_plan = compile_patch_voronoi_plan(
        graph, edge_indices, offset=0.01
    )
    second_plan = compile_patch_voronoi_plan(
        graph, edge_indices, offset=0.01
    )

    first = decal_voronoi.serialize_network_faces(
        evaluate_patch_voronoi_plan(
            first_plan, width=3.7076, preview=True
        )
    )
    repeated = decal_voronoi.serialize_network_faces(
        evaluate_patch_voronoi_plan(
            second_plan, width=3.7076, preview=True
        )
    )
    confirmed = decal_voronoi.serialize_network_faces(
        evaluate_patch_voronoi_plan(
            first_plan, width=3.7076, preview=False
        )
    )

    assert first == repeated == confirmed


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
        assert (
            decal_voronoi.classify_corner_runtime(corner)
            == decal_voronoi._CornerPolicy.KITE
        )
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
    assert (
        decal_voronoi.classify_corner_runtime(folded_corner)
        == decal_voronoi._CornerPolicy.MITER
    )

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
    assert (
        decal_voronoi.classify_corner_runtime(acute_corner)
        == decal_voronoi._CornerPolicy.ACUTE_SPLIT
    )

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
    assert (
        decal_voronoi.classify_corner_runtime(notch_corner)
        == decal_voronoi._CornerPolicy.ACUTE_SPLIT
    )


def test_corner_policy_threshold_changes_without_recompiling_plan():
    diagnostics = decal_voronoi.PatchVoronoiDiagnostics()
    plan = compile_patch_voronoi_plan(
        _acute_corner_graph(), [40, 41], offset=0.01,
        diagnostics=diagnostics,
    )
    construct_calls = diagnostics.construct_calls
    corner = next(
        item for item in plan.surfaces[0].corners if item.vert_index == 0
    )
    split_settings = decal_voronoi.CornerRuntimeSettings(
        acute_split_angle=pi / 3.0,
    )
    miter_settings = decal_voronoi.CornerRuntimeSettings(
        acute_split_angle=pi / 12.0,
    )

    assert (
        decal_voronoi.classify_corner_runtime(corner, split_settings)
        == decal_voronoi._CornerPolicy.ACUTE_SPLIT
    )
    assert (
        decal_voronoi.classify_corner_runtime(corner, miter_settings)
        == decal_voronoi._CornerPolicy.MITER
    )

    split_faces = evaluate_patch_voronoi_plan(
        plan,
        width=1.0,
        preview=True,
        corner_settings=split_settings,
        diagnostics=diagnostics,
    )
    split_policy_counts = dict(diagnostics.runtime_policy_counts)
    miter_faces = evaluate_patch_voronoi_plan(
        plan,
        width=1.0,
        preview=True,
        corner_settings=miter_settings,
        diagnostics=diagnostics,
    )
    miter_policy_counts = dict(diagnostics.runtime_policy_counts)
    assert sum(
        face.component_kind == "ACUTE_SPLIT" for face in split_faces
    ) == 2
    assert not any(face.component_kind == "ACUTE_SPLIT" for face in miter_faces)
    assert split_policy_counts["ACUTE_SPLIT"] == 1
    assert miter_policy_counts.get("ACUTE_SPLIT", 0) == 0
    assert miter_policy_counts["MITER"] > split_policy_counts.get("MITER", 0)
    assert diagnostics.construct_calls == construct_calls


def _apex_limit_evaluations(plan, acute_split_angle):
    results = {}
    for apex_limit in (1.0, 8.0, 100.0):
        settings = decal_voronoi.CornerRuntimeSettings(
            acute_split_angle=acute_split_angle,
            apex_limit=apex_limit,
        )
        diagnostics = decal_voronoi.PatchVoronoiDiagnostics()
        preview = evaluate_patch_voronoi_plan(
            plan,
            width=1.0,
            preview=True,
            corner_settings=settings,
            diagnostics=diagnostics,
        )
        confirmed = evaluate_patch_voronoi_plan(
            plan,
            width=1.0,
            preview=False,
            corner_settings=settings,
        )
        assert decal_voronoi.serialize_network_faces(
            preview
        ) == decal_voronoi.serialize_network_faces(confirmed)
        assert all(_signed_face_area(face) > 1e-10 for face in preview)
        results[apex_limit] = (preview, diagnostics)
    return results


def test_apex_limit_changes_convex_miter_contour():
    plan = compile_patch_voronoi_plan(
        _folded_turn_graph(), [30, 31], offset=0.01
    )
    results = _apex_limit_evaluations(plan, acute_split_angle=0.1)

    snapshots = {
        limit: decal_voronoi.serialize_network_faces(faces)
        for limit, (faces, _diagnostics) in results.items()
    }
    assert snapshots[1.0] != snapshots[8.0]
    assert snapshots[8.0] == snapshots[100.0]
    assert sum(
        face.component_kind == "MITER" for face in results[1.0][0]
    ) == 1
    assert results[1.0][1].clamped_miter_count == 1
    assert results[8.0][1].clamped_miter_count == 0


def test_apex_limit_changes_kite_contour():
    graph, edge_indices = _door_opening_graph()
    plan = compile_patch_voronoi_plan(graph, edge_indices, offset=0.01)
    results = _apex_limit_evaluations(plan, acute_split_angle=1.0)

    snapshots = {
        limit: decal_voronoi.serialize_network_faces(faces)
        for limit, (faces, _diagnostics) in results.items()
    }
    assert snapshots[1.0] != snapshots[8.0]
    assert snapshots[8.0] == snapshots[100.0]
    assert results[1.0][1].clamped_kite_count == 2
    assert results[8.0][1].clamped_kite_count == 0


def test_apex_limit_clamps_acute_outer_without_gap():
    plan = compile_patch_voronoi_plan(
        _acute_notch_graph(), [53, 54], offset=0.01
    )
    surface = plan.surfaces[0]
    corner = next(
        item for item in surface.corners if item.vert_index == 4
    )
    low_settings = decal_voronoi.CornerRuntimeSettings(apex_limit=1.0)
    diagnostics = decal_voronoi.PatchVoronoiDiagnostics()
    components = decal_voronoi._acute_crop_components(
        surface,
        corner,
        alpha=0.5,
        settings=low_settings,
        diagnostics=diagnostics,
    )

    inner = next(component for component in components if component.side == "INNER")
    outer = next(component for component in components if component.side == "OUTER")
    shared_cap = set(inner.points).intersection(outer.points)
    assert len(shared_cap) == 2
    assert abs(decal_voronoi._polygon_area2(inner.points)) > 1e-10
    assert abs(decal_voronoi._polygon_area2(outer.points)) > 1e-10
    assert decal_voronoi._polygon_is_simple(list(inner.points))
    assert decal_voronoi._polygon_is_simple(list(outer.points))
    outer_apex = next(point for point in outer.points if point not in shared_cap)
    assert decal_voronoi._dist2(corner.point, outer_apex) <= 0.5 + 1e-8
    cap_a, cap_b = tuple(shared_cap)
    corner_side = decal_voronoi._cross2(
        decal_voronoi._sub2(cap_b, cap_a),
        decal_voronoi._sub2(corner.point, cap_a),
    )
    apex_side = decal_voronoi._cross2(
        decal_voronoi._sub2(cap_b, cap_a),
        decal_voronoi._sub2(outer_apex, cap_a),
    )
    assert corner_side * apex_side < 0.0
    assert diagnostics.clamped_acute_count == 1

    results = _apex_limit_evaluations(plan, acute_split_angle=1.0)
    snapshots = {
        limit: decal_voronoi.serialize_network_faces(faces)
        for limit, (faces, _diagnostics) in results.items()
    }
    assert snapshots[1.0] != snapshots[8.0]
    assert snapshots[8.0] == snapshots[100.0]
    assert all(
        sum(face.component_kind == "ACUTE_SPLIT" for face in faces) == 2
        for faces, _diagnostics in results.values()
    )


def test_bevel_policy_is_reserved_and_never_classified():
    plan = compile_patch_voronoi_plan(
        _folded_turn_graph(), [30, 31], offset=0.01
    )
    corner = next(
        item for item in plan.surfaces[0].corners if item.vert_index == 0
    )
    legacy_bevel_candidate = decal_voronoi.replace(
        corner,
        is_convex=False,
        interior_angle=pi * 0.75,
        extrusion_angle=pi * 0.75,
        miter_ratio=1000.0,
    )

    assert decal_voronoi.classify_corner_runtime(
        legacy_bevel_candidate,
        decal_voronoi.CornerRuntimeSettings(apex_limit=1.0),
    ) == decal_voronoi._CornerPolicy.MITER
    assert decal_voronoi.CornerRuntimeSettings(
        miter_limit=3.0
    ).apex_limit == 3.0


def test_acute_apex_limit_saturates_outside_cap_chord():
    diagnostics = decal_voronoi.PatchVoronoiDiagnostics()
    apex = decal_voronoi._clamped_acute_apex(
        corner_point=(0.0, 0.0),
        cap_a=(2.0, -1.0),
        cap_b=(2.0, 1.0),
        intersection=(4.0, 0.0),
        alpha=0.5,
        settings=decal_voronoi.CornerRuntimeSettings(apex_limit=1.0),
        diagnostics=diagnostics,
    )

    assert apex[0] > 2.0
    assert diagnostics.clamped_acute_count == 1
    assert diagnostics.apex_limit_saturated_count == 1


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


def test_corner_absorbs_incident_point_cell_boundaries_before_collision():
    graph, edge_indices = _door_opening_graph()
    plan = compile_patch_voronoi_plan(graph, edge_indices, offset=0.01)
    faces = evaluate_patch_voronoi_plan(plan, width=0.1, preview=True)

    kite_faces = [face for face in faces if face.component_kind == "KITE"]
    segment_faces = [
        face for face in faces if face.component_kind == "SEGMENT"
    ]
    assert len(kite_faces) == 2
    assert all(len(face.positions) == 4 for face in kite_faces)
    assert len(segment_faces) == len(plan.surfaces[0].sites)


def _short_segment_endpoint_surface(reverse_site=False):
    if reverse_site:
        vert_a, vert_b = 1, 0
        point_a, point_b = (1.0, 0.0), (0.0, 0.0)
    else:
        vert_a, vert_b = 0, 1
        point_a, point_b = (0.0, 0.0), (1.0, 0.0)
    site = decal_voronoi._PatchVoronoiSite(
        patch_id=0,
        edge_index=500,
        vert_a=vert_a,
        vert_b=vert_b,
        source_a=Vector((*point_a, 0.0)),
        source_b=Vector((*point_b, 0.0)),
        point_a=point_a,
        point_b=point_b,
        arc_start=0.0,
        segment_length=1.0,
        uv_sign=1.0,
        inward_normal=(0.0, 1.0),
    )
    corners = tuple(
        decal_voronoi.CornerSpec(
            vert_index=vert_index,
            point=(float(vert_index), 0.0),
            incident_sites=(0,),
            ordered_sites=(0,),
            turn_sign=1.0,
            interior_angle=pi,
            extrusion_angle=pi,
            is_convex=vert_index == 0,
            miter_ratio=1.0,
        )
        for vert_index in (0, 1)
    )
    segment_fragment = ((0.0, -1.0), (1.0, -1.0), (1.0, 1.0), (0.0, 1.0))
    point_fragments = (
        ((-1.0, -1.0), (0.0, -1.0), (0.0, 1.0), (-1.0, 1.0)),
        ((1.0, -1.0), (2.0, -1.0), (2.0, 1.0), (1.0, 1.0)),
    )
    atoms = (
        decal_voronoi._PatchVoronoiAtom(
            site_index=0,
            fragments=(segment_fragment,),
            cell_kind="SEGMENT",
        ),
        decal_voronoi._PatchVoronoiAtom(
            site_index=0,
            fragments=(point_fragments[0],),
            cell_kind="POINT",
            corner_index=0,
        ),
        decal_voronoi._PatchVoronoiAtom(
            site_index=0,
            fragments=(point_fragments[1],),
            cell_kind="POINT",
            corner_index=1,
        ),
    )
    return SimpleNamespace(sites=(site,), corners=corners, atoms=atoms)


def _short_segment_owned_crops(monkeypatch, reverse_site=False):
    monkeypatch.setattr(
        decal_voronoi,
        "classify_corner_runtime",
        lambda corner, _settings=None: (
            decal_voronoi._CornerPolicy.MITER
            if corner.vert_index == 0
            else decal_voronoi._CornerPolicy.KITE
        ),
    )
    pending = []
    decal_voronoi._evaluate_surface_crops(
        _short_segment_endpoint_surface(reverse_site),
        alpha=1.0,
        pending=pending,
        corner_settings=decal_voronoi.CornerRuntimeSettings(),
    )
    return pending


def test_short_segment_corner_ownership_is_disjoint_and_order_independent(
    monkeypatch,
):
    pending = _short_segment_owned_crops(monkeypatch, reverse_site=False)
    assert {face.crop.kind for face in pending} == {"MITER", "KITE"}
    assert len(pending) == 2

    by_kind = {face.crop.kind: face for face in pending}
    overlap = decal_voronoi._clip_to_convex(
        by_kind["MITER"].points,
        by_kind["KITE"].points,
    )
    assert not overlap or abs(decal_voronoi._polygon_area2(overlap)) <= 1e-10
    assert sum(
        abs(decal_voronoi._polygon_area2(face.points)) for face in pending
    ) == pytest.approx(6.0)

    shared_edge = set(by_kind["MITER"].points).intersection(
        by_kind["KITE"].points
    )
    assert shared_edge == {(0.5, -1.0), (0.5, 1.0)}
    face_keys = {
        tuple(sorted(face.points))
        for face in pending
    }
    assert len(face_keys) == len(pending)

    reversed_pending = _short_segment_owned_crops(
        monkeypatch, reverse_site=True
    )

    def signature(faces):
        return {
            face.crop.kind: tuple(sorted(face.points))
            for face in faces
        }

    assert signature(reversed_pending) == signature(pending)


def test_endpoint_ownership_preserves_affine_uv_after_triangle_clip():
    surface = _short_segment_endpoint_surface()
    corner = surface.corners[0]
    crop = decal_voronoi._CropComponent(
        kind="ACUTE_SPLIT",
        side="OUTER",
        points=((-1.0, -1.0), (1.0, -1.0), (0.0, 1.0)),
        uv_anchors=((-1.0, -1.0), (1.0, -1.0), (0.0, 1.0)),
        v_origin=2.0,
    )

    owned = decal_voronoi._corner_endpoint_ownership_crop(
        surface, corner, crop
    )

    assert owned is not None
    assert len(owned.points) == 4
    assert len(owned.uv_anchors) == 4
    for point in owned.points:
        assert decal_voronoi._crop_component_uv(
            owned, point
        ) == pytest.approx((point[0], point[1] + 2.0))


def test_planar_chart_is_invariant_to_patch_normal_sign():
    graph, edge_indices = _door_opening_graph()
    flipped_graph = deepcopy(graph)
    flipped_node = flipped_graph.nodes[0]
    flipped_node.normal = flipped_node.normal * -1.0
    flipped_node.basis_v = flipped_node.basis_v * -1.0

    plan = compile_patch_voronoi_plan(graph, edge_indices, offset=0.01)
    flipped_plan = compile_patch_voronoi_plan(
        flipped_graph, edge_indices, offset=0.01
    )
    surface = plan.surfaces[0]
    flipped_surface = flipped_plan.surfaces[0]
    assert [
        (site.point_a, site.point_b) for site in surface.sites
    ] == [
        (site.point_a, site.point_b) for site in flipped_surface.sites
    ]
    assert surface.atoms == flipped_surface.atoms

    for width in (0.1, 0.5, 2.0):
        faces = evaluate_patch_voronoi_plan(plan, width, preview=True)
        flipped_faces = evaluate_patch_voronoi_plan(
            flipped_plan, width, preview=True
        )
        assert [
            (
                face.component_kind,
                face.component_side,
                tuple(face.vert_keys),
            )
            for face in faces
        ] == [
            (
                face.component_kind,
                face.component_side,
                tuple(reversed(face.vert_keys)),
            )
            for face in flipped_faces
        ]


def test_realtime_corner_partition_has_no_planar_gaps():
    graph, edge_indices = _door_opening_graph()
    plan = compile_patch_voronoi_plan(graph, edge_indices, offset=0.01)
    surface = plan.surfaces[0]
    domain_points = [
        point
        for triangle in surface.domain.boundary_triangles
        for point in triangle
    ]
    min_x = min(point[0] for point in domain_points)
    max_x = max(point[0] for point in domain_points)
    min_y = min(point[1] for point in domain_points)
    max_y = max(point[1] for point in domain_points)

    def inside_polygon(point, polygon):
        inside = False
        for index, point_a in enumerate(polygon):
            point_b = polygon[(index + 1) % len(polygon)]
            if (point_a[1] > point[1]) == (point_b[1] > point[1]):
                continue
            crossing_x = point_a[0] + (
                (point[1] - point_a[1])
                * (point_b[0] - point_a[0])
                / (point_b[1] - point_a[1])
            )
            if crossing_x > point[0]:
                inside = not inside
        return inside

    for width in (0.2, 1.0, 3.0):
        alpha = width * 0.5
        faces = evaluate_patch_voronoi_plan(plan, width, preview=True)
        polygons = [
            [
                (
                    (position - surface.origin).dot(surface.basis_u),
                    (position - surface.origin).dot(surface.basis_v),
                )
                for position in face.positions
            ]
            for face in faces
            if face.surface_id == surface.patch_id
        ]
        for grid_y in range(31):
            for grid_x in range(31):
                point = (
                    min_x + (grid_x + 0.37) / 31.0 * (max_x - min_x),
                    min_y + (grid_y + 0.61) / 31.0 * (max_y - min_y),
                )
                if not decal_voronoi._point_in_domain(
                    point, surface.domain.boundary_triangles
                ):
                    continue
                distance = min(
                    decal_voronoi._segment_point_distance2(
                        site.point_a, site.point_b, point
                    )[0]
                    for site in surface.sites
                )
                if abs(distance - alpha) <= 1e-5:
                    continue
                actual = any(
                    inside_polygon(point, polygon) for polygon in polygons
                )
                if distance < alpha:
                    assert actual


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


def test_acute_convex_corner_splits_before_wide_miter_competition():
    plan = compile_patch_voronoi_plan(
        _acute_corner_graph(), [40, 41], offset=0.01
    )
    surface = plan.surfaces[0]
    corner = next(
        corner for corner in surface.corners if corner.vert_index == 0
    )

    components = decal_voronoi._acute_crop_components(
        surface, corner, alpha=2.0
    )

    assert {component.side for component in components} == {"INNER", "OUTER"}
    assert all(
        component.kind == decal_voronoi._CornerPolicy.ACUTE_SPLIT.value
        for component in components
    )
    shared_points = set(components[0].points) & set(components[1].points)
    assert len(shared_points) == 2

    faces = evaluate_patch_voronoi_plan(plan, width=4.0, preview=False)
    acute_faces = [
        face for face in faces if face.component_kind == "ACUTE_SPLIT"
    ]
    assert {face.component_side for face in acute_faces} == {
        "INNER",
        "OUTER",
    }


def test_cross_surface_spine_station_is_mirrored_without_new_face():
    from types import SimpleNamespace

    station_key = ("pv-se", 40, 0.25)
    station = Vector((0.25, 0.0, 0.0))
    owner = decal_voronoi._NetworkFace(
        surface_id=0,
        surface_normal=Vector((0.0, 0.0, 1.0)),
        vert_keys=[("pv-sv", 0), station_key, ("pv", 0, 1, 1)],
        positions=[
            Vector((0.0, 0.0, 0.0)),
            station,
            Vector((0.0, 1.0, 0.0)),
        ],
        u_fracs=[0.0, 0.25, 1.0],
        v_lengths=[0.0, 0.25, 1.0],
    )
    neighbour = decal_voronoi._NetworkFace(
        surface_id=1,
        surface_normal=Vector((0.0, 1.0, 0.0)),
        vert_keys=[("pv-sv", 0), ("pv-sv", 1), ("pv", 1, 1, 1)],
        positions=[
            Vector((0.0, 0.0, 0.0)),
            Vector((1.0, 0.0, 0.0)),
            Vector((0.0, 0.0, 1.0)),
        ],
        u_fracs=[0.0, 1.0, 0.0],
        v_lengths=[0.0, 4.0, 0.0],
    )
    plan = SimpleNamespace(
        surfaces=(
            SimpleNamespace(
                sites=(
                    SimpleNamespace(edge_index=40, vert_a=0, vert_b=1),
                )
            ),
        )
    )

    decal_voronoi._synchronize_cross_surface_spine_stations(
        plan, [owner, neighbour]
    )

    assert neighbour.vert_keys == [
        ("pv-sv", 0),
        station_key,
        ("pv-sv", 1),
        ("pv", 1, 1, 1),
    ]
    assert tuple(neighbour.positions[1]) == pytest.approx(tuple(station))
    assert neighbour.u_fracs[1] == pytest.approx(0.25)
    assert neighbour.v_lengths[1] == pytest.approx(1.0)

def test_surface_domain_separates_planar_solver_from_intrinsic_lift():
    planar = decal_voronoi.DecalSurfaceDomain(
        patch_id=0,
        kind="PLANAR",
        origin=Vector((1.0, 2.0, 3.0)),
        reference_normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
        boundary_triangles=(),
    )
    assert tuple(planar.lift((2.0, 4.0), 0.5)) == pytest.approx(
        (3.0, 6.0, 3.5)
    )
    assert planar.project(Vector((3.0, 6.0, 3.5))) == pytest.approx(
        (2.0, 4.0)
    )

    normals = (
        Vector((0.0, 0.0, 1.0)),
        Vector((0.0, 0.0, 1.0)),
        Vector((0.0, 1.0, 1.0)).normalized(),
    )
    intrinsic = decal_voronoi.DecalSurfaceDomain(
        patch_id=1,
        kind="INTRINSIC",
        origin=Vector((0.0, 0.0, 0.0)),
        reference_normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
        boundary_triangles=(((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)),),
        intrinsic_triangles=(
            decal_voronoi._IntrinsicDomainTriangle(
                chart_points=((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)),
                positions=(
                    Vector((0.0, 0.0, 0.0)),
                    Vector((1.0, 0.0, 0.0)),
                    Vector((0.0, 1.0, 1.0)),
                ),
                normals=normals,
            ),
        ),
        periodic_axis="U",
    )
    point = (0.25, 0.25)
    expected_position = Vector((0.25, 0.25, 0.25))
    expected_normal = (
        normals[0] * 0.5 + normals[1] * 0.25 + normals[2] * 0.25
    ).normalized()
    assert tuple(intrinsic.normal_at(point)) == pytest.approx(
        tuple(expected_normal)
    )
    assert tuple(intrinsic.lift(point, 0.1)) == pytest.approx(
        tuple(expected_position + expected_normal * 0.1)
    )
    with pytest.raises(ValueError):
        intrinsic.project(Vector((0.0, 0.0, 0.0)))


def test_wide_t_junction_cells_remain_simple_and_non_overlapping():
    graph, edge_indices = _wide_t_junction_front_graph()
    plan = compile_patch_voronoi_plan(graph, edge_indices, offset=0.02)
    assert plan is not None

    for width in (1.0545852, 2.0, 4.0):
        faces = evaluate_patch_voronoi_plan(plan, width=width, preview=True)
        polygons = [
            [(point.x, point.y) for point in face.positions] for face in faces
        ]
        assert polygons
        assert all(decal_voronoi._polygon_is_simple(polygon) for polygon in polygons)
        for index, polygon in enumerate(polygons):
            for other in polygons[index + 1 :]:
                for triangle in decal_voronoi._triangulate_cell_polygon(
                    polygon
                ):
                    for other_triangle in decal_voronoi._triangulate_cell_polygon(
                        other
                    ):
                        intersection = decal_voronoi._clip_to_convex(
                            triangle, other_triangle
                        )
                        if intersection:
                            assert abs(
                                decal_voronoi._polygon_area2(intersection)
                            ) <= 1e-5


@pytest.mark.parametrize(
    ("polygon", "expected"),
    [
        (
            [(0.0, 0.0), (3.0, 0.0), (3.0, 2.0), (1.0, 1.0), (0.0, 2.0)],
            True,
        ),
        (
            [(0.0, 0.0), (3.0, 2.0), (0.0, 2.0), (3.0, 0.0)],
            False,
        ),
    ],
)
def test_polygon_sweep_preserves_exact_intersection_result(polygon, expected):
    assert decal_voronoi._polygon_is_simple(polygon) is expected


def test_polygon_sweep_accepts_dense_convex_cell():
    polygon = [
        (cos(index * 2.0 * pi / 512), sin(index * 2.0 * pi / 512))
        for index in range(512)
    ]
    assert decal_voronoi._polygon_is_simple(polygon)


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


def test_fragment_union_preserves_simple_concave_ngon():
    components = _merge_polygon_fragments(
        [
            [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0)],
            [(0.0, 0.0), (2.0, 1.0), (1.0, 1.0)],
            [(0.0, 0.0), (1.0, 1.0), (1.0, 2.0)],
            [(0.0, 0.0), (1.0, 2.0), (0.0, 2.0)],
        ]
    )
    assert len(components) == 1
    assert len(components[0]) == 6
    assert decal_voronoi._polygon_is_simple(components[0])
    assert not decal_voronoi._polygon_is_convex(components[0])


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
    attempt = decal_voronoi.compile_patch_voronoi_attempt(
        graph, [10, 12], offset=0.01, allow_partial=True
    )
    assert attempt.plan is None
    assert attempt.rejected_edge_indices == (10, 12)
    assert [(failure.patch_id, failure.reason) for failure in attempt.failures] == [
        (0, "NO_OWNER_SURFACES")
    ]


def test_warped_owner_face_uses_serialized_face_provenance():
    graph = _planar_two_site_graph()
    node = graph.nodes[0]
    node.mesh_verts[2] = Vector((4.0, 2.0, 0.01))
    node.mesh_tri_face_indices = [17, 17]
    node.mesh_tri_face_normals = [
        Vector((0.0, -0.0025, 1.0)).normalized(),
        Vector((0.0, -0.0025, 1.0)).normalized(),
    ]
    for chain in node.boundary_loops[0].chains:
        chain.side_face_indices = [17]
        chain.side_face_normals = [node.mesh_tri_face_normals[0].copy()]

    plan = compile_patch_voronoi_plan(graph, [10, 12], offset=0.01)

    assert plan is not None
    assert len(plan.surfaces) == 1
    assert sorted(site.edge_index for site in plan.surfaces[0].sites) == [10, 12]
    assert evaluate_patch_voronoi_plan(plan, width=0.5, preview=True)


def test_non_planar_topology_patch_compiles_real_planar_owner_surfaces():
    plan = compile_patch_voronoi_plan(
        _single_patch_fold_graph(), [70, 71], offset=0.01
    )
    assert plan is not None
    assert len(plan.surfaces) == 2
    assert sorted(len(surface.sites) for surface in plan.surfaces) == [1, 1]
    normals = {
        tuple(round(value, 6) for value in surface.domain.reference_normal)
        for surface in plan.surfaces
    }
    assert normals == {(0.0, -1.0, 0.0), (0.0, 0.0, 1.0)}
    assert evaluate_patch_voronoi_plan(plan, width=0.5, preview=True)


def test_planar_crop_topology_is_stable_for_reversed_normal_and_float_noise():
    base, selected = _door_opening_graph()
    perturbed = deepcopy(base)
    node = perturbed.nodes[0]
    node.normal *= -1.0
    offsets = {
        index: Vector(
            (
                ((index % 3) - 1) * 3e-6,
                (((index + 1) % 3) - 1) * 2e-6,
                0.0,
            )
        )
        for index in range(len(node.mesh_verts))
    }
    node.mesh_verts = [
        point + offsets[index]
        for index, point in enumerate(node.mesh_verts)
    ]
    node.centroid = (
        sum(node.mesh_verts, Vector((0.0, 0.0, 0.0)))
        / len(node.mesh_verts)
    )
    for loop in node.boundary_loops:
        loop.vert_cos = [
            point + offsets[vert_index]
            for vert_index, point in zip(loop.vert_indices, loop.vert_cos)
        ]
        for chain in loop.chains:
            chain.vert_cos = [
                point + offsets[vert_index]
                for vert_index, point in zip(
                    chain.vert_indices, chain.vert_cos
                )
            ]

    base_plan = compile_patch_voronoi_plan(base, selected, offset=0.01)
    perturbed_plan = compile_patch_voronoi_plan(
        perturbed, selected, offset=0.01
    )
    assert base_plan is not None
    assert perturbed_plan is not None
    for width in (0.418, 1.0, 2.5):
        base_signature = tuple(
            len(face.positions)
            for face in evaluate_patch_voronoi_plan(base_plan, width)
        )
        perturbed_signature = tuple(
            len(face.positions)
            for face in evaluate_patch_voronoi_plan(perturbed_plan, width)
        )
        assert perturbed_signature == base_signature


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
