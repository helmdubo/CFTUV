from math import atan2, cos, pi, sin

import pytest
from mathutils import Vector

from cftuv.decal_chart_admission import admit_intrinsic_strip_chart
from cftuv.decal_charts import ChartSiteSeed, build_intrinsic_strip_charts
from cftuv.decal_voronoi import (
    compile_patch_voronoi_plan,
    evaluate_patch_voronoi_plan,
    serialize_network_faces,
)
from cftuv.model import BoundaryChain, BoundaryLoop, PatchGraph, PatchNode


def _curved_strip_graph(segment_count, *, arc=pi / 2.0):
    """Developable cylinder strip с source provenance на каждом facet."""

    angles = tuple(
        arc * index / segment_count for index in range(segment_count + 1)
    )
    row_size = len(angles)
    positions = [
        Vector((cos(angle), sin(angle), 0.0)) for angle in angles
    ] + [
        Vector((cos(angle), sin(angle), 1.0)) for angle in angles
    ]
    triangles = []
    face_ids = []
    face_normals = []
    physical_edges = set()
    for index in range(segment_count):
        next_index = index + 1
        physical_edges.update(
            (
                tuple(sorted((index, next_index))),
                tuple(
                    sorted((row_size + index, row_size + next_index))
                ),
                (index, row_size + index),
                (next_index, row_size + next_index),
            )
        )
        triangles.extend(
            (
                (index, next_index, row_size + next_index),
                (index, row_size + next_index, row_size + index),
            )
        )
        face_ids.extend((100 + index, 100 + index))
        middle = (angles[index] + angles[next_index]) * 0.5
        normal = Vector((cos(middle), sin(middle), 0.0))
        face_normals.extend((normal, normal.copy()))
    edge_index_by_pair = {
        pair: 5000 + index
        for index, pair in enumerate(sorted(physical_edges))
    }
    triangle_edge_indices = []
    for first, second, third in triangles:
        triangle_edge_indices.append(
            (
                edge_index_by_pair.get(tuple(sorted((second, third))), -1),
                edge_index_by_pair.get(tuple(sorted((third, first))), -1),
                edge_index_by_pair.get(tuple(sorted((first, second))), -1),
            )
        )

    left_pair = (0, row_size)
    right_pair = (row_size - 1, row_size * 2 - 1)
    left_edge = edge_index_by_pair[left_pair]
    right_edge = edge_index_by_pair[right_pair]
    node = PatchNode(
        patch_id=61,
        face_indices=sorted(set(face_ids)),
        centroid=sum(positions, Vector((0.0, 0.0, 0.0))) / len(positions),
        normal=Vector((cos(arc * 0.5), sin(arc * 0.5), 0.0)),
        basis_u=Vector((0.0, 0.0, 1.0)),
        basis_v=Vector((-sin(arc * 0.5), cos(arc * 0.5), 0.0)),
        mesh_verts=positions,
        mesh_vert_indices=list(range(len(positions))),
        mesh_tris=triangles,
        mesh_tri_face_indices=face_ids,
        mesh_tri_face_normals=face_normals,
        mesh_tri_edge_indices=triangle_edge_indices,
    )
    node.boundary_loops = [
        BoundaryLoop(
            chains=[
                BoundaryChain(
                    vert_indices=list(left_pair),
                    vert_cos=[positions[index] for index in left_pair],
                    edge_indices=[left_edge],
                    side_face_indices=[face_ids[0]],
                    side_face_normals=[face_normals[0]],
                ),
                BoundaryChain(
                    vert_indices=list(right_pair),
                    vert_cos=[positions[index] for index in right_pair],
                    edge_indices=[right_edge],
                    side_face_indices=[face_ids[-1]],
                    side_face_normals=[face_normals[-1]],
                ),
            ]
        )
    ]
    graph = PatchGraph()
    graph.add_node(node)
    return graph, node, left_edge, right_edge


def _mixed_backend_junction_graph(segment_count=8):
    """Арка INTRINSIC встречает PLANAR-ленту в одной source-вершине."""

    graph, _curved, curved_edge, _right_edge = _curved_strip_graph(
        segment_count
    )
    planar_edge = 9000
    positions = [
        Vector((1.0, 0.0, 0.0)),
        Vector((3.0, 0.0, 0.0)),
        Vector((3.0, 2.0, 0.0)),
        Vector((1.0, 2.0, 0.0)),
    ]
    normal = Vector((0.0, 0.0, 1.0))
    node = PatchNode(
        patch_id=62,
        face_indices=[900],
        centroid=sum(positions, Vector((0.0, 0.0, 0.0))) / 4.0,
        normal=normal,
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
        mesh_verts=positions,
        # Вершина 0 общая с endpoint цилиндрической ленты.
        mesh_vert_indices=[0, 100, 101, 102],
        mesh_tris=[(0, 1, 2), (0, 2, 3)],
        mesh_tri_face_indices=[900, 900],
        mesh_tri_face_normals=[normal.copy(), normal.copy()],
        mesh_tri_edge_indices=[
            (-1, -1, planar_edge),
            (-1, -1, -1),
        ],
    )
    node.boundary_loops = [
        BoundaryLoop(
            chains=[
                BoundaryChain(
                    vert_indices=[0, 100],
                    vert_cos=positions[:2],
                    edge_indices=[planar_edge],
                    side_face_indices=[900],
                    side_face_normals=[normal.copy()],
                )
            ]
        )
    ]
    graph.add_node(node)
    return graph, curved_edge, planar_edge


def _seed(node, edge_index, pair, face_id):
    return ChartSiteSeed(
        edge_index=edge_index,
        source_vertex_ids=tuple(sorted(pair)),
        source_face_id=face_id,
        chain_ref=(node.patch_id, 0, edge_index),
    )


def _assert_manifold_connected_faces(faces, *, component_count=1):
    assert faces
    edge_owners = {}
    for face_index, face in enumerate(faces):
        assert len(face.positions) >= 3
        root = face.positions[0]
        area = sum(
            (face.positions[index] - root)
            .cross(face.positions[index + 1] - root)
            .length
            * 0.5
            for index in range(1, len(face.positions) - 1)
        )
        assert area > 1e-10
        for index, key in enumerate(face.vert_keys):
            other = face.vert_keys[(index + 1) % len(face.vert_keys)]
            edge = tuple(sorted((repr(key), repr(other))))
            edge_owners.setdefault(edge, []).append(face_index)
    assert max(map(len, edge_owners.values())) <= 2

    neighbours = [set() for _face in faces]
    for owners in edge_owners.values():
        if len(owners) == 2:
            first, second = owners
            neighbours[first].add(second)
            neighbours[second].add(first)
    unseen = set(range(len(faces)))
    actual_component_count = 0
    while unseen:
        actual_component_count += 1
        frontier = [unseen.pop()]
        while frontier:
            for neighbour in neighbours[frontier.pop()]:
                if neighbour in unseen:
                    unseen.remove(neighbour)
                    frontier.append(neighbour)
    assert actual_component_count == component_count


def test_c7_f2_multiface_bevel_is_one_intrinsic_chart_without_connectors():
    graph, _node, left_edge, _right_edge = _curved_strip_graph(
        5, arc=pi / 3.0
    )

    plan = compile_patch_voronoi_plan(
        graph, (left_edge,), offset=0.01, alpha_budget=1.2
    )

    assert plan is not None
    assert plan.backend_kind == "INTRINSIC_DEVELOPABLE"
    assert len(plan.surfaces) == 1
    assert plan.surfaces[0].domain.kind == "INTRINSIC"
    faces = evaluate_patch_voronoi_plan(plan, width=0.4, preview=True)
    _assert_manifold_connected_faces(faces)
    assert all(face.component_kind != "JUNCTION" for face in faces)
    assert serialize_network_faces(faces) == serialize_network_faces(
        evaluate_patch_voronoi_plan(plan, width=0.4, preview=False)
    )


def test_c7_f3_quarter_cylinder_admission_preserves_developable_metric():
    graph, node, left_edge, _right_edge = _curved_strip_graph(16)
    row_size = len(node.mesh_verts) // 2
    seed = _seed(node, left_edge, (0, row_size), 100)

    chart = admit_intrinsic_strip_chart(
        build_intrinsic_strip_charts(
            node, (seed,), alpha_budget=2.0
        )[0]
    )

    assert chart.metrics.discrete_angle_defect <= 1e-4
    assert chart.metrics.max_loop_closure_residual == pytest.approx(0.0)
    assert chart.metrics.max_edge_error <= 1e-5
    assert chart.metrics.chart_area_source_area_ratio == pytest.approx(
        1.0, rel=1e-3
    )
    assert chart.metrics.max_width_error_sampled <= 0.01
    assert chart.metrics.foldover_count == 0
    plan = compile_patch_voronoi_plan(
        graph, (left_edge,), offset=0.01, alpha_budget=0.5
    )
    faces = evaluate_patch_voronoi_plan(plan, width=0.4, preview=True)
    # R=1: geodesic half-width совпадает с angular travel. Offset меняет
    # радиус позиции, но не её angle; допуск oracle — 1% ширины.
    max_angle = max(
        atan2(position.y, position.x)
        for face in faces
        for position in face.positions
    )
    assert max_angle == pytest.approx(0.2, rel=0.01)


def test_c7_f5_dense_fillet_support_stays_proportional_to_strip():
    _graph, node, left_edge, _right_edge = _curved_strip_graph(96)
    row_size = len(node.mesh_verts) // 2
    seed = _seed(node, left_edge, (0, row_size), 100)

    chart = build_intrinsic_strip_charts(
        node, (seed,), alpha_budget=0.05
    )[0]
    admitted = admit_intrinsic_strip_chart(chart, initial_alpha=0.05)

    assert 0 < admitted.metrics.support_triangle_count < len(node.mesh_tris) // 2
    assert admitted.metrics.discrete_angle_defect <= 1e-4
    assert admitted.metrics.triangle_overlap_count == 0


def test_c7_f6_curved_competition_merges_and_remains_manifold():
    graph, node, left_edge, right_edge = _curved_strip_graph(24)
    row_size = len(node.mesh_verts) // 2
    seeds = (
        _seed(node, left_edge, (0, row_size), 100),
        _seed(
            node,
            right_edge,
            (row_size - 1, row_size * 2 - 1),
            100 + 23,
        ),
    )
    charts = build_intrinsic_strip_charts(
        node, tuple(reversed(seeds)), alpha_budget=0.9
    )
    assert len(charts) == 1
    assert tuple(seed.edge_index for seed in charts[0].site_seeds) == tuple(
        sorted((left_edge, right_edge))
    )

    plan = compile_patch_voronoi_plan(
        graph,
        (left_edge, right_edge),
        offset=0.01,
        alpha_budget=0.9,
    )
    assert plan is not None
    assert len(plan.surfaces) == 1
    assert len(plan.surfaces[0].sites) == 2
    for width, component_count in ((0.2, 2), (1.75, 1)):
        faces = evaluate_patch_voronoi_plan(plan, width, preview=True)
        _assert_manifold_connected_faces(
            faces, component_count=component_count
        )
        assert all(face.component_kind != "JUNCTION" for face in faces)


@pytest.mark.parametrize("width", (0.74, 1.27, 1.61))
def test_c8_5_mixed_backend_junction_closes_both_open_rails(width):
    graph, curved_edge, planar_edge = _mixed_backend_junction_graph()
    plan = compile_patch_voronoi_plan(
        graph,
        (curved_edge, planar_edge),
        offset=0.01,
        alpha_budget=1.0,
    )

    assert plan.backend_kind == "PLANAR+INTRINSIC_DEVELOPABLE"
    assert {surface.domain.kind for surface in plan.surfaces} == {
        "PLANAR",
        "INTRINSIC",
    }
    faces = evaluate_patch_voronoi_plan(plan, width, preview=True)
    _assert_manifold_connected_faces(faces)
    connectors = [
        face
        for face in faces
        if face.surface_id == -1 and ("pv-sv", 0) in face.vert_keys
    ]
    assert len(connectors) == 1

    edge_uses = {}
    for face in faces:
        for index, key_a in enumerate(face.vert_keys):
            key_b = face.vert_keys[(index + 1) % len(face.vert_keys)]
            edge_key = tuple(sorted((key_a, key_b), key=repr))
            edge_uses[edge_key] = edge_uses.get(edge_key, 0) + 1
    connector = connectors[0]
    core_key = ("pv-sv", 0)
    for outer_key in connector.vert_keys:
        if outer_key == core_key:
            continue
        edge_key = tuple(sorted((core_key, outer_key), key=repr))
        assert edge_uses[edge_key] == 2
    assert serialize_network_faces(faces) == serialize_network_faces(
        evaluate_patch_voronoi_plan(plan, width, preview=False)
    )


def _g1_source_edge_token(key):
    if (
        isinstance(key, tuple)
        and len(key) >= 3
        and key[:2] == ("pv-feature", "TRANSITION")
        and isinstance(key[2], tuple)
        and key[2][:1] == ("SOURCE_EDGE",)
    ):
        return int(key[2][1])
    return None


def _g1_source_vertex_token(key):
    if (
        isinstance(key, tuple)
        and len(key) >= 3
        and key[:2] == ("pv-feature", "TRANSITION")
        and isinstance(key[2], tuple)
        and key[2][:1] == ("SOURCE_VERTEX",)
    ):
        return int(key[2][1])
    return None


def _g1_imprint_records(faces, alpha, source_edge_by_vertices):
    edge_owners = {}
    for face_index, face in enumerate(faces):
        for point_index, key_a in enumerate(face.vert_keys):
            next_index = (point_index + 1) % len(face.vert_keys)
            key_b = face.vert_keys[next_index]
            edge_key = tuple(sorted((repr(key_a), repr(key_b))))
            edge_owners.setdefault(edge_key, []).append(
                (
                    face_index,
                    key_a,
                    key_b,
                    face.u_fracs[point_index] * alpha,
                    face.v_lengths[point_index],
                    face.u_fracs[next_index] * alpha,
                    face.v_lengths[next_index],
                )
            )
    records = {}
    for owners in edge_owners.values():
        if len(owners) != 2:
            continue
        edge_id = next(
            (
                token
                for token in (
                    _g1_source_edge_token(owners[0][1]),
                    _g1_source_edge_token(owners[0][2]),
                )
                if token is not None
            ),
            None,
        )
        if edge_id is None:
            source_vertices = frozenset(
                token
                for token in (
                    _g1_source_vertex_token(owners[0][1]),
                    _g1_source_vertex_token(owners[0][2]),
                )
                if token is not None
            )
            edge_id = source_edge_by_vertices.get(source_vertices)
        if edge_id is None:
            continue
        # Одна и та же canonical station обязана иметь одинаковый UV
        # с обеих сторон геометрического imprint, без UV-шва.
        uv_by_key = []
        for owner in owners:
            uv_by_key.append(
                {
                    owner[1]: (owner[3], owner[4]),
                    owner[2]: (owner[5], owner[6]),
                }
            )
        assert set(uv_by_key[0]) == set(uv_by_key[1])
        for key in uv_by_key[0]:
            assert uv_by_key[0][key] == pytest.approx(
                uv_by_key[1][key], abs=1e-6
            )
        records.setdefault(edge_id, []).append(owners)
    return records


def test_g1_curved_strip_imprints_crossed_source_folds_with_static_keys():
    graph, _node, left_edge, _right_edge = _curved_strip_graph(
        5, arc=pi / 3.0
    )
    plan = compile_patch_voronoi_plan(
        graph, (left_edge,), offset=0.01, alpha_budget=0.7
    )
    surface = plan.surfaces[0]

    assert surface.domain.kind == "INTRINSIC"
    assert len(set(surface.domain.triangle_merge_groups)) == 5
    source_edge_by_vertices = {}
    for triangle in surface.domain.intrinsic_triangles:
        for local_edge, edge_id in enumerate(triangle.source_edge_ids):
            if not isinstance(edge_id, int):
                continue
            endpoints = frozenset(
                triangle.source_vertex_ids[index]
                for index in range(3)
                if index != local_edge
            )
            source_edge_by_vertices[endpoints] = edge_id
    previous_imprints = set()
    for width, expected_count in ((0.4, 0), (0.8, 1), (1.2, 2)):
        alpha = width * 0.5
        faces = evaluate_patch_voronoi_plan(plan, width, preview=True)
        _assert_manifold_connected_faces(faces)
        records = _g1_imprint_records(
            faces, alpha, source_edge_by_vertices
        )
        imprint_ids = set(records)

        assert len(imprint_ids) == expected_count
        assert previous_imprints <= imprint_ids
        # S1: supporting curve задаётся compile-static source edge id;
        # при drag она только появляется/удлиняется, но не переезжает.
        previous_imprints = imprint_ids
