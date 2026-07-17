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
