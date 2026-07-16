from math import cos, pi, sin

import pytest
from mathutils import Vector

from cftuv.decal_charts import (
    ChartBuildFailure,
    ChartSiteSeed,
    build_intrinsic_strip_charts,
    unroll_intrinsic_strip_chart,
)
from cftuv.model import PatchNode


def _patch(positions, triangles, *, face_ids=None, patch_id=31):
    if face_ids is None:
        face_ids = list(range(100, 100 + len(triangles)))
    return PatchNode(
        patch_id=patch_id,
        face_indices=sorted(set(face_ids)),
        mesh_verts=[Vector(position) for position in positions],
        mesh_vert_indices=list(range(len(positions))),
        mesh_tris=list(triangles),
        mesh_tri_face_indices=list(face_ids),
        mesh_tri_face_normals=[Vector((0.0, 0.0, 1.0))] * len(triangles),
    )


def _seed(edge, face_id, *, edge_index=900, patch_id=31):
    return ChartSiteSeed(
        edge_index=edge_index,
        source_vertex_ids=tuple(sorted(edge)),
        source_face_id=face_id,
        chain_ref=(patch_id, 0, edge_index),
    )


def _full_chart(node, seed):
    charts = build_intrinsic_strip_charts(
        node,
        (seed,),
        alpha_budget=100.0,
    )
    assert len(charts) == 1
    return charts[0]


def _signed_area(points):
    first, second, third = points
    return (
        (second[0] - first[0]) * (third[1] - first[1])
        - (second[1] - first[1]) * (third[0] - first[0])
    )


def _unroll_signature(chart):
    return (
        chart.root_triangle_id,
        tuple(
            (
                triangle.triangle_id,
                triangle.parent_triangle_id,
                triangle.parent_source_edge,
                triangle.chart_points,
            )
            for triangle in chart.triangles
        ),
        chart.metrics,
    )


def test_c2_root_prefers_selected_site_owner_before_face_id():
    node = _patch(
        [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 0.0),
            (0.0, 1.0, 0.0),
        ],
        [(0, 1, 2), (0, 2, 3)],
        face_ids=[20, 10],
    )
    chart = _full_chart(node, _seed((0, 1), 20))

    unrolled = unroll_intrinsic_strip_chart(chart)

    assert unrolled.root_triangle_id == 0
    assert unrolled.triangles[0].parent_triangle_id == -1
    assert unrolled.triangles[1].parent_triangle_id == 0
    assert unrolled.triangles[1].parent_source_edge == (0, 2)


def test_c2_quarter_cylinder_unroll_preserves_edges_and_orientation():
    angles = tuple(index * pi / 12.0 for index in range(7))
    bottom = [(cos(angle), sin(angle), 0.0) for angle in angles]
    top = [(cos(angle), sin(angle), 1.0) for angle in angles]
    triangles = []
    face_ids = []
    row_size = len(angles)
    for index in range(row_size - 1):
        triangles.extend(
            (
                (index, index + 1, row_size + index + 1),
                (index, row_size + index + 1, row_size + index),
            )
        )
        face_ids.extend((100 + index, 100 + index))
    node = _patch(bottom + top, triangles, face_ids=face_ids)
    chart = _full_chart(node, _seed((0, row_size), 100))

    unrolled = unroll_intrinsic_strip_chart(chart)

    assert unrolled.placed_triangle_ids == unrolled.support_triangle_ids
    assert unrolled.metrics.max_edge_error < 1e-12
    assert unrolled.metrics.max_loop_closure_residual == 0.0
    assert unrolled.metrics.triangle_overlap_count == 0
    assert unrolled.metrics.chart_area_source_area_ratio == pytest.approx(1.0)
    signs = {_signed_area(triangle.chart_points) > 0.0 for triangle in unrolled.triangles}
    assert signs == {True}


def test_c2_planar_dual_cycle_measures_zero_closure_without_averaging():
    node = _patch(
        [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.5, 0.5, 0.0),
        ],
        [(0, 1, 4), (1, 2, 4), (2, 3, 4), (3, 0, 4)],
    )
    chart = _full_chart(node, _seed((0, 1), 100))

    first = unroll_intrinsic_strip_chart(chart)
    second = unroll_intrinsic_strip_chart(chart)

    assert first.metrics.max_loop_closure_residual < 1e-12
    assert first.metrics.discrete_angle_defect < 1e-12
    assert first.metrics.triangle_overlap_count == 0
    assert _unroll_signature(first) == _unroll_signature(second)


def test_c2_non_developable_cycle_reports_residual_and_angle_defect():
    node = _patch(
        [
            (1.0, 1.0, 1.0),
            (-1.0, -1.0, 1.0),
            (-1.0, 1.0, -1.0),
            (1.0, -1.0, -1.0),
        ],
        [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)],
    )
    chart = _full_chart(node, _seed((0, 2), 100))

    unrolled = unroll_intrinsic_strip_chart(chart)

    assert unrolled.metrics.max_loop_closure_residual > 1.0
    assert unrolled.metrics.discrete_angle_defect == pytest.approx(pi)
    assert unrolled.metrics.max_edge_error < 1e-12


def test_c2_non_adjacent_triangle_overlap_is_counted():
    positions = [(0.0, 0.0, 0.0)]
    positions.extend(
        (
            cos(index * pi / 3.0),
            sin(index * pi / 3.0),
            0.0,
        )
        for index in range(8)
    )
    triangles = [
        (0, index, index + 1) for index in range(1, 8)
    ]
    node = _patch(positions, triangles)
    chart = _full_chart(node, _seed((1, 2), 100))

    unrolled = unroll_intrinsic_strip_chart(chart)

    assert unrolled.metrics.triangle_overlap_count > 0
    assert unrolled.metrics.max_edge_error < 1e-12


def test_c2_zero_area_source_triangle_is_explicit_failure():
    node = _patch(
        [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)],
        [(0, 1, 2)],
    )
    chart = _full_chart(node, _seed((0, 1), 100))

    with pytest.raises(ChartBuildFailure) as captured:
        unroll_intrinsic_strip_chart(chart)

    assert captured.value.code == "ZERO_AREA_SOURCE_TRIANGLE"
    assert captured.value.patch_id == node.patch_id
    assert captured.value.triangle_ids == (0,)


def test_c2_inconsistent_source_winding_is_explicit_failure():
    node = _patch(
        [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 0.0),
            (0.0, 1.0, 0.0),
        ],
        [(0, 1, 2), (2, 0, 3)],
    )
    chart = _full_chart(node, _seed((0, 1), 100))

    with pytest.raises(ChartBuildFailure) as captured:
        unroll_intrinsic_strip_chart(chart)

    assert captured.value.code == "INCONSISTENT_CHART_ORIENTATION"
