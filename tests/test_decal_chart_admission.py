from math import cos, pi, sin

import pytest
from mathutils import Vector

from cftuv.decal_chart_admission import admit_intrinsic_strip_chart
from cftuv.decal_charts import (
    ChartBuildFailure,
    ChartSiteSeed,
    build_intrinsic_strip_charts,
)
from cftuv.model import PatchNode


def _node(
    positions,
    triangles,
    *,
    patch_id=51,
    face_ids=None,
    triangle_edge_indices=None,
):
    face_ids = face_ids or list(range(100, 100 + len(triangles)))
    return PatchNode(
        patch_id=patch_id,
        face_indices=sorted(set(face_ids)),
        mesh_verts=[Vector(position) for position in positions],
        mesh_vert_indices=list(range(len(positions))),
        mesh_tris=list(triangles),
        mesh_tri_face_indices=list(face_ids),
        mesh_tri_face_normals=[Vector((0.0, 0.0, 1.0))] * len(triangles),
        mesh_tri_edge_indices=list(triangle_edge_indices or ()),
    )


def _seed(edge, face_id, edge_index=1000):
    return ChartSiteSeed(
        edge_index=edge_index,
        source_vertex_ids=tuple(sorted(edge)),
        source_face_id=face_id,
        chain_ref=(51, 0, edge_index),
    )


def _c1(node, seeds, alpha=100.0):
    charts = build_intrinsic_strip_charts(node, seeds, alpha_budget=alpha)
    assert len(charts) == 1
    return charts[0]


def _annulus(segment_count=8, *, reverse_triangles=False):
    positions = []
    for height in (0.0, 1.0):
        positions.extend(
            (
                cos(index * 2.0 * pi / segment_count),
                sin(index * 2.0 * pi / segment_count),
                height,
            )
            for index in range(segment_count)
        )
    triangles = []
    face_ids = []
    physical_edges = set()
    for index in range(segment_count):
        next_index = (index + 1) % segment_count
        physical_edges.update(
            (
                tuple(sorted((index, next_index))),
                tuple(
                    sorted(
                        (
                            segment_count + index,
                            segment_count + next_index,
                        )
                    )
                ),
                (index, segment_count + index),
            )
        )
    edge_index_by_pair = {
        edge: 2000 + index for index, edge in enumerate(sorted(physical_edges))
    }
    for index in range(segment_count):
        next_index = (index + 1) % segment_count
        triangles.extend(
            (
                (index, next_index, segment_count + next_index),
                (index, segment_count + next_index, segment_count + index),
            )
        )
        face_ids.extend((100 + index, 100 + index))
    triangle_edge_indices = []
    for triangle in triangles:
        first, second, third = triangle
        triangle_edge_indices.append(
            (
                edge_index_by_pair.get(tuple(sorted((second, third))), -1),
                edge_index_by_pair.get(tuple(sorted((third, first))), -1),
                edge_index_by_pair.get(tuple(sorted((first, second))), -1),
            )
        )
    if reverse_triangles:
        triangles.reverse()
        face_ids.reverse()
        triangle_edge_indices.reverse()
    return _node(
        positions,
        triangles,
        face_ids=face_ids,
        triangle_edge_indices=triangle_edge_indices,
    )


def test_c3_planar_disk_is_admitted_without_cut():
    node = _node(
        [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.5, 0.5, 0.0),
        ],
        [(0, 1, 4), (1, 2, 4), (2, 3, 4), (3, 0, 4)],
    )

    admitted = admit_intrinsic_strip_chart(
        _c1(node, (_seed((0, 1), 100),))
    )

    assert admitted.cuts == ()
    assert admitted.metrics.discrete_angle_defect < 1e-12
    assert admitted.metrics.max_loop_closure_residual < 1e-12


def test_c3_annulus_gets_one_deterministic_cut_outside_sites():
    node = _annulus()
    chart = _c1(node, (_seed((0, 1), 100),))

    admitted = admit_intrinsic_strip_chart(chart)

    assert len(admitted.cuts) == 1
    assert admitted.cuts[0].reason == "OPEN_ANNULUS"
    assert admitted.cuts[0].source_edge != (0, 1)
    assert sum(
        edge.kind == "CHART_CUT" for edge in admitted.boundary_edges
    ) == 2
    assert admitted.metrics.max_loop_closure_residual < 1e-10
    assert admitted.metrics.discrete_angle_defect < 1e-10


def test_c3_annulus_cut_is_stable_under_triangle_enumeration():
    direct = _annulus()
    reversed_node = _annulus(reverse_triangles=True)

    direct_chart = admit_intrinsic_strip_chart(
        _c1(direct, (_seed((0, 1), 100),))
    )
    reversed_chart = admit_intrinsic_strip_chart(
        _c1(reversed_node, (_seed((0, 1), 100),))
    )

    assert direct_chart.cuts[0].source_edge == (
        reversed_chart.cuts[0].source_edge
    )


def test_c3_positive_curvature_rejects_non_developable_support():
    segment_count = 6
    positions = [(0.0, 0.0, 1.0)] + [
        (
            cos(index * 2.0 * pi / segment_count),
            sin(index * 2.0 * pi / segment_count),
            0.0,
        )
        for index in range(segment_count)
    ]
    triangles = [
        (0, index + 1, (index + 1) % segment_count + 1)
        for index in range(segment_count)
    ]
    node = _node(positions, triangles)

    with pytest.raises(ChartBuildFailure) as captured:
        admit_intrinsic_strip_chart(
            _c1(node, (_seed((1, 2), 100),))
        )

    assert captured.value.code == "NON_DEVELOPABLE_SUPPORT"


def test_c3_rejects_when_every_annulus_cut_crosses_selected_network():
    segment_count = 6
    node = _annulus(segment_count)
    seeds = tuple(
        _seed(
            (index, segment_count + index),
            100 + (index - 1) % segment_count,
            1100 + index,
        )
        for index in range(segment_count)
    )

    with pytest.raises(ChartBuildFailure) as captured:
        admit_intrinsic_strip_chart(_c1(node, seeds))

    assert captured.value.code == "CUT_CROSSES_SELECTED_CHAIN"


def test_c3_rejects_non_adjacent_chart_self_overlap():
    positions = [(0.0, 0.0, 0.0)] + [
        (
            cos(index * pi / 3.0),
            sin(index * pi / 3.0),
            0.0,
        )
        for index in range(8)
    ]
    triangles = [(0, index, index + 1) for index in range(1, 8)]
    node = _node(positions, triangles)

    with pytest.raises(ChartBuildFailure) as captured:
        admit_intrinsic_strip_chart(
            _c1(node, (_seed((1, 2), 100),))
        )

    assert captured.value.code == "CHART_SELF_OVERLAP"
    assert "overlaps=" in captured.value.details
