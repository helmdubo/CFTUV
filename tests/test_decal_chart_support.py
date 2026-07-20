from math import cos, pi, sin

import pytest
from mathutils import Vector

from cftuv.decal_charts import (
    ChartBuildFailure,
    ChartSiteSeed,
    build_intrinsic_strip_charts,
    build_triangle_adjacency,
    chart_triangles_from_patch,
)
from cftuv.model import PatchNode


def _strip_patch(stations, *, patch_id=1, vertex_id_start=0):
    """Строит triangulated strip между двумя рядами заданных stations."""

    row_size = len(stations)
    positions = [
        Vector((x, y, 0.0))
        for x, y in stations
    ] + [
        Vector((x, y, 1.0))
        for x, y in stations
    ]
    triangles = []
    face_ids = []
    for station in range(row_size - 1):
        bottom = station
        next_bottom = station + 1
        top = row_size + station
        next_top = row_size + station + 1
        triangles.extend(
            (
                (bottom, next_bottom, next_top),
                (bottom, next_top, top),
            )
        )
        face_ids.extend((100 + station, 100 + station))
    return PatchNode(
        patch_id=patch_id,
        face_indices=sorted(set(face_ids)),
        mesh_verts=positions,
        mesh_vert_indices=[
            vertex_id_start + index for index in range(len(positions))
        ],
        mesh_tris=triangles,
        mesh_tri_face_indices=face_ids,
        mesh_tri_face_normals=[
            Vector((0.0, -1.0, 0.0)) for _triangle in triangles
        ],
    )


def _end_seed(node, *, left, edge_index):
    row_size = len(node.mesh_verts) // 2
    station = 0 if left else row_size - 1
    bottom = node.mesh_vert_indices[station]
    top = node.mesh_vert_indices[row_size + station]
    face_offset = 0 if left else len(node.mesh_tri_face_indices) - 2
    triangle_offset = face_offset + (1 if left else 0)
    return ChartSiteSeed(
        edge_index=edge_index,
        source_vertex_ids=tuple(sorted((bottom, top))),
        source_face_id=node.mesh_tri_face_indices[triangle_offset],
        chain_ref=(node.patch_id, 0, edge_index),
    )


def _chart_signature(charts):
    return tuple(
        (
            chart.chart_id,
            chart.patch_id,
            chart.support_triangle_ids,
            tuple(seed.edge_index for seed in chart.site_seeds),
            tuple(
                (
                    edge.triangle_id,
                    edge.local_edge,
                    edge.source_edge,
                    edge.source_face_id,
                    edge.neighbor_triangle_id,
                    edge.kind,
                )
                for edge in chart.boundary_edges
            ),
        )
        for chart in charts
    )


def test_c1_triangle_adjacency_uses_shared_undirected_source_edges():
    node = _strip_patch([(0.0, 0.0), (1.0, 0.0)])
    triangles = chart_triangles_from_patch(node)

    adjacency = build_triangle_adjacency(triangles, patch_id=node.patch_id)

    assert len(adjacency) == 1
    relation = adjacency[0]
    assert (relation.triangle_a, relation.triangle_b) == (0, 1)
    assert relation.source_edge == (0, 3)
    assert (
        triangles[0].source_edge_ids[relation.local_edge_a]
        == relation.source_edge
    )
    assert (
        triangles[1].source_edge_ids[relation.local_edge_b]
        == relation.source_edge
    )


def test_c1_non_manifold_triangle_edge_is_explicit_failure():
    node = PatchNode(
        patch_id=41,
        face_indices=[1, 2, 3],
        mesh_verts=[
            Vector((0.0, 0.0, 0.0)),
            Vector((1.0, 0.0, 0.0)),
            Vector((0.0, 1.0, 0.0)),
            Vector((0.0, -1.0, 0.0)),
            Vector((0.0, 0.0, 1.0)),
        ],
        mesh_vert_indices=[10, 20, 30, 40, 50],
        mesh_tris=[(0, 1, 2), (1, 0, 3), (0, 1, 4)],
        mesh_tri_face_indices=[1, 2, 3],
        mesh_tri_face_normals=[Vector((0.0, 0.0, 1.0))] * 3,
    )

    with pytest.raises(ChartBuildFailure) as captured:
        build_triangle_adjacency(
            chart_triangles_from_patch(node),
            patch_id=node.patch_id,
        )

    assert captured.value.code == "NON_MANIFOLD_CHART_EDGE"
    assert captured.value.patch_id == 41
    assert captured.value.triangle_ids == (0, 1, 2)


def test_c1_straight_bevel_support_never_drops_band_intersections():
    node = _strip_patch(
        [(0.0, 0.0), (1.0, 0.0), (1.7, 0.7), (1.7, 1.7)]
    )
    seed = _end_seed(node, left=True, edge_index=700)

    chart = build_intrinsic_strip_charts(
        node, (seed,), alpha_budget=1.05
    )[0]

    # Оба первых quads касаются geodesic band: второй начинается на s=1.
    assert {0, 1, 2, 3}.issubset(chart.support_triangle_ids)
    assert chart.metrics.support_triangle_count == len(
        chart.support_triangle_ids
    )
    assert chart.metrics.boundary_edge_count == len(chart.boundary_edges)


def test_c1_open_quarter_cylinder_keeps_arc_band_conservatively():
    angles = (0.0, pi / 6.0, pi / 3.0, pi / 2.0)
    node = _strip_patch([(cos(angle), sin(angle)) for angle in angles])
    seed = _end_seed(node, left=True, edge_index=701)

    chart = build_intrinsic_strip_charts(
        node, (seed,), alpha_budget=0.55
    )[0]

    # R=1: второй sector начинается на arc length pi/6 < 0.55.
    assert {0, 1, 2, 3}.issubset(chart.support_triangle_ids)
    assert chart.adjacency == tuple(sorted(chart.adjacency, key=lambda item: item.key))


def test_c1_selected_network_supports_merge_only_when_they_intersect():
    node = _strip_patch([(float(index), 0.0) for index in range(9)])
    seeds = (
        _end_seed(node, left=True, edge_index=710),
        _end_seed(node, left=False, edge_index=720),
    )

    separate = build_intrinsic_strip_charts(
        node, tuple(reversed(seeds)), alpha_budget=0.1
    )
    merged = build_intrinsic_strip_charts(
        node, seeds, alpha_budget=3.1
    )

    assert len(separate) == 2
    assert tuple(len(chart.site_seeds) for chart in separate) == (1, 1)
    assert not set(separate[0].support_triangle_ids).intersection(
        separate[1].support_triangle_ids
    )
    assert len(merged) == 1
    assert tuple(seed.edge_index for seed in merged[0].site_seeds) == (
        710,
        720,
    )


def test_c1_support_boundary_keeps_patch_and_cut_provenance():
    node = _strip_patch([(float(index), 0.0) for index in range(7)])
    seed = _end_seed(node, left=True, edge_index=730)

    chart = build_intrinsic_strip_charts(
        node, (seed,), alpha_budget=0.1
    )[0]
    triangles = {
        triangle.triangle_id: triangle for triangle in chart.triangles
    }

    assert {edge.kind for edge in chart.boundary_edges} == {
        "PATCH_BOUNDARY",
        "SUPPORT_CUT",
    }
    for edge in chart.boundary_edges:
        triangle = triangles[edge.triangle_id]
        assert edge.source_face_id == triangle.source_face_id
        assert triangle.source_edge_ids[edge.local_edge] == edge.source_edge


def test_c1_nearby_geometry_in_another_patch_cannot_enter_support():
    stations = [(float(index), 0.0) for index in range(5)]
    target = _strip_patch(stations, patch_id=11, vertex_id_start=0)
    nearby = _strip_patch(stations, patch_id=12, vertex_id_start=1000)
    nearby.mesh_verts = [
        Vector((position.x, position.y + 0.01, position.z))
        for position in nearby.mesh_verts
    ]

    chart = build_intrinsic_strip_charts(
        target,
        (_end_seed(target, left=True, edge_index=740),),
        alpha_budget=2.0,
    )[0]

    assert chart.patch_id == target.patch_id
    assert all(
        source_vertex_id < 1000
        for triangle in chart.triangles
        for source_vertex_id in triangle.source_vertex_ids
    )
    assert not {
        source_vertex_id
        for triangle in chart.triangles
        for source_vertex_id in triangle.source_vertex_ids
    }.intersection(nearby.mesh_vert_indices)


def test_c1_support_build_is_deterministic_for_seed_order():
    node = _strip_patch([(float(index), 0.0) for index in range(9)])
    seeds = (
        _end_seed(node, left=True, edge_index=750),
        _end_seed(node, left=False, edge_index=760),
    )

    first = build_intrinsic_strip_charts(
        node, seeds, alpha_budget=0.1, chart_id_start=5
    )
    second = build_intrinsic_strip_charts(
        node, tuple(reversed(seeds)), alpha_budget=0.1, chart_id_start=5
    )

    assert _chart_signature(first) == _chart_signature(second)
