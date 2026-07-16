from dataclasses import replace

import pytest
from mathutils import Vector

from cftuv.decal_chart_admission import admit_intrinsic_strip_chart
from cftuv.decal_charts import (
    ChartCut,
    ChartSiteSeed,
    build_intrinsic_strip_charts,
)
from cftuv.decal_geometry import lift_offset_position
from cftuv.decal_voronoi import (
    _compile_intrinsic_surface,
    _intrinsic_site_points,
    build_intrinsic_surface_domain,
)
from cftuv.model import PatchNode


def _folded_patch():
    positions = (
        Vector((0.0, 0.0, 0.0)),
        Vector((1.0, 0.0, 0.0)),
        Vector((0.0, 1.0, 0.0)),
        Vector((1.0, 1.0, 1.0)),
    )
    triangles = ((0, 1, 2), (1, 3, 2))
    normals = []
    for triangle in triangles:
        first, second, third = (positions[index] for index in triangle)
        normals.append((second - first).cross(third - first).normalized())
    return PatchNode(
        patch_id=61,
        face_indices=[10, 11],
        centroid=sum(positions, Vector()) / len(positions),
        normal=(normals[0] + normals[1]).normalized(),
        mesh_verts=list(positions),
        mesh_vert_indices=[0, 1, 2, 3],
        mesh_tris=list(triangles),
        mesh_tri_face_indices=[10, 11],
        mesh_tri_face_normals=normals,
        mesh_tri_edge_indices=[(12, 13, 10), (14, 12, 15)],
    )


def _admitted_fold():
    node = _folded_patch()
    seed = ChartSiteSeed(
        edge_index=10,
        source_vertex_ids=(0, 1),
        source_face_id=10,
        chain_ref=(61, 0, 0),
    )
    chart = build_intrinsic_strip_charts(
        node, (seed,), alpha_budget=10.0
    )[0]
    return node, admit_intrinsic_strip_chart(chart)


def test_c4_chart_adapter_builds_intrinsic_domain_and_feature_maps():
    node, chart = _admitted_fold()

    domain = build_intrinsic_surface_domain(node, chart)

    assert domain.kind == "INTRINSIC"
    assert domain.chart_id == chart.chart_id
    assert domain.alpha_budget == chart.alpha_budget
    assert len(domain.boundary_triangles) == 2
    assert len(domain.intrinsic_triangles) == 2
    assert domain.triangle_grid.all_triangle_ids == (0, 1)
    assert (12, (0, 1)) in domain.source_edge_features
    assert (1, (0, 1)) in domain.source_vertex_features
    assert (("SOURCE_EDGE", 12), "EDGE", 12) in domain.transition_metadata
    shared = next(
        triangle
        for triangle in domain.intrinsic_triangles
        if 12 in triangle.source_edge_ids
    )
    local_edge = shared.source_edge_ids.index(12)
    edge_points = tuple(
        shared.chart_points[index]
        for index in range(3)
        if index != local_edge
    )
    midpoint = tuple(
        (edge_points[0][axis] + edge_points[1][axis]) * 0.5
        for axis in range(2)
    )
    location = domain.locate(midpoint)
    assert location.source_feature == "EDGE"
    assert location.source_feature_id == 12
    assert location.transition_key == ("SOURCE_EDGE", 12)


def test_d0_periodic_chart_metadata_reaches_intrinsic_domain():
    node, chart = _admitted_fold()
    cut = ChartCut(
        source_edge=(1, 2),
        triangle_ids=(0, 1),
        reason="PERIODIC_GENERATOR",
        transition_key=("periodic-cut", chart.chart_id),
        source_edge_index=12,
    )
    equivalences = (
        (
            cut.transition_key,
            (("periodic-image", -1), ("periodic-image", 1)),
        ),
    )
    chart = replace(
        chart,
        cuts=(cut,),
        periodic_axis="U",
        period=2.0,
        period_quantum=0.25,
        wrap_origin=0.0,
        periodic_cut=cut,
        transition_equivalences=equivalences,
    )

    domain = build_intrinsic_surface_domain(node, chart)

    assert domain.periodic_axis == "U"
    assert domain.period == 2.0
    assert domain.period_quantum == 0.25
    assert domain.wrap_origin == 0.0
    assert domain.periodic_cut == cut
    assert domain.transition_equivalences == equivalences


def test_c4_fold_lift_uses_shared_source_edge_normals():
    node, chart = _admitted_fold()
    domain = build_intrinsic_surface_domain(node, chart)
    shared = domain.intrinsic_triangles[0]
    local_edge = shared.source_edge_ids.index(12)
    indices = tuple(index for index in range(3) if index != local_edge)
    midpoint = tuple(
        sum(shared.chart_points[index][axis] for index in indices) * 0.5
        for axis in range(2)
    )
    location = domain.locate(midpoint)
    source = domain.source_position(location)
    expected = lift_offset_position(
        source,
        tuple(
            triangle.face_normal
            for triangle in domain.intrinsic_triangles
            if 12 in triangle.source_edge_ids
        ),
        0.1,
    )

    assert tuple(domain.lift(midpoint, 0.1, location)) == pytest.approx(
        tuple(expected)
    )


def test_c4_selected_sites_are_created_from_chart_provenance():
    node, chart = _admitted_fold()
    raw_site = {
        "edge_index": 10,
        "vert_a": 0,
        "vert_b": 1,
        "owner_face_index": 10,
    }

    points = _intrinsic_site_points(chart, (raw_site,))[10]
    owner = next(
        triangle
        for triangle in chart.triangles
        if triangle.source_face_id == 10
    )
    chart_by_vertex = dict(
        zip(owner.source_vertex_ids, owner.chart_points)
    )

    assert points == (chart_by_vertex[0], chart_by_vertex[1])


def test_c4_intrinsic_surface_uses_same_voronoi_compile_core():
    node, chart = _admitted_fold()
    raw_site = {
        "patch_id": node.patch_id,
        "edge_index": 10,
        "vert_a": 0,
        "vert_b": 1,
        "source_a": node.mesh_verts[0].copy(),
        "source_b": node.mesh_verts[1].copy(),
        "arc_start": 0.0,
        "segment_length": 1.0,
        "side_normal": node.mesh_tri_face_normals[0].copy(),
        "owner_face_index": 10,
        "uv_sign": -1.0,
        "two_sided": False,
    }

    surface = _compile_intrinsic_surface(node, chart, (raw_site,))

    assert surface.domain.kind == "INTRINSIC"
    assert len(surface.domain.intrinsic_triangles) == 2
    assert len(surface.sites) == 1
    assert surface.sites[0].edge_index == 10
    assert surface.atoms
