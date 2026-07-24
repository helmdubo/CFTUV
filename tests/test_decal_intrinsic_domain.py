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
    PatchVoronoiPlan,
    PatchVoronoiDiagnostics,
    _compile_intrinsic_surface,
    _intrinsic_site_points,
    build_intrinsic_surface_domain,
    evaluate_patch_voronoi_plan,
    serialize_network_faces,
    _periodic_transport_raw_sites,
)
from cftuv.model import PatchNode

from analysis_surface_fixtures import attach_patch_surface

_build_intrinsic_strip_charts = build_intrinsic_strip_charts


def build_intrinsic_strip_charts(node, *args, **kwargs):
    kwargs["patch_surface"] = node._test_patch_surface
    return _build_intrinsic_strip_charts(node, *args, **kwargs)


def _periodic_annulus(segment_count=8):
    from math import cos, pi, sin

    positions = []
    for height in (0.0, 1.0):
        positions.extend(
            Vector(
                (
                    cos(index * 2.0 * pi / segment_count),
                    sin(index * 2.0 * pi / segment_count),
                    height,
                )
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
        triangles.extend(
            (
                (index, next_index, segment_count + next_index),
                (index, segment_count + next_index, segment_count + index),
            )
        )
        face_ids.extend((100 + index, 100 + index))
    edge_indices = {
        edge: 2000 + index for index, edge in enumerate(sorted(physical_edges))
    }
    triangle_edge_indices = []
    for first, second, third in triangles:
        triangle_edge_indices.append(
            (
                edge_indices.get(tuple(sorted((second, third))), -1),
                edge_indices.get(tuple(sorted((third, first))), -1),
                edge_indices.get(tuple(sorted((first, second))), -1),
            )
        )
    node = PatchNode(
        patch_id=71,
        face_indices=sorted(set(face_ids)),
        centroid=sum(positions, Vector()) / len(positions),
        normal=Vector((1.0, 0.0, 0.0)),
        basis_u=Vector((0.0, 1.0, 0.0)),
        basis_v=Vector((0.0, 0.0, 1.0)),
    )
    attach_patch_surface(
        node,
        mesh_verts=positions,
        mesh_vert_indices=list(range(len(positions))),
        mesh_tris=triangles,
        mesh_tri_face_indices=face_ids,
        mesh_tri_face_normals=[Vector((1.0, 0.0, 0.0))] * len(triangles),
        mesh_tri_edge_indices=triangle_edge_indices,
    )
    selected_source_edge = (0, 1)
    selected_edge_index = edge_indices[selected_source_edge]
    seed = ChartSiteSeed(
        edge_index=selected_edge_index,
        source_vertex_ids=selected_source_edge,
        source_face_id=100,
        chain_ref=(node.patch_id, 0, 0),
    )
    chart = admit_intrinsic_strip_chart(
        build_intrinsic_strip_charts(
            node, (seed,), alpha_budget=100.0
        )[0]
    )
    raw_site = {
        "patch_id": node.patch_id,
        "edge_index": selected_edge_index,
        "vert_a": 0,
        "vert_b": 1,
        "source_a": positions[0].copy(),
        "source_b": positions[1].copy(),
        "arc_start": 0.0,
        "segment_length": (positions[1] - positions[0]).length,
        "side_normal": Vector((1.0, 0.0, 0.0)),
        "owner_face_index": 100,
        "uv_sign": -1.0,
        "two_sided": False,
    }
    return node, chart, raw_site


def _source_edge_index(node, source_edge):
    source_edge = tuple(sorted(source_edge))
    for triangle, edge_indices in zip(
        node.mesh_tris, node.mesh_tri_edge_indices
    ):
        for local_edge, edge_index in enumerate(edge_indices):
            opposite = tuple(
                sorted(
                    (
                        triangle[(local_edge + 1) % 3],
                        triangle[(local_edge + 2) % 3],
                    )
                )
            )
            if opposite == source_edge:
                return edge_index
    raise AssertionError(f"missing source edge {source_edge!r}")


def _periodic_ring_sites(node, segment_count=8):
    result = []
    for index in range(segment_count):
        next_index = (index + 1) % segment_count
        source_edge = tuple(sorted((index, next_index)))
        edge_index = _source_edge_index(node, source_edge)
        result.append(
            {
                "patch_id": node.patch_id,
                "edge_index": edge_index,
                "vert_a": index,
                "vert_b": next_index,
                "source_a": node.mesh_verts[index].copy(),
                "source_b": node.mesh_verts[next_index].copy(),
                "arc_start": 0.0,
                "segment_length": (
                    node.mesh_verts[next_index] - node.mesh_verts[index]
                ).length,
                "side_normal": Vector((0.0, 0.0, -1.0)),
                "owner_face_index": 100 + index,
                "uv_sign": -1.0,
                "two_sided": False,
            }
        )
    return tuple(result)


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
    node = PatchNode(
        patch_id=61,
        face_indices=[10, 11],
        centroid=sum(positions, Vector()) / len(positions),
        normal=(normals[0] + normals[1]).normalized(),
    )
    return attach_patch_surface(
        node,
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


@pytest.mark.requires_pyvoronoi
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


@pytest.mark.requires_pyvoronoi
def test_d2_periodic_sites_are_copied_only_inside_diagram():
    node, chart, raw_site = _periodic_annulus()
    diagnostics = PatchVoronoiDiagnostics()

    surface = _compile_intrinsic_surface(
        node, chart, (raw_site,), diagnostics
    )

    assert surface.domain.periodic_axis == "U"
    assert diagnostics.periodic_copy_count > 0
    assert len(surface.sites) == 1
    assert {atom.site_index for atom in surface.atoms} == {0}
    plan = PatchVoronoiPlan(
        offset=0.01,
        surfaces=(surface,),
        lifted_vertices={
            0: raw_site["source_a"] + raw_site["side_normal"] * 0.01,
            1: raw_site["source_b"] + raw_site["side_normal"] * 0.01,
        },
        max_lateral_lift_ratio=0.0,
        alpha_budget=chart.alpha_budget,
        budget_source=chart.budget_source,
        requested_alpha_budget=100.0,
    )
    for width in (0.1, chart.period):
        first = evaluate_patch_voronoi_plan(plan, width, preview=True)
        second = evaluate_patch_voronoi_plan(plan, width, preview=False)
        assert first
        assert serialize_network_faces(first) == serialize_network_faces(
            second
        )
        identities = [frozenset(face.vert_keys) for face in first]
        assert len(identities) == len(set(identities))


def test_d3_periodic_cycle_has_one_monotonic_v_transport_direction():
    node, _chart, _raw_site = _periodic_annulus()
    raw_sites = _periodic_ring_sites(node)

    transported = _periodic_transport_raw_sites(tuple(reversed(raw_sites)))

    intervals = []
    for raw in transported:
        start = float(raw["arc_start"])
        end = start + float(raw["arc_sign"]) * raw["segment_length"]
        intervals.append(tuple(sorted((start, end))))
    intervals.sort()
    for first, second in zip(intervals, intervals[1:]):
        assert first[1] == pytest.approx(second[0])
    circumference = sum(raw["segment_length"] for raw in raw_sites)
    assert intervals[0][0] == pytest.approx(0.0)
    assert intervals[-1][1] == pytest.approx(circumference)


@pytest.mark.requires_pyvoronoi
def test_d3_periodic_images_use_shifted_crops_and_transition_welds():
    node, chart, _raw_site = _periodic_annulus()
    raw_sites = _periodic_ring_sites(node)
    diagnostics = PatchVoronoiDiagnostics()
    surface = _compile_intrinsic_surface(
        node, chart, raw_sites, diagnostics
    )
    plan = PatchVoronoiPlan(
        offset=0.01,
        surfaces=(surface,),
        lifted_vertices={
            index: position.copy()
            for index, position in enumerate(node.mesh_verts)
        },
        max_lateral_lift_ratio=0.0,
        alpha_budget=chart.alpha_budget,
        budget_source=chart.budget_source,
        requested_alpha_budget=100.0,
    )

    preview = evaluate_patch_voronoi_plan(
        plan, chart.period, preview=True, diagnostics=diagnostics
    )
    confirm = evaluate_patch_voronoi_plan(
        plan, chart.period, preview=False
    )

    assert diagnostics.periodic_copy_count > 0
    assert diagnostics.periodic_weld_count > 0
    assert serialize_network_faces(preview) == serialize_network_faces(confirm)
    assert len({frozenset(face.vert_keys) for face in preview}) == len(preview)
    intervals = sorted(
        (
            min(site.arc_start, site.arc_start + site.arc_sign * site.uv_length),
            max(site.arc_start, site.arc_start + site.arc_sign * site.uv_length),
        )
        for site in surface.sites
    )
    assert intervals[0][0] == pytest.approx(0.0)
    assert intervals[-1][1] == pytest.approx(
        sum(raw["segment_length"] for raw in raw_sites)
    )
