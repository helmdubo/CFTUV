import pytest

from cftuv.decal_atlas import build_intrinsic_strip_atlas
from cftuv.decal_chart_admission import _prepare_disk_topology
from cftuv.decal_charts import (
    ChartBuildFailure,
    build_intrinsic_strip_charts,
    chart_triangle_overlap_pairs,
    unroll_intrinsic_strip_chart,
)
from cftuv.decal_voronoi import (
    PatchVoronoiDiagnostics,
    compile_patch_voronoi_plan,
    evaluate_patch_voronoi_plan,
    serialize_network_faces,
)
from cftuv.model import PatchGraph

from decal_organic_fixtures import crumple_fixture, saddle_fixture


def _unrolled(fixture):
    node, seeds, alpha = fixture
    chart = build_intrinsic_strip_charts(node, seeds, alpha)[0]
    return unroll_intrinsic_strip_chart(
        _prepare_disk_topology(chart), edge_relative_tolerance=1e-5
    )


def test_e3_saddle_atlas_is_locally_injective_and_owns_each_triangle_once():
    chart = _unrolled(saddle_fixture())
    atlas = build_intrinsic_strip_atlas(chart)

    assert atlas.atlas_chart_count > 1
    assert atlas.separator_iterations <= 8
    assert atlas.interior_transition_count > 0
    assert all(not chart_triangle_overlap_pairs(item) for item in atlas.charts)
    assert all(
        cut.reason == "ATLAS_SEPARATOR"
        for item in atlas.charts
        for cut in item.cuts
    )
    assert tuple(triangle_id for triangle_id, _owner in atlas.triangle_owners) == (
        chart.support_triangle_ids
    )


def test_e3_atlas_is_deterministic_under_repeat_compile():
    chart = _unrolled(saddle_fixture())
    assert build_intrinsic_strip_atlas(chart) == build_intrinsic_strip_atlas(chart)


def test_e3_transition_maps_shared_edge_and_has_canonical_owner():
    atlas = build_intrinsic_strip_atlas(_unrolled(saddle_fixture()))
    charts = {item.chart_id: item for item in atlas.charts}
    transition = atlas.transitions[0]
    owner = charts[transition.owner_chart_id]
    neighbor = charts[transition.neighbor_chart_id]
    assert transition.owner_chart_id < transition.neighbor_chart_id

    def edge_points(chart, triangle_id):
        triangle = next(
            item for item in chart.triangles
            if item.triangle_id == triangle_id
        )
        points = dict(zip(triangle.source_vertex_ids, triangle.chart_points))
        return tuple(points[index] for index in transition.source_edge)

    owner_points = edge_points(owner, transition.owner_triangle_id)
    neighbor_points = edge_points(neighbor, transition.neighbor_triangle_id)
    for actual, expected in zip(
        (transition.owner_to_neighbor(point) for point in owner_points),
        neighbor_points,
    ):
        assert actual == pytest.approx(expected)


def test_e3_crumple_rejects_at_iteration_limit_without_partial_atlas():
    chart = _unrolled(crumple_fixture())
    with pytest.raises(ChartBuildFailure) as error:
        build_intrinsic_strip_atlas(chart, max_iterations=2)
    assert error.value.code == "ATLAS_INJECTIVITY_UNRESOLVED"


def test_e3_public_saddle_compile_welds_transitions_and_drag_is_static():
    node, seeds, alpha = saddle_fixture()
    graph = PatchGraph()
    graph.add_node(node)
    diagnostics = PatchVoronoiDiagnostics()
    plan = compile_patch_voronoi_plan(
        graph,
        tuple(seed.edge_index for seed in seeds),
        offset=0.01,
        alpha_budget=alpha,
        diagnostics=diagnostics,
    )

    assert diagnostics.atlas_chart_count > 1
    assert diagnostics.atlas_site_image_count > 0
    assert diagnostics.interior_transition_count > 0
    construct_calls = diagnostics.construct_calls
    compiled_signature = tuple(
        (surface.domain.chart_id, surface.domain.transition_metadata)
        for surface in plan.surfaces
    )
    for width in (0.5, 1.0, 1.5, 2.0):
        faces = evaluate_patch_voronoi_plan(
            plan, width=width, preview=True, diagnostics=diagnostics
        )
        assert diagnostics.construct_calls == construct_calls
        assert diagnostics.interior_weld_count > 0
        assert compiled_signature == tuple(
            (surface.domain.chart_id, surface.domain.transition_metadata)
            for surface in plan.surfaces
        )
    assert serialize_network_faces(faces) == serialize_network_faces(
        evaluate_patch_voronoi_plan(plan, width=2.0, preview=False)
    )
