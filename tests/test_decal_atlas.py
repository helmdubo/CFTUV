from fractions import Fraction

import pytest

from cftuv.decal_atlas import build_intrinsic_strip_atlas
from cftuv.decal_chart_admission import (
    _prepare_disk_topology,
    admit_intrinsic_strip_runtime,
)
from cftuv.decal_charts import (
    ChartBuildFailure,
    build_intrinsic_strip_charts,
    chart_triangle_overlap_pairs,
    unroll_intrinsic_strip_chart,
)
from cftuv.decal_voronoi import (
    _m1_collinear_overlap,
    PatchVoronoiDiagnostics,
    compile_patch_voronoi_attempt,
    compile_patch_voronoi_plan,
    evaluate_patch_voronoi_plan,
    serialize_network_faces,
)
from cftuv.model import PatchGraph

from decal_organic_fixtures import (
    crumple_fixture,
    cliff_fixture,
    intermediate_dome_fixture,
    saddle_fixture,
    sphere_cap_fixture,
    tight_sphere_fixture,
    wide_support_dome_fixture,
)


def _unrolled(fixture):
    node, seeds, alpha = fixture
    chart = build_intrinsic_strip_charts(node, seeds, alpha)[0]
    return unroll_intrinsic_strip_chart(
        _prepare_disk_topology(chart), edge_relative_tolerance=1e-5
    )


def test_e4_collinear_extent_distinguishes_disjoint_and_partial_overlap():
    edge_a = (Fraction(0), Fraction(0))
    edge_b = (Fraction(10), Fraction(0))

    assert _m1_collinear_overlap(
        (Fraction(20), Fraction(0)),
        (Fraction(30), Fraction(0)),
        edge_a,
        edge_b,
    ) is None
    assert _m1_collinear_overlap(
        (Fraction(-5), Fraction(0)),
        (Fraction(5), Fraction(0)),
        edge_a,
        edge_b,
    ) == (
        Fraction(1, 2),
        Fraction(1),
        Fraction(0),
        Fraction(1, 2),
    )


def test_e3_saddle_atlas_is_locally_injective_and_owns_each_triangle_once():
    chart = _unrolled(saddle_fixture())
    atlas = build_intrinsic_strip_atlas(chart)

    assert atlas.atlas_chart_count > 1
    assert atlas.separator_iterations <= 8
    assert atlas.interior_transition_count > 0
    assert all(not chart_triangle_overlap_pairs(item) for item in atlas.charts)
    assert {
        cut.reason for item in atlas.charts for cut in item.cuts
    } <= {"ATLAS_SEPARATOR", "CURVATURE_RELIEF_MARGIN"}
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


def _runtime_admit(fixture, budget=0.02):
    node, seeds, alpha = fixture
    chart = build_intrinsic_strip_charts(node, seeds, alpha)[0]
    return admit_intrinsic_strip_runtime(
        chart, initial_alpha=alpha, distortion_budget=budget
    )


def test_e2_tiers_and_distortion_budget_follow_measured_width_error():
    sphere = _runtime_admit(sphere_cap_fixture())
    assert sphere.admission_tier == "APPROXIMATE"

    with pytest.raises(ChartBuildFailure) as default_error:
        _runtime_admit(intermediate_dome_fixture(), budget=0.02)
    assert default_error.value.code == "DISTORTION_BUDGET_EXCEEDED"
    assert _runtime_admit(
        intermediate_dome_fixture(), budget=0.05
    ).admission_tier == "APPROXIMATE"

    with pytest.raises(ChartBuildFailure) as hard_error:
        _runtime_admit(tight_sphere_fixture(), budget=0.10)
    assert hard_error.value.code == "DISTORTION_BUDGET_EXCEEDED"


def test_e2_crumple_rejects_foldover_before_atlas_approximation():
    with pytest.raises(ChartBuildFailure) as error:
        _runtime_admit(crumple_fixture())
    assert error.value.code == "FOLDOVER_DETECTED"


def test_e1_wide_dome_overlap_uses_canonical_margin_relief_cuts():
    fixture = wide_support_dome_fixture()
    unrolled = _unrolled(fixture)
    assert unrolled.metrics.triangle_overlap_count > 0

    atlas = _runtime_admit(fixture)
    margin_transitions = tuple(
        transition
        for transition in atlas.transitions
        if transition.reason == "CURVATURE_RELIEF_MARGIN"
    )
    assert margin_transitions
    assert atlas.margin_relief_cut_count == len(margin_transitions)
    assert all(
        transition.selection_distance >= atlas.charts[0].alpha_budget - 1e-9
        for transition in margin_transitions
    )
    assert all(not chart_triangle_overlap_pairs(item) for item in atlas.charts)
    cuts = {
        (cut.transition_key, cut.reason, cut.source_edge)
        for item in atlas.charts
        for cut in item.cuts
    }
    assert all(
        (transition.transition_key, transition.reason, transition.source_edge)
        in cuts
        for transition in atlas.transitions
    )


def test_e2_cliff_is_approximate_locally_injective_atlas():
    atlas = _runtime_admit(cliff_fixture())
    assert atlas.admission_tier == "APPROXIMATE"
    assert atlas.metrics.max_width_error_sampled <= 0.02
    assert all(not chart_triangle_overlap_pairs(item) for item in atlas.charts)


def test_e3_public_saddle_materializes_through_t_contract():
    node, seeds, alpha = saddle_fixture()
    graph = PatchGraph()
    graph.add_node(node)
    attempt = compile_patch_voronoi_attempt(
        graph,
        tuple(seed.edge_index for seed in seeds),
        offset=0.01,
        alpha_budget=alpha,
    )
    assert attempt.plan is not None
    assert attempt.rejected_edge_indices == ()
    assert attempt.failures == ()
    for width in (0.25, 0.5, 1.0):
        faces = evaluate_patch_voronoi_plan(
            attempt.plan, width=width, preview=True
        )
        component_count, overfull_count = _edge_component_stats(faces)
        assert component_count == 1
        assert overfull_count == 0


def _edge_component_stats(faces):
    edge_owners = {}
    for face_index, face in enumerate(faces):
        for index, first in enumerate(face.vert_keys):
            second = face.vert_keys[(index + 1) % len(face.vert_keys)]
            edge = tuple(sorted((repr(first), repr(second))))
            edge_owners.setdefault(edge, []).append(face_index)
    neighbours = [set() for _face in faces]
    for owners in edge_owners.values():
        if len(owners) == 2:
            first, second = owners
            neighbours[first].add(second)
            neighbours[second].add(first)
    unseen = set(range(len(faces)))
    component_count = 0
    while unseen:
        component_count += 1
        frontier = [unseen.pop()]
        while frontier:
            for neighbour in neighbours[frontier.pop()]:
                if neighbour in unseen:
                    unseen.remove(neighbour)
                    frontier.append(neighbour)
    return component_count, sum(
        len(owners) > 2 for owners in edge_owners.values()
    )


def test_e4_m1_sphere_is_single_cover_and_edge_connected():
    node, seeds, alpha = sphere_cap_fixture()
    graph = PatchGraph()
    graph.add_node(node)
    plan = compile_patch_voronoi_plan(
        graph,
        tuple(seed.edge_index for seed in seeds),
        offset=0.01,
        alpha_budget=alpha,
    )
    previous_component_count = None
    for width in (0.5, 1.0, 2.0):
        faces = evaluate_patch_voronoi_plan(
            plan, width=width, preview=True
        )
        component_count, overfull_count = _edge_component_stats(faces)
        assert overfull_count == 0
        assert component_count == 1
        if previous_component_count is not None:
            assert component_count <= previous_component_count
        previous_component_count = component_count
