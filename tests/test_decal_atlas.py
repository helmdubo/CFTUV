from dataclasses import replace
from fractions import Fraction
from types import SimpleNamespace

import pytest

import cftuv.decal_voronoi as decal_voronoi_module
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
from analysis_surface_fixtures import analysis_bundle_from_graph

_build_intrinsic_strip_charts = build_intrinsic_strip_charts
_compile_patch_voronoi_attempt = compile_patch_voronoi_attempt
_compile_patch_voronoi_plan = compile_patch_voronoi_plan


def build_intrinsic_strip_charts(node, *args, **kwargs):
    kwargs["patch_surface"] = node._test_patch_surface
    return _build_intrinsic_strip_charts(node, *args, **kwargs)


def compile_patch_voronoi_attempt(graph, *args, **kwargs):
    return _compile_patch_voronoi_attempt(
        analysis_bundle_from_graph(graph), *args, **kwargs
    )


def compile_patch_voronoi_plan(graph, *args, **kwargs):
    return _compile_patch_voronoi_plan(
        analysis_bundle_from_graph(graph), *args, **kwargs
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


def test_e4_t_contract_rejects_missing_canonical_owner(monkeypatch):
    transition_key = (
        "atlas-transition",
        "fixture",
        (10, 11),
        99,
    )
    segment = ((0.0, 0.0), (1.0, 0.0))
    monkeypatch.setattr(
        decal_voronoi_module,
        "_m1_atlas_transition_segments",
        lambda _domain: {((transition_key), "fixture"): segment},
    )
    groups = []
    for chart_id in (1, 2):
        surface = SimpleNamespace(
            domain=SimpleNamespace(
                admission_tier="APPROXIMATE",
                chart_id=chart_id,
            )
        )
        groups.append(((0, SimpleNamespace(surface=surface)),))

    with pytest.raises(ValueError, match="ATLAS_TRANSITION_DESYNC"):
        decal_voronoi_module._m1_build_transition_contract(groups)


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


@pytest.mark.atlas_frozen
@pytest.mark.xfail(
    reason=(
        "GL-pending: saddle materialization awaits the canonical Geometry "
        "Ledger from workplan §0c"
    ),
    strict=True,
)
def test_e3_public_saddle_materializes_through_t_contract(monkeypatch):
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
    captured_arrangements = []
    captured_contracts = []
    build_arrangement = decal_voronoi_module._build_decal_arrangement
    build_transition_contract = (
        decal_voronoi_module._m1_build_transition_contract
    )

    def capture_arrangement(*args, **kwargs):
        arrangement = build_arrangement(*args, **kwargs)
        captured_arrangements.append(arrangement)
        return arrangement

    monkeypatch.setattr(
        decal_voronoi_module,
        "_build_decal_arrangement",
        capture_arrangement,
    )

    def capture_transition_contract(*args, **kwargs):
        contract = build_transition_contract(*args, **kwargs)
        captured_contracts.append(contract)
        return contract

    monkeypatch.setattr(
        decal_voronoi_module,
        "_m1_build_transition_contract",
        capture_transition_contract,
    )
    drop_counts = []
    degenerate_merge_counts = []
    for width in (0.25, 0.5, 1.0, 2.0):
        diagnostics = PatchVoronoiDiagnostics()
        faces = evaluate_patch_voronoi_plan(
            attempt.plan,
            width=width,
            preview=True,
            diagnostics=diagnostics,
        )
        component_count, overfull_count = _edge_component_stats(faces)
        assert component_count == 1, width
        assert overfull_count == 0, width
        arrangement = captured_arrangements.pop()
        sampled_count, double_cover_count = _sample_arrangement_cover(
            arrangement, resolution=8
        )
        assert sampled_count > 0, width
        assert double_cover_count == 0, width
        assert diagnostics.atlas_sliver_owner_count > 0
        assert diagnostics.atlas_max_sliver_owner_distance <= 1.0 + 1e-9
        assert diagnostics.atlas_no_owner_drop_count == 0
        assert diagnostics.atlas_declared_owner_count > 0
        assert diagnostics.atlas_declared_owner_missing_count == 0
        assert diagnostics.atlas_degenerate_interval_merge_count > 0
        assert diagnostics.atlas_semantic_import_count > 0
        assert diagnostics.atlas_semantic_transition_count > 0
        if width == 0.25:
            multi_transition_faces = []
            for face in arrangement.faces:
                if face.surface.domain.chart_id != 16:
                    continue
                transition_keys = {
                    key[1]
                    for key in face.point_keys
                    if isinstance(key, tuple)
                    and key[:1] == ("m1-transition",)
                }
                if {
                    ("atlas-transition", 0, (15, 36), 14, 16),
                    ("atlas-transition", 0, (15, 37), 15, 16),
                } <= transition_keys:
                    multi_transition_faces.append(face)
            assert multi_transition_faces
            assert {
                face.site.edge_index for face in multi_transition_faces
            } == {12030}
        assert (
            diagnostics.atlas_touch_no_token_drop_count
            + diagnostics.atlas_single_side_drop_count
        ) > 0
        drop_counts.append(
            (
                diagnostics.atlas_touch_no_token_drop_count,
                diagnostics.atlas_single_side_drop_count,
                diagnostics.atlas_no_owner_drop_count,
            )
        )
        degenerate_merge_counts.append(
            diagnostics.atlas_degenerate_interval_merge_count
        )
        for contract in captured_contracts:
            for sides in contract.values():
                for side in sides:
                        assert all(
                            _owner is None or end - start > 1
                            for start, end, _owner in side.interval_owners
                        )
        captured_contracts.clear()
    assert len(set(drop_counts)) == 1
    assert all(count > 0 for count in degenerate_merge_counts)

    raw_arrangement_uv = decal_voronoi_module._m1_raw_arrangement_uv

    def inject_raw_uv_drift(face, point, alpha):
        u_fraction, v_length = raw_arrangement_uv(face, point, alpha)
        if face.transition_uv:
            u_fraction += 0.25
        return u_fraction, v_length

    monkeypatch.setattr(
        decal_voronoi_module,
        "_m1_raw_arrangement_uv",
        inject_raw_uv_drift,
    )
    with pytest.raises(ValueError, match="T8 O1 raw UV mismatch"):
        evaluate_patch_voronoi_plan(
            attempt.plan,
            width=1.0,
            preview=True,
        )


@pytest.mark.atlas_frozen
def test_t8_rejects_missing_owner_semantic_transport():
    """GL-pending saddle materialization; tier-2 не исполняет frozen путь."""

    node, seeds, alpha = saddle_fixture()
    graph = PatchGraph()
    graph.add_node(node)
    plan = compile_patch_voronoi_plan(
        graph,
        tuple(seed.edge_index for seed in seeds),
        offset=0.01,
        alpha_budget=alpha,
    )
    diagnostics = PatchVoronoiDiagnostics(
        semantic_transport_disabled_owner_ids=frozenset(
            {("corner", 14)}
        )
    )
    with pytest.raises(ValueError, match="ATLAS_CLASS_DESYNC"):
        evaluate_patch_voronoi_plan(
            plan,
            width=1.0,
            preview=True,
            diagnostics=diagnostics,
        )


@pytest.mark.atlas_frozen
@pytest.mark.xfail(
    reason=(
        "GL-pending: reversed saddle parity awaits the canonical Geometry "
        "Ledger from workplan §0c"
    ),
    strict=True,
)
def test_t8_saddle_is_deterministic_for_reversed_seed_enumeration():
    node, seeds, alpha = saddle_fixture()
    graph = PatchGraph()
    graph.add_node(node)
    forward = compile_patch_voronoi_plan(
        graph,
        tuple(seed.edge_index for seed in seeds),
        offset=0.01,
        alpha_budget=alpha,
    )
    reversed_plan = compile_patch_voronoi_plan(
        graph,
        tuple(seed.edge_index for seed in reversed(seeds)),
        offset=0.01,
        alpha_budget=alpha,
    )
    for width in (0.5, 1.0, 2.0):
        assert serialize_network_faces(
            evaluate_patch_voronoi_plan(
                forward, width=width, preview=True
            )
        ) == serialize_network_faces(
            evaluate_patch_voronoi_plan(
                reversed_plan, width=width, preview=True
            )
        )


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


def _sample_arrangement_cover(arrangement, resolution):
    """Сэмплирует source-triangles и считает только строгие interior hits."""

    faces_by_domain = {}
    domains = {}
    for face in arrangement.faces:
        domain = face.surface.domain
        if domain.admission_tier != "APPROXIMATE":
            continue
        domains[id(domain)] = domain
        faces_by_domain.setdefault(id(domain), []).append(face)

    triangle_copies = {}
    for domain in domains.values():
        for triangle in domain.intrinsic_triangles:
            triangle_copies.setdefault(
                repr(triangle.source_triangle_id), []
            ).append((domain, triangle))

    sampled_count = 0
    double_cover_count = 0
    denominator = resolution + 2
    for copies in triangle_copies.values():
        for first_weight in range(1, denominator - 1):
            for second_weight in range(1, denominator - first_weight):
                third_weight = denominator - first_weight - second_weight
                if third_weight <= 0:
                    continue
                weights = (
                    first_weight / denominator,
                    second_weight / denominator,
                    third_weight / denominator,
                )
                hit_count = 0
                ambiguous = False
                for domain, triangle in copies:
                    point = tuple(
                        sum(
                            triangle.chart_points[index][axis]
                            * weights[index]
                            for index in range(3)
                        )
                        for axis in range(2)
                    )
                    quantum = max(
                        float(
                            next(
                                face.surface.diagram_transform.quantum
                                for face in faces_by_domain[id(domain)]
                            )
                        ),
                        1e-10,
                    )
                    for face in faces_by_domain[id(domain)]:
                        boundary_distance = min(
                            decal_voronoi_module._segment_point_distance2(
                                first,
                                second,
                                point,
                            )[0]
                            for first, second in zip(
                                face.points,
                                face.points[1:] + face.points[:1],
                            )
                        )
                        if boundary_distance <= quantum * 2.0:
                            ambiguous = True
                            break
                        if decal_voronoi_module._m1_point_in_polygon(
                            point, face.points
                        ):
                            hit_count += 1
                    if ambiguous:
                        break
                if ambiguous:
                    continue
                sampled_count += 1
                if hit_count > 1:
                    double_cover_count += 1
    return sampled_count, double_cover_count


def test_t7_p3_degenerate_endpoint_intervals_merge_into_neighbor():
    diagnostics = PatchVoronoiDiagnostics()
    intervals = decal_voronoi_module._m1_canonicalize_transition_intervals(
        (
            (2822, 2831, 12030, ("CORNER", "KITE", ("corner", 18))),
            (2831, 2832, 12030, ("CORNER", "KITE", ("corner", 18))),
            (2832, 2833, 12032, ("SEGMENT",)),
        ),
        diagnostics,
    )
    assert intervals == (
        (2822, 2833, 12030, ("CORNER", "KITE", ("corner", 18))),
    )
    assert diagnostics.atlas_degenerate_interval_merge_count == 2


def test_t7_p3_endpoint_incidence_uses_euclidean_distance():
    """T7-P3.11e не должен повторно извлекать корень из distance."""

    assert decal_voronoi_module._dist2((0.0, 0.0), (3.0, 4.0)) == 5.0


def test_t7_p3_region_authority_stops_at_semantic_separator():
    surface = SimpleNamespace(domain=SimpleNamespace(chart_id=7))
    common = dict(
        surface=surface,
        site=SimpleNamespace(edge_index=101),
        points=((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)),
        crop=SimpleNamespace(kind="SEGMENT"),
        declared_owner_token=101,
        declared_semantic_class=("SEGMENT",),
    )
    first = decal_voronoi_module._DecalArrangementFace(
        **common,
        point_keys=("a", "b", "c"),
        boundary_corner_vertices=(7,),
        boundary_corner_edges=(("a", "b", (7,)),),
    )
    second = decal_voronoi_module._DecalArrangementFace(
        **common,
        point_keys=("b", "d", "c"),
    )
    corner = decal_voronoi_module._DecalArrangementFace(
        **{
            **common,
            "declared_owner_token": 202,
            "declared_semantic_class": (
                "CORNER",
                "KITE",
                ("corner", 7),
            ),
        },
        point_keys=("b", "a", "x"),
    )
    authorities = decal_voronoi_module._m1_region_corner_authorities(
        (first, second, corner), lambda _key: set()
    )
    assert authorities[id(first)] == (7,)
    assert authorities[id(second)] == (7,)
    assert id(corner) not in authorities

    without_separator = replace(
        first,
        boundary_corner_vertices=(),
        boundary_corner_edges=(),
    )
    authorities = decal_voronoi_module._m1_region_corner_authorities(
        (without_separator, second, corner), lambda _key: set()
    )
    assert authorities[id(without_separator)] == ()
    assert authorities[id(second)] == ()


@pytest.mark.atlas_frozen
def test_e4_m1_sphere_is_single_cover_and_edge_connected():
    """GL-pending sphere materialization на трёх ширинах."""

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
