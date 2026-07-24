from __future__ import annotations

from dataclasses import replace
from decimal import Decimal
from fractions import Fraction

import pytest
import sympy as sp

from cftuv_envelope import (
    AngularEnvelopeSpec,
    CertifiedDecimalIntervalV1,
    CertifiedReflexAngleMeasureV1,
    DecalRequestId,
    IntervalEndpointKind,
    JunctionEnvelopeSpec,
    JunctionRelationKind,
    LineageId,
    SurfaceTriangleId,
    SurfaceTriangleV1,
    TurnOrientation,
    UnavailableJunctionRouteTopologyV1,
    JunctionRouteTopologyUnavailableReason,
    validate_analysis_snapshot,
)
from cftuv_envelope.reference import (
    ReferenceOutcome,
    compile_reference_envelopes,
    evaluate_reference_raw_coverage,
)

from reference_factories import (
    angular_snapshot,
    junction_snapshot,
    physical_seam_snapshot,
    straight_snapshot,
)


@pytest.mark.parametrize(
    ("kind", "route_pair_count"),
    (
        (JunctionRelationKind.T, 1),
        (JunctionRelationKind.X, 2),
        (JunctionRelationKind.Y, 2),
    ),
)
def test_declared_t_x_y_junctions_keep_route_pairing_and_build_one_envelope(
    kind, route_pair_count
):
    snapshot, request = junction_snapshot(kind)
    assert validate_analysis_snapshot(snapshot) == ()
    compiled = compile_reference_envelopes(snapshot, request)
    assert compiled.outcome is ReferenceOutcome.EXACT
    junctions = [
        item
        for item in compiled.compilation.envelope_specs
        if isinstance(item, JunctionEnvelopeSpec)
    ]
    assert len(junctions) == 1
    assert len(junctions[0].route_pairing_ids) == route_pair_count
    evaluated = evaluate_reference_raw_coverage(compiled.compilation, Decimal("1"))
    assert (
        evaluated.outcome
        is ReferenceOutcome.REFERENCE_JUNCTION_ENVELOPE_LAW_UNPROVEN
    )
    assert evaluated.raw_coverage is None


def test_junction_route_pairing_is_never_guessed():
    snapshot, request = junction_snapshot(JunctionRelationKind.X)
    relation = next(iter(snapshot.junction_relations))
    snapshot = replace(
        snapshot,
        junction_relations=frozenset(
            {
                replace(
                    relation,
                    route_topology=UnavailableJunctionRouteTopologyV1(
                        JunctionRouteTopologyUnavailableReason.EC0_ROUTE_TOPOLOGY_NOT_RECORDED_V5
                    ),
                )
            }
        ),
    )
    compiled = compile_reference_envelopes(snapshot, request)
    assert compiled.outcome is ReferenceOutcome.JUNCTION_ROUTE_PAIRING_REQUIRED
    assert compiled.compilation is None


def test_physical_seam_a_b_is_evaluated_as_two_domain_plans():
    snapshot, request, domain_ids = physical_seam_snapshot()
    assert validate_analysis_snapshot(snapshot) == ()
    compilations = tuple(
        compile_reference_envelopes(snapshot, request, domain_id)
        for domain_id in domain_ids
    )
    assert all(item.outcome is ReferenceOutcome.EXACT for item in compilations)
    assert {
        item.compilation.plan_key.patch_domain_id for item in compilations
    } == set(domain_ids)
    results = tuple(
        evaluate_reference_raw_coverage(item.compilation, Decimal("1"))
        for item in compilations
    )
    assert all(item.outcome is ReferenceOutcome.EXACT for item in results)
    physical_edge_sets = tuple(
        frozenset(
            edge_id
            for edge in item.raw_coverage.edges
            for edge_id in edge.provenance.physical_edge_ids
        )
        for item in results
    )
    selected_use = next(
        item
        for item in snapshot.chain_uses
        if item.chain_use_id in request.selected_chain_use_ids
    )
    shared_chain = next(
        item
        for item in snapshot.physical_chains
        if item.physical_chain_id == selected_use.physical_chain_id
    )
    shared_chain_edge_ids = frozenset(
        item.value for item in shared_chain.ordered_physical_edge_ids
    )
    assert all(shared_chain_edge_ids <= item for item in physical_edge_sets)


def test_cross_patch_junction_uses_one_anchor_and_distinct_per_patch_projections():
    snapshot, request, domain_ids = physical_seam_snapshot(cross_junction=True)
    specs = []
    for domain_id in domain_ids:
        compiled = compile_reference_envelopes(snapshot, request, domain_id)
        assert compiled.outcome is ReferenceOutcome.EXACT
        spec = next(
            item
            for item in compiled.compilation.envelope_specs
            if isinstance(item, JunctionEnvelopeSpec)
        )
        specs.append(spec)
        evaluated = evaluate_reference_raw_coverage(compiled.compilation, Decimal("1"))
        assert (
            evaluated.outcome
            is ReferenceOutcome.REFERENCE_JUNCTION_ENVELOPE_LAW_UNPROVEN
        )
        assert evaluated.raw_coverage is None
    assert specs[0].shared_semantic_anchor_id == specs[1].shared_semantic_anchor_id
    assert specs[0].per_patch_projection_id != specs[1].per_patch_projection_id


def test_seam_self_two_uses_remain_distinct_in_one_patch_union():
    faces = (
        ((0.0, 0.0), (5.0, 0.0), (5.0, 10.0), (0.0, 10.0)),
        ((5.0, 0.0), (10.0, 0.0), (10.0, 10.0), (5.0, 10.0)),
    )
    snapshot, request = straight_snapshot(
        faces=faces,
        source_routes=(
            {
                "name": "self-seam",
                "points": ((5.0, 0.0), (5.0, 10.0)),
                "kind": "SEAM_SELF",
                "uses": (("a", "A_START_TO_END"), ("b", "B_START_TO_END")),
            },
        ),
        alpha="1",
    )
    assert validate_analysis_snapshot(snapshot) == ()
    compiled = compile_reference_envelopes(snapshot, request)
    assert compiled.outcome is ReferenceOutcome.EXACT
    assert len(compiled.compilation.front_components) == 2
    evaluated = evaluate_reference_raw_coverage(compiled.compilation, Decimal("1"))
    assert evaluated.outcome is ReferenceOutcome.EXACT
    assert sp.sympify(evaluated.raw_coverage.exact_area_expression) == 20
    assert len(evaluated.raw_coverage.regions) == 1
    region = next(iter(evaluated.raw_coverage.regions))
    assert len(region.contributor_envelope_spec_ids) >= 2


def test_angle_certificate_change_without_support_change_fails_closed():
    snapshot, request = angular_snapshot(0)
    baseline = compile_reference_envelopes(snapshot, request)
    assert next(
        item
        for item in baseline.compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
    ).resolved_hidden_edge_count == 0
    certificate = next(iter(snapshot.reflex_angle_certificates))
    payload = certificate.measure_payload
    changed_interval = CertifiedDecimalIntervalV1(
        Decimal("0.50"),
        Decimal("0.60"),
        IntervalEndpointKind.CLOSED,
        IntervalEndpointKind.CLOSED,
        Decimal("0.10"),
    )
    changed_payload = replace(
        payload,
        phi_over_pi=CertifiedDecimalIntervalV1(
            Decimal("1.50"),
            Decimal("1.60"),
            IntervalEndpointKind.CLOSED,
            IntervalEndpointKind.CLOSED,
            Decimal("0.10"),
        ),
        reflex_excess_over_pi=changed_interval,
    )
    changed_snapshot = replace(
        snapshot,
        reflex_angle_certificates=frozenset(
            {replace(certificate, measure_payload=changed_payload)}
        ),
    )
    changed = compile_reference_envelopes(changed_snapshot, request)
    assert (
        changed.outcome
        is ReferenceOutcome.REFERENCE_ANGLE_SUPPORT_CERTIFICATE_MISMATCH
    )
    assert changed.compilation is None


def test_source_retriangulation_does_not_change_reference_raw_digest():
    square = (((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)),)
    snapshot, request = straight_snapshot(
        faces=square,
        source_routes=({"name": "source", "points": ((0.0, 0.0), (10.0, 0.0))},),
        alpha="2",
    )
    baseline_compilation = compile_reference_envelopes(snapshot, request)
    baseline = evaluate_reference_raw_coverage(
        baseline_compilation.compilation, Decimal("2")
    ).raw_coverage
    face = next(iter(snapshot.surface_ir.source_faces))
    v0, v1, v2, v3 = face.vertex_cycle
    e0, e1, e2, e3 = face.edge_cycle
    alternate = frozenset(
        {
            SurfaceTriangleV1(
                SurfaceTriangleId("alternate-a"),
                face.face_id,
                (v0, v1, v3),
                (e0, None, e3),
                face.polygon_normal,
            ),
            SurfaceTriangleV1(
                SurfaceTriangleId("alternate-b"),
                face.face_id,
                (v1, v2, v3),
                (e1, e2, None),
                face.polygon_normal,
            ),
        }
    )
    changed_face = replace(
        face, triangle_ids=tuple(item.triangle_id for item in alternate)
    )
    changed_snapshot = replace(
        snapshot,
        surface_ir=replace(
            snapshot.surface_ir,
            source_faces=frozenset({changed_face}),
            surface_triangles=alternate,
        ),
    )
    assert validate_analysis_snapshot(changed_snapshot) == ()
    changed = evaluate_reference_raw_coverage(
        compile_reference_envelopes(changed_snapshot, request).compilation,
        Decimal("2"),
    ).raw_coverage
    assert changed.semantic_digest == baseline.semantic_digest


def test_source_permutation_and_hidden_support_record_permutation_are_invariant():
    square = (((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)),)
    routes = (
        {"name": "source-a", "points": ((0.0, 0.0), (10.0, 0.0))},
        {"name": "source-b", "points": ((0.0, 0.0), (10.0, 0.0))},
    )
    first_snapshot, first_request = straight_snapshot(
        faces=square, source_routes=routes, alpha="2"
    )
    second_snapshot, second_request = straight_snapshot(
        faces=square, source_routes=tuple(reversed(routes)), alpha="2"
    )
    first = evaluate_reference_raw_coverage(
        compile_reference_envelopes(first_snapshot, first_request).compilation,
        Decimal("2"),
    )
    second = evaluate_reference_raw_coverage(
        compile_reference_envelopes(second_snapshot, second_request).compilation,
        Decimal("2"),
    )
    assert second.raw_coverage.semantic_digest == first.raw_coverage.semantic_digest

    angular_snapshot_value, angular_request = angular_snapshot(2)
    compilation = compile_reference_envelopes(
        angular_snapshot_value, angular_request
    ).compilation
    angular = next(
        item for item in compilation.envelope_specs if isinstance(item, AngularEnvelopeSpec)
    )
    permuted = replace(
        compilation,
        envelope_specs=frozenset(
            replace(item, hidden_supports=frozenset(reversed(tuple(item.hidden_supports))))
            if item == angular
            else item
            for item in compilation.envelope_specs
        ),
    )
    baseline = evaluate_reference_raw_coverage(compilation, Decimal("1"))
    changed = evaluate_reference_raw_coverage(permuted, Decimal("1"))
    assert changed.raw_coverage.semantic_digest == baseline.raw_coverage.semantic_digest


def test_request_identity_changes_plan_and_raw_coverage_identity():
    square = (((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)),)
    snapshot, request = straight_snapshot(
        faces=square,
        source_routes=({"name": "source", "points": ((0.0, 0.0), (10.0, 0.0))},),
        alpha="2",
    )
    changed_request = replace(request, decal_request_id=DecalRequestId("changed-request"))
    baseline_compilation = compile_reference_envelopes(snapshot, request).compilation
    changed_compilation = compile_reference_envelopes(snapshot, changed_request).compilation
    baseline = evaluate_reference_raw_coverage(baseline_compilation, Decimal("2"))
    changed = evaluate_reference_raw_coverage(changed_compilation, Decimal("2"))

    assert changed_compilation.plan_key != baseline_compilation.plan_key
    assert changed.raw_coverage.semantic_digest != baseline.raw_coverage.semantic_digest


def test_coherent_quarter_turn_and_translation_preserve_area_and_topology():
    square = (((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)),)
    snapshot, request = straight_snapshot(
        faces=square,
        source_routes=({"name": "source", "points": ((0.0, 0.0), (10.0, 0.0))},),
        alpha="2",
    )
    transform = lambda point: (7 - point[1], -3 + point[0])
    transformed_snapshot, transformed_request = straight_snapshot(
        faces=tuple(tuple(transform(point) for point in face) for face in square),
        source_routes=(
            {
                "name": "source",
                "points": tuple(
                    transform(point) for point in ((0.0, 0.0), (10.0, 0.0))
                ),
            },
        ),
        alpha="2",
        revision_name="rigid-transform",
    )
    baseline = evaluate_reference_raw_coverage(
        compile_reference_envelopes(snapshot, request).compilation, Decimal("2")
    ).raw_coverage
    transformed = evaluate_reference_raw_coverage(
        compile_reference_envelopes(
            transformed_snapshot, transformed_request
        ).compilation,
        Decimal("2"),
    ).raw_coverage
    assert sp.sympify(transformed.exact_area_expression) == sp.sympify(
        baseline.exact_area_expression
    )
    assert len(transformed.regions) == len(baseline.regions)
    assert len(transformed.loops) == len(baseline.loops)


def _orthogonal_grid_oracle(rectangles):
    xs = sorted({value for rectangle in rectangles for value in (rectangle[0], rectangle[2])})
    ys = sorted({value for rectangle in rectangles for value in (rectangle[1], rectangle[3])})
    cells = set()
    area = Fraction(0)
    for x_index, (left, right) in enumerate(zip(xs, xs[1:])):
        for y_index, (bottom, top) in enumerate(zip(ys, ys[1:])):
            center = ((left + right) / 2, (bottom + top) / 2)
            if any(
                rectangle[0] < center[0] < rectangle[2]
                and rectangle[1] < center[1] < rectangle[3]
                for rectangle in rectangles
            ):
                cells.add((x_index, y_index))
                area += (right - left) * (top - bottom)
    boundary_edges = 0
    for x_index, y_index in cells:
        for neighbor in (
            (x_index - 1, y_index),
            (x_index + 1, y_index),
            (x_index, y_index - 1),
            (x_index, y_index + 1),
        ):
            if neighbor not in cells:
                boundary_edges += 1
    remaining = set(cells)
    components = 0
    while remaining:
        components += 1
        stack = [remaining.pop()]
        while stack:
            x_index, y_index = stack.pop()
            for neighbor in (
                (x_index - 1, y_index),
                (x_index + 1, y_index),
                (x_index, y_index - 1),
                (x_index, y_index + 1),
            ):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    stack.append(neighbor)
    return area, boundary_edges, components


def test_independent_orthogonal_grid_oracle_matches_union_area_topology_and_lineage():
    square = (((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)),)
    snapshot, request = straight_snapshot(
        faces=square,
        source_routes=(
            {"name": "wide", "points": ((0.0, 0.0), (10.0, 0.0))},
            {"name": "overlap", "points": ((0.0, 0.0), (10.0, 0.0))},
        ),
        alpha="2",
    )
    raw = evaluate_reference_raw_coverage(
        compile_reference_envelopes(snapshot, request).compilation, Decimal("2")
    ).raw_coverage
    oracle_area, oracle_boundary_edges, oracle_components = _orthogonal_grid_oracle(
        ((Fraction(0), Fraction(0), Fraction(10), Fraction(2)),)
    )
    assert sp.sympify(raw.exact_area_expression) == oracle_area
    assert len(raw.regions) == oracle_components
    assert len(raw.edges) == oracle_boundary_edges
    assert len(next(iter(raw.regions)).contributor_envelope_instance_ids) >= 2
    assert all(edge.provenance.envelope_instance_ids for edge in raw.edges)
