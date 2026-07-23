from __future__ import annotations

from dataclasses import replace
from decimal import Decimal

import sympy as sp

from cftuv_envelope import (
    BoundaryConstraintId,
    BoundaryConstraintKind,
    BoundaryConstraintV1,
    CapEnvelopeSpec,
    ChainUseOrientation,
    LineageId,
    PhysicalEdgeSequenceConstraintTargetV1,
    SurfacePayloadMode,
    validate_analysis_snapshot,
)
from cftuv_envelope.reference import (
    ReferenceOutcome,
    compile_reference_envelopes,
    evaluate_reference_raw_coverage,
    raw_coverage_semantic_projection,
    raw_coverage_semantic_digest,
    validate_raw_coverage_digest,
    validate_raw_coverage_semantic_digest,
)
from cftuv_envelope.reference.planar_types import exact_sign

from reference_factories import straight_snapshot


SQUARE = (((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)),)
HOLE_RING = (
    ((0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (4.0, 5.0), (4.0, 10.0), (0.0, 10.0)),
    ((6.0, 0.0), (10.0, 0.0), (10.0, 10.0), (6.0, 10.0), (6.0, 5.0), (6.0, 3.0)),
    ((4.0, 0.0), (6.0, 0.0), (6.0, 3.0), (4.0, 3.0)),
    ((4.0, 5.0), (6.0, 5.0), (6.0, 10.0), (4.0, 10.0)),
)


def _evaluate(snapshot, request, alpha):
    assert validate_analysis_snapshot(snapshot) == ()
    compiled = compile_reference_envelopes(snapshot, request)
    assert compiled.outcome is ReferenceOutcome.EXACT
    return compiled, evaluate_reference_raw_coverage(compiled.compilation, alpha)


def test_straight_strip_has_physical_caps_exact_area_and_total_provenance():
    snapshot, request = straight_snapshot(
        faces=SQUARE,
        source_routes=({"name": "source", "points": ((0.0, 0.0), (10.0, 0.0))},),
        alpha="2",
    )
    compiled, evaluated = _evaluate(snapshot, request, Decimal("2"))
    assert evaluated.outcome is ReferenceOutcome.EXACT
    assert sum(isinstance(item, CapEnvelopeSpec) for item in compiled.compilation.envelope_specs) == 2
    raw = evaluated.raw_coverage
    assert sp.sympify(raw.exact_area_expression) == 20
    assert len(raw.regions) == 1
    assert len(raw.loops) == 1
    assert all(edge.provenance.envelope_spec_ids for edge in raw.edges)
    assert all(edge.provenance.physical_edge_ids for edge in raw.edges)
    assert all(vertex.construction_certificates for vertex in raw.vertices)


def test_raw_coverage_digest_api_excludes_only_its_own_digest_field():
    snapshot, request = straight_snapshot(
        faces=SQUARE,
        source_routes=(
            {"name": "source", "points": ((0.0, 0.0), (10.0, 0.0))},
        ),
        alpha="2",
    )
    _, evaluated = _evaluate(snapshot, request, Decimal("2"))
    raw = evaluated.raw_coverage

    assert validate_raw_coverage_semantic_digest(raw)
    assert validate_raw_coverage_digest(raw)
    assert raw_coverage_semantic_projection(raw).semantic_digest == ""
    assert raw_coverage_semantic_digest(raw).sha256_hex == raw.semantic_digest
    assert not validate_raw_coverage_semantic_digest(
        replace(raw, exact_area_expression="999")
    )


def test_hole_interior_contact_saturates_only_the_affected_component_exactly():
    snapshot, request = straight_snapshot(
        faces=HOLE_RING,
        source_routes=(
            {
                "name": "bottom",
                "points": ((0.0, 0.0), (4.0, 0.0), (6.0, 0.0), (10.0, 0.0)),
            },
            {
                "name": "top",
                "points": ((10.0, 10.0), (6.0, 10.0), (4.0, 10.0), (0.0, 10.0)),
            },
        ),
        alpha="4",
    )
    _, evaluated = _evaluate(snapshot, request, Decimal("4"))
    assert evaluated.outcome is ReferenceOutcome.BARRIER_SPLIT_REQUIRED
    raw = evaluated.raw_coverage
    states = {item.effective_alpha.expression: item for item in raw.component_effective_alphas}
    assert set(states) == {"Integer(3)", "Integer(4)"}
    assert states["Integer(3)"].capacity_outcome is ReferenceOutcome.BARRIER_SPLIT_REQUIRED
    assert states["Integer(4)"].capacity_outcome is None
    assert all(not item.reachability.bypass_used for item in raw.edges)


def test_patch_union_preserves_a_hole_when_four_reachable_fronts_cover_the_ring():
    snapshot, request = straight_snapshot(
        faces=HOLE_RING,
        source_routes=(
            {"name": "bottom", "points": ((0.0, 0.0), (4.0, 0.0), (6.0, 0.0), (10.0, 0.0))},
            {"name": "right", "points": ((10.0, 0.0), (10.0, 10.0))},
            {"name": "top", "points": ((10.0, 10.0), (6.0, 10.0), (4.0, 10.0), (0.0, 10.0))},
            {"name": "left", "points": ((0.0, 10.0), (0.0, 0.0))},
        ),
        alpha="5",
    )

    _, evaluated = _evaluate(snapshot, request, Decimal("5"))

    assert evaluated.outcome is ReferenceOutcome.BARRIER_SPLIT_REQUIRED
    raw = evaluated.raw_coverage
    assert len(raw.regions) == 1
    assert len(raw.loops) == 2
    assert len(next(iter(raw.regions)).hole_loop_ids) == 1
    assert sp.sympify(raw.exact_area_expression) == 96


def test_concave_outer_boundary_reports_exact_split_capacity():
    concave = (
        (
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (6.0, 10.0),
            (6.0, 3.0),
            (4.0, 3.0),
            (4.0, 10.0),
            (0.0, 10.0),
        ),
    )
    snapshot, request = straight_snapshot(
        faces=concave,
        source_routes=({"name": "source", "points": ((0.0, 0.0), (10.0, 0.0))},),
        alpha="4",
    )
    _, evaluated = _evaluate(snapshot, request, Decimal("4"))
    assert evaluated.outcome is ReferenceOutcome.BARRIER_SPLIT_REQUIRED
    state = next(iter(evaluated.raw_coverage.component_effective_alphas))
    assert state.effective_alpha.expression == "Integer(3)"


def test_explicit_internal_barrier_reports_split_at_exact_contact_alpha():
    filled_grid = (
        ((0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (4.0, 5.0), (4.0, 10.0), (0.0, 10.0)),
        ((6.0, 0.0), (10.0, 0.0), (10.0, 10.0), (6.0, 10.0), (6.0, 5.0), (6.0, 3.0)),
        ((4.0, 0.0), (6.0, 0.0), (6.0, 3.0), (4.0, 3.0)),
        ((4.0, 3.0), (6.0, 3.0), (6.0, 5.0), (4.0, 5.0)),
        ((4.0, 5.0), (6.0, 5.0), (6.0, 10.0), (4.0, 10.0)),
    )
    snapshot, request = straight_snapshot(
        faces=filled_grid,
        source_routes=(
            {
                "name": "bottom",
                "points": ((0.0, 0.0), (4.0, 0.0), (6.0, 0.0), (10.0, 0.0)),
            },
        ),
        alpha="4",
    )
    frame = next(iter(snapshot.surface_metric_descriptors))
    coordinate_by_vertex = {
        item.source_vertex_id: (item.domain_coordinate.x, item.domain_coordinate.y)
        for item in frame.source_vertex_coordinates
    }
    barrier_edge = next(
        edge.edge_id
        for edge in snapshot.surface_ir.source_edges
        if {
            coordinate_by_vertex[edge.vertex_a_id],
            coordinate_by_vertex[edge.vertex_b_id],
        }
        == {(4.0, 3.0), (6.0, 3.0)}
    )
    domain = next(iter(snapshot.patch_domains))
    barrier_id = BoundaryConstraintId("explicit-interior-barrier")
    barrier = BoundaryConstraintV1(
        barrier_id,
        domain.patch_domain_id,
        BoundaryConstraintKind.PHYSICAL_DOMAIN_BARRIER,
        PhysicalEdgeSequenceConstraintTargetV1((barrier_edge,), False),
        None,
        False,
        True,
    )
    snapshot = replace(
        snapshot,
        patch_domains=frozenset(
            {replace(domain, boundary_constraint_ids=domain.boundary_constraint_ids | {barrier_id})}
        ),
        boundary_constraints=snapshot.boundary_constraints | {barrier},
    )

    _, evaluated = _evaluate(snapshot, request, Decimal("4"))

    assert evaluated.outcome is ReferenceOutcome.BARRIER_SPLIT_REQUIRED
    state = next(iter(evaluated.raw_coverage.component_effective_alphas))
    assert state.effective_alpha.expression == "Integer(3)"


def test_explicit_barrier_endpoint_continuation_fails_as_bypass():
    filled_grid = (
        ((0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (4.0, 5.0), (4.0, 10.0), (0.0, 10.0)),
        ((6.0, 0.0), (10.0, 0.0), (10.0, 10.0), (6.0, 10.0), (6.0, 5.0), (6.0, 3.0)),
        ((4.0, 0.0), (6.0, 0.0), (6.0, 3.0), (4.0, 3.0)),
        ((4.0, 3.0), (6.0, 3.0), (6.0, 5.0), (4.0, 5.0)),
        ((4.0, 5.0), (6.0, 5.0), (6.0, 10.0), (4.0, 10.0)),
    )
    snapshot, request = straight_snapshot(
        faces=filled_grid,
        source_routes=(
            {"name": "bottom-left", "points": ((0.0, 0.0), (4.0, 0.0))},
        ),
        alpha="4",
    )
    frame = next(iter(snapshot.surface_metric_descriptors))
    coordinate_by_vertex = {
        item.source_vertex_id: (item.domain_coordinate.x, item.domain_coordinate.y)
        for item in frame.source_vertex_coordinates
    }
    barrier_edge = next(
        edge.edge_id
        for edge in snapshot.surface_ir.source_edges
        if {
            coordinate_by_vertex[edge.vertex_a_id],
            coordinate_by_vertex[edge.vertex_b_id],
        }
        == {(4.0, 3.0), (6.0, 3.0)}
    )
    domain = next(iter(snapshot.patch_domains))
    barrier_id = BoundaryConstraintId("explicit-endpoint-barrier")
    barrier = BoundaryConstraintV1(
        barrier_id,
        domain.patch_domain_id,
        BoundaryConstraintKind.PHYSICAL_DOMAIN_BARRIER,
        PhysicalEdgeSequenceConstraintTargetV1((barrier_edge,), False),
        None,
        False,
        True,
    )
    snapshot = replace(
        snapshot,
        patch_domains=frozenset(
            {
                replace(
                    domain,
                    boundary_constraint_ids=domain.boundary_constraint_ids
                    | {barrier_id},
                )
            }
        ),
        boundary_constraints=snapshot.boundary_constraints | {barrier},
    )

    _, evaluated = _evaluate(snapshot, request, Decimal("4"))

    assert evaluated.outcome is ReferenceOutcome.BARRIER_BYPASS_UNSUPPORTED
    state = next(iter(evaluated.raw_coverage.component_effective_alphas))
    assert state.effective_alpha.expression == "Integer(3)"
    assert state.capacity_outcome is ReferenceOutcome.BARRIER_BYPASS_UNSUPPORTED


def test_closed_non_linear_chain_has_no_caps_and_fails_named_without_corner_relations():
    snapshot, request = straight_snapshot(
        faces=HOLE_RING,
        source_routes=(
            {
                "name": "closed-source",
                "points": ((4.0, 3.0), (4.0, 5.0), (6.0, 5.0), (6.0, 3.0)),
                "is_closed": True,
            },
        ),
        alpha="1",
    )
    compiled = compile_reference_envelopes(snapshot, request)
    assert compiled.outcome is ReferenceOutcome.EXACT
    assert not any(
        isinstance(item, CapEnvelopeSpec) for item in compiled.compilation.envelope_specs
    )

    evaluated = evaluate_reference_raw_coverage(compiled.compilation, Decimal("1"))

    assert evaluated.outcome is ReferenceOutcome.PLANAR_CHAIN_SUPPORT_NOT_LINEAR
    assert evaluated.raw_coverage is None


def test_overlapping_sources_form_one_single_cover_region_with_both_contributors():
    routes = (
        {"name": "source-a", "points": ((0.0, 0.0), (10.0, 0.0))},
        {"name": "source-b", "points": ((0.0, 0.0), (10.0, 0.0))},
    )
    snapshot, request = straight_snapshot(faces=SQUARE, source_routes=routes, alpha="2")
    compiled, evaluated = _evaluate(snapshot, request, Decimal("2"))
    assert len(compiled.compilation.front_components) == 2
    raw = evaluated.raw_coverage
    assert len(raw.regions) == 1
    region = next(iter(raw.regions))
    strip_ids = {
        item.envelope_spec_id.value
        for item in compiled.compilation.envelope_specs
        if item.__class__.__name__ == "StripEnvelopeSpec"
    }
    assert strip_ids.issubset(region.contributor_envelope_spec_ids)
    assert sp.sympify(raw.exact_area_expression) == 20


def test_data_record_lineage_and_storage_reversal_do_not_change_raw_digest():
    snapshot, request = straight_snapshot(
        faces=SQUARE,
        source_routes=({"name": "source", "points": ((0.0, 0.0), (10.0, 0.0))},),
        alpha="2",
    )
    _, baseline = _evaluate(snapshot, request, Decimal("2"))
    chain_use = next(
        item
        for item in snapshot.chain_uses
        if item.chain_use_id in request.selected_chain_use_ids
    )
    chain = next(
        item
        for item in snapshot.physical_chains
        if item.physical_chain_id == chain_use.physical_chain_id
    )
    changed_lineage = replace(
        snapshot,
        physical_chains=frozenset(
            replace(
                item,
                data_record_lineage=frozenset(
                    {LineageId("split-a"), LineageId("split-b")}
                ),
            )
            if item.physical_chain_id == chain.physical_chain_id
            else item
            for item in snapshot.physical_chains
        ),
    )
    _, lineage_result = _evaluate(changed_lineage, request, Decimal("2"))
    assert lineage_result.raw_coverage.semantic_digest == baseline.raw_coverage.semantic_digest

    reversed_snapshot = replace(
        snapshot,
        physical_chains=frozenset(
            (
                replace(
                    item,
                    ordered_source_vertex_ids=tuple(reversed(chain.ordered_source_vertex_ids)),
                    ordered_physical_edge_ids=tuple(reversed(chain.ordered_physical_edge_ids)),
                )
                if item.physical_chain_id == chain.physical_chain_id
                else item
            )
            for item in snapshot.physical_chains
        ),
        chain_uses=frozenset(
            replace(item, orientation=ChainUseOrientation.B_START_TO_END)
            if item.chain_use_id == chain_use.chain_use_id
            else item
            for item in snapshot.chain_uses
        ),
        terminal_relations=frozenset(
            replace(
                item,
                endpoint_role=(
                    item.endpoint_role.__class__.END
                    if item.endpoint_role.value == "START"
                    else item.endpoint_role.__class__.START
                ),
            )
            for item in snapshot.terminal_relations
        ),
    )
    _, reversed_result = _evaluate(reversed_snapshot, request, Decimal("2"))
    assert reversed_result.raw_coverage.semantic_digest == baseline.raw_coverage.semantic_digest


def test_physical_route_split_merge_preserves_exact_silhouette_and_topology():
    baseline_snapshot, baseline_request = straight_snapshot(
        faces=SQUARE,
        source_routes=({"name": "source", "points": ((0.0, 0.0), (10.0, 0.0))},),
        alpha="2",
    )
    split_face = (
        ((0.0, 0.0), (4.0, 0.0), (6.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)),
    )
    split_snapshot, split_request = straight_snapshot(
        faces=split_face,
        source_routes=(
            {"name": "source", "points": ((0.0, 0.0), (4.0, 0.0), (6.0, 0.0), (10.0, 0.0))},
        ),
        alpha="2",
    )
    _, baseline = _evaluate(baseline_snapshot, baseline_request, Decimal("2"))
    _, split = _evaluate(split_snapshot, split_request, Decimal("2"))

    def silhouette(raw):
        vertices = {item.vertex_id: item.point for item in raw.vertices}
        return frozenset(
            (
                "V",
                vertices[edge.start_vertex_id].x.expression,
            )
            if vertices[edge.start_vertex_id].x == vertices[edge.end_vertex_id].x
            else (
                "H",
                vertices[edge.start_vertex_id].y.expression,
            )
            for edge in raw.edges
        )

    assert sp.sympify(split.raw_coverage.exact_area_expression) == sp.sympify(
        baseline.raw_coverage.exact_area_expression
    )
    assert len(split.raw_coverage.regions) == len(baseline.raw_coverage.regions)
    assert silhouette(split.raw_coverage) == silhouette(baseline.raw_coverage)


def test_coordinate_free_ec0_projection_never_invents_planar_geometry(projections):
    projection = next(item for item in projections if item.case_id == "EC0-C01")
    plan_domain = projection.plans[0].plan_key.patch_domain_id
    compiled = compile_reference_envelopes(
        projection.snapshot, projection.request, plan_domain
    )
    assert compiled.outcome is ReferenceOutcome.EXACT
    assert projection.snapshot.surface_ir.payload_mode is SurfacePayloadMode.EC0_COORDINATE_FREE_FIXTURE_V5
    evaluated = evaluate_reference_raw_coverage(compiled.compilation, Decimal("1"))
    assert evaluated.outcome is ReferenceOutcome.REFERENCE_GEOMETRY_PAYLOAD_REQUIRED
    assert evaluated.raw_coverage is None


def test_uniform_scale_translation_preserves_topology_and_scales_area():
    base_snapshot, base_request = straight_snapshot(
        faces=SQUARE,
        source_routes=({"name": "source", "points": ((0.0, 0.0), (10.0, 0.0))},),
        alpha="2",
    )
    _, base = _evaluate(base_snapshot, base_request, Decimal("2"))
    transform = lambda point: (3 * point[0] + 7, 3 * point[1] - 11)
    transformed_faces = tuple(tuple(transform(point) for point in face) for face in SQUARE)
    transformed_route = tuple(transform(point) for point in ((0.0, 0.0), (10.0, 0.0)))
    scaled_snapshot, scaled_request = straight_snapshot(
        faces=transformed_faces,
        source_routes=({"name": "source", "points": transformed_route},),
        alpha="6",
        revision_name="scaled-translated",
    )
    _, scaled = _evaluate(scaled_snapshot, scaled_request, Decimal("6"))
    assert len(scaled.raw_coverage.regions) == len(base.raw_coverage.regions)
    assert len(scaled.raw_coverage.loops) == len(base.raw_coverage.loops)
    assert sp.sympify(scaled.raw_coverage.exact_area_expression) == 9 * sp.sympify(
        base.raw_coverage.exact_area_expression
    )


def test_very_short_source_and_simultaneous_exact_contacts_are_not_tolerance_selected():
    short_face = (((0.0, 0.0), (0.001, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)),)
    short_snapshot, short_request = straight_snapshot(
        faces=short_face,
        source_routes=({"name": "short", "points": ((0.0, 0.0), (0.001, 0.0))},),
        alpha="0.5",
    )
    _, short = _evaluate(short_snapshot, short_request, Decimal("0.5"))
    assert exact_sign(short.raw_coverage.exact_area_expression) > 0

    triangle = (((-1.0, 0.0), (1.0, 0.0), (0.0, 1.0)),)
    contact_snapshot, contact_request = straight_snapshot(
        faces=triangle,
        source_routes=({"name": "base", "points": ((-1.0, 0.0), (1.0, 0.0))},),
        alpha="1",
    )
    _, contact = _evaluate(contact_snapshot, contact_request, Decimal("1"))
    assert contact.outcome is ReferenceOutcome.EXACT
    assert sp.sympify(contact.raw_coverage.exact_area_expression) == 1
    event_keys = {
        event
        for edge in contact.raw_coverage.edges
        for event in edge.reachability.boundary_event_keys
    }
    assert len(event_keys) >= 2
