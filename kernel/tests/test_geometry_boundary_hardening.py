from __future__ import annotations

from dataclasses import replace
from decimal import Decimal

from cftuv_envelope import (
    AnalysisSnapshotCodecV1,
    AngleNormalizationLaw,
    BoundaryConstraintTargetUnavailableReason,
    CertifiedDecimalIntervalV1,
    CertifiedReflexAngleMeasureV1,
    ChainUseId,
    FrontSeedV1,
    IntervalEndpointKind,
    JunctionRelationKind,
    JunctionRouteTopologyUnavailableReason,
    PhysicalEdgeId,
    SemanticDigestValue,
    UnavailableBoundaryConstraintTargetV1,
    UnavailableJunctionRouteTopologyV1,
    ValidationCode,
    validate_analysis_snapshot,
    validate_cross_contract_references,
    geometry_batch_semantic_digest,
)

from factories import full_host_snapshot, geometry_batch


def _projection(projections, case_id):
    return next(item for item in projections if item.case_id == case_id)


def _interval(lower: str, upper: str) -> CertifiedDecimalIntervalV1:
    return CertifiedDecimalIntervalV1(
        lower=Decimal(lower),
        upper=Decimal(upper),
        lower_kind=IntervalEndpointKind.CLOSED,
        upper_kind=IntervalEndpointKind.CLOSED,
        absolute_error_bound=Decimal(upper) - Decimal(lower),
    )


def test_full_host_snapshot_has_complete_r1_geometry_boundary():
    snapshot = full_host_snapshot()
    assert validate_analysis_snapshot(snapshot) == ()
    payload = AnalysisSnapshotCodecV1.dumps(snapshot)
    assert AnalysisSnapshotCodecV1.loads(payload) == snapshot
    assert AnalysisSnapshotCodecV1.dumps(AnalysisSnapshotCodecV1.loads(payload)) == payload


def test_full_host_requires_concrete_metric_and_boundary_target():
    snapshot = full_host_snapshot()
    frame = next(iter(snapshot.surface_metric_descriptors))
    coordinate = next(iter(frame.source_vertex_coordinates))
    incomplete_frame = replace(
        frame,
        source_vertex_coordinates=frame.source_vertex_coordinates - {coordinate},
    )
    constraint = next(iter(snapshot.boundary_constraints))
    unavailable_constraint = replace(
        constraint,
        target=UnavailableBoundaryConstraintTargetV1(
            reason=BoundaryConstraintTargetUnavailableReason.EC0_COORDINATE_FREE_FIXTURE_V5
        ),
    )
    invalid = replace(
        snapshot,
        surface_metric_descriptors=frozenset({incomplete_frame}),
        boundary_constraints=frozenset({unavailable_constraint}),
    )
    codes = {issue.code for issue in validate_analysis_snapshot(invalid)}
    assert ValidationCode.SURFACE_METRIC in codes
    assert ValidationCode.SURFACE_TOPOLOGY in codes


def test_surface_ir_checks_edge_endpoints_and_reciprocal_triangles():
    snapshot = full_host_snapshot()
    edge = next(item for item in snapshot.surface_ir.source_edges if item.edge_id == PhysicalEdgeId("e0"))
    bad_edge = replace(edge, vertex_b_id=next(vertex for vertex in snapshot.surface_ir.source_vertex_ids if vertex.value == "v2"))
    face = next(iter(snapshot.surface_ir.source_faces))
    bad_face = replace(face, triangle_ids=())
    invalid_surface = replace(
        snapshot.surface_ir,
        source_edges=(snapshot.surface_ir.source_edges - {edge}) | {bad_edge},
        source_faces=frozenset({bad_face}),
    )
    issues = validate_analysis_snapshot(replace(snapshot, surface_ir=invalid_surface))
    assert sum(issue.code is ValidationCode.SURFACE_TOPOLOGY for issue in issues) >= 2


def test_physical_chain_requires_ordered_edge_path():
    snapshot = full_host_snapshot()
    chain = next(iter(snapshot.physical_chains))
    invalid = replace(
        snapshot,
        physical_chains=frozenset({replace(chain, ordered_physical_edge_ids=())}),
    )
    assert any(
        issue.code is ValidationCode.SURFACE_TOPOLOGY
        for issue in validate_analysis_snapshot(invalid)
    )


def test_certified_angle_proves_k_or_returns_named_uncertainty(projections):
    projection = _projection(projections, "EC0-C03")
    certificate = next(iter(projection.snapshot.reflex_angle_certificates))
    sector = next(iter(projection.snapshot.angular_owner_sectors))
    valid_payload = CertifiedReflexAngleMeasureV1(
        phi_over_pi=_interval("1.40", "1.41"),
        reflex_excess_over_pi=_interval("0.40", "0.41"),
        orientation=sector.turn_orientation,
        normalization_law=AngleNormalizationLaw.VALUE_OVER_SYMBOLIC_PI_V1,
    )
    valid_snapshot = replace(
        projection.snapshot,
        reflex_angle_certificates=frozenset(
            {replace(certificate, measure_payload=valid_payload)}
        ),
    )
    assert validate_cross_contract_references(
        valid_snapshot, projection.request, projection.plans
    ) == ()

    uncertain_payload = replace(
        valid_payload,
        phi_over_pi=_interval("1.33", "1.34"),
        reflex_excess_over_pi=_interval("0.33", "0.34"),
    )
    uncertain_snapshot = replace(
        valid_snapshot,
        reflex_angle_certificates=frozenset(
            {replace(certificate, measure_payload=uncertain_payload)}
        ),
    )
    assert any(
        issue.code is ValidationCode.ANGULAR_SELECTION_UNCERTAIN
        and issue.message == "ANGULAR_PROFILE_SELECTION_UNCERTAIN"
        for issue in validate_cross_contract_references(
            uncertain_snapshot, projection.request, projection.plans
        )
    )


def test_route_topology_variant_must_match_junction_kind(projections):
    projection = next(
        item
        for item in projections
        if any(
            not isinstance(relation.route_topology, UnavailableJunctionRouteTopologyV1)
            for relation in item.snapshot.junction_relations
        )
    )
    relation = next(iter(projection.snapshot.junction_relations))
    wrong_kind = (
        JunctionRelationKind.T
        if relation.relation_kind is not JunctionRelationKind.T
        else JunctionRelationKind.X
    )
    changed = replace(relation, relation_kind=wrong_kind)
    snapshot = replace(projection.snapshot, junction_relations=frozenset({changed}))
    assert any(
        issue.code is ValidationCode.ROUTE_TOPOLOGY
        for issue in validate_analysis_snapshot(snapshot)
    )


def test_unavailable_route_topology_is_a_tagged_fixture_state(projections):
    projection = next(item for item in projections if item.snapshot.junction_relations)
    relation = next(iter(projection.snapshot.junction_relations))
    changed = replace(
        relation,
        route_topology=UnavailableJunctionRouteTopologyV1(
            JunctionRouteTopologyUnavailableReason.EC0_ROUTE_TOPOLOGY_NOT_RECORDED_V5
        ),
    )
    snapshot = replace(projection.snapshot, junction_relations=frozenset({changed}))
    assert not any(
        issue.code is ValidationCode.ROUTE_TOPOLOGY
        for issue in validate_analysis_snapshot(snapshot)
    )


def test_deep_cross_validator_rejects_foreign_front_seed_and_missing_plan(projections):
    projection = _projection(projections, "EC0-C03")
    plan = projection.plans[0]
    front_seed = next(seed for seed in plan.seeds if isinstance(seed, FrontSeedV1))
    changed_seed = replace(front_seed, chain_use_id=ChainUseId("foreign-chain-use"))
    changed_plan = replace(plan, seeds=(plan.seeds - {front_seed}) | {changed_seed})
    issues = validate_cross_contract_references(
        projection.snapshot, projection.request, (changed_plan,)
    )
    assert any(issue.code is ValidationCode.MISSING_REFERENCE for issue in issues)
    assert any(
        issue.code is ValidationCode.CROSS_CONTRACT_MISMATCH
        for issue in validate_cross_contract_references(
            projection.snapshot, projection.request, ()
        )
    )


def test_deep_cross_validator_checks_geometry_batch_provenance(projections):
    projection = _projection(projections, "EC0-C03")
    plan = projection.plans[0]
    claim_id = next(iter(plan.ownership.claims)).ownership_claim_id
    chain_use_id = next(iter(projection.request.selected_chain_use_ids))
    batch = geometry_batch()
    provenance = replace(
        next(iter(batch.vertices)).provenance,
        source_face_ids=frozenset(),
        physical_edge_ids=frozenset(),
        chain_use_ids=frozenset({chain_use_id}),
    )
    batch = replace(
        batch,
        source_revision=projection.snapshot.source_revision,
        decal_request_id=projection.request.decal_request_id,
        patch_domain_id=plan.plan_key.patch_domain_id,
        vertices=frozenset(replace(item, provenance=provenance) for item in batch.vertices),
        faces=tuple(
            replace(item, ownership_claim_id=claim_id, provenance=provenance)
            for item in batch.faces
        ),
        station_facts=frozenset(
            replace(item, ownership_claim_id=claim_id) for item in batch.station_facts
        ),
        semantic_regions=frozenset(
            replace(item, ownership_claim_id=claim_id, provenance=provenance)
            for item in batch.semantic_regions
        ),
        semantic_digest=SemanticDigestValue("pending"),
    )
    batch = replace(
        batch,
        semantic_digest=SemanticDigestValue(
            geometry_batch_semantic_digest(batch).sha256_hex
        ),
    )
    assert validate_cross_contract_references(
        projection.snapshot, projection.request, projection.plans, (batch,)
    ) == ()

    face = batch.faces[0]
    bad_provenance = replace(
        face.provenance,
        physical_edge_ids=frozenset({PhysicalEdgeId("foreign-edge")}),
    )
    invalid = replace(batch, faces=(replace(face, provenance=bad_provenance),) + batch.faces[1:])
    assert any(
        issue.code is ValidationCode.MISSING_REFERENCE
        and "physical_edge_ids" in issue.path
        for issue in validate_cross_contract_references(
            projection.snapshot, projection.request, projection.plans, (invalid,)
        )
    )
