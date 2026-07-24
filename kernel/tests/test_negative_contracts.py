from __future__ import annotations

import json
from dataclasses import replace

import pytest

from cftuv_envelope import (
    AnalysisSnapshotCodecV1,
    AngularEnvelopeSpec,
    CompiledPlanCodecV1,
    ContractCodecError,
    CoverageId,
    DecalRequestCodecV1,
    FrontSeedId,
    SemanticDigestValue,
    ValidationCode,
    VertexKey,
    validate_analysis_snapshot,
    validate_compiled_plan,
    validate_geometry_batch,
)

from factories import geometry_batch


def _projection(projections, case_id):
    return next(item for item in projections if item.case_id == case_id)


def test_angular_relation_without_oriented_sector_is_rejected(projections):
    snapshot = _projection(projections, "EC0-C03").snapshot
    invalid = replace(snapshot, angular_owner_sectors=frozenset())
    assert any(issue.code is ValidationCode.MISSING_REFERENCE for issue in validate_analysis_snapshot(invalid))


def test_unordered_support_refs_are_rejected(projections):
    snapshot = _projection(projections, "EC0-C03").snapshot
    sector = next(iter(snapshot.angular_owner_sectors))
    changed = replace(
        sector,
        ordered_incident_chain_use_ids=tuple(reversed(sector.ordered_incident_chain_use_ids)),
    )
    invalid = replace(snapshot, angular_owner_sectors=frozenset({changed}))
    assert any(issue.code is ValidationCode.ANGULAR_CERTIFICATE for issue in validate_analysis_snapshot(invalid))


def test_exact_two_pi_is_not_an_angular_certificate(projections):
    snapshot = _projection(projections, "EC0-C03").snapshot
    certificate = next(iter(snapshot.reflex_angle_certificates))
    invalid = replace(
        snapshot,
        reflex_angle_certificates=frozenset({replace(certificate, exact_two_pi=True)}),
    )
    assert any(issue.code is ValidationCode.ANGULAR_CERTIFICATE for issue in validate_analysis_snapshot(invalid))


def test_manual_k_and_superseded_join_policy_are_not_request_fields(projections):
    request = _projection(projections, "EC0-C03").request
    data = json.loads(DecalRequestCodecV1.dumps(request))
    data["manual_hidden_edge_count"] = 1
    data["join_policy"] = "MITER"
    with pytest.raises(ContractCodecError):
        DecalRequestCodecV1.loads(json.dumps(data))


def test_runtime_event_cannot_be_smuggled_into_snapshot(projections):
    snapshot = _projection(projections, "EC0-C03").snapshot
    data = json.loads(AnalysisSnapshotCodecV1.dumps(snapshot))
    data["runtime_events"] = []
    with pytest.raises(ContractCodecError):
        AnalysisSnapshotCodecV1.loads(json.dumps(data))


def test_selection_k_and_k_plus_two_mismatch_are_rejected(projections):
    plan = _projection(projections, "EC0-C03").plans[0]
    certificate = next(iter(plan.angular_profile_selection_certificates))
    bad_certificate = replace(certificate, local_profile_support_count=99)
    invalid = replace(
        plan,
        angular_profile_selection_certificates=frozenset({bad_certificate}),
    )
    assert any(issue.code is ValidationCode.ANGULAR_CERTIFICATE for issue in validate_compiled_plan(invalid))


def test_envelope_spec_with_wrong_seed_is_rejected(projections):
    plan = _projection(projections, "EC0-C03").plans[0]
    strip = next(spec for spec in plan.envelope_specs if not isinstance(spec, AngularEnvelopeSpec))
    bad_strip = replace(strip, source_seed_id=FrontSeedId("missing-front-seed"))
    invalid = replace(plan, envelope_specs=(plan.envelope_specs - {strip}) | {bad_strip})
    assert any(issue.code is ValidationCode.MISSING_REFERENCE for issue in validate_compiled_plan(invalid))


def test_ownership_claim_on_foreign_coverage_is_rejected(projections):
    plan = _projection(projections, "EC0-C03").plans[0]
    claim = next(iter(plan.ownership.claims))
    bad_claim = replace(claim, coverage_id=CoverageId("foreign-coverage"))
    invalid = replace(
        plan,
        ownership=replace(
            plan.ownership,
            claims=(plan.ownership.claims - {claim}) | {bad_claim},
        ),
    )
    assert any(issue.code is ValidationCode.OWNERSHIP_DECLARATION for issue in validate_compiled_plan(invalid))


def test_coordinate_derived_vertex_key_is_rejected():
    batch = geometry_batch()
    vertex = next(iter(batch.vertices))
    bad_vertex = replace(vertex, vert_key=VertexKey("coord:0.0,0.0,0.0"))
    invalid = replace(
        batch,
        vertices=(batch.vertices - {vertex}) | {bad_vertex},
        semantic_digest=SemanticDigestValue("invalid"),
    )
    assert any(issue.code is ValidationCode.FORBIDDEN_TOPOLOGY_IDENTITY for issue in validate_geometry_batch(invalid))


def test_unknown_envelope_variant_fails_closed(projections):
    plan = _projection(projections, "EC0-C03").plans[0]
    data = json.loads(CompiledPlanCodecV1.dumps(plan))
    data["envelope_specs"][0]["$type"] = "RoundEnvelopeSpec"
    with pytest.raises(ContractCodecError):
        CompiledPlanCodecV1.loads(json.dumps(data))

