from __future__ import annotations

import copy
import json
from dataclasses import replace

import pytest

from cftuv_envelope import (
    AngularEnvelopeSpec,
    AnalysisSnapshotCodecV1,
    ContractCodecError,
    DecalRequestCodecV1,
    DecalRequestId,
    MaxSubturnValueId,
    semantic_plan_digest,
    snapshot_digest,
)
from ec0_adapter import project_case_data
from factories import geometry_batch
from cftuv_envelope import geometry_batch_semantic_digest


def _load_case(fixture_root, name):
    return json.loads((fixture_root / "cases" / name).read_text(encoding="utf-8"))


def test_semantically_unordered_record_permutation_is_invariant(fixture_root):
    raw = _load_case(fixture_root, "ec0-c03-linear-reflex-k1.json")
    permuted = copy.deepcopy(raw)
    snapshot = permuted["analysis_snapshot"]
    for field in (
        "patches",
        "patch_domains",
        "source_vertices",
        "physical_chains",
        "chain_uses",
        "boundary_constraints",
        "corner_relations",
        "angular_owner_sectors",
        "reflex_angle_certificates",
    ):
        snapshot[field].reverse()
    evaluation = permuted["expected_compiled_plan"]["patch_evaluations"][0]
    for field in (
        "seeds",
        "front_components",
        "envelope_specs",
        "envelope_instances",
        "angular_profile_selection_certificates",
    ):
        evaluation[field].reverse()
    evaluation["ownership"]["claims"].reverse()
    first = project_case_data(raw)
    second = project_case_data(permuted)
    assert AnalysisSnapshotCodecV1.dumps(first.snapshot) == AnalysisSnapshotCodecV1.dumps(second.snapshot)
    assert first.plans == second.plans


def test_hidden_support_storage_permutation_uses_explicit_ordinals(projections):
    plan = next(item for item in projections if item.case_id == "EC0-C03").plans[0]
    angular = next(spec for spec in plan.envelope_specs if isinstance(spec, AngularEnvelopeSpec))
    reconstructed = replace(angular, hidden_supports=frozenset(reversed(tuple(angular.hidden_supports))))
    changed_plan = replace(plan, envelope_specs=(plan.envelope_specs - {angular}) | {reconstructed})
    assert semantic_plan_digest(plan) == semantic_plan_digest(changed_plan)


def test_ordered_owner_sector_support_change_is_semantic(projections):
    snapshot = next(item for item in projections if item.case_id == "EC0-C03").snapshot
    sector = next(iter(snapshot.angular_owner_sectors))
    changed = replace(
        sector,
        ordered_incident_chain_use_ids=tuple(reversed(sector.ordered_incident_chain_use_ids)),
    )
    changed_snapshot = replace(snapshot, angular_owner_sectors=frozenset({changed}))
    assert snapshot_digest(snapshot) != snapshot_digest(changed_snapshot)


def test_resolved_k_change_is_semantic(projections):
    plan = next(item for item in projections if item.case_id == "EC0-C03").plans[0]
    certificate = next(iter(plan.angular_profile_selection_certificates))
    changed = replace(
        plan,
        angular_profile_selection_certificates=frozenset(
            {replace(certificate, resolved_hidden_edge_count=certificate.resolved_hidden_edge_count + 1)}
        ),
    )
    assert semantic_plan_digest(plan) != semantic_plan_digest(changed)


def test_request_identity_change_is_semantic(projections):
    request = projections[0].request
    changed = replace(request, decal_request_id=DecalRequestId("different-request"))
    assert DecalRequestCodecV1.dumps(request) != DecalRequestCodecV1.dumps(changed)


def test_max_subturn_and_interaction_policy_changes_require_new_v1_enum(projections):
    request = projections[0].request
    with pytest.raises(ValueError):
        MaxSubturnValueId("SILENTLY_RETUNED_ANGLE")
    data = json.loads(DecalRequestCodecV1.dumps(request))
    data["interaction_policy_id"] = "OWNERSHIP_ONLY_POLICY_A"
    with pytest.raises(ContractCodecError):
        DecalRequestCodecV1.loads(json.dumps(data))


def test_data_segmentation_and_retriangulation_do_not_change_plan_digest(projections):
    projection = next(item for item in projections if item.case_id == "EC0-C07")
    plan_digest = semantic_plan_digest(projection.plans[0])
    changed_snapshot = replace(
        projection.snapshot,
        physical_chains=frozenset(
            replace(chain, data_record_lineage=frozenset())
            for chain in projection.snapshot.physical_chains
        ),
    )
    assert snapshot_digest(changed_snapshot) != snapshot_digest(projection.snapshot)
    assert semantic_plan_digest(projection.plans[0]) == plan_digest


def test_downstream_tessellation_variation_is_digest_invariant():
    assert geometry_batch_semantic_digest(
        geometry_batch(alternate_diagonal=False)
    ) == geometry_batch_semantic_digest(geometry_batch(alternate_diagonal=True))
