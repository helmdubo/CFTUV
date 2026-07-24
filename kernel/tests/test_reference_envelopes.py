from __future__ import annotations

from dataclasses import replace
from decimal import Decimal

import pytest

from cftuv_envelope import AngularEnvelopeSpec, ExactAngleV1, validate_analysis_snapshot
from cftuv_envelope.reference import (
    ReferenceOutcome,
    compile_reference_envelopes,
    evaluate_reference_raw_coverage,
)
from cftuv_envelope.reference.arrangement import (
    ExactArrangementCollinearBranchUnproven,
    ExactArrangementRotationSystemUnproven,
    ExactTouchingHoleTopologyUnproven,
)
from cftuv_envelope.reference import raw_coverage as raw_coverage_module
from cftuv_envelope.reference.planar_types import exact_sign

from reference_factories import angular_snapshot


@pytest.mark.parametrize("hidden_count", (0, 1, 2))
def test_linear_reflex_k_uses_one_angle_driven_algorithm(hidden_count):
    snapshot, request = angular_snapshot(hidden_count)
    assert validate_analysis_snapshot(snapshot) == ()
    compiled = compile_reference_envelopes(snapshot, request)
    assert compiled.outcome is ReferenceOutcome.EXACT
    angular = next(
        item
        for item in compiled.compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
    )
    assert angular.resolved_hidden_edge_count == hidden_count
    assert len(angular.hidden_supports) == hidden_count
    assert {
        (item.turn_fraction.numerator, item.turn_fraction.denominator)
        for item in angular.hidden_supports
    } == {(item, hidden_count + 1) for item in range(1, hidden_count + 1)}
    assert angular.all_support_normal_speed == 1
    selection = next(iter(compiled.compilation.profile_selection_certificates))
    assert selection.local_profile_support_count == hidden_count + 2
    assert selection.local_profile_segment_count == hidden_count + 2

    evaluated = evaluate_reference_raw_coverage(compiled.compilation, Decimal("1"))
    assert evaluated.outcome is ReferenceOutcome.EXACT
    raw = evaluated.raw_coverage
    angular_instance = next(
        item for item in raw.envelope_instances if item.envelope_variant == "AngularEnvelope"
    )
    assert len(angular_instance.provenance.support_ids) == hidden_count + 2
    assert exact_sign(raw.exact_area_expression) > 0
    assert any(
        angular.envelope_spec_id.value in edge.provenance.envelope_spec_ids
        for edge in raw.edges
    )


def test_unproven_rotation_system_has_precise_named_outcome(monkeypatch):
    snapshot, request = angular_snapshot(0)
    compilation = compile_reference_envelopes(
        snapshot, request
    ).compilation

    def reject_rotation_system(*args, **kwargs):
        raise ExactArrangementRotationSystemUnproven(
            "exact boundary rotation system is unproven"
        )

    monkeypatch.setattr(
        raw_coverage_module.REFERENCE_ARRANGEMENT_BACKEND,
        "exact_union",
        reject_rotation_system,
    )
    evaluated = evaluate_reference_raw_coverage(
        compilation, request.requested_alpha
    )

    assert (
        evaluated.outcome
        is ReferenceOutcome.REFERENCE_ARRANGEMENT_ROTATION_SYSTEM_UNPROVEN
    )
    assert "rotation system" in evaluated.diagnostics[0].message


@pytest.mark.parametrize(
    ("failure_type", "outcome"),
    (
        (
            ExactArrangementCollinearBranchUnproven,
            ReferenceOutcome.REFERENCE_ARRANGEMENT_COLLINEAR_BRANCH_UNPROVEN,
        ),
        (
            ExactTouchingHoleTopologyUnproven,
            ReferenceOutcome.REFERENCE_TOUCHING_HOLE_TOPOLOGY_UNPROVEN,
        ),
    ),
)
def test_specific_rotation_failures_keep_specific_outcomes(
    monkeypatch, failure_type, outcome
):
    snapshot, request = angular_snapshot(0)
    compilation = compile_reference_envelopes(
        snapshot, request
    ).compilation

    def reject(*args, **kwargs):
        raise failure_type(outcome.value)

    monkeypatch.setattr(
        raw_coverage_module.REFERENCE_ARRANGEMENT_BACKEND,
        "exact_union",
        reject,
    )

    evaluated = evaluate_reference_raw_coverage(
        compilation, request.requested_alpha
    )

    assert evaluated.outcome is outcome


def test_deleted_hidden_support_fails_named_instead_of_approximating_profile():
    snapshot, request = angular_snapshot(1)
    compiled = compile_reference_envelopes(snapshot, request).compilation
    angular = next(
        item for item in compiled.envelope_specs if isinstance(item, AngularEnvelopeSpec)
    )
    damaged = replace(
        compiled,
        envelope_specs=frozenset(
            replace(item, hidden_supports=frozenset()) if item == angular else item
            for item in compiled.envelope_specs
        ),
    )

    evaluated = evaluate_reference_raw_coverage(damaged, Decimal("1"))

    assert evaluated.outcome is ReferenceOutcome.ANGULAR_PROFILE_SELECTION_UNCERTAIN
    assert evaluated.raw_coverage is None


def test_non_contract_delta_max_is_rejected_before_profile_selection():
    snapshot, request = angular_snapshot(0)
    changed = replace(request, max_subturn_exact_value=ExactAngleV1("PI_OVER_4"))

    compiled = compile_reference_envelopes(snapshot, changed)

    assert compiled.outcome is ReferenceOutcome.REFERENCE_INPUT_CONTRACT_INVALID
    assert compiled.compilation is None


def test_angular_boundary_contact_fails_named_without_event_law():
    snapshot, request = angular_snapshot(0)
    compiled = compile_reference_envelopes(snapshot, request)

    evaluated = evaluate_reference_raw_coverage(
        compiled.compilation, Decimal("20")
    )

    assert (
        evaluated.outcome
        is ReferenceOutcome.REFERENCE_ANGULAR_BOUNDARY_CONTACT_UNPROVEN
    )
    assert evaluated.raw_coverage is None
