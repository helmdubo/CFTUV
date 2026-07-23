from __future__ import annotations

from dataclasses import replace
from inspect import signature

import pytest

from cftuv_envelope import (
    AngularProfileArrivalModelV1,
    EqualityLocusOwner,
    InteractionOutcome,
    RESOLVED_COVERAGE_RESULT_SCHEMA_V1,
    resolve_coverage_interactions,
    validate_resolved_coverage_digest,
)
from session_d_factories import angular_third_strip_state, opposing_strip_state


@pytest.mark.parametrize(
    ("alpha", "application_count"),
    (("4", 0), ("5", 1), ("6", 1)),
)
def test_opposing_strips_exact_before_at_after(alpha, application_count):
    _, raw, result = opposing_strip_state(alpha)

    assert result.outcome is InteractionOutcome.EXACT
    resolved = result.resolved_coverage
    assert resolved is not None
    assert resolved.schema_version == RESOLVED_COVERAGE_RESULT_SCHEMA_V1
    assert resolved.raw_coverage_digest == raw.semantic_digest
    assert resolved.exact_area_expression == raw.exact_area_expression
    assert len(resolved.interaction_applications) == application_count
    assert len(resolved.equality_loci) == application_count
    assert len(resolved.mutual_arrival_certificates) == application_count
    assert validate_resolved_coverage_digest(resolved)

    if application_count:
        application = resolved.interaction_applications[0]
        certificate = resolved.mutual_arrival_certificates[0]
        locus = resolved.equality_loci[0]
        assert application.creates_new_matter is False
        assert certificate.exact_alpha.as_expr() == 5
        assert locus.owner is EqualityLocusOwner.NONE
        assert locus.ordered_exact_segments
        assert {
            segment.start.x.as_expr()
            for segment in locus.ordered_exact_segments
        } == {5}
        assert {
            segment.end.x.as_expr()
            for segment in locus.ordered_exact_segments
        } == {5}
        assert all(
            item.active_domain_certificates[0].locus_reachable
            and item.active_domain_certificates[1].locus_reachable
            for item in resolved.mutual_arrival_certificates
        )


def test_unequal_source_distance_has_exact_rational_event():
    _, raw, result = opposing_strip_state("5", width=9.0)
    assert result.outcome is InteractionOutcome.EXACT
    resolved = result.resolved_coverage
    assert resolved is not None
    assert resolved.exact_area_expression == raw.exact_area_expression
    assert {
        item.exact_alpha.as_expr()
        for item in resolved.mutual_arrival_certificates
    } == {pytest.importorskip("sympy").Rational(9, 2)}


@pytest.mark.parametrize("hidden_count", (0, 1, 2))
def test_c13_third_strip_meets_actual_exposed_k_profile(hidden_count):
    _, before_raw, before = angular_third_strip_state(hidden_count, "4")
    assert before.outcome is InteractionOutcome.EXACT
    assert before.resolved_coverage is not None
    assert before.resolved_coverage.interaction_applications == ()
    assert before.resolved_coverage.raw_coverage_digest == before_raw.semantic_digest

    _, event_raw, event = angular_third_strip_state(hidden_count, "5")
    assert event.outcome is InteractionOutcome.EXACT
    assert event.resolved_coverage is not None
    assert event.resolved_coverage.interaction_applications
    assert event.resolved_coverage.exact_area_expression == event_raw.exact_area_expression

    compilation, after_raw, after = angular_third_strip_state(
        hidden_count, "5.1"
    )
    assert after.outcome is InteractionOutcome.EXACT
    resolved = after.resolved_coverage
    assert resolved is not None
    assert resolved.exact_area_expression == after_raw.exact_area_expression
    assert all(
        application.creates_new_matter is False
        for application in resolved.interaction_applications
    )
    assert {
        certificate.exact_alpha.as_expr()
        for certificate in resolved.mutual_arrival_certificates
    } == {5}
    models_by_id = {
        item.arrival_model_id: item for item in after.arrival_models
    }
    assert all(
        isinstance(
            models_by_id[certificate.left_front_reading.arrival_model_id],
            AngularProfileArrivalModelV1,
        )
        or isinstance(
            models_by_id[certificate.right_front_reading.arrival_model_id],
            AngularProfileArrivalModelV1,
        )
        for certificate in resolved.mutual_arrival_certificates
    )
    merged_certificates = tuple(
        certificate
        for certificate in resolved.mutual_arrival_certificates
        if (
            len(certificate.equivalent_left_front_readings)
            + len(certificate.equivalent_right_front_readings)
        )
        > 2
    )
    assert merged_certificates
    loci_by_id = {
        application.mutual_arrival_certificate_id: next(
            locus
            for locus in resolved.equality_loci
            if locus.locus_id == application.equality_locus_id
        )
        for application in resolved.interaction_applications
    }
    for certificate in merged_certificates:
        participant_ids = {
            reading.front_reading_id
            for reading in (
                *certificate.equivalent_left_front_readings,
                *certificate.equivalent_right_front_readings,
            )
        }
        assert (
            loci_by_id[certificate.certificate_id]
            .participant_front_reading_ids
            == participant_ids
        )
    angular_models = tuple(
        item
        for item in after.arrival_models
        if isinstance(item, AngularProfileArrivalModelV1)
    )
    assert angular_models
    assert {
        len(item.component_profile_arrival_laws)
        for item in angular_models
    } == {hidden_count + 2}
    angular_spec = next(
        item
        for item in compilation.envelope_specs
        if type(item).__name__ == "AngularEnvelopeSpec"
    )
    third_front = next(
        item.front_component_id
        for item in compilation.front_components
        if item.chain_use_id.value == "third-use"
    )
    assert third_front not in angular_spec.incident_front_component_ids


def test_raw_coverage_digest_is_a_hard_input_gate():
    compilation, raw, _ = opposing_strip_state("6")
    tampered = replace(raw, semantic_digest="0" * 64)
    result = resolve_coverage_interactions(
        compilation,
        tampered.boundary_resolved_envelopes,
        tampered,
    )
    assert result.outcome is InteractionOutcome.INTERACTION_INPUT_CONTRACT_INVALID
    assert result.resolved_coverage is None


def test_resolver_exposes_no_tolerance_parameter():
    assert "tolerance" not in signature(resolve_coverage_interactions).parameters


def test_reversed_input_order_is_semantically_stable():
    _, _, forward = opposing_strip_state("6")
    _, _, reversed_order = opposing_strip_state("6", reverse_sources=True)
    assert forward.outcome is reversed_order.outcome is InteractionOutcome.EXACT
    assert forward.resolved_coverage is not None
    assert reversed_order.resolved_coverage is not None
    assert (
        forward.resolved_coverage.semantic_digest
        == reversed_order.resolved_coverage.semantic_digest
    )


def test_rigid_transform_and_uniform_scale_preserve_interaction_law():
    _, _, base = opposing_strip_state("6")
    _, _, translated = opposing_strip_state(
        "6", translation=(17.0, -3.0)
    )
    _, _, scaled = opposing_strip_state("12", scale=2.0)
    for result in (base, translated, scaled):
        assert result.outcome is InteractionOutcome.EXACT
        assert result.resolved_coverage is not None
        assert len(result.resolved_coverage.interaction_applications) == 1
    assert (
        base.resolved_coverage.mutual_arrival_certificates[0].exact_alpha.as_expr()
        == translated.resolved_coverage.mutual_arrival_certificates[
            0
        ].exact_alpha.as_expr()
        == 5
    )
    assert (
        scaled.resolved_coverage.mutual_arrival_certificates[0].exact_alpha.as_expr()
        == 10
    )
    assert (
        scaled.resolved_coverage.exact_area_expression
        == "Integer(400)"
    )
