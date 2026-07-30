from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path

import pytest
import sympy as sp

import cftuv_envelope as kernel
from cftuv_envelope import (
    AdaptiveBoundHiddenSupportSpecV2,
    AdaptiveDensityAngularEnvelopeSpecV2,
    AngularEnvelopeSpec,
    AngularProfileSelectionPolicyId,
    ContractCodecError,
    DirectionBindingReasonV1,
    ExactAngleSymbol,
    ExactAngleV1,
    ExactRatioV1,
    ExactTurnSignV1,
    MaxSubturnParameterId,
    MaxSubturnValueId,
    ReferenceOutcome,
    TurnOrientation,
)
from cftuv_envelope.codec import ContractCodecV1
from cftuv_envelope.ids import CornerRelationId
from cftuv_envelope.adaptive_density_validation import (
    adaptive_density_structure_errors,
)
from cftuv_envelope.reference.adaptive_density_fan import (
    AdaptiveDensityFanInvalid,
    DensityRationalAuthorityExhausted,
    _subturn,
    certify_adaptive_density_fan,
    certify_density_bindings_and_adaptive_fallback,
    verify_adaptive_density_fan,
)
from cftuv_envelope.reference.angular import (
    _density_exact_sign,
    _density_exact_vector,
    _interpolated_normals,
)
from cftuv_envelope.reference.common import (
    GeometryContext,
    ReferenceGeometryError,
)
from cftuv_envelope.reference.metric import ExactPlanarMetric, _DensityExactMemo
from cftuv_envelope.reference.planar_types import ExactPlanarVector
from cftuv_envelope.wavefront import prepare_conveyor
from cftuv_envelope.reference.validation import (
    validate_reference_geometry_payload,
)


_FIELD = (
    Path(__file__).parents[1]
    / "fixtures"
    / "building_002_full_selection_v1"
)


class _AdaptiveDensityAngularCodecV2(
    ContractCodecV1[AdaptiveDensityAngularEnvelopeSpecV2]
):
    root_type = AdaptiveDensityAngularEnvelopeSpecV2


def _euclidean_metric() -> ExactPlanarMetric:
    return ExactPlanarMetric(
        (
            (sp.Rational(1), sp.Rational(0)),
            (sp.Rational(0), sp.Rational(1)),
        ),
        (
            (sp.Rational(1), sp.Rational(0)),
            (sp.Rational(0), sp.Rational(1)),
        ),
        1,
    )


def _minimal_repro():
    metric = _euclidean_metric()
    orientation = TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
    ideal = _interpolated_normals(
        metric,
        ExactPlanarVector.from_values(1, 0),
        ExactPlanarVector.from_values(3, 1),
        1,
        orientation,
        huber_density=True,
    )
    return metric, orientation, ideal


def test_density_exact_memo_is_owned_by_one_metric_transaction():
    first = _euclidean_metric()
    second = _euclidean_metric()
    expression = sp.sqrt(2) - 1

    assert first == second
    assert first._density_exact_memo is not second._density_exact_memo
    assert _density_exact_sign(expression, first) == 1
    assert expression in first._density_exact_memo.signs
    assert expression not in second._density_exact_memo.signs
    assert first == second


def test_density_compile_does_not_use_global_parser_for_algebraic_values(
    monkeypatch,
):
    import cftuv_envelope.reference.planar_types as planar_types

    snapshot, request = _field_inputs(4)
    original = planar_types._parse_expr
    seen = []

    def forbidden(expression):
        seen.append(expression)
        raise AssertionError(
            f"Density compile called process-global parser for {expression}"
        )

    planar_types._parse_expr.cache_clear()
    monkeypatch.setattr(planar_types, "_parse_expr", forbidden)
    first = kernel.compile_reference_envelopes(snapshot, request)
    for value in range(-20, 21):
        original(f"Integer({value})")
    second = kernel.compile_reference_envelopes(snapshot, request)

    assert first.outcome is ReferenceOutcome.EXACT
    assert second.outcome is ReferenceOutcome.EXACT
    assert first.compilation == second.compilation
    assert seen == []


def test_density_component_is_independent_of_global_parser_population(
    monkeypatch,
):
    import cftuv_envelope.reference.planar_types as planar_types
    from cftuv_envelope.wavefront import conveyor

    snapshot, request = _field_inputs(4)
    original = planar_types._parse_expr
    calls = []

    def forbidden(expression):
        calls.append(expression)
        raise AssertionError(
            f"Density component called process-global parser for {expression}"
        )

    def run():
        inputs, refusal = conveyor._prepare_inputs(
            snapshot,
            request,
            None,
            conveyor._Clock(),
        )
        assert refusal is None
        compilation, _, domain, reading, lattice, residual = inputs
        return compilation, domain, reading, lattice, residual

    planar_types._parse_expr.cache_clear()
    monkeypatch.setattr(planar_types, "_parse_expr", forbidden)
    clear = run()
    for value in range(-20, 21):
        original(f"Integer({value})")
    populated = run()

    assert clear == populated
    assert calls == []


def _field_inputs(density: int):
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (_FIELD / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (_FIELD / "decal_request.json").read_bytes()
    )
    values = (
        (
            MaxSubturnValueId.LINEAR_REFLEX_DENSITY_0_V1,
            ExactAngleSymbol.PI_OVER_2,
        ),
        (
            MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1,
            ExactAngleSymbol.PI_OVER_3,
        ),
        (
            MaxSubturnValueId.LINEAR_REFLEX_DENSITY_2_V1,
            ExactAngleSymbol.PI_OVER_4,
        ),
        (
            MaxSubturnValueId.LINEAR_REFLEX_DENSITY_3_V1,
            ExactAngleSymbol.PI_OVER_5,
        ),
        (
            MaxSubturnValueId.LINEAR_REFLEX_DENSITY_4_V1,
            ExactAngleSymbol.PI_OVER_6,
        ),
    )
    value_id, symbol = values[density]
    return snapshot, replace(
        request,
        angular_profile_selection_policy_id=(
            AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1
        ),
        max_subturn_parameter_id=(
            MaxSubturnParameterId.LINEAR_REFLEX_DENSITY_A_V1
        ),
        max_subturn_value_id=value_id,
        max_subturn_exact_value=ExactAngleV1(symbol),
    )


def test_minimal_repro_has_common_height_five_and_vector_five_one():
    metric, orientation, ideal = _minimal_repro()

    authority = certify_adaptive_density_fan(
        metric,
        ideal,
        orientation,
        2,
        binding_reasons=(None,),
    )

    assert authority.minimal_common_height == 5
    assert authority.exhaustive_previous_height == 4
    assert authority.bound_primitive_integer_vectors == ((5, 1),)
    assert (
        authority.previous_height_witness.primitive_candidate_counts
        == (0,)
    )
    verify_adaptive_density_fan(metric, ideal, orientation, authority)


def test_minimality_and_bound_vector_forgery_fail_before_consumption():
    metric, orientation, ideal = _minimal_repro()
    authority = certify_adaptive_density_fan(
        metric,
        ideal,
        orientation,
        2,
        binding_reasons=(None,),
    )

    for forged in (
        replace(authority, minimal_common_height=4),
        replace(
            authority,
            bound_primitive_integer_vectors=((4, 1),),
        ),
    ):
        with pytest.raises(AdaptiveDensityFanInvalid):
            verify_adaptive_density_fan(
                metric,
                ideal,
                orientation,
                forged,
            )


def test_resource_height_cap_has_reachable_named_failure():
    metric, orientation, ideal = _minimal_repro()

    with pytest.raises(
        DensityRationalAuthorityExhausted,
        match="DENSITY_RATIONAL_AUTHORITY_EXHAUSTED",
    ):
        certify_adaptive_density_fan(
            metric,
            ideal,
            orientation,
            2,
            binding_reasons=(None,),
            resource_height_cap=4,
        )


def test_field_density_one_replaces_only_empty_old_authorities():
    snapshot, request = _field_inputs(1)

    result = kernel.compile_reference_envelopes(snapshot, request)

    assert result.outcome is ReferenceOutcome.EXACT, result.diagnostics
    angular = tuple(
        item
        for item in result.compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
    )
    adaptive = tuple(
        item
        for item in angular
        if type(item) is AdaptiveDensityAngularEnvelopeSpecV2
    )
    assert len(angular) == 4
    assert len(adaptive) == 2
    assert {
        (
            item.source_relation_id.value,
            item.direction_fan_authority.minimal_common_height,
            item.direction_fan_authority.bound_primitive_integer_vectors,
        )
        for item in adaptive
    } == {
        (
            "host-v0:corner-relation:60fc81c8f39e546c88f9ff6a",
            5,
            ((-1, -5),),
        ),
        (
            "host-v0:corner-relation:8728a65c43ea218fe1d1544a",
            5,
            ((1, 5),),
        ),
    }
    assert all(
        type(support) is AdaptiveBoundHiddenSupportSpecV2
        for item in adaptive
        for support in item.hidden_supports
    )


def test_field_density_four_has_one_exact_minimal_h_lift():
    snapshot, request = _field_inputs(4)

    result = kernel.compile_reference_envelopes(snapshot, request)

    assert result.outcome is ReferenceOutcome.EXACT, result.diagnostics
    adaptive = tuple(
        item
        for item in result.compilation.envelope_specs
        if type(item) is AdaptiveDensityAngularEnvelopeSpecV2
    )
    assert {
        (
            item.source_relation_id.value,
            item.resolved_hidden_edge_count,
            item.direction_fan_authority.minimal_common_height,
            item.direction_fan_authority.bound_primitive_integer_vectors,
        )
        for item in adaptive
    } == {
        (
            "host-v0:corner-relation:05676524690dd8b689264248",
            3,
            13,
            ((-4, -13), (-1, -3), (-5, -13)),
        ),
        (
            "host-v0:corner-relation:60fc81c8f39e546c88f9ff6a",
            2,
            4492,
            ((-17, -225), (-1075, -4492)),
        ),
        (
            "host-v0:corner-relation:8728a65c43ea218fe1d1544a",
            2,
            397,
            ((30, 397), (28, 117)),
        ),
        (
            "host-v0:corner-relation:a2b66a4ed578c532315cd09b",
            3,
            13,
            ((4, 13), (1, 3), (5, 13)),
        ),
    }
    lifted = tuple(
        item for item in adaptive
        if item.evaluation_subturn_count_lift is not None
    )
    assert len(lifted) == 1
    lift = lifted[0].evaluation_subturn_count_lift
    assert lift.source_hidden_edge_count == 2
    assert lift.effective_hidden_edge_count == 3
    hidden_ids = {
        support.hidden_support_id
        for item in result.compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
        for support in item.hidden_supports
    }
    feature_ids = {
        item.source_id
        for item in result.compilation.initial_front_spec.support_features
        if item.kind.value == "ANGULAR_HIDDEN_SUPPORT"
    }
    assert feature_ids == hidden_ids
    lifted_provenance = next(
        item.provenance
        for item in result.compilation.source_provenance
        if item.envelope_spec_id == lifted[0].envelope_spec_id.value
    )
    assert lifted_provenance.support_ids == frozenset(
        item.hidden_support_id.value
        for item in lifted[0].hidden_supports
    )


def test_density_context_names_missing_corner_relation_before_supports():
    snapshot, request = _field_inputs(4)
    result = kernel.compile_reference_envelopes(snapshot, request)
    compilation = result.compilation
    frame, diagnostics = validate_reference_geometry_payload(
        snapshot,
        compilation.plan_key.patch_domain_id,
        density_bounded=True,
    )
    assert frame is not None, diagnostics
    missing = CornerRelationId("missing-density-corner-relation")
    forged = replace(
        compilation,
        envelope_specs=frozenset(
            (
                replace(item, source_relation_id=missing)
                if isinstance(item, AngularEnvelopeSpec)
                else item
            )
            for item in compilation.envelope_specs
        ),
    )
    memo = _DensityExactMemo()

    with pytest.raises(ReferenceGeometryError) as failure:
        GeometryContext.build(
            forged,
            frame,
            density_exact_memo=memo,
        )

    assert failure.value.outcome is ReferenceOutcome.REFERENCE_INPUT_CONTRACT_INVALID
    assert "unknown CornerRelation" in str(failure.value)
    assert memo.support_segments == {}


def test_density_context_names_foreign_existing_corner_relation():
    snapshot, request = _field_inputs(4)
    result = kernel.compile_reference_envelopes(snapshot, request)
    compilation = result.compilation
    frame, diagnostics = validate_reference_geometry_payload(
        snapshot,
        compilation.plan_key.patch_domain_id,
        density_bounded=True,
    )
    assert frame is not None, diagnostics
    foreign = min(
        snapshot.corner_relations,
        key=lambda item: item.corner_relation_id.value,
    ).corner_relation_id
    forged = replace(
        compilation,
        envelope_specs=frozenset(
            (
                replace(item, source_relation_id=foreign)
                if isinstance(item, AngularEnvelopeSpec)
                else item
            )
            for item in compilation.envelope_specs
        ),
    )

    with pytest.raises(ReferenceGeometryError) as failure:
        GeometryContext.build(forged, frame)

    assert failure.value.outcome is ReferenceOutcome.PLANAR_CHAIN_SUPPORT_NOT_LINEAR


def test_farey_witness_and_fan_tampering_fail_closed():
    metric, orientation, ideal = _minimal_repro()
    authority = certify_adaptive_density_fan(
        metric,
        ideal,
        orientation,
        2,
        binding_reasons=(None,),
    )
    window = authority.ordinal_windows[0]
    witness = authority.previous_height_witness
    forged = (
        replace(authority, minimal_common_height=4),
        replace(authority, minimal_common_height=6),
        replace(authority, exhaustive_previous_height=True),
        replace(authority, max_subturn_q=3),
        replace(
            authority,
            bound_primitive_integer_vectors=((-5, -1),),
        ),
        replace(
            authority,
            bound_primitive_integer_vectors=((10, 2),),
        ),
        replace(
            authority,
            ordinal_windows=(
                replace(window, denominator_sign=-window.denominator_sign),
            ),
        ),
        replace(
            authority,
            ordinal_windows=(
                replace(window, ordinal=2),
            ),
        ),
        replace(
            authority,
            previous_height_witness=replace(
                witness,
                first_height=2,
            ),
        ),
        replace(
            authority,
            previous_height_witness=replace(
                witness,
                last_height=witness.last_height - 1,
            ),
        ),
        replace(
            authority,
            previous_height_witness=replace(
                witness,
                primitive_candidate_counts=(1,),
            ),
        ),
    )

    for item in forged:
        with pytest.raises(AdaptiveDensityFanInvalid):
            verify_adaptive_density_fan(
                metric,
                ideal,
                orientation,
                item,
            )


def test_v1_preservation_is_per_consumed_ordinal_not_all_records(
    monkeypatch,
):
    import cftuv_envelope.reference.adaptive_density_fan as module

    metric = _euclidean_metric()
    orientation = TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
    ideal = _interpolated_normals(
        metric,
        ExactPlanarVector.from_values(1, 0),
        ExactPlanarVector.from_values(0, 1),
        2,
        orientation,
        huber_density=True,
    )
    sentinel = object()
    answers = iter((None, sentinel))
    monkeypatch.setattr(
        module,
        "_legacy_density_certificate",
        lambda *args, **kwargs: next(answers),
    )
    monkeypatch.setattr(
        module,
        "_legacy_full_fan_valid",
        lambda *args, **kwargs: True,
    )

    certificates, authority = (
        certify_density_bindings_and_adaptive_fallback(
            metric,
            ideal,
            orientation,
            3,
            binding_reasons=(
                None,
                DirectionBindingReasonV1.SOURCE_DIRECTION_IRRATIONAL,
            ),
        )
    )

    assert certificates == (None, sentinel)
    assert authority is None


def test_h_lift_tampering_fails_before_geometry_consumption():
    snapshot, request = _field_inputs(4)
    result = kernel.compile_reference_envelopes(snapshot, request)
    compilation = result.compilation
    lifted = next(
        item
        for item in compilation.envelope_specs
        if type(item) is AdaptiveDensityAngularEnvelopeSpecV2
        and item.evaluation_subturn_count_lift is not None
    )
    lift = lifted.evaluation_subturn_count_lift
    foreign_selection_id = next(
        item.certificate_id
        for item in compilation.profile_selection_certificates
        if item.certificate_id != lifted.selection_certificate_id
    )
    forged_lifts = (
        None,
        replace(lift, source_hidden_edge_count=True),
        replace(lift, effective_hidden_edge_count=2),
        replace(lift, max_subturn_q=5),
        replace(lift, evaluation_turn_sign=ExactTurnSignV1.POSITIVE),
        replace(
            lift,
            evaluation_turn_cosine_squared=ExactRatioV1(0, 1),
        ),
        replace(
            lift,
            source_selection_certificate_id=foreign_selection_id,
        ),
        replace(
            lift,
            minimality_predecessor_hidden_edge_count=1,
        ),
    )
    frame, diagnostics = validate_reference_geometry_payload(
        snapshot,
        compilation.plan_key.patch_domain_id,
        density_bounded=True,
    )
    assert frame is not None, diagnostics

    for forged_lift in forged_lifts:
        forged_spec = replace(
            lifted,
            evaluation_subturn_count_lift=forged_lift,
        )
        forged_compilation = replace(
            compilation,
            envelope_specs=frozenset(
                forged_spec if item == lifted else item
                for item in compilation.envelope_specs
            ),
        )
        with pytest.raises(ReferenceGeometryError):
            GeometryContext.build(forged_compilation, frame)


def test_adaptive_spec_codec_roundtrip_and_tags_fail_closed():
    snapshot, request = _field_inputs(1)
    result = kernel.compile_reference_envelopes(snapshot, request)
    spec = next(
        item
        for item in result.compilation.envelope_specs
        if type(item) is AdaptiveDensityAngularEnvelopeSpecV2
    )

    payload = _AdaptiveDensityAngularCodecV2.dumps(spec)
    assert _AdaptiveDensityAngularCodecV2.loads(payload) == spec
    data = json.loads(payload)

    data["$type"] = "AngularEnvelopeSpec"
    with pytest.raises(ContractCodecError, match="expected record"):
        _AdaptiveDensityAngularCodecV2.loads(json.dumps(data))

    data["$type"] = "UnknownAdaptiveDensitySpecV9"
    with pytest.raises(ContractCodecError, match="expected record"):
        _AdaptiveDensityAngularCodecV2.loads(json.dumps(data))


def test_adaptive_structural_validator_refuses_malformed_shapes():
    snapshot, request = _field_inputs(1)
    result = kernel.compile_reference_envelopes(snapshot, request)
    angular = tuple(
        item
        for item in result.compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
    )
    adaptive = next(
        item
        for item in angular
        if type(item) is AdaptiveDensityAngularEnvelopeSpecV2
    )
    foreign_support = next(
        iter(
            next(
                item
                for item in angular
                if type(item) is AngularEnvelopeSpec
            ).hidden_supports
        )
    )
    support = next(iter(adaptive.hidden_supports))
    malformed = (
        replace(
            adaptive,
            hidden_supports=frozenset({foreign_support}),
        ),
        replace(
            adaptive,
            hidden_supports=frozenset(
                {
                    replace(
                        support,
                        bound_primitive_integer_vector=(),
                    )
                }
            ),
        ),
        replace(
            adaptive,
            direction_fan_authority=replace(
                adaptive.direction_fan_authority,
                ordinal_windows=(),
            ),
        ),
    )

    for forged in malformed:
        assert adaptive_density_structure_errors(forged)


def test_field_density_one_is_full_fan_without_miter_degradation():
    snapshot, request = _field_inputs(1)

    prepared = prepare_conveyor(snapshot, request)

    assert prepared.outcome.value == "EXACT", prepared.detail
    assert prepared.counter("CONVEYOR_RATIONAL_VERTEX_FANS") == 4
    assert prepared.counter("CONVEYOR_DEGRADED_MITER_CORNERS") == 0
    assert prepared.counter("CONVEYOR_BOUND_FAN_DIRECTIONS") == 4
    assert prepared.counter("CONVEYOR_FAN_EDGES") == 4


def test_explicit_density_refuses_instead_of_degraded_miter(monkeypatch):
    from cftuv_envelope.wavefront import conveyor as module

    snapshot, request = _field_inputs(1)
    original = module.angular_hidden_support_lines

    def algebraic(context, spec):
        relation, anchor, hidden = original(context, spec)
        return (
            relation,
            anchor,
            tuple(
                (
                    support_id,
                    ExactPlanarVector.from_values(sp.sqrt(3), 1),
                    constant,
                )
                for support_id, _, constant in hidden
            ),
        )

    monkeypatch.setattr(module, "angular_hidden_support_lines", algebraic)
    prepared = module.prepare_conveyor(snapshot, request)

    assert (
        prepared.outcome
        is module.ConveyorOutcome.DENSITY_SEALED_FAN_INVALID
    )
    assert prepared.detail == "DENSITY_SEALED_FAN_INVALID"
    assert prepared.regions == ()
    assert prepared.counter("CONVEYOR_DEGRADED_MITER_CORNERS") == 0


def test_adaptive_fan_calls_no_forbidden_symbolic_solver(monkeypatch):
    metric = _euclidean_metric()
    orientation = TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
    calls = []

    def forbidden(name):
        def reject(*args, **kwargs):
            del args, kwargs
            calls.append(name)
            raise AssertionError(name)

        return reject

    for name in ("factor", "roots", "solve", "nroots", "N"):
        monkeypatch.setattr(sp, name, forbidden(name))
    for q in range(2, 7):
        ideal = _interpolated_normals(
            metric,
            ExactPlanarVector.from_values(1, 0),
            ExactPlanarVector.from_values(0, 1),
            q - 1,
            orientation,
            huber_density=True,
        )
        authority = certify_adaptive_density_fan(
            metric,
            ideal,
            orientation,
            q,
            binding_reasons=(None,) * (q - 1),
        )
        verify_adaptive_density_fan(
            metric,
            ideal,
            orientation,
            authority,
        )

    assert calls == []


@pytest.mark.parametrize(
    ("q", "right"),
    (
        (2, (0, 1)),
        (3, (1, sp.sqrt(3))),
        (4, (1, 1)),
        (
            5,
            (
                sp.sqrt((3 + sp.sqrt(5)) / 8),
                sp.sqrt((5 - sp.sqrt(5)) / 8),
            ),
        ),
        (6, (sp.sqrt(3), 1)),
    ),
)
def test_density_subturn_accepts_exact_closed_boundary(q, right):
    metric = _euclidean_metric()
    left = _density_exact_vector(1, 0)
    boundary = _density_exact_vector(*right)

    assert _subturn(metric, left, boundary, q)


def test_full_density_prepare_calls_no_forbidden_symbolic_solver(
    monkeypatch,
):
    calls = []

    def forbidden(name):
        def reject(*args, **kwargs):
            del args, kwargs
            calls.append(name)
            raise AssertionError(name)

        return reject

    for name in ("factor", "roots", "solve", "nroots", "N"):
        monkeypatch.setattr(sp, name, forbidden(name))
    snapshot, request = _field_inputs(1)

    prepared = prepare_conveyor(snapshot, request)

    assert prepared.outcome.value == "EXACT", prepared.detail
    assert calls == []
