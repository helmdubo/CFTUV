from __future__ import annotations

from dataclasses import replace
from hashlib import sha256
import json
from pathlib import Path

import pytest
import sympy as sp

import cftuv_envelope as kernel
from cftuv_envelope import (
    AdaptiveBoundHiddenSupportSpecV2,
    AdaptiveDensityAngularEnvelopeSpecV2,
    AdaptiveMinimalRationalFanAuthorityV2,
    AdaptiveProjectivePoleOwnershipV1,
    AdaptiveRationalFanOrdinalWindowAtlasV1,
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
    DensityWindowChartUnrepresentable,
    _subturn,
    certify_adaptive_density_fan,
    certify_density_bindings_and_adaptive_fallback,
    verify_adaptive_density_fan,
    verify_sealed_adaptive_density_fan,
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


class _AdaptiveFanAuthorityCodecV2(
    ContractCodecV1[AdaptiveMinimalRationalFanAuthorityV2]
):
    root_type = AdaptiveMinimalRationalFanAuthorityV2


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


def _projective_atlas_repro(*, swap_coordinates: bool = False):
    gram = (
        (sp.Rational(18), sp.Rational(15)),
        (sp.Rational(15), sp.Rational(13)),
    )
    inverse = (
        (sp.Rational(13, 9), sp.Rational(-5, 3)),
        (sp.Rational(-5, 3), sp.Rational(2)),
    )
    normals = (
        ExactPlanarVector.from_values(sp.Rational(-58, 75), sp.Rational(17, 25)),
        ExactPlanarVector.from_values(-1, sp.Rational(17, 13)),
        ExactPlanarVector.from_values(sp.Rational(29, 51), sp.Rational(-7, 17)),
    )
    owner_orientation_sign = 1
    if swap_coordinates:
        gram = ((gram[1][1], gram[1][0]), (gram[0][1], gram[0][0]))
        inverse = (
            (inverse[1][1], inverse[1][0]),
            (inverse[0][1], inverse[0][0]),
        )
        normals = tuple(
            ExactPlanarVector.from_values(
                normal.expressions()[1],
                normal.expressions()[0],
            )
            for normal in normals
        )
        owner_orientation_sign = -1
    return (
        ExactPlanarMetric(
            gram,
            inverse,
            owner_orientation_sign,
        ),
        TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION,
        normals,
    )


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


def test_single_chart_authority_bytes_remain_frozen():
    metric, orientation, ideal = _minimal_repro()

    authority = certify_adaptive_density_fan(
        metric,
        ideal,
        orientation,
        2,
        binding_reasons=(None,),
    )
    payload = kernel.canonical_json_bytes(authority)

    assert len(payload) == 1977
    assert sha256(payload).hexdigest() == (
        "1302da818b2cf980043a7f1b2b70924db0cad3ab7cc770688d324cc2a6f9dc9a"
    )


def test_projective_atlas_has_chart_invariant_minimal_height():
    metric, orientation, ideal = _projective_atlas_repro()
    swapped_metric, swapped_orientation, swapped_ideal = (
        _projective_atlas_repro(swap_coordinates=True)
    )

    authority = certify_adaptive_density_fan(
        metric,
        ideal,
        orientation,
        2,
        binding_reasons=(None,),
    )
    swapped = certify_adaptive_density_fan(
        swapped_metric,
        swapped_ideal,
        swapped_orientation,
        2,
        binding_reasons=(None,),
    )

    assert authority.minimal_common_height == 4
    assert authority.exhaustive_previous_height == 3
    assert authority.previous_height_witness.primitive_candidate_counts == (0,)
    assert authority.bound_primitive_integer_vectors == ((3, 4),)
    assert swapped.minimal_common_height == 4
    assert swapped.bound_primitive_integer_vectors == ((4, 3),)
    window = authority.ordinal_windows[0]
    assert type(window) is AdaptiveRationalFanOrdinalWindowAtlasV1
    assert tuple(piece.pole_ownership for piece in window.pieces) == (
        AdaptiveProjectivePoleOwnershipV1.Y_ZERO,
        AdaptiveProjectivePoleOwnershipV1.X_ZERO,
        AdaptiveProjectivePoleOwnershipV1.NONE,
    )
    verify_sealed_adaptive_density_fan(authority, ideal_count=3)
    verify_adaptive_density_fan(metric, ideal, orientation, authority)
    verify_adaptive_density_fan(
        swapped_metric,
        swapped_ideal,
        swapped_orientation,
        swapped,
    )


def test_projective_atlas_codec_and_partition_are_fail_closed():
    metric, orientation, ideal = _projective_atlas_repro()
    authority = certify_adaptive_density_fan(
        metric,
        ideal,
        orientation,
        2,
        binding_reasons=(None,),
    )
    payload = _AdaptiveFanAuthorityCodecV2.dumps(authority)
    assert _AdaptiveFanAuthorityCodecV2.dumps(
        _AdaptiveFanAuthorityCodecV2.loads(payload)
    ) == payload

    document = json.loads(payload)
    atlas = document["ordinal_windows"][0]
    for tag in (
        "UnknownProjectiveAtlas",
        "AdaptiveRationalFanOrdinalWindowV2",
    ):
        forged = json.loads(payload)
        forged["ordinal_windows"][0]["$type"] = tag
        with pytest.raises(ContractCodecError):
            _AdaptiveFanAuthorityCodecV2.loads(
                json.dumps(forged).encode()
            )
    forged = json.loads(payload)
    forged["ordinal_windows"][0]["unexpected"] = True
    with pytest.raises(ContractCodecError):
        _AdaptiveFanAuthorityCodecV2.loads(json.dumps(forged).encode())
    forged = json.loads(payload)
    del forged["ordinal_windows"][0]["pieces"]
    with pytest.raises(ContractCodecError):
        _AdaptiveFanAuthorityCodecV2.loads(json.dumps(forged).encode())

    window = authority.ordinal_windows[0]
    left, right = window.pieces[:2]
    for broken in (
        replace(
            authority,
            ordinal_windows=(
                replace(
                    window,
                    pieces=(
                        replace(left, lower_endpoint_included=False),
                        right,
                        *window.pieces[2:],
                    ),
                ),
            ),
        ),
        replace(
            authority,
            ordinal_windows=(
                replace(
                    window,
                    pieces=(
                        left,
                        replace(
                            right,
                            lower_slope_envelope=right.upper_slope_envelope,
                        ),
                        *window.pieces[2:],
                    ),
                ),
            ),
        ),
    ):
        with pytest.raises(AdaptiveDensityFanInvalid):
            verify_sealed_adaptive_density_fan(broken, ideal_count=3)


def test_public_compile_names_unrepresentable_density_window(monkeypatch):
    import cftuv_envelope.reference.adaptive_density_fan as fan_module

    snapshot, request = _field_inputs(1)

    def refuse(*args, **kwargs):
        del args, kwargs
        raise DensityWindowChartUnrepresentable(
            "DENSITY_WINDOW_CHART_UNREPRESENTABLE"
        )

    monkeypatch.setattr(
        fan_module,
        "certify_density_bindings_and_adaptive_fallback",
        refuse,
    )
    result = kernel.compile_reference_envelopes(snapshot, request)

    assert result.outcome is ReferenceOutcome.DENSITY_WINDOW_CHART_UNREPRESENTABLE
    assert result.compilation is None
    messages = tuple(item.message for item in result.diagnostics)
    assert messages == (
        "DENSITY_WINDOW_CHART_UNREPRESENTABLE",
    )
    assert all("BINDING_MONOTONE" not in item for item in messages)
    assert all("Traceback" not in item for item in messages)


def test_projective_atlas_hot_path_uses_no_forbidden_symbolic_operations(
    monkeypatch,
):
    metric, orientation, ideal = _projective_atlas_repro()
    calls = []

    def forbidden(*args, **kwargs):
        calls.append((args, kwargs))
        raise AssertionError("forbidden symbolic operation in atlas hot path")

    for name in ("factor", "roots", "solve", "nroots", "N"):
        monkeypatch.setattr(sp, name, forbidden)

    authority = certify_adaptive_density_fan(
        metric,
        ideal,
        orientation,
        2,
        binding_reasons=(None,),
    )
    verify_adaptive_density_fan(metric, ideal, orientation, authority)

    assert authority.minimal_common_height == 4
    assert calls == []


def test_adaptive_structure_validator_accepts_global_atlas_height():
    metric, orientation, ideal = _projective_atlas_repro()
    authority = certify_adaptive_density_fan(
        metric,
        ideal,
        orientation,
        2,
        binding_reasons=(None,),
    )
    snapshot, request = _field_inputs(1)
    compiled = kernel.compile_reference_envelopes(snapshot, request)
    base = next(
        item
        for item in compiled.compilation.envelope_specs
        if type(item) is AdaptiveDensityAngularEnvelopeSpecV2
        and len(item.hidden_supports) == 1
    )
    support = next(iter(base.hidden_supports))
    rebound_support = replace(
        support,
        bound_primitive_integer_vector=(3, 4),
        direction_fan_authority_id=authority.authority_id,
    )
    rebound = replace(
        base,
        hidden_supports=frozenset({rebound_support}),
        direction_fan_authority=authority,
    )

    assert adaptive_density_structure_errors(rebound) == ()

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
