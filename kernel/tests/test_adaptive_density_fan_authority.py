from __future__ import annotations

from dataclasses import replace
from fractions import Fraction
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
from cftuv_envelope.reference import adaptive_density_fan as _density_fan
from cftuv_envelope.reference.adaptive_density_atlas import (
    DENSITY_FAN_PREPARATION_WORK_CAP,
    search_global_height,
    search_segments,
    segment_candidates,
)
from cftuv_envelope.reference.adaptive_density_fan import (
    AdaptiveDensityFanInvalid,
    DensityRationalAuthorityExhausted,
    DensityWindowChartUnrepresentable,
    _DENSITY_EXACT_WORK_CAP,
    _best_fan,
    _covectors,
    _subturn,
    _work_budget,
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

_PATCH10 = (
    Path(__file__).parents[1]
    / "fixtures"
    / "building_patch10_density4_v1"
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


def test_density_zero_fallback_routes_valid_projective_atlas():
    from cftuv_envelope.reference.direction_binding import (
        certify_huber_density_bindings_with_adaptive_fallback,
    )

    metric, orientation, ideal = _projective_atlas_repro()

    certificates, authority = (
        certify_huber_density_bindings_with_adaptive_fallback(
            metric,
            ideal,
            orientation,
            2,
            (DirectionBindingReasonV1.SOURCE_DIRECTION_IRRATIONAL,),
        )
    )
    payload = kernel.canonical_json_bytes(authority)

    assert certificates == (None,)
    assert type(authority) is AdaptiveMinimalRationalFanAuthorityV2
    assert authority.minimal_common_height == 4
    assert authority.bound_primitive_integer_vectors == ((3, 4),)
    assert (
        type(authority.ordinal_windows[0])
        is AdaptiveRationalFanOrdinalWindowAtlasV1
    )
    assert len(payload) == 3359
    assert sha256(payload).hexdigest() == (
        "6bbf2d44590a23d210ac3b3b8d98350aa70b29e926a59574b37a7792980e23a7"
    )
    verify_sealed_adaptive_density_fan(authority, ideal_count=3)
    verify_adaptive_density_fan(metric, ideal, orientation, authority)


@pytest.mark.parametrize("q", (2, 3, 6))
def test_legacy_density_refuses_projective_atlas_without_tuple_unpack(q):
    from cftuv_envelope.reference.adaptive_density_atlas import (
        AtlasWindowPrepared,
    )
    from cftuv_envelope.reference.adaptive_density_fan import (
        _covectors,
        _legacy_density_certificate,
        _window_envelopes,
    )

    metric, _, normals = _projective_atlas_repro()
    ideal = _covectors(metric, normals)
    _, records = _window_envelopes(ideal)

    assert type(records[0]) is AtlasWindowPrepared
    assert _legacy_density_certificate(
        metric,
        ideal,
        1,
        q,
        records[0],
    ) is None


def test_legacy_full_fan_rejects_non_none_atlas_certificate():
    from cftuv_envelope import DirectionBindingCertificateV1
    from cftuv_envelope.reference.adaptive_density_fan import (
        _covectors,
        _legacy_full_fan_valid,
        _window_envelopes,
    )
    from cftuv_envelope.reference.direction_binding import _PROVEN_PREDICATES

    metric, orientation, normals = _projective_atlas_repro()
    ideal = _covectors(metric, normals)
    _, records = _window_envelopes(ideal)
    local = records[0].local_record
    forged = DirectionBindingCertificateV1(
        bound_primitive_integer_vector=(3, 4),
        ideal_window_lower_slope_envelope=local[2],
        ideal_window_upper_slope_envelope=local[3],
        certified_window_width_lower_bound=(
            local[3].lower - local[2].upper
        ),
        proven_predicates=_PROVEN_PREDICATES,
    )

    assert not _legacy_full_fan_valid(
        metric,
        ideal,
        orientation,
        2,
        records,
        (forged,),
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


# --------------------------------------------------------------------------
# Ресурсный кап поиска минимального D*.
#
# Высота одна не была ресурсом: `_RESOURCE_HEIGHT_CAP` ограничивал номер
# последней высоты, а не работу, и при узком окне фронт уходил в счёт без
# исхода. Ниже заморожен ровно противоположный факт: работа считается, кап
# объявлен литералом, а его превышение — именованный исход.
# --------------------------------------------------------------------------


def _narrow_window_repro(denominator: int):
    """Синтетическое ordinal-окно ширины порядка 1/denominator.

    Доказанная высота останова растёт линейно по `denominator`; ровно этот
    класс входа в поле считался без исхода.
    """

    metric = _euclidean_metric()
    orientation = TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
    ideal = _interpolated_normals(
        metric,
        ExactPlanarVector.from_values(1, 0),
        ExactPlanarVector.from_values(denominator, 1),
        1,
        orientation,
        huber_density=True,
    )
    return metric, orientation, ideal


def _budget_spy(monkeypatch):
    budgets = []
    original = _density_fan._work_budget

    def spy(cap=None):
        budget = original(cap)
        budgets.append(budget)
        return budget

    monkeypatch.setattr(_density_fan, "_work_budget", spy)
    return budgets


def test_exact_work_cap_turns_endless_density_search_into_named_refusal():
    """Вход, не укладывающийся в объявленный бюджет, отказывает именем.

    Фронт обязан продолжиться, дойти до именованного события или именованно
    отказать. «Считает вечно» не является ни одним из трёх.
    """

    metric, orientation, ideal = _narrow_window_repro(100_000)

    with pytest.raises(
        DensityRationalAuthorityExhausted,
        match="DENSITY_RATIONAL_AUTHORITY_EXHAUSTED",
    ) as raised:
        certify_adaptive_density_fan(
            metric,
            ideal,
            orientation,
            2,
            binding_reasons=(None,),
        )

    message = str(raised.value)
    assert f"cap={_DENSITY_EXACT_WORK_CAP}" in message
    assert "shell_probes=" in message
    assert "order_steps=" in message


def test_exact_work_cap_counters_repeat_bit_for_bit():
    """Счётчики — точная работа, а не время: два прогона совпадают."""

    messages = []
    for _ in range(2):
        metric, orientation, ideal = _narrow_window_repro(100_000)
        with pytest.raises(DensityRationalAuthorityExhausted) as raised:
            certify_adaptive_density_fan(
                metric,
                ideal,
                orientation,
                2,
                binding_reasons=(None,),
            )
        messages.append(str(raised.value))

    assert messages[0] == messages[1]
    assert messages[0].endswith(
        f"cap={_DENSITY_EXACT_WORK_CAP} "
        f"shell_probes={_DENSITY_EXACT_WORK_CAP + 1} order_steps=0"
    )


def _atlas_search_inputs(q: int = 2):
    """Тот же вход, что у atlas-ветви сертификации: план по кускам окна."""

    metric, orientation, normals = _projective_atlas_repro()
    ideal = _covectors(metric, normals)
    _, records = _density_fan._window_envelopes(ideal)
    local_records = tuple(
        _density_fan._local_record(item) for item in records
    )
    sealed = tuple(
        _density_fan._sealed_local_interval(
            metric,
            ideal,
            ordinal,
            orientation,
            q,
            item,
        )
        for ordinal, item in enumerate(local_records, start=1)
    )
    segments = search_segments(
        metric,
        ideal,
        orientation,
        q,
        records,
        local_records,
        sealed,
        _density_fan._segment_plan_laws(),
    )
    return metric, ideal, orientation, segments


def test_atlas_search_spends_the_same_exact_work_on_every_run():
    """Atlas-ветвь тратит бюджет из того же счётчика и так же повторяемо.

    Числа счётчиков — цена ЗАКОНА ПЕРЕЧИСЛЕНИЯ, а не глубины задачи, и
    здесь они упали с 48 до 4 при том же исходе (высота 4, вектор (3, 4)).
    Прежний закон мёл всю оболочку max-нормы: до 8h примитивных ковекторов
    на высоту, то есть 4*H*(H+1)*k на проход. Новый перечисляет только
    числители внутри sealed-оболочки каждого cell-куска, как одно-картная
    ветвь. Исход обязан совпадать побитово — это и проверяется рядом с
    ценой, иначе падение цены нечем отличить от потери кандидатов.
    """

    runs = []
    for _ in range(2):
        metric, ideal, orientation, segments = _atlas_search_inputs()
        budget = _work_budget()
        result = search_global_height(
            metric,
            ideal,
            orientation,
            2,
            8,
            segments=segments,
            local_candidate=_density_fan._local_candidate,
            vector_from_slope=_density_fan._vector_from_slope,
            best_fan=_best_fan,
            budget=budget,
        )
        runs.append((result, budget.counters()))

    assert runs[0] == runs[1]
    assert runs[0][0][0] == 4
    assert runs[0][0][1] == ((3, 4),)
    assert runs[0][0][2] == (0,)
    assert runs[0][1] == (
        ("DENSITY_FAN_SHELL_PROBES", 4),
        ("DENSITY_FAN_ORDER_STEPS", 2),
    )


def _max_norm_shell(height: int):
    """Прежний закон перечисления — все примитивные ковекторы высоты h.

    Здесь он оставлен ОРАКУЛОМ: по-кусковое перечисление обязано давать ровно
    то же множество кандидатов, иначе падение цены неотличимо от потери
    кандидата. Считать им прогон нельзя — это 8h векторов на высоту.
    """

    from math import gcd

    for first in range(-height, height + 1):
        for second in (-height, height):
            if gcd(abs(first), abs(second)) == 1:
                yield first, second
    for second in range(-height + 1, height):
        for first in (-height, height):
            if gcd(abs(first), abs(second)) == 1:
                yield first, second


def _two_piece_atlas_repro(owner_orientation_sign: int = 1):
    """Atlas-окно, чей допустимый конус ПЕРЕСЕКАЕТ границу ячеек.

    В `_projective_atlas_repro` конус целиком лежит в termination-куске, и
    ветвь «соседний кусок тоже метётся» там недостижима. Здесь достижима:
    вектор перехода между кусками допустим, поэтому обход захватывает оба
    куска, а оболочка не-terminating куска строится теми же двумя лучами
    subturn, а не sealed-интервалом termination-куска.
    """

    normals = (
        ExactPlanarVector.from_values(
            sp.Rational(-58, 75), sp.Rational(17, 25)
        ),
        ExactPlanarVector.from_values(
            sp.Rational(-2074, 2235), sp.Rational(929, 745)
        ),
        ExactPlanarVector.from_values(
            sp.Rational(-69, 85), sp.Rational(97, 85)
        ),
    )
    return (
        ExactPlanarMetric(
            (
                (sp.Rational(18), sp.Rational(15)),
                (sp.Rational(15), sp.Rational(13)),
            ),
            (
                (sp.Rational(13, 9), sp.Rational(-5, 3)),
                (sp.Rational(-5, 3), sp.Rational(2)),
            ),
            owner_orientation_sign,
        ),
        TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION,
        normals,
    )


@pytest.mark.parametrize("owner_orientation_sign", (1, -1))
def test_piecewise_sweep_finds_exactly_the_max_norm_shell(
    owner_orientation_sign,
):
    """По-кусковый закон даёт ТО ЖЕ множество, что и оболочка max-нормы.

    Это оракул эквивалентности, а не тест производительности: на каждой
    высоте сверяется множество кандидатов, а не его размер. Окно выбрано так,
    что метутся ОБА куска — иначе оболочка не-terminating куска осталась бы
    непокрытой.
    """

    from cftuv_envelope.reference.adaptive_density_atlas import (
        AtlasWindowPrepared,
        reachable_pieces,
    )

    metric, orientation, normals = _two_piece_atlas_repro(
        owner_orientation_sign
    )
    ideal = _covectors(metric, normals)
    _, records = _density_fan._window_envelopes(ideal)
    local_records = tuple(
        _density_fan._local_record(item) for item in records
    )
    sealed = tuple(
        _density_fan._sealed_local_interval(
            metric,
            ideal,
            ordinal,
            orientation,
            2,
            item,
        )
        for ordinal, item in enumerate(local_records, start=1)
    )
    segments = search_segments(
        metric,
        ideal,
        orientation,
        2,
        records,
        local_records,
        sealed,
        _density_fan._segment_plan_laws(),
    )
    reached = reachable_pieces(
        records[0],
        admissible=lambda item: _density_fan._local_candidate(
            metric, ideal, 1, item, 2
        ),
        invalid=DensityWindowChartUnrepresentable,
    )

    assert type(records[0]) is AtlasWindowPrepared
    assert len(records[0].pieces) == 2
    assert records[0].termination_piece_index == 1
    assert reached == (0, 1)
    assert len(segments[0]) == 2

    budget = _work_budget(10**9)
    for height in range(1, 25):
        expected = {
            item
            for item in _max_norm_shell(height)
            if _density_fan._local_candidate(metric, ideal, 1, item, 2)
        }
        actual = set()
        for segment in segments[0]:
            actual.update(
                segment_candidates(
                    metric,
                    ideal,
                    1,
                    2,
                    segment,
                    height,
                    budget,
                    local_candidate=_density_fan._local_candidate,
                    vector_from_slope=_density_fan._vector_from_slope,
                )
            )
        assert actual == expected, height


def test_atlas_sweep_names_exhaustion_instead_of_walking_to_the_height_cap():
    """Atlas-ветвь тоже отказывает по работе, а не идёт до 10^6 высот.

    Доказанная высота останова этого окна равна 4.4e7 — ровно поэтому
    `_RESOURCE_HEIGHT_CAP` не был бюджетом. Здесь веер не находится никогда,
    и единственное, что обязано остановить проход, — счётчик работы.

    Перерасход над капом больше единицы: высота теперь стоит w*h + 1
    числителей одного куска, и последняя высота оплачивается целиком.
    """

    metric, ideal, orientation, segments = _atlas_search_inputs()
    budget = _work_budget(1 << 12)

    with pytest.raises(
        DensityRationalAuthorityExhausted,
        match="DENSITY_RATIONAL_AUTHORITY_EXHAUSTED",
    ):
        search_global_height(
            metric,
            ideal,
            orientation,
            2,
            10**6,
            segments=segments,
            local_candidate=_density_fan._local_candidate,
            vector_from_slope=_density_fan._vector_from_slope,
            best_fan=lambda *_: None,
            budget=budget,
        )

    assert budget.counters() == (
        ("DENSITY_FAN_SHELL_PROBES", 4_130),
        ("DENSITY_FAN_ORDER_STEPS", 0),
    )


def test_exhausted_density_domain_names_the_outcome_and_leaves_no_trace(
    monkeypatch,
):
    """Домен отказывает именованно, компиляция не появляется, прогон живёт."""

    monkeypatch.setattr(_density_fan, "_DENSITY_EXACT_WORK_CAP", 1)
    snapshot, request = _field_inputs(4)

    result = kernel.compile_reference_envelopes(snapshot, request)

    assert (
        result.outcome
        is ReferenceOutcome.DENSITY_RATIONAL_AUTHORITY_EXHAUSTED
    )
    assert result.compilation is None
    assert [item.outcome for item in result.diagnostics] == [
        ReferenceOutcome.DENSITY_RATIONAL_AUTHORITY_EXHAUSTED
    ]
    assert "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED" in (
        result.diagnostics[0].message
    )


def test_exhausted_density_domain_still_gets_a_prepare_receipt(monkeypatch):
    """Полевой путь QUEUE: домен отказывает именем, receipt существует.

    Именно этого не было в поле: домен считал 240+ секунд, и стадия не
    оставляла ни исхода, ни таймингов.
    """

    monkeypatch.setattr(_density_fan, "_DENSITY_EXACT_WORK_CAP", 1)
    snapshot, request = _field_inputs(4)

    prepared = prepare_conveyor(snapshot, request)

    assert prepared.outcome.value == "PLAN_IS_NOT_COMPILED"
    assert prepared.detail == "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED"
    assert prepared.regions == ()
    assert dict(prepared.timings)["PLAN_COMPILE"] >= 0


@pytest.mark.parametrize(
    "density, certificates, emitted, lift",
    (
        (2, 5, 6, (1, 2, 4)),
        (4, 9, 10, (2, 3, 6)),
    ),
)
def test_the_conveyor_emits_the_spec_count_and_the_lift_owns_the_difference(
    density, certificates, emitted, lift
):
    """ПО-ВЕЕРНАЯ сверка эмиссии: конвейер не изобретает опор и не теряет их.

    Полевая диагностика прочла сумму сертификатов селекции против счётчика
    `CONVEYOR_FAN_SUPPORTS`, увидела разницу в единицу и назвала её дефектом
    эмиссии. Здесь та же разница воспроизведена на фикстуре и разложена
    по-веерно: она принадлежит ОДНОМУ углу и несёт СВОЮ власть —
    `EvaluationGeometrySubturnCountLiftV1`.

    Закон, который здесь и заморожен, двухступенчатый:

    1. конвейер эмитит РОВНО `spec.resolved_hidden_edge_count` на веер;
    2. `spec.resolved_hidden_edge_count` равен сертификату селекции ВСЮДУ,
       кроме углов с доказанным лифтом, где он равен
       `lift.effective_hidden_edge_count`.

    Лифт — не поблажка: `_evaluation_density_spec` поднимает счёт только после
    того, как ИДЕАЛ при счёте сертификата оказался неосуществим в ГЕОМЕТРИИ
    ВЫЧИСЛЕНИЯ (метрика карты — не метрика источника), и поднимает до
    минимального осуществимого, записывая предшественника. Требование
    «эмиссия равна сертификату селекции» удалило бы этот механизм и вернуло бы
    подшаг, превышающий объявленный delta_max, на углах у границы ячейки.

    Обе строки — угол на границе `q`: при density 2 (`q = 4`) лифт 1 -> 2, при
    density 4 (`q = 6`) лифт 2 -> 3. Виновный угол один и тот же
    (`angular-spec:7dde3e10be8587bd61e928d6`).
    """

    snapshot, request = _field_inputs(density)
    result = kernel.compile_reference_envelopes(snapshot, request)
    assert result.outcome is ReferenceOutcome.EXACT, result.diagnostics
    selection_by_id = {
        item.certificate_id: item
        for item in result.compilation.profile_selection_certificates
    }
    specs = tuple(
        item
        for item in result.compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
    )

    certificate_total = 0
    spec_total = 0
    lifted = []
    for spec in specs:
        selection = selection_by_id[spec.selection_certificate_id]
        certificate_total += selection.resolved_hidden_edge_count
        spec_total += spec.resolved_hidden_edge_count
        # Спека несёт ровно столько записей опор, сколько объявила: срез
        # `_lift_probe_spec` обязан быть исполненным, а не заявленным.
        assert len(spec.hidden_supports) == spec.resolved_hidden_edge_count
        record = spec.evaluation_subturn_count_lift
        if record is None:
            assert (
                spec.resolved_hidden_edge_count
                == selection.resolved_hidden_edge_count
            ), spec.envelope_spec_id
            continue
        lifted.append((spec, record, selection))

    assert certificate_total == certificates
    assert spec_total == emitted
    # Разницу несёт РОВНО один угол, и она равна ровно его лифту.
    assert len(lifted) == 1
    spec, record, selection = lifted[0]
    assert spec.envelope_spec_id.value == (
        "angular-spec:7dde3e10be8587bd61e928d6"
    )
    assert (
        record.source_hidden_edge_count,
        record.effective_hidden_edge_count,
        record.max_subturn_q,
    ) == lift
    assert record.source_hidden_edge_count == (
        selection.resolved_hidden_edge_count
    )
    assert record.effective_hidden_edge_count == (
        spec.resolved_hidden_edge_count
    )
    assert spec_total - certificate_total == (
        record.effective_hidden_edge_count - record.source_hidden_edge_count
    )

    prepared = prepare_conveyor(snapshot, request)
    counters = dict(prepared.counters)

    assert prepared.outcome.value == "EXACT"
    assert counters["CONVEYOR_RATIONAL_VERTEX_FANS"] == len(specs)
    # Счётчик эмиссии равен сумме СПЕК, а не сумме сертификатов, и именно это
    # равенство отличает «конвейер посчитал по власти» от «конвейер приписал».
    assert counters["CONVEYOR_FAN_SUPPORTS"] == spec_total
    assert counters["CONVEYOR_BOUND_FAN_DIRECTIONS"] == spec_total


def test_deepest_green_field_authority_stays_far_below_the_work_cap(
    monkeypatch,
):
    """Замороженный якорь маржи: поле d=4 не приближается к капу.

    Самая глубокая зелёная власть поля (`60fc81c8…`, минимальная общая
    высота 4492) — это и есть худший случай замороженного корпуса, из
    которого выведен литерал капа. Число ниже двигать нельзя молча: его
    рост означает, что маржа съедена.
    """

    budgets = _budget_spy(monkeypatch)
    snapshot, request = _field_inputs(4)

    result = kernel.compile_reference_envelopes(snapshot, request)

    assert result.outcome is ReferenceOutcome.EXACT, result.diagnostics
    assert max(item.spent for item in budgets) == 9_035
    assert 9_035 * 8 < _DENSITY_EXACT_WORK_CAP


def _preparation_budget_spy(monkeypatch):
    budgets = []
    original = _density_fan._preparation_budget

    def spy(cap=None):
        budget = original(cap)
        budgets.append(budget)
        return budget

    monkeypatch.setattr(_density_fan, "_preparation_budget", spy)
    return budgets


def test_fan_preparation_work_is_capped_by_a_named_refusal():
    """Подготовка власти обязана отказать именем, а не считать без исхода.

    Кап поиска D* заводится уже ВНУТРИ поиска, поэтому всё, что посчитано
    раньше, до этой карточки не было оплачено ничем: legacy-развёртка
    ordinal-окна перебирает ceil(1/sqrt(w)) знаменателей и при сужении окна
    растёт без границы. Здесь окно шириной 5e-12 требует 447 214
    знаменателей — отказ наступает ДО первого шага развёртки.
    """

    metric, orientation, ideal = _narrow_window_repro(100_000_000_000)

    with pytest.raises(
        DensityRationalAuthorityExhausted,
        match="DENSITY_RATIONAL_AUTHORITY_EXHAUSTED",
    ) as raised:
        certify_density_bindings_and_adaptive_fallback(
            metric,
            ideal,
            orientation,
            2,
            binding_reasons=(None,),
        )

    message = str(raised.value)
    assert f"cap={DENSITY_FAN_PREPARATION_WORK_CAP}" in message
    assert "shell_probes=447214" in message
    assert "order_steps=0" in message


def test_fan_preparation_cap_leaves_the_domain_below_it_untouched(monkeypatch):
    """Домен ПОД капом считается ровно как прежде — тем же ответом.

    Окно шириной 5e-10 требует 44 722 знаменателей: это две трети капа, и
    домен обязан досчитаться. Сравнение идёт с прогоном, у которого капа
    подготовки фактически нет: если ответы совпали, кап не участвует в
    вычислении, а только его ограничивает.
    """

    def run():
        metric, orientation, ideal = _narrow_window_repro(1_000_000_000)
        return certify_density_bindings_and_adaptive_fallback(
            metric,
            ideal,
            orientation,
            2,
            binding_reasons=(None,),
        )

    original = _density_fan._preparation_budget
    capped = run()
    monkeypatch.setattr(
        _density_fan,
        "_preparation_budget",
        lambda cap=None: original(1 << 40),
    )
    uncapped = run()

    assert capped == uncapped
    assert capped[1] is None
    assert len(capped[0]) == 1


def test_fan_preparation_work_stays_far_below_its_cap(monkeypatch):
    """Замороженный якорь маржи подготовки: поле d=4 не приближается к капу.

    Тот же домен, что и у якоря поиска D*. Число ниже двигать нельзя молча:
    его рост означает, что маржа съедена. Контекст замера, из которого выведен
    литерал капа: 351 подготовка полного зелёного корпуса тратит не больше 32
    единиц, полевые слепки на плотностях 0…4 — не больше 36.
    """

    budgets = _preparation_budget_spy(monkeypatch)
    snapshot, request = _field_inputs(4)

    result = kernel.compile_reference_envelopes(snapshot, request)

    assert result.outcome is ReferenceOutcome.EXACT, result.diagnostics
    assert max(item.spent for item in budgets) == 8
    assert 36 * 1000 < DENSITY_FAN_PREPARATION_WORK_CAP


def _patch10_inputs():
    snapshot = kernel.AnalysisSnapshotCodecV1.loads_with_compatibility_receipt(
        (_PATCH10 / "analysis_snapshot.json").read_bytes()
    )
    assert snapshot.compatibility_receipt.positive_scaled_normal_compat_hits == 1
    return (
        snapshot.record,
        kernel.DecalRequestCodecV1.loads(
            (_PATCH10 / "decal_request.json").read_bytes()
        ),
    )


def test_patch10_density_four_atlas_domain_compiles_inside_the_work_cap(
    monkeypatch,
):
    """Полевой домен с atlas-окном: власть строится, а не исчерпывается.

    До по-кускового перечисления этот домен отказывал именем
    `DENSITY_RATIONAL_AUTHORITY_EXHAUSTED`, потратив весь кап 131 072 единиц
    и 723 МиБ пиковой памяти. Причина была в ЗАКОНЕ ПЕРЕЧИСЛЕНИЯ, а не в
    глубине задачи: atlas-ветвь мела всю оболочку max-нормы (до 8h примитивных
    ковекторов на высоту, 4*H*(H+1)*k на проход), тогда как одно-картная ветвь
    перечисляла только числители внутри sealed-интервала.

    Здесь заморожено, что задача была неглубокой: D* = 1711 достигается за
    3 432 единицы — в 38 раз меньше капа. Рядом заморожена доказанная высота
    останова 191 692 805: `_RESOURCE_HEIGHT_CAP` = 10^6 не был бюджетом и на
    этом домене, поэтому исчерпание нельзя было объяснить «слишком глубоко».

    Проверяется ИМЕННО atlas-ветвь: ровно одна власть домена несёт
    `AdaptiveRationalFanOrdinalWindowAtlasV1`, и это она самая глубокая.
    Без этой строки тест прошёл бы и на одно-картных окнах.
    """

    budgets = _budget_spy(monkeypatch)
    snapshot, request = _patch10_inputs()

    result = kernel.compile_reference_envelopes(snapshot, request)

    assert result.outcome is ReferenceOutcome.EXACT, result.diagnostics
    authorities = tuple(
        spec.direction_fan_authority
        for spec in result.compilation.envelope_specs
        if type(spec) is AdaptiveDensityAngularEnvelopeSpecV2
    )
    atlas = tuple(
        item
        for item in authorities
        if any(
            type(window) is AdaptiveRationalFanOrdinalWindowAtlasV1
            for window in item.ordinal_windows
        )
    )
    assert len(authorities) == 4
    assert len({item.authority_id for item in atlas}) == 1
    deepest = max(authorities, key=lambda item: item.minimal_common_height)
    assert deepest.authority_id == atlas[0].authority_id
    assert deepest.max_subturn_q == 6
    assert deepest.minimal_common_height == 1_711
    assert deepest.exhaustive_previous_height == 1_710
    assert deepest.bound_primitive_integer_vectors == (
        (1474, 1711),
        (738, 1399),
    )
    assert deepest.previous_height_witness.primitive_candidate_counts == (
        0,
        7,
    )
    assert deepest.termination_height_upper_bound == 191_692_805
    assert tuple(
        type(window) is AdaptiveRationalFanOrdinalWindowAtlasV1
        for window in deepest.ordinal_windows
    ) == (False, True)
    verify_sealed_adaptive_density_fan(deepest, ideal_count=4)
    assert max(item.spent for item in budgets) == 3_432
    assert 3_432 * 32 < _DENSITY_EXACT_WORK_CAP


def test_patch10_density_four_domain_passes_the_bridge_on_its_mirrored_chart():
    """Домен ЗЕРКАЛЬНОЙ карты проходит мост целиком, и это заморожено числами.

    Прежде здесь стоял замороженный отказ
    `LATTICE_SNAP_BREAKS_A_VERTEX_FAN` (тест назывался
    `..._reaches_its_own_named_lattice_refusal`), и имя это указывало не на ту
    причину: снап невиновен. Карта домена — `COORDINATE_CW_MATCHES_OWNER_PATCH`,
    то есть обход патча-владельца и обход координат карты противоположны, и
    веер, построенный рецептом направлений в первом смысле, приходил в полигон
    развёрнутым. Измерено: векторные произведения соседних опор всех четырёх
    вееров были ПОЛОЖИТЕЛЬНЫ (+45, +71, +5, +799408) при требовании строго
    отрицательного вращения; сырая внешняя петля решётки при этом вышла CCW
    (`2S = +255156622028`), то есть разворачивать её `PolygonV1.build` и не
    пробовал.

    Гипотеза ROADMAP «веер знаменателя 1711 не переживает привязку якоря»
    ОПРОВЕРГНУТА этими числами: тот же веер `(1474, 1711) / (738, 1399)` проходит
    без единой правки решётки, шаг которой остался прежним (65536), а
    `snap_residual` — прежним `4464165/640696385536`.

    Замораживается ИСХОД, каким он вышел, а не какой хотелось: домен доходит до
    конца конвейера (`EXACT`), скелет из 20 узлов, 22 грани, и сумма граней
    воспроизводит площадь полигона. Четыре рациональных веера сохранены — это и
    есть контроль, что домен не стал зелёным ценой выброшенного угла.
    """

    snapshot, request = _patch10_inputs()

    prepared = prepare_conveyor(snapshot, request)

    assert prepared.outcome.value == "EXACT"
    assert prepared.detail == ""
    counters = dict(prepared.counters)
    assert counters["CONVEYOR_RATIONAL_VERTEX_FANS"] == 4
    assert counters["CONVEYOR_DEGRADED_MITER_CORNERS"] == 0
    assert counters["CONVEYOR_MITERED_CORNERS"] == 0
    assert counters["CONVEYOR_FAN_EDGES"] == 10
    assert counters["CONVEYOR_FAN_SUPPORTS"] == 10
    assert counters["CONVEYOR_BOUND_FAN_DIRECTIONS"] == 10
    assert counters["CONVEYOR_SKELETON_NODES"] == 20
    assert counters["CONVEYOR_FACES"] == 22
    assert counters["CONVEYOR_LATTICE_SCALE"] == 65_536
    assert counters["CONVEYOR_SOURCE_EDGES"] == 12
    assert counters["CONVEYOR_WALL_EDGES"] == 0
    (region,) = prepared.regions
    assert region.bridge_outcome.value == "EXACT"
    assert region.skeleton_outcome.value == "EXACT"
    assert region.face_outcome.value == "EXACT"
    assert region.bridge.snap_residual == Fraction(4_464_165, 640_696_385_536)
    assert region.partition.area_reproduces_polygon
    assert dict(prepared.timings)["PLAN_COMPILE"] >= 0
