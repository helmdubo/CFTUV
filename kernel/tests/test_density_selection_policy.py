from __future__ import annotations

from dataclasses import dataclass, replace
from decimal import Decimal
from fractions import Fraction
from hashlib import sha256
import json

import pytest
import sympy as sp

from cftuv_envelope import (
    AdmissibilityUpperBound,
    AngularEnvelopeSpec,
    AngularProfileSelectionPolicyId,
    CertifiedDecimalIntervalV1,
    CompiledPlanCodecV1,
    ContractCodecError,
    DecalRequestCodecV1,
    ExactAngleSymbol,
    ExactAngleV1,
    HuberDensitySelectionIntervalCertificateV1,
    IntervalBoundKind,
    IntervalEndpointKind,
    MaxSubturnParameterId,
    MaxSubturnValueId,
    MinimalityLowerBound,
    SelectionLaw,
    TurnOrientation,
    ValidationCode,
    canonical_json_bytes,
    validate_compiled_plan,
    validate_decal_request,
)
from cftuv_envelope.reference import (
    ReferenceOutcome,
    compile_reference_envelopes,
)
from cftuv_envelope.reference.angular import _interpolated_normals
from cftuv_envelope.reference.compile import (
    _resolve_huber_density_bucket,
)
from cftuv_envelope.reference.direction_binding import (
    BINDING_INSIDE_OWN_ORDINAL_WINDOW,
    DirectionBindingCertificateUnproven,
    certify_huber_density_direction_bindings,
    verify_huber_density_direction_bindings,
)
from cftuv_envelope.reference.metric import ExactPlanarMetric
from cftuv_envelope.reference.planar_types import ExactPlanarVector

from reference_factories import angular_snapshot


_DENSITY_VALUES = (
    (
        MaxSubturnValueId.LINEAR_REFLEX_DENSITY_0_V1,
        ExactAngleSymbol.PI_OVER_2,
        2,
    ),
    (
        MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1,
        ExactAngleSymbol.PI_OVER_3,
        3,
    ),
    (
        MaxSubturnValueId.LINEAR_REFLEX_DENSITY_2_V1,
        ExactAngleSymbol.PI_OVER_4,
        4,
    ),
    (
        MaxSubturnValueId.LINEAR_REFLEX_DENSITY_3_V1,
        ExactAngleSymbol.PI_OVER_5,
        5,
    ),
    (
        MaxSubturnValueId.LINEAR_REFLEX_DENSITY_4_V1,
        ExactAngleSymbol.PI_OVER_6,
        6,
    ),
)

_LEGACY_PLAN_SHA256 = {
    1: {
        "selection": (
            1308,
            "fe2e2891a9ee84db8ae864f4981aca067051375766b96aa63396ab9124ee0357",
        ),
        "plan": (
            8671,
            "38398a330471688510ff1d339161b09680a1e8d7f15146f33059a4bd5eee6cdd",
        ),
        "direction": (
            654,
            "b62bdd12cac29f25525399bdc651f1774264caee3fd825ac24fb9c093fecbca4",
        ),
    },
    2: {
        "selection": (
            1277,
            "c33a8c78e9ed60f1d45a508b9513b624212146280922f7270a6fcb05287490bb",
        ),
        "plan": (
            9455,
            "f2a73f2bad42021fc3ed560ae86571a07f7171728b39393dad5b4f42f84aa64c",
        ),
        "direction": (
            1307,
            "e663042b43cecd5479d04edb421a49a839dd34eaf8cb815d38a81ccf27f674d9",
        ),
    },
}


def _projection(projections, case_id):
    return next(item for item in projections if item.case_id == case_id)


def _density_request(request, value_id, symbol):
    return replace(
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


def _density_certificate(certificate, *, value_id, q, bucket_c):
    return replace(
        certificate,
        selection_policy_id=(
            AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1
        ),
        max_subturn_value_id=value_id,
        resolved_hidden_edge_count=max(1, bucket_c - 1),
        resolved_subturn_count=max(2, bucket_c),
        local_profile_support_count=max(1, bucket_c - 1) + 2,
        local_profile_segment_count=max(1, bucket_c - 1) + 2,
        selection_law=SelectionLaw.HUBER_EMANATED_DENSITY_FLOOR_V1,
        minimality_lower_bound=(
            MinimalityLowerBound.HUBER_DENSITY_BUCKET_OPEN_LOWER
        ),
        admissibility_upper_bound=(
            AdmissibilityUpperBound.HUBER_DENSITY_BUCKET_CLOSED_UPPER
        ),
        selection_interval_certificate=(
            HuberDensitySelectionIntervalCertificateV1(
                q=q,
                bucket_c=bucket_c,
                lower_bound_kind=IntervalBoundKind.OPEN,
                lower_bound_numerator=bucket_c - 1,
                upper_bound_kind=IntervalBoundKind.CLOSED,
                upper_bound_numerator=bucket_c,
            )
        ),
        regression_fixture_id=None,
    )


@dataclass(frozen=True)
class _ExactInterval:
    lower: Fraction
    upper: Fraction
    lower_kind: IntervalEndpointKind
    upper_kind: IntervalEndpointKind


@dataclass(frozen=True)
class _ExactMeasure:
    reflex_excess_over_pi: _ExactInterval
    orientation: TurnOrientation


def _point_measure(
    value: Fraction,
    orientation: TurnOrientation,
) -> _ExactMeasure:
    return _ExactMeasure(
        _ExactInterval(
            value,
            value,
            IntervalEndpointKind.CLOSED,
            IntervalEndpointKind.CLOSED,
        ),
        orientation,
    )


def _narrow_high_angle_snapshot():
    snapshot, request = angular_snapshot(2)
    angle = next(iter(snapshot.reflex_angle_certificates))

    def interval(lower: str, upper: str):
        return CertifiedDecimalIntervalV1(
            Decimal(lower),
            Decimal(upper),
            IntervalEndpointKind.CLOSED,
            IntervalEndpointKind.CLOSED,
            Decimal(upper) - Decimal(lower),
        )

    measure = replace(
        angle.measure_payload,
        phi_over_pi=interval("1.88", "1.89"),
        reflex_excess_over_pi=interval("0.88", "0.89"),
    )
    changed_angle = replace(angle, measure_payload=measure)
    return (
        replace(
            snapshot,
            reflex_angle_certificates=frozenset({changed_angle}),
        ),
        request,
    )


def test_legacy_request_canonical_bytes_are_frozen(projections):
    request = _projection(projections, "EC0-C03").request
    payload = DecalRequestCodecV1.dumps(request)
    assert sha256(payload).hexdigest() == (
        "bfc4f3b15b51fe7f367127c803f73f4e383866ca580a91a85eca9b093a42e90f"
    )
    assert DecalRequestCodecV1.dumps(
        DecalRequestCodecV1.loads(payload)
    ) == payload


@pytest.mark.parametrize("hidden_count", (1, 2))
def test_legacy_selection_plan_and_direction_bytes_are_frozen(hidden_count):
    snapshot, request = angular_snapshot(hidden_count)
    compilation = compile_reference_envelopes(
        snapshot,
        request,
    ).compilation
    assert compilation is not None
    angular = next(
        item
        for item in compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
    )
    payloads = {
        "selection": canonical_json_bytes(
            compilation.profile_selection_certificates
        ),
        "plan": canonical_json_bytes(
            (
                compilation.profile_selection_certificates,
                compilation.envelope_specs,
                compilation.initial_front_spec,
            )
        ),
        "direction": canonical_json_bytes(angular.hidden_supports),
    }
    for name, payload in payloads.items():
        expected_length, expected_digest = _LEGACY_PLAN_SHA256[
            hidden_count
        ][name]
        assert len(payload) == expected_length
        assert sha256(payload).hexdigest() == expected_digest


@pytest.mark.parametrize("q", range(2, 7))
@pytest.mark.parametrize(
    "orientation",
    (
        TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION,
        TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION,
    ),
)
def test_density_boundary_matrix_is_lower_open_upper_closed(
    q,
    orientation,
):
    epsilon = Fraction(1, 100 * q)
    for numerator in range(1, q):
        boundary = Fraction(numerator, q)
        assert _resolve_huber_density_bucket(
            _point_measure(boundary - epsilon, orientation),
            q,
        ) == numerator
        assert _resolve_huber_density_bucket(
            _point_measure(boundary, orientation),
            q,
        ) == numerator
        assert _resolve_huber_density_bucket(
            _point_measure(boundary + epsilon, orientation),
            q,
        ) == numerator + 1
        straddle = _ExactMeasure(
            _ExactInterval(
                boundary - epsilon,
                boundary + epsilon,
                IntervalEndpointKind.CLOSED,
                IntervalEndpointKind.CLOSED,
            ),
            orientation,
        )
        assert _resolve_huber_density_bucket(straddle, q) is None


@pytest.mark.parametrize(
    ("value_id", "symbol", "q"),
    _DENSITY_VALUES,
)
def test_density_representative_high_angle_compiles_h_equals_q_minus_one(
    value_id,
    symbol,
    q,
):
    snapshot, legacy_request = _narrow_high_angle_snapshot()
    request = _density_request(legacy_request, value_id, symbol)

    compiled = compile_reference_envelopes(snapshot, request)

    assert compiled.outcome is ReferenceOutcome.EXACT
    angular = next(
        item
        for item in compiled.compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
    )
    assert angular.resolved_hidden_edge_count == q - 1
    assert len(angular.hidden_supports) == q - 1
    selection = next(
        iter(compiled.compilation.profile_selection_certificates)
    )
    assert selection.resolved_hidden_edge_count == q - 1
    assert selection.selection_interval_certificate.bucket_c == q


@pytest.mark.parametrize(("value_id", "symbol", "q"), _DENSITY_VALUES)
@pytest.mark.parametrize(
    ("orientation", "outgoing"),
    (
        (
            TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION,
            (0, 1),
        ),
        (
            TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION,
            (0, -1),
        ),
    ),
)
def test_density_h1_through_h5_direction_authority_is_orientation_symmetric(
    value_id,
    symbol,
    q,
    orientation,
    outgoing,
):
    del value_id, symbol
    metric = ExactPlanarMetric(
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
    ideal = _interpolated_normals(
        metric,
        ExactPlanarVector.from_values(1, 0),
        ExactPlanarVector.from_values(*outgoing),
        q - 1,
        orientation,
        huber_density=True,
    )

    certificates = certify_huber_density_direction_bindings(
        metric,
        ideal,
        orientation,
        q,
    )

    assert len(ideal) == q + 1
    assert len(certificates) == q - 1
    verify_huber_density_direction_bindings(
        metric,
        ideal,
        orientation,
        certificates,
        q,
    )


def test_density_empty_authority_rejects_forged_supplied_certificate():
    metric = ExactPlanarMetric(
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
    incoming = ExactPlanarVector.from_values(1, 0)
    outgoing = ExactPlanarVector.from_values(-117, -44)
    orientation = TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION
    h1_ideal = _interpolated_normals(
        metric,
        incoming,
        outgoing,
        1,
        orientation,
        huber_density=True,
    )
    (foreign_certificate,) = certify_huber_density_direction_bindings(
        metric,
        h1_ideal,
        orientation,
        2,
    )
    assert foreign_certificate is not None
    with pytest.raises(
        DirectionBindingCertificateUnproven,
        match=BINDING_INSIDE_OWN_ORDINAL_WINDOW,
    ):
        verify_huber_density_direction_bindings(
            metric,
            h1_ideal,
            orientation,
            (None,),
            2,
        )
    h2_ideal = _interpolated_normals(
        metric,
        incoming,
        outgoing,
        2,
        orientation,
        huber_density=True,
    )
    assert certify_huber_density_direction_bindings(
        metric,
        h2_ideal,
        orientation,
        3,
    ) == (None, None)

    with pytest.raises(
        DirectionBindingCertificateUnproven,
        match=BINDING_INSIDE_OWN_ORDINAL_WINDOW,
    ):
        verify_huber_density_direction_bindings(
            metric,
            h2_ideal,
            orientation,
            (foreign_certificate, None),
            3,
        )


def test_density_threshold_straddle_fails_closed_before_geometry():
    snapshot, legacy_request = angular_snapshot(2)
    angle = next(iter(snapshot.reflex_angle_certificates))

    def interval(lower: str, upper: str):
        return CertifiedDecimalIntervalV1(
            Decimal(lower),
            Decimal(upper),
            IntervalEndpointKind.CLOSED,
            IntervalEndpointKind.CLOSED,
            Decimal(upper) - Decimal(lower),
        )

    measure = replace(
        angle.measure_payload,
        phi_over_pi=interval("1.82", "1.90"),
        reflex_excess_over_pi=interval("0.82", "0.90"),
    )
    snapshot = replace(
        snapshot,
        reflex_angle_certificates=frozenset(
            {replace(angle, measure_payload=measure)}
        ),
    )
    request = _density_request(
        legacy_request,
        MaxSubturnValueId.LINEAR_REFLEX_DENSITY_4_V1,
        ExactAngleSymbol.PI_OVER_6,
    )

    compiled = compile_reference_envelopes(snapshot, request)

    assert (
        compiled.outcome
        is ReferenceOutcome.ANGULAR_PROFILE_SELECTION_UNCERTAIN
    )
    assert compiled.compilation is None


@pytest.mark.parametrize(("value_id", "symbol", "q"), _DENSITY_VALUES)
def test_density_request_tuple_is_additive_and_strict(
    projections, value_id, symbol, q
):
    legacy = _projection(projections, "EC0-C03").request
    request = _density_request(legacy, value_id, symbol)

    assert validate_decal_request(request) == ()
    assert request.max_subturn_value_id is value_id
    assert request.max_subturn_exact_value.symbol is symbol
    assert q in range(2, 7)
    payload = DecalRequestCodecV1.dumps(request)
    assert DecalRequestCodecV1.dumps(
        DecalRequestCodecV1.loads(payload)
    ) == payload


def test_density_request_rejects_cross_policy_value_and_exact_angle(
    projections,
):
    legacy = _projection(projections, "EC0-C03").request
    forged = replace(
        legacy,
        angular_profile_selection_policy_id=(
            AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1
        ),
        max_subturn_parameter_id=(
            MaxSubturnParameterId.LINEAR_REFLEX_DENSITY_A_V1
        ),
        max_subturn_value_id=(
            MaxSubturnValueId.LINEAR_REFLEX_DENSITY_4_V1
        ),
        max_subturn_exact_value=ExactAngleV1(ExactAngleSymbol.PI_OVER_2),
    )
    issues = validate_decal_request(forged)
    assert any(
        issue.code is ValidationCode.POLICY_MISMATCH
        and issue.path == ("max_subturn_exact_value",)
        for issue in issues
    )


def test_density_interval_tag_roundtrips_and_validator_seals_law(projections):
    plan = _projection(projections, "EC0-C03").plans[0]
    legacy = next(iter(plan.angular_profile_selection_certificates))
    density = _density_certificate(
        legacy,
        value_id=MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1,
        q=3,
        bucket_c=2,
    )
    changed = replace(
        plan,
        angular_profile_selection_certificates=frozenset({density}),
    )

    assert not any(
        issue.code is ValidationCode.ANGULAR_CERTIFICATE
        for issue in validate_compiled_plan(changed)
    )
    payload = CompiledPlanCodecV1.dumps(changed)
    data = json.loads(payload)
    encoded = data["angular_profile_selection_certificates"][0][
        "selection_interval_certificate"
    ]
    assert encoded["$type"] == (
        "HuberDensitySelectionIntervalCertificateV1"
    )
    assert CompiledPlanCodecV1.dumps(
        CompiledPlanCodecV1.loads(payload)
    ) == payload


def test_density_h3_reduced_turn_fraction_is_valid(projections):
    snapshot, legacy_request = _narrow_high_angle_snapshot()
    request = _density_request(
        legacy_request,
        MaxSubturnValueId.LINEAR_REFLEX_DENSITY_2_V1,
        ExactAngleSymbol.PI_OVER_4,
    )
    compiled = compile_reference_envelopes(snapshot, request)
    assert compiled.outcome is ReferenceOutcome.EXACT
    generated_angular = next(
        item
        for item in compiled.compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
    )
    middle = next(
        item
        for item in generated_angular.hidden_supports
        if item.ordinal == 2
    )
    assert (middle.turn_fraction.numerator, middle.turn_fraction.denominator) == (
        1,
        2,
    )

    plan = _projection(projections, "EC0-C03").plans[0]
    legacy_certificate = next(
        iter(plan.angular_profile_selection_certificates)
    )
    density_certificate = _density_certificate(
        legacy_certificate,
        value_id=MaxSubturnValueId.LINEAR_REFLEX_DENSITY_2_V1,
        q=4,
        bucket_c=4,
    )
    legacy_angular = next(
        item
        for item in plan.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
    )
    changed_angular = replace(
        legacy_angular,
        resolved_hidden_edge_count=3,
        hidden_supports=generated_angular.hidden_supports,
    )
    changed = replace(
        plan,
        angular_profile_selection_certificates=frozenset(
            {density_certificate}
        ),
        envelope_specs=frozenset(
            changed_angular if item is legacy_angular else item
            for item in plan.envelope_specs
        ),
    )

    assert not any(
        issue.code is ValidationCode.ANGULAR_CERTIFICATE
        for issue in validate_compiled_plan(changed)
    )


def test_density_interval_cross_tag_and_unknown_tag_fail_closed(projections):
    plan = _projection(projections, "EC0-C03").plans[0]
    legacy = next(iter(plan.angular_profile_selection_certificates))
    density = _density_certificate(
        legacy,
        value_id=MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1,
        q=3,
        bucket_c=2,
    )
    changed = replace(
        plan,
        angular_profile_selection_certificates=frozenset({density}),
    )
    payload = CompiledPlanCodecV1.dumps(changed)
    data = json.loads(payload)
    interval = data["angular_profile_selection_certificates"][0][
        "selection_interval_certificate"
    ]

    interval["$type"] = "SelectionIntervalCertificateV1"
    with pytest.raises(ContractCodecError, match="field mismatch"):
        CompiledPlanCodecV1.loads(json.dumps(data))

    interval["$type"] = "UnknownDensityIntervalCertificateV9"
    with pytest.raises(ContractCodecError, match="unknown tagged type"):
        CompiledPlanCodecV1.loads(json.dumps(data))


def test_density_interval_forgery_is_rejected_by_validator(projections):
    plan = _projection(projections, "EC0-C03").plans[0]
    legacy = next(iter(plan.angular_profile_selection_certificates))
    density = _density_certificate(
        legacy,
        value_id=MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1,
        q=3,
        bucket_c=2,
    )
    interval = replace(
        density.selection_interval_certificate,
        lower_bound_kind=IntervalBoundKind.CLOSED,
    )
    forged = replace(
        density,
        selection_interval_certificate=interval,
    )
    changed = replace(
        plan,
        angular_profile_selection_certificates=frozenset({forged}),
    )
    assert any(
        issue.code is ValidationCode.ANGULAR_CERTIFICATE
        and "Density A certificate" in issue.message
        for issue in validate_compiled_plan(changed)
    )


def test_unknown_density_enum_value_is_rejected_before_validation(projections):
    request = _density_request(
        _projection(projections, "EC0-C03").request,
        MaxSubturnValueId.LINEAR_REFLEX_DENSITY_0_V1,
        ExactAngleSymbol.PI_OVER_2,
    )
    data = json.loads(DecalRequestCodecV1.dumps(request))
    data["max_subturn_value_id"] = "LINEAR_REFLEX_DENSITY_9_V1"
    with pytest.raises(ContractCodecError, match="is not a valid"):
        DecalRequestCodecV1.loads(json.dumps(data))
