from __future__ import annotations

from dataclasses import replace
from hashlib import sha256
import json

import pytest

from cftuv_envelope import (
    AdmissibilityUpperBound,
    AngularProfileSelectionPolicyId,
    CompiledPlanCodecV1,
    ContractCodecError,
    DecalRequestCodecV1,
    ExactAngleSymbol,
    ExactAngleV1,
    HuberDensitySelectionIntervalCertificateV1,
    IntervalBoundKind,
    MaxSubturnParameterId,
    MaxSubturnValueId,
    MinimalityLowerBound,
    SelectionLaw,
    ValidationCode,
    validate_compiled_plan,
    validate_decal_request,
)


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


def test_legacy_request_canonical_bytes_are_frozen(projections):
    request = _projection(projections, "EC0-C03").request
    payload = DecalRequestCodecV1.dumps(request)
    assert sha256(payload).hexdigest() == (
        "bfc4f3b15b51fe7f367127c803f73f4e383866ca580a91a85eca9b093a42e90f"
    )
    assert DecalRequestCodecV1.dumps(
        DecalRequestCodecV1.loads(payload)
    ) == payload


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
