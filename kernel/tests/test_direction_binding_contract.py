from __future__ import annotations

from dataclasses import fields, replace
from decimal import Decimal
from hashlib import sha256
import json
from pathlib import Path

import pytest

import cftuv_envelope as kernel
from cftuv_envelope import (
    AngularEnvelopeSpec,
    CertifiedBoundHiddenSupportDirectionLawV1,
    CertifiedBoundHiddenSupportSpecV1,
    CertifiedDecimalIntervalV1,
    ContractCodecError,
    DirectionBindingCertificateV1,
    DirectionBindingReasonV1,
    EvaluationGeometryDirectionBindingCertificateV1,
    HiddenSupportDirectionLaw,
    HiddenSupportSpecV1,
    IntervalEndpointKind,
    ValidationCode,
    validate_compiled_plan,
)
from cftuv_envelope.codec import ContractCodecV1, canonical_json_bytes

from reference_factories import angular_snapshot


# M1 снят на неизменённом f41cb2fea13c1065981a6647e971e45c77210b15.
_HIDDEN_SUPPORTS_BEFORE_POINT_CONTACT_BOUND = {
    "angular_snapshot_k1": (
        b'[{"$type":"HiddenSupportSpecV1","direction_law":"ROTATE_INCOMING_SUPPORT_TOWARD_OUTGOING_INSIDE_ORIENTED_OWNER_SECTOR_BY_ORDINAL_SUBTURN","hidden_support_id":{"$id_type":"HiddenSupportId","value":"hidden-support:7371da7719a16ff7d21ab16b"},"ordinal":1,"owner_sector_id":{"$id_type":"OwnerSectorId","value":"angular-sector"},"scope":"ANGULAR_ENVELOPE_SPEC_LOCAL","selection_certificate_id":{"$id_type":"SelectionCertificateId","value":"profile-selection:14a84cc3debd13a19b1dac42"},"source_relation_id":{"$id_type":"CornerRelationId","value":"corner"},"turn_fraction":{"$type":"ExactRatioV1","denominator":2,"numerator":1},"zero_length_at_alpha_zero":true}]',
        "b62bdd12cac29f25525399bdc651f1774264caee3fd825ac24fb9c093fecbca4",
    ),
    "angular_snapshot_k2": (
        b'[{"$type":"HiddenSupportSpecV1","direction_law":"ROTATE_INCOMING_SUPPORT_TOWARD_OUTGOING_INSIDE_ORIENTED_OWNER_SECTOR_BY_ORDINAL_SUBTURN","hidden_support_id":{"$id_type":"HiddenSupportId","value":"hidden-support:2c663d5a15550e8cfcf8e82c"},"ordinal":2,"owner_sector_id":{"$id_type":"OwnerSectorId","value":"angular-sector"},"scope":"ANGULAR_ENVELOPE_SPEC_LOCAL","selection_certificate_id":{"$id_type":"SelectionCertificateId","value":"profile-selection:14a84cc3debd13a19b1dac42"},"source_relation_id":{"$id_type":"CornerRelationId","value":"corner"},"turn_fraction":{"$type":"ExactRatioV1","denominator":3,"numerator":2},"zero_length_at_alpha_zero":true},{"$type":"HiddenSupportSpecV1","direction_law":"ROTATE_INCOMING_SUPPORT_TOWARD_OUTGOING_INSIDE_ORIENTED_OWNER_SECTOR_BY_ORDINAL_SUBTURN","hidden_support_id":{"$id_type":"HiddenSupportId","value":"hidden-support:7371da7719a16ff7d21ab16b"},"ordinal":1,"owner_sector_id":{"$id_type":"OwnerSectorId","value":"angular-sector"},"scope":"ANGULAR_ENVELOPE_SPEC_LOCAL","selection_certificate_id":{"$id_type":"SelectionCertificateId","value":"profile-selection:14a84cc3debd13a19b1dac42"},"source_relation_id":{"$id_type":"CornerRelationId","value":"corner"},"turn_fraction":{"$type":"ExactRatioV1","denominator":3,"numerator":1},"zero_length_at_alpha_zero":true}]',
        "e663042b43cecd5479d04edb421a49a839dd34eaf8cb815d38a81ccf27f674d9",
    ),
    "point_contact_267dfe": (
        b'[{"$type":"HiddenSupportSpecV1","direction_law":"ROTATE_INCOMING_SUPPORT_TOWARD_OUTGOING_INSIDE_ORIENTED_OWNER_SECTOR_BY_ORDINAL_SUBTURN","hidden_support_id":{"$id_type":"HiddenSupportId","value":"hidden-support:ae06f5231674a1770bd535f0"},"ordinal":1,"owner_sector_id":{"$id_type":"OwnerSectorId","value":"host-v0:owner-sector:e65861ea6b0a157c0d043372"},"scope":"ANGULAR_ENVELOPE_SPEC_LOCAL","selection_certificate_id":{"$id_type":"SelectionCertificateId","value":"profile-selection:955b3036cd75b1e0bd56da0a"},"source_relation_id":{"$id_type":"CornerRelationId","value":"host-v0:corner-relation:05676524690dd8b689264248"},"turn_fraction":{"$type":"ExactRatioV1","denominator":2,"numerator":1},"zero_length_at_alpha_zero":true}]',
        "03defadbafd57d441665b836957e7ef5d38c6d6f383e198c00bd8dddee11ea5b",
    ),
    "point_contact_8e2a63": (
        b'[{"$type":"HiddenSupportSpecV1","direction_law":"ROTATE_INCOMING_SUPPORT_TOWARD_OUTGOING_INSIDE_ORIENTED_OWNER_SECTOR_BY_ORDINAL_SUBTURN","hidden_support_id":{"$id_type":"HiddenSupportId","value":"hidden-support:a5edc60ed729b5a626126fb1"},"ordinal":1,"owner_sector_id":{"$id_type":"OwnerSectorId","value":"host-v0:owner-sector:b4426220b8b8721bc83b51c0"},"scope":"ANGULAR_ENVELOPE_SPEC_LOCAL","selection_certificate_id":{"$id_type":"SelectionCertificateId","value":"profile-selection:27c820078d63ddb05421396e"},"source_relation_id":{"$id_type":"CornerRelationId","value":"host-v0:corner-relation:60fc81c8f39e546c88f9ff6a"},"turn_fraction":{"$type":"ExactRatioV1","denominator":2,"numerator":1},"zero_length_at_alpha_zero":true}]',
        "b6d89cdda82429920d766ca1c88116c0f2d0ac48fd4fdd6161399f2200ab1371",
    ),
}

_FROZEN_HIDDEN_SUPPORTS = {
    "angular_snapshot_k1": (
        _HIDDEN_SUPPORTS_BEFORE_POINT_CONTACT_BOUND["angular_snapshot_k1"]
    ),
    "angular_snapshot_k2": (
        _HIDDEN_SUPPORTS_BEFORE_POINT_CONTACT_BOUND["angular_snapshot_k2"]
    ),
    "point_contact_267dfe": (
        b'[{"$type":"CertifiedBoundHiddenSupportSpecV1","direction_binding":{"$type":"EvaluationGeometryDirectionBindingCertificateV1","binding_reason":"EVALUATION_GEOMETRY_UNBINDS_SOURCE_RATIONAL","bound_primitive_integer_vector":[-1,-3],"certified_window_width_lower_bound":"0.08025020126798924685159819","ideal_window_lower_slope_envelope":{"$type":"CertifiedDecimalIntervalV1","absolute_error_bound":"0.000000000000000000000000001","lower":"0.31296299706330317585668607","lower_kind":"CLOSED","upper":"0.312962997063303175856686071","upper_kind":"CLOSED"},"ideal_window_upper_slope_envelope":{"$type":"CertifiedDecimalIntervalV1","absolute_error_bound":"0.000000000000000000000000001","lower":"0.393213198331292422708284261","lower_kind":"CLOSED","upper":"0.393213198331292422708284262","upper_kind":"CLOSED"},"proven_predicates":["BINDING_INSIDE_OWN_ORDINAL_WINDOW","BINDING_MONOTONE","BINDING_SUBTURN_LE_DELTA_MAX"]},"direction_law":"CERTIFIED_RATIONAL_BINDING_IN_ORDINAL_SUBTURN_V1","hidden_support_id":{"$id_type":"HiddenSupportId","value":"hidden-support:ae06f5231674a1770bd535f0"},"ordinal":1,"owner_sector_id":{"$id_type":"OwnerSectorId","value":"host-v0:owner-sector:e65861ea6b0a157c0d043372"},"scope":"ANGULAR_ENVELOPE_SPEC_LOCAL","selection_certificate_id":{"$id_type":"SelectionCertificateId","value":"profile-selection:955b3036cd75b1e0bd56da0a"},"source_relation_id":{"$id_type":"CornerRelationId","value":"host-v0:corner-relation:05676524690dd8b689264248"},"turn_fraction":{"$type":"ExactRatioV1","denominator":2,"numerator":1},"zero_length_at_alpha_zero":true}]',
        "02968666e72e53bec74a9073bddfd28a2dfe29deeff269687b50ef387d08897e",
    ),
    "point_contact_8e2a63": (
        b'[{"$type":"CertifiedBoundHiddenSupportSpecV1","direction_binding":{"$type":"EvaluationGeometryDirectionBindingCertificateV1","binding_reason":"EVALUATION_GEOMETRY_UNBINDS_SOURCE_RATIONAL","bound_primitive_integer_vector":[-1,-5],"certified_window_width_lower_bound":"0.109663603700431653678276885","ideal_window_lower_slope_envelope":{"$type":"CertifiedDecimalIntervalV1","absolute_error_bound":"0.000000000000000000000000001","lower":"0.120772471296262547485663174","lower_kind":"CLOSED","upper":"0.120772471296262547485663175","upper_kind":"CLOSED"},"ideal_window_upper_slope_envelope":{"$type":"CertifiedDecimalIntervalV1","absolute_error_bound":"0.000000000000000000000000001","lower":"0.23043607499669420116394006","lower_kind":"CLOSED","upper":"0.230436074996694201163940061","upper_kind":"CLOSED"},"proven_predicates":["BINDING_INSIDE_OWN_ORDINAL_WINDOW","BINDING_MONOTONE","BINDING_SUBTURN_LE_DELTA_MAX"]},"direction_law":"CERTIFIED_RATIONAL_BINDING_IN_ORDINAL_SUBTURN_V1","hidden_support_id":{"$id_type":"HiddenSupportId","value":"hidden-support:a5edc60ed729b5a626126fb1"},"ordinal":1,"owner_sector_id":{"$id_type":"OwnerSectorId","value":"host-v0:owner-sector:b4426220b8b8721bc83b51c0"},"scope":"ANGULAR_ENVELOPE_SPEC_LOCAL","selection_certificate_id":{"$id_type":"SelectionCertificateId","value":"profile-selection:27c820078d63ddb05421396e"},"source_relation_id":{"$id_type":"CornerRelationId","value":"host-v0:corner-relation:60fc81c8f39e546c88f9ff6a"},"turn_fraction":{"$type":"ExactRatioV1","denominator":2,"numerator":1},"zero_length_at_alpha_zero":true}]',
        "66fc35ec93472762bedcb665130168fa8c1b316dcca75f36530ef8027dd767d4",
    ),
}


class _AngularEnvelopeCodecV1(ContractCodecV1[AngularEnvelopeSpec]):
    root_type = AngularEnvelopeSpec


def _angular_spec(snapshot, request) -> AngularEnvelopeSpec:
    compilation = kernel.compile_reference_envelopes(snapshot, request).compilation
    assert compilation is not None
    return next(
        item
        for item in compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
    )


def _point_contact_specs() -> dict[str, AngularEnvelopeSpec]:
    folder = Path(__file__).parents[1] / "fixtures" / "building_002_point_contact_v1"
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (folder / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (folder / "decal_request.json").read_bytes()
    )
    compilation = kernel.compile_reference_envelopes(snapshot, request).compilation
    assert compilation is not None
    return {
        item.envelope_spec_id.value: item
        for item in compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
    }


def test_m1_final_plan_bytes_move_only_with_named_geometry_binding():
    """Synthetic pass-through is bitwise; point_contact moves with history.

    M1 is a tripwire against unnamed movement, not a ban on changing the
    authority record when the declared evaluation-geometry law applies.
    """

    k1 = _angular_spec(*angular_snapshot(1))
    k2 = _angular_spec(*angular_snapshot(2))
    point_contact = _point_contact_specs()
    actual = {
        "angular_snapshot_k1": k1,
        "angular_snapshot_k2": k2,
        "point_contact_267dfe": point_contact[
            "angular-spec:267dfe530401a7b37c89e59c"
        ],
        "point_contact_8e2a63": point_contact[
            "angular-spec:8e2a63c70b86dea8a456f34b"
        ],
    }
    for name, spec in actual.items():
        if name.startswith("point_contact"):
            assert all(
                type(item) is CertifiedBoundHiddenSupportSpecV1
                for item in spec.hidden_supports
            )
            assert {
                item.direction_binding.binding_reason
                for item in spec.hidden_supports
            } == {
                DirectionBindingReasonV1.EVALUATION_GEOMETRY_UNBINDS_SOURCE_RATIONAL
            }
            historical_bytes, historical_digest = (
                _HIDDEN_SUPPORTS_BEFORE_POINT_CONTACT_BOUND[name]
            )
            assert (
                sha256(historical_bytes).hexdigest()
                == historical_digest
            )
        else:
            assert all(
                type(item) is HiddenSupportSpecV1
                for item in spec.hidden_supports
            )
            assert _FROZEN_HIDDEN_SUPPORTS[name] == (
                _HIDDEN_SUPPORTS_BEFORE_POINT_CONTACT_BOUND[name]
            )
        payload = canonical_json_bytes(spec.hidden_supports)
        if name.startswith("point_contact"):
            assert payload != historical_bytes
        expected_bytes, expected_digest = _FROZEN_HIDDEN_SUPPORTS[name]
        assert payload == expected_bytes, name
        assert sha256(payload).hexdigest() == expected_digest, name
        round_trip = _AngularEnvelopeCodecV1.loads(
            _AngularEnvelopeCodecV1.dumps(spec)
        )
        assert _AngularEnvelopeCodecV1.dumps(round_trip) == (
            _AngularEnvelopeCodecV1.dumps(spec)
        )
        assert canonical_json_bytes(round_trip.hidden_supports) == expected_bytes


def _legacy_support_data() -> tuple[AngularEnvelopeSpec, dict]:
    spec = _angular_spec(*angular_snapshot(1))
    data = json.loads(_AngularEnvelopeCodecV1.dumps(spec))
    return spec, data["hidden_supports"][0]


def test_m2_legacy_tag_rejects_extra_direction_binding_field():
    spec, support = _legacy_support_data()
    support["direction_binding"] = {}
    data = json.loads(_AngularEnvelopeCodecV1.dumps(spec))
    data["hidden_supports"] = [support]
    with pytest.raises(
        ContractCodecError,
        match=r"HiddenSupportSpecV1 field mismatch; extra=\['direction_binding'\]",
    ):
        _AngularEnvelopeCodecV1.loads(json.dumps(data))


def test_m3_tags_cannot_smuggle_fields_between_the_two_records():
    spec, legacy = _legacy_support_data()
    old_tag_with_new_fields = dict(legacy, direction_binding={})
    new_tag_without_certificate = dict(legacy)
    new_tag_without_certificate["$type"] = "CertifiedBoundHiddenSupportSpecV1"
    for support in (old_tag_with_new_fields, new_tag_without_certificate):
        data = json.loads(_AngularEnvelopeCodecV1.dumps(spec))
        data["hidden_supports"] = [support]
        with pytest.raises(ContractCodecError, match="field mismatch"):
            _AngularEnvelopeCodecV1.loads(json.dumps(data))


def test_m4_unknown_tag_inside_hidden_support_union_fails_closed():
    spec, support = _legacy_support_data()
    support["$type"] = "FutureBoundHiddenSupportSpecV9"
    data = json.loads(_AngularEnvelopeCodecV1.dumps(spec))
    data["hidden_supports"] = [support]
    with pytest.raises(
        ContractCodecError,
        match="unknown tagged type: FutureBoundHiddenSupportSpecV9",
    ):
        _AngularEnvelopeCodecV1.loads(json.dumps(data))


def _closed(value):
    return CertifiedDecimalIntervalV1(
        Decimal(value),
        Decimal(value),
        IntervalEndpointKind.CLOSED,
        IntervalEndpointKind.CLOSED,
        Decimal(0),
    )


def _binding():
    return DirectionBindingCertificateV1(
        bound_primitive_integer_vector=(3, 4),
        ideal_window_lower_slope_envelope=_closed("0.7"),
        ideal_window_upper_slope_envelope=_closed("0.9"),
        certified_window_width_lower_bound=Decimal("0.2"),
        proven_predicates=frozenset(
            {
                "BINDING_MONOTONE",
                "BINDING_SUBTURN_LE_DELTA_MAX",
                "BINDING_INSIDE_OWN_ORDINAL_WINDOW",
            }
        ),
    )


def _evaluation_binding():
    base = _binding()
    return EvaluationGeometryDirectionBindingCertificateV1(
        bound_primitive_integer_vector=(
            base.bound_primitive_integer_vector
        ),
        ideal_window_lower_slope_envelope=(
            base.ideal_window_lower_slope_envelope
        ),
        ideal_window_upper_slope_envelope=(
            base.ideal_window_upper_slope_envelope
        ),
        certified_window_width_lower_bound=(
            base.certified_window_width_lower_bound
        ),
        proven_predicates=base.proven_predicates,
        binding_reason=(
            DirectionBindingReasonV1.EVALUATION_GEOMETRY_UNBINDS_SOURCE_RATIONAL
        ),
    )


def _bound_support(legacy):
    values = {
        field.name: getattr(legacy, field.name)
        for field in fields(legacy)
        if field.name != "direction_law"
    }
    return CertifiedBoundHiddenSupportSpecV1(
        **values,
        direction_law=(
            CertifiedBoundHiddenSupportDirectionLawV1.CERTIFIED_RATIONAL_BINDING_IN_ORDINAL_SUBTURN_V1
        ),
        direction_binding=_binding(),
    )


def test_legacy_tag_rejects_certified_law_without_certificate():
    spec, support = _legacy_support_data()
    support["direction_law"] = (
        CertifiedBoundHiddenSupportDirectionLawV1.CERTIFIED_RATIONAL_BINDING_IN_ORDINAL_SUBTURN_V1.value
    )
    data = json.loads(_AngularEnvelopeCodecV1.dumps(spec))
    data["hidden_supports"] = [support]

    with pytest.raises(ContractCodecError) as caught:
        _AngularEnvelopeCodecV1.loads(json.dumps(data))

    assert str(caught.value) == (
        "'CERTIFIED_RATIONAL_BINDING_IN_ORDINAL_SUBTURN_V1' is not a valid "
        "HiddenSupportDirectionLaw"
    )


def test_bound_tag_rejects_legacy_law_even_with_certificate():
    spec = _angular_spec(*angular_snapshot(1))
    legacy = next(iter(spec.hidden_supports))
    bound_spec = replace(
        spec,
        hidden_supports=frozenset({_bound_support(legacy)}),
    )
    data = json.loads(_AngularEnvelopeCodecV1.dumps(bound_spec))
    data["hidden_supports"][0]["direction_law"] = (
        HiddenSupportDirectionLaw.ORIENTED_OWNER_SECTOR_ORDINAL_SUBTURN.value
    )

    with pytest.raises(ContractCodecError) as caught:
        _AngularEnvelopeCodecV1.loads(json.dumps(data))

    assert str(caught.value) == (
        "'ROTATE_INCOMING_SUPPORT_TOWARD_OUTGOING_INSIDE_ORIENTED_OWNER_"
        "SECTOR_BY_ORDINAL_SUBTURN' is not a valid "
        "CertifiedBoundHiddenSupportDirectionLawV1"
    )


def test_bound_record_is_a_distinct_mandatory_tagged_shape():
    spec = _angular_spec(*angular_snapshot(1))
    bound = _bound_support(next(iter(spec.hidden_supports)))
    encoded = canonical_json_bytes(bound)
    assert b'"$type":"CertifiedBoundHiddenSupportSpecV1"' in encoded
    assert b'"direction_binding"' in encoded


def test_evaluation_bound_certificate_has_one_mandatory_reasoned_tag():
    spec = _angular_spec(*angular_snapshot(1))
    legacy = next(iter(spec.hidden_supports))
    bound = replace(
        _bound_support(legacy),
        direction_binding=_evaluation_binding(),
    )
    changed = replace(spec, hidden_supports=frozenset({bound}))
    encoded = _AngularEnvelopeCodecV1.dumps(changed)
    round_trip = _AngularEnvelopeCodecV1.loads(encoded)
    assert _AngularEnvelopeCodecV1.dumps(round_trip) == encoded

    data = json.loads(encoded)
    certificate = data["hidden_supports"][0]["direction_binding"]
    assert certificate["$type"] == (
        "EvaluationGeometryDirectionBindingCertificateV1"
    )
    assert certificate["binding_reason"] == (
        "EVALUATION_GEOMETRY_UNBINDS_SOURCE_RATIONAL"
    )

    old_tag = json.loads(encoded)
    old_tag["hidden_supports"][0]["direction_binding"]["$type"] = (
        "DirectionBindingCertificateV1"
    )
    missing_reason = json.loads(encoded)
    missing_reason["hidden_supports"][0]["direction_binding"].pop(
        "binding_reason"
    )
    unknown_reason = json.loads(encoded)
    unknown_reason["hidden_supports"][0]["direction_binding"][
        "binding_reason"
    ] = "SILENT_REPAIR"
    for malformed in (old_tag, missing_reason, unknown_reason):
        with pytest.raises(ContractCodecError):
            _AngularEnvelopeCodecV1.loads(json.dumps(malformed))


def _legacy_plan(projections):
    plan = next(item for item in projections if item.case_id == "EC0-C03").plans[0]
    angular = next(
        item for item in plan.envelope_specs if isinstance(item, AngularEnvelopeSpec)
    )
    return plan, angular, next(iter(angular.hidden_supports))


def _bound_plan(projections):
    plan, angular, legacy = _legacy_plan(projections)
    bound = _bound_support(legacy)
    changed = replace(angular, hidden_supports=frozenset({bound}))
    return replace(
        plan,
        envelope_specs=(plan.envelope_specs - {angular}) | {changed},
    ), changed, bound


def test_full_plan_codec_rejects_legacy_tag_with_certified_law(projections):
    plan, angular, legacy = _legacy_plan(projections)
    forged = replace(
        legacy,
        direction_law=(
            CertifiedBoundHiddenSupportDirectionLawV1.CERTIFIED_RATIONAL_BINDING_IN_ORDINAL_SUBTURN_V1
        ),
    )
    changed = replace(angular, hidden_supports=frozenset({forged}))
    invalid = replace(
        plan,
        envelope_specs=(plan.envelope_specs - {angular}) | {changed},
    )
    payload = kernel.CompiledPlanCodecV1.dumps(invalid)
    assert payload == kernel.CompiledPlanCodecV1.dumps(invalid)

    with pytest.raises(ContractCodecError) as caught:
        kernel.CompiledPlanCodecV1.loads(payload)

    assert str(caught.value) == (
        "'CERTIFIED_RATIONAL_BINDING_IN_ORDINAL_SUBTURN_V1' is not a valid "
        "HiddenSupportDirectionLaw"
    )


def test_full_plan_codec_rejects_bound_tag_with_legacy_law(projections):
    plan, angular, bound = _bound_plan(projections)
    forged = replace(
        bound,
        direction_law=(
            HiddenSupportDirectionLaw.ORIENTED_OWNER_SECTOR_ORDINAL_SUBTURN
        ),
    )
    changed = replace(angular, hidden_supports=frozenset({forged}))
    invalid = replace(
        plan,
        envelope_specs=(plan.envelope_specs - {angular}) | {changed},
    )
    payload = kernel.CompiledPlanCodecV1.dumps(invalid)
    assert payload == kernel.CompiledPlanCodecV1.dumps(invalid)

    with pytest.raises(ContractCodecError) as caught:
        kernel.CompiledPlanCodecV1.loads(payload)

    assert str(caught.value) == (
        "'ROTATE_INCOMING_SUPPORT_TOWARD_OUTGOING_INSIDE_ORIENTED_OWNER_"
        "SECTOR_BY_ORDINAL_SUBTURN' is not a valid "
        "CertifiedBoundHiddenSupportDirectionLawV1"
    )


def test_validator_rejects_cross_enum_legacy_direction_law(projections):
    plan, angular, legacy = _legacy_plan(projections)
    forged = replace(
        legacy,
        direction_law=(
            CertifiedBoundHiddenSupportDirectionLawV1.CERTIFIED_RATIONAL_BINDING_IN_ORDINAL_SUBTURN_V1
        ),
    )
    changed = replace(angular, hidden_supports=frozenset({forged}))
    invalid = replace(
        plan,
        envelope_specs=(plan.envelope_specs - {angular}) | {changed},
    )

    issues = validate_compiled_plan(invalid)

    assert any(
        issue.code is ValidationCode.ANGULAR_CERTIFICATE
        and issue.path[-1] == "direction_law"
        and issue.message
        == "legacy hidden support must declare the oriented owner-sector ordinal subturn law"
        for issue in issues
    )


def test_validator_rejects_cross_enum_bound_direction_law(projections):
    plan, angular, bound = _bound_plan(projections)
    forged = replace(
        bound,
        direction_law=(
            HiddenSupportDirectionLaw.ORIENTED_OWNER_SECTOR_ORDINAL_SUBTURN
        ),
    )
    changed = replace(angular, hidden_supports=frozenset({forged}))
    invalid = replace(
        plan,
        envelope_specs=(plan.envelope_specs - {angular}) | {changed},
    )

    issues = validate_compiled_plan(invalid)

    assert any(
        issue.code is ValidationCode.ANGULAR_CERTIFICATE
        and issue.path[-1] == "direction_law"
        and issue.message
        == "bound support must declare the certified rational binding law"
        for issue in issues
    )


@pytest.mark.parametrize("vector", [(0, 0), (6, 8)])
def test_validator_rejects_zero_and_nonprimitive_bound_vectors(
    projections, vector
):
    plan, angular, bound = _bound_plan(projections)
    forged = replace(
        bound,
        direction_binding=replace(
            bound.direction_binding,
            bound_primitive_integer_vector=vector,
        ),
    )
    changed = replace(angular, hidden_supports=frozenset({forged}))
    invalid = replace(
        plan,
        envelope_specs=(plan.envelope_specs - {angular}) | {changed},
    )
    assert any(
        issue.code is ValidationCode.ANGULAR_CERTIFICATE
        and "nonzero primitive integer vector" in issue.message
        for issue in validate_compiled_plan(invalid)
    )


def test_validator_rejects_unordered_binding_envelopes(projections):
    plan, angular, bound = _bound_plan(projections)
    closed = lambda value: CertifiedDecimalIntervalV1(
        Decimal(value),
        Decimal(value),
        IntervalEndpointKind.CLOSED,
        IntervalEndpointKind.CLOSED,
        Decimal(0),
    )
    forged = replace(
        bound,
        direction_binding=replace(
            bound.direction_binding,
            ideal_window_lower_slope_envelope=closed("1.0"),
            ideal_window_upper_slope_envelope=closed("0.9"),
        ),
    )
    changed = replace(angular, hidden_supports=frozenset({forged}))
    invalid = replace(
        plan,
        envelope_specs=(plan.envelope_specs - {angular}) | {changed},
    )
    assert any(
        issue.code is ValidationCode.ANGULAR_CERTIFICATE
        and "strictly ordered" in issue.message
        for issue in validate_compiled_plan(invalid)
    )
