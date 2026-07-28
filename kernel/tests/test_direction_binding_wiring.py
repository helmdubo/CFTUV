from __future__ import annotations

from dataclasses import replace
from fractions import Fraction
from pathlib import Path

import pytest
import sympy as sp

import cftuv_envelope as kernel
from cftuv_envelope import (
    AngularEnvelopeSpec,
    CertifiedBoundHiddenSupportDirectionLawV1,
    CertifiedBoundHiddenSupportSpecV1,
    HiddenSupportDirectionLaw,
    HiddenSupportSpecV1,
    ReferenceOutcome,
)
from cftuv_envelope.interactions.arrival import angular_hidden_support_lines
from cftuv_envelope.reference.angular import evaluate_angular_envelope
from cftuv_envelope.reference.common import GeometryContext, ReferenceGeometryError
from cftuv_envelope.reference.validation import validate_reference_geometry_payload

from reference_factories import angular_snapshot
from direction_binding_oracle import assert_k1_dual_metric_binding


_FIXTURES = Path(__file__).parents[1] / "fixtures"
_FULL = _FIXTURES / "building_002_full_selection_v1"
_POINT = _FIXTURES / "building_002_point_contact_v1"
_BOUND_SPEC = "angular-spec:66cf5e6f75ddafed7cdb3dca"


def _inputs(folder):
    return (
        kernel.AnalysisSnapshotCodecV1.loads(
            (folder / "analysis_snapshot.json").read_bytes()
        ),
        kernel.DecalRequestCodecV1.loads(
            (folder / "decal_request.json").read_bytes()
        ),
    )


def _compiled(folder):
    snapshot, request = _inputs(folder)
    result = kernel.compile_reference_envelopes(snapshot, request)
    assert result.outcome is ReferenceOutcome.EXACT, result.diagnostics
    assert result.compilation is not None
    return snapshot, request, result.compilation


def _context(compilation):
    frame, diagnostics = validate_reference_geometry_payload(
        compilation.analysis_snapshot,
        compilation.plan_key.patch_domain_id,
    )
    assert frame is not None, diagnostics
    return GeometryContext.build(compilation, frame)


def test_compiler_binds_only_the_field_ordinal_that_needs_it():
    _, _, compilation = _compiled(_FULL)
    angular = sorted(
        (
            item
            for item in compilation.envelope_specs
            if isinstance(item, AngularEnvelopeSpec)
        ),
        key=lambda item: item.envelope_spec_id.value,
    )
    bound = [
        (spec, support)
        for spec in angular
        for support in spec.hidden_supports
        if isinstance(support, CertifiedBoundHiddenSupportSpecV1)
    ]
    assert len(angular) == 4
    assert len(bound) == 1
    spec, support = bound[0]
    assert spec.envelope_spec_id.value == _BOUND_SPEC
    assert support.ordinal == 1
    assert support.direction_law is (
        CertifiedBoundHiddenSupportDirectionLawV1.CERTIFIED_RATIONAL_BINDING_IN_ORDINAL_SUBTURN_V1
    )
    assert support.direction_binding.bound_primitive_integer_vector == (1, 3)


def test_pass_through_corpora_keep_the_legacy_runtime_record_type():
    for folder in (_POINT,):
        _, _, compilation = _compiled(folder)
        supports = [
            support
            for spec in compilation.envelope_specs
            if isinstance(spec, AngularEnvelopeSpec)
            for support in spec.hidden_supports
        ]
        assert supports
        assert all(type(item) is HiddenSupportSpecV1 for item in supports)
    snapshot, request = angular_snapshot(2)
    compilation = kernel.compile_reference_envelopes(
        snapshot, request
    ).compilation
    assert compilation is not None
    spec = next(
        item
        for item in compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
    )
    assert all(type(item) is HiddenSupportSpecV1 for item in spec.hidden_supports)


def _forged_compilation(compilation):
    spec = next(
        item
        for item in compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
        and item.envelope_spec_id.value == _BOUND_SPEC
    )
    support = next(iter(spec.hidden_supports))
    forged_support = replace(
        support,
        direction_binding=replace(
            support.direction_binding,
            bound_primitive_integer_vector=(-1, -3),
        ),
    )
    forged_spec = replace(spec, hidden_supports=frozenset({forged_support}))
    forged = replace(
        compilation,
        envelope_specs=(compilation.envelope_specs - {spec}) | {forged_spec},
    )
    return forged, forged_spec


def _replace_angular_support(compilation, spec, support):
    changed_spec = replace(spec, hidden_supports=frozenset({support}))
    changed = replace(
        compilation,
        envelope_specs=(compilation.envelope_specs - {spec}) | {changed_spec},
    )
    return changed, changed_spec


def _assert_reference_and_queue_refuse(context, spec, request):
    expected = (
        "direction binding certificate is not proven: "
        "привязка: сертификат не доказан (BINDING_MONOTONE)"
    )
    consumers = (
        lambda: evaluate_angular_envelope(
            context,
            spec,
            request.requested_alpha,
            sp.Rational(1, 4),
        ),
        lambda: angular_hidden_support_lines(context, spec),
    )
    for consume in consumers:
        with pytest.raises(ReferenceGeometryError) as caught:
            consume()
        assert caught.value.outcome is (
            ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE
        )
        assert str(caught.value) == expected


def test_reference_and_queue_recheck_the_same_forged_plan_record():
    _, request, compilation = _compiled(_FULL)
    forged, forged_spec = _forged_compilation(compilation)
    assert next(
        item
        for item in forged.envelope_specs
        if getattr(item, "envelope_spec_id", None)
        == forged_spec.envelope_spec_id
    ) is forged_spec
    _assert_reference_and_queue_refuse(
        _context(forged),
        forged_spec,
        request,
    )


@pytest.mark.parametrize("record_kind", ("legacy", "bound"))
def test_reference_and_queue_reject_cross_enum_direction_law(record_kind):
    if record_kind == "legacy":
        snapshot, request = angular_snapshot(1)
        result = kernel.compile_reference_envelopes(snapshot, request)
        assert result.outcome is ReferenceOutcome.EXACT
        compilation = result.compilation
        spec = next(
            item
            for item in compilation.envelope_specs
            if isinstance(item, AngularEnvelopeSpec)
        )
        support = next(iter(spec.hidden_supports))
        assert type(support) is HiddenSupportSpecV1
        forged_support = replace(
            support,
            direction_law=(
                CertifiedBoundHiddenSupportDirectionLawV1.CERTIFIED_RATIONAL_BINDING_IN_ORDINAL_SUBTURN_V1
            ),
        )
    else:
        _, request, compilation = _compiled(_FULL)
        spec = next(
            item
            for item in compilation.envelope_specs
            if isinstance(item, AngularEnvelopeSpec)
            and item.envelope_spec_id.value == _BOUND_SPEC
        )
        support = next(iter(spec.hidden_supports))
        assert type(support) is CertifiedBoundHiddenSupportSpecV1
        forged_support = replace(
            support,
            direction_law=(
                HiddenSupportDirectionLaw.ORIENTED_OWNER_SECTOR_ORDINAL_SUBTURN
            ),
        )
    forged, forged_spec = _replace_angular_support(
        compilation,
        spec,
        forged_support,
    )

    _assert_reference_and_queue_refuse(
        _context(forged),
        forged_spec,
        request,
    )


def test_compile_refusal_uses_the_existing_named_outcome(monkeypatch):
    import cftuv_envelope.reference.compile as compile_module

    original = compile_module.certify_direction_bindings

    def forged(*args, **kwargs):
        certificates = original(*args, **kwargs)
        return tuple(
            replace(
                certificate,
                bound_primitive_integer_vector=tuple(
                    -value
                    for value in certificate.bound_primitive_integer_vector
                ),
            )
            for certificate in certificates
        )

    monkeypatch.setattr(
        compile_module, "certify_direction_bindings", forged
    )
    snapshot, request = _inputs(_FULL)
    result = compile_module.compile_reference_envelopes(snapshot, request)
    assert result.outcome is (
        ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE
    )
    assert result.compilation is None
    assert result.diagnostics[0].message.startswith(
        "direction binding certificate is not proven"
    )


def test_irrational_anchor_degrades_before_bound_support_consumption(
    monkeypatch,
):
    from cftuv_envelope.wavefront import conveyor as conveyor_module

    _, _, compilation = _compiled(_FULL)
    spec = next(
        item
        for item in compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
        and item.envelope_spec_id.value == _BOUND_SPEC
    )
    context = _context(compilation)
    monkeypatch.setattr(conveyor_module, "_rational_point", lambda point: None)

    def must_not_consume(*args, **kwargs):
        raise AssertionError("bound support was consumed before the anchor gate")

    monkeypatch.setattr(
        conveyor_module, "angular_hidden_support_lines", must_not_consume
    )
    fan, rescaled, corner = conveyor_module._one_fan(
        context, spec, sp.Integer(1)
    )
    assert fan is None
    assert rescaled == 0
    assert corner.anchor is None
    assert corner.reason == "якорь не рационален"


def test_full_selection_bound_vector_passes_the_independent_oracle():
    _, _, compilation = _compiled(_FULL)
    spec = next(
        item
        for item in compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
        and item.envelope_spec_id.value == _BOUND_SPEC
    )
    support = next(iter(spec.hidden_supports))
    # Входные primitive covectors и G^-1 сняты с fixture facts независимо от
    # production direction_binding; oracle-модуль его даже не импортирует.
    dual_metric = (
        (
            Fraction(225566851072, 81348737089),
            Fraction(-341439021056, 406743685445),
        ),
        (
            Fraction(-341439021056, 406743685445),
            Fraction(108323405824, 406743685445),
        ),
    )
    assert_k1_dual_metric_binding(
        dual_metric,
        (2582, 9011),
        (3261066, 5676553),
        support.direction_binding.bound_primitive_integer_vector,
        clockwise=True,
    )
