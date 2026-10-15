"""Typed attachment assembly/lifecycle; здесь не доказывается геометрия fake frontend."""

from dataclasses import replace
from fractions import Fraction
import pickle

import pytest

from test_density_raw_v1 import _case_inputs, _selections, CASES
from cftuv_envelope.reference import compile as compiler, density_attach_v1 as attach
from cftuv_envelope.reference.contracts import ReferenceOutcome
from cftuv_envelope.reference.density_attach_v1 import attach_density_specs_v1, attach_density_results_v1
from cftuv_envelope.reference.density_oracle_v1 import (
    BindingCertificateFactsV1, CanonicalFanAuthorityFactsV1, CanonicalLawRefusalFactsV1, CanonUsageFactsV1,
    CanonicalSubturnAuthorityFactsV1, CountLiftFactsV1, NoiseRecordFactsV1,
    DecimalIntervalFactsV1, DecimalValueFactsV1, DensityFailureFactsV1, DensityOracleCanonV1,
    DensityPriceArticleV1, DensityShellEventFactsV1, RawDensitySpecResultV1,
    DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1,
)
from cftuv_envelope.reference.density_raw_v1 import extract_raw_density_spec_input
from cftuv_envelope.reference.density_scalar_v2 import DENSITY_SOURCE_CONSTRUCTION_V1
from cftuv_envelope.reference.density_shell_v2 import DENSITY_SHELL_CANON_V2_CANDIDATE
from cftuv_envelope.reference.density_shell_events import DensityShellEventCollector


class FixedFrontend:
    canon_ids = DensityOracleCanonV1(DENSITY_SOURCE_CONSTRUCTION_V1, DENSITY_SHELL_CANON_V2_CANDIDATE,
                                     DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1)

    def __init__(self, outputs):
        self.outputs, self.calls = outputs, []

    def evaluate(self, raw):
        self.calls.append(raw.spec_id)
        result = self.outputs[raw.spec_id]
        if isinstance(result, Exception):
            raise result
        return result


@pytest.fixture
def inputs(monkeypatch):
    source, context = _case_inputs(monkeypatch, *CASES[0])
    raw = tuple(extract_raw_density_spec_input(source, context, spec, selection)
                for spec, selection in _selections(context))
    assert len(raw) == 2
    return context.compilation, context, source, raw


def _result(raw, **changes):
    count = raw.selection.source_hidden_count
    value = RawDensitySpecResultV1("OK", count, (None,)*count, (None,)*count, None, None,
        (CanonUsageFactsV1(DENSITY_SOURCE_CONSTRUCTION_V1, "DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED"),),
        (DensityPriceArticleV1("PREPARATION", 65536, (("DENSITY_FAN_ORDER_STEPS", count),)),), (), ())
    if changes.get("failure") is not None:
        value = replace(value, effective_hidden_count=None, binding_reasons=(), binding_certificates=())
    return replace(value, **changes)


def _forbidden(*args, **kwargs):
    raise AssertionError("attachment called a legacy Density producer")


def test_same_frontend_sorted_specs_no_binding_identity_and_nondensity_preserved(inputs, monkeypatch):
    compilation, context, source, raw = inputs
    outputs = {value.spec_id: _result(value) for value in raw}
    frontend = FixedFrontend(outputs)
    for name in ("_evaluation_density_spec", "bind_density_fan", "canonical_fan_rays_decision",
                 "has_rational_density_support_direction", "_density_spec_with_hidden_count"):
        monkeypatch.setattr(compiler, name, _forbidden)
    result = attach_density_specs_v1(compilation, context, source, frontend, tuple(reversed(raw)))
    assert frontend.calls == sorted(value.spec_id for value in raw)
    assert result.failure is None and result.envelope_specs == compilation.envelope_specs
    assert all(any(old is item for old in compilation.envelope_specs) for item in result.envelope_specs)
    assert result.ordered_work_articles == tuple(outputs[key].ordered_work_articles[0] for key in frontend.calls)
    assert result.canonical_subturn_fan_authorities == result.evaluation_binding_noise_records == frozenset()
    assert pickle.loads(pickle.dumps(result)) == result


def test_complete_pairing_rejects_dropped_extra_duplicate_and_wrong_selection_before_frontend(inputs):
    compilation, context, source, raw = inputs
    wrong = replace(raw[0], selection=replace(raw[0].selection, selection_certificate_id=raw[1].selection.selection_certificate_id))
    extra = replace(raw[0], spec_id="unknown")
    for values in (raw[:1], (*raw, raw[0]), (*raw, extra), (wrong, raw[1])):
        frontend = FixedFrontend({})
        with pytest.raises(ValueError):
            attach_density_specs_v1(compilation, context, source, frontend, values)
        assert frontend.calls == []


def test_success_diagnostics_use_noise_then_rays_then_band_categories(inputs):
    compilation, context, source, raw = inputs
    outputs = {value.spec_id: _result(value,
        law_refusals=CanonicalLawRefusalFactsV1("SOURCE_COORDINATES_UNAVAILABLE", "NO_CANONICAL_ROTATION_TABLE_ENTRY"),
        named_refusals=((value.spec_id, "NO_USABLE_NARROW_BAND"),)) for value in raw}
    result = attach_density_specs_v1(compilation, context, source, FixedFrontend(outputs), raw)
    diagnostics = (*result.noise_diagnostics, *result.rays_diagnostics, *result.band_diagnostics)
    assert [value.outcome for value in diagnostics] == [ReferenceOutcome.EVALUATION_BINDING_NOISE_LAW_NOT_APPLIED]*2 + [
        ReferenceOutcome.CANONICAL_FAN_RAYS_LAW_NOT_APPLIED]*2 + [ReferenceOutcome.ADAPTIVE_FAN_NARROW_BAND_NOT_APPLIED]*2
    assert [value.envelope_spec_id for value in result.noise_diagnostics] == sorted(value.spec_id for value in raw)


def test_known_failure_is_atomic_stops_and_retains_prior_work_and_same_collector_events(inputs):
    compilation, context, source, raw = inputs
    ordered = sorted(raw, key=lambda value: value.spec_id)
    shell = DensityShellEventFactsV1((1, 1, 3), ("DENSITY_DECIMAL_SHELL_V2_APPLIED",), 1, 3, None, None)
    failure = DensityFailureFactsV1("DensityRationalAuthorityExhausted", "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED", "cap=0 shell_probes=1")
    outputs = {ordered[0].spec_id: _result(ordered[0], named_refusals=((ordered[0].spec_id, "NO_BAND"),)),
               ordered[1].spec_id: _result(ordered[1], outcome=failure.outcome, failure=failure, shell_events=(shell,))}
    frontend, collector = FixedFrontend(outputs), DensityShellEventCollector()
    result = attach_density_specs_v1(compilation, context, source, frontend, raw, shell_event_collector=collector)
    assert frontend.calls == [value.spec_id for value in ordered]
    assert result.envelope_specs is None and result.failure.compilation is None
    assert result.failure.outcome is ReferenceOutcome.DENSITY_RATIONAL_AUTHORITY_EXHAUSTED
    assert len(result.ordered_work_articles) == 2
    assert result.band_diagnostics == result.noise_diagnostics == result.rays_diagnostics == ()
    assert result.event_diagnostics == collector.diagnostics()
    assert result.failure.diagnostics[1:] == collector.diagnostics()
    assert len(collector.diagnostics()) == 3  # два canon uses и один accepted shell use


def test_first_failure_prevents_second_evaluation_and_internal_exception_stays_raised(inputs):
    compilation, context, source, raw = inputs
    ordered = sorted(raw, key=lambda value: value.spec_id)
    known = DensityFailureFactsV1("DirectionBindingCertificateUnproven", "REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE", "direction binding certificate is not proven")
    frontend = FixedFrontend({ordered[0].spec_id: _result(ordered[0], outcome=known.outcome, failure=known)})
    result = attach_density_specs_v1(compilation, context, source, frontend, raw)
    assert result.failure is not None and frontend.calls == [ordered[0].spec_id]
    internal = RuntimeError("internal profile gap")
    frontend = FixedFrontend({ordered[0].spec_id: internal})
    with pytest.raises(RuntimeError, match="internal profile gap"):
        attach_density_specs_v1(compilation, context, source, frontend, raw)
    assert frontend.calls == [ordered[0].spec_id]


def _certificate():
    lower = DecimalIntervalFactsV1(DecimalValueFactsV1(-1, 0), DecimalValueFactsV1(-1, 0), "CLOSED", "CLOSED", DecimalValueFactsV1(0, 0))
    upper = replace(lower, lower=DecimalValueFactsV1(1, 0), upper=DecimalValueFactsV1(1, 0))
    return BindingCertificateFactsV1((1, 0), lower, upper, DecimalValueFactsV1(2, 0), (), "SOURCE_DIRECTION_IRRATIONAL")


def test_certificate_assembly_keeps_ids_and_late_zip_rejects_short_result(inputs):
    compilation, context, source, raw = inputs
    count = raw[0].selection.source_hidden_count
    outputs = {value.spec_id: _result(value) for value in raw}
    outputs[raw[0].spec_id] = _result(raw[0], binding_reasons=("SOURCE_DIRECTION_IRRATIONAL",)*count,
                                      binding_certificates=(_certificate(),)*count)
    result = attach_density_specs_v1(compilation, context, source, FixedFrontend(outputs), raw)
    old = next(spec for spec in compilation.envelope_specs if spec.envelope_spec_id.value == raw[0].spec_id)
    changed = next(spec for spec in result.envelope_specs if spec.envelope_spec_id == old.envelope_spec_id)
    assert {support.hidden_support_id for support in changed.hidden_supports} == {support.hidden_support_id for support in old.hidden_supports}
    assert all(support.direction_binding.binding_reason.value == "SOURCE_DIRECTION_IRRATIONAL" for support in changed.hidden_supports)
    outputs[raw[0].spec_id] = replace(outputs[raw[0].spec_id], binding_certificates=(_certificate(),)*(count-1))
    with pytest.raises(ValueError, match="zip"):
        attach_density_specs_v1(compilation, context, source, FixedFrontend(outputs), raw)


def test_authority_assembly_and_inconsistent_no_binding_authority_is_internal(inputs):
    compilation, context, source, raw = inputs
    value = raw[0]
    count = value.selection.source_hidden_count
    vectors = tuple((ordinal+1, 1) for ordinal in range(count))
    authority = CanonicalFanAuthorityFactsV1("CANONICAL_FAN_RAYS_ON_CANONICAL_ANGLE_V1", "CANONICAL_REFLEX_EXCESS_PI_OVER_2",
        Fraction(1, 2), count, value.selection.q, vectors, vectors, ())
    outputs = {item.spec_id: _result(item) for item in raw}
    outputs[value.spec_id] = _result(value, authority=authority, binding_reasons=("SOURCE_DIRECTION_IRRATIONAL",)*count)
    result = attach_density_specs_v1(compilation, context, source, FixedFrontend(outputs), raw)
    changed = next(spec for spec in result.envelope_specs if spec.envelope_spec_id.value == value.spec_id)
    assert changed.direction_fan_authority.bound_primitive_integer_vectors == vectors
    assert all(support.direction_fan_authority_id == changed.direction_fan_authority.authority_id for support in changed.hidden_supports)
    outputs[value.spec_id] = replace(outputs[value.spec_id], binding_reasons=(None,)*count)
    with pytest.raises(ValueError, match="No-binding"):
        attach_density_specs_v1(compilation, context, source, FixedFrontend(outputs), raw)


def test_lifted_identity_assembly_and_formatted_noise_subturn_frozenset_carriers(inputs):
    compilation, context, source, raw = inputs
    value = raw[0]
    old_count = value.selection.source_hidden_count
    count = old_count + 1
    vectors = tuple((ordinal+1, 1) for ordinal in range(count))
    relation = "CANONICAL_REFLEX_EXCESS_PI_OVER_2"
    authority = CanonicalFanAuthorityFactsV1("CANONICAL_FAN_RAYS_ON_CANONICAL_ANGLE_V1", relation,
        Fraction(1, 2), count, value.selection.q, vectors, vectors, ())
    lift = CountLiftFactsV1("EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_CANONICAL_EXACT_LIMIT_V1",
        old_count, count, value.selection.q, "ZERO", Fraction(0), count-1, ())
    noise = NoiseRecordFactsV1("EVALUATION_BINDING_NOISE_ON_CANONICAL_ANGLE_V1", "CANONICAL_COUNT_LIFT",
        relation, Fraction(1, 2), old_count, count, value.selection.q, "ZERO", Fraction(0), Fraction(0),
        Fraction(0), Fraction(1, 400), ())
    subturn = CanonicalSubturnAuthorityFactsV1("SUBTURN_GUARANTEE_ON_CANONICAL_SUPPORTS_V1", relation,
        Fraction(1, 2), count, value.selection.q, Fraction(1, 2*(count+1)), Fraction(0), ())
    outputs = {item.spec_id: _result(item) for item in raw}
    outputs[value.spec_id] = _result(value, effective_hidden_count=count, authority=authority, count_lift=lift,
        binding_reasons=(None,)*count, binding_certificates=(None,)*count, noise_record=noise,
        canonical_subturn_authority=subturn)
    result = attach_density_specs_v1(compilation, context, source, FixedFrontend(outputs), raw)
    changed = next(spec for spec in result.envelope_specs if spec.envelope_spec_id.value == value.spec_id)
    assert changed.resolved_hidden_edge_count == count and len(changed.hidden_supports) == count
    assert changed.evaluation_subturn_count_lift.source_hidden_edge_count == old_count
    assert changed.evaluation_subturn_count_lift.source_selection_certificate_id == changed.selection_certificate_id
    assert len(result.evaluation_binding_noise_records) == len(result.canonical_subturn_fan_authorities) == 1
    assert next(iter(result.evaluation_binding_noise_records)).envelope_spec_id == changed.envelope_spec_id
    assert next(iter(result.canonical_subturn_fan_authorities)).selection_certificate_id == changed.selection_certificate_id


@pytest.mark.parametrize("kind", ("bool_count", "zero_count", "count_without_lift", "short_reasons", "unexpected_certificates"))
def test_malformed_success_is_rejected_before_unchanged_no_binding_return(inputs, kind):
    compilation, context, source, raw = inputs
    value = min(raw, key=lambda item: item.selection.source_hidden_count)
    count = value.selection.source_hidden_count
    changes = {"bool_count": {"effective_hidden_count": True}, "zero_count": {"effective_hidden_count": 0},
        "count_without_lift": {"effective_hidden_count": count+1, "binding_reasons": (None,)*(count+1),
                               "binding_certificates": (None,)*(count+1)},
        "short_reasons": {"binding_reasons": ()}, "unexpected_certificates": {"binding_certificates": (_certificate(),)*count}}
    outputs = {item.spec_id: _result(item) for item in raw}
    outputs[value.spec_id] = replace(outputs[value.spec_id], **changes[kind])
    with pytest.raises(ValueError):
        attach_density_specs_v1(compilation, context, source, FixedFrontend(outputs), raw)


def test_malformed_lift_and_authority_dimensions_raise(inputs):
    compilation, context, source, raw = inputs
    value = raw[0]
    count = value.selection.source_hidden_count
    lift = CountLiftFactsV1("EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_CANONICAL_EXACT_LIMIT_V1",
        count+1, count, value.selection.q, "ZERO", Fraction(0), count-1, ())
    vectors = tuple((ordinal+1, 1) for ordinal in range(count))
    authority = CanonicalFanAuthorityFactsV1("CANONICAL_FAN_RAYS_ON_CANONICAL_ANGLE_V1",
        "CANONICAL_REFLEX_EXCESS_PI_OVER_2", Fraction(1, 2), count, value.selection.q, vectors, vectors, ())
    outputs = {item.spec_id: _result(item) for item in raw}
    changes = ({"count_lift": lift}, {"authority": replace(authority, max_subturn_q=2)},
               {"authority": replace(authority, bound_primitive_integer_vectors=())},
               {"authority": replace(authority, ray_rotation_pairs=())})
    for change in changes:
        outputs[value.spec_id] = replace(_result(value, binding_reasons=("SOURCE_DIRECTION_IRRATIONAL",)*count), **change)
        with pytest.raises(ValueError):
            attach_density_specs_v1(compilation, context, source, FixedFrontend(outputs), raw)


def test_precomputed_batch_complete_and_first_failure_prefix_reuses_attachment(inputs):
    compilation, context, source, raw = inputs
    ordered = sorted(raw, key=lambda item: item.spec_id)
    pairs = tuple((item.spec_id, _result(item)) for item in ordered)
    result = attach_density_results_v1(compilation, context, source, FixedFrontend.canon_ids, raw, pairs)
    assert result.envelope_specs == compilation.envelope_specs and len(result.ordered_work_articles) == len(raw)
    failure = DensityFailureFactsV1("DensityRationalAuthorityExhausted", "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED", "cap=0")
    failed = _result(ordered[0], outcome=failure.outcome, failure=failure)
    result = attach_density_results_v1(compilation, context, source, FixedFrontend.canon_ids, raw,
                                      ((ordered[0].spec_id, failed),))
    assert result.envelope_specs is None and result.failure is not None and len(result.ordered_work_articles) == 1


def test_precomputed_batch_rejects_missing_extra_wrong_duplicate_and_postfailure_results(inputs):
    compilation, context, source, raw = inputs
    pairs = tuple((item.spec_id, _result(item)) for item in sorted(raw, key=lambda item: item.spec_id))
    failure = DensityFailureFactsV1("DensityRationalAuthorityExhausted", "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED", "cap=0")
    failed = replace(pairs[0][1], outcome=failure.outcome, failure=failure)
    invalid = (pairs[:1], (*pairs, pairs[-1]), (("unknown", pairs[0][1]), *pairs[1:]),
               (pairs[0], pairs[0]), ((pairs[0][0], failed), *pairs[1:]))
    for values in invalid:
        with pytest.raises(ValueError):
            attach_density_results_v1(compilation, context, source, FixedFrontend.canon_ids, raw, values)


def test_success_count_ceiling_false_lift_and_failed_success_payload_raise(inputs):
    compilation, context, source, raw = inputs
    value = min(raw, key=lambda item: item.spec_id)
    source_count, q = value.selection.source_hidden_count, value.selection.q
    lift = CountLiftFactsV1("EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_CANONICAL_EXACT_LIMIT_V1",
        source_count, source_count, q, "ZERO", Fraction(0), source_count-1, ())
    failure = DensityFailureFactsV1("DensityRationalAuthorityExhausted", "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED", "cap=0")
    failed = _result(value, outcome=failure.outcome, failure=failure)
    invalid = (_result(value, effective_hidden_count=q, binding_reasons=(None,)*q, binding_certificates=(None,)*q),
        _result(value, count_lift=lift), _result(value, effective_hidden_count=source_count+1,
            binding_reasons=(None,)*(source_count+1), binding_certificates=(None,)*(source_count+1),
            count_lift=replace(lift, effective_hidden_edge_count=source_count+1)),
        replace(failed, outcome="REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE"),
        replace(failed, effective_hidden_count=source_count),
        replace(failed, named_refusals=((value.spec_id, "discarded band"),)),
        replace(failed, law_refusals=CanonicalLawRefusalFactsV1("discarded noise", None)),
        replace(failed, law_refusals=CanonicalLawRefusalFactsV1(None, "discarded rays")))
    for result in invalid:
        outputs = {item.spec_id: _result(item) for item in raw}
        outputs[value.spec_id] = result
        with pytest.raises(ValueError):
            attach_density_specs_v1(compilation, context, source, FixedFrontend(outputs), raw)
