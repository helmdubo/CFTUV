"""Печать записей канонического угла сверяется с живым старым producer."""

from dataclasses import fields
from enum import Enum
from fractions import Fraction

import pytest

from cftuv_envelope.codec import canonical_json_bytes
from cftuv_envelope.contracts.metric import ExactRationalV1
from cftuv_envelope.ids import EnvelopeSpecId
from cftuv_envelope.numeric import ExactRatioV1
from cftuv_envelope.reference import density_oracle_v1 as facts, density_result_v1 as formatter
from cftuv_envelope.reference import contracts as ir


@pytest.mark.parametrize("numerator,denominator", ((1657, 5120), (3647, 5120), (1, 3), (1, 8)))
def test_shell_facts_replay_the_same_recorded_grid_diagnostics(numerator, denominator):
    from sympy import Rational
    from cftuv_envelope.reference.density_shell_events import DensityShellEventCollector
    from cftuv_envelope.reference.density_shell_v2 import density_shell_v2

    candidate = density_shell_v2(Rational(numerator, denominator))
    bundle, wire = candidate.outcomes, candidate.value.wire()
    rational = bundle.rational
    value = facts.DensityShellEventFactsV1(wire, tuple(code.value for code in bundle.codes),
        rational.numerator, rational.denominator, rational.grid_units, bundle.legacy_projected_min_width_units)
    old, new = DensityShellEventCollector(), DensityShellEventCollector()
    old.replay("spec", ((wire, bundle),))
    new.replay("spec", formatter.format_shell_events((value,)))
    assert canonical_json_bytes(old.diagnostics()) == canonical_json_bytes(new.diagnostics())


def test_failure_facts_preserve_compile_outcome_and_cannot_hide_internal_profile_error():
    from cftuv_envelope.reference.compile import _failure

    outcome = ir.ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE
    message = "Density A exact sign is not certified without generic factorization"
    value = facts.DensityFailureFactsV1("ReferenceGeometryError", outcome.value, message)
    assert formatter.format_density_failure(value) == _failure(outcome, message)
    with pytest.raises(formatter.DensityResultInvalid, match="internal failures"):
        formatter.format_density_failure(facts.DensityFailureFactsV1("InverseClassLimit", outcome.value, message))


def freeze_record(record, target):
    def convert(value):
        if isinstance(value, Enum):
            return value.value
        if isinstance(value, (ExactRatioV1, ExactRationalV1)):
            return Fraction(value.numerator, value.denominator)
        if isinstance(value, frozenset):
            return tuple(sorted(value))
        return value
    return target(**{item.name: convert(getattr(record, item.name)) for item in fields(target)})


def test_real_field_noise_records_preserve_every_typed_fact():
    from test_evaluation_binding_noise import _load, FIXTURE, compile_reference_envelopes

    compilation = compile_reference_envelopes(*_load(FIXTURE, "building_patch114", 4)).compilation
    assert compilation is not None and compilation.evaluation_binding_noise_records
    for record in compilation.evaluation_binding_noise_records:
        value = freeze_record(record, facts.NoiseRecordFactsV1)
        printed = formatter.format_noise_record(value, record.envelope_spec_id, record.selection_certificate_id)
        assert printed == record
        assert canonical_json_bytes(printed) == canonical_json_bytes(record)


def test_restored_canonical_subturn_authority_is_printed_without_recomputation():
    from test_canonical_angle_restoration import _density_compile, _DENSITY_VALUES, NEAR_RIGHT_ANGLE

    _, value_id, symbol, _ = next(row for row in _DENSITY_VALUES if row[0] == 2)
    _, result = _density_compile(NEAR_RIGHT_ANGLE, value_id, symbol)
    assert result.compilation is not None and result.compilation.canonical_subturn_fan_authorities
    for record in result.compilation.canonical_subturn_fan_authorities:
        value = freeze_record(record, facts.CanonicalSubturnAuthorityFactsV1)
        printed = formatter.format_canonical_subturn_authority(value, record.envelope_spec_id, record.selection_certificate_id)
        assert printed == record
        assert canonical_json_bytes(printed) == canonical_json_bytes(record)


def test_law_refusals_keep_enum_text_and_typed_outcomes():
    value = facts.CanonicalLawRefusalFactsV1("TURN_NOISE_EXCEEDS_DECLARED_BOUND", "CANONICAL_RAYS_IRRATIONAL_IN_CHART")
    rows = formatter.format_canonical_law_refusals(value, EnvelopeSpecId("spec"))
    assert tuple(row.outcome for row in rows) == (
        ir.ReferenceOutcome.EVALUATION_BINDING_NOISE_LAW_NOT_APPLIED,
        ir.ReferenceOutcome.CANONICAL_FAN_RAYS_LAW_NOT_APPLIED,
    )
    assert rows[0].message == "TURN_NOISE_EXCEEDS_DECLARED_BOUND: the count of a canonical angle at the exact subturn limit is decided by the sign of the evaluation binding noise"
    assert rows[1].message == "CANONICAL_RAYS_IRRATIONAL_IN_CHART: the rays of a canonical angle are bound by the adaptive atlas or the per-ray window on the evaluation geometry, not set by the rotation table"
    assert all(row.envelope_spec_id == "spec" and row.severity is ir.ReferenceDiagnosticSeverity.INFO for row in rows)
    assert formatter.format_canonical_law_refusals(facts.CanonicalLawRefusalFactsV1(), EnvelopeSpecId("spec")) == ()
    with pytest.raises(ValueError):
        formatter.format_canonical_law_refusals(facts.CanonicalLawRefusalFactsV1("FOREIGN_CODE"), EnvelopeSpecId("spec"))
