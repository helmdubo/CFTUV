"""A1-кандидат сохраняет старый допуск и передаёт явный числовой исход."""
from decimal import Decimal, localcontext

import pytest
import sympy as sp

from cftuv_envelope.reference import density_binding_values as values
from cftuv_envelope.reference import density_shell_adapter as adapter
from cftuv_envelope.reference import density_shell_events as events
from cftuv_envelope.reference.density_binding_types import DirectionBindingCertificateUnproven
from cftuv_envelope.reference.density_shell_v2 import DensityShellOutcomeCode


def test_rational_grid_change_records_exact_bundle_and_keeps_legacy_entry(monkeypatch):
    recorded = []
    monkeypatch.setattr(adapter, "record_density_shell_candidate", lambda metric, candidate: recorded.append(candidate))
    expression = sp.Rational(1657, 5120)
    legacy = values._density_decimal_envelope(expression)
    with localcontext() as context:
        context.prec = 3
        revised = adapter._density_decimal_envelope_v2(expression)
    assert legacy.lower < revised.lower == revised.upper < legacy.upper
    assert revised.lower.as_tuple() == Decimal("0.323632812500000000000000000").as_tuple()
    assert revised.absolute_error_bound.as_tuple() == Decimal("0E-27").as_tuple()
    assert len(recorded) == 1
    assert DensityShellOutcomeCode.NON_DYADIC_GRID_STRAW in recorded[0].outcomes.codes
    assert values._density_decimal_envelope(expression) == legacy


def test_nonquadratic_interval_success_keeps_legacy_without_applied_event(monkeypatch):
    recorded = []
    monkeypatch.setattr(adapter, "record_density_shell_candidate", lambda *args: recorded.append(args))
    expression = sp.sin(1)
    assert adapter._density_decimal_envelope_v2(expression) == values._density_decimal_envelope(expression)
    assert recorded == []


def test_legacy_refusal_precedes_candidate_reader(monkeypatch):
    def forbidden(_expression):
        raise AssertionError("V2 не читает отказанный старым допуском вход")

    monkeypatch.setattr(adapter, "density_shell_v2", forbidden)
    with pytest.raises(DirectionBindingCertificateUnproven):
        adapter._density_decimal_envelope_v2(sp.tan(1))


def test_legacy_certificate_windows_pass_metric_to_candidate_event_capture(monkeypatch):
    from cftuv_envelope.reference import direction_binding as binding
    from cftuv_envelope.reference.metric import ExactPlanarMetric
    from cftuv_envelope.reference.planar_types import ExactPlanarVector

    identity = ((sp.Integer(1), sp.Integer(0)), (sp.Integer(0), sp.Integer(1)))
    metric = ExactPlanarMetric(identity, identity, 1)
    vectors = tuple(ExactPlanarVector.from_values(*pair) for pair in ((1, 1), (1, 0), (0, 1), (2, 1), (1, 2)))
    before = binding._huber_density_certificate_in_window(metric, *vectors, 4)
    assert before is not None
    monkeypatch.setattr(binding, "_density_decimal_envelope", adapter._density_decimal_envelope_v2)
    with events.capture_density_shell_candidates(metric) as captured:
        after = binding._huber_density_certificate_in_window(metric, *vectors, 4)
    assert after == before
    assert len(captured) == 2
    assert {bundle.rational.numerator for bundle in captured.values()} == {1, 2}
    assert all(DensityShellOutcomeCode.APPLIED in bundle.codes for bundle in captured.values())
