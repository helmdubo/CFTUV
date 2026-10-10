"""`EXACT_SCALAR_TEXT_CANON_UNSUPPORTED` — именованный исход домена, а не исключение домена.

У многочленной точной величины нет канонической строки V2 (`ExactScalar.from_value(<RadicalSumV1 из двух классов>)`): ядро бросает
`ExactScalarTextCanonUnsupported` и прежней формы `sympy` в ответ не отдаёт. Раньше имя жило только в тексте исключения, а хост показывал общее
`PRODUCTION_DOMAIN_RAISED`. Теперь материализация и шаг ширины возвращают отказ под ЭТИМ именем (`MaterializationOutcome`), настоящим исключением,
брошенным настоящим кодом.
"""

from __future__ import annotations

from fractions import Fraction

import pytest

from cftuv_envelope.materialize import domain, step as step_module
from cftuv_envelope.materialize.admit import MaterializationOutcome
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.step import STEP_COUNTERS, step_domain
from cftuv_envelope.reference import native_exact as nx
from cftuv_envelope.reference.planar_types import ExactScalar

from test_materialize_domain import UV, _case

NAME = "EXACT_SCALAR_TEXT_CANON_UNSUPPORTED"


def _two_classes() -> nx.RadicalSumV1:
    """`sqrt(2) + sqrt(3)`: два класса квадратов, канонической строки V2 нет."""

    return nx.RadicalSumV1.sqrt_of_rational(Fraction(2)) + nx.RadicalSumV1.sqrt_of_rational(Fraction(3))


def _raises_for_real(*_arguments, **_keywords):
    """Брошено настоящим кодом: путь `from_value` для родной величины (`RadicalSumV1`) идёт в `canonical_text`."""

    ExactScalar.from_value(_two_classes())
    raise AssertionError("a multi-term RadicalSumV1 must have no canonical text")


def test_the_trigger_is_real_and_the_name_lives_in_the_exception_code_and_text():
    with pytest.raises(nx.ExactScalarTextCanonUnsupported) as caught:
        ExactScalar.from_value(_two_classes())
    assert caught.value.code == NAME
    assert str(caught.value).startswith(NAME + ":")
    assert isinstance(caught.value, ValueError)  # прежний тип остаётся: потребители, ловившие ValueError, не сломаны


def test_the_outcome_is_a_member_of_the_kernel_enum_with_the_name_of_the_exception():
    assert MaterializationOutcome(NAME) is MaterializationOutcome.EXACT_SCALAR_TEXT_CANON_UNSUPPORTED
    assert nx.EXACT_SCALAR_TEXT_CANON_UNSUPPORTED == NAME == MaterializationOutcome.EXACT_SCALAR_TEXT_CANON_UNSUPPORTED.value


def test_materialize_domain_names_the_refusal_instead_of_letting_the_exception_out(monkeypatch):
    prepared, coverage, request = _case("weighted")
    import dataclasses

    request = dataclasses.replace(request, uv_policy_id=UV)
    monkeypatch.setattr(domain, "_build", _raises_for_real)
    result = materialize_domain(prepared, coverage, request=request)
    assert result.outcome is MaterializationOutcome.EXACT_SCALAR_TEXT_CANON_UNSUPPORTED
    assert result.batch is None and not result.is_materialized
    assert result.detail.startswith(NAME + ":") and "square classes" in result.detail
    assert result.content_digest == ""


def test_the_step_of_the_width_names_the_refusal_and_counts_it_as_a_refused_step(monkeypatch):
    prepared, _coverage, request = _case("weighted")
    import dataclasses

    request = dataclasses.replace(request, uv_policy_id=UV)

    def raising_inner(*_arguments, **_keywords):
        ExactScalar.from_value(_two_classes())

    monkeypatch.setattr(step_module, "_step_domain", raising_inner)
    key = "INTERVAL_FALLBACK_" + step_module.FALLBACK_REFUSED
    before = STEP_COUNTERS[key]
    stepped = step_domain(
        prepared,
        "0.25",
        request=request,
        near_planar_lift_law=None,
        decal_topology_law=None,
    )
    assert stepped.result.outcome is MaterializationOutcome.EXACT_SCALAR_TEXT_CANON_UNSUPPORTED
    assert stepped.result.batch is None
    assert stepped.path == "FALLBACK:" + step_module.FALLBACK_REFUSED
    assert STEP_COUNTERS[key] == before + 1


def test_every_other_exception_still_leaves_the_step_untouched(monkeypatch):
    prepared, _coverage, _request = _case("weighted")

    def other(*_arguments, **_keywords):
        raise RuntimeError("not a canon refusal")

    monkeypatch.setattr(step_module, "_step_domain", other)
    with pytest.raises(RuntimeError, match="not a canon refusal"):
        step_domain(prepared, "0.25", request=None, near_planar_lift_law=None, decal_topology_law=None)
