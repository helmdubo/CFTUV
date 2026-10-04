"""Символьный бэкенд эталона: родная арифметика РАВНА sympy по значению, режимы не меняют ответ.

Шаг 2 плана SYMPY-OFF-HOT-PATH. Что проверяется и чем:

1. ЗНАЧЕНИЯ. Сумма корней по квадратным классам (`RadicalSumV1`, без факторизации) складывает,
   умножает, делит и решает знак так же, как независимая каноническая форма `SqrtSumV1`
   (бесквадратная, с разложением радикандов) — ОНА здесь оракул, а не sympy: у sympy вид выражения
   зависит от истории вычисления, у `SqrtSumV1` — нет. Корпус случайный (фиксированные зёрна) и
   состязательный: нулевые суммы, сопряжённые, `sqrt(8)` против `2*sqrt(2)`, радиканды с квадратом
   простого больше 2**15, разность двух близких корней.
2. ТЕКСТ. `srepr` одночленной величины, собранный без sympy, равен `srepr(factor(cancel(x)))` по
   сетке знаков, знаменателей и радикандов; многочленная идёт через sympy и помечена.
3. ВЫХОД ЗА ПОЛЕ. Тригонометрия, вложенные радикалы и корень из суммы — `OutsideNativeField` с
   кодом, а не догадка; `exact_sign` уступает sympy и считает уступку.
4. РЕЖИМЫ. Умолчание `SYMPY`; контекст возвращает режим; `SHADOW` ловит ПОДМЕНЁННЫЙ неверный ответ
   родной стороны (отрицательный контроль: сверка, не умеющая краснеть, — не сверка), `RECORD`
   копит расхождения, `RAISE` бросает `EXACT_SYMBOLIC_BACKEND_DISAGREEMENT`.
5. СКВОЗНО. Те же резолюции alpha и покрытие полевой подготовки под `SYMPY`, `SHADOW` и
   `NATIVE_EXACT` совпадают ПОБИТОВО (в том числе тексты `ExactScalar`, входящие в дайджесты), а
   счётчики доказывают, что новый путь действительно исполнялся.
"""

from __future__ import annotations

import os
import pickle
import random
from dataclasses import replace
from decimal import Decimal
from fractions import Fraction

import pytest
import sympy as sp

from cftuv_envelope.numeric import LocalLengthV1
from cftuv_envelope.reference import compile_reference_envelopes, native_exact as nx
from cftuv_envelope.reference import planar_types, symbolic_backend as sb
from cftuv_envelope.reference.boundary import (
    ContactCandidatesMemoV1,
    _contact_candidates,
    build_domain_geometry,
    resolve_component_alphas,
)
from cftuv_envelope.reference.common import GeometryContext
from cftuv_envelope.reference.native_exact import RadicalSumV1 as R
from cftuv_envelope.reference.planar_types import (
    ExactScalar,
    exact_quadratic_value,
    exact_sign,
)
from cftuv_envelope.reference.validation import validate_compilation_geometry_payload
from cftuv_envelope.wavefront import ConveyorOutcome, conveyor_coverage

from reference_factories import straight_snapshot
from test_contact_candidates_memo import (
    ALPHAS,
    CONCAVE,
    HOLE_RING,
    _answer,
    _field_preparation,
)

RADICANDS = (
    1, 2, 3, 5, 6, 7, 8, 12, 18, 50, 72, 140, 72607457, 233480149689470498,
    1000003 ** 2 * 3, 2 * 10 ** 9 + 11,
)


@pytest.fixture(autouse=True)
def _restore_backend():
    """Режим, политика и счётчики не переживают тест."""

    mode, policy = sb.backend_mode(), sb.disagreement_policy()
    sb.reset_backend_counts()
    planar_types.TEXT_DIFFERENCES.clear()
    yield
    sb.set_backend_mode(mode)
    sb.set_disagreement_policy(policy)
    sb.reset_backend_counts()


def _fraction(rng, big=False):
    limit = 10 ** 9 if big else 30
    return Fraction(rng.randint(-limit, limit), rng.randint(1, limit))


def _random_expression(rng, max_terms=3):
    total = sp.Integer(0)
    for _ in range(rng.randint(1, max_terms)):
        coefficient = _fraction(rng)
        radicand = rng.choice(RADICANDS) * rng.choice((1, 1, 1, 4, 9, 25))
        total += sp.Rational(coefficient.numerator, coefficient.denominator) * sp.sqrt(
            sp.Integer(radicand)
        )
    return total


def _same_as_oracle(native: R, expression) -> bool:
    return exact_quadratic_value(nx.to_sympy(native)).terms == exact_quadratic_value(
        expression
    ).terms


# --------------------------------------------------------------------------
# 1. Значения
# --------------------------------------------------------------------------


@pytest.mark.skipif(
    bool(os.environ.get("CFTUV_SYMBOLIC_BACKEND")),
    reason="the suite is being run under a chosen backend",
)
def test_the_default_backend_is_sympy():
    assert sb.backend_mode() is sb.SymbolicBackendV1.SYMPY
    assert sb.disagreement_policy() is sb.DisagreementPolicyV1.RAISE


def test_arithmetic_matches_the_canonical_oracle_on_a_random_corpus():
    rng = random.Random(20261004)
    for _ in range(600):
        left, right = _random_expression(rng), _random_expression(rng)
        a, b = nx.from_sympy(left), nx.from_sympy(right)
        assert _same_as_oracle(a, left)
        assert _same_as_oracle(a + b, left + right)
        assert _same_as_oracle(a - b, left - right)
        assert _same_as_oracle(a * b, left * right)
        if not b.is_zero:
            assert _same_as_oracle(a / b, left / right)
        oracle_sign = exact_quadratic_value(left).sign()
        try:
            assert a.signum() == oracle_sign
        except nx.NativeSignUndecided:
            pass


def test_zero_is_the_emptiness_of_the_terms_not_smallness():
    root = R.sqrt_of_rational(2)
    assert (root - root).is_zero
    assert (R.sqrt_of_rational(8) - root.scaled(2)).is_zero
    assert (R.sqrt_of_rational(8) - root.scaled(2)).signum() == 0
    # (1 + sqrt 2)(sqrt 2 - 1) = 1: сопряжённые.
    one_plus = R.rational(1) + root
    assert (one_plus * (root - R.rational(1))).as_rational() == 1
    assert (one_plus * one_plus.inverse()).as_rational() == 1
    # Разные представители одного класса складываются в один член.
    mixed = R.sqrt_of_rational(2) + R.sqrt_of_rational(8) + R.sqrt_of_rational(Fraction(1, 2))
    assert len(mixed.terms) == 1
    assert (mixed - root.scaled(Fraction(7, 2))).is_zero
    # Ноль через три класса: (sqrt2 + sqrt3 + sqrt5)(...)=0 не нужен; достаточно 0 = x - x.
    triple = root + R.sqrt_of_rational(3) + R.sqrt_of_rational(5)
    assert (triple - triple).is_zero and (triple - triple).signum() == 0


def test_equality_is_by_value_and_values_are_not_hashable():
    assert R.sqrt_of_rational(8) == R.sqrt_of_rational(2).scaled(2)
    assert R.sqrt_of_rational(2) != R.sqrt_of_rational(3)
    assert R.rational(3) == 3
    with pytest.raises(TypeError):
        hash(R.sqrt_of_rational(2))


def test_large_radicands_are_exact_without_factoring_anything():
    exact_before = nx.__dict__.get("_CLASS_RATIO")
    assert exact_before is not None
    big = 233480149689470498
    other = 933664892944496557
    one = R.sqrt_of_rational(big)
    two = R.sqrt_of_rational(other)
    assert (one * one).as_rational() == big
    assert (one * two - two * one).is_zero
    # Квадрат простого больше 2**15 под корнем сворачивается тем же классом.
    prime = 1000003
    assert (R.sqrt_of_rational(prime ** 2 * 3) - R.sqrt_of_rational(3).scaled(prime)).is_zero
    # Две близкие величины: sqrt(n^2 + 1) - n > 0 решается точно (два члена).
    n = 10 ** 15
    closeness = R.sqrt_of_rational(n * n + 1) - R.rational(n)
    assert closeness.signum() == 1
    assert (R.rational(n) - R.sqrt_of_rational(n * n + 1)).signum() == -1


def test_three_class_sign_uses_the_enclosure_and_undecided_is_a_named_outcome(monkeypatch):
    value = R.sqrt_of_rational(2) + R.sqrt_of_rational(3) - R.sqrt_of_rational(5)
    assert value.signum() == 1  # 1.414 + 1.732 - 2.236 > 0
    monkeypatch.setattr(nx, "_SIGN_BITS", (1,))
    with pytest.raises(nx.NativeSignUndecided) as caught:
        value.signum()
    assert caught.value.code == nx.EXACT_NATIVE_SIGN_UNDECIDED
    # exact_sign уступает sympy и отвечает так же.
    assert exact_sign(value) == 1
    assert sb.BACKEND_COUNTS["exact_sign.sign_undecided"] >= 1


def test_inverse_beyond_three_classes_is_outside_the_field():
    four = R.sqrt_of_rational(2) + R.sqrt_of_rational(3) + R.sqrt_of_rational(5) + R.sqrt_of_rational(7)
    with pytest.raises(nx.OutsideNativeField) as caught:
        four.inverse()
    assert caught.value.code == nx.EXACT_NATIVE_OUTSIDE_FIELD


@pytest.mark.parametrize(
    "expression",
    (
        sp.sin(1),
        sp.cos(sp.pi / 7),
        sp.sqrt(1 + sp.sqrt(2)),
        sp.sqrt(10 - 2 * sp.sqrt(5)),
        sp.pi,
        sp.Float("0.5"),
        sp.Symbol("x"),
    ),
)
def test_values_outside_the_field_are_named_not_guessed(expression):
    with pytest.raises(nx.OutsideNativeField) as caught:
        nx.from_sympy(expression)
    assert caught.value.code == "EXACT_NATIVE_OUTSIDE_FIELD"


def test_exact_sign_hands_outside_the_field_values_to_sympy_in_every_mode():
    nested = sp.sqrt(10 - 2 * sp.sqrt(5)) - 2  # sqrt(10-2*sqrt5) = 2.3511...
    for mode in sb.SymbolicBackendV1:
        with sb.symbolic_backend(mode):
            assert exact_sign(nested) == 1
            assert exact_sign(sp.cos(sp.pi / 3) - sp.Rational(1, 2)) == 0
    assert sb.BACKEND_COUNTS["exact_sign.outside_field"] >= 2


# --------------------------------------------------------------------------
# 2. Текст `srepr`
# --------------------------------------------------------------------------


def test_one_term_text_equals_the_sympy_text_over_a_grid():
    coefficients = [
        Fraction(n, d)
        for n in (1, 2, 3, 7, 60, 1180581228544)
        for d in (1, 2, 5, 7780211757696365)
    ]
    coefficients += [-value for value in coefficients]
    checked = 0
    for coefficient in coefficients:
        for radicand in RADICANDS[1:]:
            expression = sp.Rational(coefficient.numerator, coefficient.denominator) * sp.sqrt(radicand)
            legacy = sp.srepr(sp.factor(sp.cancel(expression)))
            text, emulated = nx.native_text(nx.from_sympy(expression))
            assert text == legacy, (coefficient, radicand)
            assert emulated
            checked += 1
    assert checked == len(coefficients) * (len(RADICANDS) - 1)


def test_text_round_trips_through_the_fast_parser():
    rng = random.Random(7)
    for _ in range(300):
        expression = _random_expression(rng, max_terms=1)
        text = sp.srepr(sp.factor(sp.cancel(expression)))
        assert _same_as_oracle(nx.native_of_text(text), expression)
    for text in ("Integer(-5)", "Rational(-3, 7)", "Pow(Integer(5), Rational(1, 2))"):
        assert nx.native_of_text(text) == nx.from_sympy(sp.sympify(text))


def test_a_sum_goes_through_sympy_and_says_so():
    value = R.rational(1) + R.sqrt_of_rational(2)
    text, emulated = nx.native_text(value)
    assert not emulated
    assert text == sp.srepr(sp.factor(sp.cancel(1 + sp.sqrt(2))))
    assert ExactScalar.from_value(value).expression == text
    assert sb.BACKEND_COUNTS["exact_scalar_text.native_via_sympy"] == 1


# --------------------------------------------------------------------------
# 3. Режимы и теневая сверка
# --------------------------------------------------------------------------


def test_the_context_manager_restores_the_mode_even_after_an_error():
    before = sb.backend_mode()
    with pytest.raises(RuntimeError):
        with sb.symbolic_backend(sb.SymbolicBackendV1.SHADOW, sb.DisagreementPolicyV1.RECORD):
            assert sb.backend_mode() is sb.SymbolicBackendV1.SHADOW
            raise RuntimeError("boom")
    assert sb.backend_mode() is before
    assert sb.disagreement_policy() is sb.DisagreementPolicyV1.RAISE


def test_shadow_catches_a_lying_native_sign_and_the_policy_chooses_raise_or_record(monkeypatch):
    expression = sp.sqrt(2) - 1
    real_sign = R.signum
    monkeypatch.setattr(R, "signum", lambda self: -real_sign(self))
    with sb.symbolic_backend(sb.SymbolicBackendV1.SHADOW):
        with pytest.raises(sb.ExactSymbolicBackendDisagreement) as caught:
            exact_sign(expression)
    assert caught.value.code == sb.EXACT_SYMBOLIC_BACKEND_DISAGREEMENT
    assert "exact_sign" in str(caught.value)
    sb.reset_backend_counts()
    with sb.symbolic_backend(sb.SymbolicBackendV1.SHADOW, sb.DisagreementPolicyV1.RECORD):
        assert exact_sign(expression) == 1  # ответ sympy остаётся ответом
    assert sb.BACKEND_COUNTS["exact_sign.disagreement"] == 1
    assert sb.DISAGREEMENTS and sb.DISAGREEMENTS[0][0] == "exact_sign"


def test_shadow_catches_a_lying_native_dot_product(monkeypatch):
    context, domain = _geometry_of(*_concave_snapshot())
    metric = context.metric
    source = _first_source(context)
    real = type(metric).dot_g_native
    monkeypatch.setattr(
        type(metric), "dot_g_native", lambda self, left, right: real(self, left, right) + R.rational(1)
    )
    with sb.symbolic_backend(sb.SymbolicBackendV1.SHADOW):
        with pytest.raises(sb.ExactSymbolicBackendDisagreement):
            metric.dot_g(source.tangent, source.tangent)


def test_shadow_catches_lying_contacts(monkeypatch):
    context, domain = _geometry_of(*_hole_ring_snapshot())
    source = _first_source(context)
    boundary = domain.blocking_segments[0]
    from cftuv_envelope.reference import boundary as boundary_module

    real = boundary_module.contact_candidates_native

    def wrong(ctx, src, bnd):
        return tuple((alpha + R.rational(1), station, point) for alpha, station, point in real(ctx, src, bnd))

    monkeypatch.setattr(boundary_module, "contact_candidates_native", wrong)
    with sb.symbolic_backend(sb.SymbolicBackendV1.SHADOW):
        for candidate in domain.blocking_segments:
            if _contact_candidates_sympy_of(context, source, candidate):
                with pytest.raises(sb.ExactSymbolicBackendDisagreement):
                    _contact_candidates(context, source, candidate)
                return
    raise AssertionError("no boundary segment produced a contact: the control proves nothing")


# --------------------------------------------------------------------------
# 5. Сквозные ответы
# --------------------------------------------------------------------------


def _hole_ring_snapshot():
    return straight_snapshot(
        faces=HOLE_RING,
        source_routes=(
            {"name": "bottom", "points": ((0.0, 0.0), (4.0, 0.0), (6.0, 0.0), (10.0, 0.0))},
            {"name": "top", "points": ((10.0, 10.0), (6.0, 10.0), (4.0, 10.0), (0.0, 10.0))},
        ),
        alpha="4",
    )


def _concave_snapshot():
    return straight_snapshot(
        faces=CONCAVE,
        source_routes=({"name": "source", "points": ((0.0, 0.0), (10.0, 0.0))},),
        alpha="4",
    )


def _geometry_of(snapshot, request):
    compiled = compile_reference_envelopes(snapshot, request)
    assert compiled.compilation is not None
    frame, _ = validate_compilation_geometry_payload(compiled.compilation)
    context = GeometryContext.build(compiled.compilation, frame)
    return context, build_domain_geometry(context)


def _first_source(context):
    from test_contact_candidates_memo import _sources

    return next(_sources(context))


def _contact_candidates_sympy_of(context, source, boundary):
    from cftuv_envelope.reference.boundary import _contact_candidates_sympy

    return _contact_candidates_sympy(context, source, boundary)


def test_alpha_resolutions_are_bitwise_the_same_in_every_mode():
    for build in (_hole_ring_snapshot, _concave_snapshot):
        context, domain = _geometry_of(*build())
        expected = {}
        for alpha in ALPHAS:
            value = LocalLengthV1(Decimal(alpha))
            expected[alpha] = resolve_component_alphas(context, value, domain)
        for mode in (sb.SymbolicBackendV1.SHADOW, sb.SymbolicBackendV1.NATIVE_EXACT):
            sb.reset_backend_counts()
            with sb.symbolic_backend(mode):
                for alpha in ALPHAS:
                    value = LocalLengthV1(Decimal(alpha))
                    got = resolve_component_alphas(context, value, domain, ContactCandidatesMemoV1())
                    assert got == expected[alpha], (build.__name__, mode.value, alpha)
            assert sb.BACKEND_COUNTS.get("contact_candidates.disagreement", 0) == 0
            key = "shadow_checked" if mode is sb.SymbolicBackendV1.SHADOW else "native"
            assert sb.BACKEND_COUNTS[f"contact_candidates.{key}"] > 0, (build.__name__, mode.value)


def test_the_field_coverage_is_bitwise_the_same_in_every_mode_and_the_new_path_ran():
    prepared = _field_preparation()
    assert prepared.outcome is ConveyorOutcome.EXACT, prepared.detail
    answers = {}
    for mode in sb.SymbolicBackendV1:
        fresh = replace(prepared, contact_memo=ContactCandidatesMemoV1())
        sb.reset_backend_counts()
        with sb.symbolic_backend(mode, sb.DisagreementPolicyV1.RECORD):
            answers[mode] = [_answer(conveyor_coverage(fresh, alpha)) for alpha in ("0.25", "0.5", "2")]
        assert not sb.DISAGREEMENTS, (mode.value, sb.DISAGREEMENTS[:3])
        counts = dict(sb.BACKEND_COUNTS)
        if mode is sb.SymbolicBackendV1.SHADOW:
            assert counts["dot_g.shadow_checked"] > 0
            assert counts["contact_candidates.shadow_checked"] > 0
            assert counts["exact_sign.shadow_checked"] > 0
        if mode is sb.SymbolicBackendV1.NATIVE_EXACT:
            assert counts["contact_candidates.native"] > 0
        if mode is sb.SymbolicBackendV1.SYMPY:
            assert not counts
    assert answers[sb.SymbolicBackendV1.SHADOW] == answers[sb.SymbolicBackendV1.SYMPY]
    assert answers[sb.SymbolicBackendV1.NATIVE_EXACT] == answers[sb.SymbolicBackendV1.SYMPY]


def test_native_contacts_travel_with_the_preparation_and_answer_in_any_mode():
    prepared = _field_preparation()
    fresh = replace(prepared, contact_memo=ContactCandidatesMemoV1())
    with sb.symbolic_backend(sb.SymbolicBackendV1.NATIVE_EXACT):
        native_answer = _answer(conveyor_coverage(fresh, "0.25"))
    kinds = {
        type(alpha).__name__
        for key, contacts in fresh.contact_memo.entries.items()
        if key[0] == "contacts"
        for alpha, _station, _point in contacts
    }
    assert "RadicalSumV1" in kinds, kinds
    clone = pickle.loads(pickle.dumps(fresh, protocol=pickle.HIGHEST_PROTOCOL))
    # Копия читает родные контакты из памяти (alpha другая, контакты те же), в каком бы режиме ни шла.
    assert _answer(conveyor_coverage(clone, "0.25")) == native_answer
    assert _answer(conveyor_coverage(clone, "0.5")) == _answer(conveyor_coverage(prepared, "0.5"))
