"""Быстрый рациональный путь обязан быть побитово равен символьному.

`exact_sign`, `_canonical_expr` и `exact_normalize` пропускают
`sp.factor(sp.cancel(...))`, когда значение рационально. Это законно только
потому, что SymPy сокращает дробь и нормализует знак уже при конструировании
`Rational`, поэтому факторизация возвращает ровно тот же объект и `srepr` даёт
ту же строку.

Строка `srepr` — это идентичность `ExactScalar` и вход семантических
дайджестов. Если быстрый путь когда-нибудь разойдётся со символьным хотя бы
в одном представлении, изменятся дайджесты и молча поедет вся семантика.
Поэтому равенство здесь проверяется, а не декларируется.
"""

from __future__ import annotations

from fractions import Fraction

import pytest
import sympy as sp

from cftuv_envelope.reference.planar_types import (
    ExactScalar,
    SYMBOLIC_FALLBACK_COUNTS,
    exact_normalize,
    exact_sign,
)
from cftuv_envelope.reference import planar_types


def _slow_canonical(value: sp.Expr) -> str:
    return sp.srepr(sp.factor(sp.cancel(value)))


def _slow_sign(value: sp.Expr) -> int:
    candidate = sp.factor(sp.cancel(value))
    if candidate == 0 or candidate.is_zero is True:
        return 0
    if candidate.is_positive is True:
        return 1
    if candidate.is_negative is True:
        return -1
    raise AssertionError("корпус должен состоять из разрешимых значений")


def _rational_corpus() -> tuple[sp.Expr, ...]:
    values: list[sp.Expr] = [sp.Integer(0), sp.Integer(1), sp.Integer(-1)]
    for numerator in (-97, -12, -1, 0, 1, 5, 12, 97, 10**12, -(10**12)):
        for denominator in (1, 2, 3, 7, 97, 999983, 2**20):
            values.append(sp.Rational(numerator, denominator))
    # Формы, приходящие из host-координат: точные значения binary64.
    for sample in (0.1, -0.1, 1e-9, -2.5, 3.0, 1234.5678):
        values.append(sp.Rational(*Fraction(sample).as_integer_ratio()))
    # Неприведённые произведения и суммы, как их строит арифметика evaluator'а.
    values.extend(
        [
            sp.Rational(4, 6) * sp.Rational(9, 8),
            sp.Rational(1, 3) + sp.Rational(1, 6),
            sp.Rational(-5, 11) - sp.Rational(6, 11),
            sp.Rational(7, 9) / sp.Rational(14, 3),
        ]
    )
    return tuple(values)


@pytest.mark.parametrize("value", _rational_corpus(), ids=sp.srepr)
def test_rational_fast_path_matches_symbolic_path(value: sp.Expr):
    assert planar_types._canonical_expr(value) == _slow_canonical(value)
    assert exact_sign(value) == _slow_sign(value)
    assert sp.srepr(exact_normalize(value)) == sp.srepr(sp.factor(value))


@pytest.mark.parametrize("value", _rational_corpus(), ids=sp.srepr)
def test_hand_rolled_srepr_matches_sympy(value: sp.Expr):
    """Строка — идентичность ExactScalar и вход дайджестов: только побитово."""

    assert planar_types._rational_srepr(value) == sp.srepr(value)


@pytest.mark.parametrize("value", _rational_corpus(), ids=sp.srepr)
def test_hand_rolled_parse_matches_sympify(value: sp.Expr):
    text = planar_types._canonical_expr(value)
    assert planar_types._parse_expr(text) == sp.sympify(text)


@pytest.mark.parametrize("value", _rational_corpus(), ids=sp.srepr)
def test_scalar_round_trip_preserves_the_value(value: sp.Expr):
    assert ExactScalar.from_value(value).as_expr() == value


def test_composite_srepr_is_not_mistaken_for_a_rational():
    """Якорные шаблоны не должны хватать составное выражение за префикс."""

    composite = sp.Integer(3) * sp.sqrt(2) + sp.Rational(1, 4)
    text = sp.srepr(composite)
    assert planar_types._parse_expr(text) == sp.sympify(text)
    assert planar_types._INTEGER_SREPR.match(text) is None
    assert planar_types._RATIONAL_SREPR.match(text) is None


def test_rational_corpus_never_touches_the_symbolic_path():
    """Иначе быстрый путь есть, но не срабатывает — и замер обманывает."""

    before = dict(SYMBOLIC_FALLBACK_COUNTS)
    for value in _rational_corpus():
        planar_types._canonical_expr(value)
        exact_sign(value)
        exact_normalize(value)
    assert dict(SYMBOLIC_FALLBACK_COUNTS) == before


def test_algebraic_values_still_take_the_symbolic_path():
    """Радикалы обязаны сохранить прежнее поведение, а не тихо просочиться."""

    algebraic = sp.Rational(3, 7) + sp.Rational(2, 5) * sp.sqrt(sp.Rational(13, 3))
    before = SYMBOLIC_FALLBACK_COUNTS["sign"]
    assert exact_sign(algebraic) == 1
    assert SYMBOLIC_FALLBACK_COUNTS["sign"] == before + 1
    assert exact_sign(-algebraic) == -1


# --------------------------------------------------------------------------
# Сертифицированный интервальный фильтр знака
#
# Фильтр обязан либо доказать знак, либо честно вернуть None. Ошибочный знак
# здесь означает неверную топологию arrangement'а — то есть ровно тот класс
# отказов, ради которого точное ядро и строилось.
# --------------------------------------------------------------------------


def _algebraic_corpus() -> tuple[sp.Expr, ...]:
    half_root_two = sp.sqrt(2) / 2
    return (
        sp.Rational(3, 7) + sp.Rational(2, 5) * sp.sqrt(2),
        sp.Rational(3, 7) - sp.Rational(2, 5) * sp.sqrt(2),
        sp.sqrt(3) - sp.Rational(17320508, 10**7),
        sp.Rational(17320508, 10**7) - sp.sqrt(3),
        (sp.Rational(3, 7) + sp.Rational(2, 5) * sp.sqrt(13))
        * (sp.Rational(5, 11) - sp.Rational(1, 9) * sp.sqrt(7)),
        half_root_two - sp.Rational(7071067811865475, 10**16),
        sp.Rational(1, 10**20) * sp.sqrt(3),
        -sp.Rational(1, 10**20) * sp.sqrt(5),
        sp.sqrt(sp.Rational(5, 8) - sp.sqrt(5) / 8),
    )


_TRUE_ZEROS = (
    sp.sqrt(2) * sp.sqrt(3) - sp.sqrt(6),
    sp.Rational(1, 2) * sp.sqrt(3) - sp.Rational(1, 2) * sp.sqrt(3),
    (sp.sqrt(2) + 1) * (sp.sqrt(2) - 1) - 1,
)


@pytest.mark.parametrize("value", _algebraic_corpus(), ids=str)
def test_interval_filter_never_contradicts_the_exact_path(value: sp.Expr):
    certified = planar_types._certified_interval_sign(value)
    if certified is None:
        pytest.skip("оболочка накрыла ноль — законный уход в точный путь")
    assert certified == exact_sign(value)


@pytest.mark.parametrize("value", _TRUE_ZEROS, ids=str)
def test_interval_filter_refuses_to_decide_a_true_zero(value: sp.Expr):
    """Тождественный ноль интервалом не доказать — обязан быть None."""

    assert planar_types._certified_interval_sign(value) is None
    assert exact_sign(value) == 0


def test_interval_filter_returns_none_for_unsupported_nodes():
    """Неизвестный узел — не повод угадать знак."""

    symbol = sp.Symbol("t", positive=True)
    assert planar_types._certified_interval_sign(symbol + 1) is None
    assert planar_types._certified_interval_sign(sp.cos(sp.Rational(1, 3))) is None


def test_interval_filter_restores_global_precision():
    """`mpmath.iv.prec` — глобальное состояние; фильтр не имеет права его сдвигать."""

    from mpmath import iv

    before = iv.prec
    planar_types._certified_interval_sign(sp.sqrt(2) - sp.Rational(1, 3))
    planar_types._certified_interval_sign(sp.Symbol("q"))
    assert iv.prec == before
