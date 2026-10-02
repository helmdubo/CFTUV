"""Рациональность отношения двух радикальных сумм — точно и без факторизации.

Вопрос: лежит ли `y / x` в `Q`, где `x`, `y` — SymPy-выражения из рациональных
чисел, сумм, произведений и `sqrt` рационального числа (целые степени,
в том числе отрицательные, допустимы). Ответ даёт точная арифметика над
конечной суммой `c_i * sqrt(r_i)`, а не вид выражения: `(1 + sqrt 3, 2 + 2 sqrt 3)`
— направление `(1, 2)`, хотя `ratio.is_Rational` у SymPy для него `False`.

Почему не `exact_quadratic_value` / `SqrtSumV1`: их каноническая форма требует
бесквадратного разложения радиканда, то есть ФАКТОРИЗАЦИИ, а радиканды поля —
нормы метрики карты в десятки знаков. Факторизация идёт мимо транзакционного
бюджета, и полевые ворота требуют нулевой неоплаченной работы. Здесь канон
другой, и он дешёвый: `sqrt(a)` и `sqrt(b)` лежат в одном квадратном классе
тогда и только тогда, когда `a*b` — полный квадрат (одна `isqrt`), а корни
из РАЗНЫХ квадратных классов линейно независимы над `Q`. Поэтому сумма
приводится к виду «по одному коэффициенту на класс», и отношение рационально
тогда и только тогда, когда векторы коэффициентов `y` и `x` пропорциональны с
рациональным множителем.

Выражение вне поля (нераскрытый `cos`/`sin` угла, корень из суммы) НЕ решается:
ответ `None`, и потребитель обязан отличать его от `False`.
"""

from __future__ import annotations

from fractions import Fraction
from functools import lru_cache
from math import gcd, isqrt

import sympy as sp

# Столько квадратных классов в одной величине допускается: оценка стоимости
# обращения суммы сопряжением, а не математический предел.
_CLASS_LIMIT = 8


class _OutsideTheRadicalField(Exception):
    """Выражение не читается как конечная сумма корней из рациональных."""


class _Classes:
    """Реестр представителей квадратных классов одной величины.

    `canon(r)` возвращает `(s, k)`, где `s` — представитель класса `r`, а
    `sqrt(r) = k * sqrt(s)` с рациональным `k`. Проверка класса — полный квадрат
    произведения: `sqrt(r) = isqrt(r*s)/s * sqrt(s)`, когда `r*s` — квадрат.
    """

    def __init__(self) -> None:
        self.reps: list[int] = [1]

    def canon(self, radicand: int) -> tuple[int, Fraction]:
        for rep in self.reps:
            product = radicand * rep
            root = isqrt(product)
            if root * root == product:
                return rep, Fraction(root, rep)
        self.reps.append(radicand)
        return radicand, Fraction(1)


Sum = dict[int, Fraction]


def _put(total: Sum, radicand: int, coefficient: Fraction) -> None:
    value = total.get(radicand, Fraction(0)) + coefficient
    if value:
        total[radicand] = value
    else:
        total.pop(radicand, None)


def _add(left: Sum, right: Sum) -> Sum:
    total = dict(left)
    for radicand, coefficient in right.items():
        _put(total, radicand, coefficient)
    return total


def _multiply(classes: _Classes, left: Sum, right: Sum) -> Sum:
    total: Sum = {}
    for first, first_coefficient in left.items():
        for second, second_coefficient in right.items():
            common = gcd(first, second)
            radicand, factor = classes.canon(
                (first // common) * (second // common)
            )
            _put(
                total,
                radicand,
                first_coefficient * second_coefficient * common * factor,
            )
    return total


def _invert(classes: _Classes, value: Sum) -> Sum:
    """`1 / value` сопряжением по одному классу за шаг."""

    if not value:
        raise _OutsideTheRadicalField("division by an exact zero")
    if len(value) == 1:
        ((radicand, coefficient),) = value.items()
        return {radicand: 1 / (coefficient * radicand)}
    if len(value) > _CLASS_LIMIT:
        raise _OutsideTheRadicalField("too many square classes to invert")
    # value = rest + b*sqrt(s); (rest - b sqrt s)(rest + b sqrt s) = rest^2 - b^2 s
    pivot = max(value)
    rest = {key: item for key, item in value.items() if key != pivot}
    conjugate = dict(rest)
    conjugate[pivot] = -value[pivot]
    squared = _multiply(classes, rest, rest)
    _put(squared, 1, -value[pivot] * value[pivot] * pivot)
    return _multiply(classes, conjugate, _invert(classes, squared))


def _power(classes: _Classes, base: Sum, exponent: int) -> Sum:
    result: Sum = {1: Fraction(1)}
    for _ in range(abs(exponent)):
        result = _multiply(classes, result, base)
    return _invert(classes, result) if exponent < 0 else result


def _read(classes: _Classes, expression: sp.Expr) -> Sum:
    if expression.is_Rational:
        value = Fraction(int(expression.p), int(expression.q))
        return {1: value} if value else {}
    if expression.is_Add:
        total: Sum = {}
        for term in expression.args:
            total = _add(total, _read(classes, term))
        return total
    if expression.is_Mul:
        product: Sum = {1: Fraction(1)}
        for factor in expression.args:
            product = _multiply(classes, product, _read(classes, factor))
        return product
    if expression.is_Pow:
        base, exponent = expression.args
        if exponent.is_Integer:
            return _power(classes, _read(classes, base), int(exponent))
        if exponent.is_Rational and exponent.q == 2 and base.is_Rational:
            if base < 0:
                raise _OutsideTheRadicalField("root of a negative number")
            numerator, denominator = int(base.p), int(base.q)
            radicand, factor = classes.canon(numerator * denominator)
            root: Sum = {radicand: factor / denominator}
            return _power(classes, root, int(exponent.p))
    raise _OutsideTheRadicalField(sp.srepr(expression))


@lru_cache(maxsize=32768)
def _terms(expression: sp.Expr) -> tuple[tuple[int, Fraction], ...]:
    return tuple(sorted(_read(_Classes(), expression).items()))


def radical_ratio_is_rational(
    numerator: sp.Expr,
    denominator: sp.Expr,
) -> bool | None:
    """`numerator / denominator in Q`: `True` / `False` точно, `None` — вне поля.

    Знаменатель обязан быть точным ненулём; нулевой вернёт `None`.
    """

    proportional, _ = _proportion(numerator, denominator)
    return proportional


def radical_ratio_value(
    numerator: sp.Expr,
    denominator: sp.Expr,
) -> Fraction | None:
    """Значение `numerator / denominator`, если оно рационально; иначе `None`.

    Тот же точный разбор, что у `radical_ratio_is_rational`, и тот же запрет
    факторизации: `None` объединяет «иррационально» и «вне поля» — потребителю,
    которому нужно различие, остаётся `radical_ratio_is_rational`.
    """

    proportional, ratio = _proportion(numerator, denominator)
    return ratio if proportional else None


def _proportion(
    numerator: sp.Expr,
    denominator: sp.Expr,
) -> tuple[bool | None, Fraction | None]:
    """`(рационально?, значение)`: `(None, None)` — вне поля или нулевой знаменатель."""

    try:
        upper = _terms(numerator)
        lower = _terms(denominator)
    except _OutsideTheRadicalField:
        return None, None
    if not lower:
        return None, None
    # Один реестр на обе величины: классы `x` и `y` сравниваются между собой.
    classes = _Classes()
    top: Sum = {}
    bottom: Sum = {}
    for target, source in ((top, upper), (bottom, lower)):
        for radicand, coefficient in source:
            rep, factor = classes.canon(radicand)
            _put(target, rep, coefficient * factor)
    pivot = next(iter(bottom))
    ratio = top.get(pivot, Fraction(0)) / bottom[pivot]
    proportional = all(
        top.get(rep, Fraction(0)) == ratio * bottom.get(rep, Fraction(0))
        for rep in {*top, *bottom}
    )
    return proportional, (ratio if proportional else None)
