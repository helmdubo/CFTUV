"""Сертифицированная оболочка конусного угла вершины. Без «примерно равно».

Задача. Конусный угол вершины — сумма углов треугольников при ней. Каждый угол
есть `arccos c`, где `c = (A + B − C) / (2·√(A·B))` по квадратам сторон: число
АЛГЕБРАИЧЕСКОЕ, а сумма арккосинусов — трансцендентная. Ни рациональная
арифметика, ни `SqrtSumV1` этой суммы не вмещают, поэтому ответ здесь —
ОБОЛОЧКА с именованной шириной, а не число.

Как получается доказанность. В два шага, оба проверяемых:

1. `c` заключается в рациональные границы ЦЕЛОЧИСЛЕННОЙ арифметикой:
   `√(p/q) = √(p·q·10^{2d}) / (q·10^d)`, а `isqrt` даёт точные целые границы
   корня. Ни одного float на этом шаге нет.
2. Границы угла проверяются ОБРАТНО, через интервальный косинус `mpmath.iv`:
   `arccos c ∈ [lo, hi]` тогда и только тогда, когда `cos hi ≤ c ≤ cos lo` при
   `0 ≤ lo ≤ hi ≤ π`. Кандидат берётся из быстрого `math.acos`, но НЕ
   принимается на слово: пока проверка не прошла, окно расширяется. Ошибиться
   кандидату можно, проверке — нет.

Почему не `mpmath.acos` с высокой точностью: она даёт число, а не границу.
«Много знаков» — не доказательство, а обещание; ровно этого рода обещания
запрещает модель точности поверхности.
"""

from __future__ import annotations

from decimal import Decimal
from fractions import Fraction
from math import acos, isqrt

from mpmath import iv, libmp

from .numeric import CertifiedDecimalIntervalV1, IntervalEndpointKind


# Число десятичных знаков, которыми оболочка ЗАПИСЫВАЕТСЯ, и одновременно
# стартовый радиус поиска окна. Это НЕ обещание ширины: поиск расширяет окно,
# пока проверка не пройдёт, поэтому фактическая ширина определяется тем, куда
# попал кандидат (`math.acos`, порядка 1e-16), и она ИЗМЕРЯЕТСЯ, а потом
# записывается в `named_epsilon`. Обещать ширину заранее значило бы называть
# точность, которой никто не проверял.
CONE_ANGLE_DECIMALS = 24
CONE_ANGLE_EPSILON_NAME = "SURFACE_CONE_ANGLE_INTERVAL_ENCLOSURE_V1"

# Битовая точность `mpmath.iv`. Взята с той же лестницы, что и у
# `reference/angle_measure.py`: 24 знака — это ~80 бит, запас нужен, чтобы
# оболочка сходилась заведомо тоньше последнего записываемого знака.
_ENCLOSURE_PRECISION = 256

_SCALE = 10**CONE_ANGLE_DECIMALS
_STEP = Fraction(1, _SCALE)


def _endpoints(interval) -> tuple[Fraction, Fraction]:
    """Точные рациональные концы интервала. Строкового round-trip здесь нет."""

    low, high = interval._mpi_
    return Fraction(*libmp.to_rational(low)), Fraction(*libmp.to_rational(high))


def _sqrt_bounds(value: Fraction) -> tuple[Fraction, Fraction]:
    """Точные рациональные границы `√value` целочисленной арифметикой."""

    if value < 0:
        raise ValueError("корень из отрицательного не берётся")
    if value == 0:
        return Fraction(0), Fraction(0)
    numerator = value.numerator * value.denominator * _SCALE * _SCALE
    root = isqrt(numerator)
    denominator = value.denominator * _SCALE
    lower = Fraction(root, denominator)
    upper = Fraction(root + 1, denominator)
    return lower, upper


def cosine_bounds(
    squared_left: Fraction, squared_right: Fraction, squared_opposite: Fraction
) -> tuple[Fraction, Fraction]:
    """Границы косинуса угла между сторонами с квадратами длин `left`, `right`.

    Теорема косинусов в квадратах: `cos = (A + B − C) / (2·√(A·B))`. Знаменатель
    иррационален, поэтому границы косинуса берутся делением на границы корня —
    с учётом знака числителя, иначе деление переворачивает неравенство молча.
    """

    if squared_left <= 0 or squared_right <= 0:
        raise ValueError("угол при вырожденной стороне не определён")
    numerator = squared_left + squared_right - squared_opposite
    root_low, root_high = _sqrt_bounds(squared_left * squared_right)
    if root_low <= 0:
        raise ValueError("вырожденное произведение сторон")
    low_divisor, high_divisor = 2 * root_low, 2 * root_high
    if numerator >= 0:
        bounds = (numerator / high_divisor, numerator / low_divisor)
    else:
        bounds = (numerator / low_divisor, numerator / high_divisor)
    return (max(bounds[0], Fraction(-1)), min(bounds[1], Fraction(1)))


def _cos_bounds(value: Fraction) -> tuple[Fraction, Fraction]:
    """Строгие границы косинуса рационального числа. Не оценка — оболочка."""

    point = iv.mpf(value.numerator) / iv.mpf(value.denominator)
    return _endpoints(iv.cos(point))


def _verified_angle_window(
    cos_low: Fraction, cos_high: Fraction
) -> tuple[Fraction, Fraction]:
    """Окно `[lo, hi]`, для которого ДОКАЗАНО `arccos c ∈ [lo, hi]`."""

    guess = Fraction(acos(max(-1.0, min(1.0, float((cos_low + cos_high) / 2)))))
    pi_high = _endpoints(iv.pi)[1] + _STEP
    radius = _STEP
    for _attempt in range(64):
        low = max(Fraction(0), guess - radius)
        high = min(pi_high, guess + radius)
        # cos убывает: arccos c ∈ [low, high] ⟺ cos(high) ≤ c ≤ cos(low).
        if _cos_bounds(high)[1] <= cos_low and _cos_bounds(low)[0] >= cos_high:
            return low, high
        radius *= 4
    raise ValueError("оболочка угла не подтвердилась ни на одном окне")


def angle_bounds(
    squared_left: Fraction, squared_right: Fraction, squared_opposite: Fraction
) -> tuple[Fraction, Fraction]:
    """Доказанные рациональные границы угла между двумя сторонами."""

    cos_low, cos_high = cosine_bounds(
        squared_left, squared_right, squared_opposite
    )
    saved = iv.prec
    iv.prec = _ENCLOSURE_PRECISION
    try:
        return _verified_angle_window(cos_low, cos_high)
    finally:
        iv.prec = saved


def two_pi_bounds() -> tuple[Fraction, Fraction]:
    """Рациональные границы 2π, взятые с интервальной константы mpmath."""

    saved = iv.prec
    iv.prec = _ENCLOSURE_PRECISION
    try:
        return _endpoints(2 * iv.pi)
    finally:
        iv.prec = saved


def _decimal(value: Fraction, *, round_up: bool) -> Decimal:
    scaled = value * _SCALE
    integral = scaled.numerator // scaled.denominator
    if round_up and integral * scaled.denominator != scaled.numerator:
        integral += 1
    return Decimal(integral).scaleb(-CONE_ANGLE_DECIMALS)


def certified_cone_angle(angles) -> CertifiedDecimalIntervalV1:
    """Сложить доказанные границы углов веера в одну сертифицированную оболочку.

    Складываются ГРАНИЦЫ, а не значения: сумма нижних — нижняя граница суммы,
    сумма верхних — верхняя. Ширина растёт линейно по числу треугольников, и
    это записано в `absolute_error_bound`, а не спрятано.
    """

    low = sum((item[0] for item in angles), Fraction(0))
    high = sum((item[1] for item in angles), Fraction(0))
    lower = _decimal(low, round_up=False)
    upper = _decimal(high, round_up=True)
    return CertifiedDecimalIntervalV1(
        lower=lower,
        upper=upper,
        lower_kind=IntervalEndpointKind.CLOSED,
        upper_kind=IntervalEndpointKind.CLOSED,
        absolute_error_bound=upper - lower,
    )


__all__ = (
    "CONE_ANGLE_DECIMALS",
    "CONE_ANGLE_EPSILON_NAME",
    "angle_bounds",
    "certified_cone_angle",
    "cosine_bounds",
    "two_pi_bounds",
)
