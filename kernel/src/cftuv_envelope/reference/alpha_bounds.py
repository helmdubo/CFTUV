"""Рациональная оболочка alpha контакта: знак `alpha - requested` без разворачивания выражения.

Контакт источника с границей (`boundary._contact_candidates`) несёт alpha — точное выражение `sympy` (радикалы). Оно от
запрошенной alpha не зависит и считается один раз на подготовку, а вот СРАВНЕНИЕ с запрошенной alpha идёт на каждом
нажатии и на каждом контакте: `exact_sign(alpha - requested)` строит новое выражение и обходит его интервальной
арифметикой `mpmath` заново (на `building` центральный патч — сотни таких знаков, доли секунды на нажатие).

Здесь оболочка считается один раз и едет с подготовкой: `[low, high]` — строгие границы значения (интервальная
арифметика с внешним округлением, тот же `interval_enclosure`, на котором стоит и сам `exact_sign`), записанные ТОЧНЫМИ
дробями. Знак разности объявляется, только если `requested` лежит строго вне оболочки; иначе (касание, настоящее
равенство, оболочка не посчитана) считается прежний `exact_sign(alpha - requested)`. Фильтр доказывает либо уступает, поэтому
ответ побитово тот же; меняется цена.
"""

from __future__ import annotations

from fractions import Fraction

import sympy as sp
from mpmath import iv
from mpmath.libmp import to_rational

from .planar_types import IntervalEnclosureUnsupported, exact_sign, interval_enclosure

#: Точность оболочки в битах: оболочка уже ширины 2^-100 от значения порядка единицы, то есть касание
#: с запрошенной alpha (десятичной дробью) возможно только у настоящего равенства.
BOUND_PRECISION_BITS = 128


def alpha_bounds(alpha) -> tuple[Fraction, Fraction] | None:
    """`(low, high)`: строгая оболочка значения `alpha` (`sympy`) точными дробями либо `None` (не посчитана)."""

    if alpha.is_Rational:
        exact = Fraction(int(alpha.p), int(alpha.q))
        return exact, exact
    saved = iv.prec
    iv.prec = BOUND_PRECISION_BITS
    try:
        low, high = interval_enclosure(alpha)._mpi_
        return Fraction(*to_rational(low)), Fraction(*to_rational(high))
    except (IntervalEnclosureUnsupported, ArithmeticError, ValueError, TypeError):
        return None
    finally:
        iv.prec = saved


def sign_against(alpha, bounds, requested) -> int:
    """Знак `alpha - requested` (`requested` — рациональное `sympy`): оболочка решает, когда `requested` вне её."""

    if bounds is not None:
        value = Fraction(int(requested.p), int(requested.q))
        if bounds[0] > value:
            return 1
        if bounds[1] < value:
            return -1
    return exact_sign(alpha - requested)


def bounds_of_contacts(contacts) -> tuple:
    """Оболочки alpha всех контактов `((alpha, station, point), ...)` в том же порядке."""

    return tuple(alpha_bounds(alpha) if isinstance(alpha, sp.Expr) else None for alpha, _station, _point in contacts)
