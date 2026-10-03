"""Закон окна луча адаптивного веера: узкая полоса поворота (RIGHT-ANGLE-STABLE).

Окно ординала — множество направлений, из которых власть вправе выбрать луч.
Законов два, и власть называет свой в `proven_predicates`.

* ОКНО ВОРОНОГО (прежний закон, байты власти V2 заморожены): между серединами
  углов с соседними лучами идеала. Оно ШИРОКОЕ — полшага веера в обе стороны, — а
  поиск минимальной общей высоты ставит луч на прямую наименьшей высоты решётки
  карты независимо от равного шага: на d1 `building` прямой угол давал 23 набора
  шагов (45/45 лишь 36 раз из 167, остальные 57.8/32.2, 46.8/43.2, 36/54…).
* УЗКАЯ ПОЛОСА ПОВОРОТА (решение владельца 2026-10-03): окно ординала —
  направления в пределах угла `omega` от луча РАВНОУГОЛЬНОГО идеала,
  `tan(omega) = 1/57` (1.005 градуса), СКРЕЩЁННЫЕ с окном Вороного. Поиск тот же —
  минимальная общая высота, затем ближайший к идеалу победитель, — но луч
  привязывается к рациональному направлению БЛИЗКО к равному шагу, а не к
  ближайшей простой прямой решётки. Подшаг `<= pi/q` проверяется ТОЧНО по
  настоящим соседям идеала и полосой не ослаблен. Отказ закона (исчерпание точной
  работы, неустановимый чарт, пустое окно) называется диагностикой, и веер ищет
  прежнее окно Вороного.

Полоса задана как окно Вороного при ФАНТОМНЫХ соседях идеала, повёрнутых на
`±2*omega`: середина двух единичных ковекторов на угле `2*omega` лежит ровно на
`omega`. Поворот на `2*omega` рационален (`cos = (1-t^2)/(1+t^2)`, `sin =
2t/(1+t^2)`, пифагорова пара 3248/114/3250), радикалов поворот не добавляет, и вся
прежняя точная машинерия окон (проективный чарт, atlas, внешние оболочки) работает
без единого нового предиката. Если настоящий сосед ближе `2*omega`, на этой
стороне остаётся он сам: полоса не выходит за опору (мелкий излом дуги — шаг
веера меньше двух градусов, — иначе в окно попадали бы лучи по ту сторону опоры).
"""

from __future__ import annotations

from fractions import Fraction

import sympy as sp

ADAPTIVE_FAN_NARROW_BAND_HALF_TANGENT = Fraction(1, 57)
ADAPTIVE_FAN_NARROW_ROTATION_BAND = "ADAPTIVE_FAN_NARROW_ROTATION_BAND"
WINDOW_LAW_VORONOI = "VORONOI_OF_NEIGHBOUR_MIDPOINTS"
WINDOW_LAW_NARROW_BAND = "NARROW_ROTATION_BAND"


def authority_window_law(authority) -> str:
    """Закон окна, который власть называет в своих доказанных предикатах."""

    return (
        WINDOW_LAW_NARROW_BAND
        if ADAPTIVE_FAN_NARROW_ROTATION_BAND
        in getattr(authority, "proven_predicates", ())
        else WINDOW_LAW_VORONOI
    )


def _double_band_trig():
    """`(cos 2*omega, sin 2*omega)` точной рациональной парой: `tan(omega) = t`."""

    t = ADAPTIVE_FAN_NARROW_BAND_HALF_TANGENT
    cosine = (1 - t * t) / (1 + t * t)
    sine = 2 * t / (1 + t * t)
    return (
        sp.Rational(cosine.numerator, cosine.denominator),
        sp.Rational(sine.numerator, sine.denominator),
    )


def _neighbour_is_inside_the_double_band(metric, vector, neighbour) -> bool:
    """Настоящий сосед идеала ближе `2*omega`: `cos(angle) >= cos(2*omega)`, ТОЧНО.

    В квадратах и без корней, при положительном скалярном произведении.
    """

    from .adaptive_density_fan import _dual_dot, _sign

    dot = _dual_dot(metric, vector, neighbour)
    if _sign(dot, metric) <= 0:
        return False
    cosine, _ = _double_band_trig()
    norms = _dual_dot(metric, vector, vector) * _dual_dot(metric, neighbour, neighbour)
    return _sign(dot * dot - cosine * cosine * norms, metric) >= 0


def _rotated_by_double_band(metric, vector, sign: int, orientation: int):
    """Единичный ковектор, повёрнутый на `sign * 2 * omega` в сторону ОРИЕНТАЦИИ угла.

    `orientation` — `_expected_orientation` сектора: предыдущий сосед веера лежит на
    `-orientation` от центра, следующий — на `+orientation` (так же ставит соседей
    `_subturn_boundary_vectors`). Без ориентации стороны менялись бы местами на CW.
    """

    from .adaptive_density_fan import _quarter_turn, _vector

    cosine, sine = _double_band_trig()
    perpendicular = _quarter_turn(metric, vector, orientation)
    x, y = metric.density_expressions(vector)
    px, py = metric.density_expressions(perpendicular)
    return _vector(
        cosine * x + sign * sine * px,
        cosine * y + sign * sine * py,
        metric,
    )


def window_neighbours(ideal, ordinal: int):
    """`(предыдущий, следующий)`, чьи середины с `ideal[ordinal]` задают окно ординала.

    Окно Вороного — настоящие соседи идеала. Полоса — фантом на `±2*omega` либо
    настоящий сосед, если он ближе. Подшаг `<= pi/q` читает НАСТОЯЩИХ соседей.
    """

    if ideal.window_law != WINDOW_LAW_NARROW_BAND:
        return ideal[ordinal - 1], ideal[ordinal + 1]
    cache = ideal.band_cache
    if ordinal not in cache:
        if ideal.band_orientation is None:
            raise ValueError("the narrow band needs the turn orientation of the sector")
        center, metric = ideal[ordinal], ideal.metric
        cache[ordinal] = tuple(
            real
            if _neighbour_is_inside_the_double_band(metric, center, real)
            else _rotated_by_double_band(metric, center, sign, ideal.band_orientation)
            for real, sign in ((ideal[ordinal - 1], -1), (ideal[ordinal + 1], 1))
        )
    return cache[ordinal]


def window_triple(ideal, ordinal: int):
    previous, following = window_neighbours(ideal, ordinal)
    return previous, ideal[ordinal], following


def window_law_predicates(window_law: str, base: frozenset) -> frozenset:
    """Доказанные предикаты власти под законом окна: у полосы к базовым добавлено имя полосы."""

    if window_law == WINDOW_LAW_NARROW_BAND:
        return frozenset({*base, ADAPTIVE_FAN_NARROW_ROTATION_BAND})
    return base


def known_predicate_sets(base: frozenset) -> tuple:
    return tuple(window_law_predicates(law, base) for law in (WINDOW_LAW_VORONOI, WINDOW_LAW_NARROW_BAND))
