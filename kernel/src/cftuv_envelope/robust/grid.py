"""Целочисленная решётка координат и точная невязка привязки к ней.

Смысл решётки: пока координаты целые, каждый предикат — целочисленный
определитель, то есть точен без порогов и без откатов. Как только в координату
попадает произвольная дробь, степень предиката перестаёт быть ограниченной, и
приходится либо платить точной арифметикой за каждую операцию, либо угадывать.
Решётка убирает этот выбор.

Цена — вершины «плавают»: точка пересечения в общем случае на решётке не лежит
и привязывается к ближайшему узлу. Владелец это разрешил именно для декалей.
Но плавание обязано быть **записанным**: `SnappedPointV1` несёт точную
невязку дробью, и выход за бюджет — именованный исход, а не тихое сглаживание.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction


# Граница, до которой определитель orient2d считается ТОЧНО в binary64.
#
# orient2d = (bx-ax)(cy-ay) - (by-ay)(cx-ax). При |координат| <= 2**b разности
# не превышают 2**(b+1), произведения — 2**(2b+2), их разность — 2**(2b+3).
# Мантисса binary64 держит целые до 2**53, поэтому нужно 2b+2 <= 53, то есть
# b <= 25.
#
# Здесь эта константа не используется: предикаты считаются на `int` и точны при
# любой величине. Она объявлена для порта на C++, где выбор между машинным
# словом и многоразрядным числом реален, и проверена состязательным тестом:
# при b=25 расхождений с целочисленным путём нет, при b=27 их уже 247 из 3000,
# при b=40 неверен каждый ответ. Граница тесна, а не «с запасом».
DOUBLE_EXACT_MAGNITUDE_BOUND = 25


class SnapOutcome(str, Enum):
    """Что произошло при привязке. Тихого исчезновения быть не может."""

    ON_GRID = "ON_GRID"
    SNAPPED_WITHIN_BUDGET = "SNAPPED_WITHIN_BUDGET"
    SNAP_DISPLACEMENT_EXCEEDS_BUDGET = "SNAP_DISPLACEMENT_EXCEEDS_BUDGET"


@dataclass(frozen=True, slots=True)
class GridSpecV1:
    """Решётка: сколько целых единиц на единицу источника и предел величины.

    `magnitude_bound` в Python на корректность не влияет — это объявленный
    предел, который позволяет порту на C++ выбрать машинный путь, а тесту —
    поймать вход, вылезший за проектные рамки, вместо того чтобы молча его
    посчитать.
    """

    scale: int
    magnitude_bound: int = DOUBLE_EXACT_MAGNITUDE_BOUND

    def __post_init__(self) -> None:
        if self.scale <= 0:
            raise ValueError("scale решётки должен быть положительным")
        if self.magnitude_bound <= 0:
            raise ValueError("magnitude_bound должен быть положительным")

    @property
    def limit(self) -> int:
        return 1 << self.magnitude_bound

    def within_bound(self, coordinate: int) -> bool:
        return -self.limit <= coordinate <= self.limit


@dataclass(frozen=True, slots=True)
class SnappedPointV1:
    """Узел решётки плюс точная невязка, с которой в него привязали.

    `squared_displacement` — дробь, а не float: это входит в дайджест, значит
    обязана быть воспроизводимой побитово.
    """

    x: int
    y: int
    squared_displacement: Fraction
    outcome: SnapOutcome


def snap_value(value: Fraction | int, grid: GridSpecV1) -> int:
    """Ближайший узел решётки. Половина — вверх, детерминированно.

    `round()` не годится: банковское округление зависит от чётности и читается
    как случайность в дайджесте. Здесь ровно floor(x + 1/2) на целых.
    """

    scaled = Fraction(value) * grid.scale
    numerator, denominator = scaled.numerator, scaled.denominator
    return (2 * numerator + denominator) // (2 * denominator)


def unsnap_value(coordinate: int, grid: GridSpecV1) -> Fraction:
    return Fraction(coordinate, grid.scale)


# Наблюдение за привязкой. Заводится ДО решётки намеренно: R1 иначе нечем
# принимать, а счётчик, появившийся вместе с изменением, не может показать,
# что было до него. Пока `snap_point` никем не вызывается, все четыре нуля —
# и это честный ответ «привязок не было», а не отсутствие измерения.
#
# `merged_points` — то, ради чего вся миграция: сколько РАЗЛИЧНЫХ точек легло
# в один узел. Это восстановленное вырождение, которое binary64 разнёс.
SNAP_COUNTS = {
    "points_total": 0,
    "points_moved": 0,
    "merged_points": 0,
}
# Максимум квадрата сдвига держится дробью отдельно: складывать его с
# целочисленными счётчиками в одном словаре значило бы смешать разные величины.
SNAP_DISPLACEMENT_MAX_SQUARED = [Fraction(0)]
# Узлы, уже занятые привязкой, — источник `merged_points`. Сбрасывается вместе
# со счётчиками на границе домена: слияние осмысленно только внутри одного.
_SNAPPED_NODES: set[tuple[int, int]] = set()


def reset_snap_counts() -> None:
    """Обнулить наблюдение на границе домена."""

    for key in SNAP_COUNTS:
        SNAP_COUNTS[key] = 0
    SNAP_DISPLACEMENT_MAX_SQUARED[0] = Fraction(0)
    _SNAPPED_NODES.clear()


def _record_snap(node: tuple[int, int], squared: Fraction, moved: bool) -> None:
    SNAP_COUNTS["points_total"] += 1
    if moved:
        SNAP_COUNTS["points_moved"] += 1
    if node in _SNAPPED_NODES:
        SNAP_COUNTS["merged_points"] += 1
    else:
        _SNAPPED_NODES.add(node)
    if squared > SNAP_DISPLACEMENT_MAX_SQUARED[0]:
        SNAP_DISPLACEMENT_MAX_SQUARED[0] = squared


def snap_point(
    x: Fraction | int,
    y: Fraction | int,
    grid: GridSpecV1,
    *,
    squared_budget: Fraction | None = None,
) -> SnappedPointV1:
    """Привязать точку и записать, насколько её сдвинуло.

    Бюджет задаётся в квадрате расстояния, чтобы не извлекать корень: сравнение
    квадратов эквивалентно сравнению расстояний и остаётся в дробях.
    """

    grid_x, grid_y = snap_value(x, grid), snap_value(y, grid)
    residual_x = Fraction(x) - unsnap_value(grid_x, grid)
    residual_y = Fraction(y) - unsnap_value(grid_y, grid)
    squared = residual_x * residual_x + residual_y * residual_y
    _record_snap((grid_x, grid_y), squared, bool(residual_x or residual_y))

    if squared == 0:
        outcome = SnapOutcome.ON_GRID
    elif squared_budget is not None and squared > squared_budget:
        outcome = SnapOutcome.SNAP_DISPLACEMENT_EXCEEDS_BUDGET
    else:
        outcome = SnapOutcome.SNAPPED_WITHIN_BUDGET

    return SnappedPointV1(grid_x, grid_y, squared, outcome)
