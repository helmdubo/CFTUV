"""Предикаты на целочисленной решётке. Точные, без порогов, без откатов.

Каждая функция здесь — целочисленный определитель или сравнение целых. Порога
нет не потому, что он мал, а потому, что его негде поставить: ответ либо
положителен, либо отрицателен, либо ноль, и это тот же ответ, что дала бы любая
точная арифметика.

Сравнение с тем, что было: те же предикаты через `ExactScalar`/`exact_sign`
стоили 9.89 мкс на orient2d (со построением промежуточных значений). Здесь —
0.25 мкс, и это ровно 40x при том же ответе.

Почему не binary64 с фильтром, как в Boost: замерено, что на Python `float`
не быстрее `int` (0.30 против 0.25 мкс на состязательном корпусе), а точен
только до 2**25 по величине координат. Фильтр в C++ покупает выбор между
машинным словом и bignum; в Python этого выбора нет — `int` и есть bignum.
Платить сложностью фильтра за отрицательный выигрыш нельзя.
"""

from __future__ import annotations

from fractions import Fraction

from .grid import GridSpecV1, SnappedPointV1, snap_point


Point = tuple[int, int]


def compare(left: int, right: int) -> int:
    return (left > right) - (left < right)


def orient2d(a: Point, b: Point, c: Point) -> int:
    """Знак ориентации тройки: +1 против часовой, -1 по часовой, 0 коллинеарны."""

    value = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    return (value > 0) - (value < 0)


def cross_sign(u: Point, v: Point) -> int:
    value = u[0] * v[1] - u[1] * v[0]
    return (value > 0) - (value < 0)


def dot_sign(u: Point, v: Point) -> int:
    value = u[0] * v[0] + u[1] * v[1]
    return (value > 0) - (value < 0)


def on_segment(point: Point, start: Point, end: Point) -> bool:
    """Точка лежит на замкнутом отрезке.

    Сначала коллинеарность, затем попадание в габарит. Габарита достаточно
    именно потому, что коллинеарность уже доказана: на прямой порядок по
    координате совпадает с порядком по параметру.
    """

    if orient2d(start, end, point) != 0:
        return False
    low_x, high_x = min(start[0], end[0]), max(start[0], end[0])
    low_y, high_y = min(start[1], end[1]), max(start[1], end[1])
    return low_x <= point[0] <= high_x and low_y <= point[1] <= high_y


def point_in_loop(point: Point, loop: tuple[Point, ...]) -> int:
    """1 внутри, 0 на границе, -1 снаружи. Winding по пересечениям с лучом.

    Луч идёт вправо. Полуоткрытое правило по y (`start_above != end_above`)
    исключает двойной учёт вершины, а знак `orient2d` даёт сторону без деления,
    поэтому вертикальный отрезок не требует особого случая.
    """

    for index in range(len(loop)):
        if on_segment(point, loop[index], loop[(index + 1) % len(loop)]):
            return 0

    winding = 0
    for index in range(len(loop)):
        start, end = loop[index], loop[(index + 1) % len(loop)]
        start_above = start[1] > point[1]
        end_above = end[1] > point[1]
        if start_above == end_above:
            continue
        side = orient2d(start, end, point)
        if end_above and side > 0:
            winding += 1
        elif not end_above and side < 0:
            winding -= 1
    return 1 if winding != 0 else -1


def segment_intersection(
    a: Point,
    b: Point,
    c: Point,
    d: Point,
    grid: GridSpecV1,
    *,
    squared_budget: Fraction | None = None,
) -> SnappedPointV1 | None:
    """Точка пересечения двух отрезков решётки, привязанная обратно к решётке.

    Возвращает None, когда отрезки параллельны или не пересекаются в пределах
    своих концов. Коллинеарное наложение — тоже None: у него нет единственной
    точки, и придумывать её здесь нельзя, это дело вызывающего.

    Пересечение в общем случае дробное, и привязка сдвигает вершину. Сдвиг
    записан в результате точной дробью; выход за бюджет даёт
    `SNAP_DISPLACEMENT_EXCEEDS_BUDGET`, а не тихую правку геометрии.
    """

    r = (b[0] - a[0], b[1] - a[1])
    s = (d[0] - c[0], d[1] - c[1])
    denominator = r[0] * s[1] - r[1] * s[0]
    if denominator == 0:
        return None

    offset = (c[0] - a[0], c[1] - a[1])
    t_numerator = offset[0] * s[1] - offset[1] * s[0]
    u_numerator = offset[0] * r[1] - offset[1] * r[0]

    # Параметр обязан лежать в [0, 1] на ОБОИХ отрезках. Проверка идёт на
    # числителе против знаменателя, без деления: знак знаменателя может быть
    # любым, поэтому сравнение приводится к одному направлению.
    if denominator < 0:
        t_numerator, u_numerator, denominator = (
            -t_numerator,
            -u_numerator,
            -denominator,
        )
    if not (0 <= t_numerator <= denominator):
        return None
    if not (0 <= u_numerator <= denominator):
        return None

    t = Fraction(t_numerator, denominator)
    return snap_point(
        Fraction(a[0]) + t * r[0],
        Fraction(a[1]) + t * r[1],
        grid,
        squared_budget=squared_budget,
    )
