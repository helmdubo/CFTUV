"""Робастность за счёт ограничения входа, а не за счёт точности каждого числа.

Тот приём, которым Boost.Polygon добивается скорости на Voronoi: вход —
целочисленный, поэтому степень предиката ограничена наперёд, поэтому знак
доказывается дешёвой арифметикой без порогов и без откатов.

Отличие от Boost — в языке, и оно упрощает дело. В C++ приходится выбирать
между `int64` и многоразрядным числом, отсюда трёхуровневый фильтр. В Python
`int` уже многоразрядный и на малых значениях быстрый, поэтому фильтра нет
вообще: предикат считается на `int`, точен при любой величине координат, и
откатываться некуда, потому что и не падало.

Что здесь есть:
- `grid` — решётка, привязка к ней и точная невязка привязки;
- `predicates` — предикаты на целых: orient2d, знак скалярного, сравнения.

Чего здесь нет и не будет: SymPy, строк-как-чисел, интервалов, порогов.
"""

from .grid import (
    DOUBLE_EXACT_MAGNITUDE_BOUND,
    GridSpecV1,
    SnapOutcome,
    SnappedPointV1,
    snap_point,
    snap_value,
    unsnap_value,
)
from .predicates import (
    compare,
    cross_sign,
    dot_sign,
    on_segment,
    orient2d,
    point_in_loop,
    segment_intersection,
)

__all__ = [
    "DOUBLE_EXACT_MAGNITUDE_BOUND",
    "GridSpecV1",
    "SnapOutcome",
    "SnappedPointV1",
    "compare",
    "cross_sign",
    "dot_sign",
    "on_segment",
    "orient2d",
    "point_in_loop",
    "segment_intersection",
    "snap_point",
    "snap_value",
    "unsnap_value",
]
