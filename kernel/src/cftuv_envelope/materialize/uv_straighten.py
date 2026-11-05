"""Закон `UNFOLD_ALONG_U_V1` (режим Straight strips, политика `UV_DIRECT_STRIP_V1`): ряды прямые, `u` развёрнута по геометрии, как Straighten + Unfold along U в Maya.

Физические факты `(s, r)` (`mesh.station_values`) не трогаются: регионы, цепи, станции батча и решения топологии читают их, как раньше. Здесь считаются ТОЛЬКО числители UV `(U, r)`:
`u = U / alpha`, `v = r / alpha` (`uv_law.uv_direct_strip_v1`), тем же путём `uv_numerators`, что у карт Uniform.

* `v` НЕ ТРОНУТА: `r / alpha`, источник на `v = 0`, фронт на `v = 1`, у гребня верх неровный (высота трима постоянна).
* НЕИЗВЕСТНАЯ - вершина региона `(регион, ключ)`; вершины разных регионов одного ключа - одна неизвестная, если у ребра итоговых граней этих регионов факты обоих концов
  ТОЧНО равны и обе грани - полосы либо обе - веера (там батч шва не ставит: `assemble._chains_from`), то есть остров - связная часть граней без шва: одна pChain - один
  остров, веер - свой остров, как у прежнего закона.
* `u` - решение ядра `uv_unfold.unfold` по треугольникам, которые интерполирует Blender, в 3D позициях вершин; источник держится (`hold_source`): у вершины `r = 0` `U = s`
  ТОЧНО (станция цепи), поэтому основание - накопленная длина цепи и общие вершины источника двух островов не расходятся.
* ЖИВАЯ ШИРИНА. Форма вершины `(U, tx, ty, 0)`: наклон `U` по карте в её треугольниках, у вершины источника - нуль (станция неподвижна). Это ЛИНЕАРИЗАЦИЯ: решение острова
  зависит от всей его сетки, точный кадр после паузы считает заново (замер разрыва - DECISIONS: UNFOLD_ALONG_U_V1).
* Числовое ядро нативное на продуктовой дороге (`backend.unfold_compute`), Python - эталон; числа ядра (`uv_unfold.COUNTER_NAMES`) идут в ответ, ни один исход не молчит.
"""

from __future__ import annotations

from fractions import Fraction

from .. import backend
from ..exact_sqrt_sum import SqrtSumV1
from .lift import sqrt_sum_binary64
from .uv_unfold import UnfoldInputV1, unfold

ZERO = SqrtSumV1.zero()
_ANCHORED = (ZERO, ZERO, ZERO)


def _exact(number: float) -> SqrtSumV1:
    return SqrtSumV1.rational(Fraction(number))


def _run_station(run, inverse_scale: Fraction):
    """`(s0, x0, y0, cx, cy)`: линейная станция пробега `s0 + (p - origin) . c` в метрах по точке карты в метрах."""

    inverse = sqrt_sum_binary64(run.inverse_length)
    unit = float(inverse_scale)
    return (
        sqrt_sum_binary64(run.s_origin, factor=inverse_scale),
        float(run.origin[0]) * unit,
        float(run.origin[1]) * unit,
        float(run.covector[0]) * inverse,
        float(run.covector[1]) * inverse,
    )


def unfold_input(frame_faces, layout, rings, points, facts, positions, scale: int, hold_source: bool = True):
    """`(UnfoldInputV1, слоты по неизвестным)`: числа задачи по итоговым кольцам граней (см. модуль)."""

    inverse_scale = Fraction(1, int(scale))
    slot_of_face = []
    order: dict = {}
    for frame_face, face_rings in zip(frame_faces, rings):
        region = layout.region_of(frame_face)
        slot_of_face.append(region)
        for ring in face_rings:
            for key in ring:
                order.setdefault((region, key), len(order))
    parent = list(range(len(order)))

    def root(node):
        while parent[node] != node:
            parent[node] = parent[parent[node]]
            node = parent[node]
        return node

    edges: dict = {}
    for frame_face, region, face_rings in zip(frame_faces, slot_of_face, rings):
        for ring in face_rings:
            size = len(ring)
            for at in range(size):
                a, b = ring[at], ring[(at + 1) % size]
                edges.setdefault(frozenset((a, b)), {}).setdefault(region, (a, b, bool(frame_face.is_fan)))
    for regions in edges.values():
        if len(regions) < 2:
            continue
        items = sorted(regions.items())
        for (region_a, (a, b, fan_a)), (region_b, (_a, _b, fan_b)) in zip(items, items[1:]):
            # Веер и полоса - разные острова (шов, как у прежнего закона), секторы одного веера и стык полос с равными фактами - один.
            if fan_a == fan_b and facts[(region_a, a)] == facts[(region_b, a)] and facts[(region_a, b)] == facts[(region_b, b)]:
                for key in (a, b):
                    first, second = root(order[(region_a, key)]), root(order[(region_b, key)])
                    if first != second:
                        parent[max(first, second)] = min(first, second)
    unknown_of: dict = {}
    slots: list = []
    for slot, number in order.items():
        top = root(number)
        if top not in unknown_of:
            unknown_of[top] = len(slots)
            slots.append([])
        slots[unknown_of[top]].append(slot)
    point_floats: dict = {}

    def chart_of(key):
        found = point_floats.get(key)
        if found is None:
            point = points[key]
            found = point_floats[key] = (sqrt_sum_binary64(point[0], factor=inverse_scale), sqrt_sum_binary64(point[1], factor=inverse_scale))
        return found

    position_rows, chart_rows, r_rows, s_rows, source_rows = [], [], [], [], []
    for members in slots:
        region, key = members[0]
        s, r = facts[(region, key)]
        position = positions[key]
        position_rows.extend((float(position.x), float(position.y), float(position.z)))
        chart_rows.extend(chart_of(key))
        r_rows.append(sqrt_sum_binary64(r, factor=inverse_scale))
        s_rows.append(sqrt_sum_binary64(s, factor=inverse_scale))
        source_rows.append(r.is_zero)
    starts, corners, stations = [0], [], []
    for frame_face, region, face_rings in zip(frame_faces, slot_of_face, rings):
        s0, x0, y0, cx, cy = _run_station(frame_face.run, inverse_scale)
        for ring in face_rings:
            for key in ring:
                corners.append(unknown_of[root(order[(region, key)])])
                x, y = chart_of(key)
                stations.append(s0 + (x - x0) * cx + (y - y0) * cy)
            starts.append(len(corners))
    data = UnfoldInputV1(tuple(position_rows), tuple(chart_rows), tuple(r_rows), tuple(s_rows), tuple(source_rows), tuple(starts), tuple(corners), tuple(stations), hold_source)
    return data, slots


def map_straight_uv(frame_faces, layout, rings, points, facts, positions, scale: int, hold_source: bool = True):
    """`(числители UV по (регион, ключ), формы тех же слотов, числа)` под законом `UNFOLD_ALONG_U_V1`.

    `rings` - кольца итоговых граней по слитым граням (после прохода силуэта), `points` - точки карты, `facts` - факты `(s, r)`, `positions` - 3D позиции вершин, `scale` - решётка.
    """

    data, slots = unfold_input(frame_faces, layout, rings, points, facts, positions, scale, hold_source)
    result = backend.unfold_compute(unfold, data)
    values: dict = {}
    forms: dict = {}
    for number, members in enumerate(slots):
        s, r = facts[members[0]]
        if result.held[number]:
            value = s
            form = ((s, *_ANCHORED),)
        else:
            value = SqrtSumV1.rational(Fraction(result.u[number]) * int(scale))
            form = ((value, _exact(result.slope_x[number]), _exact(result.slope_y[number]), ZERO),)
        for slot in members:
            values[slot] = (value, facts[slot][1])
            forms[slot] = form
    return values, forms, result.counters


__all__ = ("map_straight_uv", "unfold_input")
