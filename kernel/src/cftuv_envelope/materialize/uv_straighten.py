"""Закон `UNFOLD_ALONG_U_V1` (режим Straight strips, политика `UV_DIRECT_STRIP_V1`): ряды прямые, `u` развёрнута по геометрии, как Straighten + Unfold along U в Maya.

Физические факты `(s, r)` (`mesh.station_values`) не трогаются: регионы, цепи, станции батча и решения топологии читают их, как раньше. Здесь считаются ТОЛЬКО числители UV `(U, r)`:
`u = U / alpha`, `v = r / alpha` (`uv_law.uv_direct_strip_v1`), тем же путём `uv_numerators`, что у карт Uniform.

* `v` НЕ ТРОНУТА: `r / alpha`, источник на `v = 0`, фронт на `v = 1`, у гребня верх неровный (высота трима постоянна).
* ВХОД НЕ ЗАВИСИТ ОТ ЗАКОНА ТОПОЛОГИИ, РЕЗКИ И ПОДЪЁМА: треугольники - канонические `TRIANGLES_V1` слитых граней (`mesh.canonical_triangles`, до закона топологии,
  резки и силуэта), позиции - решёточный подъём (до `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1`). У вершины ОДНА UV, любой закон её читает, растворённая силуэтом выпадает.
  Вершина, которой нет среди канонических (`clip:` резки), берёт `u` линейно в каноническом треугольнике своего региона, где лежит её точка карты (названо числом;
  поиск по всем треугольникам региона в порядке граней, поэтому ответ не зависит от того, какая выпущенная грань её несёт).
  Цена: выпущенный многоугольник Blender интерполирует своими треугольниками, а не решёнными (неаффинность внутри грани, замер - DECISIONS: UNFOLD_ALONG_U_V1).
* НЕИЗВЕСТНАЯ - вершина региона `(регион, ключ)`; вершины разных регионов одного ключа - одна неизвестная, если у ребра канонических треугольников этих регионов факты обоих концов
  ТОЧНО равны и обе грани - полосы либо обе - веера (там батч шва не ставит: `assemble._chains_from`), то есть остров - связная часть граней без шва: одна pChain - один
  остров, веер - свой остров, как у прежнего закона.
* `u` - решение ядра `uv_unfold.unfold` по каноническим треугольникам в решёточных 3D позициях; источник держится (`hold_source`): у вершины `r = 0` `U = s`
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
from math import ceil, hypot

from .uv_unfold import UnfoldInputV1, polyfill, unfold

#: Вершин, взявших `u` линейно в каноническом треугольнике (их нет среди канонических: `clip:` резки), и из них - чья точка карты ни в одном треугольнике
#: своей грани (взят ближайший по наименьшей барицентрической координате, она записана в числе ниже).
INTERPOLATED = "MATERIALIZE_STRAIGHTEN_UNFOLD_INTERPOLATED_VERTICES"
INTERPOLATED_OUTSIDE = "MATERIALIZE_STRAIGHTEN_UNFOLD_INTERPOLATED_OUTSIDE"
#: Выпущенные грани (их Blender интерполирует СВОИМИ треугольниками, `uv_unfold.polyfill` в карте): треугольники, чья площадь `(U, r)` против большинства региона, их
#: площадь в мм², и наибольшая разность `(U, r)` в центре многоугольника от четырёх вершин между его треугольниками и решением (микрометры; делить на alpha - единицы UV).
#: Зависят от закона топологии по построению, ответа не меняют: это мера, а не решение.
EMITTED_FLIPPED = "MATERIALIZE_STRAIGHTEN_UV_EMITTED_FLIPPED_TRIANGLES"
EMITTED_FLIPPED_AREA = "MATERIALIZE_STRAIGHTEN_UV_EMITTED_FLIPPED_AREA_MM2"
CENTROID_GAP = "MATERIALIZE_STRAIGHTEN_UV_NGON_CENTROID_MAX_MICROMETRES"

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


def _chart(points, key, inverse_scale):
    point = points[key]
    return sqrt_sum_binary64(point[0], factor=inverse_scale), sqrt_sum_binary64(point[1], factor=inverse_scale)


def _located(triangles, chart, target):
    """`(треугольник, барицентрические координаты)` с наибольшей наименьшей координатой: содержащий точку, если такой есть."""

    best = None
    for triangle in triangles:
        (ax, ay), (bx, by), (cx, cy) = (chart(key) for key in triangle)
        doubled = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)
        if doubled == 0.0:
            continue
        px, py = target
        lb = ((px - ax) * (cy - ay) - (cx - ax) * (py - ay)) / doubled
        lc = ((bx - ax) * (py - ay) - (px - ax) * (by - ay)) / doubled
        weights = (1.0 - lb - lc, lb, lc)
        if best is None or min(weights) > min(best[1]):
            best = (triangle, weights)
    return best


def map_straight_uv(frame_faces, layout, canonical, rings, points, facts, final_facts, positions, scale: int, hold_source: bool = True):
    """`(числители UV по (регион, ключ), формы тех же слотов, числа)` под законом `UNFOLD_ALONG_U_V1`.

    `canonical` - канонические треугольники слитых граней (`mesh.canonical_triangles`), `rings` - кольца выпущенных граней (им нужна UV), `points` - точки карты,
    `facts` - факты `(s, r)` всех вершин (канонических и резки), `final_facts` - факты выпущенных слотов, `positions` - решёточные 3D позиции, `scale` - решётка.
    """

    data, slots = unfold_input(frame_faces, layout, canonical, points, facts, positions, scale, hold_source)
    result = backend.unfold_compute(unfold, data)
    solved: dict = {}
    for number, members in enumerate(slots):
        for slot in members:
            solved[slot] = number
    inverse_scale = Fraction(1, int(scale))
    charts: dict = {}

    def chart(key):
        found = charts.get(key)
        if found is None:
            found = charts[key] = _chart(points, key, inverse_scale)
        return found

    by_region: dict = {}
    for frame_face, face_triangles in zip(frame_faces, canonical):
        by_region.setdefault(layout.region_of(frame_face), []).extend(face_triangles)
    values: dict = {}
    forms: dict = {}
    interpolated = outside = 0
    for frame_face, face_rings in zip(frame_faces, rings):
        region = layout.region_of(frame_face)
        for ring in face_rings:
            for key in ring:
                slot = (region, key)
                if slot in values:
                    continue
                r = final_facts[slot][1]
                number = solved.get(slot)
                if number is not None:
                    if result.held[number]:
                        value = facts[slot][0]
                        form = ((value, *_ANCHORED),)
                    else:
                        value = SqrtSumV1.rational(Fraction(result.u[number]) * int(scale))
                        form = ((value, _exact(result.slope_x[number]), _exact(result.slope_y[number]), ZERO),)
                    values[slot], forms[slot] = (value, r), form
                    continue
                interpolated += 1
                located = _located(by_region[region], chart, chart(key))
                if located is None:
                    # Грань без треугольника с площадью: станция как есть, вершина неподвижна по ширине.
                    outside += 1
                    values[slot], forms[slot] = (facts[slot][0], r), ((facts[slot][0], *_ANCHORED),)
                    continue
                triangle, weights = located
                if min(weights) < 0.0:
                    outside += 1
                us = [result.u[solved[(region, item)]] for item in triangle]
                u = weights[0] * us[0] + weights[1] * us[1] + weights[2] * us[2]
                (ax, ay), (bx, by), (cx, cy) = (chart(item) for item in triangle)
                doubled = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)
                gx = ((us[1] - us[0]) * (cy - ay) - (us[2] - us[0]) * (by - ay)) / doubled
                gy = ((us[2] - us[0]) * (bx - ax) - (us[1] - us[0]) * (cx - ax)) / doubled
                value = SqrtSumV1.rational(Fraction(u) * int(scale))
                values[slot], forms[slot] = (value, r), ((value, _exact(gx), _exact(gy), ZERO),)
    emitted = _emitted_measures(frame_faces, layout, rings, by_region, values, facts, chart, solved, result, inverse_scale)
    # Числа выпущенных граней есть всегда, и нулём: у закона треугольников нет многоугольников, и ключи счётчиков у законов топологии одни и те же.
    counters = result.counters + tuple(item for item in ((INTERPOLATED, interpolated), (INTERPOLATED_OUTSIDE, outside)) if item[1]) + emitted
    return values, forms, counters


def _emitted_measures(frame_faces, layout, rings, by_region, values, facts, chart, solved, result, inverse_scale):
    """`((EMITTED_FLIPPED, n), (EMITTED_FLIPPED_AREA, мм²), (CENTROID_GAP, мкм))` по выпущенным граням (см. имена)."""

    def numerators(slot):
        return sqrt_sum_binary64(values[slot][0], factor=inverse_scale), sqrt_sum_binary64(values[slot][1], factor=inverse_scale)

    def solved_at(region, point):
        located = _located(by_region[region], chart, point)
        if located is None or min(located[1]) < 0.0:
            return None
        triangle, weights = located
        u = r = 0.0
        for key, weight in zip(triangle, weights):
            u += weight * result.u[solved[(region, key)]]
            r += weight * sqrt_sum_binary64(facts[(region, key)][1], factor=inverse_scale)
        return u, r

    signed = []
    balance: dict = {}
    gap = 0.0
    for frame_face, face_rings in zip(frame_faces, rings):
        region = layout.region_of(frame_face)
        for ring in face_rings:
            points = [chart(key) for key in ring]
            uv = [numerators((region, key)) for key in ring]
            doubled = sum(points[at][0] * points[(at + 1) % len(ring)][1] - points[(at + 1) % len(ring)][0] * points[at][1] for at in range(len(ring)))
            orient = 1.0 if doubled >= 0.0 else -1.0
            triangles = polyfill(points)
            for a, b, c in triangles:
                area = orient * ((uv[b][0] - uv[a][0]) * (uv[c][1] - uv[a][1]) - (uv[c][0] - uv[a][0]) * (uv[b][1] - uv[a][1]))
                signed.append((region, area))
                balance[region] = balance.get(region, 0.0) + area
            if len(ring) < 4 or not all((region, key) in solved for key in ring):
                continue
            centre = (sum(p[0] for p in points) / len(ring), sum(p[1] for p in points) / len(ring))
            exact = solved_at(region, centre)
            if exact is None:
                continue
            for a, b, c in triangles:
                found = _located([(0, 1, 2)], lambda index, tri=(a, b, c): points[tri[index]], centre)
                if found is None or min(found[1]) < 0.0:
                    continue
                weights = found[1]
                u = weights[0] * uv[a][0] + weights[1] * uv[b][0] + weights[2] * uv[c][0]
                r = weights[0] * uv[a][1] + weights[1] * uv[b][1] + weights[2] * uv[c][1]
                gap = max(gap, hypot(u - exact[0], r - exact[1]))
                break
    flipped = 0
    flipped_area = 0.0
    for region, area in signed:
        if area != 0.0 and (area > 0.0) != (balance[region] >= 0.0):
            flipped += 1
            flipped_area += abs(area) * 0.5
    return ((EMITTED_FLIPPED, flipped), (EMITTED_FLIPPED_AREA, int(ceil(flipped_area * 1e6))), (CENTROID_GAP, int(ceil(gap * 1e6))))


__all__ = ("map_straight_uv", "unfold_input")
