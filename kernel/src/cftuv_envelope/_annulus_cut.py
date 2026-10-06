"""Разрез кольца носителя полосовой карты: путь, две копии вершин, голономия и два судимых числа.

Модуль внутренний и ничем не владеет: носитель выбирает `_band_support`, развёртку полосы делает
`_developable.build_developable_chart` (шарнир, ARAP, решётка, суд растяжения) на ДИСКЕ, а здесь кольцо превращается
в диск. Носитель вокруг замкнутой цепи — кольцо (`chi = 0`, две граничные петли), и карта у него одна: РАЗВЁРТКА, то
есть диск. Кольцо режется по пути рёбер источника, и вершины пути раздваиваются.

ВЕРШИНА РАЗРЕЗА `v*` — начало первого по имени выбранного `ChainUse` обода (`rim_edges[0][0]`). Замкнутый поток мягких
изломов обода размыкается ровно там (`stations.FLOW_CYCLE_OPENED`: наименьшее по имени вхождение, и счёт `s` начинается
заново с его начала), поэтому геометрический разрез и единственный шов UV замкнутого потока совпадают: шов один
(`U_RESTARTS_AT_CLOSED_FLOW_OPENING`), а не два.

ПУТЬ идёт от `v*` по ВНУТРЕННИМ рёбрам МЕЖДУ ГРАНЯМИ (диагональ грани не ребро источника, и разрез по ней резал бы грань
пополам) до ближайшей вершины дальней граничной петли, в обход прочих граничных вершин: кратчайший по числу рёбер, а
среди кратчайших — самый ровный (первое ребро ближе всего к биссектрисе угла `v*`, дальше — наименьший излом), и лишь
затем по имени. На решётке вращения это образующая, то есть биссектриса угла обода. Выбор пути — предложение: власть —
два числа сертификата, и их судит реестр.

ДВЕ КОПИИ. Вершина пути получает правую копию `<вершина>|cut:R` (`CUT_RIGHT_COPY_MARK`); левая копия носит исходное имя.
Угол (треугольник, вершина) принадлежит правой копии, если его место в веере вершины — справа от направленного пути
(треугольник со стороной против хода пути лежит справа). Грани целиком по одну сторону (путь идёт по рёбрам между
гранями), поэтому циклы граней переименовываются тем же отображением (`chart_cycle`). Диск полосы доказывает
`_unfold.owner_topology` на переименованных треугольниках.

ГОЛОНОМИЯ И ШОВ. Копии пути на карте разведены; лучшее движение плоскости `p -> R p + t` (наименьшие квадраты по парам
копий, формула Кабша) переводит левую копию в правую, а наибольшее остаточное расхождение — `PERIODIC_CUT_SEAM_RESIDUAL`.
Арифметика точная: норма `(S_d, S_c)` — точный рациональный корень, когда он существует (сдвиг цилиндра: `R = 1` и
остаток нуль), иначе один `sqrt` binary64 (воспроизводимо побитово), а всё остальное — дроби.

ШОВ МЕРИТСЯ ТАМ, ГДЕ ДЕКАЛЬ. Фронт ширины `alpha <= cap` не уходит от обода дальше `cap`, поэтому у полосы со стеной
досягаемости шов считается по ПРЕФИКСУ пути: вершины, чья хорда от `v*` на карте не больше `cap` (не меньше двух). Дальше по
пути, до стены досягаемости, кривизна растит расхождение (`~K L^2`) там, где декали нет и не может быть, и число судило бы
не шов, а ширину носителя. У целого кольца без стены `alpha` не ограничена, и шов - по всему пути.

ДВЕ ЗАПИСАННЫЕ ВЕЛИЧИНЫ судятся ТОЧНО: шов — квадратом метров против `SEAM_RESIDUAL_BOUND`, отклонение разреза от биссектрисы —
без корней: `sin^2 d <= NOISE_DIRECTION_SINE_BOUND^2` через `N^2 >= (1 - 2 b^2)^2 D`. Записывается верхняя граница числа
(целочисленный корень, шаг `2^-64`).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, replace
from fractions import Fraction

from ._cpython311 import left_fold_sum
from ._unfold import refusal
from .contracts.metric import CUT_RIGHT_COPY_MARK, NO_CHAIN_USE_ROLES, source_vertex_of_chart_vertex
from .ids import PhysicalEdgeId, SourceVertexId
from .outcomes import NamedOutcome

#: Допуск шва разреза, метры: после лучшего движения плоскости две копии пути (в пределах `cap` от обода) расходятся не
#: больше. Запись реестра допусков `PERIODIC_CUT_SEAM_RESIDUAL_BOUND_V1` (2 мм: при ширине декали 0.25 м это один процент
#: ширины). ИЗМЕРЕНО (сфера радиуса 1 м, обод - экватор): `cap = 1/4`, рёбра 0.2 м - 0.42 мм; `cap = 1/5`, рёбра 0.1 м - 0.90 мм (шов
#: растёт с числом вершин в досягаемости: мельче сетка - дальше мерится ТА ЖЕ кривая, и шов честно больше, а не меньше);
#: `cap = 1/2`, рёбра 0.2 м - 7.6 мм (отказ по числу: полоса шириной 0.8 м на метровой сфере не карта); цилиндр - нуль, конус -
#: до ячейки решётки. Первая оценка плана, 0.5 мм («сфера ~1e-4 м»), мерила разность длин соседних рёбер обода у разреза, а не
#: расхождение пути в досягаемости, и на мелкой сфере была бы отказом лучшей сетке при приёмке грубой.
SEAM_RESIDUAL_BOUND = Fraction(1, 500)

#: Разряды целочисленного корня верхней границы отклонения (`2^-64` относительно): граница, а не допуск.
_ROOT_BITS = 64


def right_copy(vertex_id) -> SourceVertexId:
    """Правая копия вершины разреза: имя вершины источника и суффикс."""

    return SourceVertexId(vertex_id.value + CUT_RIGHT_COPY_MARK)


def is_right_copy(vertex_id) -> bool:
    return vertex_id.value.endswith(CUT_RIGHT_COPY_MARK)


source_vertex_of = source_vertex_of_chart_vertex


@dataclass(frozen=True, slots=True)
class RingStripV1:
    """Полоса-диск из кольца: путь, переименованные треугольники и углы, ставшие правыми копиями."""

    path: tuple
    triangles: tuple
    right_corners: frozenset

    def right_vertices(self) -> frozenset:
        return frozenset(right_copy(vertex) for vertex in self.path)


def boundary_loops(topology) -> list:
    """Граничные петли топологии: вершины каждой в порядке обхода (внутренность слева), петли по имени первой вершины."""

    following = {}
    for triangle_id, ordinal in topology.boundary_sides:
        triangle = topology.by_id[triangle_id]
        following[triangle.vertex_ids[ordinal]] = triangle.vertex_ids[(ordinal + 1) % 3]
    seen: set = set()
    loops = []
    for start in sorted(following, key=lambda item: item.value):
        if start in seen:
            continue
        loop = []
        cursor = start
        while cursor not in seen:
            seen.add(cursor)
            loop.append(cursor)
            cursor = following[cursor]
        loops.append(tuple(loop))
    return loops


def _unavailable(detail: str):
    return refusal(NamedOutcome.PERIODIC_CUT_PATH_UNAVAILABLE, detail)


def _between_face_adjacency(topology) -> dict:
    """`{вершина: [соседи]}` по внутренним рёбрам, у которых два треугольника лежат в РАЗНЫХ гранях источника."""

    adjacency: dict = {}
    for (triangle_id, ordinal), other in topology.opposite.items():
        if other is None:
            continue
        triangle = topology.by_id[triangle_id]
        if triangle.source_face_id == topology.by_id[other[0]].source_face_id:
            continue
        first = triangle.vertex_ids[ordinal]
        second = triangle.vertex_ids[(ordinal + 1) % 3]
        adjacency.setdefault(first, set()).add(second)
        adjacency.setdefault(second, set()).add(first)
    return {vertex: sorted(near, key=lambda item: item.value) for vertex, near in adjacency.items()}


def _unit(vector):
    length = math.sqrt(left_fold_sum(axis * axis for axis in vector))
    return tuple(axis / length for axis in vector) if length else None


def _direction(positions, first, second):
    return _unit(tuple(float(b) - float(a) for a, b in zip(positions[first], positions[second], strict=True)))


def _bisector_cost(positions, apex, out_next, in_prev, candidate) -> float:
    """Первое ребро: `|c . (u_out - u_in)|` - синус отклонения от биссектрисы (точно для плоского веера)."""

    out, back, step = (_direction(positions, apex, item) for item in (out_next, in_prev, candidate))
    if out is None or back is None or step is None:
        return math.inf
    return abs(left_fold_sum(c * (a - b) for c, a, b in zip(step, out, back, strict=True)))


def _bend_cost(positions, previous, current, candidate) -> float:
    """Следующие рёбра: `1 - cos` излома относительно предыдущего ребра."""

    before, after = _direction(positions, previous, current), _direction(positions, current, candidate)
    if before is None or after is None:
        return math.inf
    return 1.0 - left_fold_sum(a * b for a, b in zip(before, after, strict=True))


def ring_cut_path(topology, positions, rim_edges) -> tuple:
    """Путь разреза кольца от вершины `v*` обода к дальней петле: вершины пути, `v*` первая; либо именованный отказ."""

    loops = boundary_loops(topology)
    rim_pairs = set(rim_edges)
    rim_loops = [
        loop
        for loop in loops
        if rim_pairs & {(loop[index], loop[(index + 1) % len(loop)]) for index in range(len(loop))}
    ]
    if len(rim_loops) != 1 or not rim_edges:
        raise _unavailable(
            f"the rim of the ring lies on {len(rim_loops)} of its {len(loops)} boundary loops: the cut runs from "
            "the one loop that carries the rim to the other"
        )
    rim_loop = rim_loops[0]
    far_loop = next(loop for loop in loops if loop is not rim_loop)
    start = rim_edges[0][0]
    if start not in rim_loop:
        raise _unavailable(f"the cut vertex {start.value} is not on the boundary loop that carries the rim")
    rim_set, far_set = set(rim_loop), set(far_loop)
    adjacency = _between_face_adjacency(topology)
    hops = {vertex: 0 for vertex in far_loop}
    queue = deque(sorted(far_loop, key=lambda item: item.value))
    while queue:
        current = queue.popleft()
        for near in adjacency.get(current, ()):
            if near not in hops and near not in rim_set:
                hops[near] = hops[current] + 1
                queue.append(near)
    options = [near for near in adjacency.get(start, ()) if near in hops and near not in rim_set]
    if not options:
        raise _unavailable(
            f"no chain of edges between faces leads from the cut vertex {start.value} to the far boundary loop "
            "around the other boundary vertices"
        )
    index = rim_loop.index(start)
    out_next = rim_loop[(index + 1) % len(rim_loop)]
    in_prev = rim_loop[index - 1]
    path = [start]
    remaining = min(hops[near] for near in options) + 1
    while remaining:
        current = path[-1]
        candidates = [
            near
            for near in adjacency.get(current, ())
            if near not in rim_set and near not in path and hops.get(near) == remaining - 1
        ]
        if not candidates:
            raise _unavailable(f"the cut path stops at {current.value} before it reaches the far boundary loop")
        if len(path) == 1:
            cost = lambda near: _bisector_cost(positions, start, out_next, in_prev, near)
        else:
            cost = lambda near: _bend_cost(positions, path[-2], current, near)
        path.append(min(candidates, key=lambda near: (cost(near), near.value)))
        remaining -= 1
    if path[-1] not in far_set:
        raise _unavailable("the cut path ends off the far boundary loop")
    return tuple(path)


def _components(links: dict) -> list:
    seen: set = set()
    parts = []
    for first in sorted(links, key=lambda item: item.value):
        if first in seen:
            continue
        seen.add(first)
        part = {first}
        queue = deque([first])
        while queue:
            for near in links[queue.popleft()]:
                if near not in seen:
                    seen.add(near)
                    part.add(near)
                    queue.append(near)
        parts.append(part)
    return parts


def cut_strip(topology, path) -> RingStripV1:
    """Кольцо, разрезанное по пути: треугольники с правыми копиями вершин пути, либо именованный отказ."""

    owner = {}
    for triangle in topology.triangles:
        for ordinal in range(3):
            owner[(triangle.vertex_ids[ordinal], triangle.vertex_ids[(ordinal + 1) % 3])] = triangle.triangle_id
    path_edges = {frozenset(pair) for pair in zip(path, path[1:])}
    right_corners: set = set()
    for index, vertex in enumerate(path):
        first, second = (vertex, path[index + 1]) if index + 1 < len(path) else (path[index - 1], vertex)
        seeds = {owner.get((second, first))}
        if index and index + 1 < len(path):
            seeds.add(owner.get((vertex, path[index - 1])))
        if None in seeds:
            raise _unavailable(f"the cut edge at {vertex.value} has no triangle on its right: it lies on the boundary")
        fan = topology.fans[vertex][0]
        links = {item: set() for item in fan}
        for triangle_id in fan:
            triangle = topology.by_id[triangle_id]
            for ordinal in range(3):
                pair = (triangle.vertex_ids[ordinal], triangle.vertex_ids[(ordinal + 1) % 3])
                other = topology.opposite[(triangle_id, ordinal)]
                if vertex in pair and other is not None and frozenset(pair) not in path_edges:
                    links[triangle_id].add(other[0])
        parts = _components(links)
        right = [part for part in parts if part & seeds]
        if len(parts) != 2 or len(right) != 1 or not seeds <= right[0]:
            raise _unavailable(
                f"the cut path does not split the fan of {vertex.value} into a left and a right part "
                f"({len(parts)} parts)"
            )
        right_corners.update((triangle_id, vertex) for triangle_id in right[0])
    return RingStripV1(tuple(path), rename_triangles(topology.triangles, right_corners), frozenset(right_corners))


def rename_triangles(triangles, right_corners) -> tuple:
    """Треугольники в именах карты: угол из `right_corners` называется правой копией вершины."""

    return tuple(
        replace(
            item,
            vertex_ids=tuple(
                right_copy(vertex) if (item.triangle_id, vertex) in right_corners else vertex
                for vertex in item.vertex_ids
            ),
        )
        for item in triangles
    )


def chart_side_ends(sides, start, end) -> tuple:
    """Имена карты концов ребра границы `start -> end` (вершины источника): у разреза кольца копии вершины разные.

    Ищется сторона границы полосы, не являющаяся краем разреза, с этими вершинами источника; ребра нет среди сторон -
    имена возвращаются как есть (ребро не граничное либо карта без разреза).
    """

    for side in sides:
        if side.role in NO_CHAIN_USE_ROLES:
            continue
        pair = (source_vertex_of(side.start_vertex_id), source_vertex_of(side.end_vertex_id))
        if pair == (start, end):
            return side.start_vertex_id, side.end_vertex_id
        if pair == (end, start):
            return side.end_vertex_id, side.start_vertex_id
    return start, end


def strip_of(triangles, cut) -> RingStripV1:
    """Полоса-диск по треугольникам носителя и записи разреза сертификата: то же переименование, что у построителя."""

    corners = frozenset((item.triangle_id, item.vertex_id) for item in cut.right_corners)
    return RingStripV1(tuple(cut.path_vertex_ids), rename_triangles(triangles, corners), corners)


@dataclass(frozen=True, slots=True)
class ChartFaceV1:
    """Грань носителя в именах карты: циклы вершин и рёбер, как их читает проверка границы подъёма."""

    face_id: object
    vertex_cycle: tuple
    edge_cycle: tuple


def chart_faces(faces, strip) -> tuple:
    """Грани носителя после разреза: вершины пути на правой стороне - правые копии, ребро разреза - своё у каждой стороны.

    Ребро разреза на поверхности одно, а на карте две стены: у грани справа оно называется `<ребро>|cut:R`, иначе обе
    стороны были бы одним физическим ребром с несовпадающими концами.
    """

    path_edges = frozenset(frozenset(pair) for pair in zip(strip.path, strip.path[1:]))
    result = []
    for face in faces:
        cycle = chart_cycle(face.vertex_cycle, face.triangle_ids, strip.right_corners)
        size = len(face.vertex_cycle)
        edges = tuple(
            PhysicalEdgeId(edge.value + CUT_RIGHT_COPY_MARK)
            if frozenset((face.vertex_cycle[index], face.vertex_cycle[(index + 1) % size])) in path_edges
            and is_right_copy(cycle[index])
            else edge
            for index, edge in enumerate(face.edge_cycle)
        )
        result.append(ChartFaceV1(face.face_id, cycle, edges))
    return tuple(result)


def chart_cycle(vertex_cycle, triangle_ids, right_corners) -> tuple:
    """Цикл грани в именах карты: вершина пути на правой стороне разреза заменяется правой копией.

    Грань целиком по одну сторону (путь идёт по рёбрам между гранями), поэтому достаточно одного её треугольника при
    вершине.
    """

    members = frozenset(triangle_ids)
    return tuple(
        right_copy(vertex)
        if any((triangle_id, vertex) in right_corners for triangle_id in members)
        else vertex
        for vertex in vertex_cycle
    )


def _rational_root(value: Fraction):
    root_n, root_d = math.isqrt(value.numerator), math.isqrt(value.denominator)
    if root_n * root_n == value.numerator and root_d * root_d == value.denominator:
        return Fraction(root_n, root_d)
    return None


def seam_prefix(nodes: dict, path, chart_scale: int, cap) -> tuple:
    """Начало пути, на котором шов судится: вершины с хордой от `v*` на карте не длиннее `cap` (не меньше двух); `cap=None` - весь путь.

    Хорда, а не длина вдоль пути: квадрат расстояния точен, корней нет, а путь разреза почти прямой.
    """

    if cap is None:
        return tuple(path)
    start = nodes[path[0]]
    limit = Fraction(cap) * chart_scale
    kept = 1
    for vertex in path[1:]:
        offset = (Fraction(nodes[vertex][0]) - start[0], Fraction(nodes[vertex][1]) - start[1])
        if offset[0] * offset[0] + offset[1] * offset[1] > limit * limit:
            break
        kept += 1
    return tuple(path[: max(kept, 2)])


def seam_fit(nodes: dict, path, chart_scale: int) -> tuple:
    """Лучшее движение `p -> R p + t` левой копии пути в правую и остаток: `(cos, sin, tx, ty, остаток^2 в м^2)`.

    `nodes` — узлы карты (единицы `1/chart_scale` метра). Метод Кабша: центроиды, `S_d = sum a.b`, `S_c = sum a x b`,
    `(cos, sin) = (S_d, S_c) / |(S_d, S_c)|`. Нулевая норма - пара облаков без поворота, лучшего движения нет: отказ.
    """

    pairs = [
        (tuple(Fraction(axis) for axis in nodes[vertex]), tuple(Fraction(axis) for axis in nodes[right_copy(vertex)]))
        for vertex in path
    ]
    count = len(pairs)
    left_center = tuple(sum((left[axis] for left, _ in pairs), Fraction(0)) / count for axis in range(2))
    right_center = tuple(sum((right[axis] for _, right in pairs), Fraction(0)) / count for axis in range(2))
    dot = cross = Fraction(0)
    for left, right in pairs:
        a = (left[0] - left_center[0], left[1] - left_center[1])
        b = (right[0] - right_center[0], right[1] - right_center[1])
        dot += a[0] * b[0] + a[1] * b[1]
        cross += a[0] * b[1] - a[1] * b[0]
    norm_squared = dot * dot + cross * cross
    if not norm_squared:
        raise refusal(
            NamedOutcome.PERIODIC_CUT_SEAM_RESIDUAL_EXCEEDED,
            "the two copies of the cut path define no rotation between them: the seam of the cut is undefined",
        )
    root = _rational_root(norm_squared)
    if root is not None:
        cosine, sine = dot / root, cross / root
    else:
        length = math.sqrt(float(norm_squared))
        cosine, sine = Fraction(float(dot) / length), Fraction(float(cross) / length)
    shift = (
        right_center[0] - (cosine * left_center[0] - sine * left_center[1]),
        right_center[1] - (sine * left_center[0] + cosine * left_center[1]),
    )
    worst = Fraction(0)
    for left, right in pairs:
        moved = (cosine * left[0] - sine * left[1] + shift[0], sine * left[0] + cosine * left[1] + shift[1])
        worst = max(worst, (right[0] - moved[0]) ** 2 + (right[1] - moved[1]) ** 2)
    return cosine, sine, shift[0], shift[1], worst / (chart_scale * chart_scale)


def _sub(first, second):
    return (first[0] - second[0], first[1] - second[1])


def _dot2(first, second) -> Fraction:
    return first[0] * second[0] + first[1] * second[1]


def _cross2(first, second) -> Fraction:
    return first[0] * second[1] - first[1] * second[0]


def bisector_numbers(nodes: dict, a, out_end, path_a, b, in_start, path_b, bound: Fraction) -> tuple:
    """`(верхняя граница sin^2 d, судья)` отклонения разреза от биссектрисы склеенного угла вершины разреза.

    Копия `a` начинает сторону границы `a -> out_end`, копия `b` кончает сторону `in_start -> b`; `path_a`, `path_b` -
    вторые вершины сторон разреза у каждой копии. Внутренний угол при `a` - от луча на `out_end` против часовой к лучу на
    `path_a`, при `b` - от луча на `path_b` к лучу на `in_start`; `d = (theta_a - theta_b) / 2`, `cos 2d = N / sqrt(D)`,
    `sin^2 d = (1 - cos 2d) / 2`. Судья точен: `sin^2 d <= bound^2  <=>  N > 0  и  N^2 >= (1 - 2 bound^2)^2 D`.
    """

    at = {name: tuple(Fraction(axis) for axis in nodes[name]) for name in (a, out_end, path_a, b, in_start, path_b)}
    out_a, step_a = _sub(at[out_end], at[a]), _sub(at[path_a], at[a])
    step_b, out_b = _sub(at[path_b], at[b]), _sub(at[in_start], at[b])
    squares = [_dot2(item, item) for item in (out_a, step_a, step_b, out_b)]
    if not all(squares):
        raise _unavailable("a side of the cut corner has zero length on the chart")
    numerator = _dot2(out_a, step_a) * _dot2(step_b, out_b) + _cross2(out_a, step_a) * _cross2(step_b, out_b)
    denominator = squares[0] * squares[1] * squares[2] * squares[3]
    scaled = denominator * (1 << (2 * _ROOT_BITS))
    root = math.isqrt(scaled.numerator // scaled.denominator)
    low, high = Fraction(root, 1 << _ROOT_BITS), Fraction(root + 1, 1 << _ROOT_BITS)
    if not low:
        raise _unavailable("the cut corner is too small on the chart to measure")
    cosine_low = numerator / high if numerator >= 0 else numerator / low
    upper = max(Fraction(0), (1 - cosine_low) / 2)
    passes = numerator > 0 and numerator * numerator >= (1 - 2 * bound * bound) ** 2 * denominator
    return upper, passes
