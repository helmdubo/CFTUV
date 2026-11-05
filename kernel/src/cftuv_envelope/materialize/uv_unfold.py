"""Числовое ядро закона `UNFOLD_ALONG_U_V1` (режим Straight strips): `u` острова по геометрии при неподвижной `v`, как Straighten + Unfold along U в Maya.

Вход - только числа (`UnfoldInputV1`): вершины-неизвестные (3D-позиция, точка карты, `r`, станция `s`, на источнике ли), кольца итоговых граней по неизвестным и линейная
станция пробега `s_run` в углах колец (только направление «вдоль цепи»). Выход - `u` в метрах на неизвестную, наклоны `u` по карте и числа. Тот же алгоритм повторяет
Rust (`native/cftuv-unfold`): только `+ - * /` и `sqrt` binary64, порядок сложений фиксирован, словари не участвуют в порядке.

* ТРЕУГОЛЬНИКИ - те, по которым Blender интерполирует UV грани (`mesh_calc_tessellation`): треугольник как есть; четырёхгранье - диагональ `0-2`, если `1` и `3` не по одну
  сторону от неё (`is_quad_flip_v3_first_third_fast`), иначе `1-3`; многоугольник - уши `BLI_polyfill_calc` (обход вперёд с вершины `0`, два прохода, «чёт» и «развёртка»). Знаки
  считаются в карте домена, а не во float32 Blender: на почти вырожденных углах выбор может разойтись, это измеряется, а не обещается.
* ЭНЕРГИЯ. В плоскости 3D-треугольника `t` направление `t̂` - перпендикуляр к `grad r` в сторону роста `s_run`; минимум `sum_t A_t |grad u - t̂|^2` - система Пуассона
  со свободной границей (котангенсная матрица). `ARAP_ITERATIONS` шагов local/global (ближайший поворот к `[grad u; grad r]`, `v` неподвижна) переиспользуют разложение.
* СЖАТИЕ. Треугольник, у которого `grad u . t̂` ниже `GUARD_FLOOR` либо выше `GUARD_CEILING` (цель 1: клин у излома цепи, который соседние полосы сжали бы в щель и
  вывернули, и его растянутый сосед), получает вес в `GUARD_FACTOR` раз больше, остров решается заново; не больше `GUARD_ROUNDS` раз, число перевзвешиваний названо.
* ВЫРОЖДЕННОЕ. Треугольник без площади (синус угла ниже `DEGENERATE_SINE`) и треугольник, где `s_run` идёт вдоль `grad r` (`UNORIENTED_SINE`), весят ноль; `grad r` ниже
  `FLAT_GRADIENT` - цель вдоль `grad s_run`. Всё названо числом.
* КАЛИБРОВКА. Источник держится (`hold_source`): `u = s` на `r = 0`, остальное - решение с этими условиями. Остров без вершины источника и свободный ряд - сдвиг, при котором
  сумма квадратов `u - s` по вершинам источника (нет их - по всем) наименьшая, после решения с первой вершиной в нуле.
* РЕШАТЕЛЬ. Обратный Катхилл-Макки (старт - наименьшая степень, затем номер; соседи по степени, затем номеру), профильный Холецкий; неположительный ведущий элемент -
  остров на физической станции, назван.
* ВЫВОРОТ. Знак UV-площади треугольника (`u`, `r`), умноженный на знак кольца в карте, против большинства острова по площади: вывернутые и их площадь названы.
* НАТИВНЫЙ ПОРТ. Ответ Rust равен ответу этого модуля в пределах `NATIVE_EQUALITY` (`results_agree`; реестр допусков, `UNFOLD_NATIVE_EQUALITY_V1`).
"""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, sqrt

#: Синус угла треугольника, ниже которого у него нет площади: вес ноль.
DEGENERATE_SINE = 1e-9
#: `|grad r|` ниже - `r` в треугольнике постоянна: цель вдоль `grad s_run`.
FLAT_GRADIENT = 1e-6
#: Доля `grad s_run` поперёк `grad r`, ниже которой направление вдоль цепи не определено: вес ноль.
UNORIENTED_SINE = 1e-6
#: Шаги local/global после линейного решения (решение 2026-11-06, замер в DECISIONS: UNFOLD_ALONG_U_V1).
ARAP_ITERATIONS = 5
#: Перевзвешиваний сжатых и растянутых треугольников, пределы `grad u . t̂` (цель 1) и множитель веса.
GUARD_ROUNDS = 4
GUARD_FLOOR = 0.5
GUARD_CEILING = 2.0
GUARD_FACTOR = 4.0
#: Наибольшее расхождение `u` (метры) и наклонов нативного порта с этим модулем; `held` и числа - точно (`UNFOLD_NATIVE_EQUALITY_V1`).
NATIVE_EQUALITY = 1e-9

TRIANGLES = "MATERIALIZE_STRAIGHTEN_UNFOLD_TRIANGLES"
DEGENERATE = "MATERIALIZE_STRAIGHTEN_UNFOLD_DEGENERATE_TRIANGLES"
FLAT = "MATERIALIZE_STRAIGHTEN_UNFOLD_FLAT_R_TRIANGLES"
UNORIENTED = "MATERIALIZE_STRAIGHTEN_UNFOLD_UNORIENTED_TRIANGLES"
ISLANDS = "MATERIALIZE_STRAIGHTEN_UNFOLD_ISLANDS"
WITHOUT_SOURCE = "MATERIALIZE_STRAIGHTEN_UNFOLD_ISLANDS_WITHOUT_SOURCE"
ISOLATED = "MATERIALIZE_STRAIGHTEN_UNFOLD_ISOLATED_VERTICES"
SINGULAR = "MATERIALIZE_STRAIGHTEN_UNFOLD_SINGULAR_ISLANDS"
GUARDED = "MATERIALIZE_STRAIGHTEN_UNFOLD_REWEIGHTED_TRIANGLES"
FLIPPED = "MATERIALIZE_STRAIGHTEN_UV_FLIPPED_TRIANGLES"
FLIPPED_AREA = "MATERIALIZE_STRAIGHTEN_UV_FLIPPED_AREA_MM2"
ZERO_AREA = "MATERIALIZE_STRAIGHTEN_UV_ZERO_AREA"
SLOPE_SKIPPED = "MATERIALIZE_STRAIGHTEN_UNFOLD_SLOPE_SKIPPED_TRIANGLES"

COUNTER_NAMES = (TRIANGLES, DEGENERATE, FLAT, UNORIENTED, ISLANDS, WITHOUT_SOURCE, ISOLATED, SINGULAR, GUARDED, FLIPPED, FLIPPED_AREA, ZERO_AREA, SLOPE_SKIPPED)


@dataclass(frozen=True, slots=True)
class UnfoldInputV1:
    """Числа задачи. `positions` - `x, y, z` подряд, `chart` - `x, y` подряд (метры), остальные массивы по неизвестным; `ring_starts` - начала колец в `corners`
    (последний элемент - длина), `corner_station` - `s_run` угла."""

    positions: tuple
    chart: tuple
    r: tuple
    s: tuple
    source: tuple
    ring_starts: tuple
    corners: tuple
    corner_station: tuple
    hold_source: bool = True
    iterations: int = ARAP_ITERATIONS
    guard_rounds: int = GUARD_ROUNDS


@dataclass(frozen=True, slots=True)
class UnfoldResultV1:
    """`u` (метры) и наклоны `u` по карте `(du/dx, du/dy)` на неизвестную, `held` - взята ли станция как есть, и числа (только ненулевые, в порядке `COUNTER_NAMES`)."""

    u: tuple
    slope_x: tuple
    slope_y: tuple
    held: tuple
    counters: tuple


def _cross(ax, ay, bx, by):
    return ax * by - ay * bx


def _sign(value):
    return 1 if value > 0.0 else (-1 if value < 0.0 else 0)


def _doubled_area(points) -> float:
    doubled = 0.0
    size = len(points)
    for at in range(size):
        following = points[(at + 1) % size]
        doubled += points[at][0] * following[1] - following[0] * points[at][1]
    return doubled


def polyfill(points) -> list:
    """Треугольники `(i, j, k)` кольца точек карты так, как их режет Blender (см. модуль); кольцо меньше трёх - пусто."""

    size = len(points)
    if size < 3:
        return []
    if size == 3:
        return [(0, 1, 2)]
    if size == 4:
        v1, v2, v3, v4 = points
        d12 = (v2[0] - v1[0], v2[1] - v1[1])
        d13 = (v3[0] - v1[0], v3[1] - v1[1])
        d14 = (v4[0] - v1[0], v4[1] - v1[1])
        if _cross(d12[0], d12[1], d13[0], d13[1]) * _cross(d14[0], d14[1], d13[0], d13[1]) > 0.0:
            return [(0, 1, 3), (1, 2, 3)]
        return [(0, 1, 2), (0, 2, 3)]
    orient = 1.0 if _doubled_area(points) >= 0.0 else -1.0

    def span(a, b, c):
        # `span_tri_v2_sign` в ориентации Blender: выпуклая вершина `b` между `a` и `c` даёт +1
        pa, pb, pc = points[a], points[b], points[c]
        return _sign(orient * _cross(pb[0] - pa[0], pb[1] - pa[1], pc[0] - pb[0], pc[1] - pb[1]))

    following = [(at + 1) % size for at in range(size)]
    previous = [(at - 1) % size for at in range(size)]
    sign = [span(previous[at], at, following[at]) for at in range(size)]
    concave = sum(1 for value in sign if value != 1)
    head = 0
    count = size
    found = []

    def tip(ear, accept) -> bool:
        if concave == 0:
            return True
        if sign[ear] != accept:
            return False
        a, b, c = previous[ear], ear, following[ear]
        at = following[c]
        while at != a:
            if sign[at] != 1:
                if span(b, c, at) != -1 and span(c, a, at) != -1 and span(a, b, at) != -1:
                    return False
            at = following[at]
        return True

    ear_init = head
    reverse = False
    while count > 3:
        chosen = None
        for accept in (1, 0):
            ear = ear_init
            for _ in range(count):
                if tip(ear, accept):
                    chosen = ear
                    break
                ear = previous[ear] if reverse else following[ear]
            if chosen is not None:
                break
        if chosen is None:
            ear = ear_init
            for _ in range(count):
                if sign[ear] != -1:
                    chosen = ear
                    break
                ear = following[ear]
            if chosen is None:
                chosen = ear
        if sign[chosen] != 1:
            concave -= 1
        before, after = previous[chosen], following[chosen]
        found.append((before, chosen, after))
        following[before] = after
        previous[after] = before
        if head == chosen:
            head = after
        count -= 1
        for near in (before, after):
            if sign[near] != 1:
                sign[near] = span(previous[near], near, following[near])
                if sign[near] == 1:
                    concave -= 1
        ear_init = previous[before] if reverse else following[after]
        if sign[ear_init] != 1:
            ear_init = previous[ear_init] if reverse else following[ear_init]
            reverse = not reverse
    found.append((head, following[head], following[following[head]]))
    return found


class _Profile:
    """Профильный (оболочечный) Холецкий симметричной положительно определённой матрицы в порядке перестановки."""

    __slots__ = ("first", "rows")

    def __init__(self, first, rows):
        self.first = first
        self.rows = rows

    @classmethod
    def factor(cls, size, first, entries):
        """`entries[i]` - `{j: значение}` при `first[i] <= j <= i`; `None` - неположительный ведущий элемент."""

        rows = []
        for i in range(size):
            low = first[i]
            row = [0.0] * (i - low + 1)
            known = entries[i]
            for j in range(low, i + 1):
                total = known.get(j, 0.0)
                other_low = first[j]
                other = rows[j] if j < i else row
                start = low if low > other_low else other_low
                for k in range(start, j):
                    total -= row[k - low] * other[k - other_low]
                if j < i:
                    row[j - low] = total / other[j - other_low]
                else:
                    if not total > 0.0:
                        return None
                    row[i - low] = sqrt(total)
            rows.append(row)
        return cls(first, rows)

    def solve(self, right):
        first, rows = self.first, self.rows
        size = len(rows)
        y = list(right)
        for i in range(size):
            low, row = first[i], rows[i]
            total = y[i]
            for k in range(low, i):
                total -= row[k - low] * y[k]
            y[i] = total / row[i - low]
        for i in range(size - 1, -1, -1):
            low, row = first[i], rows[i]
            y[i] = y[i] / row[i - low]
            value = y[i]
            for k in range(low, i):
                y[k] -= row[k - low] * value
        return y


def _order(nodes, neighbours):
    """Обратный Катхилл-Макки по `nodes` (номера по возрастанию), соседи - только из `nodes`."""

    inside = set(nodes)
    degree = {node: sum(1 for other in neighbours[node] if other in inside and other != node) for node in nodes}
    seen = set()
    order = []
    for start in sorted(nodes, key=lambda node: (degree[node], node)):
        if start in seen:
            continue
        seen.add(start)
        queue = [start]
        at = 0
        while at < len(queue):
            node = queue[at]
            at += 1
            order.append(node)
            for other in sorted((item for item in neighbours[node] if item in inside and item not in seen), key=lambda item: (degree[item], item)):
                seen.add(other)
                queue.append(other)
    order.reverse()
    return order


def _triangle(points3, values_r, values_s):
    """`(площадь, градиенты барицентрических функций в рамке (t̂, n̂r), вид)` либо `(0, None, вид)`; вид - 0 обычный, 1 вырожденный, 2 плоская `r`, 3 без направления."""

    p0, p1, p2 = points3
    e1 = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    e2 = (p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2])
    l1 = sqrt(e1[0] * e1[0] + e1[1] * e1[1] + e1[2] * e1[2])
    l2 = sqrt(e2[0] * e2[0] + e2[1] * e2[1] + e2[2] * e2[2])
    if not (l1 > 0.0 and l2 > 0.0):
        return 0.0, None, 1
    x2 = (e2[0] * e1[0] + e2[1] * e1[1] + e2[2] * e1[2]) / l1
    rest = l2 * l2 - x2 * x2
    y2 = sqrt(rest) if rest > 0.0 else 0.0
    if not y2 > DEGENERATE_SINE * l2:
        return 0.0, None, 1
    doubled = l1 * y2
    # Плоская рамка: q0 = (0, 0), q1 = (l1, 0), q2 = (x2, y2).
    gradients = (((0.0 - y2) / doubled, (x2 - l1) / doubled), (y2 / doubled, (0.0 - x2) / doubled), (0.0, l1 / doubled))
    rx = values_r[0] * gradients[0][0] + values_r[1] * gradients[1][0] + values_r[2] * gradients[2][0]
    ry = values_r[0] * gradients[0][1] + values_r[1] * gradients[1][1] + values_r[2] * gradients[2][1]
    sx = values_s[0] * gradients[0][0] + values_s[1] * gradients[1][0] + values_s[2] * gradients[2][0]
    sy = values_s[0] * gradients[0][1] + values_s[1] * gradients[1][1] + values_s[2] * gradients[2][1]
    length_r = sqrt(rx * rx + ry * ry)
    length_s = sqrt(sx * sx + sy * sy)
    kind = 0
    if length_r < FLAT_GRADIENT:
        if not length_s > 0.0:
            return 0.0, None, 3
        tx, ty = sx / length_s, sy / length_s
        nx, ny = 0.0 - ty, tx
        kind = 2
    else:
        nx, ny = rx / length_r, ry / length_r
        across = sx * (0.0 - ny) + sy * nx
        if not abs(across) > UNORIENTED_SINE * length_s:
            return 0.0, None, 3
        tx, ty = (0.0 - ny, nx) if across > 0.0 else (ny, 0.0 - nx)
    framed = tuple((gx * tx + gy * ty, gx * nx + gy * ny) for gx, gy in gradients)
    return 0.5 * doubled, framed, kind


def unfold(data: UnfoldInputV1) -> UnfoldResultV1:
    """Решение закона (см. модуль)."""

    count = len(data.r)
    numbers = dict.fromkeys(COUNTER_NAMES, 0)
    flat_positions, flat_chart, values_r, values_s, source = data.positions, data.chart, data.r, data.s, data.source
    chart = [(flat_chart[2 * node], flat_chart[2 * node + 1]) for node in range(count)]
    starts, corners, stations = data.ring_starts, data.corners, data.corner_station
    triangles = []
    for ring in range(len(starts) - 1):
        low, high = starts[ring], starts[ring + 1]
        ids = corners[low:high]
        points = [chart[node] for node in ids]
        orient = 1.0 if _doubled_area(points) >= 0.0 else -1.0
        for a, b, c in polyfill(points):
            nodes = (ids[a], ids[b], ids[c])
            corner3 = tuple((flat_positions[3 * node], flat_positions[3 * node + 1], flat_positions[3 * node + 2]) for node in nodes)
            area, framed, kind = _triangle(corner3, tuple(values_r[node] for node in nodes), (stations[low + a], stations[low + b], stations[low + c]))
            if kind == 1:
                numbers[DEGENERATE] += 1
            elif kind == 2:
                numbers[FLAT] += 1
            elif kind == 3:
                numbers[UNORIENTED] += 1
            if framed is not None:
                numbers[TRIANGLES] += 1
            triangles.append((nodes, area, framed, orient))
    parent = list(range(count))

    def root(node):
        while parent[node] != node:
            parent[node] = parent[parent[node]]
            node = parent[node]
        return node

    touched = [False] * count
    for nodes, _area, framed, _orient in triangles:
        if framed is None:
            continue
        for node in nodes:
            touched[node] = True
        for other in (nodes[1], nodes[2]):
            a, b = root(nodes[0]), root(other)
            if a != b:
                parent[max(a, b)] = min(a, b)
    islands: dict = {}
    for node in range(count):
        if touched[node]:
            islands.setdefault(root(node), []).append(node)
    members: dict = {}
    for nodes, area, framed, _orient in triangles:
        if framed is not None:
            members.setdefault(root(nodes[0]), []).append((nodes, area, framed))
    u = [float(values_s[node]) for node in range(count)]
    held = [True] * count
    numbers[ISOLATED] = sum(1 for node in range(count) if not touched[node])
    for island in sorted(islands):
        nodes = islands[island]
        numbers[ISLANDS] += 1
        sources = [node for node in nodes if source[node]]
        if not sources:
            numbers[WITHOUT_SOURCE] += 1
        anchored = data.hold_source and bool(sources)
        fixed = set(sources) if anchored else {nodes[0]}
        values = {node: (values_s[node] if anchored else 0.0) for node in fixed}
        solution = _island_solution(data, nodes, members[island], fixed, values, numbers)
        if solution is None:
            numbers[SINGULAR] += 1
            continue
        shift = 0.0
        if not anchored:
            reference = sources if sources else nodes
            total = 0.0
            for node in reference:
                total += values_s[node] - solution[node]
            shift = total / len(reference)
        for node in nodes:
            if anchored and node in fixed:
                continue
            u[node] = solution[node] + shift
            held[node] = False
    slope_x, slope_y = _slopes(count, chart, triangles, u, held, numbers)
    _flips(triangles, u, values_r, root, numbers)
    return UnfoldResultV1(tuple(u), tuple(slope_x), tuple(slope_y), tuple(held), tuple((name, value) for name, value in numbers.items() if value))


def _island_solution(data, nodes, members, fixed, values, numbers):
    """`{неизвестная: u}` острова либо `None` (неположительный ведущий элемент): линейное решение, `data.iterations` шагов local/global на том же разложении и
    до `data.guard_rounds` перевзвешиваний (`grad u . t̂` вне `[GUARD_FLOOR, GUARD_CEILING]`: вес умножается на `GUARD_FACTOR`, разложение заново)."""

    links: dict = {node: set() for node in nodes}
    for tri_nodes, _area, _framed in members:
        for i in range(3):
            for j in range(3):
                links[tri_nodes[i]].add(tri_nodes[j])
    neighbours = {node: sorted(items) for node, items in links.items()}
    free = [node for node in nodes if node not in fixed]
    order = _order(free, neighbours)
    place = {node: at for at, node in enumerate(order)}
    weights = [area for _nodes, area, _framed in members]
    solution = dict(values)
    for guard in range(data.guard_rounds + 1):
        matrix: dict = {node: {} for node in nodes}
        for (tri_nodes, _area, framed), weight in zip(members, weights):
            for i in range(3):
                gi = framed[i]
                row = matrix[tri_nodes[i]]
                for j in range(3):
                    gj = framed[j]
                    row[tri_nodes[j]] = row.get(tri_nodes[j], 0.0) + weight * (gi[0] * gj[0] + gi[1] * gj[1])
        first = []
        entries = []
        for node in order:
            at = place[node]
            row = {}
            low = at
            for other in neighbours[node]:
                if other in fixed:
                    continue
                column = place[other]
                if column <= at:
                    row[column] = matrix[node][other]
                    if column < low:
                        low = column
            first.append(low)
            entries.append(row)
        factor = _Profile.factor(len(order), first, entries)
        if factor is None:
            return None
        targets = [(1.0, 0.0)] * len(members)
        for step in range(data.iterations + 1):
            right: dict = {node: 0.0 for node in nodes}
            for (tri_nodes, _area, framed), weight, (cx, cy) in zip(members, weights, targets):
                for i in range(3):
                    right[tri_nodes[i]] += weight * (framed[i][0] * cx + framed[i][1] * cy)
            constant = []
            for node in order:
                total = right[node]
                for other in neighbours[node]:
                    if other in fixed:
                        total -= matrix[node][other] * values[other]
                constant.append(total)
            answer = factor.solve(constant)
            for node in order:
                solution[node] = answer[place[node]]
            if step == data.iterations:
                break
            targets = []
            for tri_nodes, _area, framed in members:
                gx = solution[tri_nodes[0]] * framed[0][0] + solution[tri_nodes[1]] * framed[1][0] + solution[tri_nodes[2]] * framed[2][0]
                gy = solution[tri_nodes[0]] * framed[0][1] + solution[tri_nodes[1]] * framed[1][1] + solution[tri_nodes[2]] * framed[2][1]
                rx = data.r[tri_nodes[0]] * framed[0][0] + data.r[tri_nodes[1]] * framed[1][0] + data.r[tri_nodes[2]] * framed[2][0]
                ry = data.r[tri_nodes[0]] * framed[0][1] + data.r[tri_nodes[1]] * framed[1][1] + data.r[tri_nodes[2]] * framed[2][1]
                # Ближайший поворот к [[gx, gy], [rx, ry]] (рамка (t̂, n̂r), тождество - идеал): первая строка - (gx + ry, gy - rx) / |...|.
                px, py = gx + ry, gy - rx
                length = sqrt(px * px + py * py)
                targets.append((px / length, py / length) if length > 0.0 else (1.0, 0.0))
        if guard == data.guard_rounds:
            break
        squeezed = 0
        for at, (tri_nodes, _area, framed) in enumerate(members):
            along = solution[tri_nodes[0]] * framed[0][0] + solution[tri_nodes[1]] * framed[1][0] + solution[tri_nodes[2]] * framed[2][0]
            if along < GUARD_FLOOR or along > GUARD_CEILING:
                weights[at] = weights[at] * GUARD_FACTOR
                squeezed += 1
        if not squeezed:
            break
        numbers[GUARDED] += squeezed
    return solution


def _slopes(count, chart, triangles, u, held, numbers):
    """Наклоны `u` по карте на неизвестную: среднее по площади в карте градиентов треугольников с весом; у взятой станции - нули. Треугольник, вырожденный В КАРТЕ
    (удвоенная площадь не выше `DEGENERATE_SINE` суммы квадратов рёбер: игла из точек на одной прямой), в наклон не идёт и назван: его градиент - деление на ноль,
    и вершина только с такими треугольниками дала бы наклон без меры (поле: 6e10 UV на такте rim); у вершины без годного треугольника наклон нуль."""

    sum_x = [0.0] * count
    sum_y = [0.0] * count
    weight = [0.0] * count
    for nodes, _area, framed, _orient in triangles:
        if framed is None:
            continue
        (ax, ay), (bx, by), (cx, cy) = (chart[node] for node in nodes)
        doubled = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)
        edges = (bx - ax) * (bx - ax) + (by - ay) * (by - ay) + (cx - bx) * (cx - bx) + (cy - by) * (cy - by) + (ax - cx) * (ax - cx) + (ay - cy) * (ay - cy)
        if not abs(doubled) > DEGENERATE_SINE * edges:
            numbers[SLOPE_SKIPPED] += 1
            continue
        ua, ub, uc = (u[node] for node in nodes)
        gx = ((ub - ua) * (cy - ay) - (uc - ua) * (by - ay)) / doubled
        gy = ((uc - ua) * (bx - ax) - (ub - ua) * (cx - ax)) / doubled
        area = abs(doubled)
        for node in nodes:
            sum_x[node] += area * gx
            sum_y[node] += area * gy
            weight[node] += area
    slope_x = [0.0 if held[node] or not weight[node] > 0.0 else sum_x[node] / weight[node] for node in range(count)]
    slope_y = [0.0 if held[node] or not weight[node] > 0.0 else sum_y[node] / weight[node] for node in range(count)]
    return slope_x, slope_y


def _flips(triangles, u, values_r, root, numbers):
    """Вывернутые треугольники против большинства острова по площади (`u`, `r`), их площадь в мм² и треугольники нулевой UV-площади."""

    balance: dict = {}
    signed = []
    for nodes, _area, _framed, orient in triangles:
        a, b, c = nodes
        area = orient * ((u[b] - u[a]) * (values_r[c] - values_r[a]) - (u[c] - u[a]) * (values_r[b] - values_r[a]))
        island = root(a)
        signed.append((island, area))
        balance[island] = balance.get(island, 0.0) + area
    flipped_area = 0.0
    for island, area in signed:
        if area == 0.0:
            numbers[ZERO_AREA] += 1
        elif (area > 0.0) != (balance[island] >= 0.0):
            numbers[FLIPPED] += 1
            flipped_area += abs(area) * 0.5
    numbers[FLIPPED_AREA] = int(ceil(flipped_area * 1e6))


def results_agree(left: UnfoldResultV1, right: UnfoldResultV1) -> bool:
    """Равны ли два ответа под `UNFOLD_NATIVE_EQUALITY_V1`: `held` и числа точно, `u` и наклоны - не дальше `NATIVE_EQUALITY`."""

    if left.held != right.held or left.counters != right.counters:
        return False
    for first, second in ((left.u, right.u), (left.slope_x, right.slope_x), (left.slope_y, right.slope_y)):
        if len(first) != len(second):
            return False
        for a, b in zip(first, second):
            if not abs(a - b) <= NATIVE_EQUALITY:
                return False
    return True


__all__ = ("UnfoldInputV1", "UnfoldResultV1", "polyfill", "results_agree", "unfold")
