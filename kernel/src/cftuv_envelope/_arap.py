"""Второе предложение карты развёртки: ARAP local/global в binary64, порядок операций ФИКСИРОВАН.

Модуль внутренний, как `_unfold`: он не владеет допуском (растяжение судит `_stretch` по допуску
запроса `DecalRequestV1.developable_stretch_budget`), ничего не чинит и не принимает карту.
Он делает одно: из положений шарнирного предложения (`hinge_proposal`) выдаёт другое
положение карты, которое минимизирует не «последний треугольник принимает весь дефект
угла вершин», а энергию растяжения по всем треугольникам сразу.

ПОЧЕМУ ЭТО НУЖНО. Шарнир кладёт треугольник за треугольником и сваривает вершины при
первом достижении: веер вершины, угловой дефект которой не нуль (шум на «почти плоской»
поверхности), не замыкается, и вся невязка уходит в те треугольники, что обход достиг
последними (на `wall_noise_top`, патч 3, квадрат растяжения худшего треугольника 9.5 при
лучшей достижимой карте около 1.2). ARAP размазывает ту же невязку по всей карте.

ЗАКОН `ARAP_LOCAL_GLOBAL_80_BINARY64_V1` (число в имени — число итераций, и оно
ЧАСТЬ закона: другое число — другой закон, другое имя).
  * Корень данных — предложение шарнира `init`: стартовые положения вершин.
  * Для каждого треугольника источника — его изометрический образ на плоскости
    (`_local_triangle`: рациональные квадраты длин, один `sqrt` на величину) и
    котангенсы его углов, также от точных рациональных скалярных произведений
    (`w_e = (u . v) / |u x v|` против вершины стороны `e`).
  * Итерация: ЛОКАЛЬНО — для каждого треугольника лучший поворот `R` (замкнутая форма
    для 2x2: нормированный вектор `(S00 + S11, S10 - S01)`, без SVD, без `atan2`);
    ГЛОБАЛЬНО — решение линейной системы Дирихле `A x = b` для всех вершин, кроме
    ПРИШПИЛЕННОЙ (первая вершина порядка обхода: убирает перенос; поворот карты не
    влияет на суд). Матрица `A` одна на все итерации (котангенсы и пин не меняются),
    раскладывается ОДИН РАЗ: Холецкий по огибающей (profile/skyline), разреженно и
    без перестановок, затем 80 пар прямой/обратной подстановки.
  * Итог — положения binary64 (метры), которые дальше идут тем же путём, что положения
    шарнира: точные дроби, суд растяжения по Грамам, простота границы, привязка.

ДЕТЕРМИНИЗМ. Только `+ - * /` и `math.sqrt` (в IEEE-754 они округляются верно), никакого
`sum`, `hypot`, `fsum`, `numpy`: `sum` над float в CPython 3.12 суммирует с компенсацией,
в 3.11 нет, и воспроизводимость между платформами разрушилась бы незаметно. Порядки
обхода (треугольники по имени, стороны по номеру, вершины по первому появлению в
порядке шарнира) названы и не зависят от хеширования.

Предложение — не власть, как у шарнира: любая карта в бюджете растяжения и с простой
границей годна. Поэтому закон не доказывает сходимость итераций: он называет их число,
а судит точный сертификат.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ._unfold import UnfoldProposalV1, UnfoldTopologyV1, squared_distance

#: Число итераций local/global. Входит в имя закона (`ARAP_LOCAL_GLOBAL_80_BINARY64_V1`).
ARAP_PROPOSAL_ITERATIONS = 80

#: Потолок структурной работы ARAP (умножения-сложения): разложение по огибающей плюс
#: подстановки всех итераций плюс проходы по треугольникам. Число считается по
#: СТРУКТУРЕ задачи ДО вычислений, поэтому отказ «слишком велико» воспроизводим.
ARAP_PROPOSAL_WORK_CAP = 60_000_000


class ArapProposalUnavailable(Exception):
    """ARAP не построен: названная причина (потолок работы либо матрица не положительна)."""


@dataclass(frozen=True, slots=True)
class ArapProposalV1:
    """Положения вершин (binary64, метры) и числа, по которым их можно перечитать."""

    coordinates: dict
    iterations: int
    pinned_vertex_id: object
    unknown_count: int
    #: Структурная работа (умножения-сложения): разложение, подстановки, проходы.
    work: int


@dataclass(frozen=True, slots=True)
class _LocalTriangle:
    """Треугольник источника для ARAP: индексы вершин, веса сторон, их образы на плоскости."""

    ids: tuple
    weights: tuple
    #: `((dx, dy), ...)` для сторон `e = (v[e], v[e+1])`: `l[e] - l[e+1]` в изометрической карте.
    edges: tuple


def _local_triangle(corners, indices) -> _LocalTriangle:
    """Изометрический образ треугольника и котангенсы: порядок операций ФИКСИРОВАН.

    `corners` — точные 3D-позиции `p0, p1, p2`; образ `(0, 0), (L, 0), (x, y)`, `y > 0`
    (обход против часовой, как у предложения шарнира).
    """

    return _local_from_squares(
        squared_distance(corners[0], corners[1]),
        squared_distance(corners[0], corners[2]),
        squared_distance(corners[1], corners[2]),
        indices,
    )


def _local_from_squares(squared_01, squared_02, squared_12, indices) -> _LocalTriangle:
    """Образ треугольника по трём квадратам сторон: ему можно дать и не изометричные длины (запас угла).

    Образ с квадратами изометрии источника и есть `_local_triangle`; образ с чужими квадратами —
    ЦЕЛЬ ARAP со смещённым углом (`_cone_relief`): энергия тянет треугольник к ней, а не к источнику.
    """

    along = (squared_02 - squared_12 + squared_01) / 2
    height_squared = squared_02 - along * along / squared_01
    length = math.sqrt(float(squared_01))
    third_x = float(along) / length
    third_y = math.sqrt(float(height_squared))
    local = ((0.0, 0.0), (length, 0.0), (third_x, third_y))
    # Скалярное произведение рёбер в вершине `c` (точно) и удвоенная площадь (один корень).
    dot_0 = (squared_01 + squared_02 - squared_12) / 2
    dot_1 = (squared_01 + squared_12 - squared_02) / 2
    dot_2 = (squared_02 + squared_12 - squared_01) / 2
    twice_area = math.sqrt(float(squared_01 * squared_02 - dot_0 * dot_0))
    cotangent = (
        float(dot_0) / twice_area,
        float(dot_1) / twice_area,
        float(dot_2) / twice_area,
    )
    # Сторона `e = (v[e], v[e+1])` лежит против вершины `(e + 2) % 3`.
    weights = tuple(cotangent[(edge + 2) % 3] for edge in range(3))
    edges = tuple(
        (
            local[edge][0] - local[(edge + 1) % 3][0],
            local[edge][1] - local[(edge + 1) % 3][1],
        )
        for edge in range(3)
    )
    return _LocalTriangle(ids=indices, weights=weights, edges=edges)


def vertex_order(topology: UnfoldTopologyV1, proposal: UnfoldProposalV1) -> tuple:
    """Вершины по первому появлению в порядке обхода треугольников шарнира: узкая огибающая."""

    seen: set = set()
    order: list = []
    for triangle_id in proposal.order:
        for vertex in topology.by_id[triangle_id].vertex_ids:
            if vertex not in seen:
                seen.add(vertex)
                order.append(vertex)
    return tuple(order)


def _assemble(triangles, count):
    """Нижний треугольник матрицы `A` без пина: `rows[i] = {j: значение}`, `j <= i`.

    Индекс `0` — пин, он в матрицу не входит (неизвестная `i` стоит под номером `i - 1`).
    Сложение — в порядке треугольников по имени и сторон по номеру.
    """

    rows = [dict() for _ in range(count - 1)]
    for item in triangles:
        for edge in range(3):
            weight = item.weights[edge]
            first = item.ids[edge] - 1
            second = item.ids[(edge + 1) % 3] - 1
            for vertex in (first, second):
                if vertex >= 0:
                    rows[vertex][vertex] = rows[vertex].get(vertex, 0.0) + weight
            if first >= 0 and second >= 0:
                high, low = (first, second) if first > second else (second, first)
                rows[high][low] = rows[high].get(low, 0.0) - weight
    return rows


def _profile(rows):
    """Первый столбец каждой строки: огибающая, внутри которой живёт разложение."""

    return [min(row) for row in rows]


def _envelope_work(first) -> int:
    """Число умножений-сложений Холецкого по огибающей: сумма квадратов ширин строк."""

    return sum((index - start) * (index - start) for index, start in enumerate(first))


def _factor(rows, first):
    """Холецкий по огибающей: `A = L L^T`; `L[i]` хранит столбцы `first[i] .. i`."""

    factor: list = []
    for index, row in enumerate(rows):
        start = first[index]
        line = [row.get(column, 0.0) for column in range(start, index + 1)]
        for column in range(start, index):
            other = factor[column]
            other_start = first[column]
            low = start if start > other_start else other_start
            value = line[column - start]
            for inner in range(low, column):
                value -= line[inner - start] * other[inner - other_start]
            line[column - start] = value / other[column - other_start]
        pivot = line[index - start]
        for inner in range(start, index):
            pivot -= line[inner - start] * line[inner - start]
        if not pivot > 0.0:
            raise ArapProposalUnavailable(
                f"the ARAP system is not positive definite at unknown {index} "
                f"(pivot {pivot!r}): the owner triangles are numerically degenerate"
            )
        line[index - start] = math.sqrt(pivot)
        factor.append(line)
    return factor


def _solve(factor, first, right_x, right_y):
    """`L L^T x = b` для двух правых частей сразу: прямая и обратная подстановка по огибающей."""

    count = len(factor)
    for index in range(count):
        start = first[index]
        line = factor[index]
        value_x = right_x[index]
        value_y = right_y[index]
        for column in range(start, index):
            value_x -= line[column - start] * right_x[column]
            value_y -= line[column - start] * right_y[column]
        right_x[index] = value_x / line[index - start]
        right_y[index] = value_y / line[index - start]
    for index in range(count - 1, -1, -1):
        start = first[index]
        line = factor[index]
        value_x = right_x[index] / line[index - start]
        value_y = right_y[index] / line[index - start]
        right_x[index] = value_x
        right_y[index] = value_y
        for column in range(start, index):
            right_x[column] -= line[column - start] * value_x
            right_y[column] -= line[column - start] * value_y
    return right_x, right_y


def _rotations(triangles, xs, ys):
    """Лучший поворот каждого треугольника: `(cos, sin)` по `S = sum w dx dl^T`."""

    result = []
    for item in triangles:
        s00 = s01 = s10 = s11 = 0.0
        for edge in range(3):
            first = item.ids[edge]
            second = item.ids[(edge + 1) % 3]
            weight = item.weights[edge]
            local_x, local_y = item.edges[edge]
            delta_x = xs[first] - xs[second]
            delta_y = ys[first] - ys[second]
            s00 += weight * delta_x * local_x
            s01 += weight * delta_x * local_y
            s10 += weight * delta_y * local_x
            s11 += weight * delta_y * local_y
        cosine = s00 + s11
        sine = s10 - s01
        norm = math.sqrt(cosine * cosine + sine * sine)
        result.append((1.0, 0.0) if not norm > 0.0 else (cosine / norm, sine / norm))
    return result


def _right_hand_side(triangles, rotations, pin, count):
    """Правая часть глобального шага: `sum w R (l_i - l_j)`, пин переносится в неё же."""

    right_x = [0.0] * (count - 1)
    right_y = [0.0] * (count - 1)
    for item, (cosine, sine) in zip(triangles, rotations, strict=True):
        for edge in range(3):
            first = item.ids[edge] - 1
            second = item.ids[(edge + 1) % 3] - 1
            weight = item.weights[edge]
            local_x, local_y = item.edges[edge]
            rotated_x = cosine * local_x - sine * local_y
            rotated_y = sine * local_x + cosine * local_y
            if first >= 0:
                right_x[first] += weight * rotated_x
                right_y[first] += weight * rotated_y
                if second < 0:
                    right_x[first] += weight * pin[0]
                    right_y[first] += weight * pin[1]
            if second >= 0:
                right_x[second] -= weight * rotated_x
                right_y[second] -= weight * rotated_y
                if first < 0:
                    right_x[second] += weight * pin[0]
                    right_y[second] += weight * pin[1]
    return right_x, right_y


def _work(first, triangle_count: int) -> int:
    envelope = sum(index - start for index, start in enumerate(first))
    return (
        _envelope_work(first)
        + ARAP_PROPOSAL_ITERATIONS * (4 * envelope + 64 * triangle_count)
    )


def arap_proposal(
    topology: UnfoldTopologyV1, proposal: UnfoldProposalV1, positions, rest=None
) -> ArapProposalV1:
    """Положения карты по закону `ARAP_LOCAL_GLOBAL_80_BINARY64_V1`, старт — предложение шарнира.

    `rest` — необязательные ЦЕЛИ: `{треугольник: (квадрат 01, квадрат 02, квадрат 12)}` для треугольников,
    которым нужна не изометрия источника, а сторона со смещённым углом (запас угла у конуса,
    `_cone_relief`); без него — прежний закон побитово.

    Бросает `ArapProposalUnavailable` (названная причина), если работа по структуре выше
    потолка либо матрица не положительна.
    """

    order = vertex_order(topology, proposal)
    index = {vertex: number for number, vertex in enumerate(order)}
    rest = rest or {}
    triangles = [
        _local_from_squares(*rest[item.triangle_id], tuple(index[vertex] for vertex in item.vertex_ids))
        if item.triangle_id in rest
        else _local_triangle(
            tuple(positions[vertex] for vertex in item.vertex_ids),
            tuple(index[vertex] for vertex in item.vertex_ids),
        )
        for item in topology.triangles
    ]
    count = len(order)
    rows = _assemble(triangles, count)
    first = _profile(rows)
    work = _work(first, len(triangles))
    if work > ARAP_PROPOSAL_WORK_CAP:
        raise ArapProposalUnavailable(
            f"the ARAP proposal was not tried: its structural work {work} "
            f"exceeds the cap {ARAP_PROPOSAL_WORK_CAP} ({count} vertices, "
            f"{len(triangles)} triangles)"
        )
    factor = _factor(rows, first)
    xs = [proposal.coordinates[vertex][0] for vertex in order]
    ys = [proposal.coordinates[vertex][1] for vertex in order]
    pin = (xs[0], ys[0])
    for _ in range(ARAP_PROPOSAL_ITERATIONS):
        rotations = _rotations(triangles, xs, ys)
        right_x, right_y = _right_hand_side(triangles, rotations, pin, count)
        solved_x, solved_y = _solve(factor, first, right_x, right_y)
        xs = [pin[0], *solved_x]
        ys = [pin[1], *solved_y]
    return ArapProposalV1(
        coordinates={vertex: (xs[number], ys[number]) for vertex, number in index.items()},
        iterations=ARAP_PROPOSAL_ITERATIONS,
        pinned_vertex_id=order[0],
        unknown_count=count - 1,
        work=work,
    )
