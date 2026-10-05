"""Ячейки резки закона `SOURCE_FACES_CLIPPED_V1`: выпуклые грани источника вместо их треугольников.

ЗАЧЕМ. Закон `SOURCE_TRIANGLES_CLIPPED_V1` режет грань меша по КАЖДОМУ общему ребру двух треугольников
источника. Но триангуляция — дело хоста (`calc_loop_triangles`), а не меша: диагональ четырёхгранья общая у
двух треугольников ОДНОЙ грани, у неё нет `physical_edge_id`, и в мешe источника такого ребра нет. Замер
на `sagging_wall`: 28 вершин резки и 29 рёбер декали лежали на диагоналях (ребра ползли по сетке, которой
в источнике нет). Здесь режет только настоящее ребро меша — общее у треугольников РАЗНЫХ граней, а
диагональ перестаёт быть границей области: областью резки (ячейкой) становится замкнутая грань источника,
то есть объединение её треугольников на карте.

ЯЧЕЙКА. Объединение треугольников одной грани, если оно — один выпуклый контур (общие ребра треугольников
— диагонали — сокращаются как встречные полурёбра; остаётся одна петля). Выпуклость нужна дважды: резка
Сазерленда — Ходжмена идёт по полуплоскостям рёбер ячейки, а выпуклая оболочка куска лежит в ячейке, то есть
поверхность над ней определена (оценка хорды ниже). Прямая вершина петли (угол 180°, T-вершина n-угольника
стены) допустима, но кусок, чей край проходит через неё, обязан её нести (`ClipCellV1.straight`): соседняя
грань с поворотом в этой вершине даёт своему куску вершину, и без неё у куска этой ячейки граница с соседом не
сошлась бы.

НЕВЫПУКЛАЯ ГРАНЬ (Г- и П-образная стена с проёмом): одна простая петля того же обхода, но с поворотом
против обхода. Областями остаются её треугольники, а `ClipCellV1.group` объединяет их: диагонали внутри
группы НЕ режут (`ClipStageV1.inert`), куски многоугольника по треугольникам группы склеиваются в один
контур сокращением встречных полурёбер (`ClipStageV1._glued`), а вершины, рождённые пересечением с такой
диагональю и лежащие на прямой между соседями, снимаются. Оценка хорды — `2ρ` по всей грани (не зависит от
размера куска); для куска, чья оболочка выходит за грань (обход проёма), она держится в допущении, что
триангуляция выпущенной грани лежит в её контуре — а у простого многоугольника любая триангуляция такова.

Грань со смешанным обходом, не одной петлёй либо с изломом без определённой стороны режется по своим
треугольникам, как под `SOURCE_TRIANGLES_CLIPPED_V1`, под счётчиком
`MATERIALIZE_CLIP_DIAGONAL_KEPT_FACE_UNMERGEABLE`.

ХОРДА. Кусок ячейки лежит в ячейке целиком, но вершины его подняты в РАЗНЫХ треугольниках грани:
непланарная грань даёт кусок со складкой по диагонали, а грань меша плоская ровно по вершинам. Глубина
хорды — наибольшее расстояние точки куска (в любой его триангуляции: она лежит в выпуклой оболочке вершин)
до поверхности источника НАД ТОЙ ЖЕ точкой карты. Оценка ЗВУЧНАЯ, не измерение:

* ячейка из двух треугольников (четырёхгранье): поверхность над ячейкой — `S = L1 + k·min(0, ℓ)`, где `L1` —
  аффинный подъём первого треугольника, `ℓ` — ориентированное расстояние до диагонали (в единицах рёбер
  карты), `k` — вектор излома (точный: `(Q - L1(q)) / ℓ(q)` по четвёртой вершине). Отклонение выпуклой
  комбинации вершин куска от `S` над той же точкой — `|k|·(Σλ·m - m(Σλℓ))`, `m = min(0, ·)`, и его
  наибольшее — `|k|·a·b/(a+b)`, где `a` — наибольшая глубина вершины куска за диагональ, `b` — по эту сторону
  (на одной стороне — нуль). Числа `a`, `b` берутся верхними границами строгой оболочки `SqrtSumV1`;
  глубина — точный `Fraction` в квадрате, корень не берётся;
* выпуклая ячейка из трёх и более треугольников: `2·ρ`, где `ρ` — наибольшее уклонение угла ячейки от
  аффинного подъёма самого большого её треугольника. На ячейке поверхность и любая выпуклая комбинация
  вершин отстоят от этого аффинного подъёма не более `ρ` каждая. Оценка не зависит от размера куска.

Допуск — ЧЕТВЕРТЬ СМЕЩЕНИЯ ХОСТА (0.02 м -> 5 мм), `CLIP_DIAGONAL_CHORD_BUDGET`, запись реестра допусков
`CLIP_DIAGONAL_CHORD_DEPTH_V1`; число — умолчание в ожидании решения владельца и меняется одной строкой.
Ячейка, у которой хоть один кусок глубже допуска, расщепляется на треугольники своей грани (резка по
диагонали, как раньше) под счётчиком `MATERIALIZE_CLIP_DIAGONAL_KEPT_FACE_NOT_PLANAR`; наибольшая глубина
записана в нанометрах (до и после допуска). Точно планарная грань (`k = 0`, `ρ = 0`) диагональю не режется никогда.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from fractions import Fraction

from .lift import ENCLOSURE_BITS

#: Допуск глубины хорды куска над диагональю грани источника, метры (единицы 3D ядра): четверть смещения
#: хоста (0.02 м). УМОЛЧАНИЕ, РЕШЕНИЕ ВЛАДЕЛЬЦА ЖДЁТ. Запись реестра допусков `CLIP_DIAGONAL_CHORD_DEPTH_V1`.
CLIP_DIAGONAL_CHORD_BUDGET = Fraction(1, 200)

DIAGONAL_FACES_WHOLE = "MATERIALIZE_CLIP_DIAGONAL_FACES_KEPT_WHOLE"
DIAGONAL_PIECES_ACROSS = "MATERIALIZE_CLIP_DIAGONAL_PIECES_ACROSS"
DIAGONAL_CUTS_AVOIDED = "MATERIALIZE_CLIP_DIAGONAL_CUTS_AVOIDED"
DIAGONAL_KEPT_NOT_PLANAR = "MATERIALIZE_CLIP_DIAGONAL_KEPT_FACE_NOT_PLANAR"
DIAGONAL_KEPT_UNMERGEABLE = "MATERIALIZE_CLIP_DIAGONAL_KEPT_FACE_UNMERGEABLE"
DIAGONAL_MAX_CHORD_KEPT = "MATERIALIZE_CLIP_DIAGONAL_MAX_CHORD_KEPT_NANOMETRES"
DIAGONAL_MAX_CHORD_OVER = "MATERIALIZE_CLIP_DIAGONAL_MAX_CHORD_OVER_BUDGET_NANOMETRES"
NANOMETRES_PER_METRE = 10**9


@dataclass(frozen=True, slots=True)
class HingeV1:
    """Излом поверхности над ячейкой из двух треугольников: диагональ `edge` треугольника `triangle`."""

    triangle: int
    edge: int
    #: `|k|^2`: вектор излома, метры на единицу `ℓ` (единицы решётки в квадрате), точно.
    jump_square: Fraction


@dataclass(frozen=True, slots=True)
class ClipCellV1:
    """Область резки: треугольник источника либо выпуклая грань из нескольких его треугольников."""

    #: Ключ кэша знаков на узле: `("t", индекс треугольника)` либо `("f", грань, индекс первого треугольника)`.
    key: tuple
    #: Имя области: у ячейки — грань источника, у треугольника — его имя.
    name: str
    #: Петля вершин карты в обходе её треугольников (как у `LiftTriangleV1.chart`, но длиной `k >= 3`).
    chart: tuple
    box: tuple
    twice_area: Fraction
    #: Индексы треугольников подъёма, из которых ячейка склеена.
    members: tuple
    #: Диагонали (общие рёбра склеенных треугольников): `(индекс треугольника, индекс ребра)`.
    diagonals: tuple = ()
    hinge: HingeV1 | None = None
    #: `(2ρ)^2` для склеенной ячейки без излома (три и более треугольников), точно.
    flat_square: Fraction | None = None
    #: Прямые вершины петли: `(индекс ребра на той же прямой, точка карты)`.
    straight: tuple = ()
    #: Ключ группы невыпуклой грани: треугольники одной группы не режут друг друга по диагонали (склейка кусков).
    group: tuple | None = None


@dataclass(frozen=True, slots=True)
class CellPlanV1:
    cells: tuple
    #: Грани из двух и более треугольников, которые не склеились ни в ячейку, ни в группу: `((грань, причина), ...)`.
    #: Причины: `MIXED_WINDING`, `NOT_ONE_LOOP`, `NO_HINGE`.
    unmergeable: tuple
    #: Пары граней плана станций цепей (`CHAIN_STATION_PLAN_V1`), склеенные в группы: рёбра между ними не режут. Нуль — плана нет.
    plan_pairs: int = 0


def _edge_value(start, end, point) -> Fraction:
    """`(end - start) x (point - start)` точно (то же, что `lift_surface._edge_value`, на дробях)."""

    return (end[0] - start[0]) * (point[1] - start[1]) - (end[1] - start[1]) * (point[0] - start[0])


def _affine(triangle, point):
    """Аффинный подъём треугольника в точке карты (дроби): `(e1·A + e2·B + e0·C) / D`."""

    values = [
        _edge_value(triangle.chart[index], triangle.chart[(index + 1) % 3], point)
        for index in range(3)
    ]
    weights = (values[1], values[2], values[0])
    return tuple(
        sum(weight * corner[axis] for weight, corner in zip(weights, triangle.corners))
        / triangle.twice_area
        for axis in range(3)
    )


def _single(triangles, index: int, group=None, flat=None) -> ClipCellV1:
    item = triangles[index]
    return ClipCellV1(
        ("t", index), item.name, item.chart, item.box, item.twice_area, (index,), (), None, flat, (), group
    )


def _loop_of(triangles, members):
    """`(петля вершин, диагонали)` либо `None`: одна простая петля после сокращения общих рёбер."""

    # Вершина — пара «точка карты, угол 3D»: хеш дробей дорог, поэтому каждая вершина получает номер один раз, а рёбра
    # и петля идут по номерам (равенство вершин — равенство номеров).
    numbers: dict = {}
    vertices: list = []

    def number(item, position) -> int:
        vertex = (item.chart[position], item.corners[position])
        found = numbers.get(vertex)
        if found is None:
            found = numbers[vertex] = len(vertices)
            vertices.append(vertex)
        return found

    directed: dict = {}
    diagonals = []
    for index in members:
        item = triangles[index]
        for position in range(3):
            first, second = number(item, position), number(item, (position + 1) % 3)
            if (second, first) in directed:
                diagonals.append(directed.pop((second, first)))
            elif (first, second) in directed:
                return None
            else:
                directed[(first, second)] = (index, position)
    successor: dict = {}
    for first, second in directed:
        if first in successor:
            return None
        successor[first] = second
    if not successor:
        return None
    start = next(iter(successor))
    loop, current = [start], successor[start]
    while current != start:
        if current in loop or current not in successor:
            return None
        loop.append(current)
        current = successor[current]
    return ([vertices[found] for found in loop], diagonals) if len(loop) == len(successor) else None


def _hinge(triangles, diagonal, members) -> HingeV1 | None:
    owner, edge = diagonal
    first = triangles[owner]
    other = triangles[next(index for index in members if index != owner)]
    start, end = first.chart[edge], first.chart[(edge + 1) % 3]
    apex = next(index for index in range(3) if other.chart[index] not in (start, end))
    point, corner = other.chart[apex], other.corners[apex]
    value = _edge_value(start, end, point)
    reach = value if first.twice_area > 0 else -value
    if reach >= 0:
        return None
    base = _affine(first, point)
    jump = tuple((corner[axis] - base[axis]) / reach for axis in range(3))
    return HingeV1(owner, edge, sum(axis * axis for axis in jump))


def _flat_square(triangles, members) -> Fraction:
    """`(2ρ)^2`: `ρ` — наибольшее уклонение угла ячейки от аффинного подъёма её самого большого треугольника."""

    big = triangles[max(members, key=lambda index: abs(triangles[index].twice_area))]
    worst = Fraction(0)
    for index in members:
        for point, corner in zip(triangles[index].chart, triangles[index].corners):
            base = _affine(big, point)
            worst = max(worst, sum((corner[axis] - base[axis]) ** 2 for axis in range(3)))
    return 4 * worst


def _merged_cell(triangles, face: str, members) -> ClipCellV1 | str:
    """Ячейка из треугольников `members` одной грани либо причина отказа (строка)."""

    sign = 1 if triangles[members[0]].twice_area > 0 else -1
    if any((triangles[index].twice_area > 0) != (sign > 0) for index in members):
        return "MIXED_WINDING"
    found = _loop_of(triangles, members)
    if found is None:
        return "NOT_ONE_LOOP"
    loop, diagonals = found
    chart = [vertex[0] for vertex in loop]
    size = len(chart)
    straight = []
    for position in range(size):
        a, b, c = chart[position], chart[(position + 1) % size], chart[(position + 2) % size]
        turn = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
        if turn * sign < 0:
            return "NOT_CONVEX"
        if turn == 0:
            straight.append(((position + 1) % size, b))
    boxes = [triangles[index].box for index in members]
    hinge = _hinge(triangles, diagonals[0], members) if len(members) == 2 and len(diagonals) == 1 else None
    if len(members) == 2 and hinge is None:
        return "NO_HINGE"
    return ClipCellV1(
        ("f", face, members[0]),
        face,
        tuple(chart),
        (min(b[0] for b in boxes), max(b[1] for b in boxes), min(b[2] for b in boxes), max(b[3] for b in boxes)),
        sum((triangles[index].twice_area for index in members), Fraction(0)),
        tuple(members),
        tuple(diagonals),
        hinge,
        None if hinge is not None else _flat_square(triangles, members),
        tuple(straight),
    )


def _plan_groups(names, inert, usable) -> tuple:
    """`({имя грани: ключ группы}, число склеенных пар)`: грани, склеенные инертными рёбрами плана станций цепей (`CHAIN_STATION_PLAN_V1`).

    `inert` — множество пар `frozenset({имя, имя})` граней источника по обе стороны инертного поперечного ребра `FREE`-вершины. Склеиваются
    пары, обе грани которых есть у домена (чужой патч в подъёме не участвует) и `usable` (грань со складкой обхода или без одной петли в группу
    не идёт: её треугольники режутся как прежде); связная компонента из двух и более граней — группа, её ключ `("p", наименьшее имя)`.
    Рёбра между гранями группы не режут, как диагонали невыпуклой грани.
    """

    parent: dict = {}

    def root(name):
        parent.setdefault(name, name)
        while parent[name] != name:
            parent[name] = parent[parent[name]]
            name = parent[name]
        return name

    pairs = 0
    for pair in inert:
        if len(pair) == 2:
            first, second = sorted(pair)
            if first in names and second in names and usable(first) and usable(second):
                parent[root(first)] = root(second)
                pairs += 1
    found: dict = {}
    for name in sorted(parent):
        found.setdefault(root(name), []).append(name)
    keys = {name: ("p", min(members)) for members in found.values() if len(members) >= 2 for name in members}
    return keys, pairs if keys else 0


def build_cells(triangles, split=frozenset(), memo=None, inert=frozenset()) -> CellPlanV1:
    """Ячейки резки по треугольникам подъёма; ячейки и группы из `split` (их ключи) остаются треугольниками.

    Порядок — по индексу первого треугольника, поэтому при `split` на всех ячейках области совпадают с
    треугольниками один в один, и резка побитово та же, что у `SOURCE_TRIANGLES_CLIPPED_V1`.

    `memo` — словарь вызывающего на ОДНИ треугольники (две стадии одной резки): склейка грани и оценка невыпуклой грани —
    чистые функции треугольников грани, и вторая стадия берёт их готовыми, а не считает заново на дробях.

    `inert` — пары граней плана станций цепей: грани группы (`_plan_groups`) остаются треугольниками ОДНОЙ группы, рёбра между ними не режут, и
    куски склеиваются в один контур (`ClipStageV1._glued`). Глубину хорды группы план доказал сам (допуск хорды в обоих направлениях и плоскость
    вместе, `_chain_station`), поэтому оценка группы — нуль, а невыпуклая грань внутри неё сохраняет свою оценку `2ρ`: её защита от
    складки остаётся.
    """

    memo = {} if memo is None else memo
    groups: dict = {}
    for index, item in enumerate(triangles):
        groups.setdefault(item.face or f"\0{index}", []).append(index)

    def built_of(name):
        built = memo.get(("cell", name))
        if built is None:
            built = memo[("cell", name)] = _merged_cell(triangles, name, groups[name])
        return built

    def usable(name) -> bool:
        return len(groups[name]) < 2 or isinstance(built_of(name), ClipCellV1) or built_of(name) == "NOT_CONVEX"

    planned, plan_pairs = _plan_groups(groups, inert, usable) if inert else ({}, 0)
    group_flat: dict = {}
    for name, key in planned.items():
        group_flat[key] = max(
            group_flat.get(key, Fraction(0)),
            _non_convex_flat(triangles, name, groups[name], memo) if len(groups[name]) > 1 else Fraction(0),
        )
    cells, unmergeable = [], []
    for face, members in groups.items():
        key = planned.get(face)
        if key is not None and key not in split:
            cells.extend((index, _single(triangles, index, key, group_flat[key])) for index in members)
            continue
        if len(members) == 1:
            built = None
        else:
            built = memo.get(("cell", face))
            if built is None:
                built = memo[("cell", face)] = _merged_cell(triangles, face, members)
        if built is None:
            cells.append((members[0], _single(triangles, members[0])))
        elif isinstance(built, ClipCellV1):
            if built.key in split:
                cells.extend((index, _single(triangles, index)) for index in members)
            else:
                cells.append((members[0], built))
        elif built == "NOT_CONVEX" and ("g", face, members[0]) not in split:
            flat = memo.get(("flat", face))
            if flat is None:
                flat = memo[("flat", face)] = _flat_square(triangles, members)
            group = ("g", face, members[0])
            cells.extend((index, _single(triangles, index, group, flat)) for index in members)
        else:
            if built != "NOT_CONVEX":
                unmergeable.append((face, built))
            cells.extend((index, _single(triangles, index)) for index in members)
    cells.sort(key=lambda entry: entry[0])
    return CellPlanV1(tuple(cell for _index, cell in cells), tuple(unmergeable), plan_pairs)


def _non_convex_flat(triangles, face, members, memo) -> Fraction:
    """Оценка `(2ρ)^2` невыпуклой грани `face` (кэш `memo`) либо нуль: выпуклая и однотреугольная грани оценки группы не двигают."""

    built = memo.get(("cell", face))
    if built is None:
        built = memo[("cell", face)] = _merged_cell(triangles, face, members)
    if built != "NOT_CONVEX":
        return Fraction(0)
    flat = memo.get(("flat", face))
    if flat is None:
        flat = memo[("flat", face)] = _flat_square(triangles, members)
    return flat


def hinge_depth_square(jump_square: Fraction, column) -> Fraction:
    """`|k|^2·(a·b/(a+b))^2` по значениям `ℓ` вершин куска: верхняя граница по оболочкам, нуль при одной стороне."""

    behind = ahead = None
    for item in column:
        low, high = item.enclosure(ENCLOSURE_BITS)
        behind = -low if behind is None or -low > behind else behind
        ahead = high if ahead is None or high > ahead else ahead
    if behind <= 0 or ahead <= 0:
        return Fraction(0)
    gap = behind * ahead / (behind + ahead)
    return jump_square * gap * gap


def chord_of(cell: ClipCellV1, values, budget) -> tuple:
    """`(квадрат глубины хорды, число диагоналей, которые кусок пересекает)`.

    `values[i]` — значения ориентации вершин куска у `i`-й диагонали ячейки (`SqrtSumV1`). Пересечение — вершины
    куска СТРОГО по обе стороны диагонали (точные знаки): именно столько разрезов сделал бы закон по
    треугольникам.
    """

    crossings = 0
    for column in values:
        signs = [item.sign(budget=budget) for item in column]
        crossings += int(min(signs) < 0 < max(signs))
    if cell.hinge is not None:
        return hinge_depth_square(cell.hinge.jump_square, values[0]), crossings
    return cell.flat_square, crossings


def nanometres(depth_square: Fraction) -> int:
    """Целая верхняя граница глубины в нанометрах: `ceil(sqrt(depth_square · 10^18))`, точно."""

    scaled = depth_square * NANOMETRES_PER_METRE**2
    whole = -(-scaled.numerator // scaled.denominator)
    root = math.isqrt(whole)
    return root if root * root == whole else root + 1
