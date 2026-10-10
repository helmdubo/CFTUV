"""Контактные дефекты источника: T-вершина и самопересечение грани, названные ДО привязки к решётке.

Полевой случай `cover.008` (замороженная копия `buildings2_2.blend`): патч 18 и патч 285 отказывали
`SOURCE_SNAP_NEW_NONADJACENT_EDGE_INTERSECTION`, а причина лежала в ИСТОЧНИКЕ. У патча 18 вершины 127, 133 и 359
стоят в 6-7 мкм от рёбер, которым не принадлежат (две грани делят полметра общей границы без общих вершин), у патча 285
грань 1497 — «бабочка»: её рёбра 2257-1438 и 1437-1520 пересекаются в плане и разведены по высоте на 4e-08 м. Привязка к
решётке сдвигает каждую вершину не дальше полуячейки, поэтому зазор, меньший полуячейки, она превращает в КОНТАКТ — и
отказ приходит из ядра как счёт «4 контакта после привязки», а чинить надо две вершины, не решётку.

Зазор дефекта — ЯДРО ЭТОГО ЗАКОНА: `AUTHOR_ANGULAR_ERROR * габарит патча`, ровно половина нижней границы окна шага
(`robust.snapping.grid_window_for_patch`: `lower = 2 * AUTHOR_ANGULAR_ERROR * extent`). Любая допустимая ячейка не мельче
нижней границы, значит зазор в пределах этой величины привязка способна замкнуть на ЛЮБОМ допустимом масштабе: это не
деталь модели, а ошибка авторства меньше половины ячейки. Допуск записан в реестре (`SOURCE_CONTACT_GAP_V1`,
AUTHORING_INTENT; значение и место те же, что у `AUTHOR_ANGULAR_ERROR`: копии величины не существует), эффект его один —
сузить принимаемое множество до именованного отказа хоста.

Что называется дефектом (и чего здесь НЕТ, намеренно):

* T-вершина — вершина патча, чья ближайшая точка на ребре патча лежит ВНУТРИ ребра (строго между концами), на
  расстоянии `0 < d <= зазор`. Ребру вершина не принадлежит. Точный нуль (вершина лежит ровно на ребре) не называется:
  это уже существующий контакт, привязка его не создаёт, и ядро его не отвергает. Вершина, ближайшая к КОНЦУ ребра, —
  почти совпавшие вершины, другой класс: его называет привязка (`SOURCE_SNAP_VERTEX_INJECTIVITY_VIOLATED`), а совпавшие
  точно вершины, соединённые ребром, — предполёт `ZERO_LENGTH_EDGE`;
* самопересечение грани — два не смежных ребра ОДНОЙ грани, чьи ближайшие точки лежат внутри обоих рёбер на расстоянии
  `d <= зазор` (включая точное пересечение, `d = 0`): «бабочка» и грань, изогнутая так, что её контур касается себя. Рёбра
  с общей вершиной и параллельные рёбра сюда не входят;
* молчаливой правки нет: функция ничего не сваривает, не двигает и не отбрасывает, а возвращает записи; решение —
  отказ хоста, чинит владелец (`Merge by Distance`, растворить вершину);
* проверка ТОЧНАЯ: широкая фаза идёт в binary64 с запасом `2 * зазор` (ошибка округления binary64 на порядки меньше
  зазора, поэтому кандидат не теряется), а вердикт по паре — в рациональных числах без корней.

Модуль — лист: ни Blender, ни хоста, идентичности вершин и граней — любые упорядочиваемые значения (хост даёт номера BMesh).
"""

from __future__ import annotations

import math
from bisect import bisect_left, bisect_right
from dataclasses import dataclass
from fractions import Fraction

from ._authoring_intent import AUTHOR_ANGULAR_ERROR

#: Сколько ячеек сетки вправе покрыть ребро, прежде чем широкая фаза перейдёт на полосу вдоль оси: длинное ребро на мелкой
#: сетке иначе обходило бы тысячи пустых ячеек. Структура обхода, а не допуск.
_EDGE_CELL_LIMIT = 64


@dataclass(frozen=True, slots=True)
class TVertexContactV1:
    """Вершина `vertex` в `distance_squared ** 0.5` от ребра `edge`, которому не принадлежит (`parameter` — место на ребре)."""

    vertex: object
    edge: tuple
    distance_squared: Fraction
    parameter: Fraction


@dataclass(frozen=True, slots=True)
class FaceCrossingV1:
    """Два не смежных ребра грани `face`, чьи ближайшие точки (внутри рёбер) разведены на `distance_squared ** 0.5`."""

    face: object
    edges: tuple
    distance_squared: Fraction


@dataclass(frozen=True, slots=True)
class SourceContactDefectsV1:
    """Находки по одному патчу: габарит, зазор и записи в порядке «тесное первым» (порядок детерминирован)."""

    extent: Fraction
    gap: Fraction
    t_vertices: tuple
    face_crossings: tuple

    @property
    def clean(self) -> bool:
        return not (self.t_vertices or self.face_crossings)


def source_contact_gap(extent) -> Fraction:
    """Зазор дефекта: `AUTHOR_ANGULAR_ERROR * extent`, половина нижней границы окна шага решётки патча."""

    return AUTHOR_ANGULAR_ERROR * Fraction(extent)


def _exact(point) -> tuple:
    return tuple(Fraction(axis) for axis in point)


def _sub(left, right) -> tuple:
    return tuple(a - b for a, b in zip(left, right))


def _dot(left, right) -> Fraction:
    return sum((a * b for a, b in zip(left, right)), Fraction(0))


def point_segment_interior(point, start, end):
    """`(t, d2)`: ближайшая точка ПРЯМОЙ через концы лежит строго внутри отрезка (`0 < t < 1`), `d2` — квадрат расстояния; иначе `None`."""

    p, a, b = _exact(point), _exact(start), _exact(end)
    along = _sub(b, a)
    length = _dot(along, along)
    if length == 0:
        return None
    t = _dot(_sub(p, a), along) / length
    if not 0 < t < 1:
        return None
    offset = tuple(x - (y + t * z) for x, y, z in zip(p, a, along))
    return t, _dot(offset, offset)


def segment_crossing(first_start, first_end, second_start, second_end):
    """`d2`: квадрат расстояния между ближайшими точками двух ПРЯМЫХ, если обе точки лежат строго внутри своих отрезков; иначе `None`.

    Параллельные отрезки (определитель нуль) не пересекаются в этом смысле: ближайшей пары у них нет.
    """

    a, b, c, d = _exact(first_start), _exact(first_end), _exact(second_start), _exact(second_end)
    u, v, w = _sub(b, a), _sub(d, c), _sub(a, c)
    uu, vv, uv, uw, vw = _dot(u, u), _dot(v, v), _dot(u, v), _dot(u, w), _dot(v, w)
    determinant = uu * vv - uv * uv
    if determinant == 0:
        return None
    s = (uv * vw - vv * uw) / determinant
    t = (uu * vw - uv * uw) / determinant
    if not (0 < s < 1 and 0 < t < 1):
        return None
    offset = tuple(x + s * y - t * z for x, y, z in zip(w, u, v))
    return _dot(offset, offset)


def segment_distance_squared(first_start, first_end, second_start, second_end) -> Fraction:
    """Точный квадрат расстояния между двумя ЗАМКНУТЫМИ отрезками (ближайшая пара точек, концы включены)."""

    a, b, c, d = _exact(first_start), _exact(first_end), _exact(second_start), _exact(second_end)
    u, v, w = _sub(b, a), _sub(d, c), _sub(a, c)
    uu, vv, uv, uw, vw = _dot(u, u), _dot(v, v), _dot(u, v), _dot(u, w), _dot(v, w)

    def clamped(value):
        return min(max(value, Fraction(0)), Fraction(1))

    if uu == 0 and vv == 0:
        s = t = Fraction(0)
    elif uu == 0:
        s, t = Fraction(0), clamped(vw / vv)
    elif vv == 0:
        s, t = clamped(-uw / uu), Fraction(0)
    else:
        determinant = uu * vv - uv * uv
        s = clamped((uv * vw - vv * uw) / determinant) if determinant != 0 else Fraction(0)
        t = (uv * s + vw) / vv
        if t < 0:
            t, s = Fraction(0), clamped(-uw / uu)
        elif t > 1:
            t, s = Fraction(1), clamped((uv - uw) / uu)
    offset = tuple(x + s * y - t * z for x, y, z in zip(w, u, v))
    return _dot(offset, offset)


def _box(points, margin):
    return (
        tuple(min(p[axis] for p in points) - margin for axis in range(3)),
        tuple(max(p[axis] for p in points) + margin for axis in range(3)),
    )


def _boxes_meet(first, second) -> bool:
    return all(first[0][axis] <= second[1][axis] and second[0][axis] <= first[1][axis] for axis in range(3))


def _cycle_edges(cycle) -> tuple:
    return tuple(zip(cycle, cycle[1:] + cycle[:1]))


def _t_candidates(float_of, edges, margin) -> list:
    """Кандидаты `(вершина, a, b)` широкой фазы: вершина в рамке ребра с запасом. Сетка вершин, шаг — медиана длины ребра (не мельче `4 * margin`)."""

    if not edges:
        return []
    lengths = sorted(math.dist(float_of[a], float_of[b]) for a, b in edges)
    cell = max(lengths[len(lengths) // 2], 4 * margin) or 1.0
    cells: dict = {}
    for vertex, point in float_of.items():
        cells.setdefault(tuple(math.floor(axis / cell) for axis in point), []).append(vertex)
    axis_order = None
    found = []
    for a, b in edges:
        low, high = _box((float_of[a], float_of[b]), margin)
        first = tuple(math.floor(axis / cell) for axis in low)
        last = tuple(math.floor(axis / cell) for axis in high)
        span = (last[0] - first[0] + 1) * (last[1] - first[1] + 1) * (last[2] - first[2] + 1)
        if span <= _EDGE_CELL_LIMIT:
            nearby = [
                vertex
                for i in range(first[0], last[0] + 1)
                for j in range(first[1], last[1] + 1)
                for k in range(first[2], last[2] + 1)
                for vertex in cells.get((i, j, k), ())
            ]
        else:
            if axis_order is None:
                axis_order = sorted(float_of, key=lambda vertex: float_of[vertex][0])
                axis_keys = [float_of[vertex][0] for vertex in axis_order]
            nearby = axis_order[bisect_left(axis_keys, low[0]) : bisect_right(axis_keys, high[0])]
        for vertex in nearby:
            if vertex == a or vertex == b:
                continue
            point = float_of[vertex]
            if all(low[axis] <= point[axis] <= high[axis] for axis in range(3)):
                found.append((vertex, a, b))
    return found


def find_source_contact_defects(positions, faces, *, extent) -> SourceContactDefectsV1:
    """Дефекты ОДНОГО патча: `positions` — `{вершина: (x, y, z)}`, `faces` — `[(грань, цикл вершин)]`, `extent` — габарит патча.

    `extent` — тот же габарит, что в окне шага (`source_grid.source_extent`: наибольший разброс по одной оси), и вызывающий
    обязан передать именно его: зазор — функция габарита. Находки отсортированы «тесное первым», затем по номерам.
    """

    gap = source_contact_gap(extent)
    gap_squared = gap * gap
    margin = 2 * float(gap)
    cycles = [(face, tuple(cycle)) for face, cycle in faces]
    vertices = sorted({vertex for _face, cycle in cycles for vertex in cycle})
    float_of = {vertex: tuple(float(axis) for axis in positions[vertex]) for vertex in vertices}
    exact_of: dict = {}

    def exact(vertex):
        found = exact_of.get(vertex)
        if found is None:
            found = exact_of[vertex] = _exact(positions[vertex])
        return found

    edges = sorted(
        {(a, b) if a < b else (b, a) for _face, cycle in cycles for a, b in _cycle_edges(cycle) if a != b}
    )
    t_vertices = []
    for vertex, a, b in _t_candidates(float_of, edges, margin):
        hit = point_segment_interior(exact(vertex), exact(a), exact(b))
        if hit is None:
            continue
        parameter, distance_squared = hit
        if 0 < distance_squared <= gap_squared:
            t_vertices.append(TVertexContactV1(vertex, (a, b), distance_squared, parameter))
    face_crossings = []
    for face, cycle in cycles:
        count = len(cycle)
        if count < 4:
            continue
        sides = _cycle_edges(cycle)
        boxes = [_box((float_of[a], float_of[b]), margin) for a, b in sides]
        for i in range(count):
            for j in range(i + 2, count):
                if i == 0 and j == count - 1:
                    continue
                first, second = sides[i], sides[j]
                if len({*first, *second}) < 4 or not _boxes_meet(boxes[i], boxes[j]):
                    continue
                distance_squared = segment_crossing(exact(first[0]), exact(first[1]), exact(second[0]), exact(second[1]))
                if distance_squared is not None and distance_squared <= gap_squared:
                    face_crossings.append(FaceCrossingV1(face, (first, second), distance_squared))
    t_vertices.sort(key=lambda item: (item.distance_squared, item.vertex, item.edge))
    face_crossings.sort(key=lambda item: (item.distance_squared, item.face, item.edges))
    return SourceContactDefectsV1(Fraction(extent), gap, tuple(t_vertices), tuple(face_crossings))


__all__ = (
    "FaceCrossingV1",
    "SourceContactDefectsV1",
    "TVertexContactV1",
    "find_source_contact_defects",
    "point_segment_interior",
    "segment_crossing",
    "segment_distance_squared",
    "source_contact_gap",
)
