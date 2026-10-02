"""Закон подъёма `SOURCE_TRIANGLES_CLIPPED_V1`: каждый кусок грани лежит в ОДНОМ треугольнике источника.

ЗАЧЕМ. Подъём `SOURCE_TRIANGLES_V1` кладёт на поверхность ВЕРШИНЫ меша, а грань, чьи вершины
легли на разные треугольники источника, остаётся хордой: ребро меша, идущее через ребро
источника под изломом, срезает стену (замер: лента шириной 0.25 через сгиб 90° уходит в стену на
125 мм). Закон топологии с этим ничего не может сделать — он режет грань на треугольники, а те
всё равно хорды (замер на `rounded_wall.001`: 83 разреза, у `sagging_wall` ни одного четырёхгранья).
Ответ один: вершина на КАЖДОМ пересечении грани с ребром источника.

ЭТО ЗАКОН ПОДЪЁМА, А НЕ ТОПОЛОГИИ. Аксиома `DecalTopologyLawV1` — «новых вершин нет» — цела:
топология собирает грани из вершин, которые ей дали, а вершины `clip:<k>` рождает подъём (как
рождает позиции). Суд над доменом (σ, вложение, сертификат) прежний, допуск не заведён: закон точный.

АЛГОРИТМ (для каждого многоугольника, который выпустила тесселяция; все знаки и деления — `SqrtSumV1`
под бюджетом, ни одного порога):

1. Рёбра многоугольника (`edge_points`). Каждое ребро режется замкнутыми треугольниками источника;
   концы получившихся отрезков, лежащие на ВНУТРЕННЕМ ребре источника (два владельца) либо в его
   конце, — новые вершины ребра. Кэш по паре вершин: одно ребро — одно подразделение, с обеих
   сторон и в любом направлении. ШОВ вершин не получает (`seam_edges`): ребро на границе домена вдоль
   контура патча — источник (`r = 0` на обоих концах) и стена — сварено с соседним доменом только по
   вершинам `src:` (хост не сваривает `clip:` и не считает T-стыки), поэтому вершина на нём молча открыла бы
   шов. Многоугольник, чей шовный край пересекает внутренние рёбра источника, не режется, а остаётся ушами
   под своим счётчиком (`MATERIALIZE_CLIP_FACES_SEAM_CROSSINGS_SUPPRESSED`). Такое пересечение — не
   рельеф: внутренние рёбра источника подходят к контуру патча в его вершинах, а пересекают край лишь
   там, где вершина `src:` на решётке не совпала с углом привязанного треугольника (шум привязки: доли
   ячейки).
2. Выпуклый контур (вершины на прямой допустимы) режется по Сазерленду—Ходжмену тремя полуплоскостями
   каждого треугольника-кандидата (рамки — фильтр, ответа не меняют). Невыпуклый — по ушам точной
   триангуляции; куски одного (многоугольник, треугольник) складываются обратно сокращением
   встречных полурёбер, если объединение — один простой контур, иначе остаются отдельными.
3. Площадь: сумма удвоенных площадей кусков равна площади многоугольника ТОЧНО, и границы кусков
   (после сокращения внутренних полурёбер) — ровно подразделённый контур. Иначе многоугольник
   выходит за привязанную триангуляцию («свес», точки покрытия вне привязанной карты на ячейку-две) либо
   подразделение не сошлось: он остаётся как есть — ушами контура с теми же вершинами на рёбрах (нет
   T-стыков с соседями), под СВОИМ счётчиком, не молча.
4. Доказательство: у КАЖДОГО выпущенного куска все вершины лежат в ЗАМКНУТОМ треугольнике по трём
   знакам рёбер (`_prove`; знаки узлов старше резки берутся из кэша `_line` той же точной функции, знаки
   новых узлов считаются тут впервые); иначе отказ `CLIP_PIECE_LEFT_ITS_TRIANGLE`. Нулевой счёт
   `QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES` под резкой — не доказательство (имена подъёма обнулены): доказательство — `_prove`.

ВЕРШИНЫ. Тождество — `point_key`: точка из любой грани и любого треугольника даёт одну вершину (в
домене нет T-стыков). Ключи `clip:<k>` нумеруются ПОСЛЕ `intern_vertices` в порядке первого выпуска
(номера `node:` не двигаются); точка, уже имеющая ключ, ключа не получает. Подъём — один раз на
вершину, в треугольнике, где она родилась (на ребре результат побитово тот же из обоих: барицентрики
вдоль ребра зависят от концов ребра). `location:clip:<k>` хост не сваривает.

НОРМАЛИ СМЕЩЕНИЯ. Кусок плоский ДО смещения (точно, один треугольник источника). У домена развёртки
нормаль смещения своя на вершину, и условие «побитово равные нормали» для четырёхгранья разрезало бы
каждый кусок: оно здесь НЕ действует. Отклонение смещённого куска от плоскости — политика отображения хоста: оно
записывается (`MATERIALIZE_FACES_MAX_OFFSET_NORMAL_ANGLE_MILLIDEG` ядра, `ADAPTER_MAX_OFF_PLANE_AFTER_OFFSET_NANOMETRES`
квитанции), порога у него нет и тихого пересоздания всех граней нет (прецедент —
`DEVELOPABLE_OFFSET_MIN_GAP_COSINE`).

ВЕРШИНЫ `src:` И ХОСТ. Закон `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1` двигает вершину `src:` в позицию
хоста (до ячейки источника) и до резки сверял ориентацию только канонических треугольников слитых
граней. Куски добавляют обрезки у вершины: объединение цепи, прямой на хорде, с рёбрами источника —
треугольники площадью в ячейку, и сдвиг переворачивает их. Поэтому к канонических треугольникам закона
добавлены уши ВЫПУЩЕННЫХ кусков (`piece_triangles`): сдвиг, который перевернул бы грань, не делается
(`SOURCE_VERTEX_LIFT_REFUSED_BY_FACE_ORIENTATION`, названо и посчитано), сварка с соседом тогда даёт
`ADAPTER_WELD_POSITION_MISMATCH` — один и тот же именованный путь, что у канонических треугольников.

ЗАКОН ТОПОЛОГИИ. Резка берёт многоугольники `PLANAR_POLYGONS_V1` (как на точной плоскости: целый простой
многоугольник с аффинной UV) при ЛЮБОМ запрошенном законе, и после резки закон решает форму кусков
(`_split_for_law`): многоугольник целиком, строго выпуклое четырёхгранье либо уши. Вершины `clip:`, цепи
и семантический дайджест от закона не зависят (уши куска новых вершин не рождают).
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, replace
from functools import cmp_to_key

from ..contracts.geometry_batch import DecalTopologyLawV1
from ..exact_sqrt_sum import SqrtSumV1
from ..wavefront.faces import doubled_shoelace
from .admit import MaterializationOutcome
from .assemble import edge_kind, station_values
from .coalesce import point_key
from .frames import MaterializationRefusal
from .tessellate import convex_quad_ring, has_right_turn, triangulate_exact

#: Имена чисел закона (они же ключи счётчиков материализатора).
VERTICES_INSERTED = "MATERIALIZE_CLIP_VERTICES_INSERTED"
VERTICES_AT_SOURCE_VERTEX = "MATERIALIZE_CLIP_VERTICES_AT_SOURCE_VERTEX"
EDGES_REFINED = "MATERIALIZE_CLIP_EDGES_REFINED"
FACES_IN_ONE_TRIANGLE = "MATERIALIZE_CLIP_FACES_IN_ONE_TRIANGLE"
FACES_CUT = "MATERIALIZE_CLIP_FACES_CUT"
FACES_CUT_BY_EARS = "MATERIALIZE_CLIP_FACES_CUT_BY_EARS"
PIECES_EMITTED = "MATERIALIZE_CLIP_PIECES_EMITTED"
PIECES_MERGED = "MATERIALIZE_CLIP_PIECES_MERGED"
PIECES_KEPT_SEPARATE = "MATERIALIZE_CLIP_PIECES_KEPT_SEPARATE"
FACES_OVERHANG = "MATERIALIZE_CLIP_FACES_OVERHANG_TRIANGULATED"
FACES_BOUNDARY_MISMATCH = "MATERIALIZE_CLIP_FACES_BOUNDARY_MISMATCH_TRIANGULATED"
FACES_SEAM_SUPPRESSED = "MATERIALIZE_CLIP_FACES_SEAM_CROSSINGS_SUPPRESSED"
FACES_OFF_CORNER_SUPPRESSED = "MATERIALIZE_CLIP_FACES_SOURCE_VERTEX_OFF_CORNER_SUPPRESSED"
PREDICATES = "MATERIALIZE_CLIP_PREDICATES"
DIVISIONS = "MATERIALIZE_CLIP_DIVISIONS"


class _Node:
    """Точка стадии: одна на `point_key`. Значения ориентации по треугольникам — кэш на узле."""

    __slots__ = ("point", "key", "window", "home", "cache")

    def __init__(self, point) -> None:
        self.point = point
        self.key: str | None = None
        self.window = None
        #: Треугольник, на ребре которого узел родился при подразделении ребра (подъём — в нём).
        self.home: int | None = None
        self.cache: dict = {}


@dataclass(frozen=True, slots=True)
class ClippedV1:
    """Итог стадии резки домена."""

    #: Грани по слитым граням: `[(ключи вершин, ...), ...]`, в обходе входного многоугольника.
    polygons: list
    #: Контуры слитых граней с вершинами рёбер: `[[(ключ, точка), ...], ...]` (для цепей).
    cycles: list
    #: Все вершины грани (контур и внутренние): `[[(ключ, точка), ...], ...]` (для происхождения).
    vertex_lists: list
    #: Вершины, которых в контуре самой грани не было: `[[(ключ, точка), ...], ...]` (для станций).
    extra_lists: list
    #: `{ключ: точка}` новых вершин `clip:`.
    points: dict
    #: `{ключ: (позиция, (имя треугольника, нормаль))}` новых вершин.
    lifted: dict
    counters: tuple
    note: str


@dataclass(frozen=True, slots=True)
class _Cut:
    """Итог фазы 1 для одного многоугольника."""

    #: Узлы многоугольника против часовой, без вершин рёбер.
    nodes: list
    #: Входной обход был по часовой: куски выпускаются в нём же.
    flip: bool
    #: `[(треугольник, узлы куска, удвоенная площадь)]`; `None` — площадь не сошлась (свес).
    pieces: list | None
    by_ears: bool
    merged: int
    kept: int
    #: Многоугольник мог быть разрезан, но остаётся ушами по причине резки: `"seam"` — его шовный край пересекает
    #: внутренние рёбра источника, `"corner"` — вершина `src:` на карте не совпала с углом привязанной
    #: триангуляции (шум привязки); пусто — не отброшен.
    suppressed: str = ""


class ClipStageV1:
    """Резка многоугольников одного домена треугольниками источника (`plane` — привязанный подъём)."""

    def __init__(self, plane, budget, points) -> None:
        self.plane = plane
        self.budget = budget
        self.triangles = plane.triangles
        self.directions = tuple(1 if item.twice_area > 0 else -1 for item in self.triangles)
        owners = Counter(
            frozenset((item.chart[index], item.chart[(index + 1) % 3]))
            for item in self.triangles
            for index in range(3)
        )
        #: `interior[ti][i]` — `i`-е ребро треугольника `ti` общее у двух треугольников.
        self.interior = tuple(
            tuple(
                owners[frozenset((item.chart[index], item.chart[(index + 1) % 3]))] > 1
                for index in range(3)
            )
            for item in self.triangles
        )
        self.corners = frozenset(corner for item in self.triangles for corner in item.chart)
        self.by_point: dict = {}
        self.node_of_key: dict = {}
        for key, point in points.items():
            node = self._node(point)
            node.key = key
            self.node_of_key[key] = node
        self.edge_cache: dict = {}
        self.needed: set = set()
        self.tally: Counter = Counter()
        self.new_points: dict = {}
        self.lifted: dict = {}
        self.count = 0

    # ---- точки, знаки, пересечения ---------------------------------------

    def _node(self, point) -> _Node:
        identity = point_key(point)
        found = self.by_point.get(identity)
        if found is None:
            found = self.by_point[identity] = _Node(point)
        return found

    def _line(self, node: _Node, ti: int, index: int):
        """`(значение, знак)` точки у `index`-го ребра треугольника `ti`; знак — внутрь `>= 0`."""

        slot = node.cache.get((ti, index))
        if slot is None:
            value = self.plane.line_value(self.triangles[ti], index, node.point)
            self.tally[PREDICATES] += 1
            slot = node.cache[(ti, index)] = (
                value,
                value.sign(budget=self.budget) * self.directions[ti],
            )
        return slot

    def _sign(self, node: _Node, ti: int, index: int) -> int:
        return self._line(node, ti, index)[1]

    def _crossing(self, first: _Node, second: _Node, ti: int, index: int) -> _Node:
        """Точка отрезка на прямой ребра: `first + (second - first) * v0 / (v0 - v1)`, точно."""

        low, high = self._line(first, ti, index)[0], self._line(second, ti, index)[0]
        self.tally[DIVISIONS] += 1
        share = low.divided_by(low - high, self.budget)
        (x0, y0), (x1, y1) = first.point, second.point
        return self._node((x0 + (x1 - x0) * share, y0 + (y1 - y0) * share))

    def _window(self, node: _Node):
        if node.window is None:
            node.window = self.plane.window(node.point)
        return node.window

    def _candidates(self, nodes) -> list[int]:
        """Треугольники, чьи рамки пересекают рамку узлов: фильтр, ответа не меняет."""

        windows = [self._window(node) for node in nodes]
        x_low, x_high = min(w[0] for w in windows), max(w[1] for w in windows)
        y_low, y_high = min(w[2] for w in windows), max(w[3] for w in windows)
        return [
            ti
            for ti, item in enumerate(self.triangles)
            if not (
                x_high < item.box[0]
                or x_low > item.box[1]
                or y_high < item.box[2]
                or y_low > item.box[3]
            )
        ]

    # ---- рёбра ------------------------------------------------------------

    def _segment_in(self, first: _Node, second: _Node, ti: int):
        """Концы части отрезка в ЗАМКНУТОМ треугольнике `ti` либо `None` (в треугольнике отрезка нет)."""

        low, high = first, second
        for index in range(3):
            low_sign, high_sign = self._sign(low, ti, index), self._sign(high, ti, index)
            if low_sign >= 0 and high_sign >= 0:
                continue
            if low_sign < 0 and high_sign < 0:
                return None
            if low_sign == 0:
                high = low
            elif high_sign == 0:
                low = high
            elif low_sign < 0:
                low = self._crossing(low, high, ti, index)
            else:
                high = self._crossing(low, high, ti, index)
        return low, high

    def _on_interior_edge(self, node: _Node, ti: int) -> bool:
        return any(
            self.interior[ti][index] and self._sign(node, ti, index) == 0
            for index in range(3)
        )

    def _ordered(self, first: _Node, second: _Node, nodes):
        """Узлы отрезка в порядке от `first` к `second`: точное сравнение по оси, где концы различны."""

        axis = 0 if not (second.point[0] - first.point[0]).is_zero else 1
        direction = (second.point[axis] - first.point[axis]).sign(budget=self.budget)

        def compare(left: _Node, right: _Node) -> int:
            return direction * (left.point[axis] - right.point[axis]).sign(budget=self.budget)

        return tuple(sorted(nodes, key=cmp_to_key(compare)))

    def edge_points(self, first: _Node, second: _Node):
        """Новые вершины ребра `first -> second` по порядку: кэш по паре, обратное — обратный порядок."""

        cached = self.edge_cache.get((first, second))
        if cached is not None:
            return cached
        found: dict = {}
        for ti in self._candidates((first, second)):
            ends = self._segment_in(first, second, ti)
            if ends is None:
                continue
            for end in ends:
                if end is first or end is second or end in found:
                    continue
                if self._on_interior_edge(end, ti):
                    found[end] = None
                    if end.home is None:
                        end.home = ti
        points = self._ordered(first, second, list(found)) if found else ()
        self.edge_cache[(first, second)] = points
        self.edge_cache[(second, first)] = tuple(reversed(points))
        self.tally[EDGES_REFINED] += int(bool(points))
        return points

    def _refined(self, nodes):
        """Контур с вершинами НУЖНЫХ рёбер: нужное ребро — ребро хоть одного разрезанного многоугольника."""

        out: list = []
        for index, node in enumerate(nodes):
            following = nodes[(index + 1) % len(nodes)]
            out.append(node)
            if frozenset((node, following)) in self.needed:
                out.extend(self.edge_points(node, following))
        return out

    # ---- резка ------------------------------------------------------------

    def _clip(self, nodes, ti: int):
        """Сазерленд—Ходжмен контура против часовой по трём полуплоскостям треугольника `ti` (знак `>= 0` — внутри)."""

        for index in range(3):
            signs = [self._sign(node, ti, index) for node in nodes]
            if all(sign >= 0 for sign in signs):
                continue
            if all(sign <= 0 for sign in signs):
                return []
            result = []
            size = len(nodes)
            for position in range(size):
                current, following = nodes[position], nodes[(position + 1) % size]
                here, there = signs[position], signs[(position + 1) % size]
                if here >= 0:
                    result.append(current)
                if here == 0 or there == 0 or (here > 0) == (there > 0):
                    continue
                result.append(self._crossing(current, following, ti, index))
            nodes = result
        return nodes

    @staticmethod
    def _pruned(nodes):
        out: list = []
        for node in nodes:
            if not out or out[-1] is not node:
                out.append(node)
        if len(out) > 1 and out[0] is out[-1]:
            out.pop()
        return out if len(out) >= 3 else []

    def _area(self, nodes):
        return doubled_shoelace(tuple(node.point for node in nodes))

    def _positive_pieces(self, groups):
        """`[(ti, узлы, площадь)]` без кусков нулевой площади; обратный обход куска — отказ."""

        found = []
        for ti, pieces in groups:
            for nodes in pieces:
                area = self._area(nodes)
                sign = area.sign(budget=self.budget)
                if sign > 0:
                    found.append((ti, nodes, area))
                elif sign < 0:
                    raise MaterializationRefusal(
                        MaterializationOutcome.TESSELLATION_DID_NOT_CLOSE,
                        f"CLIP_PIECE_REVERSED: a piece in source triangle "
                        f"{self.triangles[ti].name} turns against its polygon",
                    )
        return found

    def _merged(self, group):
        """Объединение кусков одного треугольника: один контур без повторов вершин либо `None`.

        Куски — части ушей простого многоугольника, срезанные ОДНИМ выпуклым треугольником, то есть их
        внутренности не пересекаются; после сокращения встречных полурёбер оставшиеся образуют границу их
        объединения, и контур, обходящий каждую вершину ровно раз, проходит по ней без самопересечений.
        Перебор пар рёбер для этого не нужен (он дорог: точные знаки на радикалах).
        """

        directed: dict = {}
        for piece in group:
            for position, node in enumerate(piece):
                edge = (node, piece[(position + 1) % len(piece)])
                if (edge[1], edge[0]) in directed:
                    del directed[(edge[1], edge[0])]
                elif edge in directed:
                    return None
                else:
                    directed[edge] = None
        successor: dict = {}
        for start, end in directed:
            if start in successor:
                return None
            successor[start] = end
        if not successor:
            return None
        first = next(iter(successor))
        loop, current = [first], successor[first]
        while current is not first:
            if current in loop or current not in successor or len(loop) > len(successor):
                return None
            loop.append(current)
            current = successor[current]
        return loop if len(loop) == len(successor) else None

    def _by_ears(self, nodes, ears, candidates):
        """Куски невыпуклого контура: уши режутся каждым треугольником, куски одного треугольника складываются.

        Возвращает `(группы, сложено, оставлено отдельно)`: числа идут в счётчики, только если многоугольник
        в самом деле выпущен кусками.
        """

        groups, merged_count, kept_count = [], 0, 0
        for ti in candidates:
            pieces = []
            for ear in ears:
                piece = self._pruned(self._clip([nodes[index] for index in ear], ti))
                if piece:
                    pieces.append(piece)
            if not pieces:
                continue
            merged = self._merged(pieces) if len(pieces) > 1 else None
            if len(pieces) > 1:
                merged_count += int(merged is not None)
                kept_count += int(merged is None)
            groups.append((ti, [merged] if merged else pieces))
        return groups, merged_count, kept_count

    def _boundary_is(self, pieces, nodes) -> bool:
        """Границы кусков (внутренние полурёбра сокращены) — ровно подразделённый контур."""

        half: dict = {}
        for _ti, piece, _area in pieces:
            for position, node in enumerate(piece):
                edge = (node, piece[(position + 1) % len(piece)])
                if (edge[1], edge[0]) in half:
                    del half[(edge[1], edge[0])]
                elif edge in half:
                    return False
                else:
                    half[edge] = None
        size = len(nodes)
        return set(half) == {(nodes[index], nodes[(index + 1) % size]) for index in range(size)}

    # ---- выпуск -----------------------------------------------------------

    def _key(self, node: _Node, ti: int | None) -> str:
        """Ключ вершины; новая вершина получает `clip:<k>` и подъём в своём треугольнике."""

        if node.key is not None:
            return node.key
        home = node.home if node.home is not None else ti
        if home is None:
            raise MaterializationRefusal(
                MaterializationOutcome.CLIP_PIECE_LEFT_ITS_TRIANGLE,
                "a new clip vertex has no source triangle to be lifted in",
            )
        key = f"clip:{self.count}"
        self.count += 1
        node.key = key
        values = [self._line(node, home, index)[0] for index in range(3)]
        self.lifted[key] = self.plane.lift_known(self.triangles[home], values)
        self.new_points[key] = node.point
        self.node_of_key[key] = node
        point = tuple(axis.as_rational() for axis in node.point)
        self.tally[VERTICES_AT_SOURCE_VERTEX] += int(point in self.corners)
        return key

    def _prove(self, nodes, ti: int) -> None:
        """Каждая вершина куска — в ЗАМКНУТОМ треугольнике источника: три точных знака рёбер.

        Знаки берутся из кэша `_line`: у узла, пришедшего из тесселяции, они посчитаны при резке той же точной
        функцией (`line_value`), у новых узлов (пересечений) их здесь не было, и они считаются заново, независимо
        от арифметики пересечения. Кэш — не второй вычислитель: полностью независимой перепроверки (другим
        кодом) тут нет, и docstring этого не обещает.
        """

        for node in nodes:
            for index in range(3):
                if self._sign(node, ti, index) < 0:
                    raise MaterializationRefusal(
                        MaterializationOutcome.CLIP_PIECE_LEFT_ITS_TRIANGLE,
                        f"a clip piece vertex lies outside the closed source triangle "
                        f"{self.triangles[ti].name} (edge {index})",
                    )

    def _split_for_law(self, nodes, law, fan: bool):
        """Куски закона топологии: многоугольник целиком, четырёхгранья ленты либо уши куска.

        Под `QUAD_STRIPS_V1` четырёхгранье — только у ЛЕНТЫ: куски веера остаются треугольниками, как у закона.
        """

        if law is DecalTopologyLawV1.PLANAR_POLYGONS_V1 or len(nodes) == 3:
            return [nodes]
        points = tuple(node.point for node in nodes)
        if law is DecalTopologyLawV1.QUAD_STRIPS_V1 and not fan and convex_quad_ring(points, self.budget):
            return [nodes]
        ears = triangulate_exact(list(points), self.budget)
        if ears is None:
            raise MaterializationRefusal(
                MaterializationOutcome.TESSELLATION_DID_NOT_CLOSE,
                f"a clip piece of {len(nodes)} vertices has no triangulation",
            )
        return [[nodes[index] for index in ear] for ear in ears]

    def _cut(self, keys) -> _Cut:
        """Фаза 1: многоугольник режется замкнутыми треугольниками; куски приняты, если площадь сошлась ТОЧНО."""

        nodes = [self.node_of_key[key] for key in keys]
        area = self._area(nodes)
        flip = area.sign(budget=self.budget) < 0
        if flip:
            nodes.reverse()
            area = -area
        points = tuple(node.point for node in nodes)
        convex = len(nodes) == 3 or not has_right_turn(points, range(len(nodes)), self.budget)
        candidates = self._candidates(nodes)
        by_ears, merged_count, kept_count = False, 0, 0
        if convex:
            groups = []
            for ti in candidates:
                piece = self._pruned(self._clip(nodes, ti))
                if piece:
                    groups.append((ti, [piece]))
        else:
            by_ears = True
            groups, merged_count, kept_count = self._by_ears(nodes, self._ears(points), candidates)
        pieces = self._positive_pieces(groups)
        total = SqrtSumV1.zero()
        for _ti, _piece, part in pieces:
            total = total + part
        closed = (total - area).is_zero
        return _Cut(nodes, flip, pieces if closed else None, by_ears, merged_count, kept_count)

    def _emit(self, cut: _Cut, law, fan: bool = False):
        """Фаза 3: грани многоугольника в его обходе: куски либо (свес, невязка границы) уши с названным счётчиком."""

        nodes = self._refined(cut.nodes)
        pieces = cut.pieces
        if pieces is not None and self._boundary_is(pieces, nodes):
            self.tally[FACES_IN_ONE_TRIANGLE if len(pieces) == 1 else FACES_CUT] += 1
            self.tally[FACES_CUT_BY_EARS] += int(cut.by_ears)
            self.tally[PIECES_EMITTED] += len(pieces)
            self.tally[PIECES_MERGED] += cut.merged
            self.tally[PIECES_KEPT_SEPARATE] += cut.kept
            faces = []
            for ti, piece, _part in pieces:
                self._prove(piece, ti)
                # Номера `clip:` — по обходу куска, а не по порядку ушей закона: имена не зависят от закона.
                for node in piece:
                    self._key(node, ti)
                for face in self._split_for_law(piece, law, fan):
                    faces.append(self._oriented(tuple(self._key(node, ti) for node in face), cut.flip))
            return faces
        # Свес за привязанную триангуляцию (площадь не сошлась), шовный край, пересекающий внутренние рёбра
        # источника, либо подразделение, не давшее контур: многоугольник остаётся ушами с теми же вершинами
        # на нужных рёбрах, и каждая причина названа своим счётчиком.
        if cut.suppressed:
            self.tally[FACES_SEAM_SUPPRESSED if cut.suppressed == "seam" else FACES_OFF_CORNER_SUPPRESSED] += 1
        else:
            self.tally[FACES_OVERHANG if pieces is None else FACES_BOUNDARY_MISMATCH] += 1
        return [
            self._oriented(tuple(self._key(nodes[index], None) for index in ear), cut.flip)
            for ear in self._ears(tuple(node.point for node in nodes))
        ]

    def _ears(self, points):
        ears = triangulate_exact(list(points), self.budget)
        if ears is None:
            raise MaterializationRefusal(
                MaterializationOutcome.TESSELLATION_DID_NOT_CLOSE,
                f"a clip polygon of {len(points)} vertices has no triangulation",
            )
        return ears

    @staticmethod
    def _oriented(keys, flip: bool):
        return (keys[0], *reversed(keys[1:])) if flip else keys

    # ---- домен ------------------------------------------------------------

    def _off_corner(self, node: _Node) -> bool:
        """Вершина `src:` стоит НЕ в углу привязанной триангуляции (точное сравнение точки с углами карты)."""

        if node.key is None or not node.key.startswith("src:"):
            return False
        point = tuple(axis.as_rational() for axis in node.point)
        return point not in self.corners

    def _suppress_noise(self, cuts, seam) -> list:
        """Разрезанные многоугольники в шуме привязки остаются ушами (`suppressed`): шов либо угол.

        (а) Шовный край, который получил бы вершины, — шов открылся бы молча (`seam`). (б) Вершина `src:` не в
        углу триангуляции — вершина объявленной прямой цепи, сдвинутая вдоль хорды, либо привязка карты, и
        вокруг неё нет «настоящих» пересечений: рёбра, ведущие от неё внутрь, пересекают веер тонких
        треугольников у угла, и вершины на них дают обрезки площадью с ячейку, которые переворачивает сдвиг
        вершины `src:` к позиции хоста (`corner`).
        """

        seam_nodes = {frozenset(self.node_of_key[key] for key in pair) for pair in seam}
        result = []
        for face_cuts in cuts:
            kept = []
            for cut in face_cuts:
                size = len(cut.nodes)
                if cut.pieces is not None:
                    if any(
                        frozenset((cut.nodes[index], cut.nodes[(index + 1) % size])) in seam_nodes
                        and self.edge_points(cut.nodes[index], cut.nodes[(index + 1) % size])
                        for index in range(size)
                    ):
                        cut = replace(cut, pieces=None, suppressed="seam")
                    elif any(self._off_corner(node) for node in cut.nodes):
                        cut = replace(cut, pieces=None, suppressed="corner")
                kept.append(cut)
            result.append(kept)
        return result

    def run(self, cycles, polygons, law, seam=frozenset(), fans=None) -> ClippedV1:
        """Грани всех слитых граней домена, контуры с вершинами рёбер и числа стадии.

        Три фазы. (1) Каждый многоугольник режется, и по площади решается, помещается ли он в привязанную
        триангуляцию. (2) Нужные рёбра — рёбра разрезанных многоугольников: вершины рёбер получают ТОЛЬКО
        они. Свесившийся многоугольник, чьи соседи тоже свесились, их не получает: пересечения у границы
        привязанной карты — шум привязки (доли ячейки вокруг вершин объявленных прямых цепей), а не
        рельеф, и вершины на них дали бы обрезки площадью с ячейку, которые переворачивает сдвиг вершины
        `src:` к позиции хоста. (3) Выпуск: ключи `clip:<k>` в порядке выпуска, подъём новых вершин.

        `seam` — пары ключей рёбер на границе домена вдоль контура патча (`seam_edges`): вершин не получают;
        многоугольник, чей шовный край их получил бы, остаётся ушами и не даёт нужных рёбер. `fans` —
        признак веера у каждой слитой грани (под `QUAD_STRIPS_V1` куски веера остаются треугольниками).
        """

        cuts = [[self._cut(keys) for keys in face_polygons] for face_polygons in polygons]
        cuts = self._suppress_noise(cuts, seam)
        self.needed = {
            frozenset((cut.nodes[index], cut.nodes[(index + 1) % len(cut.nodes)]))
            for face_cuts in cuts
            for cut in face_cuts
            if cut.pieces is not None
            for index in range(len(cut.nodes))
        }
        faces = []
        fan_flags = fans if fans is not None else [False] * len(cuts)
        for face_cuts, fan in zip(cuts, fan_flags):
            emitted: list = []
            for cut in face_cuts:
                emitted.extend(self._emit(cut, law, fan))
            faces.append(tuple(emitted))
        refined, lists, extras = [], [], []
        for cycle, emitted in zip(cycles, faces):
            contour = self._contour(cycle)
            seen = {key: self.node_of_key[key].point for key, _point in contour}
            for keys in emitted:
                for key in keys:
                    seen.setdefault(key, self.node_of_key[key].point)
            original = {key for key, _point in cycle}
            refined.append(contour)
            lists.append(list(seen.items()))
            extras.append([(key, point) for key, point in seen.items() if key not in original])
        self.tally[VERTICES_INSERTED] = len(self.new_points)
        counters = tuple(
            (name, self.tally[name])
            for name in (
                VERTICES_INSERTED,
                VERTICES_AT_SOURCE_VERTEX,
                EDGES_REFINED,
                FACES_IN_ONE_TRIANGLE,
                FACES_CUT,
                FACES_CUT_BY_EARS,
                PIECES_EMITTED,
                PIECES_MERGED,
                PIECES_KEPT_SEPARATE,
                FACES_OVERHANG,
                FACES_BOUNDARY_MISMATCH,
                FACES_SEAM_SUPPRESSED,
                FACES_OFF_CORNER_SUPPRESSED,
                PREDICATES,
                DIVISIONS,
            )
        )
        return ClippedV1(
            polygons=faces,
            cycles=refined,
            vertex_lists=lists,
            extra_lists=extras,
            points=dict(self.new_points),
            lifted=dict(self.lifted),
            counters=counters,
            note=self._note(),
        )

    def _contour(self, cycle):
        """Контур слитой грани с вершинами её рёбер: каждая вершина обязана быть выпущена гранью."""

        nodes = [self.node_of_key[key] for key, _point in cycle]
        out = []
        for node in self._refined(nodes):
            if node.key is None:
                raise MaterializationRefusal(
                    MaterializationOutcome.BATCH_DID_NOT_VALIDATE,
                    "CLIP_CONTOUR_POINT_NOT_EMITTED: a contour vertex is on no emitted face",
                )
            out.append((node.key, node.point))
        return out

    def _note(self) -> str:
        tally = self.tally
        return (
            f"clip_vertices={len(self.new_points)} "
            f"(at_source_vertices={tally[VERTICES_AT_SOURCE_VERTEX]}) "
            f"refined_edges={tally[EDGES_REFINED]} faces_in_one_triangle={tally[FACES_IN_ONE_TRIANGLE]} "
            f"faces_cut={tally[FACES_CUT]} (by_ears={tally[FACES_CUT_BY_EARS]}) "
            f"pieces={tally[PIECES_EMITTED]} merged_groups={tally[PIECES_MERGED]} "
            f"kept_separate_groups={tally[PIECES_KEPT_SEPARATE]} "
            f"overhang_faces={tally[FACES_OVERHANG]} "
            f"boundary_mismatch_faces={tally[FACES_BOUNDARY_MISMATCH]} "
            f"seam_crossings_suppressed_faces={tally[FACES_SEAM_SUPPRESSED]} "
            f"off_corner_source_vertex_faces={tally[FACES_OFF_CORNER_SUPPRESSED]} "
            f"predicates={tally[PREDICATES]} divisions={tally[DIVISIONS]}"
        )


def seam_edges(frame_faces, polygons, facts, layout, lattice_alpha) -> frozenset:
    """Пары ключей рёбер многоугольников на границе домена вдоль контура патча: источник и стена.

    Край — граничный, если он принадлежит ровно одному многоугольнику домена; вид — `edge_kind` цепей батча
    (`SOURCE`: `r = 0` на обоих концах, `RIM`: фронт, иначе `WALL`). Фронт лежит внутри патча, на его
    поверхности, и получает вершины резки законно; источник и стена идут по контуру патча, то есть по
    границе триангуляции и по шву с соседними доменами.
    """

    count: dict = {}
    owner: dict = {}
    for index, face_polygons in enumerate(polygons):
        for keys in face_polygons:
            for position, key in enumerate(keys):
                pair = frozenset((key, keys[(position + 1) % len(keys)]))
                count[pair] = count.get(pair, 0) + 1
                owner[pair] = index
    found = set()
    for pair, number in count.items():
        if number != 1 or len(pair) != 2:
            continue
        first, second = tuple(pair)
        kind = edge_kind(facts, layout.region_of(frame_faces[owner[pair]]), first, second, lattice_alpha)
        if kind != "RIM":
            found.add(pair)
    return frozenset(found)


def piece_triangles(polygons, points, budget):
    """Треугольники (ключи) выпущенных кусков: сам треугольник либо уши точной триангуляции карты."""

    found = []
    for face_polygons in polygons:
        for keys in face_polygons:
            if len(keys) == 3:
                found.append(tuple(keys))
                continue
            ears = triangulate_exact([points[key] for key in keys], budget)
            if ears is None:
                raise MaterializationRefusal(
                    MaterializationOutcome.TESSELLATION_DID_NOT_CLOSE,
                    f"an emitted clip face of {len(keys)} vertices has no triangulation",
                )
            found.extend((keys[a], keys[b], keys[c]) for a, b, c in ears)
    return found


def cut_domain(
    plane, budget, *, frame_faces, cycles, points, polygons, facts, layout, table, lattice_alpha, law, tally=None
) -> ClippedV1:
    """Стадия резки домена: грани тесселяции -> куски; факты `(s, r)` новых вершин дописываются в `facts`.

    `points` — `{ключ: точка}` вершин до резки. Станции и `r` новой вершины — те же аффинные
    функции карты, что у остальных вершин её грани (точно); второй ответ на вершину региона —
    отказ, как у `station_values`.
    """

    seam = seam_edges(frame_faces, polygons, facts, layout, lattice_alpha)
    clipped = ClipStageV1(plane, budget, points).run(
        cycles, polygons, law, seam, [item.is_fan for item in frame_faces]
    )
    extra = station_values(frame_faces, clipped.extra_lists, layout, table, lattice_alpha, budget, tally)
    for slot, value in extra.items():
        known = facts.setdefault(slot, value)
        if known != value:
            raise MaterializationRefusal(
                MaterializationOutcome.BATCH_DID_NOT_VALIDATE,
                f"STATION_VALUE_CONFLICT: vertex {slot[1]} in region {slot[0]} "
                "has two (s, r) answers after the clip",
            )
    return clipped
