"""Закон `SLAB_DECOMPOSITION_BY_STATIONS_V1`: кусок с неаффинной UV режется на трапеции по станциям, а не на уши.

ЗАЧЕМ. Кусок полосы потока, чья UV не аффинна (вершины фронта лежат не на одной прямой UV: излом `r`
в тысячных alpha), а контур на карте не выпуклый (`_bilinear_ring` требует строгой выпуклости), раньше шёл
в уши (`UV_NOT_AFFINE`). Уши даёт первая допустимая вершина списка: диагональ идёт из вершины фронта в
угол источника под любым углом, треугольники тонкие (угол меньше пяти градусов), а резка по граням
источника потом множит каждое ухо. Мусорная линия здесь — не рельеф, а побочный продукт порядка обхода.

ЗАКОН. Образ куска в UV — простой многоугольник `(s, r)`; станция `s` — естественная ось полосы. Закон
берёт ВЕРТИКАЛЬНОЕ (по постоянному `s`) трапециевидное разбиение этого образа: из каждой вершины-угла
разрез вдоль `s = const` до ближайших границ выше и ниже, внутри куска (классическое разбиение простого
многоугольника на трапеции). Равные `s` решаются ТОЧНО, без допуска: вершины одного `s` образуют одну
вертикаль, вертикальные стороны и вершины на вертикали входят в границы граней как есть. Конец разреза
на ребре получает положение на карте и `(s, r)` ЛИНЕЙНОЙ интерполяцией вдоль этого ребра (ребро прямое
и на карте, и в UV, поэтому UV остаётся той же, какой её и интерполирует меш; точная арифметика
`SqrtSumV1`). Вершина, стоящая на прямой и в UV, и на карте, угла не образует и разрезов не даёт, но
остаётся вершиной граней (её ждёт T-стык соседа). Каждая трапеция (треугольник, четырёхгранье либо
многоугольник с прямыми вершинами на сторонах) допускается ТЕМ ЖЕ законом, что и целый кусок:
треугольник, аффинная UV, либо строго выпуклый многоугольник потока с билинейной UV
(`_bilinear_ring`, закон `QUAD_UV_BILINEAR_V1`).

ДОКАЗАТЕЛЬСТВО (правило 4 `AGENTS.md`, всё точно, допусков нет). Разбиение — допустимое планарное
разбиение куска, если: (1) образ в UV прост (нет повторных вершин, пересечений, шипов); (2) каждая грань —
простой многоугольник ТОЙ ЖЕ ориентации на карте, что и кусок (этим же знаком исключается складка UV:
вернувшаяся ориентация образа); (3) полурёбра граней сокращаются парами (внутренние разрезы — по одному
в каждую сторону), а остаток — РОВНО подразделённый контур куска: тогда сумма индикаторов граней равна
числу обмотки простого контура, то есть грани покрывают кусок один раз без наложений и щелей; (4) новые
вершины лежат на своих рёбрах строго внутри (нулевая ориентация на карте и в UV, доля в `(0, 1)`); (5)
сумма удвоенных площадей граней равна площади куска (проверяет вызывающий). Любой отказ — ВЕСЬ кусок
остаётся на ушах (прежний путь, побитово) под ИМЕНЕМ причины (`REFUSED_*`): тихого запасного пути нет.

ШОВ И T-СТЫК (инвариант проекта, не допуск). Новая вершина на ребре, которое делят два куска домена,
оставила бы соседу T-стык; на ребре источника или стены она открыла бы шов с соседним доменом (хост
сваривает только `src:`, `ADAPTER_SEAM_T_JUNCTIONS`), как и вершина резки (`clip.seam_edges`). Поэтому закон
ставит новые вершины ТОЛЬКО на свободные рёбра (граничные полурёбра фронта, `RIM`); ребро общее у двух
кусков — `REFUSED_ENDPOINT_ON_SHARED_EDGE`, ребро источника или стены — `REFUSED_ENDPOINT_ON_SEAM_EDGE`
(политика `SEAM_ENDPOINTS_ALLOWED`, по умолчанию закрыта: решение владельца, а не ядра). Сколько граней
дал бы отказанный кусок, записано (`REFUSED_WOULD_HAVE_FACES`), чтобы решение принималось по числам.

БЛИЗКИЕ СТАНЦИИ НЕ СЛИВАЮТСЯ: геометрия не двигается и не снапится; закон только ЗАПИСЫВАЕТ, сколько выпущенных
граней уже одной сотой alpha по `s` и сколько имеют угол меньше пяти градусов на карте. Эти два числа —
запись, а не суд: ответ от них не зависит, поэтому в реестр допусков они не ходят.

ИЗМЕРЕНО НА ПОЛЕ (headless, `E:/testScene.blend`, Max stretch 42 %, Fan Density 2, alpha 0.987, меш `sagging_wall`; патч 1 —
фикстура `sagging_wall_slab_stations_v1`). Все 13 кусков с ушами патча 1 имеют источник внизу и общее с соседом ребро
вверху, а каждый разрез от вершины фронта кончается на источнике: под политикой шва по умолчанию закон отказан 13 из 13
(`REFUSED_ENDPOINT_ON_SEAM_EDGE`, дал бы 38 граней), ответ побитово прежний, а кусок разбивает вторая ступень
(`convex_partition`: диагонали между своими вершинами, вершин нет). Политика шва ОТКРЫТА (контроль, не умолчание): 12 кусков из 13
разбиты (35 трапеций, 23 вершины на цепях источника), а хост пишет `ADAPTER_SEAM_T_JUNCTIONS` = 6; домен даёт 172 грани и 355
рёбер (184 и 375 без второй ступени) против 153 граней и 302 рёбер у разбиения диагоналями без единой вершины на шве. Закон станций на этих кусках ДОМИНИРУЕТСЯ
разбиением диагоналями и по числу граней, и по шву; он стоит первым ступенью потому, что так записан закон владельца, и
срабатывает там, где разрез законно кончается на свободном ребре фронта.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from functools import cmp_to_key

from ..exact_sqrt_sum import SqrtSumV1
from ..wavefront.faces import orientation, shoelace_sign
from .tessellate import contour_is_simple

#: Политика шва: можно ли ставить новую вершину на ребро источника или стены (`SOURCE`, `WALL`). По умолчанию НЕТ:
#: вершина на цепи, общей с соседним доменом, — T-стык шва (`ADAPTER_SEAM_T_JUNCTIONS`), и закон ядра их там не
#: допускает (`clip.seam_edges`). Включать её — решение владельца; закон от политики не меняется, меняется только
#: число отказов `REFUSED_ENDPOINT_ON_SEAM_EDGE`.
SEAM_ENDPOINTS_ALLOWED = False

#: Имена чисел закона (они же ключи счётчиков материализатора). Нулевые числа в счётчики не пишутся.
PIECES_DECOMPOSED = "MATERIALIZE_SLAB_PIECES_DECOMPOSED"
FACES_EMITTED = "MATERIALIZE_SLAB_FACES_EMITTED"
CUTS = "MATERIALIZE_SLAB_CUTS"
VERTICES_INSERTED = "MATERIALIZE_SLAB_VERTICES_INSERTED"
FACES_THIN = "MATERIALIZE_SLAB_FACES_THINNER_THAN_ONE_PERCENT_OF_ALPHA"
FACES_SLIVER = "MATERIALIZE_SLAB_FACES_MIN_ANGLE_UNDER_FIVE_DEGREES_IN_CHART"
PIECES_REFUSED = "MATERIALIZE_SLAB_PIECES_REFUSED"
REFUSED_WOULD_HAVE_FACES = "MATERIALIZE_SLAB_REFUSED_WOULD_HAVE_FACES"
REFUSED_PREFIX = "MATERIALIZE_SLAB_REFUSED_"

REASON_UV_NOT_SIMPLE = "UV_NOT_SIMPLE"
REASON_DEGENERATE = "DEGENERATE"
REASON_NOT_A_SUBDIVISION = "NOT_A_SUBDIVISION"
REASON_SEAM_EDGE = "ENDPOINT_ON_SEAM_EDGE"
REASON_SHARED_EDGE = "ENDPOINT_ON_SHARED_EDGE"
REASON_NOT_ON_CONTOUR = "EDGE_NOT_ON_CONTOUR"
REASON_NOT_ADMITTED = "SLAB_NOT_ADMITTED"
REASONS = (
    REASON_UV_NOT_SIMPLE,
    REASON_DEGENERATE,
    REASON_NOT_A_SUBDIVISION,
    REASON_SEAM_EDGE,
    REASON_SHARED_EDGE,
    REASON_NOT_ON_CONTOUR,
    REASON_NOT_ADMITTED,
)

#: Записываемые пороги (запись, а не суд; ответ от них не зависит): грань уже `THIN_PERCENT_OF_ALPHA` процента
#: alpha по `s` — «тонкая станция»; угол меньше пяти градусов на карте — тангенс квадрата угла как отношение целых,
#: сравнение точное (`tan^2(5 deg) = 0.0076542...`, числитель взят с избытком в восьмой цифре).
THIN_PERCENT_OF_ALPHA = 1
_SLIVER_TAN_SQUARED_NUMERATOR = 76543
_SLIVER_TAN_SQUARED_DENOMINATOR = 10_000_000

Point = tuple[SqrtSumV1, SqrtSumV1]


@dataclass(frozen=True, slots=True)
class SlabPointV1:
    """Новая вершина: на ребре кольца `edge = (a, b)` (индексы кольца, обход UV против часовой), в доле `share`."""

    edge: tuple[int, int]
    share: SqrtSumV1
    point: Point
    value: Point


@dataclass(frozen=True, slots=True)
class SlabPlanV1:
    """Разбиение куска: грани узлами (`0..n-1` — вершины кольца, `n + k` — `points[k]`), обход UV против часовой."""

    faces: tuple
    points: tuple
    #: Кольцо в обходе UV против часовой (индексы входного кольца) и знак площади куска на карте в этом обходе.
    order: tuple
    map_sign: int


@dataclass(frozen=True, slots=True)
class SlabRefusalV1:
    """Именованный отказ закона: куска не разбили, и причина названа (`REASON_*`)."""

    reason: str
    detail: str
    would_have: int = 0


def _cmp(left: SqrtSumV1, right: SqrtSumV1, budget) -> int:
    return left.difference_sign(right, budget)


def _lex(first: Point, second: Point, budget) -> int:
    """Лексикографический порядок `(s, r)`: знак `first - second`; равные `s` решает `r` (точный сдвиг)."""

    by_s = _cmp(first[0], second[0], budget)
    return by_s if by_s else _cmp(first[1], second[1], budget)


class _NotSimple(Exception):
    """Два ребра образа перекрываются: порядок по высоте не определён (образ не прост)."""


class _Refused(Exception):
    """Новую вершину на этом ребре ставить нельзя (`endpoint_refusal`): причина названа."""

    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason


class _Planner:
    """Одно разбиение: состояние шагов от углов до граней. Не живёт дольше вызова `plan_slabs`."""

    def __init__(self, points, values, budget, order, map_sign, may_insert) -> None:
        self.map = points
        self.may_insert = may_insert
        self.uv = values
        self.budget = budget
        self.order = order
        self.map_sign = map_sign
        self.count = len(points)
        self.edges: list = []
        self.stations: list = []
        self.new_points: list = []
        self._new_index: dict = {}

    # ---- углы и виртуальные рёбра ---------------------------------------

    def corners(self):
        """Позиции обхода, где граница поворачивает (в UV либо на карте); `None` — шип в образе."""

        budget, count = self.budget, self.count
        found = []
        for position in range(count):
            before = self.order[position - 1]
            here = self.order[position]
            after = self.order[(position + 1) % count]
            uv_turn = orientation(self.uv[before], self.uv[here], self.uv[after], budget)
            map_turn = orientation(self.map[before], self.map[here], self.map[after], budget)
            if uv_turn == 0:
                forward = (
                    (self.uv[here][0] - self.uv[before][0]) * (self.uv[after][0] - self.uv[here][0])
                    + (self.uv[here][1] - self.uv[before][1]) * (self.uv[after][1] - self.uv[here][1])
                ).sign(budget=budget)
                if forward <= 0:
                    return None
            if uv_turn != 0 or map_turn != 0:
                found.append(position)
        return found

    def build_edges(self, corners) -> None:
        """Виртуальное ребро — цепочка вершин между двумя соседними углами (прямые вершины внутри)."""

        count, budget = self.count, self.budget
        for number, start in enumerate(corners):
            end = corners[(number + 1) % len(corners)]
            chain = tuple(self.order[(start + step) % count] for step in range((end - start) % count + 1))
            order = _lex(self.uv[chain[0]], self.uv[chain[-1]], budget)
            low, high = (chain[0], chain[-1]) if order < 0 else (chain[-1], chain[0])
            self.edges.append(
                _Edge(chain, low, high, order < 0, _cmp(self.uv[low][0], self.uv[high][0], budget) == 0)
            )

    def build_stations(self, corners) -> None:
        """Различные `s` углов по возрастанию: границы полос."""

        budget = self.budget
        ranked = sorted(
            (self.order[position] for position in corners),
            key=cmp_to_key(lambda first, second: _cmp(self.uv[first][0], self.uv[second][0], budget)),
        )
        for index in ranked:
            if not self.stations or _cmp(self.uv[index][0], self.stations[-1], budget):
                self.stations.append(self.uv[index][0])

    # ---- полосы и трапеции -----------------------------------------------

    def _below(self, first: "_Edge", second: "_Edge") -> int:
        """`-1`, если ребро `first` ниже `second` в общей полосе, `1` — выше. Нет решения — `_NotSimple`."""

        budget, uv = self.budget, self.uv
        p1, q1, p2, q2 = uv[first.low], uv[first.high], uv[second.low], uv[second.high]
        if _cmp(p1[0], p2[0], budget) >= 0:
            sign = orientation(p2, q2, p1, budget)
            if sign:
                return 1 if sign > 0 else -1
        else:
            sign = orientation(p1, q1, p2, budget)
            if sign:
                return -1 if sign > 0 else 1
        if _cmp(q1[0], q2[0], budget) <= 0:
            sign = orientation(p2, q2, q1, budget)
            if sign:
                return 1 if sign > 0 else -1
        else:
            sign = orientation(p1, q1, q2, budget)
            if sign:
                return -1 if sign > 0 else 1
        raise _NotSimple

    def trapezoids(self):
        """`[(нижнее ребро, верхнее ребро, левая станция, правая станция), ...]`; соседние полосы с теми же рёбрами слиты."""

        budget, uv = self.budget, self.uv
        sloped = [number for number, edge in enumerate(self.edges) if not edge.vertical]
        opened: dict = {}
        done: list = []
        for strip in range(len(self.stations) - 1):
            left, right = self.stations[strip], self.stations[strip + 1]
            spanning = [
                number
                for number in sloped
                if _cmp(uv[self.edges[number].low][0], left, budget) <= 0
                and _cmp(uv[self.edges[number].high][0], right, budget) >= 0
            ]
            if len(spanning) % 2:
                raise _NotSimple
            spanning.sort(key=cmp_to_key(lambda a, b: self._below(self.edges[a], self.edges[b])))
            for offset in range(0, len(spanning), 2):
                bottom, top = spanning[offset], spanning[offset + 1]
                if not self.edges[bottom].forward or self.edges[top].forward:
                    raise _NotSimple
                found = opened.get((bottom, top))
                if found is not None and found[1] == strip:
                    found[1] = strip + 1
                    continue
                if found is not None:
                    done.append((bottom, top, found[0], found[1]))
                opened[(bottom, top)] = [strip, strip + 1]
        done.extend((bottom, top, first, last) for (bottom, top), (first, last) in opened.items())
        return sorted(done)

    # ---- вершины граней --------------------------------------------------

    def _new_point(self, first: int, second: int, station: SqrtSumV1):
        """Номер новой вершины на ребре `first -> second` при `s = station` (одна точка на пару «ребро, станция»)."""

        key = (first, second, station)
        number = self._new_index.get(key)
        if number is None:
            reason = None if self.may_insert is None else self.may_insert(first, second)
            if reason is not None:
                raise _Refused(reason)
            budget, map_, uv = self.budget, self.map, self.uv
            share = (station - uv[first][0]).divided_by(uv[second][0] - uv[first][0], budget)
            point = tuple(map_[first][axis] + (map_[second][axis] - map_[first][axis]) * share for axis in (0, 1))
            value = (station, uv[first][1] + (uv[second][1] - uv[first][1]) * share)
            number = self._new_index[key] = self.count + len(self.new_points)
            self.new_points.append(SlabPointV1((first, second), share, point, value))
        return number

    def _locate(self, edge: "_Edge", station: SqrtSumV1) -> int:
        """Узел ребра при `s = station`: вершина, если `s` вершины, иначе новая точка на звене."""

        budget, uv = self.budget, self.uv
        for first, second in zip(edge.chain, edge.chain[1:]):
            at_first = _cmp(uv[first][0], station, budget)
            at_second = _cmp(uv[second][0], station, budget)
            if at_first == 0:
                return first
            if at_second == 0:
                return second
            if at_first * at_second < 0:
                return self._new_point(first, second, station)
        raise _NotSimple

    def _between(self, edge: "_Edge", low: SqrtSumV1, high: SqrtSumV1) -> list:
        """Вершины цепочки ребра СТРОГО между `s = low` и `s = high` (в порядке обхода цепочки)."""

        budget, uv = self.budget, self.uv
        return [
            index
            for index in edge.chain
            if _cmp(uv[index][0], low, budget) > 0 and _cmp(uv[index][0], high, budget) < 0
        ]

    def _on_line(self, station: SqrtSumV1, below, above) -> list:
        """Вершины образа на вертикали `s = station` строго между узлами `below` и `above` по `r`, по возрастанию `r`."""

        budget, uv = self.budget, self.uv
        r_below, r_above = self._value_of(below)[1], self._value_of(above)[1]
        found = [
            index
            for index in range(self.count)
            if _cmp(uv[index][0], station, budget) == 0
            and _cmp(uv[index][1], r_below, budget) > 0
            and _cmp(uv[index][1], r_above, budget) < 0
        ]
        found.sort(key=cmp_to_key(lambda a, b: _cmp(uv[a][1], uv[b][1], budget)))
        return found

    def _value_of(self, node: int) -> Point:
        return self.uv[node] if node < self.count else self.new_points[node - self.count].value

    def face_of(self, trapezoid) -> tuple:
        """Кольцо узлов трапеции против часовой в UV: низ слева направо, правая сторона вверх, верх справа налево."""

        bottom_edge, top_edge, first, last = (
            self.edges[trapezoid[0]],
            self.edges[trapezoid[1]],
            trapezoid[2],
            trapezoid[3],
        )
        left, right = self.stations[first], self.stations[last]
        bottom_left, bottom_right = self._locate(bottom_edge, left), self._locate(bottom_edge, right)
        top_left, top_right = self._locate(top_edge, left), self._locate(top_edge, right)
        ring = [bottom_left, *self._between(bottom_edge, left, right), bottom_right]
        ring += self._on_line(right, bottom_right, top_right)
        ring += [top_right, *self._between(top_edge, left, right), top_left]
        ring += reversed(self._on_line(left, bottom_left, top_left))
        clean: list = []
        for node in ring:
            if not clean or clean[-1] != node:
                clean.append(node)
        if len(clean) > 1 and clean[0] == clean[-1]:
            clean.pop()
        return tuple(clean)


@dataclass(frozen=True, slots=True)
class _Edge:
    """Виртуальное ребро: цепочка индексов кольца между углами, её лексикографические концы и признаки."""

    chain: tuple
    low: int
    high: int
    forward: bool
    vertical: bool


def plan_slabs(points, values, budget, may_insert=None):
    """`SlabPlanV1` — вертикальное трапециевидное разбиение образа куска — либо `SlabRefusalV1`.

    `points[i]` — точка карты вершины кольца, `values[i]` — её точные `(s, r)`. Закон геометрический и допусков не
    знает: каждый знак точный. Кто сосед ребра, закон не знает тоже: `may_insert(a, b)` — вопрос вызывающему, можно
    ли поставить новую вершину на звено кольца `a -> b` (индексы входного кольца): `None` — можно, строка — причина
    отказа (`endpoint_refusal`). Вопрос задаётся ДО точных делений: отказанный кусок стоит только знаков порядка, а
    число трапеций, которое он дал бы, записано в отказе (`would_have`).
    """

    count = len(points)
    if count < 4:
        return SlabRefusalV1(REASON_DEGENERATE, f"{count} vertices")
    for first in range(count):
        for second in range(first + 1, count):
            if not _cmp(values[first][0], values[second][0], budget) and not _cmp(
                values[first][1], values[second][1], budget
            ):
                return SlabRefusalV1(REASON_UV_NOT_SIMPLE, f"vertices {first} and {second} share one UV point")
    if not contour_is_simple(tuple(values), budget):
        return SlabRefusalV1(REASON_UV_NOT_SIMPLE, "the UV image of the piece is not a simple polygon")
    uv_sign = shoelace_sign(tuple(values), budget)
    map_sign = shoelace_sign(tuple(points), budget)
    if not uv_sign or not map_sign:
        return SlabRefusalV1(REASON_DEGENERATE, "zero area")
    order = tuple(range(count)) if uv_sign > 0 else tuple(range(count - 1, -1, -1))
    # Знак площади куска на карте В ОБХОДЕ ПО UV против часовой: грани строятся в нём же и обязаны совпасть с ним.
    map_sign *= 1 if uv_sign > 0 else -1
    planner = _Planner(points, values, budget, order, map_sign, may_insert)
    corners = planner.corners()
    if corners is None:
        return SlabRefusalV1(REASON_UV_NOT_SIMPLE, "a spike in the UV image")
    if len(corners) < 3:
        return SlabRefusalV1(REASON_DEGENERATE, f"{len(corners)} corners")
    try:
        planner.build_edges(corners)
        planner.build_stations(corners)
        trapezoids = planner.trapezoids()
    except _NotSimple:
        return SlabRefusalV1(REASON_UV_NOT_SIMPLE, "the order of edges over a strip is not defined")
    try:
        faces = tuple(planner.face_of(item) for item in trapezoids)
    except _NotSimple:
        return SlabRefusalV1(REASON_UV_NOT_SIMPLE, "a trapezoid side is not on its edge", len(trapezoids))
    except _Refused as refused:
        return SlabRefusalV1(refused.reason, "", len(trapezoids))
    if not faces or any(len(face) < 3 for face in faces):
        return SlabRefusalV1(REASON_DEGENERATE, "a trapezoid has fewer than three vertices")
    return SlabPlanV1(faces, tuple(planner.new_points), order, map_sign)


def verify_plan(points, values, plan: SlabPlanV1, budget):
    """Доказательство разбиения (см. докстринг модуля, пункты 2-4): `(число разрезов, None)` либо `(0, отказ)`.

    Узлы — вершины кольца и новые точки; координаты и UV новых точек берутся из плана. Пункт 5 (сумма площадей)
    остаётся вызывающему: он держит площадь куска.
    """

    count = len(points)
    coords = list(points) + [item.point for item in plan.points]
    uvs = list(values) + [item.value for item in plan.points]
    for item in plan.points:
        first, second = item.edge
        on_edge = (
            not orientation(points[first], points[second], item.point, budget)
            and not orientation(values[first], values[second], item.value, budget)
            and item.share.sign(budget=budget) > 0
            and (SqrtSumV1.rational(1) - item.share).sign(budget=budget) > 0
        )
        if not on_edge:
            return 0, SlabRefusalV1(REASON_NOT_A_SUBDIVISION, f"a cut vertex is not strictly inside edge {item.edge}")
    inserted: dict = {}
    for number, item in enumerate(plan.points):
        inserted.setdefault(item.edge, []).append((item.share, count + number))
    boundary: list = []
    for position in range(count):
        first, second = plan.order[position], plan.order[(position + 1) % count]
        chain = [first]
        chain += [
            node
            for _share, node in sorted(
                inserted.get((first, second), ()),
                key=cmp_to_key(lambda a, b: _cmp(a[0], b[0], budget)),
            )
        ]
        chain.append(second)
        boundary.extend(zip(chain, chain[1:]))
    used: Counter = Counter()
    for face in plan.faces:
        if len(set(face)) != len(face):
            return 0, SlabRefusalV1(REASON_NOT_A_SUBDIVISION, "a face repeats a vertex")
        ring = tuple(coords[node] for node in face)
        if shoelace_sign(ring, budget) != plan.map_sign or shoelace_sign(tuple(uvs[node] for node in face), budget) <= 0:
            return 0, SlabRefusalV1(REASON_NOT_A_SUBDIVISION, "a face has the opposite orientation to the piece")
        if not contour_is_simple(ring, budget):
            return 0, SlabRefusalV1(REASON_NOT_A_SUBDIVISION, "a face is not a simple polygon")
        used.update(zip(face, face[1:] + face[:1]))
    on_boundary = set(boundary)
    if len(on_boundary) != len(boundary):
        return 0, SlabRefusalV1(REASON_NOT_A_SUBDIVISION, "the refined contour repeats an edge")
    interior = 0
    for (first, second), times in used.items():
        if times != 1:
            return 0, SlabRefusalV1(REASON_NOT_A_SUBDIVISION, "an edge is used twice in one direction")
        if (second, first) in used:
            if (first, second) in on_boundary:
                return 0, SlabRefusalV1(REASON_NOT_A_SUBDIVISION, "a contour edge is also an interior cut")
            interior += int(first < second)
        elif (first, second) not in on_boundary:
            return 0, SlabRefusalV1(REASON_NOT_A_SUBDIVISION, "a face edge has no partner and is not on the contour")
    if any(edge not in used for edge in boundary):
        return 0, SlabRefusalV1(REASON_NOT_A_SUBDIVISION, "a contour edge is on no face")
    return interior, None


def face_notes(points, values, plan: SlabPlanV1, unit, budget) -> tuple:
    """`(тонких станций, граней с углом меньше пяти градусов на карте)` по граням плана — ЗАПИСЬ, не суд.

    Тонкая — ширина по `s` меньше `THIN_PERCENT_OF_ALPHA` процента alpha (`unit`, alpha в единицах решётки).
    Угол — точное сравнение `cross^2 / dot^2` с `tan^2` пяти градусов на остром углу грани (`dot > 0`).
    """

    coords = list(points) + [item.point for item in plan.points]
    uvs = list(values) + [item.value for item in plan.points]
    alpha = SqrtSumV1.rational(unit)
    thin = sliver = 0
    for face in plan.faces:
        stations = [uvs[node][0] for node in face]
        low = min(stations, key=cmp_to_key(lambda a, b: _cmp(a, b, budget)))
        high = max(stations, key=cmp_to_key(lambda a, b: _cmp(a, b, budget)))
        thin += int((high - low).scaled(100).difference_sign(alpha.scaled(THIN_PERCENT_OF_ALPHA), budget) < 0)
        size = len(face)
        for position in range(size):
            here = coords[face[position]]
            first = (coords[face[position - 1]][0] - here[0], coords[face[position - 1]][1] - here[1])
            second = (coords[face[(position + 1) % size]][0] - here[0], coords[face[(position + 1) % size]][1] - here[1])
            dot = first[0] * second[0] + first[1] * second[1]
            if dot.sign(budget=budget) <= 0:
                continue
            cross = first[0] * second[1] - first[1] * second[0]
            sharp = (cross * cross).scaled(_SLIVER_TAN_SQUARED_DENOMINATOR).difference_sign(
                (dot * dot).scaled(_SLIVER_TAN_SQUARED_NUMERATOR), budget
            )
            if sharp < 0:
                sliver += 1
                break
    return thin, sliver


def endpoint_refusal(kind: str) -> str | None:
    """Причина, по которой новую вершину на ребро вида `kind` ставить нельзя, либо `None`.

    `kind`: `SHARED` — ребро общее у двух кусков домена (T-стык соседа), `RIM` — свободная граница фронта,
    `SOURCE` и `WALL` — граница по контуру патча (шов с соседним доменом, политика `SEAM_ENDPOINTS_ALLOWED`).
    """

    if kind == "SHARED":
        return REASON_SHARED_EDGE
    if kind == "RIM" or SEAM_ENDPOINTS_ALLOWED:
        return None
    return REASON_SEAM_EDGE


def slab_counters(tally) -> tuple:
    """Числа закона из `tally`, только ненулевые: домен, где закон не сработал, не получает новых строк."""

    names = (
        PIECES_DECOMPOSED,
        FACES_EMITTED,
        CUTS,
        VERTICES_INSERTED,
        FACES_THIN,
        FACES_SLIVER,
        PIECES_REFUSED,
        REFUSED_WOULD_HAVE_FACES,
        *(REFUSED_PREFIX + reason for reason in REASONS),
    )
    return tuple((name, tally[name]) for name in names if tally[name])


class EdgeLedgerV1:
    """Кто владеет рёбрами домена: ребро общее, если его полурёбра (в любую сторону) встречаются у двух кусков.

    `pieces` — `[(грань кадра, цикл части), ...]` по ВСЕМ частям всех слитых граней домена (веера входят: их
    контур — тоже край соседа). Спрашивают ребро в обходе закона (UV против часовой), а цикл куска может идти в обратную
    сторону, поэтому ребро — НЕУПОРЯДОЧЕННАЯ пара ключей. Вид граничного ребра — по фактам `(s, r)` его концов: `SOURCE`
    (`r = 0` на обоих), `RIM` (`r = alpha`), иначе `WALL`; те же правила, что у цепей батча (`assemble.edge_kind`).
    """

    def __init__(self, pieces, uv_values, unit) -> None:
        self._owner: Counter = Counter()
        for _frame_face, cycle in pieces:
            size = len(cycle)
            for position in range(size):
                self._owner[frozenset((cycle[position][0], cycle[(position + 1) % size][0]))] += 1
        self._uv_values = uv_values
        self._front = SqrtSumV1.rational(unit)

    def kind(self, frame_face, first: str, second: str) -> str:
        if self._owner[frozenset((first, second))] > 1:
            return "SHARED"
        r_first, r_second = self._uv_values(frame_face, first)[1], self._uv_values(frame_face, second)[1]
        if r_first.is_zero and r_second.is_zero:
            return "SOURCE"
        if (r_first - self._front).is_zero and (r_second - self._front).is_zero:
            return "RIM"
        return "WALL"


@dataclass(frozen=True, slots=True)
class SlabVertexV1:
    """Вершина `slab:<k>`, рождённая законом: на ребре `first -> second` контура грани кадра `frame_index`."""

    frame_index: int
    key: str
    first: str
    second: str
    share: SqrtSumV1
    point: Point
    value: Point


class SlabSinkV1:
    """Вершины закона на домен: ключи `slab:<k>` по порядку рождения, вставка в контуры граней, факты `(s, r)`."""

    def __init__(self) -> None:
        self.vertices: list = []

    def __bool__(self) -> bool:
        return bool(self.vertices)

    def new_key(self) -> str:
        return f"slab:{len(self.vertices)}"

    def add(self, vertex: SlabVertexV1) -> None:
        self.vertices.append(vertex)

    def points(self) -> dict:
        return {item.key: item.point for item in self.vertices}

    def facts(self, region_of) -> dict:
        """`{(регион, ключ): (s, r)}`; `region_of(индекс грани кадра)` — регион грани."""

        return {(region_of(item.frame_index), item.key): item.value for item in self.vertices}

    def refined_cycles(self, cycles, budget) -> list:
        """Контуры граней с новыми вершинами на своих рёбрах, по возрастанию доли вдоль ребра в направлении контура."""

        grouped: dict = {}
        for item in self.vertices:
            grouped.setdefault(item.frame_index, []).append(item)
        refined = list(cycles)
        for index, items in grouped.items():
            cycle = list(cycles[index])
            size = len(cycle)
            slots: dict = {}
            for item in items:
                for position in range(size):
                    here, after = cycle[position][0], cycle[(position + 1) % size][0]
                    if (here, after) == (item.first, item.second):
                        slots.setdefault(position, []).append((item, False))
                        break
                    if (here, after) == (item.second, item.first):
                        slots.setdefault(position, []).append((item, True))
                        break
                else:
                    raise ValueError(f"{item.key} is on no contour edge of face {index}")
            out: list = []
            for position in range(size):
                out.append(cycle[position])
                found = slots.get(position, ())
                found = sorted(
                    found,
                    key=cmp_to_key(lambda a, b: _cmp(a[0].share, b[0].share, budget) * (-1 if a[1] else 1)),
                )
                out.extend((item.key, item.point) for item, _reversed in found)
            refined[index] = out
        return refined
