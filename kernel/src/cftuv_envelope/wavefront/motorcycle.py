"""Motorcycle graph: трассы reflex-вершин и достаточное условие теоремы 2.11.

Что это. Каждая reflex-вершина полигона выпускает «мотоцикл» по той самой
биссектрисе, по которой пойдёт reflex-вершина волны. Мотоцикл едет, пока не
разобьётся: о СТЕНУ (исходное ребро полигона) или о ТРАССУ другого мотоцикла,
который был в этой точке раньше; при одновременном приходе гасятся оба. Набор
трасс и есть motorcycle graph (Huber & Held). Код оттуда не переносился —
взята идея, см. `DECISIONS.md`, 2026-07-25.

**Теорема 2.11 (Huber).** Reflex-вершина волны не уходит за свою трассу. Отсюда
достаточное условие, которого не хватало наивной проверке: время split-события
вершины не больше времени крушения её мотоцикла. Проверка «точка попала в
текущий отрезок фронта» необходима, но НЕ достаточна — это уже измерено:
перепорождение кандидатов только по попаданию в отрезок дало 39 неразрешённых
входов вместо 15 и вдвое медленнее.

Из теоремы следует и пространственное сужение, ради которого срез. Точка split
лежит на трассе, а расстояние от неё до несущей прямой рассекаемого ребра равно
времени события, УМНОЖЕННОМУ НА ЕВКЛИДОВУ СКОРОСТЬ этого ребра: из
`a*p_x + b*p_y = c + t*sqrt(q)` следует `dist(p, line) = t*sqrt(q)/|(a, b)|`.
При единичной скорости `q = a^2 + b^2`, множитель равен единице, и получается
прежнее `dist(p, line) = t` — то самое утверждение, на котором теорема стояла,
пока стен и весов не было. Значит несущая прямая кандидата проходит не дальше
`t_crash * speed_bound` от трассы, и кандидатов можно брать из ячеек сетки
вокруг трассы вместо перебора всех рёбер. Это `TraceCandidateIndexV1`, а
`speed_bound_of` — та самая целая граница скорости, равная единице всюду, где
скорости единичны.

Слабейшая форма условия, верная ВСЕГДА и не требующая теоремы: волна не выходит
за границу области, поэтому вершина не уходит за первое пересечение своей
биссектрисы со стеной. Ею ограничиваются вершины, рождённые во время счёта:
motorcycle graph определён на reflex-вершинах ВХОДА, у них время старта ноль, и
только для них крушение о трассу имеет смысл.

Стена — это `SupportLineV1` с `q = 0`. Не хитрость представления, а точное
утверждение: `a*x + b*y = c + t*sqrt(0)` — та же прямая при любом t, поэтому
`concurrency_time` работает над стенами без единой правки.

Порогов здесь нет. Сетка (`cell_grid`) — фильтр без права решать; все решения
принимают точные предикаты `sqrt_sum`. Всякий отказ назван: у трассы, которую не
удалось ограничить, `outcome != EXACT` и `crash_time is None`, и вызывающий
обязан уйти на полный перебор, а не считать её бесконечной.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from fractions import Fraction
import heapq
import itertools
import math

from .cell_grid import CellGridV1, CellIndexV1
from .event_time import (
    ZERO_TIME,
    EventPointV1,
    EventTimeOutcome,
    EventTimeV1,
    SupportLineV1,
    compare_times,
    concurrency_time,
    event_point,
)
from .polygon import PolygonV1
from .sqrt_sum import SqrtSumV1


ENCLOSURE_BITS = 64
# Бюджет уточнения оболочки при поиске рационального ВЕРХНЕГО предела времени.
# Знаменатель времени положителен по нормировке, поэтому оболочка рано или поздно
# отделяет его от нуля; шагов нужно немного, но их число объявлено, а исчерпание
# бюджета — именованный исход.
UPPER_BOUND_DOUBLINGS = 12


class CrashKind(str, Enum):
    """Чем кончилась трасса. `NONE` — не кончилась ничем известным."""

    NONE = "NONE"
    WALL = "WALL"
    TRACE = "TRACE"
    SIMULTANEOUS = "SIMULTANEOUS"


class TraceOutcome(str, Enum):
    """Почему трасса не ограничена. Ни одного тихого возврата."""

    EXACT = "EXACT"
    MOTORCYCLE_HAS_NO_BISECTOR = "MOTORCYCLE_HAS_NO_BISECTOR"
    MOTORCYCLE_NEVER_MEETS_A_WALL = "MOTORCYCLE_NEVER_MEETS_A_WALL"
    MOTORCYCLE_MARCH_BUDGET_EXHAUSTED = "MOTORCYCLE_MARCH_BUDGET_EXHAUSTED"


@dataclass(frozen=True, slots=True)
class WallV1:
    """Исходное ребро полигона как НЕПОДВИЖНАЯ прямая (`q = 0`) плюс концы."""

    ident: int
    start: tuple[int, int]
    end: tuple[int, int]
    line: SupportLineV1


def walls_of(polygon: PolygonV1) -> tuple[WallV1, ...]:
    """Стены в том же порядке, в каком `_seed` создаёт рёбра фронта.

    Совпадение нумерации — не совпадение: оба обхода идут по `polygon.loops` и
    внутри контура по его точкам, поэтому стена `i` и ребро `i` — одно ребро.
    """

    walls: list[WallV1] = []
    for loop in polygon.loops:
        points = loop.points
        size = len(points)
        for index in range(size):
            start, end = points[index], points[(index + 1) % size]
            moving = SupportLineV1.through(start, end)
            walls.append(
                WallV1(
                    len(walls),
                    start,
                    end,
                    SupportLineV1(moving.a, moving.b, moving.c, 0),
                )
            )
    return tuple(walls)


def bisector_velocity(
    left: SupportLineV1, right: SupportLineV1
) -> tuple[SqrtSumV1, SqrtSumV1] | None:
    """`dp/dt` вершины, стоящей на стыке двух движущихся прямых.

    Из `p(t)` как решения системы двух прямых: `x` и `y` аффинны по `t`, и
    производная выписывается явно, без деления на `SqrtSum`. Явная формула здесь
    не оптимизация: она избавляет от сопряжений, а `sign_by_conjugation` в этом
    срезе обязан остаться нулём.
    """

    determinant = left.a * right.b - right.a * left.b
    if determinant == 0:
        return None
    scale = Fraction(1, determinant)
    x = (
        SqrtSumV1.radical(right.b, left.q) - SqrtSumV1.radical(left.b, right.q)
    ).scaled(scale)
    y = (
        SqrtSumV1.radical(left.a, right.q) - SqrtSumV1.radical(right.a, left.q)
    ).scaled(scale)
    return x, y


@dataclass(frozen=True, slots=True)
class TraceV1:
    """Трасса: от места старта до крушения. Ничего не вычислено, всё точно."""

    ident: int
    outcome: TraceOutcome
    left_line: SupportLineV1
    right_line: SupportLineV1
    start_time: EventTimeV1
    origin: EventPointV1
    velocity: tuple[SqrtSumV1, SqrtSumV1]
    crash_time: EventTimeV1 | None
    crash_point: EventPointV1 | None
    crash_kind: CrashKind
    crash_target: int
    reach: Fraction | None

    def bounds_time(self, time: EventTimeV1) -> bool:
        """ТЕОРЕМА 2.11 в исполняемой форме: событие не позже крушения.

        Трасса без крушения не ограничивает НИЧЕГО и говорит об этом прямо:
        `False` здесь означает «я не могу поручиться», а не «события нет».
        """

        if self.crash_time is None:
            return False
        return compare_times(time, self.crash_time) <= 0

    def box(self) -> tuple[int, int, int, int] | None:
        """Целочисленная рамка трассы, округлённая НАРУЖУ. Только для сетки."""

        if self.crash_point is None:
            return None
        return _point_box((self.origin, self.crash_point))


def _point_box(points: tuple[EventPointV1, ...]) -> tuple[int, int, int, int]:
    xs: list[Fraction] = []
    ys: list[Fraction] = []
    for point in points:
        low, high = point.x.enclosure(ENCLOSURE_BITS)
        xs += [low, high]
        low, high = point.y.enclosure(ENCLOSURE_BITS)
        ys += [low, high]
    return (
        math.floor(min(xs)),
        math.ceil(max(xs)),
        math.floor(min(ys)),
        math.ceil(max(ys)),
    )


def _upper_bound_of_root(value: SqrtSumV1) -> int:
    """Степень двойки `s`, для которой доказано `s >= sqrt(value)`.

    Нужна ровно для длины шага: за шаг `cell/s` по времени луч проходит не
    больше стороны ячейки, а значит рамка шага накрывает весь пройденный участок.
    Оценка сверху — единственное требование; точность здесь не нужна и не
    покупается.
    """

    _low, high = value.enclosure(ENCLOSURE_BITS)
    bound = 1
    while bound * bound < high:
        bound *= 2
    return bound


def _upper_bound_of_time(time: EventTimeV1) -> Fraction | None:
    """Рациональный ВЕРХНИЙ предел неотрицательного времени, либо `None`.

    Знаменатель положителен по нормировке `EventTimeV1`, поэтому
    `t = d/S <= d/lo` при любом доказанном `0 < lo <= S`. Оболочка уточняется
    удваиванием разрядности; исчерпание бюджета — отказ, а не догадка.
    """

    if time.dividend < 0:
        return None
    bits = ENCLOSURE_BITS
    for _ in range(UPPER_BOUND_DOUBLINGS):
        low, _high = time.divisor.enclosure(bits)
        if low > 0:
            return time.dividend / low
        bits *= 2
    return None


def march_budget(grid: CellGridV1) -> int:
    """ОБЪЯВЛЕННАЯ граница числа шагов марша. Превышение — именованный исход.

    Не эвристика. Шаг равен `cell / s`, где `s >= sqrt(|u|^2)` получен
    удваиванием, поэтому `s < 2*|u|` и за шаг луч проходит не меньше половины
    ячейки. Путь внутри области не длиннее её диагонали, то есть не длиннее
    `(columns + rows) * cell`. Отсюда шагов хватает `2*(columns + rows)`, а
    восемь добавлены на выход за область и на округления рамок наружу.
    """

    return 2 * (grid.columns + grid.rows) + 8


def _projection_is_inside(point: EventPointV1, wall: WallV1) -> bool:
    """Точка лежит внутри отрезка стены. Проекция на направление, точный знак."""

    dx = wall.end[0] - wall.start[0]
    dy = wall.end[1] - wall.start[1]
    here = point.x.scaled(dx) + point.y.scaled(dy)
    low = dx * wall.start[0] + dy * wall.start[1]
    high = dx * wall.end[0] + dy * wall.end[1]
    if (here - SqrtSumV1.rational(low)).sign() < 0:
        return False
    return (SqrtSumV1.rational(high) - here).sign() >= 0


@dataclass(order=False, slots=True)
class _CrashEntry:
    """Запись очереди крушений. Сравнение — точное время, как в `events`."""

    time: EventTimeV1
    kind: CrashKind
    motorcycle: int
    target: int
    peer_time: EventTimeV1 | None
    sequence: int

    def __lt__(self, other: "_CrashEntry") -> bool:
        order = compare_times(self.time, other.time)
        return order < 0 if order else False


@dataclass(slots=True)
class MotorcycleGraphV1:
    """Граф трасс и служба, выдающая трассу для вершины, рождённой в счёте."""

    walls: tuple[WallV1, ...]
    grid: CellGridV1
    wall_index: CellIndexV1
    traces: dict[int, TraceV1]
    counters: dict[str, int]
    _next_ident: itertools.count = field(default_factory=itertools.count)

    def counter(self, name: str) -> int:
        return self.counters.get(name, 0)

    def trace_for(
        self,
        left: SupportLineV1,
        right: SupportLineV1,
        start_time: EventTimeV1,
        origin: EventPointV1,
    ) -> TraceV1:
        """Трасса вершины, рождённой во время счёта: крушение только о стену.

        Крушение о трассу здесь не считается намеренно. Motorcycle graph
        определён на reflex-вершинах ВХОДА; у рождённой вершины время старта
        иррационально, и переносить на неё теорему 2.11 было бы заявлением.
        Стенная граница слабее, но верна без всякой теоремы: волна не выходит
        за границу области.
        """

        ident = 1_000_000 + next(self._next_ident)
        return self._march(ident, left, right, start_time, origin)

    # ---- построение ------------------------------------------------------

    def _march(
        self,
        ident: int,
        left: SupportLineV1,
        right: SupportLineV1,
        start_time: EventTimeV1,
        origin: EventPointV1,
    ) -> TraceV1:
        """Шаг за шагом вдоль луча, пока крушение не окажется ПОЗАДИ пройденного.

        Почему шагами, а не перебором стен: перебор стоил бы O(n) на мотоцикл,
        то есть вернул бы ровно тот O(n*r), от которого срез уходит. Шаг равен
        стороне ячейки, поделённой на доказанный верхний предел скорости,
        поэтому за шаг луч проходит не больше ячейки, и рамка шага накрывает
        весь пройденный участок.
        """

        velocity = bisector_velocity(left, right)
        zero = (SqrtSumV1.zero(), SqrtSumV1.zero())
        blank = _blank_trace(ident, left, right, start_time, origin)
        if velocity is None:
            return blank(TraceOutcome.MOTORCYCLE_HAS_NO_BISECTOR, zero)
        speed_squared = velocity[0] * velocity[0] + velocity[1] * velocity[1]
        step = Fraction(self.grid.cell, _upper_bound_of_root(speed_squared))
        return self._march_steps(
            ident, left, right, start_time, origin, velocity, step, blank
        )

    def _march_steps(
        self, ident, left, right, start_time, origin, velocity, step, blank
    ) -> TraceV1:
        budget = march_budget(self.grid)
        best: tuple[EventTimeV1, EventPointV1, int] | None = None
        offset = Fraction(0)
        for _ in range(budget):
            far = offset + step
            box = self._segment_box(origin, velocity, offset, far)
            cells = self.grid.box_cells(*box)
            if not cells and best is None and offset > 0:
                return blank(TraceOutcome.MOTORCYCLE_NEVER_MEETS_A_WALL, velocity)
            for wall_ident in self.wall_index.lookup(cells):
                self.counters["motorcycle_wall_tests"] += 1
                hit = self._wall_hit(left, right, start_time, self.walls[wall_ident])
                if hit is not None and (
                    best is None or compare_times(hit[0], best[0]) < 0
                ):
                    best = hit
            self.counters["motorcycle_march_steps"] += 1
            if best is not None and _reaches(origin, velocity, best[1], far):
                return _crashed_trace(
                    ident, left, right, start_time, origin, velocity, best
                )
            offset = far
        return blank(TraceOutcome.MOTORCYCLE_MARCH_BUDGET_EXHAUSTED, velocity)

    def _segment_box(
        self,
        origin: EventPointV1,
        velocity: tuple[SqrtSumV1, SqrtSumV1],
        low: Fraction,
        high: Fraction,
    ) -> tuple[int, int, int, int]:
        ends = tuple(
            EventPointV1(
                origin.x + velocity[0].scaled(offset),
                origin.y + velocity[1].scaled(offset),
            )
            for offset in (low, high)
        )
        return _point_box(ends)

    def _wall_hit(
        self,
        left: SupportLineV1,
        right: SupportLineV1,
        start_time: EventTimeV1,
        wall: WallV1,
    ) -> tuple[EventTimeV1, EventPointV1, int] | None:
        time, outcome = concurrency_time(left, right, wall.line)
        if outcome is not EventTimeOutcome.EXACT or time is None:
            return None
        if compare_times(time, start_time) <= 0:
            return None
        point = event_point(left, right, time)
        if not _projection_is_inside(point, wall):
            return None
        return time, point, wall.ident


def _blank_trace(ident, left, right, start_time, origin):
    def make(outcome: TraceOutcome, velocity) -> TraceV1:
        return TraceV1(
            ident=ident,
            outcome=outcome,
            left_line=left,
            right_line=right,
            start_time=start_time,
            origin=origin,
            velocity=velocity,
            crash_time=None,
            crash_point=None,
            crash_kind=CrashKind.NONE,
            crash_target=-1,
            reach=None,
        )

    return make


def _crashed_trace(
    ident, left, right, start_time, origin, velocity, best
) -> TraceV1:
    time, point, wall_ident = best
    return TraceV1(
        ident=ident,
        outcome=TraceOutcome.EXACT,
        left_line=left,
        right_line=right,
        start_time=start_time,
        origin=origin,
        velocity=velocity,
        crash_time=time,
        crash_point=point,
        crash_kind=CrashKind.WALL,
        crash_target=wall_ident,
        reach=_upper_bound_of_time(time),
    )


def _reaches(
    origin: EventPointV1,
    velocity: tuple[SqrtSumV1, SqrtSumV1],
    point: EventPointV1,
    offset: Fraction,
) -> bool:
    """Точка не дальше, чем луч уходит за `offset`. Квадраты, без корней."""

    dx = point.x - origin.x
    dy = point.y - origin.y
    travelled = dx * dx + dy * dy
    reach = (velocity[0] * velocity[0] + velocity[1] * velocity[1]).scaled(
        offset * offset
    )
    return (reach - travelled).sign() >= 0


# --------------------------------------------------------------------------
# Построение графа по полигону
# --------------------------------------------------------------------------


def speed_bound_of(polygon: PolygonV1) -> int:
    """Целое `s`, для которого доказано `s >= sqrt(q/|n|^2)` у КАЖДОГО ребра.

    Ровно та величина, на которую обобщается теорема 2.11: расстояние от точки
    split до несущей прямой равно `t * sqrt(q)/|n|`, а не `t`, поэтому запас
    рамки умножается на верхнюю границу евклидовой скорости.

    Считается в дробях и без корня: сравнивается `q * 1` с `s^2 * |n|^2`, то
    есть целые с целыми. При единичных скоростях ответ ровно `1` — это и есть
    та причина, по которой обобщение НЕ двигает ни один прежний счёт
    кандидатов. Стены (`q = 0`) границу не поднимают вовсе.
    """

    bound = 1
    for start, end, speed_squared in polygon.edges():
        dx, dy = end[0] - start[0], end[1] - start[1]
        normal_squared = dx * dx + dy * dy
        while bound * bound * normal_squared < speed_squared:
            bound *= 2
    return bound


def polygon_box(polygon: PolygonV1) -> tuple[int, int, int, int]:
    points = [point for loop in polygon.loops for point in loop.points]
    xs = [x for x, _ in points]
    ys = [y for _, y in points]
    return min(xs), min(ys), max(xs), max(ys)


def build_motorcycle_graph(polygon: PolygonV1) -> MotorcycleGraphV1:
    """Граф трасс всех reflex-вершин входа, с крушениями о стены и о трассы."""

    walls = walls_of(polygon)
    x_min, y_min, x_max, y_max = polygon_box(polygon)
    grid = CellGridV1.covering(
        (x_min, y_min, x_max, y_max), targets=max(4, len(walls))
    )
    wall_index = CellIndexV1()
    for wall in walls:
        wall_index.add(wall.ident, grid.segment_cells(wall.start, wall.end))
    graph = MotorcycleGraphV1(
        walls=walls,
        grid=grid,
        wall_index=wall_index,
        traces={},
        counters={
            "motorcycle_traces": 0,
            "motorcycle_wall_tests": 0,
            "motorcycle_march_steps": 0,
            "motorcycle_wall_crashes": 0,
            "motorcycle_trace_pairs": 0,
            "motorcycle_trace_crashes": 0,
            "motorcycle_unbounded_traces": 0,
            "resting_steiner_vertices": 0,
        },
    )
    _seed_traces(polygon, walls, graph)
    _resolve_trace_crashes(graph)
    return graph


def _seed_traces(
    polygon: PolygonV1, walls: tuple[WallV1, ...], graph: MotorcycleGraphV1
) -> None:
    """По одному мотоциклу на каждую reflex-вершину входа. Время старта — ноль."""

    vertex = 0
    for loop in polygon.loops:
        size = len(loop.points)
        reflex = loop.reflex_flags()
        speeds = loop.edge_speeds_squared
        for index in range(size):
            if reflex[index]:
                # Скорости соседних рёбер — из петли: биссектриса вогнутой
                # вершины между стеной и источником идёт ВДОЛЬ стены, а не по
                # середине угла, и `through` этого не знает.
                left = SupportLineV1.with_speed(
                    loop.points[(index - 1) % size],
                    loop.points[index],
                    speeds[(index - 1) % size],
                )
                right = SupportLineV1.with_speed(
                    loop.points[index],
                    loop.points[(index + 1) % size],
                    speeds[index],
                )
                origin = EventPointV1(
                    SqrtSumV1.rational(loop.points[index][0]),
                    SqrtSumV1.rational(loop.points[index][1]),
                )
                trace = graph._march(
                    vertex + index, left, right, ZERO_TIME, origin
                )
                graph.traces[vertex + index] = trace
                graph.counters["motorcycle_traces"] += 1
                if trace.outcome is TraceOutcome.EXACT:
                    graph.counters["motorcycle_wall_crashes"] += 1
                else:
                    graph.counters["motorcycle_unbounded_traces"] += 1
        vertex += size


def _resolve_trace_crashes(graph: MotorcycleGraphV1) -> None:
    """Крушения о трассы, в порядке возрастания времени.

    Порядок — не удобство. Когда из очереди выходит самое раннее крушение, оно
    окончательно: всё, что могло бы его отменить, случилось бы раньше и вышло бы
    из очереди прежде. Именно поэтому кандидат «я приехал в точку X, где сосед
    был раньше» проверяется В МОМЕНТ ИЗВЛЕЧЕНИЯ: к этому моменту про соседа уже
    известно, добрался он до X или разбился по пути.
    """

    live = {
        ident: trace
        for ident, trace in graph.traces.items()
        if trace.outcome is TraceOutcome.EXACT
    }
    if len(live) < 2:
        return
    heap: list[_CrashEntry] = []
    counter = itertools.count()
    for ident, trace in live.items():
        heapq.heappush(
            heap,
            _CrashEntry(
                trace.crash_time,
                CrashKind.WALL,
                ident,
                trace.crash_target,
                None,
                next(counter),
            ),
        )
    for first, second, at_first, at_second in _trace_pairs(graph, live):
        graph.counters["motorcycle_trace_pairs"] += 1
        order = compare_times(at_first, at_second)
        if order >= 0:
            heapq.heappush(
                heap,
                _CrashEntry(
                    at_first,
                    CrashKind.SIMULTANEOUS if order == 0 else CrashKind.TRACE,
                    first,
                    second,
                    at_second,
                    next(counter),
                ),
            )
        if order <= 0:
            heapq.heappush(
                heap,
                _CrashEntry(
                    at_second,
                    CrashKind.SIMULTANEOUS if order == 0 else CrashKind.TRACE,
                    second,
                    first,
                    at_first,
                    next(counter),
                ),
            )
    _drain_crashes(graph, heap)


def _drain_crashes(graph: MotorcycleGraphV1, heap: list[_CrashEntry]) -> None:
    """Очередь только уменьшается: ничего не досыпается, значит цикл конечен.

    Отменённый кандидат («сосед разбился, не доехав до точки встречи») просто
    выбрасывается: он невозможен навсегда, а у мотоцикла в очереди остаются
    другие кандидаты, в том числе крушение о стену.
    """

    settled: dict[int, EventTimeV1] = {}
    while heap:
        entry = heapq.heappop(heap)
        if entry.motorcycle in settled:
            continue
        if entry.kind is CrashKind.WALL:
            settled[entry.motorcycle] = entry.time
            continue
        peer = settled.get(entry.target)
        if peer is not None and compare_times(peer, entry.peer_time) < 0:
            continue
        if _apply_trace_crash(graph, entry):
            settled[entry.motorcycle] = entry.time


def _apply_trace_crash(graph: MotorcycleGraphV1, entry: _CrashEntry) -> bool:
    trace = graph.traces[entry.motorcycle]
    if compare_times(entry.time, trace.crash_time) >= 0:
        return False
    point = EventPointV1(
        trace.origin.x + trace.velocity[0] * _as_sqrt_sum(entry.time),
        trace.origin.y + trace.velocity[1] * _as_sqrt_sum(entry.time),
    )
    graph.traces[entry.motorcycle] = TraceV1(
        ident=trace.ident,
        outcome=TraceOutcome.EXACT,
        left_line=trace.left_line,
        right_line=trace.right_line,
        start_time=trace.start_time,
        origin=trace.origin,
        velocity=trace.velocity,
        crash_time=entry.time,
        crash_point=point,
        crash_kind=entry.kind,
        crash_target=entry.target,
        reach=_upper_bound_of_time(entry.time),
    )
    graph.counters["motorcycle_trace_crashes"] += 1
    graph.counters["resting_steiner_vertices"] += 1
    return True


def _as_sqrt_sum(time: EventTimeV1) -> SqrtSumV1:
    """Значение времени как сумма корней. Одно деление, и только здесь."""

    return SqrtSumV1.rational(time.dividend) / time.divisor


def _trace_pairs(graph: MotorcycleGraphV1, live: dict[int, TraceV1]):
    """Пары трасс, чьи рамки делят ячейку, и времена прихода в точку встречи.

    Пары берутся из сетки, а не перебором: перебор стоил бы O(r^2), то есть
    вернул бы тот же порядок, от которого срез уходит.
    """

    index = CellIndexV1()
    boxes: dict[int, tuple[int, int, int, int]] = {}
    for ident, trace in live.items():
        box = trace.box()
        if box is None:
            continue
        boxes[ident] = box
        index.add(ident, graph.grid.box_cells(*box))
    seen: set[tuple[int, int]] = set()
    for ident in sorted(boxes):
        for other in index.lookup(graph.grid.box_cells(*boxes[ident])):
            if other <= ident:
                continue
            if (ident, other) in seen:
                continue
            seen.add((ident, other))
            times = _meeting_times(live[ident], live[other])
            if times is not None:
                yield ident, other, times[0], times[1]


def _meeting_times(
    first: TraceV1, second: TraceV1
) -> tuple[EventTimeV1, EventTimeV1] | None:
    """Времена, в которые каждая трасса приходит в пересечение биссектрис.

    Биссектриса — прямая с коэффициентами-`SqrtSum`: через `origin` в
    направлении `velocity`. Подстановка одной параметризации в уравнение другой
    даёт время одним делением на каждую сторону.
    """

    at_first = _arrival(first, second)
    at_second = _arrival(second, first)
    if at_first is None or at_second is None:
        return None
    if (
        compare_times(at_first, first.start_time) <= 0
        or compare_times(at_second, second.start_time) <= 0
    ):
        return None
    return at_first, at_second


def _arrival(rider: TraceV1, host: TraceV1) -> EventTimeV1 | None:
    """Время, в которое `rider` пересекает несущую прямую трассы `host`."""

    normal_x, normal_y = host.velocity[1], -host.velocity[0]
    offset = normal_x * host.origin.x + normal_y * host.origin.y
    divisor = normal_x * rider.velocity[0] + normal_y * rider.velocity[1]
    if divisor.is_zero:
        return None
    dividend = offset - (normal_x * rider.origin.x + normal_y * rider.origin.y)
    if dividend.is_zero:
        return None
    return EventTimeV1.normalized(1, divisor / dividend)


# --------------------------------------------------------------------------
# Двусторонний индекс «вершина <-> несущая прямая»
# --------------------------------------------------------------------------


@dataclass(slots=True)
class TraceCandidateIndexV1:
    """Кто с кем МОЖЕТ дать split. Симметрично, поэтому обе стороны согласованы.

    Пара `(вершина, прямая)` попадает в кандидаты тогда и только тогда, когда у
    них есть общая ячейка. Условие симметрично по построению, поэтому «прямые
    вокруг вершины» и «вершины вокруг прямой» дают одно и то же множество пар —
    а без этого перепорождение кандидатов на вновь появившийся отрезок теряло бы
    события, которые нашёл первый проход.

    Полнота обоих запросов держится на одном включении: точка split лежит на
    трассе, её расстояние до несущей прямой равно `t * sqrt(q)/|n|`, то есть
    ВРЕМЕНИ, УМНОЖЕННОМУ НА ЕВКЛИДОВУ СКОРОСТЬ прямой, а время не больше
    `reach`. Значит прямая заходит в рамку трассы, расширенную на
    `reach * speed_bound`. Если эта рамка выходит за область сетки, запрос
    НЕПОЛОН, и вершина честно объявляется неиндексируемой — вызывающий уходит
    на полный перебор.

    Теорема 2.11 в прежнем виде опиралась на `dist(p, line) = t`, и это верно
    ровно при `q = a^2 + b^2`. Обобщение оказалось дешевле отказа от индекса:
    множитель `speed_bound` — одно целое на весь полигон, при единичных
    скоростях он равен единице, и тогда рамка та же до последней ячейки. Отказ
    же от индекса на неединичных рёбрах вернул бы полный перебор `reflex * рёбра`
    (на корпусе это 1 998 000 кандидатов против 6 488 при 2002 вершинах).
    """

    grid: CellGridV1
    #: Целая верхняя граница евклидовой скорости ЛЮБОГО ребра полигона.
    #: Единица при единичных скоростях, ноль скоростей её не поднимает.
    speed_bound: int = 1
    lines: CellIndexV1 = field(default_factory=CellIndexV1)
    vertices: CellIndexV1 = field(default_factory=CellIndexV1)
    line_cells: dict[int, tuple[tuple[int, int], ...]] = field(
        default_factory=dict
    )
    vertex_cells: dict[int, tuple[tuple[int, int], ...]] = field(
        default_factory=dict
    )

    @staticmethod
    def covering(
        polygon: PolygonV1, graph: MotorcycleGraphV1
    ) -> "TraceCandidateIndexV1":
        """Сетка над областью, расширенной на самый длинный `reach`.

        Расширение обязательно: ближайшая точка несущей прямой к трассе может
        лежать вне полигона, и без запаса такой кандидат выпал бы из индекса.
        """

        x_min, y_min, x_max, y_max = polygon_box(polygon)
        reaches = [
            trace.reach for trace in graph.traces.values() if trace.reach is not None
        ]
        bound = speed_bound_of(polygon)
        margin = math.ceil(max(reaches) * bound) if reaches else 0
        return TraceCandidateIndexV1(
            CellGridV1.covering(
                (x_min - margin, y_min - margin, x_max + margin, y_max + margin),
                targets=max(4, polygon.vertex_count),
            ),
            bound,
        )

    def register_line(self, key: int, line: SupportLineV1) -> None:
        if key in self.line_cells:
            return
        cells = self.grid.line_cells(line.a, line.b, line.c)
        self.line_cells[key] = cells
        self.lines.add(key, cells)

    def register_trace(self, vertex: int, trace: TraceV1) -> bool:
        """`False` — рамка не поместилась в область, индекс за вершину не отвечает."""

        box = trace.box()
        if box is None or trace.reach is None:
            return False
        margin = trace.reach * self.speed_bound
        x_low, x_high, y_low, y_high = box
        window = (
            x_low - margin,
            x_high + margin,
            y_low - margin,
            y_high + margin,
        )
        if not self.grid.contains_box(*window):
            return False
        cells = self.grid.box_cells(*window)
        if not cells:
            return False
        self.vertex_cells[vertex] = cells
        self.vertices.add(vertex, cells)
        return True

    def lines_near(self, vertex: int) -> tuple[int, ...]:
        return self.lines.lookup(self.vertex_cells[vertex])

    def vertices_near(self, key: int) -> tuple[int, ...]:
        return self.vertices.lookup(self.line_cells.get(key, ()))

    def knows_vertex(self, vertex: int) -> bool:
        return vertex in self.vertex_cells

    def knows_line(self, key: int) -> bool:
        return key in self.line_cells
