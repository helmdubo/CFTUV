"""Цикл событий волнового фронта: LAV, уровни, кластеры, цепи.

Структура заимствована как СПЕЦИФИКАЦИЯ у Kendzi (`akashskypatel/StraightSkeleton`,
BSD-3-Clause, C++, код не переносился) и совпадает с Felkel–Obdrzalek:

    while queue:
        level = queue.pop_level()        # всё, что на этом уровне
        clusters = cluster_by_point(...) # кластеры по совпадению точки
        for cluster: chain -> MultiEdge / MultiSplit

Одно отличие от всех трёх изученных реализаций, и оно единственное: и «то же
время», и «та же точка» решаются ТОЧНО на целочисленной решётке (`sqrt_sum`,
`event_time`), а не допуском `1e-10` и не квантованием к 1 мм.

ОБЪЯВЛЕННЫЕ ГРАНИЦЫ (границ две, и обе проверяются тестом на собственном
результате этой функции):

1. Время выдаваемых узлов НЕ УБЫВАЕТ. Это не пожелание, а определение очереди.
2. Число обработанных уровней не превосходит `level_budget(polygon)`. Бюджет —
   не эвристика: каждый уровень либо убивает хотя бы одну вершину фронта, либо
   разрезает reflex-вершину, а разрезов не больше, чем reflex-вершин, умноженных
   на число рёбер (наивный поиск). Выход за бюджет — ИМЕНОВАННЫЙ исход, а не
   `return`, как `bestt == 1e100` у Трики.

ПОИСК SPLIT-КАНДИДАТОВ. Способов два, и оба живые — второй существует потому,
что первый обязан быть чем-то проверен.

| способ | кандидатов при 2002 вершинах | зачем он есть |
|---|---:|---|
| `EXHAUSTIVE` | 1 998 000 (ровно `reflex * рёбра`) | эталон дифференциального теста |
| `MOTORCYCLE` | см. `split_candidates_examined` | рабочий путь |

`MOTORCYCLE` опирается на теорему 2.11 Huber'а (`motorcycle.py`): reflex-вершина
волны не уходит за свою трассу, поэтому расстояние от точки split до несущей
прямой рассекаемого ребра не больше времени крушения, и кандидаты берутся из
ячеек вокруг трассы. Наивная проверка «точка попала в текущий отрезок фронта»
остаётся на месте: она необходима, но НЕ достаточна — это измерено.

`EXHAUSTIVE` не «старый код, который забыли убрать»: оптимизация законна ровно
настолько, насколько даёт тот же ответ, а сравнивать не с чем, если эталон
удалить.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from fractions import Fraction
from functools import cmp_to_key

from .event_time import (
    ZERO_TIME,
    EventPointV1,
    EventTimeV1,
    EventTimeOutcome,
    SupportLineV1,
    compare_times,
    concurrency_time,
    event_point,
)
from .events import CandidateEventV1, EventKind, EventQueueV1, cluster_by_point
from .motorcycle import (
    MotorcycleGraphV1,
    TraceCandidateIndexV1,
    TraceV1,
    build_motorcycle_graph,
)
from .polygon import PolygonV1
from .sqrt_sum import SqrtSumV1


class SplitSearch(str, Enum):
    """Чем ищутся split-кандидаты. Оба пути обязаны давать один ответ."""

    MOTORCYCLE = "MOTORCYCLE"
    EXHAUSTIVE = "EXHAUSTIVE"


class SkeletonOutcome(str, Enum):
    """Чем кончилось построение. Тихого выхода нет ни одного."""

    EXACT = "EXACT"
    LEVEL_BUDGET_EXHAUSTED = "LEVEL_BUDGET_EXHAUSTED"
    MULTIWAY_SPLIT_UNPROVEN = "MULTIWAY_SPLIT_UNPROVEN"
    WAVEFRONT_LEFT_UNRESOLVED = "WAVEFRONT_LEFT_UNRESOLVED"


@dataclass(slots=True)
class _Edge:
    ident: int
    line: SupportLineV1

    @property
    def key(self) -> tuple[int, int, int, int]:
        """Геометрический ключ ребра: не зависит от порядка входа."""

        return (self.line.a, self.line.b, self.line.c, self.line.q)


@dataclass(slots=True)
class _Vertex:
    ident: int
    prev_edge: int
    next_edge: int
    prev: int
    next: int
    birth: EventTimeV1
    point: EventPointV1
    reflex: bool
    alive: bool = True


@dataclass(frozen=True, slots=True)
class SkeletonNodeV1:
    """Узел скелета: момент, место и ВСЕ участники сразу.

    `participants` — геометрические ключи исходных рёбер, отсортированные.
    Три фронта, сошедшиеся в одной точке, дают ОДИН такой узел с тремя
    ключами, а не три попарных.
    """

    kind: EventKind
    time: EventTimeV1
    point: EventPointV1
    participants: tuple[tuple[int, int, int, int], ...]
    converging_vertices: int


@dataclass(frozen=True, slots=True)
class SkeletonV1:
    outcome: SkeletonOutcome
    nodes: tuple[SkeletonNodeV1, ...]
    levels: int
    counters: tuple[tuple[str, int], ...]

    def counter(self, name: str) -> int:
        return dict(self.counters).get(name, 0)


def level_budget(polygon: PolygonV1) -> int:
    """Объявленная граница числа уровней. Превышение — именованный исход."""

    n = polygon.vertex_count
    r = polygon.reflex_count
    return 4 * n + 4 * n * r + 16


def build_skeleton(
    polygon: PolygonV1,
    *,
    split_search: SplitSearch = SplitSearch.MOTORCYCLE,
) -> SkeletonV1:
    """Скелет области как последовательность событий по возрастанию времени."""

    return _Builder(polygon, split_search).run()


class _Builder:
    """Носитель состояния цикла. Разбит на короткие этапы конвейера."""

    def __init__(
        self,
        polygon: PolygonV1,
        split_search: SplitSearch = SplitSearch.MOTORCYCLE,
    ) -> None:
        self.polygon = polygon
        self.split_search = split_search
        self.edges: list[_Edge] = []
        self.vertices: list[_Vertex] = []
        self.queue = EventQueueV1()
        self.edge_start: dict[int, int] = {}
        self.edge_end: dict[int, int] = {}
        self.nodes: list[SkeletonNodeV1] = []
        self.refusal: SkeletonOutcome | None = None
        # Трассы мотоциклов и двусторонний индекс. `None` — путь полного
        # перебора: эталон, с которым сверяется рабочий путь.
        self.graph: MotorcycleGraphV1 | None = None
        self.index: TraceCandidateIndexV1 | None = None
        self.traces: dict[int, TraceV1] = {}
        self.line_id: dict[tuple[int, int, int, int], int] = {}
        self.edges_by_line: dict[int, list[int]] = {}
        # Reflex-вершины, за которые индекс НЕ отвечает: их кандидаты всегда
        # перебираются полностью. Забыть их означало бы потерять события молча.
        self.unindexed_reflex: set[int] = set()
        # Время уже снятого уровня. Кандидат раньше него — событие в прошлом
        # фронта, а не будущее: очередь его уже прошла. Без этой отсечки
        # ВЫДАВАЕМОЕ время переставало быть неубывающим на входах, где фронт
        # вывернулся, — то есть объявленная граница нарушалась собственным
        # результатом. Найдено тестом, а не рассуждением.
        self.now: EventTimeV1 = ZERO_TIME
        self.counters = {
            "edge_events": 0,
            "split_events": 0,
            "start_events": 0,
            "switch_events": 0,
            "multi_participant_nodes": 0,
            "discarded_stale_candidates": 0,
            "split_candidates_examined": 0,
            "split_candidates_beyond_trace": 0,
            "split_search_exhaustive_vertices": 0,
            "split_search_exhaustive_segments": 0,
            "coincident_split_targets": 0,
            "peaks": 0,
            "ridges": 0,
        }
        self._seed()

    # ---- инициализация ---------------------------------------------------

    def _seed(self) -> None:
        """`InitSlav` на каждый контур, включая каждую дыру."""

        self._seed_loops()
        self._seed_traces()
        for vertex in self.vertices:
            self._enqueue_for(vertex)

    def _seed_loops(self) -> None:
        for loop in self.polygon.loops:
            points = loop.points
            reflex = loop.reflex_flags()
            size = len(points)
            first_edge = len(self.edges)
            first_vertex = len(self.vertices)
            for index in range(size):
                self.edges.append(
                    _Edge(
                        first_edge + index,
                        SupportLineV1.through(
                            points[index], points[(index + 1) % size]
                        ),
                    )
                )
            for index in range(size):
                self.vertices.append(
                    _Vertex(
                        ident=first_vertex + index,
                        prev_edge=first_edge + (index - 1) % size,
                        next_edge=first_edge + index,
                        prev=first_vertex + (index - 1) % size,
                        next=first_vertex + (index + 1) % size,
                        birth=ZERO_TIME,
                        point=EventPointV1(
                            SqrtSumV1.rational(points[index][0]),
                            SqrtSumV1.rational(points[index][1]),
                        ),
                        reflex=reflex[index],
                    )
                )
                self._register(self.vertices[-1])
        for edge in self.edges:
            self._register_line(edge)

    def _register_line(self, edge: _Edge) -> int:
        """Ребро под свою НЕСУЩУЮ ПРЯМУЮ. Близнецы разреза делят одну прямую.

        Индекс опрашивается по прямым, а не по рёбрам: рассечённое ребро
        размножается в счёте на несколько отрезков фронта с одним и тем же
        геометрическим ключом, и разметка от этого не меняется — меняется только
        список владельцев.
        """

        ident = self.line_id.get(edge.key)
        if ident is None:
            ident = len(self.line_id)
            self.line_id[edge.key] = ident
            self.edges_by_line[ident] = []
        self.edges_by_line[ident].append(edge.ident)
        return ident

    def _seed_traces(self) -> None:
        """Motorcycle graph по входу и двусторонний индекс по его трассам."""

        if self.split_search is not SplitSearch.MOTORCYCLE:
            return
        self.graph = build_motorcycle_graph(self.polygon)
        self.index = TraceCandidateIndexV1.covering(self.polygon, self.graph)
        for line_key, ident in self.line_id.items():
            self.index.register_line(
                ident, SupportLineV1(*line_key)
            )
        for vertex in self.vertices:
            if not vertex.reflex:
                continue
            self._adopt_trace(vertex, self.graph.traces.get(vertex.ident))

    def _adopt_trace(self, vertex: _Vertex, trace: TraceV1 | None) -> None:
        """Принять трассу вершины либо честно объявить её неиндексируемой."""

        if trace is None or trace.crash_time is None:
            self.unindexed_reflex.add(vertex.ident)
            return
        self.traces[vertex.ident] = trace
        if not self.index.register_trace(vertex.ident, trace):
            self.unindexed_reflex.add(vertex.ident)

    # ---- порождение кандидатов ------------------------------------------

    def _enqueue_for(self, vertex: _Vertex) -> None:
        self._enqueue_edge_event(vertex)
        if vertex.reflex:
            self._enqueue_split_events(vertex)

    def _enqueue_edge_event(self, vertex: _Vertex) -> None:
        peer = self.vertices[vertex.next]
        if peer.ident == vertex.ident:
            return
        time, outcome = concurrency_time(
            self.edges[vertex.prev_edge].line,
            self.edges[vertex.next_edge].line,
            self.edges[peer.next_edge].line,
        )
        if outcome is not EventTimeOutcome.EXACT or time is None:
            return
        if not self._is_future(time, vertex, peer):
            return
        point = self._vertex_position(vertex, time)
        if point is None:
            return
        self.queue.push(
            CandidateEventV1(
                EventKind.EDGE, time, point, vertex.ident, peer.ident, -1
            )
        )

    def _enqueue_split_events(self, vertex: _Vertex) -> None:
        """Кандидаты с трассы, если индекс за вершину отвечает; иначе перебор."""

        if self.index is not None and self.index.knows_vertex(vertex.ident):
            self._enqueue_splits_from_trace(vertex)
            return
        self.counters["split_search_exhaustive_vertices"] += 1
        for edge in self.edges:
            self._try_split(vertex, edge)

    def _enqueue_splits_from_trace(self, vertex: _Vertex) -> None:
        """Только рёбра тех прямых, что заходят в рамку трассы этой вершины."""

        for line_ident in self.index.lines_near(vertex.ident):
            for edge_ident in self.edges_by_line[line_ident]:
                self._try_split(vertex, self.edges[edge_ident])

    def _try_split(self, vertex: _Vertex, edge: _Edge) -> None:
        if edge.ident in (vertex.prev_edge, vertex.next_edge):
            return
        self.counters["split_candidates_examined"] += 1
        candidate = self._split_candidate(vertex, edge)
        if candidate is not None:
            self.queue.push(candidate)

    def _split_candidate(
        self, vertex: _Vertex, edge: _Edge
    ) -> CandidateEventV1 | None:
        time, outcome = concurrency_time(
            self.edges[vertex.prev_edge].line,
            self.edges[vertex.next_edge].line,
            edge.line,
        )
        if outcome is not EventTimeOutcome.EXACT or time is None:
            return None
        if time.sign <= 0 or compare_times(time, vertex.birth) <= 0:
            return None
        if compare_times(time, self.now) < 0:
            return None
        # ТЕОРЕМА 2.11: вершина не уходит за свою трассу, значит событие позже
        # крушения невозможно. Это и есть достаточное условие, которого наивной
        # проверке попадания в отрезок не хватало.
        trace = self.traces.get(vertex.ident)
        if trace is not None and not trace.bounds_time(time):
            self.counters["split_candidates_beyond_trace"] += 1
            return None
        point = self._vertex_position(vertex, time)
        if point is None or not self._edge_span_contains(edge, point, time):
            return None
        return CandidateEventV1(
            EventKind.SPLIT, time, point, vertex.ident, -1, edge.ident
        )

    def _is_future(
        self, time: EventTimeV1, vertex: _Vertex, peer: _Vertex
    ) -> bool:
        if time.sign < 0:
            return False
        return (
            compare_times(time, vertex.birth) >= 0
            and compare_times(time, peer.birth) >= 0
            and compare_times(time, self.now) >= 0
        )

    def _vertex_position(
        self, vertex: _Vertex, time: EventTimeV1
    ) -> EventPointV1 | None:
        first = self.edges[vertex.prev_edge].line
        second = self.edges[vertex.next_edge].line
        if first.a * second.b - second.a * first.b == 0:
            return None
        return event_point(first, second, time)

    # ---- геометрические проверки ----------------------------------------

    def _edge_span_contains(
        self, edge: _Edge, point: EventPointV1, time: EventTimeV1
    ) -> bool:
        """Точка лежит внутри ТЕКУЩЕГО отрезка фронта этого ребра.

        Проекция на направление ребра — целочисленная линейная форма, поэтому
        сравнение остаётся точным знаком `SqrtSumV1` и порога не требует.
        """

        start = self._edge_start_vertex(edge.ident)
        end = self._edge_end_vertex(edge.ident)
        if start is None or end is None:
            return False
        here = _project(edge.line, point)
        low = self._span_end(start, edge, time, at_start=True)
        high = self._span_end(end, edge, time, at_start=False)
        if low is not None and (here - low).sign() < 0:
            return False
        if high is not None and (high - here).sign() < 0:
            return False
        return True

    def _span_end(
        self, vertex: _Vertex, edge: _Edge, time: EventTimeV1, *, at_start: bool
    ) -> SqrtSumV1 | None:
        other = self.edges[
            vertex.prev_edge if at_start else vertex.next_edge
        ].line
        if edge.line.a * other.b - other.a * edge.line.b == 0:
            # Параллельный сосед: с этой стороны отрезок фронта не ограничен.
            return None
        position = event_point(edge.line, other, time)
        return _project(edge.line, position)

    def _register(self, vertex: _Vertex) -> None:
        self.edge_start[vertex.next_edge] = vertex.ident
        self.edge_end[vertex.prev_edge] = vertex.ident

    def _edge_start_vertex(self, edge_id: int) -> _Vertex | None:
        """Живая вершина, с которой начинается фронт этого ребра, либо None.

        Запись в словаре может устареть: ребро исчезает вместе со схлопнутой
        цепью. Поэтому найденное сверяется с текущим состоянием, а не берётся
        на веру.
        """

        ident = self.edge_start.get(edge_id)
        if ident is None:
            return None
        vertex = self.vertices[ident]
        return vertex if vertex.alive and vertex.next_edge == edge_id else None

    def _edge_end_vertex(self, edge_id: int) -> _Vertex | None:
        ident = self.edge_end.get(edge_id)
        if ident is None:
            return None
        vertex = self.vertices[ident]
        return vertex if vertex.alive and vertex.prev_edge == edge_id else None

    # ---- цикл ------------------------------------------------------------

    def run(self) -> SkeletonV1:
        budget = level_budget(self.polygon)
        levels = 0
        while len(self.queue):
            if levels >= budget:
                return self._finish(SkeletonOutcome.LEVEL_BUDGET_EXHAUSTED, levels)
            level = self.queue.pop_level()
            self.now = level[0].time
            levels += 1
            self._apply_level(level)
            if self.refusal is not None:
                return self._finish(self.refusal, levels)
            self._close_short_lavs()
        outcome = (
            SkeletonOutcome.EXACT
            if not any(vertex.alive for vertex in self.vertices)
            else SkeletonOutcome.WAVEFRONT_LEFT_UNRESOLVED
        )
        return self._finish(outcome, levels)

    def _finish(self, outcome: SkeletonOutcome, levels: int) -> SkeletonV1:
        counters = dict(self.counters)
        if self.graph is not None:
            # Цена графа входит в отчёт. Иначе выигрыш в кандидатах мерился бы
            # против базы, у которой этой статьи расхода просто нет.
            counters.update(self.graph.counters)
        return SkeletonV1(
            outcome=outcome,
            nodes=tuple(self.nodes),
            levels=levels,
            counters=tuple(sorted(counters.items())),
        )

    def _apply_level(self, level: tuple[CandidateEventV1, ...]) -> None:
        """Весь уровень разом: сначала ВСЕ разрезы, потом ВСЕ схлопывания.

        Живость всех кандидатов уровня определяется ДО первого применения.
        Иначе порядок применения решал бы, какие события уровня выживут, — то
        есть ровно тот дефект, ради которого очередь и строится: последовательный
        попарный крой не композируется.

        Ни одно снятое событие не теряется молча: не прошедшее проверку идёт в
        счётчик устаревших, а неразрешимая одновременность — в именованный
        исход, а не в выбор «первого попавшегося».
        """

        splits = [event for event in level if event.kind is EventKind.SPLIT]
        collapses = [event for event in level if event.kind is EventKind.EDGE]
        live_splits = [event for event in splits if self._split_is_live(event)]
        self.counters["discarded_stale_candidates"] += len(splits) - len(
            live_splits
        )
        separated = self._dedupe_by_vertex(live_splits)
        if separated is None:
            return
        for edge_id, group in self._group_splits(separated):
            self._apply_multi_split(edge_id, group)

        live_collapses = [
            event for event in collapses if self._edge_event_is_live(event)
        ]
        self.counters["discarded_stale_candidates"] += len(collapses) - len(
            live_collapses
        )
        for cluster in cluster_by_point(tuple(live_collapses)):
            self._apply_multi_edge(list(cluster))

    def _dedupe_by_vertex(
        self, splits: list[CandidateEventV1]
    ) -> list[CandidateEventV1] | None:
        """Одна reflex-вершина — один разрез за уровень.

        Два случая, и они РАЗНЫЕ, поэтому и обходятся по-разному.

        Одна и та же точка у нескольких кандидатов означает, что вершина
        пришла в УГОЛ фронта — туда, где два отрезка уже сходятся. Это одно
        событие, а не два; лишние цели считаются `coincident_split_targets`,
        и цель выбирается по геометрическому ключу, то есть от порядка входа
        не зависит.

        Разные точки означают развилку: вершина одновременно достигает двух
        разных мест. Правила для этого в срезе НЕТ, и брать первое попавшееся
        значило бы вернуть порядковую зависимость чёрным ходом. Исход
        именованный.
        """

        grouped: dict[int, list[CandidateEventV1]] = {}
        for event in splits:
            grouped.setdefault(event.vertex, []).append(event)
        chosen: list[CandidateEventV1] = []
        for vertex_id in sorted(grouped):
            candidates = grouped[vertex_id]
            points = {
                (event.point.x.terms, event.point.y.terms) for event in candidates
            }
            if len(points) > 1:
                self.refusal = SkeletonOutcome.MULTIWAY_SPLIT_UNPROVEN
                return None
            candidates.sort(key=lambda event: (self.edges[event.edge].key, event.edge))
            self.counters["coincident_split_targets"] += len(candidates) - 1
            chosen.append(candidates[0])
        return chosen

    def _group_splits(
        self, splits: list[CandidateEventV1]
    ) -> list[tuple[int, list[CandidateEventV1]]]:
        """Разрезы одного уровня, сгруппированные по рассекаемому отрезку.

        Внутри группы точки упорядочены ВДОЛЬ ребра точным сравнением проекций.
        Порядок групп между собой не влияет ни на что: отрезки независимы.
        """

        grouped: dict[int, list[CandidateEventV1]] = {}
        for event in splits:
            grouped.setdefault(event.edge, []).append(event)
        ordered: list[tuple[int, list[CandidateEventV1]]] = []
        for edge_id in sorted(grouped, key=lambda ident: self.edges[ident].key):
            line = self.edges[edge_id].line
            group = sorted(
                grouped[edge_id],
                key=cmp_to_key(
                    lambda left, right: (
                        _project(line, left.point) - _project(line, right.point)
                    ).sign()
                ),
            )
            ordered.append((edge_id, group))
        return ordered

    def _edge_event_is_live(self, event: CandidateEventV1) -> bool:
        vertex = self.vertices[event.vertex]
        peer = self.vertices[event.peer]
        return (
            vertex.alive
            and peer.alive
            and vertex.next == peer.ident
            and peer.prev == vertex.ident
        )

    def _split_is_live(self, event: CandidateEventV1) -> bool:
        vertex = self.vertices[event.vertex]
        if not vertex.alive:
            return False
        edge = self.edges[event.edge]
        if edge.ident in (vertex.prev_edge, vertex.next_edge):
            return False
        return self._edge_span_contains(edge, event.point, event.time)

    # ---- применение событий ---------------------------------------------

    def _apply_multi_edge(self, events: list[CandidateEventV1]) -> None:
        """Кластер сходящихся в ОДНОЙ точке рёбер — одна цепь, один узел."""

        chains = _chains(
            {event.vertex for event in events} | {event.peer for event in events},
            self.vertices,
        )
        sample = events[0]
        participants = sorted(
            {
                self.edges[self.vertices[ident].next_edge].key
                for chain in chains
                for ident in chain
            }
            | {
                self.edges[self.vertices[ident].prev_edge].key
                for chain in chains
                for ident in chain
            }
        )
        converging = sum(len(chain) for chain in chains)
        self._emit(EventKind.EDGE, sample, tuple(participants), converging)
        self.counters["edge_events"] += 1
        if converging > 2:
            self.counters["multi_participant_nodes"] += 1
        for chain in chains:
            self._collapse_chain(chain, sample)

    def _collapse_chain(
        self, chain: tuple[int, ...], event: CandidateEventV1
    ) -> None:
        head, tail = self.vertices[chain[0]], self.vertices[chain[-1]]
        closed = self.vertices[tail.next].ident == head.ident
        for ident in chain:
            self.vertices[ident].alive = False
        if closed:
            self.counters["peaks"] += 1
            return
        before, after = self.vertices[head.prev], self.vertices[tail.next]
        merged = self._new_vertex(
            prev_edge=head.prev_edge,
            next_edge=tail.next_edge,
            prev=before.ident,
            next=after.ident,
            birth=event.time,
            point=event.point,
        )
        before.next = merged.ident
        after.prev = merged.ident
        self._enqueue_for(merged)
        # У предшественника сменился сосед, значит сменилась и третья прямая
        # его собственного edge-события. Не пересчитать её — потерять событие.
        self._enqueue_edge_event(before)

    def _apply_multi_split(
        self, edge_id: int, group: list[CandidateEventV1]
    ) -> None:
        """`MultiSplitEvent`: отрезок фронта режется СРАЗУ во всех точках.

        Это тот случай, ради которого срез: у квадрата с двумя дырами четыре
        верхних угла дыр приходят к верхнему ребру в один и тот же момент.
        Последовательное применение дало бы разный ответ на разном порядке
        входа — измерено, дайджесты расходились. Разрез сразу во всех точках
        от порядка не зависит: точки упорядочены геометрией, а не очередью.

        Раскладка отрезков вдоль ребра: `start .. p_1 .. p_2 .. p_m .. end`.
        Отрезок, примыкающий к `end`, сохраняет исходный идентификатор ребра,
        остальные получают близнецов с ТЕМ ЖЕ геометрическим ключом.
        """

        edge = self.edges[edge_id]
        span_start = self._edge_start_vertex(edge_id)
        span_end = self._edge_end_vertex(edge_id)
        if span_start is None or span_end is None:
            self.counters["discarded_stale_candidates"] += len(group)
            return
        # segments[i] — отрезок между p_{i-1} и p_i; последний примыкает к
        # `span_end` и сохраняет исходный идентификатор ребра.
        segments = [self._twin(edge) for _ in range(len(group))] + [edge]
        lefts: list[_Vertex] = []
        rights: list[_Vertex] = []
        for index, event in enumerate(group):
            vertex = self.vertices[event.vertex]
            self._emit_split_node(vertex, edge, event)
            vertex.alive = False
            lefts.append(
                self._new_vertex(
                    prev_edge=vertex.prev_edge,
                    next_edge=segments[index + 1].ident,
                    prev=vertex.prev,
                    next=vertex.ident,
                    birth=event.time,
                    point=event.point,
                )
            )
            rights.append(
                self._new_vertex(
                    prev_edge=segments[index].ident,
                    next_edge=vertex.next_edge,
                    prev=vertex.ident,
                    next=vertex.next,
                    birth=event.time,
                    point=event.point,
                )
            )
        self._relink_multi_split(group, segments, lefts, rights, span_start, span_end)
        for segment in segments:
            self._enqueue_splits_against(segment)

    def _relink_multi_split(
        self,
        group: list[CandidateEventV1],
        segments: list[_Edge],
        lefts: list[_Vertex],
        rights: list[_Vertex],
        span_start: _Vertex,
        span_end: _Vertex,
    ) -> None:
        """Сшивание LAV после многоточечного разреза. Только ссылки."""

        span_start.next_edge = segments[0].ident
        span_start.next = rights[0].ident
        rights[0].prev = span_start.ident
        self._register(span_start)
        for index in range(len(group) - 1):
            lefts[index].next = rights[index + 1].ident
            rights[index + 1].prev = lefts[index].ident
        lefts[-1].next = span_end.ident
        span_end.prev = lefts[-1].ident
        span_end.prev_edge = segments[-1].ident
        self._register(span_end)
        for index, event in enumerate(group):
            vertex = self.vertices[event.vertex]
            predecessor = self.vertices[vertex.prev]
            successor = self.vertices[vertex.next]
            predecessor.next = lefts[index].ident
            lefts[index].prev = predecessor.ident
            successor.prev = rights[index].ident
            rights[index].next = successor.ident
            self._enqueue_edge_event(predecessor)
        for vertex in lefts + rights:
            self._enqueue_for(vertex)
        self._enqueue_edge_event(span_start)

    def _enqueue_splits_against(self, edge: _Edge) -> None:
        """Кандидаты живых reflex-вершин против ВНОВЬ ПОЯВИВШЕГОСЯ отрезка.

        Без этого срез теряет события молча, и потеря видна не сразу: у
        квадрата с двумя дырами четыре верхних угла дыр целят в одно и то же
        верхнее ребро в один и тот же момент, первый разрез рассекает это ребро
        надвое, и оставшиеся кандидаты перестают попадать в СВОЙ отрезок. Рёбра
        во время счёта размножаются, поэтому перепорождение обязательно.

        Запрос обратный к `lines_near` и по построению согласован с ним: пара
        попадает в кандидаты тогда и только тогда, когда у вершины и прямой есть
        общая ячейка. Несогласованность двух направлений теряла бы ровно те
        события, которые нашёл первый проход.
        """

        for ident in self._split_partners(edge):
            vertex = self.vertices[ident]
            if not (vertex.alive and vertex.reflex):
                continue
            self.counters["split_candidates_examined"] += 1
            candidate = self._split_candidate(vertex, edge)
            if candidate is not None:
                self.queue.push(candidate)

    def _split_partners(self, edge: _Edge) -> tuple[int, ...]:
        line_ident = self.line_id.get(edge.key)
        if (
            self.index is None
            or line_ident is None
            or not self.index.knows_line(line_ident)
        ):
            self.counters["split_search_exhaustive_segments"] += 1
            return tuple(range(len(self.vertices)))
        near = set(self.index.vertices_near(line_ident))
        return tuple(sorted(near | self.unindexed_reflex))

    def _twin(self, edge: _Edge) -> _Edge:
        """Копия ребра под новым идентификатором и с ТЕМ ЖЕ ключом.

        Рассечённое ребро становится несколькими отрезками фронта, у каждого
        свой владелец. Без отдельных идентификаторов вторая reflex-вершина,
        целящая в то же ребро, не нашла бы своего отрезка. На участников
        события и на дайджест раздвоение не влияет: ключ геометрический.
        """

        twin = _Edge(len(self.edges), edge.line)
        self.edges.append(twin)
        self._register_line(twin)
        return twin

    def _emit_split_node(
        self, vertex: _Vertex, edge: _Edge, event: CandidateEventV1
    ) -> None:
        participants = sorted(
            {
                self.edges[vertex.prev_edge].key,
                self.edges[vertex.next_edge].key,
                edge.key,
            }
        )
        self._emit(EventKind.SPLIT, event, tuple(participants), 1)
        self.counters["split_events"] += 1

    def _new_vertex(self, **fields) -> _Vertex:
        ident = len(self.vertices)
        vertex = _Vertex(ident=ident, reflex=False, **fields)
        vertex.reflex = _is_reflex(
            self.edges[vertex.prev_edge].line, self.edges[vertex.next_edge].line
        )
        self.vertices.append(vertex)
        self._register(vertex)
        if vertex.reflex and self.graph is not None:
            # У рождённой вершины motorcycle graph трассы не имеет: граф
            # определён на reflex-вершинах ВХОДА. Её граница — стенная, то есть
            # слабее теоремы 2.11, но верна без всякой теоремы.
            self._adopt_trace(
                vertex,
                self.graph.trace_for(
                    self.edges[vertex.prev_edge].line,
                    self.edges[vertex.next_edge].line,
                    vertex.birth,
                    vertex.point,
                ),
            )
        return vertex

    def _emit(
        self,
        kind: EventKind,
        event: CandidateEventV1,
        participants: tuple[tuple[int, int, int, int], ...],
        converging: int,
    ) -> None:
        self.nodes.append(
            SkeletonNodeV1(
                kind, event.time, event.point, participants, converging
            )
        )

    def _close_short_lavs(self) -> None:
        """LAV из двух вершин — конёк крыши: событий больше не будет."""

        for vertex in self.vertices:
            if not vertex.alive:
                continue
            peer = self.vertices[vertex.next]
            if peer.ident == vertex.ident or self.vertices[peer.next].ident != vertex.ident:
                continue
            vertex.alive = False
            peer.alive = False
            self.counters["ridges"] += 1


def _project(line: SupportLineV1, point: EventPointV1) -> SqrtSumV1:
    """Проекция на направление ребра `(b, -a)`. Целые коэффициенты."""

    return point.x.scaled(line.b) - point.y.scaled(line.a)


def _is_reflex(first: SupportLineV1, second: SupportLineV1) -> bool:
    """Вогнутость стыка двух рёбер — знак векторного произведения направлений."""

    cross = first.b * (-second.a) - (-first.a) * second.b
    return cross < 0


def _chains(
    idents: set[int], vertices: list[_Vertex]
) -> tuple[tuple[int, ...], ...]:
    """Разложить множество вершин на СВЯЗНЫЕ участки LAV.

    Это `CreateChains` у Kendzi. Кластер одного уровня может задевать
    несколько несмежных участков фронта; каждый схлопывается отдельно.
    """

    live = {ident for ident in idents if vertices[ident].alive}
    chains: list[tuple[int, ...]] = []
    seen: set[int] = set()
    for ident in sorted(live):
        if ident in seen:
            continue
        start = ident
        guard = 0
        while (
            vertices[start].prev in live
            and vertices[start].prev != ident
            and guard <= len(live)
        ):
            start = vertices[start].prev
            guard += 1
        chain = [start]
        seen.add(start)
        cursor = vertices[start].next
        while cursor in live and cursor not in seen:
            chain.append(cursor)
            seen.add(cursor)
            cursor = vertices[cursor].next
        chains.append(tuple(chain))
    return tuple(chains)
