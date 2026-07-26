"""Типы событий и очередь с точной группировкой одновременных.

Типов ЧЕТЫРЕ, по Huber §2.3.2, а не три, как в §S4:

| событие | что происходит | сколько |
|---|---|---|
| `EDGE`   | встречаются две соседние вершины волны | O(n) |
| `SPLIT`  | reflex-вершина встречает движущуюся Steiner-вершину | O(r) |
| `START`  | вершина встречает ПОКОЯЩУЮСЯ Steiner-вершину | O(r) |
| `SWITCH` | выпуклая вершина перескакивает Steiner-вершину в соседнюю грань | O(n*r) |

`START` и `SWITCH` рождаются исключительно из Steiner-вершин на трассах
мотоциклов. Motorcycle graph в этом срезе не построен, поэтому Steiner-вершин
нет, и оба типа НЕДОСТИЖИМЫ. Они объявлены здесь, а не выброшены, потому что
шов должен быть видимым: счётчик `MotorcycleSeam` ниже возвращает ноль с
названной причиной, а не отсутствует.

Очередь. `heapq` не умеет пользовательский компаратор, поэтому сравнение живёт
в `__lt__` записи и опирается на `compare_times` — точное, без порога. Ключевое
отличие от всех трёх изученных реализаций: «тот же уровень» и «та же точка»
решаются ТОЧНО. У Kendzi это `fabs(d - level) < 1e-10`, у Трики — квантование к
решётке 1 мм; здесь уровень совпадает тогда и только тогда, когда обращается в
ноль рациональная разность, а точка — когда совпадают канонические координаты.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import heapq
import itertools

from .event_time import EventPointV1, EventTimeV1, compare_times


class EventKind(str, Enum):
    EDGE = "EDGE"
    SPLIT = "SPLIT"
    START = "START"
    SWITCH = "SWITCH"


class MotorcycleSeam(str, Enum):
    """Почему `START` и `SWITCH` не появляются. Шов назван, а не замолчан."""

    MOTORCYCLE_GRAPH_NOT_BUILT_IN_THIS_SLICE = (
        "MOTORCYCLE_GRAPH_NOT_BUILT_IN_THIS_SLICE"
    )


@dataclass(frozen=True, slots=True)
class CandidateEventV1:
    """Кандидат в события. Действителен ли он — решается при извлечении."""

    kind: EventKind
    time: EventTimeV1
    point: EventPointV1
    vertex: int
    peer: int
    edge: int


@dataclass(order=False, slots=True)
class _QueueEntry:
    event: CandidateEventV1
    sequence: int

    def __lt__(self, other: "_QueueEntry") -> bool:
        order = compare_times(self.event.time, other.event.time)
        if order:
            return order < 0
        # Одинаковое время — строгого порядка между ними НЕТ. Возврат False
        # оставляет их равными для кучи; порядок извлечения внутри уровня на
        # результат не влияет, потому что уровень снимается целиком.
        return False


@dataclass(slots=True)
class EventQueueV1:
    """Очередь по возрастанию точного времени. Уровень снимается целиком."""

    _heap: list[_QueueEntry] = field(default_factory=list)
    _counter: itertools.count = field(default_factory=itertools.count)
    pushed: int = 0
    popped: int = 0

    def push(self, event: CandidateEventV1) -> None:
        heapq.heappush(self._heap, _QueueEntry(event, next(self._counter)))
        self.pushed += 1

    def __len__(self) -> int:
        return len(self._heap)

    def pop_level(self) -> tuple[CandidateEventV1, ...]:
        """Все события ТОЧНО того же времени, что у вершины кучи.

        Это `LoadAndGroupLevelEvents` у Kendzi, но без `SplitEpsilon`: уровень
        совпадает по доказанному равенству, а не по попаданию в допуск.
        """

        if not self._heap:
            return ()
        head = heapq.heappop(self._heap)
        level = [head.event]
        while self._heap and compare_times(self._heap[0].event.time, head.event.time) == 0:
            level.append(heapq.heappop(self._heap).event)
        self.popped += len(level)
        return tuple(level)


def cluster_by_point(
    events: tuple[CandidateEventV1, ...]
) -> tuple[tuple[CandidateEventV1, ...], ...]:
    """Кластеры одного уровня по СОВПАДЕНИЮ ТОЧКИ, точно.

    Это `GroupLevelEvents` у Kendzi. У него кластер — «ближе SplitEpsilon», и
    поэтому на координатах порядка 1e6 его допуск равен 0.9 ulp и кластеризация
    перестаёт работать вовсе. Здесь `EventPointV1` каноничен, значит совпадение
    точек — это совпадение ключей словаря, и оно от величины координат не
    зависит.

    Порядок кластеров и порядок внутри кластера — от порядка ПОСТУПЛЕНИЯ, и
    поэтому наружу отдаётся отсортированным по каноническому ключу точки:
    иначе перестановка входа меняла бы порядок вывода.
    """

    buckets: dict[tuple, list[CandidateEventV1]] = {}
    for event in events:
        key = (event.point.x.terms, event.point.y.terms)
        buckets.setdefault(key, []).append(event)
    return tuple(
        tuple(buckets[key]) for key in sorted(buckets, key=_point_sort_key)
    )


def _point_sort_key(key: tuple) -> tuple:
    """Полный порядок на канонических точках. Только для воспроизводимости."""

    return tuple(
        (
            len(terms),
            tuple(
                (radicand, coefficient.numerator, coefficient.denominator)
                for radicand, coefficient in terms
            ),
        )
        for terms in key
    )
