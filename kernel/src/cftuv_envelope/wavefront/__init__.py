"""Волновой фронт: очередь событий по ТОЧНОМУ времени, отдельным модулем.

Модуль живёт рядом с `robust/` и, как он, ни к чему не подключён: проводка —
следующий срез. Здесь строится и доказывается только одно — что события можно
упорядочить и сгруппировать точно, без порогов и без вычисления времени.

Главный результат среза, вынесенный сюда, чтобы его нельзя было не заметить:

    время события в общем случае ИРРАЦИОНАЛЬНО (алгебраическое степени до 8),
    но очередь его НЕ ВЫЧИСЛЯЕТ: `t = -D0/S` хранится парой (целое, SqrtSum),
    и обе нужные операции — сравнение и проверка одновременности — сводятся
    к целочисленной арифметике.

Равенство времён при этом ЧИСТО РАЦИОНАЛЬНО: разность обращается в ноль тогда
и только тогда, когда пуст канонический набор коэффициентов. Именно поэтому
«одновременно» и «та же точка» решаются точно, а не допуском `1e-10` (Kendzi)
и не квантованием к 1 мм (Трики).

Второй результат, срез Q2: **split-кандидаты берутся с трасс мотоциклов, а не
перебором.** Теорема 2.11 Huber'а даёт границу («reflex-вершина волны не уходит
за свою трассу»), а из того, что нормаль в `SupportLineV1` имеет длину ровно
`√q`, следует `dist(p, line) = t` точно — то есть граница по времени становится
границей по РАССТОЯНИЮ, и поиск сводится к ячейкам вокруг трассы. При 2002
вершинах: 6 488 кандидатов против 1 998 000, O(n^1.04) против O(n^1.47).

Что здесь есть:
- `sqrt_sum` — сумма корней с рациональными коэффициентами, точный знак;
- `event_time` — время и место события как определитель трёх движущихся прямых;
- `polygon` — вход с дырами, ориентация нормируется на входе;
- `events` — четыре типа Huber, очередь уровнями, кластеры по точке;
- `cell_grid` — консервативная сетка ячеек, фильтр без права решать;
- `motorcycle` — трассы, крушения, теорема 2.11 и индекс кандидатов;
- `skeleton` — цикл событий; поиск кандидатов оба, эталонный и рабочий.

Чего здесь нет: порогов, ε-сравнений, пост-хилинга, тихих отказов и
РАСШИРЕННОГО фронта — последний назван швом `MotorcycleSeam.current()`, а не
замолчан: без него `START` и `SWITCH` недостижимы.
"""

from .cell_grid import CellGridRejected, CellGridV1, CellIndexV1
from .event_time import (
    ZERO_TIME,
    EventPointV1,
    EventTimeOutcome,
    EventTimeV1,
    SupportLineV1,
    compare_times,
    concurrency_time,
    event_point,
    times_are_equal,
)
from .motorcycle import (
    CrashKind,
    MotorcycleGraphV1,
    TraceCandidateIndexV1,
    TraceOutcome,
    TraceV1,
    WallV1,
    build_motorcycle_graph,
    march_budget,
    walls_of,
)
from .events import (
    CandidateEventV1,
    EventKind,
    EventQueueV1,
    MotorcycleSeam,
    cluster_by_point,
)
from .polygon import LoopV1, PolygonOutcome, PolygonRejected, PolygonV1
from .skeleton import (
    SkeletonNodeV1,
    SkeletonOutcome,
    SkeletonV1,
    SplitSearch,
    build_skeleton,
    level_budget,
)
from .sqrt_sum import SqrtSumV1

__all__ = [
    "CandidateEventV1",
    "CellGridRejected",
    "CellGridV1",
    "CellIndexV1",
    "CrashKind",
    "EventKind",
    "EventPointV1",
    "EventQueueV1",
    "EventTimeOutcome",
    "EventTimeV1",
    "LoopV1",
    "MotorcycleGraphV1",
    "MotorcycleSeam",
    "PolygonOutcome",
    "PolygonRejected",
    "PolygonV1",
    "SkeletonNodeV1",
    "SkeletonOutcome",
    "SkeletonV1",
    "SplitSearch",
    "SqrtSumV1",
    "SupportLineV1",
    "TraceCandidateIndexV1",
    "TraceOutcome",
    "TraceV1",
    "WallV1",
    "ZERO_TIME",
    "build_motorcycle_graph",
    "build_skeleton",
    "cluster_by_point",
    "compare_times",
    "concurrency_time",
    "event_point",
    "level_budget",
    "march_budget",
    "times_are_equal",
    "walls_of",
]
