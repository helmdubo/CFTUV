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

from importlib import import_module as _import_module

__all__ = [
    "CandidateEventV1",
    "CellGridRejected",
    "CellGridV1",
    "CellIndexV1",
    "ConveyorCoverageV1",
    "ConveyorFaceCoverageV1",
    "ConveyorOutcome",
    "ConveyorPreparationV1",
    "ConveyorRegionCoverageV1",
    "CrashKind",
    "DegradedMiterCornerV1",
    "EventKind",
    "EventPointV1",
    "EventQueueV1",
    "EventTimeOutcome",
    "EventTimeV1",
    "FanSupportV1",
    "LoopV1",
    "MotorcycleGraphV1",
    "MotorcycleSeam",
    "PolygonOutcome",
    "PolygonRejected",
    "PolygonV1",
    "PreparedRegionV1",
    "SkeletonNodeV1",
    "SkeletonOutcome",
    "SkeletonV1",
    "SplitSearch",
    "SqrtSumV1",
    "SupportLineV1",
    "TraceCandidateIndexV1",
    "TraceOutcome",
    "TraceV1",
    "VertexFanV1",
    "WallV1",
    "ZERO_TIME",
    "build_motorcycle_graph",
    "build_skeleton",
    "chart_lattice_for_frame",
    "cluster_by_point",
    "compare_times",
    "concurrency_time",
    "conveyor_coverage",
    "evaluate_conveyor_coverage",
    "event_point",
    "level_budget",
    "march_budget",
    "prepare_conveyor",
    "times_are_equal",
    "walls_of",
    "with_vertex_fans",
]

_EXPORT_RANGES = (
    (0, 1, "cftuv_envelope.wavefront.events"),
    (1, 4, "cftuv_envelope.wavefront.cell_grid"),
    (4, 9, "cftuv_envelope.wavefront.conveyor"),
    (9, 10, "cftuv_envelope.wavefront.motorcycle"),
    (10, 11, "cftuv_envelope.wavefront.conveyor"),
    (11, 12, "cftuv_envelope.wavefront.events"),
    (12, 13, "cftuv_envelope.wavefront.event_time"),
    (13, 14, "cftuv_envelope.wavefront.events"),
    (14, 16, "cftuv_envelope.wavefront.event_time"),
    (16, 18, "cftuv_envelope.wavefront.polygon"),
    (18, 19, "cftuv_envelope.wavefront.motorcycle"),
    (19, 20, "cftuv_envelope.wavefront.events"),
    (20, 23, "cftuv_envelope.wavefront.polygon"),
    (23, 24, "cftuv_envelope.wavefront.conveyor"),
    (24, 28, "cftuv_envelope.wavefront.skeleton"),
    (28, 29, "cftuv_envelope.wavefront.sqrt_sum"),
    (29, 30, "cftuv_envelope.wavefront.event_time"),
    (30, 33, "cftuv_envelope.wavefront.motorcycle"),
    (33, 34, "cftuv_envelope.wavefront.polygon"),
    (34, 35, "cftuv_envelope.wavefront.motorcycle"),
    (35, 36, "cftuv_envelope.wavefront.event_time"),
    (36, 37, "cftuv_envelope.wavefront.motorcycle"),
    (37, 38, "cftuv_envelope.wavefront.skeleton"),
    (38, 39, "cftuv_envelope.wavefront.conveyor"),
    (39, 40, "cftuv_envelope.wavefront.events"),
    (40, 42, "cftuv_envelope.wavefront.event_time"),
    (42, 44, "cftuv_envelope.wavefront.conveyor"),
    (44, 45, "cftuv_envelope.wavefront.event_time"),
    (45, 46, "cftuv_envelope.wavefront.skeleton"),
    (46, 47, "cftuv_envelope.wavefront.motorcycle"),
    (47, 48, "cftuv_envelope.wavefront.conveyor"),
    (48, 49, "cftuv_envelope.wavefront.event_time"),
    (49, 50, "cftuv_envelope.wavefront.motorcycle"),
    (50, 51, "cftuv_envelope.wavefront.polygon"),
)

_EXPORTS = {
    name: (module_name, name)
    for start, stop, module_name in _EXPORT_RANGES
    for name in __all__[start:stop]
}

_SUBMODULES = {
    name: f"cftuv_envelope.wavefront.{name}"
    for name in (
        "bridge",
        "cell_grid",
        "conveyor",
        "coverage",
        "event_time",
        "events",
        "faces",
        "motorcycle",
        "polygon",
        "skeleton",
        "sqrt_sum",
    )
}

_LEGACY_DIR_EXTRAS = (
    "__all__",
    "__builtins__",
    "__cached__",
    "__doc__",
    "__file__",
    "__loader__",
    "__name__",
    "__package__",
    "__path__",
    "__spec__",
    *_SUBMODULES,
)


def __getattr__(name: str):
    target = _EXPORTS.get(name)
    if target is not None:
        module_name, symbol_name = target
        value = getattr(_import_module(module_name), symbol_name)
        globals()[name] = value
        return value

    module_name = _SUBMODULES.get(name)
    if module_name is not None:
        value = _import_module(module_name)
        globals()[name] = value
        return value

    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__() -> list[str]:
    return sorted((*__all__, *_LEGACY_DIR_EXTRAS))
