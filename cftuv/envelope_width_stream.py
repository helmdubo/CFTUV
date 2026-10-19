"""Живой точный меш во время перетаскивания ширины (LIVE_DRAG_STREAM_V1): журнал перетаскивания и решение простоя. Чистый модуль: ни Blender, ни сессии.

ЧТО ВИДИТ ВЛАДЕЛЕЦ (настройка «Live exact mesh while dragging», `drag_live_mesh`, ПО УМОЛЧАНИЮ ВЫКЛЮЧЕНА). Каждое событие мыши, как и прежде, сразу двигает
меш кадром модели (`envelope_width_live.width_event_preview`) и вдобавок заказывает инкрементальный точный такт (WIDTH_REFRESH_INCREMENTAL_V1) ПОТОКОВЫМ заказом
планировщика (`envelope_alpha_preview`, `request(stream=True)`): без паузы, полёт не отменяется новым значением, следующим стартует последнее значение, а замещённые
до старта не считаются вовсе. Готовый такт ложится в меш (`lean_exact_now`), даже если рука уже ушла дальше: это точный меш ближайшей ширины, и меш тут же
сдвигается кадром НОВОЙ модели к ширине курсора (`reframe`). Подтверждение - прежний полный COMMIT, отмена - прежний возврат побитово.

СВЕРКА В ПРОСТОЕ. Пока рука движется, срок сверки (`INCREMENTAL_VERIFY_PERIOD_*`) не делает такт полным (полный lean-такт стоит 0.24 с на `building` и 1.46 с на
`cover.008` и задержал бы меш): срок ОТЛОЖЕН и назван (`deferred`). Когда такт применён и новых значений нет, планировщик спрашивает простой (`idle_answer`): образец на
экране не сверен на ширине курсора - вытесняемый заказ полного lean-такта на ширине курсора (`RECONCILE_IDLE`, через `IDLE_DELAY_SECONDS`). Новое движение руки его
вытесняет (`preempted`). Сверка, что легла, называет, что исправила: домены, чья структура разошлась с образцом на экране (`structure_changes`), смену раскладки меша
(`layout_changes`) и опровергнутые сверкой модели домены (`refuted`).

НИЧЕГО НЕ ПРОПАДАЕТ МОЛЧА. Шаги, точные такты на ширине курсора и на ближайшей ширине, замещённые и вытесненные заказы, отложенные сроки сверки, сверки и их находки -
счётчики журнала, строка панели и строка консоли в конце перетаскивания; события (шаг, точный такт) с моментами хранит ограниченный журнал для замеров.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

#: Причина полного lean-такта, заказанного простоем (сверка образца на экране на ширине курсора).
RECONCILE_IDLE = "LIVE_DRAG_RECONCILE_IDLE"
#: Шаг перетаскивания не заказал такта: на экране нет образца точной записи (модель снята с названной причиной).
STEP_NO_SAMPLE = "LIVE_DRAG_STEP_WITHOUT_DISPLAYED_SAMPLE"
#: Сколько событий хранит журнал для замеров.
EVENTS_KEPT = 8192

EVENT_STEP = "step"
EVENT_EXACT = "exact"
EVENT_RECONCILE = "reconcile"


@dataclass(slots=True)
class LiveDragLogV1:
    """Журнал одного перетаскивания с живым точным мешем: счётчики и события `(вид, номер заказа, ширина, момент)`."""

    #: Заказ последнего шага (`WidthLiveTargetV1`): по нему простой заказывает сверку.
    target: object = None
    cursor: float | None = None
    steps: int = 0
    #: Точный такт лёг, когда курсор стоял на его ширине / уже ушёл (меш сдвинут кадром новой модели к курсору).
    exact_at_cursor: int = 0
    exact_nearest: int = 0
    reframes: int = 0
    reconcile_requested: int = 0
    reconcile_applied: int = 0
    structure_changes: int = 0
    layout_changes: int = 0
    refuted: int = 0
    #: Ширина последней заказанной сверки: одна попытка на положение курсора (сверка, что не легла, не крутится в простое).
    reconcile_cursor: float | None = None
    #: Срок сверки, отложенный, пока рука движется: имя -> сколько тактов; полные такты по другим причинам: имя -> сколько.
    deferred: dict = field(default_factory=dict)
    full_ticks: dict = field(default_factory=dict)
    refused: dict = field(default_factory=dict)
    events: deque = field(default_factory=lambda: deque(maxlen=EVENTS_KEPT))
    #: Счётчики планировщика (`PreviewCountersV1`) в начале перетаскивания: журнал называет их приращение за это перетаскивание.
    baseline: object = None

    def count(self, table: dict, name: str) -> None:
        table[name] = table.get(name, 0) + 1


def note_step(log: LiveDragLogV1, request, target) -> None:
    """Шаг перетаскивания заказан потоковым заказом `request` (`PreviewRequestV1`)."""

    log.steps += 1
    log.cursor = float(request.alpha)
    log.target = target
    log.events.append((EVENT_STEP, int(request.sequence), float(request.alpha), float(request.requested_at)))


def _structures(sample) -> dict:
    return {} if sample is None else {(item.patch_id, item.domain_id): item.structure for item in sample.domains}


def note_applied(log: LiveDragLogV1, request, live, before, now: float, *, reconcile: bool) -> float | None:
    """Точный такт `live` (`LiveRunV1`) лёг в меш вместо образца `before`. Возвращает ширину курсора, к которой меш надо сдвинуть кадром модели, либо `None` (курсор на ширине такта).

    Сверка (`reconcile`) называет, что исправила: домены, чья структура на экране была другой, смену раскладки и опровергнутые сверкой модели домены.
    """

    alpha = float(request.alpha)
    plan = live.plan
    if plan is not None and plan.incremental and plan.deferred:
        log.count(log.deferred, plan.deferred)
    elif plan is not None and not plan.incremental:
        log.count(log.full_ticks, plan.reason)
    if reconcile:
        log.reconcile_applied += 1
        old, new = _structures(before), _structures(live.sample)
        log.structure_changes += sum(1 for key in old.keys() | new.keys() if old.get(key) != new.get(key))
        log.layout_changes += int(before is not None and before.layout_digest != live.sample.layout_digest)
        check = live.check
        log.refuted += 0 if check is None or check.refusal else len(check.refuted)
    log.events.append((EVENT_RECONCILE if reconcile else EVENT_EXACT, int(request.sequence), alpha, float(now)))
    if log.cursor is None or log.cursor == alpha:
        log.exact_at_cursor += 1
        return None
    log.exact_nearest += 1
    log.reframes += 1
    return log.cursor


def idle_answer(log: LiveDragLogV1 | None, state, *, drag_active: bool, has_sample: bool) -> float | None:
    """Ширина сверки в простое либо `None`: перетаскивание идёт, образец на экране есть, и он не сверен на ширине курсора (`IncrementalStateV1`: такты после сверки либо сверка на другой ширине).

    Одна попытка на положение курсора: сверка, что не легла (отказ, вытеснение руки и тот же курсор), не повторяется, пока курсор не сдвинется.
    """

    if log is None or not drag_active or not has_sample or log.target is None or log.cursor is None:
        return None
    if state is not None and state.ticks == 0 and state.verified_alpha == log.cursor:
        return None
    if log.reconcile_cursor == log.cursor:
        return None
    log.reconcile_cursor = log.cursor
    log.reconcile_requested += 1
    return log.cursor


def report(log: LiveDragLogV1, counters=None) -> dict:
    """Счётчики журнала (и планировщика, `PreviewCountersV1`) словарём для строки консоли и инструментов замера."""

    data = {
        "steps": log.steps, "exact_at_cursor": log.exact_at_cursor, "exact_nearest": log.exact_nearest, "reframes": log.reframes,
        "reconcile_requested": log.reconcile_requested, "reconcile_applied": log.reconcile_applied, "structure_changes": log.structure_changes,
        "layout_changes": log.layout_changes, "refuted": log.refuted, "deferred": dict(log.deferred), "full_ticks": dict(log.full_ticks), "refused": dict(log.refused),
    }
    if counters is not None:
        names = {"requested": "requested", "superseded": "coalesced", "started": "started", "applied": "applied", "nearest": "nearest", "preempted": "preempted",
                 "stale": "stale", "invalid": "invalid", "unavailable": "unavailable", "failed": "failed", "cancelled": "cancelled"}
        base = log.baseline
        data["scheduler"] = {name: getattr(counters, field_name) - (0 if base is None else getattr(base, field_name)) for name, field_name in names.items()}
    return data


def status_line(log: LiveDragLogV1, counters=None) -> str:
    """Строка панели и консоли: что сделал живой точный меш за перетаскивание."""

    data = report(log, counters)
    scheduler = data.get("scheduler", {})
    deferred = sum(log.deferred.values())
    full = ", ".join(f"{name} {count}" for name, count in sorted(log.full_ticks.items()))
    refused = ", ".join(f"{name} {count}" for name, count in sorted(log.refused.items()))
    return (
        f"Live drag LIVE_DRAG_STREAM_V1: {log.steps} steps, exact {log.exact_at_cursor} at the pointer + {log.exact_nearest} at a nearer width (model moved to the pointer), "
        f"superseded {scheduler.get('superseded', 0)}, stale {scheduler.get('stale', 0)}; reconciled {log.reconcile_applied}/{log.reconcile_requested} "
        f"(preempted {scheduler.get('preempted', 0)}): {log.structure_changes} domain structures, {log.layout_changes} layouts corrected, {log.refuted} refuted; "
        f"verification deferred while moving {deferred}" + (f"; full ticks: {full}" if full else "") + (f"; refused: {refused}" if refused else "")
    )


__all__ = (
    "EVENTS_KEPT",
    "EVENT_EXACT",
    "EVENT_RECONCILE",
    "EVENT_STEP",
    "LiveDragLogV1",
    "RECONCILE_IDLE",
    "STEP_NO_SAMPLE",
    "idle_answer",
    "note_applied",
    "note_step",
    "report",
    "status_line",
)
