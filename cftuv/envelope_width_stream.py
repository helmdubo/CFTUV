"""Живой точный меш во время перетаскивания ширины (LIVE_DRAG_STREAM_V1) и фоновая сверка (DRAG_VERIFY_BACKGROUND_V1): журнал перетаскивания и решение простоя. Чистый модуль: ни Blender, ни сессии.

ЧТО ВИДИТ ВЛАДЕЛЕЦ (настройка «Live exact mesh while dragging», `drag_live_mesh`, ПО УМОЛЧАНИЮ ВЫКЛЮЧЕНА). Каждое событие мыши, как и прежде, сразу двигает
меш кадром модели (`envelope_width_live.width_event_preview`) и вдобавок заказывает инкрементальный точный такт (WIDTH_REFRESH_INCREMENTAL_V1) ПОТОКОВЫМ заказом
планировщика (`envelope_alpha_preview`, `request(stream=True)`): без паузы, полёт не отменяется новым значением, следующим стартует последнее значение, а замещённые
до старта не считаются вовсе. Готовый такт ложится в меш (`lean_exact_now`), даже если рука уже ушла дальше: это точный меш ближайшей ширины, и меш тут же
сдвигается кадром НОВОЙ модели к ширине курсора (`reframe`). Подтверждение - прежний полный COMMIT, отмена - прежний возврат побитово.

СВЕРКА ФОНОМ (DRAG_VERIFY_BACKGROUND_V1, решение владельца 2026-10-19: контракт кадра DRAG_FRAME_STRUCTURE_AS_VERIFIED_V1). Срок сверки (`INCREMENTAL_VERIFY_PERIOD_*`) больше не делает такт
полным (полный lean-такт стоит 0.24 с на `building` и 1.46 с на `cover.008`, и рука его отменяла): каждый потоковый такт считает заново ещё `batch` обслуживаемых доменов (половина воркеров) в порядке риска
(`envelope_width_incremental.verify_batch`: путь ширины от их последнего точного счёта к запасу интервала), и они ложатся в образец точными - их структура становится новой основой. Обслуженный домен показывает
структуру своего последнего точного счёта (журнал `verified`), каждый домен такта обслужен либо придержан с именем (`DRAG_VERIFY_BACKGROUND_V1` - ради сверки). Сверку рука не отменяет и не ждёт: партия
идёт внутри такта, который рука и так ждёт. Когда рука стоит, простой заказывает такты на ширине курсора, пока все домены не сверены на ней (`idle_answer`); новое движение просто становится
следующим тактом. Что сверка исправила, названо: домены, чья структура разошлась с показанной (`structure_changes`), и смена раскладки меша (`layout_changes`).

НИЧЕГО НЕ ПРОПАДАЕТ МОЛЧА. Шаги, точные такты на ширине курсора и на ближайшей ширине, замещённые заказы, отложенные сроки, сверенные домены и их находки -
счётчики журнала, строка панели и строка консоли в конце перетаскивания; события (шаг, точный такт с моментами старта, конца счёта и применения) хранит ограниченный журнал для замеров.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

from .envelope_production_result import PLACEMENT_REBASED
from .envelope_width_incremental import VERIFY_BACKGROUND

#: Шаг перетаскивания не заказал такта: на экране нет образца точной записи (модель снята с названной причиной).
STEP_NO_SAMPLE = "LIVE_DRAG_STEP_WITHOUT_DISPLAYED_SAMPLE"
#: Сколько событий хранит журнал для замеров.
EVENTS_KEPT = 8192

EVENT_STEP = "step"
EVENT_EXACT = "exact"


@dataclass(slots=True)
class LiveDragLogV1:
    """Журнал одного перетаскивания с живым точным мешем: счётчики, журнал сверки доменов и события.

    События: `("step", номер, ширина, момент заказа)` и `("exact", номер, ширина, момент применения, момент заказа, старт счёта, конец счёта, начало применения)`.
    """

    #: Заказ последнего шага (`WidthLiveTargetV1`): по нему простой заказывает такты сверки.
    target: object = None
    cursor: float | None = None
    steps: int = 0
    #: Точный такт лёг, когда курсор стоял на его ширине / уже ушёл (меш сдвинут кадром новой модели к курсору).
    exact_at_cursor: int = 0
    exact_nearest: int = 0
    reframes: int = 0
    #: `{патч: ширина последнего точного счёта домена}` (DRAG_VERIFY_BACKGROUND_V1).
    verified: dict = field(default_factory=dict)
    verify_domains: int = 0
    structure_changes: int = 0
    layout_changes: int = 0
    idle_ticks: int = 0
    #: Сколько доменов не сверено на ширине курсора, когда простой заказал последний такт: простой, что не продвинулся, больше не заказывает.
    idle_left: int | None = None
    #: Срок сверки, не исполненный полным тактом: имя -> сколько тактов; полные такты по другим причинам: имя -> сколько.
    deferred: dict = field(default_factory=dict)
    full_ticks: dict = field(default_factory=dict)
    refused: dict = field(default_factory=dict)
    events: deque = field(default_factory=lambda: deque(maxlen=EVENTS_KEPT))
    #: Счётчики планировщика (`PreviewCountersV1`) в начале перетаскивания: журнал называет их приращение за это перетаскивание.
    baseline: object = None

    def count(self, table: dict, name: str) -> None:
        table[name] = table.get(name, 0) + 1


def seed_verified(log: LiveDragLogV1, displayed, state) -> None:
    """Начало перетаскивания: домены образца на экране сверены на ширине его последнего полного счёта (запись сверки `IncrementalStateV1`, если на экране инкрементальный образец, иначе ширина образца)."""

    if displayed is None:
        return
    alpha = float(state.verified_alpha) if state is not None and state.ticks > 0 else float(displayed.alpha)
    log.verified = {item.patch_id: alpha for item in displayed.domains}


def note_step(log: LiveDragLogV1, request, target) -> None:
    """Шаг перетаскивания заказан потоковым заказом `request` (`PreviewRequestV1`)."""

    log.steps += 1
    log.cursor = float(request.alpha)
    log.target = target
    log.idle_left = None
    log.events.append((EVENT_STEP, int(request.sequence), float(request.alpha), float(request.requested_at)))


def _structures(sample) -> dict:
    return {} if sample is None else {item.patch_id: item.structure for item in sample.domains}


def note_applied(log: LiveDragLogV1, request, live, before, moments: tuple) -> float | None:
    """Точный такт `live` (`LiveRunV1`) лёг в меш вместо образца `before`; `moments` - `(старт счёта, конец счёта, начало применения, конец применения)` по `time.monotonic`.

    Журнал сверки: домены, посчитанные тактом заново (полный такт - все), сверены на его ширине; домены партии сверки, чья структура разошлась с показанной, названы.
    Возвращает ширину курсора, к которой меш надо сдвинуть кадром модели, либо `None` (курсор на ширине такта).
    """

    alpha = float(request.alpha)
    plan = live.plan
    if plan is not None and plan.incremental:
        if plan.deferred:
            log.count(log.deferred, plan.deferred)
        computed = {int(item.patch_id) for item in live.run.results if item.placement != PLACEMENT_REBASED}
        checked = {patch for patch, name in (plan.held or {}).items() if name == VERIFY_BACKGROUND}
        if checked:
            old, new = _structures(before), _structures(live.sample)
            changed = sum(1 for patch in checked if old.get(patch) != new.get(patch))
            log.verify_domains += len(checked)
            log.structure_changes += changed
            log.layout_changes += int(bool(changed) and before is not None and before.layout_digest != live.sample.layout_digest)
    else:
        if plan is not None:
            log.count(log.full_ticks, plan.reason)
        computed = {item.patch_id for item in live.sample.domains}
    for patch in computed:
        log.verified[patch] = alpha
    started, finished, applying, applied = moments
    log.events.append((EVENT_EXACT, int(request.sequence), alpha, float(applied), float(request.requested_at), float(started), float(finished), float(applying)))
    if log.cursor is None or log.cursor == alpha:
        log.exact_at_cursor += 1
        return None
    log.exact_nearest += 1
    log.reframes += 1
    return log.cursor


def unverified_at(log: LiveDragLogV1, displayed, width: float) -> int:
    """Сколько доменов образца на экране не посчитаны точно на ширине `width`."""

    return sum(1 for item in displayed.domains if log.verified.get(item.patch_id) != width)


def idle_answer(log: LiveDragLogV1 | None, displayed, *, drag_active: bool) -> float | None:
    """Ширина такта сверки в простое либо `None`: перетаскивание идёт, образец на экране есть, и не все его домены посчитаны точно на ширине курсора.

    Простой, что не продвинулся (число несверенных не уменьшилось с прошлого его такта: отказ, домены без интервала), больше не заказывает, пока курсор не сдвинется.
    """

    if log is None or not drag_active or displayed is None or log.target is None or log.cursor is None:
        return None
    left = unverified_at(log, displayed, log.cursor)
    if left == 0 or (log.idle_left is not None and left >= log.idle_left):
        return None
    log.idle_left = left
    log.idle_ticks += 1
    return log.cursor


def report(log: LiveDragLogV1, counters=None) -> dict:
    """Счётчики журнала (и приращение счётчиков планировщика, `PreviewCountersV1`) словарём для строки консоли и инструментов замера."""

    data = {
        "steps": log.steps, "exact_at_cursor": log.exact_at_cursor, "exact_nearest": log.exact_nearest, "reframes": log.reframes,
        "verify_domains": log.verify_domains, "structure_changes": log.structure_changes, "layout_changes": log.layout_changes, "idle_ticks": log.idle_ticks,
        "deferred": dict(log.deferred), "full_ticks": dict(log.full_ticks), "refused": dict(log.refused),
    }
    if counters is not None:
        names = {"requested": "requested", "superseded": "coalesced", "started": "started", "applied": "applied", "nearest": "nearest",
                 "stale": "stale", "invalid": "invalid", "unavailable": "unavailable", "failed": "failed", "cancelled": "cancelled"}
        base = log.baseline
        data["scheduler"] = {name: getattr(counters, field_name) - (0 if base is None else getattr(base, field_name)) for name, field_name in names.items()}
    return data


def status_line(log: LiveDragLogV1, counters=None) -> str:
    """Строка панели и консоли: что сделал живой точный меш за перетаскивание."""

    scheduler = report(log, counters).get("scheduler", {})
    deferred = sum(log.deferred.values())
    full = ", ".join(f"{name} {count}" for name, count in sorted(log.full_ticks.items()))
    refused = ", ".join(f"{name} {count}" for name, count in sorted(log.refused.items()))
    return (
        f"Live drag LIVE_DRAG_STREAM_V1: {log.steps} steps, exact {log.exact_at_cursor} at the pointer + {log.exact_nearest} at a nearer width (model moved to the pointer), "
        f"superseded {scheduler.get('superseded', 0)}, stale {scheduler.get('stale', 0)}; {VERIFY_BACKGROUND}: {log.verify_domains} domains verified in the ticks "
        f"(+{log.idle_ticks} idle ticks), {log.structure_changes} domain structures and {log.layout_changes} layouts corrected; verification periods not run as full ticks {deferred}"
        + (f"; full ticks: {full}" if full else "") + (f"; refused: {refused}" if refused else "")
    )


__all__ = (
    "EVENTS_KEPT",
    "EVENT_EXACT",
    "EVENT_STEP",
    "LiveDragLogV1",
    "STEP_NO_SAMPLE",
    "idle_answer",
    "note_applied",
    "note_step",
    "report",
    "seed_verified",
    "status_line",
    "unverified_at",
)
