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
идёт внутри такта, который рука и так ждёт. Когда рука стоит, простой заказывает такты на ширине курсора, пока все домены не сверены на ней, но не больше `IDLE_TICKS_PER_PAUSE` за остановку (`idle_answer`); новое движение просто становится
следующим тактом. Исход сверки домена назван (`verify_outcomes`): `STRUCTURE_SAME`; `STRUCTURE_CHANGED` - структура разошлась с показанной, и посчитанный результат ПРИНЯТ новой основой домена
(он ложится в образец и его сертификат двигает домен дальше), число таких отдельно (`structure_changes`), смена раскладки меша - `layout_changes`; `WELD_CHANGED` - структура та же, но сварка группы такта разошлась с мешем на экране; `INCOMPARABLE` - домена нет в одном из образцов.
Сверка - на ширине своего такта: такт ближайшей ширины сверяет домены на СВОЕЙ ширине, а не на ширине курсора; журнал сверки привязан к ключу образца (другой ключ - журнал заведён заново).

ЧЕГО ЭТО НЕ ОБЕЩАЕТ. Обслуженный домен показывает структуру своего последнего точного счёта: `STRUCTURE_AS_OF:<ширина>` (строка панели называет, сколько доменов показаны по другой ширине и самую
старую из них). Решения с допуском (силуэт, хорды) на другой ширине могли бы выйти за свой порог: граница ошибки сверкой не заверена, она лишь находит и называет такие домены. Порядок риска -
путь ширины к запасу интервала (запас решений с допуском хост не читает дёшево). Сварка швов между доменами (`envelope_width_composite`) проверяется только в группе посчитанных тактом доменов: швы
между обслуженными остаются такими, как в последнем точном счёте, до полного такта либо подтверждения (COMMIT полный).

НИЧЕГО НЕ ПРОПАДАЕТ МОЛЧА. Шаги, точные такты на ширине курсора и на ближайшей ширине, замещённые заказы, отложенные сроки, сверенные домены и их находки -
счётчики журнала, строка панели и строка консоли в конце перетаскивания; события (шаг, точный такт с моментами старта, конца счёта и применения) хранит ограниченный журнал для замеров.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

from .envelope_production_result import PLACEMENT_REBASED
from .envelope_production_weld import OUTCOME_WELD_POSITION_MISMATCH
from .envelope_width_composite import COMPOSITE_WELD_CHANGED
from .envelope_width_incremental import VERIFY_BACKGROUND

#: Шаг перетаскивания не заказал такта: на экране нет образца точной записи (модель снята с названной причиной).
STEP_NO_SAMPLE = "LIVE_DRAG_STEP_WITHOUT_DISPLAYED_SAMPLE"
#: Сколько событий хранит журнал для замеров.
EVENTS_KEPT = 8192
#: Исходы сверки домена (DRAG_VERIFY_BACKGROUND_V1).
STRUCTURE_SAME = "STRUCTURE_SAME"
STRUCTURE_CHANGED = "STRUCTURE_CHANGED"
INCOMPARABLE = "INCOMPARABLE"
#: Структура домена та же, но сварка группы такта разошлась с мешем на экране (`INCREMENTAL_COMPOSITE_WELD_CHANGED`): образец собран заново из всех доменов, домен - не «тот же».
WELD_CHANGED = "WELD_CHANGED"
#: Простой перестал заказывать такты сверки: потрачен бюджет одной остановки руки (DRAG_VERIFY_IDLE_BUDGET_V1).
IDLE_BUDGET_SPENT = "DRAG_VERIFY_IDLE_BUDGET_SPENT"
#: Сколько тактов сверки простой заказывает за одну остановку руки: дальше домены остаются `STRUCTURE_AS_OF` до следующего шага, полного такта или COMMIT.
IDLE_TICKS_PER_PAUSE = 8
#: Имя статуса обслуженного домена: структура его последнего точного счёта на названной ширине.
STRUCTURE_AS_OF = "STRUCTURE_AS_OF"

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
    #: Ключ образца, к которому относится журнал сверки (`PreviewSampleV1.key`).
    key: object = None
    verify_outcomes: dict = field(default_factory=dict)
    verify_domains: int = 0
    structure_changes: int = 0
    layout_changes: int = 0
    idle_ticks: int = 0
    #: Такты простоя с последнего шага руки (бюджет `IDLE_TICKS_PER_PAUSE`) и сколько раз бюджет кончился, а несверенные остались (`IDLE_BUDGET_SPENT`).
    idle_in_pause: int = 0
    idle_budget_spent: int = 0
    #: Такты, у которых сварка группы разошлась с мешем на экране (`INCREMENTAL_COMPOSITE_WELD_CHANGED`), и наибольшее число общих вершин с побитово разными позициями
    #: (`ADAPTER_WELD_POSITION_MISMATCH` массивов такта): обслуженный домен равен точному лишь с точностью округления, и побитовая сварка может развести вершины.
    weld_changes: int = 0
    weld_position_mismatch: int = 0
    #: Такты, чья раскладка группы переложена (COMPOSITE_SPLICE_V1: имя причины -> сколько) и что откатились на массивы всего меша (имя отката -> сколько).
    spliced: dict = field(default_factory=dict)
    fallbacks: dict = field(default_factory=dict)
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
    log.key = displayed.key


def note_step(log: LiveDragLogV1, request, target) -> None:
    """Шаг перетаскивания заказан потоковым заказом `request` (`PreviewRequestV1`)."""

    log.steps += 1
    log.cursor = float(request.alpha)
    log.target = target
    log.idle_left = None
    log.idle_in_pause = 0
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
    key = getattr(live.sample, "key", None)
    if log.key is not None and key != log.key:  # другой образец (ключ): прежний журнал сверки не про него
        log.verified, log.key = {}, key
    if plan is not None and plan.incremental:
        if plan.deferred:
            log.count(log.deferred, plan.deferred)
        computed = {int(item.patch_id) for item in live.run.results if item.placement != PLACEMENT_REBASED}
        checked = {patch for patch, name in (plan.held or {}).items() if name == VERIFY_BACKGROUND}
        weld_changed = COMPOSITE_WELD_CHANGED in (plan.fallback, getattr(plan, "spliced", ""))
        log.weld_changes += int(weld_changed)
        if getattr(plan, "spliced", ""):
            log.count(log.spliced, plan.spliced)  # COMPOSITE_SPLICE_V1: раскладка группы переложена, отката на весь меш не было
        elif plan.fallback:
            log.count(log.fallbacks, plan.fallback)
        if checked:
            old, new = _structures(before), _structures(live.sample)
            changed = 0
            for patch in checked:
                if patch not in old or patch not in new:
                    log.count(log.verify_outcomes, INCOMPARABLE)
                elif old[patch] != new[patch]:
                    changed += 1
                    log.count(log.verify_outcomes, STRUCTURE_CHANGED)  # посчитанный результат принят новой основой домена
                else:
                    log.count(log.verify_outcomes, WELD_CHANGED if weld_changed else STRUCTURE_SAME)
            log.verify_domains += len(checked)
            log.structure_changes += changed
    else:
        if plan is not None:
            log.count(log.full_ticks, plan.reason)
        computed = {item.patch_id for item in live.sample.domains}
    log.layout_changes += int(before is not None and before.layout_digest != live.sample.layout_digest)  # раскладка меша сменилась - по чьей угодно причине, не только у партии сверки
    counters = dict(getattr(getattr(live, "arrays", None), "weld_counters", ()) or ())
    log.weld_position_mismatch = max(log.weld_position_mismatch, int(counters.get(OUTCOME_WELD_POSITION_MISMATCH, 0)))
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


def pointer_after(log: LiveDragLogV1 | None, request) -> float | None:
    """Ширина курсора, к которой такт `request` надо сдвинуть кадром модели, когда он ляжет, либо `None` (журнала нет либо курсор на ширине такта); то же решение, что вернёт `note_applied`."""

    if log is None or log.cursor is None or log.cursor == float(request.alpha):
        return None
    return log.cursor


def unverified_at(log: LiveDragLogV1, displayed, width: float) -> int:
    """Сколько доменов образца на экране не посчитаны точно на ширине `width`."""

    return sum(1 for item in displayed.domains if log.verified.get(item.patch_id) != width)


def idle_answer(log: LiveDragLogV1 | None, displayed, *, drag_active: bool) -> float | None:
    """Ширина такта сверки в простое либо `None`: перетаскивание идёт, образец на экране есть, и не все его домены посчитаны точно на ширине курсора.

    Простой, что не продвинулся (число несверенных не уменьшилось с прошлого его такта: отказ, домены без интервала), больше не заказывает, пока курсор не сдвинется.
    За одну остановку руки - не больше `IDLE_TICKS_PER_PAUSE` тактов (DRAG_VERIFY_IDLE_BUDGET_V1): полная сверка `cover.008` стоила бы ~175 тактов и 10-15 с пула после каждой паузы;
    бюджет кончился, а несверенные остались - счётчик `idle_budget_spent` (`IDLE_BUDGET_SPENT`), домены показаны `STRUCTURE_AS_OF`.
    """

    if log is None or not drag_active or displayed is None or log.target is None or log.cursor is None:
        return None
    left = unverified_at(log, displayed, log.cursor)
    if left == 0 or (log.idle_left is not None and left >= log.idle_left):
        return None
    if log.idle_in_pause >= IDLE_TICKS_PER_PAUSE:
        if log.idle_in_pause == IDLE_TICKS_PER_PAUSE:
            log.idle_budget_spent += 1
            log.idle_in_pause += 1  # названо один раз за остановку
        return None
    log.idle_left = left
    log.idle_ticks += 1
    log.idle_in_pause += 1
    return log.cursor


def structure_as_of(log: LiveDragLogV1) -> tuple:
    """`(сколько доменов показаны по структуре другой ширины, чем курсор, самая далёкая от курсора такая ширина либо None)` - статус `STRUCTURE_AS_OF:<ширина>`."""

    if log.cursor is None:
        return (0, None)
    others = [width for width in log.verified.values() if width != log.cursor]
    return (len(others), None if not others else max(others, key=lambda width: abs(width - log.cursor)))


def report(log: LiveDragLogV1, counters=None) -> dict:
    """Счётчики журнала (и приращение счётчиков планировщика, `PreviewCountersV1`) словарём для строки консоли и инструментов замера."""

    count, farthest = structure_as_of(log)
    data = {
        "structure_as_of": {"domains": count, "farthest_width": farthest}, "verify_outcomes": dict(log.verify_outcomes),
        "steps": log.steps, "exact_at_cursor": log.exact_at_cursor, "exact_nearest": log.exact_nearest, "reframes": log.reframes,
        "verify_domains": log.verify_domains, "structure_changes": log.structure_changes, "layout_changes": log.layout_changes, "idle_ticks": log.idle_ticks,
        "idle_budget_spent": log.idle_budget_spent, "weld_changes": log.weld_changes, "weld_position_mismatch": log.weld_position_mismatch,
        "deferred": dict(log.deferred), "full_ticks": dict(log.full_ticks), "refused": dict(log.refused), "spliced": dict(log.spliced), "fallbacks": dict(log.fallbacks),
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
    outcomes = ", ".join(f"{name} {count}" for name, count in sorted(log.verify_outcomes.items()))
    stale, farthest = structure_as_of(log)
    as_of = f"; {stale} domains shown {STRUCTURE_AS_OF}:<other width>" + ("" if farthest is None else f" (farthest {STRUCTURE_AS_OF}:{farthest:.4g})")
    return (
        f"Live drag LIVE_DRAG_STREAM_V1: {log.steps} steps, exact {log.exact_at_cursor} at the pointer + {log.exact_nearest} at a nearer width (model moved to the pointer), "
        f"superseded {scheduler.get('superseded', 0)}, stale {scheduler.get('stale', 0)}; {VERIFY_BACKGROUND}: {log.verify_domains} domains verified in the ticks "
        f"(+{log.idle_ticks} idle ticks" + (f", {IDLE_BUDGET_SPENT} {log.idle_budget_spent}" if log.idle_budget_spent else "") + f"), {log.structure_changes} domain structures and {log.layout_changes} layouts corrected" + (f" ({outcomes})" if outcomes else "") + as_of
        + (f"; layouts spliced (COMPOSITE_SPLICE_V1): " + ", ".join(f"{name} {count}" for name, count in sorted(log.spliced.items())) if log.spliced else "")
        + (f"; whole-mesh rollbacks: " + ", ".join(f"{name} {count}" for name, count in sorted(log.fallbacks.items())) if log.fallbacks else "")
        + (f"; weld diverged in {log.weld_changes} ticks" if log.weld_changes else "")
        + (f"; {OUTCOME_WELD_POSITION_MISMATCH} up to {log.weld_position_mismatch}" if log.weld_position_mismatch else "")
        + f"; verification periods not run as full ticks {deferred}" + (f"; full ticks: {full}" if full else "") + (f"; refused: {refused}" if refused else "")
    )


__all__ = (
    "EVENTS_KEPT",
    "EVENT_EXACT",
    "EVENT_STEP",
    "IDLE_BUDGET_SPENT",
    "IDLE_TICKS_PER_PAUSE",
    "INCOMPARABLE",
    "LiveDragLogV1",
    "STRUCTURE_AS_OF",
    "STRUCTURE_CHANGED",
    "STRUCTURE_SAME",
    "structure_as_of",
    "STEP_NO_SAMPLE",
    "idle_answer",
    "note_applied",
    "note_step",
    "pointer_after",
    "report",
    "seed_verified",
    "status_line",
    "unverified_at",
    "WELD_CHANGED",
)
