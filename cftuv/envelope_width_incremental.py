"""Инкрементальный точный такт перетаскивания ширины и политика его сверки (WIDTH_REFRESH_INCREMENTAL_V1, чистая, без Blender).

ЧТО ЭТО. Lean-такт (WIDTH_REFRESH_LEAN_V1) пересчитывает КАЖДЫЙ домен выделения, хотя за полпроцента ширины ИНТЕРВАЛ ядра кончается у единиц доменов (замер `S6_PARAMETRIC_DRAG_SPIKE`: `building` 1.4 из 122, `cover.008` 0.7
из 1046 на тик). Инкрементальный такт считает заново только домены, которых аффинный сертификат не обслуживает (`run_production(skipped=...)`), а образец складывает из образца на экране: обслуженные двигает кадр модели, посчитанные
ложатся поверх (`envelope_width_composite`); не сложился - название причины, и образец собран из результатов ВСЕХ доменов, обслуженные сдвинуты сертификатом (`envelope_width_rebase`, тот же ответ, дороже). Обслуженные домены не сканируются, не уходят
воркерам и не ложатся в кэши; точные домены побитово равны lean-такту на той же ширине, обслуженные - сертификату с точностью округления.

ЧТО ЗАВЕРЕНО, А ЧТО НЕТ. Заверены интервал ядра и аффинность внутри него (`ParametricCertificateV1`). НЕ заверены решения тесселяции, силуэта и митры у обслуженных доменов (`interval.UNCERTIFIED`): у такого домена
структура на новой ширине могла быть другой, а такт её не пересчитывает. Поэтому такт НЕ ЕСТЬ зафиксированный ответ и не заменяет проверку: сверяют её полный lean-такт (`deviation` и журнал доверия: опровергнутый
домен идёт в карантин и считается заново) и подтверждение (COMMIT всегда полный; ничего из инкрементальных тактов в него не попадает).

ПОЛИТИКА СВЕРКИ (`verification_reason`). Тик инкрементальный, пока не назван повод считать его полным (полный lean-такт - это и есть сверка). Повод считают значения, а не часы: расхождение структуры копится с ПУТЁМ ширины от
последней сверки, а не со временем, и число тактов - потолок на случай мелких шагов.
  * `INCREMENTAL_NO_VERIFIED_BASE` - нет записи сверки (кнопка и полные такты её пишут) либо образец сменился;
  * `INCREMENTAL_VERIFY_PERIOD_TICKS` - с последней полной сверки прошло `INCREMENTAL_VERIFY_AFTER_TICKS` инкрементальных тактов;
  * `INCREMENTAL_VERIFY_PERIOD_DISTANCE` - ширина ушла от ширины последней сверки на `INCREMENTAL_VERIFY_AFTER_DISTANCE` её доли и больше;
  * `INCREMENTAL_NO_MODEL` / `INCREMENTAL_MODEL_NOT_OF_THE_DISPLAYED_SAMPLE` - модели нет либо она не про образец на экране, обслуживать нечем;
  * `INCREMENTAL_NO_ORIGINS` - у образца нет результатов, которые можно сдвигать;
  * `INCREMENTAL_FEW_DOMAINS_SERVED` - обслужено меньше `INCREMENTAL_MIN_SERVED_SHARE` доменов: полный такт стоит почти столько же и заодно сверяет;
  * `INCREMENTAL_SMALL_SELECTION` - в образце меньше `INCREMENTAL_MIN_DOMAINS` доменов: полный lean-такт на них стоит доли секунды и сам сверка, а переломы структуры там частые (замер: `rounded_wall_noise_top`, 5 доменов,
    домен 2 меняет тесселяцию уже на 0.25-1 % ширины от базы; на `building`, `cover.008` и `2` за 40 тактов по 0.25 % в обе стороны (до 10 %) не переломился ни один обслуженный домен);
  * `INCREMENTAL_SWITCHED_OFF` - переменная окружения `CFTUV_INCREMENTAL=0`.
ПРЕРВАННАЯ СВЕРКА. Полный lean-такт стоит секунду-две (`cover.008`), а планировщик отменяет любой счёт в полёте новым заказом, поэтому сверка, которую рука прервала, не должна запирать инкрементальные такты: `_begin`
записывает НАЧАЛО попытки (`mark_attempt`: `attempted_at`), и пока с неё не прошло `INCREMENTAL_RETRY_AFTER_TICKS` инкрементальных тактов, тик снова инкрементальный. Завершённая сверка (`next_state`) стирает попытку. Без этого после
`INCREMENTAL_VERIFY_AFTER_TICKS` тактов каждый следующий заказ был бы полным, отменялся бы следующим движением, и точных кадров при перетаскивании не осталось бы вовсе.

Пороги выбраны замером (`tools/blender_drag_ticks.py --grade incremental --shadow`: доля обслуженных доменов, структура которых разошлась с точной, и наибольшее отклонение обслуженных от точного прогона по числу тактов
и по пути ширины от сверки; числа - в строке `DECISIONS.md`). Каждый ответ называется: в строке статуса, в счётчиках журнала превью и в `IncrementalPlanV1.reason`.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

from .envelope_production_result import REFRESH_INCREMENTAL, REFRESH_LEAN

ENVIRONMENT_SWITCH = "CFTUV_INCREMENTAL"

#: Сколько инкрементальных тактов подряд допустимо без полной сверки, и на какую долю ширины сверки можно уйти. Значения выбраны замером (шапка модуля).
INCREMENTAL_VERIFY_AFTER_TICKS = 8
INCREMENTAL_VERIFY_AFTER_DISTANCE = 0.05
#: Доля доменов, которую сертификат обязан обслужить, чтобы такт был инкрементальным: иначе полный lean-такт стоит почти столько же и сверяет заодно.
INCREMENTAL_MIN_SERVED_SHARE = 0.5
#: Сколько инкрементальных тактов должно пройти после прерванной попытки сверки, прежде чем её повторят.
INCREMENTAL_RETRY_AFTER_TICKS = 4
#: Сколько доменов в образце, чтобы инкрементальный такт стоил своей непроверенной структуры: меньшее выделение считается полным lean-тактом (он дёшев и сверяет).
INCREMENTAL_MIN_DOMAINS = 16

REASON_NO_BASE = "INCREMENTAL_NO_VERIFIED_BASE"
REASON_PERIOD_TICKS = "INCREMENTAL_VERIFY_PERIOD_TICKS"
REASON_PERIOD_DISTANCE = "INCREMENTAL_VERIFY_PERIOD_DISTANCE"
REASON_NO_MODEL = "INCREMENTAL_NO_MODEL"
REASON_MODEL_FOREIGN = "INCREMENTAL_MODEL_NOT_OF_THE_DISPLAYED_SAMPLE"
REASON_NO_ORIGINS = "INCREMENTAL_NO_ORIGINS"
REASON_FEW_SERVED = "INCREMENTAL_FEW_DOMAINS_SERVED"
REASON_SWITCHED_OFF = "INCREMENTAL_SWITCHED_OFF"
REASON_SMALL_SELECTION = "INCREMENTAL_SMALL_SELECTION"
#: Причина, по которой домен посчитан заново, когда образец не знает про него (отказан в прошлом прогоне, вне меша).
HELD_NOT_IN_BASE = "INCREMENTAL_DOMAIN_NOT_IN_THE_DISPLAYED_SAMPLE"


@dataclass(frozen=True, slots=True)
class IncrementalStateV1:
    """Что сессия знает о сверке: инкрементальных тактов после последней полной сверки и ширина этой сверки."""

    ticks: int
    verified_alpha: float
    #: Сколько тактов было, когда последний раз НАЧАЛИ сверку, не дошедшую до конца (`None`: не начинали).
    attempted_at: int | None = None


@dataclass(frozen=True, slots=True)
class IncrementalPlanV1:
    """Решение одного тика: инкрементальный (`REFRESH_INCREMENTAL`) либо полный lean (`REFRESH_LEAN`, `reason` называет почему), обслуженные домены и причины остальных.

    `served` - номера патчей доменов, которые обслуживает сертификат (они идут в `run_production(skipped=...)`: тик их не считает и не сканирует), `held` - `{патч: имя}` доменов, которые тик считает заново.
    `composite` - образец тика сложен в numpy (`envelope_width_composite`: обслуженные двигает модель, посчитанные ложатся поверх), `fallback` - название причины, по которой составить его не вышло и образец собран
    из сдвинутых результатов всех доменов (`rebase_served` + `build_mesh_arrays`: тот же ответ, дороже). `seconds` - цена решения тика.
    """

    mode: str
    reason: str = ""
    served: frozenset = frozenset()
    held: dict = None
    domains: int = 0
    seconds: float = 0.0
    composite: bool = False
    fallback: str = ""
    #: LIVE_DRAG_STREAM_V1: срок сверки (`INCREMENTAL_VERIFY_PERIOD_*`), отложенный потоковым тактом, пока рука движется (пусто - не откладывали).
    deferred: str = ""

    @property
    def incremental(self) -> bool:
        return self.mode == REFRESH_INCREMENTAL


#: DRAG_VERIFY_BACKGROUND_V1: домен, который такт считает заново ради сверки структуры (а не потому, что сертификат его не обслуживает).
VERIFY_BACKGROUND = "DRAG_VERIFY_BACKGROUND_V1"


@dataclass(frozen=True, slots=True)
class StreamPolicyV1:
    """Потоковый такт живого точного меша (LIVE_DRAG_STREAM_V1).

    `defer` - срок сверки (`DEFERRABLE_REASONS`) не делает такт полным: сверка идёт фоном, партиями доменов внутри тактов (DRAG_VERIFY_BACKGROUND_V1).
    `verified` - `{патч: ширина последнего точного счёта домена}`; `batch` - сколько обслуживаемых доменов такт считает заново ради сверки (0 - ни одного).
    """

    defer: bool = True
    verified: object = None
    batch: int = 0


#: Поводы полного такта, которые потоковый такт не исполняет полным тактом (сверка идёт фоном): это сроки, а не невозможность инкрементального такта.
DEFERRABLE_REASONS = (REASON_PERIOD_TICKS, REASON_PERIOD_DISTANCE)


def verify_batch(candidates, verified, alpha: float, batch: int) -> list:
    """DRAG_VERIFY_BACKGROUND_V1: какие из обслуживаемых доменов `candidates` (`(патч, низ интервала, верх интервала либо None)`) такт считает заново ради сверки.

    Порядок риска: путь ширины от последнего точного счёта домена, делённый на запас до ближайшего конца его интервала (ни разу не считанный и стоящий у конца интервала - первыми);
    домен, посчитанный точно на этой ширине, не нужен. Ничьи - по номеру патча. Чистая функция значений.
    """

    if batch <= 0:
        return []
    alpha = float(alpha)
    ranked = []
    for patch, low, high in candidates:
        last = None if verified is None else verified.get(patch)
        if last is not None and last == alpha:
            continue
        headroom = min(alpha - low, (float("inf") if high is None else high) - alpha)
        drift = float("inf") if last is None else abs(alpha - last)
        risk = float("inf") if not headroom > 0.0 else drift / headroom
        ranked.append((-risk, patch))
    ranked.sort()
    return [patch for _risk, patch in ranked[:batch]]


def full_plan(reason: str, domains: int = 0) -> IncrementalPlanV1:
    """Полный lean-такт под названной причиной (он же сверка): ничего не обслуживается."""

    return IncrementalPlanV1(REFRESH_LEAN, reason, frozenset(), {}, domains)


def incremental_enabled() -> bool:
    return os.environ.get(ENVIRONMENT_SWITCH, "1").strip().lower() not in ("0", "off", "false", "no")


def verification_reason(state: IncrementalStateV1 | None, alpha: float) -> str:
    """Повод считать тик на ширине `alpha` ПОЛНЫМ (пусто - тик может быть инкрементальным): запись сверки, число тактов и путь ширины от последней сверки."""

    if not incremental_enabled():
        return REASON_SWITCHED_OFF
    if state is None:
        return REASON_NO_BASE
    due = ""
    if state.ticks >= INCREMENTAL_VERIFY_AFTER_TICKS:
        due = REASON_PERIOD_TICKS
    elif not abs(alpha - state.verified_alpha) < INCREMENTAL_VERIFY_AFTER_DISTANCE * state.verified_alpha:
        due = REASON_PERIOD_DISTANCE
    if due and state.attempted_at is not None and state.ticks - state.attempted_at < INCREMENTAL_RETRY_AFTER_TICKS:
        return ""  # сверку начинали и её прервали: тики идут дальше, повтор позже
    return due


def selection_reason(domains: int) -> str:
    """Повод считать тик полным, потому что выделение мало (`INCREMENTAL_SMALL_SELECTION`); пусто - достаточно доменов."""

    return REASON_SMALL_SELECTION if domains < INCREMENTAL_MIN_DOMAINS else ""


def mark_attempt(state: IncrementalStateV1) -> IncrementalStateV1:
    """Запись сверки с отметкой «сверку начали при этом числе тактов» (её пишет `_begin`, когда заказывает полный такт по сроку сверки)."""

    return IncrementalStateV1(state.ticks, state.verified_alpha, state.ticks)


def next_state(state: IncrementalStateV1 | None, plan: IncrementalPlanV1 | None, alpha: float) -> IncrementalStateV1:
    """Запись сверки после тика на ширине `alpha`, чей образец лёг на экран: инкрементальный такт добавляет такт, полный (и любой не-инкрементальный) начинает счёт заново от своей ширины."""

    if plan is not None and plan.incremental and state is not None:
        return IncrementalStateV1(state.ticks + 1, state.verified_alpha, state.attempted_at)
    return IncrementalStateV1(0, float(alpha))


__all__ = (
    "DEFERRABLE_REASONS",
    "ENVIRONMENT_SWITCH",
    "HELD_NOT_IN_BASE",
    "INCREMENTAL_MIN_DOMAINS",
    "INCREMENTAL_MIN_SERVED_SHARE",
    "INCREMENTAL_RETRY_AFTER_TICKS",
    "INCREMENTAL_VERIFY_AFTER_DISTANCE",
    "INCREMENTAL_VERIFY_AFTER_TICKS",
    "IncrementalPlanV1",
    "IncrementalStateV1",
    "REASON_FEW_SERVED",
    "REASON_MODEL_FOREIGN",
    "REASON_NO_BASE",
    "REASON_NO_MODEL",
    "REASON_NO_ORIGINS",
    "REASON_PERIOD_DISTANCE",
    "REASON_PERIOD_TICKS",
    "REASON_SMALL_SELECTION",
    "REASON_SWITCHED_OFF",
    "StreamPolicyV1",
    "VERIFY_BACKGROUND",
    "full_plan",
    "incremental_enabled",
    "mark_attempt",
    "next_state",
    "selection_reason",
    "verification_reason",
    "verify_batch",
)
