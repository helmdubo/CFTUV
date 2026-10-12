"""Инкрементальный точный такт перетаскивания ширины и политика его сверки (WIDTH_REFRESH_INCREMENTAL_V1, чистая, без Blender).

ЧТО ЭТО. Lean-такт (WIDTH_REFRESH_LEAN_V1) пересчитывает КАЖДЫЙ домен выделения, хотя за полпроцента ширины ИНТЕРВАЛ ядра кончается у единиц доменов (замер `S6_PARAMETRIC_DRAG_SPIKE`: `building` 1.4 из 122, `cover.008` 0.7
из 1046 на тик). Инкрементальный такт считает заново только домены, которых аффинный сертификат не обслуживает, а остальные берёт из прежних результатов, сдвинутых сертификатом на ширину тика
(`envelope_width_rebase`). Обслуженные домены не сканируются, не уходят воркерам и не ложатся в кэши; точные домены побитово равны lean-такту на той же ширине, обслуженные - сертификату с точностью округления.

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
  * `INCREMENTAL_SWITCHED_OFF` - переменная окружения `CFTUV_INCREMENTAL=0`.
Пороги выбраны замером (`tools/blender_drag_ticks.py --grade incremental --shadow`: доля обслуженных доменов, структура которых разошлась с точной, и наибольшее отклонение обслуженных от точного прогона по числу тактов
и по пути ширины от сверки; числа - в строке `DECISIONS.md`). Каждый ответ называется: в строке статуса, в счётчиках журнала превью и в `IncrementalPlanV1.reason`.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

from .envelope_production_result import REFRESH_INCREMENTAL, REFRESH_LEAN

ENVIRONMENT_SWITCH = "CFTUV_INCREMENTAL"

#: Сколько инкрементальных тактов подряд допустимо без полной сверки, и на какую долю ширины сверки можно уйти. Значения выбраны замером (шапка модуля).
INCREMENTAL_VERIFY_AFTER_TICKS = 4
INCREMENTAL_VERIFY_AFTER_DISTANCE = 0.02
#: Доля доменов, которую сертификат обязан обслужить, чтобы такт был инкрементальным: иначе полный lean-такт стоит почти столько же и сверяет заодно.
INCREMENTAL_MIN_SERVED_SHARE = 0.5

REASON_NO_BASE = "INCREMENTAL_NO_VERIFIED_BASE"
REASON_PERIOD_TICKS = "INCREMENTAL_VERIFY_PERIOD_TICKS"
REASON_PERIOD_DISTANCE = "INCREMENTAL_VERIFY_PERIOD_DISTANCE"
REASON_NO_MODEL = "INCREMENTAL_NO_MODEL"
REASON_MODEL_FOREIGN = "INCREMENTAL_MODEL_NOT_OF_THE_DISPLAYED_SAMPLE"
REASON_NO_ORIGINS = "INCREMENTAL_NO_ORIGINS"
REASON_FEW_SERVED = "INCREMENTAL_FEW_DOMAINS_SERVED"
REASON_SWITCHED_OFF = "INCREMENTAL_SWITCHED_OFF"
#: Причина, по которой домен посчитан заново, когда образец не знает про него (отказан в прошлом прогоне, вне меша).
HELD_NOT_IN_BASE = "INCREMENTAL_DOMAIN_NOT_IN_THE_DISPLAYED_SAMPLE"


@dataclass(frozen=True, slots=True)
class IncrementalStateV1:
    """Что сессия знает о сверке: инкрементальных тактов после последней полной сверки и ширина этой сверки."""

    ticks: int
    verified_alpha: float


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

    @property
    def incremental(self) -> bool:
        return self.mode == REFRESH_INCREMENTAL


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
    if state.ticks >= INCREMENTAL_VERIFY_AFTER_TICKS:
        return REASON_PERIOD_TICKS
    if not abs(alpha - state.verified_alpha) < INCREMENTAL_VERIFY_AFTER_DISTANCE * state.verified_alpha:
        return REASON_PERIOD_DISTANCE
    return ""


def next_state(state: IncrementalStateV1 | None, plan: IncrementalPlanV1 | None, alpha: float) -> IncrementalStateV1:
    """Запись сверки после тика на ширине `alpha`, чей образец лёг на экран: инкрементальный такт добавляет такт, полный (и любой не-инкрементальный) начинает счёт заново от своей ширины."""

    if plan is not None and plan.incremental and state is not None:
        return IncrementalStateV1(state.ticks + 1, state.verified_alpha)
    return IncrementalStateV1(0, float(alpha))


__all__ = (
    "ENVIRONMENT_SWITCH",
    "HELD_NOT_IN_BASE",
    "INCREMENTAL_MIN_SERVED_SHARE",
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
    "REASON_SWITCHED_OFF",
    "full_plan",
    "incremental_enabled",
    "next_state",
    "verification_reason",
)
