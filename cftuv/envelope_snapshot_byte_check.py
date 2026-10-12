"""Круг кодека над снапшотом домена идёт на выборке, а не на каждом домене кнопки (SNAPSHOT_BYTE_CHECK_SAMPLED_V1), а первое нажатие проверяется целиком ФОНОМ после кнопки (SNAPSHOT_BYTE_CHECK_FIRST_PRESS_IN_BACKGROUND_V1).

`validate_exported_snapshot` делает две вещи над выгруженным снапшотом: оба валидатора ядра над ОБЪЕКТОМ и проверку БАЙТОВ - `dumps -> loads -> dumps`
плюс те же валидаторы над ДЕКОДИРОВАННЫМ объектом (канонические байты - неподвижная точка кодека, и «объект валиден, а выпущенные байты ядро
отвергает»: DECISIONS 2026-10-08, SNAPSHOT-VALIDATION-READ-PLAN). Проверка байтов - 9-10 с процессорного времени воркеров на `cover.008`
(~1.2 с стены, ~7 % кнопки). Проверка объекта остаётся на КАЖДОМ домене; здесь выбирается только проверка байтов.

ПОЛИТИКА - переключатель `CFTUV_SNAPSHOT_BYTE_CHECK` (`all` | `sample` | `off`), который читает `run_production` на каждом нажатии:

* `all` - проверка байтов на каждом домене. Её ставят тесты (`tests/conftest.py`), CI (workflows) и полевые инструменты (`tools/`): ворота
  приёмки не выбирают, что проверять. Неразборчивое значение - тоже `all` (строгое), а не молчаливое умолчание: счётчик `ENV_INVALID` и строка консоли.
* `sample` - умолчание КНОПКИ продукта. Проверяются (1) домены ВЫБОРКИ: детерминированно, дайджест идентичности домена по модулю `SAMPLE_MODULUS`
  равен нулю (тот же домен - то же решение), и (2) ВСЕ домены первого холодного нажатия после смены идентичности кода в этом сеансе Blender
  (`FIRST_PRESS`: новый код проверяется полностью один раз; идентичность - `execution_identity`: отпечатки ядра и хоста плюс бэкенды стадий с
  `native_build_id`). Идентичность помечается проверенной по концу нажатия, где `FIRST_PRESS` реально отработал; оборванное нажатие её не помечает.
* `off` - проверки байтов нет вовсе (для замеров и отладки; не для ворот).

ПЕРВОЕ НАЖАТИЕ ФОНОМ (SNAPSHOT_BYTE_CHECK_FIRST_PRESS_IN_BACKGROUND_V1; решение владельца: «Фоном после кнопки»). Домен, выгруженный ВОРКЕРОМ пула на первом нажатии, кнопка проверяет только на выборке; остальной круг
кодека (1.37 с стены `cover.008`) идёт ПОСЛЕ кнопки в тех же воркерах, пока они простаивают. Воркер, выгрузив снапшот, оставляет его у себя под именем нажатия (`DEFERRED`, счётчик `PRODUCTION_SNAPSHOT_BYTE_CHECK_DEFERRED`
под доменом), а фоновая задача родителя (`envelope_byte_check_background`) раз за разом шлёт воркерам короткий шаг (`ByteCheckStepV1`, ~40 мс работы): воркер проверяет сколько успел и отвечает,
сколько осталось и что нашёл. Нажатие или тик, пришедшие следом, не ждут дольше одного шага (`DomainPool.run_idle` уступает любому прогону). Проверка та же функция, что на нажатии
(`envelope_angle_certificate.check_snapshot_bytes`), поэтому найденное - тот же `SnapshotExportRejected` с тем же именем исхода хоста, текстом и доменом. Найденный домен помечен у сессии, и СЛЕДУЮЩЕЕ действие
(`_scan`) отказывает его этим именем, как отказала бы проверка на нажатии; статус и консоль называют исход. Идентичность кода помечается проверенной, когда проверены ВСЕ отложенные домены; прогон, который не довёл
дело (воркер умер, пул закрыт, сессия сброшена), называется `..._INCOMPLETE`, идентичность остаётся непроверенной, и следующее холодное нажатие проверит снова. Пока задача идёт, идентичность «в работе»: нажатия между ними - выборка.
Вне пула (последовательный путь, пул не стартовал, домен считал родитель) отложить некому, и первое нажатие проверяет, как раньше.

ОБЪЯВЛЕННОЕ ИЗМЕНЕНИЕ ПОВЕДЕНИЯ. Домен вне выборки, чьи байты не прошли бы проверку, теперь идёт дальше, а раньше был отказан. Это принятый риск владельца
(«в CI и поле - полная, в кнопке - выборка»). Отказ ВЫБРАННОГО домена не изменился: тот же `SnapshotExportRejected` и то же имя исхода хоста. Домен первого нажатия, чьи байты не прошли бы проверку,
остаётся на экране до следующего действия (его отказывает фон), а раньше отказывался на самой кнопке: так выбрано владельцем.

КАЖДЫЙ ПРОПУСК СЧИТАЕТСЯ. Решение по домену - счётчик профиля под доменом: `PRODUCTION_SNAPSHOT_BYTE_CHECK_{ALL, FIRST_PRESS, DEFERRED, SAMPLED, SKIPPED, OFF}`;
в конце нажатия `finish_press` складывает их в итоги без домена (их читают сводки и профиль бэкенда). Пропущенные - `SKIPPED` (вне выборки) и `OFF`; `DEFERRED` - не пропуск: круг кодека идёт позже.

ЧИСТЫЙ ВЕРДИКТ ВОРКЕРА ОТ ЭТОГО НЕ ЗАВИСИТ. `SnapshotCleanV1` (`envelope_snapshot_check`) - результат `validate_analysis_snapshot` над САМИМ объектом снапшота под допуском
запроса; декодированный объект и проверка байтов в него не входят ни до, ни после этой правки, поэтому родитель, принявший запись воркера, не принимает вердикт,
выведенный из пропущенной проверки (закреплено тестом).

План едет значением: родитель кладёт его в `EnvelopeTopologyExportV1.snapshot_byte_check` (родительский путь читает его оттуда же, откуда воркер, у которого он
приходит в `ColdProductionInputV1`). Вне продуктового пути плана нет, и проверка байтов идёт всегда.
"""

from __future__ import annotations

import hashlib
import os
import threading
import time
import uuid
from dataclasses import dataclass, replace

BYTE_CHECK_ENVIRONMENT = "CFTUV_SNAPSHOT_BYTE_CHECK"
MODE_ALL = "all"
MODE_SAMPLE = "sample"
MODE_OFF = "off"
MODES = (MODE_ALL, MODE_SAMPLE, MODE_OFF)
#: Продуктовое умолчание кнопки (переменная не задана).
DEFAULT_PRODUCT_MODE = MODE_SAMPLE
#: Доля выборки: 1 из `SAMPLE_MODULUS` доменов.
SAMPLE_MODULUS = 16

BYTE_CHECK_ALL = "PRODUCTION_SNAPSHOT_BYTE_CHECK_ALL"
BYTE_CHECK_FIRST_PRESS = "PRODUCTION_SNAPSHOT_BYTE_CHECK_FIRST_PRESS"
BYTE_CHECK_DEFERRED = "PRODUCTION_SNAPSHOT_BYTE_CHECK_DEFERRED"
BYTE_CHECK_SAMPLED = "PRODUCTION_SNAPSHOT_BYTE_CHECK_SAMPLED"
BYTE_CHECK_SKIPPED = "PRODUCTION_SNAPSHOT_BYTE_CHECK_SKIPPED"
BYTE_CHECK_OFF = "PRODUCTION_SNAPSHOT_BYTE_CHECK_OFF"
BYTE_CHECK_ENV_INVALID = "PRODUCTION_SNAPSHOT_BYTE_CHECK_ENV_INVALID"
#: Решение по домену -> проверка байтов идёт СЕЙЧАС (`True`) либо нет (`False`): `DEFERRED` идёт фоном после кнопки, `SKIPPED` и `OFF` не идут вовсе.
DECISIONS = {
    BYTE_CHECK_ALL: True,
    BYTE_CHECK_FIRST_PRESS: True,
    BYTE_CHECK_DEFERRED: False,
    BYTE_CHECK_SAMPLED: True,
    BYTE_CHECK_SKIPPED: False,
    BYTE_CHECK_OFF: False,
}
BYTE_CHECK_COUNTERS = (*DECISIONS, BYTE_CHECK_ENV_INVALID)
#: Исход, которым фон называет домен, чью проверку байтов сорвало исключение (не `SnapshotExportRejected`): имя из перечня исходов хоста.
BYTE_CHECK_RAISED_OUTCOME = "ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED"
#: Сколько секунд работы воркер тратит на один шаг фоновой проверки (после него он уступает трубу): предел ожидания следующего действия.
STEP_BUDGET_SECONDS = 0.04

#: Идентичности исполнения, на которых в этом процессе уже прошло нажатие с `FIRST_PRESS`: «новый код проверен полностью один раз».
_CHECKED_IDENTITIES: set = set()
#: Идентичности, чьё первое нажатие отложило проверку и фоновая задача ещё не кончилась: `{идентичность: имя нажатия}`.
_PENDING_IDENTITIES: dict = {}


@dataclass(frozen=True, slots=True)
class SnapshotByteCheckPlanV1:
    """План проверки байтов на одно нажатие: режим, «первое нажатие этой идентичности кода», идентичность и неразобранный текст переменной.

    `defer` - первое нажатие идёт через пул, который умеет фоновые шаги: домен, выгруженный ВОРКЕРОМ (`worker` - копия плана, которую читает воркер, `for_worker`), откладывает круг кодека на `token`
    (имя нажатия); домен, выгруженный родителем, проверяется на месте, как раньше.
    """

    mode: str = MODE_ALL
    first_press: bool = False
    identity: tuple | None = None
    invalid: str = ""
    defer: bool = False
    token: str = ""
    worker: bool = False

    def decision(self, domain_id: str | None) -> str:
        """Имя решения по домену (ключ `DECISIONS`); снапшот не одного домена (`domain_id=None`) выборке не подлежит и проверяется целиком."""

        if self.mode == MODE_ALL:
            return BYTE_CHECK_ALL
        if self.mode == MODE_OFF:
            return BYTE_CHECK_OFF
        if self.first_press:
            return BYTE_CHECK_DEFERRED if (self.defer and self.worker and domain_id is not None) else BYTE_CHECK_FIRST_PRESS
        if domain_id is None:
            return BYTE_CHECK_ALL
        return BYTE_CHECK_SAMPLED if in_sample(domain_id) else BYTE_CHECK_SKIPPED

    def for_worker(self) -> "SnapshotByteCheckPlanV1":
        """Копия плана для воркера пула: только в ней отложенное решение действует."""

        return replace(self, worker=True) if self.defer and not self.worker else self


def in_sample(domain_id: str) -> bool:
    """Домен в выборке: первые восемь байт SHA-256 его идентичности как число по модулю `SAMPLE_MODULUS` равны нулю (решение зависит только от идентичности)."""

    return int.from_bytes(hashlib.sha256(domain_id.encode("utf-8")).digest()[:8], "big") % SAMPLE_MODULUS == 0


def mode_of_environment(environ=None) -> tuple[str, str]:
    """`(режим, неразобранный текст)`: переменной нет или она пуста - `DEFAULT_PRODUCT_MODE`; значение вне `MODES` - `all` и его текст."""

    raw = (os.environ if environ is None else environ).get(BYTE_CHECK_ENVIRONMENT)
    if raw is None or not raw.strip():
        return DEFAULT_PRODUCT_MODE, ""
    mode = raw.strip().lower()
    return (mode, "") if mode in MODES else (MODE_ALL, raw)


def execution_identity_of(kernel_backend, skeleton_backend=None, embedding_backend=None) -> tuple | None:
    """Идентичность исполнения нажатия (`execution_identity`) либо `None`, если её не определить: тогда нажатие считается первым и проверяется целиком."""

    try:
        from .envelope_content_key import ContentKeyUnsupported, execution_identity

        return execution_identity(kernel_backend, skeleton_backend, embedding_backend)
    except (ImportError, ContentKeyUnsupported):
        return None


def resolve_plan(kernel_backend, skeleton_backend=None, embedding_backend=None, *, environ=None, defer: bool = False) -> SnapshotByteCheckPlanV1:
    """План проверки байтов этого нажатия: режим из переменной; в `sample` - первое ли это нажатие этой идентичности кода в процессе.

    `defer=True` - нажатие идёт через пул с фоновыми шагами (`DomainPool.run_idle`): первое нажатие откладывает круг кодека воркеров (см. модуль). Идентичность, чья проверка уже идёт фоном, первой не считается.
    """

    mode, invalid = mode_of_environment(environ)
    if invalid:
        print(
            f"[CFTUV][SnapshotByteCheck] {BYTE_CHECK_ENV_INVALID}: {BYTE_CHECK_ENVIRONMENT}={invalid!r} is not one of {'|'.join(MODES)}; the full check runs",
            flush=True,
        )
    if mode != MODE_SAMPLE:
        return SnapshotByteCheckPlanV1(mode, False, None, invalid)
    identity = execution_identity_of(kernel_backend, skeleton_backend, embedding_backend)
    first = identity is None or (identity not in _CHECKED_IDENTITIES and identity not in _PENDING_IDENTITIES)
    deferring = bool(defer) and first and identity is not None
    return SnapshotByteCheckPlanV1(mode, first, identity, invalid, deferring, uuid.uuid4().hex if deferring else "")


def byte_check_decision(plan: SnapshotByteCheckPlanV1 | None, domain_id: str | None, profile) -> bool:
    """Идёт ли проверка байтов этого домена; решение записано счётчиком под доменом. Без плана (вне продуктового пути) - да, без записи."""

    if plan is None:
        return True
    decision = plan.decision(domain_id)
    if profile is not None:
        profile.add_counter(decision, 1, domain_id)
    return DECISIONS[decision]


def validate_under_policy(kernel, snapshot, plan: SnapshotByteCheckPlanV1 | None, domain_id: str | None, profile, law=None) -> None:
    """Приёмка выгруженного снапшота под политикой нажатия: проверка объекта всегда, круг кодека - сейчас, фоном (`DEFERRED`: снапшот остаётся у воркера до шага) либо не идёт.

    Отказ - `SnapshotExportRejected`, как у `validate_exported_snapshot`. `law` - закон масштаба решётки экспорта: повтор лотереи привязки выгружает тот же домен ВТОРОЙ раз под другим законом.
    """

    from .envelope_angle_certificate import validate_exported_snapshot

    if plan is None:
        validate_exported_snapshot(kernel, snapshot)
        return
    decision = plan.decision(domain_id)
    if profile is not None:
        profile.add_counter(decision, 1, domain_id)
    validate_exported_snapshot(kernel, snapshot, byte_check=DECISIONS[decision])
    if decision == BYTE_CHECK_DEFERRED:
        defer_check(plan, domain_id, law, snapshot)


def finish_press(profile, plan: SnapshotByteCheckPlanV1 | None) -> None:
    """Конец нажатия: счётчики решений - в итоги без домена; идентичность, на которой `FIRST_PRESS` реально отработал, помечена проверенной, а отложенная - «в работе» до конца фоновой задачи."""

    if plan is None:
        return
    totals: dict = {}
    for item in profile.snapshot().counters:
        if item.name in DECISIONS and item.patch_domain_id is not None:
            totals[item.name] = totals.get(item.name, 0) + item.value
    for name in sorted(totals):
        profile.set_counter(name, totals[name])
    if plan.invalid:
        profile.set_counter(BYTE_CHECK_ENV_INVALID, 1)
    if plan.first_press and plan.identity is not None:
        if totals.get(BYTE_CHECK_DEFERRED):
            _PENDING_IDENTITIES[plan.identity] = plan.token
        elif totals.get(BYTE_CHECK_FIRST_PRESS):
            _CHECKED_IDENTITIES.add(plan.identity)


def settle_identity(identity: tuple | None, token: str, *, verified: bool) -> None:
    """Фоновая задача нажатия `token` кончилась: идентичность больше не «в работе»; проверена, если проверены ВСЕ отложенные домены, иначе следующее холодное нажатие проверит снова."""

    if identity is None:
        return
    if _PENDING_IDENTITIES.get(identity) == token:
        del _PENDING_IDENTITIES[identity]
    if verified:
        _CHECKED_IDENTITIES.add(identity)


def forget_checked_identities() -> None:
    """Сеанс начинается заново (тесты и полевые инструменты): следующее нажатие каждой идентичности снова первое."""

    _CHECKED_IDENTITIES.clear()
    _PENDING_IDENTITIES.clear()


def mark_identity_checked(identity: tuple) -> None:
    """Идентичность проверена полностью (полевой инструмент, имитирующий «второе нажатие сеанса» без первого)."""

    _CHECKED_IDENTITIES.add(identity)


# --------------------------------------------------------------------------
# Воркер: снапшоты, ожидающие круга кодека, и шаг фоновой проверки
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class ByteCheckStepV1:
    """Сообщение родителя воркеру: проверь отложенные снапшоты нажатия `token` не дольше `budget_seconds` (но хотя бы один) и ответь."""

    token: str
    budget_seconds: float = STEP_BUDGET_SECONDS


@dataclass(frozen=True, slots=True)
class ByteCheckViolationV1:
    """Отказ круга кодека, найденный фоном: домен, закон экспорта и то, что дал бы `SnapshotExportRejected` на нажатии (имя исхода хоста, текст, домен отказа)."""

    domain_id: str
    law: str | None
    outcome_name: str
    message: str
    patch_domain_id: str | None


@dataclass(frozen=True, slots=True)
class ByteCheckReplyV1:
    """Ответ воркера на шаг: сколько снапшотов проверено, сколько ещё ждёт у НЕГО и что найдено."""

    token: str
    checked: int
    remaining: int
    violations: tuple = ()


_DEFERRED: dict = {}
_DEFERRED_LOCK = threading.Lock()


def defer_check(plan: SnapshotByteCheckPlanV1, domain_id: str, law, snapshot) -> None:
    """Воркер оставляет снапшот у себя до шага фоновой проверки (`(имя нажатия, домен, закон)`); отложенное прежних нажатий забыто: задача у родителя одна."""

    with _DEFERRED_LOCK:
        for key in [key for key in _DEFERRED if key[0] != plan.token]:
            del _DEFERRED[key]
        _DEFERRED[(plan.token, str(domain_id), law)] = snapshot


def deferred_count(token: str | None = None) -> int:
    """Сколько снапшотов этого процесса ждут шага (нажатия `token`, либо всех)."""

    with _DEFERRED_LOCK:
        return sum(1 for key in _DEFERRED if token is None or key[0] == token)


def _verdict(kernel, key: tuple, snapshot) -> tuple:
    from .envelope_angle_certificate import SnapshotExportRejected, check_snapshot_bytes

    try:
        check_snapshot_bytes(kernel, snapshot)
    except SnapshotExportRejected as error:
        return (ByteCheckViolationV1(key[1], key[2], error.outcome_name, str(error), error.patch_domain_id),)
    except Exception as error:  # noqa: BLE001 - исключение проверки - названный отказ домена, а не молчаливая пропажа
        return (ByteCheckViolationV1(key[1], key[2], BYTE_CHECK_RAISED_OUTCOME, f"snapshot byte check raised {type(error).__name__}: {error}", key[1]),)
    return ()


def serve_byte_check(step: ByteCheckStepV1) -> ByteCheckReplyV1:
    """Шаг воркера: проверяет отложенные снапшоты нажатия `step.token`, пока не выйдет бюджет шага (одну проверку доводит до конца), и называет остаток и найденное."""

    from .envelope_request_export import _load_kernel

    kernel, _ = _load_kernel()
    started = time.perf_counter()
    checked = 0
    violations: list = []
    with _DEFERRED_LOCK:
        for key in [key for key in _DEFERRED if key[0] != step.token]:
            del _DEFERRED[key]
        keys = list(_DEFERRED)
    for key in keys:
        if checked and time.perf_counter() - started >= step.budget_seconds:
            break
        with _DEFERRED_LOCK:
            snapshot = _DEFERRED.pop(key, None)
        if snapshot is None:
            continue
        checked += 1
        violations.extend(_verdict(kernel, key, snapshot))
        snapshot = None
    return ByteCheckReplyV1(step.token, checked, deferred_count(step.token), tuple(violations))


__all__ = (
    "BYTE_CHECK_ALL",
    "BYTE_CHECK_COUNTERS",
    "BYTE_CHECK_DEFERRED",
    "BYTE_CHECK_ENVIRONMENT",
    "BYTE_CHECK_ENV_INVALID",
    "BYTE_CHECK_FIRST_PRESS",
    "BYTE_CHECK_OFF",
    "BYTE_CHECK_RAISED_OUTCOME",
    "BYTE_CHECK_SAMPLED",
    "BYTE_CHECK_SKIPPED",
    "ByteCheckReplyV1",
    "ByteCheckStepV1",
    "ByteCheckViolationV1",
    "DECISIONS",
    "DEFAULT_PRODUCT_MODE",
    "MODES",
    "MODE_ALL",
    "MODE_OFF",
    "MODE_SAMPLE",
    "SAMPLE_MODULUS",
    "STEP_BUDGET_SECONDS",
    "SnapshotByteCheckPlanV1",
    "byte_check_decision",
    "defer_check",
    "deferred_count",
    "execution_identity_of",
    "finish_press",
    "forget_checked_identities",
    "in_sample",
    "mark_identity_checked",
    "mode_of_environment",
    "resolve_plan",
    "serve_byte_check",
    "settle_identity",
    "validate_under_policy",
)
