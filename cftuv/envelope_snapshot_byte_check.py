"""Круг кодека над снапшотом домена идёт на выборке, а не на каждом домене кнопки (SNAPSHOT_BYTE_CHECK_SAMPLED_V1).

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

ОБЪЯВЛЕННОЕ ИЗМЕНЕНИЕ ПОВЕДЕНИЯ. Домен вне выборки, чьи байты не прошли бы проверку, теперь идёт дальше, а раньше был отказан. Это принятый риск владельца
(«в CI и поле - полная, в кнопке - выборка»). Отказ ВЫБРАННОГО домена не изменился: тот же `SnapshotExportRejected` и то же имя исхода хоста.

КАЖДЫЙ ПРОПУСК СЧИТАЕТСЯ. Решение по домену - счётчик профиля под доменом: `PRODUCTION_SNAPSHOT_BYTE_CHECK_{ALL, FIRST_PRESS, SAMPLED, SKIPPED, OFF}`;
в конце нажатия `finish_press` складывает их в итоги без домена (их читают сводки и профиль бэкенда). Пропущенные - `SKIPPED` (вне выборки) и `OFF`.

ЧИСТЫЙ ВЕРДИКТ ВОРКЕРА ОТ ЭТОГО НЕ ЗАВИСИТ. `SnapshotCleanV1` (`envelope_snapshot_check`) - результат `validate_analysis_snapshot` над САМИМ объектом снапшота под допуском
запроса; декодированный объект и проверка байтов в него не входят ни до, ни после этой правки, поэтому родитель, принявший запись воркера, не принимает вердикт,
выведенный из пропущенной проверки (закреплено тестом).

План едет значением: родитель кладёт его в `EnvelopeTopologyExportV1.snapshot_byte_check` (родительский путь читает его оттуда же, откуда воркер, у которого он
приходит в `ColdProductionInputV1`). Вне продуктового пути плана нет, и проверка байтов идёт всегда.
"""

from __future__ import annotations

import hashlib
import os
from dataclasses import dataclass

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
BYTE_CHECK_SAMPLED = "PRODUCTION_SNAPSHOT_BYTE_CHECK_SAMPLED"
BYTE_CHECK_SKIPPED = "PRODUCTION_SNAPSHOT_BYTE_CHECK_SKIPPED"
BYTE_CHECK_OFF = "PRODUCTION_SNAPSHOT_BYTE_CHECK_OFF"
BYTE_CHECK_ENV_INVALID = "PRODUCTION_SNAPSHOT_BYTE_CHECK_ENV_INVALID"
#: Решение по домену -> проверка байтов идёт (`True`) либо пропущена (`False`).
DECISIONS = {
    BYTE_CHECK_ALL: True,
    BYTE_CHECK_FIRST_PRESS: True,
    BYTE_CHECK_SAMPLED: True,
    BYTE_CHECK_SKIPPED: False,
    BYTE_CHECK_OFF: False,
}
BYTE_CHECK_COUNTERS = (*DECISIONS, BYTE_CHECK_ENV_INVALID)

#: Идентичности исполнения, на которых в этом процессе уже прошло нажатие с `FIRST_PRESS`: «новый код проверен полностью один раз».
_CHECKED_IDENTITIES: set = set()


@dataclass(frozen=True, slots=True)
class SnapshotByteCheckPlanV1:
    """План проверки байтов на одно нажатие: режим, «первое нажатие этой идентичности кода», идентичность и неразобранный текст переменной."""

    mode: str = MODE_ALL
    first_press: bool = False
    identity: tuple | None = None
    invalid: str = ""

    def decision(self, domain_id: str | None) -> str:
        """Имя решения по домену (ключ `DECISIONS`); снапшот не одного домена (`domain_id=None`) выборке не подлежит и проверяется целиком."""

        if self.mode == MODE_ALL:
            return BYTE_CHECK_ALL
        if self.mode == MODE_OFF:
            return BYTE_CHECK_OFF
        if self.first_press:
            return BYTE_CHECK_FIRST_PRESS
        if domain_id is None:
            return BYTE_CHECK_ALL
        return BYTE_CHECK_SAMPLED if in_sample(domain_id) else BYTE_CHECK_SKIPPED


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


def resolve_plan(kernel_backend, skeleton_backend=None, embedding_backend=None, *, environ=None) -> SnapshotByteCheckPlanV1:
    """План проверки байтов этого нажатия: режим из переменной; в `sample` - первое ли это нажатие этой идентичности кода в процессе."""

    mode, invalid = mode_of_environment(environ)
    if invalid:
        print(
            f"[CFTUV][SnapshotByteCheck] {BYTE_CHECK_ENV_INVALID}: {BYTE_CHECK_ENVIRONMENT}={invalid!r} is not one of {'|'.join(MODES)}; the full check runs",
            flush=True,
        )
    if mode != MODE_SAMPLE:
        return SnapshotByteCheckPlanV1(mode, False, None, invalid)
    identity = execution_identity_of(kernel_backend, skeleton_backend, embedding_backend)
    return SnapshotByteCheckPlanV1(mode, identity is None or identity not in _CHECKED_IDENTITIES, identity, invalid)


def byte_check_decision(plan: SnapshotByteCheckPlanV1 | None, domain_id: str | None, profile) -> bool:
    """Идёт ли проверка байтов этого домена; решение записано счётчиком под доменом. Без плана (вне продуктового пути) - да, без записи."""

    if plan is None:
        return True
    decision = plan.decision(domain_id)
    if profile is not None:
        profile.add_counter(decision, 1, domain_id)
    return DECISIONS[decision]


def finish_press(profile, plan: SnapshotByteCheckPlanV1 | None) -> None:
    """Конец нажатия: счётчики решений - в итоги без домена; идентичность, на которой `FIRST_PRESS` реально отработал, помечена проверенной."""

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
    if plan.first_press and plan.identity is not None and totals.get(BYTE_CHECK_FIRST_PRESS):
        _CHECKED_IDENTITIES.add(plan.identity)


def forget_checked_identities() -> None:
    """Сеанс начинается заново (тесты и полевые инструменты): следующее нажатие каждой идентичности снова первое."""

    _CHECKED_IDENTITIES.clear()


def mark_identity_checked(identity: tuple) -> None:
    """Идентичность проверена полностью (полевой инструмент, имитирующий «второе нажатие сеанса» без первого)."""

    _CHECKED_IDENTITIES.add(identity)


__all__ = (
    "BYTE_CHECK_ALL",
    "BYTE_CHECK_COUNTERS",
    "BYTE_CHECK_ENVIRONMENT",
    "BYTE_CHECK_ENV_INVALID",
    "BYTE_CHECK_FIRST_PRESS",
    "BYTE_CHECK_OFF",
    "BYTE_CHECK_SAMPLED",
    "BYTE_CHECK_SKIPPED",
    "DECISIONS",
    "DEFAULT_PRODUCT_MODE",
    "MODES",
    "MODE_ALL",
    "MODE_OFF",
    "MODE_SAMPLE",
    "SAMPLE_MODULUS",
    "SnapshotByteCheckPlanV1",
    "byte_check_decision",
    "execution_identity_of",
    "finish_press",
    "forget_checked_identities",
    "in_sample",
    "mark_identity_checked",
    "mode_of_environment",
    "resolve_plan",
)
