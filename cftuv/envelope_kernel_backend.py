"""Бэкенд ядра в хосте: настройка «Kernel backend», запись бэкенда на каждом домене и строка журнала.

Ядро умеет считать две горячие операции нативно (`cftuv_envelope.backend`: покрытие и резка), и выбор бэкенда — НЕ политика
запроса: ответ побитово один. Поэтому здесь нет ни слова о геометрии. Здесь три вещи:

* НАСТРОЙКА. `kernel_backend` (`PYTHON` | `NATIVE`) живёт в настройках сцены рядом с остальными настройками декали
  (`HOTSPOTUV_DecalMeshSettings`), едет в прогон (`run_production`), в задачу пула (`DomainTaskV1.backend`) и в запись живой
  ширины. Умолчание — `PYTHON`: без заказа ничего не меняется, ни ключи кэшей, ни строки консоли.
* ЗАПИСЬ ДОМЕНА. `with_kernel_backend` оборачивает вычисление домена в блок `use_backend` и кладёт в результат `BackendRecordV1`
  (какой бэкенд посчитал на самом деле и по какой названной причине откат на Python). Результат с записью и без неё равен по
  ответу: запись — метка запуска, как `placement`. Смена бэкенда в процессе сбрасывает память стадии резки ядра (`clip_memo`):
  её ключ бэкенд не несёт, а ключи кэшей хоста несут (`backend_identity` в `envelope_content_key` и в ключах прогона).
  Первый заказ `NATIVE` в процессе ставит диспетчер резки (`backend.install_dispatch`: подмена имени `clip.clip_geometry`): вызов резки лежит в
  закреплённом нативным портом файле (`clip.py`), и подмена имени оставляет закрепления верными. Покрытие подключено в самом ядре.
  Воркер пула ставит её сам на первом домене с заказом `NATIVE` (состояние процесса).
* СТРОКА ЖУРНАЛА. `backend_console_lines`: `[CFTUV][Production] BACKEND native 120 / python 2 (NATIVE_PORT_STALE: patch 7, 9)`.
  Печатается только когда заказан нативный бэкенд; домен из кэша сессии в счёт не идёт (в этом прогоне он не считался).

Модуль не импортирует `bpy` и не импортирует ядро при загрузке: пакет остаётся импортируемым без него.
"""

from __future__ import annotations

import functools
from contextlib import contextmanager
from dataclasses import dataclass

KERNEL_BACKEND_PYTHON = "PYTHON"
KERNEL_BACKEND_NATIVE = "NATIVE"
DEFAULT_KERNEL_BACKEND = KERNEL_BACKEND_PYTHON
#: Имя свойства в `HOTSPOTUV_DecalMeshSettings`.
SETTING_NAME = "kernel_backend"
KERNEL_BACKEND_ITEMS = (
    (
        KERNEL_BACKEND_PYTHON,
        "Python",
        "Reference kernel in Python: the answer every other backend is checked against",
    ),
    (
        KERNEL_BACKEND_NATIVE,
        "Native (Rust)",
        "Coverage and clip in the native kernel (cftuv_native). The answer is bitwise the same; a domain the native "
        "kernel cannot compute is computed in Python and named in the console",
    ),
)
#: Сколько номеров патчей называет строка журнала на исход.
PATCHES_SHOWN = 12

_LAST_BACKEND = [DEFAULT_KERNEL_BACKEND]


def normalize_kernel_backend(value) -> str:
    """`PYTHON` либо `NATIVE`; неизвестное имя — `ValueError` (тот же закон, что у ядра, без его импорта)."""

    text = str(value).strip().upper()
    if text not in (KERNEL_BACKEND_PYTHON, KERNEL_BACKEND_NATIVE):
        raise ValueError(f"unknown kernel backend {value!r}: expected PYTHON or NATIVE")
    return text


def kernel_backend_of(mesh_settings) -> str:
    """Заказанный бэкенд из настроек декали сцены; без свойства (вне Blender, старая сцена) — `PYTHON`."""

    return normalize_kernel_backend(getattr(mesh_settings, SETTING_NAME, DEFAULT_KERNEL_BACKEND) or DEFAULT_KERNEL_BACKEND)


def backend_identity_of(kernel_backend) -> str:
    """Идентичность бэкенда для ключей кэшей: `PYTHON` либо `NATIVE:<версия колеса>`; ядро не импортируется — `PYTHON`/`NATIVE`."""

    name = normalize_kernel_backend(kernel_backend)
    try:
        from cftuv_envelope.backend import backend_identity
    except ImportError:
        return name
    return backend_identity(name)


@contextmanager
def entered_backend(kernel_backend):
    """`use_backend` ядра; `NATIVE` сперва ставит диспетчер резки, а смена бэкенда в ЭТОМ процессе сбрасывает память стадии резки.

    Отдаёт журнал домена (`NATIVE`) либо `None` (`PYTHON`). Память резки сбрасывается потому, что её ключ бэкенд не несёт.
    """

    name = normalize_kernel_backend(kernel_backend)
    from cftuv_envelope.backend import install_dispatch, use_backend

    if name == KERNEL_BACKEND_NATIVE:
        install_dispatch()
    if _LAST_BACKEND[0] != name:
        from cftuv_envelope.materialize.clip_memo import MEMO

        MEMO.clear()
        _LAST_BACKEND[0] = name
    with use_backend(name) as ledger:
        yield ledger


def with_kernel_backend(produce):
    """Добавляет вычислению домена именованный параметр `backend` и кладёт в результат запись бэкенда.

    `produce(...)` возвращает результат, у которого есть `with_changes` (результат продуктового пути). С `backend=PYTHON`
    (умолчание) результат остаётся тем же объектом, без записи. Если порт отказал после частичных эффектов
    (`NATIVE_PARTIAL_EFFECTS_REFUSED`), ответ домена недействителен, каким бы он ни вернулся (исключение могла проглотить промежуточная стадия):
    домен отказан этим именем.

    Бэкенд ЗАДАЁТСЯ ЯВНО на каждом вызове (поток, начатый внутри блока `use_backend`, его не наследует): поток живой ширины передаёт его
    через `run_production(kernel_backend=...)`.
    """

    @functools.wraps(produce)
    def scoped(*args, backend=DEFAULT_KERNEL_BACKEND, **kwargs):
        with entered_backend(backend) as ledger:
            result = produce(*args, **kwargs)
        if ledger is None:
            return result
        if ledger.partial:
            result = _partial_refusal(result, ledger.partial)
        return result.with_changes(backend_record=ledger.record())

    return scoped


def _partial_refusal(result, detail):
    """Отказ домена с именем `NATIVE_PARTIAL_EFFECTS_REFUSED` на месте результата, посчитанного по грязному состоянию."""

    from cftuv_envelope.backend import BackendOutcomeV1

    from .envelope_production_export import _refusal

    return _refusal(
        result.patch_id,
        result.domain_id,
        BackendOutcomeV1.NATIVE_PARTIAL_EFFECTS_REFUSED.value,
        detail,
        result.seconds,
        result.placement,
    )


@dataclass(frozen=True, slots=True)
class BackendSummaryV1:
    """Сводка бэкенда по результатам прогона: домены по исполнителю и откаты по названным исходам."""

    requested: str
    native: int = 0
    python: int = 0
    mixed: int = 0
    cached: int = 0
    #: `((исход, (патчи, ...)), ...)` по имени исхода; патч входит в каждый исход, который у него был.
    outcomes: tuple = ()

    @property
    def computed(self) -> int:
        return self.native + self.python + self.mixed


def backend_summary(results, kernel_backend) -> BackendSummaryV1:
    """Сводка по результатам; результат без записи (отказ входа, пропавший домен) в счёт не идёт."""

    from .envelope_production_export import PLACEMENT_CACHED

    name = normalize_kernel_backend(kernel_backend)
    counts = {"native": 0, "python": 0, "mixed": 0}
    cached = 0
    patches: dict = {}
    for item in results:
        record = getattr(item, "backend_record", None)
        if item.placement == PLACEMENT_CACHED:
            cached += 1
        elif record is not None:
            counts[record.ran] += 1
            for outcome in record.outcomes:
                patches.setdefault(outcome, set()).add(int(item.patch_id))
    return BackendSummaryV1(
        name,
        counts["native"],
        counts["python"],
        counts["mixed"],
        cached,
        tuple((outcome, tuple(sorted(found))) for outcome, found in sorted(patches.items())),
    )


def _patch_list(patches) -> str:
    shown = ", ".join(str(item) for item in patches[:PATCHES_SHOWN])
    more = f", ... (+{len(patches) - PATCHES_SHOWN})" if len(patches) > PATCHES_SHOWN else ""
    return f"patch {shown}{more}"


def backend_text(summary: BackendSummaryV1) -> str:
    """`native 120 / python 2 / mixed 1 / cached 3 (NATIVE_PORT_STALE: patch 7, 9; ...)`."""

    parts = [f"native {summary.native}", f"python {summary.python}"]
    if summary.mixed:
        parts.append(f"mixed {summary.mixed}")
    if summary.cached:
        parts.append(f"cached {summary.cached}")
    text = " / ".join(parts)
    if summary.outcomes:
        text += " (" + "; ".join(f"{name}: {_patch_list(found)}" for name, found in summary.outcomes) + ")"
    return text


def backend_console_lines(results, kernel_backend) -> list:
    """Одна строка журнала прогона; пусто, пока заказан `PYTHON` (умолчание ничего не печатает)."""

    if normalize_kernel_backend(kernel_backend) != KERNEL_BACKEND_NATIVE:
        return []
    return [f"[CFTUV][Production] BACKEND {backend_text(backend_summary(results, kernel_backend))}"]


def backend_timing_suffix(results, kernel_backend) -> str:
    """` | backend native 120 / python 2` в строку панели; пусто, пока заказан `PYTHON`."""

    if normalize_kernel_backend(kernel_backend) != KERNEL_BACKEND_NATIVE:
        return ""
    summary = backend_summary(results, kernel_backend)
    return f" | backend native {summary.native} / python {summary.python + summary.mixed}"


def draw_kernel_backend_row(layout, mesh_settings) -> None:
    """Строка настройки в панели; при заказе нативного бэкенда — статус нативного ядра одной строкой."""

    if not hasattr(mesh_settings, SETTING_NAME):
        return
    layout.prop(mesh_settings, SETTING_NAME)
    if kernel_backend_of(mesh_settings) != KERNEL_BACKEND_NATIVE:
        return
    try:
        from cftuv_envelope.backend import native_status

        status = native_status()
    except Exception as exc:  # noqa: BLE001 - панель не падает из-за статуса
        layout.label(text=f"Native status: {type(exc).__name__}", icon="ERROR")
        return
    if status.available:
        layout.label(text=f"Native {status.version or '?'}: coverage and clip available", icon="CHECKMARK")
    else:
        layout.label(
            text=f"Native: coverage {status.coverage}, clip {status.clip} (Python computes)",
            icon="ERROR",
        )


__all__ = (
    "BackendSummaryV1",
    "DEFAULT_KERNEL_BACKEND",
    "KERNEL_BACKEND_ITEMS",
    "KERNEL_BACKEND_NATIVE",
    "KERNEL_BACKEND_PYTHON",
    "SETTING_NAME",
    "backend_console_lines",
    "backend_identity_of",
    "backend_summary",
    "backend_text",
    "backend_timing_suffix",
    "draw_kernel_backend_row",
    "entered_backend",
    "kernel_backend_of",
    "normalize_kernel_backend",
    "with_kernel_backend",
)
