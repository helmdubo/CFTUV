"""Бэкенд ядра в хосте: настройка «Kernel backend», запись бэкенда на каждом домене и строка журнала.

Ядро умеет считать две горячие операции нативно (`cftuv_envelope.backend`: покрытие и резка), и выбор бэкенда — НЕ политика
запроса: ответ побитово один. Поэтому здесь нет ни слова о геометрии. Здесь три вещи:

* НАСТРОЙКА. `kernel_backend` (`PYTHON` | `NATIVE`) живёт в настройках сцены рядом с остальными настройками декали
  (`HOTSPOTUV_DecalMeshSettings`), едет в прогон (`run_production`), в задачу пула (`DomainTaskV1.backend`) и в запись живой
  ширины. Умолчание — `NATIVE` (`DEFAULT_KERNEL_BACKEND`, решение владельца 2026-10-07: покрытие и резка — стадии, переведённые на
  Rust; Python — замороженный эталон и именованный откат). Нет колеса либо порт устарел — домен считает Python, и строка журнала это
  называет (`NATIVE_UNAVAILABLE`, `NATIVE_PORT_STALE`, ...): тихого отката нет. СТАРЫЕ СЦЕНЫ: свойство Blender хранит значение только
  когда его присвоили (`is_property_set`), поэтому сцена, где `kernel_backend` не трогали, читает новое умолчание `NATIVE`, а сцена, где
  выбрали `PYTHON` (хоть то же значение, что было умолчанием), хранит его и остаётся на `PYTHON`; миграции нет и не нужна. Порядок
  `KERNEL_BACKEND_ITEMS` — формат хранения (в сцене лежит индекс): `PYTHON` — 0, `NATIVE` — 1, порядок не менять
  (проверяет `tests/blender/test_envelope_kernel_backend_default.py`).
* ЗАПИСЬ ДОМЕНА. `with_kernel_backend` оборачивает вычисление домена в блок `use_backend` и кладёт в результат `BackendRecordV1`
  (какой бэкенд посчитал на самом деле и по какой названной причине откат на Python). Результат с записью и без неё равен по
  ответу: запись — метка запуска, как `placement`. Смена бэкенда в процессе сбрасывает память стадии резки ядра (`clip_memo`):
  её ключ бэкенд не несёт, а ключи кэшей хоста несут (`backend_identity` в `envelope_content_key` и в ключах прогона).
  Диспетчеры покрытия и резки подключены в самом ядре (`cftuv_envelope.backend`), при запуске ничего не ставится: воркер пула, как и главный
  процесс, считает заказанным бэкендом с первого домена. Домен, который нативное ядро отказало по имени (`NATIVE_DIVISION_DIVERGED`, пояс
  `NATIVE_PARTIAL_EFFECTS_REFUSED`), получает ЭТОТ исход отказом.
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
#: УМОЛЧАНИЕ ПРОДУКТА — нативный бэкенд (решение владельца 2026-10-07: покрытие и резка переведены на Rust; Python-эталон заморожен и
#: остаётся именованным откатом). ЕДИНСТВЕННОЕ место, где умолчание названо: настройка сцены, прогон, задача пула и запись живой ширины
#: берут его отсюда (тест `test_every_backend_default_of_the_host_is_the_one_named_constant`), литерал `"PYTHON"` в умолчании параметра — дефект.
DEFAULT_KERNEL_BACKEND = KERNEL_BACKEND_NATIVE
#: Имя свойства в `HOTSPOTUV_DecalMeshSettings`.
SETTING_NAME = "kernel_backend"
#: ПОРЯДОК — ФОРМАТ ХРАНЕНИЯ: Blender кладёт в сцену индекс пункта (`PYTHON` = 0, `NATIVE` = 1); новый пункт — только в конец.
KERNEL_BACKEND_ITEMS = (
    (
        KERNEL_BACKEND_PYTHON,
        "Python",
        "Frozen reference kernel in Python: the answer the native kernel is checked against, and the named fallback "
        "when the native one is unavailable",
    ),
    (
        KERNEL_BACKEND_NATIVE,
        "Native (Rust)",
        "Default. Coverage and clip in the native kernel (cftuv_native). The answer is bitwise the same; a domain the "
        "native kernel cannot compute is computed in Python and named in the console",
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
    """Идентичность бэкенда для ключей кэшей: `PYTHON` либо `NATIVE:<native_build_id()>`; ядро не импортируется — `PYTHON`/`NATIVE`."""

    name = normalize_kernel_backend(kernel_backend)
    try:
        from cftuv_envelope.backend import backend_identity
    except ImportError:
        return name
    return backend_identity(name)


@contextmanager
def entered_backend(kernel_backend):
    """`use_backend` ядра; смена бэкенда в ЭТОМ процессе сбрасывает память стадии резки.

    Отдаёт журнал домена (`NATIVE`) либо `None` (`PYTHON`). Память резки сбрасывается потому, что её ключ бэкенд не несёт.
    """

    name = normalize_kernel_backend(kernel_backend)
    from cftuv_envelope.backend import use_backend

    if _LAST_BACKEND[0] != name:
        from cftuv_envelope.materialize.clip_memo import MEMO

        MEMO.clear()
        _LAST_BACKEND[0] = name
    with use_backend(name) as ledger:
        yield ledger


def with_kernel_backend(produce):
    """Добавляет вычислению домена именованный параметр `backend` и кладёт в результат запись бэкенда.

    `produce(...)` возвращает результат, у которого есть `with_changes` (результат продуктового пути). Умолчание `backend` —
    `DEFAULT_KERNEL_BACKEND` (`NATIVE`); с `backend=PYTHON` результат остаётся тем же объектом, без записи. Если нативное ядро отказало ДОМЕНУ (`NATIVE_DIVISION_DIVERGED`: эталон на этом входе не
    завершился бы; `NATIVE_PARTIAL_EFFECTS_REFUSED`: пояс, состояние сдвинулось), ответ домена недействителен, каким бы он ни вернулся (исключение могла
    проглотить промежуточная стадия): домен отказан этим именем.

    Бэкенд ЗАДАЁТСЯ ЯВНО на каждом вызове (поток, начатый внутри блока `use_backend`, его не наследует): поток живой ширины передаёт его
    через `run_production(kernel_backend=...)`.
    """

    @functools.wraps(produce)
    def scoped(*args, backend=DEFAULT_KERNEL_BACKEND, **kwargs):
        with entered_backend(backend) as ledger:
            result = produce(*args, **kwargs)
        if ledger is None:
            return result
        if ledger.refusal is not None:
            result = _domain_refusal(result, *ledger.refusal)
        return result.with_changes(backend_record=ledger.record())

    return scoped


def _domain_refusal(result, outcome, detail):
    """Отказ домена именем `outcome` (`NATIVE_DIVISION_DIVERGED` либо `NATIVE_PARTIAL_EFFECTS_REFUSED`) на месте результата, который домен всё же вернул."""

    from .envelope_production_export import _refusal

    return _refusal(
        result.patch_id,
        result.domain_id,
        outcome,
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
    """Одна строка журнала прогона; пусто, пока заказан `PYTHON`. Умолчание продукта — `NATIVE`, поэтому строка печатается каждым нажатием, а откат на Python назван в ней."""

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
        layout.label(text=f"Native {status.version or '?'} ({status.build_id[:10] or '?'}): coverage and clip available", icon="CHECKMARK")
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
