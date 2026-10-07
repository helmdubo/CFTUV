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
* СТРОКА ЖУРНАЛА. `backend_console_lines`: `[CFTUV][Production] BACKEND coverage/clip native 120 / python 2 (NATIVE_PORT_STALE: patch 7, 9); skeleton native 118 / python 4 (NATIVE_UNAVAILABLE: patch 3)`.
  Печатается, когда нативным заказана хоть одна стадия; домен из кэша сессии в счёт не идёт (в этом прогоне он не считался). Скелет назван ОТДЕЛЬНО от покрытия и
  резки (`skeleton python` — стадия заказана на Python, счёта нет); у скелета домена «не считался» значит «подготовка из кэша», а не откат.

СТАДИЯ SKELETON — ВТОРАЯ НАСТРОЙКА (`skeleton_backend`, `PYTHON` | `NATIVE`). Продукт переводит стадии на Rust по одной: покрытие и резка уже NATIVE по умолчанию, скелет —
`DEFAULT_SKELETON_BACKEND` (`PYTHON` в этом срезе: на `NATIVE` его переводит владелец после строгой сверки Python и Rust на полевых случаях). Настройка едет всюду, где едет `kernel_backend`
(`run_production`, `DomainTaskV1.skeleton_backend`, запись живой ширины, свойство сцены).

ПОДГОТОВКА ПОД БЛОКОМ БЭКЕНДА. Скелет считается в подготовке (`prepare_conveyor` -> `_prepare_region`), которая идёт ДО `produce_domain` — в родителе и в воркерах пула. Поэтому блок `use_backend`
стоит и вокруг подготовки (`prepared_under_backend`, `prepare_for_production`, `run_queue_domain`), а запись домена — слияние записи подготовки и записи материализации
(`BackendRecordV1.merged`). Подготовка зависит только от бэкенда скелета, поэтому ключ её кэша в сессии несёт `skeleton_identity_of` (`PYTHON` | `NATIVE:<native_build_id()>`):
подготовка, построенная Python, не читается как построенная Rust и наоборот. Ключи результата и хранилища по содержимому несут идентичность обеих стадий (`backend_identity_of`).
Домен, чей скелет нативное ядро отказало по имени (`NATIVE_DIVISION_DIVERGED`), отказан этим именем (`PreparationRefused`), а не посчитан эталоном.

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
#: УМОЛЧАНИЕ СТАДИИ SKELETON — Python. Покрытие и резка переведены на Rust, скелет ещё нет: нативный скелет включается (константой либо настройкой сцены) ПОСЛЕ строгой
#: сверки Python и Rust на полевых случаях (ответы, цены и дайджесты равны; `tools/blender_native_ab.py --stage skeleton`). Названо ОДНИМ местом, как и `DEFAULT_KERNEL_BACKEND`.
DEFAULT_SKELETON_BACKEND = KERNEL_BACKEND_PYTHON
#: Имя свойства в `HOTSPOTUV_DecalMeshSettings`.
SETTING_NAME = "kernel_backend"
#: Имя свойства стадии скелета там же.
SKELETON_SETTING_NAME = "skeleton_backend"
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
#: Порядок тот же (`PYTHON` — 0, `NATIVE` — 1): в сцене лежит индекс пункта.
SKELETON_BACKEND_ITEMS = (
    (
        KERNEL_BACKEND_PYTHON,
        "Python",
        "Default. Frozen reference skeleton in Python (the preparation stage SKELETON): the answer the native skeleton is checked against",
    ),
    (
        KERNEL_BACKEND_NATIVE,
        "Native (Rust)",
        "The skeleton in the native kernel (cftuv_native.build_skeleton). The answer and the price are bitwise the same; a domain the native "
        "kernel cannot compute is computed in Python and named in the console. Applies to the next Build Decal Mesh",
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
    """Заказанный бэкенд из настроек декали сцены; без свойства (вне Blender) и в сцене, где его не трогали, — умолчание продукта (`NATIVE`)."""

    return normalize_kernel_backend(getattr(mesh_settings, SETTING_NAME, DEFAULT_KERNEL_BACKEND) or DEFAULT_KERNEL_BACKEND)


def skeleton_backend_of(mesh_settings) -> str:
    """Заказанный бэкенд скелета из настроек декали сцены; без свойства (вне Blender, старая сцена) — умолчание стадии (`DEFAULT_SKELETON_BACKEND`)."""

    return normalize_kernel_backend(getattr(mesh_settings, SKELETON_SETTING_NAME, DEFAULT_SKELETON_BACKEND) or DEFAULT_SKELETON_BACKEND)


def backend_identity_of(kernel_backend, skeleton_backend=DEFAULT_SKELETON_BACKEND) -> str:
    """Идентичность исполнения для ключей кэшей: `PYTHON` либо `NATIVE:<native_build_id()>` для покрытия и резки, плюс `|skeleton=...` при нативном скелете.

    Скелет на `PYTHON` (умолчание) строку не меняет. Ядро не импортируется — `PYTHON`/`NATIVE` по именам.
    """

    name = normalize_kernel_backend(kernel_backend)
    skeleton = normalize_kernel_backend(skeleton_backend)
    try:
        from cftuv_envelope.backend import backend_identity
    except ImportError:
        return name if skeleton == KERNEL_BACKEND_PYTHON else f"{name}|skeleton={skeleton}"
    return backend_identity(name, skeleton)


def skeleton_identity_of(skeleton_backend=DEFAULT_SKELETON_BACKEND) -> str:
    """Идентичность стадии скелета: `PYTHON` либо `NATIVE:<native_build_id()>`. Ключ кэша подготовки сессии несёт её, и только её (подготовка от покрытия и резки не зависит)."""

    skeleton = normalize_kernel_backend(skeleton_backend)
    try:
        from cftuv_envelope.backend import stage_identity
    except ImportError:
        return skeleton
    return stage_identity(skeleton)


@contextmanager
def entered_backend(kernel_backend, skeleton_backend=DEFAULT_SKELETON_BACKEND):
    """`use_backend` ядра; смена бэкенда покрытия и резки в ЭТОМ процессе сбрасывает память стадии резки.

    Отдаёт журнал домена, если нативным заказана хоть одна стадия, и `None`, если обе `PYTHON`. Память резки сбрасывается потому, что её ключ бэкенд не несёт;
    скелет в этой памяти не участвует (ответ скелета побитово один), поэтому его смена память резки не трогает.
    """

    name = normalize_kernel_backend(kernel_backend)
    skeleton = normalize_kernel_backend(skeleton_backend)
    from cftuv_envelope.backend import use_backend

    if _LAST_BACKEND[0] != name:
        from cftuv_envelope.materialize.clip_memo import MEMO

        MEMO.clear()
        _LAST_BACKEND[0] = name
    with use_backend(name, skeleton) as ledger:
        yield ledger


class PreparationRefused(RuntimeError):
    """Домен отказан по имени ещё в подготовке: нативное ядро отказало скелету (`NATIVE_DIVISION_DIVERGED`) либо сдвинуло состояние (`NATIVE_PARTIAL_EFFECTS_REFUSED`).

    `outcome` — имя исхода (строка), `detail` — текст, `record` — запись подготовки (`BackendRecordV1`), чтобы отказ нёс, кто и что считал. Подготовки у такого домена нет.
    """

    def __init__(self, outcome: str, detail: str, record) -> None:
        super().__init__(f"{outcome}: {detail}")
        self.outcome = str(outcome)
        self.detail = str(detail)
        self.record = record


def prepared_under_backend(build, kernel_backend=DEFAULT_KERNEL_BACKEND, skeleton_backend=DEFAULT_SKELETON_BACKEND):
    """`(подготовка, запись | None)`: `build()` под блоком бэкенда, скелет считается в нём.

    Запись — `BackendRecordV1` подготовки (скелет ОТДЕЛЬНО от покрытия и резки: в подготовке считается только он) либо `None`, когда обе стадии `PYTHON` (блок журнала
    не заводит, путь побитово равен вызову без блока). Домен, которому нативное ядро отказало по имени, отказан `PreparationRefused`, как бы ни кончилась `build` (исключение могла
    проглотить промежуточная стадия: ответ после отказа недействителен). Память стадии резки здесь не трогается: подготовка резки не зовёт.
    """

    from cftuv_envelope.backend import NativeDomainRefused, use_backend

    with use_backend(normalize_kernel_backend(kernel_backend), normalize_kernel_backend(skeleton_backend)) as ledger:
        try:
            prepared = build()
        except NativeDomainRefused as exc:
            raise PreparationRefused(exc.outcome.value, str(exc), ledger.record()) from exc
    if ledger is None:
        return prepared, None
    if ledger.refusal is not None:
        raise PreparationRefused(*ledger.refusal, ledger.record())
    return prepared, ledger.record()


def with_preparation_record(result, record):
    """`result` с записью домена, слитой из записи подготовки и записи материализации (`BackendRecordV1.merged`); без записи подготовки — тот же объект."""

    if record is None:
        return result
    return result.with_changes(backend_record=record.merged(getattr(result, "backend_record", None)))


def with_kernel_backend(produce):
    """Добавляет вычислению домена именованный параметр `backend` и кладёт в результат запись бэкенда.

    `produce(...)` возвращает результат, у которого есть `with_changes` (результат продуктового пути). Умолчание `backend` —
    `DEFAULT_KERNEL_BACKEND` (`NATIVE`); с `backend=PYTHON` результат остаётся тем же объектом, без записи. Если нативное ядро отказало ДОМЕНУ (`NATIVE_DIVISION_DIVERGED`: эталон на этом входе не
    завершился бы; `NATIVE_PARTIAL_EFFECTS_REFUSED`: пояс, состояние сдвинулось), ответ домена недействителен, каким бы он ни вернулся (исключение могла
    проглотить промежуточная стадия): домен отказан этим именем.

    Бэкенд ЗАДАЁТСЯ ЯВНО на каждом вызове (поток, начатый внутри блока `use_backend`, его не наследует): поток живой ширины передаёт его
    через `run_production(kernel_backend=...)`. `skeleton_backend` (умолчание `DEFAULT_SKELETON_BACKEND`) заказывает стадию скелета того же журнала:
    материализация скелета не считает, но запись домена несёт заказ обеих стадий, а запись подготовки сливается с ней (`with_preparation_record`).
    """

    @functools.wraps(produce)
    def scoped(*args, backend=DEFAULT_KERNEL_BACKEND, skeleton_backend=DEFAULT_SKELETON_BACKEND, **kwargs):
        with entered_backend(backend, skeleton_backend) as ledger:
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
    """Сводка бэкенда по результатам прогона: домены по исполнителю и откаты по названным исходам.

    `requested`, `native`, `python`, `mixed`, `cached`, `outcomes` — покрытие и резка. Скелет (`skeleton_*`) считается ОТДЕЛЬНО: у него свой заказ, и домен, чей скелет в этом
    прогоне не считался (подготовка из кэша, отказ до скелета), в его счёт не идёт.
    """

    requested: str
    native: int = 0
    python: int = 0
    mixed: int = 0
    cached: int = 0
    #: `((исход, (патчи, ...)), ...)` по имени исхода; патч входит в каждый исход, который у него был.
    outcomes: tuple = ()
    skeleton_requested: str = KERNEL_BACKEND_PYTHON
    skeleton_native: int = 0
    skeleton_python: int = 0
    skeleton_mixed: int = 0
    #: Как `outcomes`, для скелета.
    skeleton_outcomes: tuple = ()

    @property
    def computed(self) -> int:
        return self.native + self.python + self.mixed

    @property
    def skeleton_computed(self) -> int:
        return self.skeleton_native + self.skeleton_python + self.skeleton_mixed


def backend_summary(results, kernel_backend, skeleton_backend=DEFAULT_SKELETON_BACKEND) -> BackendSummaryV1:
    """Сводка по результатам; результат без записи (отказ входа, пропавший домен) в счёт не идёт."""

    from .envelope_production_export import PLACEMENT_CACHED

    name = normalize_kernel_backend(kernel_backend)
    skeleton_name = normalize_kernel_backend(skeleton_backend)
    counts = {"native": 0, "python": 0, "mixed": 0}
    skeleton_counts = {"native": 0, "python": 0, "mixed": 0}
    cached = 0
    patches: dict = {}
    skeleton_patches: dict = {}
    for item in results:
        record = getattr(item, "backend_record", None)
        if item.placement == PLACEMENT_CACHED:
            cached += 1
        elif record is not None:
            counts[record.ran] += 1
            for outcome in record.outcomes:
                patches.setdefault(outcome, set()).add(int(item.patch_id))
            if record.skeleton_ran:
                skeleton_counts[record.skeleton_ran] += 1
            for outcome in record.skeleton_outcomes:
                skeleton_patches.setdefault(outcome, set()).add(int(item.patch_id))
    return BackendSummaryV1(
        name,
        counts["native"],
        counts["python"],
        counts["mixed"],
        cached,
        tuple((outcome, tuple(sorted(found))) for outcome, found in sorted(patches.items())),
        skeleton_name,
        skeleton_counts["native"],
        skeleton_counts["python"],
        skeleton_counts["mixed"],
        tuple((outcome, tuple(sorted(found))) for outcome, found in sorted(skeleton_patches.items())),
    )


def _patch_list(patches) -> str:
    shown = ", ".join(str(item) for item in patches[:PATCHES_SHOWN])
    more = f", ... (+{len(patches) - PATCHES_SHOWN})" if len(patches) > PATCHES_SHOWN else ""
    return f"patch {shown}{more}"


def _outcomes_text(outcomes) -> str:
    return " (" + "; ".join(f"{name}: {_patch_list(found)}" for name, found in outcomes) + ")" if outcomes else ""


def backend_text(summary: BackendSummaryV1) -> str:
    """`coverage/clip native 120 / python 2 / mixed 1 / cached 3 (NATIVE_PORT_STALE: patch 7, 9); skeleton native 118 / python 4 / mixed 1 (NATIVE_UNAVAILABLE: patch 3)`.

    Стадия, заказанная на `PYTHON`, называется одним словом без счёта (`coverage/clip python`, `skeleton python`): она считала эталон по заказу, а не по откату.
    """

    if summary.requested == KERNEL_BACKEND_NATIVE:
        parts = [f"native {summary.native}", f"python {summary.python}"]
        if summary.mixed:
            parts.append(f"mixed {summary.mixed}")
        if summary.cached:
            parts.append(f"cached {summary.cached}")
        coverage = "coverage/clip " + " / ".join(parts) + _outcomes_text(summary.outcomes)
    else:
        coverage = "coverage/clip python"
    if summary.skeleton_requested == KERNEL_BACKEND_NATIVE:
        parts = [f"native {summary.skeleton_native}", f"python {summary.skeleton_python}"]
        if summary.skeleton_mixed:
            parts.append(f"mixed {summary.skeleton_mixed}")
        skeleton = "skeleton " + " / ".join(parts) + _outcomes_text(summary.skeleton_outcomes)
    else:
        skeleton = "skeleton python"
    return f"{coverage}; {skeleton}"


def backend_console_lines(results, kernel_backend, skeleton_backend=DEFAULT_SKELETON_BACKEND) -> list:
    """Одна строка журнала прогона; пусто, пока обе стадии заказаны на `PYTHON`. Умолчание продукта — `NATIVE` для покрытия и резки, поэтому строка печатается каждым нажатием, а откат на Python назван в ней."""

    if KERNEL_BACKEND_NATIVE not in (normalize_kernel_backend(kernel_backend), normalize_kernel_backend(skeleton_backend)):
        return []
    return [f"[CFTUV][Production] BACKEND {backend_text(backend_summary(results, kernel_backend, skeleton_backend))}"]


def backend_timing_suffix(results, kernel_backend, skeleton_backend=DEFAULT_SKELETON_BACKEND) -> str:
    """` | backend native 120 / python 2 | skeleton native 118 / python 4` в строку панели; каждая часть — только для стадии, заказанной на `NATIVE`."""

    summary = backend_summary(results, kernel_backend, skeleton_backend)
    text = ""
    if summary.requested == KERNEL_BACKEND_NATIVE:
        text += f" | backend native {summary.native} / python {summary.python + summary.mixed}"
    if summary.skeleton_requested == KERNEL_BACKEND_NATIVE:
        text += f" | skeleton native {summary.skeleton_native} / python {summary.skeleton_python + summary.skeleton_mixed}"
    return text


def draw_kernel_backend_row(layout, mesh_settings) -> None:
    """Строки настройки в панели; при заказе нативного бэкенда — статус нативного ядра одной строкой на стадию."""

    if not hasattr(mesh_settings, SETTING_NAME):
        return
    layout.prop(mesh_settings, SETTING_NAME)
    if hasattr(mesh_settings, SKELETON_SETTING_NAME):
        layout.prop(mesh_settings, SKELETON_SETTING_NAME)
    coverage_native = kernel_backend_of(mesh_settings) == KERNEL_BACKEND_NATIVE
    skeleton_native = skeleton_backend_of(mesh_settings) == KERNEL_BACKEND_NATIVE
    if not (coverage_native or skeleton_native):
        return
    try:
        from cftuv_envelope.backend import native_status

        status = native_status()
    except Exception as exc:  # noqa: BLE001 - панель не падает из-за статуса
        layout.label(text=f"Native status: {type(exc).__name__}", icon="ERROR")
        return
    if coverage_native:
        if status.available:
            layout.label(text=f"Native {status.version or '?'} ({status.build_id[:10] or '?'}): coverage and clip available", icon="CHECKMARK")
        else:
            layout.label(
                text=f"Native: coverage {status.coverage}, clip {status.clip} (Python computes)",
                icon="ERROR",
            )
    if skeleton_native:
        if status.skeleton_available:
            layout.label(text="Native skeleton available", icon="CHECKMARK")
        else:
            layout.label(text=f"Native: skeleton {status.skeleton} (Python computes)", icon="ERROR")


__all__ = (
    "BackendSummaryV1",
    "DEFAULT_KERNEL_BACKEND",
    "DEFAULT_SKELETON_BACKEND",
    "KERNEL_BACKEND_ITEMS",
    "KERNEL_BACKEND_NATIVE",
    "KERNEL_BACKEND_PYTHON",
    "PreparationRefused",
    "SETTING_NAME",
    "SKELETON_BACKEND_ITEMS",
    "SKELETON_SETTING_NAME",
    "backend_console_lines",
    "backend_identity_of",
    "backend_summary",
    "backend_text",
    "backend_timing_suffix",
    "draw_kernel_backend_row",
    "entered_backend",
    "kernel_backend_of",
    "normalize_kernel_backend",
    "prepared_under_backend",
    "skeleton_backend_of",
    "skeleton_identity_of",
    "with_kernel_backend",
    "with_preparation_record",
)
