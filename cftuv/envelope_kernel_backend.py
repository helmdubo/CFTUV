"""Бэкенд ядра в хосте: ГЛАВНЫЙ переключатель «Kernel backend», порядки стадий, запись бэкенда на каждом домене и строка журнала.

Ядро умеет считать горячие операции нативно (`cftuv_envelope.backend`: покрытие и резка, скелет, вложение привязки источника), и выбор бэкенда — НЕ политика
запроса: ответ побитово один. Поэтому здесь нет ни слова о геометрии. Здесь четыре вещи:

* ГЛАВНЫЙ ПЕРЕКЛЮЧАТЕЛЬ. `kernel_backend` (`PYTHON` | `NATIVE`) — ЕДИНСТВЕННАЯ настройка владельца: она заказывает ВСЕ стадии, которые ядро считает на Rust
  (покрытие и резка, скелет, вложение привязки источника и любая будущая стадия). `NATIVE` — каждая стадия считает нативную реализацию там, где она есть
  (нет колеса либо порт устарел — стадия считает Python, и строка журнала это называет: `NATIVE_UNAVAILABLE`, `NATIVE_PORT_STALE`, ...; тихого отката нет);
  `PYTHON` — Python-эталон на каждой стадии. Настройка живёт в настройках сцены (`HOTSPOTUV_DecalMeshSettings`), едет в прогон (`run_production`), в задачу пула
  (`DomainTaskV1`) и в запись живой ширины. Умолчание — `NATIVE` (`DEFAULT_KERNEL_BACKEND`, решение владельца 2026-10-07; Python — замороженный эталон и
  именованный откат). СТАРЫЕ СЦЕНЫ: свойство Blender хранит значение только когда его присвоили, поэтому сцена, где `kernel_backend` не трогали, читает умолчание
  `NATIVE`, а сцена с выбранным `PYTHON` остаётся на нём. Порядок `KERNEL_BACKEND_ITEMS` — формат хранения (в сцене лежит индекс): `PYTHON` — 0, `NATIVE` — 1
  (проверяет `tests/blender/test_envelope_kernel_backend_default.py`).
* ПОРЯДКИ СТАДИЙ. `stage_orders` — ЕДИНСТВЕННОЕ место, где из главного переключателя выводятся порядки стадий (`StageOrdersV1`). Постадийный выбор остаётся
  ТОЛЬКО в API и инструментах (`run_production(..., skeleton_backend=..., embedding_backend=...)`, A/B-инструменты): `None` значит «как главный переключатель»,
  явное имя — порядок этой стадии. Параметр стадии со значением по умолчанию, отличным от `None`, — дефект (тест
  `test_every_backend_default_of_the_host_is_the_one_named_constant`). Записи, которые едут в пул и в живую ширину, приводят свои поля к порядкам через
  `settle_stage_orders` (то же единственное место).
* МИГРАЦИЯ. Прежняя отдельная настройка сцены `skeleton_backend` удалена из панели и из RNA. В старых сценах её значение ещё лежит ключом `skeleton_backend`:
  явно сохранённый `PYTHON` в ЛЮБОЙ из двух настроек (старой стадии скелета либо `kernel_backend`) делает главный переключатель `PYTHON`; ничего не сохранено — умолчание
  `NATIVE`. `kernel_backend_of` читает это сразу (без записи), `fold_legacy_skeleton_setting` переносит выбор в главный переключатель и снимает ключ (обработчик
  загрузки файла, нажатие кнопки), а смена переключателя владельцем снимает старый ключ (`drop_legacy_skeleton_setting`), чтобы выбор владельца не перебивался прошлым.
* ЗАПИСЬ ДОМЕНА И СТРОКА ЖУРНАЛА. `with_kernel_backend` оборачивает вычисление домена в блок `use_backend` и кладёт в результат `BackendRecordV1` (какой
  бэкенд посчитал на самом деле и по какой названной причине откат на Python). Результат с записью и без неё равен по ответу: запись — метка запуска, как `placement`.
  Смена бэкенда в процессе сбрасывает память стадии резки ядра (`clip_memo`): её ключ бэкенд не несёт, а ключи кэшей хоста несут (`backend_identity` в
  `envelope_content_key` и в ключах прогона). Диспетчеры подключены в самом ядре (`cftuv_envelope.backend`), при запуске ничего не ставится: воркер пула, как и главный
  процесс, считает заказанным бэкендом с первого домена. Домен, который нативное ядро отказало по имени (`NATIVE_DIVISION_DIVERGED`, пояс
  `NATIVE_PARTIAL_EFFECTS_REFUSED`), получает ЭТОТ исход отказом. Строка журнала называет стадии:
  `[CFTUV][Production] BACKEND native: coverage/clip 120, skeleton 118, embedding 118 | python: coverage/clip 2 (NATIVE_PORT_STALE: patch 7, 9), skeleton 4 (NATIVE_UNAVAILABLE: patch 3) | cached 3`.
  Счёт — в ДОМЕНАХ, посчитанных в этом прогоне (`native` — домен целиком нативно, `python` — домен, где стадия откатилась хоть раз); домен из кэша сессии в счёт покрытия, резки и скелета
  не идёт (`cached`). Стадия, заказанная на `PYTHON` порядком API, называется в `ordered python:` без счёта: она считала эталон по заказу, а не по откату. Все стадии на
  `PYTHON` — строки нет.

ПОДГОТОВКА ПОД БЛОКОМ БЭКЕНДА. Скелет считается в подготовке (`prepare_conveyor` -> `_prepare_region`), которая идёт ДО `produce_domain` — в родителе и в воркерах пула. Поэтому блок `use_backend`
стоит и вокруг подготовки (`prepared_under_backend`, `prepare_for_production`, `run_queue_domain`), а запись домена — слияние записи подготовки и записи материализации
(`BackendRecordV1.merged`). Подготовка зависит только от бэкенда скелета, поэтому ключ её кэша в сессии несёт `skeleton_identity_of` (`PYTHON` | `NATIVE:<native_build_id()>`):
подготовка, построенная Python, не читается как построенная Rust и наоборот. Ключи результата и хранилища по содержимому несут идентичность всех стадий (`backend_identity_of`).
Домен, чей скелет нативное ядро отказало по имени (`NATIVE_DIVISION_DIVERGED`), отказан этим именем (`PreparationRefused`), а не посчитан эталоном.

Модуль не импортирует `bpy` и не импортирует ядро при загрузке: пакет остаётся импортируемым без него.
"""

from __future__ import annotations

import functools
from contextlib import contextmanager
from dataclasses import dataclass

KERNEL_BACKEND_PYTHON = "PYTHON"
KERNEL_BACKEND_NATIVE = "NATIVE"
#: УМОЛЧАНИЕ ПРОДУКТА — нативный бэкенд (решение владельца 2026-10-07; Python-эталон заморожен и остаётся именованным откатом). ЕДИНСТВЕННОЕ место, где умолчание
#: названо: настройка сцены, прогон, задача пула, запись живой ширины и порядки ВСЕХ стадий берут его отсюда (тест
#: `test_every_backend_default_of_the_host_is_the_one_named_constant`); у стадии собственного умолчания нет — её порядок по умолчанию это главный переключатель.
#: Литерал `"PYTHON"` в умолчании параметра — дефект.
DEFAULT_KERNEL_BACKEND = KERNEL_BACKEND_NATIVE
#: Имя свойства главного переключателя в `HOTSPOTUV_DecalMeshSettings`.
SETTING_NAME = "kernel_backend"
#: Ключ УДАЛЁННОЙ настройки стадии скелета: в сценах, сохранённых до единого переключателя, он ещё лежит индексом пункта (`PYTHON` = 0, `NATIVE` = 1).
LEGACY_SKELETON_SETTING_NAME = "skeleton_backend"
#: ПОРЯДОК — ФОРМАТ ХРАНЕНИЯ: Blender кладёт в сцену индекс пункта (`PYTHON` = 0, `NATIVE` = 1); новый пункт — только в конец.
KERNEL_BACKEND_ITEMS = (
    (
        KERNEL_BACKEND_PYTHON,
        "Python",
        "Frozen Python reference for every stage: the answer the native kernel is checked against, and the named fallback "
        "when the native one is unavailable",
    ),
    (
        KERNEL_BACKEND_NATIVE,
        "Native (Rust)",
        "Default. Every stage runs its native (Rust) implementation where there is one (coverage/clip, skeleton, embedding). "
        "The answer is bitwise the same; a stage the native kernel cannot compute is computed in Python and named in the console",
    ),
)
#: Стадии, которыми владеет главный переключатель, в порядке аргументов `cftuv_envelope.backend.use_backend`: `(поле StageOrdersV1, подпись в журнале и панели)`.
#: Новая стадия ядра добавляется ЗДЕСЬ, в `StageOrdersV1`, `_STATUS_FLAGS` и `stage_orders` — и больше нигде (тест сверяет их с сигнатурой `use_backend`).
STAGES = (
    ("coverage_clip", "coverage/clip"),
    ("skeleton", "skeleton"),
    ("snap_embedding", "embedding"),
)
#: Какой признак `NativeStatusV1` говорит, что стадия готова нативно.
_STATUS_FLAGS = {"coverage_clip": "available", "skeleton": "skeleton_available", "snap_embedding": "embedding_available"}
#: Сколько номеров патчей называет строка журнала на исход.
PATCHES_SHOWN = 12

_LAST_BACKEND = [DEFAULT_KERNEL_BACKEND]


def normalize_kernel_backend(value) -> str:
    """`PYTHON` либо `NATIVE`; неизвестное имя — `ValueError` (тот же закон, что у ядра, без его импорта)."""

    text = str(value).strip().upper()
    if text not in (KERNEL_BACKEND_PYTHON, KERNEL_BACKEND_NATIVE):
        raise ValueError(f"unknown kernel backend {value!r}: expected PYTHON or NATIVE")
    return text


@dataclass(frozen=True, slots=True)
class StageOrdersV1:
    """Порядки стадий прогона: какой бэкенд заказан каждой стадии ядра (`PYTHON` | `NATIVE`)."""

    coverage_clip: str
    skeleton: str
    snap_embedding: str

    def as_arguments(self) -> tuple:
        """`(backend, skeleton_backend, embedding_backend)` для `cftuv_envelope.backend.use_backend`."""

        return (self.coverage_clip, self.skeleton, self.snap_embedding)


def stage_orders(kernel_backend=DEFAULT_KERNEL_BACKEND, skeleton_backend=None, embedding_backend=None) -> StageOrdersV1:
    """Порядки стадий из ГЛАВНОГО переключателя: ЕДИНСТВЕННОЕ место, где они выводятся.

    `kernel_backend` — главный переключатель (он же порядок покрытия и резки); `None` у `skeleton_backend` и `embedding_backend` — «как главный переключатель»,
    явное имя — постадийный порядок (API и инструменты: A/B одной стадии, корпус эталона). Продуктовый путь (кнопка, живая ширина, воркеры пула) постадийного порядка не задаёт.
    """

    master = normalize_kernel_backend(kernel_backend)
    return StageOrdersV1(
        master,
        master if skeleton_backend is None else normalize_kernel_backend(skeleton_backend),
        master if embedding_backend is None else normalize_kernel_backend(embedding_backend),
    )


def settle_stage_orders(record, master_field: str = "backend") -> None:
    """Приводит поля порядков записи (`skeleton_backend`, `embedding_backend` со значением `None` — «как главный переключатель») к конкретным именам.

    Для замороженных записей прогона, задачи пула и живой ширины: зовётся из их `__post_init__`, поле главного переключателя называется `master_field`.
    """

    orders = stage_orders(getattr(record, master_field), record.skeleton_backend, record.embedding_backend)
    object.__setattr__(record, master_field, orders.coverage_clip)
    object.__setattr__(record, "skeleton_backend", orders.skeleton)
    object.__setattr__(record, "embedding_backend", orders.snap_embedding)


def legacy_skeleton_choice(mesh_settings):
    """Выбор удалённой настройки `skeleton_backend`, сохранённый в сцене (`PYTHON` | `NATIVE`), либо `None`: ключа нет либо он нечитаем."""

    try:
        raw = mesh_settings[LEGACY_SKELETON_SETTING_NAME]  # ключ свойства Blender: индекс пункта
    except (KeyError, TypeError, AttributeError, IndexError):
        raw = getattr(mesh_settings, LEGACY_SKELETON_SETTING_NAME, None)
    if raw is None:
        return None
    try:
        if isinstance(raw, int) and not isinstance(raw, bool):
            return KERNEL_BACKEND_ITEMS[raw][0] if 0 <= raw < len(KERNEL_BACKEND_ITEMS) else None
        return normalize_kernel_backend(raw)
    except ValueError:
        return None


def kernel_backend_of(mesh_settings) -> str:
    """Главный переключатель из настроек декали сцены.

    Без свойства (вне Blender) и в сцене, где его не трогали, — умолчание продукта (`NATIVE`). Явно сохранённый `PYTHON` в старой настройке скелета (`legacy_skeleton_choice`)
    делает переключатель `PYTHON`, даже если свойство ещё не перенесено (`fold_legacy_skeleton_setting`): чтение ничего не пишет.
    """

    chosen = normalize_kernel_backend(getattr(mesh_settings, SETTING_NAME, DEFAULT_KERNEL_BACKEND) or DEFAULT_KERNEL_BACKEND)
    return KERNEL_BACKEND_PYTHON if legacy_skeleton_choice(mesh_settings) == KERNEL_BACKEND_PYTHON else chosen


def drop_legacy_skeleton_setting(mesh_settings) -> None:
    """Снимает ключ удалённой настройки скелета (нет ключа — ничего не делает)."""

    try:
        del mesh_settings[LEGACY_SKELETON_SETTING_NAME]
    except (KeyError, TypeError, AttributeError, IndexError):
        if hasattr(mesh_settings, LEGACY_SKELETON_SETTING_NAME):
            delattr(mesh_settings, LEGACY_SKELETON_SETTING_NAME)


def fold_legacy_skeleton_setting(mesh_settings):
    """Переносит выбор удалённой настройки скелета в главный переключатель и снимает её ключ; возвращает перенесённый выбор либо `None` (переносить нечего).

    Явный `PYTHON` старой настройки делает главный переключатель явным `PYTHON`; `NATIVE` старой настройки и сцена без неё оставляют переключатель как есть (не присвоен — умолчание `NATIVE`).
    """

    legacy = legacy_skeleton_choice(mesh_settings)
    if legacy == KERNEL_BACKEND_PYTHON:
        setattr(mesh_settings, SETTING_NAME, KERNEL_BACKEND_PYTHON)
    drop_legacy_skeleton_setting(mesh_settings)
    return legacy


def backend_identity_of(kernel_backend, skeleton_backend=None, embedding_backend=None) -> str:
    """Идентичность исполнения для ключей кэшей: `PYTHON` либо `NATIVE:<native_build_id()>` для покрытия и резки, плюс `|skeleton=...` и `|snap_embedding=...` при нативных стадиях.

    Стадия на `PYTHON` строку не меняет. Ядро не импортируется — `PYTHON`/`NATIVE` по именам.
    """

    orders = stage_orders(kernel_backend, skeleton_backend, embedding_backend)
    try:
        from cftuv_envelope.backend import backend_identity
    except ImportError:
        base = orders.coverage_clip if orders.skeleton == KERNEL_BACKEND_PYTHON else f"{orders.coverage_clip}|skeleton={orders.skeleton}"
        return base if orders.snap_embedding == KERNEL_BACKEND_PYTHON else f"{base}|snap_embedding={orders.snap_embedding}"
    return backend_identity(*orders.as_arguments())


def skeleton_identity_of(skeleton_backend=None) -> str:
    """Идентичность стадии скелета: `PYTHON` либо `NATIVE:<native_build_id()>`. Ключ кэша подготовки сессии несёт её, и только её (подготовка от покрытия и резки не зависит).

    `None` — порядок скелета по умолчанию (как главный переключатель по умолчанию).
    """

    skeleton = stage_orders(DEFAULT_KERNEL_BACKEND, skeleton_backend).skeleton
    try:
        from cftuv_envelope.backend import stage_identity
    except ImportError:
        return skeleton
    return stage_identity(skeleton)


@contextmanager
def entered_backend(kernel_backend, skeleton_backend=None, embedding_backend=None):
    """`use_backend` ядра; смена бэкенда покрытия и резки в ЭТОМ процессе сбрасывает память стадии резки.

    Отдаёт журнал домена, если нативным заказана хоть одна стадия, и `None`, если все `PYTHON`. Память резки сбрасывается потому, что её ключ бэкенд не несёт;
    скелет в этой памяти не участвует (ответ скелета побитово один), поэтому его смена память резки не трогает.
    """

    orders = stage_orders(kernel_backend, skeleton_backend, embedding_backend)
    from cftuv_envelope.backend import use_backend

    if _LAST_BACKEND[0] != orders.coverage_clip:
        from cftuv_envelope.materialize.clip_memo import MEMO

        MEMO.clear()
        _LAST_BACKEND[0] = orders.coverage_clip
    with use_backend(*orders.as_arguments()) as ledger:
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


def prepared_under_backend(build, kernel_backend=DEFAULT_KERNEL_BACKEND, skeleton_backend=None, embedding_backend=None, *, borrowed_ledger=None):
    """`(подготовка, запись | None)`: `build()` под блоком бэкенда, скелет считается в нём.

    Заимствованный журнал принадлежит вызывающему: при нём возвращаемая запись — None, чтобы счёт не удвоился.
    Запись — `BackendRecordV1` подготовки (скелет ОТДЕЛЬНО от покрытия и резки: в подготовке считается только он) либо `None`, когда все стадии `PYTHON` (блок журнала
    не заводит, путь побитово равен вызову без блока). Домен, которому нативное ядро отказало по имени, отказан `PreparationRefused`, как бы ни кончилась `build` (исключение могла
    проглотить промежуточная стадия: ответ после отказа недействителен). Память стадии резки здесь не трогается: подготовка резки не зовёт.
    """

    from cftuv_envelope.backend import NativeDomainRefused, use_backend

    orders = stage_orders(kernel_backend, skeleton_backend, embedding_backend)
    with use_backend(*orders.as_arguments(), borrowed_ledger=borrowed_ledger) as ledger:
        try:
            prepared = build()
        except NativeDomainRefused as exc:
            raise PreparationRefused(exc.outcome.value, str(exc), None if borrowed_ledger is not None else ledger.record()) from exc
    if ledger is None:
        return prepared, None
    if ledger.refusal is not None:
        raise PreparationRefused(*ledger.refusal, None if borrowed_ledger is not None else ledger.record())
    return prepared, None if borrowed_ledger is not None else ledger.record()


def merge_backend_records(first, second):
    return second if first is None else first.merged(second)


def ledger_record(ledger):
    return None if ledger is None else ledger.record()


def with_preparation_record(result, record):
    """`result` с записью домена, слитой из записи подготовки и записи материализации (`BackendRecordV1.merged`); без записи подготовки — тот же объект."""

    if record is None:
        return result
    return result.with_changes(backend_record=record.merged(getattr(result, "backend_record", None)))


def with_kernel_backend(produce):
    """Добавляет вычислению домена именованный параметр `backend` и кладёт в результат запись бэкенда.

    `produce(...)` возвращает результат, у которого есть `with_changes` (результат продуктового пути). Умолчание `backend` —
    `DEFAULT_KERNEL_BACKEND` (`NATIVE`); с `backend=PYTHON` результат остаётся тем же объектом, без записи (стадии `skeleton_backend` и `embedding_backend` — как `backend`,
    пока API не задал им порядок). Если нативное ядро отказало ДОМЕНУ (`NATIVE_DIVISION_DIVERGED`: эталон на этом входе не
    завершился бы; `NATIVE_PARTIAL_EFFECTS_REFUSED`: пояс, состояние сдвинулось), ответ домена недействителен, каким бы он ни вернулся (исключение могла
    проглотить промежуточная стадия): домен отказан этим именем.

    Бэкенд ЗАДАЁТСЯ ЯВНО на каждом вызове (поток, начатый внутри блока `use_backend`, его не наследует): поток живой ширины передаёт его
    через `run_production(kernel_backend=...)`. Материализация скелет не считает, но запись домена несёт заказ всех стадий, а запись подготовки сливается с ней (`with_preparation_record`).
    """

    @functools.wraps(produce)
    def scoped(*args, backend=DEFAULT_KERNEL_BACKEND, skeleton_backend=None, embedding_backend=None, **kwargs):
        with entered_backend(backend, skeleton_backend, embedding_backend) as ledger:
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
class StageTallyV1:
    """Счёт одной стадии по результатам прогона: домены, посчитанные в нём нативно либо откатом на Python, и названные исходы отката.

    `python` — домены, где стадия откатилась хоть раз (`python` и `mixed` записи). Домен, где стадия не считалась (подготовка из кэша, отказ до неё), в счёт не идёт.
    """

    stage: str
    requested: str
    native: int = 0
    python: int = 0
    #: `((исход, (патчи, ...)), ...)` по имени исхода; патч входит в каждый исход, который у него был.
    outcomes: tuple = ()


@dataclass(frozen=True, slots=True)
class BackendSummaryV1:
    """Сводка бэкенда по результатам прогона: счёт по стадиям (в порядке `STAGES`), домены из кэша и попадания памяти вложения."""

    stages: tuple
    cached: int = 0
    embedding_memo_hits: int = 0

    def stage(self, name: str) -> StageTallyV1:
        return next(item for item in self.stages if item.stage == name)

    @property
    def native_stages(self) -> tuple:
        return tuple(item for item in self.stages if item.requested == KERNEL_BACKEND_NATIVE)


def _outcome_list(patches: dict) -> tuple:
    return tuple((outcome, tuple(sorted(found))) for outcome, found in sorted(patches.items()))


def backend_summary(results, kernel_backend, skeleton_backend=None, embedding_backend=None) -> BackendSummaryV1:
    """Сводка по результатам прогона; результат без записи (отказ входа, пропавший домен) в счёт не идёт.

    Порядки стадий называют записи доменов (`BackendRecordV1.requested`, `skeleton_requested`, `embedding_requested`: что заказал прогон на самом деле); пока записей нет,
    порядки берутся из аргументов (`stage_orders`). Домен из кэша сессии в счёт покрытия, резки и скелета не идёт (в этом прогоне он их не считал), а вложение привязки источника
    считается в экспорте и для него.
    """

    from .envelope_production_export import PLACEMENT_CACHED

    orders = stage_orders(kernel_backend, skeleton_backend, embedding_backend)
    recorded = next((item.backend_record for item in results if getattr(item, "backend_record", None) is not None), None)
    if recorded is not None:
        orders = StageOrdersV1(recorded.requested, recorded.skeleton_requested, recorded.embedding_requested)
    counts = {name: {"native": 0, "python": 0} for name, _label in STAGES}
    patches = {name: {} for name, _label in STAGES}
    cached = memo_hits = 0
    for item in results:
        record = getattr(item, "backend_record", None)
        fresh = item.placement != PLACEMENT_CACHED
        cached += not fresh
        if record is None:
            continue
        memo_hits += record.embedding_cache_hits
        views = {
            "coverage_clip": (record.ran, record.outcomes) if fresh else ("", ()),
            "skeleton": (record.skeleton_ran, record.skeleton_outcomes) if fresh else ("", ()),
            "snap_embedding": (record.embedding_ran, record.embedding_outcomes),
        }
        for name, _label in STAGES:
            ran, outcomes = views[name]
            ran = ran or ("python" if outcomes else "")  # домен, которому стадию отказали по имени, нативно её не считал
            if not ran:
                continue
            counts[name]["native" if ran == "native" else "python"] += 1
            for outcome in outcomes:
                patches[name].setdefault(outcome, set()).add(int(item.patch_id))
    tallies = tuple(
        StageTallyV1(name, getattr(orders, name), counts[name]["native"], counts[name]["python"], _outcome_list(patches[name]))
        for name, _label in STAGES
    )
    return BackendSummaryV1(tallies, cached, memo_hits if orders.snap_embedding == KERNEL_BACKEND_NATIVE else 0)


def _patch_list(patches) -> str:
    shown = ", ".join(str(item) for item in patches[:PATCHES_SHOWN])
    more = f", ... (+{len(patches) - PATCHES_SHOWN})" if len(patches) > PATCHES_SHOWN else ""
    return f"patch {shown}{more}"


def _outcomes_text(outcomes) -> str:
    return " (" + "; ".join(f"{name}: {_patch_list(found)}" for name, found in outcomes) + ")" if outcomes else ""


def _tally_segments(summary: BackendSummaryV1, *, reasons: bool) -> list:
    """Части строки по порядку: `native: ...`, `python: ...`, `ordered python: ...`; стадия без откатов в `python:` не попадает."""

    labels = dict(STAGES)
    segments = []
    ordered = summary.native_stages
    if ordered:
        segments.append("native: " + ", ".join(f"{labels[item.stage]} {item.native}" for item in ordered))
    fallen = [item for item in ordered if item.python]
    if fallen:
        segments.append(
            "python: " + ", ".join(f"{labels[item.stage]} {item.python}" + (_outcomes_text(item.outcomes) if reasons else "") for item in fallen)
        )
    python_ordered = [labels[item.stage] for item in summary.stages if item.requested == KERNEL_BACKEND_PYTHON]
    if python_ordered:
        segments.append("ordered python: " + ", ".join(python_ordered))
    return segments


def backend_text(summary: BackendSummaryV1) -> str:
    """`native: coverage/clip 120, skeleton 118, embedding 118 | python: coverage/clip 2 (NATIVE_PORT_STALE: patch 7, 9), skeleton 4 (NATIVE_UNAVAILABLE: patch 3) | cached 3`.

    Стадия, заказанная на `PYTHON`, называется в `ordered python:` без счёта: она считала эталон по заказу, а не по откату. `embedding memo K` — попадания памяти вложения (только при `K > 0`).
    """

    segments = _tally_segments(summary, reasons=True)
    if summary.embedding_memo_hits:
        segments.append(f"embedding memo {summary.embedding_memo_hits}")
    if summary.cached:
        segments.append(f"cached {summary.cached}")
    return " | ".join(segments)


def backend_console_lines(results, kernel_backend, skeleton_backend=None, embedding_backend=None) -> list:
    """Одна строка журнала прогона; пусто, пока все стадии заказаны на `PYTHON`. Умолчание продукта — `NATIVE`, поэтому строка печатается каждым нажатием, а откат на Python назван в ней."""

    summary = backend_summary(results, kernel_backend, skeleton_backend, embedding_backend)
    if not summary.native_stages:
        return []
    return [f"[CFTUV][Production] BACKEND {backend_text(summary)}"]


def backend_timing_suffix(results, kernel_backend, skeleton_backend=None, embedding_backend=None) -> str:
    """` | native: coverage/clip 120, skeleton 118, embedding 118 | python: coverage/clip 2, skeleton 4` в строку панели (без причин); пусто, пока все стадии `PYTHON`."""

    summary = backend_summary(results, kernel_backend, skeleton_backend, embedding_backend)
    if not summary.native_stages:
        return ""
    return "".join(f" | {segment}" for segment in _tally_segments(summary, reasons=False) if not segment.startswith("ordered"))


def native_status_line(kernel_backend, status) -> tuple:
    """`(текст, значок)` одной строки состояния под переключателем: какие стадии готовы нативно (по `native_status()`).

    `NATIVE`: все готовы — `Native (Rust): coverage/clip, skeleton, embedding`; часть — `Native: ...; Python: ...` (недостающие считает Python); ни одной — Python считает всё.
    `PYTHON`: `Python for all stages` и, что готово нативно.
    """

    ready = [label for name, label in STAGES if getattr(status, _STATUS_FLAGS[name])]
    missing = [label for name, label in STAGES if not getattr(status, _STATUS_FLAGS[name])]
    if normalize_kernel_backend(kernel_backend) == KERNEL_BACKEND_PYTHON:
        return ("Python for all stages; native ready: " + (", ".join(ready) or "none")), "INFO"
    if not missing:
        return "Native (Rust): " + ", ".join(ready), "CHECKMARK"
    if ready:
        return f"Native: {', '.join(ready)}; Python: {', '.join(missing)}", "ERROR"
    return "Native unavailable: Python computes every stage", "ERROR"


def draw_kernel_backend_row(layout, mesh_settings) -> None:
    """Панель: один выпадающий список главного переключателя и одна строка состояния нативных стадий."""

    if not hasattr(mesh_settings, SETTING_NAME):
        return
    layout.prop(mesh_settings, SETTING_NAME)
    try:
        from cftuv_envelope.backend import native_status

        text, icon = native_status_line(kernel_backend_of(mesh_settings), native_status())
    except Exception as exc:  # noqa: BLE001 - панель не падает из-за статуса
        layout.label(text=f"Native status: {type(exc).__name__}", icon="ERROR")
        return
    layout.label(text=text, icon=icon)


__all__ = (
    "BackendSummaryV1",
    "DEFAULT_KERNEL_BACKEND",
    "KERNEL_BACKEND_ITEMS",
    "KERNEL_BACKEND_NATIVE",
    "KERNEL_BACKEND_PYTHON",
    "LEGACY_SKELETON_SETTING_NAME",
    "PreparationRefused",
    "SETTING_NAME",
    "STAGES",
    "StageOrdersV1",
    "StageTallyV1",
    "backend_console_lines",
    "backend_identity_of",
    "backend_summary",
    "backend_text",
    "backend_timing_suffix",
    "draw_kernel_backend_row",
    "drop_legacy_skeleton_setting",
    "entered_backend",
    "fold_legacy_skeleton_setting",
    "kernel_backend_of",
    "ledger_record",
    "legacy_skeleton_choice",
    "merge_backend_records",
    "native_status_line",
    "normalize_kernel_backend",
    "prepared_under_backend",
    "settle_stage_orders",
    "skeleton_identity_of",
    "stage_orders",
    "with_kernel_backend",
    "with_preparation_record",
)
