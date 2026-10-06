"""Выбор бэкенда вычисления ядра: Python-эталон (умолчание) либо нативное ядро `cftuv_native`.

ЕДИНСТВЕННЫЙ ИМПОРТЁР `cftuv_native`: правило в `tests/test_architecture.py`. Остальной код (ядро, хост, инструменты) видит
нативное ядро только через этот модуль.

ДВЕ ЦЕЛЫЕ ОПЕРАЦИИ. Нативно считаются ровно две операции, каждая побитово равна одной версии Python-эталона и отвечает на
тот же вызов: покрытие (`wavefront.coverage._coverage_at`) и геометрическая резка (`materialize.clip.clip_geometry`, то есть
`compute` промаха `clip_memo.run_clip`). `coverage_compute` и `clip_compute` — точки диспетчеризации: они зовутся ВМЕСТО
эталона на месте его вызова и сами решают, кто считает.

БЭКЕНД — НЕ ПОЛИТИКА ЗАПРОСА. Ответ побитово один, каким бы бэкендом ни считали (иначе нативное ядро неверно, и это находит
сверка, а не выбор). Но бэкенд входит в идентичность исполнения: ключи кэшей хоста несут `backend_identity`, чтобы результат
одного бэкенда не подменял результат другого и чтобы сверка двух бэкендов не читала память друг друга.

ТИХОГО ОТКАТА НЕТ. Нативное ядро заказано, а считать им нельзя, — домен считает эталон, и это ЗАПИСАНО в журнал домена
(`BackendLedgerV1` -> `BackendRecordV1`) именем причины (`BackendOutcomeV1`):

* `NATIVE_UNAVAILABLE` — `cftuv_native` не импортируется (колесо не установлено, чужой интерпретатор воркера, сломанное
  расширение); текст исключения лежит в записи;
* `NATIVE_PORT_STALE` — эталон ушёл дальше версии, с которой сверен порт (`cftuv_native.NativePortStale`);
* `NATIVE_UNSUPPORTED_PYTHON` — порт воспроизводит `list.sort` и `sum` CPython 3.11 и 3.13, и только их;
* `NATIVE_PORT_UNSUPPORTED` — порт отказывает на ЭТОМ входе по имени (эталон его считает);
* `NATIVE_TRACES_UNSUPPORTED` — покрытие вызвано с `traces` (запись шаблона шага ширины), а колесо такой записи не даёт (старое колесо:
  у `cftuv_native.coverage_at` нет параметра `traces`);
* `NATIVE_PARTIAL_EFFECTS_REFUSED` — порт отказал ПОСЛЕ того, как применил часть побочных эффектов (бюджет, память канонизации, счётчики знаков,
  нормали плоскости, `traces`, `store`): считать эталоном по грязному состоянию нельзя, домен получает ЭТОТ исход отказом (`NativePartialEffectsRefused`;
  `with_kernel_backend` называет им домен), а не тихий откат;
* `NATIVE_NOT_REACHED` — заказан нативный бэкенд, а домен закончился, ни разу не позвав ни одну нативную операцию (отказ до
  покрытия, попадание резки в память стадии, либо точки диспетчеризации не подключены в этой версии ядра).

Любое другое исключение нативного ядра (в том числе `MaterializationRefusal`, который эталон бросает так же) уходит вызывающему
как есть: именованный отказ домена остаётся тем же, а исключение, которого эталон не бросает, называется выше как
`PRODUCTION_DOMAIN_RAISED`, а не прячется откатом.

ГДЕ СЧИТАЕТ ЗАКАЗАННЫЙ БЭКЕНД. Заказ — контекст вычисления (`use_backend`, `ContextVar`): поток, начатый внутри блока, его не
наследует и считает эталоном. Вне блока и в блоке `PYTHON` точки диспетчеризации зовут эталон напрямую, без журнала: этот путь
побитово совпадает с вызовом эталона (тест `test_backend_dispatch.py`).

ПОДКЛЮЧЕНИЕ ТОЧЕК. Нативный порт закрепляет по дайджесту файлы эталона, которые зеркалит (`cftuv_native.pin.OPERATION_FILES`: покрытие —
`wavefront/coverage.py`, `wavefront/event_time.py` и основа точной арифметики; резка — `materialize/clip*.py`, `coalesce`, `frames`, `lift*`,
`offset_normal`, `tessellate`, `numeric`, `_cpython311` и та же основа): правка любого из них делает операцию `stale`. Поэтому диспетчеры
стоят в НЕЗАКРЕПЛЁННЫХ файлах, у вызывающих:

* покрытие региона — `wavefront/conveyor.py::_region_coverage` зовёт `covered_at` (то же, что `coverage.coverage_at`: источник покрытия шага
  ширины, затем `coverage_compute`; тест держит равенство обёртки по тексту и по поведению);
* запись шаблона шага ширины — `materialize/step.py::_Recorder.coverage` зовёт `coverage_compute(..., traces)`.

Вызов резки `run_clip(clip_geometry, ...)` лежит в `materialize/clip.py::cut_domain`, а `clip.py` закреплён: прямая правка сделает порт резки `stale`
до перевыпуска закреплений. `install_dispatch` ставит диспетчеры подменой имён `wavefront.coverage._coverage_at` и `materialize.clip.clip_geometry` при ЗАПУСКЕ,
не меняя ни одного файла ядра (закрепления остаются верными; подмена `_coverage_at` берёт и повтор покрытия в `coalesce.py`). Хост ставит её при первом заказе
`NATIVE` в процессе (`envelope_kernel_backend.entered_backend`), пока `PYTHON` не ставит ничего; прямая правка `clip.py` с перевыпуском закреплений её заменит.

Модуль не импортирует ничего из ядра на уровне модуля: эталон берётся при вызове, поэтому точки диспетчеризации можно
ставить в модули самого ядра без цикла импорта.
"""

from __future__ import annotations

import importlib
import inspect
import threading
from fractions import Fraction
from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import dataclass
from enum import Enum

COVERAGE = "coverage"
CLIP = "clip"
#: Статус операции нативного ядра, когда `cftuv_native` не импортируется.
UNAVAILABLE = "unavailable"
#: Сколько символов текста исключения несёт запись об откате.
DETAIL_LIMIT = 240


class KernelBackendV1(str, Enum):
    PYTHON = "PYTHON"
    NATIVE = "NATIVE"


class BackendOutcomeV1(str, Enum):
    """Именованные причины, по которым заказанный нативный бэкенд не посчитал (считал эталон)."""

    NATIVE_UNAVAILABLE = "NATIVE_UNAVAILABLE"
    NATIVE_PORT_STALE = "NATIVE_PORT_STALE"
    NATIVE_UNSUPPORTED_PYTHON = "NATIVE_UNSUPPORTED_PYTHON"
    NATIVE_PORT_UNSUPPORTED = "NATIVE_PORT_UNSUPPORTED"
    NATIVE_TRACES_UNSUPPORTED = "NATIVE_TRACES_UNSUPPORTED"
    NATIVE_PARTIAL_EFFECTS_REFUSED = "NATIVE_PARTIAL_EFFECTS_REFUSED"
    NATIVE_NOT_REACHED = "NATIVE_NOT_REACHED"


#: Имена классов отказа нативного шима и исход, которым каждый записывается.
_NAMED_REFUSALS = (
    ("NativePortStale", BackendOutcomeV1.NATIVE_PORT_STALE),
    ("NativeUnsupportedPython", BackendOutcomeV1.NATIVE_UNSUPPORTED_PYTHON),
    ("NativePortUnsupported", BackendOutcomeV1.NATIVE_PORT_UNSUPPORTED),
)
#: Что обязан экспортировать шим, чтобы им можно было пользоваться; без этого он `NATIVE_UNAVAILABLE`.
_REQUIRED = ("coverage_at", "clip_geometry", "native_status", *(name for name, _ in _NAMED_REFUSALS))


def normalize_backend(value) -> KernelBackendV1:
    """`KernelBackendV1` по имени либо члену; неизвестное имя — `ValueError`, а не молчаливый Python."""

    try:
        return value if isinstance(value, KernelBackendV1) else KernelBackendV1(str(value).strip().upper())
    except ValueError as exc:
        names = ", ".join(item.value for item in KernelBackendV1)
        raise ValueError(f"unknown kernel backend {value!r}: expected one of {names}") from exc


# --------------------------------------------------------------------------
# Нативное ядро: загрузка и статус
# --------------------------------------------------------------------------

_LOAD_LOCK = threading.Lock()
_LOADED: list = []
#: `{id(модуль): у его coverage_at есть параметр traces}`.
_TRACES: dict = {}


def _import_native() -> tuple:
    try:
        import cftuv_native as module
    except Exception as exc:  # noqa: BLE001 - любой отказ загрузки расширения — тот же именованный исход
        return None, f"{type(exc).__name__}: {exc}"[:DETAIL_LIMIT]
    missing = [name for name in _REQUIRED if not hasattr(module, name)]
    if missing:
        return None, f"cftuv_native lacks {', '.join(missing)}"
    return module, ""


def _takes_traces(module) -> bool:
    """У `cftuv_native.coverage_at` есть параметр `traces` (колесо с записью знаков шаблона шага ширины)."""

    found = _TRACES.get(id(module))
    if found is None:
        try:
            found = "traces" in inspect.signature(module.coverage_at).parameters
        except (TypeError, ValueError):  # подпись не читается: считаем, что записи нет, и называем это откатом
            found = False
        _TRACES[id(module)] = found
    return found


def _native() -> tuple:
    """`(модуль | None, текст причины)`: вердикт загрузки, один на процесс (`refresh_native` его забывает)."""

    with _LOAD_LOCK:
        if not _LOADED:
            _LOADED.append(_import_native())
        return _LOADED[0]


def refresh_native() -> None:
    """Забывает вердикт загрузки: следующий вызов импортирует заново (колесо установили, а процесс живёт)."""

    with _LOAD_LOCK:
        _LOADED.clear()
        _TRACES.clear()
    importlib.invalidate_caches()


@dataclass(frozen=True, slots=True)
class NativeStatusV1:
    """Статус нативных операций: `available`, `stale(<файлы>)`, `unsupported_python` либо `unavailable`."""

    coverage: str
    clip: str
    version: str = ""
    detail: str = ""

    @property
    def available(self) -> bool:
        return self.coverage == "available" and self.clip == "available"

    def as_record(self) -> dict:
        return {"coverage": self.coverage, "clip": self.clip, "version": self.version, "detail": self.detail}


def native_status() -> NativeStatusV1:
    """Статус нативного ядра; без `cftuv_native` — именованный `unavailable` с причиной, а не исключение."""

    module, detail = _native()
    if module is None:
        return NativeStatusV1(UNAVAILABLE, UNAVAILABLE, "", detail)
    try:
        raw = dict(module.native_status())
        version = str(module.native_version()) if hasattr(module, "native_version") else ""
    except Exception as exc:  # noqa: BLE001 - статус, который не удалось снять, назван, а не брошен
        return NativeStatusV1(UNAVAILABLE, UNAVAILABLE, "", f"native_status failed: {type(exc).__name__}: {exc}"[:DETAIL_LIMIT])
    return NativeStatusV1(str(raw.get(COVERAGE, UNAVAILABLE)), str(raw.get(CLIP, UNAVAILABLE)), version)


def backend_identity(backend) -> str:
    """Идентичность бэкенда для ключей кэшей: `PYTHON` либо `NATIVE:<версия колеса>` (`NATIVE:unavailable` без него)."""

    if normalize_backend(backend) is KernelBackendV1.PYTHON:
        return KernelBackendV1.PYTHON.value
    module, _detail = _native()
    version = ""
    if module is not None and hasattr(module, "native_version"):
        try:
            version = str(module.native_version())
        except Exception:  # noqa: BLE001 - версия, которую не снять, совпадает с отсутствующей
            version = ""
    return f"{KernelBackendV1.NATIVE.value}:{version or UNAVAILABLE}"


# --------------------------------------------------------------------------
# Журнал домена
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class BackendRecordV1:
    """Что заказанный нативный бэкенд сделал с ОДНИМ доменом: сколько операций посчитал он, сколько эталон, почему.

    `fallbacks` — `((исход, операция, сколько раз, текст первой причины), ...)` по порядку имён. Запись плоская (строки и числа):
    она едет из воркера пула ответом домена.
    """

    requested: str
    native_calls: int
    python_calls: int
    fallbacks: tuple = ()

    @property
    def ran(self) -> str:
        """`native` (все операции домена посчитало нативное ядро), `python` (ни одной) либо `mixed`."""

        if not self.native_calls:
            return "python"
        return "native" if not self.python_calls else "mixed"

    @property
    def outcomes(self) -> tuple:
        """Имена исходов отката домена по порядку; заказ нативного бэкенда без единой нативной операции — `NATIVE_NOT_REACHED`."""

        names = sorted({item[0] for item in self.fallbacks})
        reached = self.native_calls or self.python_calls or self.fallbacks
        if self.requested == KernelBackendV1.NATIVE.value and not reached:
            names.append(BackendOutcomeV1.NATIVE_NOT_REACHED.value)
        return tuple(names)

    def as_record(self) -> dict:
        return {
            "requested": self.requested,
            "ran": self.ran,
            "native_calls": self.native_calls,
            "python_calls": self.python_calls,
            "outcomes": list(self.outcomes),
            "fallbacks": [list(item) for item in self.fallbacks],
        }


class BackendLedgerV1:
    """Журнал ОДНОГО домена, пока идёт блок `use_backend`: считает операции и откаты (домен считается в одном потоке)."""

    __slots__ = ("requested", "native", "python", "fallbacks", "partial")

    def __init__(self, requested: str) -> None:
        self.requested = requested
        self.native: dict = {}
        self.python: dict = {}
        self.fallbacks: dict = {}
        #: Текст первого отказа порта по грязному состоянию (`NATIVE_PARTIAL_EFFECTS_REFUSED`) либо `""`: ответ домена после него недействителен.
        self.partial = ""

    def note_native(self, operation: str) -> None:
        self.native[operation] = self.native.get(operation, 0) + 1

    def note_python(self, operation: str, outcome: BackendOutcomeV1, detail: str = "") -> None:
        self.python[operation] = self.python.get(operation, 0) + 1
        slot = self.fallbacks.setdefault((outcome.value, operation), [0, detail[:DETAIL_LIMIT]])
        slot[0] += 1

    def note_partial(self, operation: str, detail: str) -> None:
        """Отказ порта после частичных эффектов: считал не эталон и не порт, а домен отказан именем (`NATIVE_PARTIAL_EFFECTS_REFUSED`)."""

        slot = self.fallbacks.setdefault((BackendOutcomeV1.NATIVE_PARTIAL_EFFECTS_REFUSED.value, operation), [0, detail[:DETAIL_LIMIT]])
        slot[0] += 1
        self.partial = self.partial or f"{operation}: {detail}"[:DETAIL_LIMIT]

    def record(self) -> BackendRecordV1:
        return BackendRecordV1(
            self.requested,
            sum(self.native.values()),
            sum(self.python.values()),
            tuple((outcome, operation, slot[0], slot[1]) for (outcome, operation), slot in sorted(self.fallbacks.items())),
        )


#: Журнал домена, пока идёт блок `use_backend` с нативным бэкендом; `None` — считает эталон, без журнала.
_SCOPE: ContextVar = ContextVar("cftuv_kernel_backend", default=None)


@contextmanager
def use_backend(backend):
    """Внутри блока вычисление идёт заказанным бэкендом. Отдаёт журнал домена (`NATIVE`) либо `None` (`PYTHON`)."""

    choice = normalize_backend(backend)
    ledger = BackendLedgerV1(choice.value) if choice is KernelBackendV1.NATIVE else None
    token = _SCOPE.set(ledger)
    try:
        yield ledger
    finally:
        _SCOPE.reset(token)


def active_backend() -> KernelBackendV1:
    """Бэкенд, заказанный в этом потоке (вне блока — `PYTHON`)."""

    return KernelBackendV1.PYTHON if _SCOPE.get() is None else KernelBackendV1.NATIVE


# --------------------------------------------------------------------------
# Точки диспетчеризации
# --------------------------------------------------------------------------


#: Эталоны, сохранённые `install_dispatch` до подмены имён: диспетчер зовёт их, а не подменённое имя (иначе он позвал бы сам себя).
_ORACLES: dict = {}
#: `(модуль, имя, прежнее значение)` подмен `install_dispatch`; пусто — ядро не тронуто. Заполняется ОДНИМ `extend` после всех подмен: непустой — значит, готов.
_INSTALLED: list = []
#: Первый заказ `NATIVE` могут сделать два потока (главный и поток живой ширины): подмена идёт под замком, иначе второй поток снял бы с модуля
#: УЖЕ подставленный диспетчер как «эталон», и диспетчер позвал бы сам себя.
_INSTALL_LOCK = threading.Lock()


def _python_coverage():
    found = _ORACLES.get(COVERAGE)
    if found is None:
        from .wavefront.coverage import _coverage_at as found
    return found


def _python_clip():
    found = _ORACLES.get(CLIP)
    if found is None:
        from .materialize.clip import clip_geometry as found
    return found


def install_dispatch() -> tuple:
    """Ставит диспетчеры на место вызова эталона подменой имён в модулях ядра; `("модуль.имя", ...)` подменённых. Повтор — то же.

    Файлы ядра не правятся (закрепления нативного порта остаются верными): подменяются `wavefront.coverage._coverage_at` (его читает
    `coverage_at`, а с ним `coalesce` при повторе покрытия), `materialize.step._coverage_at` (если модуль ещё берёт его по имени) и
    `materialize.clip.clip_geometry` (его читает `cut_domain`) — имена, которые вызывающий читает при каждом вызове. Вне блока
    `use_backend` (и под `PYTHON`) диспетчер зовёт эталон напрямую. Процессное состояние: воркеру пула подмена нужна своя.
    """

    if not _INSTALLED:
        with _INSTALL_LOCK:
            if not _INSTALLED:  # двойная проверка: пока ждали замок, первый поток мог всё поставить
                _install_locked()
    return tuple(f"{module.__name__.rsplit('.', 1)[-1]}.{name}" for module, name, _old in _INSTALLED)


def _install_locked() -> None:
    from .materialize import clip, step
    from .wavefront import coverage

    _ORACLES[COVERAGE], _ORACLES[CLIP] = coverage._coverage_at, clip.clip_geometry
    replaced = []
    for module, name, dispatcher in (
        (coverage, "_coverage_at", coverage_compute),
        (step, "_coverage_at", coverage_compute),
        (clip, "clip_geometry", clip_compute),
    ):
        if hasattr(module, name):
            replaced.append((module, name, getattr(module, name)))
            setattr(module, name, dispatcher)
    _INSTALLED.extend(replaced)


def uninstall_dispatch() -> None:
    """Возвращает подменённые имена; без `install_dispatch` ничего не делает."""

    with _INSTALL_LOCK:
        while _INSTALLED:
            module, name, original = _INSTALLED.pop()
            setattr(module, name, original)
        _ORACLES.clear()


def dispatch_installed() -> bool:
    return bool(_INSTALLED)


class NativePartialEffectsRefused(RuntimeError):
    """Порт отказал по имени ПОСЛЕ частичных эффектов: состояние грязное, эталон по нему не считает; домен отказан (`NATIVE_PARTIAL_EFFECTS_REFUSED`)."""


def _effects_snapshot(budget, plane=None, *sized) -> tuple:
    """Всё, что нативная операция меняет до отказа: статьи бюджета, память канонизации, счётчики знаков, нормали плоскости, размеры `traces`/`store`.

    Дёшево (длины и короткие кортежи; нормали плоскости копируются раз на резку). Сравнение до и после отказа решает, можно ли считать эталоном.
    """

    from . import exact_sqrt_sum as exact

    normals = getattr(plane, "_normal_by_position", None)
    return (
        None if budget is None else tuple(budget.spent_by_article()),
        tuple(
            len(table)
            for table in (exact._KNOWN_PRIMES, exact._KNOWN_PRIME_SET, exact._FACTORIZATION_MEMO, exact._SQUAREFREE_MEMO, exact._PRIME_SUPPORT_MEMO)
        ),
        tuple(exact.SIGN_COUNTS.items()),
        tuple(exact.UNBUDGETED_WORK.spent_by_article()),
        None if normals is None else dict(normals),
        tuple(None if item is None else len(item) for item in sized),
    )


class _TracesUnsupported(RuntimeError):
    """Внутренний отказ диспетчера (не исключение шима): колесо не умеет `traces`; записывается как `NATIVE_TRACES_UNSUPPORTED`."""


def _attempt(ledger: BackendLedgerV1, operation: str, call, effects=None) -> tuple:
    """`(True, ответ)`, когда посчитало нативное ядро; `(False, None)` — откат назван и записан, считает эталон.

    `effects()` — снимок побочных эффектов операции (`_effects_snapshot`): отказ порта, после которого он сдвинулся, — не откат, а
    `NativePartialEffectsRefused` (поздний `NativePortUnsupported` приходит уже после того, как порт применил бюджет, память и нормали).
    """

    module, detail = _native()
    if module is None:
        ledger.note_python(operation, BackendOutcomeV1.NATIVE_UNAVAILABLE, detail)
        return False, None
    refusals = tuple((getattr(module, name), outcome) for name, outcome in _NAMED_REFUSALS)
    refusals += ((_TracesUnsupported, BackendOutcomeV1.NATIVE_TRACES_UNSUPPORTED),)
    before = None if effects is None else effects()
    try:
        answer = call(module)
    except tuple(cls for cls, _outcome in refusals) as exc:
        text = f"{type(exc).__name__}: {exc}"
        if effects is not None and effects() != before:
            ledger.note_partial(operation, text)
            raise NativePartialEffectsRefused(f"{operation}: the native port refused after partial effects ({text})") from exc
        ledger.note_python(operation, next(item for cls, item in refusals if isinstance(exc, cls)), text)
        return False, None
    ledger.note_native(operation)
    return True, answer


def coverage_compute(partition, alpha, work_budget=None, store=None, traces=None):
    """`wavefront.coverage._coverage_at(partition, alpha, work_budget, store, traces)`, посчитанное заказанным бэкендом."""

    ledger = _SCOPE.get()
    oracle = _python_coverage()
    if ledger is None:
        return oracle(partition, alpha, work_budget, store, traces)

    def call(module):
        if traces is None:
            return module.coverage_at(partition, alpha, work_budget, store)
        if not _takes_traces(module):
            raise _TracesUnsupported("this cftuv_native wheel has no `traces` in coverage_at (the template recording pass needs it)")
        return module.coverage_at(partition, alpha, work_budget, store, traces)

    done, answer = _attempt(ledger, COVERAGE, call, lambda: _effects_snapshot(work_budget, None, store, traces))
    return answer if done else oracle(partition, alpha, work_budget, store, traces)


def covered_at(partition, alpha, work_budget=None, store=None):
    """`wavefront.coverage.coverage_at(partition, alpha, work_budget, store)` с диспетчером вместо `_coverage_at`.

    Обёртка эталона — источник покрытия шага ширины (`current_coverage_source`: покрытие из шаблона либо запись шаблона), затем сам счёт —
    повторена здесь дословно, потому что стоит в закреплённом файле (`coverage.py`), а закреплённые файлы не правятся. Тест
    `test_covered_at_mirrors_the_coverage_at_wrapper` держит её по поведению и по тексту: правка обёртки в эталоне красит его.
    """

    from .wavefront.coverage import current_coverage_source

    alpha = Fraction(alpha)
    source = current_coverage_source()
    if source is not None:
        produced = source.coverage(partition, alpha, work_budget, store)
        if produced is not None:
            return produced
    return coverage_compute(partition, alpha, work_budget, store)


def clip_compute(plane, budget, **inputs):
    """`materialize.clip.clip_geometry(plane, budget, **inputs)`, посчитанное заказанным бэкендом.

    Подходит как `compute` для `clip_memo.run_clip(compute, plane, budget, policy, **inputs)`: стадия получает ровно именованные
    аргументы ключа памяти.
    """

    ledger = _SCOPE.get()
    oracle = _python_clip()
    if ledger is None:
        return oracle(plane, budget, **inputs)
    done, answer = _attempt(ledger, CLIP, lambda module: module.clip_geometry(plane, budget, **inputs), lambda: _effects_snapshot(budget, plane))
    return answer if done else oracle(plane, budget, **inputs)


__all__ = (
    "BackendLedgerV1",
    "BackendOutcomeV1",
    "BackendRecordV1",
    "COVERAGE",
    "CLIP",
    "KernelBackendV1",
    "NativePartialEffectsRefused",
    "NativeStatusV1",
    "UNAVAILABLE",
    "active_backend",
    "backend_identity",
    "clip_compute",
    "coverage_compute",
    "covered_at",
    "dispatch_installed",
    "install_dispatch",
    "native_status",
    "normalize_backend",
    "refresh_native",
    "uninstall_dispatch",
    "use_backend",
)
