"""Выбор бэкенда вычисления ядра: Python-эталон (умолчание) либо нативное ядро `cftuv_native`.

ЕДИНСТВЕННЫЙ ИМПОРТЁР `cftuv_native`: правило в `tests/test_architecture.py`. Остальной код (ядро, хост, инструменты) видит
нативное ядро только через этот модуль.

ТРИ ЦЕЛЫЕ ОПЕРАЦИИ. Нативно считаются ровно три операции, каждая побитово равна одной версии Python-эталона и отвечает на
тот же вызов: покрытие (`wavefront.coverage._coverage_at`), геометрическая резка (`materialize.clip.clip_geometry`, то есть
`compute` промаха `clip_memo.run_clip`) и скелет (`wavefront.skeleton.build_skeleton`, стадия SKELETON подготовки).
`coverage_compute`, `clip_compute` и `skeleton_compute` — точки диспетчеризации: они зовутся ВМЕСТО эталона на месте его
вызова и сами решают, кто считает.

СТАДИИ ЗАКАЗЫВАЮТСЯ ПОРОЗНЬ. `use_backend(backend, skeleton_backend)`: `backend` заказывает покрытие и резку, `skeleton_backend`
(умолчание `PYTHON`) — скелет; продукт переводит стадии на Rust по одной, и покрытие с резкой могут считать нативно, пока скелет
считает эталон. Журнал домена (`BackendRecordV1`) несёт скелет ОТДЕЛЬНО (`skeleton_*`): счёты, откаты и исходы покрытия с резкой
его не смешивают. Колесо без `build_skeleton` (или без ключа `skeleton` в `native_status()`) — именованный `unavailable`
скелета, а покрытие и резка остаются доступными; скелет такого колеса считает эталон, откат назван `NATIVE_UNAVAILABLE`.

БЭКЕНД — НЕ ПОЛИТИКА ЗАПРОСА. Ответ побитово один, каким бы бэкендом ни считали (иначе нативное ядро неверно, и это находит
сверка, а не выбор). Но бэкенд входит в идентичность исполнения: ключи кэшей хоста несут `backend_identity`
(`NATIVE:<native_build_id()>`: содержательный отпечаток нативного кода и шима, а не номер версии колеса), чтобы результат
одного бэкенда не подменял результат другого, пересобранное колесо не читало память прежнего и сверка двух бэкендов
не читала память друг друга.

ТИХОГО ОТКАТА НЕТ. Нативное ядро заказано, а считать им нельзя, — домен считает эталон, и это ЗАПИСАНО в журнал домена
(`BackendLedgerV1` -> `BackendRecordV1`) именем причины (`BackendOutcomeV1`):

* `NATIVE_UNAVAILABLE` — `cftuv_native` не импортируется (колесо не установлено, чужой интерпретатор воркера, сломанное
  расширение) либо не даёт того, что хост требует от него (`_REQUIRED`: `NATIVE_REFUSALS`, `native_build_id`, ...); текст причины лежит в записи;
* `NATIVE_PORT_STALE` — эталон ушёл дальше версии, с которой сверен порт (`cftuv_native.NativePortStale`);
* `NATIVE_UNSUPPORTED_PYTHON` — интерпретатор младше порога порта (`cftuv_native.NativeUnsupportedPython`);
* `NATIVE_PORT_UNSUPPORTED` — порт отказывает на ЭТОМ входе по имени (`cftuv_native.NativePortUnsupported`; эталон его считает);
* `NATIVE_TRACES_UNSUPPORTED` — покрытие вызвано с `traces` (запись шаблона шага ширины), а колесо такой записи не даёт (старое колесо:
  у `cftuv_native.coverage_at` нет параметра `traces`);
* `NATIVE_INTERNAL_ERROR` — вход, который расширение не умеет нести (`TypeError("cftuv_native: ...")`), либо паника Rust, пойманная
  расширением (`RuntimeError` с «native panic»). Оба случая до любого эффекта, но это НЕ отказ порта (их нет в `NATIVE_REFUSALS`), а дефект:
  домен считает эталон (владелец получает результат), а дефект виден — имя и текст исключения в записи и в строке BACKEND;
* `NATIVE_DIVISION_DIVERGED` — ОТКАЗ ДОМЕНА, а не откат: `cftuv_native.NativeDivisionDiverged` (обобщённое деление порта не закончилось). Делением эталона
  на этом входе был бы бесконечный цикл, поэтому эталону домен не отдаётся; домен получает ЭТОТ исход (`NativeDomainRefused`; `with_kernel_backend` называет им домен);
* `NATIVE_PARTIAL_EFFECTS_REFUSED` — страховочный пояс: порт отказал, а видимое из Python состояние (бюджет, память канонизации, счётчики знаков,
  нормали плоскости, `traces`, `store`) сдвинулось. По контракту порта (`cftuv_native.NATIVE_REFUSALS`) этого не бывает: отказ порта оставляет состояние
  как до вызова. Если пояс всё же сработал, считать эталоном по грязному состоянию нельзя, и домен получает ЭТОТ исход отказом, а не тихий откат;
* `NATIVE_NOT_REACHED` — заказан нативный бэкенд, а домен закончился, ни разу не позвав ни одну нативную операцию (отказ до
  покрытия, попадание резки в память стадии, покрытие из шаблона шага ширины).

ЧТО ОТКАТ, А ЧТО ОТВЕТ. Откат на эталон безопасен ТОЛЬКО для `cftuv_native.NATIVE_REFUSALS` (`NativePortStale`, `NativeUnsupportedPython`,
`NativePortUnsupported`, `NativeDivisionDiverged`): каждый из них оставляет состояние как до вызова, и эталон на том же бюджете, плоскости и таблицах отвечает
как чистый прогон эталона (кроме `NativeDivisionDiverged`, см. выше). Исход ЭТАЛОНА (`MaterializationRefusal`, `ExactCanonicalizationWorkBudgetExhausted`,
`OverflowError`, `ValueError`, `KeyError`, `ZeroDivisionError`, ...) нативное ядро применяет с теми частичными эффектами, какие оставило бы исключение эталона:
он И ЕСТЬ ответ и уходит вызывающему как есть (имя отказа домена остаётся тем же; исключение, которого эталон не бросает, называется выше как
`PRODUCTION_DOMAIN_RAISED`, а не прячется откатом). Так же уходит всё прочее, чего этот модуль не знает по имени (в том числе `NativeMirrorError` и сбой
журнала памяти: их состояние могло быть тронуто).

ГДЕ СЧИТАЕТ ЗАКАЗАННЫЙ БЭКЕНД. Заказ — контекст вычисления (`use_backend`, `ContextVar`): поток, начатый внутри блока, его не
наследует и считает эталоном. Вне блока и в блоке `PYTHON` точки диспетчеризации зовут эталон напрямую, без журнала: этот путь
побитово совпадает с вызовом эталона (тест `test_backend_dispatch.py`).

ПОДКЛЮЧЕНИЕ ТОЧЕК — прямой правкой вызывающего, без подмены имён в процессе:

* покрытие региона — `wavefront/conveyor.py::_region_coverage` зовёт `covered_at` (то же, что `coverage.coverage_at`: источник покрытия шага
  ширины, затем `coverage_compute`; тест держит равенство обёртки по тексту и по поведению);
* запись шаблона шага ширины — `materialize/step.py::_Recorder.coverage` зовёт `coverage_compute(..., traces)`;
* резка — `materialize/clip.py::cut_domain` зовёт `run_clip(backend.clip_compute, ...)`;
* скелет — `wavefront/conveyor.py::_prepare_region` зовёт `backend.skeleton_compute(...)` на месте `build_skeleton(...)` (те же аргументы и ответ).

Нативный порт закрепляет по дайджесту файлы эталона, которые зеркалит (`cftuv_native.pin.OPERATION_FILES`: покрытие — `wavefront/coverage.py`,
`wavefront/event_time.py` и основа точной арифметики; резка — `materialize/clip*.py`, `coalesce`, `frames`, `lift*`, `offset_normal`, `tessellate`,
`numeric`, `_cpython311` и та же основа): правка любого из них делает операцию `stale` до перевыпуска закреплений, а тест архитектуры требует нового закрепления
в том же слиянии с диспетчером в закреплённом файле. `clip.py` закреплён, поэтому правка `cut_domain` идёт вместе с перевыпуском закрепления резки; покрытие
стоит в незакреплённых файлах. Повтор покрытия в `coalesce.py` (контуры региона, без цены) стоит в закреплённом файле и считает эталон под любым бэкендом (ответ тот же).

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
#: Третья целая операция: `wavefront.skeleton.build_skeleton` (стадия SKELETON подготовки; своя настройка, см. `use_backend`).
SKELETON = "skeleton"
#: Статус операции нативного ядра, когда `cftuv_native` не импортируется.
UNAVAILABLE = "unavailable"
#: Сколько символов текста исключения несёт запись об откате.
DETAIL_LIMIT = 240
#: Как начинается текст `TypeError` расширения о входе, который оно не умеет нести (`pyobj.rs`, `cost.py`).
SHIM_TEXT_PREFIX = "cftuv_native:"
#: Что есть в тексте `RuntimeError`, в который расширение превращает пойманную панику Rust (`clip.rs`, `coverage.rs`).
PANIC_TEXT_MARK = "native panic"


class KernelBackendV1(str, Enum):
    PYTHON = "PYTHON"
    NATIVE = "NATIVE"


class BackendOutcomeV1(str, Enum):
    """Именованные причины, по которым заказанный нативный бэкенд не посчитал (считал эталон либо домен отказан)."""

    NATIVE_UNAVAILABLE = "NATIVE_UNAVAILABLE"
    NATIVE_PORT_STALE = "NATIVE_PORT_STALE"
    NATIVE_UNSUPPORTED_PYTHON = "NATIVE_UNSUPPORTED_PYTHON"
    NATIVE_PORT_UNSUPPORTED = "NATIVE_PORT_UNSUPPORTED"
    NATIVE_TRACES_UNSUPPORTED = "NATIVE_TRACES_UNSUPPORTED"
    NATIVE_INTERNAL_ERROR = "NATIVE_INTERNAL_ERROR"
    NATIVE_DIVISION_DIVERGED = "NATIVE_DIVISION_DIVERGED"
    NATIVE_PARTIAL_EFFECTS_REFUSED = "NATIVE_PARTIAL_EFFECTS_REFUSED"
    NATIVE_NOT_REACHED = "NATIVE_NOT_REACHED"


#: Имена классов отказа порта (все члены `cftuv_native.NATIVE_REFUSALS`) и исход, которым каждый записывается. `NativeDivisionDiverged` — отказ ДОМЕНА.
_NAMED_REFUSALS = (
    ("NativePortStale", BackendOutcomeV1.NATIVE_PORT_STALE),
    ("NativeUnsupportedPython", BackendOutcomeV1.NATIVE_UNSUPPORTED_PYTHON),
    ("NativePortUnsupported", BackendOutcomeV1.NATIVE_PORT_UNSUPPORTED),
    ("NativeDivisionDiverged", BackendOutcomeV1.NATIVE_DIVISION_DIVERGED),
)
#: Что обязан экспортировать шим, чтобы им можно было пользоваться; без этого он `NATIVE_UNAVAILABLE`. `NATIVE_REFUSALS` — перечень отказов, после
#: которых откат на эталон безопасен; без него хост не знает, какие исключения порта оставляют состояние нетронутым.
_REQUIRED = ("coverage_at", "clip_geometry", "native_status", "native_build_id", "NATIVE_REFUSALS", *(name for name, _ in _NAMED_REFUSALS))


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
    refusals = module.NATIVE_REFUSALS
    if not (isinstance(refusals, tuple) and all(isinstance(item, type) and issubclass(item, BaseException) for item in refusals)):
        return None, "cftuv_native.NATIVE_REFUSALS is not a tuple of exception classes"
    unlisted = [name for name, _ in _NAMED_REFUSALS if getattr(module, name) not in refusals]
    if unlisted:
        return None, f"cftuv_native.NATIVE_REFUSALS does not list {', '.join(unlisted)}"
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
    """Статус нативных операций: `available`, `stale(<файлы>)`, `unsupported_python` либо `unavailable`.

    `version` — номер колеса (для панели), `build_id` — содержательный отпечаток сборки (`native_build_id()`, то, что входит в идентичность бэкенда).
    `skeleton` — статус третьей операции: колесо без ключа `skeleton` в `native_status()` (старое колесо) — именованный `unavailable`.
    `available` говорит о покрытии и резке (стадиях, которые продукт уже считает нативно); готовность скелета — `skeleton_available`.
    """

    coverage: str
    clip: str
    version: str = ""
    detail: str = ""
    build_id: str = ""
    skeleton: str = UNAVAILABLE

    @property
    def available(self) -> bool:
        return self.coverage == "available" and self.clip == "available"

    @property
    def skeleton_available(self) -> bool:
        return self.skeleton == "available"

    def as_record(self) -> dict:
        return {
            "coverage": self.coverage,
            "clip": self.clip,
            "skeleton": self.skeleton,
            "version": self.version,
            "detail": self.detail,
            "build_id": self.build_id,
        }


def native_status() -> NativeStatusV1:
    """Статус нативного ядра; без `cftuv_native` — именованный `unavailable` с причиной, а не исключение."""

    module, detail = _native()
    if module is None:
        return NativeStatusV1(UNAVAILABLE, UNAVAILABLE, "", detail)
    try:
        raw = dict(module.native_status())
        version = str(module.native_version()) if hasattr(module, "native_version") else ""
        build_id = str(module.native_build_id())
    except Exception as exc:  # noqa: BLE001 - статус, который не удалось снять, назван, а не брошен
        return NativeStatusV1(UNAVAILABLE, UNAVAILABLE, "", f"native_status failed: {type(exc).__name__}: {exc}"[:DETAIL_LIMIT])
    return NativeStatusV1(str(raw.get(COVERAGE, UNAVAILABLE)), str(raw.get(CLIP, UNAVAILABLE)), version, "", build_id, str(raw.get(SKELETON, UNAVAILABLE)))


def stage_identity(backend) -> str:
    """Идентичность ОДНОЙ стадии для ключей кэшей: `PYTHON` либо `NATIVE:<native_build_id()>` (`NATIVE:unavailable` без колеса).

    Отпечаток сборки, а не номер колеса: любая правка Rust, шима или закреплений меняет его, и результат прежней сборки не читается как результат новой.
    Подготовка зависит только от бэкенда скелета, поэтому ключ её кэша несёт ровно эту идентичность (`stage_identity(skeleton_backend)`).
    """

    if normalize_backend(backend) is KernelBackendV1.PYTHON:
        return KernelBackendV1.PYTHON.value
    module, _detail = _native()
    build_id = ""
    if module is not None:
        try:
            build_id = str(module.native_build_id())
        except Exception:  # noqa: BLE001 - отпечаток, который не снять, совпадает с отсутствующим
            build_id = ""
    return f"{KernelBackendV1.NATIVE.value}:{build_id or UNAVAILABLE}"


def backend_identity(backend, skeleton_backend=KernelBackendV1.PYTHON) -> str:
    """Идентичность исполнения для ключей кэшей: стадия покрытия и резки (`stage_identity(backend)`) и, если скелет считает нативное ядро, его стадия.

    Скелет на `PYTHON` (умолчание) не меняет строку: ключи прогона без нативного скелета те же, что и были. Нативный скелет добавляет `|skeleton=NATIVE:<id>`,
    и результат (подготовка, ответ домена), посчитанный одним составом бэкендов, не читается как результат другого.
    """

    base = stage_identity(backend)
    if normalize_backend(skeleton_backend) is KernelBackendV1.PYTHON:
        return base
    return f"{base}|skeleton={stage_identity(skeleton_backend)}"


# --------------------------------------------------------------------------
# Журнал домена
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class BackendRecordV1:
    """Что заказанный нативный бэкенд сделал с ОДНИМ доменом: сколько операций посчитал он, сколько эталон, почему.

    `fallbacks` — `((исход, операция, сколько раз, текст первой причины), ...)` по порядку имён. Запись плоская (строки и числа):
    она едет из воркера пула ответом домена.

    `requested`, `native_calls`, `python_calls`, `fallbacks` — ПОКРЫТИЕ И РЕЗКА (стадии материализации); СКЕЛЕТ (стадия подготовки) — отдельными
    полями `skeleton_*`: у него свой заказ (`skeleton_requested`), свои счёты и свои исходы, и они не смешиваются с первыми. Подготовка и материализация
    домена считаются под РАЗНЫМИ блоками `use_backend`, и запись домена — их слияние (`merged`).
    """

    requested: str
    native_calls: int
    python_calls: int
    fallbacks: tuple = ()
    skeleton_requested: str = KernelBackendV1.PYTHON.value
    skeleton_native_calls: int = 0
    skeleton_python_calls: int = 0
    skeleton_fallbacks: tuple = ()

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

    @property
    def skeleton_ran(self) -> str:
        """`native` / `python` / `mixed` по счёту скелета этого домена; пусто — скелет в этом прогоне не считался (подготовка из кэша, отказ до скелета)."""

        if not (self.skeleton_native_calls or self.skeleton_python_calls):
            return ""
        if not self.skeleton_native_calls:
            return "python"
        return "native" if not self.skeleton_python_calls else "mixed"

    @property
    def skeleton_outcomes(self) -> tuple:
        """Имена исходов отката скелета по порядку. Домен, до скелета не дошедший, не откат: `NATIVE_NOT_REACHED` у скелета нет (кэш подготовки — не отказ)."""

        return tuple(sorted({item[0] for item in self.skeleton_fallbacks}))

    def merged(self, other: "BackendRecordV1 | None") -> "BackendRecordV1":
        """Запись домена из двух блоков (`other` — второй по времени, обычно материализация): счёты складываются, откаты сливаются по `(исход, операция)`.

        Заказ (`requested`, `skeleton_requested`) берётся у этой записи: обе записи одного домена заказаны одним прогоном.
        """

        if other is None:
            return self
        return BackendRecordV1(
            self.requested,
            self.native_calls + other.native_calls,
            self.python_calls + other.python_calls,
            _merged_fallbacks(self.fallbacks, other.fallbacks),
            self.skeleton_requested,
            self.skeleton_native_calls + other.skeleton_native_calls,
            self.skeleton_python_calls + other.skeleton_python_calls,
            _merged_fallbacks(self.skeleton_fallbacks, other.skeleton_fallbacks),
        )

    def as_record(self) -> dict:
        return {
            "requested": self.requested,
            "ran": self.ran,
            "native_calls": self.native_calls,
            "python_calls": self.python_calls,
            "outcomes": list(self.outcomes),
            "fallbacks": [list(item) for item in self.fallbacks],
            "skeleton_requested": self.skeleton_requested,
            "skeleton_ran": self.skeleton_ran,
            "skeleton_native_calls": self.skeleton_native_calls,
            "skeleton_python_calls": self.skeleton_python_calls,
            "skeleton_outcomes": list(self.skeleton_outcomes),
            "skeleton_fallbacks": [list(item) for item in self.skeleton_fallbacks],
        }


def _merged_fallbacks(first: tuple, second: tuple) -> tuple:
    """Откаты двух записей: `(исход, операция)` один раз, счёт складывается, текст — первой причины."""

    slots: dict = {}
    for outcome, operation, count, detail in (*first, *second):
        slot = slots.setdefault((outcome, operation), [0, detail])
        slot[0] += count
    return tuple((outcome, operation, slot[0], slot[1]) for (outcome, operation), slot in sorted(slots.items()))


class BackendLedgerV1:
    """Журнал ОДНОГО домена, пока идёт блок `use_backend`: считает операции и откаты (домен считается в одном потоке).

    `requested` заказывает покрытие и резку, `skeleton_requested` — скелет; точка диспетчеризации стадии, чей заказ `PYTHON`, зовёт эталон напрямую и журнала не пишет.
    """

    __slots__ = ("requested", "skeleton_requested", "native", "python", "fallbacks", "refusal")

    def __init__(self, requested: str, skeleton_requested: str = KernelBackendV1.PYTHON.value) -> None:
        self.requested = requested
        self.skeleton_requested = skeleton_requested
        self.native: dict = {}
        self.python: dict = {}
        self.fallbacks: dict = {}
        #: `(исход, текст)` первого отказа ДОМЕНА (`NATIVE_DIVISION_DIVERGED`, `NATIVE_PARTIAL_EFFECTS_REFUSED`) либо `None`: ответ домена после него недействителен.
        self.refusal = None

    @property
    def coverage_clip_native(self) -> bool:
        return self.requested == KernelBackendV1.NATIVE.value

    @property
    def skeleton_native(self) -> bool:
        return self.skeleton_requested == KernelBackendV1.NATIVE.value

    def note_native(self, operation: str) -> None:
        self.native[operation] = self.native.get(operation, 0) + 1

    def note_python(self, operation: str, outcome: BackendOutcomeV1, detail: str = "") -> None:
        self.python[operation] = self.python.get(operation, 0) + 1
        slot = self.fallbacks.setdefault((outcome.value, operation), [0, detail[:DETAIL_LIMIT]])
        slot[0] += 1

    def note_refusal(self, operation: str, outcome: BackendOutcomeV1, detail: str) -> None:
        """Отказ домена: считал не эталон и не порт, а домен отказан именем (`NATIVE_DIVISION_DIVERGED`, `NATIVE_PARTIAL_EFFECTS_REFUSED`)."""

        slot = self.fallbacks.setdefault((outcome.value, operation), [0, detail[:DETAIL_LIMIT]])
        slot[0] += 1
        if self.refusal is None:
            self.refusal = (outcome.value, f"{operation}: {detail}"[:DETAIL_LIMIT])

    def record(self) -> BackendRecordV1:
        def fallbacks(of_skeleton: bool) -> tuple:
            return tuple(
                (outcome, operation, slot[0], slot[1])
                for (outcome, operation), slot in sorted(self.fallbacks.items())
                if (operation == SKELETON) is of_skeleton
            )

        return BackendRecordV1(
            self.requested,
            sum(count for operation, count in self.native.items() if operation != SKELETON),
            sum(count for operation, count in self.python.items() if operation != SKELETON),
            fallbacks(False),
            self.skeleton_requested,
            self.native.get(SKELETON, 0),
            self.python.get(SKELETON, 0),
            fallbacks(True),
        )


#: Журнал домена, пока идёт блок `use_backend` с нативным бэкендом (хоть одной стадии); `None` — считает эталон, без журнала.
_SCOPE: ContextVar = ContextVar("cftuv_kernel_backend", default=None)


@contextmanager
def use_backend(backend, skeleton_backend=KernelBackendV1.PYTHON):
    """Внутри блока вычисление идёт заказанными бэкендами: `backend` — покрытие и резка, `skeleton_backend` — скелет.

    Отдаёт журнал домена, если нативным заказана хоть одна стадия, и `None`, если обе `PYTHON` (тогда точки диспетчеризации зовут эталон напрямую).
    Стадия с заказом `PYTHON` в журнал не пишет и в блоке с нативной другой стадией: она считает эталон тем же путём, что вне блока.
    """

    choice, skeleton = normalize_backend(backend), normalize_backend(skeleton_backend)
    native = KernelBackendV1.NATIVE
    ledger = BackendLedgerV1(choice.value, skeleton.value) if native in (choice, skeleton) else None
    token = _SCOPE.set(ledger)
    try:
        yield ledger
    finally:
        _SCOPE.reset(token)


def active_backend() -> KernelBackendV1:
    """Бэкенд покрытия и резки, заказанный в этом потоке (вне блока — `PYTHON`)."""

    ledger = _SCOPE.get()
    return KernelBackendV1.NATIVE if ledger is not None and ledger.coverage_clip_native else KernelBackendV1.PYTHON


def active_skeleton_backend() -> KernelBackendV1:
    """Бэкенд скелета, заказанный в этом потоке (вне блока — `PYTHON`)."""

    ledger = _SCOPE.get()
    return KernelBackendV1.NATIVE if ledger is not None and ledger.skeleton_native else KernelBackendV1.PYTHON


# --------------------------------------------------------------------------
# Точки диспетчеризации
# --------------------------------------------------------------------------


def _python_coverage():
    """Эталон покрытия, `wavefront.coverage._coverage_at` (имя в модуле ядра не подменяется: диспетчер стоит у вызывающего)."""

    from .wavefront.coverage import _coverage_at

    return _coverage_at


def _python_clip():
    """Эталон резки, `materialize.clip.clip_geometry` (имя в модуле ядра не подменяется: диспетчер стоит у вызывающего)."""

    from .materialize.clip import clip_geometry

    return clip_geometry


def _python_skeleton():
    """Эталон скелета, `wavefront.skeleton.build_skeleton` (имя в модуле ядра не подменяется: диспетчер стоит у вызывающего)."""

    from .wavefront.skeleton import build_skeleton

    return build_skeleton


class NativeDomainRefused(RuntimeError):
    """Домен отказан по имени (`.outcome`): эталону его не отдать (`NATIVE_DIVISION_DIVERGED`) либо состояние грязное (`NATIVE_PARTIAL_EFFECTS_REFUSED`)."""

    def __init__(self, outcome: BackendOutcomeV1, message: str) -> None:
        super().__init__(message)
        self.outcome = outcome


def _effects_snapshot(budget, plane=None, *sized) -> tuple:
    """Всё, что нативная операция меняет до отказа: статьи бюджета, память канонизации, счётчики знаков, нормали плоскости, размеры `traces`/`store`.

    Страховочный пояс: по контракту порта отказ из `NATIVE_REFUSALS` ничего из этого не двигает. Дёшево (длины и короткие кортежи; нормали плоскости копируются
    раз на резку). Сравнение до и после отказа решает, можно ли считать эталоном.
    """

    from . import exact_sqrt_sum as exact

    normals = getattr(plane, "_normal_by_position", None)
    return (
        # строка `superlevel` бюджета пишется скелетом на месте (`build_skeleton`); у покрытия и резки она неподвижна
        None if budget is None else (tuple(budget.spent_by_article()), getattr(budget, "superlevel", None)),
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


class _OperationAbsent(RuntimeError):
    """Внутренний отказ диспетчера (не исключение шима): колесо не даёт самой операции (старое колесо без `build_skeleton`); записывается как `NATIVE_UNAVAILABLE`."""


def _port_outcome(module, exc) -> BackendOutcomeV1:
    """Исход, которым записывается отказ порта; член `NATIVE_REFUSALS`, которого хост по имени не знает, — `NATIVE_INTERNAL_ERROR` (виден как дефект)."""

    for name, outcome in _NAMED_REFUSALS:
        if isinstance(exc, getattr(module, name)):
            return outcome
    return BackendOutcomeV1.NATIVE_INTERNAL_ERROR


def _is_native_defect(exc) -> bool:
    """Дефект порта, поднятый до любого эффекта, но не отказ из `NATIVE_REFUSALS`: вход, который расширение не несёт, либо пойманная паника Rust.

    Только эти два вида: исход эталона и всё остальное (`NativeMirrorError`, сбой журнала памяти, `ValueError` буфера) здесь не дефект порта и уходят как есть.
    """

    if type(exc) is TypeError:
        return str(exc).startswith(SHIM_TEXT_PREFIX)
    return type(exc) is RuntimeError and PANIC_TEXT_MARK in str(exc)


def _declined(ledger: BackendLedgerV1, operation: str, outcome: BackendOutcomeV1, text: str, effects, before, cause) -> tuple:
    """Откат на эталон, названный `outcome`; `(False, None)`. Страховочный пояс: если видимое состояние сдвинулось, отката нет — домен отказан `NATIVE_PARTIAL_EFFECTS_REFUSED`."""

    if effects is not None and effects() != before:
        refused = BackendOutcomeV1.NATIVE_PARTIAL_EFFECTS_REFUSED
        ledger.note_refusal(operation, refused, text)
        raise NativeDomainRefused(refused, f"{operation}: the native port declined after partial effects ({text})") from cause
    ledger.note_python(operation, outcome, text)
    return False, None


def _attempt(ledger: BackendLedgerV1, operation: str, call, effects=None) -> tuple:
    """`(True, ответ)`, когда посчитало нативное ядро; `(False, None)` — откат назван и записан, считает эталон.

    Три рода исключений нативного вызова:

    * отказ порта (`module.NATIVE_REFUSALS`): состояние как до вызова, откат на эталон назван (`NativeDivisionDiverged` — исключение: эталон на нём не завершится,
      домен отказан `NATIVE_DIVISION_DIVERGED`);
    * дефект порта до эффектов (`TypeError("cftuv_native: ...")`, паника Rust): откат на эталон назван `NATIVE_INTERNAL_ERROR`;
    * всё остальное — исход эталона (его частичные эффекты оставлены нативным вызовом так, как оставило бы исключение эталона) и чужие исключения: как есть.

    `effects()` — снимок побочных эффектов операции (`_effects_snapshot`), страховочный пояс отката: если после отказа он сдвинулся, откат запрещён.
    """

    module, detail = _native()
    if module is None:
        ledger.note_python(operation, BackendOutcomeV1.NATIVE_UNAVAILABLE, detail)
        return False, None
    before = None if effects is None else effects()
    try:
        answer = call(module)
    except _TracesUnsupported as exc:
        return _declined(ledger, operation, BackendOutcomeV1.NATIVE_TRACES_UNSUPPORTED, str(exc), effects, before, exc)
    except _OperationAbsent as exc:
        return _declined(ledger, operation, BackendOutcomeV1.NATIVE_UNAVAILABLE, str(exc), effects, before, exc)
    except module.NATIVE_REFUSALS as exc:
        text = f"{type(exc).__name__}: {exc}"
        outcome = _port_outcome(module, exc)
        if outcome is BackendOutcomeV1.NATIVE_DIVISION_DIVERGED:
            ledger.note_refusal(operation, outcome, text)
            raise NativeDomainRefused(outcome, f"{operation}: the native division did not finish, and the oracle would not either ({text})") from exc
        return _declined(ledger, operation, outcome, text, effects, before, exc)
    except Exception as exc:
        if not _is_native_defect(exc):
            raise
        return _declined(ledger, operation, BackendOutcomeV1.NATIVE_INTERNAL_ERROR, f"{type(exc).__name__}: {exc}", effects, before, exc)
    ledger.note_native(operation)
    return True, answer


def coverage_compute(partition, alpha, work_budget=None, store=None, traces=None):
    """`wavefront.coverage._coverage_at(partition, alpha, work_budget, store, traces)`, посчитанное заказанным бэкендом."""

    ledger = _SCOPE.get()
    oracle = _python_coverage()
    if ledger is None or not ledger.coverage_clip_native:
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
    if ledger is None or not ledger.coverage_clip_native:
        return oracle(plane, budget, **inputs)
    done, answer = _attempt(ledger, CLIP, lambda module: module.clip_geometry(plane, budget, **inputs), lambda: _effects_snapshot(budget, plane))
    return answer if done else oracle(plane, budget, **inputs)


def skeleton_compute(polygon, *, split_search=None, work_budget=None, dense_hydration=False):
    """`wavefront.skeleton.build_skeleton(polygon, split_search=..., work_budget=..., dense_hydration=...)`, посчитанное заказанным бэкендом скелета.

    Сигнатура и ответ эталона; `split_search=None` — умолчание эталона (поиск мотоциклом), его эталону не передают. Вне блока и в блоке, где скелет заказан `PYTHON`,
    вызов побитово равен вызову эталона и журнала не пишет. Заказан `NATIVE`: колесо без `build_skeleton` — откат `NATIVE_UNAVAILABLE`; отказ порта
    (`NativePortStale`: файлы скелета ушли от пина, в том числе `python -O`; `NativeUnsupportedPython`; `NativePortUnsupported`) оставляет состояние как до вызова и откатывается
    на эталон с именем; `NativeDivisionDiverged` — отказ домена (эталон на этом входе не закончил бы), `NativeDomainRefused`; дефект порта до эффектов — `NATIVE_INTERNAL_ERROR`.
    Исход эталона (исключение с его текстом, статья бюджета, строка `superlevel`, память канонизации, `SIGN_COUNTS`, `UNBUDGETED_WORK`) нативный вызов оставляет так, как оставило бы
    исключение эталона, и оно уходит вызывающему как есть.
    """

    ledger = _SCOPE.get()
    oracle = _python_skeleton()
    keywords = {"work_budget": work_budget, "dense_hydration": dense_hydration}
    if split_search is not None:
        keywords["split_search"] = split_search
    if ledger is None or not ledger.skeleton_native:
        return oracle(polygon, **keywords)

    def call(module):
        if not hasattr(module, "build_skeleton"):
            raise _OperationAbsent("this cftuv_native wheel has no `build_skeleton` (it predates the skeleton operation)")
        return module.build_skeleton(polygon, split_search=split_search, work_budget=work_budget, dense_hydration=dense_hydration)

    done, answer = _attempt(ledger, SKELETON, call, lambda: _effects_snapshot(work_budget))
    return answer if done else oracle(polygon, **keywords)


__all__ = (
    "BackendLedgerV1",
    "BackendOutcomeV1",
    "BackendRecordV1",
    "COVERAGE",
    "CLIP",
    "SKELETON",
    "KernelBackendV1",
    "NativeDomainRefused",
    "NativeStatusV1",
    "PANIC_TEXT_MARK",
    "SHIM_TEXT_PREFIX",
    "UNAVAILABLE",
    "active_backend",
    "active_skeleton_backend",
    "backend_identity",
    "clip_compute",
    "coverage_compute",
    "covered_at",
    "native_status",
    "normalize_backend",
    "refresh_native",
    "skeleton_compute",
    "stage_identity",
    "use_backend",
)
