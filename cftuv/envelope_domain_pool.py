"""Пул процессов-воркеров для доменов очереди (срез PARALLEL-DOMAINS).

Домены независимы по ключу исполнения `(DecalRequestId, PatchDomainId)`:
подготовка, покрытие и контур одного домена не читают ничего из соседнего. На
поле (`building`, 122 домена) пять доменов дают 76 % времени, а пул процессов
по доменам даёт ПОБИТОВО те же исходы, статьи бюджета, счётчики и отпечатки
(`artifacts/parallel_domains_spike/`), поэтому этот модуль ничего не решает о
геометрии: он пересылает готовую задачу и возвращает готовый ответ. Задача
может нести и выгрузку снапшота домена (`DomainTaskV1.export`, см.
`envelope_export_input`): воркер выгружает `(snapshot, request)` сам, тем же
кодом, что и хост, и возвращает снапшот вместе с ответом. Либо только покрытие
готовой подготовки (`DomainTaskV1.coverage`, см. `envelope_queue_pool`): ей
воркер получает пикл подготовки и возвращает запись домена без неё. Либо
продуктовый путь на той же готовой подготовке (`DomainTaskV1.production`, см.
`envelope_production_export`): воркер считает покрытие и материализует
`GeometryBatchV1`, а отказ материализации приходит ОТВЕТОМ с названным исходом.
Либо ХОЛОДНЫЙ домен продуктового пути (`DomainTaskV1.cold`): подготовка и
материализация одной задачей, без промежуточного прогона отладочного вычислителя.

ПРИВЯЗКА ПЕРВОГО КРУГА (`plan_first_round`). Воркер держит память стадии резки (`cftuv_envelope.materialize.clip_memo`), и
попадание в неё возможно, только если домен снова попал к тому же воркеру. Задачи идут тяжёлыми первыми, и первые `N` из них
стартуют вместе на `N` воркерах в любом порядке, поэтому раздача этих `N` между воркерами длину прогона не меняет: задача с
`DomainTaskV1.affinity` идёт к воркеру, который считал её в прошлый раз (если он жив и не занят другой задачей первого круга).
Остальные задачи берут воркеры по очереди, как прежде. Это подсказка размещения: ответ от неё не зависит. Куда легла каждая задача с ключом привязки
относительно прошлого прогона (тому же воркеру, без живого владельца, чужому - кража), называет `PoolStatsV1` (`tasks_to_owner`, `tasks_unowned`, `tasks_stolen`).

ПАМЯТЬ ПОДГОТОВОК ВОРКЕРА (`envelope_worker_store`). Подготовка домена alpha-независима, а шаг ширины раньше слал воркерам пиклы всех подготовок
(8.65 МБ на `building`) и воркеры их разворачивали. Теперь воркер держит развёрнутые подготовки под ключом пикла (хеш байтов и кода), а
`DomainTaskV1.production`/`coverage` с непустым `key` уходят к воркеру БЕЗ пикла, если родитель знает, что тот подготовку держит (зеркало
`_Worker.held` повторяет вложения и вытеснение воркера); не держит - пикл идёт, держал и потерял - воркер отвечает `needs_blob`, и
родитель пересылает пикл. Холодная задача оставляет подготовку у воркера и отдаёт родителю пикл, снятый один раз (`_packed_cold_reply`):
первый шаг ширины после холодной кнопки не пересылает ничего. Свободный воркер берёт из очереди задачу, чью подготовку держит
(`_PendingTasks`), кроме крупных (`PULL_BIG_DIVISOR`): самая тяжёлая определяет длину прогона и не ждёт. Цену пересылок (байты туда и
обратно, пиклы и ключи, промахи, разбор ответов в родителе) называет `DomainPoolRunV1.stats`.

ПОЧЕМУ ПОДПРОЦЕССЫ, А НЕ `multiprocessing`. Внутри `blender.exe --python
script.py` стартовый метод `spawn` заново исполняет главный скрипт и падает на
`import bpy`. Воркер здесь — обычный `python -c`, который сам поднимает пакет
хоста по явно переданным путям и не исполняет ничего чужого.

ПРОТОКОЛ. Кадр — 8 байт длины (big-endian) и pickle протокола 5, по бинарным
stdin/stdout воркера. Внутри воркера fd 1 перенаправлен на stderr, поэтому
печать не портит кадры. Воркер живёт, пока открыт его stdin: закрытие родителя
закрывает трубу, и осиротевших процессов не остаётся.

ОТКАЗ НАЗЫВАЕТСЯ. Пул, который не стартовал, бросает `DomainPoolUnavailable` с
причиной текстом; задача, которая упала или чей воркер умер, возвращается
записью с `error`. Молча пропавшей задачи нет: вызывающий досчитывает её сам.

ИНТЕРПРЕТАТОР ВОРКЕРОВ. По умолчанию воркер идёт на том же Python, что и
родитель (во Blender — встроенный), с ПОЛНЫМ `sys.path` родителя. Настройка
«Worker Python» (`external_python`) отдаёт воркерам внешний CPython: на нём то же
ядро быстрее (замер в `DECISIONS.md`). Чужой интерпретатор не наследует ни
stdlib, ни site-packages родителя (они чужой версии): он стартует с `-I -S`, а
родитель передаёт только каталоги пакетов, которыми пользуется САМ — ядро,
`sympy`, `mpmath`, — и ставит их ПОСЛЕ собственной stdlib воркера, чтобы ни один
из них не затенил стандартную библиотеку. Ответ при этом обязан быть тем же, и
это проверяется, а не предполагается: воркер ПЕРВЫМ кадром (`hello`) называет
версии Python, `sympy`, `mpmath`, их числовой бэкенд и отпечаток исходников ядра,
родитель сверяет их со своими, и любое расхождение — именованный исход
`INTERPRETER_MISMATCH` (`INTERPRETER_UNUSABLE`, если интерпретатор не стартовал
вовсе). Отвергнутый внешний интерпретатор не отключает пул: воркеры идут на
встроенном, причина лежит в `PoolInterpreterV1` и доходит до панели.
"""

from __future__ import annotations

import atexit
import hashlib
import importlib
import importlib.util
import json
import os
import pickle
import queue
import re
import struct
import subprocess
import sys
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass, field, replace

from .envelope_gc_pause import gc_paused
from .envelope_kernel_backend import DEFAULT_KERNEL_BACKEND, resolve_density_backend, settle_stage_orders
from .envelope_pool_affinity import AFFINITY_NOT_EVALUATED, apply_worker_affinity
from .envelope_worker_store import STORE, PreparationLruV1, PreparationMissing, blob_key

#: Меньше двух воркеров — это последовательный путь, пула не заводится.
MIN_POOL_WORKERS = 2

#: Умолчание настройки: ни одного воркера сверх числа логических ядер (считается при импорте). Предел 12 (POOL_DEFAULT_12_V1) - число физических ядер
#: машины владельца; около 280 МБ пика на воркер. Настройка сцены `envelope_debug_workers` остаётся за владельцем: умолчание действует, где она не записана.
DEFAULT_POOL_WORKERS = min(12, os.cpu_count() or 1)

#: Сколько ждать готовности воркера (импорт ядра и sympy на холодном диске).
READY_TIMEOUT_SECONDS = 60.0

#: Сколько ключей привязки помнит пул (при переполнении забывает все): запись — строка и число.
MAX_AFFINITY_KEYS = 4096

FRAME_HEADER = struct.Struct(">Q")
PICKLE_PROTOCOL = 5
SPEC_ENVIRONMENT_VARIABLE = "CFTUV_DOMAIN_POOL_SPEC"
STDERR_TAIL_LINES = 40
DRAIN_TIMEOUT_SECONDS = 1.0

#: Во сколько раз снапшот с запросом больше лёгкого входа выгрузки того же
#: домена. Замер на `building` (121 домен, `artifacts/host_export_parallel/
#: rank_probe.py`): 1.76 МБ против 0.34 МБ, то есть 5.2; округлено до целого,
#: порядок задач от точности не зависит.
EXPORT_FRAME_COST_SCALE = 5

#: Во сколько раз кадр «только покрытие» (пикл подготовки) стоит МЕНЬШЕ снапшота
#: того же размера. Покрытие самого тяжёлого домена `building` — 2 с против ~20 с
#: полного решения при пикле 357 КБ и снапшоте ~83 КБ, то есть ~1/40 за байт;
#: округлено до степени двойки, чтобы произведение с длиной кадра было точным.
#: Нужна только смешанной партии (часть доменов в кэше подготовок, часть нет);
#: порядок меняет стену, но не ответ.
COVERAGE_FRAME_COST_DIVISOR = 32

#: Внешний интерпретатор воркеров отвергнут, и воркеры идут на встроенном. Имена
#: исходов — и записи диагностики, и счётчики панели: «не стартовал» и «стартовал,
#: но отвечал бы не тем же» чинятся по-разному, поэтому и называются по-разному.
INTERPRETER_UNUSABLE = "ENVELOPE_DOMAIN_POOL_INTERPRETER_UNUSABLE"
INTERPRETER_MISMATCH = "ENVELOPE_DOMAIN_POOL_INTERPRETER_MISMATCH"

#: Причина отказа внешнему интерпретатору: код — индекс (0 — отказа нет). Коды до
#: `FIRST_MISMATCH_REASON` — UNUSABLE, начиная с него — MISMATCH. Коды идут в
#: числовые счётчики профиля (строки в них не кладутся), текст панели берёт их
#: отсюда, а полный текст причины с числами лежит в диагностике.
INTERPRETER_REASONS = (
    "",
    "path is not a usable Python executable",
    "worker did not start or did not identify itself",
    "host cannot locate its kernel, sympy or mpmath",
    "Python is older than 3.10",
    "sympy version differs",
    "mpmath version differs",
    "numeric backend differs",
    "kernel source differs",
)
FIRST_MISMATCH_REASON = 4
MIN_EXTERNAL_PYTHON = (3, 10)

#: Пакеты, которые воркер на внешнем интерпретаторе берёт у РОДИТЕЛЯ: чистый
#: Python, версии у них одни, а побитовое равенство ответа держится на том, что
#: это те же самые файлы, а не «такая же версия» из чужого site-packages.
HOST_PACKAGES = ("cftuv_envelope", "sympy", "mpmath")

#: Пакеты, которые внешний воркер берёт у родителя, ЕСЛИ родитель их нашёл: отсутствие не отказ интерпретатору (ответ даёт Python-эталон),
#: а `NATIVE_UNAVAILABLE` на каждом домене, заказавшем нативный бэкенд (`cftuv_envelope.backend`). Колесо `cftuv_native` ставится в
#: каталог модулей родителя (`tools/install_native_to_blender.ps1`), и воркер видит его оттуда же, откуда ядро и `sympy`.
OPTIONAL_HOST_PACKAGES = ("cftuv_native",)

# Запускается как `python -u -c`. Пакет хоста поднимается по файлу под ТЕМ ЖЕ
# именем, под которым он загружен у родителя: в Blender 4.2+ это
# `bl_ext.<репозиторий>.<id>`, которого нет ни в каком `sys.path`, и pickle
# иначе не нашёл бы классы по `__module__`. Пути родителя встают ПЕРЕД путями
# воркера (встроенный интерпретатор: тот же `sys.path`, что и у родителя) либо
# ПОСЛЕ них (`after_stdlib`: внешний, чьи stdlib и site родитель не знает).
_BOOTSTRAP = """
import importlib, importlib.util, json, os, sys
spec = json.loads(os.environ[{variable!r}])
parent = [item for item in spec["sys_path"] if item]
own = [item for item in sys.path if item and item not in parent]
sys.path[:] = own + parent if spec.get("after_stdlib") else parent + own
root = spec["package_dir"]
package = importlib.util.spec_from_file_location(
    spec["package"],
    os.path.join(root, "__init__.py"),
    submodule_search_locations=[root],
)
module = importlib.util.module_from_spec(package)
sys.modules[spec["package"]] = module
package.loader.exec_module(module)
importlib.import_module(spec["package"] + ".envelope_domain_pool").worker_main()
""".format(variable=SPEC_ENVIRONMENT_VARIABLE)


class DomainPoolUnavailable(RuntimeError):
    """Пул не может работать; текст — причина, которую читает владелец."""


@dataclass(frozen=True, slots=True)
class DomainTaskV1:
    """Один домен очереди: всё, что нужно воркеру, и ничего сверх.

    `export` (`HostExportInputV1`) — домен, чей снапшот в кэше метрики сессии
    отсутствует: воркер выгружает `(snapshot, request)` сам, и `snapshot` с
    `request` тогда `None`. Иначе оба пришли готовыми из родителя.

    `coverage` (`CoverageInputV1`) — домен, чья подготовка уже лежит в кэше
    сессии: воркер считает только покрытие и запись хоста, а `snapshot` с
    `request` тогда `None`.

    `production` (`ProductionInputV1`) — продуктовый путь на той же готовой
    подготовке: воркер считает покрытие и материализует `GeometryBatchV1`
    (`envelope_production_export`), а `snapshot` с `request` тогда `None`.

    `cold` (`ColdProductionInputV1`) — продуктовый путь на домене БЕЗ подготовки:
    воркер готовит её и сразу материализует (`prepare -> produce_domain`), а вход
    берёт как задача с выгрузкой (`export`) либо как задача с готовыми `snapshot` и
    `request`. Подготовка возвращается ответом: у родителя её ещё нет.
    """

    task_id: int
    patch_id: int
    domain_id: str
    snapshot: object
    request: object
    alpha_text: str
    selected_edges: frozenset
    export: object | None = None
    coverage: object | None = None
    production: object | None = None
    cold: object | None = None
    #: Ключ привязки к воркеру (`plan_first_round`): задача с тем же ключом идёт к тому же воркеру, если он жив; пусто — без привязки.
    affinity: str = ""
    #: ГЛАВНЫЙ переключатель ядра воркера для домена продуктового пути (`PYTHON` | `NATIVE`, `envelope_kernel_backend`): ответ от него не зависит;
    #: запись «кто посчитал на самом деле и какой названный откат» приходит в ответе домена (`backend_record`). Умолчание — продуктовое.
    backend: str = DEFAULT_KERNEL_BACKEND
    #: Постадийные порядки воркера (API): `None` — как `backend`, и `__post_init__` приводит поле к имени (`settle_stage_orders`). Скелет считается в подготовке (холодная
    #: задача, задача очереди), а не в материализации, и воркер ставит блок бэкенда вокруг подготовки (`prepare_for_production_recorded`, `run_queue_domain`); сертификат
    #: вложения привязки источника считается в экспорте, подготовке и материализации.
    skeleton_backend: str | None = None
    embedding_backend: str | None = None
    density_backend: str | None = field(default=None, kw_only=True)

    def __post_init__(self) -> None:
        settle_stage_orders(self)
        object.__setattr__(self, "density_backend", resolve_density_backend(self.backend, self.density_backend))


@dataclass(frozen=True, slots=True)
class DomainTaskResultV1:
    """Ответ воркера: `error` непуст, либо `refusal`, либо подготовка и запись.

    `refusal` — ИМЕНОВАННЫЙ отказ выгрузки домена (`ExportRefusalV1`): это
    ответ, а не сбой, и родитель разбирает его как разобрал бы сам. Снапшот
    `snapshot` есть у задачи с выгрузкой в воркере, когда выгрузка удалась;
    `export_timings` и `export_counters` — её стадии, которые родитель
    проигрывает в профиль кнопки. `production` — ответ продуктового пути
    (`ProductionDomainResultV1`): отказ материализации там названный исход, а
    не сбой задачи. У холодного домена ответ несёт и подготовку (`prepared`), и снапшот
    выгрузки, если выгружал воркер.
    """

    task_id: int
    prepared: object | None = None
    queue_domain: object | None = None
    error: str = ""
    snapshot: object | None = None
    refusal: object | None = None
    export_timings: tuple = ()
    export_counters: tuple = ()
    production: object | None = None
    #: Воркер не держит подготовку с присланным ключом (задача пришла без пикла): задачи не было, родитель пересылает пикл.
    needs_blob: bool = False
    #: Подготовка холодного домена ПИКЛОМ, который воркер снял сам (`prepared` тогда `None`), и ключ этого пикла в памяти воркера:
    #: воркер оставил у себя ту же подготовку, родитель берёт пикл как блоб этой подготовки (`LazyPreparationV1`, `PreparationBlobsV1`)
    #: и НЕ разворачивает его, пока живой объект не понадобится, - первый шаг ширины после холодной кнопки не пересылает и не снимает пиклов.
    prepared_blob: bytes | None = None
    prepared_key: str = ""
    #: `band_facts_of(снапшот подготовки)`, снятые воркером: всё, что родитель читает в подготовке без разворота; `None` - не сняты.
    prepared_facts: tuple | None = None
    #: `((ключ, размер), ...)` подготовок, которые воркер положил в память, пока считал эту задачу: зеркало родителя повторяет это.
    stored: tuple = ()
    #: Запись экспорта, если он завершился до production (отказ либо ошибка).
    backend_record: object | None = None
    #: `SnapshotCleanV1` (`envelope_snapshot_check`): воркер проверил `snapshot` и замечаний нет. Едет ТОЛЬКО со снапшотом, который выгрузил воркер.
    snapshot_check: object | None = None
    #: REQUEST_CARRIED_IN_REPLY_V1: запрос, который воркер собрал и проверил на ЭТОМ снапшоте (`DecalRequestV1`); едет в том же кадре, что и снапшот (общие объекты остаются общими), и только вместе с ним.
    #: Родитель берёт его вместо второй сборки того же запроса (0.14 с на `cover.008`); нет запроса (отказ сборки, снапшот родителя) - родитель собирает сам, как прежде.
    request: object | None = None

    @property
    def ok(self) -> bool:
        return not self.error and (
            self.queue_domain is not None or self.production is not None
        )

    @property
    def refused(self) -> bool:
        return not self.error and self.refusal is not None


@dataclass(frozen=True, slots=True)
class PoolInterpreterV1:
    """Интерпретатор, на котором РАБОТАЮТ воркеры, и почему не тот, что заказан.

    `external` — воркеры на заказанном внешнем интерпретаторе. Иначе они на
    встроенном, и если внешний был заказан, `outcome` называет, почему он
    отвергнут (`INTERPRETER_UNUSABLE` либо `INTERPRETER_MISMATCH`), `reason` —
    полный текст с числами, `reason_code` — индекс в `INTERPRETER_REASONS`.
    """

    version: tuple
    external: bool
    outcome: str = ""
    reason: str = ""
    reason_code: int = 0

    @property
    def version_code(self) -> int:
        major, minor, micro = (tuple(self.version) + (0, 0, 0))[:3]
        return major * 10000 + minor * 100 + micro

    @property
    def version_text(self) -> str:
        return ".".join(str(item) for item in self.version)


@dataclass(frozen=True, slots=True)
class PoolStatsV1:
    """Что стоил прогон по трубам: байты, пересылки подготовок и разбор ответов в родителе.

    `blob_hits` — задачи, ушедшие к воркеру одним ключом (воркер держал подготовку); `blobs_shipped` и `blob_bytes_shipped` — задачи,
    у которых ушёл пикл (первая пересылка, воркер не держал, либо промах); `blob_misses` — из них те, где родитель считал, что
    воркер подготовку держит, а он её не держал (вытеснение, расхождение): ответ тот же, цена — один лишний обмен.
    `unpickle_cpu_seconds` и `unpickle_wall_seconds` — разбор кадров ответов потоками-читателями родителя (CPU потока и стена с ожиданием GIL);
    пикл подготовки холодного домена в кадре остаётся байтами и сюда не входит (его разворот считает сессия, `lazy_unpickled`).
    """

    bytes_sent: int = 0
    bytes_received: int = 0
    blob_hits: int = 0
    blobs_shipped: int = 0
    blob_bytes_shipped: int = 0
    blob_misses: int = 0
    unpickle_cpu_seconds: float = 0.0
    unpickle_wall_seconds: float = 0.0
    #: Куда легли задачи с ключом привязки относительно ПРОШЛОГО прогона: к воркеру, считавшему их в прошлый раз; без живого владельца (новый домен,
    #: воркер умер); к другому воркеру (кража в хвосте прогона: подготовки у вора может не быть, и тогда полный путь `NO_CERTIFICATE`).
    tasks_to_owner: int = 0
    tasks_unowned: int = 0
    tasks_stolen: int = 0
    #: HEAD_STREAMED_FIRST_ROUND_V1: прогон начал первый круг по отбору кандидатов (1), и сколько задач остатка тяжелее самой лёгкой задачи первого круга (0 - первый круг тот же, что дал бы полный порядок).
    head_streamed: int = 0
    head_misses: int = 0


@dataclass(frozen=True, slots=True)
class LateTasksV1:
    """Задачи, которые вызывающий достроит ПОКА воркеры считают (SCAN_BEHIND_THE_POOL_V1): `total` - сколько задач будет всего (решает, идёт ли прогон по отбору кандидатов), `provide()` - остаток задач,
    его зовёт поток вызывающего после старта воркеров (один раз); задачи с теми же `task_id`, что у полного списка. Исключение `provide` вызывающий ловит сам и отдаёт пустой остаток: прогон кончается, а отказ он бросает после."""

    total: int
    provide: object


@dataclass(frozen=True, slots=True)
class DomainPoolRunV1:
    """Итог прогона: ответы по `task_id` и сколько воркеров в нём участвовало.

    Задачи, которых в `results` нет, не исполнялись вовсе (все воркеры умерли
    раньше, чем до них дошла очередь). `interpreter` — на чём шёл прогон, `stats` — цена по трубам.
    """

    results: dict
    workers: int
    interpreter: PoolInterpreterV1 | None = None
    stats: PoolStatsV1 | None = None
    #: Код привязки воркеров к физическим ядрам (`envelope_pool_affinity.AFFINITY_*`): 1 - маска поставлена, иначе почему воркеры идут без неё.
    affinity: int = AFFINITY_NOT_EVALUATED


# --------------------------------------------------------------------------
# Кадры
# --------------------------------------------------------------------------


def encode_frame(value) -> bytes:
    payload = pickle.dumps(value, protocol=PICKLE_PROTOCOL)
    return FRAME_HEADER.pack(len(payload)) + payload


def _read_exactly(stream, size: int) -> bytes:
    chunks = []
    remaining = size
    while remaining:
        chunk = stream.read(remaining)
        if not chunk:
            break
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def read_frame_bytes(stream) -> bytes | None:
    """Полезная нагрузка следующего кадра (без разбора) либо `None` на чистом конце потока.

    Обрыв посреди кадра - `EOFError` (см. `read_frame_sized`).
    """

    header = _read_exactly(stream, FRAME_HEADER.size)
    if not header:
        return None
    if len(header) < FRAME_HEADER.size:
        raise EOFError("truncated frame header")
    (length,) = FRAME_HEADER.unpack(header)
    payload = _read_exactly(stream, length)
    if len(payload) < length:
        raise EOFError(f"truncated frame: {len(payload)} of {length} bytes")
    return payload


def read_frame_sized(stream, between=None):
    """`(объект, длина кадра, CPU разбора, стена разбора)` либо `None` на чистом конце потока.

    Обрыв посреди кадра - не конец, а `EOFError`: отличить «воркер закончил» от
    «воркер умер на записи» иначе нечем. Разбор (`pickle.loads`) измерен отдельно от чтения трубы. `between()` зовут, когда байты кадра
    прочитаны, а разбор ещё не начат (EARLY_DISPATCH_BEFORE_UNPICKLE_V1): время вызова в разбор не входит.
    """

    payload = read_frame_bytes(stream)
    if payload is None:
        return None
    if between is not None:
        between()
    cpu, wall = time.thread_time(), time.perf_counter()
    value = pickle.loads(payload)
    return value, len(payload), time.thread_time() - cpu, time.perf_counter() - wall


def read_frame(stream):
    """Следующий объект потока либо `None` на чистом конце потока (см. `read_frame_sized`)."""

    sized = read_frame_sized(stream)
    return None if sized is None else sized[0]


def write_frame(stream, value) -> None:
    stream.write(encode_frame(value))
    stream.flush()


def order_by_cost(tasks) -> list[tuple[DomainTaskV1, bytes]]:
    """Пары `(задача, готовый кадр)`, самые тяжёлые первыми.

    Цена — размер кадра: снапшот домена растёт с числом его граней и рёбер, а
    время домена — с тем же (на поле 83 КБ у пяти дорогих доменов против
    медианы 10 КБ). Порядок важен ровно затем, что потолок ускорения — самый
    тяжёлый домен: поставь его последним, и остальные воркеры ждут его хвост.
    Равные по цене идут в порядке `task_id`, чтобы порядок был воспроизводим.

    Задача с выгрузкой в воркере шлёт лёгкий вход вместо снапшота, и кадр её
    в `EXPORT_FRAME_COST_SCALE` раз меньше: цена приводится к единицам снапшота,
    иначе прогон, где часть доменов в кэше метрики, ставил бы тяжёлый домен с
    готовым снапшотом перед ещё более тяжёлым, которого кадр прячет. Кадр с
    пиклом подготовки, наоборот, тяжелее своей работы (`COVERAGE_FRAME_COST_
    DIVISOR`).
    """

    framed = [(task, encode_frame(task)) for task in tasks]
    framed.sort(key=lambda item: (-_frame_cost(*item), item[0].task_id))
    return framed


def plan_first_round(ordered, workers, last_worker) -> dict[int, int]:
    """`{индекс воркера: позиция в ordered}`: первая задача каждого воркера.

    `ordered` — пары `(задача, кадр)`, тяжёлые первыми; `workers` — индексы воркеров прогона; `last_worker` — `{ключ привязки:
    индекс воркера}` прошлых прогонов. Первые `len(workers)` задач стартуют одновременно, поэтому между воркерами их можно
    раздать как угодно без потери в длине прогона; задача, чей прежний воркер среди `workers` и ещё не взят, идёт к нему,
    остальные — свободным воркерам по порядку.
    """

    free = list(workers)
    plan: dict[int, int] = {}
    unmatched: list[int] = []
    for position in range(min(len(free), len(ordered))):
        affinity = ordered[position][0].affinity
        owner = last_worker.get(affinity) if affinity else None
        if owner in free:
            plan[owner] = position
            free.remove(owner)
        else:
            unmatched.append(position)
    plan.update(zip(free, unmatched))
    return plan


#: HEAD_STREAMED_FIRST_ROUND_V1. Полный порядок (`order_by_cost`) кодирует кадр КАЖДОЙ задачи: на холодной `cover.008` 0.155 с, и все воркеры стоят без дела, пока он не кончится (первая отправка на 0.155 с прогона).
#: Первый круг нужен только как `N` самых тяжёлых задач, и его можно отдать раньше: кандидатов (`HEAD_CANDIDATES_PER_WORKER * N` с наибольшим числом цепочек - число считается без кодирования, 0.4 мс на 1051 задачу)
#: кодируют и упорядочивают точно, первые `N` из них уходят воркерам сразу (первая отправка на 0.016 с), а остаток упорядочивается, пока воркеры считают: стена пула холодной `cover.008` -0.09 с. Если точные `N` самых
#: тяжёлых среди всех лежат среди кандидатов (на `cover.008` так: точная двенадцатка внутри первых двадцати четырёх по числу цепочек), порядок ВСЕХ задач тот же, что у полного. Иначе тяжёлая задача идёт чуть позже
#: своего места, но в голове остатка: это порядок запуска, ответ он не меняет (`PoolStatsV1.head_misses` считает такие задачи). Только когда задач не меньше `STREAMED_HEAD_MIN_TASKS_PER_WORKER * N` (иначе порядок
#: дёшев) и у каждой есть вход выгрузки (`export`): число цепочек есть только у него.
HEAD_CANDIDATES_PER_WORKER = 2
STREAMED_HEAD_MIN_TASKS_PER_WORKER = 8


def split_first_round(tasks, workers: int):
    """`(первый круг, остаток)`: первый круг - пары `(задача, кадр)` ровно `workers` самых тяжёлых среди кандидатов по числу цепочек (тяжёлые первыми); остаток - задачи без кадров, в порядке `tasks`."""

    candidates = sorted(tasks, key=lambda task: (-len(task.export.host_chains), task.task_id))[: HEAD_CANDIDATES_PER_WORKER * workers]
    head = order_by_cost(candidates)[:workers]
    taken = {task.task_id for task, _frame in head}
    return head, [task for task in tasks if task.task_id not in taken]


def _frame_cost(task: DomainTaskV1, frame: bytes) -> float:
    if task.export is not None:
        return len(frame) * EXPORT_FRAME_COST_SCALE
    if task.coverage is not None or task.production is not None:
        return len(frame) / COVERAGE_FRAME_COST_DIVISOR
    return float(len(frame))


# --------------------------------------------------------------------------
# Воркер
# --------------------------------------------------------------------------


def solve_task(task: DomainTaskV1) -> DomainTaskResultV1:
    """Домен целиком, как его считает последовательный путь (без профиля).

    Отделено от цикла воркера затем, что та же функция служит пулу в процессе в
    тестах: склейка фаз проверяется без подпроцессов, а сам подпроцесс — одним
    настоящим прогоном.
    """

    try:
        if task.cold is not None:
            from .envelope_production_export import solve_cold_production_task

            return solve_cold_production_task(task)
        if task.export is not None:
            from .envelope_export_input import solve_exported_task

            return solve_exported_task(task)
        if task.coverage is not None:
            from .envelope_queue_pool import solve_coverage_task

            return solve_coverage_task(task)
        if task.production is not None:
            from .envelope_production_export import solve_production_task

            return solve_production_task(task)
        from .envelope_queue_export import run_queue_domain

        prepared, domain = run_queue_domain(
            task.patch_id,
            task.domain_id,
            task.snapshot,
            task.request,
            task.alpha_text,
            selected_edges=task.selected_edges,
            profile=None,
            backend=task.backend,
            skeleton_backend=task.skeleton_backend,
            density_backend=task.density_backend,
        )
        return DomainTaskResultV1(
            task.task_id, prepared, replace(domain, preparation=None)
        )
    except PreparationMissing:
        # Задача пришла одним ключом, а подготовки в памяти воркера нет: это не сбой, а просьба прислать пикл.
        return DomainTaskResultV1(task.task_id, needs_blob=True)
    except Exception:  # noqa: BLE001 - ответ несёт трассу, а не теряет её
        return DomainTaskResultV1(task.task_id, error=traceback.format_exc())


def _packed_cold_reply(task: DomainTaskV1, result: DomainTaskResultV1) -> DomainTaskResultV1:
    """Ответ холодного домена с подготовкой пиклом, снятым ОДИН раз, и той же подготовкой в памяти воркера.

    Подготовка нужна родителю целиком (кэш сессии), а шаг ширины потом шлёт воркерам её пикл: родитель берёт этот же пикл, и перепикливать
    ему нечего, а воркер держит ровно ту подготовку, которую пикл развернёт (бюджет снят в этот же момент, память точных предикатов -
    чистые кэши). Ключа не построить (ядро не импортируется) - ответ остаётся прежним, с объектом.
    """

    if task.cold is None or result.prepared is None or result.production is None:
        return result
    blob = pickle.dumps(result.prepared, protocol=PICKLE_PROTOCOL)
    key = blob_key(blob)
    if not key:
        return result
    STORE.put(key, result.prepared, len(blob))
    stored = ((key, len(blob)),) if key in STORE else ()
    try:
        from .envelope_chart_band import band_facts_of

        facts = band_facts_of(result.prepared.context.snapshot)
    except Exception:  # noqa: BLE001 - без фактов родитель разворачивает пикл, когда факты нужны: ответ тот же
        facts = None
    return replace(result, prepared=None, prepared_blob=blob, prepared_key=key, stored=stored, prepared_facts=facts)


def _reply_frame(task_id: int, result: DomainTaskResultV1) -> bytes:
    try:
        return encode_frame(result)
    except Exception:  # noqa: BLE001 - непиклящийся ответ тоже названный отказ
        return encode_frame(
            DomainTaskResultV1(task_id, error=traceback.format_exc())
        )


def serve_task(task: DomainTaskV1) -> bytes:
    """Кадр ответа воркера на задачу: решение, упаковка подготовки и кадр идут под паузой циклического сборщика (GC_PAUSED_DURING_BUTTON_V1).

    Воркер платил за сборщик ~16 процессорных секунд на холодной кнопке `cover.008` (восемь воркеров, ~2 с на воркер), хотя задача держит в памяти только то, что
    ещё нужно ей самой. Пауза кончается вместе с кадром: сборка идёт между задачами, а не внутри них. Состояние сборщика после блока прежнее (`gc_paused`),
    при любом выходе. Запись в трубу и ожидание следующей задачи - вне паузы. Ответ от паузы не зависит.
    """

    with gc_paused():
        return _reply_frame(task.task_id, _packed_cold_reply(task, solve_task(task)))


def serve_message(message) -> bytes:
    """Кадр ответа на сообщение, которое не задача домена: шаг фоновой проверки (`ByteCheckStepV1`, SNAPSHOT_BYTE_CHECK_FIRST_PRESS_IN_BACKGROUND_V1).

    Шаг идёт под паузой сборщика, как задача. Чужое сообщение - названный отказ ответом (`DomainTaskResultV1.error`), а не молчание: родитель узнает протокол по типу ответа.
    """

    from .envelope_snapshot_byte_check import ByteCheckStepV1, serve_byte_check

    try:
        if isinstance(message, ByteCheckStepV1):
            with gc_paused():
                return _reply_frame(-1, serve_byte_check(message))
        return _reply_frame(-1, DomainTaskResultV1(-1, error=f"unknown worker message: {type(message).__name__}"))
    except Exception:  # noqa: BLE001 - ответ несёт трассу, а не теряет её
        return _reply_frame(-1, DomainTaskResultV1(-1, error=traceback.format_exc()))


# --------------------------------------------------------------------------
# Тождество окружения: что сверяют родитель и воркер на внешнем интерпретаторе
# --------------------------------------------------------------------------


def kernel_fingerprint(directory: str) -> str:
    """SHA-256 исходников пакета ядра: относительные имена и байты файлов.

    `__pycache__` не входит: байт-код — следствие исходника и версии Python, а
    сверяется именно исходник. Читают оба конца по одному и тому же файлу на
    диске, поэтому окончания строк и права доступа от отпечатка не зависят.
    """

    digest = hashlib.sha256()
    for folder, subfolders, files in os.walk(directory):
        subfolders[:] = sorted(
            item for item in subfolders if item != "__pycache__"
        )
        for name in sorted(files):
            if name.endswith((".pyc", ".pyo")):
                continue
            path = os.path.join(folder, name)
            relative = os.path.relpath(path, directory).replace(os.sep, "/")
            digest.update(relative.encode("utf-8") + b"\0")
            with open(path, "rb") as handle:
                digest.update(handle.read())
            digest.update(b"\0")
    return digest.hexdigest()


def package_directory(name: str) -> str | None:
    """Каталог пакета на пути ЭТОГО процесса, не импортируя его; иначе `None`."""

    try:
        spec = importlib.util.find_spec(name)
    except (ImportError, ValueError):
        return None
    origin = None if spec is None else spec.origin
    if not origin or not os.path.isfile(origin):
        return None
    return os.path.dirname(os.path.abspath(origin))


def _module_version(name: str) -> str:
    try:
        return str(importlib.import_module(name).__version__)
    except Exception as exc:  # noqa: BLE001 - отсутствие пакета — тоже факт
        return f"missing: {type(exc).__name__}"


def _numeric_backend() -> str:
    """Бэкенд арифметики `sympy`/`mpmath` (`python` либо `gmpy`): ответ от него
    не зависит, а время зависит, и сверка не даёт одному концу быть быстрее
    другого по причине, о которой никто не знает."""

    names = []
    for module, attribute in (
        ("sympy.external.gmpy", "GROUND_TYPES"),
        ("mpmath.libmp", "BACKEND"),
    ):
        try:
            names.append(str(getattr(importlib.import_module(module), attribute)))
        except Exception:  # noqa: BLE001 - неизвестное совпадает с неизвестным
            names.append("unknown")
    return f"sympy:{names[0]}/mpmath:{names[1]}"


def describe_environment() -> dict:
    """Тождество окружения ЭТОГО процесса: версии, бэкенд, отпечаток ядра."""

    kernel = package_directory("cftuv_envelope")
    return {
        "python": tuple(sys.version_info[:3]),
        "executable": sys.executable,
        "sympy": _module_version("sympy"),
        "mpmath": _module_version("mpmath"),
        "numeric_backend": _numeric_backend(),
        "kernel_path": kernel or "",
        "kernel_fingerprint": "" if kernel is None else kernel_fingerprint(kernel),
        # Откуда загружены пакеты: диагностика, а не предмет сверки (версия не
        # отличает «те же файлы» от «такой же версии» из чужого каталога).
        "sympy_path": package_directory("sympy") or "",
        "mpmath_path": package_directory("mpmath") or "",
    }


def _identity_difference(host: dict, worker: dict) -> tuple[int, str] | None:
    """Первое расхождение окружений: `(код причины, текст с числами)` либо `None`.

    Версию Python не сравнивают — она различается нарочно, — а требуют от неё
    минимума. Остальное обязано совпасть: ответ ядра держится на тех же файлах
    ядра и той же арифметике.
    """

    try:
        python = tuple(worker.get("python") or ())
        too_old = python < MIN_EXTERNAL_PYTHON
    except TypeError:
        python, too_old = (), True
    if too_old:
        found = ".".join(str(item) for item in python) or "unknown"
        return 4, f"Python {found} is older than 3.10"
    for code, key in (
        (5, "sympy"),
        (6, "mpmath"),
        (7, "numeric_backend"),
        (8, "kernel_fingerprint"),
    ):
        if worker.get(key) != host.get(key):
            return code, (
                f"{INTERPRETER_REASONS[code]}: worker {worker.get(key)!r}, "
                f"host {host.get(key)!r}"
            )
    return None


#: Исход прогрева воркера: отпечатки кода и бэкенды считаются ДО «готов», а не на первой задаче (WORKER_IDENTITY_PREWARM_V1).
WORKER_IDENTITY_PREWARMED = "ENVELOPE_DOMAIN_POOL_WORKER_IDENTITY_PREWARMED"
WORKER_IDENTITY_PREWARM_UNAVAILABLE = "ENVELOPE_DOMAIN_POOL_WORKER_IDENTITY_PREWARM_UNAVAILABLE"


def warm_worker_identities() -> dict:
    """Отпечатки кода и идентичность бэкендов ЭТОГО процесса, посчитанные заранее; `{"outcome": ..., "seconds": ...}` уходит в кадр «готов».

    ЗАЧЕМ. Первая задача каждого воркера платила ~0.2-0.3 с одноразовой работы, которая не зависит от домена: отпечаток кода хоста и ядра
    (`envelope_content_key.code_identity`, sha256 по ~290 файлам: его берёт проверка снапшота и ключ пикла), отпечаток ядра для памяти резки
    (`materialize.clip_memo.kernel_code_identity`, тот же закон, но свой кэш) и импорт модулей нативного расширения (идентичность бэкенда).
    На холодной кнопке первые задачи - самые тяжёлые домены, поэтому это лежало на критическом пути стены `building` (12 x 0.2 с процессорных). Теперь это
    считается в старте воркера, который прячет прогрев пула при регистрации аддона. Значения те же: кэши процесса, результат от порядка не зависит.

    Сбой - названный исход в кадре и строка в stderr воркера, а не падение: первая задача посчитает недостающее сама (ответ тот же).
    """

    started = time.perf_counter()
    try:
        from .envelope_content_key import execution_identity

        execution_identity(DEFAULT_KERNEL_BACKEND)  # отпечаток ядра и хоста (кэш процесса) и идентичность бэкенда: нативное расширение импортируется здесь
        from cftuv_envelope.backend import native_status
        from cftuv_envelope.materialize.clip_memo import kernel_code_identity

        kernel_code_identity()
        native_status()  # расширение и проверка закреплённых файлов: кэш процесса, как у первой задачи
        from .envelope_kernel_backend import stage_orders

        if stage_orders(DEFAULT_KERNEL_BACKEND).snapshot_check == "NATIVE":
            from cftuv_envelope.backend import warm_snapshot_check

            warm_snapshot_check()  # привязка нативной проверки снапшота (дайджесты функций эталона, таблица классов): разовые десятки мс, не на первом домене
        if stage_orders(DEFAULT_KERNEL_BACKEND).frame == "NATIVE":
            from cftuv_envelope.backend import warm_frame

            warm_frame()  # привязка нативного допуска рамки (дайджесты функций контрактов, таблица классов): разовые десятки мс, не на первом домене
        outcome = WORKER_IDENTITY_PREWARMED
    except Exception as exc:  # noqa: BLE001 - прогрев не ломает воркера: именованный исход и строка в stderr
        outcome = WORKER_IDENTITY_PREWARM_UNAVAILABLE
        print(f"[CFTUV][EnvelopeDomainPool] {outcome}: {type(exc).__name__}: {exc}; the first task computes it", flush=True)
    return {"outcome": outcome, "seconds": time.perf_counter() - started}


def worker_main() -> None:
    """Цикл воркера: задача на входе, ответ на выходе, до конца stdin."""

    channel = os.fdopen(os.dup(1), "wb")
    os.dup2(2, 1)
    sys.stdout = sys.stderr
    source = sys.stdin.buffer
    if json.loads(os.environ.get(SPEC_ENVIRONMENT_VARIABLE) or "{}").get(
        "handshake"
    ):
        # До тяжёлой загрузки ядра: расхождение окружений родитель называет
        # сразу, а не после секунд импорта (и не как «воркер упал»).
        identity = {**describe_environment(), "sys_path": list(sys.path)}
        write_frame(channel, ("hello", identity))
    from .envelope_export_input import load_export_modules
    from .envelope_queue_export import load_queue_kernel

    # Живой след стадий пишет в stdout, то есть в stderr воркера: читать его
    # некому, а профиль кнопки получает те же секунды значением, ответом.
    # `from . import x` не годится: под чужим именем пакета (`bl_ext.*`) он
    # просит у импорта родителя, которого в воркере нет.
    importlib.import_module(
        ".envelope_debug_profile", __package__
    ).LIVE_STAGE_TRACE = False
    load_queue_kernel()
    load_export_modules()
    # Склейка покрытия и продуктовый путь поднимаются до «готов», а не на
    # первой задаче.
    importlib.import_module(".envelope_queue_pool", __package__)
    importlib.import_module(".envelope_production_export", __package__)
    write_frame(channel, ("ready", os.getpid(), warm_worker_identities()))
    while True:
        task = read_frame(source)
        if task is None:
            return
        channel.write(serve_message(task) if not isinstance(task, DomainTaskV1) else serve_task(task))
        channel.flush()


# --------------------------------------------------------------------------
# Родитель
# --------------------------------------------------------------------------


class _WorkerGone(RuntimeError):
    """Воркер умер либо поток его ответов оборвался."""


_GONE = object()


class _Worker:
    """Один подпроцесс и два потока-читателя его труб."""

    def __init__(self, index: int, command, environment) -> None:
        self.index = index
        self.dead = False
        #: Что воркер назвал о себе в `hello` (внешний интерпретатор), иначе пусто.
        self.identity: dict = {}
        #: Прогрев воркера из кадра «готов» (`warm_worker_identities`): исход и секунды; пусто, если воркер его не прислал.
        self.prewarm: dict = {}
        #: Зеркало памяти подготовок воркера (те же вложения и порядок вытеснения): ключ без пикла уходит, если воркер его держит.
        self.held = PreparationLruV1()
        #: Сколько байтов ответов прочитано у воркера и во что обошёлся их разбор (потоком-читателем родителя); растёт монотонно.
        self.received_bytes = 0
        self.unpickle_cpu = 0.0
        self.unpickle_wall = 0.0
        #: Зовётся потоком-читателем, когда байты КАЖДОГО ответа прочитаны, а разбор ещё не начат (`DomainPool._run_locked` ставит его на прогон и снимает): следующая задача уходит воркеру до разбора ответа.
        self.on_reply_bytes = None
        self._stdout_closed = False
        self._inbox: queue.Queue = queue.Queue()
        self._stderr_tail: deque = deque(maxlen=STDERR_TAIL_LINES)
        flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=environment,
            creationflags=flags,
        )
        self._stderr_thread = threading.Thread(
            target=self._pump_stderr, daemon=True
        )
        self._stderr_thread.start()
        threading.Thread(target=self._pump_frames, daemon=True).start()

    def _pump_frames(self) -> None:
        try:
            while True:
                sized = read_frame_sized(self.process.stdout, self._reply_bytes_read)
                if sized is None:
                    break
                frame, length, cpu, wall = sized
                self.received_bytes += length
                self.unpickle_cpu += cpu
                self.unpickle_wall += wall
                self._inbox.put(frame)
        except Exception as exc:  # noqa: BLE001 - обрыв кадра == смерть воркера
            self._inbox.put(_WorkerGone(f"{type(exc).__name__}: {exc}"))
        self._stdout_closed = True
        self._inbox.put(_GONE)

    def _reply_bytes_read(self) -> None:
        hook = self.on_reply_bytes
        if hook is not None:
            hook(self)

    def _pump_stderr(self) -> None:
        try:
            for line in self.process.stderr:
                self._stderr_tail.append(line.decode("utf-8", "replace").rstrip())
        except (OSError, ValueError):
            pass

    def describe(self) -> str:
        code = self.process.poll()
        if code is None and self._stdout_closed:
            # Поток ответов кончился, а процесс ещё не пожат: дать ему уйти.
            try:
                code = self.process.wait(timeout=DRAIN_TIMEOUT_SECONDS)
            except subprocess.TimeoutExpired:
                pass
        if code is not None:
            # Причина смерти лежит в stderr, а его читает отдельный поток: без
            # ожидания диагностика уходила бы раньше, чем трасса в неё попала.
            self._stderr_thread.join(timeout=DRAIN_TIMEOUT_SECONDS)
        state = "still running" if code is None else f"exit code {code}"
        tail = " | ".join(line for line in self._stderr_tail if line)
        return f"worker {self.index} {state}" + (f": {tail}" if tail else "")

    def send(self, frame: bytes) -> None:
        try:
            self.process.stdin.write(frame)
            self.process.stdin.flush()
        except (OSError, ValueError) as exc:
            raise _WorkerGone(f"{self.describe()} ({type(exc).__name__})") from exc

    def receive(self, timeout: float | None = None):
        try:
            item = self._inbox.get(timeout=timeout)
        except queue.Empty as exc:
            raise _WorkerGone(f"{self.describe()} (no reply in {timeout} s)") from exc
        if isinstance(item, _WorkerGone):
            raise _WorkerGone(f"{item}; {self.describe()}")
        if item is _GONE:
            self._inbox.put(_GONE)
            raise _WorkerGone(self.describe())
        return item

    def close(self, grace: float = 2.0) -> None:
        self.dead = True
        try:
            self.process.stdin.close()
        except (OSError, ValueError):
            pass
        try:
            self.process.wait(timeout=grace)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        for stream in (self.process.stdout, self.process.stderr):
            try:
                stream.close()
            except (OSError, ValueError):
                pass


_PYTHON_NAME = re.compile(r"python(3(\.\d+)?)?", re.IGNORECASE)


def resolve_python_executable(candidate: str | None = None) -> str:
    """Интерпретатор для воркеров — либо отказ по имени, а не догадка.

    В Blender 2.92+ и в pytest `sys.executable` — сам интерпретатор. Во
    встроенном процессе, где это `blender.exe` либо пусто, запуск воркера через
    него открыл бы второй Blender: такое имя отвергается здесь.
    """

    path = sys.executable if candidate is None else candidate
    name = os.path.basename(str(path or ""))
    stem = name[:-4] if name.lower().endswith(".exe") else name
    if not _PYTHON_NAME.fullmatch(stem):
        raise DomainPoolUnavailable(
            f"sys.executable is not a Python interpreter: {path!r}"
        )
    return str(path)


def _worker_specification() -> dict:
    package = __package__ or ""
    if not package:
        raise DomainPoolUnavailable("the host package name is unknown")
    return {
        "sys_path": [
            os.path.abspath(item)
            for item in sys.path
            if isinstance(item, str) and item
        ],
        "package": package,
        "package_dir": os.path.dirname(os.path.abspath(__file__)),
    }


class _InterpreterRejected(RuntimeError):
    """Заказанный внешний интерпретатор не годится; `code` — индекс причины."""

    def __init__(self, code: int, detail: str) -> None:
        super().__init__(detail)
        self.code = code

    @property
    def outcome(self) -> str:
        if self.code < FIRST_MISMATCH_REASON:
            return INTERPRETER_UNUSABLE
        return INTERPRETER_MISMATCH


def _checked_identity(host: dict, reply) -> dict:
    """Тождество воркера из его первого кадра либо отказ интерпретатору."""

    if not (
        isinstance(reply, tuple)
        and len(reply) == 2
        and reply[0] == "hello"
        and isinstance(reply[1], dict)
    ):
        raise _InterpreterRejected(
            2, f"unexpected first frame {str(reply)[:120]!r}"
        )
    difference = _identity_difference(host, reply[1])
    if difference is not None:
        raise _InterpreterRejected(*difference)
    return reply[1]


#: EARLY_DISPATCH_BEFORE_UNPICKLE_V1: следующая холодная задача уходит воркеру, как только прочитаны БАЙТЫ ответа на предыдущую, а не после его разбора (разбор ответа - 1.1 мс процессора родителя, 1.9 мс стены
#: под GIL, и всё это время воркер стоял без задачи: зазор между задачами на холодной `cover.008` 3.2 -> 0.45 мс, стена пула -0.13 с). Выбор задачи, порядок очереди и число задач у воркера (одна в работе) те же;
#: выключатель - для теста, что порядок всего видимого прежний.
EARLY_DISPATCH = True

#: HEAD_STREAMED_FIRST_ROUND_V1: выключатель отбора кандидатов первого круга (см. `split_first_round`); выключен - полный порядок до старта, как прежде.
STREAMED_HEAD = True


class _Lane:
    """Задачи, ушедшие воркеру в этом прогоне и ещё не отвеченные (по порядку отправки), и сбой отправки вперёд; одно звено на воркера.

    Поток-читатель дописывает задачу, ушедшую вперёд (`take_ahead`), ДО того, как положит в ящик ответ, после которого она ушла, - поэтому поток воркера, получив тот ответ, видит её здесь.
    """

    __slots__ = ("flying", "failure")

    def __init__(self) -> None:
        self.flying: deque = deque()
        self.failure: str | None = None


#: Свободный воркер берёт задачу, подготовку которой держит, вместо первой в очереди, только если первая не «крупная»: задача дороже
#: `суммарная цена / (PULL_BIG_DIVISOR * воркеров)` идёт первой всегда (самая тяжёлая определяет длину прогона и не ждёт).
PULL_BIG_DIVISOR = 4


def _blob_input(task):
    """Вход задачи с пиклом подготовки в памяти воркера (`production` либо `coverage`, ключ непуст, пикл есть) либо `None`."""

    for inputs in (task.production, task.coverage):
        if inputs is not None and getattr(inputs, "key", "") and getattr(inputs, "blob", None) is not None:
            return inputs
    return None


def _key_only(task):
    """Та же задача без пикла подготовки: воркер берёт её из своей памяти по ключу."""

    if task.production is not None:
        return replace(task, production=replace(task.production, blob=None))
    return replace(task, coverage=replace(task.coverage, blob=None))


def _held_by(worker, task) -> bool:
    inputs = _blob_input(task)
    return inputs is not None and inputs.key in worker.held


class _PendingTasks:
    """Очередь невзятых задач, тяжёлые первыми; воркер предпочитает задачу, подготовку которой держит (меньше пересылок).

    Очередь, которую наполняют позже (`fill`, HEAD_STREAMED_FIRST_ROUND_V1), держит воркеров на `take` до наполнения: первый круг уже считается, а остаток ещё упорядочивается.
    Очередь, у которой есть готовая часть и будет остаток (`prefill`, потом `extend`; SCAN_BEHIND_THE_POOL_V1), отдаёт готовую часть сразу, а воркер, опустошивший её раньше остатка, ждёт остаток, а не уходит.
    """

    def __init__(self, items=None, big_cost: float = 0.0) -> None:
        self._items: list = []
        self._big = big_cost
        self._cond = threading.Condition()
        self._warm = 0
        #: Очередь можно брать (`fill` или `prefill` были) и очередь полна (остатка не будет): пустая неполная очередь держит воркера на `take`.
        self._ready = False
        self._final = False
        if items is not None:
            self.fill(items, big_cost)

    def _set(self, items, big_cost: float, final: bool) -> None:
        self._items = list(items)
        self._big = big_cost
        #: Сколько в очереди задач, не холодных: пока они есть, выбор зависит от памяти подготовок воркера (`_held_by`), и вперёд ответа его делать нельзя.
        self._warm = sum(1 for item in self._items if item[0].cold is None)
        self._ready = True
        self._final = self._final or final
        self._cond.notify_all()

    def fill(self, items, big_cost: float) -> None:
        with self._cond:
            self._set(items, big_cost, True)

    def prefill(self, items, big_cost: float) -> None:
        """Готовая часть очереди: воркеры берут её сразу; остаток придёт в `extend`."""

        with self._cond:
            self._set(items, big_cost, False)

    def extend(self, items, big_cost: float) -> None:
        """Остаток пришёл: он встаёт в нерасхваченную часть очереди по цене (порядок `order_by_cost`), очередь полна."""

        with self._cond:
            merged = sorted([*self._items, *items], key=lambda item: (-_frame_cost(*item), item[0].task_id))
            self._set(merged, big_cost, True)

    def release(self) -> None:
        """Очередь, которую так и не наполнили до конца, становится полной как есть: воркеры не ждут того, чего не будет."""

        with self._cond:
            if not self._final:
                self._set(self._items, self._big, True)

    def take(self, worker):
        with self._cond:
            while not self._ready or (not self._items and not self._final):
                self._cond.wait()
            if not self._items:
                return None
            head = self._items[0]
            if _frame_cost(*head) >= self._big or _held_by(worker, head[0]):
                return self._pop(0)
            for position, item in enumerate(self._items):
                if _held_by(worker, item[0]):
                    return self._pop(position)
            return self._pop(0)

    def take_ahead(self):
        """Первая задача очереди, если ВЫБОР не зависит от воркера и от его памяти, иначе `None`.

        В очереди из одних холодных задач `take` отдаёт голову при любом воркере (холодная задача подготовку не держит и ничьей памяти не нужна), поэтому задачу можно выбрать, не дожидаясь
        разбора ответа, который эту память пополняет (EARLY_DISPATCH_BEFORE_UNPICKLE_V1): выбранное то же, что выбрал бы `take` после разбора. Есть в очереди хоть одна не холодная - `None`, и выбирает `take`.
        """

        with self._cond:
            if not self._ready or self._warm or not self._items:
                return None
            return self._pop(0)

    def _pop(self, position):
        item = self._items.pop(position)
        if item[0].cold is None:
            self._warm -= 1
        return item


class _PoolCounter:
    """Счёт пересылок прогона из потоков воркеров."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._values = {"sent": 0, "hits": 0, "shipped": 0, "shipped_bytes": 0, "misses": 0, "owner": 0, "unowned": 0, "stolen": 0, "head": 0, "head_misses": 0}

    def add(self, **amounts) -> None:
        with self._lock:
            for name, amount in amounts.items():
                self._values[name] += amount

    def stats(self, received: int, cpu: float, wall: float) -> PoolStatsV1:
        values = self._values
        return PoolStatsV1(
            values["sent"],
            received,
            values["hits"],
            values["shipped"],
            values["shipped_bytes"],
            values["misses"],
            cpu,
            wall,
            values["owner"],
            values["unowned"],
            values["stolen"],
            values["head"],
            values["head_misses"],
        )


def _received(worker, reply, counted: _PoolCounter):
    """Ответ воркера для родителя: зеркало памяти воркера пополнено.

    Пикл подготовки холодного домена (`prepared_blob`) остаётся байтами: прежде поток-читатель разворачивал его здесь (`pickle.loads` под GIL,
    ~12 мс на домен), и на `cover.008` эти развороты, идущие по очереди через GIL, держали воркеров в простое. Разворачивает родитель, и только
    там, где живая подготовка нужна (`EnvelopeDebugSessionController.live_preparation`); не читающийся пикл тогда - названный отказ домена.
    """

    if isinstance(reply, DomainTaskResultV1):
        for key, size in reply.stored:
            worker.held.add(key, size)
    return reply


def _exchange(worker, task, frame: bytes, counted: _PoolCounter, sent: bool = False):
    """Задача воркеру и его ответ: подготовка уходит ключом, если воркер её держит, и пиклом, если нет либо он её не нашёл.

    `sent` - задача уже ушла воркеру (`_Ahead`: вперёд разбора ответа на предыдущую): остаётся её ответ. Так уходят только холодные задачи, у которых блоба нет.
    """

    inputs = _blob_input(task)
    if inputs is None:
        if not sent:
            worker.send(frame)
            counted.add(sent=len(frame))
        return _received(worker, worker.receive(), counted)
    if sent:
        raise RuntimeError("a task with a preparation blob cannot be sent ahead of the previous reply")
    key = inputs.key
    if key in worker.held:
        worker.held.touch(key)
        stub = encode_frame(_key_only(task))
        worker.send(stub)
        counted.add(sent=len(stub))
        reply = _received(worker, worker.receive(), counted)
        if not (isinstance(reply, DomainTaskResultV1) and reply.needs_blob):
            counted.add(hits=1)
            return reply
        worker.held.discard(key)
        counted.add(misses=1)
    worker.held.add(key, len(inputs.blob))
    worker.send(frame)
    counted.add(sent=len(frame), shipped=1, shipped_bytes=len(inputs.blob))
    reply = _received(worker, worker.receive(), counted)
    if isinstance(reply, DomainTaskResultV1) and (reply.error or reply.needs_blob):
        worker.held.discard(key)  # воркер мог её не запомнить: в следующий раз пикл уйдёт снова
    return reply


class _Run:
    """Один прогон `DomainPool`: звенья воркеров, очередь, ответы и счёт пересылок; потоки воркеров (`feed`) и потоки-читатели (`ahead`) работают с ним."""

    def __init__(self, last_worker: dict, participants, tasks, cancel, late: LateTasksV1 | None = None) -> None:
        self.last_worker = last_worker  # `{ключ привязки: индекс воркера}` пула: поток воркера дописывает, кто считал задачу
        self.participants = participants
        self.cancel = cancel
        self.indices = [item.index for item in participants]
        self.owners = dict(last_worker)  # воркеры ПРОШЛЫХ прогонов: снимок, потоки воркеров дописывают новых по ходу
        self.count = max(1, len(participants))
        self.results: dict[int, DomainTaskResultV1] = {}
        self.counted = _PoolCounter()
        self.lanes = {item.index: _Lane() for item in participants}
        self.streamed = (
            STREAMED_HEAD
            and (len(tasks) if late is None else late.total) >= STREAMED_HEAD_MIN_TASKS_PER_WORKER * self.count
            and all(task.export is not None for task in tasks)
        )
        if late is not None and not self.streamed:
            tasks, late = [*tasks, *late.provide()], None  # отбора кандидатов нет: остаток нужен до старта, как у полного списка
        self.late = late
        if self.streamed:
            self.head, self.rest_tasks = split_first_round(tasks, self.count)
            first = plan_first_round(self.head, self.indices, self.owners)
            self.started = {worker: self.head[position] for worker, position in first.items()}
            self.pending = _PendingTasks()
        else:
            ordered = order_by_cost(tasks)
            first = plan_first_round(ordered, self.indices, self.owners)
            self.started = {worker: ordered[position] for worker, position in first.items()}
            taken = set(first.values())
            self.pending = _PendingTasks(
                [item for position, item in enumerate(ordered) if position not in taken],
                sum(_frame_cost(*item) for item in ordered) / (PULL_BIG_DIVISOR * self.count),
            )

    def execute(self):
        """Потоки воркеров считают, пока этот поток (при отборе кандидатов) упорядочивает остаток; `None` либо отказ упорядочения, который вызывающий бросает после прогона."""

        threads = [threading.Thread(target=self.feed, args=(worker,), daemon=True) for worker in self.participants]
        for worker in self.participants:
            worker.on_reply_bytes = self.ahead if EARLY_DISPATCH else None
        ordering_failure = None
        try:
            for thread in threads:
                thread.start()
            if self.streamed:
                try:
                    self._order_rest()
                except Exception as exc:  # noqa: BLE001 - воркеры не должны висеть на очереди, которой не будет: наполняем пустой, прогон кончается, а отказ уходит вызывающему
                    ordering_failure = exc
                    self.pending.release()
            for thread in threads:
                thread.join()
        finally:
            self.pending.release()
            for worker in self.participants:
                worker.on_reply_bytes = None
        return ordering_failure

    def _order_rest(self) -> None:
        early: list = []
        if self.late is None:
            rest = order_by_cost(self.rest_tasks)
        else:
            early = order_by_cost(self.rest_tasks) if self.rest_tasks else []
            if early:  # SCAN_BEHIND_THE_POOL_V1: кандидаты, которых вызывающий дал вместе с головой, уже готовы - воркеры берут их, пока он достраивает остальное
                self.pending.prefill(early, sum(_frame_cost(*item) for item in (*self.head, *early)) / (PULL_BIG_DIVISOR * self.count))
            late = order_by_cost(self.late.provide())
            rest = sorted([*early, *late], key=lambda item: (-_frame_cost(*item), item[0].task_id))
        lightest = min(_frame_cost(*item) for item in self.head)
        self.counted.add(head=1, head_misses=sum(1 for item in rest if _frame_cost(*item) > lightest))
        big = sum(_frame_cost(*item) for item in (*self.head, *rest)) / (PULL_BIG_DIVISOR * self.count)
        if self.late is None:
            self.pending.fill(rest, big)
        else:
            self.pending.extend(late, big)

    def ahead(self, worker: _Worker) -> None:
        """Поток-читатель: байты ответа прочитаны, разбор впереди. Следующая холодная задача уходит воркеру сейчас (`_PendingTasks.take_ahead`)."""

        lane = self.lanes[worker.index]
        if worker.dead or lane.failure is not None or (self.cancel is not None and self.cancel.is_set()):
            return
        item = self.pending.take_ahead()
        if item is None:
            return
        lane.flying.append(item)
        try:
            worker.send(item[1])
        except _WorkerGone as exc:  # сбой отправки называет поток воркера: он ждёт ответа на эту задачу и получает отказ воркера
            lane.failure = str(exc)
            return
        self.counted.add(sent=len(item[1]))

    def lose(self, lane: _Lane, error: str) -> None:
        """Каждая задача, ушедшая воркеру и не отвеченная, получает названный ответ с ошибкой: молча пропавшей задачи нет."""

        while lane.flying:
            lost = lane.flying.popleft()[0]
            self.results[lost.task_id] = DomainTaskResultV1(lost.task_id, error=error)

    def feed(self, worker: _Worker) -> None:
        lane = self.lanes[worker.index]
        first = self.started.get(worker.index)
        while True:
            late = not lane.flying  # задачи нет у воркера: берём и шлём; иначе её отправил поток-читатель (`ahead`), остаётся ответ
            if late:
                if self.cancel is not None and self.cancel.is_set():
                    return
                item = first if first is not None else self.pending.take(worker)
                first = None
                if item is None:
                    return
                lane.flying.append(item)
            task, frame = lane.flying[0]
            try:
                if lane.failure is not None and not late:
                    raise _WorkerGone(lane.failure)
                reply = _exchange(worker, task, frame, self.counted, sent=not late)
            except _WorkerGone as exc:
                worker.dead = True
                self.lose(lane, f"worker died: {exc}")
                return
            lane.flying.popleft()
            if not isinstance(reply, DomainTaskResultV1) or reply.task_id != task.task_id:
                worker.dead = True
                self.results[task.task_id] = DomainTaskResultV1(task.task_id, error=f"protocol violation: {type(reply).__name__}")
                self.lose(lane, f"worker died: protocol violation on the reply to task {task.task_id}")
                return
            self.results[task.task_id] = reply
            if task.affinity:
                owner = self.owners.get(task.affinity)
                kind = "owner" if owner == worker.index else ("stolen" if owner in self.indices else "unowned")
                self.counted.add(**{kind: 1})
                self.last_worker[task.affinity] = worker.index


class DomainPool:
    """Постоянные воркеры: стартуют лениво, живут между нажатиями кнопки.

    `python_executable` — встроенный интерпретатор (по умолчанию `sys.executable`):
    воркеры на нём наследуют `sys.path` родителя. `external_python` — путь
    заказанного ВНЕШНЕГО интерпретатора; его воркеры не наследуют ничего, кроме
    каталогов пакетов хоста (см. докстроку модуля). Отказ внешнему — не отказ
    пулу: воркеры идут на встроенном, а причина лежит в `interpreter`.
    """

    def __init__(
        self,
        workers: int,
        *,
        python_executable: str | None = None,
        external_python: str | None = None,
    ):
        self.requested = max(0, int(workers))
        self._python_executable = python_executable
        self._external_python = str(external_python or "")
        self._workers: list[_Worker] = []
        self._interpreter: PoolInterpreterV1 | None = None
        self._rejection: _InterpreterRejected | None = None
        # Воркер однопоточен по кадрам: два одновременных `run` перемешали бы кадры. Раньше пулом
        # пользовался один главный поток; теперь превью alpha гонит `run` из потока счёта.
        self._run_lock = threading.Lock()
        #: Сколько прогонов (`run`) ждут замка либо идут: фоновые шаги (`run_idle`) не берут замок, пока их хоть один, и тем уступают следующему действию.
        self._foreground = 0
        self._wanted = threading.Condition()
        #: `{ключ привязки задачи: индекс воркера, который её считал}`; индексы воркеров не переиспользуются.
        self._last_worker: dict[str, int] = {}
        #: Код привязки воркеров к первым потокам физических ядер (POOL_WORKERS_ON_PRIMARY_THREADS_V1) по последнему старту воркеров.
        self._affinity = AFFINITY_NOT_EVALUATED

    @property
    def affinity(self) -> int:
        return self._affinity

    @property
    def worker_count(self) -> int:
        return sum(not item.dead for item in self._workers)

    @property
    def worker_prewarms(self) -> tuple:
        """Прогрев каждого живого воркера, как он назвал его в кадре «готов» (`warm_worker_identities`)."""

        return tuple(dict(item.prewarm) for item in self._workers if not item.dead)

    @property
    def external_python(self) -> str:
        return self._external_python

    @property
    def interpreter(self) -> PoolInterpreterV1 | None:
        """На чём идут воркеры; `None`, пока ни один не стартовал."""

        return self._interpreter

    @property
    def rejected(self) -> bool:
        """Внешний интерпретатор заказан, но отвергнут и воркеры на встроенном."""

        return self._rejection is not None

    def ensure_started(self) -> None:
        """Доводит число живых воркеров до заказанного либо бросает отказ."""

        self._workers = [item for item in self._workers if not item.dead]
        missing = self.requested - len(self._workers)
        if missing <= 0:
            return
        if self._external_python and self._rejection is None:
            try:
                self._launch(missing, external=True)
                return
            except _InterpreterRejected as rejection:
                self._rejection = rejection
                # Выжившие — воркеры внешнего интерпретатора (докомплектация),
                # а встроенный ответит так же, но не вперемешку с ними.
                self.close()
                missing = self.requested
        try:
            self._launch(missing, external=False)
        except DomainPoolUnavailable as exc:
            if self._rejection is None:
                raise
            raise DomainPoolUnavailable(
                f"external Python rejected ({self._rejection}); "
                f"bundled Python failed too: {exc}"
            ) from exc

    def warm(self) -> float:
        """Поднимает воркеров ЗАРАНЕЕ (под замком прогона) и возвращает секунды; прогрев, что опоздал за снятием пула, воркеров не оставляет.

        Кнопка, пришедшая во время прогрева, ждёт замок и находит пул готовым; прогрев, пришедший во время прогона, ждёт его конца.
        """

        started = time.perf_counter()
        with self._run_lock:
            self.ensure_started()
        with _POOL_LOCK:
            if _POOL is not self:
                self.close()
        return time.perf_counter() - started

    def _external_plan(self) -> tuple[str, dict, dict]:
        """Интерпретатор, спецификация и тождество хоста для внешних воркеров."""

        path = self._external_python
        if not os.path.isfile(path):
            raise _InterpreterRejected(1, f"Worker Python is not a file: {path!r}")
        try:
            python = resolve_python_executable(path)
        except DomainPoolUnavailable as exc:
            raise _InterpreterRejected(1, str(exc)) from exc
        roots = {name: package_directory(name) for name in HOST_PACKAGES}
        absent = [name for name, root in roots.items() if root is None]
        if absent:
            raise _InterpreterRejected(
                3, f"host cannot locate {', '.join(absent)} on its own path"
            )
        host = describe_environment()
        if any(str(host[key]).startswith("missing") for key in ("sympy", "mpmath")):
            raise _InterpreterRejected(
                3, f"host cannot import sympy/mpmath: {host['sympy']}, {host['mpmath']}"
            )
        entries: list[str] = []
        optional = (package_directory(name) for name in OPTIONAL_HOST_PACKAGES)
        for root in (*(roots[name] for name in HOST_PACKAGES), *(item for item in optional if item)):
            entry = os.path.dirname(root)
            if entry not in entries:
                entries.append(entry)
        specification = {
            **_worker_specification(),
            "sys_path": entries,
            "after_stdlib": True,
            "handshake": True,
        }
        return python, specification, host

    def _launch(self, missing: int, *, external: bool) -> None:
        """Стартует `missing` воркеров на внешнем либо встроенном интерпретаторе."""

        host: dict = {}
        flags: list[str] = []
        if external:
            python, specification, host = self._external_plan()
            flags = ["-I", "-S"]
        else:
            python = resolve_python_executable(self._python_executable)
            specification = _worker_specification()
        environment = dict(os.environ)
        environment[SPEC_ENVIRONMENT_VARIABLE] = json.dumps(specification)
        command = [python, *flags, "-u", "-c", _BOOTSTRAP]
        started: list[_Worker] = []
        failures: list[str] = []
        version = tuple(sys.version_info[:3])
        first_free = max((item.index for item in self._workers), default=-1) + 1
        for offset in range(missing):
            try:
                started.append(
                    _Worker(first_free + offset, command, environment)
                )
            except OSError as exc:
                failures.append(f"cannot start {python!r}: {exc}")
                break
        if started:
            self._affinity = apply_worker_affinity([item.process.pid for item in started], self.requested)
        for position, worker in enumerate(started):
            try:
                reply = worker.receive(timeout=READY_TIMEOUT_SECONDS)
                if external:
                    identity = _checked_identity(host, reply)
                    worker.identity = identity
                    version = tuple(identity["python"])
                    reply = worker.receive(timeout=READY_TIMEOUT_SECONDS)
                if not (isinstance(reply, tuple) and reply[:1] == ("ready",)):
                    raise _WorkerGone(f"unexpected first frame {reply!r}")
                worker.prewarm = dict(reply[2]) if len(reply) > 2 and isinstance(reply[2], dict) else {}
            except _WorkerGone as exc:
                failures.append(f"worker failed to start: {exc}")
                worker.close(grace=0.5)
                continue
            except _InterpreterRejected:
                for pending in started[position:]:
                    pending.close(grace=0.5)
                raise
            self._workers.append(worker)
        if not self._workers:
            detail = "; ".join(failures) or "no worker started"
            if external:
                raise _InterpreterRejected(2, detail)
            raise DomainPoolUnavailable(detail)
        self._interpreter = self._describe_interpreter(external, version)

    def _describe_interpreter(self, external: bool, version: tuple):
        rejection = self._rejection
        if rejection is None:
            return PoolInterpreterV1(version, external)
        return PoolInterpreterV1(
            version, False, rejection.outcome, str(rejection), rejection.code
        )

    #: Пул принимает `late` (`LateTasksV1`): вызывающий достраивает остаток задач, пока воркеры считают (SCAN_BEHIND_THE_POOL_V1).
    streams_late = True

    def run(self, tasks, cancel=None, late=None) -> DomainPoolRunV1:
        """Считает задачи воркерами, тяжёлые первыми, и собирает ответы.

        Один прогон за раз (`_run_lock`). `cancel` — `threading.Event`: после его установки воркеры
        не берут НОВЫЕ задачи (идущая дочитывается), и ответ вернёт только сделанное; кто отменил,
        тот и разбирает неполный ответ (`SliderCoveragePool.cover` бросает `CoverageCancelled`).
        `late` (`LateTasksV1`) - остаток задач, который вызывающий достроит ПОСЛЕ старта воркеров: `tasks` тогда голова (отобранные кандидаты первого круга), а порядок и первый круг те же, что дал бы полный список.
        """

        with self._wanted:
            self._foreground += 1  # ДО замка: фоновый шаг, идущий сейчас, дочитывается (не дольше одного шага), следующего он не начнёт
        try:
            with self._run_lock:
                return self._run_locked(tasks, cancel) if late is None else self._run_locked(tasks, cancel, late)
        finally:
            with self._wanted:
                self._foreground -= 1
                self._wanted.notify_all()

    def run_idle(self, message, on_reply, *, stop: threading.Event, timeout: float = 120.0) -> tuple[bool, dict]:
        """Фоновая работа на простаивающих воркерах: `message` каждому живому воркеру шаг за шагом, пока `on_reply` не скажет «этому больше не надо» (SNAPSHOT_BYTE_CHECK_FIRST_PRESS_IN_BACKGROUND_V1).

        `on_reply(индекс воркера, ответ) -> bool`: `True` - воркеру нужен ещё шаг. Шаг берёт замок прогона и держит его на один обмен с каждым воркером (воркер отвечает за свой бюджет шага, десятки мс);
        между шагами замок отдан, и пока есть хоть один `run`, шаги не начинаются. `stop` (`threading.Event`) прерывает задачу между шагами. Возвращает `(все воркеры довели, {индекс воркера: причина})`:
        воркер, умерший посреди задачи, в причинах и закрыт; отсутствие живых воркеров - тоже причина. Воркеры, которых пул пересоздал после начала, в задаче не участвуют: их памяти у задачи нет.
        """

        frame = encode_frame(message)
        pending = {item.index for item in self._workers if not item.dead}
        failures: dict[int, str] = {}
        while pending and not stop.is_set():
            with self._wanted:
                while self._foreground and not stop.is_set():
                    self._wanted.wait(0.25)
            if stop.is_set():
                break
            with self._run_lock:
                if self._foreground:
                    continue
                batch = [item for item in self._workers if item.index in pending and not item.dead]
                for index in pending - {item.index for item in batch}:
                    failures[index] = "the worker is gone"
                    pending.discard(index)
                sent = []
                for worker in batch:
                    try:
                        worker.send(frame)
                        sent.append(worker)
                    except _WorkerGone as exc:
                        worker.dead = True
                        failures[worker.index] = str(exc)
                        pending.discard(worker.index)
                for worker in sent:
                    try:
                        reply = worker.receive(timeout)
                    except _WorkerGone as exc:
                        worker.dead = True
                        failures[worker.index] = str(exc)
                        pending.discard(worker.index)
                        continue
                    if not on_reply(worker.index, reply):
                        pending.discard(worker.index)
                for worker in batch:
                    if worker.dead:
                        worker.close(grace=0.5)
                self._workers = [item for item in self._workers if not item.dead]
        return (not pending and not failures and not stop.is_set()), failures

    def _run_locked(self, tasks, cancel, late=None) -> DomainPoolRunV1:
        self.ensure_started()
        participants = list(self._workers)
        run = _Run(self._last_worker, participants, list(tasks), cancel, late)
        before = {worker.index: (worker.received_bytes, worker.unpickle_cpu, worker.unpickle_wall) for worker in participants}
        ordering_failure = run.execute()
        for worker in participants:
            if worker.dead:
                worker.close(grace=0.5)
        self._workers = [item for item in self._workers if not item.dead]
        if len(self._last_worker) > MAX_AFFINITY_KEYS:
            self._last_worker.clear()
        received = [
            (worker.received_bytes - before[worker.index][0], worker.unpickle_cpu - before[worker.index][1], worker.unpickle_wall - before[worker.index][2])
            for worker in participants
        ]
        stats = run.counted.stats(
            sum(item[0] for item in received), sum(item[1] for item in received), sum(item[2] for item in received)
        )
        if ordering_failure is not None:
            raise ordering_failure
        return DomainPoolRunV1(run.results, len(participants), self._interpreter, stats, self._affinity)

    def close(self) -> None:
        workers, self._workers = self._workers, []
        for worker in workers:
            worker.close()


_POOL: DomainPool | None = None
_POOL_LOCK = threading.Lock()
_ATEXIT_REGISTERED = False


def get_domain_pool(workers: int, external_python: str = "") -> DomainPool | None:
    """Общий пул на заданное число воркеров; `None` — параллельность выключена.

    Перед сменой размера старый пул закрывается, а при `workers < 2` (0 и 1 —
    последовательный путь) закрывается и тёплый: простаивающие воркеры держат
    около 100 МБ каждый. То же при смене `external_python` (пустая строка —
    встроенный интерпретатор) и для пула, чей внешний интерпретатор отвергнут:
    он пересоздаётся при каждом нажатии, чтобы исправленное окружение подхватывалось
    без перезапуска, а не жило до конца сеанса в устаревшем отказе.
    """

    global _POOL, _ATEXIT_REGISTERED
    requested = int(workers)
    external = str(external_python or "")
    with _POOL_LOCK:
        if requested < MIN_POOL_WORKERS:
            if _POOL is not None:
                _POOL.close()
                _POOL = None
            return None
        if _POOL is not None and (
            _POOL.requested != requested
            or _POOL.external_python != external
            or _POOL.rejected
        ):
            _POOL.close()
            _POOL = None
        if _POOL is None:
            _POOL = DomainPool(requested, external_python=external)
            if not _ATEXIT_REGISTERED:
                # Страховка сверх `unregister`: воркеры умирают и сами, когда
                # закрывается труба родителя, но обычный выход не должен ждать
                # этого от операционной системы.
                atexit.register(shutdown_domain_pool)
                _ATEXIT_REGISTERED = True
        return _POOL


def peek_domain_pool(workers: int) -> DomainPool | None:
    """Живой общий пул на `workers` воркеров либо `None`; пул не заводит.

    Ползунок alpha не имеет права стартовать воркеров посреди перетаскивания
    (старт — секунды): он пользуется пулом, который кнопка уже подняла, и
    только им. Пул с погибшими воркерами не «живой»: его воскрешает кнопка.
    """

    requested = int(workers)
    with _POOL_LOCK:
        pool = _POOL
        if pool is None or pool.requested != requested:
            return None
        return pool if pool.worker_count else None


def shutdown_domain_pool() -> None:
    global _POOL
    with _POOL_LOCK:
        if _POOL is not None:
            _POOL.close()
            _POOL = None


__all__ = (
    "DomainPool",
    "DomainPoolRunV1",
    "DomainPoolUnavailable",
    "DomainTaskResultV1",
    "DEFAULT_POOL_WORKERS",
    "DomainTaskV1",
    "HOST_PACKAGES",
    "INTERPRETER_MISMATCH",
    "INTERPRETER_REASONS",
    "INTERPRETER_UNUSABLE",
    "MIN_POOL_WORKERS",
    "PoolInterpreterV1",
    "PoolStatsV1",
    "WORKER_IDENTITY_PREWARMED",
    "WORKER_IDENTITY_PREWARM_UNAVAILABLE",
    "describe_environment",
    "encode_frame",
    "get_domain_pool",
    "kernel_fingerprint",
    "order_by_cost",
    "package_directory",
    "peek_domain_pool",
    "plan_first_round",
    "read_frame",
    "read_frame_sized",
    "resolve_python_executable",
    "shutdown_domain_pool",
    "solve_task",
    "split_first_round",
    "warm_worker_identities",
    "worker_main",
    "write_frame",
)
