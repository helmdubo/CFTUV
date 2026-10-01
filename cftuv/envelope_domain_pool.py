"""Пул процессов-воркеров для доменов очереди (срез PARALLEL-DOMAINS).

Домены независимы по ключу исполнения `(DecalRequestId, PatchDomainId)`:
подготовка, покрытие и контур одного домена не читают ничего из соседнего. На
поле (`building`, 122 домена) пять доменов дают 76 % времени, а пул процессов
по доменам даёт ПОБИТОВО те же исходы, статьи бюджета, счётчики и отпечатки
(`artifacts/parallel_domains_spike/`), поэтому этот модуль ничего не решает о
геометрии: он пересылает готовую задачу и возвращает готовый ответ. Задача
может нести и выгрузку снапшота домена (`DomainTaskV1.export`, см.
`envelope_export_input`): воркер выгружает `(snapshot, request)` сам, тем же
кодом, что и хост, и возвращает снапшот вместе с ответом.

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
"""

from __future__ import annotations

import atexit
import importlib
import json
import os
import pickle
import queue
import re
import struct
import subprocess
import sys
import threading
import traceback
from collections import deque
from dataclasses import dataclass, replace

#: Меньше двух воркеров — это последовательный путь, пула не заводится.
MIN_POOL_WORKERS = 2

#: Умолчание настройки: около 100 МБ на воркер и ни одного воркера сверх числа
#: логических ядер (считается при импорте). Предел 8 — плато спайка.
DEFAULT_POOL_WORKERS = min(8, os.cpu_count() or 1)

#: Сколько ждать готовности воркера (импорт ядра и sympy на холодном диске).
READY_TIMEOUT_SECONDS = 60.0

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

# Запускается как `python -u -c`. Пакет хоста поднимается по файлу под ТЕМ ЖЕ
# именем, под которым он загружен у родителя: в Blender 4.2+ это
# `bl_ext.<репозиторий>.<id>`, которого нет ни в каком `sys.path`, и pickle
# иначе не нашёл бы классы по `__module__`.
_BOOTSTRAP = """
import importlib, importlib.util, json, os, sys
spec = json.loads(os.environ[{variable!r}])
parent = [item for item in spec["sys_path"] if item]
sys.path[:] = parent + [
    item for item in sys.path if item and item not in parent
]
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
    """

    task_id: int
    patch_id: int
    domain_id: str
    snapshot: object
    request: object
    alpha_text: str
    selected_edges: frozenset
    export: object | None = None


@dataclass(frozen=True, slots=True)
class DomainTaskResultV1:
    """Ответ воркера: `error` непуст, либо `refusal`, либо подготовка и запись.

    `refusal` — ИМЕНОВАННЫЙ отказ выгрузки домена (`ExportRefusalV1`): это
    ответ, а не сбой, и родитель разбирает его как разобрал бы сам. Снапшот
    `snapshot` есть у задачи с выгрузкой в воркере, когда выгрузка удалась;
    `export_timings` и `export_counters` — её стадии, которые родитель
    проигрывает в профиль кнопки.
    """

    task_id: int
    prepared: object | None = None
    queue_domain: object | None = None
    error: str = ""
    snapshot: object | None = None
    refusal: object | None = None
    export_timings: tuple = ()
    export_counters: tuple = ()

    @property
    def ok(self) -> bool:
        return not self.error and self.queue_domain is not None

    @property
    def refused(self) -> bool:
        return not self.error and self.refusal is not None


@dataclass(frozen=True, slots=True)
class DomainPoolRunV1:
    """Итог прогона: ответы по `task_id` и сколько воркеров в нём участвовало.

    Задачи, которых в `results` нет, не исполнялись вовсе (все воркеры умерли
    раньше, чем до них дошла очередь).
    """

    results: dict
    workers: int


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


def read_frame(stream):
    """Следующий объект потока либо `None` на чистом конце потока.

    Обрыв посреди кадра — не конец, а `EOFError`: отличить «воркер закончил» от
    «воркер умер на записи» иначе нечем.
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
    return pickle.loads(payload)


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
    готовым снапшотом перед ещё более тяжёлым, которого кадр прячет.
    """

    framed = [(task, encode_frame(task)) for task in tasks]
    framed.sort(
        key=lambda item: (
            -len(item[1])
            * (EXPORT_FRAME_COST_SCALE if item[0].export is not None else 1),
            item[0].task_id,
        )
    )
    return framed


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
        if task.export is not None:
            from .envelope_export_input import solve_exported_task

            return solve_exported_task(task)
        from .envelope_queue_export import run_queue_domain

        prepared, domain = run_queue_domain(
            task.patch_id,
            task.domain_id,
            task.snapshot,
            task.request,
            task.alpha_text,
            selected_edges=task.selected_edges,
            profile=None,
        )
        return DomainTaskResultV1(
            task.task_id, prepared, replace(domain, preparation=None)
        )
    except Exception:  # noqa: BLE001 - ответ несёт трассу, а не теряет её
        return DomainTaskResultV1(task.task_id, error=traceback.format_exc())


def _reply_frame(task_id: int, result: DomainTaskResultV1) -> bytes:
    try:
        return encode_frame(result)
    except Exception:  # noqa: BLE001 - непиклящийся ответ тоже названный отказ
        return encode_frame(
            DomainTaskResultV1(task_id, error=traceback.format_exc())
        )


def worker_main() -> None:
    """Цикл воркера: задача на входе, ответ на выходе, до конца stdin."""

    channel = os.fdopen(os.dup(1), "wb")
    os.dup2(2, 1)
    sys.stdout = sys.stderr
    source = sys.stdin.buffer
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
    write_frame(channel, ("ready", os.getpid()))
    while True:
        task = read_frame(source)
        if task is None:
            return
        channel.write(_reply_frame(task.task_id, solve_task(task)))
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
                frame = read_frame(self.process.stdout)
                if frame is None:
                    break
                self._inbox.put(frame)
        except Exception as exc:  # noqa: BLE001 - обрыв кадра == смерть воркера
            self._inbox.put(_WorkerGone(f"{type(exc).__name__}: {exc}"))
        self._stdout_closed = True
        self._inbox.put(_GONE)

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


class DomainPool:
    """Постоянные воркеры: стартуют лениво, живут между нажатиями кнопки."""

    def __init__(self, workers: int, *, python_executable: str | None = None):
        self.requested = max(0, int(workers))
        self._python_executable = python_executable
        self._workers: list[_Worker] = []

    @property
    def worker_count(self) -> int:
        return sum(not item.dead for item in self._workers)

    def ensure_started(self) -> None:
        """Доводит число живых воркеров до заказанного либо бросает отказ."""

        self._workers = [item for item in self._workers if not item.dead]
        missing = self.requested - len(self._workers)
        if missing <= 0:
            return
        python = resolve_python_executable(self._python_executable)
        environment = dict(os.environ)
        environment[SPEC_ENVIRONMENT_VARIABLE] = json.dumps(
            _worker_specification()
        )
        command = [python, "-u", "-c", _BOOTSTRAP]
        started: list[_Worker] = []
        failures: list[str] = []
        first_free = max((item.index for item in self._workers), default=-1) + 1
        for offset in range(missing):
            try:
                started.append(
                    _Worker(first_free + offset, command, environment)
                )
            except OSError as exc:
                failures.append(f"cannot start {python!r}: {exc}")
                break
        for worker in started:
            try:
                reply = worker.receive(timeout=READY_TIMEOUT_SECONDS)
                if not (isinstance(reply, tuple) and reply[:1] == ("ready",)):
                    raise _WorkerGone(f"unexpected first frame {reply!r}")
            except _WorkerGone as exc:
                failures.append(f"worker failed to start: {exc}")
                worker.close(grace=0.5)
                continue
            self._workers.append(worker)
        if not self._workers:
            raise DomainPoolUnavailable(
                "; ".join(failures) or "no worker started"
            )

    def run(self, tasks) -> DomainPoolRunV1:
        """Считает задачи воркерами, тяжёлые первыми, и собирает ответы."""

        self.ensure_started()
        pending: queue.Queue = queue.Queue()
        for item in order_by_cost(tasks):
            pending.put(item)
        results: dict[int, DomainTaskResultV1] = {}
        participants = list(self._workers)

        def feed(worker: _Worker) -> None:
            while True:
                try:
                    task, frame = pending.get_nowait()
                except queue.Empty:
                    return
                try:
                    worker.send(frame)
                    reply = worker.receive()
                except _WorkerGone as exc:
                    worker.dead = True
                    results[task.task_id] = DomainTaskResultV1(
                        task.task_id, error=f"worker died: {exc}"
                    )
                    return
                if (
                    not isinstance(reply, DomainTaskResultV1)
                    or reply.task_id != task.task_id
                ):
                    worker.dead = True
                    results[task.task_id] = DomainTaskResultV1(
                        task.task_id,
                        error=f"protocol violation: {type(reply).__name__}",
                    )
                    return
                results[task.task_id] = reply

        threads = [
            threading.Thread(target=feed, args=(worker,), daemon=True)
            for worker in participants
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        for worker in participants:
            if worker.dead:
                worker.close(grace=0.5)
        self._workers = [item for item in self._workers if not item.dead]
        return DomainPoolRunV1(results, len(participants))

    def close(self) -> None:
        workers, self._workers = self._workers, []
        for worker in workers:
            worker.close()


_POOL: DomainPool | None = None
_POOL_LOCK = threading.Lock()
_ATEXIT_REGISTERED = False


def get_domain_pool(workers: int) -> DomainPool | None:
    """Общий пул на заданное число воркеров; `None` — параллельность выключена.

    Перед сменой размера старый пул закрывается, а при `workers < 2` (0 и 1 —
    последовательный путь) закрывается и тёплый: простаивающие воркеры держат
    около 100 МБ каждый.
    """

    global _POOL, _ATEXIT_REGISTERED
    requested = int(workers)
    with _POOL_LOCK:
        if requested < MIN_POOL_WORKERS:
            if _POOL is not None:
                _POOL.close()
                _POOL = None
            return None
        if _POOL is not None and _POOL.requested != requested:
            _POOL.close()
            _POOL = None
        if _POOL is None:
            _POOL = DomainPool(requested)
            if not _ATEXIT_REGISTERED:
                # Страховка сверх `unregister`: воркеры умирают и сами, когда
                # закрывается труба родителя, но обычный выход не должен ждать
                # этого от операционной системы.
                atexit.register(shutdown_domain_pool)
                _ATEXIT_REGISTERED = True
        return _POOL


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
    "MIN_POOL_WORKERS",
    "encode_frame",
    "get_domain_pool",
    "order_by_cost",
    "read_frame",
    "resolve_python_executable",
    "shutdown_domain_pool",
    "solve_task",
    "worker_main",
    "write_frame",
)
