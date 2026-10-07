"""Швы слоя времён событий и закона кандидата скелета (WP-S0/S2): записи вызовов эталона из воспроизведения корпуса и читалка к ним.

Нативный лист (`compare_times`, `concurrency_time`, `sliding_time`, `sliding_point`, места событий, нормировка времени, очередь, закон кандидата) можно
проверить РАНЬШЕ целого скелета, если у него есть вызовы эталона с аргументами, исходом и ценой. Этот модуль воспроизводит записи корпуса скелета (`native_corpus`)
эталоном, ставит записывающие обёртки на функции слоя в КАЖДОМ модуле, держащем их по имени, и пишет по файлу на запись корпуса:

    <каталог корпуса>/seams/<id записи>.seams         xz(pickle): вызовы, журнал памяти, счётчики
    <каталог корпуса>/seams/index.json                по файлам: записано/видено по швам, секунды, размер

Вызов шва (`SeamCall`): аргументы (объекты ядра: `EventTimeV1`, `SupportLineV1`, `SqrtSumV1`; тождество объектов внутри файла сохранено пиклом: ключи памяти
`PositionMemoV1` держат `id()` прямых), результат либо исключение `(класс, текст)`, ЦЕНА (сдвиг пяти счётчиков знака, шести статей бюджета либо неоплаченного, бюджет ДО:
режим, потолок, стадия, статьи), события журнала памяти до и после, секунды. Закон кандидата несёт ещё ЖУРНАЛ ВИДА (ответы `vertex_state`, `span_state`, `trace_bounds` по
порядку), журнал памяти мест (`PositionMemoV1`: какой запрос попал, какой нет, значения попаданий) и ответ фабрики тождества отказа: вызов самодостаточен.

Память канонизации — ЖУРНАЛ СОСТОЯНИЙ: событие 0 — полное состояние до вызова скелета, каждое следующее — разность таблиц (`tail`: сколько записей ушло с начала и что добавлено
в конец в порядке касания; либо `full`). Событие создаётся, когда на границе записанного вызова меняется отпечаток таблиц (длины и последние ключи), поэтому изменения невидимых
вызовов свёрнуты в ближайшее событие. `SeamFile.memory(i)` восстанавливает таблицы события `i`.

Выборка: голова и шаг по каждому шву (`Sampling`), плюс всегда вызовы, которые платят (статьи бюджета), меняют память, бросают исключение или попадают в новый класс знака;
`--full` пишет ВСЕ вызовы названных швов у названных записей (целая лента закона кандидата тяжёлого домена для решающих ворот G1).

    python tools/native_skeleton_seams.py record --corpus field|synthetic [--meshes a,b] [--full EVALUATE_SPLIT,QUEUE --full-ids 000042,...] [--max-seconds S]
    python tools/native_skeleton_seams.py summary --corpus field|synthetic
    python tools/native_skeleton_seams.py verify --corpus field|synthetic [--limit N]       # вызовы шва -> эталон ещё раз, точное сравнение (проверка самих записей)
"""

from __future__ import annotations

import argparse
import functools
import json
import lzma
import os
import pickle
import sys
import time
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))  # `PYTHONSAFEPATH=1` каталог скрипта в путь не кладёт


def _third_party_path() -> None:
    try:
        import mpmath  # noqa: F401
    except ModuleNotFoundError:
        modules = Path(os.environ.get("APPDATA", "")) / "Blender Foundation" / "Blender" / "4.5" / "scripts" / "modules"
        if modules.is_dir():
            sys.path.append(str(modules))


_third_party_path()

import native_corpus as nc  # noqa: E402
import native_skeleton_corpus as sc  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
import cftuv_envelope.wavefront.candidate_law as candidate_law  # noqa: E402
import cftuv_envelope.wavefront.event_time as event_time  # noqa: E402
import cftuv_envelope.wavefront.events as events  # noqa: E402
import cftuv_envelope.wavefront.exact_candidate_view as candidate_view  # noqa: E402

SCHEMA = "cftuv.native-skeleton-seams.v1"
SAMPLED_CAP = 60_000
FULL_CAP = 600_000
SIGN_KEYS = tuple(exact.SIGN_COUNTS)
PRIMITIVES = (
    "COMPARE_TIMES", "CONCURRENCY_TIME", "SLIDING_TIME", "SLIDING_POINT", "EVENT_POINT", "EVENT_POINT_UNIVERSE", "TIME_NORMALIZED", "TIMES_ARE_EQUAL",
)
CANDIDATE_SEAMS = ("EVALUATE_SPLIT", "EVALUATE_EDGE")
QUEUE_SEAM = "QUEUE"
SEAM_NAMES = (*PRIMITIVES, *CANDIDATE_SEAMS, QUEUE_SEAM)
#: Имена параметров шва по порядку (`budget` — последний у функций с бюджетом: он идёт в цену, а не в аргументы).
PARAMETERS = {
    "COMPARE_TIMES": ("left", "right", "budget"),
    "CONCURRENCY_TIME": ("first", "second", "third", "budget"),
    "SLIDING_TIME": ("line", "along", "other", "budget"),
    "SLIDING_POINT": ("line", "along", "time", "budget"),
    "EVENT_POINT": ("first", "second", "time", "budget"),
    "EVENT_POINT_UNIVERSE": ("first", "second", "time", "prime_universe", "budget"),
    "TIME_NORMALIZED": ("dividend", "divisor", "budget"),
    "TIMES_ARE_EQUAL": ("left", "right"),
}
#: Оригиналы берутся при импорте модуля, до любой подмены.
ORIGINALS = {
    "COMPARE_TIMES": event_time.compare_times,
    "CONCURRENCY_TIME": event_time.concurrency_time,
    "SLIDING_TIME": event_time.sliding_time,
    "SLIDING_POINT": event_time.sliding_point,
    "EVENT_POINT": event_time.event_point,
    "EVENT_POINT_UNIVERSE": event_time._event_point_with_prime_universe,
    "TIME_NORMALIZED": event_time.EventTimeV1.__dict__["normalized"].__func__,
    "TIMES_ARE_EQUAL": event_time.times_are_equal,
    "EVALUATE_SPLIT": candidate_law.evaluate_split_candidate,
    "EVALUATE_EDGE": candidate_law.evaluate_edge_candidate,
}
_MISSING = object()


@dataclass
class Sampling:
    """Сколько вызовов шва писать: голова, шаг, потолок и потолок «интересных» (платящих, меняющих память, исключений, новых классов знака)."""

    head: int = 24
    stride: int = 64
    cap: int = 400
    interesting: int = 300
    class_head: int = 40

    def wants(self, seen: int, recorded: int) -> bool:
        return recorded < self.cap and (seen < self.head or seen % self.stride == 0)


SAMPLING = {
    "COMPARE_TIMES": Sampling(head=40, stride=211, cap=500, interesting=200, class_head=60),
    "TIMES_ARE_EQUAL": Sampling(head=20, stride=307, cap=150, interesting=0, class_head=0),
    "EVALUATE_SPLIT": Sampling(head=40, stride=89, cap=500, interesting=0, class_head=0),
    "EVALUATE_EDGE": Sampling(head=40, stride=89, cap=300, interesting=0, class_head=0),
}
DEFAULT_SAMPLING = Sampling(head=40, stride=37, cap=400, interesting=300, class_head=0)


# --------------------------------------------------------------------------
# Журнал памяти канонизации: разности таблиц
# --------------------------------------------------------------------------


def live_tables() -> dict:
    """Четыре таблицы памяти процесса как упорядоченные списки `(ключ, значение)`; известные простые — `(p, None)`."""

    return {
        "known_primes": [(prime, None) for prime in exact._KNOWN_PRIMES],
        "factorization": list(exact._FACTORIZATION_MEMO.items()),
        "squarefree": list(exact._SQUAREFREE_MEMO.items()),
        "prime_support": list(exact._PRIME_SUPPORT_MEMO.items()),
    }


def tables_of_state(state: nc.StateV1) -> dict:
    return {
        "known_primes": [(prime, None) for prime in state.known_primes],
        "factorization": list(state.factorization),
        "squarefree": list(state.squarefree),
        "prime_support": list(state.prime_support),
    }


def fingerprint() -> tuple:
    """Дешёвый отпечаток таблиц: длины и последние ключи (касание записи ставит её последней, вытеснение и вставка меняют длину или последний ключ)."""

    def last(table):
        return next(reversed(table)) if table else None

    known = exact._KNOWN_PRIMES
    return (
        len(known), known[-1] if known else None,
        len(exact._FACTORIZATION_MEMO), last(exact._FACTORIZATION_MEMO),
        len(exact._SQUAREFREE_MEMO), last(exact._SQUAREFREE_MEMO),
        len(exact._PRIME_SUPPORT_MEMO), last(exact._PRIME_SUPPORT_MEMO),
    )


def diff_items(old: list, new: list) -> tuple:
    """Разность двух упорядоченных списков пар: `("tail", ушло_с_начала, добавлено_в_конец)` либо `("full", новый)`."""

    position = {key: index for index, (key, _value) in enumerate(old)}
    last, stable = -1, 0
    for index, (key, value) in enumerate(new):
        at = position.get(key)
        if at is None or at <= last or old[at][1] != value:
            break
        last, stable = at, index + 1
    tail = new[stable:]
    moved = {key for key, _value in tail}
    rest = [item for item in old if item[0] not in moved]
    evicted = len(rest) - stable
    if evicted >= 0 and rest[evicted:] == new[:stable]:
        return ("tail", evicted, tail)
    return ("full", new)


def apply_diff(old: list, diff: tuple) -> list:
    if diff[0] == "full":
        return list(diff[1])
    _kind, evicted, tail = diff
    moved = {key for key, _value in tail}
    return [item for item in old if item[0] not in moved][evicted:] + list(tail)


class Journal:
    """События состояния памяти: `[0]` — полное состояние до вызова скелета, далее разности; индекс события — ссылка вызова."""

    def __init__(self, base: dict) -> None:
        self.events: list = [{"full": base}]
        self.tables = base
        self.fingerprint = fingerprint()
        self.index = 0

    def observe(self) -> int:
        """Индекс события, равного СЕЙЧАС состоянию памяти процесса (новое событие создаётся, только если отпечаток сменился)."""

        current = fingerprint()
        if current == self.fingerprint:
            return self.index
        live = live_tables()
        self.events.append({name: diff_items(self.tables[name], live[name]) for name in live})
        self.tables, self.fingerprint, self.index = live, current, len(self.events) - 1
        return self.index


# --------------------------------------------------------------------------
# Записывающие обёртки
# --------------------------------------------------------------------------


def _articles(budget) -> tuple:
    return (exact.UNBUDGETED_WORK if budget is None else budget).spent_by_article()


def _signs() -> tuple:
    counts = exact.SIGN_COUNTS
    return tuple(counts[key] for key in SIGN_KEYS)


def _difference(after: tuple, before: tuple) -> tuple:
    return tuple(new - old for new, old in zip(after, before))


def _budget_before(budget, articles: tuple):
    if budget is None:
        return None
    return {
        "mode": budget.mode.value, "cap": budget.cap, "stage": budget.stage, "domain_id": budget.domain_id,
        "superlevel": budget.superlevel, "articles": list(articles),
    }


def _arguments(seam: str, args: tuple, kwargs: dict) -> tuple:
    """`(аргументы по именам без бюджета, бюджет)` вызова функции шва."""

    names = PARAMETERS[seam]
    values = dict(zip(names, args))
    values.update(kwargs)
    budget = values.pop("budget", None)
    return values, budget


@dataclass
class SeamCall:
    """Записанный вызов шва (см. описание модуля)."""

    seam: str
    ordinal: int
    clock: int
    arguments: dict
    result: object
    error: tuple | None
    sign: tuple
    articles: tuple
    budget: dict | None
    mem_pre: int
    mem_post: int
    seconds: float
    extra: dict = field(default_factory=dict)

    def exception(self):
        return self.error


class SeamRecorder:
    """Ставит обёртки на швы (`installed`) и собирает вызовы одного воспроизведения записи корпуса."""

    def __init__(self, base: nc.StateV1, *, full: frozenset = frozenset(), sampling: dict | None = None, hard_cap: int = SAMPLED_CAP) -> None:
        self.journal = Journal(tables_of_state(base))
        self.full = full
        self.sampling = {**SAMPLING, **(sampling or {})}
        self.calls: list[SeamCall] = []
        self.seen: Counter = Counter()
        self.recorded: Counter = Counter()
        self.interesting: Counter = Counter()
        self.classes: dict = {}
        self.queue_ops: list = []
        self.clock = 0
        self.hard_cap = hard_cap
        self.errors: Counter = Counter()

    # ---- общий путь примитивов --------------------------------------------------

    def _wants(self, seam: str, seen: int) -> bool:
        if len(self.calls) >= self.hard_cap:
            return False
        return seam in self.full or self.sampling.get(seam, DEFAULT_SAMPLING).wants(seen, self.recorded[seam])

    def primitive(self, seam: str, original):
        recorder = self
        sampling = self.sampling.get(seam, DEFAULT_SAMPLING)

        @functools.wraps(original)
        def wrapper(*args, **kwargs):
            ordinal = recorder.seen[seam]
            recorder.seen[seam] = ordinal + 1
            recorder.clock += 1
            clock = recorder.clock
            values, budget = _arguments(seam, args, kwargs)
            signs0, articles0 = _signs(), _articles(budget)
            pre = recorder.journal.observe()
            started = time.perf_counter()
            try:
                result, error, raised = original(*args, **kwargs), None, None
            except Exception as exc:  # noqa: BLE001 - исключение эталона — часть исхода шва
                result, error, raised = None, (type(exc).__qualname__, str(exc)), exc
            seconds = time.perf_counter() - started
            signs1, articles1 = _signs(), _articles(budget)
            post = recorder.journal.observe()
            sign, spent = _difference(signs1, signs0), _difference(articles1, articles0)
            record = recorder._wants(seam, ordinal)
            if not record and sampling.interesting and recorder.interesting[seam] < sampling.interesting:
                kind = (any(spent), post != pre, error is not None)
                label = (sign[1:], kind)
                novel = sampling.class_head and recorder.classes.get((seam, label), 0) < sampling.class_head
                if any(kind) or novel:
                    recorder.classes[(seam, label)] = recorder.classes.get((seam, label), 0) + 1
                    recorder.interesting[seam] += 1
                    record = True
            if record and len(recorder.calls) < recorder.hard_cap:
                recorder.recorded[seam] += 1
                recorder.calls.append(
                    SeamCall(seam, ordinal, clock, values, result, error, sign, spent, _budget_before(budget, articles0), pre, post, seconds)
                )
            if raised is not None:
                raise raised
            return result

        return wrapper

    # ---- закон кандидата ----------------------------------------------------------

    def candidate(self, seam: str, original):
        recorder = self

        @functools.wraps(original)
        def wrapper(view, vertex_ref, other_ref, *, now, proof_identity_factory=None, **options):
            ordinal = recorder.seen[seam]
            recorder.seen[seam] = ordinal + 1
            recorder.clock += 1
            clock = recorder.clock
            if not recorder._wants(seam, ordinal):
                return original(view, vertex_ref, other_ref, now=now, proof_identity_factory=proof_identity_factory, **options)
            transcript = _ViewTranscript(view, proof_identity_factory)
            budget = view.budget
            signs0, articles0 = _signs(), _articles(budget)
            pre = recorder.journal.observe()
            started = time.perf_counter()
            try:
                with transcript.memo_tracking():
                    result, error, raised = (
                        original(transcript.view, vertex_ref, other_ref, now=now, proof_identity_factory=transcript.factory, **options), None, None,
                    )
            except Exception as exc:  # noqa: BLE001 - исключение закона — часть исхода
                result, error, raised = None, (type(exc).__qualname__, str(exc)), exc
            seconds = time.perf_counter() - started
            signs1, articles1 = _signs(), _articles(budget)
            post = recorder.journal.observe()
            values = {"vertex_ref": vertex_ref, "other_ref": other_ref, "now": now, **options}
            recorder.recorded[seam] += 1
            recorder.calls.append(
                SeamCall(
                    seam, ordinal, clock, values, result, error, _difference(signs1, signs0), _difference(articles1, articles0),
                    _budget_before(budget, articles0), pre, post, seconds, transcript.extra(),
                )
            )
            if raised is not None:
                raise raised
            return result

        return wrapper

    # ---- очередь ---------------------------------------------------------------------

    def queue_methods(self) -> dict:
        """Обёртки методов `EventQueueV1`: каждая операция — запись ленты (аргументы, результат, сдвиг знаков, порядок кучи после)."""

        recorder = self
        queue_class = events.EventQueueV1
        originals = {name: getattr(queue_class, name) for name in ("push", "pop_level", "_count_at_time", "peek_time")}

        def operation(name: str):
            original = originals[name]

            @functools.wraps(original)
            def wrapper(queue, *args):
                signs0 = _signs()
                recorder.clock += 1
                result = original(queue, *args)
                recorder.queue_ops.append(
                    {
                        "op": name, "clock": recorder.clock, "args": args, "result": result, "sign": _difference(_signs(), signs0),
                        "heap": tuple(entry.sequence for entry in queue._heap), "queue": id(queue),
                    }
                )
                return result

            return wrapper

        return {name: operation(name) for name in originals}

    # ---- подмена -------------------------------------------------------------------------

    def _targets(self) -> list:
        """`(объект, имя, новое значение)` для каждого шва: модули, держащие функцию по имени, методы классов."""

        plan: list = []
        wrappers = {seam: self.primitive(seam, ORIGINALS[seam]) for seam in PRIMITIVES if seam != "TIME_NORMALIZED"}
        wrappers.update({seam: self.candidate(seam, ORIGINALS[seam]) for seam in CANDIDATE_SEAMS})
        for seam, wrapper in wrappers.items():
            original = ORIGINALS[seam]
            for module in list(sys.modules.values()):
                if getattr(module, "__name__", "").startswith("cftuv_envelope"):
                    for attribute, value in list(vars(module).items()):
                        if value is original:
                            plan.append((module, attribute, wrapper))
        normalized = self.primitive("TIME_NORMALIZED", ORIGINALS["TIME_NORMALIZED"])
        plan.append((event_time.EventTimeV1, "normalized", staticmethod(normalized)))
        return plan

    def installed(self):
        return _Installed(self)


class _Installed:
    def __init__(self, recorder: SeamRecorder) -> None:
        self.recorder = recorder
        self.applied: list = []

    def __enter__(self):
        recorder = self.recorder
        plan = recorder._targets()
        plan.extend((events.EventQueueV1, name, method) for name, method in recorder.queue_methods().items())
        for owner, name, value in plan:
            self.applied.append((owner, name, owner.__dict__[name] if isinstance(owner, type) else getattr(owner, name)))
            setattr(owner, name, value)
        return recorder

    def __exit__(self, *_exception) -> None:
        for owner, name, original in reversed(self.applied):
            setattr(owner, name, original)


class _TrackedEntries:
    """Прокси словаря `PositionMemoV1.entries` на время вызова: какой запрос попал, какой нет (значения попаданий — после вызова)."""

    def __init__(self, inner: dict) -> None:
        self.inner = inner
        self.lookups: list = []

    def get(self, key, default=None):
        value = self.inner.get(key, _MISSING)
        self.lookups.append((key, value is not _MISSING, None if value is _MISSING else value))
        return default if value is _MISSING else value

    def __setitem__(self, key, value) -> None:
        self.inner[key] = value

    def __getitem__(self, key):
        return self.inner[key]

    def __contains__(self, key) -> bool:
        return key in self.inner

    def clear(self) -> None:
        self.inner.clear()


def _memo_kind(key) -> str:
    return key[0] if isinstance(key, tuple) and key and isinstance(key[0], str) else "POSITION"


class _ViewTranscript:
    """Вид закона кандидата с журналом ответов: по порядку `vertex_state`, `span_state`, `trace_bounds` и запросы памяти мест."""

    def __init__(self, view, factory) -> None:
        self.original = view
        self.log: list = []
        self.memory_effects = False
        self.identities: list = []
        self.tracked: _TrackedEntries | None = None
        self.factory = None if factory is None else self._factory(factory)
        self.view = candidate_view.ExactCandidateViewV1(
            view.prime_universe, self._answer("vertex_state", view.vertex_state), self._answer("span_state", view.span_state),
            self._bounds(view.trace_bounds), view.budget, view.position_memo,
        )

    def _measured(self, function, *arguments):
        """Вызов ответа вида с ценой: `(значение, исключение, (сдвиг знаков, сдвиг статей))`; память, изменённая ответом, помечается."""

        budget = self.original.budget
        signs0, articles0, fp0 = _signs(), _articles(budget), fingerprint()
        try:
            value, raised = function(*arguments), None
        except Exception as exc:  # noqa: BLE001 - исключение ответа идёт в журнал и дальше тем же объектом
            value, raised = None, exc
        if fingerprint() != fp0:
            self.memory_effects = True
        return value, raised, (_difference(_signs(), signs0), _difference(_articles(budget), articles0))

    def _answer(self, name: str, function):
        def answer(ref):
            value, raised, cost = self._measured(function, ref)
            if raised is not None:
                self.log.append((name, ref, ("raised", type(raised).__qualname__, str(raised)), cost))
                raise raised
            self.log.append((name, ref, ("value", value), cost))
            return value

        return answer

    def _bounds(self, function):
        def bounds(ref, time_):
            value, raised, cost = self._measured(function, ref, time_)
            if raised is not None:
                self.log.append(("trace_bounds", ref, time_, ("raised", type(raised).__qualname__, str(raised)), cost))
                raise raised
            self.log.append(("trace_bounds", ref, time_, ("value", value), cost))
            return value

        return bounds

    def _factory(self, function):
        def factory():
            value = function()
            self.identities.append(value)
            return value

        return factory

    def memo_tracking(self):
        return _MemoTracking(self)

    def extra(self) -> dict:
        memo = self.original.position_memo
        rows = []
        if self.tracked is not None:
            for key, hit, value in self.tracked.lookups:
                kind = _memo_kind(key)
                after = self.tracked.inner.get(key, _MISSING)
                objects = tuple(after[:3]) if kind != "POSITION" and after is not _MISSING else tuple(key)
                rows.append({"kind": kind, "hit": hit, "objects": objects, "value": None if after is _MISSING else (after[3] if kind != "POSITION" else after)})
        return {
            "view_log": self.log,
            "view_memory_effects": self.memory_effects,
            "identity_calls": self.identities,
            "identity_factory": self.factory is not None,
            "prime_universe": self.original.prime_universe,
            "memo": None if memo is None else {"prime_universe": memo.prime_universe, "lookups": rows},
        }


class _MemoTracking:
    def __init__(self, transcript: _ViewTranscript) -> None:
        self.transcript = transcript
        self.memo = transcript.original.position_memo

    def __enter__(self):
        if self.memo is not None:
            self.transcript.tracked = _TrackedEntries(self.memo.entries)
            object.__setattr__(self.memo, "entries", self.transcript.tracked)
        return self

    def __exit__(self, *_exception) -> None:
        if self.memo is not None:
            object.__setattr__(self.memo, "entries", self.transcript.tracked.inner)


# --------------------------------------------------------------------------
# Файл швов и читалка
# --------------------------------------------------------------------------


@dataclass
class SeamFile:
    """Прочитанный файл швов одной записи корпуса."""

    record: str
    python: str
    journal: list
    calls: list
    queue_ops: list
    seen: dict
    recorded: dict
    seconds: dict

    def memory(self, event: int) -> dict:
        """Таблицы памяти события `event`: `{таблица: [(ключ, значение)]}`; известные простые — `(p, None)`."""

        tables = dict(self.journal[0]["full"])
        for step in self.journal[1 : event + 1]:
            tables = {name: apply_diff(tables[name], step[name]) for name in tables}
        return tables

    def state_before(self, call: SeamCall) -> nc.StateV1:
        """Состояние процесса ДО вызова: память события `mem_pre`, бюджет вызова, счётчики знаков и неоплаченное нулевые (сравнивается сдвиг)."""

        tables = self.memory(call.mem_pre)
        budget = None
        if call.budget is not None:
            budget = {**{key: call.budget[key] for key in ("mode", "cap", "stage", "domain_id", "superlevel")}, "articles": tuple(call.budget["articles"])}
        return nc.StateV1(
            [key for key, _ in tables["known_primes"]], tables["factorization"], tables["squarefree"], tables["prime_support"], budget,
            {key: 0 for key in SIGN_KEYS}, (0,) * 6, None, False,
        )

    def by_seam(self, seam: str) -> list:
        return [call for call in self.calls if call.seam == seam]


def write_file(path: Path, recorder: SeamRecorder, record_id: str, seconds: dict) -> int:
    payload = {
        "schema": SCHEMA, "record": record_id, "python": sys.version.split()[0], "journal": recorder.journal.events,
        "calls": [call.__dict__ for call in recorder.calls], "queue_ops": recorder.queue_ops,
        "seen": dict(recorder.seen), "recorded": dict(recorder.recorded), "seconds": seconds,
    }
    body = lzma.compress(pickle.dumps(payload, protocol=5), format=lzma.FORMAT_XZ, preset=3)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(body)
    return len(body)


def read_file(path: Path) -> SeamFile:
    payload = pickle.loads(lzma.decompress(Path(path).read_bytes()))
    if payload.get("schema") != SCHEMA:
        raise nc.CorpusError(f"{path} is not a seam file of schema {SCHEMA}")
    return SeamFile(
        payload["record"], payload["python"], payload["journal"], [SeamCall(**item) for item in payload["calls"]], payload["queue_ops"],
        payload["seen"], payload["recorded"], payload["seconds"],
    )


def seams_root(corpus: Path) -> Path:
    return Path(corpus) / sc.SEAMS_DIR


def load_index(corpus: Path, out: Path | None = None) -> dict:
    return json.loads(((Path(out) if out else seams_root(corpus)) / "index.json").read_text(encoding="utf-8"))


def iter_files(corpus: Path, *, mesh: str | None = None, limit: int | None = None, out: Path | None = None):
    """`(строка индекса швов, SeamFile)` по файлам корпуса."""

    count = 0
    for row in load_index(corpus, out)["files"]:
        if mesh is not None and row["mesh"] != mesh:
            continue
        yield row, read_file((Path(out) if out else seams_root(corpus)) / row["path"])
        count += 1
        if limit and count >= limit:
            return


def iter_calls(corpus: Path, seam: str | None = None, *, mesh: str | None = None, limit: int | None = None):
    """`(SeamFile, SeamCall)` по вызовам шва `seam` (все швы, если `None`)."""

    for _row, seam_file in iter_files(corpus, mesh=mesh):
        for call in seam_file.calls:
            if seam is None or call.seam == seam:
                yield seam_file, call
                if limit:
                    limit -= 1
                    if limit == 0:
                        return


# --------------------------------------------------------------------------
# Запись по корпусу
# --------------------------------------------------------------------------


def record_corpus(
    corpus: Path, *, meshes=(), ids=(), full: frozenset = frozenset(), full_ids=(), max_seconds: float = 0.0, limit: int = 0, out: Path | None = None
) -> dict:
    """Воспроизводит записи корпуса эталоном со швами; пишет файлы и `index.json` (в `<корпус>/seams` либо `out`); возвращает индекс."""

    rows = sc.rows_of(corpus)
    if meshes:
        rows = [row for row in rows if row["mesh"] in meshes]
    if ids:
        rows = [row for row in rows if row["id"].split("-")[0] in set(ids) or row["id"] in set(ids)]
    if max_seconds:
        rows = [row for row in rows if row["seconds"] <= max_seconds]
    rows = rows[:limit] if limit else rows
    if "all" in full:
        full = frozenset(SEAM_NAMES)
    out = Path(out) if out else seams_root(corpus)
    out.mkdir(parents=True, exist_ok=True)
    for stale in out.glob("*.seams"):
        stale.unlink()
    files, totals = [], Counter()
    for number, row in enumerate(rows, 1):
        record = sc.read(corpus, row)
        before = record.before()
        # производная запись — обрыв прогона основной записи: её вызовы повторяют начало основной ленты, поэтому полная лента только у основных, у производных — выборка и каждое исключение
        is_full = row.get("derived") is None and ("all" in full_ids or row["id"].split("-")[0] in set(full_ids) or row["id"] in set(full_ids))
        recorder = SeamRecorder(before, full=full if is_full else frozenset(), hard_cap=FULL_CAP if is_full else SAMPLED_CAP)
        call = nc.prepare_call(record.op, record.call_blob, before)
        with recorder.installed():
            started = time.perf_counter()
            outcome = nc.execute(call)
            wall = time.perf_counter() - started
        expected = record.expected()
        if nc.outcome_digest(record.op, before, outcome) != nc.outcome_digest(record.op, before, expected):
            raise nc.CorpusError(f"record {row['id']} differs from its recorded outcome while the seam wrappers are installed")
        path = out / f"{row['id']}.seams"
        size = write_file(path, recorder, row["id"], {"oracle": row["seconds"], "instrumented": wall})
        entry = {"record": row["id"], "mesh": row["mesh"], "path": path.name, "bytes": size, "recorded": dict(recorder.recorded), "seen": dict(recorder.seen),
                 "queue_ops": len(recorder.queue_ops), "journal_events": len(recorder.journal.events), "full": is_full,
                 "seconds_oracle": row["seconds"], "seconds_instrumented": wall}
        files.append(entry)
        totals.update(recorder.recorded)
        totals["bytes"] += size
        if number % 20 == 0:
            print(f"  seams {number}/{len(rows)} {dict(totals)}", flush=True)
    index = {
        "schema": SCHEMA, "kernel_identity": nc.clip_memo.kernel_code_identity(), "python": sys.version.split()[0],
        "files": files, "recorded": {name: totals[name] for name in SEAM_NAMES if totals[name]},
        "seen": dict(sum((Counter(entry["seen"]) for entry in files), Counter())),
        "queue_ops": sum(entry["queue_ops"] for entry in files), "bytes": totals["bytes"],
    }
    (out / "index.json").write_text(json.dumps(index, indent=0, sort_keys=True) + "\n", encoding="utf-8")
    return index


# --------------------------------------------------------------------------
# Проверка записей: вызов шва -> эталон ещё раз
# --------------------------------------------------------------------------


def _difference_text(name: str, expected, actual) -> str:
    return f"{name}: expected {nc._clip_text(nc.canonical(expected))} got {nc._clip_text(nc.canonical(actual))}"


def verify_call(seam_file: SeamFile, call: SeamCall) -> list:
    """Расхождения повторного вызова с записанным (пусто — равны точно): результат, исключение, цена, память после."""

    state = seam_file.state_before(call)
    budget, _store = nc.restore_state(state)
    found = []
    if call.seam in CANDIDATE_SEAMS:
        try:
            result, error = _replay_candidate(call, budget)
        except nc.CorpusError as exc:
            return [str(exc)]
    else:
        arguments = dict(call.arguments)
        arguments_for = {name: arguments[name] for name in PARAMETERS[call.seam] if name in arguments}
        if "budget" in PARAMETERS[call.seam]:
            arguments_for["budget"] = budget
        signs0, articles0 = _signs(), _articles(budget)
        try:
            result, error = ORIGINALS[call.seam](**arguments_for), None
        except Exception as exc:  # noqa: BLE001
            result, error = None, (type(exc).__qualname__, str(exc))
        found.extend(_compare_cost(call, signs0, articles0, budget))
    if error != call.error:
        found.append(f"exception: {call.error!r} != {error!r}")
    if nc.canonical(result) != nc.canonical(call.result):
        found.append(_difference_text("result", call.result, result))
    after = live_tables()
    expected_after = seam_file.memory(call.mem_post)
    if call.extra.get("view_memory_effects"):
        return found  # ответы вида меняли память канонизации: этот вызов нативный лист по записи вида не воспроизводит
    for name in after:
        if nc.canonical(after[name]) != nc.canonical(expected_after[name]):
            found.append(f"memory.{name}: differs")
    return found


def _compare_cost(call: SeamCall, signs0: tuple, articles0: tuple, budget) -> list:
    found = []
    sign = _difference(_signs(), signs0)
    if sign != call.sign:
        found.append(f"sign_counts: {call.sign} != {sign}")
    spent = _difference(_articles(budget), articles0)
    if spent != call.articles:
        found.append(f"budget: {call.articles} != {spent}")
    return found


def _replay_candidate(call: SeamCall, budget):
    """Закон кандидата на ЖУРНАЛЕ вида: ответы вида по порядку, память мест из попаданий, фабрика тождества из записи."""

    extra = call.extra
    log = iter(extra["view_log"])
    callback_cost = [[0] * len(SIGN_KEYS), [0] * 6]

    def charge(cost) -> None:
        for slot, part in zip(callback_cost, cost):
            for index, value in enumerate(part):
                slot[index] += value

    def outcome_of(entry_outcome):
        if entry_outcome[0] == "raised":
            raise RuntimeError(entry_outcome[2])
        return entry_outcome[1]

    def answer(name):
        def respond(ref):
            kind, logged_ref, outcome, cost = next(log)
            if kind != name or logged_ref != ref:
                raise nc.CorpusError(f"view log diverges: wanted {name}({ref!r}), recorded {kind}({logged_ref!r})")
            charge(cost)
            return outcome_of(outcome)

        return respond

    def bounds(ref, time_):
        kind, logged_ref, logged_time, outcome, cost = next(log)
        if kind != "trace_bounds" or logged_ref != ref or nc.canonical(logged_time) != nc.canonical(time_):
            raise nc.CorpusError("view log diverges at trace_bounds")
        charge(cost)
        return outcome_of(outcome)

    memo = None
    if extra["memo"] is not None:
        memo = candidate_view.PositionMemoV1(extra["memo"]["prime_universe"])
        asked: set = set()
        for row in extra["memo"]["lookups"]:
            key = _memo_key(row)
            # запись, которой вызов сам же и положил (промах, потом попадание), до вызова в памяти не было
            if row["hit"] and key not in asked:
                memo.entries[key] = row["value"] if row["kind"] == "POSITION" else (*row["objects"], row["value"])
            asked.add(key)
    identities = iter(extra["identity_calls"])
    factory = (lambda: next(identities)) if extra["identity_factory"] else None
    view = candidate_view.ExactCandidateViewV1(extra["prime_universe"], answer("vertex_state"), answer("span_state"), bounds, budget, memo)
    values = dict(call.arguments)
    original = ORIGINALS[call.seam]
    signs0, articles0 = _signs(), _articles(budget)
    try:
        result, error = original(view, values.pop("vertex_ref"), values.pop("other_ref"), proof_identity_factory=factory, **values), None
    except Exception as exc:  # noqa: BLE001
        result, error = None, (type(exc).__qualname__, str(exc))
    # цена ответов вида (знаки и статьи внутри `trace_bounds`, `span_state`) в нативный лист не входит: она записана в журнале вида и прибавляется к измеренной
    spent = _difference(_articles(budget), articles0)
    sign = _difference(_signs(), signs0)
    problems = []
    if tuple(a + b for a, b in zip(sign, callback_cost[0])) != call.sign:
        problems.append(f"sign_counts: {call.sign} != {tuple(a + b for a, b in zip(sign, callback_cost[0]))}")
    if tuple(a + b for a, b in zip(spent, callback_cost[1])) != call.articles:
        problems.append(f"budget: {call.articles} != {tuple(a + b for a, b in zip(spent, callback_cost[1]))}")
    if problems:
        raise nc.CorpusError(f"candidate call {call.seam}#{call.ordinal}: " + "; ".join(problems))
    return result, error


def _memo_key(row: dict):
    from cftuv_envelope.wavefront.exact_identity import ExactIdentityKeyV1

    if row["kind"] == "POSITION":
        return ExactIdentityKeyV1(row["objects"])
    return (row["kind"], *(id(item) for item in row["objects"]))


def verify_queue(seam_file: SeamFile) -> list:
    """Лента операций очереди -> свежая очередь эталона: результат каждой операции, сдвиг знаков и порядок кучи после неё (пусто — равны точно)."""

    if not seam_file.queue_ops:
        return []
    tables = seam_file.memory(0)
    nc.restore_state(
        nc.StateV1(
            [key for key, _ in tables["known_primes"]], tables["factorization"], tables["squarefree"], tables["prime_support"], None,
            {key: 0 for key in SIGN_KEYS}, (0,) * 6, None, False,
        )
    )
    queue = events.EventQueueV1(work_budget=None)
    found = []
    for number, operation in enumerate(seam_file.queue_ops):
        signs0 = _signs()
        result = getattr(queue, operation["op"])(*operation["args"])
        if nc.canonical(result) != nc.canonical(operation["result"]):
            found.append(f"queue op {number} {operation['op']}: result differs")
        if _difference(_signs(), signs0) != operation["sign"]:
            found.append(f"queue op {number} {operation['op']}: sign_counts {operation['sign']} != {_difference(_signs(), signs0)}")
        if tuple(entry.sequence for entry in queue._heap) != operation["heap"]:
            found.append(f"queue op {number} {operation['op']}: heap order differs")
        if len(found) >= 5:
            break
    return found


def verify_corpus(corpus: Path, limit: int | None = None, out: Path | None = None) -> dict:
    """Все записанные вызовы всех файлов -> эталон ещё раз, и ленты очередей; `{проверено по швам, расхождения}`."""

    checked: Counter = Counter()
    problems: list = []
    for _row, seam_file in iter_files(corpus, out=out):
        queue_found = verify_queue(seam_file)
        checked[QUEUE_SEAM] += len(seam_file.queue_ops)
        if queue_found:
            problems.append({"record": seam_file.record, "seam": QUEUE_SEAM, "ordinal": 0, "found": queue_found[:3]})
        for call in seam_file.calls:
            if limit and sum(checked.values()) >= limit:
                break
            try:
                found = verify_call(seam_file, call)
            except Exception as exc:  # noqa: BLE001
                found = [f"verify raised {type(exc).__qualname__}: {exc}"]
            checked[call.seam] += 1
            if found:
                problems.append({"record": seam_file.record, "seam": call.seam, "ordinal": call.ordinal, "found": found[:3]})
    return {"checked": dict(checked), "problems": problems[:50], "problem_count": len(problems)}


def summary_of(corpus: Path, out: Path | None = None) -> dict:
    index = load_index(corpus, out)
    return {key: index[key] for key in ("recorded", "seen", "queue_ops", "bytes", "python")} | {"files": len(index["files"])}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("record", "summary", "verify"))
    parser.add_argument("--corpus", required=True)
    parser.add_argument("--meshes", default="")
    parser.add_argument("--ids", default="")
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--full", default="")
    parser.add_argument("--full-ids", default="")
    parser.add_argument("--max-seconds", type=float, default=0.0)
    parser.add_argument("--limit", type=int, default=0)
    arguments = parser.parse_args(argv)
    corpus = sc.matching(arguments.corpus) if arguments.corpus in sc.KINDS else Path(arguments.corpus)
    if corpus is None:
        raise SystemExit(f"NATIVE_SKELETON_SEAMS_FAILED {sc.describe_missing(arguments.corpus)}")
    if arguments.command == "record":
        index = record_corpus(
            corpus, meshes=[item for item in arguments.meshes.split(",") if item], ids=[item for item in arguments.ids.split(",") if item], out=arguments.out, full=frozenset(item for item in arguments.full.split(",") if item),
            full_ids=[item for item in arguments.full_ids.split(",") if item], max_seconds=arguments.max_seconds, limit=arguments.limit,
        )
        print(json.dumps({key: index[key] for key in ("recorded", "seen", "queue_ops", "bytes")}))
        print("NATIVE_SKELETON_SEAMS_OK", len(index["files"]), index["bytes"])
        return 0
    if arguments.command == "summary":
        print(json.dumps(summary_of(corpus, arguments.out), indent=1))
        return 0
    report = verify_corpus(corpus, arguments.limit or None, arguments.out)
    print(json.dumps(report, indent=1, default=str))
    print("NATIVE_SKELETON_SEAMS_VERIFY_OK" if not report["problem_count"] else "NATIVE_SKELETON_SEAMS_VERIFY_FAILED", sum(report["checked"].values()))
    return 0 if not report["problem_count"] else 1


if __name__ == "__main__":
    sys.exit(main())
