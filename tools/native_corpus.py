"""Корпус вызовов для нативного ускорителя ядра: запись, восстановление состояния процесса, эталон, точное сравнение.

Единица замены нативным кодом — ЦЕЛАЯ операция ядра: `wavefront.coverage._coverage_at` и `materialize.clip.clip_geometry`
(ровно то, что `clip_memo.run_clip` запоминает). Ответ операции — не только её результат: цена (шесть статей бюджета),
память канонизации (`_KNOWN_PRIMES` и три словаря, порядок вставки входит), счётчики знаков (`SIGN_COUNTS`), телеметрия
неоплаченной работы (`UNBUDGETED_WORK`) и запись `prime-universe` в `store` — тоже ответ: за них платят стадии ПОСЛЕ операции.
Поэтому запись корпуса держит всё, что операция читает и пишет мимо аргументов, ДО и ПОСЛЕ вызова.

Модуль не знает ни Blender, ни нативной сборки (ядро питона — эталон): запись (`Recorder`, ставится вместо операции),
восстановление состояния (`restore_state`), эталонное воспроизведение (`prepare_call` + `execute`) и сравнение двух исходов
(`compare_outcomes`: результат каноническим кодом `clip_memo._encode`, где `int` и `Fraction` различны, а `float` — по
`hex`; исключение; цена; память с порядком; знаки; неоплаченное; `store`). Нативная сторона строит такой же `Outcome` и
отдаёт его тому же сравнению. Импортируйте модуль ПОСЛЕ того, как дерево ядра загружено (хост-инструмент `_load_tree`):
операции эталона берутся из загруженных модулей в момент импорта.

Формат записи: `CFTUVNC1 | uint32 длина заголовка | заголовок JSON | xz(pickle(payload))`; в payload только словари, кортежи,
байты и объекты ядра (классы этого модуля в пикл не попадают, поэтому читать запись можно под любым именем модуля).
Бюджет транзакции в пиклах входов подменён постоянным идентификатором и при чтении встаёт на пересобранный бюджет с теми же
режимом, потолком, стадией, идентичностями и статьями: тождество `plane._budget is budget` сохраняется.
"""

from __future__ import annotations

import contextlib
import dataclasses
import hashlib
import io
import json
import lzma
import os
import pickle
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
KERNEL_SOURCE = ROOT / "kernel" / "src"
if str(KERNEL_SOURCE) not in sys.path:
    sys.path.insert(0, str(KERNEL_SOURCE))

import cftuv_envelope._radicand_products as radicand_products  # noqa: E402
import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
import cftuv_envelope.float_filter as float_filter  # noqa: E402
import cftuv_envelope.materialize.clip as clip  # noqa: E402
import cftuv_envelope.materialize.clip_memo as clip_memo  # noqa: E402
import cftuv_envelope.wavefront.coverage as coverage  # noqa: E402
import cftuv_envelope.wavefront.exact_identity as exact_identity  # noqa: E402
import cftuv_envelope.wavefront.skeleton as skeleton  # noqa: E402
import cftuv_envelope.wavefront.symbolic_superlevel_coordinator as coordinator  # noqa: E402

OP_COVERAGE = "coverage_at"
OP_CLIP = "clip_geometry"
OP_SKELETON = "build_skeleton"
#: Операции полевого корпуса покрытия и резки: ровно их воспроизводят замеры (`native_bench`); скелет живёт в своём корпусе (`native_skeleton_*`).
OPERATIONS = (OP_COVERAGE, OP_CLIP)
SKELETON_OPERATIONS = (OP_SKELETON,)
#: Операции эталона: берутся в момент импорта, поэтому подмена модулей рекордером (`Recorder.wrap`) их не задевает.
ORACLE = {OP_COVERAGE: coverage._coverage_at, OP_CLIP: clip.clip_geometry, OP_SKELETON: skeleton.build_skeleton}
SKELETON_OPTIONS = frozenset({"split_search", "work_budget", "dense_hydration"})

RECORD_MAGIC = b"CFTUVNC1"
RECORD_SCHEMA = "cftuv.native-corpus.v1"
PRIME_UNIVERSE_KEY = "prime-universe"
MEMORY_TABLES = ("known_primes", "factorization", "squarefree", "prime_support")
DEFAULT_CORPUS_BASE = "E:/cftuv_native_corpus"
CORPUS_ENVIRONMENT = "CFTUV_NATIVE_CORPUS"
DEFAULT_MAX_BYTES = 1_500_000_000
DEFAULT_PRESET = 3
_ARTICLES = (
    "modular_squarings",
    "gcd_operations",
    "miller_rabin_rounds",
    "pollard_attempts",
    "radical_materializations",
    "exact_position_hydrations",
)
_BUDGET_FIELDS = ("mode", "cap", "stage", "domain_id", "superlevel")


class CorpusError(RuntimeError):
    """Корпус нарушен: именованный отказ записи или чтения, а не молчаливое «почти»."""


# --------------------------------------------------------------------------
# Каноническая запись и состояние процесса
# --------------------------------------------------------------------------


def canonical(value) -> str:
    """Точная запись значения: тип и значение каждого узла (`clip_memo._encode`: int != Fraction, float по `hex`)."""

    out: list = []
    clip_memo._encode(value, out)
    return "".join(out)


@dataclass
class StateV1:
    """Состояние процесса, которое операция читает и пишет мимо аргументов (память — упорядоченными списками)."""

    known_primes: list
    factorization: list
    squarefree: list
    prime_support: list
    budget: dict | None
    sign_counts: dict
    unbudgeted: tuple
    store: list | None
    canonical_audit: bool
    #: Представление ключей тождества (`exact_identity`): модульный переключатель, который читает скелет; поле дописано позже, поэтому у старых записей умолчание.
    identity_mode: str = "CACHED"
    #: Самопроверка повтора замыкания (`CFTUV_SYMBOLIC_REPLAY_CHECK`, `coordinator.replay_check_enabled()`): среда процесса, которую эталон читает при КАЖДОМ замыкании пакета и от которой зависит цена
    #: скелета (повтор удваивает счёт замыкания). Набор тестов ядра включает её, продукт и поле — нет, поэтому запись обязана нести то, что было при вызове; поле дописано позже, умолчание — продукт.
    replay_check: bool = False

    def as_payload(self) -> dict:
        return {item.name: getattr(self, item.name) for item in dataclasses.fields(self)}


def _is_prime_universe_key(key) -> bool:
    head = key[0] if isinstance(key, tuple) and key else key
    return isinstance(head, str) and head.startswith(PRIME_UNIVERSE_KEY)


def budget_state(budget) -> dict | None:
    """Режим, потолок, стадия, идентичности и шесть статей бюджета (`None` — вызов без бюджета)."""

    if budget is None:
        return None
    return {
        "mode": budget.mode.value,
        "cap": budget.cap,
        "stage": budget.stage,
        "domain_id": budget.domain_id,
        "superlevel": budget.superlevel,
        "articles": budget.spent_by_article(),
    }


def capture_state(budget, store) -> StateV1:
    """Снимок состояния процесса: память канонизации с порядком, бюджет, знаки, неоплаченное, записи `prime-universe`."""

    if set(exact._KNOWN_PRIMES) != exact._KNOWN_PRIME_SET:
        raise CorpusError("the known-prime list and the known-prime set of the process disagree")
    entries = None if store is None else [(key, value) for key, value in store.items() if _is_prime_universe_key(key)]
    return StateV1(
        list(exact._KNOWN_PRIMES),
        list(exact._FACTORIZATION_MEMO.items()),
        list(exact._SQUAREFREE_MEMO.items()),
        list(exact._PRIME_SUPPORT_MEMO.items()),
        budget_state(budget),
        dict(exact.SIGN_COUNTS),
        exact.UNBUDGETED_WORK.spent_by_article(),
        entries,
        exact.canonical_audit_enabled(),
        exact_identity.identity_mode().value,
        coordinator.replay_check_enabled(),
    )


def _set_articles(budget, articles) -> None:
    for name, value in zip(_ARTICLES, articles):
        setattr(budget, name, value)


def build_budget(state: dict | None):
    """Пересобранный `ExactWorkBudgetV1` с теми же режимом, потолком, стадией, идентичностями и статьями."""

    if state is None:
        return None
    budget = exact.ExactWorkBudgetV1(
        mode=exact.ExactWorkBudgetModeV1(state["mode"]),
        cap=state["cap"],
        stage=state["stage"],
        domain_id=state["domain_id"],
        superlevel=state["superlevel"],
    )
    _set_articles(budget, state["articles"])
    return budget


def set_replay_check(enabled: bool) -> None:
    """Ставит среду процесса так, как её прочтёт `replay_check_enabled()` эталона (и нативная вставка: шим читает ту же функцию при вызове)."""

    os.environ[coordinator.ENVIRONMENT_REPLAY_CHECK] = "1" if enabled else "0"


def restore_state(state: StateV1):
    """Ставит процесс в `state`: `(бюджет, store)` для вызова. Чистые кэши (центры binary64, произведения радикандов) сброшены."""

    exact.reset_factorization_memory()
    float_filter.clear_table()
    radicand_products.clear_products()
    exact._KNOWN_PRIMES.extend(state.known_primes)
    exact._KNOWN_PRIME_SET.update(state.known_primes)
    exact._FACTORIZATION_MEMO.update(state.factorization)
    exact._SQUAREFREE_MEMO.update(state.squarefree)
    exact._PRIME_SUPPORT_MEMO.update(state.prime_support)
    exact.SIGN_COUNTS.update(state.sign_counts)
    _set_articles(exact.UNBUDGETED_WORK, state.unbudgeted)
    exact.set_canonical_audit(state.canonical_audit)
    exact_identity.set_identity_mode(state.identity_mode)
    set_replay_check(state.replay_check)
    store = None if state.store is None else dict(state.store)
    return build_budget(state.budget), store


# --------------------------------------------------------------------------
# Вызов: аргументы, пикл входов, исполнение эталона
# --------------------------------------------------------------------------


@dataclass
class Call:
    """Один вызов операции: позиционные и именованные аргументы (без бюджета и `store`), бюджет, `store`."""

    op: str
    args: tuple
    kwargs: dict
    budget: object
    store: dict | None


class _CallPickler(pickle.Pickler):
    """Пикл входов, в котором бюджет транзакции и телеметрия неоплаченного — постоянные идентификаторы, а не значения."""

    def __init__(self, file, budget) -> None:
        super().__init__(file, protocol=5)
        self._budget = budget

    def persistent_id(self, obj):
        if obj is exact.UNBUDGETED_WORK:
            return "unbudgeted"
        if self._budget is not None and obj is self._budget:
            return "budget"
        return None


class _CallUnpickler(pickle.Unpickler):
    def __init__(self, file, budget) -> None:
        super().__init__(file)
        self._budget = budget

    def persistent_load(self, pid):
        if pid == "unbudgeted":
            return exact.UNBUDGETED_WORK
        if pid == "budget":
            return self._budget
        raise CorpusError(f"unknown persistent id {pid!r}")


def encode_call(call: Call) -> bytes:
    buffer = io.BytesIO()
    _CallPickler(buffer, call.budget).dump((call.args, call.kwargs))
    return buffer.getvalue()


def decode_call(op: str, blob: bytes, budget, store) -> Call:
    args, kwargs = _CallUnpickler(io.BytesIO(blob), budget).load()
    return Call(op, args, kwargs, budget, store)


def skeleton_kwargs(polygon, given: dict) -> dict:
    """Именованные входы `build_skeleton` в записи: режим поиска, плотная гидратация и ДЕЙСТВУЮЩАЯ граница уровней.

    `level_budget` — не аргумент `build_skeleton`, а функция модуля, которую `_Builder.run` зовёт по имени: тест, подменивший её
    (`test_wavefront_event_queue.py` ставит единицу), меняет исход, и запись обязана нести значение, которым вызов шёл на самом деле.
    Воспроизведение ставит его тем же способом (`pinned_level_budget`), а нативная вставка получает целое число аргументом."""

    unknown = set(given) - SKELETON_OPTIONS
    if unknown:
        raise CorpusError(f"build_skeleton is called with unknown options {sorted(unknown)}")
    return {
        "split_search": given.get("split_search", skeleton.SplitSearch.MOTORCYCLE),
        "dense_hydration": bool(given.get("dense_hydration", False)),
        "level_budget": skeleton.level_budget(polygon),
    }


@contextlib.contextmanager
def pinned_level_budget(limit):
    """На время вызова `skeleton.level_budget` отвечает `limit` (`None` — штатная функция); имя модуля возвращается в точности."""

    original = skeleton.level_budget
    if limit is None:
        yield
        return
    skeleton.level_budget = lambda _polygon: limit
    try:
        yield
    finally:
        skeleton.level_budget = original


def unpack_call(op: str, args: tuple, kwargs: dict) -> Call:
    """Аргументы подменяемой функции -> `Call`: `_coverage_at(partition, alpha, budget, store[, traces])`, `clip_geometry(plane, budget, **)`, `build_skeleton(polygon, **)`.

    `traces` (запись знаков для шаблона покрытия шага ширины, `materialize.step`) в вызов не входит: список лишь наполняется, ответ, цену и память он
    не меняет, а воспроизведённый без него вызов считает то же самое."""

    if op == OP_COVERAGE:
        partition, alpha, budget, store, *_traces = args
        if len(_traces) > 1:
            raise CorpusError("coverage._coverage_at takes at most five positional arguments")
        if kwargs:
            raise CorpusError("coverage._coverage_at is called with positional arguments only")
        return Call(op, (partition, alpha), {}, budget, store)
    if op == OP_SKELETON:
        (polygon,) = args
        return Call(op, (polygon,), skeleton_kwargs(polygon, kwargs), kwargs.get("work_budget"), None)
    plane, budget = args
    return Call(op, (plane,), dict(kwargs), budget, None)


def invoke(call: Call, function=None):
    """Результат операции эталона (ядро питона) на этом вызове; `function` подменяет операцию (нативная вставка с той же сигнатурой)."""

    if call.op == OP_COVERAGE:
        return (function or ORACLE[OP_COVERAGE])(call.args[0], call.args[1], call.budget, call.store)
    if call.op == OP_SKELETON:
        options = dict(call.kwargs)
        with pinned_level_budget(options.pop("level_budget", None)):
            return (function or ORACLE[OP_SKELETON])(call.args[0], work_budget=call.budget, **options)
    return (function or ORACLE[OP_CLIP])(call.args[0], call.budget, **call.kwargs)


def answer_view(op: str, result):
    """Результат без полей, которые не ответ: `work_budget` покрытия, метка запуска `memo` резки."""

    if op == OP_COVERAGE:
        return dataclasses.replace(result, work_budget=None)
    if op == OP_SKELETON:
        return result
    return dataclasses.replace(result, memo="")


def observe(call: Call) -> dict:
    """Что вызов оставил в аргументах: у резки — нормали смещения плоскости (`replay_lifted` ставит их при попадании)."""

    if call.op != OP_CLIP:
        return {}
    normals = getattr(call.args[0], "_normal_by_position", None)
    return {} if normals is None else {"plane_normals": canonical(list(normals.items()))}


@dataclass
class Outcome:
    """Исход вызова: результат либо исключение (тип и текст), состояние ПОСЛЕ, наблюдения за аргументами, секунды."""

    result: object | None
    exception: tuple | None
    after: StateV1
    observed: dict
    seconds: float = 0.0


def execute(call: Call, function=None) -> Outcome:
    """Эталонный вызов (или `function` с той же сигнатурой): время меряет только сама операция; состояние после снимается вне замера."""

    started = time.perf_counter()
    try:
        result, error = invoke(call, function), None
    except Exception as exc:  # noqa: BLE001 - исключение операции — часть её исхода
        result, error = None, (type(exc).__qualname__, str(exc))
    seconds = time.perf_counter() - started
    return Outcome(result, error, capture_state(call.budget, call.store), observe(call), seconds)


def prepare_call(op: str, blob: bytes, before: StateV1) -> Call:
    """Ставит процесс в состояние ДО вызова и собирает вход из пикла заново (вызов портит свои аргументы)."""

    budget, store = restore_state(before)
    return decode_call(op, blob, budget, store)


# --------------------------------------------------------------------------
# Сравнение исходов
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Difference:
    """Расхождение двух исходов: имя поля и описание (первое различающееся место)."""

    field: str
    detail: str

    def __str__(self) -> str:
        return f"{self.field}: {self.detail}"


def _clip_text(text: str, limit: int = 160) -> str:
    return text if len(text) <= limit else text[:limit] + f"...(+{len(text) - limit})"


def _locate(expected, actual) -> str:
    """Где именно расходятся два значения: длины, первый различающийся элемент или ключ."""

    if isinstance(expected, (list, tuple)) and isinstance(actual, (list, tuple)):
        if len(expected) != len(actual):
            return f"length {len(expected)} != {len(actual)}"
        for index, (left, right) in enumerate(zip(expected, actual)):
            first, second = canonical(left), canonical(right)
            if first != second:
                return f"[{index}] expected {_clip_text(first)} got {_clip_text(second)}"
    if isinstance(expected, dict) and isinstance(actual, dict):
        if list(expected) != list(actual):
            return "keys or key order differ"
        for key in expected:
            first, second = canonical(expected[key]), canonical(actual[key])
            if first != second:
                return f"[{key!r}] expected {_clip_text(first)} got {_clip_text(second)}"
    return f"expected {_clip_text(canonical(expected))} got {_clip_text(canonical(actual))}"


def _compare_results(op: str, expected, actual) -> list:
    if expected is None or actual is None:
        return [] if expected is actual else [Difference("result", "present in one outcome only")]
    left, right = answer_view(op, expected), answer_view(op, actual)
    if type(left) is not type(right):
        return [Difference("result", f"type {type(left).__qualname__} != {type(right).__qualname__}")]
    found = []
    for item in dataclasses.fields(left):
        first, second = getattr(left, item.name), getattr(right, item.name)
        if canonical(first) != canonical(second):
            found.append(Difference(f"result.{item.name}", _locate(first, second)))
    return found


def _delta(after: tuple, before: tuple) -> tuple:
    return tuple(new - old for new, old in zip(after, before))


def _compare_budget(before: StateV1, expected: StateV1, actual: StateV1) -> list:
    if (expected.budget is None) != (actual.budget is None):
        return [Difference("budget", "present in one outcome only")]
    if expected.budget is None:
        return []
    found = []
    for name in _BUDGET_FIELDS:
        if expected.budget[name] != actual.budget[name]:
            found.append(Difference(f"budget.{name}", f"{expected.budget[name]!r} != {actual.budget[name]!r}"))
    spent_expected = _delta(expected.budget["articles"], before.budget["articles"])
    spent_actual = _delta(actual.budget["articles"], before.budget["articles"])
    if spent_expected != spent_actual:
        names = [name for name, a, b in zip(_ARTICLES, spent_expected, spent_actual) if a != b]
        found.append(Difference("budget.delta", f"articles {names}: {spent_expected} != {spent_actual}"))
    return found


def _compare_counters(before: StateV1, expected: StateV1, actual: StateV1) -> list:
    found = []
    for key in sorted(set(expected.sign_counts) | set(actual.sign_counts)):
        spent_expected = expected.sign_counts.get(key, 0) - before.sign_counts.get(key, 0)
        spent_actual = actual.sign_counts.get(key, 0) - before.sign_counts.get(key, 0)
        if spent_expected != spent_actual:
            found.append(Difference(f"sign_counts.{key}", f"delta {spent_expected} != {spent_actual}"))
    unbudgeted_expected = _delta(expected.unbudgeted, before.unbudgeted)
    unbudgeted_actual = _delta(actual.unbudgeted, before.unbudgeted)
    if unbudgeted_expected != unbudgeted_actual:
        found.append(Difference("unbudgeted.delta", f"{unbudgeted_expected} != {unbudgeted_actual}"))
    return found


def _compare_table(name: str, expected, actual) -> list:
    if expected is None or actual is None:
        return [] if expected is actual else [Difference(name, "present in one outcome only")]
    if canonical(expected) == canonical(actual):
        return []
    texts_expected = [canonical(item) for item in expected]
    texts_actual = [canonical(item) for item in actual]
    if sorted(texts_expected) == sorted(texts_actual):
        return [Difference(name, "same entries, different insertion order")]
    return [Difference(name, _locate(list(expected), list(actual)))]


def _compare_tables(expected: StateV1, actual: StateV1) -> list:
    found = []
    for name in (*MEMORY_TABLES, "store"):
        found.extend(_compare_table(f"memory.{name}" if name != "store" else "store", getattr(expected, name), getattr(actual, name)))
    return found


def compare_outcomes(op: str, before: StateV1, expected: Outcome, actual: Outcome) -> list:
    """Расхождения двух исходов ОДНОГО вызова (пусто — исходы равны точно). `before` — общее состояние до вызова."""

    found = []
    if expected.exception != actual.exception:
        found.append(Difference("exception", f"{expected.exception!r} != {actual.exception!r}"))
    found.extend(_compare_results(op, expected.result, actual.result))
    found.extend(_compare_budget(before, expected.after, actual.after))
    found.extend(_compare_counters(before, expected.after, actual.after))
    found.extend(_compare_tables(expected.after, actual.after))
    for key in sorted(set(expected.observed) | set(actual.observed)):
        if expected.observed.get(key) != actual.observed.get(key):
            found.append(Difference(f"observed.{key}", "differs"))
    return found


def answer_digest(op: str, result) -> str:
    """sha256 канонической записи ответа (по полям): дешёвая проверка равенства результата."""

    digest = hashlib.sha256()
    view = answer_view(op, result)
    for item in dataclasses.fields(view):
        digest.update(item.name.encode())
        digest.update(canonical(getattr(view, item.name)).encode())
    return digest.hexdigest()


def outcome_digest(op: str, before: StateV1, outcome: Outcome) -> str:
    """sha256 всего, что сравнивает `compare_outcomes`: цифровая подпись исхода (равные подписи — равные исходы)."""

    digest = hashlib.sha256()
    digest.update(repr(outcome.exception).encode())
    digest.update(b"" if outcome.result is None else answer_digest(op, outcome.result).encode())
    after = outcome.after
    if after.budget is not None:
        digest.update(repr((tuple(after.budget[name] for name in _BUDGET_FIELDS), _delta(after.budget["articles"], before.budget["articles"]))).encode())
    digest.update(repr(sorted((key, after.sign_counts[key] - before.sign_counts.get(key, 0)) for key in after.sign_counts)).encode())
    digest.update(repr(_delta(after.unbudgeted, before.unbudgeted)).encode())
    for name in (*MEMORY_TABLES, "store"):
        table = getattr(after, name)
        digest.update(b"-" if table is None else canonical(table).encode())
    digest.update(repr(sorted(outcome.observed.items())).encode())
    return digest.hexdigest()


# --------------------------------------------------------------------------
# Формат записи
# --------------------------------------------------------------------------


@dataclass
class Record:
    """Прочитанная запись: заголовок (`meta`) и содержимое (`payload`)."""

    meta: dict
    payload: dict

    @property
    def op(self) -> str:
        return self.meta["op"]

    def before(self) -> StateV1:
        return StateV1(**self.payload["before"])

    def expected(self) -> Outcome:
        """Записанный исход: результат (пикл) или исключение, состояние после, наблюдения, секунды вызова."""

        item = self.payload["expected"]
        result = None if item["result"] is None else pickle.loads(item["result"])
        return Outcome(result, item["exception"], StateV1(**item["after"]), item["observed"], self.payload["seconds"])

    @property
    def call_blob(self) -> bytes:
        return self.payload["call"]


def make_payload(op: str, before: StateV1, blob: bytes, outcome: Outcome) -> dict:
    """Содержимое записи: состояние до, пикл входов, исход (результат — пиклом, без `work_budget` покрытия)."""

    result = outcome.result
    stored = None if result is None else pickle.dumps(answer_view(op, result) if op == OP_COVERAGE else result, protocol=5)
    return {
        "before": before.as_payload(),
        "call": blob,
        "seconds": outcome.seconds,
        "expected": {
            "result": stored,
            "exception": outcome.exception,
            "after": outcome.after.as_payload(),
            "observed": outcome.observed,
            "answer_digest": None if result is None else answer_digest(op, result),
        },
    }


def write_record(path: Path, meta: dict, payload: dict, preset: int = DEFAULT_PRESET) -> int:
    """Пишет запись, возвращает размер файла в байтах."""

    body = lzma.compress(pickle.dumps(payload, protocol=5), format=lzma.FORMAT_XZ, preset=preset)
    head = json.dumps(meta, sort_keys=True, ensure_ascii=False).encode("utf-8")
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as handle:
        handle.write(RECORD_MAGIC + struct.pack(">I", len(head)) + head)
        handle.write(body)
    return len(RECORD_MAGIC) + 4 + len(head) + len(body)


def _read_head(handle) -> dict:
    if handle.read(len(RECORD_MAGIC)) != RECORD_MAGIC:
        raise CorpusError("not a corpus record (magic differs)")
    (size,) = struct.unpack(">I", handle.read(4))
    return json.loads(handle.read(size).decode("utf-8"))


def read_meta(path: Path) -> dict:
    with open(path, "rb") as handle:
        return _read_head(handle)


def read_record(path: Path) -> Record:
    with open(path, "rb") as handle:
        meta = _read_head(handle)
        payload = pickle.loads(lzma.decompress(handle.read()))
    return Record(meta, payload)


# --------------------------------------------------------------------------
# Запись корпуса (рекордер на месте операции)
# --------------------------------------------------------------------------


def git_head(root: Path = ROOT) -> str:
    return subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=str(root), capture_output=True, text=True, check=True
    ).stdout.strip()


def corpus_directory(head: str, base: str | None = None) -> Path:
    """`<база>/<8 первых знаков HEAD>`; база — аргумент, либо `CFTUV_NATIVE_CORPUS`, либо `E:/cftuv_native_corpus`."""

    return Path(base or os.environ.get(CORPUS_ENVIRONMENT) or DEFAULT_CORPUS_BASE) / head[:8]


def matching_corpus(base: str | None = None) -> Path | None:
    """Каталог корпуса, записанного под ЭТО ядро (`kernel_identity` индекса равен отпечатку кода ядра процесса); новейший по времени индекса.

    Корпус старого ядра не подставляется: его записи описывают другой эталон (счётчики, группы резки), и сверка с ним ничего не доказывает.
    Нет подходящего — `None`, вызывающий пропускает сверку с названной причиной (`describe_missing_corpus`).
    """

    root = Path(base or os.environ.get(CORPUS_ENVIRONMENT) or DEFAULT_CORPUS_BASE)
    identity = clip_memo.kernel_code_identity()
    found = []
    for path in root.glob("*/index.json"):
        try:
            if load_index(path.parent).get("kernel_identity") == identity:
                found.append(path)
        except (OSError, ValueError):
            continue
    return max(found, key=lambda item: item.stat().st_mtime).parent if found else None


def describe_missing_corpus(base: str | None = None) -> str:
    """Причина пропуска, когда `matching_corpus` ничего не нашёл: какого ядра нет и какие корпуса лежат."""

    root = Path(base or os.environ.get(CORPUS_ENVIRONMENT) or DEFAULT_CORPUS_BASE)
    present = []
    for path in sorted(root.glob("*/index.json")):
        try:
            present.append(f"{path.parent.name}={load_index(path.parent).get('kernel_identity')}")
        except (OSError, ValueError):
            present.append(f"{path.parent.name}=<индекс не читается>")
    return f"нет корпуса под ядро {clip_memo.kernel_code_identity()} в {root} (лежат: {', '.join(present) or 'ничего'}): `tools/native_corpus_export.py`"


def run_description(extra: dict | None = None) -> dict:
    """Что идентифицирует запись корпуса: python, отпечаток кода ядра, HEAD репозитория."""

    return {
        "python": sys.version.split()[0],
        "kernel_identity": clip_memo.kernel_code_identity(),
        "git_head": git_head(),
        **(extra or {}),
    }


def result_shape(op: str, result) -> dict:
    """Числа топологии результата для индекса: у покрытия — грани с площадью и вершины контуров, у резки — грани и точки."""

    if result is None:
        return {}
    if op == OP_COVERAGE:
        return {
            "faces": sum(1 for item in result.faces if len(item.points) >= 3),
            "vertices": sum(len(item.points) for item in result.faces),
        }
    if op == OP_SKELETON:
        return {"nodes": len(result.nodes), "levels": result.levels, "obligations": len(result.proof_obligations)}
    return {"faces": len(result.polygons), "vertices": len(result.points)}


def input_shape(call: Call) -> dict:
    """Числа входа для индекса (у скелета — размеры полигона и условия вызова); у прочих операций пусто."""

    if call.op != OP_SKELETON:
        return {}
    polygon = call.args[0]
    return {
        "polygon_vertices": polygon.vertex_count,
        "polygon_loops": len(polygon.loops),
        "fan_supports": polygon.fan_edge_count,
        "reflex": polygon.reflex_count,
        "split_search": call.kwargs["split_search"].value,
        "dense_hydration": call.kwargs["dense_hydration"],
        "level_budget": call.kwargs["level_budget"],
    }


def outcome_label(op: str, result, error) -> str:
    if error is not None:
        return f"raised:{type(error).__qualname__}"
    if op == OP_SKELETON:
        return str(result.outcome.value)
    return str(result.outcome.value) if op == OP_COVERAGE else "CLIPPED"


class Recorder:
    """Ставится вместо операции (`wrap`): пишет вход, состояние до и после, исход и секунды каждого вызова.

    Время самого рекордера (снимки, пикл, сжатие, запись) копится в `overhead`: вызывающий вычитает его из времени домена.
    """

    def __init__(
        self, root: Path, description: dict, *, preset: int = DEFAULT_PRESET, max_bytes: int = DEFAULT_MAX_BYTES, operations: tuple = OPERATIONS
    ) -> None:
        self.operations = tuple(operations)
        self.root = Path(root)
        self.description = description
        self.preset = preset
        self.max_bytes = max_bytes
        self.rows: list = []
        self.domains: list = []
        self.context: dict = {"mesh": "", "mesh_digest": "", "alpha": None, "patch_id": None, "domain_id": None}
        self.sequence = 0
        self.bytes = 0
        self.overhead = 0.0
        self._op_count = {op: 0 for op in self.operations}
        self._domain: dict | None = None

    def wrap(self, op: str, original):
        def recorded(*args, **kwargs):
            return self._record(op, original, args, kwargs)

        return recorded

    def resume(self) -> dict:
        """Продолжает корпус каталога `root`: строки, нумерация и размер берутся из его индекса (записи дописываются, а не заменяют прежние); возвращает индекс."""

        index = load_index(self.root)
        self.rows = [row for row in index["records"] if row.get("derived") is None]
        self.domains = list(index.get("domains", []))
        self.sequence = max((row["seq"] for row in self.rows), default=0)
        self.bytes = sum(row["bytes"] for row in self.rows)
        for op in self.operations:
            self._op_count[op] = sum(1 for row in self.rows if row["op"] == op)
        return index

    def begin_domain(self, patch_id, domain_id, alpha) -> None:
        self.context.update(patch_id=patch_id, domain_id=domain_id, alpha=alpha)
        self._domain = {
            "calls": {op: 0 for op in self.operations},
            "seconds": {op: 0.0 for op in self.operations},
            "op_overhead": {op: 0.0 for op in self.operations},
            "stages": {},
            "overhead": self.overhead,
        }

    def note_stages(self, label: str, timings) -> None:
        """Секунды стадий домена (`(имя, секунды)` из результата ядра) под именем `метка.стадия`; вне домена — ничего."""

        if self._domain is not None:
            for stage, seconds in timings:
                key = f"{label}.{stage}"
                self._domain["stages"][key] = self._domain["stages"].get(key, 0.0) + seconds

    def end_domain(self, wall: float, outcome: str, reported: float) -> dict:
        """Строка домена: чистое время (стенка минус время рекордера), доли операций, число вызовов."""

        state, self._domain = self._domain, None
        net = wall - (self.overhead - state["overhead"])
        row = {
            "mesh": self.context["mesh"],
            "alpha": self.context["alpha"],
            "patch_id": self.context["patch_id"],
            "domain_id": self.context["domain_id"],
            "outcome": outcome,
            "seconds_net": net,
            "seconds_wall": wall,
            "seconds_reported": reported,
            "calls": state["calls"],
            "op_seconds": state["seconds"],
            "op_overhead": state["op_overhead"],
            "stages": state["stages"],
        }
        self.domains.append(row)
        self.context.update(patch_id=None, domain_id=None, alpha=None)
        return row

    def _record(self, op: str, original, args: tuple, kwargs: dict):
        started = time.perf_counter()
        call = unpack_call(op, args, kwargs)
        before = capture_state(call.budget, call.store)
        blob = encode_call(call)
        spent = time.perf_counter() - started
        began = time.perf_counter()
        try:
            result, error = original(*args, **kwargs), None
        except Exception as exc:  # noqa: BLE001 - исключение записывается и летит дальше тем же объектом
            result, error = None, exc
        seconds = time.perf_counter() - began
        started = time.perf_counter()
        self._write(call, before, blob, result, error, seconds)
        spent += time.perf_counter() - started
        self.overhead += spent
        if self._domain is not None:
            self._domain["op_overhead"][op] += spent
        if error is not None:
            raise error
        return result

    def _write(self, call: Call, before: StateV1, blob: bytes, result, error, seconds: float) -> None:
        op = call.op
        info = None if error is None else (type(error).__qualname__, str(error))
        outcome = Outcome(result, info, capture_state(call.budget, call.store), observe(call), seconds)
        payload = make_payload(op, before, blob, outcome)
        self.sequence += 1
        self._op_count[op] += 1
        if self._domain is not None:
            self._domain["calls"][op] += 1
            self._domain["seconds"][op] += seconds
        meta = {
            "id": f"{self.sequence:06d}-{op}",
            "schema": RECORD_SCHEMA,
            "op": op,
            "seq": self.sequence,
            "op_seq": self._op_count[op],
            **{key: self.context[key] for key in ("mesh", "mesh_digest", "alpha", "patch_id", "domain_id")},
            "domain_call": None if self._domain is None else self._domain["calls"][op],
            "seconds": seconds,
            "outcome": outcome_label(op, result, error),
            "exception": info,
            "budget": call.budget is not None,
            "python": self.description["python"],
            "kernel_identity": self.description["kernel_identity"],
            "git_head": self.description["git_head"],
            "canonical_audit": before.canonical_audit,
            **result_shape(op, result),
            **input_shape(call),
        }
        if op == OP_COVERAGE:
            meta["lattice_alpha"] = str(call.args[1])
        if op == OP_SKELETON and meta["domain_id"] is None and call.budget is not None and call.budget.domain_id:
            meta["domain_id"] = call.budget.domain_id
        mesh_dir = "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in str(self.context["mesh"] or "_"))
        patch = self.context["patch_id"]
        relative = f"records/{mesh_dir}/{self.sequence:06d}-{op}-p{patch if patch is not None else 'x'}.rec"
        size = write_record(self.root / relative, meta, payload, self.preset)
        self.bytes += size
        if self.bytes > self.max_bytes:
            raise CorpusError(f"corpus exceeds {self.max_bytes} bytes (at record {meta['id']})")
        self.rows.append({**meta, "path": relative, "bytes": size})

    def write_index(self, extra: dict | None = None) -> Path:
        document = {
            "schema": RECORD_SCHEMA,
            **self.description,
            **(extra or {}),
            "records_count": len(self.rows),
            "total_bytes": self.bytes,
            "records": self.rows,
            "domains": self.domains,
        }
        path = self.root / "index.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(document, ensure_ascii=False, indent=0, sort_keys=True) + "\n", encoding="utf-8")
        return path


def load_index(root: Path) -> dict:
    return json.loads((Path(root) / "index.json").read_text(encoding="utf-8"))


def bounded_before(before: StateV1, cap: int) -> StateV1:
    """Производная с потолком всегда ограничена, даже если исходный прогон эталонный."""
    return dataclasses.replace(before, budget={**before.budget, "mode": exact.ExactWorkBudgetModeV1.BOUNDED.value, "cap": cap})


def remove_indexed_derived(root: Path, rows: list) -> None:
    """Удаляет только перечисленные производные файлы после проверки всех путей."""
    derived_root = (root / "records" / "_derived").resolve()
    previous = [(root / row["path"]).resolve() for row in rows if row.get("derived") is not None]
    if any(not path.is_relative_to(derived_root) for path in previous):
        raise ValueError("derived record path escapes records/_derived")
    for path in previous:
        path.unlink(missing_ok=True)
