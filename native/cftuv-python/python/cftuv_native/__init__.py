"""Шим нативного ускорителя: ЕДИНСТВЕННЫЙ импортёр расширения `cftuv_native._core` (правило — `tests/test_architecture.py`).

Шим переводит объекты ядра в буферы целой операции и воспроизводит её побочные эффекты (бюджет точной работы, память
канонизации), как их воспроизводит попадание `clip_memo`. Тихого отката на Python здесь нет: нет расширения — `ImportError`.

Потоки: каждый публичный метод `CostMirror` идёт под ОДНОЙ процессной блокировкой `cost.NATIVE_LOCK` (синхронизация памяти, нативная операция, журнал, статьи и
счётчики — целиком): продукт зовёт `coverage_at` из потока предпросмотра alpha и из главного, а таблицы памяти и сессия — состояние процесса.

Три части:

* числа (`run_number_ops`): сценарий операций в ОДНОМ вызове, только для сверки с эталоном;
* целые операции: `coverage_at` (`wavefront.coverage._coverage_at`, с разбиениями, которые нативная сессия переводит один раз) и `clip_geometry`
  (`materialize.clip.clip_geometry`: подъём плоскости переводится один раз, результат строится из Rust, нормали смещения пишутся в плоскость);
  обе сверены с ОДНОЙ версией эталона и отказываются по имени, если дерево ядра ушло от неё (`pin`, `native_status`, `NativePortStale`);
* стоимость (`default_mirror`, `new_mirror`, `sign`, `divided_by`, ...): долгоживущая нативная сессия владеет зеркалом памяти
  канонизации, а `cost.CostMirror` держит зеркало равным настоящим таблицам Python до вызова и применяет журнал изменений
  к ним после (бюджет, `SIGN_COUNTS`, `UNBUDGETED_WORK`, исключения). Подробности — в `cost.py`.

Отказ целой операции бывает двух видов (`NATIVE_REFUSALS`, `cost.ORACLE_STATUSES`). Исход ЭТАЛОНА (`MaterializationRefusal`, `ExactCanonicalizationWorkBudgetExhausted`,
`OverflowError`, `ValueError`, `ZeroDivisionError`, `KeyError`, ...) оставляет частичные эффекты ровно так, как их оставляет исключение Python. Отказ ПОРТА
(`NativePortStale`, `NativeUnsupportedPython`, `NativePortUnsupported` — в том числе поздний, посреди вычисления —, `NativeDivisionDiverged`) оставляет ВСЁ видимое из
Python состояние, как оно было до вызова: статьи бюджета, `SIGN_COUNTS`, `UNBUDGETED_WORK`, четыре таблицы памяти с порядком, `plane._normal_by_position`, `store`,
`traces`. Вызывающий вправе запустить эталон на тех же бюджете, плоскости и таблицах и получить исход и состояние чистого прогона эталона.

Личность сборки: `native_build_id()` — содержательный отпечаток (sha256) нативного кода и шима, устойчивый к перелинковке (`buildid.py`).
"""

from __future__ import annotations

from . import _core, buildid, codec, cost, pin

__all__ = (
    "CostMirror",
    "NATIVE_REFUSALS",
    "NativeDivisionDiverged",
    "NativePortStale",
    "NativePortUnsupported",
    "NativeUnsupportedPython",
    "clip_geometry",
    "clip_seam_run",
    "clip_seam_table",
    "coverage_at",
    "default_mirror",
    "divide_with_prime_universe",
    "divided_by",
    "int_round_trip",
    "last_clip_timings",
    "last_coverage_timings",
    "native_build_id",
    "native_build_parts",
    "native_status",
    "native_version",
    "new_clip_seam_session",
    "new_skeleton_seam_session",
    "new_mirror",
    "number_op_table",
    "oracle_statuses",
    "prime_support",
    "prime_universe_remembered",
    "radical",
    "radical_sum",
    "run_number_ops",
    "sign",
    "skeleton_seam_run",
    "skeleton_seam_table",
    "squarefree_split",
    "tree_digest",
)

CostMirror = cost.CostMirror
NativePortStale = pin.NativePortStale
NativePortUnsupported = pin.NativePortUnsupported
NativeUnsupportedPython = pin.NativeUnsupportedPython
NativeDivisionDiverged = cost.NativeDivisionDiverged

#: The exceptions `coverage_at` and `clip_geometry` raise for a refusal of the PORT (not an outcome of the oracle). Each of them leaves every Python-visible state
#: exactly as it was before the call (budget articles, `SIGN_COUNTS`, `UNBUDGETED_WORK`, the four memory tables with their order, `plane._normal_by_position`,
#: the `store`, the `traces`), so the caller may run the Python oracle on the same budget, plane and tables. Not in the tuple: the exceptions of the oracle
#: (`MaterializationRefusal`, `ExactCanonicalizationWorkBudgetExhausted`, `OverflowError`, `ValueError`, `ZeroDivisionError`, `KeyError`, ...), which leave the partial
#: effects the oracle's own exception leaves.
NATIVE_REFUSALS = (NativePortStale, NativeUnsupportedPython, NativePortUnsupported, NativeDivisionDiverged)

_DEFAULT: list = []
_BUILD: list = []


def native_version() -> str:
    return _core.version()


def native_build_parts() -> dict:
    """`{"id", "rust", "shim", "version"}`: the content identity of this build and the two halves it is made of (`buildid.py`).

    `rust` is the sha256 of the Rust sources the extension was built from (embedded at build time), `shim` the sha256 of the shim's own `.py` files as they are
    now; both normalise CRLF to LF. Computed once per process.
    """

    with cost.NATIVE_LOCK:
        if not _BUILD:
            rust, shim = _core.source_digest(), buildid.shim_digest()
            _BUILD.append({"id": buildid.compose(rust, shim), "rust": rust, "shim": shim, "version": _core.version()})
        return dict(_BUILD[0])


def native_build_id() -> str:
    """A content identity of this build: 64 lowercase hex characters, the sha256 of the Rust sources it was built from and of the shim's `.py` files.

    Stable across relinks and rebuilds of the same content (the MSVC link stamps a timestamp and a PDB GUID into `_core.pyd`, so the binary's bytes are NOT the identity),
    different whenever the native code, its manifests or the shim change. See `buildid.py` and `tools/native_build_id.py`.
    """

    return native_build_parts()["id"]


def oracle_statuses() -> tuple:
    """Test-only: the status codes the extension treats as outcomes of the oracle (`refusal.rs`); `cost.ORACLE_STATUSES` is the same table."""

    return tuple(_core.oracle_statuses())


def tree_digest(native_root) -> str:
    """Test-only: the digest of the Rust sources of the workspace at `native_root`, by the algorithm `build.rs` embeds (`digest.rs`, `tools/native_build_id.py`)."""

    return _core.tree_digest(str(native_root))


def native_status() -> dict:
    """`{operation: "available" | "stale(files)" | "unsupported_python"}` for the whole operations (`coverage`, `clip`).

    `unsupported_python` only below the floor (`pin.MINIMUM_PYTHON`, 3.11); the ports are tested on 3.11 and 3.13 (`pin.TESTED_PYTHON`) and nothing in their
    answers depends on the interpreter version (the kernel names the CPython 3.11 sort and float fold explicitly, `_cpython311.py`).
    """

    return pin.native_status()


def int_round_trip(value: int) -> int:
    """Test-only: an `int` through the boundary conversions (`pyobj.rs`) and back."""

    return _core.int_round_trip(value)


def number_op_table() -> tuple[tuple[int, str], ...]:
    """`(opcode, name)` of every number operation the extension knows."""

    return tuple(_core.number_op_table())


def run_number_ops(ops, *, strict: bool = True, memo: bool = True) -> list:
    """Test-only differential entry: a whole script of number operations in ONE crossing.

    `ops` is a sequence of `(operation name, arguments)`; the result is one decoded value per operation, a
    `codec.NativeError` where the operation failed with a named error (`OverflowError`, `ZeroDivisionError`,
    `ValueError`). `strict` makes the native decoder check that every rational is in lowest terms; `memo=False`
    turns the radicand-product memory off. A buffer the extension refuses is a `ValueError`.
    """

    return codec.decode_response(_core.run_number_ops(codec.encode_request(ops, strict=strict, memo=memo)))


def new_mirror() -> cost.CostMirror:
    """A new native session with its own mirror (empty): for tests and for callers that isolate state."""

    return cost.CostMirror(_core.Session())


def default_mirror() -> cost.CostMirror:
    """The process-wide mirror: one native session per process, matching the process-wide Python tables."""

    with cost.NATIVE_LOCK:
        if not _DEFAULT:
            _DEFAULT.append(new_mirror())
        return _DEFAULT[0]


def coverage_at(partition, alpha, work_budget=None, store=None, traces=None):
    """`wavefront.coverage._coverage_at(partition, alpha, work_budget, store)`, native and whole (see `CostMirror.coverage_at`).

    A refusal of the port (`NATIVE_REFUSALS`) leaves every Python-visible state exactly as before the call; an exception of the oracle leaves its partial effects.
    """

    return default_mirror().coverage_at(partition, alpha, work_budget, store, traces)


def clip_geometry(plane, budget, *, points, cycles, polygons, law, seam, fans, flows, by_faces, inert=frozenset()):
    """`materialize.clip.clip_geometry(plane, budget, ...)`, native and whole (see `CostMirror.clip_geometry`).

    A refusal of the port (`NATIVE_REFUSALS`) leaves every Python-visible state exactly as before the call; an exception of the oracle leaves its partial effects.
    """

    return default_mirror().clip_geometry(plane, budget, points=points, cycles=cycles, polygons=polygons, law=law, seam=seam, fans=fans, flows=flows, by_faces=by_faces, inert=inert)


def last_clip_timings() -> tuple:
    """Nanoseconds of the last `clip_geometry` of the default mirror: `(sync in, native call, post, total, plane, arguments, compute, result)`."""

    return default_mirror().last_clip_timings


def last_coverage_timings() -> tuple:
    """Nanoseconds of the last `coverage_at` of the default mirror: `(sync in, native call, post, total, prepare, arguments, compute, result, memory log)`."""

    return default_mirror().last_timings


def sign(value, *, filter_bits: int = 64, budget=None) -> int:
    """`SqrtSumV1.sign(filter_bits=..., budget=...)`, native; budget, counters and memory updated like Python's."""

    return default_mirror().sign(value, filter_bits=filter_bits, budget=budget)


def divided_by(numerator, denominator, budget=None):
    """`numerator.divided_by(denominator, budget)`, native."""

    return default_mirror().divided_by(numerator, denominator, budget)


def divide_with_prime_universe(numerator, denominator, prime_universe, budget=None):
    """`_divide_with_prime_universe(numerator, denominator, prime_universe, budget)`, native."""

    return default_mirror().divide_with_prime_universe(numerator, denominator, prime_universe, budget)


def radical(coefficient, radicand, budget=None):
    """`SqrtSumV1.radical(coefficient, radicand, budget)`, native."""

    return default_mirror().radical(coefficient, radicand, budget)


def radical_sum(parts, budget=None):
    """`radical_sum(parts, budget)`, native."""

    return default_mirror().radical_sum(parts, budget)


def squarefree_split(n: int, budget=None) -> tuple:
    """`squarefree_split(n, budget)`, native."""

    return default_mirror().squarefree_split(n, budget)


def prime_support(radicand: int, budget=None) -> tuple:
    """`prime_support(radicand, budget)`, native."""

    return default_mirror().prime_support(radicand, budget)


def prime_universe_remembered(q_values, budget=None, store=None) -> tuple:
    """`prime_universe_remembered(q_values, budget, store)` with the default `build`, native."""

    return default_mirror().prime_universe_remembered(q_values, budget, store)


def new_clip_seam_session():
    """Test-only: a fresh native session for the clip differential seams (`cftuv_native.clip_seams`)."""

    return _core.Session()


def clip_seam_run(session, request: bytes) -> bytes:
    """Test-only: ONE clip seam (`cftuv-clip/src/seam.rs`) on `session`; the answer buffer is decoded by `clip_seams`."""

    return _core.clip_seam_run(session, request)


def clip_seam_table() -> tuple[tuple[int, str], ...]:
    """Test-only: `(opcode, name)` of every clip seam the extension knows."""

    return tuple(_core.clip_seam_table())


def new_skeleton_seam_session():
    """Test-only: a fresh native session for the skeleton differential seams (`cftuv_native.skeleton_seams`)."""

    return _core.Session()


def skeleton_seam_run(session, request: bytes) -> bytes:
    """Test-only: ONE skeleton seam (`cftuv-skeleton/src/seam.rs`) on `session`; the answer buffer is decoded by `skeleton_seams`."""

    return _core.skeleton_seam_run(session, request)


def skeleton_seam_table() -> tuple[tuple[int, str], ...]:
    """Test-only: `(opcode, name)` of every skeleton seam the extension knows."""

    return tuple(_core.skeleton_seam_table())
