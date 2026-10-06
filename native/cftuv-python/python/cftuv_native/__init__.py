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
"""

from __future__ import annotations

from . import _core, codec, cost, pin

__all__ = (
    "CostMirror",
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
    "native_status",
    "native_version",
    "new_clip_seam_session",
    "new_mirror",
    "number_op_table",
    "prime_support",
    "prime_universe_remembered",
    "radical",
    "radical_sum",
    "run_number_ops",
    "sign",
    "squarefree_split",
)

CostMirror = cost.CostMirror
NativePortStale = pin.NativePortStale
NativePortUnsupported = pin.NativePortUnsupported
NativeUnsupportedPython = pin.NativeUnsupportedPython

_DEFAULT: list = []


def native_version() -> str:
    return _core.version()


def native_status() -> dict:
    """`{operation: "available" | "stale(files)" | "unsupported_python"}` for the whole operations (`coverage`, `clip`)."""

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


def coverage_at(partition, alpha, work_budget=None, store=None):
    """`wavefront.coverage._coverage_at(partition, alpha, work_budget, store)`, native and whole (see `CostMirror.coverage_at`)."""

    return default_mirror().coverage_at(partition, alpha, work_budget, store)


def clip_geometry(plane, budget, *, points, cycles, polygons, law, seam, fans, flows, by_faces, inert=frozenset()):
    """`materialize.clip.clip_geometry(plane, budget, ...)`, native and whole (see `CostMirror.clip_geometry`)."""

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
