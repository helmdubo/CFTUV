"""Шим нативного ускорителя: ЕДИНСТВЕННЫЙ импортёр расширения `cftuv_native._core` (правило — `tests/test_architecture.py`).

Шим переводит объекты ядра в буферы целой операции и воспроизводит её побочные эффекты (бюджет точной работы, память
канонизации), как их воспроизводит попадание `clip_memo`. Тихого отката на Python здесь нет: нет расширения — `ImportError`.

Две части:

* числа (`run_number_ops`): сценарий операций в ОДНОМ вызове, только для сверки с эталоном;
* стоимость (`default_mirror`, `new_mirror`, `sign`, `divided_by`, ...): долгоживущая нативная сессия владеет зеркалом памяти
  канонизации, а `cost.CostMirror` держит зеркало равным настоящим таблицам Python до вызова и применяет журнал изменений
  к ним после (бюджет, `SIGN_COUNTS`, `UNBUDGETED_WORK`, исключения). Подробности — в `cost.py`.
"""

from __future__ import annotations

from . import _core, codec, cost

__all__ = (
    "CostMirror",
    "clip_seam_run",
    "clip_seam_table",
    "default_mirror",
    "divide_with_prime_universe",
    "divided_by",
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

_DEFAULT: list = []


def native_version() -> str:
    return _core.version()


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

    if not _DEFAULT:
        _DEFAULT.append(new_mirror())
    return _DEFAULT[0]


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
