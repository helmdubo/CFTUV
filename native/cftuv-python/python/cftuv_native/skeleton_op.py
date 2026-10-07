"""The skeleton as a WHOLE operation: `skeleton.build_skeleton(polygon, *, split_search, work_budget, dense_hydration)`, native, with the oracle's side effects.

The unit the product replaces is the one call `wavefront/conveyor.py` makes (`build_skeleton(report.polygon, work_budget=..., dense_hydration=...)`): the builder (the
prime universe of the speeds, the loops with their fans, the motorcycle graph, the first candidates of every vertex), the event loop over exact-time packets (a frozen snapshot,
the symbolic closure replayed twice, the commit of the final overlay), and the result. Everything the oracle does besides returning a `SkeletonV1` is part of the answer and is
reproduced here by `CostMirror.build_skeleton` (this module holds its body, so that `cost.py` stays what it is): the six articles of the budget and the string `superlevel` the loop
writes into it at every level, the four tables of the canonicalization memory with their order, `SIGN_COUNTS`, `UNBUDGETED_WORK` for a call without a budget, and the real exceptions
with the oracle's text (an exhaustion gets its detail from `ExactWorkBudgetV1.exhaustion_detail` of the real budget, after every partial effect is in place).

What the oracle reads LIVE, and what the shim does about it:

* `skeleton.level_budget(polygon)` (a test replaces it to force `LEVEL_BUDGET_EXHAUSTED`): called here, the number goes into the call;
* `motorcycle.march_budget(grid)` (a test replaces it to force the exhaustion of a march): the port cannot see a replaced function, and the grid is made inside the call, so a replaced
  function is a named refusal (`NativePortUnsupported`); the declared one is what the port computes;
* `__debug__`: the oracle's two `assert compare_times(effect.evaluation_level, self.now, ...) == 0` COST a sign each, and `python -O` strips them. The port pays them, so an interpreter
  started with `-O` is a named refusal;
* `split_search`: both searches are carried (`MOTORCYCLE`, the product, and `EXHAUSTIVE`, the reference of the tests and benchmarks: no graph, no index); anything else is a named refusal.

A refusal of the PORT (`NativePortStale`, `NativePortUnsupported` whenever it is raised, an input the port does not carry, an internal state of the port) leaves every Python-visible state
exactly as it was before the call (articles, `superlevel`, `SIGN_COUNTS`, `UNBUDGETED_WORK`, the four memory tables with their order): the caller runs `skeleton.build_skeleton` on the same
budget and tables and gets what a pure oracle run would. The oracle's own internal failures (a `TypeError` of a symbolic reference without end points, ...) are such refusals too: the
oracle raises them itself when the caller falls back. An exception of the ORACLE (`ExactCanonicalizationWorkBudgetExhausted`, `ZeroDivisorTimeError`, `ValueError`, ...) leaves the partial effects
the oracle's exception leaves.

The call holds `cost.NATIVE_LOCK` (it is a public method of `CostMirror`) and the extension keeps the GIL while it computes (as `clip_geometry` does): the mirror and the real tables are process
state, the log of the call is replayed on the tables as they were when it started, and no Python code of the process may write them in between (a fallback of the oracle in another thread would).
A native coverage of the alpha preview thread waits for the lock, which a heavy domain holds for a fraction of a second.
"""

from __future__ import annotations

import dataclasses
from fractions import Fraction
from time import perf_counter_ns

from . import codec, cost, pin

STATUS_VALUE = 10
STATUS_UNSUPPORTED = 12
STATUS_ZERO_DIVISOR_TIME = 14
STATUS_PARALLEL_LINES = 15
STATUS_DEGENERATE_EDGE = 16
STATUS_NEGATIVE_SPEED = 17
STATUS_CELL_GRID_REJECTED = 18

#: The status codes of THIS operation that are outcomes of the ORACLE; every other code (5, 6, 7, 12, an unknown one) is a refusal of the port, applied nowhere. The extension keeps the
#: same table (`skeleton.rs`, `_core.skeleton_oracle_statuses()`).
SKELETON_STATUSES = frozenset(
    {cost.STATUS_OK, cost.STATUS_EXHAUSTED, cost.STATUS_NEGATIVE_RADICAND, cost.STATUS_ZERO_DIVISOR, cost.STATUS_RECONSTRUCTION, STATUS_VALUE, STATUS_ZERO_DIVISOR_TIME, STATUS_PARALLEL_LINES,
     STATUS_DEGENERATE_EDGE, STATUS_NEGATIVE_SPEED, STATUS_CELL_GRID_REJECTED}
)

#: The fixed texts of the oracle's exceptions (`event_time.py`).
ZERO_DIVISOR_TIME_TEXT = "знаменатель времени доказанно нулевой"
PARALLEL_LINES_TEXT = "прямые параллельны, точки пересечения нет"

SKELETON_FIELDS = ("outcome", "nodes", "levels", "counters", "proof_status", "proof_obligations")
NODE_FIELDS = ("kind", "time", "point", "participants", "converging_vertices", "kinds", "incidences")
TIME_FIELDS = ("dividend", "divisor")
POINT_FIELDS = ("x", "y")
OBLIGATION_FIELDS = ("cause", "disposition", "vertex_ids", "participant_edge_keys", "target_edge_keys", "level", "event_kind")

_KERNEL: list = []


def _kernel():
    """The kernel modules and classes the drop-in is built from (on `sys.path`), resolved on first use."""

    if not _KERNEL:
        from cftuv_envelope import exact_sqrt_sum
        from cftuv_envelope.wavefront import candidate_refusal, cell_grid, event_time, events, motorcycle, proof, skeleton, superlevel

        _KERNEL.append((exact_sqrt_sum, skeleton, superlevel, event_time, events, proof, candidate_refusal, motorcycle, cell_grid))
    return _KERNEL[0]


def _members(enum) -> dict:
    return {member.value: member for member in enum}


def bind(mirror) -> None:
    """Hands the result classes to the extension, after checking they have the slots the Rust side fills (`NativePortStale` else)."""

    exact, skeleton, superlevel, event_time, events, proof, refusal, _motorcycle, _cell_grid = _kernel()
    names = lambda cls: tuple(item.name for item in dataclasses.fields(cls))  # noqa: E731
    pin.check_shapes(
        (
            ("wavefront/superlevel.py", "SkeletonV1 fields", names(superlevel.SkeletonV1) == SKELETON_FIELDS),
            ("wavefront/superlevel.py", "SkeletonNodeV1 fields", names(superlevel.SkeletonNodeV1) == NODE_FIELDS),
            ("wavefront/event_time.py", "EventTimeV1 fields", names(event_time.EventTimeV1) == TIME_FIELDS),
            ("wavefront/event_time.py", "EventPointV1 fields", names(event_time.EventPointV1) == POINT_FIELDS),
            ("wavefront/proof.py", "ProofObligationV1 fields", names(proof.ProofObligationV1) == OBLIGATION_FIELDS),
        )
    )
    try:
        mirror._session.bind_skeleton(
            exact.SqrtSumV1, Fraction,
            superlevel.SkeletonV1, superlevel.SkeletonNodeV1, event_time.EventTimeV1, event_time.EventPointV1, proof.ProofObligationV1,
            _members(superlevel.SkeletonOutcome), _members(events.EventKind), _members(proof.ProofStatus), _members(proof.ProofObligationBranch),
            _members(proof.ProofObligationDisposition), _members(refusal.CandidateRefusal),
        )
    except TypeError as error:
        raise pin.NativePortStale(f"the oracle classes the native skeleton is built from changed: {error}") from None
    mirror._skeleton_bound = True


def _raise(status, detail, counts, articles, budget) -> None:
    """The oracle's exception for a refused skeleton (after every effect was applied); only the outcomes in `SKELETON_STATUSES` come here."""

    _exact, _skeleton, _superlevel, event_time, _events, _proof, _refusal, _motorcycle, cell_grid = _kernel()
    if status == STATUS_VALUE:
        raise ValueError(detail[0])
    if status == STATUS_ZERO_DIVISOR_TIME:
        raise event_time.ZeroDivisorTimeError(ZERO_DIVISOR_TIME_TEXT)
    if status == STATUS_PARALLEL_LINES:
        raise event_time.ParallelSupportLinesError(PARALLEL_LINES_TEXT)
    if status == STATUS_DEGENERATE_EDGE:
        raise event_time.DegenerateEdgeError(detail[0])
    if status == STATUS_NEGATIVE_SPEED:
        raise event_time.NegativeSpeedError(detail[0])
    if status == STATUS_CELL_GRID_REJECTED:
        raise cell_grid.CellGridRejected(detail[0])
    cost.OpResult(status, None, detail, counts, articles, (), None).raise_for(budget)


def build_skeleton(mirror, polygon, *, split_search=None, work_budget=None, dense_hydration=False):
    """`wavefront.skeleton.build_skeleton(polygon, split_search=..., work_budget=..., dense_hydration=...)`, whole, with its exact side effects (see the module note).

    `split_search=None` is the oracle's default (`SplitSearch.MOTORCYCLE`; the kernel's enum is not importable when this module is, so the default is spelt by its absence).
    """

    pin.require("skeleton")
    if not __debug__:
        raise pin.NativePortUnsupported(
            "the native `skeleton` port pays the sign of the oracle's two `assert compare_times(...)` effects; an interpreter started with -O strips them from the oracle, so the port refuses"
        )
    started = perf_counter_ns()
    exact, skeleton, _superlevel, _event_time, _events, _proof, _refusal, motorcycle, _cell_grid = _kernel()
    if not mirror._skeleton_bound:
        bind(mirror)
    if split_search is not None and split_search is not skeleton.SplitSearch.MOTORCYCLE and split_search is not skeleton.SplitSearch.EXHAUSTIVE:
        raise pin.NativePortUnsupported(f"the native `skeleton` port carries the motorcycle and the exhaustive split search, not {split_search!r}")
    if work_budget is not None and type(work_budget) is not exact.ExactWorkBudgetV1:
        raise pin.NativePortUnsupported(f"the native `skeleton` port takes an ExactWorkBudgetV1 or None, not {type(work_budget).__name__}")
    march = motorcycle.march_budget
    if getattr(march, "__module__", None) != motorcycle.__name__ or getattr(march, "__name__", None) != "march_budget":
        raise pin.NativePortUnsupported("`motorcycle.march_budget` was replaced: the native port computes the declared bound of the march and cannot see a patched function")
    try:
        level_limit = skeleton.level_budget(polygon)
    except Exception as error:  # noqa: BLE001 - the oracle's own call raises it again, on the same state
        raise pin.NativePortUnsupported(f"`skeleton.level_budget(polygon)` failed ({type(error).__name__}: {error})") from None
    if type(level_limit) is not int:
        raise pin.NativePortUnsupported(f"`skeleton.level_budget(polygon)` answered {level_limit!r}, not an int")
    sync, slow = mirror._sync_in(exact)
    try:
        value = mirror._budget_value(work_budget)
    except ValueError as error:
        raise pin.NativePortUnsupported(str(error)) from None
    state = None if value is None else (value[0], tuple(value[1:]))
    real = (exact._KNOWN_PRIMES, exact._KNOWN_PRIME_SET, exact._FACTORIZATION_MEMO, exact._SQUAREFREE_MEMO, exact._PRIME_SUPPORT_MEMO)
    called = perf_counter_ns()
    try:
        result, status, detail, counts, articles, bits, native, superlevel = mirror._session.build_skeleton(
            polygon, bool(dense_hydration), split_search is skeleton.SplitSearch.EXHAUSTIVE, level_limit, None, None if sync is cost.UNCHANGED_SYNC else codec.encode_value(sync), state, real
        )
    except BaseException:
        mirror.invalidate()
        raise
    returned = perf_counter_ns()
    if status not in SKELETON_STATUSES:
        # a refusal of the port: the extension applied nothing and forgot its mirror; neither do we apply the (zero) cost, and the mirror is forgotten here too
        mirror.invalidate()
        mirror.last_skeleton_timings = (called - started, returned - called, perf_counter_ns() - returned, perf_counter_ns() - started, *native)
        raise mirror._port_refusal(status, detail)
    mirror._settle_counts(exact, counts)
    mirror._settle_articles(exact, work_budget, articles)
    if superlevel is not None:
        work_budget.superlevel = superlevel
    mirror._remember(exact, bits, slow)
    finished = perf_counter_ns()
    mirror.last_skeleton_timings = (called - started, returned - called, finished - returned, finished - started, *native)
    if status:
        _raise(status, detail, counts, articles, work_budget)
    return result
