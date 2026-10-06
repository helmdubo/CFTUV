"""The cost mirror: what makes a native exact operation cost exactly what the Python one costs.

Equality with the oracle covers the answer AND its side effects on process state: the six articles of the
`ExactWorkBudgetV1`, the `SIGN_COUNTS` counters, `UNBUDGETED_WORK` for a call without a budget, and the four tables
of the canonicalization memory (`_KNOWN_PRIMES` + `_KNOWN_PRIME_SET`, `_FACTORIZATION_MEMO` with its LRU order,
`_SQUAREFREE_MEMO`, `_PRIME_SUPPORT_MEMO`) with their insertion order. The native `Session` owns a mirror of the
tables. This module keeps the REAL Python objects and the mirror equal:

* BEFORE a call (`sync in`): every table is compared, order and values, with what the session last mirrored. The session keeps the very objects of the tables as they
  were after the last call (`native/cftuv-python/src/view.rs`): tables made of the same objects in the same order cost one pointer comparison per entry and no
  allocation, anything else goes through the full comparison over the lists of that view.
  Entries appended since are sent as a tail; anything else (a clear, an eviction, an LRU touch, an isolated block, a
  poisoned value) is a general diff: the kept prefix (a subsequence of the mirror in mirror order) stays, the rest is
  deleted and re-sent. The session never guesses: a sync it cannot apply is a named refusal.
* AFTER a call (`sync out`): the native op-log is replayed on the real containers IN PLACE (never rebinding a module
  global): `insort` + `set.add` for the registry, `d[key] = d.pop(key)` for a touch, `del d[next(iter(d))]` for an
  eviction, `clear()` for a reset. Values have exactly Python's types (tuples of int pairs, `(outside, inside)`,
  tuples of ints). The oldest key an eviction names is checked against the real one.
* The budget: `cap` and the six articles go in; the articles are written back into the real object in place (a
  call without a budget adds its deltas to `UNBUDGETED_WORK`); the exception of an exhaustion is the REAL
  `ExactCanonicalizationWorkBudgetExhausted` built from `budget.exhaustion_detail`, raised AFTER every partial
  effect (articles, counters, tables) is in place, as Python's exception would leave them.

This module imports no extension: it drives a session object (`run(bytes) -> bytes`, `clear()`, `lengths()`) given
by the shim. The mirror assumes nothing else mutates the tables during a call, and the tables are PROCESS state, so every
public method of `CostMirror` runs under ONE process-wide re-entrant lock (`NATIVE_LOCK`): the product calls the native
operations from the alpha-preview worker thread and from the main thread, and two such calls would read and write the same
tables and the same session (the extension releases the GIL while it computes). The lock is held for the whole call: sync in,
the native operation, the replay of its log on the real tables, the settling of the budget and the counters. It guards the
NATIVE calls against each other, not the tables against Python code that mutates them in another thread (the next sync in
would name such a change as a general diff, never guess it).

Adding a native operation that pays budget or touches the memory: either give it an opcode in `codec.OPS` and the Rust
table `script::OPS`, make its Rust answer the cost answer `[outcome, counts, articles, log, state]` (`session.rs`), and run
it through `CostMirror.execute` — the sync, the budget, the log replay and the exception come for free (`OpResult.value`
is the operation's own value) — or, for a WHOLE operation that builds its own Python result and keeps state of its own in the
session (`coverage_at`, the first one), pass the sync and the budget state as plain arguments and take the cost back as
plain tuples (`CostMirror.coverage_at`): the same `_sync_in`, `_apply_entries`, `_settle_*` and `OpResult.raise_for`, without
the script buffers.

A call that raises inside the extension (`ValueError`: a refused buffer; `RuntimeError`: a panic the extension caught)
leaves the real tables as they were before the call and resets the session; the mirror forgets what it held, so the next
call reloads the tables whole.
"""

from __future__ import annotations

import functools
import threading
import types
from bisect import insort
from fractions import Fraction
from time import perf_counter_ns

from . import codec, pin

#: The one lock of the process for every native call that reads or writes process state (see the module note). Re-entrant: a store
#: that is read through its own methods may call native again from inside a call.
NATIVE_LOCK = threading.RLock()

#: `Operation::ALL` of the native side, in the order of `ExactWorkOperationV1`.
OPERATION_NAMES = ("PRIME_UNIVERSE", "COPRIME_BASIS", "PRIMALITY", "POLLARD_RHO_BRENT", "SQUAREFREE_SPLIT", "PRIME_SUPPORT", "EXACT_POSITION")
COUNT_KEYS = ("total", "closed_rational_zero", "closed_rational_nonzero", "closed_by_enclosure", "closed_by_conjugation")
ARTICLES = ("modular_squarings", "gcd_operations", "miller_rabin_rounds", "pollard_attempts", "radical_materializations", "exact_position_hydrations")

OPTION_FULL_STATE = 1
#: The bits of the tables a call changed: `CLIP_CHANGED` below (the extension names them for a clip), `TABLE_BITS` for the names `_apply_entries` gives.
TABLE_BITS = {"registry": 1, "factorization": 2, "squarefree": 4, "support": 8}
ALL_TABLES = 15
PRIME_UNIVERSE_KEY = "prime-universe"
ZERO_DIVISOR_MESSAGE = "деление на точный ноль"

STATUS_OK = 0
STATUS_EXHAUSTED = 1
STATUS_NEGATIVE_RADICAND = 2
STATUS_ZERO_DIVISOR = 3
STATUS_RECONSTRUCTION = 4
STATUS_INVALID_INPUT = 5
STATUS_DIVERGED = 6
STATUS_INTERNAL = 7
STATUS_MISSING_LINE = 8

#: Outcome codes of `clip_geometry` beyond the exact layer's (1..7): the exceptions the clip stage raises on its own.
CLIP_STATUS_OVERFLOW = 8
CLIP_STATUS_ZERO_DIVISION = 9
CLIP_STATUS_VALUE = 10
CLIP_STATUS_REFUSAL = 11
CLIP_STATUS_UNSUPPORTED = 12
CLIP_STATUS_MISSING_KEY = 13
#: `OverflowError` texts by kind (`float(int)`; `int / int` and `float(Fraction)`).
OVERFLOW_TEXTS = ("int too large to convert to float", "integer division result too large for a float")
#: The outcomes of `MaterializationRefusal` the clip stage names, and the slots of the classes its result is built from.
CLIP_REFUSAL_OUTCOMES = ("TESSELLATION_DID_NOT_CLOSE", "CLIP_PIECE_LEFT_ITS_TRIANGLE", "BATCH_DID_NOT_VALIDATE", "SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE")
CLIPPED_FIELDS = ("polygons", "cycles", "vertex_lists", "extra_lists", "points", "snapped", "lifted", "counters", "note", "memo")
#: The bits of the changed-tables answer of `clip_geometry` (the extension replays the memory log on the real tables itself).
CLIP_CHANGED = ((1, "registry"), (2, "factorization"), (4, "squarefree"), (8, "support"))
LIFT_TRIANGLE_FIELDS = ("name", "chart", "corners", "twice_area", "box", "normals", "face")

_U64 = (1 << 64) - 1


class NativeMirrorError(RuntimeError):
    """The mirror and the real tables disagree (or the native side refused a sync): a bug, never a silent repair."""


class NativeDivisionDiverged(ArithmeticError):
    """The generic division fallback did not finish (Python's loop would run on): named refusal, not a hang."""


_EXACT: list = []


def _exact():
    """The kernel's `exact_sqrt_sum` (on `sys.path`), resolved on first use."""

    if not _EXACT:
        from cftuv_envelope import exact_sqrt_sum

        _EXACT.append(exact_sqrt_sum)
    return _EXACT[0]


_COVERAGE: list = []


def _coverage():
    """The kernel's `wavefront.coverage` and `wavefront.faces` (on `sys.path`), resolved on first use."""

    if not _COVERAGE:
        from cftuv_envelope.wavefront import coverage, faces

        _COVERAGE.append((coverage, faces))
    return _COVERAGE[0]


_CLIP: list = []


def _clip_kernel():
    """The kernel classes `clip_geometry` is built from (on `sys.path`), resolved on first use."""

    if not _CLIP:
        from cftuv_envelope import numeric
        from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
        from cftuv_envelope.materialize import admit, clip, frames, lift_surface

        _CLIP.append((clip, numeric, DecalTopologyLawV1, admit.MaterializationOutcome, frames.MaterializationRefusal, lift_surface.LiftTriangleV1))
    return _CLIP[0]


#: The sync of a call where nothing changed since the last one (the common case), encoded once.
UNCHANGED_SYNC = codec.Raw(codec.encode_value([[False, [], []] for _ in range(4)]))


class StoreKey(tuple):
    """The lookup key of a `prime-universe` store, `(name, fractions, hash, plain, seen)`, with its hash taken once.

    `hash(("prime-universe", (Fraction, ...)))` runs Python code per fraction (`Fraction.__hash__`), so a lookup of a partition of 54 faces costs 80 us on 3.11
    and the call itself 1.5 ms. The extension keeps one of these per prepared partition and presents it where it would present `plain` (the key a miss
    writes, so a store never holds this class): `dict` asks it for `hash` (the number `hash(plain)` gave) and, on a hash match with another key, for `==`,
    answered by identity with `plain` first and by the comparison of the two tuples otherwise: the same answer the lookup of `plain` itself gets. A key
    the store holds that is equal to `plain` but is another object (the oracle made it, a pickle brought it) is compared once: `seen` holds it afterwards,
    and a tuple of immutable numbers equal to `plain` stays equal to it.
    """

    __slots__ = ()

    def __hash__(self):
        return self[2]

    def __eq__(self, other):
        plain, seen = self[3], self[4]
        if other is plain or (seen and seen[0] is other):
            return True
        if plain == other:
            seen[:] = (other,)
            return True
        return False

    def __ne__(self, other):
        return not self.__eq__(other)


class OpResult:
    """One operation of a native script: its outcome and the cost it left behind (already applied to the host)."""

    __slots__ = ("status", "value", "detail", "counts", "articles", "log", "state")

    def __init__(self, status, value, detail, counts, articles, log, state) -> None:
        self.status = status
        self.value = value
        self.detail = detail
        self.counts = counts
        self.articles = articles
        self.log = log
        self.state = state

    @property
    def ok(self) -> bool:
        return self.status == STATUS_OK

    def exhaustion(self) -> tuple[str, int]:
        """`(operation name, radicand)` of an exhaustion."""

        return OPERATION_NAMES[self.detail[0]], self.detail[1]

    def raise_for(self, budget) -> None:
        """The Python exception of a refused outcome (no-op for an ok one). Call AFTER the effects were applied."""

        exact = _exact()
        status, detail = self.status, self.detail
        if status == STATUS_OK:
            return
        if status == STATUS_EXHAUSTED:
            operation, radicand = self.exhaustion()
            raise exact.ExactCanonicalizationWorkBudgetExhausted(budget.exhaustion_detail(exact.ExactWorkOperationV1(operation), radicand))
        if status == STATUS_NEGATIVE_RADICAND:
            raise exact.NegativeRadicandError(f"под корнем {Fraction(detail[0], detail[1])}")
        if status == STATUS_ZERO_DIVISOR:
            raise exact.ZeroSqrtSumDivisorError(ZERO_DIVISOR_MESSAGE)
        if status == STATUS_RECONSTRUCTION:
            raise ArithmeticError(f"факторизация {detail[0]} не восстановила исходное число")
        if status == STATUS_DIVERGED:
            raise NativeDivisionDiverged("the generic division fallback did not finish")
        raise NativeMirrorError(f"native exact operation failed with status {status}")


def _record_wire(found) -> list:
    """The wire form of a store record `(universe, delta, price, FactorizationMemoryDeltaV1)` (the memory's squarefree entries flattened)."""

    universe, delta, price, memory = found
    squarefree = [[key, split[0], split[1]] for key, split in memory.squarefree]
    return [list(universe), list(delta), price, [list(memory.factorizations), squarefree, list(memory.supports), list(memory.primes)]]


def _record_of_wire(universe: tuple, record) -> tuple:
    """The store record the oracle writes, from the extension's answer: tuples of ints all the way down and a `FactorizationMemoryDeltaV1`."""

    _universe, delta, price, memory = record
    factorizations, squarefree, supports, primes = memory
    exact = _exact()
    return (
        universe,
        tuple((number, tuple(tuple(pair) for pair in pairs)) for number, pairs in delta),
        None if price is None else tuple(price),
        exact.FactorizationMemoryDeltaV1(
            tuple((key, tuple(tuple(pair) for pair in pairs)) for key, pairs in factorizations),
            tuple((key, (outside, inside)) for key, outside, inside in squarefree),
            tuple((key, tuple(support)) for key, support in supports),
            tuple(primes),
        ),
    )


def exhaustion_detail(budget, result: OpResult) -> str:
    """The detail string of an exhaustion in the middle of a script: the budget as it stood at that operation."""

    exact = _exact()
    shadow = exact.ExactWorkBudgetV1(mode=budget.mode, cap=budget.cap, stage=budget.stage, domain_id=budget.domain_id, superlevel=budget.superlevel)
    for name, value in zip(ARTICLES, result.articles):
        setattr(shadow, name, value)
    operation, radicand = result.exhaustion()
    return shadow.exhaustion_detail(exact.ExactWorkOperationV1(operation), radicand)


# --------------------------------------------------------------------------
# sync in
# --------------------------------------------------------------------------


def _general_diff(mirror_keys: list, mirror_values: list, keys: list, values: list) -> tuple:
    """`(clear, deleted keys, cut)`: keep the longest prefix of `keys` that is a subsequence of the mirror in mirror
    order with equal values; `keys[cut:]` is the tail (new, moved or changed entries)."""

    if 2 * len(keys) < len(mirror_keys):
        # fewer than half of the mirror's entries can be kept, whatever the content: the loop below would end in the same answer
        return True, [], 0
    index = {key: position for position, key in enumerate(mirror_keys)}
    last = -1
    cut = len(keys)
    for position, key in enumerate(keys):
        found = index.get(key)
        if found is None or found <= last or mirror_values[found] != values[position]:
            cut = position
            break
        last = found
    kept = set(keys[:cut])
    deleted = [key for key in mirror_keys if key not in kept]
    if len(deleted) > len(kept):
        return True, [], 0
    return False, deleted, cut


def _entry_factorization(key, value):
    return (key, value)


def _entry_squarefree(key, value):
    return (key, value[0], value[1])


def _entry_support(key, value):
    return (key, value)


def _serialized(function):
    """`function` under `NATIVE_LOCK` (looked up per call, so a harness can swap the lock to prove the lock is what holds the calls apart)."""

    @functools.wraps(function)
    def locked(*arguments, **keywords):
        with NATIVE_LOCK:
            return function(*arguments, **keywords)

    return locked


def serialize_public_methods(cls):
    """Every public method of `cls` (a plain function whose name has no leading underscore) runs under `NATIVE_LOCK`.

    The private helpers are reached only through a public method, so they run under the lock too; static and class methods are pure.
    A method added later is covered without a decorator of its own.
    """

    for name, member in list(vars(cls).items()):
        if isinstance(member, types.FunctionType) and not name.startswith("_"):
            setattr(cls, name, _serialized(member))
    return cls


@serialize_public_methods
class CostMirror:
    """Keeps one native session equal to the real process state and runs cost-bearing scripts on it."""

    def __init__(self, session) -> None:
        self._session = session
        self._coverage_bound = False
        self._face_exact = None
        self._clip_bound = False
        self._clip_classes: tuple = ()
        #: Nanoseconds of the last `clip_geometry`: `(sync in, native call, post, total, plane, arguments, compute, result, memory log)`; the last
        #: five are measured inside the extension (see `native/cftuv-python/src/clip.rs`).
        self.last_clip_timings: tuple = ()
        #: Nanoseconds of the last `coverage_at`: `(sync in, native call, post, total, prepare, arguments, compute, result, memory log)`;
        #: the last five are measured inside the extension (see `native/cftuv-python/src/coverage.rs`).
        self.last_timings: tuple = ()
        #: Calls whose tables were not provably unchanged by identity (the session's view of the host's tables, `view.rs`) and went through the full comparison.
        self.slow_syncs = 0

    def invalidate(self) -> None:
        """Forget everything the session mirrors and the view of the host's tables it kept: the next call reloads the tables whole."""

        self._session.clear()

    def lengths(self) -> tuple:
        """Native mirror lengths: registry, factorizations, squarefree splits, supports (a debugging view)."""

        return self._session.lengths()

    # ---- sync in ---------------------------------------------------------------------------------------------------

    @staticmethod
    def _registry_sync(primes: list, mirror: list):
        if primes == mirror:
            return None
        present, wanted = set(mirror), set(primes)
        removed = sorted(present - wanted)
        if len(removed) > 32 and 2 * len(removed) > len(mirror):
            return [True, [], list(primes)]
        return [False, removed, sorted(wanted - present)]

    @staticmethod
    def _table_sync(table: dict, entry, mirror: tuple):
        mirror_keys, mirror_values = mirror
        keys, values = list(table), list(table.values())
        if keys == mirror_keys and values == mirror_values:
            return None
        size = len(mirror_keys)
        if len(keys) > size and keys[:size] == mirror_keys and values[:size] == mirror_values:
            clear, deleted, cut = False, [], size
        else:
            clear, deleted, cut = _general_diff(mirror_keys, mirror_values, keys, values)
        tail = [entry(keys[position], values[position]) for position in range(cut, len(keys))]
        return [clear, deleted, tail]

    def _sync_in(self, exact) -> tuple:
        """`(sync, slow)`: what the session's mirror needs to equal the host's tables, and whether the full comparison was needed (then the view is taken anew after the call).

        The session keeps the very objects of the tables it saw after the last call: tables made of the same objects in the same order are unchanged (one pointer
        comparison per entry, in the extension). Anything else is compared in full over the lists of that view, entry by entry, order and values, as it always was.
        """

        registry, factorization, squarefree, support = exact._KNOWN_PRIMES, exact._FACTORIZATION_MEMO, exact._SQUAREFREE_MEMO, exact._PRIME_SUPPORT_MEMO
        session = self._session
        if session.view_matches(registry, factorization, squarefree, support):
            return UNCHANGED_SYNC, False
        self.slow_syncs += 1
        mirror_registry, *mirror_tables = session.view_lists()
        parts = [self._registry_sync(list(registry), mirror_registry)]
        for table, entry, mirror in zip((factorization, squarefree, support), (_entry_factorization, _entry_squarefree, _entry_support), mirror_tables):
            parts.append(self._table_sync(table, entry, mirror))
        if all(part is None for part in parts):
            return UNCHANGED_SYNC, True
        return [part if part is not None else [False, [], []] for part in parts], True

    def _remember(self, exact, mask: int, slow: bool) -> None:
        """The session's view of the host's tables after a call: the tables in `mask` (the call changed them), all of them after a full comparison."""

        if slow:
            mask = ALL_TABLES
        if mask:
            self._session.view_capture(exact._KNOWN_PRIMES, exact._FACTORIZATION_MEMO, exact._SQUAREFREE_MEMO, exact._PRIME_SUPPORT_MEMO, mask)

    # ---- sync out --------------------------------------------------------------------------------------------------

    def _apply_log(self, exact, results: list) -> set:
        """Replays the op-logs of every result on the real tables in place; returns the tables that changed."""

        changed: set = set()
        for result in results:
            changed |= self._apply_entries(exact, result.log)
        return changed

    @staticmethod
    def _apply_entries(exact, entries) -> set:
        """Replays one op-log on the real tables in place; returns the tables that changed."""

        primes, prime_set = exact._KNOWN_PRIMES, exact._KNOWN_PRIME_SET
        factorization, squarefree, support = exact._FACTORIZATION_MEMO, exact._SQUAREFREE_MEMO, exact._PRIME_SUPPORT_MEMO
        changed: set = set()
        for entry in entries:
            code = entry[0]
            if code == 2:
                insort(primes, entry[1])
                prime_set.add(entry[1])
                changed.add("registry")
            elif code == 4:
                factorization[entry[1]] = tuple(tuple(pair) for pair in entry[2])
                changed.add("factorization")
            elif code == 6:
                squarefree[entry[1]] = (entry[2], entry[3])
                changed.add("squarefree")
            elif code == 7:
                support[entry[1]] = tuple(entry[2])
                changed.add("support")
            elif code == 5:
                factorization[entry[1]] = factorization.pop(entry[1])
                changed.add("factorization")
            elif code == 3:
                if not factorization or next(iter(factorization)) != entry[1]:
                    raise NativeMirrorError("the native eviction names an oldest factorization the real table does not hold")
                del factorization[entry[1]]
                changed.add("factorization")
            elif code == 1:
                primes.clear()
                prime_set.clear()
                changed.add("registry")
            elif code == 0:
                exact.reset_factorization_memory()
                changed.update(("registry", "factorization", "squarefree", "support"))
            else:
                raise NativeMirrorError(f"unknown memory log entry {code}")
        return changed

    # ---- the budget ------------------------------------------------------------------------------------------------

    @staticmethod
    def _budget_value(budget):
        if budget is None:
            return None
        cap = budget.cap
        if cap is not None and (type(cap) is not int or cap < 0 or cap > _U64):
            raise ValueError(f"the native cost mirror takes a cap in 0..2**64, not {cap!r}")
        articles = budget.spent_by_article()
        if any(type(article) is not int or article < 0 or article > _U64 for article in articles):
            raise ValueError(f"the native cost mirror takes budget articles in 0..2**64, not {articles!r}")
        return [cap, *articles]

    # ---- the call --------------------------------------------------------------------------------------------------

    def execute(self, ops, budget=None, *, full_state: bool = False) -> list:
        """Runs a script of cost operations on the session; every effect is applied to the real objects.

        `ops` is a sequence of `(operation name, arguments)` of the `EXACT_*` family. The outcomes are returned
        per operation (`OpResult`), none is raised: a script runs to its end like a sequence of Python calls
        guarded by the caller. The budget articles written back are those after the LAST operation.
        """

        exact = _exact()
        sync, slow = self._sync_in(exact)
        header = [OPTION_FULL_STATE if full_state else 0, sync, self._budget_value(budget)]
        request = codec.encode_request(ops, cost=header)
        try:
            response = self._session.run(request)
        except BaseException:
            self.invalidate()
            raise
        results = [self._result(entry) for entry in codec.decode_response(response)]
        try:
            changed = self._apply_log(exact, results)
        except BaseException:
            self.invalidate()
            raise
        self._settle(exact, budget, results)
        self._remember(exact, sum(TABLE_BITS[name] for name in changed), slow)
        return results

    @staticmethod
    def _result(entry: list) -> OpResult:
        outcome, counts, articles, log, state = entry
        status = outcome[0]
        value = outcome[1] if status == STATUS_OK else None
        detail = outcome[1:] if status != STATUS_OK else None
        return OpResult(status, value, detail, counts, articles, log, state)

    @classmethod
    def _settle(cls, exact, budget, results: list) -> None:
        for result in results:
            cls._settle_counts(exact, result.counts)
        if results:
            cls._settle_articles(exact, budget, results[-1].articles)

    @staticmethod
    def _settle_counts(exact, counts) -> None:
        counters = exact.SIGN_COUNTS
        for key, delta in zip(COUNT_KEYS, counts):
            if delta:
                counters[key] += delta

    @staticmethod
    def _settle_articles(exact, budget, articles) -> None:
        """The articles after the call go into the real budget; a call without one adds its deltas to the telemetry."""

        if budget is None:
            target = exact.UNBUDGETED_WORK
            for name, delta in zip(ARTICLES, articles):
                if delta:
                    setattr(target, name, getattr(target, name) + delta)
        else:
            for name, value in zip(ARTICLES, articles):
                setattr(budget, name, value)

    # ---- the whole coverage operation -------------------------------------------------------------------------------

    def _bind_coverage(self) -> None:
        coverage, faces = _coverage()
        exact = _exact()
        outcome = coverage.CoverageOutcome
        self._session.bind_coverage(
            exact.SqrtSumV1, Fraction, coverage.CoverageV1, coverage.FaceCoverageV1,
            outcome.EXACT, outcome.PARTITION_IS_NOT_EXACT, outcome.ALPHA_IS_NEGATIVE, faces.FaceOutcome.EXACT, StoreKey, exact.FactorizationMemoryDeltaV1,
        )
        self._face_exact = faces.FaceOutcome.EXACT
        self._coverage_bound = True

    def coverage_at(self, partition, alpha, work_budget=None, store=None, traces=None):
        """`wavefront.coverage._coverage_at(partition, alpha, work_budget, store)`, whole, with its exact side effects.

        The partition is converted once per session (kept by identity); per call only `alpha`, the memory sync (nothing
        when no table changed), the budget state and the store cross the boundary, and the extension builds the
        `CoverageV1` itself and answers with plain tuples. The two refusals are the oracle's first two statements. Every
        effect (budget articles, counters, memory tables, the `store` entry) is applied before the result is returned or
        the exception of a refusal (an exhaustion, a face without a line) raised.

        The budget is the one the caller passes: the product runs the coverage of a domain on a FORK of the preparation's budget
        (`ExactWorkBudgetV1.forked("COVERAGE")`) and under `isolated_factorization_memory()` (a cold memory for the call, the caller's
        memory restored after it); both wrap this call from the CALLER, and the mirror follows them like any other change of the tables
        (the next sync in is a general diff). The `store` holds `(universe, delta, price, memory)` records: a hit of a budgeted call
        pays the recorded price (a record without a price, or one that does not fit under the cap, is computed again and replaced).

        `traces` (the signs and values `wavefront.coverage_template` records on its recording pass) is not produced by the extension: a
        call that asks for them is refused by name (`NativePortUnsupported`), not answered without them.
        """

        pin.require("coverage")
        if traces is not None:
            raise pin.NativePortUnsupported("the native coverage does not record the sign traces of `wavefront.coverage_template`: run the oracle for a recording pass")
        started = perf_counter_ns()
        if not self._coverage_bound:
            self._bind_coverage()
        if partition.outcome is not self._face_exact:
            return self._session.refused_coverage(partition, alpha, False)
        if alpha < 0:
            return self._session.refused_coverage(partition, alpha, True)
        exact = _exact()
        sync, slow = self._sync_in(exact)
        state = None if work_budget is None else (work_budget.cap, work_budget.spent_by_article())
        real = (exact._KNOWN_PRIMES, exact._KNOWN_PRIME_SET, exact._FACTORIZATION_MEMO, exact._SQUAREFREE_MEMO, exact._PRIME_SUPPORT_MEMO)
        called = perf_counter_ns()
        try:
            result, status, detail, counts, articles, bits, native = self._session.coverage_at(
                partition, alpha, None if sync is UNCHANGED_SYNC else codec.encode_value(sync), state, store, work_budget, real
            )
        except BaseException:
            self.invalidate()
            raise
        returned = perf_counter_ns()
        self._settle_counts(exact, counts)
        self._settle_articles(exact, work_budget, articles)
        self._remember(exact, bits, slow)
        finished = perf_counter_ns()
        self.last_timings = (called - started, returned - called, finished - returned, finished - started, *native)
        if status:
            if status == STATUS_MISSING_LINE:
                raise ValueError(f"у грани {partition.faces[detail[0]].owner} нет несущей прямой")
            OpResult(status, None, detail, counts, articles, (), None).raise_for(work_budget)
        return result

    # ---- the whole clip operation -----------------------------------------------------------------------------------

    def _bind_clip(self) -> None:
        """Hands the result classes to the extension, after checking they have the slots the Rust side fills (`NativePortStale` else)."""

        import dataclasses

        clip, numeric, laws, outcomes, refusal, triangle = _clip_kernel()
        pin.check_shapes(
            (
                ("materialize/clip.py", "ClippedV1 fields", tuple(item.name for item in dataclasses.fields(clip.ClippedV1)) == CLIPPED_FIELDS),
                ("numeric.py", "LocalPoint3V1 fields", tuple(item.name for item in dataclasses.fields(numeric.LocalPoint3V1)) == ("x", "y", "z")),
                ("materialize/lift_surface.py", "LiftTriangleV1 fields", tuple(item.name for item in dataclasses.fields(triangle)) == LIFT_TRIANGLE_FIELDS),
                ("materialize/admit.py", "MaterializationOutcome names", all(name in outcomes.__members__ for name in CLIP_REFUSAL_OUTCOMES)),
                ("contracts/geometry_batch.py", "DecalTopologyLawV1 members", hasattr(laws, "PLANAR_POLYGONS_V1") and hasattr(laws, "QUAD_STRIPS_V1")),
            )
        )
        self._session.bind_clip(_exact().SqrtSumV1, Fraction, clip.ClippedV1, numeric.LocalPoint3V1)
        self._clip_classes = (laws.PLANAR_POLYGONS_V1, laws.QUAD_STRIPS_V1, outcomes, refusal)
        self._clip_bound = True

    def clip_geometry(self, plane, budget, *, points, cycles, polygons, law, seam, fans, flows, by_faces, inert=frozenset()):
        """`materialize.clip.clip_geometry(plane, budget, ...)`, whole, with its exact side effects (the compute `clip_memo.run_clip` memoizes).

        The plane's triangles are converted once per session (kept by the identity of `plane.triangles`); per call the arguments are
        read from the Python containers by the extension, which builds the `ClippedV1` itself, reusing the objects the oracle would
        return as they are (input points, key strings, triangle names) and writing the offset normals straight into
        `plane._normal_by_position`, in call order, also when the operation fails afterwards. Every effect (budget articles, `SIGN_COUNTS`,
        `UNBUDGETED_WORK`, the four memory tables) is applied before the result is returned or the oracle's exception raised:
        `ExactCanonicalizationWorkBudgetExhausted` (text from `budget.exhaustion_detail`), `MaterializationRefusal`, `OverflowError`,
        `ZeroDivisionError`, `ValueError`, `KeyError`. The port refuses by name, and before it touches any state, when the oracle moved past
        the pin (`NativePortStale`), on an interpreter below the supported floor (`NativeUnsupportedPython`, see `pin.py`) and on an input
        it does not cover (`NativePortUnsupported`); there is no fallback to Python here. The sort of `_ordered` and the float fold of
        the offset normal are the kernel's explicit CPython 3.11 semantics (`_cpython311.py`), so nothing here depends on the version of
        the interpreter that runs the host.

        `inert` is the chain station plan's set of face pairs (`frozenset[frozenset[str]]`, only read by a clip by faces): the extension
        reads it in the iteration order of the set the caller passed, which is the order the oracle's `_plan_groups` walks it in.
        """

        pin.require("clip")
        started = perf_counter_ns()
        if not self._clip_bound:
            self._bind_clip()
        planar, quad, outcomes, refusal = self._clip_classes
        code = 0 if law is planar else 1 if quad is law else 2
        normals = getattr(plane, "_normal_by_position", None)
        if normals is None:
            raise pin.NativePortUnsupported("the plane has no `_normal_by_position` table to write the offset normals into")
        exact = _exact()
        sync, slow = self._sync_in(exact)
        state = None if budget is None else (budget.cap, budget.spent_by_article())
        real = (exact._KNOWN_PRIMES, exact._KNOWN_PRIME_SET, exact._FACTORIZATION_MEMO, exact._SQUAREFREE_MEMO, exact._PRIME_SUPPORT_MEMO)
        called = perf_counter_ns()
        try:
            result, status, detail, counts, articles, bits, native = self._session.clip_geometry(
                plane.triangles, points, cycles, polygons, code, seam, fans, flows, bool(by_faces), inert,
                None if sync is UNCHANGED_SYNC else codec.encode_value(sync), state, normals, real,
            )
        except BaseException:
            self.invalidate()
            raise
        returned = perf_counter_ns()
        self._settle_counts(exact, counts)
        self._settle_articles(exact, budget, articles)
        self._remember(exact, bits, slow)
        finished = perf_counter_ns()
        self.last_clip_timings = (called - started, returned - called, finished - returned, finished - started, *native)
        if status:
            self._raise_clip(status, detail, counts, articles, budget, outcomes, refusal)
        return result

    def raw_layouts(self) -> dict:
        """`{slot class: raw access engaged}`: a layout the probe did not confirm is read through the attribute protocol (slower, not wrong). Binds both entry points."""

        if not self._coverage_bound:
            self._bind_coverage()
        if not self._clip_bound:
            self._bind_clip()
        return dict(self._session.raw_layouts())

    def disable_raw(self) -> None:
        """Test-only: every slot through the attribute protocol (the fallback of the raw access); answers and cost are the same."""

        self._session.disable_raw()

    def round_trip(self, value, *, sum: bool = False):
        """Test-only: a `Fraction` (or, with `sum`, a `SqrtSumV1`) through the boundary conversions and back."""

        if not self._coverage_bound:
            self._bind_coverage()
        return self._session.round_trip(value, sum)

    def clip_cache_size(self) -> int:
        """Planes the extension keeps converted for `clip_geometry` (a debugging view)."""

        return self._session.clip_cache()

    def forget_clip(self) -> None:
        """Drops every converted plane and every cross-call result of `clip_geometry`: the next call starts cold (answers and cost unchanged)."""

        self._session.forget_clip()

    def set_clip_warm_enabled(self, enabled: bool) -> None:
        """Switches the cross-call cache of `clip_geometry` on or off (a harness knob: answers and cost are the same either way)."""

        self._session.set_clip_warm_enabled(enabled)

    def clear_clip_warm(self) -> None:
        """Drops the cross-call results of `clip_geometry` and keeps the converted planes (the next call is the first alpha on a known plane)."""

        self._session.clear_clip_warm()

    def clip_warm_stats(self) -> tuple:
        """`(crossing hits, crossings stored, crossing entries, value hits, lift hits)` of the cross-call cache of exact results (a debugging view)."""

        return self._session.clip_warm_stats()

    def set_clip_warm_limit(self, limit: int) -> None:
        """Test knob: the size at which the cross-call cache drops everything."""

        self._session.set_clip_warm_limit(limit)

    @staticmethod
    def _raise_clip(status, detail, counts, articles, budget, outcomes, refusal) -> None:
        """The oracle's exception for a refused clip (after every effect was applied)."""

        if status == CLIP_STATUS_OVERFLOW:
            raise OverflowError(OVERFLOW_TEXTS[detail[0]])
        if status == CLIP_STATUS_ZERO_DIVISION:
            raise ZeroDivisionError(detail[0])
        if status == CLIP_STATUS_VALUE:
            raise ValueError(detail[0])
        if status == CLIP_STATUS_REFUSAL:
            raise refusal(outcomes[detail[0]], detail[1])
        if status == CLIP_STATUS_MISSING_KEY:
            raise KeyError(detail[0])
        if status == CLIP_STATUS_UNSUPPORTED:
            raise pin.NativePortUnsupported(detail[0])
        OpResult(status, None, detail, counts, articles, (), None).raise_for(budget)

    # ---- one operation ----------------------------------------------------------------------------------------------

    def run_one(self, name: str, arguments, budget):
        """One cost operation as a Python call: effects applied, the exception raised, the value returned."""

        result = self.execute([(name, arguments)], budget)[0]
        if not result.ok:
            result.raise_for(budget)
        return result.value

    def sign(self, value, *, filter_bits: int = 64, budget=None) -> int:
        """`SqrtSumV1.sign(filter_bits=..., budget=...)`."""

        return self.run_one("EXACT_SIGN", (value, filter_bits), budget)

    def difference_sign(self, left, right, budget=None) -> int:
        """`left.difference_sign(right, budget)`: the sign of `left - right` (integer filter first, then the exact sign)."""

        return self.run_one("EXACT_DIFFERENCE_SIGN", (left, right), budget)

    def divided_by(self, numerator, denominator, budget=None):
        """`numerator.divided_by(denominator, budget)`."""

        return self.run_one("EXACT_DIVIDED_BY", (numerator, denominator), budget)

    def divided_by_generic(self, numerator, denominator, budget=None):
        """`_divided_by_generic(numerator, denominator, budget)`: the fallback of the integer loop, as an operation of its own."""

        return self.run_one("EXACT_DIVIDED_BY_GENERIC", (numerator, denominator), budget)

    def divide_with_prime_universe(self, numerator, denominator, prime_universe, budget=None):
        """`_divide_with_prime_universe(numerator, denominator, prime_universe, budget)`."""

        return self.run_one("EXACT_DIVIDE_WITH_UNIVERSE", (numerator, denominator, list(prime_universe)), budget)

    def radical(self, coefficient, radicand, budget=None):
        """`SqrtSumV1.radical(coefficient, radicand, budget)`."""

        return self.run_one("EXACT_RADICAL", (coefficient, radicand), budget)

    def radical_sum(self, parts, budget=None):
        """`radical_sum(parts, budget)`."""

        return self.run_one("EXACT_RADICAL_SUM", ([list(part) for part in parts],), budget)

    def squarefree_split(self, n: int, budget=None) -> tuple:
        """`squarefree_split(n, budget)`: `(outside, inside)`."""

        outside, inside = self.run_one("EXACT_SQUAREFREE_SPLIT", (n,), budget)
        return outside, inside

    def prime_support(self, radicand: int, budget=None) -> tuple:
        """`prime_support(radicand, budget)`."""

        return tuple(self.run_one("EXACT_PRIME_SUPPORT", (radicand,), budget))

    def reset_memory(self) -> None:
        """`reset_factorization_memory()`: the four tables (and the two pure caches Python clears with them)."""

        self.run_one("EXACT_RESET_MEMORY", (), None)

    def prime_universe_remembered(self, q_values, budget=None, store=None) -> tuple:
        """`prime_universe_remembered(q_values, budget, store)` (the default `build`): the universe; on a store miss (or a hit the budget
        could not afford) the `(universe, delta, price, memory)` record is written into `store` — only after the call succeeded, as Python
        does. A stored tuple of another length than four is no record: a miss, replaced."""

        q_values = tuple(q_values)
        if store is None:
            argument = None
        else:
            key = (PRIME_UNIVERSE_KEY, tuple(Fraction(value) for value in q_values))
            found = store.get(key)
            argument = False if found is None or len(found) != 4 else _record_wire(found)
        result = self.execute([("EXACT_PRIME_UNIVERSE", (list(q_values), argument))], budget)[0]
        if not result.ok:
            result.raise_for(budget)
        universe, record = result.value
        universe = tuple(universe)
        if record is not None:
            store[key] = _record_of_wire(universe, record)
        return universe
