"""The cost mirror: what makes a native exact operation cost exactly what the Python one costs.

Equality with the oracle covers the answer AND its side effects on process state: the six articles of the
`ExactWorkBudgetV1`, the `SIGN_COUNTS` counters, `UNBUDGETED_WORK` for a call without a budget, and the four tables
of the canonicalization memory (`_KNOWN_PRIMES` + `_KNOWN_PRIME_SET`, `_FACTORIZATION_MEMO` with its LRU order,
`_SQUAREFREE_MEMO`, `_PRIME_SUPPORT_MEMO`) with their insertion order. The native `Session` owns a mirror of the
tables. This module keeps the REAL Python objects and the mirror equal:

* BEFORE a call (`sync in`): every table is compared, order and values, with what the session last mirrored (the key
  and value lists are kept from the previous call, so an unchanged table costs one list copy and one comparison).
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
by the shim. The mirror is NOT thread-safe: it assumes nothing else mutates the tables during a call.

Adding a native operation that pays budget or touches the memory (the whole `coverage_at` / `clip_geometry` of the next
steps): give it an opcode in `codec.OPS` and the Rust table `script::OPS`, make its Rust answer the cost answer
`[outcome, counts, articles, log, state]` (`session.rs`), and run it through `CostMirror.execute` — the sync, the budget,
the log replay and the exception come for free. `OpResult.value` is the operation's own value.

A call that raises inside the extension (`ValueError`: a refused buffer; `RuntimeError`: a panic the extension caught)
leaves the real tables as they were before the call and resets the session; the mirror forgets what it held, so the next
call reloads the tables whole.
"""

from __future__ import annotations

from bisect import insort
from fractions import Fraction

from . import codec

#: `Operation::ALL` of the native side, in the order of `ExactWorkOperationV1`.
OPERATION_NAMES = ("PRIME_UNIVERSE", "COPRIME_BASIS", "PRIMALITY", "POLLARD_RHO_BRENT", "SQUAREFREE_SPLIT", "PRIME_SUPPORT", "EXACT_POSITION")
COUNT_KEYS = ("total", "closed_rational_zero", "closed_rational_nonzero", "closed_by_enclosure", "closed_by_conjugation")
ARTICLES = ("modular_squarings", "gcd_operations", "miller_rabin_rounds", "pollard_attempts", "radical_materializations", "exact_position_hydrations")

OPTION_FULL_STATE = 1
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


#: The sync of a call where nothing changed since the last one (the common case), encoded once.
UNCHANGED_SYNC = codec.Raw(codec.encode_value([[False, [], []] for _ in range(4)]))


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


class CostMirror:
    """Keeps one native session equal to the real process state and runs cost-bearing scripts on it."""

    def __init__(self, session) -> None:
        self._session = session
        self._reset_snapshots()

    def _reset_snapshots(self) -> None:
        self._primes: list = []
        self._tables = {"factorization": ([], []), "squarefree": ([], []), "support": ([], [])}

    def invalidate(self) -> None:
        """Forget everything the session mirrors: the next call reloads the tables whole."""

        self._session.clear()
        self._reset_snapshots()

    def lengths(self) -> tuple:
        """Native mirror lengths: registry, factorizations, squarefree splits, supports (a debugging view)."""

        return self._session.lengths()

    # ---- sync in ---------------------------------------------------------------------------------------------------

    def _registry_sync(self, primes: list):
        mirror = self._primes
        if primes == mirror:
            return None
        present, wanted = set(mirror), set(primes)
        removed = sorted(present - wanted)
        if len(removed) > 32 and 2 * len(removed) > len(mirror):
            return [True, [], list(primes)]
        return [False, removed, sorted(wanted - present)]

    def _table_sync(self, name: str, table: dict, entry) -> tuple:
        mirror_keys, mirror_values = self._tables[name]
        keys, values = list(table), list(table.values())
        if keys == mirror_keys and values == mirror_values:
            return None, (keys, values)
        size = len(mirror_keys)
        if len(keys) > size and keys[:size] == mirror_keys and values[:size] == mirror_values:
            clear, deleted, cut = False, [], size
        else:
            clear, deleted, cut = _general_diff(mirror_keys, mirror_values, keys, values)
        tail = [entry(keys[position], values[position]) for position in range(cut, len(keys))]
        return [clear, deleted, tail], (keys, values)

    def _sync_in(self, exact) -> tuple:
        primes = list(exact._KNOWN_PRIMES)
        parts = [self._registry_sync(primes)]
        tables = {}
        for name, table, entry in (
            ("factorization", exact._FACTORIZATION_MEMO, _entry_factorization),
            ("squarefree", exact._SQUAREFREE_MEMO, _entry_squarefree),
            ("support", exact._PRIME_SUPPORT_MEMO, _entry_support),
        ):
            sync, snapshot = self._table_sync(name, table, entry)
            parts.append(sync)
            tables[name] = snapshot
        if all(part is None for part in parts):
            return UNCHANGED_SYNC, primes, tables
        return [part if part is not None else [False, [], []] for part in parts], primes, tables

    # ---- sync out --------------------------------------------------------------------------------------------------

    def _apply_log(self, exact, results: list) -> set:
        """Replays the op-logs of every result on the real tables in place; returns the tables that changed."""

        primes, prime_set = exact._KNOWN_PRIMES, exact._KNOWN_PRIME_SET
        factorization, squarefree, support = exact._FACTORIZATION_MEMO, exact._SQUAREFREE_MEMO, exact._PRIME_SUPPORT_MEMO
        changed = set()
        for result in results:
            for entry in result.log:
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
        sync, primes, tables = self._sync_in(exact)
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
        self._snapshot(exact, primes, tables, changed)
        return results

    @staticmethod
    def _result(entry: list) -> OpResult:
        outcome, counts, articles, log, state = entry
        status = outcome[0]
        value = outcome[1] if status == STATUS_OK else None
        detail = outcome[1:] if status != STATUS_OK else None
        return OpResult(status, value, detail, counts, articles, log, state)

    @staticmethod
    def _settle(exact, budget, results: list) -> None:
        counters = exact.SIGN_COUNTS
        for result in results:
            for key, delta in zip(COUNT_KEYS, result.counts):
                if delta:
                    counters[key] += delta
        if not results:
            return
        articles = results[-1].articles
        target = exact.UNBUDGETED_WORK if budget is None else budget
        if budget is None:
            for name, delta in zip(ARTICLES, articles):
                if delta:
                    setattr(target, name, getattr(target, name) + delta)
        else:
            for name, value in zip(ARTICLES, articles):
                setattr(target, name, value)

    def _snapshot(self, exact, primes: list, tables: dict, changed: set) -> None:
        self._primes = list(exact._KNOWN_PRIMES) if "registry" in changed else primes
        real = {"factorization": exact._FACTORIZATION_MEMO, "squarefree": exact._SQUAREFREE_MEMO, "support": exact._PRIME_SUPPORT_MEMO}
        for name, table in real.items():
            self._tables[name] = (list(table), list(table.values())) if name in changed else tables[name]

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
        """`prime_universe_remembered(q_values, budget, store)` (the default `build`): the universe; on a store miss the
        `(universe, delta)` record is written into `store` — only after the call succeeded, as Python does."""

        q_values = tuple(q_values)
        if store is None:
            argument = None
        else:
            key = (PRIME_UNIVERSE_KEY, tuple(Fraction(value) for value in q_values))
            found = store.get(key)
            argument = False if found is None else found
        result = self.execute([("EXACT_PRIME_UNIVERSE", (list(q_values), argument))], budget)[0]
        if not result.ok:
            result.raise_for(budget)
        universe, record = result.value
        universe = tuple(universe)
        if record is not None:
            store[key] = (universe, tuple((number, tuple(tuple(pair) for pair in pairs)) for number, pairs in record[1]))
        return universe
