"""Oracle side of the skeleton leaf seams and gate G1: records the calls of the Python kernel's exact event layer and checks them against the native ones.

The unit of the native port is the whole `build_skeleton`; its leaf is the event layer (`event_time.py`, `candidate_law.py`, `exact_candidate_view.py`).
This module installs recording wrappers on those functions (module names are replaced in every module that imported them, and restored exactly) while the
oracle runs a real `build_skeleton`, and for each sampled call runs the native seam on the SAME state:

1. the state BEFORE the call is copied (the four memory tables with their order, `SIGN_COUNTS`, the six budget articles, the unbudgeted telemetry);
2. the ORACLE runs on the live state (its effects stay: the run goes on exactly as without the wrapper) and the wrappers write down what it read: the view
   it was given (`skeleton_seams.CallRecorder`), the memory entries it hit (`LoggingEntries`);
3. the NATIVE seam runs on the copy of the state before (an incremental sync of the native session's mirror) and its answer is compared EXACTLY: the result
   by `native_corpus.canonical` (type and value of every node: `int` and `Fraction` differ), the exception `(class, text)`, the `SIGN_COUNTS` delta, the
   budget articles (or the unbudgeted telemetry), and the memory log of the native call applied to the copy against the live tables AFTER the oracle.

`gate` is G1: every evaluate_split_candidate call of the polygons, compute-only and whole-call, native against Python on the same before-state.

    python tools/native_leaf_gate.py fetch                          # prepare the kernel fixtures (needs sympy) and pickle the polygons the skeleton gets
    python tools/native_leaf_gate.py gate [--only patch_006 ...]    # G1: p50 / p95 / total speed-up per polygon, by outcome and by the size of the call
    PYTHONPATH=<site of a wheel built with --features skeleton-profile> python tools/native_leaf_gate.py profile --only patch_006
                                                                    # where the native time of the evaluate calls goes (the phases of the leaf)

The oracle is timed WITHOUT the recording (a stopwatch around each call, the least of `--repeats` runs); the native compute time is measured inside the extension around
`evaluate_split_candidate` alone (the view and the memory it starts with are decoded before the stopwatch starts); the whole-call time adds the encoding of the view in
Python, the crossing and the decoding of the answer. Run it with Blender's Python (3.11, the product runtime) as well as the dev venv (3.13): the oracle differs, the
native side does not.
"""

from __future__ import annotations

import contextlib
import os
import pickle
import sys
import time
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import native_corpus as nc  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
import cftuv_envelope.wavefront.candidate_law as candidate_law  # noqa: E402
import cftuv_envelope.wavefront.event_time as event_time  # noqa: E402
import cftuv_envelope.wavefront.exact_candidate_view as candidate_view  # noqa: E402
import cftuv_envelope.wavefront.motorcycle as motorcycle  # noqa: E402

COUNT_KEYS = ("total", "closed_rational_zero", "closed_rational_nonzero", "closed_by_enclosure", "closed_by_conjugation")
SEAM_OF = {
    "evaluate_split_candidate": "EVALUATE_SPLIT_CANDIDATE",
    "compare_times": "COMPARE_TIMES",
    "concurrency_time": "CONCURRENCY_TIME",
    "sliding_time": "SLIDING_TIME",
    "sliding_point": "SLIDING_POINT",
    "_event_point": "EVENT_POINT",
    "_event_point_with_prime_universe": "EVENT_POINT",
}
#: The phases of the native evaluate call, in the order of `cftuv_skeleton::profile::PHASES` (answered only by a wheel built with the feature `skeleton-profile`).
PHASE_NAMES = ("other (glue, memory lookups)", "compare_times", "concurrency_time", "sliding_time", "sliding_point", "event_point: radicals", "event_point: sums and products", "event_point: divisions", "difference_sign", "memo keys", "memo lookups (hash, compare, clone)", "projections on a span")
#: What a `proof_identity_factory` answers in the native comparison (only its presence is compared).
IDENTITY_SENTINEL = ("identity",)


@dataclass
class Sampling:
    """How many calls of each seam to check: the head, then every `stride`-th, up to `cap` (a zero `cap` of a seam means none)."""

    head: int = 200
    stride: int = 25
    cap: int = 4000
    per_seam: dict = field(default_factory=dict)

    def wants(self, seam: str, seen: int, recorded: int) -> bool:
        head, stride, cap = self.per_seam.get(seam, (self.head, self.stride, self.cap))
        return recorded < cap and (seen < head or seen % stride == 0)


@dataclass
class Mismatch:
    seam: str
    field: str
    detail: str

    def __str__(self) -> str:
        return f"{self.seam}.{self.field}: {self.detail}"


@dataclass
class Pre:
    """The state of the process before a call (shallow copies: the entries are immutable tuples of ints)."""

    primes: list
    factorization: dict
    squarefree: dict
    support: dict
    counts: dict
    articles: tuple | None
    cap: int | None
    unbudgeted: tuple


class Tables:
    """The copies of the tables, with the names of the kernel module: the native memory log is applied here."""

    def __init__(self, pre: Pre) -> None:
        self._KNOWN_PRIMES = list(pre.primes)
        self._KNOWN_PRIME_SET = set(pre.primes)
        self._FACTORIZATION_MEMO = dict(pre.factorization)
        self._SQUAREFREE_MEMO = dict(pre.squarefree)
        self._PRIME_SUPPORT_MEMO = dict(pre.support)

    def reset_factorization_memory(self) -> None:
        self._KNOWN_PRIMES.clear()
        self._KNOWN_PRIME_SET.clear()
        self._FACTORIZATION_MEMO.clear()
        self._SQUAREFREE_MEMO.clear()
        self._PRIME_SUPPORT_MEMO.clear()


def take_pre(budget) -> Pre:
    return Pre(
        list(exact._KNOWN_PRIMES),
        dict(exact._FACTORIZATION_MEMO),
        dict(exact._SQUAREFREE_MEMO),
        dict(exact._PRIME_SUPPORT_MEMO),
        dict(exact.SIGN_COUNTS),
        None if budget is None else budget.spent_by_article(),
        None if budget is None else budget.cap,
        exact.UNBUDGETED_WORK.spent_by_article(),
    )


def normalize_decision(decision) -> tuple:
    """The decision with each proof identity reduced to whether one was asked for (the native side never builds the host's identity)."""

    effects = tuple((effect.reason, effect.proof_identity is not None, effect.counter_deltas) for effect in decision.effects)
    return (decision.candidate, effects)


class LeafVerifier:
    """Runs the native seams beside the oracle; collects mismatches, counts and the time of both sides."""

    def __init__(self, sampling: Sampling | None = None, *, only=None, mirror=None) -> None:
        import cftuv_native
        from cftuv_native import skeleton_seams as wire

        self.wire = wire
        self.cost = cftuv_native.cost
        self.mirror = cftuv_native.new_mirror() if mirror is None else mirror
        self.runner = wire.SeamRunner(self.mirror._session)
        self.sampling = sampling or Sampling()
        self.only = only
        self.mismatches: list[Mismatch] = []
        self.seen: Counter = Counter()
        self.checked: Counter = Counter()
        self.unsupported: Counter = Counter()
        self.timing: dict = {}
        self.depth = 0
        self.probe: list = []
        #: every checked `evaluate_split_candidate` call in order: `[oracle seconds, native compute seconds, whole-call seconds, outcome label]`
        self.calls: list = []
        self.outcomes: Counter = Counter()
        self.raised: Counter = Counter()
        #: the profile mode: only the calls made INSIDE an `evaluate_split_candidate` are checked, and that function itself is only marked
        self.restrict = False
        #: nanoseconds per phase summed over the checked evaluate calls (empty without the profile wheel)
        self.phases: list = []
        self.inside = 0

    # ---- the pieces of one lockstep call -----------------------------------------------------------------------------

    def header(self, pre: Pre):
        namespace = SimpleNamespace(_KNOWN_PRIMES=pre.primes, _FACTORIZATION_MEMO=pre.factorization, _SQUAREFREE_MEMO=pre.squarefree, _PRIME_SUPPORT_MEMO=pre.support)
        sync, _slow = self.mirror._sync_in(namespace)
        budget = None if pre.articles is None else [pre.cap, *pre.articles]
        return [0, sync, budget]

    def native(self, seam: str, arguments, pre: Pre):
        started = time.perf_counter()
        answer = self.runner.call(seam, arguments, self.header(pre))
        return answer, time.perf_counter() - started

    def compare_cost(self, seam: str, pre: Pre, answer, budget) -> list:
        found = []
        delta = [exact.SIGN_COUNTS[key] - pre.counts[key] for key in COUNT_KEYS]
        if list(answer.counts) != delta:
            found.append(Mismatch(seam, "sign_counts", f"oracle {delta} != native {list(answer.counts)}"))
        if budget is None:
            spent = [after - before for after, before in zip(exact.UNBUDGETED_WORK.spent_by_article(), pre.unbudgeted)]
        else:
            spent = list(budget.spent_by_article())
        if list(answer.articles) != spent:
            found.append(Mismatch(seam, "budget", f"oracle {spent} != native {list(answer.articles)}"))
        tables = Tables(pre)
        self.cost.CostMirror._apply_entries(tables, answer.log)
        live = {
            "known_primes": (tables._KNOWN_PRIMES, list(exact._KNOWN_PRIMES)),
            "factorization": (list(tables._FACTORIZATION_MEMO.items()), list(exact._FACTORIZATION_MEMO.items())),
            "squarefree": (list(tables._SQUAREFREE_MEMO.items()), list(exact._SQUAREFREE_MEMO.items())),
            "prime_support": (list(tables._PRIME_SUPPORT_MEMO.items()), list(exact._PRIME_SUPPORT_MEMO.items())),
        }
        for name, (got, want) in live.items():
            if got != want:
                found.append(Mismatch(seam, f"memory.{name}", f"{len(got)} native entries against {len(want)} of the oracle, order or value differs"))
        return found

    def remember(self, clean: bool) -> None:
        """The native session mirrors the live tables again (the call agreed), or forgets them (it did not: the next call reloads)."""

        if clean:
            self.mirror._session.view_capture(exact._KNOWN_PRIMES, exact._FACTORIZATION_MEMO, exact._SQUAREFREE_MEMO, exact._PRIME_SUPPORT_MEMO, 15)
        else:
            self.mirror.invalidate()

    def compare_outcome(self, seam: str, error, answer, budget, result, decoded) -> list:
        if error is not None:
            if answer.ok:
                return [Mismatch(seam, "exception", f"oracle raised {type(error).__qualname__}: {error}, native answered {nc.canonical(decoded)[:300]}")]
            native = self.wire.exception_of(answer, budget)
            return [] if native == (type(error).__qualname__, str(error)) else [Mismatch(seam, "exception", f"{(type(error).__qualname__, str(error))!r} != {native!r}")]
        if not answer.ok:
            return [Mismatch(seam, "result", f"native refused (status {answer.status} {answer.detail!r}), the oracle answered")]
        expected, actual = nc.canonical(result), nc.canonical(decoded)
        return [] if expected == actual else [Mismatch(seam, "result", f"oracle {expected[:400]} != native {actual[:400]}")]

    # ---- the lockstep call -------------------------------------------------------------------------------------------

    def lockstep(self, seam: str, oracle, encode, decode, *, budget, normalize=None, extra=None):
        """Runs `oracle()` on the live state, then the native seam on the copy of the state before it, and compares. Returns the oracle's result (or raises its exception)."""

        pre = take_pre(budget)
        self.depth += 1
        try:
            started = time.perf_counter()
            try:
                result, error = oracle(), None
            except Exception as exc:  # noqa: BLE001 - the oracle's exception is part of its outcome
                result, error = None, exc
            oracle_seconds = time.perf_counter() - started
            self.check(seam, pre, encode, decode, budget, result, error, oracle_seconds, normalize, extra)
        finally:
            self.depth -= 1
        if error is not None:
            raise error
        return result

    def check(self, seam, pre, encode, decode, budget, result, error, oracle_seconds, normalize, extra) -> None:
        try:
            encoded_at = time.perf_counter()
            arguments = encode()
            encode_seconds = time.perf_counter() - encoded_at
        except self.wire.SeamUnsupported:
            self.unsupported[seam] += 1
            self.remember(False)
            return
        answer, native_seconds = self.native(seam, arguments, pre)
        if answer.unsupported:
            self.unsupported[seam] += 1
            self.remember(False)
            return
        decoded, decode_seconds = None, 0.0
        if answer.ok:
            started = time.perf_counter()
            decoded = decode(answer.value)
            decode_seconds = time.perf_counter() - started
        found = self.compare_outcome(seam, error, answer, budget, normalize(result) if normalize and error is None else result, normalize(decoded) if normalize and decoded is not None else decoded)
        found += self.compare_cost(seam, pre, answer, budget)
        if extra is not None and answer.ok:
            found += extra(answer)
        self.remember(not found)
        self.mismatches.extend(found)
        self.checked[seam] += 1
        self.raised[seam] += error is not None
        row = self.timing.setdefault(seam, Counter())
        row["oracle"] += oracle_seconds
        row["encode"] += encode_seconds
        row["native_call"] += native_seconds
        row["decode"] += decode_seconds
        row["compute"] += (answer.extras[0] if answer.extras else answer.nanoseconds) * 1e-9
        row["inside"] += answer.nanoseconds * 1e-9
        if seam == "EVALUATE_SPLIT_CANDIDATE":
            if len(answer.extras) > 1:
                self.phases = [sum(pair) for pair in zip(self.phases or [0] * len(answer.extras[1]), answer.extras[1])]
            self.calls.append([oracle_seconds, (answer.extras[0] if answer.extras else answer.nanoseconds) * 1e-9, encode_seconds + native_seconds + decode_seconds, ""])

    def wanted(self, name: str) -> bool:
        seam = SEAM_OF[name]
        if self.depth or (self.only is not None and seam not in self.only) or (self.restrict and not self.inside):
            return False
        seen = self.seen[seam]
        self.seen[seam] = seen + 1
        return self.sampling.wants(seam, seen, self.checked[seam])

    # ---- the wrappers ------------------------------------------------------------------------------------------------

    def view_lockstep(self, seam: str, view, call, tail, decode, *, normalize=None, grown=True):
        """A lockstep call that reads a view: `call(view)` is the oracle on the recording view, `tail(recorder)` the arguments after the view and the memory."""

        memo = view.position_memo
        active = memo is not None and memo.admits(view.prime_universe)
        log = None
        if memo is not None:
            if not isinstance(memo.entries, self.wire.LoggingEntries):
                object.__setattr__(memo, "entries", self.wire.LoggingEntries(memo.entries))
            log = memo.entries.log = self.wire.CallLog()
        recorder = self.wire.CallRecorder(view, self.probe)

        def encode():
            return [recorder.view_wire(), self.wire.memo_wire(memo, log, active), *tail(recorder)]

        def extra(answer):
            added = list(self.wire.placed_in(log))
            reported = list(answer.value[-1])
            return [] if not grown or reported == added else [Mismatch(seam, "memory_growth", f"oracle inserted {added}, native {reported}")]

        try:
            return self.lockstep(seam, lambda: call(recorder.view), encode, decode, budget=view.budget, normalize=normalize, extra=extra)
        finally:
            if log is not None:
                memo.entries.log = None

    def check_position(self, view, vertex_ref, time):
        """`position(view, vertex, time)` against the native seam (the place, or `None`; the memory growth is part of the answer)."""

        def decode(wire):
            return None if wire[0] is None else self.wire.dec_point(wire[0])

        return self.view_lockstep(
            "POSITION", view, lambda recorded: candidate_view.position(recorded, vertex_ref, time), lambda r: [r.vertex_ids(vertex_ref), self.wire.enc_time(time)], decode
        )

    def check_containment(self, view, span_ref, point, time):
        """`span_containment(view, span, point, time)` against the native seam."""

        def decode(wire):
            return self.wire.dec_containment(wire)

        return self.view_lockstep(
            "SPAN_CONTAINMENT",
            view,
            lambda recorded: candidate_view.span_containment(recorded, span_ref, point, time),
            lambda r: [r.span_ids(span_ref), self.wire.enc_point(point), self.wire.enc_time(time)],
            decode,
        )

    def wrap_evaluate(self, original):
        verifier = self

        def evaluate_split_candidate(view, vertex_ref, target_ref, *, now, proof_identity_factory=None):
            if not verifier.wanted("evaluate_split_candidate"):
                return original(view, vertex_ref, target_ref, now=now, proof_identity_factory=proof_identity_factory)
            decision = verifier.view_lockstep(
                "EVALUATE_SPLIT_CANDIDATE",
                view,
                lambda recorded: original(recorded, vertex_ref, target_ref, now=now, proof_identity_factory=proof_identity_factory),
                lambda r: [r.vertex_ids(vertex_ref), r.span_ids(target_ref), verifier.wire.enc_time(now)],
                lambda wire: verifier.wire.dec_decision(wire, now, None if proof_identity_factory is None else (lambda: IDENTITY_SENTINEL)),
                normalize=normalize_decision,
            )
            verifier.note_decision(decision)
            return decision

        return evaluate_split_candidate

    def mark_evaluate(self, original):
        """The profile mode's `evaluate_split_candidate`: the call itself is not checked, the calls inside it are."""

        verifier = self

        def evaluate_split_candidate(*arguments, **keywords):
            verifier.inside += 1
            try:
                return original(*arguments, **keywords)
            finally:
                verifier.inside -= 1

        return evaluate_split_candidate

    def note_decision(self, decision) -> None:
        """Which outcome the call had (the coverage of the law's branches)."""

        label = "CANDIDATE" if decision.candidate is not None else decision.effects[0].reason.value
        self.outcomes[label] += 1
        if self.calls:
            self.calls[-1][3] = label

    def wrap_time_function(self, name: str, original):
        """`concurrency_time`, `sliding_time`, `sliding_point`, `_event_point`, `compare_times`: arguments in the wire, the budget is the last argument."""

        verifier = self
        encoders = {
            "compare_times": lambda a: [verifier.wire.enc_time(a[0]), verifier.wire.enc_time(a[1])],
            "concurrency_time": lambda a: [verifier.wire.enc_line(item) for item in a[:3]],
            "sliding_time": lambda a: [verifier.wire.enc_line(a[0]), a[1], verifier.wire.enc_line(a[2])],
            "sliding_point": lambda a: [verifier.wire.enc_line(a[0]), a[1], verifier.wire.enc_time(a[2])],
            "_event_point_with_prime_universe": lambda a: [verifier.wire.enc_line(a[0]), verifier.wire.enc_line(a[1]), verifier.wire.enc_time(a[2]), [int(prime) for prime in a[3]]],
        }
        decoders = {
            "compare_times": lambda wire: wire,
            "concurrency_time": verifier.wire.dec_time_entry,
            "sliding_time": verifier.wire.dec_time_entry,
            "sliding_point": verifier.wire.dec_point,
            "_event_point_with_prime_universe": verifier.wire.dec_point,
        }

        def wrapper(*arguments, **keywords):
            if not verifier.wanted(name) or name not in encoders:
                return original(*arguments, **keywords)
            budget = keywords.get("budget", arguments[-1] if len(arguments) > encoder_arity[name] else None)
            return verifier.lockstep(
                SEAM_OF[name],
                lambda: original(*arguments, **keywords),
                lambda: encoders[name](arguments),
                decoders[name],
                budget=budget,
            )

        encoder_arity = {"compare_times": 2, "concurrency_time": 3, "sliding_time": 3, "sliding_point": 3, "_event_point_with_prime_universe": 4}
        return wrapper

    @contextlib.contextmanager
    def installed(self):
        """Puts the recording wrappers on the oracle and the trace probe on `TraceV1.bounds_time`; everything is restored in the end (see `swapped`)."""

        original_bounds = motorcycle.TraceV1.bounds_time
        verifier = self

        def bounds_time(trace, time_value, budget=None):
            verifier.probe[:] = [trace.crash_time]
            return original_bounds(trace, time_value, budget)

        factories = (
            ("evaluate_split_candidate", self.mark_evaluate if self.restrict else self.wrap_evaluate, candidate_law),
            *((name, (lambda original, name=name: self.wrap_time_function(name, original)), event_time) for name in ("compare_times", "concurrency_time", "sliding_time", "sliding_point", "_event_point_with_prime_universe")),
        )
        motorcycle.TraceV1.bounds_time = bounds_time
        try:
            with swapped(factories):
                yield self
        finally:
            motorcycle.TraceV1.bounds_time = original_bounds

    # ---- report ------------------------------------------------------------------------------------------------------

    def report(self) -> dict:
        rows = {}
        for seam, count in sorted(self.checked.items()):
            row = self.timing[seam]
            rows[seam] = {
                "checked": count,
                "oracle_us": 1e6 * row["oracle"] / count,
                "compute_us": 1e6 * row["compute"] / count,
                "inside_us": 1e6 * row["inside"] / count,
                "whole_call_us": 1e6 * (row["encode"] + row["native_call"] + row["decode"]) / count,
                "compute_speedup": row["oracle"] / row["compute"] if row["compute"] else float("inf"),
            }
        return rows


@contextlib.contextmanager
def swapped(factories):
    """For each `(name, factory, home)`: the function `home.name` is replaced, in EVERY kernel module that holds it by name, by `factory(original)`.

    A module the oracle imports LATER (a lazy import inside a function) takes the wrapper from the module it imports it from, so on the way out every kernel module in
    `sys.modules` is swept for the wrappers, not only those that held the original on the way in."""

    originals = {}
    for name, factory, home in factories:
        original = getattr(home, name)
        originals[name] = (original, factory(original))

    def kernel_modules():
        return [module for module in list(sys.modules.values()) if module is not None and getattr(module, "__name__", "").startswith("cftuv_envelope.")]

    try:
        for module in kernel_modules():
            for name, (original, wrapper) in originals.items():
                if getattr(module, name, None) is original:
                    setattr(module, name, wrapper)
        yield
    finally:
        for module in kernel_modules():
            for name, (original, wrapper) in originals.items():
                if getattr(module, name, None) is wrapper:
                    setattr(module, name, original)


def fresh_process_state():
    """The product's entry state: empty memory, zero counters, a fresh `PREPARE` budget, the canonical audit off."""

    exact.reset_factorization_memory()
    exact.reset_sign_counts()
    exact.reset_unbudgeted_work()
    exact.set_canonical_audit(False)
    return exact.exact_work_budget(stage="PREPARE", domain_id="leaf-gate")


def run_polygon(polygon, *, work_budget=True):
    """`build_skeleton(polygon)` on a cold state; returns `(skeleton or exception, budget)`."""

    from cftuv_envelope.wavefront.skeleton import build_skeleton

    budget = fresh_process_state() if work_budget else None
    try:
        return build_skeleton(polygon, work_budget=budget), budget
    except Exception as exc:  # noqa: BLE001 - a refusal of the oracle is an outcome
        return exc, budget


def load_polygons(path: Path, only=None) -> list:
    """`[(name, polygon)]` of a pickle `{name: {"polygons": [...]}}` (the field fixtures prepared by the conveyor)."""

    with open(path, "rb") as handle:
        data = pickle.load(handle)
    found = []
    for name, row in data.items():
        polygons = row["polygons"]
        if polygons and polygons[0] is not None and (only is None or any(token in name for token in only)):
            found.append((name, polygons[0]))
    return found


# --------------------------------------------------------------------------
# gate G1
# --------------------------------------------------------------------------


def oracle_call_seconds(polygon, repeats: int) -> list:
    """Seconds of every `evaluate_split_candidate` call of a cold `build_skeleton`: the oracle alone, a stopwatch around each call and nothing else.

    The run is deterministic, so the calls of repeated runs correspond one to one and the least of the repeats is the call's time without the noise of the machine."""

    best = None
    for _ in range(repeats):
        seconds: list = []

        def factory(original, seconds=seconds):
            def timed(*arguments, **keywords):
                started = time.perf_counter()
                try:
                    return original(*arguments, **keywords)
                finally:
                    seconds.append(time.perf_counter() - started)

            return timed

        with swapped([("evaluate_split_candidate", factory, candidate_law)]):
            run_polygon(polygon)
        if best is not None and len(best) != len(seconds):
            raise RuntimeError("two runs of the same polygon made a different number of calls: the run is not deterministic")
        best = seconds if best is None else [min(left, right) for left, right in zip(best, seconds)]
    return best


def native_calls(polygon, repeats: int):
    """`(verifier, calls)`: every `evaluate_split_candidate` call checked against the native seam; the compute and whole-call times are the least of the repeats."""

    best = None
    for _ in range(repeats):
        verifier = LeafVerifier(Sampling(head=10**12, stride=1, cap=10**12), only={"EVALUATE_SPLIT_CANDIDATE"})
        with verifier.installed():
            run_polygon(polygon)
        if best is None:
            best = verifier
        else:
            for kept, new in zip(best.calls, verifier.calls):
                kept[1], kept[2] = min(kept[1], new[1]), min(kept[2], new[2])
            best.mismatches.extend(verifier.mismatches)
    return best, best.calls


def profile_polygon(name: str, polygon, repeats: int) -> list:
    """Where the native time of the evaluate calls goes: the primitives called INSIDE them, each checked on a sample and counted on every call."""

    verifier = LeafVerifier(Sampling(head=400, stride=5, cap=10**9), only={"COMPARE_TIMES", "CONCURRENCY_TIME", "SLIDING_TIME", "SLIDING_POINT", "EVENT_POINT"})
    verifier.restrict = True
    with verifier.installed():
        run_polygon(polygon)
    counted, calls = native_calls(polygon, repeats)
    total = sum(call[1] for call in calls)
    oracle = sum(call[0] for call in calls)
    lines = [f"{name[-60:]}: {len(calls)} evaluate calls, native {1e3 * total:.1f} ms, oracle (with its logging) {1e3 * oracle:.1f} ms; mismatches {len(verifier.mismatches)}"]
    for seam in sorted(verifier.seen):
        row = verifier.timing.get(seam)
        if not row:
            continue
        count = verifier.checked[seam]
        native_us, oracle_us = 1e6 * row["compute"] / count, 1e6 * row["oracle"] / count
        estimate = verifier.seen[seam] * native_us * 1e-6
        lines.append(f"  {seam:18s} calls {verifier.seen[seam]:7d}  native {native_us:7.2f} us  oracle {oracle_us:7.2f} us  x{oracle_us / native_us:5.1f}  = {1e3 * estimate:8.1f} ms ({100 * estimate / total:5.1f} % of the native time of the evaluate calls)")
    if counted.phases:
        lines.append(f"  phases of the native evaluate calls (the profile wheel, exclusive times, {1e-6 * sum(counted.phases):.1f} ms in all):")
        lines += [f"    {name:34s} {1e-6 * value:9.1f} ms  {100 * value / sum(counted.phases):5.1f} %" for name, value in zip(PHASE_NAMES, counted.phases)]
    return lines


def percentile(values: list, fraction: float) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(fraction * len(ordered)))]


def speedup_row(label: str, oracle: list, compute: list, whole: list) -> str:
    ratios = [left / right for left, right in zip(oracle, compute) if right > 0]
    wholes = [left / right for left, right in zip(oracle, whole) if right > 0]
    return (
        f"{label:44s} {len(oracle):7d}  oracle {1e6 * sum(oracle) / len(oracle):8.1f} us  native {1e6 * sum(compute) / len(compute):7.2f} us  "
        f"total x{sum(oracle) / sum(compute):6.1f}  p50 x{percentile(ratios, 0.5):6.1f}  p95 x{percentile(ratios, 0.95):6.1f}  "
        f"whole-call x{sum(oracle) / sum(whole):5.2f} (p50 x{percentile(wholes, 0.5):5.2f})"
    )


def gate_report(rows: list) -> list:
    """Table lines of the gate: per polygon, all together, by outcome and by the size of the call."""

    oracle, compute, whole, labels = [], [], [], []
    lines = []
    for name, seconds, calls in rows:
        oracle += seconds
        compute += [call[1] for call in calls]
        whole += [call[2] for call in calls]
        labels += [call[3] for call in calls]
        lines.append(speedup_row(name[-44:], seconds, [call[1] for call in calls], [call[2] for call in calls]))
    lines.append(speedup_row("ALL", oracle, compute, whole))
    for label in sorted(set(labels)):
        chosen = [index for index, found in enumerate(labels) if found == label]
        lines.append(speedup_row(f"  outcome {label}"[:44], [oracle[i] for i in chosen], [compute[i] for i in chosen], [whole[i] for i in chosen]))
    for low, high in ((0, 50e-6), (50e-6, 200e-6), (200e-6, 1e-3), (1e-3, float("inf"))):
        chosen = [index for index, seconds in enumerate(oracle) if low <= seconds < high]
        if chosen:
            lines.append(speedup_row(f"  oracle call {1e6 * low:.0f}..{1e6 * min(high, 1e9):.0f} us", [oracle[i] for i in chosen], [compute[i] for i in chosen], [whole[i] for i in chosen]))
    return lines


def run_gate(arguments) -> int:
    polygons = load_polygons(Path(arguments.polygons), arguments.only)
    if not polygons:
        raise SystemExit(f"no polygon of {arguments.polygons} matches {arguments.only}")
    rows, mismatches = [], 0
    print(f"python {sys.version.split()[0]}  repeats {arguments.repeats}  polygons {len(polygons)}", flush=True)
    for name, polygon in polygons:
        seconds = oracle_call_seconds(polygon, arguments.repeats)
        verifier, calls = native_calls(polygon, arguments.repeats)
        if len(seconds) != len(calls):
            raise SystemExit(f"{name}: the timed run made {len(seconds)} calls and the checked run {len(calls)}")
        mismatches += len(verifier.mismatches)
        for item in verifier.mismatches[:5]:
            print("MISMATCH", name, item)
        if not calls:
            print(f"{name[-44:]:44s}       0  no evaluate_split_candidate call: nothing to compare", flush=True)
            continue
        rows.append((name, seconds, calls))
        print(speedup_row(name[-44:], seconds, [call[1] for call in calls], [call[2] for call in calls]), flush=True)
    print()
    print("\n".join(gate_report(rows)))
    print(f"\nmismatches {mismatches}; calls {sum(len(seconds) for _name, seconds, _calls in rows)}")
    return 1 if mismatches else 0


def fetch_polygons(out: Path) -> int:
    """Prepares the conveyor on every kernel fixture (needs sympy) and pickles the polygons the skeleton gets: the input of the gate."""

    import json

    import cftuv_envelope as kernel
    from cftuv_envelope.wavefront import prepare_conveyor

    fixtures, found = ROOT / "kernel" / "fixtures", {}
    for snapshot in sorted(fixtures.rglob("analysis_snapshot.json")):
        for request in sorted(snapshot.parent.glob("decal_request*.json")):
            name = str(request.relative_to(fixtures))
            try:
                prepared = prepare_conveyor(kernel.AnalysisSnapshotCodecV1.loads(snapshot.read_bytes()), kernel.DecalRequestCodecV1.loads(request.read_bytes()))
            except Exception as exc:  # noqa: BLE001 - a fixture that does not prepare is named and skipped
                print("skipped", name, type(exc).__name__, str(exc)[:80])
                continue
            found[name] = {"outcome": prepared.outcome.value, "polygons": [region.bridge.polygon for region in prepared.regions]}
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "wb") as handle:
        pickle.dump(found, handle)
    print(json.dumps({"polygons": sum(len(row["polygons"]) for row in found.values()), "fixtures": len(found), "out": str(out)}))
    return 0


def main(argv=None) -> int:
    import argparse

    default = Path(os.environ.get("CFTUV_NATIVE_CORPUS", "E:/cftuv_native_corpus")) / "skeleton_polygons" / "field_polygons.pkl"
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest="command", required=True)
    gate = commands.add_parser("gate", help="G1: every evaluate_split_candidate call of the polygons, native against Python on the same state")
    gate.add_argument("--polygons", default=str(default))
    gate.add_argument("--only", nargs="*", default=None, help="substrings of the polygon names")
    gate.add_argument("--repeats", type=int, default=2)
    profile = commands.add_parser("profile", help="where the native time of the evaluate calls goes (the primitives inside them)")
    profile.add_argument("--polygons", default=str(default))
    profile.add_argument("--only", nargs="*", default=None)
    profile.add_argument("--repeats", type=int, default=1)
    fetch = commands.add_parser("fetch", help="prepare the kernel fixtures and pickle the skeleton polygons (needs sympy)")
    fetch.add_argument("--out", default=str(default))
    arguments = parser.parse_args(argv)
    if arguments.command == "profile":
        for name, polygon in load_polygons(Path(arguments.polygons), arguments.only):
            print(os.linesep.join(profile_polygon(name, polygon, arguments.repeats)), flush=True)
        return 0
    return run_gate(arguments) if arguments.command == "gate" else fetch_polygons(Path(arguments.out))


if __name__ == "__main__":
    sys.exit(main())
