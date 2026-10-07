"""Oracle side of the builder slice of the skeleton port (WP-S3 builder, WP-S4 snapshot and plans): the calls of the Python kernel's `_Builder` and of the planning layer,
checked against the native seams on the SAME state, exactly like `native_leaf_gate.py` and `native_motorcycle_gate.py` do for the leaf.

What is checked, from a real `build_skeleton` (named, weighted, fan, generated, field and corpus polygons):

* `_Builder.__init__` (`BUILDER_INIT`): the state the oracle's builder is in after the seed (edges and their lines, vertices, the queue as a heap ARRAY with its sequence numbers,
  traces, the graph and the index, the counters, the memory), with the cost of the seed;
* the LOOP (`BUILDER_RUN`), step by step: the state at every call of `apply_superlevel_transaction` is recorded, and the native loop, started from the state the PREVIOUS
  transaction left, must stop at the next call in the state the oracle has there (or finish with the oracle's `SkeletonV1`), with the cost of the steps between (the pops of
  the queue, the `_count_at_time` walk, the residual test, the closing of the short LAVs, `_finish`); the transaction itself is the boundary and is not run natively;
* `collect_superlevel_snapshot` (`COLLECT_SNAPSHOT`): `repr` of the snapshot against the native text, the memory growth, the cost;
* `plan_split_materialization` (`PLAN_SPLIT_MATERIALIZATION`, every call the symbolic closure makes, on the snapshots it makes) or `plan_superlevel_components`
  (`PLAN_COMPONENTS`): `repr` of the plans.

    python tools/native_builder_gate.py gate [--only patch_006 ...] [--repeats 3]     # compute-only native against the oracle: the init, the loop to the first transaction, the snapshot, the plans

The oracle is timed WITHOUT the recording (a stopwatch around each call); the native compute time is measured inside the extension (the state is decoded before the stopwatch starts).
"""

from __future__ import annotations

import contextlib
import sys
import time
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import native_corpus as nc  # noqa: E402
import native_leaf_gate as leaf  # noqa: E402
import native_motorcycle_gate as motorcycle_gate  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
import cftuv_envelope.wavefront.skeleton as skeleton_module  # noqa: E402
import cftuv_envelope.wavefront.superlevel as superlevel  # noqa: E402
import cftuv_envelope.wavefront.superlevel_closure as closure_module  # noqa: E402
import cftuv_envelope.wavefront.symbolic_superlevel_coordinator as coordinator_module  # noqa: E402

SEAM_OF = {
    "collect_superlevel_snapshot": "COLLECT_SNAPSHOT",
    "plan_split_materialization": "PLAN_SPLIT_MATERIALIZATION",
    "plan_superlevel_components": "PLAN_COMPONENTS",
}


def text_of(value) -> str:
    return value if isinstance(value, str) else repr(value)


def is_canonical_pair(found) -> bool:
    """A native answer already in the canonical shape `(result, canonical state)` (the oracle's results are brought to it by the `normalize` of the call)."""

    return type(found) is tuple and len(found) == 2 and isinstance(found[1], dict)


class BuilderVerifier(leaf.LeafVerifier):
    """`LeafVerifier` plus the wrappers of the builder slice."""

    def __init__(self, sampling=None, *, only=None, mirror=None, plans: str = "materialization") -> None:
        super().__init__(sampling, only=only, mirror=mirror)
        from cftuv_native import builder_seams

        self.bseams = builder_seams
        self.plans = plans
        self.timed |= {"BUILDER_INIT", "BUILDER_RUN", "COLLECT_SNAPSHOT", "PLAN_COMPONENTS", "PLAN_SPLIT_MATERIALIZATION"}
        #: the oracle raised an internal error (`TypeError` of a sort) and the port answered by name that it does: a correct pair
        self.internal_agreed: Counter = Counter()
        self.stepper: Stepper | None = None
        self.steps: Counter = Counter()
        #: `(seam, text)` of every call the port refused: a refusal of the port is a failure of the comparison unless it is the named internal error
        self.unsupported_detail: list = []
        #: how many calls of each primitive were checked
        self.prims: Counter = Counter()

    def wanted(self, name: str) -> bool:
        seam = SEAM_OF.get(name, name)
        if self.depth or (self.only is not None and seam not in self.only):
            return False
        seen = self.seen[seam]
        self.seen[seam] = seen + 1
        return self.sampling.wants(seam, seen, self.checked[seam])

    # ---- a lockstep call whose oracle may end in the internal error the port names ---------------------------------------------

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
            detail = self.bseams.decode_text(answer.detail[0]) if answer.detail else ""
            if isinstance(error, TypeError) and detail.startswith(self.bseams.ORACLE_UNSUPPORTED_MARK):
                self.internal_agreed[seam] += 1
                self.remember(False)
                return
            self.unsupported[seam] += 1
            self.unsupported_detail.append((seam, detail))
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
        self.call_log.setdefault(seam, []).append([oracle_seconds, (answer.extras[0] if answer.extras else answer.nanoseconds) * 1e-9, encode_seconds + native_seconds + decode_seconds, ""])

    # ---- the init -----------------------------------------------------------------------------------------------------------------

    def check_init(self, polygon, budget, *, dense: bool = False):
        """`_Builder(polygon, ...)` on the live state against `BUILDER_INIT`; returns the oracle's builder (or raises the oracle's exception)."""

        seam = "BUILDER_INIT"
        options = self.bseams.enc_options(dense, motorcycle_gate.march_steps_for(motorcycle_gate.polygon_grid(polygon)), budget is not None)
        held: dict = {}

        def oracle():
            started = time.perf_counter()
            held["builder"] = skeleton_module._Builder(polygon, skeleton_module.SplitSearch.MOTORCYCLE, work_budget=budget, dense_hydration=dense)
            held["seconds"] = time.perf_counter() - started
            return held["builder"]

        def view_of(builder):
            return self.bseams.canon_state(self.bseams.enc_builder_state(builder))

        builder = self.lockstep(
            seam,
            oracle,
            lambda: [self.wire.enc_polygon(polygon), options],
            lambda value: self.bseams.canon_state(value),
            budget=budget,
            normalize=lambda value: value if isinstance(value, dict) else view_of(value),
        )
        return builder

    # ---- the loop ---------------------------------------------------------------------------------------------------------------

    @contextlib.contextmanager
    def stepping(self):
        """Wrappers on `_Builder.run` and `apply_superlevel_transaction`: the native loop is checked step by step against the oracle's (see the module note)."""

        stepper = Stepper(self)
        self.stepper = stepper
        original_run = skeleton_module._Builder.run
        saved = skeleton_module._Builder.__dict__["run"]
        skeleton_module._Builder.run = stepper.wrap_run(original_run)
        try:
            with leaf.swapped([("apply_superlevel_transaction", stepper.wrap_apply, superlevel)]):
                yield stepper
        finally:
            skeleton_module._Builder.run = saved
            self.stepper = None

    # ---- the snapshot and the plans ------------------------------------------------------------------------------------------

    def wrap_collect(self, original):
        verifier = self

        def collect_superlevel_snapshot(builder, level):
            if not verifier.wanted("collect_superlevel_snapshot"):
                return original(builder, level)
            memo = builder._position_memo
            active = memo is not None
            log = None
            if memo is not None:
                if not isinstance(memo.entries, verifier.wire.LoggingEntries):
                    object.__setattr__(memo, "entries", verifier.wire.LoggingEntries(memo.entries))
                log = memo.entries.log = verifier.wire.CallLog()
            budget = builder.work_budget
            options = verifier.bseams.enc_options(memo is None, None, budget is not None)

            def encode():
                state = verifier.bseams.enc_builder_state(builder, light=True, memo=verifier.wire.memo_wire(memo, log, active))
                return [state, verifier.bseams.enc_level(level), options]

            def extra(answer):
                added = list(verifier.wire.placed_in(log))
                reported = list(answer.value[-1])
                return [] if reported == added else [leaf.Mismatch("COLLECT_SNAPSHOT", "memory_growth", f"oracle inserted {added}, native {reported}")]

            try:
                return verifier.lockstep("COLLECT_SNAPSHOT", lambda: original(builder, level), encode, lambda value: verifier.bseams.dec_str(value[0]), budget=budget, normalize=text_of, extra=extra)
            finally:
                if log is not None:
                    memo.entries.log = None

        return collect_superlevel_snapshot

    def wrap_plan(self, original, seam: str):
        verifier = self

        def plan(snapshot, budget=None):
            name = {"PLAN_SPLIT_MATERIALIZATION": "plan_split_materialization", "PLAN_COMPONENTS": "plan_superlevel_components"}[seam]
            if not verifier.wanted(name):
                return original(snapshot, budget)
            return verifier.lockstep(seam, lambda: original(snapshot, budget), lambda: [verifier.bseams.enc_snapshot(snapshot)], verifier.bseams.dec_str, budget=budget, normalize=text_of)

        return plan

    # ---- the primitives ------------------------------------------------------------------------------------------------------

    def primitive_call(self, name: str, spec: tuple, builder, args: tuple, keywords: dict, run_original):
        """One call of a primitive of the builder or a helper of `superlevel`, on the state the oracle has at the call, against `BUILDER_PRIMITIVE`: the result and the
        whole state after it are compared, and the cost."""

        if not self.wanted(name):
            return run_original()
        code, encode, result_of = spec
        bseams = self.bseams
        try:
            state = bseams.enc_builder_state(builder)
            arguments = encode(*args, **keywords)
        except self.wire.SeamUnsupported:
            self.unsupported["BUILDER_PRIMITIVE"] += 1
            return run_original()
        queue = builder.queue
        future = self.wire.enc_time(queue._now) if hasattr(queue, "_now") else None
        options = bseams.enc_options(builder._position_memo is None, None, builder.work_budget is not None)
        request = [state, options, code, future, *arguments]

        def normalize(found):
            if is_canonical_pair(found):
                return found
            return (result_of(found), bseams.canon_state(bseams.enc_builder_state(builder)))

        def decode(value):
            return (value[0], bseams.canon_state(value[1]))

        found = self.lockstep("BUILDER_PRIMITIVE", run_original, lambda: request, decode, budget=builder.work_budget, normalize=normalize)
        self.prims[name] += 1
        return found

    def primitive_factory(self, name: str):
        spec = self.bseams.PRIMITIVES[name]
        verifier = self

        def factory(original):
            def call(builder, *args, **keywords):
                return verifier.primitive_call(name, spec, builder, args, keywords, lambda: original(builder, *args, **keywords))

            return call

        return factory

    @contextlib.contextmanager
    def primitives_installed(self):
        """The recording wrappers on the primitives of `_Builder` (class attributes) and on the helpers of `superlevel` (module functions)."""

        names = tuple(self.bseams.PRIMITIVES)
        methods = [name for name in names if hasattr(skeleton_module._Builder, name)]
        saved = {name: skeleton_module._Builder.__dict__[name] for name in methods}
        try:
            for name in methods:
                setattr(skeleton_module._Builder, name, self.primitive_factory(name)(getattr(skeleton_module._Builder, name)))
            helpers = [(name, self.primitive_factory(name), superlevel) for name in names if name not in methods and hasattr(superlevel, name)]
            helpers.append(("apply_superlevel_transaction", self.wrap_transaction, superlevel))
            with leaf.swapped(helpers):
                yield self
        finally:
            for name, raw in saved.items():
                setattr(skeleton_module._Builder, name, raw)

    # ---- the head of the transaction -------------------------------------------------------------------------------------------

    def wrap_transaction(self, original):
        """`apply_superlevel_transaction`: where the oracle returns before the symbolic closure the native HEAD must do the same (state and cost); where the oracle enters the
        closure the native head must say `continue` with the budget the closure was given."""

        verifier = self
        bseams = self.bseams

        def apply_superlevel_transaction(builder, level):
            if not verifier.wanted("apply_superlevel_transaction"):
                return original(builder, level)
            budget = builder.work_budget
            pre = leaf.take_pre(budget)
            try:
                state = bseams.enc_builder_state(builder)
                level_wire = bseams.enc_level(level)
            except verifier.wire.SeamUnsupported:
                verifier.unsupported["BUILDER_PRIMITIVE"] += 1
                return original(builder, level)
            entered: list = []

            def watch(closure_original):
                def plan_symbolic_superlevel_closure(*arguments, **keywords):
                    entered.append(keywords.get("outer_budget"))
                    return closure_original(*arguments, **keywords)

                return plan_symbolic_superlevel_closure

            error = None
            with leaf.swapped([("plan_symbolic_superlevel_closure", watch, coordinator_module)]):
                try:
                    original(builder, level)
                except Exception as exc:  # noqa: BLE001 - the oracle's refusal is an outcome
                    error = exc
            options = bseams.enc_options(builder._position_memo is None, None, budget is not None)
            request = [state, options, 21, None, level_wire]
            if entered:
                answer, _seconds = verifier.native("BUILDER_PRIMITIVE", request, pre)
                verifier.remember(False)
                if answer.unsupported or not answer.ok:
                    verifier.mismatches.append(leaf.Mismatch("TRANSACTION_HEAD", "decision", f"the oracle entered the closure, the native head answered status {answer.status}"))
                elif answer.value[0] != [1, entered[0]]:
                    verifier.mismatches.append(leaf.Mismatch("TRANSACTION_HEAD", "decision", f"the oracle entered the closure with budget {entered[0]}, the native head answered {answer.value[0]}"))
                verifier.prims["transaction head: into the closure"] += 1
            else:
                result = ("done", bseams.canon_state(bseams.enc_builder_state(builder))) if error is None else None
                verifier.check(
                    "BUILDER_PRIMITIVE",
                    pre,
                    lambda: request,
                    lambda value: ("done" if value[0] == [0, 0] else "continue", bseams.canon_state(value[1])),
                    budget,
                    result,
                    error,
                    0.0,
                    None,
                    None,
                )
                verifier.prims["transaction head: done"] += 1
            if error is not None:
                raise error

        return apply_superlevel_transaction

    def factories(self):
        found = [("collect_superlevel_snapshot", self.wrap_collect, superlevel)]
        if self.plans == "materialization":
            found.append(("plan_split_materialization", lambda original: self.wrap_plan(original, "PLAN_SPLIT_MATERIALIZATION"), closure_module))
        else:
            found.append(("plan_superlevel_components", lambda original: self.wrap_plan(original, "PLAN_COMPONENTS"), superlevel))
        return found

    @contextlib.contextmanager
    def installed(self):
        with leaf.swapped(self.factories()):
            yield self


class Stepper:
    """The recording of one `build_skeleton`: the state after each transaction is the start of the next native step, the state before the next transaction its answer."""

    def __init__(self, verifier: BuilderVerifier) -> None:
        self.verifier = verifier
        self.pending = None
        self.transactions = 0
        self.limit = 0
        self.budget = None
        self.options = None

    def capture(self, builder, resume: tuple):
        wire = self.verifier.bseams
        return {"state": wire.enc_builder_state(builder), "pre": leaf.take_pre(builder.work_budget), "resume": resume, "budget": builder.work_budget}

    # ---- the wrappers ------------------------------------------------------------------------------------------------------------

    def wrap_run(self, original):
        stepper = self

        def run(builder):
            stepper.transactions = 0
            stepper.budget = builder.work_budget
            stepper.limit = skeleton_module.level_budget(builder.polygon)
            stepper.options = stepper.verifier.bseams.enc_options(builder._position_memo is None, None, builder.work_budget is not None)
            try:
                stepper.pending = stepper.capture(builder, (0, 0))
            except stepper.verifier.wire.SeamUnsupported:
                stepper.pending = None
            try:
                result = original(builder)
            except Exception as error:  # noqa: BLE001 - the oracle's refusal ends the last step
                stepper.settle(builder, None, error)
                raise
            stepper.settle(builder, result, None)
            return result

        return run

    def wrap_apply(self, original):
        stepper = self

        def apply_superlevel_transaction(builder, level):
            stepper.arrive(builder, level)
            stepper.pending = None
            original(builder, level)
            try:
                stepper.pending = stepper.capture(builder, (1, stepper.transactions))
            except stepper.verifier.wire.SeamUnsupported:
                stepper.pending = None

        return apply_superlevel_transaction

    # ---- the steps ---------------------------------------------------------------------------------------------------------------

    def native_step(self, pending):
        mode, levels = pending["resume"]
        return [1, self.options, self.limit, pending["state"], mode, levels]

    def arrive(self, builder, level) -> None:
        """The oracle is at a call of the transaction: the native step that started at the previous one must be in the same place."""

        self.transactions += 1
        pending = self.pending
        verifier = self.verifier
        if pending is None:
            return
        bseams = verifier.bseams
        try:
            expected = ("stopped", self.transactions, [bseams._decode_event(bseams.enc_event(event)) for event in level], bseams.canon_state(bseams.enc_builder_state(builder), memo=False))
        except verifier.wire.SeamUnsupported:
            return

        def decode(value):
            found = bseams.dec_skeleton(value)
            if not found["stopped"]:
                return ("finished", found["skeleton"])
            return ("stopped", found["levels"], found["level"], bseams.canon_state(found["state"], memo=False))

        verifier.check("BUILDER_RUN", pending["pre"], lambda: self.native_step(pending), decode, pending["budget"], expected, None, 0.0, None, None)
        verifier.steps["stopped"] += 1

    def settle(self, builder, result, error) -> None:
        """The oracle's run is over: the last native step must finish with the same result (or the same exception)."""

        pending = self.pending
        verifier = self.verifier
        self.pending = None
        if pending is None:
            return
        bseams = verifier.bseams
        expected = None if error is not None else ("finished", bseams.canon_skeleton(bseams.skeleton_wire(result)))

        def decode(value):
            found = bseams.dec_skeleton(value)
            if found["stopped"]:
                return ("stopped", found["levels"], found["level"], None)
            return ("finished", found["skeleton"])

        verifier.check("BUILDER_RUN", pending["pre"], lambda: self.native_step(pending), decode, pending["budget"], expected, error, 0.0, None, None)
        verifier.steps["finished" if error is None else "raised"] += 1


def fresh_state():
    return leaf.fresh_process_state()


# --------------------------------------------------------------------------
# the gate: compute-only timing, native against the oracle
# --------------------------------------------------------------------------


class FirstTransaction(Exception):
    """Raised by the timing wrapper at the first call of the transaction: the oracle's run up to it is what is measured."""


def oracle_seconds(polygon, repeats: int) -> dict:
    """Seconds of the oracle, without any recording, the least of `repeats` runs of each: the init, the loop up to the first transaction, and (over a whole run) the
    snapshots, the plans of the closure and everything else of the transaction. The run is deterministic, so the repeats do the same work."""

    best: dict = {}
    for _ in range(repeats):
        budget = leaf.fresh_process_state()
        started = time.perf_counter()
        builder = skeleton_module._Builder(polygon, skeleton_module.SplitSearch.MOTORCYCLE, work_budget=budget)
        init = time.perf_counter() - started
        stamps: dict = {}

        def first(original):
            def apply_superlevel_transaction(builder, level):
                stamps["first"] = time.perf_counter()
                raise FirstTransaction

            return apply_superlevel_transaction

        started = time.perf_counter()
        with leaf.swapped([("apply_superlevel_transaction", first, superlevel)]):
            try:
                builder.run()
            except FirstTransaction:
                pass
        loop = stamps.get("first", time.perf_counter()) - started
        totals: Counter = Counter()

        def timed(name):
            def factory(original):
                def call(*arguments, **keywords):
                    began = time.perf_counter()
                    try:
                        return original(*arguments, **keywords)
                    finally:
                        totals[name] += time.perf_counter() - began

                return call

            return factory

        budget = leaf.fresh_process_state()
        factories = [("collect_superlevel_snapshot", timed("collect"), superlevel), ("plan_split_materialization", timed("plans"), closure_module), ("apply_superlevel_transaction", timed("transactions"), superlevel)]
        started = time.perf_counter()
        with leaf.swapped(factories):
            try:
                skeleton_module.build_skeleton(polygon, work_budget=budget)
            except Exception:  # noqa: BLE001 - a refusal of the oracle is an outcome here
                pass
        whole = time.perf_counter() - started
        row = {"init": init, "loop": loop, "collect": totals["collect"], "plans": totals["plans"], "transactions": totals["transactions"], "whole": whole}
        best = row if not best else {key: min(best[key], row[key]) for key in row}
    return best


def native_seconds(verifier: BuilderVerifier, polygon) -> dict:
    """The native compute of the same calls (inside the extension, the state decoded before the stopwatch): the init and the loop up to the first transaction by the whole-run
    seam, the snapshots and the plans over a whole checked run."""

    wire, bseams = verifier.wire, verifier.bseams
    budget = leaf.fresh_process_state()
    options = bseams.enc_options(False, None, True)
    answer = verifier.runner.call("BUILDER_RUN", [0, options, skeleton_module.level_budget(polygon), wire.enc_polygon(polygon), 0, 0], verifier.header(leaf.take_pre(budget)))
    verifier.remember(False)
    loop, init = (answer.extras[0] * 1e-9, answer.extras[1] * 1e-9) if answer.ok and len(answer.extras) > 1 else (float("nan"), float("nan"))
    before = {seam: Counter(row) for seam, row in verifier.timing.items()}
    with verifier.stepping(), verifier.installed():
        try:
            skeleton_module.build_skeleton(polygon, work_budget=leaf.fresh_process_state())
        except Exception:  # noqa: BLE001
            pass
    delta = lambda seam: verifier.timing.get(seam, Counter())["compute"] - before.get(seam, Counter())["compute"]  # noqa: E731
    return {"init": init, "loop": loop, "collect": delta("COLLECT_SNAPSHOT"), "plans": delta("PLAN_SPLIT_MATERIALIZATION")}


def gate(polygons, repeats: int = 3) -> list:
    """`[(name, oracle seconds by part, native seconds by part)]`; every checked call of the runs was also compared (a mismatch raises)."""

    verifier = BuilderVerifier(leaf.Sampling(head=10**9, stride=1, cap=10**9))
    rows = []
    for name, polygon in polygons:
        expected = oracle_seconds(polygon, repeats)
        found = native_seconds(verifier, polygon)
        assert not verifier.mismatches, [str(item)[:500] for item in verifier.mismatches[:2]]
        rows.append((name, expected, found))
    return rows


def format_gate(rows) -> str:
    lines = [f"{'polygon':58} {'part':8} {'oracle ms':>10} {'native ms':>10} {'speed-up':>9}"]
    for name, expected, found in rows:
        for part in ("init", "loop", "collect", "plans"):
            native = found[part]
            lines.append(f"{name[-58:]:58} {part:8} {1e3 * expected[part]:10.2f} {1e3 * native:10.2f} {expected[part] / native if native else float('inf'):9.1f}")
    return chr(10).join(lines)


def main(argv=None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("gate",))
    parser.add_argument("--only", nargs="*", default=None, help="tokens of the field polygon names")
    parser.add_argument("--repeats", type=int, default=3)
    arguments = parser.parse_args(argv)
    polygons = leaf.load_polygons(motorcycle_gate.FIELD_POLYGONS, arguments.only)
    print(format_gate(gate(polygons, arguments.repeats)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
