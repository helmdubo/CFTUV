"""Oracle side of the second slice of the skeleton port (WP-S1 grid and motorcycle graph, WP-S2b edge law and the rest of the exact view): records the calls of the Python
kernel's functions and checks them against the native seams, on the SAME state, exactly like `native_leaf_gate.py` does for the first slice.

What is recorded, from a real `build_skeleton` (the named corpora, the field polygons), from the kernel tests (the pytest plugin below) and from direct calls:

* `build_motorcycle_graph(polygon, budget)`: the whole graph (walls, grid, buckets in order, every trace field, the eight counters, the next trace identity) with the cost
  (budget articles or the unbudgeted work, `SIGN_COUNTS`, the four memory tables with order) and the exceptions;
* `MotorcycleGraphV1.trace_for(...)`: the trace of a born vertex, the counters it moved and the identity it took;
* `TraceCandidateIndexV1`: every instance's calls (`register_line`, `register_trace`, `lines_near`, `vertices_near`, `knows_*`) as a SCRIPT replayed natively at the end (the
  index has no cost: its answers and its final tables are compared);
* `evaluate_edge_candidate`, `edge_event_time`, `is_future`, `collapsing_span`, `span_end`, `sliding_projection`, `classify_poststate_span`: on the view the call read
  (`native_skeleton_seams`-style recorder of `cftuv_native.skeleton_seams.CallRecorder`) with the memory entries it hit;
* `ProofLedger`: every ledger's operations as a SCRIPT replayed natively (the status, the obligations in their order).

    python tools/native_motorcycle_gate.py gate [--only patch_006 ...] [--repeats 3]      # timing, compute-only native against the oracle: the graph and the edge law
    python -m pytest kernel/tests/test_wavefront_motorcycle_graph.py -p native_motorcycle_gate     # the same checks inside the kernel tests (env: `CFTUV_MOTORCYCLE_GATE_OUT`)

The pytest plugin (`pytest_configure` .. `pytest_sessionfinish` below) is inert unless it is registered with `-p`.
"""

from __future__ import annotations

import contextlib
import functools
import json
import os
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

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
import cftuv_envelope.wavefront.candidate_law as candidate_law  # noqa: E402
import cftuv_envelope.wavefront.cell_grid as cell_grid  # noqa: E402
import cftuv_envelope.wavefront.exact_candidate_view as candidate_view  # noqa: E402
import cftuv_envelope.wavefront.motorcycle as motorcycle  # noqa: E402
import cftuv_envelope.wavefront.poststate_span as poststate_span  # noqa: E402
import cftuv_envelope.wavefront.proof as proof  # noqa: E402

SEAM_OF = {
    **leaf.SEAM_OF,
    "build_motorcycle_graph": "BUILD_MOTORCYCLE_GRAPH",
    "trace_for": "TRACE_FOR",
    "evaluate_edge_candidate": "EVALUATE_EDGE_CANDIDATE",
    "edge_event_time": "EDGE_EVENT_TIME",
    "is_future": "IS_FUTURE",
    "collapsing_span": "COLLAPSING_SPAN",
    "span_end": "SPAN_END",
    "sliding_projection": "SLIDING_PROJECTION",
    "classify_poststate_span": "CLASSIFY_POSTSTATE_SPAN",
}
OUT_ENVIRONMENT = "CFTUV_MOTORCYCLE_GATE_OUT"
FIELD_POLYGONS = Path(os.environ.get("CFTUV_NATIVE_CORPUS", "E:/cftuv_native_corpus")) / "skeleton_polygons" / "field_polygons.pkl"


#: `march_budget` as the module was written: a test that replaces it (to force the exhaustion of the march) is something the native side cannot see.
_MARCH_BUDGET = motorcycle.march_budget


def march_steps_for(grid):
    """The number of march steps the LIVE oracle uses for `grid` when its `march_budget` was replaced (the host hands it to the native side), else `None`."""

    return None if motorcycle.march_budget is _MARCH_BUDGET else motorcycle.march_budget(grid)


def polygon_grid(polygon):
    """The grid `build_motorcycle_graph` makes over the polygon."""

    return cell_grid.CellGridV1.covering(motorcycle.polygon_box(polygon), targets=max(4, polygon.vertex_count))


class Script:
    """The calls of one object (a trace index, a proof ledger) in order, as wire operations, with what the oracle answered."""

    def __init__(self, subject, head: list) -> None:
        self.subject = subject
        self.head = head
        self.ops: list = []
        self.results: list = []
        self.depth = 0
        self.unsupported = False


class MotorcycleVerifier(leaf.LeafVerifier):
    """`LeafVerifier` plus the wrappers of the second slice."""

    def __init__(self, sampling=None, *, only=None, mirror=None) -> None:
        super().__init__(sampling, only=only, mirror=mirror)
        self.timed |= {"EVALUATE_EDGE_CANDIDATE", "BUILD_MOTORCYCLE_GRAPH", "TRACE_FOR"}
        self.indexes: dict = {}
        self.ledgers: dict = {}
        self.replayed: Counter = Counter()

    def wanted(self, name: str) -> bool:
        seam = SEAM_OF[name]
        if self.depth or (self.only is not None and seam not in self.only) or (self.restrict and not self.inside):
            return False
        seen = self.seen[seam]
        self.seen[seam] = seen + 1
        return self.sampling.wants(seam, seen, self.checked[seam])

    # ---- the graph --------------------------------------------------------------------------------------------------

    def wrap_build(self, original):
        def build_motorcycle_graph(polygon, work_budget=None):
            if not self.wanted("build_motorcycle_graph"):
                return original(polygon, work_budget)
            return self.lockstep(
                "BUILD_MOTORCYCLE_GRAPH",
                lambda: original(polygon, work_budget),
                lambda: [self.wire.enc_polygon(polygon), march_steps_for(polygon_grid(polygon))],
                self.wire.dec_graph,
                budget=work_budget,
                normalize=self.wire.graph_view,
            )

        return build_motorcycle_graph

    def wrap_trace_for(self, original):
        def trace_for(graph, left, right, start_time, origin):
            if not self.wanted("trace_for"):
                return original(graph, left, right, start_time, origin)
            counters, next_ident = dict(graph.counters), self.wire.count_value(graph._next_ident)
            return self.lockstep(
                "TRACE_FOR",
                lambda: self.wire.trace_for_view(original(graph, left, right, start_time, origin), graph),
                lambda: [
                    self.wire.enc_graph(graph, traces=False, counters=counters, next_ident=next_ident),
                    self.wire.enc_line(left),
                    self.wire.enc_line(right),
                    self.wire.enc_time(start_time),
                    self.wire.enc_point(origin),
                    march_steps_for(graph.grid),
                ],
                self.wire.dec_trace_for,
                budget=graph.work_budget,
            )[0]

        return trace_for

    # ---- the trace index and the ledger: scripts ---------------------------------------------------------------------

    def record(self, script: Script | None, encode, original_call, result_of):
        """One call of a scripted object: the oracle runs, the operation and its answer are written down (only the outermost call of a nest)."""

        if script is None or script.depth:
            return original_call()
        script.depth += 1
        try:
            try:
                operation = encode()
            except self.wire.SeamUnsupported:
                script.unsupported = True
                return original_call()
            try:
                result = original_call()
            except KeyError:
                script.ops.append(operation)
                script.results.append([1])
                raise
            script.ops.append(operation)
            script.results.append(result_of(result))
            return result
        finally:
            script.depth -= 1

    def index_patches(self) -> list:
        """`(class, name, factory)` of the methods that are patched on the class: `trace_for`, `TraceCandidateIndexV1` and `ProofLedger` (the last two make scripts)."""

        verifier, wire, index_class, ledger_class = self, self.wire, motorcycle.TraceCandidateIndexV1, proof.ProofLedger

        def covering(original):
            def make(polygon, graph):
                index = original(polygon, graph)
                try:
                    head = [wire.enc_polygon(polygon), [wire.enc_trace(trace) for trace in graph.traces.values()]]
                except wire.SeamUnsupported:
                    return index
                verifier.indexes[id(index)] = Script(index, head)
                return index

            return staticmethod(make)

        def method(original, encode, result_of):
            def call(index, *arguments):
                script = verifier.indexes.get(id(index))
                return verifier.record(script, lambda: encode(*arguments), lambda: original(index, *arguments), result_of)

            return call

        index_methods = (
            ("register_line", lambda key, line: [0, key, wire.enc_line(line)], lambda found: None),
            ("register_trace", lambda vertex, trace: [1, vertex, wire.enc_trace(trace)], lambda found: found),
            ("lines_near", lambda vertex: [2, vertex], lambda found: [0, list(found)]),
            ("vertices_near", lambda key: [3, key], list),
            ("knows_vertex", lambda vertex: [4, vertex], lambda found: found),
            ("knows_line", lambda key: [5, key], lambda found: found),
        )
        patches = [(motorcycle.MotorcycleGraphV1, "trace_for", self.wrap_trace_for), (index_class, "covering", covering)]
        patches += [(index_class, name, (lambda original, encode=encode, result_of=result_of: method(original, encode, result_of))) for name, encode, result_of in index_methods]
        return patches + self.ledger_patches(ledger_class)

    def ledger_patches(self, ledger_class) -> list:
        verifier, wire = self, self.wire

        def init(original):
            def make(ledger, *arguments):
                original(ledger, *arguments)
                verifier.ledgers[id(ledger)] = Script(ledger, [])

            return make

        def operation(original, encode, result_of):
            def call(ledger, *arguments, **keywords):
                script = verifier.ledgers.get(id(ledger))
                return verifier.record(script, lambda: encode(*arguments, **keywords), lambda: original(ledger, *arguments, **keywords), result_of)

            return call

        def record_args(*, cause, disposition, vertex_ids=(), participant_edge_keys=(), target_edge_keys=(), level, event_kind=None):
            return wire.enc_obligation_record(cause, disposition, tuple(vertex_ids), tuple(participant_edge_keys), tuple(target_edge_keys), level, event_kind)

        def refusal_args(reason, *, vertex_ids, participant_edge_keys, target_edge_keys, level):
            return wire.enc_refusal_record(reason, tuple(vertex_ids), tuple(participant_edge_keys), tuple(target_edge_keys), level)

        def finalize(original):
            def call(ledger, dead_vertex_ids):
                dead = list(dead_vertex_ids)
                script = verifier.ledgers.get(id(ledger))
                return verifier.record(script, lambda: [3, dead], lambda: original(ledger, dead), lambda found: (found[0].value, tuple(found[1])))

            return call

        def discharge(original):
            def call(ledger, dead_vertex_ids):
                dead = list(dead_vertex_ids)
                script = verifier.ledgers.get(id(ledger))
                return verifier.record(script, lambda: [2, dead], lambda: original(ledger, dead), lambda found: None)

            return call

        return [
            (ledger_class, "__init__", init),
            (ledger_class, "record", lambda original: operation(original, record_args, lambda found: None)),
            (ledger_class, "record_refusal", lambda original: operation(original, refusal_args, lambda found: None)),
            (ledger_class, "discharge", discharge),
            (ledger_class, "finalize", finalize),
        ]

    @contextlib.contextmanager
    def class_patches(self):
        patches = self.index_patches()
        saved = [(owner, name, owner.__dict__[name]) for owner, name, _ in patches]
        try:
            for owner, name, factory in patches:
                original = getattr(owner, name)
                wrapper = factory(original)
                if isinstance(wrapper, staticmethod):
                    functools.update_wrapper(wrapper.__func__, original)
                else:
                    functools.update_wrapper(wrapper, original)
                setattr(owner, name, wrapper)
            yield
        finally:
            for owner, name, raw in saved:
                setattr(owner, name, raw)

    # ---- the view functions -----------------------------------------------------------------------------------------

    def wrap_edge(self, original):
        def evaluate_edge_candidate(view, vertex_ref, peer_ref, *, now, same_vertex, proof_identity_factory=None):
            keywords = {"now": now, "same_vertex": same_vertex, "proof_identity_factory": proof_identity_factory}
            if not self.wanted("evaluate_edge_candidate"):
                return original(view, vertex_ref, peer_ref, **keywords)
            decision = self.view_lockstep(
                "EVALUATE_EDGE_CANDIDATE",
                view,
                lambda recorded: original(recorded, vertex_ref, peer_ref, **keywords),
                lambda r: [r.vertex_ids(vertex_ref), r.vertex_ids(peer_ref), self.wire.enc_time(now), same_vertex],
                lambda wire: self.wire.dec_edge_decision(wire, now, None if proof_identity_factory is None else (lambda: leaf.IDENTITY_SENTINEL)),
                normalize=leaf.normalize_decision,
            )
            self.note_decision(decision, "EVALUATE_EDGE_CANDIDATE")
            return decision

        return evaluate_edge_candidate

    def wrap_view_call(self, name: str, original, seam: str, tail, decode, *, grown: bool = True):
        """A view function `name(view, *arguments, **keywords)`: `tail(r, *arguments, **keywords)` are the wire arguments after the view and the memory."""

        def call(view, *arguments, **keywords):
            if not self.wanted(name):
                return original(view, *arguments, **keywords)
            return self.view_lockstep(seam, view, lambda recorded: original(recorded, *arguments, **keywords), lambda r: tail(r, *arguments, **keywords), decode, grown=grown)

        return call

    def wrap_sliding_projection(self, original):
        def sliding_projection(first, second, point):
            if not self.wanted("sliding_projection"):
                return original(first, second, point)
            return self.lockstep(
                "SLIDING_PROJECTION",
                lambda: original(first, second, point),
                lambda: [self.wire.enc_line(first), self.wire.enc_line(second), self.wire.enc_point(point)],
                lambda wire: wire,
                budget=None,
            )

        return sliding_projection

    def factories(self):
        wire = self.wire
        view_calls = (
            ("edge_event_time", "EDGE_EVENT_TIME", lambda r, vertex, peer, now: [r.vertex_ids(vertex), r.vertex_ids(peer), wire.enc_time(now)], lambda found: wire.dec_time_entry(found[0]), True),
            (
                "is_future",
                "IS_FUTURE",
                lambda r, time_value, *vertices, now: [wire.enc_time(time_value), [r.vertex_ids(vertex) for vertex in vertices], wire.enc_time(now)],
                lambda found: found,
                False,
            ),
            ("collapsing_span", "COLLAPSING_SPAN", lambda r, vertex, peer, time_value: [r.vertex_ids(vertex), r.vertex_ids(peer), wire.enc_time(time_value)], lambda found: found[0], True),
            (
                "span_end",
                "SPAN_END",
                lambda r, vertex, span, time_value, *, at_start: [r.vertex_ids(vertex), r.span_ids(span), wire.enc_time(time_value), at_start],
                lambda found: found[0],
                True,
            ),
        )
        made = [
            (name, (lambda original, name=name, seam=seam, tail=tail, decode=decode, grown=grown: self.wrap_view_call(name, original, seam, tail, decode, grown=grown)), candidate_view)
            for name, seam, tail, decode, grown in view_calls
        ]
        made.append(
            (
                "classify_poststate_span",
                lambda original: self.wrap_view_call(
                    "classify_poststate_span",
                    original,
                    "CLASSIFY_POSTSTATE_SPAN",
                    lambda r, vertex, peer, birth: [r.vertex_ids(vertex), r.vertex_ids(peer), wire.enc_time(birth)],
                    wire.dec_poststate,
                ),
                poststate_span,
            )
        )
        return (
            *super().factories(),
            ("build_motorcycle_graph", self.wrap_build, motorcycle),
            ("evaluate_edge_candidate", self.wrap_edge, candidate_law),
            ("sliding_projection", self.wrap_sliding_projection, candidate_view),
            *made,
        )

    @contextlib.contextmanager
    def installed(self):
        with super().installed(), self.class_patches():
            yield self

    # ---- the scripts, replayed -------------------------------------------------------------------------------------

    def replay_indexes(self) -> None:
        for script in list(self.indexes.values()):
            if script.unsupported:
                self.unsupported["TRACE_INDEX_SCRIPT"] += 1
                continue
            answer = self.runner.call("TRACE_INDEX_SCRIPT", [*script.head, script.ops])
            if not answer.ok:
                self.unsupported["TRACE_INDEX_SCRIPT"] += 1
                continue
            index = script.subject
            wire = self.wire
            expected = [
                script.results,
                wire.enc_grid(index.grid),
                index.speed_bound,
                [[key, [list(cell) for cell in cells]] for key, cells in index.line_cells.items()],
                [[key, [list(cell) for cell in cells]] for key, cells in index.vertex_cells.items()],
            ]
            native = answer.value
            if nc.canonical(expected) != nc.canonical(native):
                self.mismatches.append(leaf.Mismatch("TRACE_INDEX_SCRIPT", "script", f"{len(script.ops)} operations: oracle {nc.canonical(expected)[:300]} != native {nc.canonical(native)[:300]}"))
            self.checked["TRACE_INDEX_SCRIPT"] += 1
            self.replayed["index operations"] += len(script.ops)

    def replay_ledgers(self) -> None:
        for script in list(self.ledgers.values()):
            if script.unsupported or not script.ops:
                continue
            answer = self.runner.call("PROOF_SCRIPT", [script.ops])
            if not answer.ok:
                self.unsupported["PROOF_SCRIPT"] += 1
                continue
            wire = self.wire
            results = [item if not isinstance(item, tuple) else (item[0], tuple(item[1])) for item in script.results]
            native_results = [(wire.dec_str(item[0]), tuple(wire.dec_obligation(entry) for entry in item[1])) if item is not None else None for item in answer.value[0]]
            expected = (results, tuple(script.subject.obligations))
            native = (native_results, tuple(wire.dec_obligation(entry) for entry in answer.value[1]))
            if nc.canonical(expected) != nc.canonical(native):
                self.mismatches.append(leaf.Mismatch("PROOF_SCRIPT", "script", f"{len(script.ops)} operations: oracle {nc.canonical(expected)[:300]} != native {nc.canonical(native)[:300]}"))
            self.checked["PROOF_SCRIPT"] += 1
            self.replayed["ledger operations"] += len(script.ops)

    def finish(self) -> None:
        """Replays the scripts of the objects seen (their calls are all recorded by now)."""

        self.replay_indexes()
        self.replay_ledgers()


# --------------------------------------------------------------------------
# the pytest plugin: the same checks inside the kernel tests
# --------------------------------------------------------------------------

_PLUGIN: dict = {}

#: Calls per seam the plugin checks in the kernel tests: `(head, stride, cap)`. The graph, the born traces and the poststate classes are few and heavy in meaning (all of them);
#: the pure projection of the symbolic overlay is called per vertex state (a sample).
PLUGIN_SAMPLING = {
    "BUILD_MOTORCYCLE_GRAPH": (10**9, 1, 10**9),
    "TRACE_FOR": (10**9, 1, 10**9),
    "CLASSIFY_POSTSTATE_SPAN": (400, 4, 3000),
    "SLIDING_PROJECTION": (120, 60, 1500),
    "COMPARE_TIMES": (60, 200, 400),
    "CONCURRENCY_TIME": (60, 200, 400),
    "SLIDING_TIME": (60, 200, 400),
    "SLIDING_POINT": (60, 200, 400),
    "EVENT_POINT": (60, 200, 400),
}


def pytest_configure(config) -> None:
    """Installs the verifier around the whole test session (a module imported later takes the wrappers from the modules it imports from)."""

    verifier = MotorcycleVerifier(leaf.Sampling(head=200, stride=12, cap=2500, per_seam=dict(PLUGIN_SAMPLING)))
    manager = verifier.installed()
    manager.__enter__()
    _PLUGIN.update(verifier=verifier, manager=manager, tagged=0)


def pytest_runtest_setup(item) -> None:
    """The product's value of the canonical audit (off) for the calls the test makes: the suite switches it on, and the native side does not model it."""

    if _PLUGIN:
        exact.set_canonical_audit(False)


def pytest_runtest_teardown(item) -> None:
    """A mismatch found while a test ran carries the test's id (a test that patches the oracle's internals is named in the verdict, not hidden in it)."""

    if _PLUGIN:
        verifier = _PLUGIN["verifier"]
        for found in verifier.mismatches[_PLUGIN["tagged"] :]:
            found.test = item.nodeid
        _PLUGIN["tagged"] = len(verifier.mismatches)


def pytest_sessionfinish(session, exitstatus) -> None:
    if not _PLUGIN:
        return
    verifier, manager = _PLUGIN.pop("verifier"), _PLUGIN.pop("manager")
    verifier.finish()
    manager.__exit__(None, None, None)
    summary = {
        "checked": dict(verifier.checked),
        "raised": dict(verifier.raised),
        "unsupported": dict(verifier.unsupported),
        "outcomes": dict(verifier.outcomes),
        "replayed": dict(verifier.replayed),
        "mismatches": [f"[{getattr(item, 'test', '')}] {item}"[:1500] for item in verifier.mismatches[:60]],
        "mismatch_count": len(verifier.mismatches),
        "mismatch_tests": dict(Counter(getattr(item, "test", "") for item in verifier.mismatches)),
    }
    out = os.environ.get(OUT_ENVIRONMENT)
    if out:
        Path(out).write_text(json.dumps(summary, indent=1), encoding="utf8")
    print("NATIVE_MOTORCYCLE_GATE", json.dumps(summary))


# --------------------------------------------------------------------------
# timing
# --------------------------------------------------------------------------


def best_seconds(call, repeats: int) -> float:
    best = float("inf")
    for _ in range(repeats):
        started = time.perf_counter()
        call()
        best = min(best, time.perf_counter() - started)
    return best


def graph_row(name: str, polygon, repeats: int) -> tuple:
    """`(oracle seconds, native compute seconds, native whole-call seconds, mismatch count)` of `build_motorcycle_graph` on a cold state (the least of the repeats)."""

    def cold():
        budget = leaf.fresh_process_state()
        motorcycle.build_motorcycle_graph(polygon, budget)

    oracle = best_seconds(cold, repeats)
    compute, whole, mismatches = float("inf"), float("inf"), 0
    for _ in range(repeats):
        verifier = MotorcycleVerifier(leaf.Sampling(head=10**9, stride=1, cap=10**9), only={"BUILD_MOTORCYCLE_GRAPH"})
        budget = leaf.fresh_process_state()
        with verifier.installed():
            motorcycle.build_motorcycle_graph(polygon, budget)
        row = verifier.timing["BUILD_MOTORCYCLE_GRAPH"]
        compute = min(compute, row["compute"])
        whole = min(whole, row["encode"] + row["native_call"] + row["decode"])
        mismatches += len(verifier.mismatches)
    return oracle, compute, whole, mismatches


def edge_calls(polygon, repeats: int) -> tuple:
    """`(oracle seconds per call, native compute seconds per call, whole-call seconds, labels, mismatches)` of every `evaluate_edge_candidate` of a cold `build_skeleton`."""

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

        with leaf.swapped([("evaluate_edge_candidate", factory, candidate_law)]):
            leaf.run_polygon(polygon)
        best = seconds if best is None else [min(left, right) for left, right in zip(best, seconds)]
    checked = None
    mismatches = 0
    for _ in range(repeats):
        verifier = MotorcycleVerifier(leaf.Sampling(head=10**12, stride=1, cap=10**12), only={"EVALUATE_EDGE_CANDIDATE"})
        with verifier.installed():
            leaf.run_polygon(polygon)
        calls = verifier.call_log.get("EVALUATE_EDGE_CANDIDATE", [])
        mismatches += len(verifier.mismatches)
        if checked is None:
            checked = calls
        else:
            for kept, new in zip(checked, calls):
                kept[1], kept[2] = min(kept[1], new[1]), min(kept[2], new[2])
    return best, checked or [], mismatches


def run_gate(arguments) -> int:
    polygons = leaf.load_polygons(Path(arguments.polygons), arguments.only)
    if not polygons:
        raise SystemExit(f"no polygon of {arguments.polygons} matches {arguments.only}")
    print(f"python {sys.version.split()[0]}  repeats {arguments.repeats}  polygons {len(polygons)}", flush=True)
    total_mismatches = 0
    graph_totals = [0.0, 0.0, 0.0]
    edge_oracle, edge_compute, edge_whole = [], [], []
    for name, polygon in polygons:
        oracle, compute, whole, mismatches = graph_row(name, polygon, arguments.repeats)
        total_mismatches += mismatches
        graph_totals = [graph_totals[0] + oracle, graph_totals[1] + compute, graph_totals[2] + whole]
        print(f"{name[-44:]:44s} build_motorcycle_graph  oracle {1e3 * oracle:9.2f} ms  native {1e3 * compute:8.2f} ms  x{oracle / compute:6.1f}  whole-call x{oracle / whole:5.2f}  mismatches {mismatches}", flush=True)
        if arguments.edges:
            seconds, calls, mismatches = edge_calls(polygon, arguments.repeats)
            total_mismatches += mismatches
            if len(seconds) != len(calls):
                raise SystemExit(f"{name}: the timed run made {len(seconds)} edge calls and the checked run {len(calls)}")
            if calls:
                edge_oracle += seconds
                edge_compute += [call[1] for call in calls]
                edge_whole += [call[2] for call in calls]
                print(leaf.speedup_row(name[-44:] + " edge", seconds, [call[1] for call in calls], [call[2] for call in calls]), flush=True)
    print(f"\nbuild_motorcycle_graph ALL: oracle {1e3 * graph_totals[0]:.1f} ms, native compute {1e3 * graph_totals[1]:.1f} ms: x{graph_totals[0] / graph_totals[1]:.1f}, whole-call x{graph_totals[0] / graph_totals[2]:.2f}")
    if edge_oracle:
        print(leaf.speedup_row("evaluate_edge_candidate ALL", edge_oracle, edge_compute, edge_whole))
    print(f"mismatches {total_mismatches}")
    return 1 if total_mismatches else 0


def main(argv=None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest="command", required=True)
    gate = commands.add_parser("gate", help="timing and equality of build_motorcycle_graph and evaluate_edge_candidate on the field polygons")
    gate.add_argument("--polygons", default=str(FIELD_POLYGONS))
    gate.add_argument("--only", nargs="*", default=None, help="substrings of the polygon names")
    gate.add_argument("--repeats", type=int, default=3)
    gate.add_argument("--edges", action="store_true", help="also every evaluate_edge_candidate call of a cold build_skeleton (slow: the whole skeleton runs 2 x repeats times)")
    arguments = parser.parse_args(argv)
    return run_gate(arguments)


if __name__ == "__main__":
    sys.exit(main())
