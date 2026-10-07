"""Oracle side of the symbolic-closure seams of the skeleton port (WP-S5): the calls of the Python kernel's symbolic layer, checked against the native seams on the SAME state,
exactly like `native_builder_gate.py` does for the builder and the plans.

What is checked, from a real `build_skeleton` (named, weighted, fan, generated, field and corpus polygons), in THREE passes (a wrapped call hides the calls inside it, so every
function is checked at the place where the oracle calls it from outside the others):

* `closure`: `plan_symbolic_superlevel_closure` (`PLAN_SYMBOLIC_CLOSURE`), the whole closure of a packet on the front the oracle has at the call, with the canonical text of the
  whole result (the materialization, the overlay, the contacts, the junction fixed point, the signatures, the counts), the cost, and the growth of the memory of places;
* `parts`: `with_line_ports`, `build_f0_overlay`, `initial_interior_contacts`, `build_symbolic_overlay`, `discover_interior_split_contacts`, `plan_mixed_generations`;
* `inner`: `discover_junction_contacts`, `apply_component_deltas`, `overlay_signature` (and `discover_interior_split_contacts` where `plan_mixed_generations` is not wrapped);
* `fabricated`: at every call of `plan_mixed_generations` (a real overlay of a real closure) the contacts and the spoiled overlays of `native_closure_fabricate` are given to the
  oracle's `normalize_mixed_generation` / `apply_mixed_generation`, to the discoveries and to `plan_mixed_generations`, and to the native seams (`NORMALIZE_MIXED_GENERATION`
  and the others) on the same state. A refusal of the oracle that is not a contract (`KeyError` of a spoiled overlay) is garbage and is counted, not compared.

    python tools/native_closure_gate.py gate [--only patch_006 ...] [--repeats 3]     # compute-only native against the oracle: the closure of every packet of a run

The oracle is timed WITHOUT the recording (a stopwatch around each call); the native compute time is measured inside the extension (the state, the snapshot and the overlays
are decoded before the stopwatch starts).
"""

from __future__ import annotations

import contextlib
import dataclasses
import random
import sys
import time
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import native_builder_gate as builder_gate  # noqa: E402
import native_closure_fabricate as fabricate  # noqa: E402
import native_leaf_gate as leaf  # noqa: E402
import native_motorcycle_gate as motorcycle_gate  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
import cftuv_envelope.wavefront.skeleton as skeleton_module  # noqa: E402
import cftuv_envelope.wavefront.event_time as event_time_module  # noqa: E402
import cftuv_envelope.wavefront.symbolic_component as component_module  # noqa: E402
import cftuv_envelope.wavefront.symbolic_f0_overlay as f0_module  # noqa: E402
import cftuv_envelope.wavefront.symbolic_junction_contacts as junction_module  # noqa: E402
import cftuv_envelope.wavefront.symbolic_mixed_generation as mixed_module  # noqa: E402
import cftuv_envelope.wavefront.symbolic_overlay as overlay_module  # noqa: E402
import cftuv_envelope.wavefront.symbolic_sparse_ports as ports_module  # noqa: E402
import cftuv_envelope.wavefront.symbolic_superlevel_coordinator as coordinator_module  # noqa: E402

SEAM_OF = {
    "plan_symbolic_superlevel_closure": "PLAN_SYMBOLIC_CLOSURE",
    "build_symbolic_overlay": "BUILD_SYMBOLIC_OVERLAY",
    "discover_interior_split_contacts": "DISCOVER_INTERIOR_CONTACTS",
    "plan_mixed_generations": "PLAN_MIXED_GENERATIONS",
    "discover_junction_contacts": "DISCOVER_JUNCTION_CONTACTS",
    "apply_component_deltas": "APPLY_COMPONENT_DELTAS",
    "with_line_ports": "CLOSURE_PART",
    "build_f0_overlay": "CLOSURE_PART",
    "initial_interior_contacts": "CLOSURE_PART",
    "overlay_signature": "CLOSURE_PART",
}
#: the passes: which functions are wrapped together (see the module note)
PASSES = {
    "closure": ("plan_symbolic_superlevel_closure",),
    "parts": ("with_line_ports", "build_f0_overlay", "initial_interior_contacts", "build_symbolic_overlay", "discover_interior_split_contacts", "plan_mixed_generations"),
    "inner": ("discover_junction_contacts", "apply_component_deltas", "overlay_signature", "discover_interior_split_contacts"),
    "fabricated": ("plan_symbolic_superlevel_closure", "plan_mixed_generations", "build_symbolic_overlay", "build_f0_overlay", "with_line_ports"),
}
HOMES = {
    "plan_symbolic_superlevel_closure": coordinator_module,
    "build_symbolic_overlay": overlay_module,
    "discover_interior_split_contacts": coordinator_module,
    "plan_mixed_generations": mixed_module,
    "discover_junction_contacts": junction_module,
    "apply_component_deltas": component_module,
    "with_line_ports": ports_module,
    "build_f0_overlay": f0_module,
    "initial_interior_contacts": coordinator_module,
    "overlay_signature": component_module,
}
#: the operation numbers of `CLOSURE_PART`
PART_OP = {"with_line_ports": 0, "build_f0_overlay": 1, "initial_interior_contacts": 2, "overlay_signature": 3}


def cold(memo) -> bool:
    """Whether the oracle's `SplitDecisionMemoV1` of a call holds no decision yet. A native seam starts from an EMPTY memo (the wire carries none), so a call whose memo is warm (the endpoint pass
    of the round filled it before the interior pass asked, or the last pass of the initial closure left it for generation zero) pays less than the seam and is not comparable on its own: the
    closure that contains it is (`PLAN_SYMBOLIC_CLOSURE`), and so is a call that starts cold."""

    return memo is None or not memo.decisions


class ClosureVerifier(builder_gate.BuilderVerifier):
    """`BuilderVerifier` plus the wrappers of the symbolic layer."""

    def __init__(self, sampling=None, *, only=None, mirror=None, passes: tuple = ("closure",), seed: int = 2026, cases: int = 4, spoiled: int = 3, scripted: int = 2, snapshots: int = 3) -> None:
        super().__init__(sampling, only=only, mirror=mirror)
        from cftuv_native import closure_seams

        self.cseams = closure_seams
        self.passes = passes
        self.rng = random.Random(seed)
        self.cases, self.spoiled, self.scripted, self.snapshots = cases, spoiled, scripted, snapshots
        self.timed |= set(SEAM_OF.values())
        #: how often each function was called from outside the others, and how many of those were checked
        self.part_calls: Counter = Counter()
        #: nanoseconds per phase of the native closure, summed over the checked calls (empty without a wheel built with the feature `profile`)
        self.closure_phases: list = []

    def native(self, seam: str, arguments, pre):
        answer, seconds = super().native(seam, arguments, pre)
        if seam in SEAM_OF.values() and len(answer.extras) > 1 and isinstance(answer.extras[1], list):
            self.closure_phases = [sum(pair) for pair in zip(self.closure_phases or [0] * len(answer.extras[1]), answer.extras[1])]
        return answer, seconds

    def wanted(self, name: str) -> bool:
        seam = SEAM_OF.get(name, name)
        if self.depth or (self.only is not None and seam not in self.only):
            return False
        self.part_calls[name] += 1
        key = name if seam == "CLOSURE_PART" else seam
        seen = self.seen[key]
        self.seen[key] = seen + 1
        return self.sampling.wants(seam, seen, self.checked[seam])

    # ---- one call --------------------------------------------------------------------------------------------------------------------

    def text(self, found) -> str:
        return found if isinstance(found, str) else self.cseams.canon_text(found)

    def memo_call(self, seam: str, builder, oracle, tail, normalize=None, *, guarded: bool = False):
        """`oracle()` on the live state against the seam: the arguments are `[state, options, *tail()]`; the memory entries the call hit are the memory the native call starts with.
        `guarded`: a refusal of the oracle that is not a contract is garbage (counted, not compared)."""

        memo = builder._position_memo
        active = memo is not None
        log = None
        if memo is not None:
            if not isinstance(memo.entries, self.wire.LoggingEntries):
                object.__setattr__(memo, "entries", self.wire.LoggingEntries(memo.entries))
            log = memo.entries.log = self.wire.CallLog()
        budget = builder.work_budget
        options = self.bseams.enc_options(memo is None, None, budget is not None)

        def encode():
            return [self.cseams.enc_closure_state(builder, self.wire.memo_wire(memo, log, active)), options, *tail()]

        def extra(answer):
            added = list(self.wire.placed_in(log))
            reported = list(answer.value[-1])
            return [] if reported == added else [leaf.Mismatch(seam, "memory_growth", f"oracle inserted {added}, native {reported}")]

        decode = lambda value: self.bseams.dec_str(value[0])  # noqa: E731
        try:
            if not guarded:
                return self.lockstep(seam, oracle, encode, decode, budget=budget, normalize=normalize or self.text, extra=extra)
            return self.guarded(seam, oracle, encode, decode, budget, normalize or self.text, extra)
        finally:
            if log is not None:
                memo.entries.log = None

    def guarded(self, seam, oracle, encode, decode, budget, normalize, extra):
        """`lockstep` for inputs no front makes: the oracle's internal errors are garbage."""

        pre = leaf.take_pre(budget)
        self.depth += 1
        try:
            started = time.perf_counter()
            try:
                result, error = oracle(), None
            except Exception as exc:  # noqa: BLE001 - the oracle's refusal of a spoiled input is an outcome
                result, error = None, exc
            seconds = time.perf_counter() - started
        finally:
            self.depth -= 1
        if error is not None and not isinstance(error, (TypeError, exact.ExactCanonicalizationWorkBudgetExhausted)):
            self.garbage[type(error).__name__] += 1
            self.remember(False)
            return None
        self.check(seam, pre, encode, decode, budget, result, error, seconds, normalize, extra)
        return result

    def pure_call(self, seam: str, overlay_time, oracle, tail, *, guarded: bool = False):
        """A call that reads no builder (`apply_component_deltas`, `overlay_signature`): a bare state."""

        def encode():
            return [self.cseams.bare_state(overlay_time), self.bseams.enc_options(False, None, False), *tail()]

        decode = lambda value: self.bseams.dec_str(value[0])  # noqa: E731
        if guarded:
            return self.guarded(seam, oracle, encode, decode, None, self.text, None)
        return self.lockstep(seam, oracle, encode, decode, budget=None, normalize=self.text)

    # ---- the wrappers ----------------------------------------------------------------------------------------------------------------

    def wrap_closure(self, original):
        verifier = self

        def plan_symbolic_superlevel_closure(builder, snapshot, *, outer_budget, junction_budget):
            run = lambda: original(builder, snapshot, outer_budget=outer_budget, junction_budget=junction_budget)  # noqa: E731
            if not verifier.wanted("plan_symbolic_superlevel_closure"):
                return run()
            return verifier.memo_call("PLAN_SYMBOLIC_CLOSURE", builder, run, lambda: [verifier.bseams.enc_snapshot(snapshot), outer_budget, junction_budget])

        return plan_symbolic_superlevel_closure

    def wrap_closure_fabricating(self, original):
        """`plan_symbolic_superlevel_closure` over packets no front freezes (see `native_closure_fabricate.spoiled_snapshots`), then the real call, unchecked."""

        verifier = self

        def plan_symbolic_superlevel_closure(builder, snapshot, *, outer_budget, junction_budget):
            if not verifier.depth and (verifier.only is None or "PLAN_SYMBOLIC_CLOSURE" in verifier.only):
                # the real packet with budgets that run out (a fixed point that is cut short: the named exhaustion), then packets no front freezes
                for spoiled in (snapshot, *fabricate.spoiled_snapshots(snapshot, verifier.rng, verifier.snapshots)):
                    outer, junction = verifier.rng.choice((0, 1, 3, outer_budget)), verifier.rng.choice((0, 1, 3, junction_budget))
                    run = lambda spoiled=spoiled, outer=outer, junction=junction: original(builder, spoiled, outer_budget=outer, junction_budget=junction)  # noqa: E731
                    tail = lambda spoiled=spoiled, outer=outer, junction=junction: [verifier.bseams.enc_snapshot(spoiled), outer, junction]  # noqa: E731
                    verifier.memo_call("PLAN_SYMBOLIC_CLOSURE", builder, run, tail, guarded=True)
            return original(builder, snapshot, outer_budget=outer_budget, junction_budget=junction_budget)

        return plan_symbolic_superlevel_closure

    def wrap_build_overlay(self, original):
        verifier = self

        def build_symbolic_overlay(builder, snapshot, materialization, *, include_line_ports=False):
            run = lambda: original(builder, snapshot, materialization, include_line_ports=include_line_ports)  # noqa: E731
            if not verifier.wanted("build_symbolic_overlay"):
                return run()
            tail = lambda: [verifier.cseams.enc_vertices(snapshot.vertices), verifier.cseams.enc_materialization(materialization), include_line_ports]  # noqa: E731
            return verifier.memo_call("BUILD_SYMBOLIC_OVERLAY", builder, run, tail)

        return build_symbolic_overlay

    def wrap_overlay_call(self, name: str, seam: str):
        """`discover_interior_split_contacts(builder, overlay, memo)` and `discover_junction_contacts(builder, overlay, memo)`; only a call that starts with an empty memo is compared (see `cold`)."""

        verifier = self

        def factory(original):
            def call(builder, overlay, memo=None):
                if not cold(memo) or not verifier.wanted(name):
                    return original(builder, overlay, memo)
                return verifier.memo_call(seam, builder, lambda: original(builder, overlay, memo), lambda: [verifier.cseams.enc_overlay(overlay, builder)])

            return call

        return factory

    def wrap_mixed(self, original):
        verifier = self

        def plan_mixed_generations(builder, initial, discover_interior, *, budget, initial_memo=None):
            run = lambda: original(builder, initial, discover_interior, budget=budget, initial_memo=initial_memo)  # noqa: E731
            if not cold(initial_memo) or not verifier.wanted("plan_mixed_generations"):
                return run()
            return verifier.memo_call("PLAN_MIXED_GENERATIONS", builder, run, lambda: [verifier.cseams.enc_overlay(initial, builder), budget])

        return plan_mixed_generations

    def check_generation(self, builder, overlay, junction, interior):
        """`normalize_mixed_generation` and `apply_mixed_generation` of the oracle on contacts that were made (see `native_closure_fabricate`) against the native seam."""

        def oracle():
            expanded, generation, reason = mixed_module.normalize_mixed_generation(builder, overlay, junction, interior)
            if reason is not None:
                return (None, None, reason, None, None)
            applied, why = mixed_module.apply_mixed_generation(expanded, generation)
            return (expanded, generation, None, applied, why)

        cseams = self.cseams
        tail = lambda: [cseams.enc_overlay(overlay, builder), [cseams.enc_junction_contact(item) for item in junction], [cseams.enc_interior_contact(item) for item in interior]]  # noqa: E731
        found = self.memo_call("NORMALIZE_MIXED_GENERATION", builder, oracle, tail, guarded=True)
        if found is not None and found[1] is not None:
            for variant in fabricate.delta_variants(self.rng, found[0], found[1].deltas):
                self.check_apply(found[0], variant)
        return found

    def check_apply(self, overlay, deltas) -> None:
        """`apply_component_deltas` over deltas the normalisation does not make (a delta twice, a birth the overlay already holds, an arm to a leaf nobody owns)."""

        cseams = self.cseams
        reason = "SYMBOLIC_MIXED_COMPONENT_DELTAS_OVERLAP"
        oracle = lambda: component_module.apply_component_deltas(overlay, deltas, collision_reason=reason)  # noqa: E731
        tail = lambda: [cseams.enc_overlay(overlay), [cseams.enc_delta(delta) for delta in deltas], self.wire.enc_str(reason)]  # noqa: E731
        self.pure_call("APPLY_COMPONENT_DELTAS", overlay.time, oracle, tail, guarded=True)

    def wrap_part_fabricating(self, name: str, original):
        """`build_f0_overlay(builder, snapshot, time)` and `with_line_ports(builder, snapshot, time)` over frozen vertices and builders no front makes, then the real call, unchecked."""

        verifier = self
        op = PART_OP[name]
        cseams, wire = self.cseams, self.wire

        def with_builder(builder, snapshot, time_value):
            if not verifier.depth and (verifier.only is None or "CLOSURE_PART" in verifier.only):
                for turn in range(verifier.spoiled):
                    vertices = fabricate.spoiled_vertices(verifier.rng, snapshot.vertices)
                    owners = fabricate.SharedOwner(builder, verifier.rng) if turn % 3 == 2 else contextlib.nullcontext()
                    spoiled = dataclasses.replace(snapshot, vertices=vertices)
                    tail = lambda vertices=vertices: [op, cseams.enc_vertices(vertices), wire.enc_time(time_value)]  # noqa: E731
                    normalize = (lambda found: found if isinstance(found, str) else ("None" if found is None else cseams.canon_text(found.vertices))) if name == "with_line_ports" else None
                    with owners:
                        verifier.memo_call("CLOSURE_PART", builder, lambda: original(builder, spoiled, time_value), tail, normalize=normalize, guarded=True)
            return original(builder, snapshot, time_value)

        return with_builder

    def wrap_overlay_fabricating(self, original):
        """`build_symbolic_overlay` over materializations a packet never makes (see `native_closure_fabricate.spoiled_materialization`), then the real call, unchecked."""

        verifier = self

        def build_symbolic_overlay(builder, snapshot, materialization, *, include_line_ports=False):
            if not verifier.depth and (verifier.only is None or "BUILD_SYMBOLIC_OVERLAY" in verifier.only):
                for _ in range(verifier.spoiled):
                    made = fabricate.spoiled_materialization(verifier.rng, snapshot.vertices, materialization)
                    if made is None:
                        continue
                    vertices, spoiled = made
                    cseams = verifier.cseams
                    tail = lambda vertices=vertices, spoiled=spoiled: [cseams.enc_vertices(vertices), cseams.enc_materialization(spoiled), include_line_ports]  # noqa: E731
                    run = lambda vertices=vertices, spoiled=spoiled: original(builder, dataclasses.replace(snapshot, vertices=vertices), spoiled, include_line_ports=include_line_ports)  # noqa: E731
                    verifier.memo_call("BUILD_SYMBOLIC_OVERLAY", builder, run, tail, guarded=True)
                with fabricate.SharedOwner(builder, verifier.rng):
                    run = lambda: original(builder, snapshot, materialization, include_line_ports=include_line_ports)  # noqa: E731
                    tail = lambda: [verifier.cseams.enc_vertices(snapshot.vertices), verifier.cseams.enc_materialization(materialization), include_line_ports]  # noqa: E731
                    verifier.memo_call("BUILD_SYMBOLIC_OVERLAY", builder, run, tail, guarded=True)
            return original(builder, snapshot, materialization, include_line_ports=include_line_ports)

        return build_symbolic_overlay

    def check_spoiled(self, builder, overlay, budget: int) -> None:
        """The discoveries and the generations over an overlay no front would make, against the native seams."""

        cseams = self.cseams
        tail = lambda: [cseams.enc_overlay(overlay, builder)]  # noqa: E731
        self.memo_call("DISCOVER_INTERIOR_CONTACTS", builder, lambda: coordinator_module.discover_interior_split_contacts(builder, overlay), tail, guarded=True)
        self.memo_call("DISCOVER_JUNCTION_CONTACTS", builder, lambda: junction_module.discover_junction_contacts(builder, overlay), tail, guarded=True)
        run = lambda: mixed_module.plan_mixed_generations(builder, overlay, coordinator_module.discover_interior_split_contacts, budget=budget)  # noqa: E731
        self.memo_call("PLAN_MIXED_GENERATIONS", builder, run, lambda: [cseams.enc_overlay(overlay, builder), budget], guarded=True)

    def check_scripted(self, builder, initial, original, budget: int) -> None:
        """`plan_mixed_generations` with the discoveries of its first rounds made up (see `native_closure_fabricate.ScriptedDiscovery`) against the native seam given the script."""

        script = fabricate.ScriptedDiscovery(self.rng, self.rng.choice((1, 2, 3)), junction_module.discover_junction_contacts, coordinator_module.discover_interior_split_contacts)

        def oracle():
            held = mixed_module.discover_junction_contacts
            mixed_module.discover_junction_contacts = script.junction
            try:
                return original(builder, initial, script.interior, budget=budget)
            finally:
                mixed_module.discover_junction_contacts = held

        cseams = self.cseams
        tail = lambda: [cseams.enc_overlay(initial, builder), budget, script.wire(cseams, self.wire.enc_str)]  # noqa: E731
        self.memo_call("PLAN_MIXED_GENERATIONS", builder, oracle, tail, guarded=True)

    def wrap_fabricating(self, original):
        """`plan_mixed_generations` itself is not checked: it is the place where a real overlay of a real closure is at hand for the contacts and the spoiled copies."""

        verifier = self

        def plan_mixed_generations(builder, initial, discover_interior, *, budget, initial_memo=None):
            if not verifier.depth and (verifier.only is None or "NORMALIZE_MIXED_GENERATION" in verifier.only):
                made = fabricate.Fabricator(initial)
                for junction, interior in made.generation_cases(verifier.rng, verifier.cases):
                    verifier.check_generation(builder, initial, junction, interior)
                for spoiled in fabricate.mutants(initial, verifier.rng, verifier.spoiled):
                    verifier.check_spoiled(builder, spoiled, min(budget, 3))
                    for junction, interior in fabricate.Fabricator(spoiled).generation_cases(verifier.rng, 1):
                        verifier.check_generation(builder, spoiled, junction, interior)
                for _ in range(verifier.scripted):
                    verifier.check_scripted(builder, initial, original, verifier.rng.choice((0, 1, 2, 4, 8)))
            return original(builder, initial, discover_interior, budget=budget, initial_memo=initial_memo)

        return plan_mixed_generations

    def wrap_apply(self, original):
        verifier = self

        def apply_component_deltas(overlay, deltas, *, collision_reason):
            if not verifier.wanted("apply_component_deltas"):
                return original(overlay, deltas, collision_reason=collision_reason)
            tail = lambda: [verifier.cseams.enc_overlay(overlay), [verifier.cseams.enc_delta(delta) for delta in deltas], verifier.wire.enc_str(collision_reason)]  # noqa: E731
            return verifier.pure_call("APPLY_COMPONENT_DELTAS", overlay.time, lambda: original(overlay, deltas, collision_reason=collision_reason), tail)

        return apply_component_deltas

    def wrap_part(self, name: str, original):
        """`with_line_ports(builder, snapshot, time)`, `build_f0_overlay(builder, snapshot, time)`, `initial_interior_contacts(snapshot)`, `overlay_signature(overlay)`."""

        verifier = self
        op = PART_OP[name]
        cseams, bseams, wire = self.cseams, self.bseams, self.wire

        def ports_text(found) -> str:
            """`with_line_ports` answers a snapshot: its vertices are what the seam answers."""

            if isinstance(found, str):
                return found
            return "None" if found is None else cseams.canon_text(found.vertices)

        if name == "initial_interior_contacts":

            def initial_interior_contacts(snapshot):
                if not verifier.wanted(name):
                    return original(snapshot)
                return verifier.pure_call("CLOSURE_PART", event_time_module.ZERO_TIME, lambda: original(snapshot), lambda: [op, bseams.enc_snapshot(snapshot)])

            return initial_interior_contacts

        if name == "overlay_signature":

            def overlay_signature(overlay):
                if not verifier.wanted(name):
                    return original(overlay)
                return verifier.pure_call("CLOSURE_PART", overlay.time, lambda: original(overlay), lambda: [op, cseams.enc_overlay(overlay)])

            return overlay_signature

        def with_builder(builder, snapshot, time_value):
            if not verifier.wanted(name):
                return original(builder, snapshot, time_value)
            tail = lambda: [op, cseams.enc_vertices(snapshot.vertices), wire.enc_time(time_value)]  # noqa: E731
            return verifier.memo_call("CLOSURE_PART", builder, lambda: original(builder, snapshot, time_value), tail, normalize=ports_text if name == "with_line_ports" else None)

        return with_builder

    def factories(self):
        found = []
        for name in sorted({item for each in self.passes for item in PASSES[each]}):
            home = HOMES[name]
            if name == "plan_symbolic_superlevel_closure":
                found.append((name, self.wrap_closure_fabricating if "fabricated" in self.passes else self.wrap_closure, home))
            elif name == "build_symbolic_overlay":
                found.append((name, self.wrap_overlay_fabricating if "fabricated" in self.passes else self.wrap_build_overlay, home))
            elif name == "discover_interior_split_contacts":
                found.append((name, self.wrap_overlay_call(name, "DISCOVER_INTERIOR_CONTACTS"), home))
            elif name == "discover_junction_contacts":
                found.append((name, self.wrap_overlay_call(name, "DISCOVER_JUNCTION_CONTACTS"), home))
            elif name == "plan_mixed_generations":
                found.append((name, self.wrap_fabricating if "fabricated" in self.passes else self.wrap_mixed, home))
            elif name == "apply_component_deltas":
                found.append((name, self.wrap_apply, home))
            elif "fabricated" in self.passes and name in ("build_f0_overlay", "with_line_ports"):
                found.append((name, (lambda original, name=name: self.wrap_part_fabricating(name, original)), home))
            else:
                found.append((name, (lambda original, name=name: self.wrap_part(name, original)), home))
        return found

    @contextlib.contextmanager
    def installed(self):
        with leaf.swapped(self.factories()):
            yield self


def native_closure_seconds(verifier: ClosureVerifier, polygon) -> dict:
    """Seconds of the native closure over every packet of a checked run of the polygon (the compute time measured inside the extension)."""

    before = {seam: Counter(row) for seam, row in verifier.timing.items()}
    with verifier.installed():
        try:
            skeleton_module.build_skeleton(polygon, work_budget=leaf.fresh_process_state())
        except Exception:  # noqa: BLE001 - a refusal of the oracle is an outcome here
            pass
    row = verifier.timing.get("PLAN_SYMBOLIC_CLOSURE", Counter())
    previous = before.get("PLAN_SYMBOLIC_CLOSURE", Counter())
    return {"native": row["compute"] - previous["compute"], "oracle_recorded": row["oracle"] - previous["oracle"]}


def oracle_closure_seconds(polygon, repeats: int) -> dict:
    """Seconds the oracle spends in `plan_symbolic_superlevel_closure` over a whole run (a stopwatch around each call, no recording), and the whole run; the least of `repeats`."""

    best: dict = {}
    for _ in range(repeats):
        totals: Counter = Counter()

        def timed(original):
            def plan_symbolic_superlevel_closure(*arguments, **keywords):
                began = time.perf_counter()
                try:
                    return original(*arguments, **keywords)
                finally:
                    totals["closure"] += time.perf_counter() - began
                    totals["calls"] += 1

            return plan_symbolic_superlevel_closure

        budget = leaf.fresh_process_state()
        started = time.perf_counter()
        with leaf.swapped([("plan_symbolic_superlevel_closure", timed, coordinator_module)]):
            try:
                skeleton_module.build_skeleton(polygon, work_budget=budget)
            except Exception:  # noqa: BLE001 - a refusal of the oracle is an outcome here
                pass
        row = {"closure": totals["closure"], "calls": totals["calls"], "whole": time.perf_counter() - started}
        best = row if not best else {key: min(best[key], row[key]) if key != "calls" else row[key] for key in row}
    return best


def gate(polygons, repeats: int = 3) -> list:
    """`[(name, oracle seconds, native seconds)]`; every checked call of the runs was also compared (a mismatch raises)."""

    verifier = ClosureVerifier(leaf.Sampling(head=10**9, stride=1, cap=10**9), passes=("closure",))
    rows = []
    for name, polygon in polygons:
        expected = oracle_closure_seconds(polygon, repeats)
        found = native_closure_seconds(verifier, polygon)
        assert not verifier.mismatches, [str(item)[:500] for item in verifier.mismatches[:2]]
        rows.append((name, expected, found))
    return rows


def format_gate(rows) -> str:
    lines = [f"{'polygon':58} {'calls':>6} {'oracle ms':>10} {'native ms':>10} {'speed-up':>9}"]
    for name, expected, found in rows:
        native = found["native"]
        lines.append(f"{name[-58:]:58} {int(expected['calls']):6d} {1e3 * expected['closure']:10.2f} {1e3 * native:10.2f} {expected['closure'] / native if native else float('inf'):9.1f}")
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
