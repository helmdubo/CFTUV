"""Python side of the skeleton differential seams (test-only): the wire of `native/cftuv-skeleton/src/seam.rs`.

One native entry runs ONE seam (a function of the exact event layer: `compare_times`, `concurrency_time`, `evaluate_split_candidate`, ...) on the
arguments the oracle saw and answers the cost answer `[outcome, counts, articles, log, state, nanoseconds, extras...]` of `cftuv_native.cost`. This module
encodes oracle objects (lines, times, points, the VIEW a candidate call read, the memory entries it hit) into the wire, decodes native answers into the
oracle's own types (`EventTimeV1`, `EventPointV1`, `SplitCandidateDecisionV1`, ...) and turns a refused outcome into the `(exception class name, text)`
the oracle's exception has. It imports no extension: the runner comes from the shim (`cftuv_native.skeleton_seam_run`).

THE VIEW. `evaluate_split_candidate` reads three callbacks of an `ExactCandidateViewV1`. `ViewRecorder` wraps them for ONE call, answers exactly as the real
ones and writes down every answer it gave (vertex states, span states, the trace of a vertex), so the native seam can answer from that snapshot. The
identity of the objects matters (the oracle's time memos are keyed by `id()` of the line and of the sliding projection and which lookups hit decides
the sign counters), so every line travels with its `id()` and a sliding projection with the `id()` of the object the call used.

THE MEMORY. `LoggingEntries` replaces the `entries` dict of a `PositionMemoV1` and writes down which keys a call HIT (and the value it got) and which
it inserted; the hits that were not inserted by the call itself are the memory the call started with, as far as the call can tell (`memo_wire`).
"""

from __future__ import annotations

import dataclasses
import itertools
from dataclasses import dataclass
from enum import Enum
from fractions import Fraction
from types import SimpleNamespace

from . import clip_seams, codec, cost, pin
from .clip_seams import Answer, SeamUnsupported, answer_of, dec_str, enc_str, full_header

__all__ = (
    "Answer",
    "CallRecorder",
    "LoggingEntries",
    "OPCODES",
    "SEAMS",
    "SeamRunner",
    "SeamUnsupported",
    "dec_time",
    "enc_graph",
    "enc_polygon",
    "enc_repr",
    "exception_of",
    "full_header",
)

#: Native opcodes (`cftuv_skeleton::seam::SEAMS`); a test compares this table with the extension's own.
SEAMS = (
    (200, "PY_REPR"),
    (201, "SUPPORT_LINE"),
    (202, "COMPARE_TIMES"),
    (203, "TIMES_ARE_EQUAL"),
    (204, "TIME_NORMALIZED"),
    (205, "TIME_CANONICAL"),
    (206, "CONCURRENCY_TIME"),
    (207, "SLIDING_TIME"),
    (208, "SLIDING_POINT"),
    (209, "EVENT_POINT"),
    (210, "QUEUE_SCRIPT"),
    (220, "EVALUATE_SPLIT_CANDIDATE"),
    (221, "POSITION"),
    (222, "SPAN_CONTAINMENT"),
    (230, "CELL_GRID"),
    (231, "BUILD_MOTORCYCLE_GRAPH"),
    (232, "TRACE_FOR"),
    (233, "TRACE_INDEX_SCRIPT"),
    (234, "MOTORCYCLE_PART"),
    (240, "EVALUATE_EDGE_CANDIDATE"),
    (241, "EDGE_EVENT_TIME"),
    (242, "IS_FUTURE"),
    (243, "COLLAPSING_SPAN"),
    (244, "SPAN_END"),
    (245, "SLIDING_PROJECTION"),
    (246, "CLASSIFY_POSTSTATE_SPAN"),
    (250, "PROOF_SCRIPT"),
    (251, "BUILDER_INIT"),
    (252, "BUILDER_RUN"),
    (253, "COLLECT_SNAPSHOT"),
    (254, "PLAN_COMPONENTS"),
    (255, "PLAN_SPLIT_MATERIALIZATION"),
    (300, "BUILDER_PRIMITIVE"),
    (301, "PYSET_SCRIPT"),
    (302, "SET_ORDER_NONE_LAST"),
)
OPCODES = {name: code for code, name in SEAMS}

STATUS_VALUE = 10
STATUS_UNSUPPORTED = 12
STATUS_ZERO_DIVISOR_TIME = 14
STATUS_PARALLEL_LINES = 15
STATUS_DEGENERATE_EDGE = 16
STATUS_NEGATIVE_SPEED = 17
STATUS_CELL_GRID_REJECTED = 18

#: The oracle's own exceptions among the native statuses (the others are refusals of the port: `12`, `5`, `6`, `7`).
ORACLE_STATUSES = frozenset({0, 1, 2, 3, 4, STATUS_VALUE, STATUS_ZERO_DIVISOR_TIME, STATUS_PARALLEL_LINES, STATUS_DEGENERATE_EDGE, STATUS_NEGATIVE_SPEED, STATUS_CELL_GRID_REJECTED})

#: The oracle files the leaf seams mirror (relative to the `cftuv_envelope` package). The leaf is a test-only slice of the future whole-operation port, so it has
#: no `OPERATION_FILES` entry (that table is the whole operations'); the test module skips with a named reason when one of these files moved.
LEAF_FILES = (
    "wavefront/event_time.py",
    "wavefront/events.py",
    "wavefront/candidate_law.py",
    "wavefront/candidate_refusal.py",
    "wavefront/exact_candidate_view.py",
    "wavefront/cell_grid.py",
    "wavefront/motorcycle.py",
    "wavefront/poststate_span.py",
    "wavefront/proof.py",
    "wavefront/polygon.py",
    "robust/predicates.py",
    "wavefront/skeleton.py",
    "wavefront/superlevel.py",
    "wavefront/superlevel_snapshot.py",
    "wavefront/superlevel_germ.py",
    "wavefront/superlevel_closure.py",
    "wavefront/symbolic_initial_composition.py",
    "wavefront/exact_identity.py",
    "wavefront/digest.py",
    *pin.FOUNDATION,
)

#: `{file: sha256}` of the files of `LEAF_FILES` the whole-operation pins do not hold (`pin.PINS` has the rest).
LEAF_PINS = {
    "wavefront/candidate_law.py": "e8663e8c15b1703404b73a8f62138ef42a1658d9b8effc6fd24f8d4ee4b8c069",
    "wavefront/candidate_refusal.py": "b1ca2217abbfe87286ef49ca1bb17c5121e1889b94700bcbeed9cf4e2aada787",
    "wavefront/exact_candidate_view.py": "dd53dbd3e3736966fe706d02842cf2729df5e2fd6fbb37413573613c525d87e0",
    "wavefront/events.py": "e69a1328c2aac9001f7d0830a6fe5d644d61195e1fb063058584717da288067f",
    "wavefront/cell_grid.py": "e2ef50ff61222345e338ca9836f4281350007039daf40292f383ecba8f60030a",
    "wavefront/motorcycle.py": "4460f277c899186319b405187025b60692187a33d95b14d80d75d907e244866f",
    "wavefront/poststate_span.py": "2f60c4689cd442e88c3fe68e28fd5feb96c4bcc304202c87e7c0189d42022886",
    "wavefront/proof.py": "093e9b8de7b8e88687f0ce24d184cb434b608ebab2161b9c4e2a8721fa465de0",
    "wavefront/polygon.py": "d3fe537c884581844cbb62f626af79e6c085332ee53ba647100109e40c84c408",
    "robust/predicates.py": "913a134bb932fa6579072fa82549d086ad975c3b552e0a7d18953526ffe75ff8",
    "wavefront/skeleton.py": "437f35e1faca6897235390ad9cc267d4086312198412148ec976dd0e7757a2ba",
    "wavefront/superlevel.py": "26061fdbfeed2e0aab553cdf9d6fad4c284c1c2ec5caa3a7d14058971d2ac68d",
    "wavefront/superlevel_snapshot.py": "8a9b948a0b91074b56b5bc780e38a22c33d9c234b21081df4b013f866ce2b38d",
    "wavefront/superlevel_germ.py": "7063de9c4497322fe1282b3c511a266fe9c20d361be9f35b5febd869778aee02",
    "wavefront/superlevel_closure.py": "8d4db91b5635cbb277e3cd3f31381756e395f830ed5bdfc80ae620f5c24f9352",
    "wavefront/symbolic_initial_composition.py": "9b83f05bc7dabd2e980a7574bd42be1558ece4523bdd30c8b64e440e13231f5e",
    "wavefront/exact_identity.py": "e306c46e645d9137ac0ec2edccb01aa3be3809ebff24996ce8c8770875663d47",
    "wavefront/digest.py": "d0b56d73e0da58d196e72541208d7bfd3c91d5d46449573076184ec4b782432f",
}


def stale_leaf_files(root=None) -> tuple:
    """The files of `LEAF_FILES` whose digest differs from the pin (a gone file is named `<file> (missing)`)."""

    root = pin.kernel_root() if root is None else root
    pins = {**pin.PINS, **LEAF_PINS}
    stale = []
    for name in LEAF_FILES:
        found = pin.digest(root, name)
        if found is None or found != pins.get(name):
            stale.append(f"{name} (missing)" if found is None else name)
    return tuple(stale)


def main(argv=None) -> int:
    """`python -m cftuv_native.skeleton_seams [kernel/src/cftuv_envelope]`: prints the `LEAF_PINS` literal of the tree (what to paste after a catch-up) and the verdict against the current one."""

    import sys
    from pathlib import Path

    arguments = list(sys.argv[1:] if argv is None else argv)
    root = Path(arguments[0]) if arguments else pin.kernel_root()
    print("LEAF_PINS = {")
    for name in LEAF_PINS:
        print(f'    "{name}": "{pin.digest(root, name)}",')
    print("}")
    stale = stale_leaf_files(root)
    print("# the leaf matches its pins" if not stale else "# the leaf differs from its pins: " + ", ".join(stale))
    return 0


#: `|a|, |b| < LINE_LIMIT` and `|c| < OFFSET_LIMIT` (`native/cftuv-skeleton/src/line.rs`).
LINE_LIMIT = 1 << 62
OFFSET_LIMIT = 1 << 124

ZERO_DIVISOR_TIME_TEXT = "знаменатель времени доказанно нулевой"
PARALLEL_LINES_TEXT = "прямые параллельны, точки пересечения нет"

OUTCOME_CODES = {"EXACT": 0, "WAVEFRONT_TRIPLE_NEVER_CONCURRENT": 1, "WAVEFRONT_TRIPLE_ALWAYS_CONCURRENT": 2}


def _kernel():
    from cftuv_envelope.wavefront import candidate_law, event_time, exact_candidate_view, exact_identity

    return event_time, candidate_law, exact_candidate_view, exact_identity


def _motorcycle():
    from cftuv_envelope.wavefront import cell_grid, motorcycle

    return motorcycle, cell_grid


# --------------------------------------------------------------------------
# encoders
# --------------------------------------------------------------------------


def enc_line(line, ident=None) -> list:
    """`[a, b, c, q, ident]` of a `SupportLineV1`; `ident` is the identity of the object (default `id(line)`)."""

    for name, limit in (("a", LINE_LIMIT), ("b", LINE_LIMIT), ("c", OFFSET_LIMIT)):
        value = getattr(line, name)
        if type(value) is not int or not -limit < value < limit:
            raise SeamUnsupported(f"a support line coefficient {name} = {value!r} is not a machine integer of the port")
    if type(line.q) not in (int, Fraction):
        raise SeamUnsupported(f"a line speed must be an int or a Fraction, not {type(line.q).__name__}")
    return [line.a, line.b, line.c, line.q, id(line) if ident is None else ident]


def enc_time(time) -> list:
    if type(time.dividend) is not Fraction:
        raise SeamUnsupported(f"a time dividend must be a Fraction, not {type(time.dividend).__name__}")
    return [time.dividend, time.divisor]


def enc_point(point) -> list:
    return [point.x, point.y]


def _enc_optional(value, encode):
    return None if value is None else encode(value)


def enc_repr(value) -> list:
    """The repr node `[tag, payload]` of an oracle value: scalars, tuples, lists, dataclass instances (class `__qualname__`, fields with `repr=True`),
    enum members (`<Class.NAME: value>`), `SqrtSumV1`, `EventTimeV1`, `EventPointV1`."""

    event_time, *_ = _kernel()
    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1

    kind = type(value)
    if value is None:
        return [0, None]
    if kind is bool:
        return [1, value]
    if kind is int:
        return [2, value]
    if kind is str:
        return [3, enc_str(value)]
    if kind is Fraction:
        return [4, value]
    if isinstance(value, SqrtSumV1):
        return [9, value]
    if isinstance(value, event_time.EventTimeV1):
        return [10, enc_time(value)]
    if isinstance(value, event_time.EventPointV1):
        return [11, enc_point(value)]
    if isinstance(value, tuple):
        return [5, [enc_repr(item) for item in value]]
    if kind is list:
        return [6, [enc_repr(item) for item in value]]
    if isinstance(value, Enum):
        return [8, [enc_str(kind.__name__), enc_str(value.name), enc_repr(value.value)]]
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        fields = [[enc_str(item.name), enc_repr(getattr(value, item.name))] for item in dataclasses.fields(value) if item.repr]
        return [7, [enc_str(kind.__qualname__), fields]]
    raise SeamUnsupported(f"{kind.__qualname__} has no repr node")


# --------------------------------------------------------------------------
# decoders
# --------------------------------------------------------------------------


def dec_time(wire):
    event_time, *_ = _kernel()
    return event_time.EventTimeV1(wire[0], wire[1])


def dec_point(wire):
    event_time, *_ = _kernel()
    return event_time.EventPointV1(wire[0], wire[1])


def dec_outcome(code: int):
    event_time, *_ = _kernel()
    return (event_time.EventTimeOutcome.EXACT, event_time.EventTimeOutcome.WAVEFRONT_TRIPLE_NEVER_CONCURRENT, event_time.EventTimeOutcome.WAVEFRONT_TRIPLE_ALWAYS_CONCURRENT)[code]


def dec_time_entry(wire) -> tuple:
    """`(EventTimeV1 | None, EventTimeOutcome)` of `concurrency_time` / `sliding_time`."""

    return (None if wire[0] is None else dec_time(wire[0])), dec_outcome(wire[1])


def dec_decision(wire, now, identity_factory=None):
    """The `SplitCandidateDecisionV1` of the answer; `identity_factory()` is asked for an effect that needs a proof identity, as the oracle's `refuse` does."""

    _event_time, candidate_law, *_ = _kernel()
    from cftuv_envelope.wavefront.candidate_refusal import CandidateRefusal

    candidate_wire, effects_wire, _memo = wire
    candidate = None
    if candidate_wire is not None:
        time, point, at_start, at_end = candidate_wire
        candidate = candidate_law.SplitCandidateV1(dec_time(time), dec_point(point), at_start, at_end)
    effects = []
    for reason, needs_identity, deltas in effects_wire:
        identity = identity_factory() if needs_identity and identity_factory is not None else None
        effects.append(candidate_law.CandidateRefusalEffectV1(CandidateRefusal(dec_str(reason)), identity, now, tuple((dec_str(name), delta) for name, delta in deltas)))
    return candidate_law.SplitCandidateDecisionV1(candidate, tuple(effects))


def dec_containment(wire):
    _event_time, _law, view, _identity = _kernel()
    return view.SpanContainmentV1(wire[0], wire[1], wire[2])


DECODERS = {
    "PY_REPR": dec_str,
    "SUPPORT_LINE": lambda wire: tuple(wire),
    "COMPARE_TIMES": lambda wire: wire,
    "TIMES_ARE_EQUAL": lambda wire: wire,
    "TIME_NORMALIZED": dec_time,
    "TIME_CANONICAL": dec_time,
    "CONCURRENCY_TIME": dec_time_entry,
    "SLIDING_TIME": dec_time_entry,
    "SLIDING_POINT": dec_point,
    "EVENT_POINT": dec_point,
    "BUILDER_INIT": lambda wire: wire,
    "BUILDER_RUN": lambda wire: wire,
    "COLLECT_SNAPSHOT": lambda wire: (dec_str(wire[0]), tuple(wire[1])),
    "PLAN_COMPONENTS": dec_str,
    "PLAN_SPLIT_MATERIALIZATION": dec_str,
    "BUILDER_PRIMITIVE": lambda wire: wire,
    "PYSET_SCRIPT": lambda wire: wire,
    "SET_ORDER_NONE_LAST": lambda wire: wire,
}


# --------------------------------------------------------------------------
# the call
# --------------------------------------------------------------------------


def request_bytes(name: str, arguments, header=None) -> bytes:
    """One seam request: `[header, opcode, arguments]` in the boundary format (`header` is the cost header or `None`)."""

    return codec.encode_value([header, OPCODES[name], list(arguments)])


class SeamRunner:
    """Runs seams on one native session. A call with a full header is self-contained; a call with an incremental sync (`cost.CostMirror._sync_in`) continues
    from what the session mirrors."""

    def __init__(self, session=None) -> None:
        import cftuv_native

        self._native = cftuv_native
        self.session = cftuv_native.new_skeleton_seam_session() if session is None else session

    def call(self, name: str, arguments, header=None) -> Answer:
        return answer_of(self._native.skeleton_seam_run(self.session, request_bytes(name, arguments, header)))

    def value(self, name: str, arguments, header=None):
        """The decoded value of a successful call (a refused call raises `RuntimeError` naming the status)."""

        answer = self.call(name, arguments, header)
        if not answer.ok:
            raise RuntimeError(f"seam {name} refused: status {answer.status} {answer.detail!r}")
        return DECODERS[name](answer.value)


def exception_of(answer: Answer, budget) -> tuple:
    """`(exception class name, text)` the oracle's exception has for a refused answer (`budget` only for its identity fields: the articles are the native ones)."""

    status = answer.status
    if status == cost.STATUS_NEGATIVE_RADICAND:
        return ("NegativeRadicandError", f"под корнем {Fraction(answer.detail[0], answer.detail[1])}")
    if status == STATUS_ZERO_DIVISOR_TIME:
        return ("ZeroDivisorTimeError", ZERO_DIVISOR_TIME_TEXT)
    if status == STATUS_PARALLEL_LINES:
        return ("ParallelSupportLinesError", PARALLEL_LINES_TEXT)
    if status == STATUS_DEGENERATE_EDGE:
        return ("DegenerateEdgeError", dec_str(answer.detail[0]))
    if status == STATUS_NEGATIVE_SPEED:
        return ("NegativeSpeedError", dec_str(answer.detail[0]))
    if status == STATUS_CELL_GRID_REJECTED:
        return ("CellGridRejected", dec_str(answer.detail[0]))
    return clip_seams.exception_of(answer, budget)


# --------------------------------------------------------------------------
# the memory a call hit
# --------------------------------------------------------------------------

_MISSING = object()


class CallLog:
    """What one call did to the memory: the keys it hit that it had not inserted itself (in order, once), and the keys it inserted."""

    __slots__ = ("hits", "puts", "seen")

    def __init__(self) -> None:
        self.hits: list = []
        self.puts: list = []
        self.seen: set = set()


class LoggingEntries(dict):
    """The `entries` dict of a `PositionMemoV1`, watched. Behaves as a dict; while `log` is set every read that hits and every write is noted."""

    def __init__(self, source=()) -> None:
        super().__init__(source)
        self.log: CallLog | None = None

    def get(self, key, default=None):
        value = super().get(key, _MISSING)
        log = self.log
        if log is not None and value is not _MISSING and key not in log.seen:
            log.seen.add(key)
            log.hits.append((key, value))
        return default if value is _MISSING else value

    def __setitem__(self, key, value) -> None:
        log = self.log
        if log is not None:
            log.puts.append(key)
            log.seen.add(key)
        super().__setitem__(key, value)


def memo_wire(memo, log: CallLog | None, active: bool) -> list:
    """`[active, entries]`: the entries of the memory the call hit and had not made itself (see the module note)."""

    event_time, *_ = _kernel()
    entries: list = []
    if active and log is not None:
        for key, value in log.hits:
            if key[0] == "CONCURRENCY" or key[0] == "SLIDING":
                kind = 1 if key[0] == "CONCURRENCY" else 2
                time, outcome = value[3]
                entries.append([kind, key[1], key[2], key[3], _enc_optional(time, enc_time), OUTCOME_CODES[outcome.value]])
            else:
                first, second, sliding, time = key
                entries.append([0, enc_line(first), enc_line(second), sliding, enc_time(time), _enc_optional(value, enc_point)])
    return [bool(active), entries]


def placed_in(log: CallLog | None) -> tuple:
    """`(places inserted, times inserted)` by the call (the growth of the memory the native seam reports)."""

    if log is None:
        return (0, 0)
    times = sum(1 for key in log.puts if key[0] in ("CONCURRENCY", "SLIDING"))
    return (len(log.puts) - times, times)


# --------------------------------------------------------------------------
# the view a call read
# --------------------------------------------------------------------------


class _Refs:
    """Dense numbers for the references of one namespace (references are ints in the runtime view, dataclasses in the symbolic one)."""

    def __init__(self) -> None:
        self.numbers: dict = {}

    def __call__(self, ref) -> int:
        found = self.numbers.get(ref)
        if found is None:
            found = self.numbers[ref] = len(self.numbers)
        return found


class CallRecorder:
    """Wraps the view of ONE call. `view` is the wrapped view to hand to the oracle; afterwards `view_wire()` is what the callbacks answered.

    `probe` is a one-element list the caller keeps patched into `TraceV1.bounds_time` (the trace is reached only through that method): the wrapper there
    writes `[crash_time]` into it, and `trace_bounds` reads it to learn whether the vertex has a trace and when it crashes.
    """

    def __init__(self, view, probe: list) -> None:
        self.original = view
        self.probe = probe
        self.vertex_ids, self.span_ids = _Refs(), _Refs()
        self.vertices: dict = {}
        self.spans: dict = {}
        self.traces: dict = {}
        self.view = dataclasses.replace(view, vertex_state=self._vertex_state, span_state=self._span_state, trace_bounds=self._trace_bounds)

    def _vertex_state(self, ref):
        state = self.original.vertex_state(ref)
        number = self.vertex_ids(ref)
        if number not in self.vertices:
            sliding = None if state.sliding is None else [state.sliding, id(state.sliding)]
            self.vertices[number] = [number, self.span_ids(state.prev_span), self.span_ids(state.next_span), enc_time(state.birth), sliding]
        return state

    def _span_state(self, ref):
        state = self.original.span_state(ref)
        number = self.span_ids(ref)
        if number not in self.spans:
            for item in state.source_span:
                if type(item) is not int or not -LINE_LIMIT < item < LINE_LIMIT:
                    raise SeamUnsupported(f"a source span node {item!r} is not a machine integer of the port")
            self.spans[number] = [
                number,
                enc_line(state.line),
                list(state.source_span),
                _enc_optional(state.start_vertex, self.vertex_ids),
                _enc_optional(state.end_vertex, self.vertex_ids),
                _enc_optional(state.frozen_instant, enc_time),
                _enc_optional(state.frozen_start, enc_point),
                _enc_optional(state.frozen_end, enc_point),
                span_occurrence(ref),
            ]
        return state

    def _trace_bounds(self, ref, time):
        self.probe[:] = []
        answer = self.original.trace_bounds(ref, time)
        if self.probe:
            self.traces[self.vertex_ids(ref)] = _enc_optional(self.probe[0], enc_time)
        elif answer is not None:
            raise SeamUnsupported("a trace bound answered without the trace the recorder can see")
        return answer

    def view_wire(self) -> list:
        return [[int(prime) for prime in self.original.prime_universe], list(self.vertices.values()), list(self.spans.values()), [[number, crash] for number, crash in self.traces.items()]]


# --------------------------------------------------------------------------
# the span occurrence, polygons, traces, the graph (WP-S1) and the ledger (WP-S2b)
# --------------------------------------------------------------------------


def span_occurrence(ref):
    """`[start_x, start_y, end_x, end_y]` of `getattr(ref, "occurrence", None)` when it has three parts (what `_span_orientation` reads), else `None`."""

    occurrence = getattr(ref, "occurrence", None)
    if occurrence is None or len(occurrence) != 3:
        return None
    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1

    try:
        start_x, start_y = (SqrtSumV1(item) for item in occurrence[1])
        end_x, end_y = (SqrtSumV1(item) for item in occurrence[2])
    except (TypeError, ValueError):
        return None
    return [start_x, start_y, end_x, end_y]


#: `|x|, |y| < COORDINATE_LIMIT` of a polygon point (`native/cftuv-skeleton/src/polygon.rs`).
COORDINATE_LIMIT = 1 << 60


def _checked_int(value, limit: int, what: str) -> int:
    if type(value) is not int or not -limit < value < limit:
        raise SeamUnsupported(f"{what} {value!r} is not a machine integer of the port")
    return value


def _speed(value, what: str):
    if type(value) not in (int, Fraction):
        raise SeamUnsupported(f"{what} must be an int or a Fraction, not {type(value).__name__}")
    return value


def enc_polygon(polygon) -> list:
    """`[loops, fans]`: a loop is `[x0, y0, x1, y1, ..., speeds]` split in two lists (`points`, the speeds with the default spelt out), a fan `[x, y, [[nx, ny, q], ...]]`."""

    loops = []
    for loop in polygon.loops:
        flat = [_checked_int(value, COORDINATE_LIMIT, "a polygon coordinate") for point in loop.points for value in point]
        loops.append([flat, [_speed(speed, "an edge speed") for speed in loop.edge_speeds_squared]])
    fans = []
    for fan in polygon.vertex_fans:
        point = [_checked_int(value, COORDINATE_LIMIT, "a fan vertex coordinate") for value in fan.point]
        supports = [[_checked_int(item.normal_x, LINE_LIMIT, "a fan normal"), _checked_int(item.normal_y, LINE_LIMIT, "a fan normal"), _speed(item.speed_squared, "a fan speed")] for item in fan.supports]
        fans.append([*point, supports])
    return [loops, fans]


def enc_line4(line) -> list:
    """A line without its identity: `[a, b, c, q]` (the native side gives it identity zero)."""

    return enc_line(line, 0)[:4]


def enc_wall(wall) -> list:
    return [wall.ident, wall.start[0], wall.start[1], wall.end[0], wall.end[1], enc_line4(wall.line)]


def enc_trace(trace) -> list:
    motorcycle, _grid = _motorcycle()
    outcomes = list(motorcycle.TraceOutcome)
    kinds = list(motorcycle.CrashKind)
    reach = trace.reach
    if reach is not None and type(reach) is not Fraction:
        raise SeamUnsupported(f"a trace reach must be a Fraction, not {type(reach).__name__}")
    return [
        _checked_int(trace.ident, 1 << 62, "a trace identity"),
        outcomes.index(trace.outcome),
        enc_line4(trace.left_line),
        enc_line4(trace.right_line),
        enc_time(trace.start_time),
        enc_point(trace.origin),
        [trace.velocity[0], trace.velocity[1]],
        _enc_optional(trace.crash_time, enc_time),
        _enc_optional(trace.crash_point, enc_point),
        kinds.index(trace.crash_kind),
        _checked_int(trace.crash_target, 1 << 62, "a crash target"),
        reach,
    ]


def enc_grid(grid) -> list:
    return [_checked_int(value, 1 << 61, "a grid number") for value in (grid.x_min, grid.y_min, grid.x_max, grid.y_max, grid.cell)]


def enc_graph(graph, *, traces=True, counters=None, next_ident=None) -> list:
    """`[walls, grid, buckets, traces, counters, next identity]` of a `MotorcycleGraphV1`; the counters and the next identity may be given (the state BEFORE a call)."""

    buckets = [[cell[0], cell[1], list(idents)] for cell, idents in graph.wall_index.buckets.items()]
    counters = graph.counters if counters is None else counters
    if next_ident is None:
        next_ident = count_value(graph._next_ident)
    names = _counter_names()
    return [
        [enc_wall(wall) for wall in graph.walls],
        enc_grid(graph.grid),
        buckets,
        [enc_trace(trace) for trace in graph.traces.values()] if traces else [],
        [counters[name] for name in names],
        next_ident,
    ]


def count_value(counter) -> int:
    """The next value an `itertools.count` gives, read from its `repr` (`count(5)`) without consuming it (`copy` of an iterator is deprecated)."""

    return int(repr(counter).split("(", 1)[1].rstrip(")").split(",", 1)[0])


def _counter_names() -> tuple:
    return (
        "motorcycle_traces",
        "motorcycle_wall_tests",
        "motorcycle_march_steps",
        "motorcycle_wall_crashes",
        "motorcycle_trace_pairs",
        "motorcycle_trace_crashes",
        "motorcycle_unbounded_traces",
        "resting_steiner_vertices",
    )


def dec_line4(wire):
    event_time, *_ = _kernel()
    return event_time.SupportLineV1(wire[0], wire[1], wire[2], wire[3])


def dec_trace(wire):
    motorcycle, _grid = _motorcycle()
    ident, outcome, left, right, start, origin, velocity, crash_time, crash_point, kind, target, reach = wire
    return motorcycle.TraceV1(
        ident,
        list(motorcycle.TraceOutcome)[outcome],
        dec_line4(left),
        dec_line4(right),
        dec_time(start),
        dec_point(origin),
        (velocity[0], velocity[1]),
        None if crash_time is None else dec_time(crash_time),
        None if crash_point is None else dec_point(crash_point),
        list(motorcycle.CrashKind)[kind],
        target,
        reach,
    )


def dec_wall(wire):
    motorcycle, _grid = _motorcycle()
    ident, sx, sy, ex, ey, line = wire
    return motorcycle.WallV1(ident, (sx, sy), (ex, ey), dec_line4(line))


def dec_cells(wire) -> tuple:
    return tuple((cell[0], cell[1]) for cell in wire)


def dec_graph(wire):
    """A stand-in for a `MotorcycleGraphV1` built from the native answer: the same attributes `graph_view` reads."""

    _motorcycle_module, cell_grid = _motorcycle()
    walls, grid, buckets, traces, counters, next_ident = wire
    decoded = [dec_trace(trace) for trace in traces]
    return SimpleNamespace(
        walls=tuple(dec_wall(wall) for wall in walls),
        grid=cell_grid.CellGridV1(*grid),
        wall_index=SimpleNamespace(buckets={(column, row): list(idents) for column, row, idents in buckets}),
        traces={trace.ident: trace for trace in decoded},
        counters=dict(zip(_counter_names(), counters)),
        _next_ident=itertools.count(next_ident),
    )


def graph_view(graph) -> tuple:
    """The comparable shape of a graph (the oracle's or a decoded one): walls, grid, the buckets in order, the traces in order, the counters in order, the next identity."""

    return (
        tuple(graph.walls),
        graph.grid,
        tuple((cell, tuple(idents)) for cell, idents in graph.wall_index.buckets.items()),
        tuple(graph.traces.items()),
        tuple(graph.counters.items()),
        repr(graph._next_ident),
    )


def dec_trace_for(wire):
    """`(trace, counters, next identity)` of a `trace_for` answer, in the shape `trace_for_view` gives the oracle's."""

    trace, counters, next_ident = wire
    return dec_trace(trace), tuple(zip(_counter_names(), counters)), repr(itertools.count(next_ident))


def trace_for_view(trace, graph) -> tuple:
    return trace, tuple(graph.counters.items()), repr(graph._next_ident)


def dec_cell_grid(name_or_op, wire):
    """The grid seam's answers as Python values (`op` as in `native/cftuv-skeleton/src/seam_graph.rs::cell_grid`)."""

    _motorcycle_module, cell_grid = _motorcycle()
    if name_or_op == 0:
        return cell_grid.CellGridV1(*wire)
    if name_or_op in (3, 5, 6):
        return dec_cells(wire)
    if name_or_op == 7:
        results, buckets = wire
        return [None if item is None else tuple(item) for item in results], tuple(((column, row), tuple(idents)) for column, row, idents in buckets)
    return tuple(wire) if isinstance(wire, list) else wire


def dec_edge_decision(wire, now, identity_factory=None):
    """The `EdgeCandidateDecisionV1` of the answer (the effects as in `dec_decision`)."""

    _event_time, candidate_law, *_ = _kernel()
    from cftuv_envelope.wavefront.candidate_refusal import CandidateRefusal

    candidate_wire, effects_wire, _memo = wire
    candidate = None
    if candidate_wire is not None:
        time, point, span_unproven = candidate_wire
        candidate = candidate_law.EdgeCandidateV1(dec_time(time), dec_point(point), span_unproven)
    effects = []
    for reason, needs_identity, deltas in effects_wire:
        identity = identity_factory() if needs_identity and identity_factory is not None else None
        effects.append(candidate_law.CandidateRefusalEffectV1(CandidateRefusal(dec_str(reason)), identity, now, tuple((dec_str(name), delta) for name, delta in deltas)))
    return candidate_law.EdgeCandidateDecisionV1(candidate, tuple(effects))


def dec_poststate(wire):
    """`PoststateSpanClassificationV1` of the answer."""

    from cftuv_envelope.wavefront import poststate_span

    index, birth_length, slope, orientation_sign, _memo = wire
    return poststate_span.PoststateSpanClassificationV1(list(poststate_span.PoststateSpanDisposition)[index], birth_length, slope, orientation_sign)


def dec_obligation(wire):
    from cftuv_envelope.wavefront import proof
    from cftuv_envelope.wavefront.candidate_refusal import CandidateRefusal
    from cftuv_envelope.wavefront.events import EventKind

    kind, cause, disposition, vertices, participants, targets, level, event_kind = wire
    name = dec_str(cause)
    return proof.ProofObligationV1(
        cause=CandidateRefusal(name) if kind == 0 else proof.ProofObligationBranch(name),
        disposition=proof.ProofObligationDisposition(dec_str(disposition)),
        vertex_ids=tuple(vertices),
        participant_edge_keys=tuple(tuple(key) for key in participants),
        target_edge_keys=tuple(tuple(key) for key in targets),
        level=dec_time(level),
        event_kind=None if event_kind is None else EventKind(dec_str(event_kind)),
    )


def enc_obligation_record(cause, disposition, vertex_ids, participants, targets, level, event_kind) -> list:
    from cftuv_envelope.wavefront import proof

    kind = 1 if isinstance(cause, proof.ProofObligationBranch) else 0
    for ids in (vertex_ids, *participants, *targets):
        for value in ids:
            _checked_int(value, 1 << 62, "an obligation number")
    return [
        0,
        kind,
        enc_str(cause.value),
        enc_str(disposition.value),
        list(vertex_ids),
        [list(key) for key in participants],
        [list(key) for key in targets],
        enc_time(level),
        None if event_kind is None else enc_str(event_kind.value),
    ]


def enc_refusal_record(reason, vertex_ids, participants, targets, level) -> list:
    for ids in (vertex_ids, *participants, *targets):
        for value in ids:
            _checked_int(value, 1 << 62, "an obligation number")
    return [1, enc_str(reason.value), list(vertex_ids), [list(key) for key in participants], [list(key) for key in targets], enc_time(level)]


if __name__ == "__main__":
    raise SystemExit(main())
