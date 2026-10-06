"""Python side of the clip differential seams (test-only): the wire of `native/cftuv-clip/src/seam.rs`.

One native entry runs ONE seam (a function of the first third of the `clip_geometry` port) on the arguments the
oracle saw and answers the cost answer `[outcome, counts, articles, log, state]` of `cftuv_native.cost`. This module
encodes oracle objects (triangles, cells, points, memo) into the wire, decodes the native answer into the oracle's own
types (`ClipCellV1`, `CellPlanV1`, `CornerSnapV1`, `LocalPoint3V1`, `SqrtSumV1`, `Fraction`, tuples) and turns a refused
outcome into the `(exception class name, text)` the oracle's exception has. It imports no extension: the runner comes
from the shim (`cftuv_native.clip_seam_run`).

Strings travel as `[None, int]` (UTF-8 bytes plus a final 0x01 sentinel, little endian) because the codec has no tag for
them. Chart and corner numbers must be `Fraction`s: that is what `SurfaceLiftV1.from_triangles` builds, and the seam
refuses (`SeamUnsupported`) anything else instead of guessing what an `int` would have done in the oracle.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from fractions import Fraction

from . import codec, cost

#: Native opcodes (`cftuv_clip::seam::SEAMS`); a test compares this table with the extension's own.
SEAMS = (
    (100, "SORT_SEQUENCE"),
    (101, "FLOAT_SUM"),
    (102, "LINE_VALUE"),
    (103, "WINDOW"),
    (104, "VALUES_IN"),
    (105, "STRETCH_SQUARE"),
    (106, "LIFT_KNOWN"),
    (107, "NANOMETRES"),
    (108, "MILLI_CELLS"),
    (109, "ORIENTATION"),
    (110, "SHOELACE_SIGN"),
    (111, "DOUBLED_SHOELACE"),
    (112, "WITHIN_EDGE_GAP"),
    (113, "HINGE_DEPTH_SQUARE"),
    (114, "CHORD_OF"),
    (115, "EDGE_CONSTANTS"),
    (116, "CHEAP_SIGN"),
    (117, "RATIONAL_PAIR"),
    (118, "BUILD_CELLS"),
    (119, "SNAP_SOURCE_VERTICES"),
    (120, "TRIANGULATE_EXACT"),
    (121, "CONVEX_QUAD_RING"),
    (122, "HAS_RIGHT_TURN"),
    (123, "ORDERED"),
    (124, "CLIP_GEOMETRY"),
)
OPCODES = {name: code for code, name in SEAMS}

#: The outcome codes of the clip stage's own refusals: one table with the drop-in's (`cost`), so the two wires cannot drift apart.
STATUS_OVERFLOW = cost.CLIP_STATUS_OVERFLOW
STATUS_ZERO_DIVISION = cost.CLIP_STATUS_ZERO_DIVISION
STATUS_VALUE = cost.CLIP_STATUS_VALUE
STATUS_REFUSAL = cost.CLIP_STATUS_REFUSAL
STATUS_UNSUPPORTED = cost.CLIP_STATUS_UNSUPPORTED
STATUS_MISSING_KEY = cost.CLIP_STATUS_MISSING_KEY

OVERFLOW_TEXTS = cost.OVERFLOW_TEXTS


class SeamUnsupported(Exception):
    """The seam cannot carry this input (an `int` where a `Fraction` is required, ...): never a comparison failure."""


@dataclass
class Answer:
    """The native answer of one seam: the raw outcome, the cost it left and the memory log of the operation."""

    status: int
    value: object
    detail: list
    counts: list
    articles: list
    log: list
    state: object
    #: Compute time of the seam inside the extension (nanoseconds; decoding the arguments is part of it, the crossing is not).
    nanoseconds: int = 0
    #: What a seam adds after the common answer: for `CLIP_GEOMETRY` `[normal writes, compute nanoseconds of the operation alone]`.
    extras: list = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.status == cost.STATUS_OK

    @property
    def unsupported(self) -> bool:
        return self.status == STATUS_UNSUPPORTED

    def result(self) -> "cost.OpResult":
        return cost.OpResult(self.status, self.value, self.detail, self.counts, self.articles, self.log, self.state)


# --------------------------------------------------------------------------
# strings and numbers
# --------------------------------------------------------------------------


def enc_str(text: str) -> list:
    return [None, int.from_bytes(text.encode("utf-8") + b"\x01", "little")]


def dec_str(wire) -> str:
    if not (isinstance(wire, list) and len(wire) == 2 and wire[0] is None):
        raise SeamUnsupported(f"not a wire string: {wire!r}")
    number = wire[1]
    data = number.to_bytes((number.bit_length() + 7) // 8, "little")
    if not data or data[-1] != 1:
        raise SeamUnsupported("a wire string without its sentinel")
    return data[:-1].decode("utf-8")


def fraction(value, what: str) -> Fraction:
    if type(value) is not Fraction:
        raise SeamUnsupported(f"{what} must be a Fraction, not {type(value).__name__}")
    return value


# --------------------------------------------------------------------------
# encoders
# --------------------------------------------------------------------------


def enc_chart_point(point) -> list:
    return [fraction(point[0], "a chart x"), fraction(point[1], "a chart y")]


def enc_chart(chart) -> list:
    return [enc_chart_point(point) for point in chart]


def enc_point(point) -> list:
    return [point[0], point[1]]


def enc_points(points) -> list:
    return [enc_point(point) for point in points]


def enc_triangle(triangle) -> list:
    if len(triangle.chart) != 3:
        raise SeamUnsupported("a source triangle has three chart points")
    normals = None
    if triangle.normals:
        normals = [[float(axis) if type(axis) is float else _not_float(axis) for axis in row] for row in triangle.normals]
    return [
        enc_str(triangle.name),
        enc_chart(triangle.chart),
        [[fraction(axis, "a corner axis") for axis in corner] for corner in triangle.corners],
        fraction(triangle.twice_area, "a twice area"),
        [edge if type(edge) is float else _not_float(edge) for edge in triangle.box],
        normals,
        enc_str(triangle.face),
    ]


def _not_float(value):
    raise SeamUnsupported(f"a binary64 expected, got {type(value).__name__}")


def enc_triangles(triangles) -> list:
    return [enc_triangle(triangle) for triangle in triangles]


def enc_key(key) -> list:
    return [enc_str(part) if isinstance(part, str) else part for part in key]


def enc_cell(cell) -> list:
    return [
        enc_key(cell.key),
        enc_str(cell.name),
        enc_chart(cell.chart),
        list(cell.box),
        fraction(cell.twice_area, "a cell area"),
        list(cell.members),
        [list(pair) for pair in cell.diagonals],
        None if cell.hinge is None else [cell.hinge.triangle, cell.hinge.edge, cell.hinge.jump_square],
        cell.flat_square,
        [[edge, enc_chart_point(corner)] for edge, corner in cell.straight],
        None if cell.group is None else enc_key(cell.group),
    ]


def enc_memo(memo: dict) -> list:
    out = []
    for (kind, face), value in memo.items():
        if kind == "cell":
            out.append([0, enc_str(face), [1, enc_str(value)] if isinstance(value, str) else [0, enc_cell(value)]])
        elif kind == "flat":
            out.append([1, enc_str(face), value])
        else:
            raise SeamUnsupported(f"a memo key of kind {kind!r}")
    return out


def enc_named_points(points: dict) -> list:
    return [[enc_str(key), enc_point(point)] for key, point in points.items()]


# --------------------------------------------------------------------------
# decoders (oracle types)
# --------------------------------------------------------------------------


def _kernel():
    from cftuv_envelope.materialize import clip_cells, clip_snap
    from cftuv_envelope.numeric import LocalPoint3V1

    return clip_cells, clip_snap, LocalPoint3V1


def dec_key(wire) -> tuple:
    tag = dec_str(wire[0])
    if tag == "t":
        return ("t", wire[1])
    if tag == "p":
        return ("p", dec_str(wire[1]))
    return (tag, dec_str(wire[1]), wire[2])


def dec_cell(wire):
    clip_cells, _snap, _point = _kernel()
    key, name, chart, box, area, members, diagonals, hinge, flat, straight, group = wire
    return clip_cells.ClipCellV1(
        dec_key(key),
        dec_str(name),
        tuple((point[0], point[1]) for point in chart),
        tuple(box),
        area,
        tuple(members),
        tuple(tuple(pair) for pair in diagonals),
        None if hinge is None else clip_cells.HingeV1(hinge[0], hinge[1], hinge[2]),
        flat,
        tuple((edge, (corner[0], corner[1])) for edge, corner in straight),
        None if group is None else dec_key(group),
    )


def dec_plan(wire):
    clip_cells, _snap, _point = _kernel()
    cells, unmergeable, plan_pairs = wire
    return clip_cells.CellPlanV1(tuple(dec_cell(cell) for cell in cells), tuple((dec_str(face), dec_str(reason)) for face, reason in unmergeable), plan_pairs)


def dec_memo(wire) -> dict:
    memo: dict = {}
    for kind, face, payload in wire:
        if kind == 0:
            memo[("cell", dec_str(face))] = dec_str(payload[1]) if payload[0] == 1 else dec_cell(payload[1])
        else:
            memo[("flat", dec_str(face))] = payload
    return memo


def dec_named_points(wire) -> dict:
    return {dec_str(key): (point[0], point[1]) for key, point in wire}


def dec_snap(wire):
    _cells, clip_snap, _point = _kernel()
    points, moved, counters = wire
    names = clip_snap.COUNTER_NAMES
    return clip_snap.CornerSnapV1(dec_named_points(points), dec_named_points(moved), tuple(zip(names, counters)))


def dec_lift(wire):
    _cells, _snap, local_point = _kernel()
    position, name, normal = wire
    return local_point(*position), (dec_str(name), None if normal is None else tuple(normal))


def optional_tuple(wire):
    return None if wire is None else tuple(wire)


#: Wire value -> the object the oracle function returns, per seam.
DECODERS = {
    "SORT_SEQUENCE": lambda wire: (list(wire[0]), [tuple(pair) for pair in wire[1]]),
    "FLOAT_SUM": lambda wire: wire,
    "LINE_VALUE": lambda wire: wire,
    "WINDOW": tuple,
    "VALUES_IN": list,
    "STRETCH_SQUARE": lambda wire: wire,
    "LIFT_KNOWN": dec_lift,
    "NANOMETRES": lambda wire: wire,
    "MILLI_CELLS": lambda wire: wire,
    "ORIENTATION": lambda wire: wire,
    "SHOELACE_SIGN": lambda wire: wire,
    "DOUBLED_SHOELACE": lambda wire: wire,
    "WITHIN_EDGE_GAP": tuple,
    "HINGE_DEPTH_SQUARE": lambda wire: wire,
    "CHORD_OF": tuple,
    "EDGE_CONSTANTS": optional_tuple,
    "CHEAP_SIGN": tuple,
    "RATIONAL_PAIR": optional_tuple,
    "BUILD_CELLS": lambda wire: dec_plan(wire[0]),
    "SNAP_SOURCE_VERTICES": dec_snap,
    "TRIANGULATE_EXACT": lambda wire: None if wire is None else tuple(tuple(triangle) for triangle in wire),
    "CONVEX_QUAD_RING": optional_tuple,
    "HAS_RIGHT_TURN": lambda wire: wire,
    "ORDERED": lambda wire: list(wire),
    "CLIP_GEOMETRY": lambda wire: dec_clipped(wire),
}


# --------------------------------------------------------------------------
# the whole operation (CLIP_GEOMETRY)
# --------------------------------------------------------------------------


def law_code(law) -> int:
    """`0` planar polygons, `1` quad strips, `2` any other member (`_split_for_law` compares by identity and asks the ears of the rest)."""

    from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1

    if law is DecalTopologyLawV1.PLANAR_POLYGONS_V1:
        return 0
    return 1 if law is DecalTopologyLawV1.QUAD_STRIPS_V1 else 2


def _flags(items):
    return None if items is None else [bool(item) for item in items]


def enc_inert(inert) -> list:
    """The chain station plan's pairs of faces in the iteration order of the set (`[[name, name], ...]`); a pair that is not two names is dropped, as the oracle skips it."""

    pairs = []
    for pair in inert:
        names = list(pair)
        if len(names) == 2:
            pairs.append([enc_str(names[0]), enc_str(names[1])])
    return pairs


def enc_geometry(plane, kwargs: dict, version=None) -> list:
    """The arguments of `clip_geometry(plane, budget, **kwargs)` as the `CLIP_GEOMETRY` seam reads them."""

    seam = []
    for pair in kwargs["seam"]:
        keys = tuple(pair)
        seam.append([enc_str(keys[0]), enc_str(keys[-1])])
    return [
        *(_version() if version is None else version),
        enc_triangles(plane.triangles),
        enc_named_points(kwargs["points"]),
        [[enc_str(key) for key, _point in cycle] for cycle in kwargs["cycles"]],
        [[[enc_str(key) for key in keys] for keys in face] for face in kwargs["polygons"]],
        law_code(kwargs["law"]),
        seam,
        _flags(kwargs["fans"]),
        _flags(kwargs["flows"]),
        bool(kwargs["by_faces"]),
        enc_inert(kwargs.get("inert", ())),
    ]


def _version() -> list:
    import sys

    return [sys.version_info.major, sys.version_info.minor]


def _keyed(entries) -> list:
    return [(dec_str(key), (point[0], point[1])) for key, point in entries]


def dec_clipped(wire):
    """The `ClippedV1` of the oracle from the answer value of `CLIP_GEOMETRY` (same containers, tuples and lists as `run` builds)."""

    from cftuv_envelope.materialize import clip

    _cells, _snap, local_point = _kernel()
    polygons, cycles, lists, extras, points, snapped, lifted, counters, note = wire
    return clip.ClippedV1(
        polygons=[tuple(tuple(dec_str(key) for key in keys) for keys in face) for face in polygons],
        cycles=[_keyed(cycle) for cycle in cycles],
        vertex_lists=[_keyed(entries) for entries in lists],
        extra_lists=[_keyed(entries) for entries in extras],
        points=dict(_keyed(points)),
        snapped=dict(_keyed(snapped)),
        lifted={
            dec_str(key): (local_point(*position), (dec_str(name), None if normal is None else tuple(normal)))
            for key, position, name, normal in lifted
        },
        counters=tuple((dec_str(name), value) for name, value in counters),
        note=dec_str(note),
    )


def dec_writes(extras: list) -> list:
    """`[(position, normal), ...]` of the `_normal_by_position` writes in the order of the calls (tuples of floats)."""

    return [(tuple(position), tuple(normal)) for position, normal in extras[0]]


# --------------------------------------------------------------------------
# the call
# --------------------------------------------------------------------------


def request_bytes(name: str, arguments, header=None) -> bytes:
    """One seam request: `[header, opcode, arguments]` in the boundary format (`header` is the cost header or `None`)."""

    return codec.encode_value([header, OPCODES[name], list(arguments)])


def answer_of(response: bytes) -> Answer:
    outcome, counts, articles, log, state, nanoseconds, *extras = codec.decode_value(response)
    status = outcome[0]
    ok = status == cost.STATUS_OK
    return Answer(status, outcome[1] if ok else None, outcome[1:] if not ok else [], counts, articles, log, state, nanoseconds, extras)


class SeamRunner:
    """Runs seams on one native session. A call with a header (the full memory sync) is self-contained: the sync clears
    and reloads every table, so the session keeps nothing from the call before."""

    def __init__(self) -> None:
        import cftuv_native

        self._native = cftuv_native
        self._session = cftuv_native.new_clip_seam_session()

    def call(self, name: str, arguments, header=None) -> Answer:
        return answer_of(self._native.clip_seam_run(self._session, request_bytes(name, arguments, header)))

    def value(self, name: str, arguments, header=None):
        """The decoded value of a successful call (a refused call raises `RuntimeError` naming the status)."""

        answer = self.call(name, arguments, header)
        if not answer.ok:
            raise RuntimeError(f"seam {name} refused: status {answer.status} {answer.detail!r}")
        return DECODERS[name](answer.value)


def exception_of(answer: Answer, budget) -> tuple:
    """`(exception class name, text)` the oracle's exception has for a refused answer. `budget` is the budget object of
    the call BEFORE it (only its identity fields are read, the articles are the native ones)."""

    status = answer.status
    if status == cost.STATUS_EXHAUSTED:
        text = cost.exhaustion_detail(budget, answer.result())
        return ("ExactCanonicalizationWorkBudgetExhausted", text)
    if status == STATUS_OVERFLOW:
        return ("OverflowError", OVERFLOW_TEXTS[answer.detail[0]])
    if status == STATUS_ZERO_DIVISION:
        return ("ZeroDivisionError", dec_str(answer.detail[0]))
    if status == STATUS_VALUE:
        return ("ValueError", dec_str(answer.detail[0]))
    if status == STATUS_REFUSAL:
        return ("MaterializationRefusal", f"{dec_str(answer.detail[0])}: {dec_str(answer.detail[1])}")
    if status == STATUS_MISSING_KEY:
        return ("KeyError", repr(dec_str(answer.detail[0])))
    if status == cost.STATUS_ZERO_DIVISOR:
        return ("ZeroSqrtSumDivisorError", cost.ZERO_DIVISOR_MESSAGE)
    raise RuntimeError(f"no oracle exception for the native status {status}: {answer.detail!r}")


# --------------------------------------------------------------------------
# the memory sync of a full-load header
# --------------------------------------------------------------------------


def _entries(table, entry) -> list:
    return [entry(key, value) for key, value in table]


def full_header(budget_state, primes, factorization, squarefree, support, *, want_state: bool = False) -> list:
    """The cost header that loads a recorded state whole: registry and tables cleared and re-sent, the budget as
    `[cap, six articles]` (`None`: a call without a budget)."""

    sync = [
        [True, [], list(primes)],
        [True, [], _entries(factorization, cost._entry_factorization)],
        [True, [], _entries(squarefree, cost._entry_squarefree)],
        [True, [], _entries(support, cost._entry_support)],
    ]
    budget = None if budget_state is None else [budget_state["cap"], *budget_state["articles"]]
    return [cost.OPTION_FULL_STATE if want_state else 0, sync, budget]
