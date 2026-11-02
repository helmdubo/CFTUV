"""W1 of the MATERIALIZE chain as a WHOLE operation: `mesh.exact_mesh` and `mesh.canonical_triangles`, native, with the oracle's side effects.

The unit the product replaces is what `domain._assemble` and `domain._at_host_positions` call (through `backend.materialize_w1_compute`): the vertices of the merged faces of a domain,
their exact `(s, r)`, their faces (`exact_mesh`), and the second, canonical triangulation of the same faces (`canonical_triangles`, which pays again when it is called). Everything the
oracle does besides returning a value is part of the answer and is reproduced here by `CostMirror.exact_mesh` / `CostMirror.canonical_triangles` (this module holds their bodies, so that
`cost.py` stays what it is): the six articles of the budget (one `radical` per distinct squared speed, every exact sign the binary64 filter cannot decide), the four tables of the
canonicalization memory with their order, `SIGN_COUNTS`, `UNBUDGETED_WORK` for a call without a budget, the lines `SOURCE_VERTEX_NAME_DROPPED` appended to `notes` in place, and the real
exceptions with the oracle's text.

The texts that need Python objects are composed HERE from structured fields the extension returns, never in Rust: `owner {owner}: ... differ from the face` takes the `repr` of the owner tuple
of the face (or of the part of its merged run) it names, and `STATION_VALUE_CONFLICT` takes the `Decimal` text of the answers (`mesh.decimal_of`, the oracle's own function, so the context and the
rounding are the oracle's).

Around the clip stage (`clip.cut_domain`, `domain._at_host_positions`), on the same mirror: `chord_stations` (CHORD_STATIONS_NATIVE_V1, `mesh.station_values` of the new vertices of the
clip with `chords` and `anchors`), `seam_edges` and `piece_triangles` (CLIP_STAGE_NATIVE_V1, the seam edges the clip may not cut and the triangles of the emitted pieces).

What the oracle reads LIVE, and what the shim does about it: `mesh.VERIFY_PLAIN_AFFINE` (a test turns it on to force the exact check of a plain face and an `AssertionError`): read at the call, a port
refusal when on.

A refusal of the PORT (`NativePortStale`, `NativePortUnsupported` whenever it is raised, an input the port does not carry) leaves every Python-visible state exactly as it was before the call
(articles, `SIGN_COUNTS`, `UNBUDGETED_WORK`, the four memory tables with their order, the lines of `notes`): the caller runs the oracle on the same budget, notes and tables. An exception of the
ORACLE (`MaterializationRefusal`, `ExactCanonicalizationWorkBudgetExhausted`) leaves the partial effects the oracle's exception leaves.
"""

from __future__ import annotations

import dataclasses
import hashlib
import inspect
from collections import Counter
from fractions import Fraction
from time import perf_counter_ns

from . import codec, cost, pin

OPERATION = "materialize_w1"
STATUS_REFUSAL = 11

RESULT_FIELDS = ("cycles", "points", "facts", "polygons", "tally", "rungs")
FRAME_FACE_FIELDS = (
    "face", "line", "claim_key", "frame_key", "station_model", "run", "fan_station", "physical_edge_ids", "chain_use_ids", "chain_ids", "flow_key", "instance_claim",
)
COVERED_FACE_FIELDS = (
    "region_id", "owner", "envelope_spec_id", "envelope_instance_id", "points", "doubled_area", "source_chain_id", "merged_owners", "parts",
)
RUN_FIELDS = ("run_id", "chain_id", "chain_use_id", "run_index", "origin", "direction", "s_origin", "inverse_length", "covector", "physical_edge_ids", "lineage_ids")
LINE_FIELDS = ("a", "b", "c", "q")
TABLE_NAMES = ("node_vertex_ids", "joins", "frame_of_run")

#: `(file, function, sha256 of its source)`: the functions of files too shared to pin whole (`stations.py`) that the Rust code mirrors. `native_status` does not see them; the binding does.
SHAPE_FUNCTIONS = (
    ("materialize/stations.py", "station_of", "c86c90a836635a7138771b59d3bbb0169c7547d9ea1c1ee658e7da20a7e8aae5"),
    ("materialize/stations.py", "transverse_root", "95b6f4bc07b11729758f79a453b36d7e3dca91264f119c3dabfbf4c445cbc395"),
    ("materialize/stations.py", "transverse_of", "c4e0d7b6b7e3145efc963e86c45daab0489441caa0560fc5bf18e06ed0dd653c"),
    ("materialize/stations.py", "ChainStationTableV1.join_station", "74ac842508c9c1bd457472e3bd81356201a599d7fb1b48b00a5b2d920fb95e59"),
    ("materialize/stations.py", "ChainStationTableV1.cut_join_station", "3f945e2850f612b9990cc76a1fa698b1712c2ac8213e599d04af72d76f0fcc17"),
)

_KERNEL: list = []


def _names(cls) -> tuple:
    return tuple(item.name for item in dataclasses.fields(cls))


def source_digest(function) -> str:
    """sha256 of the source text of `function` (CRLF turned into LF): the shape of a function the pin does not hold whole."""

    return hashlib.sha256(inspect.getsource(function).replace("\r\n", "\n").encode("utf-8")).hexdigest()


def _resolve(module, qualified: str):
    found = module
    for part in qualified.split("."):
        found = getattr(found, part)
    return found


def _kernel():
    """The kernel modules and classes the drop-in is built from (on `sys.path`), resolved on first use."""

    if not _KERNEL:
        from cftuv_envelope import exact_sqrt_sum
        from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
        from cftuv_envelope.materialize import admit, coalesce, frames, mesh, stations
        from cftuv_envelope.wavefront import event_time

        _KERNEL.append((exact_sqrt_sum, mesh, frames, coalesce, stations, event_time, admit, DecalTopologyLawV1))
    return _KERNEL[0]


def shape_digests() -> dict:
    """`{(file, function): digest or None}` of the `SHAPE_FUNCTIONS`, read from the live kernel (`None`: the function or its source is gone)."""

    _exact, _mesh, _frames, _coalesce, stations, _time, _admit, _law = _kernel()
    found = {}
    for file, name, _wanted in SHAPE_FUNCTIONS:
        try:
            found[(file, name)] = source_digest(_resolve(stations, name))
        except (AttributeError, OSError, TypeError):
            found[(file, name)] = None
    return found


def bind(mirror) -> None:
    """Hands the result classes to the extension, after checking they have the shape the Rust side reads and builds (`NativePortStale` else)."""

    exact, mesh, frames, coalesce, stations, event_time, admit, laws = _kernel()
    live = shape_digests()
    pin.check_shapes(
        (
            ("materialize/mesh.py", "ExactMeshV1 fields", _names(mesh.ExactMeshV1) == RESULT_FIELDS),
            ("materialize/frames.py", "FrameFaceV1 fields", _names(frames.FrameFaceV1) == FRAME_FACE_FIELDS),
            ("materialize/coalesce.py", "CoveredFaceV1 fields", _names(coalesce.CoveredFaceV1) == COVERED_FACE_FIELDS),
            ("materialize/stations.py", "StationRunV1 fields", _names(stations.StationRunV1) == RUN_FIELDS),
            ("wavefront/event_time.py", "SupportLineV1 fields", _names(event_time.SupportLineV1) == LINE_FIELDS),
            ("materialize/stations.py", "ChainStationTableV1 has the tables W1 reads", all(name in _names(stations.ChainStationTableV1) for name in TABLE_NAMES)),
            ("materialize/admit.py", "MaterializationOutcome names", all(name in admit.MaterializationOutcome.__members__ for name in ("BATCH_DID_NOT_VALIDATE", "TESSELLATION_DID_NOT_CLOSE"))),
            ("contracts/geometry_batch.py", "DecalTopologyLawV1 members", hasattr(laws, "PLANAR_POLYGONS_V1") and hasattr(laws, "QUAD_STRIPS_V1")),
        )
    )
    pin.check_mirrored_functions(OPERATION, tuple((file, f"{name} is the function the Rust code mirrors", live[(file, name)] == wanted) for file, name, wanted in SHAPE_FUNCTIONS))
    try:
        mirror._session.bind_w1(exact.SqrtSumV1, Fraction, mesh.ExactMeshV1, Counter)
    except TypeError as error:
        if "raw slot access was forced" in str(error):
            raise  # the refusal of a forced slot mode (CFTUV_NATIVE_SLOTS=raw): not a change of the oracle's classes
        raise pin.NativePortStale(f"the oracle classes the native W1 is built from changed: {error}") from None
    mirror._w1_bound = True


def _owner_of(frame_faces, face: int, part: int):
    """The `owner` a text of the oracle names: that of the merged face, or of the part of its merged run."""

    covered = frame_faces[face].face
    return covered.owner if part < 0 else covered.parts[part].owner


def _raise_refusal(detail, frame_faces, table) -> None:
    """The `MaterializationRefusal` of the oracle for a refused mesh (after every effect was applied), with its text composed from the structured detail."""

    _exact, mesh, frames, _coalesce, _stations, _time, admit, _laws = _kernel()
    outcomes = admit.MaterializationOutcome
    kind = detail[0]
    if kind == "collision":
        _kind, key, region, place = detail
        raise frames.MaterializationRefusal(outcomes.BATCH_DID_NOT_VALIDATE, f"VERTEX_KEY_COLLISION: {key} names two different points (region {region}, {place})")
    if kind == "conflict":
        _kind, key, region, face, answered = detail
        owner = frame_faces[face].face.owner
        text = ", ".join(f"{name}=(s {mesh.decimal_of(s, table.scale):.6f}, r {mesh.decimal_of(r, table.scale):.6f})" for name, s, r in answered)
        run_id = answered[-1][0]
        raise frames.MaterializationRefusal(
            outcomes.BATCH_DID_NOT_VALIDATE,
            f"STATION_VALUE_CONFLICT: vertex {key} in region {region} has two (s, r) answers from owner {owner}: {text}",
            station_conflict=(run_id, *(name for name, _s, _r in answered[:-1])),
        )
    if kind == "not_closed":
        _kind, face, part, what = detail
        raise frames.MaterializationRefusal(outcomes.TESSELLATION_DID_NOT_CLOSE, f"owner {_owner_of(frame_faces, face, part)}: {what} differ from the face")
    if kind == "no_triangulation":
        _kind, face, part, points = detail
        raise frames.MaterializationRefusal(outcomes.TESSELLATION_DID_NOT_CLOSE, f"owner {_owner_of(frame_faces, face, part)}: {points} contour points")
    raise pin.NativePortUnsupported(f"the native W1 answered a refusal it cannot name: {kind!r}")


def _begin(mirror):
    """`(sync, slow, real tables)` of a call, after the pin check and the binding."""

    pin.require(OPERATION)
    if not mirror._w1_bound:
        bind(mirror)
    exact = cost._exact()
    sync, slow = mirror._sync_in(exact)
    real = (exact._KNOWN_PRIMES, exact._KNOWN_PRIME_SET, exact._FACTORIZATION_MEMO, exact._SQUAREFREE_MEMO, exact._PRIME_SUPPORT_MEMO)
    return exact, sync, slow, real


def exact_mesh(mirror, frame_faces, region_ids, layout, table, notes, chord_names, lattice_alpha, budget, chart_cw, law, exact_plane, lap=None):
    """`mesh.exact_mesh(frame_faces, region_ids, layout, table, notes, chord_names, lattice_alpha, budget, chart_cw, law, exact_plane, lap)`, whole, with its exact side effects (see the module note).

    Returns the oracle's `ExactMeshV1`. `lap` is called as the oracle calls it, after the stations and after the tessellation, but only once the call is over: the clock sees the whole operation under
    `STATIONS_UV` and a moment under `TESSELLATE`.
    """

    _exact_module, mesh, _frames, _coalesce, _stations, _time, _admit, laws = _kernel()
    if mesh.VERIFY_PLAIN_AFFINE:
        raise pin.NativePortUnsupported("mesh.VERIFY_PLAIN_AFFINE is on: the oracle checks every plain face and raises an AssertionError the port does not claim")
    started = perf_counter_ns()
    exact, sync, slow, real = _begin(mirror)
    code = 0 if law is laws.PLANAR_POLYGONS_V1 else 1 if law is laws.QUAD_STRIPS_V1 else 2
    regions = [layout.region_of(frame_face) for frame_face in frame_faces]
    state = None if budget is None else (budget.cap, budget.spent_by_article())
    called = perf_counter_ns()
    try:
        result, status, detail, counts, articles, bits, native = mirror._session.w1_exact_mesh(
            frame_faces, region_ids, regions, table, notes, chord_names, lattice_alpha, bool(chart_cw), code, bool(exact_plane),
            None if sync is cost.UNCHANGED_SYNC else codec.encode_value(sync), state, real,
        )
    except BaseException:
        mirror.invalidate()
        raise
    returned = perf_counter_ns()
    if status not in cost.ORACLE_STATUSES:
        # a refusal of the port: the extension applied nothing and forgot its mirror; neither do we apply the (zero) cost, and the mirror is forgotten here too
        mirror.invalidate()
        mirror.last_w1_timings = (called - started, returned - called, perf_counter_ns() - returned, perf_counter_ns() - started, *native)
        raise mirror._port_refusal(status, detail)
    mirror._settle_counts(exact, counts)
    mirror._settle_articles(exact, budget, articles)
    mirror._remember(exact, bits, slow)
    finished = perf_counter_ns()
    mirror.last_w1_timings = (called - started, returned - called, finished - returned, finished - started, *native)
    if status == 0:
        if lap is not None:
            lap("STATIONS_UV")
            lap("TESSELLATE")
        return result
    if lap is not None and detail[0] in ("not_closed", "no_triangulation"):
        lap("STATIONS_UV")  # the refusal is the tessellation's: the stations were done
    if status == STATUS_REFUSAL:
        _raise_refusal(detail, frame_faces, table)
    if status == cost.CLIP_STATUS_MISSING_KEY:
        raise KeyError(tuple(detail))
    cost.OpResult(status, None, detail, counts, articles, (), None).raise_for(budget)
    raise cost.NativeMirrorError(f"the native W1 answered the outcome code {status}")


def _raise_seam_refusal(detail, owner) -> None:
    """The `MaterializationRefusal` of a seam (one contour): the owner is that of the piece or of the fan."""

    _exact, _mesh, frames, _coalesce, _stations, _time, admit, _laws = _kernel()
    outcomes = admit.MaterializationOutcome
    if detail[0] == "not_closed":
        raise frames.MaterializationRefusal(outcomes.TESSELLATION_DID_NOT_CLOSE, f"owner {owner}: {detail[3]} differ from the face")
    if detail[0] == "no_triangulation":
        raise frames.MaterializationRefusal(outcomes.TESSELLATION_DID_NOT_CLOSE, f"owner {owner}: {detail[3]} contour points")
    raise pin.NativePortUnsupported(f"the native W1 seam answered a refusal it cannot name: {detail[0]!r}")


def _seam(mirror, kind, owner, text_owner, area, cycle, budget, reverse, exact_plane, tally, uv, unit, in_flow, partition, affine_known):
    """The part seam of the tessellation of ONE contour (`mesh._contour_polygons`, `mesh._fan_faces`) with the `(s, r)` of its vertices in a mapping `uv` (the oracle takes a `uv_of` callable)."""

    started = perf_counter_ns()
    exact, sync, slow, real = _begin(mirror)
    state = None if budget is None else (budget.cap, budget.spent_by_article())
    called = perf_counter_ns()
    try:
        result, status, detail, counts, articles, bits, native = mirror._session.w1_seam(
            kind, [key for key, _point in cycle], [point for _key, point in cycle], dict(uv), area, owner, bool(reverse), bool(exact_plane), unit, bool(in_flow), bool(partition),
            bool(affine_known), None if sync is cost.UNCHANGED_SYNC else codec.encode_value(sync), state, real,
        )
    except BaseException:
        mirror.invalidate()
        raise
    returned = perf_counter_ns()
    if status not in cost.ORACLE_STATUSES:
        mirror.invalidate()
        mirror.last_w1_timings = (called - started, returned - called, perf_counter_ns() - returned, perf_counter_ns() - started, *native)
        raise mirror._port_refusal(status, detail)
    mirror._settle_counts(exact, counts)
    mirror._settle_articles(exact, budget, articles)
    mirror._remember(exact, bits, slow)
    mirror.last_w1_timings = (called - started, returned - called, perf_counter_ns() - returned, perf_counter_ns() - started, *native)
    polygons, items = result
    for name, value in items:
        if name.endswith("_MAX_MILLI_ALPHA"):
            tally[name] = max(tally[name], value)
        else:
            tally[name] += value
    if status == 0:
        return polygons
    if status == STATUS_REFUSAL:
        _raise_seam_refusal(detail, text_owner)
    if status == cost.CLIP_STATUS_MISSING_KEY:
        raise KeyError(detail[1])
    cost.OpResult(status, None, detail, counts, articles, (), None).raise_for(budget)
    raise cost.NativeMirrorError(f"the native W1 seam answered the outcome code {status}")


def contour_polygons(mirror, piece, cycle, budget, reverse, exact_plane, tally, uv, unit=1, in_flow=False, partition=False, affine_known=False):
    """`mesh._contour_polygons(piece, cycle, budget, reverse, exact_plane, tally, uv_of, unit, in_flow, partition, affine_known)`, whole, with `uv` a mapping `key -> (s, r)` (a test seam of the port)."""

    return _seam(mirror, 0, (), piece.owner, piece.doubled_area, cycle, budget, reverse, exact_plane, tally, uv, Fraction(unit), in_flow, partition, affine_known)


def fan_faces(mirror, frame_face, cycle, budget, reverse, exact_plane, tally, uv, affine_known=False):
    """`mesh._fan_faces(frame_face, cycle, budget, reverse, exact_plane, tally, uv_of, affine_known)`, whole, with `uv` a mapping `key -> (s, r)` (a test seam of the port)."""

    return _seam(mirror, 1, frame_face.face.owner, frame_face.face.owner, frame_face.face.doubled_area, cycle, budget, reverse, exact_plane, tally, uv, Fraction(1), False, False, affine_known)


def canonical_triangles(mirror, frame_faces, cycles, budget, reverse):
    """`mesh.canonical_triangles(frame_faces, cycles, budget, reverse)`, whole, with its exact side effects: the ear-clipped triangles of every merged face by keys."""

    started = perf_counter_ns()
    exact, sync, slow, real = _begin(mirror)
    state = None if budget is None else (budget.cap, budget.spent_by_article())
    called = perf_counter_ns()
    try:
        result, status, detail, counts, articles, bits, native = mirror._session.w1_canonical(
            frame_faces, cycles, bool(reverse), None if sync is cost.UNCHANGED_SYNC else codec.encode_value(sync), state, real,
        )
    except BaseException:
        mirror.invalidate()
        raise
    returned = perf_counter_ns()
    if status not in cost.ORACLE_STATUSES:
        mirror.invalidate()
        mirror.last_w1_timings = (called - started, returned - called, perf_counter_ns() - returned, perf_counter_ns() - started, *native)
        raise mirror._port_refusal(status, detail)
    mirror._settle_counts(exact, counts)
    mirror._settle_articles(exact, budget, articles)
    mirror._remember(exact, bits, slow)
    finished = perf_counter_ns()
    mirror.last_w1_timings = (called - started, returned - called, finished - returned, finished - started, *native)
    if status == 0:
        return result
    if status == STATUS_REFUSAL:
        _raise_refusal(detail, frame_faces, None)
    cost.OpResult(status, None, detail, counts, articles, (), None).raise_for(budget)
    raise cost.NativeMirrorError(f"the native W1 answered the outcome code {status}")

def chord_stations(mirror, frame_faces, lists, layout, table, budget, tally, chords, anchors):
    """`mesh.station_values(frame_faces, lists, layout, table, lattice_alpha, budget, tally, chords=chords, anchors=anchors)` of the clip stage (`clip.cut_domain`), whole, with its
    exact side effects: the `(s, r)` of the new vertices of the clip (`lists`: `ClippedV1.extra_lists`), with the station of a vertex on a SHARED edge interpolated between the facts of its
    ends (`anchors`, law `RUNG_CHORD_STATION_V1`). Returns the oracle's dict `facts` in its order; the oracle's additions to `tally` are made in place, also when it refuses afterwards.
    """

    started = perf_counter_ns()
    exact, sync, slow, real = _begin(mirror)
    regions = [layout.region_of(frame_face) for frame_face in frame_faces]
    state = None if budget is None else (budget.cap, budget.spent_by_article())
    called = perf_counter_ns()
    try:
        result, status, detail, counts, articles, bits, native = mirror._session.w1_chord_stations(
            frame_faces, regions, lists, chords, anchors, table, None if sync is cost.UNCHANGED_SYNC else codec.encode_value(sync), state, real,
        )
    except BaseException:
        mirror.invalidate()
        raise
    returned = perf_counter_ns()
    if status not in cost.ORACLE_STATUSES:
        mirror.invalidate()
        mirror.last_w1_timings = (called - started, returned - called, perf_counter_ns() - returned, perf_counter_ns() - started, *native)
        raise mirror._port_refusal(status, detail)
    mirror._settle_counts(exact, counts)
    mirror._settle_articles(exact, budget, articles)
    mirror._remember(exact, bits, slow)
    finished = perf_counter_ns()
    mirror.last_w1_timings = (called - started, returned - called, finished - returned, finished - started, *native)
    facts, items = result
    if tally is not None:
        for name, value in items:
            tally[name] += value
    if status == 0:
        return facts
    if status == STATUS_REFUSAL:
        _raise_refusal(detail, frame_faces, table)
    cost.OpResult(status, None, detail, counts, articles, (), None).raise_for(budget)
    raise cost.NativeMirrorError(f"the native W1 stations answered the outcome code {status}")

#: `assemble.edge_kind` (a file the W1 pin does not hold): the law `seam_edges` reads, checked by the digest of its source once per process.
EDGE_KIND_DIGEST = "aeba298fa369fad48d44bf4fa9937878fd0ce1c4fc9c7994ffc7db54ae3a700b"
_EDGE_KIND_CHECKED: list = []


def _clip_stage(mirror):
    """The pins of a port of the clip stage around the clip (CLIP_STAGE_NATIVE_V1): the clip's (`clip.py`, `tessellate.py`), W1's and the digest of `edge_kind`."""

    pin.require("clip")
    exact, sync, slow, real = _begin(mirror)
    if not _EDGE_KIND_CHECKED:
        from cftuv_envelope.materialize import assemble

        pin.check_mirrored_functions("clip", (("materialize/assemble.py", "edge_kind is the function the Rust code mirrors", source_digest(assemble.edge_kind) == EDGE_KIND_DIGEST),))
        _EDGE_KIND_CHECKED.append(True)
    return exact, sync, slow, real


def piece_triangles(mirror, polygons, points, budget):
    """`clip.piece_triangles(polygons, points, budget)` (CLIP_STAGE_NATIVE_V1), whole, with its exact side effects: a triangle as it is, every other emitted polygon by
    the exact ear triangulation of its chart points (the signs on `budget`, the memory and `SIGN_COUNTS` as the oracle leaves them)."""

    started = perf_counter_ns()
    exact, sync, slow, real = _clip_stage(mirror)
    state = None if budget is None else (budget.cap, budget.spent_by_article())
    called = perf_counter_ns()
    try:
        result, status, detail, counts, articles, bits, native = mirror._session.w1_piece_triangles(
            polygons, points, None if sync is cost.UNCHANGED_SYNC else codec.encode_value(sync), state, real,
        )
    except BaseException:
        mirror.invalidate()
        raise
    returned = perf_counter_ns()
    if status not in cost.ORACLE_STATUSES:
        mirror.invalidate()
        mirror.last_w1_timings = (called - started, returned - called, perf_counter_ns() - returned, perf_counter_ns() - started, *native)
        raise mirror._port_refusal(status, detail)
    mirror._settle_counts(exact, counts)
    mirror._settle_articles(exact, budget, articles)
    mirror._remember(exact, bits, slow)
    finished = perf_counter_ns()
    mirror.last_w1_timings = (called - started, returned - called, finished - returned, finished - started, *native)
    if status == 0:
        return result
    if status == STATUS_REFUSAL and detail[0] == "no_piece_triangulation":
        _exact_module, _mesh, frames, _coalesce, _stations, _time, admit, _laws = _kernel()
        raise frames.MaterializationRefusal(
            admit.MaterializationOutcome.TESSELLATION_DID_NOT_CLOSE, f"an emitted clip face of {detail[1]} vertices has no triangulation"
        )
    cost.OpResult(status, None, detail, counts, articles, (), None).raise_for(budget)
    raise cost.NativeMirrorError(f"the native piece triangles answered the outcome code {status}")


def seam_edges(mirror, frame_faces, polygons, facts, layout, lattice_alpha):
    """`clip.seam_edges(frame_faces, polygons, facts, layout, lattice_alpha)` (CLIP_STAGE_NATIVE_V1): the same frozenset of key pairs (no budget, no memory); a missing
    fact (the oracle's `KeyError`) or a class the port does not carry is declined."""

    _clip_stage(mirror)
    regions = [layout.region_of(frame_face) for frame_face in frame_faces[: len(polygons)]]
    if len(regions) != len(polygons):
        raise pin.NativePortUnsupported("seam_edges: more polygon lists than frame faces (IndexError in the oracle)")
    found = mirror._session.w1_seam_edges(polygons, regions, facts, lattice_alpha)
    if found is None:
        raise pin.NativePortUnsupported("seam_edges: an input the port does not carry")
    return found


#: `certificate.rs` statuses of an oracle refusal (CERTIFICATE_SIDE_CUTS_NATIVE_V1) and the decline of the port.
CERTIFICATE_STATUS_DECLINED = 12
CERTIFICATE_REFUSALS = {21: "PARAM_CLIP_NODE", 22: "PARAM_CLIP_FACT", 23: "PARAM_CLIP_STRUCTURE_CHANGED", 24: "PARAM_CLIP_REFUSED"}


def certificate_side_slopes(mirror, inputs, step, key_slope, fact_slope):
    """`parametric._clip_slopes_at(inputs, step, key_slope, fact_slope)` (CERTIFICATE_SIDE_CUTS_NATIVE_V1, `certificate.rs`): the side cuts of the drag certificate, their structure
    against the domain's cut and the second differences in ONE native call; the inputs are read once and nothing in between crosses the boundary.

    Returns `(point slopes, fact slopes)` (the order of the dicts is not part of the answer: the certificate looks slopes up by key), or raises the oracle's `ParametricRefusal`
    by name (`PARAM_CLIP_REFUSED` carries the value of the clip's `MaterializationOutcome`). The memory tables and `SIGN_COUNTS` end as the Python side cuts leave them; the offset
    normals of the side lifts, the articles of the throwaway unlimited budgets and the throwaway tally are not reproduced (no answer reads them). A decline of the port
    (`NativePortUnsupported`: a plane that is not a bound surface, an input or an outcome the port leaves to the oracle) applies nothing, so the caller runs the oracle on the same state.
    """

    from . import materialize_hot_op

    materialize_hot_op._begin(mirror)
    if type(inputs.plane) is not materialize_hot_op._bound_surface():
        raise pin.NativePortUnsupported(f"certificate_side_slopes: the side clip runs on a BoundSurfaceLiftV1, not a {type(inputs.plane).__name__}")
    if inputs.cut is None:
        raise pin.NativePortUnsupported("certificate_side_slopes: the domain has no cut")
    started = perf_counter_ns()
    if not mirror._clip_bound:
        mirror._bind_clip()
    exact, sync, slow, real = _clip_stage(mirror)
    _exact_module, _mesh, _frames, _coalesce, _stations, _time, admit, laws = _kernel()
    from cftuv_envelope.materialize import parametric

    law = inputs.law
    code = 0 if law is laws.PLANAR_POLYGONS_V1 else 1 if law is laws.QUAD_STRIPS_V1 else 2
    frame_faces = inputs.frame_faces
    layout = inputs.layout
    regions = [layout.region_of(frame_face) for frame_face in frame_faces]
    fans = [item.is_fan for item in frame_faces]
    flows = [getattr(item, "flow_key", None) is not None for item in frame_faces]
    called = perf_counter_ns()
    try:
        status, detail, point_slopes, fact_slopes, counts, bits, native, attempts = mirror._session.certificate_side_slopes(
            inputs.plane.triangles, inputs.points_before, inputs.cycles, inputs.facts_before, key_slope, fact_slope, frame_faces, regions, fans, flows, inputs.polygons, code,
            bool(inputs.by_faces), inputs.inert, inputs.table, inputs.lattice_alpha, inputs.scale, step, parametric._STEP_ATTEMPTS, inputs.cut, inputs.facts_after,
            None if sync is cost.UNCHANGED_SYNC else codec.encode_value(sync), real,
        )
    except BaseException:
        mirror.invalidate()
        raise
    returned = perf_counter_ns()
    if status == CERTIFICATE_STATUS_DECLINED:
        mirror.invalidate()
        mirror.last_certificate_timings = (called - started, returned - called, 0, *native, attempts)
        raise pin.NativePortUnsupported(f"certificate_side_slopes: {detail}")
    mirror._settle_counts(exact, counts)
    mirror._remember(exact, bits, slow)
    mirror.last_certificate_timings = (called - started, returned - called, perf_counter_ns() - returned, *native, attempts)
    if status == 0:
        return point_slopes, fact_slopes
    name = CERTIFICATE_REFUSALS.get(status)
    if name is None:
        raise cost.NativeMirrorError(f"the native certificate side cuts answered the status {status}")
    if status == 24:
        raise parametric.ParametricRefusal(parametric.PARAM_CLIP_REFUSED, admit.MaterializationOutcome[detail].value)
    raise parametric.ParametricRefusal(getattr(parametric, name))

#: `certificate.rs` statuses of the arrays' oracle refusals (CERTIFICATE_CORE_NATIVE_V1).
CORE_REFUSALS = {31: "PARAM_LIFT_UNSUPPORTED", 32: "PARAM_LIFT_ON_EDGE", 33: "PARAM_FACT_UNRESOLVED"}


def certificate_core(mirror, inputs, batch, interval, request, key_slope, fact_slope, fact_floats, developable):
    """`parametric._certificate_tail(...)` on a bound surface outside the exact mode (CERTIFICATE_CORE_NATIVE_V1, `certificate.rs`): the side cuts of the domain's cut, the slopes
    of the clip and the arrays of the batch (positions, loops, limits, floors, the rate of the offset normal) in ONE native call that reads the batch once.

    Returns the oracle's `ParametricCertificateV1` (the `memo` label is set by the caller), or raises the oracle's `ParametricRefusal` by name. The positions are computed, not taken
    from the slope memory (the label counts the facts). Effects as `certificate_side_slopes`; a decline of the port applies nothing.
    """

    from array import array

    from . import materialize_hot_op
    from cftuv_envelope.materialize import parametric

    materialize_hot_op._begin(mirror)
    if type(inputs.plane) is not materialize_hot_op._bound_surface():
        raise pin.NativePortUnsupported(f"certificate_core: the arrays are read on a BoundSurfaceLiftV1, not a {type(inputs.plane).__name__}")
    started = perf_counter_ns()
    if not mirror._clip_bound:
        mirror._bind_clip()
    exact, sync, slow, real = _clip_stage(mirror)
    scale = inputs.scale
    side = None
    if inputs.cut is not None:
        step = parametric._step_for(inputs.alpha, interval)
        if step is None:
            raise parametric.ParametricRefusal(parametric.PARAM_WINDOW_NOT_CERTIFIED)
        _exact_module, _mesh, _frames, _coalesce, _stations, _time, _admit, laws = _kernel()
        law = inputs.law
        frame_faces = inputs.frame_faces
        layout = inputs.layout
        side = (
            inputs.points_before, inputs.cycles, inputs.facts_before, key_slope, fact_slope, frame_faces, [layout.region_of(item) for item in frame_faces],
            [item.is_fan for item in frame_faces], [getattr(item, "flow_key", None) is not None for item in frame_faces], inputs.polygons,
            0 if law is laws.PLANAR_POLYGONS_V1 else 1 if law is laws.QUAD_STRIPS_V1 else 2, bool(inputs.by_faces), inputs.inert, inputs.table, inputs.lattice_alpha, scale,
            step, parametric._STEP_ATTEMPTS, inputs.cut, inputs.facts_after,
        )
    limits = parametric._cut_point_limits(request.templates, scale)
    reach = {}
    if limits:
        for key, slope in key_slope.items():
            if slope is not None:
                found = limits.get(id(slope[0]))
                if found is not None:
                    reach[key] = found
    window = (float(interval.low), float("inf") if interval.high is None else float(interval.high))
    called = perf_counter_ns()
    try:
        status, detail, columns, rate, sizes, counts, bits, native, attempts = mirror._session.certificate_core(
            inputs.plane.triangles, side, batch.vertices, batch.faces, inputs.points_after, key_slope, inputs.lifted_names, reach, window, bool(developable), scale,
            fact_floats if inputs.uv_forms is None else (fact_floats, fact_slope, inputs.uv_forms),
            None if sync is cost.UNCHANGED_SYNC else codec.encode_value(sync), real,
        )
    except BaseException:
        mirror.invalidate()
        raise
    returned = perf_counter_ns()
    if status == CERTIFICATE_STATUS_DECLINED:
        mirror.invalidate()
        mirror.last_certificate_timings = (called - started, returned - called, 0, *native, attempts)
        raise pin.NativePortUnsupported(f"certificate_core: {detail}")
    mirror._settle_counts(exact, counts)
    mirror._remember(exact, bits, slow)
    mirror.last_certificate_timings = (called - started, returned - called, perf_counter_ns() - returned, *native, attempts)
    if status:
        if status == 24:
            _exact_module, _mesh, _frames, _coalesce, _stations, _time, admit, _laws = _kernel()
            raise parametric.ParametricRefusal(parametric.PARAM_CLIP_REFUSED, admit.MaterializationOutcome[detail].value)
        name = CERTIFICATE_REFUSALS.get(status) or CORE_REFUSALS.get(status)
        if name is None:
            raise cost.NativeMirrorError(f"the native certificate answered the status {status}")
        raise parametric.ParametricRefusal(getattr(parametric, name), detail or "")
    positions, loops, upper, lower = (array("d", item) for item in columns)
    vertex_count, loop_count, moving, nodes, facts = sizes
    return parametric.ParametricCertificateV1(
        parametric.PARAMETRIC_LAW,
        inputs.alpha,
        scale,
        (),
        "",
        vertex_count,
        loop_count,
        positions,
        loops,
        (("PARAMETRIC_MOVING_VERTICES", moving), ("PARAMETRIC_CLIP_NODE_SLOPES", nodes), ("PARAMETRIC_CLIP_FACT_SLOPES", facts)),
        None,
        approximate_rate=rate,
        vertex_limit_widths=upper,
        vertex_floor_widths=lower,
    )
