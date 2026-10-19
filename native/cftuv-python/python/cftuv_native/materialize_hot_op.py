"""The hot part of the materializer's SILHOUETTE stage as native operations: `silhouette.apply_silhouette` and `assemble.lift_vertices` on an exact plane.

The Python law stays the ORACLE (`materialize/silhouette.py`, `materialize/assemble.py`, `materialize/lift.py`); the Rust mirror (`native/cftuv-silhouette`, read by
`cftuv-python/src/materialize_hot.rs`) answers bit for bit what it carries, counters and the exact side effects included, and declines by name (`NativePortUnsupported`, nothing applied)
every input it does not carry; the dispatcher then runs the oracle on the same state.

* `apply_silhouette(mirror, source, budget)`: the pass and its check. The exact questions (orientations the binary64 filter cannot decide, signs of sums of products) are asked in the
  oracle's order on the oracle's budget: the INNER budget of the pass when the domain's budget has a cap (cap = what the domain has left, articles zero; spent replayed into the domain's
  budget after a pass that finished, nothing on an exhaustion, which is the pass's own `SKIPPED_WORK_BUDGET`), the domain's budget itself otherwise. The canonicalization memory, `SIGN_COUNTS`
  and the articles end where the oracle leaves them (`CostMirror`). A violation of the law is the oracle's `MaterializationRefusal(BATCH_DID_NOT_VALIDATE, "SILHOUETTE:...")`, raised here with
  the oracle's text. `HALF_EDGE_SHARED_IN_ONE_DIRECTION` (the chains of a mesh whose half-edge two faces own) is declined: the oracle names it with its own text.
* `source_lift(mirror, plane, positions, triangles, host_positions, step, polygons, points, budget, reverse)`: `domain._sourced` (the law `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1` and the
  check of the emitted faces), the triangulations exact and priced on the domain's budget.
* `lift_vertices(mirror, points, plane)`: `({key: PlaneLiftV1.lift(point)}, {key: (None, None)})` for a `PlaneLiftV1` (no budget, no memory); any other lift is declined.

What the shim reads LIVE: the chord budget (`silhouette.CHORD_BUDGET`), the copy mark of a cut vertex (`audit.CUT_RIGHT_COPY_MARK`), the classes the result is built from.

PINS: gate-approved (MATERIALIZE_HOT_PINS_APPROVED_V1, PIN_APPROVAL_BY_GATES_V1, `materialize_hot_approved_pins.py`, data only). A file that differs from its pin makes the operation
`stale(<file>)` and a call refuse (`NativePortStale`); `audit.location_key` (a file that changes for other reasons) is checked by the digest of its source when the classes bind.
"""

from __future__ import annotations

import dataclasses
import hashlib
import importlib
import inspect
from fractions import Fraction
from time import perf_counter_ns
from types import MappingProxyType

from . import codec, cost, pin
from .materialize_hot_approved_pins import APPROVED as _APPROVED

OPERATION = "materialize_hot"

#: The pins the operation answers under: exactly the gate-approved table (empty: `unavailable`).
APPROVED_PINS = MappingProxyType(_APPROVED)

#: `(module, function, sha256 of its source)`: functions of files the pin does not hold whole that the Rust code mirrors.
SHAPE_FUNCTIONS = (
    ("materialize/audit.py", "location_key", "c0b3a2846c5b4fe542bd2de580281527a613f7ae5243e376312a18ece1c30fe5"),
    # the port replaces the whole body of the oracle `domain._sourced` (lift at the host, the rebinding of the normals, the settle of the faces)
    ("materialize/domain.py", "_sourced", "d7065c4f855d2df5f687b47b77f18e57525071dcdcfc744c8677a3f0f528b811"),
    ("materialize/mesh.py", "decimal_of", "6de4873688c25968df0f5f2569c7fd34ceeae1e83edbd93387386b7e8bbcce7d"),
    ("ids.py", "OpaqueId.__post_init__", "e0f93f57af960b5533329678d2d2b0c2009e19689bd50ab4c013fe911c823bc3"),
)

#: The classes `assemble_batch` builds, in the order of `assemble_hot.rs` `CLASS_NAMES`: `(module, name)`.
ASSEMBLE_CLASSES = (
    ("ids", "SourceFaceId"), ("ids", "PhysicalEdgeId"), ("ids", "ChainUseId"), ("ids", "LineageId"), ("ids", "VertexKey"), ("ids", "SemanticLocationId"), ("ids", "GeometryFaceId"),
    ("ids", "SemanticRegionId"), ("ids", "OwnershipClaimId"), ("ids", "GeometryStationFactId"), ("ids", "SemanticBoundaryId"), ("ids", "SemanticInterfaceId"),
    ("contracts.geometry_batch", "GeometryProvenanceV1"), ("contracts.geometry_batch", "GeometryVertexV1"), ("contracts.geometry_batch", "GeometryUvFactV1"), ("numeric", "UvPoint2V1"),
    ("contracts.geometry_batch", "GeometryStationFactV1"), ("numeric", "LocalCoordinateV1"), ("contracts.geometry_batch", "GeometryFaceV1"), ("contracts.geometry_batch", "GeometrySemanticRegionV1"),
    ("contracts.geometry_batch", "GeometryBoundaryChainV1"), ("contracts.geometry_batch", "GeometryInterfaceChainV1"),
)

#: `mesh.DECIMAL_CONTEXT` as the Rust side divides (`cftuv_silhouette::assemble_law`): 28 digits, half even, exponent limits a station never reaches, the three traps.
DECIMAL_CONTEXT_SHAPE = (28, "ROUND_HALF_EVEN", -999999, 999999, 1, 0, ("DivisionByZero", "InvalidOperation", "Overflow"))

SILHOUETTE_INPUT_FIELDS = ("polygons", "cycles", "vertex_cycles", "positions", "points", "facts", "frame_faces", "layout", "lattice_alpha", "uv_slide", "reverse", "free")
SILHOUETTE_FIELDS = (
    "polygons", "cycles", "vertex_cycles", "positions", "facts", "merged_frames", "dissolved", "counters", "note", "changed", "runs", "fits", "boundary_chains", "interface_chains",
)
PLANE_FIELDS = ("origin", "basis_a", "basis_b", "scale")

_KERNEL: list = []


def refresh() -> None:
    """Forgets the verdict: the next call reads the pins again. The verdict lives in `pin`'s cache of verdicts, so `pin.refresh()` forgets it too."""

    pin._VERDICTS.pop(OPERATION, None)


def verdict() -> tuple:
    """`("unavailable",)` without approved pins, `("unsupported_python",)`, `("stale", files)` or `("available",)`; once per process (until `pin.refresh()`)."""

    found = pin._VERDICTS.get(OPERATION)
    if found is None:
        if not APPROVED_PINS:
            found = ("unavailable",)
        elif not pin.python_supported():
            found = ("unsupported_python",)
        else:
            root = pin.kernel_root()
            stale = tuple(name if pin.digest(root, name) is not None else f"{name} (missing)" for name, wanted in APPROVED_PINS.items() if pin.digest(root, name) != wanted)
            found = ("stale", stale) if stale else ("available",)
        pin._VERDICTS[OPERATION] = found
    return found


def status() -> str:
    found = verdict()
    return f"stale({', '.join(found[1])})" if found[0] == "stale" else found[0]


def _require() -> None:
    found = verdict()
    if found[0] == "unavailable":
        raise pin.NativePortStale("MATERIALIZE_HOT_PINS_NOT_APPROVED: the native materialize_hot has no approved pins (materialize_hot_approved_pins.APPROVED is empty)")
    if found[0] == "unsupported_python":
        raise pin.NativeUnsupportedPython(f"the native `{OPERATION}` port needs CPython {'.'.join(map(str, pin.MINIMUM_PYTHON))} or newer")
    if found[0] == "stale":
        raise pin.NativePortStale(f"the native `{OPERATION}` port was compared with another version of the Python oracle; changed since the pin: " + ", ".join(found[1]))


def _kernel():
    if not _KERNEL:
        from cftuv_envelope import exact_sqrt_sum
        from cftuv_envelope.materialize import assemble, audit, lift, silhouette
        from cftuv_envelope.numeric import LocalPoint3V1

        _KERNEL.append((silhouette, assemble, audit, lift, exact_sqrt_sum, LocalPoint3V1))
    return _KERNEL[0]


def source_digest(function) -> str:
    """sha256 of the source text of `function` (CRLF turned into LF)."""

    return hashlib.sha256(inspect.getsource(function).replace("\r\n", "\n").encode("utf-8")).hexdigest()


def _function(file: str, name: str):
    """The function `name` (a dotted path inside the module) of the kernel module `file` (`materialize/audit.py` -> `cftuv_envelope.materialize.audit`)."""

    found = importlib.import_module("cftuv_envelope." + file[: -len(".py")].replace("/", "."))
    for part in name.split("."):
        found = getattr(found, part)
    return found


def shape_digests() -> dict:
    found = {}
    for file, name, _wanted in SHAPE_FUNCTIONS:
        try:
            found[(file, name)] = source_digest(_function(file, name))
        except (AttributeError, ImportError, OSError, TypeError):
            found[(file, name)] = None
    return found


def _names(cls) -> tuple:
    return tuple(item.name for item in dataclasses.fields(cls))


def bind(mirror) -> None:
    """Hands the classes to the extension after checking the shapes the Rust side reads and builds (`NativePortStale` else)."""

    silhouette, assemble, _audit, lift, exact, point = _kernel()
    live = shape_digests()
    pin.check_shapes(
        (
            ("materialize/silhouette.py", "SilhouetteInputV1 fields", _names(silhouette.SilhouetteInputV1) == SILHOUETTE_INPUT_FIELDS),
            ("materialize/silhouette.py", "SilhouetteV1 fields", _names(silhouette.SilhouetteV1) == SILHOUETTE_FIELDS),
            ("materialize/lift.py", "PlaneLiftV1 fields", _names(lift.PlaneLiftV1) == PLANE_FIELDS),
            *((file, f"{name} is the function the Rust code mirrors", live[(file, name)] == wanted) for file, name, wanted in SHAPE_FUNCTIONS),
        )
    )
    import decimal

    from cftuv_envelope.materialize import mesh

    context = mesh.DECIMAL_CONTEXT
    shape = (context.prec, context.rounding, context.Emin, context.Emax, context.capitals, context.clamp, tuple(sorted(name.__name__ for name, on in context.traps.items() if on)))
    pin.check_shapes((("materialize/mesh.py", "DECIMAL_CONTEXT is the context the Rust division mirrors", shape == DECIMAL_CONTEXT_SHAPE),))
    table = []
    for module, name in ASSEMBLE_CLASSES:
        cls = getattr(importlib.import_module("cftuv_envelope." + module), name)
        table.append((name, cls, _names(cls)))
    try:
        mirror._session.bind_hot(exact.SqrtSumV1, Fraction, point)
        mirror._session.bind_assemble(table, decimal.Decimal, _metric_default())
    except TypeError as error:
        if "raw slot access was forced" in str(error):
            raise
        raise pin.NativePortStale(f"the oracle classes the native materialize_hot is built from changed: {error}") from None
    mirror._hot_bound = True


def _metric_default():
    """The default `metric_space` of `LocalCoordinateV1` (the oracle builds it with one argument)."""

    from cftuv_envelope.numeric import LocalCoordinateV1

    fields = dataclasses.fields(LocalCoordinateV1)
    if tuple(field.name for field in fields) != ("value", "metric_space") or fields[1].default is dataclasses.MISSING:
        raise pin.NativePortStale("LocalCoordinateV1 is not (value, metric_space=default) any more")
    return fields[1].default


def _begin(mirror):
    _require()
    if not getattr(mirror, "_hot_bound", False):
        bind(mirror)


def warm(mirror) -> bool:
    """Binds ahead of the first domain. Raises what a call would raise."""

    _begin(mirror)
    return True


def lift_vertices(mirror, points, plane):
    """`assemble.lift_vertices(points, plane)` for a `PlaneLiftV1`: `({key: position}, {key: (None, None)})`; any other lift, or an input the port does not carry, is declined."""

    _begin(mirror)
    _silhouette, _assemble, _audit, lift, _exact, _point = _kernel()
    if type(plane) is not lift.PlaneLiftV1:
        raise pin.NativePortUnsupported(f"lift_vertices: the port lifts on a PlaneLiftV1, not a {type(plane).__name__}")
    positions = mirror._session.hot_plane_lift(points, plane.origin, plane.basis_a, plane.basis_b, plane.scale)
    if positions is None:
        raise pin.NativePortUnsupported("lift_vertices: an input the port does not carry (a point that is not a pair of SqrtSumV1, a coordinate that is not a finite float, a zero scale)")
    return positions, dict.fromkeys(positions, (None, None))


def apply_silhouette(mirror, source, budget):
    """`silhouette.apply_silhouette(source, budget)`, whole, with its exact side effects (see the module note)."""

    started = perf_counter_ns()
    _begin(mirror)
    silhouette, assemble, audit, _lift, _exact, _point = _kernel()
    if budget is None:
        raise pin.NativePortUnsupported("apply_silhouette without a budget: the oracle reads `budget.cap`")
    mark = audit.CUT_RIGHT_COPY_MARK
    if not isinstance(mark, str) or not mark:
        raise pin.NativePortUnsupported("the copy mark of a cut vertex is not a non-empty str")
    layout, frame_faces = source.layout, source.frame_faces
    regions = [layout.region_of(frame_face) for frame_face in frame_faces]
    cut = [tuple(pair) for pair in layout.cut_pairs(frame_faces)]
    exact = cost._exact()
    sync, slow = mirror._sync_in(exact)
    real = (exact._KNOWN_PRIMES, exact._KNOWN_PRIME_SET, exact._FACTORIZATION_MEMO, exact._SQUAREFREE_MEMO, exact._PRIME_SUPPORT_MEMO)
    inner = budget.cap is not None
    state = (max(budget.remaining, 0), (0, 0, 0, 0, 0, 0)) if inner else (None, budget.spent_by_article())
    called = perf_counter_ns()
    try:
        result, status, detail, counts, articles, bits, native = mirror._session.hot_silhouette(
            source.polygons, source.cycles, source.positions, source.points, source.facts, regions, cut, mark, source.lattice_alpha, source.uv_slide, silhouette.CHORD_BUDGET,
            bool(source.reverse), source.free, None if sync is cost.UNCHANGED_SYNC else codec.encode_value(sync), state, real,
        )
    except BaseException:
        mirror.invalidate()
        raise
    returned = perf_counter_ns()
    if status not in cost.ORACLE_STATUSES or status != 0:
        mirror.invalidate()
        mirror.last_hot_timings = (called - started, returned - called, perf_counter_ns() - returned, perf_counter_ns() - started, *native)
        raise mirror._port_refusal(status, detail) if status not in cost.ORACLE_STATUSES else cost.NativeMirrorError(f"the native silhouette answered the outcome code {status}")
    mirror._settle_counts(exact, counts)
    kind = result[0]
    if kind == 0:
        if inner:
            budget.replay(tuple(articles))
        else:
            mirror._settle_articles(exact, budget, articles)
    mirror._remember(exact, bits, slow)
    shaped = _result(silhouette, assemble, source, result)
    mirror.last_hot_timings = (called - started, returned - called, perf_counter_ns() - returned, perf_counter_ns() - started, *native)
    problems = result[9]
    if problems:
        raise silhouette.MaterializationRefusal(silhouette.MaterializationOutcome.BATCH_DID_NOT_VALIDATE, "SILHOUETTE:" + ",".join(problems), shaped.counters)
    return shaped


def _chains(assemble, rows):
    if rows is None:
        return None, None
    boundary, interface = rows
    vertex = assemble.VertexKey
    return (
        frozenset(assemble.GeometryBoundaryChainV1(assemble.SemanticBoundaryId(text), tuple(vertex(key) for key in keys)) for text, keys in boundary),
        frozenset(assemble.GeometryInterfaceChainV1(assemble.SemanticInterfaceId(text), tuple(vertex(key) for key in keys)) for text, keys in interface),
    )


def _result(silhouette, assemble, source, result):
    """`SilhouetteV1` from the native answer, as the oracle builds it (`_Mesh.result`, `_unchanged`)."""

    kind, faces, gone, counters, note, changed, runs, fits, chains, _problems = result
    boundary, interface = _chains(assemble, chains)
    if kind != 0:
        return silhouette.SilhouetteV1(
            source.polygons, source.cycles, source.vertex_cycles, source.positions, source.facts, {}, frozenset(), counters, "", boundary_chains=boundary, interface_chains=interface,
        )
    polygons = [[] for _ in source.polygons]
    merged = {}
    for index, ring, others in faces:
        polygons[index].append(ring)
        if others:
            merged[(index, ring)] = others
    gone = frozenset(gone)
    return silhouette.SilhouetteV1(
        polygons,
        [[item for item in cycle if item[0] not in gone] for cycle in source.cycles],
        None if source.vertex_cycles is None else [[item for item in cycle if item[0] not in gone] for cycle in source.vertex_cycles],
        {key: value for key, value in source.positions.items() if key not in gone},
        {slot: value for slot, value in source.facts.items() if slot[1] not in gone},
        merged,
        gone,
        counters,
        note,
        changed,
        runs,
        tuple(fits),
        boundary,
        interface,
    )


def source_lift(mirror, plane, positions, triangles, host_positions, step, polygons, points, budget, reverse):
    """`materialize.domain._sourced(...)`: `lift_source_vertices`, `rebind_offset_normals` (the oracle's own function), `settle_emitted_faces`, whole, with the exact side effects of the
    triangulations. Returns `(faces, SourceLiftV1, SourceFacesV1)`; an exact outcome of the oracle in the settle (exhaustion, ...) is raised after the normals were rebound, as the oracle does."""

    started = perf_counter_ns()
    _begin(mirror)
    from cftuv_envelope.materialize import source_lift as law

    mark = law.CUT_RIGHT_COPY_MARK
    if not isinstance(mark, str) or not mark:
        raise pin.NativePortUnsupported("the copy mark of a cut vertex is not a non-empty str")
    exact = cost._exact()
    sync, slow = mirror._sync_in(exact)
    real = (exact._KNOWN_PRIMES, exact._KNOWN_PRIME_SET, exact._FACTORIZATION_MEMO, exact._SQUAREFREE_MEMO, exact._PRIME_SUPPORT_MEMO)
    state = None if budget is None else (budget.cap, budget.spent_by_article())
    called = perf_counter_ns()
    try:
        result, status, detail, counts, articles, bits, native = mirror._session.hot_source_lift(
            positions, triangles, host_positions, step, mark, polygons, points, bool(reverse), None if sync is cost.UNCHANGED_SYNC else codec.encode_value(sync), state, real,
        )
    except BaseException:
        mirror.invalidate()
        raise
    returned = perf_counter_ns()
    if status not in cost.ORACLE_STATUSES:
        mirror.invalidate()
        mirror.last_hot_timings = (called - started, returned - called, perf_counter_ns() - returned, perf_counter_ns() - started, *native)
        raise mirror._port_refusal(status, detail)
    mirror._settle_counts(exact, counts)
    mirror._settle_articles(exact, budget, articles)
    mirror._remember(exact, bits, slow)
    (final, (lifted, moved, displaced, unavailable, kept, total), (worst, max_lifted, metres, followed, drivers, max_follow)), faces = result
    sourced = law.SourceLiftV1(
        positions=final, lifted=lifted, moved=moved, displaced=displaced, unavailable=unavailable, kept_for_orientation=kept, total=total, worst_displaced=worst,
        max_lifted_displacement=max_lifted, budget=metres, followed=followed, followed_drivers=drivers, max_follow_displacement=max_follow,
    )
    law.rebind_offset_normals(plane, positions, sourced)
    mirror.last_hot_timings = (called - started, returned - called, perf_counter_ns() - returned, perf_counter_ns() - started, *native)
    if status != 0:
        cost.OpResult(status, None, detail, counts, articles, (), None).raise_for(budget)
        raise cost.NativeMirrorError(f"the native source lift answered the outcome code {status}")
    if faces is None:
        return polygons, sourced, law.SourceFacesV1(0.0, 0, 0)
    settled, worst_plane, cut, flipped = faces
    return settled, sourced, law.SourceFacesV1(worst_plane, cut, flipped)


def assemble_batch(mirror, *, frame_faces, cycles, positions, polygons, facts, layout, scale, lattice_alpha, edge_faces, request, source_revision, patch_domain_id, contract_versions,
                   diagnostics, vertex_cycles=None, merged_frames=None, chains=None):
    """`assemble.assemble_batch(...)`: the records natively (`assemble_hot.rs`), the diagnostics, the contract versions and the batch here, as the oracle builds them.

    The frame faces are read here (a few per domain: the region, the claim number, the lineage of the provenance, the physical edges, the chain uses, the run's
    lineage, the station model); a layout that is not the oracle's `Layout`, an attribute the frame face does not have, or anything the Rust side declines is
    `NativePortUnsupported` (the dispatcher runs the oracle on the same input; nothing was built that it keeps)."""

    started = perf_counter_ns()
    _begin(mirror)
    _silhouette, assemble, audit, _lift, _exact, _point = _kernel()
    if type(layout) is not assemble.Layout:
        raise pin.NativePortUnsupported(f"assemble_batch: the layout is a {type(layout).__name__}, not assemble.Layout")
    try:
        frames = [
            (layout.region_of(item), layout.claim[item.claim_key], (*item.run.lineage_ids, *assemble._claim_lineage(item)), tuple(item.physical_edge_ids), tuple(item.chain_use_ids),
             tuple(item.run.lineage_ids), item.station_model)
            for item in frame_faces
        ]
        cut = () if chains is not None else [tuple(pair) for pair in layout.cut_pairs(frame_faces)]
        material = assemble.MaterialId(request.material_policy_id.value)
    except Exception as error:  # noqa: BLE001 - the oracle meets the same input and names it itself
        raise pin.NativePortUnsupported(f"assemble_batch: a frame face the port does not read ({type(error).__name__}: {error})") from None
    called = perf_counter_ns()
    done, records = mirror._session.hot_assemble(
        frames, edge_faces, cycles, vertex_cycles, positions, polygons, facts, merged_frames, chains, cut, audit.CUT_RIGHT_COPY_MARK, lattice_alpha, scale, material,
    )
    returned = perf_counter_ns()
    if not done:
        raise pin.NativePortUnsupported(f"assemble_batch: {records}")
    vertices, faces, station_facts, regions, boundary, interface = records
    batch = assemble.GeometryBatchV1(
        schema_version=assemble.GEOMETRY_BATCH_SCHEMA_V1,
        source_revision=source_revision,
        decal_request_id=request.decal_request_id,
        patch_domain_id=patch_domain_id,
        vertices=vertices,
        faces=faces,
        station_facts=station_facts,
        semantic_regions=regions,
        boundary_chains=boundary,
        interface_chains=interface,
        diagnostics=frozenset(diagnostics()),
        contract_versions=frozenset(assemble.ContractVersionId(x) for x in contract_versions),
        semantic_digest=assemble.SemanticDigestValue(assemble.PENDING_SEMANTIC_DIGEST),
    )
    mirror.last_assemble_timings = (called - started, returned - called, perf_counter_ns() - returned)
    return batch