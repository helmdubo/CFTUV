"""`CHAIN_STATION_PLAN_V1` as a native operation: `chain_station_plans(snapshot, patch_domain_id)`, the same frozenset of `ChainStationPlanV1` the Python law answers.

The Python law (`cftuv_envelope._chain_station.chain_station_plans`) stays the ORACLE: the Rust mirror (`native/cftuv-station`, read by `cftuv-python/src/chain_station.rs`) answers only what
it computes bit for bit, and DECLINES by name (`NativePortUnsupported`, before anything changed) every input it does not hold and every input on which the oracle would raise; the caller then
runs the oracle, with its answer or its exception. The operation has no state and no counters (the law has none).

What crosses the boundary: the snapshot graph, read slot by slot from exactly the contract classes; the chord budget and the bend limit, read LIVE from the law on every call (they are
inputs, not pinned constants); and back the codes of each station, with the chain records and face id objects of the snapshot itself. The records (`ChainStationV1`,
`ChainStationPlanV1`) are built here from the live contract classes, and `FREE_UNDER_CHART` is named by the law's own `_chart_is_developed`.

PINS: a candidate awaiting the owner. The mirrored files are `_chain_station.py` (the law, whole) and `contracts/chain_station.py` (the records and the enum members the codes name).
`CANDIDATE` holds their digests with `approved: False`; while it is not approved the stage is `unavailable` and a call refuses (`NativePortStale`), whatever the files say. Approving is
the owner's step (flip `approved`); nothing here approves. A test harness may install the candidate in memory (`APPROVED_PINS`) for an A/B: that changes no file and no build id.
"""

from __future__ import annotations

import importlib
import sys

from . import pin

OPERATION = "chain_station"

#: The oracle files the Rust code mirrors and their sha256 (CRLF turned into LF), compared on 61e46e89. `approved: False` - awaiting the owner (R3+ S2).
CANDIDATE = {
    "approved": False,
    "files": {
        "_chain_station.py": "ee1394436daf4539658b63fcdba8b1d5e629fd8cc51f704d797f2a8868cb2c21",
        "contracts/chain_station.py": "7c74d5446e2aded5af0d711885fec8218dfaa5de9ef8adf9b7074afda1d47e8e",
    },
}

#: The pins the stage answers under: the candidate's files once the owner approved it, empty otherwise (empty: `unavailable`).
APPROVED_PINS: dict = dict(CANDIDATE["files"]) if CANDIDATE["approved"] else {}

#: The status codes of the extension's answer (`chain_station.rs`).
STATUS_PLANS = 0
STATUS_DECLINED = 2

#: `cftuv_station::{StationKind, Disposition, Reason}` codes -> member names of the contract enums.
KIND_NAMES = ("INTERIOR_VERTEX", "CHAIN_JOINT")
DISPOSITION_NAMES = ("FREE", "REQUIRED")
REASON_NAMES = (
    "TRANSVERSE_EDGES_INERT",
    "FREE_UNDER_CHART",
    "FOLD",
    "GROUP_NOT_FLAT",
    "BEND_BEYOND_STRAIGHT",
    "CHORD_BEYOND_BUDGET",
    "RUN_CHORD_BEYOND_BUDGET",
    "JUNCTION",
    "NEIGHBOUR_SIDE_UNKNOWN",
    "NOT_MANIFOLD",
    "POSITIONS_UNAVAILABLE",
    "CLOSED_CHAIN",
)

#: The classes the extension reads, by module.
CLASS_NAMES = {
    "cftuv_envelope.ids": ("SourceVertexId", "SourceFaceId", "PatchId", "PhysicalChainId", "PatchDomainId"),
    "cftuv_envelope.contracts.analysis": ("SourceVertexV1", "PhysicalChainV1", "PhysicalChainKind", "SeamNeighbourFaceV1"),
    "cftuv_envelope.contracts.surface": ("SourceFaceV1", "SurfacePayloadMode"),
    "cftuv_envelope.numeric": ("LocalPoint3V1", "UnavailableSourcePositionV1"),
}
LAW_MODULE = "cftuv_envelope._chain_station"
RECORDS_MODULE = "cftuv_envelope.contracts.chain_station"

_VERDICT: list = []
_KERNEL: list = []


def refresh() -> None:
    """Forgets the verdict and the binding: the next call reads the pins and binds again (the harness that installs pins calls it)."""

    _VERDICT.clear()
    _KERNEL.clear()


def verdict() -> tuple:
    """`("unavailable",)` without approved pins, `("unsupported_python",)`, `("stale", files)` or `("available",)`; once per process."""

    if not _VERDICT:
        if not APPROVED_PINS:
            found = ("unavailable",)
        elif not pin.python_supported():
            found = ("unsupported_python",)
        else:
            root = pin.kernel_root()
            stale = tuple(name if pin.digest(root, name) is not None else f"{name} (missing)" for name, wanted in APPROVED_PINS.items() if pin.digest(root, name) != wanted)
            found = ("stale", stale) if stale else ("available",)
        _VERDICT.append(found)
    return _VERDICT[0]


def status() -> str:
    found = verdict()
    return f"stale({', '.join(found[1])})" if found[0] == "stale" else found[0]


def _require() -> None:
    found = verdict()
    if found[0] == "unavailable":
        raise pin.NativePortStale("CHAIN_STATION_PINS_NOT_APPROVED: the pin candidate of the native chain_station awaits the owner (chain_station_op.CANDIDATE)")
    if found[0] == "unsupported_python":
        raise pin.NativeUnsupportedPython(f"the native `{OPERATION}` port needs CPython {'.'.join(map(str, pin.MINIMUM_PYTHON))} or newer")
    if found[0] == "stale":
        raise pin.NativePortStale(f"the native `{OPERATION}` port was compared with another version of the Python oracle; changed since the pin: " + ", ".join(found[1]))


def _modules() -> tuple:
    return tuple(sys.modules.get(name) for name in (*CLASS_NAMES, LAW_MODULE, RECORDS_MODULE))


def _bind(core) -> tuple:
    table = {}
    for module_name, names in CLASS_NAMES.items():
        module = importlib.import_module(module_name)
        for name in names:
            table[name] = getattr(module, name)
    law = importlib.import_module(LAW_MODULE)
    records = importlib.import_module(RECORDS_MODULE)
    try:
        handle = core.ChainStation(table)
        kinds = tuple(records.ChainStationKindV1[name] for name in KIND_NAMES)
        dispositions = tuple(records.ChainStationDispositionV1[name] for name in DISPOSITION_NAMES)
        reasons = tuple(records.ChainStationReasonV1[name] for name in REASON_NAMES)
    except (TypeError, KeyError) as error:
        if "raw slot access was forced" in str(error):
            raise
        raise pin.NativePortStale(f"the oracle classes the native chain_station is built from changed: {error}") from None
    if len(records.ChainStationReasonV1) != len(REASON_NAMES) or len(records.ChainStationKindV1) != len(KIND_NAMES) or len(records.ChainStationDispositionV1) != len(DISPOSITION_NAMES):
        raise pin.NativePortStale("the chain station enums have members the native port does not name")
    return handle, law, kinds, dispositions, reasons, records.ChainStationV1, records.ChainStationPlanV1, table["SurfacePayloadMode"].FULL_HOST_SURFACE


def _handle(core) -> tuple:
    """The binding to the live kernel: a rebind when the kernel modules were reloaded."""

    live = _modules()
    if _KERNEL and _KERNEL[0][0] is core and None not in live and all(a is b for a, b in zip(_KERNEL[0][1], live)):
        return _KERNEL[0][2]
    bound = _bind(core)
    _KERNEL[:] = [(core, _modules(), bound)]
    return bound


def chain_station_plans(core, snapshot, patch_domain_id) -> frozenset:
    """The plans of `chain_station_plans(snapshot, patch_domain_id)`; `NativePortUnsupported` naming why the port declined, `NativePortStale` without approved pins."""

    _require()
    handle, law, kinds, dispositions, reasons, station_class, plan_class, full_mode = _handle(core)
    budget, bend = law.flatness_budget(), law.BEND_LIMIT
    surface = snapshot.surface_ir
    # the oracle's own expressions (`StationFacts.__init__`, `chain_station_plans`): equality of ids and the payload mode stay Python
    full = surface.payload_mode is full_mode
    corner_vertices = tuple(item.source_vertex_id for relations in (snapshot.corner_relations, snapshot.junction_relations) for item in relations)
    domain_chains = tuple(item.physical_chain_id for item in snapshot.chain_uses if item.patch_domain_id == patch_domain_id)
    answer = handle.plans(
        full,
        snapshot.source_vertices,
        surface.source_faces,
        snapshot.seam_neighbour_faces,
        corner_vertices,
        snapshot.physical_chains,
        domain_chains,
        (budget.numerator, budget.denominator),
        (bend.numerator, bend.denominator),
        float(budget * budget),
    )
    if answer[0] != STATUS_PLANS:
        raise pin.NativePortUnsupported(f"chain_station declined: {answer[1]}")
    inert = reasons[0]
    named = reasons[1] if law._chart_is_developed(snapshot, patch_domain_id) else inert
    plans = []
    for chain, rows in answer[1]:
        vertices = chain.ordered_source_vertex_ids
        stations = tuple(
            station_class(vertices[ordinal], ordinal, kinds[kind], dispositions[disposition], named if reason == 0 else reasons[reason], pairs)
            for ordinal, kind, disposition, reason, pairs in rows
        )
        plans.append(plan_class(law.LAW, chain.physical_chain_id, law.budget_record(), stations))
    return frozenset(plans)


def warm(core) -> bool:
    """Binds ahead of the first domain (CHAIN_STATION_BIND_AT_WORKER_START_V1). Raises what a call would raise."""

    _require()
    _handle(core)
    return True
