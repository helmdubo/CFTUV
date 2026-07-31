"""Request export and reference evaluation for Envelope debug.

The public compatibility facade lives in :mod:`envelope_host_adapter`.
Topology, metric and session lifecycle entry points are layered around this
module without changing the accepted Envelope semantics.
"""

from __future__ import annotations

import hashlib
import importlib
import inspect
import json
import math
import time
from contextlib import nullcontext
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from enum import Enum
from typing import TYPE_CHECKING

from .model import ChainNeighborKind, LoopKind, PatchType
from .surface_ir import HOST_GRID_POLICY, HOST_PLANARITY_POLICY
from .envelope_request_policy import (
    build_envelope_request_contract,
    envelope_angular_policy,
    envelope_decal_request_id_value,
)
from .envelope_angle_certificate import (
    SnapshotExportRejected,
    certified_reflex_measure,
    validate_exported_snapshot,
)
from .envelope_debug_profile import (
    ANGULAR_STAGE_COUNTERS,
    EnvelopeDebugProfileBuilderV1,
    EnvelopeDebugTimingV1,
    EnvelopeDomainStage,
    EnvelopeDomainStageReceiptV1,
)
from .envelope_topology_debug import (
    ENVELOPE_TOPOLOGY_DEBUG_SCENE_SCHEMA_V1,
    EnvelopeTopologyDebugPairV1,
    EnvelopeTopologyDebugPathV1,
    EnvelopeTopologyDebugSceneV1,
    EnvelopeTopologyPairKind,
    EnvelopeTopologyPathKind,
    EnvelopeTopologySelectionDiagnosticV1,
)

if TYPE_CHECKING:
    import cftuv_envelope as envelope_kernel

    from .surface_ir import AnalysisBundle


class EnvelopeDebugHostOutcome(str, Enum):
    EXACT = "EXACT"
    ENVELOPE_DEBUG_KERNEL_UNAVAILABLE = "ENVELOPE_DEBUG_KERNEL_UNAVAILABLE"
    ENVELOPE_DEBUG_SYMPY_VERSION_UNSUPPORTED = (
        "ENVELOPE_DEBUG_SYMPY_VERSION_UNSUPPORTED"
    )
    ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID = (
        "ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID"
    )
    ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE = (
        "ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE"
    )
    RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED = (
        "RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED"
    )
    # Отказы плоскости и решётки выходят каждый под СВОИМ именем. Прежде хост
    # сводил их все к `RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED`, и
    # полевое сообщение объявляло «нужна near-planar политика» в тот момент,
    # когда она уже была включена, а отказал бюджет либо сам снап. Имя,
    # указывающее не на ту причину, дороже отсутствующего.
    NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED = (
        "NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED"
    )
    GRID_SNAP_LEFT_THE_PATCH_PLANE = "GRID_SNAP_LEFT_THE_PATCH_PLANE"
    GRID_WINDOW_CLOSED = "GRID_WINDOW_CLOSED"
    NO_POWER_OF_TWO_STEP_IN_WINDOW = "NO_POWER_OF_TWO_STEP_IN_WINDOW"
    NO_GRID_SCALE_RESTORES_RELATIONS = "NO_GRID_SCALE_RESTORES_RELATIONS"
    ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE = (
        "ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE"
    )
    ENVELOPE_DEBUG_MULTIPLE_ANGULAR_RELATIONS_PER_CHAIN_UNSUPPORTED = (
        "ENVELOPE_DEBUG_MULTIPLE_ANGULAR_RELATIONS_PER_CHAIN_UNSUPPORTED"
    )
    ENVELOPE_DEBUG_PHYSICAL_CHAIN_INVALID = (
        "ENVELOPE_DEBUG_PHYSICAL_CHAIN_INVALID"
    )
    ENVELOPE_DEBUG_CHAIN_USE_PAIR_UNAVAILABLE = (
        "ENVELOPE_DEBUG_CHAIN_USE_PAIR_UNAVAILABLE"
    )
    ENVELOPE_DEBUG_SELF_SEAM_USE_PAIR_UNAVAILABLE = (
        "ENVELOPE_DEBUG_SELF_SEAM_USE_PAIR_UNAVAILABLE"
    )
    ENVELOPE_DEBUG_EMPTY_SELECTION = "ENVELOPE_DEBUG_EMPTY_SELECTION"
    ENVELOPE_DEBUG_SELECTED_EDGE_UNKNOWN = (
        "ENVELOPE_DEBUG_SELECTED_EDGE_UNKNOWN"
    )
    ENVELOPE_DEBUG_PARTIAL_CHAIN_SELECTION_UNSUPPORTED = (
        "ENVELOPE_DEBUG_PARTIAL_CHAIN_SELECTION_UNSUPPORTED"
    )
    ENVELOPE_DEBUG_SELECTED_EDGE_OFF_PHYSICAL_CHAIN = (
        "ENVELOPE_DEBUG_SELECTED_EDGE_OFF_PHYSICAL_CHAIN"
    )
    ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED = (
        "ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED"
    )


class EnvelopeDebugHostSeverity(str, Enum):
    INFO = "INFO"
    UNSUPPORTED = "UNSUPPORTED"
    ERROR = "ERROR"


@dataclass(frozen=True, slots=True)
class EnvelopeDebugHostDiagnosticV1:
    outcome: EnvelopeDebugHostOutcome | str
    severity: EnvelopeDebugHostSeverity
    message: str
    patch_domain_id: str | None = None


@dataclass(frozen=True, slots=True)
class EnvelopeDebugEvaluationV1:
    snapshot: envelope_kernel.AnalysisSnapshotV1 | None
    request: envelope_kernel.DecalRequestV1 | None
    compilations: tuple[envelope_kernel.ReferenceEnvelopeCompilationV1, ...]
    raw_results: tuple[envelope_kernel.RawCoverageResultV1, ...]
    interaction_results: tuple[envelope_kernel.InteractionResolutionResultV1, ...]
    debug_scene: envelope_kernel.EnvelopeDebugSceneV1 | None
    diagnostics: tuple[EnvelopeDebugHostDiagnosticV1, ...]
    timings: tuple[EnvelopeDebugTimingV1, ...]


@dataclass(frozen=True, slots=True)
class EnvelopeDebugDomainEvaluationV1:
    patch_id: int
    patch_domain_id: str
    snapshot: envelope_kernel.AnalysisSnapshotV1 | None
    request: envelope_kernel.DecalRequestV1 | None
    compilation: envelope_kernel.ReferenceEnvelopeCompilationV1 | None
    raw_result: envelope_kernel.RawCoverageResultV1 | None
    interaction_result: (
        envelope_kernel.InteractionResolutionResultV1 | None
    )
    debug_scene: envelope_kernel.EnvelopeDebugSceneV1 | None
    receipt: EnvelopeDomainStageReceiptV1
    diagnostics: tuple[EnvelopeDebugHostDiagnosticV1, ...]
    # Результат движка QUEUE. `None` на движке LEGACY — и это не «нет данных»,
    # а «этот путь очередь не проходил».
    queue: object | None = None


@dataclass(frozen=True, slots=True)
class EnvelopeDebugStagedEvaluationV1:
    topology_scene: EnvelopeTopologyDebugSceneV1
    domains: tuple[EnvelopeDebugDomainEvaluationV1, ...]

    @property
    def receipts(self) -> tuple[EnvelopeDomainStageReceiptV1, ...]:
        return tuple(item.receipt for item in self.domains)

    @property
    def queue_domains(self) -> tuple:
        return tuple(
            item.queue for item in self.domains if item.queue is not None
        )

    @property
    def exact_debug_scenes(
        self,
    ) -> tuple[envelope_kernel.EnvelopeDebugSceneV1, ...]:
        return tuple(
            item.debug_scene
            for item in self.domains
            if item.debug_scene is not None
        )

    @property
    def diagnostics(self) -> tuple[EnvelopeDebugHostDiagnosticV1, ...]:
        return tuple(
            diagnostic
            for item in self.domains
            for diagnostic in item.diagnostics
        )


class EnvelopeHostAdapterError(RuntimeError):
    def __init__(
        self,
        outcome: EnvelopeDebugHostOutcome,
        message: str,
        *,
        patch_domain_id: str | None = None,
    ):
        self.outcome = outcome
        self.patch_domain_id = patch_domain_id
        super().__init__(message)

    def diagnostic(self) -> EnvelopeDebugHostDiagnosticV1:
        return EnvelopeDebugHostDiagnosticV1(
            self.outcome,
            EnvelopeDebugHostSeverity.UNSUPPORTED,
            str(self),
            self.patch_domain_id,
        )


@dataclass(frozen=True, slots=True)
class _HostChainSlice:
    neighbor_kind: ChainNeighborKind
    neighbor_patch_id: int
    is_closed: bool
    vert_indices: tuple[int, ...]
    edge_indices: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class _HostChainRecord:
    patch_id: int
    loop_index: int
    chain_index: int
    source_chain_index: int
    source_segment_index: int
    chain: object
    canonical_vertex_ids: tuple[int, ...]
    canonical_edge_ids: tuple[int, ...]
    reversed_from_canonical: bool


def _measure(profile, stage: str, patch_domain_id: str | None = None):
    if profile is None:
        return nullcontext()
    return profile.measure(stage, patch_domain_id)


def _load_kernel():
    try:
        kernel = importlib.import_module("cftuv_envelope")
    except ModuleNotFoundError as exc:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_KERNEL_UNAVAILABLE,
            f"cftuv-envelope-core is unavailable: {exc}",
        ) from exc
    try:
        sympy = importlib.import_module("sympy")
    except ModuleNotFoundError as exc:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_KERNEL_UNAVAILABLE,
            f"SymPy is unavailable: {exc}",
        ) from exc
    if sympy.__version__ != "1.14.0":
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_SYMPY_VERSION_UNSUPPORTED,
            f"exact Envelope debug requires sympy==1.14.0, found {sympy.__version__}",
        )
    return kernel, sympy


def _stable_token(kind: str, revision: str, *parts: object) -> str:
    payload = json.dumps(
        (kind, revision, parts),
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:24]


def _typed_value(kind: str, revision: str, *parts: object) -> str:
    return f"host-v0:{kind}:{_stable_token(kind, revision, *parts)}"


def _revision_value(source_revision) -> str:
    source_name = str(source_revision.source_name).strip()
    digest = str(source_revision.digest).strip()
    if not source_name or not digest:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
            "AnalysisBundle SourceRevision requires non-empty source_name and digest",
        )
    return f"host-source:{digest}:{source_name}"


def _vector3(value) -> tuple[float, float, float]:
    result = tuple(float(item) for item in value)
    if len(result) != 3 or not all(math.isfinite(item) for item in result):
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
            "host vector must contain three finite components",
        )
    return result


def _canonical_chain(chain) -> tuple[tuple[int, ...], tuple[int, ...], bool]:
    vertices = tuple(int(item) for item in chain.vert_indices)
    edges = tuple(int(item) for item in chain.edge_indices)
    is_closed = bool(chain.is_closed)
    if is_closed and len(vertices) > 1 and vertices[0] == vertices[-1]:
        vertices = vertices[:-1]
    expected_edges = len(vertices) if is_closed else len(vertices) - 1
    if len(vertices) < 2 or len(edges) != expected_edges:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PHYSICAL_CHAIN_INVALID,
            "host BoundaryChain does not satisfy open E=V-1 or closed E=V",
        )
    if is_closed:
        candidates = []
        for start in range(len(vertices)):
            forward_vertices = vertices[start:] + vertices[:start]
            forward_edges = edges[start:] + edges[:start]
            candidates.append(
                ((forward_edges, forward_vertices), forward_vertices, forward_edges, False)
            )
            reverse_vertices = tuple(
                vertices[(start - offset) % len(vertices)]
                for offset in range(len(vertices))
            )
            reverse_edges = tuple(
                edges[(start - offset - 1) % len(edges)]
                for offset in range(len(edges))
            )
            candidates.append(
                ((reverse_edges, reverse_vertices), reverse_vertices, reverse_edges, True)
            )
        _, canonical_vertices, canonical_edges, reversed_from_canonical = min(
            candidates, key=lambda item: item[0]
        )
        return canonical_vertices, canonical_edges, reversed_from_canonical
    forward = (edges, vertices)
    reverse = (tuple(reversed(edges)), tuple(reversed(vertices)))
    if reverse < forward:
        return reverse[1], reverse[0], True
    return vertices, edges, False


def _host_chain_slice(chain) -> _HostChainSlice:
    return _HostChainSlice(
        chain.neighbor_kind,
        int(chain.neighbor_patch_id),
        bool(chain.is_closed),
        tuple(int(item) for item in chain.vert_indices),
        tuple(int(item) for item in chain.edge_indices),
    )


def _split_host_chain_slice(
    chain: _HostChainSlice,
    cut_vertex_ids: frozenset[int],
) -> tuple[_HostChainSlice, ...]:
    vertices = chain.vert_indices
    edges = chain.edge_indices
    if not chain.is_closed:
        cut_indices = (
            0,
            *(
                index
                for index in range(1, len(vertices) - 1)
                if vertices[index] in cut_vertex_ids
            ),
            len(vertices) - 1,
        )
        return tuple(
            _HostChainSlice(
                chain.neighbor_kind,
                chain.neighbor_patch_id,
                False,
                vertices[start : end + 1],
                edges[start:end],
            )
            for start, end in zip(
                cut_indices[:-1],
                cut_indices[1:],
                strict=True,
            )
        )

    cut_indices = tuple(
        index
        for index, vertex_id in enumerate(vertices)
        if vertex_id in cut_vertex_ids
    )
    if not cut_indices:
        return (chain,)
    if len(cut_indices) < 2:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_CHAIN_USE_PAIR_UNAVAILABLE,
            "closed physical seam has fewer than two exact partition endpoints",
        )
    pieces = []
    vertex_count = len(vertices)
    for start, end in zip(
        cut_indices,
        cut_indices[1:] + (cut_indices[0] + vertex_count,),
        strict=True,
    ):
        piece_vertices = tuple(
            vertices[index % vertex_count]
            for index in range(start, end + 1)
        )
        piece_edges = tuple(
            edges[index % vertex_count]
            for index in range(start, end)
        )
        pieces.append(
            _HostChainSlice(
                chain.neighbor_kind,
                chain.neighbor_patch_id,
                False,
                piece_vertices,
                piece_edges,
            )
        )
    return tuple(pieces)


def _normalize_physical_seam_partitions(
    raw_records: tuple[_HostChainRecord, ...],
) -> tuple[_HostChainRecord, ...]:
    """Apply the exact common refinement declared by both patch-side uses."""

    records_by_pair: dict[tuple[int, int], list[_HostChainRecord]] = {}
    for record in raw_records:
        if record.chain.neighbor_kind is not ChainNeighborKind.PATCH:
            continue
        pair = tuple(
            sorted((record.patch_id, int(record.chain.neighbor_patch_id)))
        )
        records_by_pair.setdefault(pair, []).append(record)

    cut_vertices_by_pair: dict[tuple[int, int], frozenset[int]] = {}
    for pair, records in records_by_pair.items():
        records_by_side = {
            patch_id: [
                record for record in records if record.patch_id == patch_id
            ]
            for patch_id in pair
        }
        # Request-scoped snapshots may intentionally omit the opposite domain.
        if any(not side_records for side_records in records_by_side.values()):
            continue
        edge_sets_by_side = {}
        for patch_id, side_records in records_by_side.items():
            ordered_edges = [
                edge_id
                for record in side_records
                for edge_id in record.chain.edge_indices
            ]
            if len(ordered_edges) != len(set(ordered_edges)):
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_CHAIN_USE_PAIR_UNAVAILABLE,
                    "one patch-side seam partition repeats a physical edge",
                )
            edge_sets_by_side[patch_id] = frozenset(ordered_edges)
        if len(set(edge_sets_by_side.values())) != 1:
            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_CHAIN_USE_PAIR_UNAVAILABLE,
                "patch-side seam partitions cover different physical edges",
            )
        cut_vertices_by_pair[pair] = frozenset(
            vertex_id
            for record in records
            if not record.chain.is_closed
            for vertex_id in (
                record.chain.vert_indices[0],
                record.chain.vert_indices[-1],
            )
        )

    expanded = []
    for record in raw_records:
        pair = (
            tuple(
                sorted((record.patch_id, int(record.chain.neighbor_patch_id)))
            )
            if record.chain.neighbor_kind is ChainNeighborKind.PATCH
            else None
        )
        pieces = _split_host_chain_slice(
            record.chain,
            cut_vertices_by_pair.get(pair, frozenset()),
        )
        for segment_index, piece in enumerate(pieces):
            vertices, edges, reversed_from_canonical = _canonical_chain(piece)
            expanded.append(
                _HostChainRecord(
                    record.patch_id,
                    record.loop_index,
                    record.chain_index,
                    record.source_chain_index,
                    segment_index,
                    piece,
                    vertices,
                    edges,
                    reversed_from_canonical,
                )
            )

    normalized = []
    for patch_loop in sorted(
        {(record.patch_id, record.loop_index) for record in expanded}
    ):
        loop_records = sorted(
            (
                record
                for record in expanded
                if (record.patch_id, record.loop_index) == patch_loop
            ),
            key=lambda record: (
                record.source_chain_index,
                record.source_segment_index,
            ),
        )
        for chain_index, record in enumerate(loop_records):
            normalized.append(
                _HostChainRecord(
                    record.patch_id,
                    record.loop_index,
                    chain_index,
                    record.source_chain_index,
                    record.source_segment_index,
                    record.chain,
                    record.canonical_vertex_ids,
                    record.canonical_edge_ids,
                    record.reversed_from_canonical,
                )
            )
    return tuple(normalized)


def _collect_raw_host_chains(
    analysis_bundle: AnalysisBundle,
) -> tuple[_HostChainRecord, ...]:
    records = []
    for patch_id, patch in sorted(analysis_bundle.patch_graph.nodes.items()):
        for loop_index, loop in enumerate(patch.boundary_loops):
            for chain_index, chain in enumerate(loop.chains):
                chain_slice = _host_chain_slice(chain)
                vertices, edges, reversed_from_canonical = _canonical_chain(
                    chain_slice
                )
                records.append(
                    _HostChainRecord(
                        int(patch_id),
                        loop_index,
                        chain_index,
                        chain_index,
                        0,
                        chain_slice,
                        vertices,
                        edges,
                        reversed_from_canonical,
                    )
                )
    if not records:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
            "AnalysisBundle contains no BoundaryChain records",
        )
    return tuple(records)


def _collect_host_chains(
    analysis_bundle: AnalysisBundle,
    *,
    profile: EnvelopeDebugProfileBuilderV1 | None = None,
    patch_domain_id: str | None = None,
) -> tuple[_HostChainRecord, ...]:
    with _measure(profile, "HOST_CHAIN_COLLECTION", patch_domain_id):
        records = _collect_raw_host_chains(analysis_bundle)
    with _measure(
        profile,
        "SEAM_PARTITION_NORMALIZATION",
        patch_domain_id,
    ):
        return _normalize_physical_seam_partitions(records)


def _host_chain_key(
    record: _HostChainRecord,
) -> tuple[bool, tuple[int, ...], tuple[int, ...]]:
    return (
        bool(record.chain.is_closed),
        record.canonical_edge_ids,
        record.canonical_vertex_ids,
    )


def _group_host_chains(
    host_chains: tuple[_HostChainRecord, ...],
) -> dict[tuple[bool, tuple[int, ...], tuple[int, ...]], list[_HostChainRecord]]:
    groups: dict[
        tuple[bool, tuple[int, ...], tuple[int, ...]],
        list[_HostChainRecord],
    ] = {}
    for record in host_chains:
        groups.setdefault(_host_chain_key(record), []).append(record)
    return groups


def _validate_chain_group_topology(
    records: list[_HostChainRecord],
) -> None:
    neighbor_kinds = {record.chain.neighbor_kind for record in records}
    patch_group = {record.patch_id for record in records}
    if len(neighbor_kinds) != 1:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PHYSICAL_CHAIN_INVALID,
            "one canonical PhysicalChain has conflicting host neighbor kinds",
        )
    neighbor_kind = next(iter(neighbor_kinds))
    if neighbor_kind is ChainNeighborKind.SEAM_SELF:
        if len(records) != 2 or len(patch_group) != 1:
            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_SELF_SEAM_USE_PAIR_UNAVAILABLE,
                "SEAM_SELF requires exactly two host ChainUses in one PatchDomain",
            )
    elif neighbor_kind is ChainNeighborKind.PATCH:
        if len(records) != 2 or len(patch_group) != 2:
            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_CHAIN_USE_PAIR_UNAVAILABLE,
                "physical seam requires two exact whole-chain patch-side uses",
            )
    elif len(records) != 1:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PHYSICAL_CHAIN_INVALID,
            "ordinary host boundary source requires exactly one ChainUse",
        )


def _selected_scope(
    analysis_bundle: AnalysisBundle,
    selected_physical_edge_ids: frozenset[int],
):
    """Resolve whole-chain selection before any per-domain metric admission."""

    from .envelope_topology_export import resolve_selection_scope

    return resolve_selection_scope(
        analysis_bundle,
        selected_physical_edge_ids,
        _collect_host_chains(analysis_bundle),
    )


def _host_local_points(
    vertex_ids: tuple[int, ...],
    host_vertex_by_id,
) -> tuple[tuple[float, float, float], ...]:
    try:
        return tuple(
            _vector3(host_vertex_by_id[vertex_id].position)
            for vertex_id in vertex_ids
        )
    except KeyError as exc:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
            f"host topology references unknown source vertex {exc.args[0]}",
        ) from exc


def build_envelope_topology_debug_scene(
    analysis_bundle: AnalysisBundle,
    selected_physical_edge_ids: frozenset[int],
    *,
    profile: EnvelopeDebugProfileBuilderV1 | None = None,
    topology_export=None,
) -> EnvelopeTopologyDebugSceneV1:
    """Project request-scoped host topology without loading kernel or SymPy."""

    if topology_export is None:
        revision = _revision_value(analysis_bundle.source_revision)
        with _measure(profile, "HOST_CHAIN_COLLECTION"):
            raw_host_chains = _collect_raw_host_chains(analysis_bundle)
        with _measure(profile, "SEAM_PARTITION_NORMALIZATION"):
            host_chains = _normalize_physical_seam_partitions(raw_host_chains)
    else:
        revision = topology_export.source_revision_value
        host_chains = topology_export.host_chains
    from .envelope_topology_export import (
        SELECTION_COMPLETED_DIAGNOSTIC_CODE,
        resolve_selection_scope,
    )

    with _measure(profile, "SELECTION_SCOPE"):
        scope = resolve_selection_scope(
            analysis_bundle,
            selected_physical_edge_ids,
            host_chains,
            profile=profile,
        )
    selected_keys = scope.chain_keys
    selected_patch_ids = scope.patch_ids

    domain_by_patch = {
        patch_id: _typed_value("patch-domain", revision, patch_id)
        for patch_id in sorted(selected_patch_ids)
    }
    host_vertex_by_id = analysis_bundle.patch_surface.vertex_by_id
    chain_groups = _group_host_chains(host_chains)
    paths: list[EnvelopeTopologyDebugPathV1] = []
    pairs: list[EnvelopeTopologyDebugPairV1] = []
    selection_diagnostics = []

    # Дополнение выделения объявляется сценой, а не только счётчиком: владелец
    # должен увидеть в sidecar, КАКАЯ цепочка была достроена и какими рёбрами.
    for key in scope.completed_chain_keys:
        selection_diagnostics.append(
            EnvelopeTopologySelectionDiagnosticV1(
                SELECTION_COMPLETED_DIAGNOSTIC_CODE,
                "Partial selection completed to the whole PhysicalChain; "
                f"added host edges "
                f"{sorted(frozenset(key[1]) - scope.requested_edge_ids)}",
                physical_chain_id=_typed_value(
                    "physical-chain",
                    revision,
                    key[0],
                    key[1],
                    key[2],
                ),
            )
        )

    if profile is not None:
        profile.set_counter(
            "MESH_FACES",
            len(analysis_bundle.patch_surface.faces),
        )
        profile.set_counter(
            "MESH_EDGES",
            len(analysis_bundle.patch_surface.edges),
        )
        profile.set_counter(
            "PATCH_COUNT",
            len(analysis_bundle.patch_graph.nodes),
        )
        profile.set_counter("SELECTED_CHAINS", len(selected_keys))
        profile.set_counter(
            "SELECTED_DOMAINS",
            len(selected_patch_ids),
        )

    with _measure(profile, "TOPOLOGY_SCENE"):
        for patch_id in sorted(selected_patch_ids):
            patch = analysis_bundle.patch_graph.nodes[patch_id]
            domain_id = domain_by_patch[patch_id]
            normal = _vector3(patch.normal)
            patch_faces = analysis_bundle.patch_surface.patch_faces(patch_id)
            patch_edge_ids = {
                int(edge_id)
                for face in patch_faces
                for edge_id in face.edge_cycle
            }
            if profile is not None:
                profile.set_counter(
                    "FACES_PER_DOMAIN",
                    len(patch_faces),
                    domain_id,
                )
                profile.set_counter(
                    "PHYSICAL_EDGES_PER_DOMAIN",
                    len(patch_edge_ids),
                    domain_id,
                )
                profile.set_receipt(
                    EnvelopeDomainStageReceiptV1(
                        patch_id,
                        domain_id,
                        EnvelopeDomainStage.TOPOLOGY_READY,
                        EnvelopeDomainStage.TOPOLOGY_READY.value,
                        "Topology built from AnalysisBundle host facts",
                    )
                )
            for loop_index, loop in enumerate(patch.boundary_loops):
                vertex_ids = tuple(int(item) for item in loop.vert_indices)
                if (
                    len(vertex_ids) > 1
                    and vertex_ids[0] == vertex_ids[-1]
                ):
                    vertex_ids = vertex_ids[:-1]
                edge_ids = tuple(int(item) for item in loop.edge_indices)
                loop_kind = (
                    EnvelopeTopologyPathKind.PATCH_HOLE_LOOP
                    if loop.kind is LoopKind.HOLE
                    else EnvelopeTopologyPathKind.PATCH_OUTER_LOOP
                )
                style_key = (
                    "ENV_01_HOLES"
                    if loop.kind is LoopKind.HOLE
                    else "ENV_00_PATCH_DOMAIN"
                )
                loop_id = _typed_value(
                    "boundary-loop",
                    revision,
                    patch_id,
                    loop_index,
                )
                paths.append(
                    EnvelopeTopologyDebugPathV1(
                        semantic_id=loop_id,
                        patch_domain_id=domain_id,
                        kind=loop_kind,
                        local_points=_host_local_points(
                            vertex_ids,
                            host_vertex_by_id,
                        ),
                        host_vertex_ids=vertex_ids,
                        host_edge_ids=edge_ids,
                        closed=True,
                        directed=False,
                        selected=False,
                        style_key=style_key,
                        label=f"Patch {patch_id} {loop.kind.value}",
                        boundary_loop_id=loop_id,
                        display_normal=normal,
                    )
                )

        for key, records in sorted(chain_groups.items()):
            visible_records = tuple(
                record
                for record in records
                if record.patch_id in selected_patch_ids
            )
            if not visible_records:
                continue
            is_closed, canonical_edges, canonical_vertices = key
            physical_chain_id = _typed_value(
                "physical-chain",
                revision,
                is_closed,
                canonical_edges,
                canonical_vertices,
            )
            selected = key in selected_keys
            first_patch = analysis_bundle.patch_graph.nodes[
                visible_records[0].patch_id
            ]
            paths.append(
                EnvelopeTopologyDebugPathV1(
                    semantic_id=physical_chain_id,
                    patch_domain_id=None,
                    kind=EnvelopeTopologyPathKind.PHYSICAL_CHAIN,
                    local_points=_host_local_points(
                        canonical_vertices,
                        host_vertex_by_id,
                    ),
                    host_vertex_ids=canonical_vertices,
                    host_edge_ids=canonical_edges,
                    closed=is_closed,
                    directed=False,
                    selected=selected,
                    style_key="ENV_10_PHYSICAL_CHAINS",
                    label=f"PhysicalChain {physical_chain_id}",
                    physical_chain_id=physical_chain_id,
                    display_normal=_vector3(first_patch.normal),
                )
            )
            if selected:
                paths.append(
                    EnvelopeTopologyDebugPathV1(
                        semantic_id=_typed_value(
                            "selected-source",
                            revision,
                            physical_chain_id,
                        ),
                        patch_domain_id=None,
                        kind=EnvelopeTopologyPathKind.SELECTED_SOURCE,
                        local_points=_host_local_points(
                            canonical_vertices,
                            host_vertex_by_id,
                        ),
                        host_vertex_ids=canonical_vertices,
                        host_edge_ids=canonical_edges,
                        closed=is_closed,
                        directed=False,
                        selected=True,
                        style_key="ENV_12_SELECTED_SOURCES",
                        label=f"Selected source {physical_chain_id}",
                        physical_chain_id=physical_chain_id,
                        display_normal=_vector3(first_patch.normal),
                    )
                )
                selection_diagnostics.append(
                    EnvelopeTopologySelectionDiagnosticV1(
                        "WHOLE_PHYSICAL_CHAIN_SELECTED",
                        "Selected edges resolve to one complete PhysicalChain",
                        physical_chain_id=physical_chain_id,
                    )
                )

            use_ids = []
            use_domain_ids = []
            for record in sorted(
                visible_records,
                key=lambda item: (
                    item.patch_id,
                    item.loop_index,
                    item.chain_index,
                ),
            ):
                domain_id = domain_by_patch[record.patch_id]
                use_id = _typed_value(
                    "chain-use",
                    revision,
                    record.patch_id,
                    record.loop_index,
                    record.chain_index,
                    canonical_edges,
                )
                use_ids.append(use_id)
                use_domain_ids.append(domain_id)
                vertex_ids = tuple(int(item) for item in record.chain.vert_indices)
                if (
                    len(vertex_ids) > 1
                    and vertex_ids[0] == vertex_ids[-1]
                ):
                    vertex_ids = vertex_ids[:-1]
                patch = analysis_bundle.patch_graph.nodes[record.patch_id]
                paths.append(
                    EnvelopeTopologyDebugPathV1(
                        semantic_id=use_id,
                        patch_domain_id=domain_id,
                        kind=EnvelopeTopologyPathKind.DIRECTED_CHAIN_USE,
                        local_points=_host_local_points(
                            vertex_ids,
                            host_vertex_by_id,
                        ),
                        host_vertex_ids=vertex_ids,
                        host_edge_ids=tuple(
                            int(item) for item in record.chain.edge_indices
                        ),
                        closed=bool(record.chain.is_closed),
                        directed=True,
                        selected=selected,
                        style_key="ENV_11_CHAIN_USES",
                        label=(
                            f"Patch {record.patch_id} directed ChainUse "
                            f"{record.loop_index}:{record.chain_index}"
                        ),
                        physical_chain_id=physical_chain_id,
                        chain_use_id=use_id,
                        boundary_loop_id=_typed_value(
                            "boundary-loop",
                            revision,
                            record.patch_id,
                            record.loop_index,
                        ),
                        display_normal=_vector3(patch.normal),
                    )
                )

            if len(visible_records) == 2:
                neighbor_kind = visible_records[0].chain.neighbor_kind
                pair_kind = (
                    EnvelopeTopologyPairKind.SEAM_SELF
                    if neighbor_kind is ChainNeighborKind.SEAM_SELF
                    else (
                        EnvelopeTopologyPairKind.PATCH
                        if neighbor_kind is ChainNeighborKind.PATCH
                        else None
                    )
                )
                if pair_kind is not None:
                    pairs.append(
                        EnvelopeTopologyDebugPairV1(
                            pair_id=_typed_value(
                                "chain-use-pair",
                                revision,
                                pair_kind.value,
                                physical_chain_id,
                            ),
                            kind=pair_kind,
                            physical_chain_id=physical_chain_id,
                            chain_use_ids=tuple(use_ids),
                            patch_domain_ids=tuple(use_domain_ids),
                            host_edge_ids=canonical_edges,
                        )
                    )

        scene = EnvelopeTopologyDebugSceneV1(
            ENVELOPE_TOPOLOGY_DEBUG_SCENE_SCHEMA_V1,
            revision,
            tuple(domain_by_patch.values()),
            tuple(sorted(scope.edge_ids)),
            tuple(paths),
            tuple(pairs),
            tuple(selection_diagnostics),
        )
    return scene


def _shape_class(kernel, patch) -> object:
    value = getattr(patch, "shape_class", None)
    if value is None:
        # PatchNode v1 has no shape-class field. MIX is the declared host
        # contract for that schema, not a geometry-repair fallback.
        return kernel.PatchShapeClass.MIX
    text = value.value if hasattr(value, "value") else str(value)
    try:
        return kernel.PatchShapeClass(text)
    except ValueError as exc:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
            f"unsupported host Patch shape_class: {text}",
        ) from exc


def _context_tags(kernel, patch) -> frozenset:
    mapping = {
        PatchType.WALL: kernel.PatchContextTag.WALL,
        PatchType.FLOOR: kernel.PatchContextTag.FLOOR,
        PatchType.SLOPE: kernel.PatchContextTag.SLOPE,
    }
    patch_type = patch.patch_type
    if not isinstance(patch_type, PatchType):
        try:
            patch_type = PatchType(str(getattr(patch_type, "value", patch_type)))
        except ValueError as exc:
            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
                f"unsupported host PatchType: {patch_type}",
            ) from exc
    return frozenset({mapping[patch_type]})


def _source_ids(kernel, revision: str, analysis_bundle: AnalysisBundle):
    vertex_ids = {
        int(item.vertex_id): kernel.SourceVertexId(
            f"host-vertex:{revision}:{int(item.vertex_id)}"
        )
        for item in analysis_bundle.patch_surface.vertices
    }
    edge_ids = {
        int(item.edge_id): kernel.PhysicalEdgeId(
            f"host-edge:{revision}:{int(item.edge_id)}"
        )
        for item in analysis_bundle.patch_surface.edges
    }
    face_ids = {
        int(item.face_id): kernel.SourceFaceId(
            f"host-face:{revision}:{int(item.face_id)}"
        )
        for item in analysis_bundle.patch_surface.faces
    }
    triangle_ids = {
        int(item.triangle_id): kernel.SurfaceTriangleId(
            f"host-triangle:{revision}:{int(item.triangle_id)}"
        )
        for item in analysis_bundle.patch_surface.triangles
    }
    patch_ids = {
        int(patch_id): kernel.PatchId(f"host-patch:{revision}:{int(patch_id)}")
        for patch_id in analysis_bundle.patch_graph.nodes
    }
    return vertex_ids, edge_ids, face_ids, triangle_ids, patch_ids


def _exact_frame(
    kernel,
    sympy,
    *,
    revision: str,
    patch_id: int,
    patch,
    patch_domain_id,
    patch_vertex_ids: tuple[int, ...],
    host_vertex_by_id: dict[int, object],
    kernel_vertex_ids: dict[int, object],
):
    scalar = lambda value: sympy.Rational(str(float(value)))
    if not patch_vertex_ids:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
            "PatchDomain has no source vertices",
            patch_domain_id=patch_domain_id.value,
        )

    def dot(left, right):
        return sympy.factor(sum(a * b for a, b in zip(left, right, strict=True)))

    def cross(left, right):
        return (
            sympy.factor(left[1] * right[2] - left[2] * right[1]),
            sympy.factor(left[2] * right[0] - left[0] * right[2]),
            sympy.factor(left[0] * right[1] - left[1] * right[0]),
        )

    def subtract(left, right):
        return tuple(
            sympy.factor(a - b)
            for a, b in zip(left, right, strict=True)
        )

    def negate(vector):
        return tuple(-item for item in vector)

    def exact_float(value):
        value = sympy.factor(value)
        result = float(value)
        if not math.isfinite(result) or scalar(result) != value:
            return None
        return result

    def exact_float_vector(vector):
        values = tuple(exact_float(item) for item in vector)
        if any(item is None for item in values):
            return None
        return values

    positions = {
        vertex_id: tuple(
            scalar(value)
            for value in _vector3(host_vertex_by_id[vertex_id].position)
        )
        for vertex_id in patch_vertex_ids
    }
    source_origin_id = min(patch_vertex_ids)
    source_origin = positions[source_origin_id]

    raw_normal = None
    other_vertex_ids = tuple(
        vertex_id
        for vertex_id in patch_vertex_ids
        if vertex_id != source_origin_id
    )
    for left_index, left_vertex_id in enumerate(other_vertex_ids):
        left = subtract(positions[left_vertex_id], source_origin)
        for right_vertex_id in other_vertex_ids[left_index + 1:]:
            right = subtract(positions[right_vertex_id], source_origin)
            candidate = cross(left, right)
            if any(item != 0 for item in candidate):
                raw_normal = candidate
                break
        if raw_normal is not None:
            break
    if raw_normal is None:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
            f"host Patch {patch_id} source vertices do not define an exact plane",
            patch_domain_id=patch_domain_id.value,
        )

    raw_normal_norm_squared = dot(raw_normal, raw_normal)
    for vertex_id in patch_vertex_ids:
        plane_delta = dot(
            subtract(positions[vertex_id], source_origin),
            raw_normal,
        )
        if plane_delta != 0:
            signed_distance = float(plane_delta) / math.sqrt(
                float(raw_normal_norm_squared)
            )
            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
                f"host Patch {patch_id} source vertex {vertex_id} is not "
                "exactly coplanar; signed plane delta="
                f"{signed_distance!r}",
                patch_domain_id=patch_domain_id.value,
            )

    host_axis_u = tuple(scalar(value) for value in _vector3(patch.basis_u))
    host_axis_v = tuple(scalar(value) for value in _vector3(patch.basis_v))
    host_normal = tuple(scalar(value) for value in _vector3(patch.normal))
    orientation_dot = dot(raw_normal, host_normal)
    if orientation_dot == 0:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
            f"host Patch {patch_id} exact plane orientation is not declared",
            patch_domain_id=patch_domain_id.value,
        )
    if orientation_dot < 0:
        raw_normal = negate(raw_normal)

    normal_length = sympy.sqrt(dot(raw_normal, raw_normal))
    if normal_length.is_Rational is not True:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
            f"host Patch {patch_id} exact plane has no rational unit normal",
            patch_domain_id=patch_domain_id.value,
        )
    normal = tuple(sympy.factor(item / normal_length) for item in raw_normal)
    normal_values = exact_float_vector(normal)
    if normal_values is None:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
            f"host Patch {patch_id} exact unit normal has no exact public "
            "float representation",
            patch_domain_id=patch_domain_id.value,
        )

    def unit_vector(vector):
        length_squared = dot(vector, vector)
        if length_squared == 0:
            return None
        length = sympy.sqrt(length_squared)
        if length.is_Rational is not True:
            return None
        result = tuple(sympy.factor(item / length) for item in vector)
        if exact_float_vector(result) is None:
            return None
        return result

    def canonical_direction(vector):
        for item in vector:
            if item == 0:
                continue
            return vector if item > 0 else negate(vector)
        return vector

    coordinate_axes = (
        (sympy.Integer(1), sympy.Integer(0), sympy.Integer(0)),
        (sympy.Integer(0), sympy.Integer(1), sympy.Integer(0)),
        (sympy.Integer(0), sympy.Integer(0), sympy.Integer(1)),
    )

    def signed_axis_index(vector):
        nonzero = tuple(
            index for index, value in enumerate(vector) if value != 0
        )
        if (
            len(nonzero) == 1
            and abs(vector[nonzero[0]]) == 1
        ):
            return nonzero[0]
        return None

    normal_axis_index = signed_axis_index(normal)
    tangent_candidates = set()
    if normal_axis_index is not None:
        for index, coordinate_axis in enumerate(coordinate_axes):
            if index != normal_axis_index:
                tangent_candidates.add(coordinate_axis)
    else:
        for coordinate_axis in coordinate_axes:
            projected = tuple(
                sympy.factor(
                    component - dot(coordinate_axis, normal) * normal_component
                )
                for component, normal_component in zip(
                    coordinate_axis,
                    normal,
                    strict=True,
                )
            )
            candidate = unit_vector(projected)
            if candidate is not None:
                tangent_candidates.add(canonical_direction(candidate))
        for vertex_id in other_vertex_ids:
            candidate = unit_vector(
                subtract(positions[vertex_id], source_origin)
            )
            if candidate is not None:
                tangent_candidates.add(canonical_direction(candidate))

    if not tangent_candidates:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
            f"host Patch {patch_id} exact plane has no exact public "
            "float-valued tangent basis",
            patch_domain_id=patch_domain_id.value,
        )

    host_handedness_measure = dot(
        cross(host_axis_u, host_axis_v),
        host_normal,
    )
    right_handed = host_handedness_measure >= 0
    basis_candidates = []
    for canonical_u in tangent_candidates:
        for axis_u_candidate in (canonical_u, negate(canonical_u)):
            axis_v_candidate = (
                cross(normal, axis_u_candidate)
                if right_handed
                else cross(axis_u_candidate, normal)
            )
            if exact_float_vector(axis_v_candidate) is None:
                continue
            alignment = sympy.factor(
                dot(axis_u_candidate, host_axis_u)
                + dot(axis_v_candidate, host_axis_v)
            )
            basis_candidates.append(
                (
                    float(alignment),
                    tuple(str(item) for item in axis_u_candidate),
                    axis_u_candidate,
                    axis_v_candidate,
                )
            )
    if not basis_candidates:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
            f"host Patch {patch_id} exact plane has no exact public "
            "float-valued orthonormal basis",
            patch_domain_id=patch_domain_id.value,
        )
    _, _, axis_u, axis_v = max(
        basis_candidates,
        key=lambda item: (item[0], item[1]),
    )
    axis_u_values = exact_float_vector(axis_u)
    axis_v_values = exact_float_vector(axis_v)
    assert axis_u_values is not None
    assert axis_v_values is not None

    basis_checks = (
        dot(axis_u, axis_u) - 1,
        dot(axis_v, axis_v) - 1,
        dot(normal, normal) - 1,
        dot(axis_u, axis_v),
        dot(axis_u, normal),
        dot(axis_v, normal),
    )
    if any(item != 0 for item in basis_checks):
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
            f"host Patch {patch_id} derived planar basis is not exactly orthonormal",
            patch_domain_id=patch_domain_id.value,
        )
    cross_uv = cross(axis_u, axis_v)
    if cross_uv == normal:
        handedness = kernel.PatchFrameHandedness.RIGHT_HANDED_U_V_NORMAL
    elif cross_uv == tuple(-item for item in normal):
        handedness = kernel.PatchFrameHandedness.LEFT_HANDED_U_V_NORMAL
    else:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
            f"host Patch {patch_id} planar basis handedness is not exactly certified",
            patch_domain_id=patch_domain_id.value,
        )

    source_origin_values = _vector3(
        host_vertex_by_id[source_origin_id].position
    )
    if normal_axis_index is not None:
        origin_values = tuple(
            source_origin_values[index] if index == normal_axis_index else 0.0
            for index in range(3)
        )
    else:
        origin_values = source_origin_values
    origin = tuple(scalar(value) for value in origin_values)

    coordinates = []
    for vertex_id in patch_vertex_ids:
        position = positions[vertex_id]
        relative = tuple(
            value - base for value, base in zip(position, origin, strict=True)
        )
        plane_delta = dot(relative, normal)
        if plane_delta != 0:
            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
                f"host Patch {patch_id} source vertex {vertex_id} is not "
                "exactly coplanar; signed plane delta="
                f"{float(plane_delta)!r}",
                patch_domain_id=patch_domain_id.value,
            )
        exact_x = sympy.factor(dot(relative, axis_u))
        exact_y = sympy.factor(dot(relative, axis_v))
        float_x = float(exact_x)
        float_y = float(exact_y)
        if (
            not math.isfinite(float_x)
            or not math.isfinite(float_y)
            or scalar(float_x) != exact_x
            or scalar(float_y) != exact_y
        ):
            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
                f"host Patch {patch_id} source vertex {vertex_id} planar "
                "coordinate has no exact float round-trip",
                patch_domain_id=patch_domain_id.value,
            )
        coordinates.append(
            kernel.PlanarSourceVertexCoordinateV1(
                kernel_vertex_ids[vertex_id],
                kernel.LocalPoint2V1(float_x, float_y),
            )
        )
    certificate = kernel.PlanarityCertificateV1(
        kernel.PlanarityCertificateId(
            _typed_value("planarity", revision, patch_id)
        ),
        patch_domain_id,
        kernel.PlanarityLaw.ALL_DOMAIN_SOURCE_VERTICES_ON_CERTIFIED_PLANE_V1,
        True,
        kernel.LocalLengthV1(Decimal(0)),
        kernel.SurfaceMetricAuthority.HOST_ANALYSIS_AUTHORITATIVE_V1,
    )
    return kernel.PlanarPatchFrameV1(
        patch_domain_id,
        kernel.LocalPoint3V1(*origin_values),
        kernel.LocalVector3V1(*axis_u_values),
        kernel.LocalVector3V1(*axis_v_values),
        kernel.LocalVector3V1(*normal_values),
        handedness,
        certificate,
        kernel.PlanarProjectionLaw.DOT_WITH_AUTHORITATIVE_U_V_BASIS_V1,
        frozenset(coordinates),
    )


def _build_angular_relations(
    kernel,
    sympy,
    *,
    analysis_bundle,
    revision: str,
    patch_ids: dict[int, object],
    patch_domains: dict[int, object],
    frames: dict[int, object],
    use_id_by_ref: dict[tuple[int, int, int], object],
    sector_id_by_ref: dict[tuple[int, int, int], object],
    launch_id_by_ref: dict[tuple[int, int, int], object],
    record_by_ref: dict[tuple[int, int, int], _HostChainRecord],
    vertex_ids: dict[int, object],
    profile: EnvelopeDebugProfileBuilderV1 | None = None,
):
    scalar = lambda value: sympy.Rational(str(float(value)))
    metric_types = importlib.import_module(
        "cftuv_envelope.reference.planar_types"
    )
    angle_measure = importlib.import_module(
        "cftuv_envelope.reference.angle_measure"
    )

    def point_map(frame):
        if isinstance(frame, kernel.RationalAffinePlanarMetricV2):
            return {
                item.source_vertex_id: (
                    sympy.Rational(
                        item.domain_coordinate.x.numerator,
                        item.domain_coordinate.x.denominator,
                    ),
                    sympy.Rational(
                        item.domain_coordinate.y.numerator,
                        item.domain_coordinate.y.denominator,
                    ),
                )
                for item in frame.exact_source_vertex_coordinates
            }
        return {
            item.source_vertex_id: (
                scalar(item.domain_coordinate.x),
                scalar(item.domain_coordinate.y),
            )
            for item in frame.source_vertex_coordinates
        }

    angular_sectors = []
    angle_certificates = []
    corner_relations = []
    used_sector_ids = set()
    host_face_by_patch = {
        int(patch_id): analysis_bundle.patch_surface.patch_faces(int(patch_id))
        for patch_id in analysis_bundle.patch_graph.nodes
    }
    normalized_refs_by_source: dict[
        tuple[int, int, int],
        tuple[tuple[int, int, int], ...],
    ] = {}
    source_refs = {
        (
            record.patch_id,
            record.loop_index,
            record.source_chain_index,
        )
        for record in record_by_ref.values()
    }
    for source_ref in source_refs:
        normalized_refs_by_source[source_ref] = tuple(
            ref
            for ref, record in sorted(record_by_ref.items())
            if (
                record.patch_id,
                record.loop_index,
                record.source_chain_index,
            )
            == source_ref
        )

    for patch_id, patch in sorted(analysis_bundle.patch_graph.nodes.items()):
        patch_id = int(patch_id)
        domain_id = patch_domains[patch_id]
        frame = frames[patch_id]
        coordinates = point_map(frame)
        metric = kernel.ExactPlanarMetric.from_descriptor(frame)
        kernel_vertex_for_host = vertex_ids

        def vector(value):
            return metric_types.ExactPlanarVector.from_values(*value)

        def expressions(value):
            return value.expressions()

        def owner_support(record: _HostChainRecord, anchor_vertex_id: int):
            host_vertices = tuple(int(item) for item in record.chain.vert_indices)
            if anchor_vertex_id == host_vertices[0]:
                start_vertex_id, end_vertex_id = host_vertices[0], host_vertices[1]
                physical_edge_id = int(record.chain.edge_indices[0])
            elif anchor_vertex_id == host_vertices[-1]:
                start_vertex_id, end_vertex_id = host_vertices[-2], host_vertices[-1]
                physical_edge_id = int(record.chain.edge_indices[-1])
            else:
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                    "BoundaryCorner anchor is not a physical ChainUse endpoint",
                    patch_domain_id=domain_id.value,
                )
            start = coordinates[kernel_vertex_for_host[start_vertex_id]]
            end = coordinates[kernel_vertex_for_host[end_vertex_id]]
            tangent = (end[0] - start[0], end[1] - start[1])
            tangent_vector = vector(tangent)
            if metric.dot_g(tangent_vector, tangent_vector) == 0:
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                    "BoundaryCorner incident support is degenerate",
                    patch_domain_id=domain_id.value,
                )
            candidates = []
            for face in host_face_by_patch[patch_id]:
                if physical_edge_id not in face.edge_cycle:
                    continue
                ordinal = face.edge_cycle.index(physical_edge_id)
                cycle = tuple(
                    coordinates[kernel_vertex_for_host[int(item)]]
                    for item in face.vertex_cycle
                )
                twice_area = sum(
                    cycle[index][0] * cycle[(index + 1) % len(cycle)][1]
                    - cycle[index][1] * cycle[(index + 1) % len(cycle)][0]
                    for index in range(len(cycle))
                )
                if twice_area == 0:
                    continue
                face_start = cycle[ordinal]
                face_end = cycle[(ordinal + 1) % len(cycle)]
                face_direction = (
                    face_end[0] - face_start[0],
                    face_end[1] - face_start[1],
                )
                same_direction = (
                    metric.dot_g(tangent_vector, vector(face_direction)) > 0
                )
                coordinate_interior_is_left = (
                    twice_area > 0
                ) == same_direction
                interior_is_left = coordinate_interior_is_left == (
                    metric.owner_orientation_sign > 0
                )
                candidates.append(
                    metric.owner_normal_g(
                        tangent_vector,
                        owner_left=interior_is_left,
                        normalize=False,
                    )
                )
            if not candidates:
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                    "BoundaryCorner support has no exact owner-face direction",
                    patch_domain_id=domain_id.value,
                )
            first = candidates[0]
            if any(
                metric.oriented_cross(first, item) != 0
                or metric.dot_g(first, item) <= 0
                for item in candidates[1:]
            ):
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                    "BoundaryCorner owner-face directions disagree",
                    patch_domain_id=domain_id.value,
                )
            return expressions(first), tangent

        # Счётчики заводятся ДО обхода и всегда, даже нулями: «углов не
        # нашлось» и «стадия ничего не смотрела» иначе — одна пустота.
        stage_counters = dict.fromkeys(ANGULAR_STAGE_COUNTERS, 0)
        for loop_index, loop in enumerate(patch.boundary_loops):
            for corner_index, corner in enumerate(loop.corners):
                stage_counters["ANGULAR_CORNERS_CONSIDERED"] += 1
                prev_source_ref = (
                    patch_id,
                    loop_index,
                    int(corner.prev_chain_index),
                )
                next_source_ref = (
                    patch_id,
                    loop_index,
                    int(corner.next_chain_index),
                )
                if (
                    prev_source_ref not in normalized_refs_by_source
                    or next_source_ref not in normalized_refs_by_source
                ):
                    raise EnvelopeHostAdapterError(
                        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                        "BoundaryCorner incident ChainUse references are incomplete",
                        patch_domain_id=domain_id.value,
                    )
                prev_ref = normalized_refs_by_source[prev_source_ref][-1]
                next_ref = normalized_refs_by_source[next_source_ref][0]
                anchor_vertex_id = int(corner.vert_index)
                if anchor_vertex_id not in vertex_ids:
                    raise EnvelopeHostAdapterError(
                        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                        "BoundaryCorner source vertex is absent from PatchSurfaceIR",
                        patch_domain_id=domain_id.value,
                    )
                incoming_normal, incoming_tangent = owner_support(
                    record_by_ref[prev_ref], anchor_vertex_id
                )
                outgoing_normal, outgoing_tangent = owner_support(
                    record_by_ref[next_ref], anchor_vertex_id
                )
                incoming_vector = vector(incoming_normal)
                outgoing_vector = vector(outgoing_normal)
                incoming_tangent_vector = vector(incoming_tangent)
                outgoing_tangent_vector = vector(outgoing_tangent)
                turn_cross = metric.oriented_cross(
                    incoming_tangent_vector,
                    outgoing_tangent_vector,
                )
                if turn_cross == 0:
                    if metric.dot_g(
                        incoming_tangent_vector,
                        outgoing_tangent_vector,
                    ) > 0:
                        stage_counters["ANGULAR_COLLINEAR_SKIPPED"] += 1
                        continue
                    raise EnvelopeHostAdapterError(
                        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                        "exact 2*pi corner must be Terminal/Junction, not Angular",
                        patch_domain_id=domain_id.value,
                    )
                incoming_interior_side = metric.oriented_cross(
                    incoming_tangent_vector,
                    incoming_vector,
                )
                outgoing_interior_side = metric.oriented_cross(
                    outgoing_tangent_vector,
                    outgoing_vector,
                )
                if (
                    incoming_interior_side == 0
                    or outgoing_interior_side == 0
                    or (
                        incoming_interior_side > 0
                    ) != (
                        outgoing_interior_side > 0
                    )
                ):
                    raise EnvelopeHostAdapterError(
                        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                        "BoundaryCorner exact owner-interior sides disagree",
                        patch_domain_id=domain_id.value,
                    )
                is_reflex = (
                    turn_cross * incoming_interior_side < 0
                )
                if not is_reflex:
                    continue
                stage_counters["ANGULAR_REFLEX_CORNERS"] += 1
                cosine, sine = metric.angle_g(
                    incoming_vector, outgoing_vector
                )
                orientation = (
                    kernel.TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
                    if turn_cross > 0
                    else kernel.TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION
                )
                try:
                    # δ выводится из φ уже после канонизации: иначе равенство
                    # `δ == φ − 1` не доживает до записи (см. модуль меры).
                    measure = certified_reflex_measure(
                        kernel, angle_measure, sine, cosine, orientation
                    )
                except angle_measure.CertifiedAngleUnavailable as error:
                    raise EnvelopeHostAdapterError(
                        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                        str(error),
                        patch_domain_id=domain_id.value,
                    ) from error
                owner_sector_id = sector_id_by_ref[prev_ref]
                if owner_sector_id in used_sector_ids:
                    raise EnvelopeHostAdapterError(
                        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_MULTIPLE_ANGULAR_RELATIONS_PER_CHAIN_UNSUPPORTED,
                        "V0 cannot encode two Angular relations through one owner-sector record",
                        patch_domain_id=domain_id.value,
                    )
                used_sector_ids.add(owner_sector_id)

                def exact_rational(value):
                    value = sympy.factor(value)
                    if value.is_Rational is not True:
                        raise EnvelopeHostAdapterError(
                            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                            "affine support direction is not rational",
                            patch_domain_id=domain_id.value,
                        )
                    return kernel.ExactRationalV1(
                        int(value.p), int(value.q)
                    )

                def direction_payload(value):
                    return kernel.CertifiedAffineSupportDirectionV2(
                        kernel.ExactVector2V1(
                            exact_rational(value[0]),
                            exact_rational(value[1]),
                        ),
                        frame.reference_metric_id,
                    )

                incoming_use_id = use_id_by_ref[prev_ref]
                outgoing_use_id = use_id_by_ref[next_ref]
                angular_sectors.append(
                    kernel.OrientedOwnerSectorV1(
                        owner_sector_id,
                        patch_ids[patch_id],
                        domain_id,
                        True,
                        (incoming_use_id, outgoing_use_id),
                        kernel.SourceSupportRefV1(
                            incoming_use_id,
                            launch_id_by_ref[prev_ref],
                            direction_payload(incoming_normal),
                        ),
                        kernel.SourceSupportRefV1(
                            outgoing_use_id,
                            launch_id_by_ref[next_ref],
                            direction_payload(outgoing_normal),
                        ),
                        orientation,
                        kernel.InteriorSelectionLaw.OWNER_PATCH_INTERIOR_BETWEEN_ORDERED_SUPPORTS,
                    )
                )
                angle_id = kernel.AngleCertificateId(
                    _typed_value(
                        "angle-certificate",
                        revision,
                        patch_id,
                        loop_index,
                        corner_index,
                    )
                )
                angle_certificates.append(
                    kernel.ReflexAngleCertificateV1(
                        angle_id,
                        owner_sector_id,
                        kernel.AngleMeasureLaw.ORIENTED_OWNER_SECTOR_ANGLE,
                        kernel.AngleMeasureSource.HOST_ANALYSIS_EXACT_OR_CERTIFIED,
                        kernel.StrictAngleRangeCertificate.STRICT_PI_LT_PHI_LT_2PI,
                        kernel.ReflexExcessLaw.DELTA_EQUALS_PHI_MINUS_PI,
                        measure,
                        False,
                    )
                )
                corner_relations.append(
                    kernel.CornerRelationV1(
                        kernel.CornerRelationId(
                            _typed_value(
                                "corner-relation",
                                revision,
                                patch_id,
                                loop_index,
                                corner_index,
                            )
                        ),
                        vertex_ids[anchor_vertex_id],
                        owner_sector_id,
                        angle_id,
                        False,
                    )
                )
                stage_counters["ANGULAR_RELATIONS_BUILT"] += 1
        if profile is not None:
            for name, value in sorted(stage_counters.items()):
                profile.set_counter(name, value, domain_id.value)
    return (
        frozenset(angular_sectors),
        frozenset(angle_certificates),
        frozenset(corner_relations),
    )


def _host_outcome_for(outcome):
    """Исход ядра — в исход хоста, по имени.

    Именем, а не таблицей соответствий: у каждого из этих отказов имя ядра и
    имя хоста совпадают, и совпадать они обязаны — расхождение означало бы, что
    поле читает одно, а лог ядра говорит другое. Неизвестное имя не
    подменяется ближайшим: тогда новый исход ядра молча выходил бы под чужим.
    """

    try:
        return EnvelopeDebugHostOutcome(outcome.value)
    except ValueError:
        return EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED


def _rational_affine_metric(
    kernel,
    *,
    source_revision,
    patch_domain_id,
    owner_patch_id,
    source_vertices,
    source_faces,
):
    """Thin host delegation; exact chart construction belongs to the kernel."""

    try:
        return kernel.build_rational_affine_planar_metric(
            source_revision=source_revision,
            patch_domain_id=patch_domain_id,
            owner_patch_id=owner_patch_id,
            source_vertices=source_vertices,
            source_faces=source_faces,
            planarity_policy=kernel.PlanarityAdmissionLawV1(HOST_PLANARITY_POLICY.value),
            grid_policy=kernel.GridSnappingLawV1(HOST_GRID_POLICY.value),
            source_lineage=frozenset(
                {
                    kernel.LineageId(
                        _typed_value(
                            "metric-source",
                            source_revision.value,
                            patch_domain_id.value,
                        )
                    )
                }
            ),
        )
    except kernel.PlanarMetricAdmissionError as exc:
        # Исход ядра переносится ПО ИМЕНИ, а не сводится к одному. Ядро
        # различает «политика проекции не запрошена», «источник вне бюджета»,
        # «снап вывел источник из плоскости» и три исхода окна решётки; хост,
        # схлопывая их, оставлял в поле причину, которой не было.
        raise EnvelopeHostAdapterError(
            _host_outcome_for(exc.outcome),
            str(exc),
            patch_domain_id=patch_domain_id.value,
        ) from exc
    except ValueError as exc:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
            str(exc),
            patch_domain_id=patch_domain_id.value,
        ) from exc


def build_envelope_analysis_snapshot(
    analysis_bundle: AnalysisBundle,
    *,
    included_patch_ids: frozenset[int] | None = None,
    profile: EnvelopeDebugProfileBuilderV1 | None = None,
    topology_export=None,
    analysis_view=None,
) -> envelope_kernel.AnalysisSnapshotV1:
    """Map one real host AnalysisBundle without geometry repair or fallback."""

    kernel, sympy = _load_kernel()
    source_revision_value = _revision_value(analysis_bundle.source_revision)
    profile_domain_id = None
    if included_patch_ids is not None and len(included_patch_ids) == 1:
        profile_patch_id = int(next(iter(included_patch_ids)))
        profile_domain_id = _typed_value(
            "patch-domain",
            source_revision_value,
            profile_patch_id,
        )
    request_scoped = included_patch_ids is not None
    if included_patch_ids is not None:
        with _measure(profile, "BUNDLE_SLICE", profile_domain_id):
            if analysis_view is None:
                from .envelope_topology_export import (
                    build_analysis_bundle_id_view,
                )

                analysis_view = build_analysis_bundle_id_view(
                    analysis_bundle,
                    frozenset(int(item) for item in included_patch_ids),
                )
            analysis_bundle = analysis_view
    analysis_bundle.capabilities.require_supported()
    revision = source_revision_value
    source_revision = kernel.SourceRevision(revision)
    (
        vertex_ids,
        edge_ids,
        face_ids,
        triangle_ids,
        patch_ids,
    ) = _source_ids(kernel, revision, analysis_bundle)
    host_vertex_by_id = analysis_bundle.patch_surface.vertex_by_id

    source_vertices = frozenset(
        kernel.SourceVertexV1(
            vertex_ids[int(item.vertex_id)],
            kernel.LocalPoint3V1(*_vector3(item.position)),
        )
        for item in analysis_bundle.patch_surface.vertices
    )
    source_edges = frozenset(
        kernel.SourceEdgeV1(
            edge_ids[int(item.edge_id)],
            vertex_ids[int(item.vertex_ids[0])],
            vertex_ids[int(item.vertex_ids[1])],
        )
        for item in analysis_bundle.patch_surface.edges
    )
    source_faces = frozenset(
        kernel.SourceFaceV1(
            face_ids[int(item.face_id)],
            patch_ids[int(item.patch_id)],
            tuple(vertex_ids[int(value)] for value in item.vertex_cycle),
            tuple(edge_ids[int(value)] for value in item.edge_cycle),
            kernel.LocalVector3V1(*_vector3(item.polygon_normal)),
            tuple(triangle_ids[int(value)] for value in item.triangle_ids),
        )
        for item in analysis_bundle.patch_surface.faces
    )
    surface_triangles = frozenset(
        kernel.SurfaceTriangleV1(
            triangle_ids[int(item.triangle_id)],
            face_ids[int(item.source_face_id)],
            tuple(vertex_ids[int(value)] for value in item.vertex_ids),
            tuple(
                edge_ids[int(value)] if value is not None else None
                for value in (
                    item.physical_edge_ids[2],
                    item.physical_edge_ids[0],
                    item.physical_edge_ids[1],
                )
            ),
            kernel.LocalVector3V1(*_vector3(item.triangle_normal)),
        )
        for item in analysis_bundle.patch_surface.triangles
    )
    surface_ir = kernel.PatchSurfaceIRV1(
        "cftuv.envelope.patch_surface_ir.v1",
        source_revision,
        kernel.SurfacePayloadMode.FULL_HOST_SURFACE,
        frozenset(vertex_ids.values()),
        source_edges,
        source_faces,
        surface_triangles,
    )

    if topology_export is None:
        host_chains = _collect_host_chains(
            analysis_bundle,
            profile=profile,
            patch_domain_id=profile_domain_id,
        )
    else:
        selected_patch_ids = frozenset(
            int(item) for item in analysis_bundle.patch_graph.nodes
        )
        host_chains = tuple(
            record
            for record in topology_export.host_chains
            if record.patch_id in selected_patch_ids
        )
    chain_groups = _group_host_chains(host_chains)

    physical_chains = []
    chain_id_by_key = {}
    use_id_by_ref = {}
    for key, records in sorted(chain_groups.items(), key=lambda item: item[0]):
        is_closed, canonical_edges, canonical_vertices = key
        neighbor_kinds = {record.chain.neighbor_kind for record in records}
        patch_group = {record.patch_id for record in records}
        if len(neighbor_kinds) != 1:
            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PHYSICAL_CHAIN_INVALID,
                "one canonical PhysicalChain has conflicting host neighbor kinds",
            )
        neighbor_kind = next(iter(neighbor_kinds))
        if neighbor_kind is ChainNeighborKind.SEAM_SELF:
            if len(records) != 2 or len(patch_group) != 1:
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_SELF_SEAM_USE_PAIR_UNAVAILABLE,
                    "SEAM_SELF requires exactly two host ChainUses in one PatchDomain",
                )
            chain_kind = kernel.PhysicalChainKind.SEAM_SELF
        elif neighbor_kind is ChainNeighborKind.PATCH:
            # A request-scoped snapshot may contain one use of a seam whose
            # opposite PatchDomain was not selected. resolve_selection_scope()
            # has already proved the complete pair in the source bundle.
            exact_pair = len(records) == 2 and len(patch_group) == 2
            external_scoped_use = (
                request_scoped
                and len(records) == 1
                and len(patch_group) == 1
            )
            if not exact_pair and not external_scoped_use:
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_CHAIN_USE_PAIR_UNAVAILABLE,
                    "physical seam requires two exact whole-chain patch-side uses",
                )
            chain_kind = kernel.PhysicalChainKind.PHYSICAL_SEAM
        else:
            if len(records) != 1:
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PHYSICAL_CHAIN_INVALID,
                    "ordinary host boundary source requires exactly one ChainUse",
                )
            chain_kind = kernel.PhysicalChainKind.PHYSICAL_DECAL_SOURCE
        chain_id = kernel.PhysicalChainId(
            _typed_value(
                "physical-chain",
                revision,
                is_closed,
                canonical_edges,
                canonical_vertices,
            )
        )
        chain_id_by_key[key] = chain_id
        lineage_id = kernel.LineageId(
            _typed_value("physical-lineage", revision, canonical_edges)
        )
        physical_chains.append(
            kernel.PhysicalChainV1(
                chain_id,
                chain_kind,
                is_closed,
                tuple(vertex_ids[item] for item in canonical_vertices),
                tuple(edge_ids[item] for item in canonical_edges),
                frozenset({lineage_id}),
                frozenset(
                    kernel.LineageId(
                        _typed_value(
                            "chain-record",
                            revision,
                            item.patch_id,
                            item.loop_index,
                            item.chain_index,
                        )
                    )
                    for item in records
                ),
            )
        )
        for record in records:
            use_id_by_ref[
                (record.patch_id, record.loop_index, record.chain_index)
            ] = kernel.ChainUseId(
                _typed_value(
                    "chain-use",
                    revision,
                    record.patch_id,
                    record.loop_index,
                    record.chain_index,
                    canonical_edges,
                )
            )

    patch_domains = {}
    loop_ids = {}
    boundary_loops = []
    boundary_constraints = []
    chain_uses = []
    sector_id_by_ref = {}
    launch_id_by_ref = {}
    sectors_by_patch: dict[int, list] = {
        int(patch_id): [] for patch_id in analysis_bundle.patch_graph.nodes
    }
    terminal_relations = []
    domain_constraint_ids: dict[int, set] = {
        int(patch_id): set() for patch_id in analysis_bundle.patch_graph.nodes
    }

    for patch_id, patch in sorted(analysis_bundle.patch_graph.nodes.items()):
        patch_id = int(patch_id)
        domain_id = kernel.PatchDomainId(
            _typed_value("patch-domain", revision, patch_id)
        )
        patch_domains[patch_id] = domain_id
        for loop_index, loop in enumerate(patch.boundary_loops):
            loop_id = kernel.BoundaryLoopId(
                _typed_value("boundary-loop", revision, patch_id, loop_index)
            )
            loop_ids[(patch_id, loop_index)] = loop_id
            ordered_refs = tuple(
                (item.patch_id, item.loop_index, item.chain_index)
                for item in sorted(
                    host_chains,
                    key=lambda record: record.chain_index,
                )
                if (
                    item.patch_id == patch_id
                    and item.loop_index == loop_index
                )
            )
            ordered_uses = tuple(
                use_id_by_ref[ref] for ref in ordered_refs
            )
            boundary_loops.append(
                kernel.BoundaryLoopV1(
                    loop_id,
                    domain_id,
                    kernel.BoundaryLoopKind(loop.kind.value),
                    ordered_uses,
                )
            )
            topology_constraint_id = kernel.BoundaryConstraintId(
                _typed_value(
                    "topological-boundary",
                    revision,
                    patch_id,
                    loop_index,
                )
            )
            boundary_constraints.append(
                kernel.BoundaryConstraintV1(
                    topology_constraint_id,
                    domain_id,
                    kernel.BoundaryConstraintKind.TOPOLOGICAL_BOUNDARY_USE,
                    kernel.BoundaryLoopConstraintTargetV1(loop_id),
                    None,
                    False,
                    False,
                )
            )
            domain_constraint_ids[patch_id].add(topology_constraint_id)

    record_by_ref = {
        (item.patch_id, item.loop_index, item.chain_index): item
        for item in host_chains
    }
    for ref, use_id in sorted(use_id_by_ref.items()):
        patch_id, loop_index, chain_index = ref
        record = record_by_ref[ref]
        key = (
            bool(record.chain.is_closed),
            record.canonical_edge_ids,
            record.canonical_vertex_ids,
        )
        physical_chain_id = chain_id_by_key[key]
        domain_id = patch_domains[patch_id]
        launch_id = kernel.BoundaryConstraintId(
            _typed_value("source-launch", revision, *ref)
        )
        launch_id_by_ref[ref] = launch_id
        launch = kernel.LaunchLocusV1(
            launch_id,
            kernel.OwnerInteriorDirection.OWNER_INTERIOR,
            True,
        )
        roles = {
            kernel.ChainUseRole.TOPOLOGICAL_BOUNDARY_USE,
            kernel.ChainUseRole.DOMAIN_BOUNDARY,
        }
        chain_uses.append(
            kernel.ChainUseV1(
                use_id,
                physical_chain_id,
                patch_ids[patch_id],
                domain_id,
                loop_ids[(patch_id, loop_index)],
                kernel.ChainUseOrientation.B_START_TO_END
                if record.reversed_from_canonical
                else kernel.ChainUseOrientation.A_START_TO_END,
                frozenset(roles),
                launch,
            )
        )
        boundary_constraints.append(
            kernel.BoundaryConstraintV1(
                launch_id,
                domain_id,
                kernel.BoundaryConstraintKind.SOURCE_LAUNCH_BOUNDARY,
                kernel.ChainUseConstraintTargetV1(use_id),
                use_id,
                False,
                True,
            )
        )
        domain_constraint_ids[patch_id].add(launch_id)
        sector_id = kernel.OwnerSectorId(
            _typed_value("owner-sector", revision, *ref)
        )
        sector_id_by_ref[ref] = sector_id
        sectors_by_patch[patch_id].append(
            kernel.PatchSectorV1(
                sector_id,
                use_id,
                True,
                kernel.LawId("HOST_BOUNDARY_CHAIN_OWNER_INTERIOR_V1"),
            )
        )
        if not record.chain.is_closed:
            lineage = next(
                item.source_lineage
                for item in physical_chains
                if item.physical_chain_id == physical_chain_id
            )
            for role, vertex_id in (
                (
                    kernel.TerminalEndpointRole.START,
                    record.canonical_vertex_ids[0],
                ),
                (
                    kernel.TerminalEndpointRole.END,
                    record.canonical_vertex_ids[-1],
                ),
            ):
                terminal_relations.append(
                    kernel.TerminalRelationV1(
                        kernel.TerminalRelationId(
                            _typed_value(
                                "terminal",
                                revision,
                                *ref,
                                role.value,
                            )
                        ),
                        vertex_ids[vertex_id],
                        use_id,
                        patch_ids[patch_id],
                        domain_id,
                        kernel.TerminalRelationKind.PHYSICAL_CHAIN_ENDPOINT,
                        role,
                        sector_id,
                        lineage,
                    )
                )

    patch_descriptors = []
    metric_descriptors = []
    frames = {}
    for patch_id, patch in sorted(analysis_bundle.patch_graph.nodes.items()):
        patch_id = int(patch_id)
        if patch_id not in patch_ids:
            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
                f"PatchGraph patch {patch_id} has no PatchSurfaceIR identity",
            )
        patch_descriptors.append(
            kernel.PatchDescriptorV1(
                patch_ids[patch_id],
                kernel.SurfaceRegime.PLANAR,
                _shape_class(kernel, patch),
                _context_tags(kernel, patch),
            )
        )
        domain_value = patch_domains[patch_id].value
        with _measure(profile, "FRAME_ADMISSION", domain_value):
            frame = _rational_affine_metric(
                kernel,
                source_revision=source_revision,
                patch_domain_id=patch_domains[patch_id],
                owner_patch_id=patch_ids[patch_id],
                source_vertices=source_vertices,
                source_faces=source_faces,
            )
        frames[patch_id] = frame
        metric_descriptors.append(frame)

    angular_timing_domain = (
        next(iter(patch_domains.values())).value
        if len(patch_domains) == 1
        else None
    )


    with _measure(profile, "ANGULAR_RELATIONS", angular_timing_domain):
        (
            angular_owner_sectors,
            reflex_angle_certificates,
            corner_relations,
        ) = _build_angular_relations(
            kernel,
            sympy,
            analysis_bundle=analysis_bundle,
            revision=revision,
            patch_ids=patch_ids,
            patch_domains=patch_domains,
            frames=frames,
            use_id_by_ref=use_id_by_ref,
            sector_id_by_ref=sector_id_by_ref,
            launch_id_by_ref=launch_id_by_ref,
            record_by_ref=record_by_ref,
            vertex_ids=vertex_ids,
            profile=profile,
        )

    domain_records = frozenset(
        kernel.PatchDomainV1(
            patch_domains[patch_id],
            patch_ids[patch_id],
            kernel.SurfaceRegime.PLANAR,
            frozenset(domain_constraint_ids[patch_id]),
            frozenset(sectors_by_patch[patch_id]),
        )
        for patch_id in sorted(patch_domains)
    )
    snapshot = kernel.AnalysisSnapshotV1(
        kernel.ANALYSIS_SNAPSHOT_SCHEMA_V1,
        source_revision,
        frozenset(
            {
                kernel.AnalysisCapability.PATCH_DOMAIN_V1,
                kernel.AnalysisCapability.PHYSICAL_CHAIN_DIRECTED_USES_V1,
                kernel.AnalysisCapability.PATCH_SURFACE_IR_V1,
                kernel.AnalysisCapability.ORIENTED_OWNER_SECTOR_V1,
                kernel.AnalysisCapability.REFLEX_ANGLE_CERTIFICATE_V1,
                kernel.AnalysisCapability.RELATION_LINEAGE_V1,
                kernel.AnalysisCapability.AUTHORITATIVE_SURFACE_METRIC_V1,
                kernel.AnalysisCapability.PHYSICAL_EDGE_TABLE_V1,
                kernel.AnalysisCapability.BOUNDARY_CONSTRAINT_TARGET_V1,
                kernel.AnalysisCapability.TYPED_JUNCTION_ROUTE_TOPOLOGY_V1,
                kernel.AnalysisCapability.TERMINAL_RELATION_V1,
            }
        ),
        frozenset(patch_descriptors),
        domain_records,
        frozenset(metric_descriptors),
        surface_ir,
        source_vertices,
        frozenset(physical_chains),
        frozenset(chain_uses),
        frozenset(boundary_loops),
        frozenset(boundary_constraints),
        angular_owner_sectors,
        reflex_angle_certificates,
        corner_relations,
        frozenset(),
        frozenset(terminal_relations),
    )
    with _measure(profile, "SNAPSHOT_VALIDATION", angular_timing_domain):
        # Оба валидатора ядра, и над объектом, и над каноническими байтами:
        # полевой отказ прошёл именно здесь — в памяти снапшот был валиден, а
        # выпущенные байты ядро отвергало целиком.
        try:
            validate_exported_snapshot(kernel, snapshot)
        except SnapshotExportRejected as error:
            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome(error.outcome_name),
                str(error),
                patch_domain_id=error.patch_domain_id,
            ) from error
    return snapshot


def _host_edge_number(edge_id) -> int:
    try:
        return int(edge_id.value.rsplit(":", 1)[1])
    except (AttributeError, ValueError) as exc:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
            f"PhysicalEdgeId is not a V0 host identity: {edge_id}",
        ) from exc


def build_envelope_decal_request(
    snapshot: envelope_kernel.AnalysisSnapshotV1,
    selected_physical_edge_ids: frozenset[int],
    alpha: float,
    *,
    decal_request_id_value: str | None = None,
    density=None,
) -> envelope_kernel.DecalRequestV1:
    """Compile whole-chain selection into one immutable debug request."""

    kernel, _ = _load_kernel()
    angular_policy = envelope_angular_policy(kernel, density)
    if not selected_physical_edge_ids:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EMPTY_SELECTION,
            "Envelope debug requires at least one selected physical edge",
        )
    selected_edges = frozenset(int(item) for item in selected_physical_edge_ids)
    known_edges = {
        _host_edge_number(item.edge_id) for item in snapshot.surface_ir.source_edges
    }
    unknown = selected_edges - known_edges
    if unknown:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_SELECTED_EDGE_UNKNOWN,
            f"selected physical edges are absent from AnalysisBundle: {sorted(unknown)}",
        )
    selected_chain_ids = set()
    for chain in snapshot.physical_chains:
        chain_edges = frozenset(
            _host_edge_number(item) for item in chain.ordered_physical_edge_ids
        )
        overlap = selected_edges & chain_edges
        if not overlap:
            continue
        if overlap != chain_edges:
            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PARTIAL_CHAIN_SELECTION_UNSUPPORTED,
                "V0 accepts only complete PhysicalChain selection; "
                f"selected={sorted(overlap)} chain={sorted(chain_edges)}",
            )
        selected_chain_ids.add(chain.physical_chain_id)
    covered = {
        _host_edge_number(edge_id)
        for chain in snapshot.physical_chains
        if chain.physical_chain_id in selected_chain_ids
        for edge_id in chain.ordered_physical_edge_ids
    }
    if covered != selected_edges:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PARTIAL_CHAIN_SELECTION_UNSUPPORTED,
            "selected edges do not resolve to an exact set of whole PhysicalChains",
        )
    selected_use_ids = frozenset(
        item.chain_use_id
        for item in snapshot.chain_uses
        if item.physical_chain_id in selected_chain_ids
    )
    if not selected_use_ids:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EMPTY_SELECTION,
            "whole-chain selection resolved to no ChainUse",
        )
    try:
        alpha_decimal = Decimal(str(float(alpha)))
    except (InvalidOperation, ValueError) as exc:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED,
            f"requested alpha is invalid: {alpha}",
        ) from exc
    if not alpha_decimal.is_finite() or alpha_decimal < 0:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED,
            f"requested alpha must be finite and non-negative: {alpha}",
        )
    request_id = kernel.DecalRequestId(
        envelope_decal_request_id_value(
            _typed_value,
            snapshot.source_revision.value,
            selected_chain_ids,
            decal_request_id_value,
            angular_policy,
        )
    )
    request = build_envelope_request_contract(
        kernel,
        request_id,
        selected_use_ids,
        alpha_decimal,
        angular_policy,
    )
    issues = kernel.validate_decal_request(request)
    issues += kernel.validate_snapshot_request_references(snapshot, request)
    if issues:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
            "; ".join(
                f"{item.code.value}:{'.'.join(item.path)}:{item.message}"
                for item in issues
            ),
        )
    return request


def _host_diagnostic_from_stage(
    outcome: object,
    message: str,
    patch_domain_id: object | None,
) -> EnvelopeDebugHostDiagnosticV1:
    value = outcome.value if hasattr(outcome, "value") else str(outcome)
    domain_value = (
        patch_domain_id.value
        if patch_domain_id is not None and hasattr(patch_domain_id, "value")
        else None
    )
    return EnvelopeDebugHostDiagnosticV1(
        value,
        EnvelopeDebugHostSeverity.UNSUPPORTED,
        message,
        domain_value,
    )


def _scene_diagnostics(kernel, diagnostics):
    result = []
    for ordinal, diagnostic in enumerate(diagnostics):
        domain_id = (
            kernel.PatchDomainId(diagnostic.patch_domain_id)
            if diagnostic.patch_domain_id is not None
            else None
        )
        outcome = (
            diagnostic.outcome.value
            if hasattr(diagnostic.outcome, "value")
            else str(diagnostic.outcome)
        )
        token = _stable_token(
            "host-diagnostic",
            outcome,
            ordinal,
            diagnostic.patch_domain_id,
            diagnostic.message,
        )
        result.append(
            kernel.DebugDiagnosticV1(
                kernel.DebugDiagnosticId(f"host-debug-diagnostic:{token}"),
                domain_id,
                kernel.EnvelopeDebugStage.DIAGNOSTIC,
                kernel.DebugPrimitiveKind.DIAGNOSTIC,
                outcome,
                kernel.DebugDiagnosticSeverity.UNSUPPORTED,
                diagnostic.message,
                None,
                frozenset(),
                "ENV_80_DIAGNOSTICS",
                outcome,
            )
        )
    return tuple(result)


def evaluate_envelope_debug(
    analysis_bundle: AnalysisBundle,
    selected_physical_edge_ids: frozenset[int],
    alpha: float, *,
    density=None,
) -> EnvelopeDebugEvaluationV1:
    """Run V0-A full recompute and publish only one complete DebugScene."""

    timings = []
    diagnostics: list[EnvelopeDebugHostDiagnosticV1] = []
    started = time.perf_counter()
    try:
        kernel, _ = _load_kernel()
        scope = _selected_scope(
            analysis_bundle,
            selected_physical_edge_ids,
        )
        snapshot = build_envelope_analysis_snapshot(
            analysis_bundle,
            included_patch_ids=scope.patch_ids,
        )
        # Запрос собирается по ДОПОЛНЕННОМУ выделению: ядро принимает только
        # полные цепочки, и передать сюда исходное значило бы вернуть тот же
        # отказ на слое ниже.
        request = build_envelope_decal_request(
            snapshot, scope.edge_ids, alpha, density=density
        )
    except EnvelopeHostAdapterError as exc:
        timings.append(
            EnvelopeDebugTimingV1(
                "HOST_ADAPTER", time.perf_counter() - started
            )
        )
        return EnvelopeDebugEvaluationV1(
            None,
            None,
            (),
            (),
            (),
            None,
            (exc.diagnostic(),),
            tuple(timings),
        )
    except (KeyError, TypeError, ValueError) as exc:
        timings.append(
            EnvelopeDebugTimingV1(
                "HOST_ADAPTER", time.perf_counter() - started
            )
        )
        diagnostic = EnvelopeDebugHostDiagnosticV1(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
            EnvelopeDebugHostSeverity.ERROR,
            f"AnalysisBundle cannot satisfy the V0 host contract: {exc}",
        )
        return EnvelopeDebugEvaluationV1(
            None,
            None,
            (),
            (),
            (),
            None,
            (diagnostic,),
            tuple(timings),
        )
    timings.append(
        EnvelopeDebugTimingV1("HOST_ADAPTER", time.perf_counter() - started)
    )

    use_by_id = {item.chain_use_id: item for item in snapshot.chain_uses}
    domain_ids = tuple(
        sorted(
            {
                use_by_id[use_id].patch_domain_id
                for use_id in request.selected_chain_use_ids
            },
            key=lambda item: item.value,
        )
    )
    compilations = []
    raw_results = []
    interaction_results = []

    started = time.perf_counter()
    for domain_id in domain_ids:
        compile_result = kernel.compile_reference_envelopes(
            snapshot, request, domain_id
        )
        if compile_result.compilation is None:
            diagnostics.extend(
                _host_diagnostic_from_stage(
                    item.outcome, item.message, domain_id
                )
                for item in compile_result.diagnostics
            )
            continue
        compilations.append(compile_result.compilation)
    timings.append(
        EnvelopeDebugTimingV1("COMPILE", time.perf_counter() - started)
    )

    started = time.perf_counter()
    for compilation in compilations:
        result = kernel.evaluate_reference_raw_coverage(
            compilation, request.requested_alpha
        )
        if result.raw_coverage is None:
            diagnostics.extend(
                _host_diagnostic_from_stage(
                    item.outcome,
                    item.message,
                    compilation.plan_key.patch_domain_id,
                )
                for item in result.diagnostics
            )
            continue
        raw_results.append(result.raw_coverage)
    timings.append(
        EnvelopeDebugTimingV1("RAW_COVERAGE", time.perf_counter() - started)
    )

    compilation_by_domain = {
        item.plan_key.patch_domain_id: item for item in compilations
    }
    started = time.perf_counter()
    for raw in raw_results:
        result = kernel.resolve_coverage_interactions(
            compilation_by_domain[raw.plan_key.patch_domain_id],
            raw.boundary_resolved_envelopes,
            raw,
        )
        interaction_results.append(result)
        if result.resolved_coverage is None:
            diagnostics.extend(
                _host_diagnostic_from_stage(
                    item.outcome,
                    item.message,
                    raw.plan_key.patch_domain_id,
                )
                for item in result.diagnostics
            )
    timings.append(
        EnvelopeDebugTimingV1("INTERACTION", time.perf_counter() - started)
    )

    started = time.perf_counter()
    try:
        scene = kernel.build_envelope_debug_scene(
            snapshot,
            request,
            tuple(compilations),
            tuple(raw_results),
            tuple(interaction_results),
            _scene_diagnostics(kernel, diagnostics),
        )
    except Exception as exc:
        diagnostics.append(
            EnvelopeDebugHostDiagnosticV1(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED,
                EnvelopeDebugHostSeverity.ERROR,
                f"DebugScene projection failed without fallback: {exc}",
            )
        )
        scene = None
    timings.append(
        EnvelopeDebugTimingV1("DEBUG_SCENE", time.perf_counter() - started)
    )
    return EnvelopeDebugEvaluationV1(
        snapshot,
        request,
        tuple(compilations),
        tuple(raw_results),
        tuple(interaction_results),
        scene,
        tuple(diagnostics),
        tuple(timings),
    )


def _receipt_for_failure(
    patch_id: int,
    patch_domain_id: str,
    stage: EnvelopeDomainStage,
    diagnostic: EnvelopeDebugHostDiagnosticV1,
) -> EnvelopeDomainStageReceiptV1:
    outcome = (
        diagnostic.outcome.value
        if hasattr(diagnostic.outcome, "value")
        else str(diagnostic.outcome)
    )
    return EnvelopeDomainStageReceiptV1(
        patch_id,
        patch_domain_id,
        stage,
        outcome,
        diagnostic.message,
    )


def _profile_raw_telemetry(profile, patch_domain_id: str):
    def record(
        stage: str,
        elapsed_seconds: float,
        counters: dict[str, int | float] | None = None,
    ) -> None:
        profile.add_timing(stage, elapsed_seconds, patch_domain_id)
        for name, value in (counters or {}).items():
            profile.set_counter(name, value, patch_domain_id)

    return record


def evaluate_envelope_debug_staged(
    analysis_bundle: AnalysisBundle,
    selected_physical_edge_ids: frozenset[int],
    alpha: float,
    *,
    profile: EnvelopeDebugProfileBuilderV1 | None = None,
    topology_export=None,
    domain_snapshot_provider=None,
    density,
) -> EnvelopeDebugStagedEvaluationV1:
    """Evaluate exact reference coverage per domain after host topology."""

    from .envelope_topology_export import stage_domain_inputs

    if profile is None:
        profile = EnvelopeDebugProfileBuilderV1(
            analysis_bundle.source_revision.source_name,
            "EXACT_REFERENCE",
        )
    (
        topology_scene,
        revision,
        patch_ids,
        global_request_id,
        selected_edges_by_domain,
    ) = stage_domain_inputs(
        analysis_bundle,
        selected_physical_edge_ids,
        profile=profile,
        topology_export=topology_export,
    )
    domain_evaluations = []

    try:
        kernel, _ = _load_kernel()
    except EnvelopeHostAdapterError as exc:
        diagnostic = exc.diagnostic()
        for patch_id in patch_ids:
            domain_id = _typed_value(
                "patch-domain",
                revision,
                patch_id,
            )
            receipt = _receipt_for_failure(
                patch_id,
                domain_id,
                EnvelopeDomainStage.COMPILE_REJECTED,
                diagnostic,
            )
            profile.set_receipt(receipt)
            domain_evaluations.append(
                EnvelopeDebugDomainEvaluationV1(
                    patch_id,
                    domain_id,
                    None,
                    None,
                    None,
                    None,
                    None,
                    None,
                    receipt,
                    (diagnostic,),
                )
            )
        return EnvelopeDebugStagedEvaluationV1(
            topology_scene,
            tuple(domain_evaluations),
        )

    for patch_id in patch_ids:
        domain_id = _typed_value("patch-domain", revision, patch_id)
        snapshot = None
        request = None
        compilation = None
        raw_result = None
        interaction_result = None
        debug_scene = None
        diagnostics: list[EnvelopeDebugHostDiagnosticV1] = []

        try:
            with _measure(profile, "SNAPSHOT_EXPORT", domain_id):
                if domain_snapshot_provider is None:
                    snapshot = build_envelope_analysis_snapshot(
                        analysis_bundle,
                        included_patch_ids=frozenset({patch_id}),
                        profile=profile,
                        topology_export=topology_export,
                    )
                else:
                    snapshot = domain_snapshot_provider(
                        patch_id,
                        domain_id,
                    )
                request = build_envelope_decal_request(
                    snapshot,
                    frozenset(selected_edges_by_domain[domain_id]),
                    alpha,
                    decal_request_id_value=global_request_id,
                    density=density,
                )
        except EnvelopeHostAdapterError as exc:
            diagnostic = exc.diagnostic()
            diagnostics.append(diagnostic)
            stage = (
                EnvelopeDomainStage.METRIC_REJECTED
                if exc.outcome
                in {
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
                    EnvelopeDebugHostOutcome.RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED,
                }
                else EnvelopeDomainStage.COMPILE_REJECTED
            )
            receipt = _receipt_for_failure(
                patch_id,
                domain_id,
                stage,
                diagnostic,
            )
            profile.set_receipt(receipt)
            domain_evaluations.append(
                EnvelopeDebugDomainEvaluationV1(
                    patch_id,
                    domain_id,
                    snapshot,
                    request,
                    None,
                    None,
                    None,
                    None,
                    receipt,
                    tuple(diagnostics),
                )
            )
            continue

        # Величина, которая на самом деле объясняет полевое время. Профиль bf6:
        # `_canonical_expr` — 95 с из 158, и это не предикаты (интервальный
        # фильтр знака отдаёт в символьный путь 33 запроса из 13347), а
        # конструирование: каждое алгебраическое значение канонизируется через
        # factor(cancel(...)) по ~6 мс против 0.25 мкс у рационального.
        fallbacks = importlib.import_module(
            "cftuv_envelope.reference.planar_types"
        ).SYMBOLIC_FALLBACK_COUNTS
        algebraic_before = fallbacks["canonical"] + fallbacks["normalize"]
        # Наблюдение за привязкой к решётке. Заведено до самой решётки: срез,
        # который её вводит, иначе нечем принимать. Сейчас печатает нули, и это
        # ответ «привязок не было», а не отсутствие измерения.
        grid = importlib.import_module("cftuv_envelope.robust.grid")
        grid.reset_snap_counts()
        with _measure(profile, "COMPILE", domain_id):
            compile_result = kernel.compile_reference_envelopes(
                snapshot,
                request,
            )
        if compile_result.compilation is None:
            diagnostics.extend(
                _host_diagnostic_from_stage(
                    item.outcome,
                    item.message,
                    next(iter(snapshot.patch_domains)).patch_domain_id,
                )
                for item in compile_result.diagnostics
            )
            diagnostic = diagnostics[0]
            receipt = _receipt_for_failure(
                patch_id,
                domain_id,
                EnvelopeDomainStage.COMPILE_REJECTED,
                diagnostic,
            )
        else:
            compilation = compile_result.compilation
            profile.set_counter(
                "ENVELOPE_SPECS",
                len(compilation.envelope_specs),
                domain_id,
            )
            raw_callable = kernel.evaluate_reference_raw_coverage
            if "telemetry" in inspect.signature(raw_callable).parameters:
                raw_evaluation = raw_callable(
                    compilation,
                    request.requested_alpha,
                    telemetry=_profile_raw_telemetry(
                        profile,
                        domain_id,
                    ),
                )
            else:
                with _measure(profile, "RAW_UNION", domain_id):
                    raw_evaluation = raw_callable(
                        compilation,
                        request.requested_alpha,
                    )
            if raw_evaluation.raw_coverage is None:
                diagnostics.extend(
                    _host_diagnostic_from_stage(
                        item.outcome,
                        item.message,
                        compilation.plan_key.patch_domain_id,
                    )
                    for item in raw_evaluation.diagnostics
                )
                diagnostic = diagnostics[0]
                receipt = _receipt_for_failure(
                    patch_id,
                    domain_id,
                    EnvelopeDomainStage.RAW_REJECTED,
                    diagnostic,
                )
            else:
                raw_result = raw_evaluation.raw_coverage
                with _measure(profile, "INTERACTION", domain_id):
                    interaction_result = kernel.resolve_coverage_interactions(
                        compilation,
                        raw_result.boundary_resolved_envelopes,
                        raw_result,
                    )
                # Наблюдение по возвращённому контракту, как ENVELOPE_SPECS выше.
                # INTERACTION — самая дорогая стадия в поле, и без этих чисел
                # она остаётся единственной, про которую нечего сказать.
                for name, value in (
                    ("INTERACTION_COMPONENTS", interaction_result.interaction_components),
                    ("INTERACTION_ARRIVAL_MODELS", interaction_result.arrival_models),
                    ("INTERACTION_CANDIDATES", interaction_result.candidates),
                ):
                    profile.set_counter(name, len(value), domain_id)
                for name, value in (
                    ("SNAP_POINTS_TOTAL", grid.SNAP_COUNTS["points_total"]),
                    ("SNAP_POINTS_MOVED", grid.SNAP_COUNTS["points_moved"]),
                    ("SNAP_MERGED_POINTS", grid.SNAP_COUNTS["merged_points"]),
                    (
                        "SNAP_DISPLACEMENT_MAX_SQUARED",
                        float(grid.SNAP_DISPLACEMENT_MAX_SQUARED[0]),
                    ),
                ):
                    profile.set_counter(name, value, domain_id)
                # Ненадзорные диагностики ядра пробрасываются НА УСПЕХЕ тоже.
                # Раньше `diagnostics.extend(...)` стоял только в ветках отказа,
                # поэтому именованное событие на разрешившемся домене исчезало
                # молча — печать в рендерере его бы всё равно не увидела.
                diagnostics.extend(
                    _host_diagnostic_from_stage(
                        item.outcome,
                        item.message,
                        compilation.plan_key.patch_domain_id,
                    )
                    for item in interaction_result.diagnostics
                    if item.severity is not kernel.InteractionDiagnosticSeverity.UNSUPPORTED
                )
                profile.set_counter(
                    "ALGEBRAIC_CANONICALIZATIONS",
                    fallbacks["canonical"]
                    + fallbacks["normalize"]
                    - algebraic_before,
                    domain_id,
                )
                if interaction_result.resolved_coverage is None:
                    diagnostics.extend(
                        _host_diagnostic_from_stage(
                            item.outcome,
                            item.message,
                            compilation.plan_key.patch_domain_id,
                        )
                        for item in interaction_result.diagnostics
                    )
                    diagnostic = diagnostics[0]
                    receipt = _receipt_for_failure(
                        patch_id,
                        domain_id,
                        EnvelopeDomainStage.INTERACTION_REJECTED,
                        diagnostic,
                    )
                else:
                    resolved = interaction_result.resolved_coverage
                    for name, value in (
                        ("RESOLVED_REGIONS", resolved.regions),
                        ("RESOLVED_EDGES", resolved.edges),
                        ("INTERACTION_APPLICATIONS", resolved.interaction_applications),
                        ("INTERACTION_EQUALITY_LOCI", resolved.equality_loci),
                    ):
                        profile.set_counter(name, len(value), domain_id)
                    receipt = EnvelopeDomainStageReceiptV1(
                        patch_id,
                        domain_id,
                        EnvelopeDomainStage.RESOLVED,
                        EnvelopeDomainStage.RESOLVED.value,
                        "Exact reference Envelope reached ResolvedCoverage",
                    )

        profile.set_receipt(receipt)
        with _measure(profile, "DEBUG_SCENE", domain_id):
            try:
                debug_scene = kernel.build_envelope_debug_scene(
                    snapshot,
                    request,
                    (compilation,) if compilation is not None else (),
                    (raw_result,) if raw_result is not None else (),
                    (
                        (interaction_result,)
                        if interaction_result is not None
                        else ()
                    ),
                    _scene_diagnostics(kernel, diagnostics),
                )
            except (KeyError, TypeError, ValueError) as exc:
                diagnostics.append(
                    EnvelopeDebugHostDiagnosticV1(
                        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED,
                        EnvelopeDebugHostSeverity.ERROR,
                        f"DebugScene projection failed without fallback: {exc}",
                        domain_id,
                    )
                )
        domain_evaluations.append(
            EnvelopeDebugDomainEvaluationV1(
                patch_id,
                domain_id,
                snapshot,
                request,
                compilation,
                raw_result,
                interaction_result,
                debug_scene,
                receipt,
                tuple(diagnostics),
            )
        )

    return EnvelopeDebugStagedEvaluationV1(
        topology_scene,
        tuple(domain_evaluations),
    )


__all__ = (
    "EnvelopeDebugDomainEvaluationV1",
    "EnvelopeDebugEvaluationV1",
    "EnvelopeDebugHostDiagnosticV1",
    "EnvelopeDebugHostOutcome",
    "EnvelopeDebugHostSeverity",
    "EnvelopeDebugStagedEvaluationV1",
    "EnvelopeDebugTimingV1",
    "EnvelopeHostAdapterError",
    "build_envelope_analysis_snapshot",
    "build_envelope_decal_request",
    "build_envelope_topology_debug_scene",
    "evaluate_envelope_debug",
    "evaluate_envelope_debug_staged",
)
