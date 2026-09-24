"""Immutable host-topology export for Envelope debug.

The export owns PhysicalChain common refinement and directed ChainUse
collection for one SourceRevision.  Request selection is a cheap view over
this immutable result; it never rebuilds host analysis.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import TYPE_CHECKING, Mapping

from .envelope_debug_profile import EnvelopeDebugProfileBuilderV1

if TYPE_CHECKING:
    from .surface_ir import AnalysisBundle


#: Ключ канонической PhysicalChain: замкнутость плюс рёбра и вершины.
HostChainKey = tuple[bool, tuple[int, ...], tuple[int, ...]]

#: Счётчики дополнения выделения. Объявлены перечнем и пишутся ВСЕГДА, даже
#: нулями: иначе «выделение не дополнялось» неотличимо от «дополнение не
#: измерялось» — тот же дефект, который уже лечили счётчиками стадии углов.
SELECTION_COMPLETION_COUNTERS = (
    "SELECTION_COMPLETED_CHAINS",
    "SELECTION_COMPLETED_EDGES",
)

#: Код диагностики сцены топологии о том, что выделение было дополнено.
SELECTION_COMPLETED_DIAGNOSTIC_CODE = "PARTIAL_CHAIN_SELECTION_COMPLETED"


@dataclass(frozen=True, slots=True)
class EnvelopeTopologyExportV1:
    """SourceRevision-scoped host topology prepared exactly once."""

    source_revision_value: str
    analysis_bundle: AnalysisBundle
    host_chains: tuple[object, ...]
    patch_domain_id_by_patch: Mapping[int, str]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "patch_domain_id_by_patch",
            MappingProxyType(dict(self.patch_domain_id_by_patch)),
        )


@dataclass(frozen=True, slots=True)
class _PatchGraphIdView:
    source_revision: object
    nodes: Mapping[int, object]
    edges: Mapping[object, object]

    def __post_init__(self) -> None:
        object.__setattr__(self, "nodes", MappingProxyType(dict(self.nodes)))
        object.__setattr__(self, "edges", MappingProxyType(dict(self.edges)))


@dataclass(frozen=True, slots=True)
class _PatchSurfaceIdView:
    source_revision: object
    vertices: tuple[object, ...]
    edges: tuple[object, ...]
    faces: tuple[object, ...]
    triangles: tuple[object, ...]

    @property
    def vertex_by_id(self) -> dict[int, object]:
        return {int(item.vertex_id): item for item in self.vertices}

    def patch_faces(self, patch_id: int) -> tuple[object, ...]:
        return tuple(
            item for item in self.faces if int(item.patch_id) == int(patch_id)
        )


@dataclass(frozen=True, slots=True)
class AnalysisBundleIdView:
    """Immutable patch-ID view over a full AnalysisBundle.

    The view stores no copied PatchGraph or PatchSurfaceIR records.  Exporters
    filter the referenced full bundle by ``included_patch_ids``.
    """

    analysis_bundle: AnalysisBundle
    included_patch_ids: frozenset[int]
    _patch_graph: _PatchGraphIdView = field(init=False, repr=False)
    _patch_surface: _PatchSurfaceIdView = field(init=False, repr=False)

    def __post_init__(self) -> None:
        available = frozenset(
            int(value) for value in self.analysis_bundle.patch_graph.nodes
        )
        unknown = self.included_patch_ids - available
        if not self.included_patch_ids or unknown:
            from .envelope_request_export import (
                EnvelopeDebugHostOutcome,
                EnvelopeHostAdapterError,
            )

            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
                "request-scoped PatchDomain set is empty or unknown: "
                f"{sorted(self.included_patch_ids)}",
            )
        graph = self.analysis_bundle.patch_graph
        surface = self.analysis_bundle.patch_surface
        nodes = {
            int(patch_id): node
            for patch_id, node in graph.nodes.items()
            if int(patch_id) in self.included_patch_ids
        }
        edges = {
            key: edge
            for key, edge in graph.edges.items()
            if (
                int(edge.patch_a_id) in self.included_patch_ids
                and int(edge.patch_b_id) in self.included_patch_ids
            )
        }
        faces = tuple(
            item
            for item in surface.faces
            if int(item.patch_id) in self.included_patch_ids
        )
        face_ids = frozenset(int(item.face_id) for item in faces)
        edge_ids = frozenset(
            int(edge_id) for face in faces for edge_id in face.edge_cycle
        )
        vertex_ids = frozenset(
            int(vertex_id) for face in faces for vertex_id in face.vertex_cycle
        )
        object.__setattr__(
            self,
            "_patch_graph",
            _PatchGraphIdView(self.source_revision, nodes, edges),
        )
        object.__setattr__(
            self,
            "_patch_surface",
            _PatchSurfaceIdView(
                self.source_revision,
                tuple(
                    item
                    for item in surface.vertices
                    if int(item.vertex_id) in vertex_ids
                ),
                tuple(
                    item
                    for item in surface.edges
                    if int(item.edge_id) in edge_ids
                ),
                faces,
                tuple(
                    item
                    for item in surface.triangles
                    if int(item.source_face_id) in face_ids
                ),
            ),
        )

    @property
    def source_revision(self):
        return self.analysis_bundle.source_revision

    @property
    def capabilities(self):
        return self.analysis_bundle.capabilities

    @property
    def patch_graph(self):
        return self._patch_graph

    @property
    def patch_surface(self):
        return self._patch_surface


def build_analysis_bundle_id_view(
    analysis_bundle: AnalysisBundle,
    included_patch_ids: frozenset[int],
) -> AnalysisBundleIdView:
    return AnalysisBundleIdView(
        analysis_bundle,
        frozenset(int(item) for item in included_patch_ids),
    )


@dataclass(frozen=True, slots=True)
class EnvelopeSelectionScopeV1:
    """Выделение владельца, дополненное до полных PhysicalChain.

    `requested_edge_ids` — то, что стояло выделенным во вьюпорте; `edge_ids` —
    то, чем считает движок. Два поля, а не одно, намеренно: восстановление
    выделения владельца обязано вернуть ИСХОДНОЕ, и одно поле сделало бы
    возврат исходного неотличимым от возврата дополненного.
    """

    requested_edge_ids: frozenset[int]
    edge_ids: frozenset[int]
    chain_keys: frozenset[HostChainKey]
    patch_ids: frozenset[int]
    #: Цепочки, которые были выделены ЧАСТИЧНО и достроены до полных.
    completed_chain_keys: tuple[HostChainKey, ...]

    @property
    def added_edge_ids(self) -> frozenset[int]:
        return self.edge_ids - self.requested_edge_ids

    @property
    def completed(self) -> bool:
        return bool(self.completed_chain_keys)


def _complete_selection_to_whole_chains(
    selected_edges: frozenset[int],
    chain_groups: Mapping[HostChainKey, list],
) -> tuple[frozenset[int], tuple[HostChainKey, ...]]:
    """Дополнить выделение до объединения ПОЛНЫХ задетых цепочек.

    Владелец выделяет рёбра мышью, а не цепочки: на меше, где шов между двумя
    патчами разбит вершинами на четыре ребра, попадание в одно из них — норма,
    а не ошибка. Прежний жёсткий отказ гасил при этом ВЕСЬ билд, и владелец
    видел warning вместо результата.
    """

    completed = set(selected_edges)
    completed_keys = []
    for key in sorted(chain_groups):
        chain_edges = frozenset(key[1])
        overlap = selected_edges & chain_edges
        if not overlap:
            continue
        if overlap != chain_edges:
            completed_keys.append(key)
        completed.update(chain_edges)
    return frozenset(completed), tuple(completed_keys)


def resolve_selection_scope(
    analysis_bundle: AnalysisBundle,
    selected_physical_edge_ids: frozenset[int],
    host_chains: tuple[object, ...],
    *,
    profile: EnvelopeDebugProfileBuilderV1 | None = None,
) -> EnvelopeSelectionScopeV1:
    """Разрешить выделение владельца в полные цепочки и домены."""

    from .envelope_request_export import (
        EnvelopeDebugHostOutcome,
        EnvelopeHostAdapterError,
        _group_host_chains,
        _validate_chain_group_topology,
    )

    if not selected_physical_edge_ids:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EMPTY_SELECTION,
            "Envelope debug requires at least one selected physical edge",
        )
    requested = frozenset(int(item) for item in selected_physical_edge_ids)
    known_edges = {
        int(item.edge_id) for item in analysis_bundle.patch_surface.edges
    }
    unknown = requested - known_edges
    if unknown:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_SELECTED_EDGE_UNKNOWN,
            "selected physical edges are absent from AnalysisBundle: "
            f"{sorted(unknown)}",
        )

    chain_groups = _group_host_chains(host_chains)
    selected_edges, completed_keys = _complete_selection_to_whole_chains(
        requested,
        chain_groups,
    )
    if profile is not None:
        profile.set_counter(
            "SELECTION_COMPLETED_CHAINS",
            len(completed_keys),
        )
        profile.set_counter(
            "SELECTION_COMPLETED_EDGES",
            len(selected_edges - requested),
        )
    selected_keys = {
        key
        for key in chain_groups
        if selected_edges & frozenset(key[1])
    }
    covered = {edge_id for key in selected_keys for edge_id in key[1]}
    off_chain = sorted(requested - covered)
    if off_chain:
        # Дополнение здесь НЕВОЗМОЖНО: ребро принадлежит поверхности, но не
        # входит ни в одну граничную цепочку ни одного патча — то есть лежит
        # внутри патча. Достраивать его не до чего.
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_SELECTED_EDGE_OFF_PHYSICAL_CHAIN,
            "selected physical edges belong to no PhysicalChain and cannot be "
            f"completed: {off_chain}",
        )

    selected_patch_ids = frozenset(
        record.patch_id
        for key in selected_keys
        for record in chain_groups[key]
    )
    if not selected_patch_ids:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EMPTY_SELECTION,
            "whole-chain selection resolved to no PatchDomain",
        )

    # Exact-frame admission is request-scoped, but topology is not guessed.
    # Every seam touching a selected domain must still have its real counterpart
    # in the original AnalysisBundle before the opposite unselected domain is
    # omitted from the evaluation slice.
    for records in chain_groups.values():
        if any(record.patch_id in selected_patch_ids for record in records):
            _validate_chain_group_topology(records)
    return EnvelopeSelectionScopeV1(
        requested,
        selected_edges,
        frozenset(selected_keys),
        selected_patch_ids,
        completed_keys,
    )


def build_envelope_topology_export(
    analysis_bundle: AnalysisBundle,
    *,
    profile: EnvelopeDebugProfileBuilderV1 | None = None,
) -> EnvelopeTopologyExportV1:
    """Collect and normalize host chains once for one SourceRevision."""

    from .envelope_request_export import (
        _collect_host_chains,
        _revision_value,
        _typed_value,
    )

    revision = _revision_value(analysis_bundle.source_revision)
    host_chains = _collect_host_chains(
        analysis_bundle,
        profile=profile,
    )
    return EnvelopeTopologyExportV1(
        revision,
        analysis_bundle,
        host_chains,
        {
            int(patch_id): _typed_value(
                "patch-domain",
                revision,
                int(patch_id),
            )
            for patch_id in analysis_bundle.patch_graph.nodes
        },
    )


def build_envelope_topology_debug_scene(
    analysis_bundle: AnalysisBundle,
    selected_physical_edge_ids: frozenset[int],
    *,
    profile: EnvelopeDebugProfileBuilderV1 | None = None,
    topology_export: EnvelopeTopologyExportV1 | None = None,
):
    """Build a request-scoped display scene from cached host topology."""

    from .envelope_request_export import (
        build_envelope_topology_debug_scene as _build_scene,
    )

    export = topology_export or build_envelope_topology_export(
        analysis_bundle,
        profile=profile,
    )
    if export.analysis_bundle.source_revision != analysis_bundle.source_revision:
        raise ValueError("topology export SourceRevision does not match bundle")
    return _build_scene(
        analysis_bundle,
        selected_physical_edge_ids,
        profile=profile,
        topology_export=export,
    )


def stage_domain_inputs(
    analysis_bundle: AnalysisBundle,
    selected_physical_edge_ids: frozenset[int],
    *,
    profile: EnvelopeDebugProfileBuilderV1 | None = None,
    topology_export: EnvelopeTopologyExportV1 | None = None,
):
    """Сцена топологии плюс адресация доменов одного постадийного прогона.

    Возвращает `(topology_scene, revision, patch_ids, decal_request_id,
    selected_edges_by_domain)`.

    Общая точка ОБОИХ движков (LEGACY и QUEUE) намеренно: одинаковая нумерация
    доменов и один `DecalRequestId` — это и есть то, чем их результаты
    сравнимы. Две разошедшиеся копии этого пролога сделали бы сравнение
    свойством копии, а не свойством движков.
    """

    from .envelope_request_export import (
        _revision_value,
        _typed_value,
        build_envelope_topology_debug_scene as _build_scene,
    )
    from .envelope_topology_debug import EnvelopeTopologyPathKind

    topology_scene = _build_scene(
        analysis_bundle,
        selected_physical_edge_ids,
        profile=profile,
        topology_export=topology_export,
    )
    revision = _revision_value(analysis_bundle.source_revision)
    patch_ids = tuple(
        sorted(
            int(patch_id)
            for patch_id in analysis_bundle.patch_graph.nodes
            if _typed_value("patch-domain", revision, int(patch_id))
            in topology_scene.patch_domain_ids
        )
    )
    decal_request_id = _typed_value(
        "decal-request",
        revision,
        tuple(
            sorted(
                path.physical_chain_id
                for path in topology_scene.paths
                if path.kind is EnvelopeTopologyPathKind.SELECTED_SOURCE
                and path.physical_chain_id is not None
            )
        ),
    )
    selected_edges_by_domain: dict[str, set[int]] = {
        domain_id: set()
        for domain_id in topology_scene.patch_domain_ids
    }
    for path in topology_scene.paths:
        if (
            path.kind is EnvelopeTopologyPathKind.DIRECTED_CHAIN_USE
            and path.selected
            and path.patch_domain_id is not None
        ):
            selected_edges_by_domain[path.patch_domain_id].update(
                path.host_edge_ids
            )
    return (
        topology_scene,
        revision,
        patch_ids,
        decal_request_id,
        selected_edges_by_domain,
    )


__all__ = (
    "SELECTION_COMPLETED_DIAGNOSTIC_CODE",
    "SELECTION_COMPLETION_COUNTERS",
    "AnalysisBundleIdView",
    "EnvelopeSelectionScopeV1",
    "EnvelopeTopologyExportV1",
    "HostChainKey",
    "build_analysis_bundle_id_view",
    "build_envelope_topology_debug_scene",
    "build_envelope_topology_export",
    "resolve_selection_scope",
    "stage_domain_inputs",
)
