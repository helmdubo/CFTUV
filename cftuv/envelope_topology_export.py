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


__all__ = (
    "AnalysisBundleIdView",
    "EnvelopeTopologyExportV1",
    "build_analysis_bundle_id_view",
    "build_envelope_topology_debug_scene",
    "build_envelope_topology_export",
)
