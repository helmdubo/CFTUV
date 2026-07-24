"""Immutable host-only scene contract for Envelope topology diagnostics."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


Vec3 = tuple[float, float, float]

ENVELOPE_TOPOLOGY_DEBUG_SCENE_SCHEMA_V1 = (
    "cftuv.envelope.topology_debug_scene.v1"
)


class EnvelopeTopologyPathKind(str, Enum):
    PATCH_OUTER_LOOP = "PATCH_OUTER_LOOP"
    PATCH_HOLE_LOOP = "PATCH_HOLE_LOOP"
    PHYSICAL_CHAIN = "PHYSICAL_CHAIN"
    DIRECTED_CHAIN_USE = "DIRECTED_CHAIN_USE"
    SELECTED_SOURCE = "SELECTED_SOURCE"


class EnvelopeTopologyPairKind(str, Enum):
    PATCH = "PATCH"
    SEAM_SELF = "SEAM_SELF"


@dataclass(frozen=True, slots=True)
class EnvelopeTopologyDebugPathV1:
    semantic_id: str
    patch_domain_id: str | None
    kind: EnvelopeTopologyPathKind
    local_points: tuple[Vec3, ...]
    host_vertex_ids: tuple[int, ...]
    host_edge_ids: tuple[int, ...]
    closed: bool
    directed: bool
    selected: bool
    style_key: str
    label: str
    physical_chain_id: str | None = None
    chain_use_id: str | None = None
    boundary_loop_id: str | None = None
    display_normal: Vec3 = (0.0, 0.0, 0.0)


@dataclass(frozen=True, slots=True)
class EnvelopeTopologyDebugPairV1:
    pair_id: str
    kind: EnvelopeTopologyPairKind
    physical_chain_id: str
    chain_use_ids: tuple[str, ...]
    patch_domain_ids: tuple[str, ...]
    host_edge_ids: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class EnvelopeTopologySelectionDiagnosticV1:
    code: str
    message: str
    patch_domain_id: str | None = None
    physical_chain_id: str | None = None


@dataclass(frozen=True, slots=True)
class EnvelopeTopologyDebugSceneV1:
    schema_version: str
    source_revision: str
    patch_domain_ids: tuple[str, ...]
    selected_physical_edge_ids: tuple[int, ...]
    paths: tuple[EnvelopeTopologyDebugPathV1, ...]
    pairs: tuple[EnvelopeTopologyDebugPairV1, ...]
    selection_diagnostics: tuple[
        EnvelopeTopologySelectionDiagnosticV1, ...
    ]

    def __post_init__(self) -> None:
        if self.schema_version != ENVELOPE_TOPOLOGY_DEBUG_SCENE_SCHEMA_V1:
            raise ValueError(
                f"unsupported topology debug schema: {self.schema_version}"
            )
        domain_ids = set(self.patch_domain_ids)
        if not domain_ids:
            raise ValueError("topology debug scene requires PatchDomains")
        for path in self.paths:
            if path.patch_domain_id is not None:
                if path.patch_domain_id not in domain_ids:
                    raise ValueError(
                        f"path {path.semantic_id} references an unknown domain"
                    )
            if len(path.local_points) < 2:
                raise ValueError(
                    f"path {path.semantic_id} requires at least two points"
                )
            if len(path.host_edge_ids) not in {
                len(path.host_vertex_ids),
                len(path.host_vertex_ids) - 1,
            }:
                raise ValueError(
                    f"path {path.semantic_id} has inconsistent host topology"
                )
        for pair in self.pairs:
            if len(pair.chain_use_ids) != 2:
                raise ValueError(
                    f"pair {pair.pair_id} must contain two directed ChainUses"
                )
            if pair.kind is EnvelopeTopologyPairKind.PATCH:
                if len(set(pair.patch_domain_ids)) != 2:
                    raise ValueError(
                        f"PATCH pair {pair.pair_id} requires two domains"
                    )
            elif len(set(pair.patch_domain_ids)) != 1:
                raise ValueError(
                    f"SEAM_SELF pair {pair.pair_id} requires one domain"
                )


__all__ = (
    "ENVELOPE_TOPOLOGY_DEBUG_SCENE_SCHEMA_V1",
    "EnvelopeTopologyDebugPairV1",
    "EnvelopeTopologyDebugPathV1",
    "EnvelopeTopologyDebugSceneV1",
    "EnvelopeTopologyPairKind",
    "EnvelopeTopologyPathKind",
    "EnvelopeTopologySelectionDiagnosticV1",
)
