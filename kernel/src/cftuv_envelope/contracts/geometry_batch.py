"""Immutable output boundary shared by future GPU and BMesh adapters."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..ids import (
    ChainUseId,
    ContractVersionId,
    DecalRequestId,
    GeometryDiagnosticId,
    GeometryFaceId,
    LineageId,
    MaterialId,
    OwnershipClaimId,
    PatchDomainId,
    PhysicalEdgeId,
    SemanticBoundaryId,
    SemanticDigestValue,
    SemanticInterfaceId,
    SemanticLocationId,
    SemanticRegionId,
    SourceFaceId,
    SourceRevision,
    VertexKey,
)
from ..numeric import LocalPoint3V1, UvPoint2V1
from ..outcomes import NamedOutcome


GEOMETRY_BATCH_SCHEMA_V1 = "cftuv.envelope.geometry_batch.v1"


class GeometryDiagnosticSeverity(str, Enum):
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"


@dataclass(frozen=True, slots=True)
class GeometryProvenanceV1:
    source_face_ids: frozenset[SourceFaceId]
    physical_edge_ids: frozenset[PhysicalEdgeId]
    chain_use_ids: frozenset[ChainUseId]
    lineage_ids: frozenset[LineageId]


@dataclass(frozen=True, slots=True)
class GeometryVertexV1:
    vert_key: VertexKey
    position: LocalPoint3V1
    semantic_location_ref: SemanticLocationId
    provenance: GeometryProvenanceV1


@dataclass(frozen=True, slots=True)
class GeometryUvFactV1:
    vert_key: VertexKey
    uv: UvPoint2V1


@dataclass(frozen=True, slots=True)
class GeometryFaceV1:
    face_id: GeometryFaceId
    ordered_vert_keys: tuple[VertexKey, ...]
    uv_facts: tuple[GeometryUvFactV1, ...]
    semantic_region_id: SemanticRegionId
    ownership_claim_id: OwnershipClaimId
    provenance: GeometryProvenanceV1
    material_id: MaterialId

    def __post_init__(self) -> None:
        if len(self.ordered_vert_keys) < 3:
            raise ValueError("GeometryFaceV1 requires at least three vertices")
        if tuple(fact.vert_key for fact in self.uv_facts) != self.ordered_vert_keys:
            raise ValueError("GeometryFaceV1 UV facts must follow the ordered vertex cycle")


@dataclass(frozen=True, slots=True)
class GeometrySemanticRegionV1:
    semantic_region_id: SemanticRegionId
    ownership_claim_id: OwnershipClaimId
    material_id: MaterialId
    provenance: GeometryProvenanceV1


@dataclass(frozen=True, slots=True)
class GeometryBoundaryChainV1:
    semantic_boundary_id: SemanticBoundaryId
    ordered_vert_keys: tuple[VertexKey, ...]


@dataclass(frozen=True, slots=True)
class GeometryInterfaceChainV1:
    semantic_interface_id: SemanticInterfaceId
    ordered_vert_keys: tuple[VertexKey, ...]


@dataclass(frozen=True, slots=True)
class GeometryDiagnosticV1:
    diagnostic_id: GeometryDiagnosticId
    severity: GeometryDiagnosticSeverity
    outcome: NamedOutcome
    provenance: frozenset[LineageId]


@dataclass(frozen=True, slots=True)
class GeometryBatchV1:
    schema_version: str
    source_revision: SourceRevision
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    vertices: frozenset[GeometryVertexV1]
    faces: tuple[GeometryFaceV1, ...]
    semantic_regions: frozenset[GeometrySemanticRegionV1]
    boundary_chains: frozenset[GeometryBoundaryChainV1]
    interface_chains: frozenset[GeometryInterfaceChainV1]
    diagnostics: frozenset[GeometryDiagnosticV1]
    contract_versions: frozenset[ContractVersionId]
    semantic_digest: SemanticDigestValue

