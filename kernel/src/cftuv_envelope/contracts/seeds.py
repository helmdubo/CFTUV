"""Request-specific compile records; seeds не содержат emitted geometry."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..ids import (
    CapSeedId,
    ChainUseId,
    CornerRelationId,
    CornerSeedId,
    DecalRequestId,
    EndpointClaimSeedId,
    FrontSeedId,
    JunctionRelationId,
    JunctionSeedId,
    OwnerSectorId,
    PatchDomainId,
    PatchId,
    SelectionCertificateId,
    SharedSemanticAnchorId,
    SourceVertexId,
)


class JunctionProjectionRole(str, Enum):
    GLOBAL_RELATION = "GLOBAL_RELATION"
    PER_PATCH_PROJECTION = "PER_PATCH_PROJECTION"


class EndpointRole(str, Enum):
    START = "START"
    END = "END"


@dataclass(frozen=True, slots=True)
class FrontSeedV1:
    seed_id: FrontSeedId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    chain_use_id: ChainUseId
    owner_patch_id: PatchId
    sector_ids: frozenset[OwnerSectorId]


@dataclass(frozen=True, slots=True)
class CornerSeedV1:
    seed_id: CornerSeedId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    source_relation_id: CornerRelationId
    owner_patch_id: PatchId
    owner_sector_id: OwnerSectorId
    ordered_incident_chain_use_ids: tuple[ChainUseId, ...]
    profile_selection_certificate_id: SelectionCertificateId


@dataclass(frozen=True, slots=True)
class JunctionSeedV1:
    seed_id: JunctionSeedId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    source_relation_id: JunctionRelationId
    owner_patch_id: PatchId
    incident_chain_use_ids: frozenset[ChainUseId]
    projection_role: JunctionProjectionRole
    shared_semantic_anchor_id: SharedSemanticAnchorId | None


@dataclass(frozen=True, slots=True)
class CapSeedV1:
    seed_id: CapSeedId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    source_vertex_id: SourceVertexId
    chain_use_id: ChainUseId
    owner_patch_id: PatchId
    endpoint_role: EndpointRole


@dataclass(frozen=True, slots=True)
class EndpointClaimSeedV1:
    seed_id: EndpointClaimSeedId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    source_vertex_id: SourceVertexId
    chain_use_id: ChainUseId
    owner_patch_id: PatchId
    endpoint_role: EndpointRole


SeedV1 = FrontSeedV1 | CornerSeedV1 | JunctionSeedV1 | CapSeedV1 | EndpointClaimSeedV1

