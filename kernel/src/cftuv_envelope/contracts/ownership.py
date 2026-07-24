"""Ownership declarations; partition solver не входит в EC1."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..ids import (
    CoverageId,
    EnvelopeInstanceId,
    EnvelopeSpecId,
    EqualityBoundaryId,
    LineageId,
    OwnershipClaimId,
)
from ..outcomes import NamedOutcome
from .envelopes import StationModelId


class OwnershipPartitionContractId(str, Enum):
    TOTAL_DISJOINT_RESOLVED_COVERAGE_V1 = "TOTAL_DISJOINT_RESOLVED_COVERAGE_V1"


class OwnershipClaimantKind(str, Enum):
    ENVELOPE_SPEC = "EnvelopeSpec"
    ENVELOPE_SPEC_GROUP = "EnvelopeSpecGroup"
    ENDPOINT_CLAIM_SEED = "EndpointClaimSeed"
    FRONT_READING = "FrontReading"


class SilhouetteEffect(str, Enum):
    NONE = "NONE"


class EqualityBoundaryOwner(str, Enum):
    NONE = "NONE"


class ClaimInteriorsObligation(str, Enum):
    PAIRWISE_DISJOINT = "PAIRWISE_DISJOINT"


class ClaimClosureObligation(str, Enum):
    EQUALS_RESOLVED_COVERAGE = "EQUALS_RESOLVED_COVERAGE"


@dataclass(frozen=True, slots=True)
class EqualityBoundaryRef:
    equality_boundary_id: EqualityBoundaryId
    owner: EqualityBoundaryOwner
    provenance: frozenset[LineageId]


@dataclass(frozen=True, slots=True)
class OwnershipClaimV1:
    ownership_claim_id: OwnershipClaimId
    coverage_id: CoverageId
    claimant_kind: OwnershipClaimantKind
    claimant_lineage_id: LineageId
    envelope_spec_ids: frozenset[EnvelopeSpecId]
    envelope_instance_ids: frozenset[EnvelopeInstanceId]
    selection_law_id: LineageId
    station_model_ref: StationModelId


@dataclass(frozen=True, slots=True)
class OwnershipPartitionV1:
    coverage_id: CoverageId
    partition_contract_id: OwnershipPartitionContractId
    total: bool
    disjoint: bool
    silhouette_effect: SilhouetteEffect
    equality_boundary_owner: EqualityBoundaryOwner
    named_unproven_outcome: NamedOutcome
    claims: frozenset[OwnershipClaimV1]
    equality_boundaries: frozenset[EqualityBoundaryRef]
    claim_interiors_obligation: ClaimInteriorsObligation
    claim_closure_obligation: ClaimClosureObligation

