"""CompiledPatchEvaluationPlanV1: patch-level declarative aggregate."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..ids import (
    CoverageId,
    ChainUseId,
    DecalRequestId,
    EvaluationPlanId,
    FrontComponentId,
    FrontReadingId,
    FrontSeedId,
    LineageId,
    OwnerSectorId,
    PatchDomainId,
    PatchId,
    SemanticArrangementId,
    SharedSemanticAnchorId,
    SourceRevision,
)
from ..numeric import MetricLengthV1
from ..outcomes import NamedOutcome
from .coverage import InteractionDeclarationV1, RawCoverageRef, ResolvedCoverageRef
from .envelopes import (
    AngularProfileSelectionCertificateV1,
    EnvelopeInstanceV1,
    EnvelopeSpec,
)
from .events import (
    EventLedgerContractV1,
    EventPredicateDeclarationV1,
    EventTransitionDeclarationV1,
    InitialFrontSpec,
)
from .ownership import OwnershipPartitionV1
from .seeds import SeedV1
from .tessellation import TessellationPlanV1


COMPILED_PLAN_SCHEMA_V1 = "cftuv.envelope.compiled_patch_evaluation_plan.v1"


class ActiveIntervalModel(str, Enum):
    CONTINUOUS_INTERVAL_SET = "CONTINUOUS_INTERVAL_SET"


class BranchCountPolicy(str, Enum):
    NON_INCREASING = "NON_INCREASING"


class FrontLifecycle(str, Enum):
    COMPILE_TRACKED = "COMPILE_TRACKED"


class CapacityReason(str, Enum):
    NONE = "NONE"
    BARRIER_SPLIT_REQUIRED = "BARRIER_SPLIT_REQUIRED"


class FrontTerminalOutcome(str, Enum):
    NONE = "NONE"
    BOUNDARY_CAPACITY_REACHED = "BOUNDARY_CAPACITY_REACHED"
    FRONT_EXHAUSTED_OR_CAPACITY_REACHED = "FRONT_EXHAUSTED_OR_CAPACITY_REACHED"


class StationFlowAuthority(str, Enum):
    SEMANTIC_CHAIN_USE_ORIENTATION = "SEMANTIC_CHAIN_USE_ORIENTATION"


class StationAxis(str, Enum):
    S = "s"
    R = "r"


class StorageReversalEffect(str, Enum):
    NONE = "NONE"


@dataclass(frozen=True, slots=True)
class PlanKeyV1:
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId


@dataclass(frozen=True, slots=True)
class FrontComponentV1:
    front_component_id: FrontComponentId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    front_seed_id: FrontSeedId
    chain_use_id: ChainUseId
    owner_patch_id: PatchId
    sector_id: OwnerSectorId
    active_interval_model: ActiveIntervalModel
    initial_branch_count: int
    branch_count_policy: BranchCountPolicy
    lifecycle: FrontLifecycle
    front_reading_ids: frozenset[FrontReadingId]
    requested_alpha: MetricLengthV1
    effective_alpha: MetricLengthV1
    capacity_reason: CapacityReason
    terminal_outcome: FrontTerminalOutcome


@dataclass(frozen=True, slots=True)
class StationFlowV1:
    authority: StationFlowAuthority
    longitudinal: StationAxis
    transverse: StationAxis
    storage_reversal_effect: StorageReversalEffect


@dataclass(frozen=True, slots=True)
class GeometryBatchProvenanceV1:
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    resolved_coverage_id: CoverageId
    source_lineage_ids: frozenset[LineageId]
    shared_semantic_anchor_id: SharedSemanticAnchorId | None


@dataclass(frozen=True, slots=True)
class CompiledPatchEvaluationPlanV1:
    schema_version: str
    evaluation_plan_id: EvaluationPlanId
    source_revision: SourceRevision
    plan_key: PlanKeyV1
    owner_patch_id: PatchId
    seeds: frozenset[SeedV1]
    front_components: frozenset[FrontComponentV1]
    angular_profile_selection_certificates: frozenset[
        AngularProfileSelectionCertificateV1
    ]
    envelope_specs: frozenset[EnvelopeSpec]
    envelope_instances: frozenset[EnvelopeInstanceV1]
    initial_front_spec: InitialFrontSpec
    event_predicates: frozenset[EventPredicateDeclarationV1]
    event_transitions: frozenset[EventTransitionDeclarationV1]
    event_ledger: EventLedgerContractV1
    raw_coverage: RawCoverageRef
    resolved_coverage: ResolvedCoverageRef
    interactions: frozenset[InteractionDeclarationV1]
    ownership: OwnershipPartitionV1
    station_flow: StationFlowV1
    tessellation_plan: TessellationPlanV1
    geometry_batch_provenance: GeometryBatchProvenanceV1
    named_outcomes: frozenset[NamedOutcome]


CompiledPatchEvaluationPlan = CompiledPatchEvaluationPlanV1
