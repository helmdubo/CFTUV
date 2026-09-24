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
    HiddenSupportId,
    LineageId,
    LawId,
    OwnerSectorId,
    PatchDomainId,
    PatchId,
    PhysicalEdgeId,
    PhysicalChainId,
    ReferenceMetricId,
    SemanticArrangementId,
    SelfContactPairDeclarationId,
    SharedSemanticAnchorId,
    SourceSupportId,
    SourceRevision,
    SourceVertexId,
)
from .metric import ExactPoint2V1, ExactRationalV1, ExactVector2V1
from ..numeric import MetricLengthV1
from ..outcomes import NamedOutcome
from .coverage import InteractionDeclarationV1, RawCoverageRef, ResolvedCoverageRef
from .envelopes import (
    AngularProfileSelectionCertificateV1,
    CanonicalAngleRestorationCertificateV1,
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
EVALUATION_GEOMETRY_BINDING_SCHEMA_V1 = (
    "cftuv.envelope.evaluation_geometry_binding.v1"
)
CHAIN_STRAIGHT_EVALUATION_GEOMETRY_BINDING_SCHEMA_V2 = (
    "cftuv.envelope.chain_straight_evaluation_geometry_binding.v2"
)


class EvaluationGeometryBindingLawV1(str, Enum):
    EVALUATION_GEOMETRY_CHART_LATTICE_BOUND_V1 = (
        "EVALUATION_GEOMETRY_CHART_LATTICE_BOUND_V1"
    )


class ChainStraightEvaluationGeometryBindingLawV2(str, Enum):
    EVALUATION_GEOMETRY_CHART_LATTICE_BOUND_CHAIN_STRAIGHT_V2 = (
        "EVALUATION_GEOMETRY_CHART_LATTICE_BOUND_CHAIN_STRAIGHT_V2"
    )


class ChainStraightVertexAuthorityV2(str, Enum):
    BASE_BOUND_ENDPOINT_V1 = "BASE_BOUND_ENDPOINT_V1"
    BASE_BOUND_NON_CHAIN_V1 = "BASE_BOUND_NON_CHAIN_V1"
    CHAIN_STRAIGHT_INTERNAL_REFINED_V2 = (
        "CHAIN_STRAIGHT_INTERNAL_REFINED_V2"
    )


class ChainStraightAssignmentDispositionV2(str, Enum):
    UNCLAMPED_WITHIN_HALF_STEP = "UNCLAMPED_WITHIN_HALF_STEP"
    CLAMPED_WITHIN_HALF_STEP = "CLAMPED_WITHIN_HALF_STEP"
    CLAMPED_CONSTRAINT_EXCESS_ALLOWED = (
        "CLAMPED_CONSTRAINT_EXCESS_ALLOWED"
    )


@dataclass(frozen=True, slots=True)
class EvaluationGeometrySourceVertexV1:
    source_vertex_id: SourceVertexId
    domain_coordinate: ExactPoint2V1


@dataclass(frozen=True, slots=True)
class EvaluationGeometryBindingV1:
    """Одна chart-lattice геометрия, которую обязаны читать оба evaluator'а."""

    schema_version: str
    source_revision: SourceRevision
    patch_domain_id: PatchDomainId
    reference_metric_id: ReferenceMetricId
    binding_law: EvaluationGeometryBindingLawV1
    lattice_scale: int
    source_vertex_coordinates: frozenset[EvaluationGeometrySourceVertexV1]
    bound_hidden_support_ids: frozenset[HiddenSupportId]


@dataclass(frozen=True, slots=True)
class ChainStraightVertexAuthorityRecordV2:
    source_vertex_id: SourceVertexId
    authority: ChainStraightVertexAuthorityV2
    physical_chain_ids: frozenset[PhysicalChainId]
    source_domain_coordinate: ExactPoint2V1
    base_bound_node: tuple[int, int]
    base_bound_coordinate: ExactPoint2V1
    assigned_refined_node: tuple[int, int]
    assigned_domain_coordinate: ExactPoint2V1
    exact_offset_from_source: ExactVector2V1


@dataclass(frozen=True, slots=True)
class ChainStraightInternalAssignmentV2:
    physical_chain_id: PhysicalChainId
    ordinal: int
    source_vertex_id: SourceVertexId
    lower_k: int
    upper_k: int
    projection_k_gram: ExactRationalV1
    unconstrained_canonical_k: int
    selected_k: int
    clamped: bool
    disposition: ChainStraightAssignmentDispositionV2
    assigned_refined_node: tuple[int, int]
    exact_offset_from_source: ExactVector2V1
    exact_gram_displacement_squared: ExactRationalV1
    half_step_gram_squared_bound: ExactRationalV1
    exact_longitudinal_displacement_k: ExactRationalV1
    longitudinal_half_step_bound_k: ExactRationalV1


@dataclass(frozen=True, slots=True)
class ChainStraightPhysicalChainBindingV2:
    physical_chain_id: PhysicalChainId
    ordered_source_vertex_ids: tuple[SourceVertexId, ...]
    primitive_direction: tuple[int, int]
    base_start_node: tuple[int, int]
    base_end_node: tuple[int, int]
    base_endpoint_span_k: int
    refined_endpoint_span_k: int
    internal_assignments: tuple[ChainStraightInternalAssignmentV2, ...]


@dataclass(frozen=True, slots=True)
class ChainStraightCapacityDeficitV2:
    physical_chain_id: PhysicalChainId
    previous_refinement_power: int
    previous_endpoint_span_k: int
    capacity_required: int
    exact_deficit: int


@dataclass(frozen=True, slots=True)
class ChainStraightEvaluationGeometryBindingV2:
    """Общая S' chart-lattice geometry с точным straight-chain authority."""

    schema_version: str
    source_revision: SourceRevision
    patch_domain_id: PatchDomainId
    reference_metric_id: ReferenceMetricId
    binding_law: ChainStraightEvaluationGeometryBindingLawV2
    base_lattice_scale: int
    refinement_power: int
    lattice_scale: int
    source_vertex_coordinates: frozenset[EvaluationGeometrySourceVertexV1]
    vertex_authorities: frozenset[ChainStraightVertexAuthorityRecordV2]
    straight_chain_bindings: frozenset[
        ChainStraightPhysicalChainBindingV2
    ]
    previous_refinement_capacity_deficits: frozenset[
        ChainStraightCapacityDeficitV2
    ]
    bound_hidden_support_ids: frozenset[HiddenSupportId]


EvaluationGeometryBinding = (
    EvaluationGeometryBindingV1 | ChainStraightEvaluationGeometryBindingV2
)


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
class PhysicalSupportIntervalV1:
    physical_edge_id: PhysicalEdgeId
    start_vertex_id: SourceVertexId
    end_vertex_id: SourceVertexId


@dataclass(frozen=True, slots=True)
class FrontReadingDeclarationV1:
    front_reading_id: FrontReadingId
    front_component_id: FrontComponentId
    source_support_id: SourceSupportId
    physical_interval: PhysicalSupportIntervalV1
    chain_use_id: ChainUseId
    owner_sector_id: OwnerSectorId
    arrival_law_id: LawId


@dataclass(frozen=True, slots=True)
class SelfContactPairDeclarationV1:
    declaration_id: SelfContactPairDeclarationId
    left_front_component_id: FrontComponentId
    right_front_component_id: FrontComponentId
    left_front_reading_id: FrontReadingId
    right_front_reading_id: FrontReadingId
    source_lineage_ids: frozenset[LineageId]


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
    front_reading_declarations: frozenset[
        FrontReadingDeclarationV1
    ] = frozenset()
    self_contact_pair_declarations: frozenset[
        SelfContactPairDeclarationV1
    ] = frozenset()
    # Восстановления канонического авторского угла, применённые ДО селектора
    # плотности. Пустое множество означает «ни один угол плана не
    # восстанавливался», а не «поле не заполнили»: проверяющий обязан
    # доказывать счёт по сырому углу всюду, где записи нет.
    canonical_angle_restorations: frozenset[
        CanonicalAngleRestorationCertificateV1
    ] = frozenset()


CompiledPatchEvaluationPlan = CompiledPatchEvaluationPlanV1
