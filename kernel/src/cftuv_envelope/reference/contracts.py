"""Immutable stage-specific EC2 reference evaluator result records."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..contracts.envelopes import (
    AngularProfileSelectionCertificateV1,
    EnvelopeSpec,
)
from ..contracts.analysis import AnalysisSnapshotV1
from ..contracts.events import InitialFrontSpec
from ..contracts.plan import (
    EvaluationGeometryBinding,
    FrontComponentV1,
    FrontReadingDeclarationV1,
    PlanKeyV1,
    SelfContactPairDeclarationV1,
)
from ..contracts.request import DecalRequestV1
from ..contracts.seeds import SeedV1
from ..ids import PatchId, SourceRevision
from ..numeric import LocalLengthV1
from .planar_types import (
    BoundedSupportSegment,
    ConstructionCertificate,
    ExactPlanarPoint,
    ExactScalar,
    PlanarRegion,
)
from .provenance import ReferenceProvenanceV1


REFERENCE_COMPILATION_SCHEMA_V1 = "cftuv.envelope.reference_compilation.v1"
RAW_COVERAGE_RESULT_SCHEMA_V1 = "cftuv.envelope.raw_coverage_result.v1"
RAW_COVERAGE_RESULT_SCHEMA_V2 = "cftuv.envelope.raw_coverage_result.v2"


class ReferenceOutcome(str, Enum):
    EXACT = "EXACT"
    REFERENCE_INPUT_CONTRACT_INVALID = "REFERENCE_INPUT_CONTRACT_INVALID"
    REFERENCE_MULTISECTOR_CHAIN_USE_UNSUPPORTED = (
        "REFERENCE_MULTISECTOR_CHAIN_USE_UNSUPPORTED"
    )
    REFERENCE_PLANAR_FRAME_CERTIFICATE_MISMATCH = (
        "REFERENCE_PLANAR_FRAME_CERTIFICATE_MISMATCH"
    )
    REFERENCE_ANGLE_SUPPORT_CERTIFICATE_MISMATCH = (
        "REFERENCE_ANGLE_SUPPORT_CERTIFICATE_MISMATCH"
    )
    REFERENCE_JUNCTION_ENVELOPE_LAW_UNPROVEN = (
        "REFERENCE_JUNCTION_ENVELOPE_LAW_UNPROVEN"
    )
    REFERENCE_ANGULAR_BOUNDARY_CONTACT_UNPROVEN = (
        "REFERENCE_ANGULAR_BOUNDARY_CONTACT_UNPROVEN"
    )
    REFERENCE_GEOMETRY_PAYLOAD_REQUIRED = "REFERENCE_GEOMETRY_PAYLOAD_REQUIRED"
    REFERENCE_PLANAR_REGIME_REQUIRED = "REFERENCE_PLANAR_REGIME_REQUIRED"
    REFERENCE_PLANAR_FRAME_REQUIRED = "REFERENCE_PLANAR_FRAME_REQUIRED"
    REFERENCE_PLANAR_FRAME_AMBIGUOUS = "REFERENCE_PLANAR_FRAME_AMBIGUOUS"
    REFERENCE_PLANAR_METRIC_REQUIRED = "REFERENCE_PLANAR_METRIC_REQUIRED"
    REFERENCE_PLANAR_METRIC_AMBIGUOUS = "REFERENCE_PLANAR_METRIC_AMBIGUOUS"
    REFERENCE_PLANAR_METRIC_CERTIFICATE_MISMATCH = (
        "REFERENCE_PLANAR_METRIC_CERTIFICATE_MISMATCH"
    )
    REFERENCE_EVALUATION_GEOMETRY_BINDING_INVALID = (
        "REFERENCE_EVALUATION_GEOMETRY_BINDING_INVALID"
    )
    REFINEMENT_BUDGET_EXHAUSTED = "REFINEMENT_BUDGET_EXHAUSTED"
    SOURCE_DECLARED_STRAIGHT_ENDPOINTS_COINCIDE = (
        "SOURCE_DECLARED_STRAIGHT_ENDPOINTS_COINCIDE"
    )
    SOURCE_DECLARED_STRAIGHT_CHAIN_IS_NOT_LINEAR = (
        "SOURCE_DECLARED_STRAIGHT_CHAIN_IS_NOT_LINEAR"
    )
    REFERENCE_PATCH_DOMAIN_SELECTION_REQUIRED = (
        "REFERENCE_PATCH_DOMAIN_SELECTION_REQUIRED"
    )
    REFERENCE_INVALID_ALPHA = "REFERENCE_INVALID_ALPHA"
    # Запрошенная полоса тоньше четырёх ячеек решётки: решётка её уничтожит.
    # Отказ именованный, а не тихо нарисованный мусор.
    ENVELOPE_BAND_BELOW_GRID_RESOLUTION = (
        "ENVELOPE_BAND_BELOW_GRID_RESOLUTION"
    )
    PHYSICAL_EDGE_PROVENANCE_INCOMPLETE = "PHYSICAL_EDGE_PROVENANCE_INCOMPLETE"
    PLANAR_CHAIN_SUPPORT_NOT_LINEAR = "PLANAR_CHAIN_SUPPORT_NOT_LINEAR"
    PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED = (
        "PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED"
    )
    ANGULAR_PROFILE_SELECTION_UNCERTAIN = "ANGULAR_PROFILE_SELECTION_UNCERTAIN"
    JUNCTION_ROUTE_PAIRING_REQUIRED = "JUNCTION_ROUTE_PAIRING_REQUIRED"
    BARRIER_SPLIT_REQUIRED = "BARRIER_SPLIT_REQUIRED"
    BARRIER_BYPASS_UNSUPPORTED = "BARRIER_BYPASS_UNSUPPORTED"
    SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN = (
        "SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN"
    )
    REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE = (
        "REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE"
    )
    DENSITY_RATIONAL_AUTHORITY_EXHAUSTED = (
        "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED"
    )
    DENSITY_WINDOW_CHART_UNREPRESENTABLE = (
        "DENSITY_WINDOW_CHART_UNREPRESENTABLE"
    )
    DENSITY_SEALED_FAN_INVALID = "DENSITY_SEALED_FAN_INVALID"
    REFERENCE_ARRANGEMENT_NON_MANIFOLD = "REFERENCE_ARRANGEMENT_NON_MANIFOLD"
    REFERENCE_ARRANGEMENT_ROTATION_SYSTEM_UNPROVEN = (
        "REFERENCE_ARRANGEMENT_ROTATION_SYSTEM_UNPROVEN"
    )
    REFERENCE_ARRANGEMENT_COLLINEAR_BRANCH_UNPROVEN = (
        "REFERENCE_ARRANGEMENT_COLLINEAR_BRANCH_UNPROVEN"
    )
    REFERENCE_TOUCHING_HOLE_TOPOLOGY_UNPROVEN = (
        "REFERENCE_TOUCHING_HOLE_TOPOLOGY_UNPROVEN"
    )
    RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED = (
        "RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED"
    )


class ReferenceDiagnosticSeverity(str, Enum):
    INFO = "INFO"
    CAPACITY = "CAPACITY"
    UNSUPPORTED = "UNSUPPORTED"


class BoundaryContactKind(str, Enum):
    ENDPOINT_SLIDE = "ENDPOINT_SLIDE"
    INTERIOR_SPLIT = "INTERIOR_SPLIT"
    CLIP_OR_SHRINK = "CLIP_OR_SHRINK"


class RawCoverageLoopKind(str, Enum):
    OUTER = "OUTER"
    HOLE = "HOLE"


@dataclass(frozen=True, slots=True)
class ReferenceEvaluationDiagnosticV1:
    outcome: ReferenceOutcome
    severity: ReferenceDiagnosticSeverity
    message: str
    envelope_spec_id: str | None = None
    front_component_id: str | None = None
    requested_alpha: LocalLengthV1 | None = None
    effective_alpha: ExactScalar | None = None
    construction: ConstructionCertificate | None = None


@dataclass(frozen=True, slots=True)
class EnvelopeSourceProvenanceV1:
    envelope_spec_id: str
    provenance: ReferenceProvenanceV1


@dataclass(frozen=True, slots=True)
class ReferenceEnvelopeCompilationV1:
    schema_version: str
    source_revision: SourceRevision
    analysis_snapshot: AnalysisSnapshotV1
    decal_request: DecalRequestV1
    plan_key: PlanKeyV1
    owner_patch_id: PatchId
    seeds: frozenset[SeedV1]
    front_components: frozenset[FrontComponentV1]
    profile_selection_certificates: frozenset[
        AngularProfileSelectionCertificateV1
    ]
    envelope_specs: frozenset[EnvelopeSpec]
    initial_front_spec: InitialFrontSpec
    source_provenance: frozenset[EnvelopeSourceProvenanceV1]
    front_reading_declarations: frozenset[
        FrontReadingDeclarationV1
    ] = frozenset()
    self_contact_pair_declarations: frozenset[
        SelfContactPairDeclarationV1
    ] = frozenset()
    evaluation_geometry_binding: EvaluationGeometryBinding | None = None
    diagnostics: tuple[ReferenceEvaluationDiagnosticV1, ...] = ()


@dataclass(frozen=True, slots=True)
class ReferenceCompileResultV1:
    outcome: ReferenceOutcome
    compilation: ReferenceEnvelopeCompilationV1 | None
    diagnostics: tuple[ReferenceEvaluationDiagnosticV1, ...]


@dataclass(frozen=True, slots=True)
class ReferenceEnvelopeInstanceV1:
    envelope_instance_id: str
    envelope_spec_id: str
    envelope_variant: str
    requested_alpha: LocalLengthV1
    effective_alpha: ExactScalar
    regions: tuple[PlanarRegion, ...]
    exposed_segments: tuple[BoundedSupportSegment, ...]
    provenance: ReferenceProvenanceV1


@dataclass(frozen=True, slots=True)
class ReachabilityCertificateV1:
    front_component_ids: frozenset[str]
    source_launch_reachable: bool
    initial_branch_count: int
    effective_branch_count: int
    boundary_event_keys: tuple[str, ...]
    bypass_used: bool


@dataclass(frozen=True, slots=True)
class BoundaryResolvedEnvelopeV1:
    envelope_instance: ReferenceEnvelopeInstanceV1
    requested_alpha: LocalLengthV1
    effective_alpha: ExactScalar
    reachability: ReachabilityCertificateV1
    capacity_outcome: ReferenceOutcome | None
    diagnostics: tuple[ReferenceEvaluationDiagnosticV1, ...]


@dataclass(frozen=True, slots=True)
class RawCoverageVertexV1:
    vertex_id: str
    point: ExactPlanarPoint
    construction_certificates: frozenset[ConstructionCertificate]
    provenance: ReferenceProvenanceV1


@dataclass(frozen=True, slots=True)
class RawCoverageEdgeV1:
    edge_id: str
    start_vertex_id: str
    end_vertex_id: str
    provenance: ReferenceProvenanceV1
    construction_certificates: frozenset[ConstructionCertificate]
    reachability: ReachabilityCertificateV1


@dataclass(frozen=True, slots=True)
class RawCoverageLoopV1:
    loop_id: str
    kind: RawCoverageLoopKind
    ordered_edge_ids: tuple[str, ...]
    signed_area: str


@dataclass(frozen=True, slots=True)
class RawCoverageRegionV1:
    region_id: str
    outer_loop_id: str
    hole_loop_ids: tuple[str, ...]
    contributor_envelope_spec_ids: frozenset[str]
    contributor_envelope_instance_ids: frozenset[str]
    provenance: ReferenceProvenanceV1


@dataclass(frozen=True, slots=True)
class BoundaryVertexOccurrenceV1:
    """Один топологический проход boundary через exact arrangement point."""

    boundary_occurrence_id: str
    arrangement_point_id: str
    loop_id: str
    incoming_boundary_edge_id: str
    outgoing_boundary_edge_id: str
    covered_sector_id: str
    construction_certificates: frozenset[ConstructionCertificate]
    provenance: ReferenceProvenanceV1


@dataclass(frozen=True, slots=True)
class PointContactRecordV1:
    """Lower-dimensional contact без area-connectivity."""

    point_contact_id: str
    arrangement_point_id: str
    participating_loop_ids: tuple[str, ...]
    participating_envelope_instance_ids: frozenset[str]
    participating_front_component_ids: frozenset[str]
    construction_certificates: frozenset[ConstructionCertificate]
    provenance: ReferenceProvenanceV1


@dataclass(frozen=True, slots=True)
class ComponentEffectiveAlphaV1:
    front_component_id: str
    requested_alpha: LocalLengthV1
    effective_alpha: ExactScalar
    capacity_outcome: ReferenceOutcome | None


@dataclass(frozen=True, slots=True)
class RawCoverageResultV1:
    schema_version: str
    source_revision: SourceRevision
    plan_key: PlanKeyV1
    requested_alpha: LocalLengthV1
    component_effective_alphas: frozenset[ComponentEffectiveAlphaV1]
    vertices: frozenset[RawCoverageVertexV1]
    edges: frozenset[RawCoverageEdgeV1]
    loops: frozenset[RawCoverageLoopV1]
    regions: frozenset[RawCoverageRegionV1]
    envelope_instances: frozenset[ReferenceEnvelopeInstanceV1]
    boundary_resolved_envelopes: frozenset[BoundaryResolvedEnvelopeV1]
    exact_area_expression: str
    semantic_digest: str
    diagnostics: tuple[ReferenceEvaluationDiagnosticV1, ...]


@dataclass(frozen=True, slots=True)
class RawCoverageResultV2(RawCoverageResultV1):
    """V2 adds explicit boundary occurrences without changing point identity."""

    boundary_vertex_occurrences: frozenset[BoundaryVertexOccurrenceV1]
    point_contacts: frozenset[PointContactRecordV1]


@dataclass(frozen=True, slots=True)
class ReferenceEvaluationResultV1:
    outcome: ReferenceOutcome
    raw_coverage: RawCoverageResultV1 | None
    diagnostics: tuple[ReferenceEvaluationDiagnosticV1, ...]
