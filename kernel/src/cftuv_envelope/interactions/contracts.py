"""Immutable EC2.5 exact interaction-stage records."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..contracts.plan import PlanKeyV1
from ..ids import SourceRevision
from ..numeric import LocalLengthV1
from ..reference.contracts import (
    RawCoverageEdgeV1,
    RawCoverageLoopV1,
    RawCoverageRegionV1,
    RawCoverageVertexV1,
    ReachabilityCertificateV1,
    ReferenceEnvelopeInstanceV1,
)
from ..reference.planar_types import (
    BoundedSupportSegment,
    ExactPlanarPoint,
    ExactPlanarVector,
    ExactScalar,
    PlanarRegion,
)
from .provenance import InteractionProvenanceV1


RESOLVED_COVERAGE_RESULT_SCHEMA_V1 = (
    "cftuv.envelope.resolved_coverage_result.v1"
)
INTRAPATCH_POLICY_B_V1 = "INTRAPATCH_POLICY_B_V1"
SELF_CONTACT_POLICY_B_V1 = "SELF_CONTACT_POLICY_B_V1"


class InteractionOutcome(str, Enum):
    EXACT = "EXACT"
    INTERACTION_INPUT_CONTRACT_INVALID = "INTERACTION_INPUT_CONTRACT_INVALID"
    INTERACTION_JUNCTION_ARRIVAL_UNPROVEN = (
        "INTERACTION_JUNCTION_ARRIVAL_UNPROVEN"
    )
    INTERACTION_POLICY_B_PARTITION_UNPROVEN = (
        "INTERACTION_POLICY_B_PARTITION_UNPROVEN"
    )
    SELF_CONTACT_READING_IDENTITY_UNPROVEN = (
        "SELF_CONTACT_READING_IDENTITY_UNPROVEN"
    )
    MULTIWAY_INTERACTION_POLICY_UNPROVEN = (
        "MULTIWAY_INTERACTION_POLICY_UNPROVEN"
    )
    INTERACTION_COINCIDENT_ARRIVAL_LAWS_UNPROVEN = (
        "INTERACTION_COINCIDENT_ARRIVAL_LAWS_UNPROVEN"
    )
    INTERACTION_MISSING_FRONT_READING = "INTERACTION_MISSING_FRONT_READING"
    INTERACTION_EXACT_PREDICATE_UNDECIDABLE = (
        "INTERACTION_EXACT_PREDICATE_UNDECIDABLE"
    )


class InteractionDiagnosticSeverity(str, Enum):
    INFO = "INFO"
    UNSUPPORTED = "UNSUPPORTED"


class ArrivalModelKind(str, Enum):
    STRIP = "StripArrivalModelV1"
    ANGULAR_PROFILE = "AngularProfileArrivalModelV1"
    CAP = "CapArrivalModelV1"
    JUNCTION = "JunctionArrivalModelV1"
    UNSUPPORTED_JUNCTION = "UnsupportedJunctionArrivalModelV1"


class InteractionCandidateKind(str, Enum):
    DISTINCT_COMPONENTS = "DISTINCT_COMPONENTS"
    EXPLICIT_SELF_CONTACT = "EXPLICIT_SELF_CONTACT"


class EqualityLocusOwner(str, Enum):
    NONE = "NONE"


class InteractionCoverageEffect(str, Enum):
    NONE_BEFORE_MUTUAL_ARRIVAL = "NONE_BEFORE_MUTUAL_ARRIVAL"
    EXACT_POLICY_B_CLIP = "EXACT_POLICY_B_CLIP"
    EXACT_SELF_CONTACT_POLICY_B_CLIP = "EXACT_SELF_CONTACT_POLICY_B_CLIP"


class FreezeState(str, Enum):
    UNCHANGED = "UNCHANGED"
    FROZEN_AT_MUTUAL_EQUALITY_LOCUS = "FROZEN_AT_MUTUAL_EQUALITY_LOCUS"
    BOUNDARY_FROZEN_PRESERVED = "BOUNDARY_FROZEN_PRESERVED"


@dataclass(frozen=True, slots=True)
class InteractionDiagnosticV1:
    outcome: InteractionOutcome
    severity: InteractionDiagnosticSeverity
    message: str
    interaction_component_ids: frozenset[str] = frozenset()
    arrival_model_ids: frozenset[str] = frozenset()


@dataclass(frozen=True, slots=True)
class InteractionComponentV1:
    interaction_component_id: str
    decal_request_id: str
    patch_domain_id: str
    envelope_spec_ids: frozenset[str]
    envelope_instance_ids: frozenset[str]
    front_component_ids: frozenset[str]
    relation_ids: frozenset[str]
    source_lineage_ids: frozenset[str]


@dataclass(frozen=True, slots=True)
class ExactFrontArrivalLawV1:
    law_id: str
    support_id: str
    normal: ExactPlanarVector
    source_constant: ExactScalar
    normal_speed: ExactScalar


@dataclass(frozen=True, slots=True)
class StripArrivalModelV1:
    arrival_model_id: str
    model_kind: ArrivalModelKind
    interaction_component_id: str
    decal_request_id: str
    patch_domain_id: str
    envelope_spec_id: str
    envelope_instance_id: str
    front_component_id: str
    front_reading_id: str
    source_lineage_ids: frozenset[str]
    arrival_law: ExactFrontArrivalLawV1
    active_segments: tuple[BoundedSupportSegment, ...]
    active_regions: tuple[PlanarRegion, ...]
    effective_alpha: ExactScalar
    reachability: ReachabilityCertificateV1


@dataclass(frozen=True, slots=True)
class AngularProfileArrivalModelV1:
    arrival_model_id: str
    model_kind: ArrivalModelKind
    interaction_component_id: str
    decal_request_id: str
    patch_domain_id: str
    envelope_spec_id: str
    envelope_instance_id: str
    front_component_ids: frozenset[str]
    front_reading_id: str
    source_relation_id: str
    profile_support_ordinal: int
    source_lineage_ids: frozenset[str]
    arrival_law: ExactFrontArrivalLawV1
    component_profile_arrival_laws: tuple[ExactFrontArrivalLawV1, ...]
    active_segments: tuple[BoundedSupportSegment, ...]
    active_regions: tuple[PlanarRegion, ...]
    effective_alpha: ExactScalar
    reachability: ReachabilityCertificateV1
    exposed_on_component_boundary: bool


@dataclass(frozen=True, slots=True)
class CapArrivalModelV1:
    arrival_model_id: str
    model_kind: ArrivalModelKind
    interaction_component_id: str
    decal_request_id: str
    patch_domain_id: str
    envelope_spec_id: str
    envelope_instance_id: str
    incident_strip_spec_id: str
    front_component_id: str
    front_reading_id: str
    source_lineage_ids: frozenset[str]
    arrival_law: ExactFrontArrivalLawV1
    active_segments: tuple[BoundedSupportSegment, ...]
    active_regions: tuple[PlanarRegion, ...]
    effective_alpha: ExactScalar
    reachability: ReachabilityCertificateV1
    inherits_incident_strip_reading: bool


@dataclass(frozen=True, slots=True)
class UnsupportedJunctionArrivalModelV1:
    arrival_model_id: str
    model_kind: ArrivalModelKind
    interaction_component_id: str
    decal_request_id: str
    patch_domain_id: str
    envelope_spec_id: str
    envelope_instance_id: str | None
    source_relation_id: str
    outcome: InteractionOutcome
    message: str


ArrivalModelV1 = (
    StripArrivalModelV1
    | AngularProfileArrivalModelV1
    | CapArrivalModelV1
    | UnsupportedJunctionArrivalModelV1
)


@dataclass(frozen=True, slots=True)
class InteractionCandidateV1:
    candidate_id: str
    left_component_id: str
    right_component_id: str
    candidate_kind: InteractionCandidateKind
    decal_request_id: str
    patch_domain_id: str
    source_provenance: InteractionProvenanceV1


@dataclass(frozen=True, slots=True)
class FrontArrivalReadingV1:
    front_reading_id: str
    arrival_model_id: str
    interaction_component_id: str
    envelope_spec_id: str
    envelope_instance_id: str
    arrival_law: ExactFrontArrivalLawV1
    effective_alpha: ExactScalar
    active_segment_ids: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class ActiveDomainCertificateV1:
    certificate_id: str
    arrival_model_id: str
    active_region_ids: tuple[str, ...]
    active_segment_ids: tuple[str, ...]
    locus_reachable: bool


@dataclass(frozen=True, slots=True)
class MutualArrivalCertificateV1:
    certificate_id: str
    candidate_id: str
    left_front_reading: FrontArrivalReadingV1
    right_front_reading: FrontArrivalReadingV1
    exact_alpha: ExactScalar
    arrival_laws: tuple[ExactFrontArrivalLawV1, ExactFrontArrivalLawV1]
    active_domain_certificates: tuple[
        ActiveDomainCertificateV1, ActiveDomainCertificateV1
    ]
    reachability_certificates: tuple[
        ReachabilityCertificateV1, ReachabilityCertificateV1
    ]
    same_alpha_batch_identity: str


@dataclass(frozen=True, slots=True)
class EqualityLocusSegmentV1:
    segment_id: str
    start: ExactPlanarPoint
    end: ExactPlanarPoint
    left_front_reading_id: str
    right_front_reading_id: str
    construction_certificate_id: str


@dataclass(frozen=True, slots=True)
class EqualityLocusV1:
    locus_id: str
    ordered_exact_segments: tuple[EqualityLocusSegmentV1, ...]
    event_anchor_points: tuple[ExactPlanarPoint, ...]
    participant_front_reading_ids: tuple[str, str]
    construction_certificate_ids: tuple[str, ...]
    owner: EqualityLocusOwner


@dataclass(frozen=True, slots=True)
class ResolvedContributionV1:
    original_envelope_instance: ReferenceEnvelopeInstanceV1
    interaction_component_id: str
    retained_front_reading_ids: frozenset[str]
    retained_exact_regions: tuple[PlanarRegion, ...]
    removed_exact_regions: tuple[PlanarRegion, ...]
    interaction_ids: tuple[str, ...]
    freeze_state: FreezeState


@dataclass(frozen=True, slots=True)
class InteractionApplicationV1:
    interaction_id: str
    policy_id: str
    candidate_id: str
    mutual_arrival_certificate_id: str
    equality_locus_id: str
    participant_component_ids: tuple[str, str]
    coverage_effect: InteractionCoverageEffect
    creates_new_matter: bool
    exact_alpha: ExactScalar
    same_alpha_batch_identity: str


@dataclass(frozen=True, slots=True)
class ResolvedCoverageResultV1:
    schema_version: str
    source_revision: SourceRevision
    plan_key: PlanKeyV1
    requested_alpha: LocalLengthV1
    raw_coverage_digest: str
    resolved_contributions: tuple[ResolvedContributionV1, ...]
    vertices: frozenset[RawCoverageVertexV1]
    edges: frozenset[RawCoverageEdgeV1]
    loops: frozenset[RawCoverageLoopV1]
    regions: frozenset[RawCoverageRegionV1]
    exact_area_expression: str
    interaction_applications: tuple[InteractionApplicationV1, ...]
    mutual_arrival_certificates: tuple[MutualArrivalCertificateV1, ...]
    equality_loci: tuple[EqualityLocusV1, ...]
    diagnostics: tuple[InteractionDiagnosticV1, ...]
    semantic_digest: str


@dataclass(frozen=True, slots=True)
class InteractionResolutionResultV1:
    outcome: InteractionOutcome
    resolved_coverage: ResolvedCoverageResultV1 | None
    interaction_components: tuple[InteractionComponentV1, ...]
    arrival_models: tuple[ArrivalModelV1, ...]
    candidates: tuple[InteractionCandidateV1, ...]
    diagnostics: tuple[InteractionDiagnosticV1, ...]
