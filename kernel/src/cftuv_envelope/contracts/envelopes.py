"""Typed EnvelopeSpec union и alpha-bound EnvelopeInstance records."""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal
from enum import Enum

from ..ids import (
    AngleCertificateId,
    CapSeedId,
    ChainUseId,
    CornerRelationId,
    CornerSeedId,
    DecalRequestId,
    EnvelopeInstanceId,
    EnvelopeSpecId,
    FrontComponentId,
    FrontSeedId,
    HiddenSupportId,
    JunctionRelationId,
    JunctionSeedId,
    LineageId,
    OwnerSectorId,
    PatchDomainId,
    PerPatchProjectionId,
    RoutePairingId,
    SelectionCertificateId,
    SharedSemanticAnchorId,
    SourceVertexId,
    TerminalRelationId,
)
from ..numeric import CertifiedDecimalIntervalV1, ExactRatioV1
from .request import (
    AngularProfileFamilyId,
    AngularProfileSelectionPolicyId,
    MaxSubturnValueId,
)
from .seeds import EndpointRole


class EnvelopeSpecVariant(str, Enum):
    STRIP = "StripEnvelopeSpec"
    ANGULAR = "AngularEnvelopeSpec"
    JUNCTION = "JunctionEnvelopeSpec"
    CAP = "CapEnvelopeSpec"


class StripSupportLawId(str, Enum):
    PLANAR_LINEAR_NORMAL_OFFSET_V1 = "PLANAR_LINEAR_NORMAL_OFFSET_V1"


class StationModelId(str, Enum):
    SEMANTIC_CHAIN_USE_S = "SEMANTIC_CHAIN_USE_S"
    CONSTANT_PHYSICAL_ENDPOINT_S = "CONSTANT_PHYSICAL_ENDPOINT_S"
    PLAN_STATION_FLOW = "PLAN_STATION_FLOW"


class TransverseModelId(str, Enum):
    SIGNED_OWNER_INTERIOR_R = "SIGNED_OWNER_INTERIOR_R"


class TerminalInterfacePolicy(str, Enum):
    REFERENCED_ENVELOPE_SPECS = "REFERENCED_ENVELOPE_SPECS"
    DOMAIN_OR_REQUEST_TERMINAL = "DOMAIN_OR_REQUEST_TERMINAL"


class AngularSubdivisionPolicy(str, Enum):
    EQUAL_REFLEX_EXCESS = "EQUAL_REFLEX_EXCESS"


class HiddenSupportDirectionLaw(str, Enum):
    ORIENTED_OWNER_SECTOR_ORDINAL_SUBTURN = (
        "ROTATE_INCOMING_SUPPORT_TOWARD_OUTGOING_INSIDE_ORIENTED_OWNER_SECTOR_BY_ORDINAL_SUBTURN"
    )


class CertifiedBoundHiddenSupportDirectionLawV1(str, Enum):
    CERTIFIED_RATIONAL_BINDING_IN_ORDINAL_SUBTURN_V1 = (
        "CERTIFIED_RATIONAL_BINDING_IN_ORDINAL_SUBTURN_V1"
    )


class DirectionBindingReasonV1(str, Enum):
    SOURCE_DIRECTION_IRRATIONAL = "SOURCE_DIRECTION_IRRATIONAL"
    EVALUATION_GEOMETRY_UNBINDS_SOURCE_RATIONAL = (
        "EVALUATION_GEOMETRY_UNBINDS_SOURCE_RATIONAL"
    )


class HiddenSupportScope(str, Enum):
    ANGULAR_ENVELOPE_SPEC_LOCAL = "ANGULAR_ENVELOPE_SPEC_LOCAL"


class AngularExposurePolicy(str, Enum):
    EXPOSED_ONLY_WHEN_ON_RESOLVED_COVERAGE_SILHOUETTE = (
        "EXPOSED_ONLY_WHEN_ON_RESOLVED_COVERAGE_SILHOUETTE"
    )
    MAY_BECOME_INTERNAL_AFTER_EXACT_UNION = "MAY_BECOME_INTERNAL_AFTER_EXACT_UNION"


class JunctionSupportLawId(str, Enum):
    DECLARED_JUNCTION_RELATION_SUPPORTS_V1 = "DECLARED_JUNCTION_RELATION_SUPPORTS_V1"


class MixedAlphaPolicy(str, Enum):
    INCIDENT_EFFECTIVE_ALPHA_VECTOR = "INCIDENT_EFFECTIVE_ALPHA_VECTOR"


class CapClosureLawId(str, Enum):
    PHYSICAL_TERMINAL_LINEAR_CLOSURE_V1 = "PHYSICAL_TERMINAL_LINEAR_CLOSURE_V1"


class ExactTwoPiHandling(str, Enum):
    TERMINAL_OR_JUNCTION_NEVER_ANGULAR_PROFILE = (
        "TERMINAL_OR_JUNCTION_NEVER_ANGULAR_PROFILE"
    )


class AngularRegressionFixtureId(str, Enum):
    K0 = "LINEAR_REFLEX_K0_EQUAL_FIXTURE_V1"
    K1 = "LINEAR_REFLEX_K1_EQUAL_FIXTURE_V1"


class SelectionLaw(str, Enum):
    MIN_K_FOR_MAX_SUBTURN = "K_EQUALS_MAX_ZERO_CEIL_DELTA_OVER_DELTA_MAX_MINUS_ONE"
    HUBER_EMANATED_DENSITY_FLOOR_V1 = "HUBER_EMANATED_DENSITY_FLOOR_V1"


class MinimalityLowerBound(str, Enum):
    K_ZERO_OR_STRICT_LOWER = "K_EQ_ZERO_OR_K_TIMES_DELTA_MAX_LT_DELTA"
    HUBER_DENSITY_BUCKET_OPEN_LOWER = "HUBER_DENSITY_BUCKET_OPEN_LOWER"


class AdmissibilityUpperBound(str, Enum):
    CLOSED_UPPER = "DELTA_LEQ_K_PLUS_ONE_TIMES_DELTA_MAX"
    HUBER_DENSITY_BUCKET_CLOSED_UPPER = "HUBER_DENSITY_BUCKET_CLOSED_UPPER"


class SelectionCertificateAuthority(str, Enum):
    EXACT_OR_CERTIFIED_ANGLE_COMPARISON = "EXACT_OR_CERTIFIED_ANGLE_COMPARISON"


class IntervalBoundKind(str, Enum):
    OPEN = "OPEN"
    CLOSED = "CLOSED"


class BoundaryResolutionState(str, Enum):
    REQUIRED_BEFORE_PATCH_UNION = "REQUIRED_BEFORE_PATCH_UNION"
    BOUNDARY_RESOLVED = "BOUNDARY_RESOLVED"


class EffectiveAlphaBindingKind(str, Enum):
    INCIDENT_FRONT_COMPONENT_VECTOR = "INCIDENT_FRONT_COMPONENT_VECTOR"
    INCIDENT_STRIP_EFFECTIVE_ALPHA = "INCIDENT_STRIP_EFFECTIVE_ALPHA"


@dataclass(frozen=True, slots=True)
class SelectionIntervalCertificateV1:
    lower_bound_kind: IntervalBoundKind
    lower_bound_integer: int
    upper_bound_kind: IntervalBoundKind
    upper_bound_integer: int


@dataclass(frozen=True, slots=True)
class HuberDensitySelectionIntervalCertificateV1:
    """Именованная ячейка `(C-1)/q < u <= C/q` политики Density A."""

    q: int
    bucket_c: int
    lower_bound_kind: IntervalBoundKind
    lower_bound_numerator: int
    upper_bound_kind: IntervalBoundKind
    upper_bound_numerator: int


@dataclass(frozen=True, slots=True)
class AngularProfileSelectionCertificateV1:
    certificate_id: SelectionCertificateId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    corner_relation_id: CornerRelationId
    owner_sector_id: OwnerSectorId
    reflex_angle_certificate_id: AngleCertificateId
    profile_family_id: AngularProfileFamilyId
    selection_policy_id: AngularProfileSelectionPolicyId
    max_subturn_value_id: MaxSubturnValueId
    resolved_hidden_edge_count: int
    resolved_subturn_count: int
    local_profile_support_count: int
    local_profile_segment_count: int
    selection_law: SelectionLaw
    minimality_lower_bound: MinimalityLowerBound
    admissibility_upper_bound: AdmissibilityUpperBound
    selection_interval_certificate: (
        SelectionIntervalCertificateV1
        | HuberDensitySelectionIntervalCertificateV1
    )
    certificate_authority: SelectionCertificateAuthority
    regression_fixture_id: AngularRegressionFixtureId | None


@dataclass(frozen=True, slots=True)
class HiddenSupportSpecV1:
    hidden_support_id: HiddenSupportId
    ordinal: int
    turn_fraction: ExactRatioV1
    direction_law: HiddenSupportDirectionLaw
    zero_length_at_alpha_zero: bool
    scope: HiddenSupportScope
    source_relation_id: CornerRelationId
    owner_sector_id: OwnerSectorId
    selection_certificate_id: SelectionCertificateId


@dataclass(frozen=True, slots=True)
class DirectionBindingCertificateV1:
    bound_primitive_integer_vector: tuple[int, int]
    ideal_window_lower_slope_envelope: CertifiedDecimalIntervalV1
    ideal_window_upper_slope_envelope: CertifiedDecimalIntervalV1
    certified_window_width_lower_bound: Decimal
    proven_predicates: frozenset[str]


@dataclass(frozen=True, slots=True)
class EvaluationGeometryDirectionBindingCertificateV1:
    """Сертификат направления, доказанный на записанной evaluation-геометрии."""

    bound_primitive_integer_vector: tuple[int, int]
    ideal_window_lower_slope_envelope: CertifiedDecimalIntervalV1
    ideal_window_upper_slope_envelope: CertifiedDecimalIntervalV1
    certified_window_width_lower_bound: Decimal
    proven_predicates: frozenset[str]
    binding_reason: DirectionBindingReasonV1


@dataclass(frozen=True, slots=True)
class CertifiedBoundHiddenSupportSpecV1:
    hidden_support_id: HiddenSupportId
    ordinal: int
    turn_fraction: ExactRatioV1
    direction_law: CertifiedBoundHiddenSupportDirectionLawV1
    zero_length_at_alpha_zero: bool
    scope: HiddenSupportScope
    source_relation_id: CornerRelationId
    owner_sector_id: OwnerSectorId
    selection_certificate_id: SelectionCertificateId
    direction_binding: (
        DirectionBindingCertificateV1
        | EvaluationGeometryDirectionBindingCertificateV1
    )


@dataclass(frozen=True, slots=True)
class StripEnvelopeSpec:
    envelope_spec_id: EnvelopeSpecId
    source_seed_id: FrontSeedId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    source_lineage_ids: frozenset[LineageId]
    front_component_ids: frozenset[FrontComponentId]
    support_law_id: StripSupportLawId
    station_model_id: StationModelId
    transverse_model_id: TransverseModelId
    terminal_interface_spec_ids: frozenset[EnvelopeSpecId]
    terminal_interface_policy: TerminalInterfacePolicy


@dataclass(frozen=True, slots=True)
class AngularEnvelopeSpec:
    envelope_spec_id: EnvelopeSpecId
    source_seed_id: CornerSeedId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    source_lineage_ids: frozenset[LineageId]
    source_relation_id: CornerRelationId
    owner_sector_id: OwnerSectorId
    angle_certificate_id: AngleCertificateId
    selection_certificate_id: SelectionCertificateId
    profile_family_id: AngularProfileFamilyId
    resolved_hidden_edge_count: int
    subdivision_policy: AngularSubdivisionPolicy
    hidden_supports: frozenset[
        HiddenSupportSpecV1 | CertifiedBoundHiddenSupportSpecV1
    ]
    incident_front_component_ids: tuple[FrontComponentId, ...]
    all_support_normal_speed: int
    exposure_policy: AngularExposurePolicy
    mixed_alpha_policy: MixedAlphaPolicy


@dataclass(frozen=True, slots=True)
class JunctionEnvelopeSpec:
    envelope_spec_id: EnvelopeSpecId
    source_seed_id: JunctionSeedId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    source_lineage_ids: frozenset[LineageId]
    source_relation_id: JunctionRelationId
    incident_front_component_ids: frozenset[FrontComponentId]
    support_law_id: JunctionSupportLawId
    route_pairing_ids: frozenset[RoutePairingId]
    shared_semantic_anchor_id: SharedSemanticAnchorId
    per_patch_projection_id: PerPatchProjectionId | None
    mixed_alpha_policy: MixedAlphaPolicy


@dataclass(frozen=True, slots=True)
class CapEnvelopeSpec:
    envelope_spec_id: EnvelopeSpecId
    source_seed_id: CapSeedId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    source_lineage_ids: frozenset[LineageId]
    physical_terminal_source_vertex_id: SourceVertexId
    terminal_relation_id: TerminalRelationId | None
    incident_chain_use_id: ChainUseId
    incident_strip_spec_id: EnvelopeSpecId
    closure_law_id: CapClosureLawId
    endpoint_role: EndpointRole
    station_law: StationModelId
    effective_alpha_binding: EffectiveAlphaBindingKind
    exact_two_pi_handling: ExactTwoPiHandling


EnvelopeSpec = StripEnvelopeSpec | AngularEnvelopeSpec | JunctionEnvelopeSpec | CapEnvelopeSpec


@dataclass(frozen=True, slots=True)
class EffectiveAlphaBindingV1:
    kind: EffectiveAlphaBindingKind
    front_component_ids: frozenset[FrontComponentId]


@dataclass(frozen=True, slots=True)
class EnvelopeInstanceV1:
    instance_id: EnvelopeInstanceId
    spec_id: EnvelopeSpecId
    spec_variant: EnvelopeSpecVariant
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    effective_alpha_binding: EffectiveAlphaBindingV1
    boundary_resolution_state: BoundaryResolutionState
