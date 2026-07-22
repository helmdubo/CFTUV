"""AnalysisSnapshotV1: только host-observed facts и relations."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..ids import (
    AngleCertificateId,
    BoundaryConstraintId,
    BoundaryLoopId,
    ChainUseId,
    CornerRelationId,
    JunctionRelationId,
    LawId,
    LineageId,
    OwnerSectorId,
    PatchDomainId,
    PatchId,
    PhysicalEdgeId,
    PhysicalChainId,
    PlanarityCertificateId,
    RoutePairingId,
    SharedSemanticAnchorId,
    SourceRevision,
    SourceVertexId,
    TerminalRelationId,
)
from ..numeric import (
    CertifiedDecimalIntervalV1,
    LocalLengthV1,
    LocalPoint2V1,
    LocalPoint3V1,
    LocalVector2V1,
    LocalVector3V1,
    SourcePositionV1,
    SurfaceCoordinateUnavailableReason,
)
from .surface import PatchSurfaceIRV1


ANALYSIS_SNAPSHOT_SCHEMA_V1 = "cftuv.envelope.analysis_snapshot.v1"


class AnalysisCapability(str, Enum):
    PATCH_DOMAIN_V1 = "PATCH_DOMAIN_V1"
    PHYSICAL_CHAIN_DIRECTED_USES_V1 = "PHYSICAL_CHAIN_DIRECTED_USES_V1"
    PATCH_SURFACE_IR_V1 = "PATCH_SURFACE_IR_V1"
    ORIENTED_OWNER_SECTOR_V1 = "ORIENTED_OWNER_SECTOR_V1"
    REFLEX_ANGLE_CERTIFICATE_V1 = "REFLEX_ANGLE_CERTIFICATE_V1"
    RELATION_LINEAGE_V1 = "RELATION_LINEAGE_V1"
    AUTHORITATIVE_SURFACE_METRIC_V1 = "AUTHORITATIVE_SURFACE_METRIC_V1"
    PHYSICAL_EDGE_TABLE_V1 = "PHYSICAL_EDGE_TABLE_V1"
    BOUNDARY_CONSTRAINT_TARGET_V1 = "BOUNDARY_CONSTRAINT_TARGET_V1"
    TYPED_JUNCTION_ROUTE_TOPOLOGY_V1 = "TYPED_JUNCTION_ROUTE_TOPOLOGY_V1"
    TERMINAL_RELATION_V1 = "TERMINAL_RELATION_V1"
    EC0_COORDINATE_FREE_FIXTURE_V5 = "EC0_COORDINATE_FREE_FIXTURE_V5"


class SurfaceRegime(str, Enum):
    INTRINSIC_PATCH_SURFACE = "INTRINSIC_PATCH_SURFACE"
    PLANAR = "PLANAR"
    DEVELOPABLE = "DEVELOPABLE"
    GENERAL_CURVED = "GENERAL_CURVED"


class PatchShapeClass(str, Enum):
    MIX = "MIX"
    BAND = "BAND"
    CABLE = "CABLE"
    CYLINDER = "CYLINDER"
    NOT_RECORDED_IN_EC0_V5 = "NOT_RECORDED_IN_EC0_V5"


class PatchContextTag(str, Enum):
    WALL = "WALL"
    FLOOR = "FLOOR"
    SLOPE = "SLOPE"


class BoundaryLoopKind(str, Enum):
    OUTER = "OUTER"
    HOLE = "HOLE"


class PhysicalChainKind(str, Enum):
    PHYSICAL_DECAL_SOURCE = "PHYSICAL_DECAL_SOURCE"
    PHYSICAL_SEAM = "PHYSICAL_SEAM"
    SEAM_SELF = "SEAM_SELF"


class ChainUseOrientation(str, Enum):
    SEMANTIC_START_TO_END = "SEMANTIC_START_TO_END"
    A_START_TO_END = "A_START_TO_END"
    B_START_TO_END = "B_START_TO_END"
    AWAY_FROM_SHARED_VERTEX = "AWAY_FROM_SHARED_VERTEX"


class ChainUseRole(str, Enum):
    SELECTED_SPINE = "SELECTED_SPINE"
    DOMAIN_BOUNDARY = "DOMAIN_BOUNDARY"
    BARRIER = "BARRIER"
    FOLD_GUIDE = "FOLD_GUIDE"
    CHART_CUT = "CHART_CUT"
    MARKED_ROUTE = "MARKED_ROUTE"
    DECAL_SOURCE = "DECAL_SOURCE"
    TOPOLOGICAL_BOUNDARY_USE = "TOPOLOGICAL_BOUNDARY_USE"
    SURFACE_ROUTE = "SURFACE_ROUTE"
    CHARACTERISTIC_HINT = "CHARACTERISTIC_HINT"


class BoundaryConstraintKind(str, Enum):
    TOPOLOGICAL_BOUNDARY_USE = "TOPOLOGICAL_BOUNDARY_USE"
    PHYSICAL_DOMAIN_BARRIER = "PHYSICAL_DOMAIN_BARRIER"
    SOURCE_LAUNCH_BOUNDARY = "SOURCE_LAUNCH_BOUNDARY"


class BoundaryConstraintTargetUnavailableReason(str, Enum):
    EC0_COORDINATE_FREE_FIXTURE_V5 = "EC0_COORDINATE_FREE_FIXTURE_V5"


class OwnerInteriorDirection(str, Enum):
    OWNER_INTERIOR = "OWNER_INTERIOR"


class TurnOrientation(str, Enum):
    CCW_IN_OWNER_PATCH_ORIENTATION = "CCW_IN_OWNER_PATCH_ORIENTATION"
    CW_IN_OWNER_PATCH_ORIENTATION = "CW_IN_OWNER_PATCH_ORIENTATION"


class InteriorSelectionLaw(str, Enum):
    OWNER_PATCH_INTERIOR_BETWEEN_ORDERED_SUPPORTS = (
        "OWNER_PATCH_INTERIOR_BETWEEN_ORDERED_SUPPORTS"
    )


class AngleMeasureLaw(str, Enum):
    ORIENTED_OWNER_SECTOR_ANGLE = "ORIENTED_OWNER_SECTOR_ANGLE"


class AngleMeasureSource(str, Enum):
    HOST_ANALYSIS_EXACT_OR_CERTIFIED = "HOST_ANALYSIS_EXACT_OR_CERTIFIED"


class StrictAngleRangeCertificate(str, Enum):
    STRICT_PI_LT_PHI_LT_2PI = "STRICT_PI_LT_PHI_LT_2PI"


class ReflexExcessLaw(str, Enum):
    DELTA_EQUALS_PHI_MINUS_PI = "DELTA_EQUALS_PHI_MINUS_PI"


class PatchFrameHandedness(str, Enum):
    RIGHT_HANDED_U_V_NORMAL = "RIGHT_HANDED_U_V_NORMAL"
    LEFT_HANDED_U_V_NORMAL = "LEFT_HANDED_U_V_NORMAL"


class PlanarityLaw(str, Enum):
    ALL_DOMAIN_SOURCE_VERTICES_ON_CERTIFIED_PLANE_V1 = (
        "ALL_DOMAIN_SOURCE_VERTICES_ON_CERTIFIED_PLANE_V1"
    )


class PlanarProjectionLaw(str, Enum):
    DOT_WITH_AUTHORITATIVE_U_V_BASIS_V1 = "DOT_WITH_AUTHORITATIVE_U_V_BASIS_V1"


class SurfaceMetricAuthority(str, Enum):
    HOST_ANALYSIS_AUTHORITATIVE_V1 = "HOST_ANALYSIS_AUTHORITATIVE_V1"


class AngleMeasureUnavailableReason(str, Enum):
    EC0_SYMBOLIC_ANGLE_ONLY_V5 = "EC0_SYMBOLIC_ANGLE_ONLY_V5"


class SupportDirectionUnavailableReason(str, Enum):
    EC0_SYMBOLIC_SUPPORT_ONLY_V5 = "EC0_SYMBOLIC_SUPPORT_ONLY_V5"


class SupportDirectionAuthority(str, Enum):
    HOST_ANALYSIS_AUTHORITATIVE_V1 = "HOST_ANALYSIS_AUTHORITATIVE_V1"


class AngleNormalizationLaw(str, Enum):
    VALUE_OVER_SYMBOLIC_PI_V1 = "VALUE_OVER_SYMBOLIC_PI_V1"


class StationContinuityLaw(str, Enum):
    SEMANTIC_CHAIN_USE_S_CONTINUOUS_V1 = "SEMANTIC_CHAIN_USE_S_CONTINUOUS_V1"


class JunctionRouteTopologyUnavailableReason(str, Enum):
    EC0_ROUTE_TOPOLOGY_NOT_RECORDED_V5 = "EC0_ROUTE_TOPOLOGY_NOT_RECORDED_V5"


class TerminalRelationKind(str, Enum):
    PHYSICAL_CHAIN_ENDPOINT = "PHYSICAL_CHAIN_ENDPOINT"
    SEAM_SELF_TERMINAL = "SEAM_SELF_TERMINAL"
    POLE = "POLE"


class TerminalEndpointRole(str, Enum):
    START = "START"
    END = "END"
    NON_CHAIN_ENDPOINT = "NON_CHAIN_ENDPOINT"


class JunctionRelationKind(str, Enum):
    T = "T"
    X = "X"
    Y = "Y"
    CROSS_PATCH = "CROSS_PATCH"
    MIXED_ALPHA = "MIXED_ALPHA"


@dataclass(frozen=True, slots=True)
class PlanarityCertificateV1:
    certificate_id: PlanarityCertificateId
    patch_domain_id: PatchDomainId
    law: PlanarityLaw
    exact: bool
    max_absolute_plane_distance: LocalLengthV1
    authority: SurfaceMetricAuthority


@dataclass(frozen=True, slots=True)
class PlanarSourceVertexCoordinateV1:
    source_vertex_id: SourceVertexId
    domain_coordinate: LocalPoint2V1


@dataclass(frozen=True, slots=True)
class PlanarPatchFrameV1:
    patch_domain_id: PatchDomainId
    origin: LocalPoint3V1
    axis_u: LocalVector3V1
    axis_v: LocalVector3V1
    normal: LocalVector3V1
    handedness: PatchFrameHandedness
    planarity_certificate: PlanarityCertificateV1
    projection_law: PlanarProjectionLaw
    source_vertex_coordinates: frozenset[PlanarSourceVertexCoordinateV1]


@dataclass(frozen=True, slots=True)
class IntrinsicSurfaceMetricDescriptorV1:
    patch_domain_id: PatchDomainId
    surface_regime: SurfaceRegime
    authority: SurfaceMetricAuthority
    source_lineage: frozenset[LineageId]


@dataclass(frozen=True, slots=True)
class UnavailableSurfaceMetricDescriptorV1:
    patch_domain_id: PatchDomainId
    reason: SurfaceCoordinateUnavailableReason


SurfaceMetricDescriptorV1 = (
    PlanarPatchFrameV1
    | IntrinsicSurfaceMetricDescriptorV1
    | UnavailableSurfaceMetricDescriptorV1
)


@dataclass(frozen=True, slots=True)
class SourceVertexV1:
    vertex_id: SourceVertexId
    position: SourcePositionV1


@dataclass(frozen=True, slots=True)
class PatchDescriptorV1:
    patch_id: PatchId
    surface_regime: SurfaceRegime
    shape_class: PatchShapeClass
    context_tags: frozenset[PatchContextTag]


@dataclass(frozen=True, slots=True)
class PatchSectorV1:
    owner_sector_id: OwnerSectorId
    chain_use_id: ChainUseId
    analysis_proven: bool
    multiplicity_reason: LawId


@dataclass(frozen=True, slots=True)
class PatchDomainV1:
    patch_domain_id: PatchDomainId
    owner_patch_id: PatchId
    surface_regime: SurfaceRegime
    boundary_constraint_ids: frozenset[BoundaryConstraintId]
    sectors: frozenset[PatchSectorV1]


@dataclass(frozen=True, slots=True)
class BoundaryLoopV1:
    boundary_loop_id: BoundaryLoopId
    patch_domain_id: PatchDomainId
    kind: BoundaryLoopKind
    ordered_chain_use_ids: tuple[ChainUseId, ...]


@dataclass(frozen=True, slots=True)
class PhysicalChainV1:
    physical_chain_id: PhysicalChainId
    kind: PhysicalChainKind
    is_closed: bool
    ordered_source_vertex_ids: tuple[SourceVertexId, ...]
    ordered_physical_edge_ids: tuple[PhysicalEdgeId, ...]
    source_lineage: frozenset[LineageId]
    data_record_lineage: frozenset[LineageId]


@dataclass(frozen=True, slots=True)
class LaunchLocusV1:
    boundary_constraint_id: BoundaryConstraintId
    direction: OwnerInteriorDirection
    source_support_non_blocking: bool


@dataclass(frozen=True, slots=True)
class ChainUseV1:
    chain_use_id: ChainUseId
    physical_chain_id: PhysicalChainId
    owner_patch_id: PatchId
    patch_domain_id: PatchDomainId
    boundary_loop_id: BoundaryLoopId | None
    orientation: ChainUseOrientation
    roles: frozenset[ChainUseRole]
    launch_locus: LaunchLocusV1


@dataclass(frozen=True, slots=True)
class BoundaryConstraintV1:
    boundary_constraint_id: BoundaryConstraintId
    patch_domain_id: PatchDomainId
    constraint_kind: BoundaryConstraintKind
    target: BoundaryConstraintTargetV1
    originating_chain_use_id: ChainUseId | None
    blocks_originating_seed: bool
    blocks_other_fronts: bool


@dataclass(frozen=True, slots=True)
class BoundaryLoopConstraintTargetV1:
    boundary_loop_id: BoundaryLoopId


@dataclass(frozen=True, slots=True)
class ChainUseConstraintTargetV1:
    chain_use_id: ChainUseId


@dataclass(frozen=True, slots=True)
class PhysicalEdgeSequenceConstraintTargetV1:
    ordered_physical_edge_ids: tuple[PhysicalEdgeId, ...]
    is_closed: bool


@dataclass(frozen=True, slots=True)
class UnavailableBoundaryConstraintTargetV1:
    reason: BoundaryConstraintTargetUnavailableReason


BoundaryConstraintTargetV1 = (
    BoundaryLoopConstraintTargetV1
    | ChainUseConstraintTargetV1
    | PhysicalEdgeSequenceConstraintTargetV1
    | UnavailableBoundaryConstraintTargetV1
)


@dataclass(frozen=True, slots=True)
class SourceSupportRefV1:
    chain_use_id: ChainUseId
    source_launch_boundary_id: BoundaryConstraintId
    direction_payload: SourceSupportDirectionPayloadV1


@dataclass(frozen=True, slots=True)
class CertifiedPlanarSupportDirectionV1:
    direction_in_domain: LocalVector2V1
    authority: SupportDirectionAuthority


@dataclass(frozen=True, slots=True)
class UnavailableSourceSupportDirectionV1:
    reason: SupportDirectionUnavailableReason


SourceSupportDirectionPayloadV1 = (
    CertifiedPlanarSupportDirectionV1 | UnavailableSourceSupportDirectionV1
)


@dataclass(frozen=True, slots=True)
class OrientedOwnerSectorV1:
    owner_sector_id: OwnerSectorId
    owner_patch_id: PatchId
    patch_domain_id: PatchDomainId
    analysis_proven: bool
    ordered_incident_chain_use_ids: tuple[ChainUseId, ...]
    incoming_support_ref: SourceSupportRefV1
    outgoing_support_ref: SourceSupportRefV1
    turn_orientation: TurnOrientation
    interior_selection_law: InteriorSelectionLaw


@dataclass(frozen=True, slots=True)
class ReflexAngleCertificateV1:
    certificate_id: AngleCertificateId
    owner_sector_id: OwnerSectorId
    measure_law: AngleMeasureLaw
    measure_source: AngleMeasureSource
    strict_range_certificate: StrictAngleRangeCertificate
    reflex_excess_law: ReflexExcessLaw
    measure_payload: ReflexAngleMeasurePayloadV1
    exact_two_pi: bool


@dataclass(frozen=True, slots=True)
class CertifiedReflexAngleMeasureV1:
    phi_over_pi: CertifiedDecimalIntervalV1
    reflex_excess_over_pi: CertifiedDecimalIntervalV1
    orientation: TurnOrientation
    normalization_law: AngleNormalizationLaw


@dataclass(frozen=True, slots=True)
class UnavailableReflexAngleMeasureV1:
    reason: AngleMeasureUnavailableReason


ReflexAngleMeasurePayloadV1 = (
    CertifiedReflexAngleMeasureV1 | UnavailableReflexAngleMeasureV1
)


@dataclass(frozen=True, slots=True)
class CornerRelationV1:
    corner_relation_id: CornerRelationId
    source_vertex_id: SourceVertexId
    owner_sector_id: OwnerSectorId
    reflex_angle_certificate_id: AngleCertificateId
    exact_two_pi: bool


@dataclass(frozen=True, slots=True)
class JunctionRoutePairV1:
    route_pairing_id: RoutePairingId
    first_chain_use_id: ChainUseId
    second_chain_use_id: ChainUseId
    station_continuity_law: StationContinuityLaw


@dataclass(frozen=True, slots=True)
class TJunctionRouteTopologyV1:
    through_route_pair: JunctionRoutePairV1
    branch_chain_use_ids: frozenset[ChainUseId]


@dataclass(frozen=True, slots=True)
class XJunctionRouteTopologyV1:
    route_pairs: frozenset[JunctionRoutePairV1]


@dataclass(frozen=True, slots=True)
class YJunctionRouteTopologyV1:
    trunk_chain_use_id: ChainUseId
    ordered_branch_chain_use_ids: tuple[ChainUseId, ...]
    route_pairs: frozenset[JunctionRoutePairV1]
    station_continuity_law: StationContinuityLaw


@dataclass(frozen=True, slots=True)
class DeclaredRoutePairsTopologyV1:
    route_pairs: frozenset[JunctionRoutePairV1]


@dataclass(frozen=True, slots=True)
class UnavailableJunctionRouteTopologyV1:
    reason: JunctionRouteTopologyUnavailableReason


JunctionRouteTopologyV1 = (
    TJunctionRouteTopologyV1
    | XJunctionRouteTopologyV1
    | YJunctionRouteTopologyV1
    | DeclaredRoutePairsTopologyV1
    | UnavailableJunctionRouteTopologyV1
)


@dataclass(frozen=True, slots=True)
class JunctionRelationV1:
    junction_relation_id: JunctionRelationId
    source_vertex_id: SourceVertexId
    incident_chain_use_ids: tuple[ChainUseId, ...]
    relation_kind: JunctionRelationKind
    route_topology: JunctionRouteTopologyV1
    shared_semantic_anchor_id: SharedSemanticAnchorId


@dataclass(frozen=True, slots=True)
class TerminalRelationV1:
    terminal_relation_id: TerminalRelationId
    source_vertex_id: SourceVertexId
    chain_use_id: ChainUseId
    owner_patch_id: PatchId
    patch_domain_id: PatchDomainId
    relation_kind: TerminalRelationKind
    endpoint_role: TerminalEndpointRole
    owner_sector_id: OwnerSectorId | None
    source_lineage: frozenset[LineageId]


@dataclass(frozen=True, slots=True)
class AnalysisSnapshotV1:
    schema_version: str
    source_revision: SourceRevision
    analysis_capabilities: frozenset[AnalysisCapability]
    patches: frozenset[PatchDescriptorV1]
    patch_domains: frozenset[PatchDomainV1]
    surface_metric_descriptors: frozenset[SurfaceMetricDescriptorV1]
    surface_ir: PatchSurfaceIRV1
    source_vertices: frozenset[SourceVertexV1]
    physical_chains: frozenset[PhysicalChainV1]
    chain_uses: frozenset[ChainUseV1]
    boundary_loops: frozenset[BoundaryLoopV1]
    boundary_constraints: frozenset[BoundaryConstraintV1]
    angular_owner_sectors: frozenset[OrientedOwnerSectorV1]
    reflex_angle_certificates: frozenset[ReflexAngleCertificateV1]
    corner_relations: frozenset[CornerRelationV1]
    junction_relations: frozenset[JunctionRelationV1]
    terminal_relations: frozenset[TerminalRelationV1]
