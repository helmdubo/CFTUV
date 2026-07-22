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
    PhysicalChainId,
    SharedSemanticAnchorId,
    SourceRevision,
    SourceVertexId,
)
from ..numeric import SourcePositionV1
from .surface import PatchSurfaceIRV1


ANALYSIS_SNAPSHOT_SCHEMA_V1 = "cftuv.envelope.analysis_snapshot.v1"


class AnalysisCapability(str, Enum):
    PATCH_DOMAIN_V1 = "PATCH_DOMAIN_V1"
    PHYSICAL_CHAIN_DIRECTED_USES_V1 = "PHYSICAL_CHAIN_DIRECTED_USES_V1"
    PATCH_SURFACE_IR_V1 = "PATCH_SURFACE_IR_V1"
    ORIENTED_OWNER_SECTOR_V1 = "ORIENTED_OWNER_SECTOR_V1"
    REFLEX_ANGLE_CERTIFICATE_V1 = "REFLEX_ANGLE_CERTIFICATE_V1"
    RELATION_LINEAGE_V1 = "RELATION_LINEAGE_V1"
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


class JunctionRelationKind(str, Enum):
    T = "T"
    X = "X"
    Y = "Y"
    CROSS_PATCH = "CROSS_PATCH"
    MIXED_ALPHA = "MIXED_ALPHA"


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
    ordered_source_vertex_ids: tuple[SourceVertexId, ...]
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
    originating_chain_use_id: ChainUseId | None
    blocks_originating_seed: bool
    blocks_other_fronts: bool


@dataclass(frozen=True, slots=True)
class SourceSupportRefV1:
    chain_use_id: ChainUseId
    source_launch_boundary_id: BoundaryConstraintId


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
    exact_two_pi: bool


@dataclass(frozen=True, slots=True)
class CornerRelationV1:
    corner_relation_id: CornerRelationId
    source_vertex_id: SourceVertexId
    owner_sector_id: OwnerSectorId
    reflex_angle_certificate_id: AngleCertificateId
    exact_two_pi: bool


@dataclass(frozen=True, slots=True)
class JunctionRelationV1:
    junction_relation_id: JunctionRelationId
    source_vertex_id: SourceVertexId
    incident_chain_use_ids: tuple[ChainUseId, ...]
    relation_kind: JunctionRelationKind
    route_pairing: tuple[ChainUseId, ...]
    shared_semantic_anchor_id: SharedSemanticAnchorId


@dataclass(frozen=True, slots=True)
class AnalysisSnapshotV1:
    schema_version: str
    source_revision: SourceRevision
    analysis_capabilities: frozenset[AnalysisCapability]
    patches: frozenset[PatchDescriptorV1]
    patch_domains: frozenset[PatchDomainV1]
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
