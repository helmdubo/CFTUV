"""Blender-free immutable Envelope debug-scene contracts."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..ids import (
    DebugDiagnosticId,
    DebugPrimitiveId,
    DecalRequestId,
    LineageId,
    PatchDomainId,
    SourceRevision,
)
from ..numeric import LocalLengthV1, LocalPoint3V1, LocalVector3V1


ENVELOPE_DEBUG_SCENE_SCHEMA_V1 = "cftuv.envelope.debug_scene.v1"


class EnvelopeDebugStage(str, Enum):
    ANALYSIS = "ANALYSIS"
    COMPILE = "COMPILE"
    ENVELOPE_INSTANCE = "ENVELOPE_INSTANCE"
    RAW_COVERAGE = "RAW_COVERAGE"
    INTERACTION = "INTERACTION"
    RESOLVED_COVERAGE = "RESOLVED_COVERAGE"
    DIAGNOSTIC = "DIAGNOSTIC"


class DebugPrimitiveKind(str, Enum):
    PATCH_DOMAIN_OUTER = "PATCH_DOMAIN_OUTER"
    PATCH_DOMAIN_HOLE = "PATCH_DOMAIN_HOLE"
    PHYSICAL_BARRIER = "PHYSICAL_BARRIER"
    PHYSICAL_CHAIN = "PHYSICAL_CHAIN"
    CHAIN_USE = "CHAIN_USE"
    SOURCE_SUPPORT = "SOURCE_SUPPORT"
    MOVING_FRONT = "MOVING_FRONT"
    HIDDEN_SUPPORT = "HIDDEN_SUPPORT"
    ENVELOPE_INSTANCE = "ENVELOPE_INSTANCE"
    RAW_COVERAGE = "RAW_COVERAGE"
    INTERACTION_COMPONENT = "INTERACTION_COMPONENT"
    FRONT_READING = "FRONT_READING"
    EQUALITY_LOCUS = "EQUALITY_LOCUS"
    RESOLVED_COVERAGE = "RESOLVED_COVERAGE"
    RETAINED_REGION = "RETAINED_REGION"
    REMOVED_REGION = "REMOVED_REGION"
    EVENT_ANCHOR = "EVENT_ANCHOR"
    DIAGNOSTIC = "DIAGNOSTIC"
    LABEL = "LABEL"


class DebugDiagnosticSeverity(str, Enum):
    INFO = "INFO"
    CAPACITY = "CAPACITY"
    UNSUPPORTED = "UNSUPPORTED"
    ERROR = "ERROR"


@dataclass(frozen=True, slots=True)
class DebugPatchFrameV1:
    patch_domain_id: PatchDomainId
    origin: LocalPoint3V1
    axis_u: LocalVector3V1
    axis_v: LocalVector3V1
    normal: LocalVector3V1


@dataclass(frozen=True, slots=True)
class DebugExactPoint2V1:
    x_expression: str
    y_expression: str

    def __post_init__(self) -> None:
        if not self.x_expression or not self.y_expression:
            raise ValueError("DebugExactPoint2V1 requires canonical expressions")


@dataclass(frozen=True, slots=True)
class DebugPointV1:
    semantic_id: DebugPrimitiveId
    patch_domain_id: PatchDomainId
    stage: EnvelopeDebugStage
    kind: DebugPrimitiveKind
    exact_point: DebugExactPoint2V1
    source_lineage_ids: frozenset[LineageId]
    style_key: str
    label: str


@dataclass(frozen=True, slots=True)
class DebugPathV1:
    semantic_id: DebugPrimitiveId
    patch_domain_id: PatchDomainId
    stage: EnvelopeDebugStage
    kind: DebugPrimitiveKind
    exact_points: tuple[DebugExactPoint2V1, ...]
    closed: bool
    source_lineage_ids: frozenset[LineageId]
    style_key: str
    label: str

    def __post_init__(self) -> None:
        if len(self.exact_points) < 2:
            raise ValueError("DebugPathV1 requires at least two points")


@dataclass(frozen=True, slots=True)
class DebugLoopV1:
    semantic_id: DebugPrimitiveId
    patch_domain_id: PatchDomainId
    stage: EnvelopeDebugStage
    kind: DebugPrimitiveKind
    exact_points: tuple[DebugExactPoint2V1, ...]
    closed: bool
    source_lineage_ids: frozenset[LineageId]
    style_key: str
    label: str

    def __post_init__(self) -> None:
        if not self.closed or len(self.exact_points) < 3:
            raise ValueError("DebugLoopV1 requires a closed loop with three points")


@dataclass(frozen=True, slots=True)
class DebugRegionV1:
    semantic_id: DebugPrimitiveId
    patch_domain_id: PatchDomainId
    stage: EnvelopeDebugStage
    kind: DebugPrimitiveKind
    outer_exact_points: tuple[DebugExactPoint2V1, ...]
    hole_exact_point_loops: tuple[tuple[DebugExactPoint2V1, ...], ...]
    closed: bool
    source_lineage_ids: frozenset[LineageId]
    style_key: str
    label: str

    def __post_init__(self) -> None:
        if not self.closed or len(self.outer_exact_points) < 3:
            raise ValueError("DebugRegionV1 requires a closed outer loop")
        if any(len(loop) < 3 for loop in self.hole_exact_point_loops):
            raise ValueError("DebugRegionV1 hole loops require at least three points")


@dataclass(frozen=True, slots=True)
class DebugLabelV1:
    semantic_id: DebugPrimitiveId
    patch_domain_id: PatchDomainId
    stage: EnvelopeDebugStage
    kind: DebugPrimitiveKind
    exact_anchor: DebugExactPoint2V1
    source_lineage_ids: frozenset[LineageId]
    style_key: str
    label: str


@dataclass(frozen=True, slots=True)
class DebugDiagnosticV1:
    diagnostic_id: DebugDiagnosticId
    patch_domain_id: PatchDomainId | None
    stage: EnvelopeDebugStage
    kind: DebugPrimitiveKind
    outcome: str
    severity: DebugDiagnosticSeverity
    message: str
    exact_anchor: DebugExactPoint2V1 | None
    source_lineage_ids: frozenset[LineageId]
    style_key: str
    label: str


@dataclass(frozen=True, slots=True)
class EnvelopeDebugSceneV1:
    schema_version: str
    source_revision: SourceRevision
    request_ids: tuple[DecalRequestId, ...]
    patch_domain_ids: tuple[PatchDomainId, ...]
    requested_alpha: LocalLengthV1
    patch_frames: tuple[DebugPatchFrameV1, ...]
    points: tuple[DebugPointV1, ...]
    paths: tuple[DebugPathV1, ...]
    loops: tuple[DebugLoopV1, ...]
    regions: tuple[DebugRegionV1, ...]
    labels: tuple[DebugLabelV1, ...]
    diagnostics: tuple[DebugDiagnosticV1, ...]
    raw_coverage_digests: tuple[str, ...]
    resolved_coverage_digests: tuple[str, ...]
