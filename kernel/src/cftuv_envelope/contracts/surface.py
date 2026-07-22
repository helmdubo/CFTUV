"""Blender-free PatchSurfaceIR boundary."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..ids import (
    PatchId,
    PhysicalEdgeId,
    SourceFaceId,
    SourceRevision,
    SourceVertexId,
    SurfaceTriangleId,
)
from ..numeric import LocalVector3V1


class SurfacePayloadMode(str, Enum):
    FULL_HOST_SURFACE = "FULL_HOST_SURFACE"
    EC0_COORDINATE_FREE_FIXTURE_V5 = "EC0_COORDINATE_FREE_FIXTURE_V5"


@dataclass(frozen=True, slots=True)
class SourceFaceV1:
    face_id: SourceFaceId
    patch_id: PatchId
    vertex_cycle: tuple[SourceVertexId, ...]
    edge_cycle: tuple[PhysicalEdgeId, ...]
    polygon_normal: LocalVector3V1
    triangle_ids: tuple[SurfaceTriangleId, ...]

    def __post_init__(self) -> None:
        if len(self.vertex_cycle) < 3:
            raise ValueError("SourceFaceV1 requires at least three vertices")
        if len(self.edge_cycle) != len(self.vertex_cycle):
            raise ValueError("SourceFaceV1 edge and vertex cycles must have equal length")


@dataclass(frozen=True, slots=True)
class SurfaceTriangleV1:
    triangle_id: SurfaceTriangleId
    source_face_id: SourceFaceId
    vertex_ids: tuple[SourceVertexId, SourceVertexId, SourceVertexId]
    physical_edge_ids: tuple[
        PhysicalEdgeId | None,
        PhysicalEdgeId | None,
        PhysicalEdgeId | None,
    ]
    triangle_normal: LocalVector3V1

    def __post_init__(self) -> None:
        if len(set(self.vertex_ids)) != 3:
            raise ValueError("SurfaceTriangleV1 requires three distinct source vertices")


@dataclass(frozen=True, slots=True)
class PatchSurfaceIRV1:
    schema_version: str
    source_revision: SourceRevision
    payload_mode: SurfacePayloadMode
    source_vertex_ids: frozenset[SourceVertexId]
    source_faces: frozenset[SourceFaceV1]
    surface_triangles: frozenset[SurfaceTriangleV1]

