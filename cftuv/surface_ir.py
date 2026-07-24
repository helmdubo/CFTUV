"""Immutable surface contracts shared by analysis and decal runtime.

The module is deliberately Blender-free. Analysis is the only writer;
rail/chart compilers consume emitted polygon cycles and loop triangles
without reconstructing source topology from derived triangulation.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .model import PatchGraph


Vec3 = tuple[float, float, float]


class AnalysisSchemaError(ValueError):
    """Analysis output cannot be consumed by the current decal runtime."""

    def __init__(self, reason: str, details: str = ""):
        self.reason = str(reason)
        self.details = str(details)
        message = self.reason
        if self.details:
            message += f": {self.details}"
        super().__init__(message)


class AnalysisCrossIrError(ValueError):
    """PatchGraph and PatchSurfaceIR disagree about one source revision."""

    def __init__(self, invariant: str, details: str):
        self.invariant = str(invariant)
        self.reason = "DECAL_ANALYSIS_CROSS_IR_INVALID"
        self.details = str(details)
        super().__init__(f"{self.reason}:{self.invariant}: {self.details}")


@dataclass(frozen=True)
class SourceRevision:
    """Stable identity of the source topology/coordinates used by analysis."""

    source_name: str
    digest: str


@dataclass(frozen=True)
class AnalysisCapabilities:
    """Versioned schemas advertised by one analysis result."""

    patch_graph_schema: int = 1
    patch_surface_schema: int = 1
    geometry_batch_schema: int = 1

    def require_supported(self) -> None:
        expected = (1, 1, 1)
        actual = (
            int(self.patch_graph_schema),
            int(self.patch_surface_schema),
            int(self.geometry_batch_schema),
        )
        if actual != expected:
            raise AnalysisSchemaError(
                "DECAL_ANALYSIS_SCHEMA_UNSUPPORTED",
                f"expected={expected} actual={actual}",
            )


@dataclass(frozen=True, order=True)
class SourceVertex:
    vertex_id: int
    position: Vec3


@dataclass(frozen=True, order=True)
class SourceEdge:
    edge_id: int
    vertex_ids: tuple[int, int]
    source_face_ids: tuple[int, ...]


@dataclass(frozen=True)
class SourceFace:
    face_id: int
    patch_id: int
    vertex_cycle: tuple[int, ...]
    edge_cycle: tuple[int, ...]
    polygon_normal: Vec3
    triangle_ids: tuple[int, ...]


@dataclass(frozen=True)
class SurfaceTriangle:
    triangle_id: int
    source_face_id: int
    vertex_ids: tuple[int, int, int]
    # Indexed by the opposite triangle vertex. Blender tessellation
    # diagonals are None: they are not physical source edges.
    physical_edge_ids: tuple[int | None, int | None, int | None]
    triangle_normal: Vec3


@dataclass(frozen=True)
class PatchSurfaceIR:
    """Authoritative source surface: polygon cycles plus Blender triangles."""

    source_revision: SourceRevision
    vertices: tuple[SourceVertex, ...]
    edges: tuple[SourceEdge, ...]
    faces: tuple[SourceFace, ...]
    triangles: tuple[SurfaceTriangle, ...]

    @property
    def vertex_by_id(self) -> dict[int, SourceVertex]:
        return {vertex.vertex_id: vertex for vertex in self.vertices}

    @property
    def edge_by_id(self) -> dict[int, SourceEdge]:
        return {edge.edge_id: edge for edge in self.edges}

    @property
    def face_by_id(self) -> dict[int, SourceFace]:
        return {face.face_id: face for face in self.faces}

    @property
    def triangle_by_id(self) -> dict[int, SurfaceTriangle]:
        return {triangle.triangle_id: triangle for triangle in self.triangles}

    def patch_faces(self, patch_id: int) -> tuple[SourceFace, ...]:
        return tuple(face for face in self.faces if face.patch_id == int(patch_id))

    def patch_triangles(self, patch_id: int) -> tuple[SurfaceTriangle, ...]:
        face_ids = {face.face_id for face in self.patch_faces(patch_id)}
        return tuple(
            triangle
            for triangle in self.triangles
            if triangle.source_face_id in face_ids
        )


@dataclass(frozen=True)
class AnalysisBundle:
    """Atomic analysis output: topology and surface from one revision."""

    source_revision: SourceRevision
    patch_graph: PatchGraph
    patch_surface: PatchSurfaceIR
    capabilities: AnalysisCapabilities = AnalysisCapabilities()

    def __post_init__(self) -> None:
        self.capabilities.require_supported()
        graph_revision = getattr(self.patch_graph, "source_revision", None)
        if graph_revision is not self.source_revision:
            raise AnalysisCrossIrError(
                "REVISION_IDENTITY",
                "PatchGraph does not carry the bundle SourceRevision instance",
            )
        if self.patch_surface.source_revision is not self.source_revision:
            raise AnalysisCrossIrError(
                "REVISION_IDENTITY",
                "PatchSurfaceIR does not carry the bundle SourceRevision instance",
            )

    def __getattr__(self, name):
        """Topology-only compatibility view for existing solve consumers."""

        return getattr(self.patch_graph, name)


class DecalBackendKind(str, Enum):
    RAIL_PLANAR = "RAIL_PLANAR"
    PATCH_VORONOI = "PATCH_VORONOI"


class HostPlanarityPolicy(str, Enum):
    """Какую плоскость хост объявляет ядру для патча.

    Объявляется явно, а не выбирается ядром при отказе: координаты Blender —
    binary64 без гарантии компланарности, поэтому строгая приёмка отвергала
    почти любой отредактированный меш.
    """

    EXACT_SOURCE_PLANE_V1 = "EXACT_SOURCE_PLANE_V1"
    NEAR_PLANAR_PROJECTION_V1 = "NEAR_PLANAR_PROJECTION_V1"


HOST_PLANARITY_POLICY = HostPlanarityPolicy.NEAR_PLANAR_PROJECTION_V1


class PreviewFailurePolicy(str, Enum):
    CLEAR = "CLEAR"


class CapacityPolicy(str, Enum):
    SATURATE_PROVEN = "SATURATE_PROVEN"
    CONTROLLED_RECOMPILE = "CONTROLLED_RECOMPILE"
    REJECT_UNPROVEN = "REJECT_UNPROVEN"


__all__ = (
    "AnalysisBundle",
    "AnalysisCapabilities",
    "AnalysisCrossIrError",
    "AnalysisSchemaError",
    "CapacityPolicy",
    "DecalBackendKind",
    "HOST_PLANARITY_POLICY",
    "HostPlanarityPolicy",
    "PatchSurfaceIR",
    "PreviewFailurePolicy",
    "SourceEdge",
    "SourceFace",
    "SourceRevision",
    "SourceVertex",
    "SurfaceTriangle",
    "Vec3",
)
