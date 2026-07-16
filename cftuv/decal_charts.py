"""Immutable IR для developable intrinsic decal charts.

Модуль намеренно не импортирует ``bpy``/``bmesh`` и не строит adjacency или
развёртку: C0 фиксирует data boundary, C1/C2 наполнят её алгоритмами. Входом
служит только сериализованная геометрия ``PatchNode`` и provenance chains.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite


Vec2 = tuple[float, float]
Vec3 = tuple[float, float, float]
SourceEdge = tuple[int, int]
ChainRef = tuple[int, int, int]


def _vec3(value) -> Vec3:
    return (float(value[0]), float(value[1]), float(value[2]))


def _source_edge(first, second) -> SourceEdge:
    first = int(first)
    second = int(second)
    return (first, second) if first < second else (second, first)


@dataclass(frozen=True)
class ChartTriangle:
    """Один serialized source triangle и его будущая chart placement."""

    triangle_id: int
    source_face_id: int
    source_vertex_ids: tuple[int, int, int]
    positions: tuple[Vec3, Vec3, Vec3]
    face_normal: Vec3
    chart_points: tuple[Vec2, ...] = ()

    def __post_init__(self):
        if self.triangle_id < 0:
            raise ValueError("Chart triangle id must be non-negative")
        if self.source_face_id < 0:
            raise ValueError("Chart triangle source face must be explicit")
        if len(set(self.source_vertex_ids)) != 3:
            raise ValueError("Chart triangle source vertices must be unique")
        if len(self.positions) != 3 or len(self.face_normal) != 3:
            raise ValueError("Chart triangle source geometry must be 3D")
        if self.chart_points and len(self.chart_points) != 3:
            raise ValueError("Placed chart triangle must have three points")

    @property
    def is_placed(self) -> bool:
        return len(self.chart_points) == 3

    @property
    def source_edge_ids(self) -> tuple[SourceEdge, SourceEdge, SourceEdge]:
        """Edges индексируются противоположной локальной вершиной."""

        first, second, third = self.source_vertex_ids
        return (
            _source_edge(second, third),
            _source_edge(third, first),
            _source_edge(first, second),
        )


@dataclass(frozen=True)
class ChartSiteSeed:
    """Selected BoundaryChain segment без live mesh references."""

    edge_index: int
    source_vertex_ids: SourceEdge
    source_face_id: int
    chain_ref: ChainRef | None = None

    def __post_init__(self):
        if self.edge_index < 0:
            raise ValueError("Chart site seed edge index must be non-negative")
        if self.source_face_id < 0:
            raise ValueError("Chart site seed source face must be explicit")
        if self.source_vertex_ids[0] >= self.source_vertex_ids[1]:
            raise ValueError("Chart site seed edge must be canonical")


@dataclass(frozen=True)
class ChartAdjacency:
    """Одна undirected triangle relation по общему source edge."""

    triangle_a: int
    triangle_b: int
    source_edge: SourceEdge
    local_edge_a: int
    local_edge_b: int

    def __post_init__(self):
        if self.triangle_a >= self.triangle_b:
            raise ValueError("Chart adjacency triangle ids must be ordered")
        if self.source_edge[0] >= self.source_edge[1]:
            raise ValueError("Chart adjacency edge must be canonical")
        if not 0 <= self.local_edge_a < 3 or not 0 <= self.local_edge_b < 3:
            raise ValueError("Chart adjacency local edge must be in [0, 2]")

    @property
    def key(self):
        return (self.triangle_a, self.triangle_b, self.source_edge)


@dataclass(frozen=True)
class ChartCut:
    """Детерминированный разрыв одной source-edge relation."""

    source_edge: SourceEdge
    triangle_ids: tuple[int, ...]
    reason: str
    transition_key: object

    def __post_init__(self):
        if self.source_edge[0] >= self.source_edge[1]:
            raise ValueError("Chart cut edge must be canonical")
        if not self.triangle_ids or len(self.triangle_ids) > 2:
            raise ValueError("Chart cut must reference one or two triangles")
        if tuple(sorted(set(self.triangle_ids))) != self.triangle_ids:
            raise ValueError("Chart cut triangle ids must be unique/sorted")
        if not self.reason:
            raise ValueError("Chart cut reason must be explicit")
        if self.transition_key is None:
            raise ValueError("Chart cut requires a transition key")


@dataclass(frozen=True)
class ChartBuildMetrics:
    """C0 container для измерений C1/C2/C3 без admission policy."""

    support_triangle_count: int = 0
    adjacency_count: int = 0
    boundary_edge_count: int = 0
    cut_count: int = 0
    max_edge_error: float = 0.0
    max_loop_closure_residual: float = 0.0
    discrete_angle_defect: float = 0.0
    triangle_overlap_count: int = 0
    chart_area_source_area_ratio: float = 1.0

    def __post_init__(self):
        counts = (
            self.support_triangle_count,
            self.adjacency_count,
            self.boundary_edge_count,
            self.cut_count,
            self.triangle_overlap_count,
        )
        if any(count < 0 for count in counts):
            raise ValueError("Chart metric counts must be non-negative")
        values = (
            self.max_edge_error,
            self.max_loop_closure_residual,
            self.discrete_angle_defect,
            self.chart_area_source_area_ratio,
        )
        if any(not isfinite(float(value)) for value in values):
            raise ValueError("Chart metrics must be finite")
        if self.chart_area_source_area_ratio < 0.0:
            raise ValueError("Chart/source area ratio must be non-negative")


@dataclass(frozen=True)
class IntrinsicStripChart:
    """Полный immutable chart result, заполняемый последующими slices."""

    chart_id: int
    patch_id: int
    triangles: tuple[ChartTriangle, ...]
    adjacency: tuple[ChartAdjacency, ...] = ()
    site_seeds: tuple[ChartSiteSeed, ...] = ()
    cuts: tuple[ChartCut, ...] = ()
    support_triangle_ids: tuple[int, ...] = ()
    alpha_budget: float = float("inf")
    budget_source: str = "FULL_CONNECTED_COMPONENT"
    metrics: ChartBuildMetrics = field(default_factory=ChartBuildMetrics)

    def __post_init__(self):
        if self.chart_id < 0 or self.patch_id < 0:
            raise ValueError("Chart and patch ids must be non-negative")
        triangle_ids = tuple(triangle.triangle_id for triangle in self.triangles)
        if tuple(sorted(set(triangle_ids))) != triangle_ids:
            raise ValueError("Chart triangle ids must be unique/sorted")
        support_ids = self.support_triangle_ids or triangle_ids
        if not self.support_triangle_ids:
            object.__setattr__(self, "support_triangle_ids", support_ids)
        if tuple(sorted(set(support_ids))) != support_ids:
            raise ValueError("Chart support ids must be unique/sorted")
        if not set(support_ids).issubset(triangle_ids):
            raise ValueError("Chart support references an unknown triangle")
        triangles_by_id = {
            triangle.triangle_id: triangle for triangle in self.triangles
        }
        adjacency_keys = tuple(relation.key for relation in self.adjacency)
        if tuple(sorted(set(adjacency_keys))) != adjacency_keys:
            raise ValueError("Chart adjacency must be unique/sorted")
        for relation in self.adjacency:
            if (
                relation.triangle_a not in support_ids
                or relation.triangle_b not in support_ids
            ):
                raise ValueError("Chart adjacency leaves support")
            edge_a = triangles_by_id[
                relation.triangle_a
            ].source_edge_ids[relation.local_edge_a]
            edge_b = triangles_by_id[
                relation.triangle_b
            ].source_edge_ids[relation.local_edge_b]
            if edge_a != relation.source_edge or edge_b != relation.source_edge:
                raise ValueError(
                    "Chart adjacency local edges disagree with provenance"
                )
        support_edges = {
            edge
            for triangle_id in support_ids
            for edge in triangles_by_id[triangle_id].source_edge_ids
        }
        if any(
            seed.source_vertex_ids not in support_edges
            for seed in self.site_seeds
        ):
            raise ValueError("Chart site seed leaves support")
        for cut in self.cuts:
            if not set(cut.triangle_ids).issubset(support_ids):
                raise ValueError("Chart cut leaves support")
            if cut.source_edge not in support_edges:
                raise ValueError("Chart cut edge leaves support")
        if not self.alpha_budget > 0.0:
            raise ValueError("Chart alpha budget must be positive")
        if not self.budget_source:
            raise ValueError("Chart budget source must be explicit")

    @property
    def placed_triangle_ids(self) -> tuple[int, ...]:
        return tuple(
            triangle.triangle_id
            for triangle in self.triangles
            if triangle.is_placed
        )


class ChartBuildFailure(ValueError):
    """Локализованный отказ chart construction/admission."""

    def __init__(
        self,
        code,
        patch_id,
        *,
        triangle_ids=(),
        edge_ids=(),
        details="",
    ):
        self.code = str(code)
        self.patch_id = int(patch_id)
        self.triangle_ids = tuple(
            sorted({int(value) for value in triangle_ids})
        )
        self.edge_ids = tuple(sorted({int(value) for value in edge_ids}))
        self.details = str(details)
        message = f"{self.code}: patch={self.patch_id}"
        if self.triangle_ids:
            message += f" triangles={self.triangle_ids}"
        if self.edge_ids:
            message += f" edges={self.edge_ids}"
        if self.details:
            message += f" ({self.details})"
        super().__init__(message)


def chart_triangles_from_patch(node) -> tuple[ChartTriangle, ...]:
    """Создаёт C0 triangle snapshots только из serialized PatchNode fields."""

    patch_id = int(node.patch_id)
    vertices = tuple(node.mesh_verts)
    source_vertex_ids = tuple(getattr(node, "mesh_vert_indices", ()))
    triangles = tuple(node.mesh_tris)
    face_ids = tuple(getattr(node, "mesh_tri_face_indices", ()))
    face_normals = tuple(getattr(node, "mesh_tri_face_normals", ()))
    if not vertices or not triangles:
        raise ChartBuildFailure("EMPTY_PATCH_GEOMETRY", patch_id)
    if len(source_vertex_ids) != len(vertices):
        raise ChartBuildFailure(
            "MISSING_SOURCE_VERTEX_PROVENANCE",
            patch_id,
            details=f"verts={len(vertices)} ids={len(source_vertex_ids)}",
        )
    if len(set(source_vertex_ids)) != len(source_vertex_ids):
        raise ChartBuildFailure(
            "DUPLICATE_SOURCE_VERTEX_PROVENANCE",
            patch_id,
        )
    if len(face_ids) != len(triangles) or len(face_normals) != len(triangles):
        raise ChartBuildFailure(
            "MISSING_TRIANGLE_FACE_PROVENANCE",
            patch_id,
            details=(
                f"tris={len(triangles)} faces={len(face_ids)} "
                f"normals={len(face_normals)}"
            ),
        )

    result = []
    for triangle_id, local_vertex_ids in enumerate(triangles):
        if len(local_vertex_ids) != 3 or any(
            index < 0 or index >= len(vertices) for index in local_vertex_ids
        ):
            raise ChartBuildFailure(
                "INVALID_SERIALIZED_TRIANGLE",
                patch_id,
                triangle_ids=(triangle_id,),
                details=f"local_vertices={tuple(local_vertex_ids)!r}",
            )
        try:
            triangle = ChartTriangle(
                triangle_id=triangle_id,
                source_face_id=int(face_ids[triangle_id]),
                source_vertex_ids=tuple(
                    int(source_vertex_ids[index]) for index in local_vertex_ids
                ),
                positions=tuple(
                    _vec3(vertices[index]) for index in local_vertex_ids
                ),
                face_normal=_vec3(face_normals[triangle_id]),
            )
        except (TypeError, ValueError) as exc:
            raise ChartBuildFailure(
                "INVALID_SERIALIZED_TRIANGLE",
                patch_id,
                triangle_ids=(triangle_id,),
                details=str(exc),
            ) from exc
        result.append(triangle)
    return tuple(result)


def chart_site_seeds_from_chain(
    chain, chain_ref: ChainRef | None = None
) -> tuple[ChartSiteSeed, ...]:
    """Сериализует selected chain segments для C1 seed lookup."""

    vertex_ids = tuple(int(value) for value in chain.vert_indices)
    edge_ids = tuple(int(value) for value in chain.edge_indices)
    segment_count = len(vertex_ids) if chain.is_closed else len(vertex_ids) - 1
    if segment_count <= 0 or len(edge_ids) != segment_count:
        raise ValueError("BoundaryChain segment provenance is inconsistent")
    face_ids = tuple(int(value) for value in chain.side_face_indices)
    if len(face_ids) < segment_count:
        raise ValueError("BoundaryChain face provenance is inconsistent")
    result = []
    for segment_index, edge_id in enumerate(edge_ids):
        next_index = (segment_index + 1) % len(vertex_ids)
        result.append(
            ChartSiteSeed(
                edge_index=edge_id,
                source_vertex_ids=_source_edge(
                    vertex_ids[segment_index], vertex_ids[next_index]
                ),
                source_face_id=face_ids[segment_index],
                chain_ref=chain_ref,
            )
        )
    return tuple(result)


__all__ = (
    "ChartAdjacency",
    "ChartBuildFailure",
    "ChartBuildMetrics",
    "ChartCut",
    "ChartSiteSeed",
    "ChartTriangle",
    "IntrinsicStripChart",
    "chart_site_seeds_from_chain",
    "chart_triangles_from_patch",
)
