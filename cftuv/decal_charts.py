"""Immutable IR и support builder для developable intrinsic decal charts.

Модуль намеренно не импортирует ``bpy``/``bmesh``. C0 фиксирует data boundary,
C1 строит triangle adjacency и conservative strip support; hinge unroll
остаётся следующим отдельным этапом. Входом служит только сериализованная
геометрия ``PatchNode`` и provenance chains.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite, sqrt


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
class ChartBoundaryEdge:
    """Граница support с source provenance и excluded neighbor."""

    triangle_id: int
    local_edge: int
    source_edge: SourceEdge
    source_face_id: int
    neighbor_triangle_id: int = -1
    kind: str = "PATCH_BOUNDARY"

    def __post_init__(self):
        if self.triangle_id < 0 or not 0 <= self.local_edge < 3:
            raise ValueError("Chart boundary triangle/local edge is invalid")
        if self.source_edge[0] >= self.source_edge[1]:
            raise ValueError("Chart boundary source edge must be canonical")
        if self.source_face_id < 0:
            raise ValueError("Chart boundary source face must be explicit")
        if self.kind not in {"PATCH_BOUNDARY", "SUPPORT_CUT"}:
            raise ValueError("Chart boundary kind is unsupported")
        if self.kind == "PATCH_BOUNDARY" and self.neighbor_triangle_id >= 0:
            raise ValueError("Patch boundary cannot reference a neighbor")
        if self.kind == "SUPPORT_CUT" and self.neighbor_triangle_id < 0:
            raise ValueError("Support cut requires an excluded neighbor")

    @property
    def key(self):
        return (self.triangle_id, self.local_edge, self.source_edge)


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
    boundary_edges: tuple[ChartBoundaryEdge, ...] = ()
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
        boundary_keys = tuple(edge.key for edge in self.boundary_edges)
        if tuple(sorted(set(boundary_keys))) != boundary_keys:
            raise ValueError("Chart boundary edges must be unique/sorted")
        for edge in self.boundary_edges:
            if edge.triangle_id not in support_ids:
                raise ValueError("Chart boundary edge leaves support")
            source_edge = triangles_by_id[
                edge.triangle_id
            ].source_edge_ids[edge.local_edge]
            if source_edge != edge.source_edge:
                raise ValueError(
                    "Chart boundary local edge disagrees with provenance"
                )
            if (
                edge.kind == "SUPPORT_CUT"
                and edge.neighbor_triangle_id in support_ids
            ):
                raise ValueError("Chart support cut points inside support")
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


def _edge_uses(triangles):
    uses = {}
    for triangle in triangles:
        for local_edge, source_edge in enumerate(triangle.source_edge_ids):
            uses.setdefault(source_edge, []).append(
                (triangle.triangle_id, local_edge)
            )
    return {
        edge: tuple(sorted(edge_uses)) for edge, edge_uses in uses.items()
    }


def build_triangle_adjacency(
    triangles,
    *,
    patch_id=-1,
) -> tuple[ChartAdjacency, ...]:
    """Строит C1 adjacency по общим undirected source edges."""

    triangles = tuple(triangles)
    adjacency = []
    for source_edge, uses in sorted(_edge_uses(triangles).items()):
        if len(uses) > 2:
            raise ChartBuildFailure(
                "NON_MANIFOLD_CHART_EDGE",
                patch_id,
                triangle_ids=(triangle_id for triangle_id, _edge in uses),
                details=f"source_edge={source_edge!r} uses={len(uses)}",
            )
        if len(uses) != 2:
            continue
        (triangle_a, local_edge_a), (triangle_b, local_edge_b) = uses
        if triangle_a > triangle_b:
            triangle_a, triangle_b = triangle_b, triangle_a
            local_edge_a, local_edge_b = local_edge_b, local_edge_a
        adjacency.append(
            ChartAdjacency(
                triangle_a=triangle_a,
                triangle_b=triangle_b,
                source_edge=source_edge,
                local_edge_a=local_edge_a,
                local_edge_b=local_edge_b,
            )
        )
    return tuple(sorted(adjacency, key=lambda relation: relation.key))


def _seed_sort_key(seed):
    return (
        seed.edge_index,
        seed.source_vertex_ids,
        seed.source_face_id,
        seed.chain_ref or (-1, -1, -1),
    )


def _seed_components(site_seeds):
    """Selected edge network components по общим source vertices."""

    remaining = set(range(len(site_seeds)))
    components = []
    while remaining:
        first = min(remaining)
        remaining.remove(first)
        component = {first}
        vertices = set(site_seeds[first].source_vertex_ids)
        changed = True
        while changed:
            changed = False
            for seed_index in sorted(remaining):
                if not vertices.intersection(
                    site_seeds[seed_index].source_vertex_ids
                ):
                    continue
                remaining.remove(seed_index)
                component.add(seed_index)
                vertices.update(site_seeds[seed_index].source_vertex_ids)
                changed = True
        components.append(tuple(sorted(component)))
    return tuple(components)


def _triangle_bounds(triangle):
    return tuple(
        (
            min(position[axis] for position in triangle.positions),
            max(position[axis] for position in triangle.positions),
        )
        for axis in range(3)
    )


def _segment_bounds(first, second):
    return tuple(
        (min(first[axis], second[axis]), max(first[axis], second[axis]))
        for axis in range(3)
    )


def _aabb_distance(first, second):
    distance2 = 0.0
    for axis in range(3):
        if first[axis][1] < second[axis][0]:
            gap = second[axis][0] - first[axis][1]
        elif second[axis][1] < first[axis][0]:
            gap = first[axis][0] - second[axis][1]
        else:
            gap = 0.0
        distance2 += gap * gap
    return sqrt(distance2)


def _triangle_neighbors(triangle_ids, adjacency):
    neighbors = {triangle_id: set() for triangle_id in triangle_ids}
    for relation in adjacency:
        neighbors[relation.triangle_a].add(relation.triangle_b)
        neighbors[relation.triangle_b].add(relation.triangle_a)
    return {
        triangle_id: tuple(sorted(values))
        for triangle_id, values in neighbors.items()
    }


def _seed_owner_triangles(
    site_seeds,
    seed_indices,
    triangles,
    edge_uses,
    patch_id,
):
    triangles_by_id = {
        triangle.triangle_id: triangle for triangle in triangles
    }
    owners = set()
    for seed_index in seed_indices:
        seed = site_seeds[seed_index]
        uses = edge_uses.get(seed.source_vertex_ids, ())
        face_matches = tuple(
            triangle_id
            for triangle_id, _local_edge in uses
            if triangles_by_id[triangle_id].source_face_id
            == seed.source_face_id
        )
        if not face_matches:
            raise ChartBuildFailure(
                "SITE_SEED_NOT_IN_PATCH",
                patch_id,
                edge_ids=(seed.edge_index,),
                details=(
                    f"source_edge={seed.source_vertex_ids!r} "
                    f"source_face={seed.source_face_id}"
                ),
            )
        owners.update(face_matches)
    return tuple(sorted(owners))


def _component_support(
    site_seeds,
    seed_indices,
    triangles,
    adjacency,
    alpha_budget,
    patch_id,
):
    triangles_by_id = {
        triangle.triangle_id: triangle for triangle in triangles
    }
    edge_uses = _edge_uses(triangles)
    owner_ids = _seed_owner_triangles(
        site_seeds,
        seed_indices,
        triangles,
        edge_uses,
        patch_id,
    )
    positions_by_vertex = {}
    for triangle in triangles:
        for vertex_id, position in zip(
            triangle.source_vertex_ids, triangle.positions
        ):
            positions_by_vertex.setdefault(vertex_id, position)
    seed_bounds = []
    for seed_index in seed_indices:
        first_id, second_id = site_seeds[seed_index].source_vertex_ids
        if (
            first_id not in positions_by_vertex
            or second_id not in positions_by_vertex
        ):
            raise ChartBuildFailure(
                "SITE_SEED_VERTEX_NOT_IN_PATCH",
                patch_id,
                edge_ids=(site_seeds[seed_index].edge_index,),
            )
        seed_bounds.append(
            _segment_bounds(
                positions_by_vertex[first_id],
                positions_by_vertex[second_id],
            )
        )

    tolerance = max(1e-9, alpha_budget * 1e-9)
    candidates = {
        triangle.triangle_id
        for triangle in triangles
        if any(
            _aabb_distance(_triangle_bounds(triangle), bounds)
            <= alpha_budget + tolerance
            for bounds in seed_bounds
        )
    }
    neighbors = _triangle_neighbors(triangles_by_id, adjacency)
    # One-ring safety защищает от пограничной потери из-за AABB/numeric ties.
    candidates.update(
        neighbor
        for triangle_id in tuple(candidates)
        for neighbor in neighbors[triangle_id]
    )
    candidates.update(owner_ids)

    support = set()
    frontier = list(reversed(owner_ids))
    while frontier:
        triangle_id = frontier.pop()
        if triangle_id in support or triangle_id not in candidates:
            continue
        support.add(triangle_id)
        frontier.extend(
            reversed(
                tuple(
                    neighbor
                    for neighbor in neighbors[triangle_id]
                    if neighbor not in support
                )
            )
        )
    return tuple(sorted(support))


def _merge_overlapping_supports(component_supports):
    parent = list(range(len(component_supports)))

    def root(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    for first in range(len(component_supports)):
        first_support = set(component_supports[first][1])
        for second in range(first + 1, len(component_supports)):
            if not first_support.intersection(component_supports[second][1]):
                continue
            root_first = root(first)
            root_second = root(second)
            if root_first != root_second:
                parent[root_second] = root_first

    merged = {}
    for index, (seed_indices, triangle_ids) in enumerate(component_supports):
        bucket = merged.setdefault(root(index), [set(), set()])
        bucket[0].update(seed_indices)
        bucket[1].update(triangle_ids)
    return tuple(
        (tuple(sorted(seeds)), tuple(sorted(triangles)))
        for _root, (seeds, triangles) in sorted(
            merged.items(),
            key=lambda item: (min(item[1][0]), min(item[1][1])),
        )
    )


def _support_boundary_edges(triangles, support_ids):
    triangles_by_id = {
        triangle.triangle_id: triangle for triangle in triangles
    }
    uses = _edge_uses(triangles)
    support = set(support_ids)
    boundary = []
    for triangle_id in support_ids:
        triangle = triangles_by_id[triangle_id]
        for local_edge, source_edge in enumerate(triangle.source_edge_ids):
            other_uses = tuple(
                other_id
                for other_id, _other_edge in uses[source_edge]
                if other_id != triangle_id
            )
            if any(other_id in support for other_id in other_uses):
                continue
            neighbor_id = other_uses[0] if other_uses else -1
            boundary.append(
                ChartBoundaryEdge(
                    triangle_id=triangle_id,
                    local_edge=local_edge,
                    source_edge=source_edge,
                    source_face_id=triangle.source_face_id,
                    neighbor_triangle_id=neighbor_id,
                    kind=(
                        "SUPPORT_CUT"
                        if neighbor_id >= 0
                        else "PATCH_BOUNDARY"
                    ),
                )
            )
    return tuple(sorted(boundary, key=lambda edge: edge.key))


def build_intrinsic_strip_charts(
    node,
    site_seeds,
    alpha_budget,
    *,
    chart_id_start=0,
) -> tuple[IntrinsicStripChart, ...]:
    """Строит C1 adjacency/support IR без hinge unroll и admission."""

    alpha_budget = float(alpha_budget)
    if not isfinite(alpha_budget) or alpha_budget <= 0.0:
        raise ValueError("C1 alpha budget must be finite and positive")
    site_seeds = tuple(sorted(tuple(site_seeds), key=_seed_sort_key))
    if not site_seeds:
        raise ChartBuildFailure("NO_CHART_SITE_SEEDS", int(node.patch_id))
    triangles = chart_triangles_from_patch(node)
    patch_id = int(node.patch_id)
    adjacency = build_triangle_adjacency(triangles, patch_id=patch_id)
    components = _seed_components(site_seeds)
    supports = tuple(
        (
            seed_indices,
            _component_support(
                site_seeds,
                seed_indices,
                triangles,
                adjacency,
                alpha_budget,
                patch_id,
            ),
        )
        for seed_indices in components
    )
    merged_supports = _merge_overlapping_supports(supports)
    triangles_by_id = {
        triangle.triangle_id: triangle for triangle in triangles
    }
    charts = []
    for chart_offset, (seed_indices, support_ids) in enumerate(
        merged_supports
    ):
        support_set = set(support_ids)
        support_adjacency = tuple(
            relation
            for relation in adjacency
            if relation.triangle_a in support_set
            and relation.triangle_b in support_set
        )
        boundary_edges = _support_boundary_edges(triangles, support_ids)
        chart_seeds = tuple(site_seeds[index] for index in seed_indices)
        charts.append(
            IntrinsicStripChart(
                chart_id=int(chart_id_start) + chart_offset,
                patch_id=int(node.patch_id),
                triangles=tuple(
                    triangles_by_id[triangle_id]
                    for triangle_id in support_ids
                ),
                adjacency=support_adjacency,
                site_seeds=chart_seeds,
                boundary_edges=boundary_edges,
                support_triangle_ids=support_ids,
                alpha_budget=alpha_budget,
                budget_source="STRIP_BUDGET",
                metrics=ChartBuildMetrics(
                    support_triangle_count=len(support_ids),
                    adjacency_count=len(support_adjacency),
                    boundary_edge_count=len(boundary_edges),
                ),
            )
        )
    return tuple(charts)


__all__ = (
    "ChartAdjacency",
    "ChartBoundaryEdge",
    "ChartBuildFailure",
    "ChartBuildMetrics",
    "ChartCut",
    "ChartSiteSeed",
    "ChartTriangle",
    "IntrinsicStripChart",
    "build_intrinsic_strip_charts",
    "build_triangle_adjacency",
    "chart_site_seeds_from_chain",
    "chart_triangles_from_patch",
)
