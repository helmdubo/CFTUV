"""Compile-only multi-chart atlas для non-developable decal support."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
from math import isfinite, sqrt

from .decal_charts import (
    ChartBoundaryEdge,
    ChartBuildFailure,
    ChartBuildMetrics,
    ChartCut,
    IntrinsicStripChart,
    chart_triangle_overlap_pairs,
    unroll_intrinsic_strip_chart,
)


@dataclass(frozen=True)
class InteriorChartTransition:
    """Изометрия и canonical ownership одной atlas-границы."""

    transition_key: object
    source_edge: tuple[int, int]
    source_edge_index: int
    owner_chart_id: int
    neighbor_chart_id: int
    owner_triangle_id: int
    neighbor_triangle_id: int
    rotation_cos: float
    rotation_sin: float
    translation: tuple[float, float]
    reason: str = "ATLAS_SEPARATOR"
    selection_distance: float = 0.0

    def __post_init__(self):
        if self.source_edge[0] >= self.source_edge[1]:
            raise ValueError("Atlas transition edge must be canonical")
        if self.owner_chart_id >= self.neighbor_chart_id:
            raise ValueError("Atlas transition owner must be canonical")
        values = (
            self.rotation_cos,
            self.rotation_sin,
            *self.translation,
        )
        if any(not isfinite(float(value)) for value in values):
            raise ValueError("Atlas transition transform must be finite")
        if self.reason not in {
            "ATLAS_SEPARATOR",
            "CURVATURE_RELIEF_MARGIN",
        }:
            raise ValueError("Atlas transition reason is invalid")

    def owner_to_neighbor(self, point):
        x, y = point
        return (
            self.rotation_cos * x - self.rotation_sin * y
            + self.translation[0],
            self.rotation_sin * x + self.rotation_cos * y
            + self.translation[1],
        )


@dataclass(frozen=True)
class IntrinsicStripAtlas:
    """Immutable compile IR нескольких locally-injective hinge charts."""

    atlas_id: int
    patch_id: int
    charts: tuple[IntrinsicStripChart, ...]
    transitions: tuple[InteriorChartTransition, ...]
    triangle_owners: tuple[tuple[int, int], ...]
    separator_iterations: int
    unresolved_overlap_count: int = 0
    metrics: ChartBuildMetrics = ChartBuildMetrics()
    admission_tier: str = "APPROXIMATE"

    def __post_init__(self):
        chart_ids = tuple(chart.chart_id for chart in self.charts)
        if tuple(sorted(set(chart_ids))) != chart_ids:
            raise ValueError("Atlas charts must have unique ordered ids")
        triangle_ids = tuple(item[0] for item in self.triangle_owners)
        if tuple(sorted(set(triangle_ids))) != triangle_ids:
            raise ValueError("Atlas triangle ownership must be unique")
        if self.separator_iterations < 0 or self.unresolved_overlap_count < 0:
            raise ValueError("Atlas counters must be non-negative")
        if self.admission_tier not in {"EXACT", "APPROXIMATE"}:
            raise ValueError("Intrinsic atlas admission tier is invalid")

    @property
    def atlas_chart_count(self):
        return len(self.charts)

    @property
    def interior_transition_count(self):
        return len(self.transitions)

    @property
    def margin_relief_cut_count(self):
        return sum(
            transition.reason == "CURVATURE_RELIEF_MARGIN"
            for transition in self.transitions
        )


def _chart_neighbors(chart):
    neighbors = {triangle_id: [] for triangle_id in chart.support_triangle_ids}
    for relation in chart.adjacency:
        neighbors[relation.triangle_a].append(relation.triangle_b)
        neighbors[relation.triangle_b].append(relation.triangle_a)
    return {
        triangle_id: tuple(sorted(values))
        for triangle_id, values in neighbors.items()
    }


def _bfs_parent(neighbors, root):
    parent = {root: -1}
    queue = deque((root,))
    while queue:
        current = queue.popleft()
        for neighbor in neighbors[current]:
            if neighbor in parent:
                continue
            parent[neighbor] = current
            queue.append(neighbor)
    return parent


def _tree_path(parent, first, second):
    ancestors = {}
    current = first
    while current >= 0:
        ancestors[current] = len(ancestors)
        current = parent[current]
    tail = []
    current = second
    while current not in ancestors:
        tail.append(current)
        current = parent[current]
    common = current
    head = []
    current = first
    while current != common:
        head.append(current)
        current = parent[current]
    head.append(common)
    return tuple(head + list(reversed(tail)))


def _source_positions(chart):
    return {
        vertex_id: position
        for triangle in chart.triangles
        for vertex_id, position in zip(
            triangle.source_vertex_ids, triangle.positions
        )
    }


def _point_segment_distance(point, first, second):
    direction = tuple(second[i] - first[i] for i in range(3))
    relative = tuple(point[i] - first[i] for i in range(3))
    denominator = sum(value * value for value in direction)
    if denominator <= 1e-24:
        return sqrt(sum(value * value for value in relative))
    factor = max(
        0.0,
        min(
            1.0,
            sum(relative[i] * direction[i] for i in range(3))
            / denominator,
        ),
    )
    delta = tuple(
        point[i] - (first[i] + direction[i] * factor)
        for i in range(3)
    )
    return sqrt(sum(value * value for value in delta))


def _site_segments(chart, positions):
    return tuple(
        (positions[first], positions[second])
        for seed in chart.site_seeds
        for first, second in (seed.source_vertex_ids,)
    )


def _edge_site_distance(source_edge, positions, site_segments):
    edge_a, edge_b = (positions[index] for index in source_edge)
    return min(
        (
            distance
            for site_a, site_b in site_segments
            for distance in (
                _point_segment_distance(edge_a, site_a, site_b),
                _point_segment_distance(edge_b, site_a, site_b),
                _point_segment_distance(site_a, edge_a, edge_b),
                _point_segment_distance(site_b, edge_a, edge_b),
            )
        ),
        default=float("inf"),
    )


def _triangle_site_distance(chart, triangle_id, positions, site_segments):
    triangle = next(
        item for item in chart.triangles if item.triangle_id == triangle_id
    )
    return min(
        _point_segment_distance(point, site_a, site_b)
        for point in triangle.positions
        for site_a, site_b in site_segments
    )


def _split_support(chart, overlap_pairs, source_chart):
    """Делит support; margin-overlap режет максимально далеко от chain."""

    neighbors = _chart_neighbors(chart)
    root = min(neighbors)
    parent = _bfs_parent(neighbors, root)
    if set(parent) != set(neighbors):
        raise ChartBuildFailure("DISCONNECTED_CHART_SUPPORT", chart.patch_id)
    children = {triangle_id: [] for triangle_id in parent}
    for triangle_id, parent_id in parent.items():
        if parent_id >= 0:
            children[parent_id].append(triangle_id)
    def descendants(candidate):
        result = set()
        stack = [candidate]
        while stack:
            current = stack.pop()
            result.add(current)
            stack.extend(children[current])
        return result

    positions = _source_positions(source_chart)
    site_segments = _site_segments(source_chart, positions)
    relations = {
        tuple(sorted((relation.triangle_a, relation.triangle_b))): relation
        for relation in chart.adjacency
    }
    triangle_distances = {
        triangle_id: _triangle_site_distance(
            source_chart, triangle_id, positions, site_segments
        )
        for pair in overlap_pairs
        for triangle_id in pair
    }
    candidates = []
    total = len(neighbors)
    for overlap_pair in overlap_pairs:
        margin_overlap = all(
            triangle_distances[triangle_id]
            >= source_chart.alpha_budget - 1e-9
            for triangle_id in overlap_pair
        )
        path = _tree_path(parent, *overlap_pair)
        for first, second in zip(path, path[1:]):
            child = first if parent[first] == second else second
            subtree = descendants(child)
            balance = abs(total - 2 * len(subtree))
            distance = -1.0
            margin_rank = 1
            if margin_overlap:
                relation = relations[tuple(sorted((first, second)))]
                distance = _edge_site_distance(
                    relation.source_edge, positions, site_segments
                )
                if distance >= source_chart.alpha_budget - 1e-9:
                    margin_rank = 0
            candidates.append(
                (
                    balance,
                    margin_rank,
                    -distance,
                    min(first, second),
                    max(first, second),
                    overlap_pair,
                    subtree,
                )
            )
    if not candidates:
        raise ChartBuildFailure("ATLAS_INJECTIVITY_UNRESOLVED", chart.patch_id)
    (
        _balance,
        _margin_rank,
        _distance,
        _edge_a,
        _edge_b,
        _pair,
        subtree,
    ) = min(candidates, key=lambda item: item[:6])
    other = set(neighbors).difference(subtree)
    if not subtree or not other:
        raise ChartBuildFailure("ATLAS_INJECTIVITY_UNRESOLVED", chart.patch_id)
    return tuple(sorted(subtree)), tuple(sorted(other))


def _boundary_edges(source_chart, support_ids):
    support = set(support_ids)
    uses = {}
    triangles = {triangle.triangle_id: triangle for triangle in source_chart.triangles}
    for triangle in source_chart.triangles:
        for local_edge, source_edge in enumerate(triangle.source_edge_ids):
            uses.setdefault(source_edge, []).append((triangle.triangle_id, local_edge))
    result = []
    for triangle_id in sorted(support):
        triangle = triangles[triangle_id]
        for local_edge, source_edge in enumerate(triangle.source_edge_ids):
            other = tuple(
                candidate
                for candidate, _local in uses[source_edge]
                if candidate != triangle_id
            )
            if any(candidate in support for candidate in other):
                continue
            neighbor = other[0] if other else -1
            result.append(
                ChartBoundaryEdge(
                    triangle_id=triangle_id,
                    local_edge=local_edge,
                    source_edge=source_edge,
                    source_face_id=triangle.source_face_id,
                    neighbor_triangle_id=neighbor,
                    kind="CHART_CUT" if neighbor >= 0 else "PATCH_BOUNDARY",
                )
            )
    return tuple(sorted(result, key=lambda edge: edge.key))


def _subchart(source_chart, support_ids, chart_id):
    support = set(support_ids)
    triangles = tuple(
        replace(
            triangle,
            chart_points=(),
            parent_triangle_id=-1,
            parent_source_edge=None,
        )
        for triangle in source_chart.triangles
        if triangle.triangle_id in support
    )
    adjacency = tuple(
        relation
        for relation in source_chart.adjacency
        if relation.triangle_a in support and relation.triangle_b in support
    )
    support_edges = {
        edge
        for triangle in triangles
        for edge in triangle.source_edge_ids
    }
    seeds = tuple(
        seed for seed in source_chart.site_seeds
        if seed.source_vertex_ids in support_edges
    )
    provisional = IntrinsicStripChart(
        chart_id=chart_id,
        patch_id=source_chart.patch_id,
        triangles=triangles,
        adjacency=adjacency,
        site_seeds=seeds,
        boundary_edges=_boundary_edges(source_chart, support_ids),
        support_triangle_ids=tuple(sorted(support)),
        alpha_budget=source_chart.alpha_budget,
        budget_source=source_chart.budget_source,
        metrics=ChartBuildMetrics(
            support_triangle_count=len(triangles),
            adjacency_count=len(adjacency),
        ),
    )
    return unroll_intrinsic_strip_chart(
        provisional, edge_relative_tolerance=1e-5
    )


def _transition(source_chart, owner, neighbor, relation, atlas_id):
    charts = {owner.chart_id: owner, neighbor.chart_id: neighbor}
    owner_triangle_id = (
        relation.triangle_a
        if relation.triangle_a in set(owner.support_triangle_ids)
        else relation.triangle_b
    )
    neighbor_triangle_id = (
        relation.triangle_b
        if owner_triangle_id == relation.triangle_a
        else relation.triangle_a
    )
    owner_triangle = next(
        triangle for triangle in owner.triangles
        if triangle.triangle_id == owner_triangle_id
    )
    neighbor_triangle = next(
        triangle for triangle in neighbor.triangles
        if triangle.triangle_id == neighbor_triangle_id
    )
    owner_points = dict(zip(owner_triangle.source_vertex_ids, owner_triangle.chart_points))
    neighbor_points = dict(zip(neighbor_triangle.source_vertex_ids, neighbor_triangle.chart_points))
    first, second = relation.source_edge
    oa, ob = owner_points[first], owner_points[second]
    na, nb = neighbor_points[first], neighbor_points[second]
    ov = (ob[0] - oa[0], ob[1] - oa[1])
    nv = (nb[0] - na[0], nb[1] - na[1])
    length = sqrt(ov[0] * ov[0] + ov[1] * ov[1])
    if length <= 1e-12:
        raise ChartBuildFailure("DEGENERATE_SOURCE_TRIANGLE", source_chart.patch_id)
    cosine = (ov[0] * nv[0] + ov[1] * nv[1]) / (length * length)
    sine = (ov[0] * nv[1] - ov[1] * nv[0]) / (length * length)
    translation = (
        na[0] - (cosine * oa[0] - sine * oa[1]),
        na[1] - (sine * oa[0] + cosine * oa[1]),
    )
    source_edge_index = min(
        (
            triangle.source_edge_indices[local_edge]
            for triangle in source_chart.triangles
            for local_edge, edge in enumerate(triangle.source_edge_ids)
            if edge == relation.source_edge
            and triangle.source_edge_indices[local_edge] >= 0
        ),
        default=-1,
    )
    source_positions = _source_positions(source_chart)
    selection_distance = _edge_site_distance(
        relation.source_edge,
        source_positions,
        _site_segments(source_chart, source_positions),
    )
    reason = (
        "CURVATURE_RELIEF_MARGIN"
        if selection_distance >= source_chart.alpha_budget - 1e-9
        else "ATLAS_SEPARATOR"
    )
    return InteriorChartTransition(
        transition_key=(
            "atlas-transition", atlas_id, relation.source_edge,
            owner.chart_id, neighbor.chart_id,
        ),
        source_edge=relation.source_edge,
        source_edge_index=source_edge_index,
        owner_chart_id=owner.chart_id,
        neighbor_chart_id=neighbor.chart_id,
        owner_triangle_id=owner_triangle_id,
        neighbor_triangle_id=neighbor_triangle_id,
        rotation_cos=cosine,
        rotation_sin=sine,
        translation=translation,
        reason=reason,
        selection_distance=selection_distance,
    )


def build_intrinsic_strip_atlas(chart, *, max_iterations=None):
    """Разбивает self-overlapping hinge chart на locally-injective charts."""

    source_chart = chart
    initial = chart if chart.placed_triangle_ids else unroll_intrinsic_strip_chart(
        chart, edge_relative_tolerance=1e-5
    )
    limit = min(8, len(chart.support_triangle_ids))
    if max_iterations is not None:
        limit = min(limit, max(0, int(max_iterations)))
    supports = [tuple(initial.support_triangle_ids)]
    placed = [initial]
    iterations = 0
    while any(chart_triangle_overlap_pairs(item) for item in placed):
        if iterations >= limit:
            unresolved = sum(
                len(chart_triangle_overlap_pairs(item)) for item in placed
            )
            raise ChartBuildFailure(
                "ATLAS_INJECTIVITY_UNRESOLVED",
                chart.patch_id,
                triangle_ids=chart.support_triangle_ids,
                details=f"iterations={iterations} overlaps={unresolved}",
            )
        next_supports = []
        for item in placed:
            pairs = chart_triangle_overlap_pairs(item)
            if not pairs:
                next_supports.append(tuple(item.support_triangle_ids))
                continue
            next_supports.extend(_split_support(item, pairs, source_chart))
        supports = sorted(next_supports, key=lambda values: (min(values), values))
        placed = [
            _subchart(source_chart, support, (source_chart.chart_id << 16) + index)
            for index, support in enumerate(supports)
        ]
        iterations += 1

    owner_by_triangle = {
        triangle_id: item.chart_id
        for item in placed
        for triangle_id in item.support_triangle_ids
    }
    if set(owner_by_triangle) != set(source_chart.support_triangle_ids):
        raise ChartBuildFailure("AMBIGUOUS_PLACEMENT", chart.patch_id)
    charts_by_id = {item.chart_id: item for item in placed}
    transitions = []
    for relation in source_chart.adjacency:
        first_id = owner_by_triangle[relation.triangle_a]
        second_id = owner_by_triangle[relation.triangle_b]
        if first_id == second_id:
            continue
        owner_id, neighbor_id = sorted((first_id, second_id))
        transitions.append(
            _transition(
                source_chart,
                charts_by_id[owner_id],
                charts_by_id[neighbor_id],
                relation,
                source_chart.chart_id,
            )
        )
    transitions.sort(
        key=lambda item: (
            item.owner_chart_id, item.neighbor_chart_id, item.source_edge
        )
    )
    cuts_by_chart = {item.chart_id: [] for item in placed}
    for transition in transitions:
        for chart_id, triangle_id in (
            (transition.owner_chart_id, transition.owner_triangle_id),
            (transition.neighbor_chart_id, transition.neighbor_triangle_id),
        ):
            cuts_by_chart[chart_id].append(
                ChartCut(
                    source_edge=transition.source_edge,
                    triangle_ids=(triangle_id,),
                    reason=transition.reason,
                    transition_key=transition.transition_key,
                    source_edge_index=transition.source_edge_index,
                    selection_distance=transition.selection_distance,
                    source_edges=(transition.source_edge,),
                )
            )
    placed = [
        replace(
            item,
            cuts=tuple(
                sorted(
                    cuts_by_chart[item.chart_id],
                    key=lambda cut: (cut.source_edge, cut.triangle_ids),
                )
            ),
            metrics=replace(
                item.metrics, cut_count=len(cuts_by_chart[item.chart_id])
            ),
        )
        for item in placed
    ]
    return IntrinsicStripAtlas(
        atlas_id=source_chart.chart_id,
        patch_id=source_chart.patch_id,
        charts=tuple(sorted(placed, key=lambda item: item.chart_id)),
        transitions=tuple(transitions),
        triangle_owners=tuple(sorted(owner_by_triangle.items())),
        separator_iterations=iterations,
        metrics=source_chart.metrics,
    )


__all__ = (
    "InteriorChartTransition",
    "IntrinsicStripAtlas",
    "build_intrinsic_strip_atlas",
)
