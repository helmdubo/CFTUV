"""Strict developable admission и single-cut policy для Tranche C.

Модуль остаётся чистым: он принимает immutable C1 chart, при необходимости
открывает один annulus source-edge cut, запускает C2 hinge unroll и либо
возвращает доказанно допустимый chart, либо локальный ``ChartBuildFailure``.
"""

from __future__ import annotations

from dataclasses import replace
from math import isfinite

from .decal_charts import (
    ChartBoundaryEdge,
    ChartBuildFailure,
    ChartCut,
    unroll_intrinsic_strip_chart,
)


CHART_DISTORTION_BUDGET = 0.02
CHART_EDGE_RELATIVE_ERROR_LIMIT = 1e-5
CHART_AREA_RATIO_ERROR_LIMIT = 0.01


def _edge_uses(chart):
    uses = {}
    for triangle in chart.triangles:
        for local_edge, source_edge in enumerate(triangle.source_edge_ids):
            uses.setdefault(source_edge, []).append(
                (triangle.triangle_id, local_edge)
            )
    return {
        source_edge: tuple(sorted(edge_uses))
        for source_edge, edge_uses in uses.items()
    }


def _boundary_components(edge_uses, patch_id):
    boundary_edges = tuple(
        source_edge
        for source_edge, uses in edge_uses.items()
        if len(uses) == 1
    )
    neighbors = {}
    for first, second in boundary_edges:
        neighbors.setdefault(first, set()).add(second)
        neighbors.setdefault(second, set()).add(first)
    invalid_vertices = tuple(
        sorted(
            vertex_id
            for vertex_id, values in neighbors.items()
            if len(values) != 2
        )
    )
    if invalid_vertices:
        raise ChartBuildFailure(
            "AMBIGUOUS_PLACEMENT",
            patch_id,
            details=f"boundary_vertices={invalid_vertices!r}",
        )

    remaining = set(neighbors)
    components = []
    while remaining:
        first = min(remaining)
        stack = [first]
        component = set()
        while stack:
            vertex_id = stack.pop()
            if vertex_id in component:
                continue
            component.add(vertex_id)
            remaining.discard(vertex_id)
            stack.extend(sorted(neighbors[vertex_id], reverse=True))
        components.append(frozenset(component))
    return tuple(sorted(components, key=lambda values: min(values)))


def _euler_characteristic(chart, edge_uses):
    vertices = {
        vertex_id
        for triangle in chart.triangles
        for vertex_id in triangle.source_vertex_ids
    }
    return len(vertices) - len(edge_uses) + len(chart.triangles)


def _adjacency_connected(chart, adjacency):
    neighbors = {
        triangle_id: set() for triangle_id in chart.support_triangle_ids
    }
    for relation in adjacency:
        neighbors[relation.triangle_a].add(relation.triangle_b)
        neighbors[relation.triangle_b].add(relation.triangle_a)
    if not neighbors:
        return False
    first = min(neighbors)
    reached = set()
    stack = [first]
    while stack:
        triangle_id = stack.pop()
        if triangle_id in reached:
            continue
        reached.add(triangle_id)
        stack.extend(sorted(neighbors[triangle_id], reverse=True))
    return reached == set(neighbors)


def _annulus_cut(chart, edge_uses, boundary_components):
    if chart.cuts:
        raise ChartBuildFailure("MULTI_CUT_REQUIRED", chart.patch_id)
    component_by_vertex = {
        vertex_id: component_index
        for component_index, component in enumerate(boundary_components)
        for vertex_id in component
    }
    selected_edges = {
        seed.source_vertex_ids for seed in chart.site_seeds
    }
    adjacency_by_edge = {
        relation.source_edge: relation for relation in chart.adjacency
    }
    triangles = {
        triangle.triangle_id: triangle for triangle in chart.triangles
    }
    physical_edge_indices = {}
    bridge_edges = []
    for source_edge, uses in sorted(edge_uses.items()):
        if len(uses) != 2 or source_edge not in adjacency_by_edge:
            continue
        edge_indices = {
            triangles[triangle_id].source_edge_indices[local_edge]
            for triangle_id, local_edge in uses
        }
        if len(edge_indices) != 1 or min(edge_indices) < 0:
            continue
        first_component = component_by_vertex.get(source_edge[0], -1)
        second_component = component_by_vertex.get(source_edge[1], -1)
        if (
            first_component < 0
            or second_component < 0
            or first_component == second_component
        ):
            continue
        bridge_edges.append(source_edge)
        physical_edge_indices[source_edge] = min(edge_indices)
    eligible = tuple(
        source_edge
        for source_edge in bridge_edges
        if source_edge not in selected_edges
    )
    if not eligible:
        reason = (
            "CUT_CROSSES_SELECTED_CHAIN"
            if bridge_edges
            else "MULTI_CUT_REQUIRED"
        )
        raise ChartBuildFailure(
            reason,
            chart.patch_id,
            edge_ids=(
                seed.edge_index
                for seed in chart.site_seeds
                if seed.source_vertex_ids in bridge_edges
            ),
        )

    source_edge = min(eligible)
    relation = adjacency_by_edge[source_edge]
    adjacency = tuple(
        candidate
        for candidate in chart.adjacency
        if candidate.source_edge != source_edge
    )
    if not _adjacency_connected(chart, adjacency):
        raise ChartBuildFailure(
            "MULTI_CUT_REQUIRED",
            chart.patch_id,
            triangle_ids=(relation.triangle_a, relation.triangle_b),
            details=f"disconnecting_cut={source_edge!r}",
        )

    uses = edge_uses[source_edge]
    cut_boundary = tuple(
        ChartBoundaryEdge(
            triangle_id=triangle_id,
            local_edge=local_edge,
            source_edge=source_edge,
            source_face_id=triangles[triangle_id].source_face_id,
            neighbor_triangle_id=(
                relation.triangle_b
                if triangle_id == relation.triangle_a
                else relation.triangle_a
            ),
            kind="CHART_CUT",
        )
        for triangle_id, local_edge in uses
    )
    cut = ChartCut(
        source_edge=source_edge,
        triangle_ids=tuple(
            sorted((relation.triangle_a, relation.triangle_b))
        ),
        reason="OPEN_ANNULUS",
        transition_key=("chart-cut", chart.chart_id, source_edge),
        source_edge_index=physical_edge_indices[source_edge],
    )
    return replace(
        chart,
        adjacency=adjacency,
        boundary_edges=tuple(
            sorted(
                chart.boundary_edges + cut_boundary,
                key=lambda edge: edge.key,
            )
        ),
        cuts=(cut,),
    )


def _prepare_disk_topology(chart):
    edge_uses = _edge_uses(chart)
    boundary_components = _boundary_components(edge_uses, chart.patch_id)
    euler = _euler_characteristic(chart, edge_uses)
    if euler == 1 and len(boundary_components) == 1:
        return chart
    if euler == 0 and len(boundary_components) == 2:
        return _annulus_cut(chart, edge_uses, boundary_components)
    raise ChartBuildFailure(
        "MULTI_CUT_REQUIRED",
        chart.patch_id,
        details=(
            f"euler={euler} boundary_components={len(boundary_components)}"
        ),
    )


def _average_source_edge_length(chart):
    lengths = []
    seen = set()
    for triangle in chart.triangles:
        positions = dict(zip(triangle.source_vertex_ids, triangle.positions))
        for source_edge in triangle.source_edge_ids:
            if source_edge in seen:
                continue
            seen.add(source_edge)
            first = positions[source_edge[0]]
            second = positions[source_edge[1]]
            lengths.append(
                sum(
                    (first[axis] - second[axis]) ** 2
                    for axis in range(3)
                )
                ** 0.5
            )
    return sum(lengths) / len(lengths)


def _closure_normalization(chart, initial_alpha):
    if isfinite(chart.alpha_budget):
        return chart.alpha_budget
    initial_alpha = max(0.0, float(initial_alpha or 0.0))
    return max(initial_alpha, _average_source_edge_length(chart))


def admit_intrinsic_strip_chart(
    chart,
    *,
    initial_alpha=None,
):
    """Применяет G1-G8 и возвращает только production-safe chart."""

    prepared = _prepare_disk_topology(chart)
    admitted = unroll_intrinsic_strip_chart(
        prepared,
        edge_relative_tolerance=CHART_EDGE_RELATIVE_ERROR_LIMIT,
    )
    metrics = admitted.metrics
    if metrics.triangle_overlap_count:
        raise ChartBuildFailure(
            "CHART_SELF_OVERLAP",
            admitted.patch_id,
            triangle_ids=admitted.support_triangle_ids,
            details=f"overlaps={metrics.triangle_overlap_count}",
        )
    normalization = _closure_normalization(admitted, initial_alpha)
    closure_limit = CHART_DISTORTION_BUDGET * normalization
    if (
        metrics.discrete_angle_defect > CHART_DISTORTION_BUDGET
        or metrics.max_loop_closure_residual > closure_limit
    ):
        raise ChartBuildFailure(
            "NON_DEVELOPABLE_SUPPORT",
            admitted.patch_id,
            triangle_ids=admitted.support_triangle_ids,
            details=(
                f"defect={metrics.discrete_angle_defect:.12g} "
                f"closure={metrics.max_loop_closure_residual:.12g} "
                f"closure_limit={closure_limit:.12g}"
            ),
        )
    if metrics.max_edge_error > CHART_EDGE_RELATIVE_ERROR_LIMIT:
        raise ChartBuildFailure(
            "CHART_NUMERIC_DISTORTION",
            admitted.patch_id,
            details=f"edge_error={metrics.max_edge_error:.12g}",
        )
    if (
        abs(metrics.chart_area_source_area_ratio - 1.0)
        > CHART_AREA_RATIO_ERROR_LIMIT
    ):
        raise ChartBuildFailure(
            "CHART_NUMERIC_DISTORTION",
            admitted.patch_id,
            details=(
                f"area_ratio={metrics.chart_area_source_area_ratio:.12g}"
            ),
        )
    if admitted.placed_triangle_ids != admitted.support_triangle_ids:
        raise ChartBuildFailure(
            "AMBIGUOUS_PLACEMENT",
            admitted.patch_id,
            triangle_ids=admitted.support_triangle_ids,
        )
    if len(admitted.cuts) > 1:
        raise ChartBuildFailure("MULTI_CUT_REQUIRED", admitted.patch_id)
    selected_edges = {
        seed.source_vertex_ids for seed in admitted.site_seeds
    }
    if any(cut.source_edge in selected_edges for cut in admitted.cuts):
        raise ChartBuildFailure(
            "CUT_CROSSES_SELECTED_CHAIN",
            admitted.patch_id,
        )
    return admitted


def admit_intrinsic_strip_charts(charts, *, initial_alpha=None):
    return tuple(
        admit_intrinsic_strip_chart(
            chart,
            initial_alpha=initial_alpha,
        )
        for chart in charts
    )


__all__ = (
    "CHART_AREA_RATIO_ERROR_LIMIT",
    "CHART_DISTORTION_BUDGET",
    "CHART_EDGE_RELATIVE_ERROR_LIMIT",
    "admit_intrinsic_strip_chart",
    "admit_intrinsic_strip_charts",
)
