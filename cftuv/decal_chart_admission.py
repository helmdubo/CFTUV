"""Strict developable admission и single-cut policy для Tranche C.

Модуль остаётся чистым: он принимает immutable C1 chart, при необходимости
открывает один annulus source-edge cut, запускает C2 hinge unroll и либо
возвращает доказанно допустимый chart, либо локальный ``ChartBuildFailure``.
"""

from __future__ import annotations

from dataclasses import replace
from math import acos, atan2, cos, isfinite, sin, sqrt

from .decal_charts import (
    ChartBoundaryEdge,
    ChartBuildFailure,
    ChartCut,
    unroll_intrinsic_strip_chart,
)
from .decal_diagram import DiagramTransformError, build_diagram_transform


CHART_DISTORTION_BUDGET = 0.02
CHART_EDGE_RELATIVE_ERROR_LIMIT = 1e-5
CHART_AREA_RATIO_ERROR_LIMIT = 0.01


def _distance3(first, second):
    return sqrt(
        sum((first[axis] - second[axis]) ** 2 for axis in range(3))
    )


def _point_segment_distance3(point, first, second):
    direction = tuple(second[i] - first[i] for i in range(3))
    relative = tuple(point[i] - first[i] for i in range(3))
    denominator = sum(value * value for value in direction)
    if denominator <= 1e-24:
        return _distance3(point, first)
    parameter = sum(
        relative[i] * direction[i] for i in range(3)
    ) / denominator
    parameter = max(0.0, min(1.0, parameter))
    closest = tuple(
        first[i] + direction[i] * parameter for i in range(3)
    )
    return _distance3(point, closest)


def _segment_distance3(first_a, first_b, second_a, second_b):
    # Source mesh edges не пересекаются без общей source vertex. Для
    # непересекающихся отрезков minimum достигается endpoint projection.
    return min(
        _point_segment_distance3(first_a, second_a, second_b),
        _point_segment_distance3(first_b, second_a, second_b),
        _point_segment_distance3(second_a, first_a, first_b),
        _point_segment_distance3(second_b, first_a, first_b),
    )


def _source_positions(chart):
    result = {}
    for triangle in chart.triangles:
        for vertex_id, position in zip(
            triangle.source_vertex_ids, triangle.positions
        ):
            result.setdefault(vertex_id, position)
    return result


def _distance_to_selected_sites(chart, source_edge):
    positions = _source_positions(chart)
    first_a, first_b = (positions[index] for index in source_edge)
    distances = []
    for seed in chart.site_seeds:
        second_a, second_b = (
            positions[index] for index in seed.source_vertex_ids
        )
        distances.append(
            _segment_distance3(
                first_a, first_b, second_a, second_b
            )
        )
    return min(distances) if distances else float("inf")


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

    scored = tuple(
        (_distance_to_selected_sites(chart, source_edge), source_edge)
        for source_edge in eligible
    )
    maximum_distance = max(score for score, _edge in scored)
    source_edge = min(
        edge
        for score, edge in scored
        if abs(score - maximum_distance) <= 1e-12
    )
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
        selection_distance=maximum_distance,
        candidate_count=len(eligible),
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


def _sub2(first, second):
    return (first[0] - second[0], first[1] - second[1])


def _length2(vector):
    return sqrt(vector[0] * vector[0] + vector[1] * vector[1])


def _periodic_cut_segments(chart, cut):
    triangles = {
        triangle.triangle_id: triangle for triangle in chart.triangles
    }
    result = []
    for triangle_id in cut.triangle_ids:
        triangle = triangles[triangle_id]
        points = dict(zip(triangle.source_vertex_ids, triangle.chart_points))
        try:
            result.append(
                (points[cut.source_edge[0]], points[cut.source_edge[1]])
            )
        except KeyError as exc:
            raise ChartBuildFailure(
                "PERIODIC_HOLONOMY_UNSUPPORTED",
                chart.patch_id,
                triangle_ids=cut.triangle_ids,
                details="cut image loses source-edge endpoint",
            ) from exc
    if len(result) != 2:
        raise ChartBuildFailure(
            "PERIODIC_HOLONOMY_UNSUPPORTED",
            chart.patch_id,
            triangle_ids=cut.triangle_ids,
            details=f"cut_images={len(result)}",
        )
    return tuple(result)


def _periodicize_annulus(chart):
    """Доказывает translation holonomy и канонизирует period по DP2/DP3."""

    cut = chart.cuts[0]
    first, second = _periodic_cut_segments(chart, cut)
    translations = (
        _sub2(second[0], first[0]),
        _sub2(second[1], first[1]),
    )
    lengths = tuple(_length2(value) for value in translations)
    if min(lengths) <= 1e-12:
        raise ChartBuildFailure(
            "PERIODIC_HOLONOMY_UNSUPPORTED",
            chart.patch_id,
            triangle_ids=cut.triangle_ids,
            details=f"translation_lengths={lengths!r}",
        )
    cosine = sum(
        translations[0][axis] * translations[1][axis]
        for axis in range(2)
    ) / (lengths[0] * lengths[1])
    angle = acos(max(-1.0, min(1.0, cosine)))
    relative_length_error = abs(lengths[0] - lengths[1]) / max(lengths)
    if (
        angle > CHART_DISTORTION_BUDGET
        or relative_length_error > CHART_DISTORTION_BUDGET
    ):
        raise ChartBuildFailure(
            "PERIODIC_HOLONOMY_UNSUPPORTED",
            chart.patch_id,
            triangle_ids=cut.triangle_ids,
            details=(
                f"rotation={angle:.12g} "
                f"length_error={relative_length_error:.12g}"
            ),
        )
    translation = tuple(
        (translations[0][axis] + translations[1][axis]) * 0.5
        for axis in range(2)
    )
    rotation = -atan2(translation[1], translation[0])
    cosine_rotation = cos(rotation)
    sine_rotation = sin(rotation)

    def rotate(point):
        return (
            point[0] * cosine_rotation - point[1] * sine_rotation,
            point[0] * sine_rotation + point[1] * cosine_rotation,
        )

    rotated = tuple(
        replace(
            triangle,
            chart_points=tuple(rotate(point) for point in triangle.chart_points),
        )
        for triangle in chart.triangles
    )
    minimum_u = min(
        point[0] for triangle in rotated for point in triangle.chart_points
    )
    rotated = tuple(
        replace(
            triangle,
            chart_points=tuple(
                (point[0] - minimum_u, point[1])
                for point in triangle.chart_points
            ),
        )
        for triangle in rotated
    )
    provisional = replace(chart, triangles=rotated)
    try:
        transform = build_diagram_transform(
            point
            for triangle in provisional.triangles
            for point in triangle.chart_points
        )
    except DiagramTransformError as exc:
        raise ChartBuildFailure(
            "PERIODIC_HOLONOMY_UNSUPPORTED",
            chart.patch_id,
            details=exc.details,
        ) from exc
    first, second = _periodic_cut_segments(provisional, cut)
    first = tuple(transform.quantize(point) for point in first)
    second = tuple(transform.quantize(point) for point in second)
    quantized_translations = (
        _sub2(second[0], first[0]),
        _sub2(second[1], first[1]),
    )
    period_values = tuple(value[0] for value in quantized_translations)
    transverse_values = tuple(value[1] for value in quantized_translations)
    if (
        min(period_values) <= 0.0
        or abs(period_values[0] - period_values[1]) > transform.quantum * 1e-7
        or max(abs(value) for value in transverse_values)
        > transform.quantum * 1e-7
    ):
        raise ChartBuildFailure(
            "PERIODIC_HOLONOMY_UNSUPPORTED",
            chart.patch_id,
            triangle_ids=cut.triangle_ids,
            details=(
                f"quantized_translation={quantized_translations!r} "
                f"quantum={transform.quantum:.12g}"
            ),
        )
    period = period_values[0]
    canonical_key = cut.transition_key
    equivalences = (
        (
            canonical_key,
            (
                ("periodic-image", chart.chart_id, cut.source_edge, -1),
                ("periodic-image", chart.chart_id, cut.source_edge, 1),
            ),
        ),
    )
    return replace(
        provisional,
        alpha_budget=min(float(provisional.alpha_budget), period * 0.5),
        budget_source="PERIODIC_HALF_PERIOD",
        periodic_axis="U",
        period=period,
        period_quantum=transform.quantum,
        wrap_origin=0.0,
        periodic_cut=cut,
        transition_equivalences=equivalences,
    )


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
    if admitted.cuts and admitted.cuts[0].reason == "OPEN_ANNULUS":
        admitted = _periodicize_annulus(admitted)
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
