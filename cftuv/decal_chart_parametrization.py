"""Локальная width-изометричная параметризация organic strip chart.

W4 намеренно отделён от admission: модуль строит диагностический кандидат,
но не меняет production routing до отдельного решения G3.
"""

from __future__ import annotations

from collections import deque
from dataclasses import replace
from math import sqrt

from .decal_charts import (
    ChartBuildFailure,
    _hinge_child_points,
    _root_chart_points,
    chart_triangle_overlap_pairs,
)


def _distance3(first, second):
    return sqrt(
        sum((second[axis] - first[axis]) ** 2 for axis in range(3))
    )


def _cross2(first, second, point):
    return (
        (second[0] - first[0]) * (point[1] - first[1])
        - (second[1] - first[1]) * (point[0] - first[0])
    )


def _triangle_graph(chart):
    graph = {triangle_id: [] for triangle_id in chart.support_triangle_ids}
    for relation in chart.adjacency:
        graph[relation.triangle_a].append((relation.triangle_b, relation))
        graph[relation.triangle_b].append((relation.triangle_a, relation))
    return {
        triangle_id: tuple(
            sorted(
                neighbors,
                key=lambda item: (item[0], item[1].source_edge),
            )
        )
        for triangle_id, neighbors in graph.items()
    }


def _seed_owner(chart, seed):
    candidates = tuple(
        triangle
        for triangle in chart.triangles
        if seed.source_vertex_ids in triangle.source_edge_ids
        and triangle.source_face_id == seed.source_face_id
    )
    return min(candidates, key=lambda item: item.triangle_id, default=None)


def _ordered_open_seed_path(chart):
    seeds = tuple(
        sorted(
            chart.site_seeds,
            key=lambda item: item.chain_ref
            or (-1, -1, item.edge_index),
        )
    )
    if not seeds:
        raise ChartBuildFailure(
            "WIDTH_PARAMETERIZATION_UNSUPPORTED",
            chart.patch_id,
            details="site path is empty",
        )
    if len({seed.chain_ref[:2] for seed in seeds if seed.chain_ref}) > 1:
        raise ChartBuildFailure(
            "WIDTH_PARAMETERIZATION_UNSUPPORTED",
            chart.patch_id,
            details="multiple selected chains require atlas semantics",
        )
    edges = [list(seed.source_vertex_ids) for seed in seeds]
    if len(edges) == 1:
        return ((seeds[0], tuple(edges[0])),)
    shared = set(edges[0]).intersection(edges[1])
    if len(shared) != 1:
        raise ChartBuildFailure(
            "WIDTH_PARAMETERIZATION_UNSUPPORTED",
            chart.patch_id,
            details="site segments do not form an open path",
        )
    joint = next(iter(shared))
    edges[0] = [next(value for value in edges[0] if value != joint), joint]
    for index in range(1, len(edges)):
        joint = edges[index - 1][1]
        if joint not in edges[index]:
            raise ChartBuildFailure(
                "WIDTH_PARAMETERIZATION_UNSUPPORTED",
                chart.patch_id,
                details=f"site path disconnects at segment {index}",
            )
        edges[index] = [
            joint,
            next(value for value in edges[index] if value != joint),
        ]
    if edges[-1][1] == edges[0][0]:
        raise ChartBuildFailure(
            "WIDTH_PARAMETERIZATION_UNSUPPORTED",
            chart.patch_id,
            details="periodic site path belongs to tranche D",
        )
    return tuple(
        (seed, tuple(edge)) for seed, edge in zip(seeds, edges)
    )


def _source_geometry(chart):
    positions = {}
    occurrences = {}
    for triangle in chart.triangles:
        for vertex_id, position in zip(
            triangle.source_vertex_ids, triangle.positions
        ):
            previous = positions.setdefault(vertex_id, position)
            if _distance3(previous, position) > 1e-10:
                raise ChartBuildFailure(
                    "WIDTH_PARAMETERIZATION_DESYNC",
                    chart.patch_id,
                    triangle_ids=(triangle.triangle_id,),
                    details=f"source vertex {vertex_id} has two positions",
                )
            occurrences.setdefault(vertex_id, []).append(
                triangle.triangle_id
            )
    return positions, {
        vertex_id: tuple(sorted(triangle_ids))
        for vertex_id, triangle_ids in occurrences.items()
    }


def _unfold_from_owner(chart, owner, graph, triangles):
    placed = {
        owner.triangle_id: replace(
            owner, chart_points=_root_chart_points(owner)
        )
    }
    queue = deque((owner.triangle_id,))
    while queue:
        parent_id = queue.popleft()
        for child_id, relation in graph[parent_id]:
            if child_id in placed:
                continue
            child = triangles[child_id]
            placed[child_id] = replace(
                child,
                chart_points=_hinge_child_points(
                    placed[parent_id], child, relation, chart.patch_id
                ),
            )
            queue.append(child_id)
    if set(placed) != set(chart.support_triangle_ids):
        raise ChartBuildFailure(
            "WIDTH_PARAMETERIZATION_DESYNC",
            chart.patch_id,
            triangle_ids=set(chart.support_triangle_ids).difference(placed),
            details="site owner cannot reach the complete support",
        )
    return placed


def _validate_injective(chart):
    signs = []
    for triangle in chart.triangles:
        signs.append(_cross2(*triangle.chart_points))
    nonzero = tuple(value for value in signs if abs(value) > 1e-12)
    if len(nonzero) != len(signs) or (
        min(nonzero) < 0.0 < max(nonzero)
    ):
        raise ChartBuildFailure(
            "WIDTH_PARAMETERIZATION_NON_INJECTIVE",
            chart.patch_id,
            triangle_ids=chart.support_triangle_ids,
            details="triangle orientation changes after width mapping",
        )
    overlaps = chart_triangle_overlap_pairs(chart)
    if overlaps:
        raise ChartBuildFailure(
            "WIDTH_PARAMETERIZATION_NON_INJECTIVE",
            chart.patch_id,
            triangle_ids={value for pair in overlaps for value in pair},
            details=f"overlaps={len(overlaps)}",
        )


def parameterize_intrinsic_strip_width(chart):
    """Строит `(s, v)` от open site path через локальные hinge-unfold.

    Каждая source-вершина получает одну каноническую координату. `v` —
    подписанное расстояние до опорной прямой ближайшего site-сегмента;
    `s` продолжает крайние сегменты по касательной, поэтому endcap support
    не схлопывается в конечную станцию.
    """

    if chart.placed_triangle_ids != chart.support_triangle_ids:
        raise ChartBuildFailure(
            "WIDTH_PARAMETERIZATION_UNSUPPORTED",
            chart.patch_id,
            details="chart must be hinge-unrolled first",
        )
    if chart.periodic_axis:
        raise ChartBuildFailure(
            "WIDTH_PARAMETERIZATION_UNSUPPORTED",
            chart.patch_id,
            details="periodic chart keeps tranche D coordinates",
        )
    ordered_path = _ordered_open_seed_path(chart)
    positions, occurrences = _source_geometry(chart)
    triangles = {
        triangle.triangle_id: triangle for triangle in chart.triangles
    }
    graph = _triangle_graph(chart)
    candidates = {vertex_id: [] for vertex_id in occurrences}
    anchors = {}
    station = 0.0

    for segment_index, (seed, edge) in enumerate(ordered_path):
        owner = _seed_owner(chart, seed)
        if owner is None:
            raise ChartBuildFailure(
                "WIDTH_PARAMETERIZATION_DESYNC",
                chart.patch_id,
                edge_ids=(seed.edge_index,),
                details="site owner triangle is missing",
            )
        segment_length = _distance3(positions[edge[0]], positions[edge[1]])
        if segment_length <= 1e-15:
            raise ChartBuildFailure(
                "WIDTH_PARAMETERIZATION_DESYNC",
                chart.patch_id,
                edge_ids=(seed.edge_index,),
                details="site segment is collapsed",
            )
        anchors[edge[0]] = (station, 0.0)
        anchors[edge[1]] = (station + segment_length, 0.0)
        placed = _unfold_from_owner(chart, owner, graph, triangles)
        owner_points = dict(
            zip(owner.source_vertex_ids, placed[owner.triangle_id].chart_points)
        )
        first = owner_points[edge[0]]
        second = owner_points[edge[1]]
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        length2 = dx * dx + dy * dy

        for vertex_id, triangle_ids in occurrences.items():
            # Один canonical occurrence исключает width-зависимый выбор
            # представления вершины на цикле support-графа.
            triangle = placed[triangle_ids[0]]
            point = dict(
                zip(triangle.source_vertex_ids, triangle.chart_points)
            )[vertex_id]
            factor = (
                (point[0] - first[0]) * dx
                + (point[1] - first[1]) * dy
            ) / length2
            projection = (
                first[0] + factor * dx,
                first[1] + factor * dy,
            )
            signed_width = _cross2(first, second, point) / segment_length
            outside = max(0.0, -factor, factor - 1.0)
            # Выбор сегмента использует расстояние до конечного segment,
            # но координата продолжает его бесконечную опорную прямую.
            # Это сохраняет естественный open-endcap вместо clamp-collapse.
            segment_distance = (
                abs(signed_width) + outside * segment_length
            )
            candidates[vertex_id].append(
                (
                    segment_distance,
                    outside,
                    segment_index,
                    station + factor * segment_length,
                    signed_width,
                    projection,
                )
            )
        station += segment_length

    coordinates = {
        vertex_id: (min(values)[3], min(values)[4])
        for vertex_id, values in candidates.items()
    }
    coordinates.update(anchors)
    result = replace(
        chart,
        triangles=tuple(
            replace(
                triangle,
                chart_points=tuple(
                    coordinates[vertex_id]
                    for vertex_id in triangle.source_vertex_ids
                ),
            )
            for triangle in chart.triangles
        ),
    )
    _validate_injective(result)
    return result


__all__ = ("parameterize_intrinsic_strip_width",)
