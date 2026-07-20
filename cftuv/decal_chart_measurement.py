"""Policy-free замеры фактической метрики intrinsic chart (E0)."""

from __future__ import annotations

from collections import deque
from dataclasses import replace
from math import acos, pi, sqrt

from .decal_charts import (
    _hinge_child_points,
    _root_chart_points,
    _triangle_scale,
    _triangles_overlap_area,
)


_STATION_COUNT = 20
_PATH_LIMIT = 16


def _sub2(first, second):
    return (first[0] - second[0], first[1] - second[1])


def _cross2(first, second):
    return first[0] * second[1] - first[1] * second[0]


def _distance2(first, second):
    delta = _sub2(first, second)
    return sqrt(delta[0] * delta[0] + delta[1] * delta[1])


def _barycentric(point, triangle):
    first, second, third = triangle
    denominator = _cross2(_sub2(second, first), _sub2(third, first))
    if abs(denominator) <= 1e-15:
        return None
    second_weight = _cross2(
        _sub2(point, first), _sub2(third, first)
    ) / denominator
    third_weight = _cross2(
        _sub2(second, first), _sub2(point, first)
    ) / denominator
    return (1.0 - second_weight - third_weight, second_weight, third_weight)


def _locate(chart, point):
    candidates = []
    for triangle in chart.triangles:
        weights = _barycentric(point, triangle.chart_points)
        if weights is None or min(weights) < -1e-9:
            continue
        candidates.append((triangle.triangle_id, weights))
    return min(candidates, default=None)


def _relation_graph(chart):
    graph = {triangle_id: [] for triangle_id in chart.support_triangle_ids}
    for relation in chart.adjacency:
        graph[relation.triangle_a].append((relation.triangle_b, relation))
        graph[relation.triangle_b].append((relation.triangle_a, relation))
    return {
        triangle_id: tuple(sorted(values, key=lambda item: item[0]))
        for triangle_id, values in graph.items()
    }


def _distances_to_target(graph, target):
    result = {target: 0}
    queue = deque((target,))
    while queue:
        triangle_id = queue.popleft()
        for neighbor, _relation in graph.get(triangle_id, ()):
            if neighbor in result:
                continue
            result[neighbor] = result[triangle_id] + 1
            queue.append(neighbor)
    return result


def _candidate_paths(graph, start, target):
    if start == target:
        return ((start,),)
    distances = _distances_to_target(graph, target)
    if start not in distances:
        return ()
    depth_limit = distances[start]
    result = []

    def visit(path):
        current = path[-1]
        if current == target:
            result.append(tuple(path))
            return
        if len(path) - 1 >= depth_limit:
            return
        for neighbor, _relation in graph.get(current, ()):
            if neighbor in path or neighbor not in distances:
                continue
            if len(path) + distances[neighbor] > depth_limit + 1:
                continue
            visit(path + [neighbor])
            if len(result) >= _PATH_LIMIT:
                return

    visit([start])
    return tuple(result)


def _segment_edge_parameter(first, second, edge_first, edge_second):
    direction = _sub2(second, first)
    edge_direction = _sub2(edge_second, edge_first)
    denominator = _cross2(direction, edge_direction)
    if abs(denominator) <= 1e-12:
        return None
    relative = _sub2(edge_first, first)
    parameter = _cross2(relative, edge_direction) / denominator
    edge_parameter = _cross2(relative, direction) / denominator
    if not -1e-8 <= parameter <= 1.0 + 1e-8:
        return None
    if not -1e-8 <= edge_parameter <= 1.0 + 1e-8:
        return None
    return parameter


def _normal_angle(first, second):
    first_length = sqrt(sum(value * value for value in first))
    second_length = sqrt(sum(value * value for value in second))
    if first_length <= 1e-15 or second_length <= 1e-15:
        return 0.0
    cosine = sum(first[i] * second[i] for i in range(3)) / (
        first_length * second_length
    )
    return acos(max(-1.0, min(1.0, cosine)))


def _unfolded_path_measurement(
    chart,
    path,
    relation_lookup,
    station_edge,
    station_factor,
    target_weights,
):
    triangles = {
        triangle.triangle_id: triangle for triangle in chart.triangles
    }
    root_source = triangles[path[0]]
    placed = {
        path[0]: replace(
            root_source,
            chart_points=_root_chart_points(root_source),
        )
    }
    crossed_edges = []
    for first_id, second_id in zip(path, path[1:]):
        relation = relation_lookup[frozenset((first_id, second_id))]
        child = triangles[second_id]
        child_points = _hinge_child_points(
            placed[first_id], child, relation, chart.patch_id
        )
        placed[second_id] = replace(child, chart_points=child_points)
        parent_points = dict(
            zip(
                placed[first_id].source_vertex_ids,
                placed[first_id].chart_points,
            )
        )
        crossed_edges.append(
            (
                parent_points[relation.source_edge[0]],
                parent_points[relation.source_edge[1]],
            )
        )
    root_points = dict(
        zip(root_source.source_vertex_ids, placed[path[0]].chart_points)
    )
    station = tuple(
        root_points[station_edge[0]][axis]
        + (
            root_points[station_edge[1]][axis]
            - root_points[station_edge[0]][axis]
        )
        * station_factor
        for axis in range(2)
    )
    target = tuple(
        sum(
            weight * point[axis]
            for weight, point in zip(
                target_weights, placed[path[-1]].chart_points
            )
        )
        for axis in range(2)
    )
    previous_parameter = -1e-8
    for edge_first, edge_second in crossed_edges:
        parameter = _segment_edge_parameter(
            station, target, edge_first, edge_second
        )
        if parameter is None or parameter + 1e-8 < previous_parameter:
            return None
        previous_parameter = parameter
    root_normal = root_source.face_normal
    normal_variation = max(
        (
            _normal_angle(root_normal, triangles[triangle_id].face_normal)
            for triangle_id in path
        ),
        default=0.0,
    )
    return _distance2(station, target), normal_variation


def _owner_triangle(chart, seed):
    candidates = tuple(
        triangle
        for triangle in chart.triangles
        if seed.source_vertex_ids in triangle.source_edge_ids
        and triangle.source_face_id == seed.source_face_id
    )
    return min(candidates, key=lambda triangle: triangle.triangle_id, default=None)


def _station_samples(chart):
    segments = []
    for seed in sorted(
        chart.site_seeds,
        key=lambda item: item.chain_ref or (-1, -1, item.edge_index),
    ):
        owner = _owner_triangle(chart, seed)
        if owner is None:
            continue
        points = dict(zip(owner.source_vertex_ids, owner.chart_points))
        length = _distance2(
            points[seed.source_vertex_ids[0]],
            points[seed.source_vertex_ids[1]],
        )
        if length > 1e-15:
            segments.append((seed, owner, length))
    total_length = sum(item[2] for item in segments)
    if total_length <= 1e-15:
        return ()
    result = []
    for station_index in range(_STATION_COUNT):
        distance = total_length * (station_index + 0.5) / _STATION_COUNT
        traversed = 0.0
        for seed, owner, length in segments:
            if distance <= traversed + length or (
                seed is segments[-1][0]
            ):
                factor = max(0.0, min(1.0, (distance - traversed) / length))
                result.append((seed, owner, factor))
                break
            traversed += length
    return tuple(result)


def _rail_probe(chart, station, direction, requested_distance):
    endpoint = (
        station[0] + direction[0] * requested_distance,
        station[1] + direction[1] * requested_distance,
    )
    located = _locate(chart, endpoint)
    if located is not None:
        return requested_distance, endpoint, located
    low = 0.0
    high = requested_distance
    for _iteration in range(32):
        middle = (low + high) * 0.5
        probe = (
            station[0] + direction[0] * middle,
            station[1] + direction[1] * middle,
        )
        if _locate(chart, probe) is None:
            high = middle
        else:
            low = middle
    sampled_distance = low * 0.98
    if sampled_distance <= requested_distance * 1e-4:
        return None
    endpoint = (
        station[0] + direction[0] * sampled_distance,
        station[1] + direction[1] * sampled_distance,
    )
    located = _locate(chart, endpoint)
    if located is None:
        return None
    return sampled_distance, endpoint, located


def _foldover_count(chart):
    signs = []
    for triangle in chart.triangles:
        first, second, third = triangle.chart_points
        area = _cross2(_sub2(second, first), _sub2(third, first))
        signs.append(area)
    if not signs:
        return 0
    reference = next((value for value in signs if abs(value) > 1e-15), 1.0)
    count = sum(value * reference <= 0.0 for value in signs)
    adjacent = {
        (relation.triangle_a, relation.triangle_b)
        for relation in chart.adjacency
    }
    triangles = tuple(chart.triangles)
    scale = max(
        (_triangle_scale(triangle) for triangle in triangles),
        default=1.0,
    )
    tolerance = max(1e-12, scale * 1e-10)
    for first_index, first in enumerate(triangles):
        for second in triangles[first_index + 1 :]:
            if (first.triangle_id, second.triangle_id) in adjacent:
                continue
            if not _triangles_overlap_area(
                first.chart_points, second.chart_points, tolerance
            ):
                continue
            if _normal_angle(first.face_normal, second.face_normal) > pi * 0.5:
                count += 1
    return count


def measure_chart_width(chart):
    """Сэмплирует geodesic half-width по triangle-strip unfold paths."""

    graph = _relation_graph(chart)
    relation_lookup = {
        frozenset((relation.triangle_a, relation.triangle_b)): relation
        for relation in chart.adjacency
    }
    maximum_error = 0.0
    maximum_normal_variation = 0.0
    requested_distance = float(chart.alpha_budget)
    path_cache = {}
    for seed, owner, factor in _station_samples(chart):
        points = dict(zip(owner.source_vertex_ids, owner.chart_points))
        first = points[seed.source_vertex_ids[0]]
        second = points[seed.source_vertex_ids[1]]
        edge = _sub2(second, first)
        edge_length = _distance2(first, second)
        if edge_length <= 1e-15:
            continue
        normals = (
            (-edge[1] / edge_length, edge[0] / edge_length),
            (edge[1] / edge_length, -edge[0] / edge_length),
        )
        station = (
            first[0] + edge[0] * factor,
            first[1] + edge[1] * factor,
        )
        for direction in normals:
            probe = _rail_probe(
                chart, station, direction, requested_distance
            )
            if probe is None:
                continue
            sampled_distance, _endpoint, located = probe
            target_id, target_weights = located
            cache_key = (owner.triangle_id, target_id)
            paths = path_cache.get(cache_key)
            if paths is None:
                paths = _candidate_paths(graph, *cache_key)
                path_cache[cache_key] = paths
            candidates = []
            for path in paths:
                measurement = _unfolded_path_measurement(
                    chart,
                    path,
                    relation_lookup,
                    seed.source_vertex_ids,
                    factor,
                    target_weights,
                )
                if measurement is not None:
                    candidates.append(measurement)
            if not candidates:
                continue
            geodesic_distance, normal_variation = min(candidates)
            maximum_error = max(
                maximum_error,
                abs(geodesic_distance - sampled_distance)
                / sampled_distance,
            )
            maximum_normal_variation = max(
                maximum_normal_variation, normal_variation
            )
    return {
        "max_width_error_sampled": maximum_error,
        "max_station_normal_variation": maximum_normal_variation,
        "foldover_count": _foldover_count(chart),
    }


__all__ = ["measure_chart_width"]
