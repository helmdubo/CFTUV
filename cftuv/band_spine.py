from __future__ import annotations

from heapq import heappop, heappush
import math
from types import MappingProxyType
from typing import Optional

from mathutils import Vector

try:
    from .analysis_records import BandSpineData
    from .model import BoundaryChain, FrameRole, PatchGraph
except ImportError:
    from analysis_records import BandSpineData
    from model import BoundaryChain, FrameRole, PatchGraph


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def _chain_has_corner(chain: BoundaryChain, corner_index: int) -> bool:
    return corner_index >= 0 and corner_index in {chain.start_corner_index, chain.end_corner_index}


def _group_has_corner(
    graph: PatchGraph,
    chain_refs: tuple[tuple[int, int, int], ...],
    corner_index: int,
) -> bool:
    for chain_ref in chain_refs:
        chain = graph.get_chain(*chain_ref)
        if chain is not None and _chain_has_corner(chain, corner_index):
            return True
    return False


def _polyline_cumulative_lengths(points: tuple[Vector, ...]) -> tuple[tuple[float, ...], float]:
    if not points:
        return (), 0.0
    cumulative = [0.0]
    walked = 0.0
    for point_index in range(1, len(points)):
        walked += (points[point_index] - points[point_index - 1]).length
        cumulative.append(walked)
    return tuple(cumulative), walked


def _sample_polyline_at_distance(
    points: tuple[Vector, ...],
    cumulative: tuple[float, ...],
    distance: float,
) -> Vector:
    if not points:
        return Vector((0.0, 0.0, 0.0))
    if len(points) == 1 or not cumulative:
        return points[0].copy()
    if distance <= 0.0:
        return points[0].copy()
    total_length = cumulative[-1]
    if distance >= total_length:
        return points[-1].copy()

    for segment_index in range(len(points) - 1):
        start_distance = cumulative[segment_index]
        end_distance = cumulative[segment_index + 1]
        if distance > end_distance and segment_index < len(points) - 2:
            continue
        segment_span = end_distance - start_distance
        if segment_span <= 1e-8:
            return points[segment_index + 1].copy()
        t = _clamp01((distance - start_distance) / segment_span)
        return points[segment_index].lerp(points[segment_index + 1], t)

    return points[-1].copy()


def _resample_polyline(points: tuple[Vector, ...], sample_count: int) -> tuple[Vector, ...]:
    if not points:
        return ()
    if sample_count <= 1 or len(points) == 1:
        return (points[0].copy(),)

    cumulative, total_length = _polyline_cumulative_lengths(points)
    if total_length <= 1e-8:
        return tuple(points[0].copy() for _ in range(sample_count))

    samples = []
    for sample_index in range(sample_count):
        distance = total_length * float(sample_index) / float(sample_count - 1)
        samples.append(_sample_polyline_at_distance(points, cumulative, distance))
    return tuple(samples)


def _safe_normalized(vector: Vector, fallback: Vector) -> Vector:
    if vector.length > 1e-8:
        return vector.normalized()
    if fallback.length > 1e-8:
        return fallback.normalized()
    return Vector((1.0, 0.0, 0.0))


def _sample_cubic_hermite(
    p0: Vector,
    p1: Vector,
    m0: Vector,
    m1: Vector,
    t: float,
) -> Vector:
    t = _clamp01(t)
    t2 = t * t
    t3 = t2 * t
    h00 = 2.0 * t3 - 3.0 * t2 + 1.0
    h10 = t3 - 2.0 * t2 + t
    h01 = -2.0 * t3 + 3.0 * t2
    h11 = t3 - t2
    return p0 * h00 + m0 * h10 + p1 * h01 + m1 * h11


def _sample_polyline_direction(points: tuple[Vector, ...], sample_index: int) -> Vector:
    if len(points) < 2:
        return Vector((0.0, 0.0, 0.0))
    if sample_index <= 0:
        return points[1] - points[0]
    if sample_index >= len(points) - 1:
        return points[-1] - points[-2]
    return points[sample_index + 1] - points[sample_index - 1]


def _line_intersection_in_patch_plane(
    point_a: Vector,
    direction_a: Vector,
    point_b: Vector,
    direction_b: Vector,
    basis_u: Vector,
    basis_v: Vector,
) -> Optional[Vector]:
    if direction_a.length <= 1e-8 or direction_b.length <= 1e-8:
        return None

    origin = (point_a + point_b) * 0.5
    point_a_2d = Vector(((point_a - origin).dot(basis_u), (point_a - origin).dot(basis_v)))
    point_b_2d = Vector(((point_b - origin).dot(basis_u), (point_b - origin).dot(basis_v)))
    dir_a_2d = Vector((direction_a.dot(basis_u), direction_a.dot(basis_v)))
    dir_b_2d = Vector((direction_b.dot(basis_u), direction_b.dot(basis_v)))

    determinant = dir_a_2d.x * dir_b_2d.y - dir_a_2d.y * dir_b_2d.x
    if abs(determinant) <= 1e-8:
        return None

    delta = point_b_2d - point_a_2d
    t = (delta.x * dir_b_2d.y - delta.y * dir_b_2d.x) / determinant
    intersection_2d = point_a_2d + dir_a_2d * t
    return origin + basis_u * intersection_2d.x + basis_v * intersection_2d.y


def _inward_normal_direction(
    tangent: Vector,
    point: Vector,
    toward_point: Vector,
    plane_normal: Vector,
) -> Vector:
    normal = plane_normal.cross(tangent)
    if normal.dot(toward_point - point) < 0.0:
        normal.negate()
    return _safe_normalized(normal, toward_point - point)


# Ratio of normal-direction extent to the dominant in-plane extent above which
# patch-plane projections lose enough information to make 2D-refinement steps
# (inverse-offset guide, cap-tangent constraint, spine-normal sections) produce
# spurious results.  Mild in-plane C/L-shapes stay below this and keep full
# refinement; serpentines folded along the face normal exceed it and fall back
# to uniform arc-length parametrization.
_BAND_OUT_OF_PLANE_EXTENT_RATIO = 0.20


def _band_is_out_of_patch_plane(
    side_a_points: tuple[Vector, ...],
    side_b_points: tuple[Vector, ...],
    basis_u: Vector,
    basis_v: Vector,
) -> bool:
    """Detect bands whose 3D geometry deviates significantly from basis_u × basis_v.

    The 2D refinements in the spine builder project rails and section lines
    onto the patch plane.  When the band bends substantially along the face
    normal (e.g. a serpentine strip folded out of the surface), the projection
    collapses the normal component and yields spurious 2D intersections,
    distorting section distances for intermediate vertices.  The caller should
    skip those refinements and rely on pure 3D arc-length parametrization.
    """

    if not side_a_points or not side_b_points:
        return False
    plane_normal = _safe_normalized(
        basis_u.cross(basis_v), Vector((0.0, 0.0, 1.0))
    )
    origin = side_a_points[0]
    n_values: list[float] = []
    u_values: list[float] = []
    v_values: list[float] = []
    for points in (side_a_points, side_b_points):
        for point in points:
            offset = point - origin
            n_values.append(offset.dot(plane_normal))
            u_values.append(offset.dot(basis_u))
            v_values.append(offset.dot(basis_v))
    if not n_values:
        return False
    n_extent = max(n_values) - min(n_values)
    inplane_extent = max(
        max(u_values) - min(u_values),
        max(v_values) - min(v_values),
        1e-8,
    )
    return n_extent > _BAND_OUT_OF_PLANE_EXTENT_RATIO * inplane_extent


def _build_inverse_offset_guide(
    side_a_points: tuple[Vector, ...],
    side_b_points: tuple[Vector, ...],
    basis_u: Vector,
    basis_v: Vector,
) -> tuple[Vector, ...]:
    if not side_a_points or not side_b_points or len(side_a_points) != len(side_b_points):
        return ()

    plane_normal = _safe_normalized(basis_u.cross(basis_v), Vector((0.0, 0.0, 1.0)))
    guide_points = []
    sample_count = len(side_a_points)
    for sample_index, (point_a, point_b) in enumerate(zip(side_a_points, side_b_points)):
        midpoint = (point_a + point_b) * 0.5
        if sample_index == 0 or sample_index == sample_count - 1:
            guide_points.append(midpoint)
            continue

        tangent_a = _sample_polyline_direction(side_a_points, sample_index)
        tangent_b = _sample_polyline_direction(side_b_points, sample_index)
        inward_a = _inward_normal_direction(tangent_a, point_a, midpoint, plane_normal)
        inward_b = _inward_normal_direction(tangent_b, point_b, midpoint, plane_normal)
        intersection = _line_intersection_in_patch_plane(
            point_a,
            inward_a,
            point_b,
            inward_b,
            basis_u,
            basis_v,
        )
        if intersection is None:
            guide_points.append(midpoint)
            continue

        width = max((point_b - point_a).length, 1e-8)
        if (intersection - midpoint).length > width:
            guide_points.append(midpoint)
            continue

        normal_alignment = abs(inward_a.dot(inward_b))
        curvature_confidence = _clamp01(1.0 - normal_alignment)
        guide_points.append(midpoint.lerp(intersection, curvature_confidence))

    return tuple(guide_points)


def _distances_to_normalized_stations(
    cumulative: tuple[float, ...],
    total_length: float,
) -> tuple[float, ...]:
    if not cumulative:
        return ()
    if total_length <= 1e-8:
        count = len(cumulative)
        if count <= 1:
            return (0.0,)
        return tuple(float(index) / float(count - 1) for index in range(count))
    return tuple(distance / total_length for distance in cumulative)


def _sample_polyline_at_stations(
    points: tuple[Vector, ...],
    cumulative: tuple[float, ...],
    total_length: float,
    stations: tuple[float, ...],
) -> tuple[tuple[Vector, ...], tuple[float, ...]]:
    if not points or not stations:
        return (), ()
    if total_length <= 1e-8:
        return (
            tuple(points[0].copy() for _ in stations),
            tuple(0.0 for _ in stations),
        )

    sampled_points = []
    sampled_distances = []
    for station_t in stations:
        distance = _clamp01(station_t) * total_length
        sampled_distances.append(distance)
        sampled_points.append(_sample_polyline_at_distance(points, cumulative, distance))
    return tuple(sampled_points), tuple(sampled_distances)


def _build_bootstrap_sections(
    side_a_points: tuple[Vector, ...],
    side_b_points: tuple[Vector, ...],
) -> tuple[
    tuple[Vector, ...],
    tuple[Vector, ...],
    tuple[float, ...],
    tuple[float, ...],
    tuple[float, ...],
]:
    if not side_a_points or not side_b_points:
        return (), (), (), (), ()

    cumulative_a, total_a = _polyline_cumulative_lengths(side_a_points)
    cumulative_b, total_b = _polyline_cumulative_lengths(side_b_points)
    section_count = max(len(side_a_points), len(side_b_points), 2)
    section_stations = tuple(
        float(section_index) / float(section_count - 1)
        for section_index in range(section_count)
    )
    side_a_sections, side_a_section_distances = _sample_polyline_at_stations(
        side_a_points,
        cumulative_a,
        total_a,
        section_stations,
    )
    side_b_sections, side_b_section_distances = _sample_polyline_at_stations(
        side_b_points,
        cumulative_b,
        total_b,
        section_stations,
    )
    if not side_a_sections or not side_b_sections:
        return (), (), (), (), ()

    return (
        side_a_sections,
        side_b_sections,
        side_a_section_distances,
        side_b_section_distances,
        section_stations,
    )


def _chain_vertex_indices(chain: BoundaryChain, reversed_points: bool) -> tuple[int, ...]:
    indices = tuple(int(vertex_index) for vertex_index in chain.vert_indices)
    return tuple(reversed(indices)) if reversed_points else indices


def _collect_chain_group_vert_indices(
    graph: PatchGraph,
    chain_refs: tuple[tuple[int, int, int], ...],
) -> set[int]:
    result: set[int] = set()
    for chain_ref in chain_refs:
        chain = graph.get_chain(*chain_ref)
        if chain is None:
            continue
        result.update(int(vertex_index) for vertex_index in chain.vert_indices)
    return result


def _build_patch_vertex_cos(node) -> dict[int, Vector]:
    mesh_vert_indices = tuple(getattr(node, "mesh_vert_indices", ()) or ())
    if len(mesh_vert_indices) == len(node.mesh_verts):
        return {
            int(vertex_index): node.mesh_verts[index].copy()
            for index, vertex_index in enumerate(mesh_vert_indices)
        }

    result: dict[int, Vector] = {}
    for boundary_loop in node.boundary_loops:
        for chain in boundary_loop.chains:
            for vertex_index, point in zip(chain.vert_indices, chain.vert_cos):
                result.setdefault(int(vertex_index), point.copy())
    return result


def _build_patch_edge_adjacency(node, vertex_cos: dict[int, Vector]) -> dict[int, list[tuple[int, float]]]:
    adjacency: dict[int, list[tuple[int, float]]] = {vertex_index: [] for vertex_index in vertex_cos}
    for a, b in tuple(getattr(node, "mesh_edges", ()) or ()):
        a = int(a)
        b = int(b)
        if a == b or a not in vertex_cos or b not in vertex_cos:
            continue
        weight = max((vertex_cos[a] - vertex_cos[b]).length, 1e-8)
        adjacency[a].append((b, weight))
        adjacency[b].append((a, weight))
    return adjacency


def _dijkstra_distances(
    adjacency: dict[int, list[tuple[int, float]]],
    seed_vertices: set[int],
) -> dict[int, float]:
    distances: dict[int, float] = {}
    heap: list[tuple[float, int]] = []
    for vertex_index in seed_vertices:
        if vertex_index not in adjacency:
            continue
        distances[vertex_index] = 0.0
        heappush(heap, (0.0, vertex_index))

    while heap:
        distance, vertex_index = heappop(heap)
        if distance > distances.get(vertex_index, math.inf) + 1e-10:
            continue
        for neighbor_index, weight in adjacency.get(vertex_index, ()):
            next_distance = distance + weight
            if next_distance + 1e-10 >= distances.get(neighbor_index, math.inf):
                continue
            distances[neighbor_index] = next_distance
            heappush(heap, (next_distance, neighbor_index))
    return distances


def _initialize_station_values(
    adjacency: dict[int, list[tuple[int, float]]],
    vertex_indices: tuple[int, ...],
    start_vertices: set[int],
    end_vertices: set[int],
) -> dict[int, float]:
    start_distances = _dijkstra_distances(adjacency, start_vertices)
    end_distances = _dijkstra_distances(adjacency, end_vertices)
    values: dict[int, float] = {}
    for vertex_index in vertex_indices:
        if vertex_index in start_vertices:
            values[vertex_index] = 0.0
            continue
        if vertex_index in end_vertices:
            values[vertex_index] = 1.0
            continue
        start_distance = start_distances.get(vertex_index)
        end_distance = end_distances.get(vertex_index)
        if start_distance is None or end_distance is None:
            values[vertex_index] = 0.5
            continue
        denominator = start_distance + end_distance
        values[vertex_index] = (
            _clamp01(start_distance / denominator)
            if denominator > 1e-8 else
            0.5
        )
    return values


def _relax_harmonic_station_values(
    adjacency: dict[int, list[tuple[int, float]]],
    values: dict[int, float],
    fixed_vertices: set[int],
    *,
    iteration_count: int = 96,
) -> dict[int, float]:
    if not values:
        return {}
    solved = dict(values)
    free_vertices = tuple(
        vertex_index
        for vertex_index in sorted(solved)
        if vertex_index not in fixed_vertices and adjacency.get(vertex_index)
    )
    if not free_vertices:
        return solved

    for _iteration in range(max(iteration_count, 1)):
        max_delta = 0.0
        for vertex_index in free_vertices:
            weighted_sum = 0.0
            weight_sum = 0.0
            for neighbor_index, edge_length in adjacency.get(vertex_index, ()):
                if neighbor_index not in solved:
                    continue
                weight = 1.0 / max(edge_length, 1e-8)
                weighted_sum += solved[neighbor_index] * weight
                weight_sum += weight
            if weight_sum <= 1e-12:
                continue
            next_value = _clamp01(weighted_sum / weight_sum)
            max_delta = max(max_delta, abs(next_value - solved[vertex_index]))
            solved[vertex_index] = next_value
        if max_delta <= 1e-6:
            break
    return solved


def _build_topology_station_map(
    graph: PatchGraph,
    node,
    cap_start_refs: tuple[tuple[int, int, int], ...],
    cap_end_refs: tuple[tuple[int, int, int], ...],
) -> dict[int, float]:
    vertex_cos = _build_patch_vertex_cos(node)
    if not vertex_cos:
        return {}

    adjacency = _build_patch_edge_adjacency(node, vertex_cos)
    if not any(adjacency.values()):
        return {}

    start_vertices = _collect_chain_group_vert_indices(graph, cap_start_refs)
    end_vertices = _collect_chain_group_vert_indices(graph, cap_end_refs)
    if not start_vertices or not end_vertices:
        return {}

    vertex_indices = tuple(sorted(vertex_cos))
    initial_values = _initialize_station_values(
        adjacency,
        vertex_indices,
        start_vertices,
        end_vertices,
    )
    return _relax_harmonic_station_values(
        adjacency,
        initial_values,
        start_vertices | end_vertices,
    )


def _make_monotonic_stations(stations: tuple[float, ...]) -> tuple[float, ...]:
    if len(stations) < 2:
        return ()
    result = []
    previous = 0.0
    for index, station in enumerate(stations):
        value = _clamp01(station)
        if index == 0:
            previous = value
        elif value < previous:
            value = previous
        result.append(value)
        previous = value
    if result[-1] - result[0] <= 1e-5:
        return ()
    return tuple(result)


def _station_sequence_for_chain(
    chain: BoundaryChain,
    reversed_points: bool,
    station_map: dict[int, float],
) -> tuple[float, ...]:
    vertex_indices = _chain_vertex_indices(chain, reversed_points)
    if len(vertex_indices) < 2:
        return ()
    stations = []
    for vertex_index in vertex_indices:
        station = station_map.get(vertex_index)
        if station is None:
            return ()
        stations.append(station)
    return _make_monotonic_stations(tuple(stations))


def _merge_section_stations(*station_sequences: tuple[float, ...]) -> tuple[float, ...]:
    values = [0.0, 1.0]
    for sequence in station_sequences:
        values.extend(_clamp01(station) for station in sequence)
    values.sort()

    merged = []
    for value in values:
        if merged and abs(value - merged[-1]) <= 1e-5:
            continue
        merged.append(value)
    if len(merged) < 2:
        return ()
    merged[0] = 0.0
    merged[-1] = 1.0
    return tuple(merged)


def _sample_side_at_station(
    points: tuple[Vector, ...],
    point_distances: tuple[float, ...],
    point_stations: tuple[float, ...],
    station: float,
) -> tuple[Vector, float]:
    if not points or len(points) != len(point_distances) or len(points) != len(point_stations):
        return Vector((0.0, 0.0, 0.0)), 0.0
    target = _clamp01(station)
    if target <= point_stations[0] + 1e-6:
        return points[0].copy(), point_distances[0]
    if target >= point_stations[-1] - 1e-6:
        return points[-1].copy(), point_distances[-1]

    for index in range(len(points) - 1):
        start_station = point_stations[index]
        end_station = point_stations[index + 1]
        if end_station < start_station:
            continue
        if target < start_station - 1e-6 or target > end_station + 1e-6:
            continue
        span = end_station - start_station
        if span <= 1e-8:
            return points[index].copy(), point_distances[index]
        local_t = _clamp01((target - start_station) / span)
        point = points[index].lerp(points[index + 1], local_t)
        distance = point_distances[index] + (point_distances[index + 1] - point_distances[index]) * local_t
        return point, distance

    nearest_index = min(
        range(len(point_stations)),
        key=lambda index: abs(point_stations[index] - target),
    )
    return points[nearest_index].copy(), point_distances[nearest_index]


def _sample_side_sections_by_station(
    points: tuple[Vector, ...],
    point_stations: tuple[float, ...],
    section_stations: tuple[float, ...],
) -> tuple[tuple[Vector, ...], tuple[float, ...]]:
    point_distances, _total_length = _polyline_cumulative_lengths(points)
    if len(point_distances) != len(points) or len(point_stations) != len(points):
        return (), ()

    section_points = []
    section_distances = []
    for station in section_stations:
        point, distance = _sample_side_at_station(
            points,
            point_distances,
            point_stations,
            station,
        )
        section_points.append(point)
        section_distances.append(distance)
    return tuple(section_points), tuple(section_distances)


def _build_topology_sections(
    graph: PatchGraph,
    node,
    side_a_ref: tuple[int, int, int],
    side_b_ref: tuple[int, int, int],
    side_a_points: tuple[Vector, ...],
    side_b_points: tuple[Vector, ...],
    side_a_reversed: bool,
    side_b_reversed: bool,
    cap_start_refs: tuple[tuple[int, int, int], ...],
    cap_end_refs: tuple[tuple[int, int, int], ...],
) -> tuple[
    tuple[Vector, ...],
    tuple[Vector, ...],
    tuple[float, ...],
    tuple[float, ...],
    tuple[float, ...],
    tuple[float, ...],
    tuple[float, ...],
]:
    side_a_chain = graph.get_chain(*side_a_ref)
    side_b_chain = graph.get_chain(*side_b_ref)
    if side_a_chain is None or side_b_chain is None:
        return (), (), (), (), (), (), ()

    station_map = _build_topology_station_map(
        graph,
        node,
        cap_start_refs,
        cap_end_refs,
    )
    if not station_map:
        return (), (), (), (), (), (), ()

    side_a_stations = _station_sequence_for_chain(side_a_chain, side_a_reversed, station_map)
    side_b_stations = _station_sequence_for_chain(side_b_chain, side_b_reversed, station_map)
    if not side_a_stations or not side_b_stations:
        return (), (), (), (), (), (), ()
    if len(side_a_stations) != len(side_a_points) or len(side_b_stations) != len(side_b_points):
        return (), (), (), (), (), (), ()

    section_stations = _merge_section_stations(side_a_stations, side_b_stations)
    if not section_stations:
        return (), (), (), (), (), (), ()

    side_a_sections, side_a_distances = _sample_side_sections_by_station(
        side_a_points,
        side_a_stations,
        section_stations,
    )
    side_b_sections, side_b_distances = _sample_side_sections_by_station(
        side_b_points,
        side_b_stations,
        section_stations,
    )
    if not side_a_sections or not side_b_sections:
        return (), (), (), (), (), (), ()
    return (
        side_a_sections,
        side_b_sections,
        side_a_distances,
        side_b_distances,
        section_stations,
        side_a_stations,
        side_b_stations,
    )


def _intersect_section_plane_with_polyline(
    polyline_points: tuple[Vector, ...],
    cumulative_distances: tuple[float, ...],
    plane_point: Vector,
    plane_normal: Vector,
    *,
    previous_distance: Optional[float],
    fallback_point: Vector,
    fallback_distance: float,
) -> tuple[Vector, float]:
    """Intersect a 3D plane with a polyline; return the forward-progress crossing.

    The plane passes through ``plane_point`` with normal ``plane_normal``.
    Each polyline segment is tested for a sign change of the signed distance
    ``(X - plane_point) · plane_normal``; a change implies the segment crosses
    the plane, and the exact crossing is linear interpolation of the signed
    values.  Among crossings, the one with the smallest forward-progress
    relative to ``previous_distance`` is returned (falling back to closest
    approach when no forward crossing exists).

    This is a 3D-native replacement for the earlier patch-plane 2D-projection
    intersection: it produces correct section distances for both in-plane and
    out-of-plane bands (serpentines folded along the face normal) and
    preserves the local rail compression/expansion at bends that drives the
    rigidity-weighted section redistribution.
    """

    if plane_normal.length <= 1e-8:
        return fallback_point.copy(), fallback_distance
    normal_dir = plane_normal.normalized()
    candidates: list[tuple[float, float, Vector]] = []
    for segment_index in range(len(polyline_points) - 1):
        segment_start = polyline_points[segment_index]
        segment_end = polyline_points[segment_index + 1]
        d_start = (segment_start - plane_point).dot(normal_dir)
        d_end = (segment_end - plane_point).dot(normal_dir)
        # Skip segments entirely on one side of the plane; a near-grazing
        # tolerance keeps duplicate hits at shared vertices from exploding.
        if d_start > 1e-8 and d_end > 1e-8:
            continue
        if d_start < -1e-8 and d_end < -1e-8:
            continue
        denominator = d_start - d_end
        if abs(denominator) <= 1e-10:
            # Segment lies within the plane; treat its start as the crossing.
            segment_t = 0.0
        else:
            segment_t = _clamp01(d_start / denominator)
        point = segment_start.lerp(segment_end, segment_t)
        segment_length = (segment_end - segment_start).length
        distance = cumulative_distances[segment_index] + segment_length * segment_t
        line_offset = (point - plane_point).length
        candidates.append((distance, line_offset, point))

    if not candidates:
        return fallback_point.copy(), fallback_distance

    if previous_distance is None:
        best_distance, _offset, best_point = min(candidates, key=lambda item: item[1])
        return best_point, best_distance

    forward_candidates = [
        candidate for candidate in candidates
        if candidate[0] + 1e-6 >= previous_distance
    ]
    if forward_candidates:
        best_distance, _offset, best_point = min(
            forward_candidates,
            key=lambda item: (item[0] - previous_distance, item[1]),
        )
        return best_point, best_distance

    best_distance, _offset, best_point = min(
        candidates,
        key=lambda item: (abs(item[0] - previous_distance), item[1]),
    )
    return best_point, best_distance


def _build_spine_normal_sections(
    side_a_points: tuple[Vector, ...],
    side_b_points: tuple[Vector, ...],
    spine_points: tuple[Vector, ...],
    fallback_side_a_sections: tuple[Vector, ...],
    fallback_side_b_sections: tuple[Vector, ...],
    fallback_side_a_distances: tuple[float, ...],
    fallback_side_b_distances: tuple[float, ...],
) -> tuple[
    tuple[Vector, ...],
    tuple[Vector, ...],
    tuple[float, ...],
    tuple[float, ...],
]:
    """Cut spine-normal sections in 3D and intersect with rail polylines.

    The cutting plane at each interior spine station is perpendicular to the
    spine 3D tangent (the plane normal IS the tangent).  Rail crossings are
    located by signed-distance sign changes along each polyline segment, so
    the sectioning is intrinsically 3D and preserves the local rail
    compression/expansion at bends -- both in-plane bends (cones, arcs in the
    patch plane) and out-of-plane bends (serpentines folded along the face
    normal).
    """

    if not side_a_points or not side_b_points or not spine_points:
        return (), (), (), ()

    cumulative_a, _total_a = _polyline_cumulative_lengths(side_a_points)
    cumulative_b, _total_b = _polyline_cumulative_lengths(side_b_points)

    side_a_sections = []
    side_b_sections = []
    side_a_section_distances = []
    side_b_section_distances = []
    previous_a_distance: Optional[float] = None
    previous_b_distance: Optional[float] = None

    for sample_index, spine_point in enumerate(spine_points):
        if sample_index < len(fallback_side_a_sections):
            fallback_a_point = fallback_side_a_sections[sample_index]
            fallback_a_distance = fallback_side_a_distances[sample_index]
        else:
            fallback_a_point = side_a_points[-1]
            fallback_a_distance = cumulative_a[-1]

        if sample_index < len(fallback_side_b_sections):
            fallback_b_point = fallback_side_b_sections[sample_index]
            fallback_b_distance = fallback_side_b_distances[sample_index]
        else:
            fallback_b_point = side_b_points[-1]
            fallback_b_distance = cumulative_b[-1]

        if sample_index == 0:
            side_a_sections.append(fallback_a_point.copy())
            side_b_sections.append(fallback_b_point.copy())
            side_a_section_distances.append(fallback_a_distance)
            side_b_section_distances.append(fallback_b_distance)
            previous_a_distance = fallback_a_distance
            previous_b_distance = fallback_b_distance
            continue

        if sample_index == len(spine_points) - 1:
            side_a_sections.append(fallback_a_point.copy())
            side_b_sections.append(fallback_b_point.copy())
            side_a_section_distances.append(fallback_a_distance)
            side_b_section_distances.append(fallback_b_distance)
            previous_a_distance = fallback_a_distance
            previous_b_distance = fallback_b_distance
            continue

        tangent = _sample_polyline_direction(spine_points, sample_index)
        # The spine 3D tangent is the section plane's normal; no 2D projection.
        plane_normal_3d = _safe_normalized(
            tangent,
            fallback_b_point - fallback_a_point,
        )

        side_a_section, side_a_distance = _intersect_section_plane_with_polyline(
            side_a_points,
            cumulative_a,
            spine_point,
            plane_normal_3d,
            previous_distance=previous_a_distance,
            fallback_point=fallback_a_point,
            fallback_distance=fallback_a_distance,
        )
        side_b_section, side_b_distance = _intersect_section_plane_with_polyline(
            side_b_points,
            cumulative_b,
            spine_point,
            plane_normal_3d,
            previous_distance=previous_b_distance,
            fallback_point=fallback_b_point,
            fallback_distance=fallback_b_distance,
        )
        side_a_sections.append(side_a_section)
        side_b_sections.append(side_b_section)
        side_a_section_distances.append(side_a_distance)
        side_b_section_distances.append(side_b_distance)
        previous_a_distance = side_a_distance
        previous_b_distance = side_b_distance

    return (
        tuple(side_a_sections),
        tuple(side_b_sections),
        tuple(side_a_section_distances),
        tuple(side_b_section_distances),
    )


def _median(values: tuple[float, ...]) -> float:
    if not values:
        return 0.0
    ordered = sorted(float(value) for value in values)
    mid = len(ordered) // 2
    if len(ordered) % 2 == 1:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def _polyline_turn_strengths(points: tuple[Vector, ...]) -> tuple[float, ...]:
    if not points:
        return ()
    strengths = [0.0] * len(points)
    for point_index in range(1, len(points) - 1):
        prev_dir = points[point_index] - points[point_index - 1]
        next_dir = points[point_index + 1] - points[point_index]
        if prev_dir.length <= 1e-8 or next_dir.length <= 1e-8:
            continue
        prev_dir.normalize()
        next_dir.normalize()
        strengths[point_index] = _clamp01((1.0 - prev_dir.dot(next_dir)) * 0.5)
    return tuple(strengths)


def _build_weighted_section_target_distances(
    side_a_section_distances: tuple[float, ...],
    side_b_section_distances: tuple[float, ...],
    spine_points: tuple[Vector, ...],
    target_total_override: Optional[float] = None,
) -> tuple[tuple[float, ...], float]:
    section_count = min(
        len(side_a_section_distances),
        len(side_b_section_distances),
        len(spine_points),
    )
    if section_count <= 0:
        return (), 0.0
    if section_count == 1:
        return (0.0,), 0.0

    spine_turns = _polyline_turn_strengths(spine_points)
    avg_lengths = []
    spine_lengths = []
    asymmetries = []
    curvatures = []
    for section_index in range(1, section_count):
        delta_a = max(
            0.0,
            side_a_section_distances[section_index] - side_a_section_distances[section_index - 1],
        )
        delta_b = max(
            0.0,
            side_b_section_distances[section_index] - side_b_section_distances[section_index - 1],
        )
        avg_lengths.append(0.5 * (delta_a + delta_b))
        spine_lengths.append(
            (spine_points[section_index] - spine_points[section_index - 1]).length
        )
        asymmetries.append(
            _clamp01(abs(delta_a - delta_b) / max(delta_a, delta_b, 1e-8))
        )
        curvatures.append(
            max(
                spine_turns[section_index - 1] if section_index - 1 < len(spine_turns) else 0.0,
                spine_turns[section_index] if section_index < len(spine_turns) else 0.0,
            )
        )

    median_avg = max(_median(tuple(avg_lengths)), 1e-8)
    rigidity_weights = []
    for avg_len, asymmetry, curvature in zip(
        avg_lengths,
        asymmetries,
        curvatures,
    ):
        len_factor = min(max(avg_len / median_avg, 0.35), 3.25)
        straight_factor = max(0.05, 1.0 - curvature)
        symmetry_factor = max(0.1, 1.0 - asymmetry)
        rigidity = (
            (len_factor ** 1.9)
            * (straight_factor ** 2.8)
            * (symmetry_factor ** 1.15)
        )
        if len_factor > 1.05 and curvature <= 0.08:
            rigidity *= 1.0 + min((len_factor - 1.0) * 0.8, 0.6)
        rigidity_weights.append(max(0.02, rigidity))

    rest_total = sum(avg_lengths)
    target_total = (
        target_total_override
        if target_total_override is not None and target_total_override > 1e-8
        else sum(spine_lengths)
    )
    inv_rigidity_sum = sum(1.0 / weight for weight in rigidity_weights)
    if rest_total <= 1e-8 or inv_rigidity_sum <= 1e-8:
        adjusted_lengths = list(avg_lengths)
    else:
        lambda_scale = (target_total - rest_total) / inv_rigidity_sum
        adjusted_lengths = [
            max(avg_len + (lambda_scale / rigidity), 1e-6)
            for avg_len, rigidity in zip(avg_lengths, rigidity_weights)
        ]
        adjusted_total = sum(adjusted_lengths)
        if adjusted_total > 1e-8:
            total_scale = target_total / adjusted_total
            adjusted_lengths = [max(length * total_scale, 1e-6) for length in adjusted_lengths]

    target_distances = [0.0]
    walked = 0.0
    for interval_length in adjusted_lengths:
        walked += max(interval_length, 0.0)
        target_distances.append(walked)

    return tuple(target_distances), walked


def _remap_distances_to_section_profile(
    point_distances: tuple[float, ...],
    section_source_distances: tuple[float, ...],
    section_target_distances: tuple[float, ...],
) -> tuple[float, ...]:
    if not point_distances:
        return ()
    if (
        len(section_source_distances) < 2
        or len(section_source_distances) != len(section_target_distances)
    ):
        return tuple(0.0 for _ in point_distances)

    mapped_distances = []
    section_index = 0
    for distance in point_distances:
        while (
            section_index < len(section_source_distances) - 2
            and distance > section_source_distances[section_index + 1]
        ):
            section_index += 1

        start_source = section_source_distances[section_index]
        end_source = section_source_distances[section_index + 1]
        start_target = section_target_distances[section_index]
        end_target = section_target_distances[section_index + 1]

        if end_source - start_source <= 1e-8:
            local_t = 0.0
        else:
            local_t = _clamp01((distance - start_source) / (end_source - start_source))
        mapped_distances.append(start_target + (end_target - start_target) * local_t)

    return tuple(mapped_distances)


def _remap_stations_to_section_profile(
    point_stations: tuple[float, ...],
    section_stations: tuple[float, ...],
    section_target_distances: tuple[float, ...],
) -> tuple[float, ...]:
    if not point_stations:
        return ()
    if len(section_stations) < 2 or len(section_stations) != len(section_target_distances):
        return tuple(0.0 for _ in point_stations)

    mapped_distances = []
    section_index = 0
    for station in point_stations:
        value = _clamp01(station)
        while (
            section_index < len(section_stations) - 2
            and value > section_stations[section_index + 1]
        ):
            section_index += 1

        start_station = section_stations[section_index]
        end_station = section_stations[section_index + 1]
        start_target = section_target_distances[section_index]
        end_target = section_target_distances[section_index + 1]

        if end_station - start_station <= 1e-8:
            local_t = 0.0
        else:
            local_t = _clamp01((value - start_station) / (end_station - start_station))
        mapped_distances.append(start_target + (end_target - start_target) * local_t)

    return tuple(mapped_distances)


def _endpoint_constraint_weight(station_t: float) -> float:
    edge = _clamp01(2.0 * abs(float(station_t) - 0.5))
    return edge * edge


def _endpoint_direction(points: tuple[Vector, ...], *, at_start: bool) -> Vector:
    if len(points) < 2:
        return Vector((0.0, 0.0, 0.0))
    if at_start:
        return points[1] - points[0]
    return points[-1] - points[-2]


def _endpoint_corner_index(chain: BoundaryChain, reversed_points: bool, *, at_start: bool) -> int:
    if at_start:
        return chain.end_corner_index if reversed_points else chain.start_corner_index
    return chain.start_corner_index if reversed_points else chain.end_corner_index


def _chain_corner_tangent(
    chain: Optional[BoundaryChain],
    corner_index: int,
) -> Vector:
    if chain is None or len(chain.vert_cos) < 2 or corner_index < 0:
        return Vector((0.0, 0.0, 0.0))
    if chain.start_corner_index == corner_index:
        return chain.vert_cos[1] - chain.vert_cos[0]
    if chain.end_corner_index == corner_index:
        return chain.vert_cos[-2] - chain.vert_cos[-1]
    return Vector((0.0, 0.0, 0.0))


def _find_group_corner_tangent(
    graph: PatchGraph,
    cap_refs: tuple[tuple[int, int, int], ...],
    corner_index: int,
) -> Vector:
    for cap_ref in cap_refs:
        cap_chain = graph.get_chain(*cap_ref)
        tangent = _chain_corner_tangent(cap_chain, corner_index)
        if tangent.length > 1e-8:
            return tangent
    return Vector((0.0, 0.0, 0.0))


def _mean_direction(
    first: Vector,
    second: Vector,
    fallback: Vector,
) -> Vector:
    if first.length <= 1e-8 and second.length <= 1e-8:
        if fallback.length <= 1e-8:
            return Vector((0.0, 0.0, 0.0))
        return fallback.normalized()
    if first.length <= 1e-8:
        return _safe_normalized(second, fallback)
    if second.length <= 1e-8:
        return _safe_normalized(first, fallback)
    aligned_second = second.copy()
    if first.dot(aligned_second) < 0.0:
        aligned_second.negate()
    return _safe_normalized(first + aligned_second, fallback)


def _corner_informed_endpoint_tangent(
    graph: PatchGraph,
    side_a_ref: tuple[int, int, int],
    side_b_ref: tuple[int, int, int],
    cap_refs: tuple[tuple[int, int, int], ...],
    side_a_points: tuple[Vector, ...],
    side_b_points: tuple[Vector, ...],
    side_a_reversed: bool,
    side_b_reversed: bool,
    plane_normal: Vector,
    raw_dir: Vector,
    *,
    at_start: bool,
) -> Vector:
    side_a_chain = graph.get_chain(*side_a_ref)
    side_b_chain = graph.get_chain(*side_b_ref)

    side_a_flow = _endpoint_direction(side_a_points, at_start=at_start)
    side_b_flow = _endpoint_direction(side_b_points, at_start=at_start)
    side_flow = _mean_direction(side_a_flow, side_b_flow, raw_dir)

    side_a_corner_index = _endpoint_corner_index(side_a_chain, side_a_reversed, at_start=at_start)
    side_b_corner_index = _endpoint_corner_index(side_b_chain, side_b_reversed, at_start=at_start)
    cap_a_tangent = _find_group_corner_tangent(graph, cap_refs, side_a_corner_index)
    cap_b_tangent = _find_group_corner_tangent(graph, cap_refs, side_b_corner_index)
    cap_axis = _mean_direction(cap_a_tangent, cap_b_tangent, Vector((0.0, 0.0, 0.0)))
    if cap_axis.length <= 1e-8:
        return side_flow

    cap_normal_tangent = plane_normal.cross(cap_axis)
    if cap_normal_tangent.dot(raw_dir) < 0.0:
        cap_normal_tangent.negate()
    cap_normal_tangent = _safe_normalized(cap_normal_tangent, side_flow)

    orthogonality_samples = []
    for side_dir in (side_a_flow, side_b_flow):
        if side_dir.length <= 1e-8:
            continue
        orthogonality_samples.append(
            _clamp01(1.0 - abs(_safe_normalized(side_dir, raw_dir).dot(cap_axis)))
        )
    orthogonality_confidence = (
        sum(orthogonality_samples) / float(len(orthogonality_samples))
        if orthogonality_samples else 0.0
    )
    cap_consistency = 1.0
    if cap_a_tangent.length > 1e-8 and cap_b_tangent.length > 1e-8:
        norm_a = _safe_normalized(cap_a_tangent, cap_axis)
        norm_b = _safe_normalized(cap_b_tangent, cap_axis)
        if norm_a.dot(norm_b) < 0.0:
            norm_b.negate()
        cap_consistency = _clamp01(norm_a.dot(norm_b))

    blend = 0.75 * orthogonality_confidence * cap_consistency
    return _safe_normalized(side_flow.lerp(cap_normal_tangent, blend), raw_dir)


def _constrain_spine_with_cap_tangents(
    graph: PatchGraph,
    side_a_ref: tuple[int, int, int],
    side_b_ref: tuple[int, int, int],
    cap_start_refs: tuple[tuple[int, int, int], ...],
    cap_end_refs: tuple[tuple[int, int, int], ...],
    raw_spine_points: tuple[Vector, ...],
    side_a_points: tuple[Vector, ...],
    side_b_points: tuple[Vector, ...],
    side_a_reversed: bool,
    side_b_reversed: bool,
    basis_u: Vector,
    basis_v: Vector,
) -> tuple[Vector, ...]:
    if len(raw_spine_points) <= 2:
        return raw_spine_points

    sample_stations = tuple(
        float(sample_index) / float(len(raw_spine_points) - 1)
        for sample_index in range(len(raw_spine_points))
    )
    plane_normal = _safe_normalized(basis_u.cross(basis_v), Vector((0.0, 0.0, 1.0)))

    raw_start_dir = raw_spine_points[1] - raw_spine_points[0]
    raw_end_dir = raw_spine_points[-1] - raw_spine_points[-2]

    start_tangent_dir = _corner_informed_endpoint_tangent(
        graph,
        side_a_ref,
        side_b_ref,
        cap_start_refs,
        side_a_points,
        side_b_points,
        side_a_reversed,
        side_b_reversed,
        plane_normal,
        raw_start_dir,
        at_start=True,
    )
    end_tangent_dir = _corner_informed_endpoint_tangent(
        graph,
        side_a_ref,
        side_b_ref,
        cap_end_refs,
        side_a_points,
        side_b_points,
        side_a_reversed,
        side_b_reversed,
        plane_normal,
        raw_end_dir,
        at_start=False,
    )

    start_step = max(raw_start_dir.length, 1e-8)
    end_step = max(raw_end_dir.length, 1e-8)
    tangent_scale = float(len(raw_spine_points) - 1)
    start_tangent = start_tangent_dir * start_step * tangent_scale
    end_tangent = end_tangent_dir * end_step * tangent_scale

    constrained_points = []
    for station_t, raw_point in zip(sample_stations, raw_spine_points):
        tangent_point = _sample_cubic_hermite(
            raw_spine_points[0],
            raw_spine_points[-1],
            start_tangent,
            end_tangent,
            station_t,
        )
        blend = 0.6 * _endpoint_constraint_weight(station_t)
        constrained_points.append(raw_point.lerp(tangent_point, blend))

    constrained_points[0] = raw_spine_points[0].copy()
    constrained_points[-1] = raw_spine_points[-1].copy()
    return tuple(constrained_points)


def _resolve_spine_axis_from_points(
    side_a_points: tuple[Vector, ...],
    side_b_points: tuple[Vector, ...],
    basis_u: Vector,
    basis_v: Vector,
) -> FrameRole:
    if not side_a_points or not side_b_points:
        return FrameRole.H_FRAME
    start_mid = (side_a_points[0] + side_b_points[0]) * 0.5
    end_mid = (side_a_points[-1] + side_b_points[-1]) * 0.5
    chord = end_mid - start_mid
    u_comp = abs(chord.dot(basis_u))
    v_comp = abs(chord.dot(basis_v))
    return FrameRole.H_FRAME if u_comp >= v_comp else FrameRole.V_FRAME


def _canonicalize_band_orientation(
    side_a_ref: tuple[int, int, int],
    side_b_ref: tuple[int, int, int],
    side_a_points: tuple[Vector, ...],
    side_b_points: tuple[Vector, ...],
    cap_start_refs: tuple[tuple[int, int, int], ...],
    cap_end_refs: tuple[tuple[int, int, int], ...],
    side_a_reversed: bool,
    side_b_reversed: bool,
    basis_u: Vector,
    basis_v: Vector,
) -> tuple[
    tuple[int, int, int],
    tuple[int, int, int],
    tuple[Vector, ...],
    tuple[Vector, ...],
    tuple[tuple[int, int, int], ...],
    tuple[tuple[int, int, int], ...],
    bool,
    bool,
]:
    spine_axis = _resolve_spine_axis_from_points(
        side_a_points,
        side_b_points,
        basis_u,
        basis_v,
    )
    along_basis = basis_u if spine_axis == FrameRole.H_FRAME else basis_v
    start_mid = (side_a_points[0] + side_b_points[0]) * 0.5
    end_mid = (side_a_points[-1] + side_b_points[-1]) * 0.5
    if (end_mid - start_mid).dot(along_basis) < 0.0:
        side_a_points = tuple(reversed(side_a_points))
        side_b_points = tuple(reversed(side_b_points))
        cap_start_refs, cap_end_refs = cap_end_refs, cap_start_refs
        side_a_reversed = not side_a_reversed
        side_b_reversed = not side_b_reversed

    cross_basis = basis_v if spine_axis == FrameRole.H_FRAME else (-basis_u)
    side_a_cross = sum(point.dot(cross_basis) for point in side_a_points) / float(max(len(side_a_points), 1))
    side_b_cross = sum(point.dot(cross_basis) for point in side_b_points) / float(max(len(side_b_points), 1))
    if side_a_cross < side_b_cross:
        side_a_ref, side_b_ref = side_b_ref, side_a_ref
        side_a_points, side_b_points = side_b_points, side_a_points
        side_a_reversed, side_b_reversed = side_b_reversed, side_a_reversed

    return (
        side_a_ref,
        side_b_ref,
        side_a_points,
        side_b_points,
        cap_start_refs,
        cap_end_refs,
        side_a_reversed,
        side_b_reversed,
    )


def _resolve_spine_axis(chain: BoundaryChain, basis_u: Vector, basis_v: Vector) -> FrameRole:
    if len(chain.vert_cos) < 2:
        return FrameRole.H_FRAME
    chord = chain.vert_cos[-1] - chain.vert_cos[0]
    u_comp = abs(chord.dot(basis_u))
    v_comp = abs(chord.dot(basis_v))
    return FrameRole.H_FRAME if u_comp >= v_comp else FrameRole.V_FRAME


def _refs_from_indices(
    patch_id: int,
    loop_index: int,
    chain_indices: tuple[int, ...],
) -> tuple[tuple[int, int, int], ...]:
    return tuple((patch_id, loop_index, chain_index) for chain_index in chain_indices)


def _orient_side_pair(
    graph: PatchGraph,
    side_a_ref: tuple[int, int, int],
    side_b_ref: tuple[int, int, int],
    cap_path_refs: tuple[tuple[tuple[int, int, int], ...], ...],
) -> Optional[
    tuple[
        tuple[int, int, int],
        tuple[int, int, int],
        tuple[Vector, ...],
        tuple[Vector, ...],
        tuple[tuple[int, int, int], ...],
        tuple[tuple[int, int, int], ...],
        bool,
        bool,
    ]
]:
    side_a = graph.get_chain(*side_a_ref)
    side_b = graph.get_chain(*side_b_ref)
    if side_a is None or side_b is None:
        return None
    if len(side_a.vert_cos) < 2 or len(side_b.vert_cos) < 2:
        return None
    if len(cap_path_refs) != 2:
        return None

    cap_start_refs = next(
        (
            group_refs
            for group_refs in cap_path_refs
            if _group_has_corner(graph, group_refs, side_a.start_corner_index)
        ),
        (),
    )
    cap_end_refs = next(
        (
            group_refs
            for group_refs in cap_path_refs
            if _group_has_corner(graph, group_refs, side_a.end_corner_index)
        ),
        (),
    )
    if not cap_start_refs or not cap_end_refs or cap_start_refs == cap_end_refs:
        return None

    side_a_points = tuple(point.copy() for point in side_a.vert_cos)
    side_b_points = tuple(point.copy() for point in side_b.vert_cos)
    side_a_reversed = False
    side_b_reversed = False

    if _group_has_corner(graph, cap_start_refs, side_b.end_corner_index):
        side_b_points = tuple(reversed(side_b_points))
        side_b_reversed = True
    elif not _group_has_corner(graph, cap_start_refs, side_b.start_corner_index):
        start_to_start = (side_a_points[0] - side_b_points[0]).length_squared
        start_to_end = (side_a_points[0] - side_b_points[-1]).length_squared
        if start_to_end < start_to_start:
            side_b_points = tuple(reversed(side_b_points))
            side_b_reversed = True

    return (
        side_a_ref,
        side_b_ref,
        side_a_points,
        side_b_points,
        cap_start_refs,
        cap_end_refs,
        side_a_reversed,
        side_b_reversed,
    )


def _parametrize_side(
    side_points: tuple[Vector, ...],
    section_source_distances: tuple[float, ...],
    section_target_distances: tuple[float, ...],
    total_length: float,
    cap_start_width: float,
    cap_end_width: float,
    side_sign: float,
) -> tuple[tuple[float, float], ...]:
    side_distances, _ = _polyline_cumulative_lengths(side_points)
    mapped_distances = _remap_distances_to_section_profile(
        side_distances,
        section_source_distances,
        section_target_distances,
    )
    uv_targets = []
    for v_distance in mapped_distances:
        v_ratio = _clamp01(v_distance / total_length) if total_length > 1e-8 else 0.0
        half_width = 0.5 * (
            cap_start_width
            + (cap_end_width - cap_start_width) * v_ratio
        )
        uv_targets.append((side_sign * half_width, v_distance))
    return tuple(uv_targets)


def _parametrize_side_by_stations(
    side_stations: tuple[float, ...],
    section_stations: tuple[float, ...],
    section_target_distances: tuple[float, ...],
    total_length: float,
    cap_start_width: float,
    cap_end_width: float,
    side_sign: float,
) -> tuple[tuple[float, float], ...]:
    mapped_distances = _remap_stations_to_section_profile(
        side_stations,
        section_stations,
        section_target_distances,
    )
    uv_targets = []
    for v_distance in mapped_distances:
        v_ratio = _clamp01(v_distance / total_length) if total_length > 1e-8 else 0.0
        half_width = 0.5 * (
            cap_start_width
            + (cap_end_width - cap_start_width) * v_ratio
        )
        uv_targets.append((side_sign * half_width, v_distance))
    return tuple(uv_targets)


def _parametrize_cap(
    cap_chain: BoundaryChain,
    side_a_endpoint: Vector,
    side_b_endpoint: Vector,
    guide_tangent: Vector,
    v_distance: float,
    cap_width: float,
) -> tuple[tuple[float, float], ...]:
    segment = side_b_endpoint - side_a_endpoint
    segment_len_sq = segment.length_squared
    tangent_dir = guide_tangent.normalized() if guide_tangent.length > 1e-8 else Vector((0.0, 0.0, 0.0))
    uv_targets = []
    for point in cap_chain.vert_cos:
        if segment_len_sq <= 1e-12:
            side_t = 0.5
            segment_point = (side_a_endpoint + side_b_endpoint) * 0.5
        else:
            side_t = _clamp01((point - side_a_endpoint).dot(segment) / segment_len_sq)
            segment_point = side_a_endpoint.lerp(side_b_endpoint, side_t)
        u_distance = (0.5 - side_t) * cap_width
        v_offset = (point - segment_point).dot(tangent_dir) if tangent_dir.length > 1e-8 else 0.0
        uv_targets.append((u_distance, v_distance + v_offset))
    return tuple(uv_targets)


def build_canonical_4chain_band_spine(
    graph: PatchGraph,
    patch_id: int,
    loop_index: int,
    side_chain_indices: tuple[int, ...],
    cap_path_groups: tuple[tuple[int, ...], ...],
) -> Optional[BandSpineData]:
    """Unified UV builder for canonical 4-chain BAND patches.

    Handles exactly 2 side chains + 2 single-chain cap groups.  Bypasses
    the topology-heavy general path (harmonic station map, Dijkstra) but
    retains curvature-aware parameterization:

    - Arc-length resampling of both side rails
    - Inverse-offset guide spine (curvature correction)
    - Rigidity-weighted section distance redistribution
    - Per-vertex remap through section profile

    For curved bands (C/L-shape) long straight segments keep their UV
    proportion while short curved segments absorb compression.  For
    uniform geometry (rings, straight strips) the corrections are
    near-zero and the result is equivalent to simple midpoint + linear.
    """

    node = graph.nodes.get(patch_id)
    if node is None or loop_index < 0 or loop_index >= len(node.boundary_loops):
        return None
    if len(side_chain_indices) != 2 or len(cap_path_groups) != 2:
        return None
    if any(len(group) != 1 for group in cap_path_groups):
        return None

    side_a_ref = (patch_id, loop_index, side_chain_indices[0])
    side_b_ref = (patch_id, loop_index, side_chain_indices[1])
    cap_0_ref = (patch_id, loop_index, cap_path_groups[0][0])
    cap_1_ref = (patch_id, loop_index, cap_path_groups[1][0])

    side_a = graph.get_chain(*side_a_ref)
    side_b = graph.get_chain(*side_b_ref)
    cap_0 = graph.get_chain(*cap_0_ref)
    cap_1 = graph.get_chain(*cap_1_ref)
    if side_a is None or side_b is None or cap_0 is None or cap_1 is None:
        return None
    if len(side_a.vert_cos) < 2 or len(side_b.vert_cos) < 2:
        return None

    # ── orient: match side_a start → cap_start, side_a end → cap_end ──
    if _chain_has_corner(cap_0, side_a.start_corner_index):
        cap_start_ref, cap_end_ref = cap_0_ref, cap_1_ref
    elif _chain_has_corner(cap_1, side_a.start_corner_index):
        cap_start_ref, cap_end_ref = cap_1_ref, cap_0_ref
    else:
        d0 = (side_a.vert_cos[0] - cap_0.vert_cos[0]).length_squared
        d1 = (side_a.vert_cos[0] - cap_1.vert_cos[0]).length_squared
        if d0 <= d1:
            cap_start_ref, cap_end_ref = cap_0_ref, cap_1_ref
        else:
            cap_start_ref, cap_end_ref = cap_1_ref, cap_0_ref

    cap_start = graph.get_chain(*cap_start_ref)
    cap_end = graph.get_chain(*cap_end_ref)
    if cap_start is None or cap_end is None:
        return None

    side_a_points = tuple(v.copy() for v in side_a.vert_cos)
    side_b_points = tuple(v.copy() for v in side_b.vert_cos)
    side_a_reversed = False
    side_b_reversed = False

    if _chain_has_corner(cap_start, side_b.end_corner_index):
        side_b_points = tuple(reversed(side_b_points))
        side_b_reversed = True
    elif not _chain_has_corner(cap_start, side_b.start_corner_index):
        d_start = (side_a_points[0] - side_b_points[0]).length_squared
        d_end = (side_a_points[0] - side_b_points[-1]).length_squared
        if d_end < d_start:
            side_b_points = tuple(reversed(side_b_points))
            side_b_reversed = True

    cap_start_refs = (cap_start_ref,)
    cap_end_refs = (cap_end_ref,)

    # ── canonicalize orientation (align with basis) ──
    (
        side_a_ref, side_b_ref,
        side_a_points, side_b_points,
        cap_start_refs, cap_end_refs,
        side_a_reversed, side_b_reversed,
    ) = _canonicalize_band_orientation(
        side_a_ref, side_b_ref,
        side_a_points, side_b_points,
        cap_start_refs, cap_end_refs,
        side_a_reversed, side_b_reversed,
        node.basis_u, node.basis_v,
    )

    # Re-fetch cap chains after canonicalization (may have been swapped).
    cap_start = graph.get_chain(*cap_start_refs[0])
    cap_end = graph.get_chain(*cap_end_refs[0])
    if cap_start is None or cap_end is None:
        return None

    # ── resample both sides to uniform arc-length stations ──
    cumul_a, total_a = _polyline_cumulative_lengths(side_a_points)
    cumul_b, total_b = _polyline_cumulative_lengths(side_b_points)
    section_count = max(len(side_a_points), len(side_b_points), 4)
    stations = tuple(
        float(i) / float(section_count - 1) for i in range(section_count)
    )
    resampled_a, section_dist_a = _sample_polyline_at_stations(
        side_a_points, cumul_a, total_a, stations,
    )
    resampled_b, section_dist_b = _sample_polyline_at_stations(
        side_b_points, cumul_b, total_b, stations,
    )
    if not resampled_a or not resampled_b:
        return None

    # ── detect cases where patch-plane projection is unreliable ──
    # Two independent triggers feed the same bypass: closed-surface topology
    # (ring, truncated cone — side chains form nearly-closed arcs at the seam)
    # and 3D-bent open bands (serpentines folded along the face normal).  In
    # both cases basis_u × basis_v projections collapse geometry that the
    # refinement steps depend on, so we keep the plain midpoint spine and
    # uniform arc-length section distances.  For closed surfaces the uniform
    # parametrization is the correct answer by construction; for 3D-bent bands
    # it unrolls the band along its 3D arc length without introducing
    # projection artefacts.
    side_a_closure = (side_a_points[0] - side_a_points[-1]).length
    side_b_closure = (side_b_points[0] - side_b_points[-1]).length
    avg_side_length = 0.5 * (total_a + total_b)
    sides_nearly_closed = (
        avg_side_length > 1e-8
        and max(side_a_closure, side_b_closure) < 0.15 * avg_side_length
    )
    out_of_patch_plane = _band_is_out_of_patch_plane(
        side_a_points, side_b_points, node.basis_u, node.basis_v,
    )
    plane_refinement_unreliable = sides_nearly_closed or out_of_patch_plane

    # ── curvature-corrected spine (inverse-offset guide) ──
    # For curved bands (C/L-shape) the spine shifts toward the concave
    # side, compensating for the length asymmetry.  For flat/uniform
    # geometry (rings, straight strips) the correction is near-zero and
    # the spine stays at the midpoint.  For bands whose plane projection
    # is unreliable the guide is skipped outright — the inward normals
    # are computed via basis_u × basis_v and collapse to noise.
    if plane_refinement_unreliable:
        spine_points = tuple(
            (resampled_a[i] + resampled_b[i]) * 0.5
            for i in range(len(resampled_a))
        )
    else:
        spine_points = _build_inverse_offset_guide(
            resampled_a, resampled_b, node.basis_u, node.basis_v,
        )
        if not spine_points:
            spine_points = tuple(
                (resampled_a[i] + resampled_b[i]) * 0.5
                for i in range(len(resampled_a))
            )

    if not plane_refinement_unreliable:
        # Cap-tangent constraint is only meaningful for open bands that lie
        # near the patch plane — it uses basis_u × basis_v projections
        # internally.  Closed surfaces and 3D-bent serpentines take the
        # straight midpoint spine; cap tangents would collapse under the
        # projection and introduce noise.
        spine_points = _constrain_spine_with_cap_tangents(
            graph,
            side_a_ref,
            side_b_ref,
            cap_start_refs,
            cap_end_refs,
            spine_points,
            side_a_points,
            side_b_points,
            side_a_reversed,
            side_b_reversed,
            node.basis_u,
            node.basis_v,
        )

    # ── spine-normal re-sectioning (3D-native) ──
    # Cut sections perpendicular to the 3D spine tangent and intersect
    # with the rail polylines directly in 3D.  At a bend the outer side
    # intercepts are further apart (long delta) while inner intercepts
    # are closer (short delta), producing non-uniform section distances
    # that drive the rigidity-weighted redistribution.  Unlike the old
    # 2D-projected variant this works for BOTH in-plane bends (cones,
    # arcs in the patch plane) AND out-of-plane bends (serpentines
    # folded along the face normal), so it runs unconditionally and
    # preserves the local rail compression/expansion in every case.
    (
        _snorm_pts_a, _snorm_pts_b,
        snorm_dist_a, snorm_dist_b,
    ) = _build_spine_normal_sections(
        side_a_points,
        side_b_points,
        spine_points,
        resampled_a,
        resampled_b,
        section_dist_a,
        section_dist_b,
    )
    if snorm_dist_a and snorm_dist_b:
        section_dist_a = snorm_dist_a
        section_dist_b = snorm_dist_b

    # ── rigidity-weighted section target distances ──
    # Long straight segments keep their UV proportion; short curved
    # segments absorb compression.  For uniform geometry the weights
    # are equal and the redistribution is a no-op.
    section_target_distances, total_spine = _build_weighted_section_target_distances(
        section_dist_a,
        section_dist_b,
        spine_points,
    )
    if not section_target_distances or total_spine <= 1e-8:
        spine_cumul, total_spine = _polyline_cumulative_lengths(spine_points)
        if total_spine <= 1e-8:
            return None
        section_target_distances = spine_cumul

    spine_cumul, _ = _polyline_cumulative_lengths(spine_points)
    spine_arc_lengths = tuple(
        d / total_spine if total_spine > 1e-8 else 0.0 for d in spine_cumul
    )

    # ── endpoint widths ──
    cap_start_width = (side_a_points[0] - side_b_points[0]).length
    cap_end_width = (side_a_points[-1] - side_b_points[-1]).length

    # ── UV targets for side chains (remap through section profile) ──
    side_a_targets = _parametrize_side(
        side_a_points, section_dist_a, section_target_distances,
        total_spine, cap_start_width, cap_end_width, side_sign=1.0,
    )
    side_b_targets = _parametrize_side(
        side_b_points, section_dist_b, section_target_distances,
        total_spine, cap_start_width, cap_end_width, side_sign=-1.0,
    )
    if side_a_reversed:
        side_a_targets = tuple(reversed(side_a_targets))
    if side_b_reversed:
        side_b_targets = tuple(reversed(side_b_targets))

    # ── UV targets for cap chains ──
    start_tangent = _endpoint_direction(spine_points, at_start=True)
    end_tangent = _endpoint_direction(spine_points, at_start=False)

    cap_start_uv = _parametrize_cap(
        cap_start, side_a_points[0], side_b_points[0],
        start_tangent, 0.0, cap_start_width,
    )
    cap_end_uv = _parametrize_cap(
        cap_end, side_a_points[-1], side_b_points[-1],
        end_tangent, total_spine, cap_end_width,
    )

    chain_uv_targets = {
        side_a_ref: side_a_targets,
        side_b_ref: side_b_targets,
        cap_start_refs[0]: cap_start_uv,
        cap_end_refs[0]: cap_end_uv,
    }

    # ── spine axis: use mid-chord for robustness (handles closed rings) ──
    mid_idx = len(spine_points) // 2
    mid_chord = spine_points[mid_idx] - spine_points[0]
    u_comp = abs(mid_chord.dot(node.basis_u))
    v_comp = abs(mid_chord.dot(node.basis_v))
    if max(u_comp, v_comp) > 1e-8:
        spine_axis = FrameRole.H_FRAME if u_comp >= v_comp else FrameRole.V_FRAME
    else:
        spine_axis = _resolve_spine_axis_from_points(
            side_a_points, side_b_points, node.basis_u, node.basis_v,
        )

    return BandSpineData(
        patch_id=patch_id,
        side_a_ref=side_a_ref,
        side_b_ref=side_b_ref,
        cap_start_ref=cap_start_refs[0],
        cap_end_ref=cap_end_refs[0],
        cap_start_refs=cap_start_refs,
        cap_end_refs=cap_end_refs,
        spine_points_3d=tuple(p.copy() for p in spine_points),
        spine_arc_lengths=spine_arc_lengths,
        spine_arc_length=total_spine,
        cap_start_width=cap_start_width,
        cap_end_width=cap_end_width,
        chain_uv_targets=MappingProxyType(dict(chain_uv_targets)),
        spine_axis=spine_axis,
    )


def build_band_spine_from_groups(
    graph: PatchGraph,
    patch_id: int,
    loop_index: int,
    side_chain_indices: tuple[int, ...],
    cap_path_groups: tuple[tuple[int, ...], ...],
) -> Optional[BandSpineData]:
    """Build BAND UV targets from section-based master-rail stations."""

    node = graph.nodes.get(patch_id)
    if node is None or loop_index < 0 or loop_index >= len(node.boundary_loops):
        return None
    if len(side_chain_indices) != 2 or len(cap_path_groups) != 2:
        return None

    side_a_ref = (patch_id, loop_index, side_chain_indices[0])
    side_b_ref = (patch_id, loop_index, side_chain_indices[1])
    cap_path_refs = tuple(
        _refs_from_indices(patch_id, loop_index, group)
        for group in cap_path_groups
        if group
    )
    if len(cap_path_refs) != 2:
        return None

    oriented = _orient_side_pair(
        graph,
        side_a_ref,
        side_b_ref,
        cap_path_refs,
    )
    if oriented is None:
        return None

    (
        side_a_ref,
        side_b_ref,
        side_a_points,
        side_b_points,
        cap_start_refs,
        cap_end_refs,
        side_a_reversed,
        side_b_reversed,
    ) = oriented
    (
        side_a_ref,
        side_b_ref,
        side_a_points,
        side_b_points,
        cap_start_refs,
        cap_end_refs,
        side_a_reversed,
        side_b_reversed,
    ) = _canonicalize_band_orientation(
        side_a_ref,
        side_b_ref,
        side_a_points,
        side_b_points,
        cap_start_refs,
        cap_end_refs,
        side_a_reversed,
        side_b_reversed,
        node.basis_u,
        node.basis_v,
    )
    topology_station_sections = False
    side_a_stations: tuple[float, ...] = ()
    side_b_stations: tuple[float, ...] = ()
    (
        bootstrap_side_a_sections,
        bootstrap_side_b_sections,
        bootstrap_side_a_distances,
        bootstrap_side_b_distances,
        section_stations,
        side_a_stations,
        side_b_stations,
    ) = _build_topology_sections(
        graph,
        node,
        side_a_ref,
        side_b_ref,
        side_a_points,
        side_b_points,
        side_a_reversed,
        side_b_reversed,
        cap_start_refs,
        cap_end_refs,
    )
    if not bootstrap_side_a_sections:
        (
            bootstrap_side_a_sections,
            bootstrap_side_b_sections,
            bootstrap_side_a_distances,
            bootstrap_side_b_distances,
            section_stations,
        ) = _build_bootstrap_sections(
            side_a_points,
            side_b_points,
        )
    else:
        topology_station_sections = True
    if not bootstrap_side_a_sections or not bootstrap_side_b_sections or not section_stations:
        return None

    out_of_patch_plane = _band_is_out_of_patch_plane(
        side_a_points, side_b_points, node.basis_u, node.basis_v,
    )
    if out_of_patch_plane:
        # 3D-bent serpentine-style bands: skip the basis_u × basis_v
        # projections used by the inverse-offset guide and cap-tangent
        # constraint (they collapse to noise for out-of-plane geometry) and
        # keep the plain midpoint spine instead.  The spine-normal
        # re-sectioning below still runs — it is 3D-native and produces
        # the correct local compression/expansion at serpentine bends.
        spine_points = tuple(
            (bootstrap_side_a_sections[i] + bootstrap_side_b_sections[i]) * 0.5
            for i in range(
                min(len(bootstrap_side_a_sections), len(bootstrap_side_b_sections))
            )
        )
    else:
        raw_spine_points = _build_inverse_offset_guide(
            bootstrap_side_a_sections,
            bootstrap_side_b_sections,
            node.basis_u,
            node.basis_v,
        )
        if not raw_spine_points:
            return None
        spine_points = _constrain_spine_with_cap_tangents(
            graph,
            side_a_ref,
            side_b_ref,
            cap_start_refs,
            cap_end_refs,
            raw_spine_points,
            side_a_points,
            side_b_points,
            side_a_reversed,
            side_b_reversed,
            node.basis_u,
            node.basis_v,
        )

    # ── spine-normal re-sectioning (3D-native) ──
    # Runs for every geometry class: in-plane bends, out-of-plane
    # serpentines, and curve-split group assemblies.  The cutting plane's
    # normal is the 3D spine tangent itself, so rail crossings are located
    # directly in 3D and reflect real local arc-distance asymmetries at
    # bends (inner rail shorter delta, outer rail longer delta).  Topology
    # stations, when present, are a stronger bootstrap than uniform
    # sampling; otherwise fall back to the resampled bootstrap sections.
    if topology_station_sections:
        side_a_sections = bootstrap_side_a_sections
        side_b_sections = bootstrap_side_b_sections
        side_a_section_distances = bootstrap_side_a_distances
        side_b_section_distances = bootstrap_side_b_distances
    else:
        (
            side_a_sections,
            side_b_sections,
            side_a_section_distances,
            side_b_section_distances,
        ) = _build_spine_normal_sections(
            side_a_points,
            side_b_points,
            spine_points,
            bootstrap_side_a_sections,
            bootstrap_side_b_sections,
            bootstrap_side_a_distances,
            bootstrap_side_b_distances,
        )
    if not side_a_sections or not side_b_sections:
        return None

    topology_target_total = None
    if topology_station_sections:
        topology_target_total = 0.5 * (
            side_a_section_distances[-1]
            + side_b_section_distances[-1]
        )

    section_target_distances, total_arc_length = _build_weighted_section_target_distances(
        side_a_section_distances,
        side_b_section_distances,
        spine_points,
        target_total_override=topology_target_total,
    )
    if not section_target_distances:
        return None
    spine_arc_lengths = _distances_to_normalized_stations(
        section_target_distances,
        total_arc_length,
    )
    if not spine_arc_lengths:
        return None
    start_tangent = _endpoint_direction(spine_points, at_start=True)
    end_tangent = _endpoint_direction(spine_points, at_start=False)

    cap_start_width = (side_a_points[0] - side_b_points[0]).length
    cap_end_width = (side_a_points[-1] - side_b_points[-1]).length

    side_a_chain = graph.get_chain(*side_a_ref)
    side_b_chain = graph.get_chain(*side_b_ref)
    if side_a_chain is None or side_b_chain is None:
        return None

    if topology_station_sections:
        side_a_targets = _parametrize_side_by_stations(
            side_a_stations,
            section_stations,
            section_target_distances,
            total_arc_length,
            cap_start_width,
            cap_end_width,
            side_sign=1.0,
        )
    else:
        side_a_targets = _parametrize_side(
            side_a_points,
            side_a_section_distances,
            section_target_distances,
            total_arc_length,
            cap_start_width,
            cap_end_width,
            side_sign=1.0,
        )
    if side_a_reversed:
        side_a_targets = tuple(reversed(side_a_targets))
    if topology_station_sections:
        side_b_targets = _parametrize_side_by_stations(
            side_b_stations,
            section_stations,
            section_target_distances,
            total_arc_length,
            cap_start_width,
            cap_end_width,
            side_sign=-1.0,
        )
    else:
        side_b_targets = _parametrize_side(
            side_b_points,
            side_b_section_distances,
            section_target_distances,
            total_arc_length,
            cap_start_width,
            cap_end_width,
            side_sign=-1.0,
        )
    if side_b_reversed:
        side_b_targets = tuple(reversed(side_b_targets))

    chain_uv_targets = {
        side_a_ref: side_a_targets,
        side_b_ref: side_b_targets,
    }

    for cap_ref in cap_start_refs:
        cap_chain = graph.get_chain(*cap_ref)
        if cap_chain is None:
            return None
        chain_uv_targets[cap_ref] = _parametrize_cap(
            cap_chain,
            side_a_points[0],
            side_b_points[0],
            start_tangent,
            0.0,
            cap_start_width,
        )

    for cap_ref in cap_end_refs:
        cap_chain = graph.get_chain(*cap_ref)
        if cap_chain is None:
            return None
        chain_uv_targets[cap_ref] = _parametrize_cap(
            cap_chain,
            side_a_points[-1],
            side_b_points[-1],
            end_tangent,
            total_arc_length,
            cap_end_width,
        )

    return BandSpineData(
        patch_id=patch_id,
        side_a_ref=side_a_ref,
        side_b_ref=side_b_ref,
        cap_start_ref=cap_start_refs[0],
        cap_end_ref=cap_end_refs[0],
        cap_start_refs=cap_start_refs,
        cap_end_refs=cap_end_refs,
        spine_points_3d=tuple(point.copy() for point in spine_points),
        spine_arc_lengths=spine_arc_lengths,
        spine_arc_length=total_arc_length,
        cap_start_width=cap_start_width,
        cap_end_width=cap_end_width,
        chain_uv_targets=MappingProxyType(dict(chain_uv_targets)),
        spine_axis=_resolve_spine_axis(side_a_chain, node.basis_u, node.basis_v),
    )
