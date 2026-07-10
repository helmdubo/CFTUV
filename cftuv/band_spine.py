from __future__ import annotations

from types import MappingProxyType
from typing import Optional

from mathutils import Vector

try:
    from .analysis_records import BandSpineData
    from .model import BoundaryChain, FrameRole, PatchGraph
    from .structural_tokens import ChainRoleClass, LoopSignature, PatchShapeClass
except ImportError:
    from analysis_records import BandSpineData
    from model import BoundaryChain, FrameRole, PatchGraph
    from structural_tokens import ChainRoleClass, LoopSignature, PatchShapeClass


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


def _polyline_normalized_stations(points: tuple[Vector, ...]) -> tuple[tuple[float, ...], float]:
    cumulative, total_length = _polyline_cumulative_lengths(points)
    if not cumulative:
        return (), 0.0
    if total_length <= 1e-8:
        count = len(points)
        if count <= 1:
            return (0.0,), 0.0
        return tuple(float(index) / float(count - 1) for index in range(count)), 0.0
    return tuple(distance / total_length for distance in cumulative), total_length


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


def _project_point_onto_segment(point: Vector, start: Vector, end: Vector) -> tuple[Vector, float]:
    segment = end - start
    seg_len_sq = segment.length_squared
    if seg_len_sq <= 1e-12:
        return start.copy(), 0.0
    t = _clamp01((point - start).dot(segment) / seg_len_sq)
    return start + segment * t, t


def _project_point_onto_polyline(
    point: Vector,
    polyline: tuple[Vector, ...],
    polyline_stations: tuple[float, ...],
    total_length: float,
) -> tuple[Vector, float]:
    if not polyline:
        return Vector((0.0, 0.0, 0.0)), 0.0
    if len(polyline) == 1:
        return polyline[0].copy(), 0.0

    best_projection = polyline[0].copy()
    best_station = 0.0
    best_distance_sq = float("inf")
    for segment_index in range(len(polyline) - 1):
        seg_start = polyline[segment_index]
        seg_end = polyline[segment_index + 1]
        projection, local_t = _project_point_onto_segment(point, seg_start, seg_end)
        dist_sq = (point - projection).length_squared
        if dist_sq >= best_distance_sq:
            continue
        best_distance_sq = dist_sq
        start_station = polyline_stations[segment_index]
        end_station = polyline_stations[segment_index + 1]
        best_projection = projection
        best_station = start_station + (end_station - start_station) * local_t

    return best_projection, best_station * total_length


def _resolve_spine_axis(chain: BoundaryChain, basis_u: Vector, basis_v: Vector) -> FrameRole:
    if len(chain.vert_cos) < 2:
        return FrameRole.H_FRAME
    chord = chain.vert_cos[-1] - chain.vert_cos[0]
    u_comp = abs(chord.dot(basis_u))
    v_comp = abs(chord.dot(basis_v))
    return FrameRole.H_FRAME if u_comp >= v_comp else FrameRole.V_FRAME


def _build_band_cap_path_groups(chain_count: int, side_pair: tuple[int, ...]) -> tuple[tuple[int, ...], ...]:
    if chain_count < 4 or len(side_pair) != 2:
        return ()
    side_a_index, side_b_index = side_pair
    if side_a_index == side_b_index:
        return ()

    forward_span = (side_b_index - side_a_index) % chain_count
    backward_span = (side_a_index - side_b_index) % chain_count
    if forward_span <= 1 or backward_span <= 1:
        return ()

    forward_group = tuple(
        (side_a_index + offset) % chain_count
        for offset in range(1, forward_span)
    )
    backward_group = tuple(
        (side_b_index + offset) % chain_count
        for offset in range(1, backward_span)
    )
    if not forward_group or not backward_group:
        return ()
    return (forward_group, backward_group)


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
        tuple[Vector, ...],
        tuple[Vector, ...],
        tuple[tuple[int, int, int], ...],
        tuple[tuple[int, int, int], ...],
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

    return side_a_points, side_b_points, cap_start_refs, cap_end_refs, side_b_reversed


def _parametrize_side(
    side_points: tuple[Vector, ...],
    spine_points: tuple[Vector, ...],
    spine_arc_lengths: tuple[float, ...],
    total_length: float,
    cap_start_width: float,
    cap_end_width: float,
    side_sign: float,
) -> tuple[tuple[float, float], ...]:
    uv_targets = []
    for point in side_points:
        _, v_distance = _project_point_onto_polyline(
            point,
            spine_points,
            spine_arc_lengths,
            total_length,
        )
        v_ratio = _clamp01(v_distance / max(total_length, 1e-8)) if total_length > 1e-8 else 0.0
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
    v_distance: float,
    cap_width: float,
) -> tuple[tuple[float, float], ...]:
    segment = side_b_endpoint - side_a_endpoint
    segment_len_sq = segment.length_squared
    uv_targets = []
    for point in cap_chain.vert_cos:
        if segment_len_sq <= 1e-12:
            side_t = 0.5
        else:
            side_t = _clamp01((point - side_a_endpoint).dot(segment) / segment_len_sq)
        u_distance = (0.5 - side_t) * cap_width
        uv_targets.append((u_distance, v_distance))
    return tuple(uv_targets)


def build_band_spine_from_groups(
    graph: PatchGraph,
    patch_id: int,
    loop_index: int,
    side_chain_indices: tuple[int, ...],
    cap_path_groups: tuple[tuple[int, ...], ...],
) -> Optional[BandSpineData]:
    """Build midpoint-spine UV targets for BAND loops with split CAP paths."""

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

    side_a_points, side_b_points, cap_start_refs, cap_end_refs, side_b_reversed = oriented
    sample_count = max(len(side_a_points), len(side_b_points), 2)
    resampled_side_a = _resample_polyline(side_a_points, sample_count)
    resampled_side_b = _resample_polyline(side_b_points, sample_count)
    if not resampled_side_a or not resampled_side_b:
        return None

    spine_points = tuple(
        (point_a + point_b) * 0.5
        for point_a, point_b in zip(resampled_side_a, resampled_side_b)
    )
    spine_arc_lengths, total_arc_length = _polyline_normalized_stations(spine_points)
    if not spine_arc_lengths:
        return None

    cap_start_width = (side_a_points[0] - side_b_points[0]).length
    cap_end_width = (side_a_points[-1] - side_b_points[-1]).length

    side_a_chain = graph.get_chain(*side_a_ref)
    side_b_chain = graph.get_chain(*side_b_ref)
    if side_a_chain is None or side_b_chain is None:
        return None

    side_a_targets = _parametrize_side(
        side_a_points,
        spine_points,
        spine_arc_lengths,
        total_arc_length,
        cap_start_width,
        cap_end_width,
        side_sign=1.0,
    )
    side_b_targets = _parametrize_side(
        side_b_points,
        spine_points,
        spine_arc_lengths,
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


def build_band_spine_data(
    graph: PatchGraph,
    patch_id: int,
    loop_signature: LoopSignature,
    shape_class: PatchShapeClass,
) -> Optional[BandSpineData]:
    """Build midpoint-spine UV targets for structurally classified 4-chain BAND loops."""

    if shape_class != PatchShapeClass.BAND:
        return None

    side_chain_indices = tuple(
        token.chain_ref[2]
        for token in loop_signature.chain_tokens
        if token.role_class == ChainRoleClass.SIDE
    )
    if len(side_chain_indices) != 2:
        return None

    cap_path_groups = _build_band_cap_path_groups(
        loop_signature.chain_count,
        side_chain_indices,
    )
    if len(cap_path_groups) != 2:
        return None

    return build_band_spine_from_groups(
        graph,
        patch_id,
        loop_signature.loop_index,
        side_chain_indices,
        cap_path_groups,
    )
