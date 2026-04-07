from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Optional

from mathutils import Vector

try:
    from .model import (
        AnchorAdjustment,
        AxisAuthorityKind,
        BoundaryChain,
        ChainNeighborKind,
        ChainRef,
        FrameRole,
        ParameterAuthorityKind,
        PatchGraph,
        PatchNode,
        PlacementSourceKind,
        ScaffoldChainPlacement,
        ScaffoldPointKey,
        SpanAuthorityKind,
        SourcePoint,
    )
    from .solve_records import (
        ChainAnchor,
        frame_row_class_key,
        frame_column_class_key,
        PointRegistry,
        SeedPlacementResult,
        _point_registry_key,
    )
except ImportError:
    from model import (
        AnchorAdjustment,
        AxisAuthorityKind,
        BoundaryChain,
        ChainNeighborKind,
        ChainRef,
        FrameRole,
        ParameterAuthorityKind,
        PatchGraph,
        PatchNode,
        PlacementSourceKind,
        ScaffoldChainPlacement,
        ScaffoldPointKey,
        SpanAuthorityKind,
        SourcePoint,
    )
    from solve_records import (
        ChainAnchor,
        frame_row_class_key,
        frame_column_class_key,
        PointRegistry,
        SeedPlacementResult,
        _point_registry_key,
    )


@dataclass(frozen=True)
class _ResolvedFrameSegment:
    start_uv: Vector
    end_uv: Vector
    axis_direction: Vector
    target_span: float


def _build_temporary_chain_placement(
    chain_ref: ChainRef,
    chain: BoundaryChain,
    uv_points: list[Vector],
    start_anchor: Optional[ChainAnchor],
    end_anchor: Optional[ChainAnchor],
    effective_role: Optional[FrameRole] = None,
    axis_authority_kind: AxisAuthorityKind = AxisAuthorityKind.NONE,
    span_authority_kind: SpanAuthorityKind = SpanAuthorityKind.NONE,
    parameter_authority_kind: ParameterAuthorityKind = ParameterAuthorityKind.NONE,
) -> ScaffoldChainPlacement:
    patch_id, loop_index, chain_index = chain_ref
    if start_anchor is not None:
        _pak = start_anchor.source_kind
    elif end_anchor is not None:
        _pak = end_anchor.source_kind
    else:
        _pak = PlacementSourceKind.CHAIN
    return ScaffoldChainPlacement(
        patch_id=patch_id,
        loop_index=loop_index,
        chain_index=chain_index,
        frame_role=effective_role if effective_role is not None else chain.frame_role,
        axis_authority_kind=axis_authority_kind,
        span_authority_kind=span_authority_kind,
        parameter_authority_kind=parameter_authority_kind,
        source_kind=PlacementSourceKind.CHAIN,
        anchor_count=_cf_anchor_count(start_anchor, end_anchor),
        primary_anchor_kind=_pak,
        points=tuple(
            (ScaffoldPointKey(patch_id, loop_index, chain_index, point_index), uv.copy())
            for point_index, uv in enumerate(uv_points)
        ),
    )


def _cf_build_seed_placement(
    seed_ref: ChainRef,
    seed_chain: BoundaryChain,
    root_node: PatchNode,
    final_scale: float,
    effective_role: Optional[FrameRole] = None,
    axis_authority_kind: AxisAuthorityKind = AxisAuthorityKind.NONE,
    span_authority_kind: SpanAuthorityKind = SpanAuthorityKind.NONE,
    parameter_authority_kind: ParameterAuthorityKind = ParameterAuthorityKind.NONE,
) -> Optional[SeedPlacementResult]:
    seed_src = _cf_chain_source_points(seed_chain)
    role = effective_role if effective_role is not None else seed_chain.frame_role
    seed_dir = _cf_determine_direction_for_role(seed_chain, root_node, role)

    if role in (FrameRole.H_FRAME, FrameRole.V_FRAME):
        seed_uvs = _build_frame_chain_from_one_end(
            seed_src,
            Vector((0.0, 0.0)),
            seed_dir,
            role,
            final_scale,
        )
    else:
        seed_uvs = _build_guided_free_chain_from_one_end(
            root_node,
            seed_src,
            Vector((0.0, 0.0)),
            seed_dir,
            final_scale,
        )

    if not seed_uvs:
        return None

    seed_placement = ScaffoldChainPlacement(
        patch_id=seed_ref[0],
        loop_index=seed_ref[1],
        chain_index=seed_ref[2],
        frame_role=role,
        axis_authority_kind=axis_authority_kind,
        span_authority_kind=span_authority_kind,
        parameter_authority_kind=parameter_authority_kind,
        source_kind=PlacementSourceKind.CHAIN,
        anchor_count=0,
        points=tuple(
            (ScaffoldPointKey(seed_ref[0], seed_ref[1], seed_ref[2], i), uv.copy())
            for i, uv in enumerate(seed_uvs)
        ),
    )
    return SeedPlacementResult(
        placement=seed_placement,
        uv_points=seed_uvs,
    )


def _cf_chain_source_points(chain):
    """ÐšÐ¾Ð½Ð²ÐµÑ€Ñ‚Ð¸Ñ€ÑƒÐµÑ‚ chain.vert_cos Ð² Ñ„Ð¾Ñ€Ð¼Ð°Ñ‚ [(index, Vector)] Ð´Ð»Ñ placement Ñ„ÑƒÐ½ÐºÑ†Ð¸Ð¹."""
    return [(i, co.copy()) for i, co in enumerate(chain.vert_cos)]


def _cf_anchor_count(start_anchor: Optional[ChainAnchor], end_anchor: Optional[ChainAnchor]) -> int:
    return (1 if start_anchor is not None else 0) + (1 if end_anchor is not None else 0)


def _cf_anchor_debug_label(start_anchor: Optional[ChainAnchor], end_anchor: Optional[ChainAnchor]) -> str:
    def _label(anchor: Optional[ChainAnchor]) -> str:
        if anchor is None:
            return '-'
        prefix = 'S' if anchor.source_kind == PlacementSourceKind.SAME_PATCH else 'X'
        return f"{prefix}P{anchor.source_ref[0]}"

    return f"{_label(start_anchor)}/{_label(end_anchor)}"


def _cf_chain_total_length(chain: BoundaryChain, final_scale: float) -> float:
    if len(chain.vert_cos) < 2:
        return 0.0

    return sum(
        (chain.vert_cos[i + 1] - chain.vert_cos[i]).length
        for i in range(len(chain.vert_cos) - 1)
    ) * final_scale


def _snap_direction_to_role(direction: Vector, role: FrameRole) -> Vector:
    if role == FrameRole.H_FRAME:
        axis_value = direction.x if abs(direction.x) >= abs(direction.y) else direction.y
        return Vector((1.0 if axis_value >= 0.0 else -1.0, 0.0))
    if role == FrameRole.V_FRAME:
        axis_value = direction.y if abs(direction.y) >= abs(direction.x) else direction.x
        return Vector((0.0, 1.0 if axis_value >= 0.0 else -1.0))
    return Vector((0.0, 0.0))


def _rotate_direction(direction: Vector, angle_deg: float) -> Vector:
    angle_rad = math.radians(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return Vector((
        direction.x * cos_a - direction.y * sin_a,
        direction.x * sin_a + direction.y * cos_a,
    ))


def _chain_edge_lengths(ordered_source_points: list[SourcePoint], final_scale: float) -> list[float]:
    edge_lengths = []
    for point_index in range(1, len(ordered_source_points)):
        prev_point = ordered_source_points[point_index - 1][1]
        next_point = ordered_source_points[point_index][1]
        edge_lengths.append((next_point - prev_point).length * final_scale)
    return edge_lengths


def _default_role_direction(role: FrameRole) -> Vector:
    if role == FrameRole.V_FRAME:
        return Vector((0.0, 1.0))
    return Vector((1.0, 0.0))


def _normalize_direction(direction: Vector, fallback: Optional[Vector] = None) -> Vector:
    if direction.length > 1e-8:
        return direction.normalized()
    if fallback is not None and fallback.length > 1e-8:
        return fallback.normalized()
    return Vector((1.0, 0.0))


def _segment_source_step_directions(node: PatchNode, ordered_source_points: list[SourcePoint]) -> list[Vector]:
    directions = []
    fallback = Vector((1.0, 0.0))
    for point_index in range(1, len(ordered_source_points)):
        prev_point = ordered_source_points[point_index - 1][1]
        next_point = ordered_source_points[point_index][1]
        delta = next_point - prev_point
        delta_2d = Vector((delta.dot(node.basis_u), delta.dot(node.basis_v)))
        if delta_2d.length <= 1e-8:
            directions.append(fallback.copy())
            continue
        fallback = delta_2d.normalized()
        directions.append(fallback.copy())
    return directions


def _interpolate_between_anchors_by_lengths(start_point: Vector, end_point: Vector, edge_lengths: list[float]) -> list[Vector]:
    if not edge_lengths:
        return [start_point.copy()]

    total_length = sum(edge_lengths)
    if total_length <= 1e-8:
        return [
            start_point.lerp(end_point, float(point_index) / float(len(edge_lengths)))
            for point_index in range(len(edge_lengths) + 1)
        ]

    points = [start_point.copy()]
    walked = 0.0
    for edge_length in edge_lengths:
        walked += max(edge_length, 0.0)
        factor = walked / total_length
        points.append(start_point.lerp(end_point, factor))
    points[-1] = end_point.copy()
    return points


def _sample_cubic_bezier_point(p0: Vector, p1: Vector, p2: Vector, p3: Vector, t: float) -> Vector:
    omt = 1.0 - t
    return (
        p0 * (omt * omt * omt)
        + p1 * (3.0 * omt * omt * t)
        + p2 * (3.0 * omt * t * t)
        + p3 * (t * t * t)
    )


def _sample_cubic_bezier_polyline(
    p0: Vector,
    p1: Vector,
    p2: Vector,
    p3: Vector,
    sample_count: int,
) -> list[Vector]:
    sample_count = max(sample_count, 2)
    return [
        _sample_cubic_bezier_point(p0, p1, p2, p3, float(index) / float(sample_count - 1))
        for index in range(sample_count)
    ]


def _resample_polyline_by_edge_lengths(polyline_points: list[Vector], edge_lengths: list[float]) -> Optional[list[Vector]]:
    if not polyline_points:
        return None
    if not edge_lengths:
        return [polyline_points[0].copy()]

    distances = [0.0]
    for index in range(1, len(polyline_points)):
        distances.append(distances[-1] + (polyline_points[index] - polyline_points[index - 1]).length)
    total_polyline_length = distances[-1]
    if total_polyline_length <= 1e-8:
        return None

    target_distances = [0.0]
    total_target_length = 0.0
    for edge_length in edge_lengths:
        total_target_length += max(edge_length, 0.0)
        target_distances.append(total_target_length)
    if total_polyline_length + 1e-5 < total_target_length:
        return None

    resampled = []
    source_index = 1
    for target_distance in target_distances:
        while source_index < len(distances) and distances[source_index] < target_distance:
            source_index += 1
        if source_index >= len(distances):
            resampled.append(polyline_points[-1].copy())
            continue
        prev_distance = distances[source_index - 1]
        next_distance = distances[source_index]
        if next_distance - prev_distance <= 1e-8:
            resampled.append(polyline_points[source_index].copy())
            continue
        factor = (target_distance - prev_distance) / (next_distance - prev_distance)
        resampled.append(polyline_points[source_index - 1].lerp(polyline_points[source_index], factor))

    if resampled:
        resampled[0] = polyline_points[0].copy()
        resampled[-1] = polyline_points[-1].copy()
    return resampled


def _build_guided_free_chain_from_one_end(
    node: PatchNode,
    ordered_source_points: list[SourcePoint],
    start_point: Vector,
    start_direction: Vector,
    final_scale: float,
) -> list[Vector]:
    if not ordered_source_points:
        return []
    if len(ordered_source_points) == 1:
        return [start_point.copy()]

    edge_lengths = _chain_edge_lengths(ordered_source_points, final_scale)
    source_directions = _segment_source_step_directions(node, ordered_source_points)
    aligned_start = _normalize_direction(start_direction, source_directions[0] if source_directions else None)

    rotated_directions = []
    if source_directions:
        base_direction = _normalize_direction(source_directions[0], aligned_start)
        rotate_deg = math.degrees(
            math.atan2(aligned_start.y, aligned_start.x) - math.atan2(base_direction.y, base_direction.x)
        )
        rotated_directions = [
            _normalize_direction(_rotate_direction(direction, rotate_deg), aligned_start)
            for direction in source_directions
        ]

    points = [start_point.copy()]
    current_point = start_point.copy()
    current_direction = aligned_start
    for edge_index, edge_length in enumerate(edge_lengths):
        if edge_index < len(rotated_directions):
            current_direction = _normalize_direction(rotated_directions[edge_index], current_direction)
        current_point = current_point + current_direction * edge_length
        points.append(current_point.copy())
    return points


def _build_guided_free_chain_between_anchors(
    node: PatchNode,
    ordered_source_points: list[SourcePoint],
    start_point: Vector,
    end_point: Vector,
    start_direction: Vector,
    end_direction: Optional[Vector],
    final_scale: float,
) -> list[Vector]:
    if not ordered_source_points:
        return []
    if len(ordered_source_points) == 1:
        return [start_point.copy()]

    edge_lengths = _chain_edge_lengths(ordered_source_points, final_scale)
    target_delta = end_point - start_point
    chord_length = target_delta.length
    if chord_length <= 1e-8:
        return _interpolate_between_anchors_by_lengths(start_point, end_point, edge_lengths)

    source_origin = ordered_source_points[0][1]
    source_points_2d = [
        Vector((
            (point - source_origin).dot(node.basis_u) * final_scale,
            (point - source_origin).dot(node.basis_v) * final_scale,
        ))
        for _, point in ordered_source_points
    ]
    source_chord = source_points_2d[-1] - source_points_2d[0]
    source_chord_length = source_chord.length
    if source_chord_length <= 1e-8:
        return _interpolate_between_anchors_by_lengths(start_point, end_point, edge_lengths)

    source_dir = source_chord / source_chord_length
    source_perp = Vector((-source_dir.y, source_dir.x))
    target_dir = target_delta / chord_length
    base_target_perp = Vector((-target_dir.y, target_dir.x))
    scale = chord_length / source_chord_length

    source_directions = _segment_source_step_directions(node, ordered_source_points)
    start_ref = _normalize_direction(start_direction, source_directions[0] if source_directions else target_delta)
    end_fallback = source_directions[-1] if source_directions else target_delta
    end_ref = _normalize_direction(end_direction if end_direction is not None else end_fallback, end_fallback)

    def _map_profile(target_perp: Vector) -> list[Vector]:
        mapped = []
        for source_point in source_points_2d:
            relative = source_point - source_points_2d[0]
            along = relative.dot(source_dir) * scale
            across = relative.dot(source_perp) * scale
            mapped.append(start_point + target_dir * along + target_perp * across)
        mapped[0] = start_point.copy()
        mapped[-1] = end_point.copy()
        return mapped

    def _profile_score(mapped_points: list[Vector]) -> float:
        if len(mapped_points) < 2:
            return -1e9
        start_vec = _normalize_direction(mapped_points[1] - mapped_points[0], target_dir)
        end_vec = _normalize_direction(mapped_points[-1] - mapped_points[-2], target_dir)
        score = start_vec.dot(start_ref) + end_vec.dot(end_ref)

        total_length = 0.0
        for point_index in range(len(mapped_points) - 1):
            total_length += (mapped_points[point_index + 1] - mapped_points[point_index]).length
        target_total = sum(edge_lengths)
        if target_total > 1e-8:
            score -= abs((total_length / target_total) - 1.0)
        return score

    candidates = [
        _map_profile(base_target_perp),
        _map_profile(-base_target_perp),
    ]
    best_profile = max(candidates, key=_profile_score)
    return best_profile


def _frontier_chain_cross_uv(placement: ScaffoldChainPlacement) -> float:
    """Mean cross-axis UV for a placed chain. H_FRAME->avg Y, V_FRAME->avg X."""
    if not placement.points:
        return 0.0
    if placement.frame_role == FrameRole.H_FRAME:
        return sum(uv.y for _, uv in placement.points) / float(len(placement.points))
    if placement.frame_role == FrameRole.V_FRAME:
        return sum(uv.x for _, uv in placement.points) / float(len(placement.points))
    return 0.0


def _frontier_chain_uv_length(chain: BoundaryChain, final_scale: float) -> float:
    """Total UV-space edge length for a chain (scaled 3D edge lengths)."""
    pts = chain.vert_cos
    if len(pts) < 2:
        return 0.0
    total = 0.0
    for i in range(len(pts) - 1):
        total += (pts[i + 1] - pts[i]).length * final_scale
    return total


def _frame_group_cross_axis_consensus(
    chain: BoundaryChain,
    node: PatchNode,
    role: FrameRole,
    placed_chains_map: dict,
    graph: PatchGraph,
    final_scale: float,
) -> Optional[float]:
    """Weighted average cross-axis UV of trusted already-placed chains in the same frame group.

    Returns None if no trusted same-group chains are placed yet or chain is not eligible.
    Only applies to WALL.SIDE patches with H_FRAME or V_FRAME role.

    Trusted = dual-anchor (anchor_count >= 2) or non-CROSS_PATCH primary anchor.
    Weight = chain UV length (scaled 3D edge sum).
    """
    if node.semantic_key != 'WALL.SIDE':
        return None
    if role == FrameRole.H_FRAME:
        target_key = frame_row_class_key(chain)
    elif role == FrameRole.V_FRAME:
        target_key = frame_column_class_key(chain)
    else:
        return None

    total_weight = 0.0
    weighted_sum = 0.0

    for ref, placement in placed_chains_map.items():
        if placement.frame_role != role:
            continue
        if not placement.points:
            continue
        # Trusted filter: dual-anchor or non-XP primary anchor
        if placement.anchor_count < 2 and placement.primary_anchor_kind == PlacementSourceKind.CROSS_PATCH:
            continue
        other_patch_id = ref[0]
        other_node = graph.nodes.get(other_patch_id)
        if other_node is None or other_node.semantic_key != 'WALL.SIDE':
            continue
        other_loop_index = ref[1]
        other_chain_index = ref[2]
        if other_loop_index < 0 or other_loop_index >= len(other_node.boundary_loops):
            continue
        other_loop = other_node.boundary_loops[other_loop_index]
        if other_chain_index < 0 or other_chain_index >= len(other_loop.chains):
            continue
        other_chain = other_loop.chains[other_chain_index]
        if len(other_chain.vert_cos) < 2:
            continue

        if role == FrameRole.H_FRAME:
            other_key = frame_row_class_key(other_chain)
        else:
            other_key = frame_column_class_key(other_chain)

        if other_key != target_key:
            continue

        w = max(_frontier_chain_uv_length(other_chain, final_scale), 1e-8)
        weighted_sum += _frontier_chain_cross_uv(placement) * w
        total_weight += w

    if total_weight <= 1e-8:
        return None
    return weighted_sum / total_weight


def _frame_axis_value(uv: Vector, role: FrameRole) -> float:
    return uv.x if role == FrameRole.H_FRAME else uv.y


def _frame_cross_value(uv: Vector, role: FrameRole) -> float:
    return uv.y if role == FrameRole.H_FRAME else uv.x


def _frame_uv_from_axis_cross(role: FrameRole, axis_value: float, cross_value: float) -> Vector:
    if role == FrameRole.H_FRAME:
        return Vector((axis_value, cross_value))
    return Vector((cross_value, axis_value))


def _placement_axis_span(placement: ScaffoldChainPlacement) -> float:
    if len(placement.points) < 2:
        return 0.0
    start_uv = placement.points[0][1]
    end_uv = placement.points[-1][1]
    if placement.frame_role == FrameRole.H_FRAME:
        return abs(end_uv.x - start_uv.x)
    if placement.frame_role == FrameRole.V_FRAME:
        return abs(end_uv.y - start_uv.y)
    return (end_uv - start_uv).length


def _frame_anchor_source_score(
    anchor: ChainAnchor,
    graph: PatchGraph,
    placed_chains_map: dict[ChainRef, ScaffoldChainPlacement],
) -> int:
    source_chain = graph.get_chain(*anchor.source_ref)
    if source_chain is None:
        return 0
    placed_source = placed_chains_map.get(anchor.source_ref)
    source_role = placed_source.frame_role if placed_source is not None else source_chain.frame_role
    score = 0
    if anchor.source_kind == PlacementSourceKind.CROSS_PATCH:
        score += 4
    elif anchor.source_kind == PlacementSourceKind.SAME_PATCH:
        score += 2
    if source_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        score += 2
    if len(source_chain.vert_cos) <= 2:
        score += 1
    return score


def _resolve_frame_segment(
    chain_ref: Optional[ChainRef],
    chain: BoundaryChain,
    node: PatchNode,
    start_anchor: Optional[ChainAnchor],
    end_anchor: Optional[ChainAnchor],
    final_scale: float,
    direction: Vector,
    role: FrameRole,
    *,
    placed_chains_map: Optional[dict[ChainRef, ScaffoldChainPlacement]] = None,
    graph: Optional[PatchGraph] = None,
    runtime_policy=None,
) -> Optional[_ResolvedFrameSegment]:
    if role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        return None

    axis_direction = _snap_direction_to_role(direction, role)
    axis_direction = _normalize_direction(axis_direction, _default_role_direction(role))

    start_uv = start_anchor.uv.copy() if start_anchor is not None else None
    end_uv = end_anchor.uv.copy() if end_anchor is not None else None

    if (
        placed_chains_map is not None and graph is not None
        and start_anchor is not None and end_anchor is None
        and start_anchor.source_kind == PlacementSourceKind.CROSS_PATCH
    ):
        consensus = _frame_group_cross_axis_consensus(
            chain, node, role, placed_chains_map, graph, final_scale,
        )
        if consensus is not None and start_uv is not None:
            start_uv = _frame_uv_from_axis_cross(role, _frame_axis_value(start_uv, role), consensus)
    if (
        placed_chains_map is not None and graph is not None
        and end_anchor is not None and start_anchor is None
        and end_anchor.source_kind == PlacementSourceKind.CROSS_PATCH
    ):
        consensus = _frame_group_cross_axis_consensus(
            chain, node, role, placed_chains_map, graph, final_scale,
        )
        if consensus is not None and end_uv is not None:
            end_uv = _frame_uv_from_axis_cross(role, _frame_axis_value(end_uv, role), consensus)

    target_span = _cf_chain_total_length(chain, final_scale)
    if runtime_policy is not None and chain_ref is not None:
        target_span = runtime_policy.resolve_target_span(
            chain_ref,
            chain,
            start_anchor,
            end_anchor,
            effective_role=role,
        )
    if target_span <= 1e-8:
        target_span = _cf_chain_total_length(chain, final_scale)
    if target_span <= 1e-8:
        return None

    if start_uv is not None and end_uv is not None:
        fixed_anchor = start_anchor
        adjustable_anchor = end_anchor
        fixed_uv = start_uv.copy()
        adjustable_uv = end_uv.copy()
        if start_anchor is not None and end_anchor is not None and graph is not None and placed_chains_map is not None:
            start_score = _frame_anchor_source_score(start_anchor, graph, placed_chains_map)
            end_score = _frame_anchor_source_score(end_anchor, graph, placed_chains_map)
            if end_score > start_score:
                fixed_anchor = end_anchor
                adjustable_anchor = start_anchor
                fixed_uv = end_uv.copy()
                adjustable_uv = start_uv.copy()

        cross_value = _frame_cross_value(fixed_uv, role)
        fixed_axis = _frame_axis_value(fixed_uv, role)
        adjustable_axis = _frame_axis_value(adjustable_uv, role)
        axis_sign = 1.0 if (
            (axis_direction.x if role == FrameRole.H_FRAME else axis_direction.y) >= 0.0
        ) else -1.0
        if abs(adjustable_axis - fixed_axis) > 1e-8:
            axis_sign = 1.0 if adjustable_axis >= fixed_axis else -1.0
        resolved_axis = adjustable_axis
        if (
            start_anchor is not None and end_anchor is not None
            and start_anchor.source_kind == PlacementSourceKind.SAME_PATCH
            and end_anchor.source_kind == PlacementSourceKind.SAME_PATCH
        ):
            resolved_axis = fixed_axis + axis_sign * target_span
        resolved_adjustable_uv = _frame_uv_from_axis_cross(role, resolved_axis, cross_value)

        if fixed_anchor is start_anchor:
            return _ResolvedFrameSegment(
                start_uv=fixed_uv.copy(),
                end_uv=resolved_adjustable_uv,
                axis_direction=axis_direction,
                target_span=target_span,
            )
        return _ResolvedFrameSegment(
            start_uv=resolved_adjustable_uv,
            end_uv=fixed_uv.copy(),
            axis_direction=axis_direction,
            target_span=target_span,
        )

    if start_uv is not None:
        end_uv = start_uv + axis_direction * target_span
        return _ResolvedFrameSegment(
            start_uv=start_uv.copy(),
            end_uv=end_uv,
            axis_direction=axis_direction,
            target_span=target_span,
        )

    if end_uv is not None:
        start_uv = end_uv - axis_direction * target_span
        return _ResolvedFrameSegment(
            start_uv=start_uv,
            end_uv=end_uv.copy(),
            axis_direction=axis_direction,
            target_span=target_span,
        )

    return None


def _build_frame_chain_from_one_end(
    ordered_source_points: list[SourcePoint],
    start_point: Vector,
    start_direction: Vector,
    role: FrameRole,
    final_scale: float,
) -> list[Vector]:
    if not ordered_source_points:
        return []
    if len(ordered_source_points) == 1:
        return [start_point.copy()]

    axis_direction = _snap_direction_to_role(start_direction, role)
    axis_direction = _normalize_direction(axis_direction, _default_role_direction(role))
    edge_lengths = _chain_edge_lengths(ordered_source_points, final_scale)

    points = [start_point.copy()]
    current_point = start_point.copy()
    for edge_length in edge_lengths:
        current_point = current_point + axis_direction * edge_length
        points.append(current_point.copy())
    return points


def _build_frame_chain_between_anchors(
    ordered_source_points: list[SourcePoint],
    start_point: Vector,
    end_point: Vector,
    role: FrameRole,
    final_scale: float,
) -> list[Vector]:
    """Build straight H/V chain in inherited scaffold frame using local chain stationing."""
    if not ordered_source_points:
        return []
    if len(ordered_source_points) == 1:
        return [start_point.copy()]

    snapped_end = end_point.copy()
    if role == FrameRole.H_FRAME:
        snapped_end = Vector((end_point.x, start_point.y))
    elif role == FrameRole.V_FRAME:
        snapped_end = Vector((start_point.x, end_point.y))

    edge_lengths = _chain_edge_lengths(ordered_source_points, final_scale)
    return _interpolate_between_anchors_by_lengths(start_point, snapped_end, edge_lengths)


def _cf_rebuild_chain_points_for_endpoints(
    chain: BoundaryChain,
    start_uv: Vector,
    end_uv: Vector,
    final_scale: float,
    effective_role: Optional[FrameRole] = None,
) -> Optional[list[Vector]]:
    source_pts = _cf_chain_source_points(chain)
    if len(source_pts) != len(chain.vert_indices):
        return None

    role = effective_role if effective_role is not None else chain.frame_role

    if role in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        return _build_frame_chain_between_anchors(source_pts, start_uv, end_uv, role, final_scale)

    if role == FrameRole.FREE and len(source_pts) <= 2:
        if len(source_pts) == 0:
            return []
        if len(source_pts) == 1:
            return [start_uv.copy()]
        return [start_uv.copy(), end_uv.copy()]

    return None


def _cf_apply_anchor_adjustments(
    adjustments: tuple[AnchorAdjustment, ...],
    graph: PatchGraph,
    placed_chains_map: dict[ChainRef, ScaffoldChainPlacement],
    point_registry: PointRegistry,
    final_scale: float,
) -> bool:
    if not adjustments:
        return True

    grouped_targets: dict[ChainRef, dict[int, Vector]] = {}
    for chain_ref, source_point_index, target_uv in adjustments:
        grouped_targets.setdefault(chain_ref, {})[source_point_index] = target_uv.copy()

    staged_updates: dict[ChainRef, ScaffoldChainPlacement] = {}
    staged_registry: PointRegistry = {}

    for chain_ref, point_updates in grouped_targets.items():
        existing = placed_chains_map.get(chain_ref)
        if existing is None or not existing.points:
            return False
        chain = graph.get_chain(chain_ref[0], chain_ref[1], chain_ref[2])
        if chain is None:
            return False

        point_count = len(existing.points)
        start_uv = existing.points[0][1].copy()
        end_uv = existing.points[-1][1].copy()
        for source_point_index, target_uv in point_updates.items():
            if source_point_index == 0:
                start_uv = target_uv.copy()
            elif source_point_index == point_count - 1:
                end_uv = target_uv.copy()
            else:
                return False

        rebuilt_uvs = _cf_rebuild_chain_points_for_endpoints(
            chain,
            start_uv,
            end_uv,
            final_scale,
            effective_role=existing.frame_role,
        )
        if rebuilt_uvs is None or len(rebuilt_uvs) != point_count:
            return False

        new_points = tuple(
            (point_key, rebuilt_uvs[index].copy())
            for index, (point_key, _) in enumerate(existing.points)
        )
        staged_updates[chain_ref] = replace(existing, points=new_points)
        for index, (_, uv) in enumerate(new_points):
            staged_registry[_point_registry_key(chain_ref, index)] = uv.copy()

    placed_chains_map.update(staged_updates)
    point_registry.update(staged_registry)
    return True


def _cf_determine_direction(chain, node):
    """UV direction Ð´Ð»Ñ chain Ð¸Ð· Ð±Ð°Ð·Ð¸ÑÐ° patch.

    H_FRAME â€” snap Ðº (Â±1, 0)
    V_FRAME â€” snap Ðº (0, Â±1)
    FREE â€” Ð¿Ñ€Ð¾ÐµÐºÑ†Ð¸Ñ 3D direction Ð½Ð° basis
    """
    if len(chain.vert_cos) < 2:
        if chain.frame_role == FrameRole.H_FRAME:
            return Vector((1.0, 0.0))
        if chain.frame_role == FrameRole.V_FRAME:
            return Vector((0.0, 1.0))
        return Vector((1.0, 0.0))

    chain_3d = chain.vert_cos[-1] - chain.vert_cos[0]

    if chain.frame_role == FrameRole.H_FRAME:
        dot = chain_3d.dot(node.basis_u)
        return Vector((1.0 if dot >= 0.0 else -1.0, 0.0))

    if chain.frame_role == FrameRole.V_FRAME:
        dot = chain_3d.dot(node.basis_v)
        return Vector((0.0, 1.0 if dot >= 0.0 else -1.0))

    u_comp = chain_3d.dot(node.basis_u)
    v_comp = chain_3d.dot(node.basis_v)
    direction = Vector((u_comp, v_comp))
    if direction.length > 1e-8:
        return direction.normalized()
    return Vector((1.0, 0.0))


def _cf_determine_direction_for_role(chain, node, role: FrameRole):
    """Resolve direction from explicit placement role, not only raw chain.frame_role."""
    if role == chain.frame_role:
        return _cf_determine_direction(chain, node)

    if len(chain.vert_cos) < 2:
        if role == FrameRole.H_FRAME:
            return Vector((1.0, 0.0))
        if role == FrameRole.V_FRAME:
            return Vector((0.0, 1.0))
        return Vector((1.0, 0.0))

    chain_3d = chain.vert_cos[-1] - chain.vert_cos[0]

    if role == FrameRole.H_FRAME:
        dot = chain_3d.dot(node.basis_u)
        return Vector((1.0 if dot >= 0.0 else -1.0, 0.0))

    if role == FrameRole.V_FRAME:
        dot = chain_3d.dot(node.basis_v)
        return Vector((0.0, 1.0 if dot >= 0.0 else -1.0))

    u_comp = chain_3d.dot(node.basis_u)
    v_comp = chain_3d.dot(node.basis_v)
    direction = Vector((u_comp, v_comp))
    if direction.length > 1e-8:
        return direction.normalized()
    return Vector((1.0, 0.0))


def _is_orthogonal_hv_pair(role_a, role_b):
    return {role_a, role_b} == {FrameRole.H_FRAME, FrameRole.V_FRAME}


def _cf_resolve_anchor_corner(src_chain, anchor, node):
    if anchor.source_point_index == 0:
        corner_index = src_chain.start_corner_index
    else:
        corner_index = src_chain.end_corner_index

    if corner_index < 0:
        return None

    loop_index = anchor.source_ref[1]
    if loop_index < 0 or loop_index >= len(node.boundary_loops):
        return None

    boundary_loop = node.boundary_loops[loop_index]
    if 0 <= corner_index < len(boundary_loop.corners):
        return boundary_loop.corners[corner_index]
    return None


def _compute_corner_turn_sign(chain, src_chain, anchor, is_start_anchor, node):
    """Resolve turn sign in corner wedge-space using BoundaryCorner.wedge_normal."""
    src_cos = src_chain.vert_cos
    if not src_cos or len(src_cos) < 2:
        return 0

    if anchor.source_point_index == 0:
        src_tangent = src_cos[1] - src_cos[0]
    else:
        src_tangent = src_cos[-2] - src_cos[-1]

    chain_cos = chain.vert_cos
    if not chain_cos or len(chain_cos) < 2:
        return 0

    if is_start_anchor:
        chain_tangent = chain_cos[1] - chain_cos[0]
    else:
        chain_tangent = chain_cos[-2] - chain_cos[-1]

    if src_tangent.length_squared <= 1e-12 or chain_tangent.length_squared <= 1e-12:
        return 0

    incoming = -src_tangent
    outgoing = chain_tangent

    corner = _cf_resolve_anchor_corner(src_chain, anchor, node)
    if corner is None or not corner.wedge_normal_valid:
        return 0
    if corner.wedge_normal.length_squared <= 1e-12:
        return 0

    cross_vec = incoming.cross(outgoing)
    if cross_vec.length_squared <= 1e-12:
        return 0

    dot_val = cross_vec.dot(corner.wedge_normal)
    if abs(dot_val) < 1e-8:
        return 0
    return 1 if dot_val > 0 else -1


def _perpendicular_direction_for_role(src_uv_delta, target_role, turn_sign):
    if target_role == FrameRole.H_FRAME:
        # src_chain â€” V_FRAME, UV Ð¸Ð´Ñ‘Ñ‚ Ð²Ð´Ð¾Ð»ÑŒ Y. Perpendicular = Ð²Ð´Ð¾Ð»ÑŒ X.
        src_y_sign = 1.0 if src_uv_delta.y > 0 else -1.0
        # CCW turn (1) Ð¾Ñ‚ +Y â†’ -X; CW turn (-1) Ð¾Ñ‚ +Y â†’ +X
        x_sign = -src_y_sign * turn_sign
        return Vector((x_sign, 0.0))
    if target_role == FrameRole.V_FRAME:
        # src_chain â€” H_FRAME, UV Ð¸Ð´Ñ‘Ñ‚ Ð²Ð´Ð¾Ð»ÑŒ X. Perpendicular = Ð²Ð´Ð¾Ð»ÑŒ Y.
        src_x_sign = 1.0 if src_uv_delta.x > 0 else -1.0
        # CCW turn (1) Ð¾Ñ‚ +X â†’ +Y; CW turn (-1) Ð¾Ñ‚ +X â†’ -Y
        y_sign = src_x_sign * turn_sign
        return Vector((0.0, y_sign))
    return None


def _cf_can_inherit_corner_turn_direction(chain, src_chain):
    """Ð Ð°Ð·Ñ€ÐµÑˆÐ°ÐµÑ‚ turn-sign inheritance Ð´Ð»Ñ sharp same-patch turns, Ð½Ðµ Ñ‚Ð¾Ð»ÑŒÐºÐ¾ border."""
    if chain.is_corner_split or src_chain.is_corner_split:
        return True
    if (
        chain.neighbor_kind == ChainNeighborKind.PATCH
        and (
            chain.frame_role == FrameRole.FREE
            or src_chain.frame_role == FrameRole.FREE
            or len(chain.vert_cos) <= 2
            or len(src_chain.vert_cos) <= 2
        )
    ):
        return True
    return chain.neighbor_kind in {
        ChainNeighborKind.MESH_BORDER,
        ChainNeighborKind.SEAM_SELF,
    }


def _try_inherit_direction(
    chain, node, start_anchor, end_anchor, graph, point_registry,
    chain_ref=None,
    effective_role: Optional[FrameRole] = None,
    placed_chains_map: Optional[dict[ChainRef, ScaffoldChainPlacement]] = None,
):
    role = effective_role if effective_role is not None else chain.frame_role
    if role not in (FrameRole.H_FRAME, FrameRole.V_FRAME):
        return None

    own_direction = _cf_determine_direction_for_role(chain, node, role)

    for is_start_anchor, anchor in ((True, start_anchor), (False, end_anchor)):
        if anchor is None or anchor.source_kind != PlacementSourceKind.SAME_PATCH:
            continue
        src_chain = graph.get_chain(*anchor.source_ref)
        if src_chain is None:
            continue

        src_ref = anchor.source_ref
        src_placement = placed_chains_map.get(src_ref) if placed_chains_map is not None else None
        src_role = src_placement.frame_role if src_placement is not None else src_chain.frame_role
        n_pts = len(src_chain.vert_cos)
        src_start_uv = point_registry.get(_point_registry_key(src_ref, 0))
        src_end_uv = point_registry.get(_point_registry_key(src_ref, n_pts - 1))
        if src_start_uv is None or src_end_uv is None:
            continue

        # === Ð¡ÑƒÑ‰ÐµÑÑ‚Ð²ÑƒÑŽÑ‰Ð¸Ð¹ Ð¿ÑƒÑ‚ÑŒ: same-role inheritance ===
        if src_role == role:
            if role == FrameRole.H_FRAME:
                du = src_end_uv.x - src_start_uv.x
                if abs(du) > 1e-9:
                    inherited = Vector((1.0 if du > 0 else -1.0, 0.0))
                    if inherited.dot(own_direction) > 0.0:
                        return inherited
            else:
                dv = src_end_uv.y - src_start_uv.y
                if abs(dv) > 1e-9:
                    inherited = Vector((0.0, 1.0 if dv > 0 else -1.0))
                    if inherited.dot(own_direction) > 0.0:
                        return inherited
            continue

        # === Corner-split: 3D turn-sign inheritance ===
        if not _is_orthogonal_hv_pair(role, src_role):
            continue
        if not _cf_can_inherit_corner_turn_direction(chain, src_chain):
            continue

        # Ð’Ñ‹Ñ‡Ð¸ÑÐ»ÑÐµÐ¼ Ð·Ð½Ð°Ðº Ð¿Ð¾Ð²Ð¾Ñ€Ð¾Ñ‚Ð° Ð² 3D
        turn_sign = _compute_corner_turn_sign(
            chain, src_chain, anchor, is_start_anchor, node,
        )
        if turn_sign == 0:
            continue

        src_uv_delta = src_end_uv - src_start_uv
        perp = _perpendicular_direction_for_role(
            src_uv_delta, role, turn_sign,
        )
        if perp is not None:
            return perp

    return None


def _cf_place_chain(
    chain,
    node,
    start_anchor: Optional[ChainAnchor],
    end_anchor: Optional[ChainAnchor],
    final_scale,
    direction_override=None,
    placed_chains_map=None,
    graph=None,
    effective_role=None,
    chain_ref: Optional[ChainRef] = None,
    runtime_policy=None,
):
    """Ð Ð°Ð·Ð¼ÐµÑ‰Ð°ÐµÑ‚ Ð¾Ð´Ð¸Ð½ chain Ð² UV, Ð¸ÑÐ¿Ð¾Ð»ÑŒÐ·ÑƒÑ Ð¿Ñ€Ð¾Ð²ÐµÑ€ÐµÐ½Ð½Ñ‹Ðµ anchors."""
    source_pts = _cf_chain_source_points(chain)
    role = effective_role if effective_role is not None else chain.frame_role
    direction = (
        direction_override
        if direction_override is not None else
        _cf_determine_direction_for_role(chain, node, role)
    )

    if role in (FrameRole.H_FRAME, FrameRole.V_FRAME):
        resolved_segment = _resolve_frame_segment(
            chain_ref,
            chain,
            node,
            start_anchor,
            end_anchor,
            final_scale,
            direction,
            role,
            placed_chains_map=placed_chains_map,
            graph=graph,
            runtime_policy=runtime_policy,
        )
        if resolved_segment is not None:
            return _build_frame_chain_between_anchors(
                source_pts,
                resolved_segment.start_uv,
                resolved_segment.end_uv,
                role,
                final_scale,
            )

    start_uv = start_anchor.uv.copy() if start_anchor is not None else None
    end_uv = end_anchor.uv.copy() if end_anchor is not None else None

    if start_uv is not None and end_uv is not None:
        return _build_guided_free_chain_between_anchors(
            node, source_pts, start_uv, end_uv, direction, None, final_scale)

    if start_uv is not None:
        return _build_guided_free_chain_from_one_end(
            node, source_pts, start_uv, direction, final_scale)

    if end_uv is not None:
        rev_pts = list(reversed(source_pts))
        rev_dir = Vector((-direction.x, -direction.y))
        rev_uvs = _build_guided_free_chain_from_one_end(
            node, rev_pts, end_uv, rev_dir, final_scale)
        return list(reversed(rev_uvs))

    return []


