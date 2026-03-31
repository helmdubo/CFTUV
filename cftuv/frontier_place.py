from __future__ import annotations

from dataclasses import replace
import math
from typing import Optional

from mathutils import Vector

try:
    from .model import (
        AnchorAdjustment,
        BoundaryChain,
        ChainNeighborKind,
        ChainRef,
        FrameRole,
        PatchGraph,
        PatchNode,
        PlacementSourceKind,
        ScaffoldChainPlacement,
        ScaffoldPointKey,
        SourcePoint,
    )
    from .solve_records import (
        ChainAnchor,
        PointRegistry,
        SeedPlacementResult,
        _point_registry_key,
    )
except ImportError:
    from model import (
        AnchorAdjustment,
        BoundaryChain,
        ChainNeighborKind,
        ChainRef,
        FrameRole,
        PatchGraph,
        PatchNode,
        PlacementSourceKind,
        ScaffoldChainPlacement,
        ScaffoldPointKey,
        SourcePoint,
    )
    from solve_records import (
        ChainAnchor,
        PointRegistry,
        SeedPlacementResult,
        _point_registry_key,
    )


def _build_temporary_chain_placement(
    chain_ref: ChainRef,
    chain: BoundaryChain,
    uv_points: list[Vector],
    start_anchor: Optional[ChainAnchor],
    end_anchor: Optional[ChainAnchor],
) -> ScaffoldChainPlacement:
    patch_id, loop_index, chain_index = chain_ref
    return ScaffoldChainPlacement(
        patch_id=patch_id,
        loop_index=loop_index,
        chain_index=chain_index,
        frame_role=chain.frame_role,
        source_kind=PlacementSourceKind.CHAIN,
        anchor_count=_cf_anchor_count(start_anchor, end_anchor),
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
) -> Optional[SeedPlacementResult]:
    seed_src = _cf_chain_source_points(seed_chain)
    seed_dir = _cf_determine_direction(seed_chain, root_node)

    if seed_chain.frame_role in (FrameRole.H_FRAME, FrameRole.V_FRAME):
        seed_uvs = _build_frame_chain_from_one_end(
            seed_src,
            Vector((0.0, 0.0)),
            seed_dir,
            seed_chain.frame_role,
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
        frame_role=seed_chain.frame_role,
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
    """Конвертирует chain.vert_cos в формат [(index, Vector)] для placement функций."""
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
    final_scale: float,
) -> list[Vector]:
    if not ordered_source_points:
        return []
    if len(ordered_source_points) == 1:
        return [start_point.copy()]
    edge_lengths = _chain_edge_lengths(ordered_source_points, final_scale)
    return _interpolate_between_anchors_by_lengths(start_point, end_point, edge_lengths)


def _cf_rebuild_chain_points_for_endpoints(
    chain: BoundaryChain,
    start_uv: Vector,
    end_uv: Vector,
    final_scale: float,
) -> Optional[list[Vector]]:
    source_pts = _cf_chain_source_points(chain)
    if len(source_pts) != len(chain.vert_indices):
        return None

    if chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        return _build_frame_chain_between_anchors(source_pts, start_uv, end_uv, final_scale)

    if chain.frame_role == FrameRole.FREE and len(source_pts) <= 2:
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

        rebuilt_uvs = _cf_rebuild_chain_points_for_endpoints(chain, start_uv, end_uv, final_scale)
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
    """UV direction для chain из базиса patch.

    H_FRAME — snap к (±1, 0)
    V_FRAME — snap к (0, ±1)
    FREE — проекция 3D direction на basis
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


def _is_orthogonal_hv_pair(role_a, role_b):
    return {role_a, role_b} == {FrameRole.H_FRAME, FrameRole.V_FRAME}


def _cf_corner_co_from_anchor(src_chain: BoundaryChain, anchor: ChainAnchor) -> Optional[Vector]:
    src_cos = src_chain.vert_cos
    if not src_cos:
        return None
    if anchor.source_point_index == 0:
        return src_cos[0].copy()
    return src_cos[-1].copy()


def _cf_compute_local_normal(node: PatchNode, corner_co: Vector) -> Vector:
    """Approximate a corner-local normal from real patch triangles touching the corner.

    Primary intent:
    - keep flat/concave patches on the 2D basis path;
    - use a local 3D fallback only when basis projection becomes unreliable
      on strongly wrapped patches.

    This is still a transitional fallback because PatchNode currently stores
    serialized triangles, not explicit wedge-local face normals. Once the
    topology layer exposes per-corner wedge faces/normals, this helper should
    be replaced with that stronger source of truth.
    """
    if corner_co is None:
        return node.normal.normalized() if node.normal.length > 1e-8 else Vector((0.0, 0.0, 1.0))

    mesh_verts = getattr(node, 'mesh_verts', None) or []
    mesh_tris = getattr(node, 'mesh_tris', None) or []
    if not mesh_verts or not mesh_tris:
        return node.normal.normalized() if node.normal.length > 1e-8 else Vector((0.0, 0.0, 1.0))

    eps_corner_sq = 1e-10
    acc_touch = Vector((0.0, 0.0, 0.0))
    acc_near = Vector((0.0, 0.0, 0.0))
    touch_weight = 0.0
    near_weight = 0.0

    for tri in mesh_tris:
        if len(tri) != 3:
            continue
        try:
            a = mesh_verts[tri[0]]
            b = mesh_verts[tri[1]]
            c = mesh_verts[tri[2]]
        except (IndexError, TypeError):
            continue

        tri_cross = (b - a).cross(c - a)
        tri_area2 = tri_cross.length
        if tri_area2 <= 1e-12:
            continue
        tri_normal = tri_cross / tri_area2

        tri_verts = (a, b, c)
        touches_corner = any((vert - corner_co).length_squared <= eps_corner_sq for vert in tri_verts)

        if touches_corner:
            weight = tri_area2
            acc_touch += tri_normal * weight
            touch_weight += weight
            continue

        centroid = (a + b + c) / 3.0
        dist_sq = (centroid - corner_co).length_squared
        if dist_sq > 1e-4:
            continue

        weight = tri_area2 / max(dist_sq, 1e-12)
        acc_near += tri_normal * weight
        near_weight += weight

    local_normal = Vector((0.0, 0.0, 0.0))
    if touch_weight > 1e-12 and acc_touch.length > 1e-8:
        local_normal = acc_touch.normalized()
    elif near_weight > 1e-12 and acc_near.length > 1e-8:
        local_normal = acc_near.normalized()
    elif node.normal.length > 1e-8:
        local_normal = node.normal.normalized()
    else:
        local_normal = Vector((0.0, 0.0, 1.0))

    return local_normal


def _compute_corner_turn_sign(chain, src_chain, anchor, is_start_anchor, node):
    """Determine turn sign at a corner with a hybrid local solver.

    Primary path:
    - project incoming/outgoing tangents into the patch basis and use a 2D
      oriented cross when projection retains enough information.

    Fallback path:
    - if one tangent becomes nearly orthogonal to the basis plane (typical on
      >180° wrapped WALL patches), evaluate the 3D turn against a corner-local
      normal reconstructed from triangles touching that corner.

    This keeps flat/concave patches on the stable chart-space path while
    restoring local orientation on heavily wrapped patches.
    """
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

    in_u = incoming.dot(node.basis_u)
    in_v = incoming.dot(node.basis_v)
    out_u = outgoing.dot(node.basis_u)
    out_v = outgoing.dot(node.basis_v)

    len_in_sq = in_u * in_u + in_v * in_v
    len_out_sq = out_u * out_u + out_v * out_v
    src_len_sq = incoming.length_squared
    out_len_sq_3d = outgoing.length_squared

    proj_ok = (
        src_len_sq > 1e-12
        and out_len_sq_3d > 1e-12
        and len_in_sq > 0.01 * src_len_sq
        and len_out_sq > 0.01 * out_len_sq_3d
    )

    if proj_ok:
        cross_2d = in_u * out_v - in_v * out_u
        denom = math.sqrt(len_in_sq * len_out_sq)
        if denom > 1e-12:
            signed_sin = cross_2d / denom
            if abs(signed_sin) > 1e-4:
                return 1 if signed_sin > 0 else -1

    corner_co = _cf_corner_co_from_anchor(src_chain, anchor)
    if corner_co is None:
        return 0

    local_normal = _cf_compute_local_normal(node, corner_co)
    cross_vec = incoming.cross(outgoing)
    if cross_vec.length_squared <= 1e-12 or local_normal.length_squared <= 1e-12:
        return 0

    dot_normal = cross_vec.dot(local_normal)
    if abs(dot_normal) < 1e-8:
        return 0
    return 1 if dot_normal > 0 else -1


def _perpendicular_direction_for_role(src_uv_delta, target_role, turn_sign):
    if target_role == FrameRole.H_FRAME:
        # src_chain — V_FRAME, UV идёт вдоль Y. Perpendicular = вдоль X.
        src_y_sign = 1.0 if src_uv_delta.y > 0 else -1.0
        # CCW turn (1) от +Y → -X; CW turn (-1) от +Y → +X
        x_sign = -src_y_sign * turn_sign
        return Vector((x_sign, 0.0))
    if target_role == FrameRole.V_FRAME:
        # src_chain — H_FRAME, UV идёт вдоль X. Perpendicular = вдоль Y.
        src_x_sign = 1.0 if src_uv_delta.x > 0 else -1.0
        # CCW turn (1) от +X → +Y; CW turn (-1) от +X → -Y
        y_sign = src_x_sign * turn_sign
        return Vector((0.0, y_sign))
    return None


def _cf_can_inherit_corner_turn_direction(chain, src_chain):
    """Разрешает turn-sign inheritance для legacy corner-split и новых border chains."""
    if chain.is_corner_split or src_chain.is_corner_split:
        return True
    return chain.neighbor_kind == ChainNeighborKind.MESH_BORDER


def _try_inherit_direction(
    chain, node, start_anchor, end_anchor, graph, point_registry,
    chain_ref=None,
):
    if chain.frame_role not in (FrameRole.H_FRAME, FrameRole.V_FRAME):
        return None

    own_direction = _cf_determine_direction(chain, node)

    for is_start_anchor, anchor in ((True, start_anchor), (False, end_anchor)):
        if anchor is None or anchor.source_kind != PlacementSourceKind.SAME_PATCH:
            continue
        src_chain = graph.get_chain(*anchor.source_ref)
        if src_chain is None:
            continue

        src_ref = anchor.source_ref
        n_pts = len(src_chain.vert_cos)
        src_start_uv = point_registry.get(_point_registry_key(src_ref, 0))
        src_end_uv = point_registry.get(_point_registry_key(src_ref, n_pts - 1))
        if src_start_uv is None or src_end_uv is None:
            continue

        # === Существующий путь: same-role inheritance ===
        if src_chain.frame_role == chain.frame_role:
            if chain.frame_role == FrameRole.H_FRAME:
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
        if not _is_orthogonal_hv_pair(chain.frame_role, src_chain.frame_role):
            continue
        if not _cf_can_inherit_corner_turn_direction(chain, src_chain):
            continue

        # Вычисляем знак поворота в 3D
        turn_sign = _compute_corner_turn_sign(
            chain, src_chain, anchor, is_start_anchor, node,
        )
        if turn_sign == 0:
            continue

        src_uv_delta = src_end_uv - src_start_uv
        perp = _perpendicular_direction_for_role(
            src_uv_delta, chain.frame_role, turn_sign,
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
):
    """Размещает один chain в UV, используя проверенные anchors."""
    source_pts = _cf_chain_source_points(chain)
    direction = direction_override if direction_override is not None else _cf_determine_direction(chain, node)
    role = chain.frame_role
    start_uv = start_anchor.uv.copy() if start_anchor is not None else None
    end_uv = end_anchor.uv.copy() if end_anchor is not None else None

    if start_uv is not None and end_uv is not None:
        if role in (FrameRole.H_FRAME, FrameRole.V_FRAME):
            return _build_frame_chain_between_anchors(source_pts, start_uv, end_uv, final_scale)
        return _build_guided_free_chain_between_anchors(
            node, source_pts, start_uv, end_uv, direction, None, final_scale)

    if start_uv is not None:
        if role in (FrameRole.H_FRAME, FrameRole.V_FRAME):
            return _build_frame_chain_from_one_end(source_pts, start_uv, direction, role, final_scale)
        return _build_guided_free_chain_from_one_end(
            node, source_pts, start_uv, direction, final_scale)

    if end_uv is not None:
        rev_pts = list(reversed(source_pts))
        rev_dir = Vector((-direction.x, -direction.y))
        if role in (FrameRole.H_FRAME, FrameRole.V_FRAME):
            rev_uvs = _build_frame_chain_from_one_end(rev_pts, end_uv, rev_dir, role, final_scale)
        else:
            rev_uvs = _build_guided_free_chain_from_one_end(
                node, rev_pts, end_uv, rev_dir, final_scale)
        return list(reversed(rev_uvs))

    return []
