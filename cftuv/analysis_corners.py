from __future__ import annotations

import math

from mathutils import Vector

try:
    from .constants import (
        CORNER_ANGLE_THRESHOLD_DEG,
        FRAME_ALIGNMENT_THRESHOLD_H,
        FRAME_ALIGNMENT_THRESHOLD_V,
        NB_MESH_BORDER,
    )
    from .model import BoundaryCorner, CornerKind, FrameRole, LoopKind
    from .analysis_records import (
        _CornerTurnCandidate,
        _CornerDetectionPolicy,
        _OpenBorderCornerDetectionResult,
        _RawBoundaryChain,
    )
    from .console_debug import trace_console
except ImportError:
    from constants import (
        CORNER_ANGLE_THRESHOLD_DEG,
        FRAME_ALIGNMENT_THRESHOLD_H,
        FRAME_ALIGNMENT_THRESHOLD_V,
        NB_MESH_BORDER,
    )
    from model import BoundaryCorner, CornerKind, FrameRole, LoopKind
    from analysis_records import (
        _CornerTurnCandidate,
        _CornerDetectionPolicy,
        _OpenBorderCornerDetectionResult,
        _RawBoundaryChain,
    )
    from console_debug import trace_console


def _measure_chain_axis_metrics(chain_vert_cos, basis_u, basis_v):
    if len(chain_vert_cos) < 2:
        return None

    h_support = 0.0
    v_support = 0.0
    h_deviation_sum = 0.0
    v_deviation_sum = 0.0
    h_max_deviation = 0.0
    v_max_deviation = 0.0
    total_length = 0.0

    for point_index in range(len(chain_vert_cos) - 1):
        delta = chain_vert_cos[point_index + 1] - chain_vert_cos[point_index]
        seg_len = delta.length
        if seg_len < 1e-8:
            continue

        du = delta.dot(basis_u)
        dv = delta.dot(basis_v)
        abs_du = abs(du)
        abs_dv = abs(dv)

        h_support += abs_du
        v_support += abs_dv
        h_deviation = abs_dv / seg_len
        v_deviation = abs_du / seg_len
        h_deviation_sum += h_deviation * seg_len
        v_deviation_sum += v_deviation * seg_len
        h_max_deviation = max(h_max_deviation, h_deviation)
        v_max_deviation = max(v_max_deviation, v_deviation)
        total_length += seg_len

    if total_length < 1e-8:
        return None

    return {
        "h_support": h_support,
        "v_support": v_support,
        "h_avg_deviation": h_deviation_sum / total_length,
        "v_avg_deviation": v_deviation_sum / total_length,
        "h_max_deviation": h_max_deviation,
        "v_max_deviation": v_max_deviation,
        "total_length": total_length,
    }


def _classify_chain_frame_role(chain_vert_cos, basis_u, basis_v, strict_guards=True):
    metrics = _measure_chain_axis_metrics(chain_vert_cos, basis_u, basis_v)
    if metrics is None:
        return FrameRole.FREE

    threshold_h = FRAME_ALIGNMENT_THRESHOLD_H
    threshold_v = FRAME_ALIGNMENT_THRESHOLD_V
    if strict_guards:
        h_max_deviation_limit = max(threshold_h * 2.0, threshold_h + 0.03)
        v_max_deviation_limit = max(threshold_v * 2.0, threshold_v + 0.03)
    else:
        h_max_deviation_limit = max(threshold_h * 4.0, threshold_h + 0.08)
        v_max_deviation_limit = max(threshold_v * 4.0, threshold_v + 0.08)

    h_ok = (
        metrics["h_support"] > 1e-6
        and metrics["h_avg_deviation"] < threshold_h
        and metrics["h_max_deviation"] < h_max_deviation_limit
    )
    v_ok = (
        metrics["v_support"] > 1e-6
        and metrics["v_avg_deviation"] < threshold_v
        and metrics["v_max_deviation"] < v_max_deviation_limit
    )

    if h_ok and v_ok:
        if abs(metrics["h_avg_deviation"] - metrics["v_avg_deviation"]) > 1e-6:
            return FrameRole.H_FRAME if metrics["h_avg_deviation"] < metrics["v_avg_deviation"] else FrameRole.V_FRAME
        return FrameRole.H_FRAME if metrics["h_support"] >= metrics["v_support"] else FrameRole.V_FRAME
    if h_ok:
        return FrameRole.H_FRAME
    if v_ok:
        return FrameRole.V_FRAME
    return FrameRole.FREE


def _find_corner_reference_point(points, corner_co, reverse=False):
    """Find a non-degenerate neighbor point around a chain junction."""

    if reverse:
        candidates = list(reversed(points[:-1]))
    else:
        candidates = list(points[1:])

    for point in candidates:
        if (point - corner_co).length_squared > 1e-12:
            return point
    return None


def _measure_corner_turn_angle(corner_co, prev_point, next_point, basis_u, basis_v):
    """Measure the turn angle at a chain junction in patch-local 2D space."""

    if prev_point is None or next_point is None:
        return 0.0

    prev_vec_3d = corner_co - prev_point
    next_vec_3d = next_point - corner_co
    prev_vec = Vector((prev_vec_3d.dot(basis_u), prev_vec_3d.dot(basis_v)))
    next_vec = Vector((next_vec_3d.dot(basis_u), next_vec_3d.dot(basis_v)))
    if prev_vec.length_squared < 1e-12 or next_vec.length_squared < 1e-12:
        return 0.0

    return math.degrees(prev_vec.angle(next_vec, 0.0))


def _measure_corner_turn_angle_3d(corner_co, prev_point, next_point):
    """Measure the geometric turn angle in full 3D space."""

    if prev_point is None or next_point is None:
        return 0.0

    prev_vec = corner_co - prev_point
    next_vec = next_point - corner_co
    if prev_vec.length_squared < 1e-12 or next_vec.length_squared < 1e-12:
        return 0.0

    return math.degrees(prev_vec.angle(next_vec, 0.0))


def _build_geometric_loop_corners(boundary_loop, basis_u, basis_v):
    """Build geometric corners directly from loop turns, even inside one FREE chain."""

    vertex_count = len(boundary_loop.vert_cos)
    if vertex_count < 3:
        return []

    corners = []
    for loop_vert_index in range(vertex_count):
        corner_co = boundary_loop.vert_cos[loop_vert_index].copy()
        prev_point = boundary_loop.vert_cos[(loop_vert_index - 1) % vertex_count]
        next_point = boundary_loop.vert_cos[(loop_vert_index + 1) % vertex_count]
        turn_angle_deg = _measure_corner_turn_angle(corner_co, prev_point, next_point, basis_u, basis_v)
        if turn_angle_deg < CORNER_ANGLE_THRESHOLD_DEG:
            continue
        corners.append(
            BoundaryCorner(
                loop_vert_index=loop_vert_index,
                vert_index=boundary_loop.vert_indices[loop_vert_index] if loop_vert_index < len(boundary_loop.vert_indices) else -1,
                vert_co=corner_co,
                prev_chain_index=0,
                next_chain_index=0,
                corner_kind=CornerKind.GEOMETRIC,
                turn_angle_deg=turn_angle_deg,
                prev_role=FrameRole.FREE,
                next_role=FrameRole.FREE,
            )
        )

    return corners


def _loop_arc_length(loop_vert_cos, start_loop_index, end_loop_index):
    """Measure the closed-loop arc length from start to end."""

    vertex_count = len(loop_vert_cos)
    if vertex_count < 2:
        return 0.0

    length = 0.0
    loop_index = start_loop_index % vertex_count
    safety = 0
    while safety < vertex_count + 1:
        safety += 1
        next_loop_index = (loop_index + 1) % vertex_count
        length += (loop_vert_cos[next_loop_index] - loop_vert_cos[loop_index]).length
        loop_index = next_loop_index
        if loop_index == end_loop_index % vertex_count:
            break

    return length


def _is_tangent_plane_turn(corner_co, prev_point, next_point, vert_normal):
    edge_in = corner_co - prev_point
    edge_out = next_point - corner_co
    if edge_in.length_squared < 1e-12 or edge_out.length_squared < 1e-12:
        return True
    edge_in_n = edge_in.normalized()
    edge_out_n = edge_out.normalized()
    turn_axis = edge_in_n.cross(edge_out_n)
    if turn_axis.length_squared < 1e-12:
        dot_val = edge_in_n.dot(edge_out_n)
        if dot_val < -0.5:
            return False
        return True
    alignment = abs(turn_axis.normalized().dot(vert_normal.normalized()))
    return alignment > 0.3


def _collect_geometric_split_indices(loop_vert_cos, basis_u, basis_v, loop_vert_indices=None, bm=None):
    """Compatibility wrapper over the unified closed-loop corner detector."""

    return _detect_closed_loop_corner_indices(
        loop_vert_cos,
        basis_u,
        basis_v,
        loop_vert_indices=loop_vert_indices,
        bm=bm,
    )


def _split_closed_loop_by_corner_indices(raw_loop, split_indices, neighbor_patch_id):
    """Split one closed raw loop into raw chains at geometric corners."""

    loop_vert_indices = list(raw_loop.vert_indices)
    loop_vert_cos = list(raw_loop.vert_cos)
    loop_edge_indices = list(raw_loop.edge_indices)
    loop_side_face_indices = list(raw_loop.side_face_indices)
    vertex_count = len(loop_vert_cos)
    edge_count = len(loop_edge_indices)
    if vertex_count < 2 or edge_count == 0:
        return []

    ordered_split_indices = sorted({loop_vert_index % vertex_count for loop_vert_index in split_indices})
    if len(ordered_split_indices) < 2:
        return []

    raw_chains = []
    split_count = len(ordered_split_indices)
    for split_index in range(split_count):
        start_loop_index = ordered_split_indices[split_index]
        end_loop_index = ordered_split_indices[(split_index + 1) % split_count]

        chain_vert_indices = []
        chain_vert_cos = []
        chain_edge_indices = []
        chain_side_face_indices = []

        loop_index = start_loop_index
        safety = 0
        while safety < vertex_count + 2:
            safety += 1
            chain_vert_indices.append(loop_vert_indices[loop_index % vertex_count])
            chain_vert_cos.append(loop_vert_cos[loop_index % vertex_count])
            chain_edge_indices.append(loop_edge_indices[loop_index % edge_count])
            chain_side_face_indices.append(loop_side_face_indices[loop_index % vertex_count])
            loop_index += 1
            if loop_index % vertex_count == end_loop_index % vertex_count:
                chain_vert_indices.append(loop_vert_indices[end_loop_index % vertex_count])
                chain_vert_cos.append(loop_vert_cos[end_loop_index % vertex_count])
                chain_side_face_indices.append(loop_side_face_indices[end_loop_index % vertex_count])
                break

        if len(chain_vert_indices) < 2:
            continue

        raw_chains.append(
            _RawBoundaryChain(
                vert_indices=chain_vert_indices,
                vert_cos=chain_vert_cos,
                edge_indices=chain_edge_indices,
                side_face_indices=chain_side_face_indices,
                neighbor=neighbor_patch_id,
                is_closed=False,
                start_loop_index=start_loop_index,
                end_loop_index=end_loop_index,
            )
        )

    return raw_chains


def _find_open_chain_corners(vert_cos, basis_u, basis_v, min_spacing=2, vert_indices=None, bm=None):
    """Compatibility wrapper over the unified open-border corner detector."""

    return _detect_open_border_corner_indices(
        vert_cos,
        basis_u,
        basis_v,
        min_spacing=min_spacing,
        vert_indices=vert_indices,
        bm=bm,
    )


def _find_open_chain_corners_filtered(vert_cos, basis_u, basis_v, min_spacing=2, vert_indices=None, bm=None):
    """Compatibility wrapper over the unified open-border corner detector."""

    return _detect_open_border_corner_indices(
        vert_cos,
        basis_u,
        basis_v,
        min_spacing=min_spacing,
        vert_indices=vert_indices,
        bm=bm,
    )


def _collect_corner_turn_candidates_shared(polyline_cos, basis_u, basis_v, policy, vert_indices=None, bm=None):
    """Shared corner candidate collection for closed loops and open border chains."""

    vertex_count = len(polyline_cos)
    if vertex_count < (4 if policy.closed_loop else 3):
        return [], []

    use_vert_normals = (vert_indices is not None and bm is not None)
    if use_vert_normals:
        bm.verts.ensure_lookup_table()

    raw_candidates: list[_CornerTurnCandidate] = []
    filtered_candidates: list[_CornerTurnCandidate] = []
    candidate_indices = range(vertex_count) if policy.closed_loop else range(1, vertex_count - 1)

    for point_index in candidate_indices:
        prev_point = polyline_cos[(point_index - 1) % vertex_count]
        corner_co = polyline_cos[point_index]
        next_point = polyline_cos[(point_index + 1) % vertex_count]
        turn_angle_deg = _measure_corner_turn_angle(corner_co, prev_point, next_point, basis_u, basis_v)
        trace_console(
            f"[CFTUV][CornerDetect] idx={point_index} turn={turn_angle_deg:.1f} "
            f"threshold={CORNER_ANGLE_THRESHOLD_DEG} use_normals={use_vert_normals}"
        )
        if turn_angle_deg < CORNER_ANGLE_THRESHOLD_DEG:
            continue

        candidate = _CornerTurnCandidate(point_index, turn_angle_deg)
        raw_candidates.append(candidate)
        is_hairpin_turn = turn_angle_deg >= 150.0

        if policy.reject_projected_hairpins and is_hairpin_turn:
            turn_angle_3d = _measure_corner_turn_angle_3d(corner_co, prev_point, next_point)
            if turn_angle_3d < 120.0:
                trace_console(
                    f"[CFTUV][CornerDetect] skip idx={point_index} reason=projected_hairpin "
                    f"turn2d={turn_angle_deg:.1f} turn3d={turn_angle_3d:.1f}"
                )
                continue

        should_filter_bevel_turn = (
            policy.filter_all_bevel_turns
            or (policy.filter_hairpin_bevel_turns and is_hairpin_turn)
        )
        if use_vert_normals and should_filter_bevel_turn:
            vert_index = vert_indices[point_index]
            if vert_index < len(bm.verts):
                vert_normal = bm.verts[vert_index].normal
                if vert_normal.length_squared > 1e-12 and not _is_tangent_plane_turn(
                    corner_co,
                    prev_point,
                    next_point,
                    vert_normal,
                ):
                    reason = "hairpin_not_tangent" if is_hairpin_turn else "bevel_not_tangent"
                    trace_console(
                        f"[CFTUV][CornerDetect] skip idx={point_index} reason={reason} "
                        f"turn2d={turn_angle_deg:.1f}"
                    )
                    continue

        filtered_candidates.append(candidate)

    return raw_candidates, filtered_candidates


def _filter_corner_candidates_by_index_spacing(candidates, min_spacing_vertices):
    """Keep the strongest nearby corner candidate on open chains."""

    if not candidates:
        return []

    filtered = [candidates[0]]
    for candidate in candidates[1:]:
        prev_candidate = filtered[-1]
        if candidate.index - prev_candidate.index < min_spacing_vertices:
            if candidate.turn_angle_deg > prev_candidate.turn_angle_deg:
                filtered[-1] = candidate
            continue
        filtered.append(candidate)

    return filtered


def _filter_closed_corner_candidates_by_loop_spacing(loop_vert_cos, candidates, min_span_length, min_vertex_gap):
    """Cluster closed-loop corner candidates by wrap-aware arc spacing."""

    if not candidates:
        return []

    vertex_count = len(loop_vert_cos)
    filtered: list[_CornerTurnCandidate] = []
    for candidate in candidates:
        if not filtered:
            filtered.append(candidate)
            continue

        prev_candidate = filtered[-1]
        span_length = _loop_arc_length(loop_vert_cos, prev_candidate.index, candidate.index)
        span_vertex_count = (candidate.index - prev_candidate.index) % vertex_count
        if span_vertex_count < min_vertex_gap or span_length < min_span_length:
            if candidate.turn_angle_deg > prev_candidate.turn_angle_deg:
                filtered[-1] = candidate
            continue

        filtered.append(candidate)

    while len(filtered) >= 2:
        first_candidate = filtered[0]
        last_candidate = filtered[-1]
        wrap_span_length = _loop_arc_length(loop_vert_cos, last_candidate.index, first_candidate.index)
        wrap_span_vertex_count = (first_candidate.index - last_candidate.index) % vertex_count
        if wrap_span_vertex_count >= min_vertex_gap and wrap_span_length >= min_span_length:
            break

        if first_candidate.turn_angle_deg >= last_candidate.turn_angle_deg:
            filtered.pop()
        else:
            filtered.pop(0)

    return filtered


def _detect_closed_loop_corner_indices(loop_vert_cos, basis_u, basis_v, loop_vert_indices=None, bm=None):
    """Unified closed-loop corner detection used by geometric OUTER-loop split."""

    policy = _CornerDetectionPolicy(
        closed_loop=True,
        min_spacing_vertices=1,
        min_corner_count=4,
        loop_min_span_fraction=0.04,
        relaxed_loop_min_span_fraction=0.015,
        filter_all_bevel_turns=True,
        allow_raw_non_hairpin_fallback=True,
    )

    raw_candidates, candidate_corners = _collect_corner_turn_candidates_shared(
        loop_vert_cos,
        basis_u,
        basis_v,
        policy,
        vert_indices=loop_vert_indices,
        bm=bm,
    )
    if len(candidate_corners) < policy.min_corner_count and len(raw_candidates) >= policy.min_corner_count:
        non_hairpin_candidates = [
            candidate
            for candidate in raw_candidates
            if candidate.turn_angle_deg < 150.0
        ]
        if policy.allow_raw_non_hairpin_fallback and len(non_hairpin_candidates) >= policy.min_corner_count:
            trace_console(f"[CFTUV][GeoSplit] fallback raw_non_hairpin candidates={len(non_hairpin_candidates)}")
            candidate_corners = non_hairpin_candidates

    if len(candidate_corners) < policy.min_corner_count:
        return []

    perimeter = _loop_arc_length(loop_vert_cos, 0, 0)
    strict_min_span_length = max(perimeter * policy.loop_min_span_fraction, 1e-4)
    filtered_corners = _filter_closed_corner_candidates_by_loop_spacing(
        loop_vert_cos,
        candidate_corners,
        strict_min_span_length,
        policy.min_spacing_vertices,
    )
    if (
        len(filtered_corners) < policy.min_corner_count
        and len(candidate_corners) >= policy.min_corner_count
        and policy.relaxed_loop_min_span_fraction > 0.0
    ):
        relaxed_min_span_length = max(perimeter * policy.relaxed_loop_min_span_fraction, 1e-4)
        if relaxed_min_span_length < strict_min_span_length:
            relaxed_corners = _filter_closed_corner_candidates_by_loop_spacing(
                loop_vert_cos,
                candidate_corners,
                relaxed_min_span_length,
                policy.min_spacing_vertices,
            )
            if len(relaxed_corners) >= policy.min_corner_count:
                filtered_corners = relaxed_corners

    if len(filtered_corners) < policy.min_corner_count:
        return []

    return [candidate.index for candidate in filtered_corners]


def _detect_open_border_corner_indices(vert_cos, basis_u, basis_v, min_spacing=2, vert_indices=None, bm=None):
    """Unified open-border corner detection used before border-chain split."""

    policy = _CornerDetectionPolicy(
        closed_loop=False,
        min_spacing_vertices=min_spacing,
        reject_projected_hairpins=True,
        filter_hairpin_bevel_turns=True,
    )

    _, candidates = _collect_corner_turn_candidates_shared(
        vert_cos,
        basis_u,
        basis_v,
        policy,
        vert_indices=vert_indices,
        bm=bm,
    )
    if not candidates:
        return []

    filtered = _filter_corner_candidates_by_index_spacing(candidates, policy.min_spacing_vertices)
    return [candidate.index for candidate in filtered]


def _detect_open_border_corners(raw_chain, basis_u, basis_v, bm=None, min_spacing=2):
    """Run the full open-border corner pipeline and return staged detection results."""

    candidate_indices = _detect_open_border_corner_indices(
        raw_chain.vert_cos,
        basis_u,
        basis_v,
        min_spacing=min_spacing,
        vert_indices=raw_chain.vert_indices,
        bm=bm,
    )
    supported_indices = _filter_open_chain_corner_indices_with_support(
        raw_chain,
        candidate_indices,
        basis_u,
        basis_v,
    )
    return _OpenBorderCornerDetectionResult(
        candidate_indices=tuple(candidate_indices),
        supported_indices=tuple(supported_indices),
    )


def _measure_polyline_length(vert_cos):
    """Measure total polyline length for local split/support diagnostics."""

    if len(vert_cos) < 2:
        return 0.0

    total_length = 0.0
    for point_index in range(len(vert_cos) - 1):
        total_length += (vert_cos[point_index + 1] - vert_cos[point_index]).length
    return total_length


def _measure_open_corner_side_support(vert_cos, corner_index, basis_u, basis_v, direction, target_length):
    """Measure directional support around one open-border corner candidate."""

    projected_segments = []
    support_length = 0.0

    if direction < 0:
        segment_indices = range(corner_index - 1, -1, -1)
    else:
        segment_indices = range(corner_index, len(vert_cos) - 1)

    for segment_index in segment_indices:
        delta_3d = vert_cos[segment_index + 1] - vert_cos[segment_index]
        seg_len = delta_3d.length
        if seg_len < 1e-8:
            continue

        delta_2d = Vector((delta_3d.dot(basis_u), delta_3d.dot(basis_v)))
        if delta_2d.length_squared < 1e-12:
            continue

        projected_segments.append(delta_2d)
        support_length += seg_len
        if support_length >= target_length:
            break

    if not projected_segments or support_length < 1e-8:
        return None

    support_vec = Vector((0.0, 0.0))
    for segment in projected_segments:
        support_vec += segment
    if support_vec.length_squared < 1e-12:
        return None

    support_dir = support_vec.normalized()
    min_alignment = 1.0
    for segment in projected_segments:
        segment_dir = segment.normalized()
        min_alignment = min(min_alignment, support_dir.dot(segment_dir))

    return {
        "support_length": support_length,
        "direction": support_dir,
        "min_alignment": min_alignment,
        "segment_count": len(projected_segments),
    }


def _open_corner_has_support(vert_cos, corner_index, basis_u, basis_v):
    """Require a candidate to have stable geometric support on both sides."""

    chain_length = _measure_polyline_length(vert_cos)
    if chain_length < 1e-8:
        return None

    min_support_length = max(0.03, min(chain_length * 0.05, 0.35))
    min_alignment = 0.90

    prev_support = _measure_open_corner_side_support(
        vert_cos, corner_index, basis_u, basis_v, direction=-1, target_length=min_support_length
    )
    next_support = _measure_open_corner_side_support(
        vert_cos, corner_index, basis_u, basis_v, direction=+1, target_length=min_support_length
    )

    if prev_support is None or next_support is None:
        return None

    if (
        prev_support["support_length"] < min_support_length
        or next_support["support_length"] < min_support_length
    ):
        return None

    if (
        prev_support["min_alignment"] < min_alignment
        or next_support["min_alignment"] < min_alignment
    ):
        return None

    support_turn = math.degrees(prev_support["direction"].angle(next_support["direction"], 0.0))
    if support_turn < CORNER_ANGLE_THRESHOLD_DEG:
        return None

    return {
        "support_turn": support_turn,
        "min_support_length": min_support_length,
        "prev_support": prev_support,
        "next_support": next_support,
    }


def _measure_open_segment_direction(vert_cos, start_index, end_index, basis_u, basis_v):
    """Measure aggregate 2D direction for a contiguous open-chain segment."""

    if end_index <= start_index:
        return None

    support_vec = Vector((0.0, 0.0))
    total_length = 0.0
    for point_index in range(start_index, end_index):
        delta_3d = vert_cos[point_index + 1] - vert_cos[point_index]
        seg_len = delta_3d.length
        if seg_len < 1e-8:
            continue

        delta_2d = Vector((delta_3d.dot(basis_u), delta_3d.dot(basis_v)))
        if delta_2d.length_squared < 1e-12:
            continue
        support_vec += delta_2d
        total_length += seg_len

    if support_vec.length_squared < 1e-12 or total_length < 1e-8:
        return None

    return {
        "direction": support_vec.normalized(),
        "length": total_length,
    }


def _open_corner_pair_has_support(vert_cos, left_corner_index, right_corner_index, basis_u, basis_v):
    """Accept close paired corners that form a short but legitimate connector."""

    if right_corner_index <= left_corner_index:
        return None

    chain_length = _measure_polyline_length(vert_cos)
    if chain_length < 1e-8:
        return None

    middle_direction = _measure_open_segment_direction(
        vert_cos, left_corner_index, right_corner_index, basis_u, basis_v
    )
    if middle_direction is None:
        return None

    middle_points = [point.copy() for point in vert_cos[left_corner_index:right_corner_index + 1]]
    middle_metrics = _measure_chain_axis_metrics(middle_points, basis_u, basis_v)
    if middle_metrics is None:
        return None

    connector_length = middle_metrics["total_length"]
    connector_length_limit = max(0.04, min(chain_length * 0.05, 0.25))
    if connector_length > connector_length_limit:
        return None

    outer_support_target = max(0.03, connector_length * 2.0)
    left_outer = _measure_open_corner_side_support(
        vert_cos, left_corner_index, basis_u, basis_v, direction=-1, target_length=outer_support_target
    )
    right_outer = _measure_open_corner_side_support(
        vert_cos, right_corner_index, basis_u, basis_v, direction=+1, target_length=outer_support_target
    )
    if left_outer is None or right_outer is None:
        return None

    if left_outer["min_alignment"] < 0.90 or right_outer["min_alignment"] < 0.90:
        return None

    left_turn = math.degrees(left_outer["direction"].angle(middle_direction["direction"], 0.0))
    right_turn = math.degrees(middle_direction["direction"].angle(right_outer["direction"], 0.0))
    if left_turn < CORNER_ANGLE_THRESHOLD_DEG or right_turn < CORNER_ANGLE_THRESHOLD_DEG:
        return None

    middle_role = _classify_chain_frame_role(middle_points, basis_u, basis_v, strict_guards=False)
    if middle_role == FrameRole.FREE:
        return None

    return {
        "connector_length": connector_length,
        "middle_role": middle_role,
        "left_turn": left_turn,
        "right_turn": right_turn,
    }


def _filter_open_chain_corner_indices_with_support(raw_chain, corner_indices, basis_u, basis_v):
    """Keep only hard open-border corners that create meaningful split segments."""

    if not corner_indices:
        return []

    vert_cos = raw_chain.vert_cos
    supported_corner_indices = []
    for corner_index in corner_indices:
        support = _open_corner_has_support(vert_cos, corner_index, basis_u, basis_v)
        if support is None:
            trace_console(f"[CFTUV][CornerDetect] skip idx={corner_index} reason=insufficient_support")
            continue

        trace_console(
            f"[CFTUV][CornerDetect] keep idx={corner_index} support_turn={support['support_turn']:.1f} "
            f"support_prev={support['prev_support']['support_length']:.4f} "
            f"support_next={support['next_support']['support_length']:.4f} "
            f"align_prev={support['prev_support']['min_alignment']:.3f} "
            f"align_next={support['next_support']['min_alignment']:.3f}"
        )
        supported_corner_indices.append(corner_index)

    supported_corner_set = set(supported_corner_indices)
    for corner_pos in range(len(corner_indices) - 1):
        left_corner_index = corner_indices[corner_pos]
        right_corner_index = corner_indices[corner_pos + 1]
        if left_corner_index in supported_corner_set and right_corner_index in supported_corner_set:
            continue

        pair_support = _open_corner_pair_has_support(
            vert_cos, left_corner_index, right_corner_index, basis_u, basis_v
        )
        if pair_support is None:
            continue

        trace_console(
            f"[CFTUV][CornerDetect] keep_pair idxs=[{left_corner_index}, {right_corner_index}] "
            f"connector_role={pair_support['middle_role'].value} "
            f"connector_length={pair_support['connector_length']:.4f} "
            f"turns={pair_support['left_turn']:.1f}/{pair_support['right_turn']:.1f}"
        )
        supported_corner_set.add(left_corner_index)
        supported_corner_set.add(right_corner_index)

    supported_corner_indices = sorted(supported_corner_set)
    if len(supported_corner_indices) < 2:
        return supported_corner_indices

    chain_length = _measure_polyline_length(vert_cos)
    micro_gap_length_limit = max(0.04, min(chain_length * 0.05, 0.20))

    filtered_corner_indices = list(supported_corner_indices)
    changed = True
    while changed and len(filtered_corner_indices) >= 2:
        changed = False
        sub_chains = _split_open_chain_at_corners(raw_chain, filtered_corner_indices)
        if len(sub_chains) < 3:
            break

        segment_lengths = [_measure_polyline_length(sub_chain.vert_cos) for sub_chain in sub_chains]
        segment_roles = [
            _classify_chain_frame_role(sub_chain.vert_cos, basis_u, basis_v, strict_guards=False)
            for sub_chain in sub_chains
        ]

        for segment_index in range(1, len(sub_chains) - 1):
            segment_length = segment_lengths[segment_index]
            if segment_length > micro_gap_length_limit:
                continue

            prev_length = segment_lengths[segment_index - 1]
            next_length = segment_lengths[segment_index + 1]
            prev_role = segment_roles[segment_index - 1]
            curr_role = segment_roles[segment_index]
            next_role = segment_roles[segment_index + 1]

            has_supported_neighbors = (
                prev_length >= segment_length * 2.0 and next_length >= segment_length * 2.0
            )
            same_role_bridge = (
                prev_role == next_role
                and prev_role in (FrameRole.H_FRAME, FrameRole.V_FRAME)
                and curr_role in (FrameRole.FREE, prev_role)
            )
            if not (has_supported_neighbors and same_role_bridge):
                continue

            left_corner = filtered_corner_indices[segment_index - 1]
            right_corner = filtered_corner_indices[segment_index]
            trace_console(
                f"[CFTUV][BorderSplit] drop micro_gap corners=[{left_corner}, {right_corner}] "
                f"role={curr_role.value} bridge={prev_role.value}->{next_role.value} "
                f"length={segment_length:.4f} prev={prev_length:.4f} next={next_length:.4f}"
            )
            del filtered_corner_indices[segment_index - 1:segment_index + 1]
            changed = True
            break

    return filtered_corner_indices


def _split_open_chain_at_corners(raw_chain, corner_indices):
    """Split one open raw chain at interior corner indices into multiple sub-chains."""

    vert_indices = raw_chain.vert_indices
    vert_cos = raw_chain.vert_cos
    edge_indices = raw_chain.edge_indices
    side_face_indices = raw_chain.side_face_indices
    neighbor = raw_chain.neighbor
    boundaries = [0] + sorted(corner_indices) + [len(vert_indices) - 1]
    parent_start_loop_index = raw_chain.start_loop_index

    sub_chains = []
    for seg_idx in range(len(boundaries) - 1):
        start = boundaries[seg_idx]
        end = boundaries[seg_idx + 1]
        if end <= start:
            continue

        seg_verts = vert_indices[start:end + 1]
        seg_cos = [co.copy() for co in vert_cos[start:end + 1]]
        seg_edges = edge_indices[start:end] if start < len(edge_indices) else []
        seg_sides = side_face_indices[start:end + 1] if start < len(side_face_indices) else []

        if len(seg_verts) < 2:
            continue

        sub_chains.append(
            _RawBoundaryChain(
                vert_indices=seg_verts,
                vert_cos=seg_cos,
                edge_indices=seg_edges,
                side_face_indices=seg_sides,
                neighbor=neighbor,
                is_closed=False,
                start_loop_index=parent_start_loop_index + start,
                end_loop_index=parent_start_loop_index + end,
                is_corner_split=True,
            )
        )

    return sub_chains if sub_chains else [raw_chain]


def _split_border_chains_by_corners(raw_chains, basis_u, basis_v, bm=None):
    """Split open MESH_BORDER chains at strong geometric corners."""

    result = []
    for raw_chain in raw_chains:
        neighbor = int(raw_chain.neighbor)
        is_closed = bool(raw_chain.is_closed)

        if neighbor != NB_MESH_BORDER or is_closed:
            result.append(raw_chain)
            continue

        vert_cos = raw_chain.vert_cos
        if len(vert_cos) < 3:
            result.append(raw_chain)
            continue

        detection = _detect_open_border_corners(
            raw_chain,
            basis_u,
            basis_v,
            bm=bm,
            min_spacing=1,
        )

        corner_indices = list(detection.supported_indices)
        points_2d = [Vector((co.dot(basis_u), co.dot(basis_v))) for co in vert_cos]
        trace_console(
            f"[CFTUV][BorderSplit] MESH_BORDER chain verts={len(vert_cos)} "
            f"candidates={list(detection.candidate_indices)} corners={corner_indices} "
            f"pts={[(round(p.x, 3), round(p.y, 3)) for p in points_2d]}"
        )

        if not corner_indices:
            result.append(raw_chain)
            continue

        sub_chains = _split_open_chain_at_corners(raw_chain, corner_indices)
        trace_console(f"[CFTUV][BorderSplit] Split into {len(sub_chains)} sub-chains")
        result.extend(sub_chains)

    return result


def _try_geometric_outer_loop_split(raw_loop, raw_chains, basis_u, basis_v, bm=None):
    """Fallback split for isolated OUTER loops that collapsed into one chain."""

    loop_kind = raw_loop.kind
    if not isinstance(loop_kind, LoopKind):
        loop_kind = LoopKind(loop_kind)

    if loop_kind != LoopKind.OUTER or len(raw_chains) != 1:
        return raw_chains

    raw_chain = raw_chains[0]
    if not raw_chain.is_closed:
        return raw_chains

    split_indices = _detect_closed_loop_corner_indices(
        raw_loop.vert_cos,
        basis_u,
        basis_v,
        loop_vert_indices=raw_loop.vert_indices,
        bm=bm,
    )
    trace_console(f"[CFTUV][GeoSplit] split_indices={len(split_indices)} indices={split_indices}")
    if len(split_indices) < 4:
        trace_console("[CFTUV][GeoSplit] BAIL: <4 split indices")
        return raw_chains

    derived_raw_chains = _split_closed_loop_by_corner_indices(
        raw_loop,
        split_indices,
        int(raw_chain.neighbor),
    )
    trace_console(f"[CFTUV][GeoSplit] derived_chains={len(derived_raw_chains)}")
    if len(derived_raw_chains) < 4:
        trace_console("[CFTUV][GeoSplit] BAIL: <4 derived chains")
        return raw_chains

    derived_roles = [
        _classify_chain_frame_role(derived_raw_chain.vert_cos, basis_u, basis_v, strict_guards=False)
        for derived_raw_chain in derived_raw_chains
    ]
    trace_console(f"[CFTUV][GeoSplit] derived_roles={[role.value for role in derived_roles]}")
    non_free_count = sum(1 for role in derived_roles if role != FrameRole.FREE)
    if non_free_count < 1:
        trace_console("[CFTUV][GeoSplit] BAIL: all derived chains are FREE")
        return raw_chains
    trace_console(f"[CFTUV][GeoSplit] SUCCESS: split into {len(derived_raw_chains)} chains, non_free={non_free_count}")

    return derived_raw_chains
