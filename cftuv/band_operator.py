"""Dedicated narrow operator for BAND-shaped patches.

A BAND patch has exactly 4 boundary chains: 2 SIDE chains (long, parallel,
same frame role) and 2 CAP chains (short, connecting the SIDEs).

The operator projects each SIDE chain's 3D vertices onto the resolved band axis
to compute spine-projection stations, then places both SIDEs with shared
stations so they share the same s-domain.  CAP chains are placed with
arc-length-proportional interpolation between the SIDE endpoints.
"""

from __future__ import annotations

from typing import Optional

from mathutils import Vector

try:
    from .console_debug import trace_console
    from .frontier_place import (
        _build_frame_chain_from_stations,
        _cf_chain_total_length,
        _cf_determine_direction_for_role,
        _normalized_stations_from_edge_lengths,
        _snap_direction_to_role,
        _normalize_direction,
        _default_role_direction,
        _chain_edge_lengths,
        _cf_chain_source_points,
    )
    from .frontier_state import FrontierRuntimePolicy
    from .model import (
        AxisAuthorityKind,
        BoundaryChain,
        ChainRef,
        FrameRole,
        ParameterAuthorityKind,
        PatchGraph,
        PatchNode,
        PlacementSourceKind,
        ScaffoldChainPlacement,
        ScaffoldPointKey,
        SpanAuthorityKind,
        StationAuthorityKind,
    )
    from .structural_tokens import (
        build_loop_signature,
        ChainRoleClass,
        LoopSignature,
        PatchShapeClass,
    )
except ImportError:
    from console_debug import trace_console
    from frontier_place import (
        _build_frame_chain_from_stations,
        _cf_chain_total_length,
        _cf_determine_direction_for_role,
        _normalized_stations_from_edge_lengths,
        _snap_direction_to_role,
        _normalize_direction,
        _default_role_direction,
        _chain_edge_lengths,
        _cf_chain_source_points,
    )
    from frontier_state import FrontierRuntimePolicy
    from model import (
        AxisAuthorityKind,
        BoundaryChain,
        ChainRef,
        FrameRole,
        ParameterAuthorityKind,
        PatchGraph,
        PatchNode,
        PlacementSourceKind,
        ScaffoldChainPlacement,
        ScaffoldPointKey,
        SpanAuthorityKind,
        StationAuthorityKind,
    )
    from structural_tokens import (
        build_loop_signature,
        ChainRoleClass,
        LoopSignature,
        PatchShapeClass,
    )


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _project_chain_stations_onto_axis(
    chain: BoundaryChain,
    node: PatchNode,
    axis_direction: Vector,
    final_scale: float,
) -> list[float]:
    """Project chain 3D vertices onto the resolved band axis direction.

    Returns a list of normalised stations in [0, 1] with length equal to
    len(chain.vert_cos).  The stations are forced monotone.
    """
    verts = chain.vert_cos
    n = len(verts)
    if n == 0:
        return []
    if n == 1:
        return [0.0]

    # Project each vertex onto the band axis using the patch local basis.
    axis_norm = axis_direction.normalized() if axis_direction.length > 1e-9 else Vector((1.0, 0.0))
    origin_3d = verts[0]

    raw: list[float] = []
    for v_3d in verts:
        delta_3d = v_3d - origin_3d
        u = delta_3d.dot(node.basis_u) * final_scale
        v = delta_3d.dot(node.basis_v) * final_scale
        v_uv = Vector((u, v))
        raw.append(v_uv.dot(axis_norm))

    # Shift so first projection == 0
    raw_min = raw[0]
    raw = [r - raw_min for r in raw]
    raw_max = raw[-1]

    # If the chord projects to near zero, fall back to edge-length stations.
    if abs(raw_max) < 1e-9:
        src_pts = _cf_chain_source_points(chain)
        edge_lens = _chain_edge_lengths(src_pts, final_scale)
        return _normalized_stations_from_edge_lengths(edge_lens)

    # Normalise to [0, 1].
    stations = [min(max(r / raw_max, 0.0), 1.0) for r in raw]

    # Enforce monotonicity (clamping only upward).
    prev = 0.0
    for i, s in enumerate(stations):
        s = max(s, prev)
        stations[i] = s
        prev = s
    stations[0] = 0.0
    stations[-1] = 1.0
    return stations


def _uniform_stations(n: int) -> list[float]:
    """Return n uniformly-spaced stations in [0, 1]."""
    if n <= 0:
        return []
    if n == 1:
        return [0.0]
    return [float(i) / float(n - 1) for i in range(n)]


def _lerp_stations(n: int) -> list[float]:
    """Alias for _uniform_stations — used for CAP chains."""
    return _uniform_stations(n)


def _make_chain_placement(
    chain_ref: ChainRef,
    chain: BoundaryChain,
    uv_points: list[Vector],
    role: FrameRole,
) -> ScaffoldChainPlacement:
    patch_id, loop_index, chain_index = chain_ref
    return ScaffoldChainPlacement(
        patch_id=patch_id,
        loop_index=loop_index,
        chain_index=chain_index,
        frame_role=role,
        axis_authority_kind=AxisAuthorityKind.NONE,
        span_authority_kind=SpanAuthorityKind.NONE,
        station_authority_kind=StationAuthorityKind.NONE,
        parameter_authority_kind=ParameterAuthorityKind.NONE,
        source_kind=PlacementSourceKind.CHAIN,
        anchor_count=0,
        primary_anchor_kind=PlacementSourceKind.CHAIN,
        points=tuple(
            (ScaffoldPointKey(patch_id, loop_index, chain_index, i), uv.copy())
            for i, uv in enumerate(uv_points)
        ),
    )


def _average_station_pair(
    s0: list[float],
    s1: list[float],
) -> Optional[list[float]]:
    """Return element-wise average of two equal-length station lists, or None."""
    if len(s0) != len(s1):
        return None
    n = len(s0)
    if n == 0:
        return None
    averaged = [(a + b) * 0.5 for a, b in zip(s0, s1)]
    averaged[0] = 0.0
    averaged[-1] = 1.0
    prev = 0.0
    for i, s in enumerate(averaged):
        s = max(s, prev)
        averaged[i] = s
        prev = s
    averaged[-1] = 1.0
    return averaged


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------


def band_operator(
    patch_id: int,
    graph: PatchGraph,
    runtime_policy: FrontierRuntimePolicy,
    loop_signature: LoopSignature,
    final_scale: float,
) -> Optional[list[ScaffoldChainPlacement]]:
    """Dedicated operator for BAND-shaped patches.

    Places the two SIDE chains with spine-projected stations (shared s-domain)
    and the two CAP chains with arc-length-proportional interpolation between
    the SIDE chain endpoints.

    Returns a list of ScaffoldChainPlacement records (one per chain) on
    success, or None on any failure (caller falls back to generic frontier).
    """
    try:
        return _band_operator_impl(patch_id, graph, runtime_policy, loop_signature, final_scale)
    except Exception as exc:
        trace_console(
            f"[CFTUV][BandOp] P{patch_id} fallback — exception: {exc}"
        )
        return None


def _band_operator_impl(
    patch_id: int,
    graph: PatchGraph,
    runtime_policy: FrontierRuntimePolicy,
    loop_signature: LoopSignature,
    final_scale: float,
) -> Optional[list[ScaffoldChainPlacement]]:
    node = graph.nodes.get(patch_id)
    if node is None:
        return None

    loop_index = loop_signature.loop_index
    if loop_index < 0 or loop_index >= len(node.boundary_loops):
        return None
    loop = node.boundary_loops[loop_index]

    # ---- Separate SIDE and CAP tokens -----------------------------------
    side_tokens = [t for t in loop_signature.chain_tokens if t.role_class == ChainRoleClass.SIDE]
    cap_tokens = [t for t in loop_signature.chain_tokens if t.role_class == ChainRoleClass.CAP]

    if len(side_tokens) != 2 or len(cap_tokens) != 2:
        return None

    # ---- Fetch actual BoundaryChain objects -----------------------------
    def _get_chain(token_chain_ref: ChainRef) -> Optional[BoundaryChain]:
        _, li, ci = token_chain_ref
        if li < 0 or li >= len(node.boundary_loops):
            return None
        lp = node.boundary_loops[li]
        if ci < 0 or ci >= len(lp.chains):
            return None
        return lp.chains[ci]

    side_chains = [_get_chain(t.chain_ref) for t in side_tokens]
    cap_chains = [_get_chain(t.chain_ref) for t in cap_tokens]

    if any(c is None for c in side_chains) or any(c is None for c in cap_chains):
        return None

    # ---- Resolve band axis direction from the first SIDE chain ----------
    side_role = side_tokens[0].effective_frame_role
    if side_role not in (FrameRole.H_FRAME, FrameRole.V_FRAME):
        trace_console(
            f"[CFTUV][BandOp] P{patch_id} fallback — SIDE role not H/V ({side_role})"
        )
        return None

    axis_direction = _snap_direction_to_role(
        _cf_determine_direction_for_role(side_chains[0], node, side_role),
        side_role,
    )
    axis_direction = _normalize_direction(axis_direction, _default_role_direction(side_role))

    # ---- Compute shared span --------------------------------------------
    span0 = _cf_chain_total_length(side_chains[0], final_scale)
    span1 = _cf_chain_total_length(side_chains[1], final_scale)
    shared_span = (span0 + span1) * 0.5
    if shared_span < 1e-9:
        return None

    # ---- Spine-project stations for each SIDE chain --------------------
    raw_stations_0 = _project_chain_stations_onto_axis(
        side_chains[0], node, axis_direction, final_scale
    )
    raw_stations_1 = _project_chain_stations_onto_axis(
        side_chains[1], node, axis_direction, final_scale
    )

    # Use averaged stations if both SIDEs have the same vertex count;
    # otherwise keep per-chain stations (different topology).
    if len(raw_stations_0) == len(raw_stations_1):
        shared_stations_0 = _average_station_pair(raw_stations_0, raw_stations_1)
        shared_stations_1 = shared_stations_0
        if shared_stations_0 is None:
            shared_stations_0 = raw_stations_0
            shared_stations_1 = raw_stations_1
    else:
        shared_stations_0 = raw_stations_0
        shared_stations_1 = raw_stations_1

    # ---- Place SIDE 0 at origin along axis ------------------------------
    side0_start = Vector((0.0, 0.0))
    side0_end = side0_start + axis_direction * shared_span

    side0_uvs = _build_frame_chain_from_stations(
        side0_start, side0_end, side_role, shared_stations_0
    )
    if not side0_uvs:
        return None

    # ---- Determine cross-axis offset for SIDE 1 -------------------------
    # Use CAP arc-lengths as a proxy for band width.
    cap_len_0 = _cf_chain_total_length(cap_chains[0], final_scale)
    cap_len_1 = _cf_chain_total_length(cap_chains[1], final_scale)
    width = (cap_len_0 + cap_len_1) * 0.5
    if width < 1e-9:
        width = shared_span * 0.25  # reasonable fallback

    # Perpendicular direction (cross-axis)
    if side_role == FrameRole.H_FRAME:
        cross_dir = Vector((0.0, 1.0))
    else:
        cross_dir = Vector((1.0, 0.0))

    side1_start = side0_start + cross_dir * width
    side1_end = side0_end + cross_dir * width

    side1_uvs = _build_frame_chain_from_stations(
        side1_start, side1_end, side_role, shared_stations_1
    )
    if not side1_uvs:
        return None

    # ---- Build SIDE placements ------------------------------------------
    side0_ref = side_tokens[0].chain_ref
    side1_ref = side_tokens[1].chain_ref

    side0_placement = _make_chain_placement(side0_ref, side_chains[0], side0_uvs, side_role)
    side1_placement = _make_chain_placement(side1_ref, side_chains[1], side1_uvs, side_role)

    # ---- Place CAP chains -----------------------------------------------
    # Each CAP connects one end of SIDE0 to the corresponding end of SIDE1.
    # We identify which CAP connects which end by checking shared corners.

    # SIDE0 endpoints: side0_start = side0_uvs[0], side0_end = side0_uvs[-1]
    # SIDE1 endpoints: side1_start = side1_uvs[0], side1_end = side1_uvs[-1]

    # Pair caps to ends: CAP that shares start_corner of SIDE0 → start end,
    # otherwise → end end.  Fall back to index order.

    def _cap_connects_start_end(
        cap_token,
        side0_chain: BoundaryChain,
        side1_chain: BoundaryChain,
    ) -> bool:
        """Guess whether cap connects the *start* ends of the SIDE chains."""
        cap_ref_li, cap_ci = cap_token.chain_ref[1], cap_token.chain_ref[2]
        if cap_ref_li < 0 or cap_ref_li >= len(node.boundary_loops):
            return True
        cap_chain = node.boundary_loops[cap_ref_li].chains[cap_ci]
        # Check corner overlap between cap and SIDE0 start corner.
        cap_corners = {cap_chain.start_corner_index, cap_chain.end_corner_index}
        side0_start_corner = side0_chain.start_corner_index
        if side0_start_corner >= 0 and side0_start_corner in cap_corners:
            return True
        return False

    cap0_token = cap_tokens[0]
    cap1_token = cap_tokens[1]

    cap0_at_start = _cap_connects_start_end(cap0_token, side_chains[0], side_chains[1])
    # cap0 is at-start → cap1 is at-end
    cap1_at_start = not cap0_at_start

    def _place_cap(
        cap_token,
        at_start: bool,
    ) -> Optional[ScaffoldChainPlacement]:
        cap_chain = _get_chain(cap_token.chain_ref)
        if cap_chain is None:
            return None
        n = len(cap_chain.vert_cos)
        if n == 0:
            return None

        if at_start:
            pt_a = side0_uvs[0]
            pt_b = side1_uvs[0]
        else:
            pt_a = side0_uvs[-1]
            pt_b = side1_uvs[-1]

        # Arc-length-proportional interpolation along the connecting segment.
        src_pts = _cf_chain_source_points(cap_chain)
        edge_lens = _chain_edge_lengths(src_pts, final_scale)
        stations = _normalized_stations_from_edge_lengths(edge_lens)

        # Build UV points by lerping between pt_a and pt_b.
        uvs = [pt_a.lerp(pt_b, s) for s in stations]
        if uvs:
            uvs[0] = pt_a.copy()
            uvs[-1] = pt_b.copy()

        cap_role = cap_chain.frame_role
        return _make_chain_placement(cap_token.chain_ref, cap_chain, uvs, cap_role)

    cap0_placement = _place_cap(cap0_token, cap0_at_start)
    cap1_placement = _place_cap(cap1_token, cap1_at_start)

    if cap0_placement is None or cap1_placement is None:
        return None

    placements = [side0_placement, side1_placement, cap0_placement, cap1_placement]

    trace_console(
        f"[CFTUV][BandOp] P{patch_id} OK "
        f"role={side_role.value} span={shared_span:.4f} width={width:.4f}"
    )
    return placements
