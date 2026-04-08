from __future__ import annotations

from dataclasses import replace
from types import MappingProxyType

try:
    from .constants import CORNER_ANGLE_THRESHOLD_DEG
    from .model import ChainNeighborKind, FrameRole, LoopKind, PatchType, WorldFacing
    from .analysis_records import (
        _FrameRun,
        _Junction,
        _JunctionStructuralKind,
        _JunctionStructuralRole,
        _LoopFrameRunBuildResult,
        _LoopDerivedTopologySummary,
        _PatchDerivedTopologySummary,
        _PatchGraphAggregateCounts,
        _PatchGraphDerivedTopology,
        _RunStructuralRole,
        JunctionPatchKey,
        RunKey,
    )
    from .analysis_frame_runs import _build_patch_graph_loop_frame_results
    from .analysis_junctions import _build_junction_run_refs_by_corner, _build_patch_graph_junctions
except ImportError:
    from constants import CORNER_ANGLE_THRESHOLD_DEG
    from model import ChainNeighborKind, FrameRole, LoopKind, PatchType, WorldFacing
    from analysis_records import (
        _FrameRun,
        _Junction,
        _JunctionStructuralKind,
        _JunctionStructuralRole,
        _LoopFrameRunBuildResult,
        _LoopDerivedTopologySummary,
        _PatchDerivedTopologySummary,
        _PatchGraphAggregateCounts,
        _PatchGraphDerivedTopology,
        _RunStructuralRole,
        JunctionPatchKey,
        RunKey,
    )
    from analysis_frame_runs import _build_patch_graph_loop_frame_results
    from analysis_junctions import _build_junction_run_refs_by_corner, _build_patch_graph_junctions


def _build_patch_topology_summaries(graph, loop_frame_results):
    """Build canonical per-patch and aggregate topology summaries."""

    patch_summaries: list[_PatchDerivedTopologySummary] = []

    walls = 0
    floors = 0
    slopes = 0
    singles = 0
    total_loops = 0
    total_chains = 0
    total_corners = 0
    total_sharp_corners = 0
    total_holes = 0
    total_h = 0
    total_v = 0
    total_free = 0
    total_up = 0
    total_down = 0
    total_side = 0
    total_patch_links = 0
    total_self_seams = 0
    total_mesh_borders = 0
    total_run_h = 0
    total_run_v = 0
    total_run_free = 0

    for patch_id in sorted(graph.nodes.keys()):
        node = graph.nodes[patch_id]
        patch_type = node.patch_type
        world_facing = node.world_facing

        if world_facing == WorldFacing.UP:
            total_up += 1
        elif world_facing == WorldFacing.DOWN:
            total_down += 1
        else:
            total_side += 1

        if patch_type == PatchType.WALL:
            walls += 1
        elif patch_type == PatchType.FLOOR:
            floors += 1
        elif patch_type == PatchType.SLOPE:
            slopes += 1

        if len(node.face_indices) == 1:
            singles += 1

        patch_chain_count = 0
        patch_corner_count = 0
        patch_hole_count = 0
        patch_run_count = 0
        patch_h = 0
        patch_v = 0
        patch_free = 0
        role_sequence: list[FrameRole] = []
        loop_kinds: list[LoopKind] = []
        loop_summaries: list[_LoopDerivedTopologySummary] = []

        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            loop_kind = boundary_loop.kind
            frame_runs = loop_frame_results.get(
                (patch_id, loop_index),
                _LoopFrameRunBuildResult(effective_roles=(), runs=()),
            ).runs

            total_loops += 1
            patch_chain_count += len(boundary_loop.chains)
            patch_corner_count += len(boundary_loop.corners)
            total_chains += len(boundary_loop.chains)
            total_corners += len(boundary_loop.corners)
            patch_run_count += len(frame_runs)
            loop_kinds.append(loop_kind)

            if loop_kind == LoopKind.HOLE:
                total_holes += 1
                patch_hole_count += 1

            total_sharp_corners += sum(
                1
                for corner in boundary_loop.corners
                if corner.turn_angle_deg >= CORNER_ANGLE_THRESHOLD_DEG
            )

            for chain in boundary_loop.chains:
                role_sequence.append(chain.frame_role)
                if chain.frame_role == FrameRole.H_FRAME:
                    total_h += 1
                    patch_h += 1
                elif chain.frame_role == FrameRole.V_FRAME:
                    total_v += 1
                    patch_v += 1
                else:
                    total_free += 1
                    patch_free += 1

                if chain.neighbor_kind == ChainNeighborKind.PATCH:
                    total_patch_links += 1
                elif chain.neighbor_kind == ChainNeighborKind.SEAM_SELF:
                    total_self_seams += 1
                else:
                    total_mesh_borders += 1

            for frame_run in frame_runs:
                if frame_run.dominant_role == FrameRole.H_FRAME:
                    total_run_h += 1
                elif frame_run.dominant_role == FrameRole.V_FRAME:
                    total_run_v += 1
                else:
                    total_run_free += 1

            loop_summaries.append(
                _LoopDerivedTopologySummary(
                    patch_id=patch_id,
                    loop_index=loop_index,
                    kind=loop_kind,
                    chain_count=len(boundary_loop.chains),
                    corner_count=len(boundary_loop.corners),
                    run_count=len(frame_runs),
                )
            )

        patch_summaries.append(
            _PatchDerivedTopologySummary(
                patch_id=patch_id,
                semantic_key=graph.get_patch_semantic_key(patch_id),
                patch_type=patch_type,
                world_facing=world_facing,
                face_count=len(node.face_indices),
                loop_kinds=tuple(loop_kinds),
                chain_count=patch_chain_count,
                corner_count=patch_corner_count,
                hole_count=patch_hole_count,
                run_count=patch_run_count,
                role_sequence=tuple(role_sequence),
                h_count=patch_h,
                v_count=patch_v,
                free_count=patch_free,
                loop_summaries=tuple(loop_summaries),
            )
        )

    aggregate_counts = _PatchGraphAggregateCounts(
        total_patches=len(graph.nodes),
        walls=walls,
        floors=floors,
        slopes=slopes,
        singles=singles,
        total_loops=total_loops,
        total_chains=total_chains,
        total_corners=total_corners,
        total_sharp_corners=total_sharp_corners,
        total_holes=total_holes,
        total_h=total_h,
        total_v=total_v,
        total_free=total_free,
        total_up=total_up,
        total_down=total_down,
        total_side=total_side,
        total_patch_links=total_patch_links,
        total_self_seams=total_self_seams,
        total_mesh_borders=total_mesh_borders,
        total_run_h=total_run_h,
        total_run_v=total_run_v,
        total_run_free=total_run_free,
    )
    return tuple(patch_summaries), aggregate_counts


def _clamp01(x):
    """Clamp a float to the [0.0, 1.0] range."""
    return max(0.0, min(1.0, x))


def _chain_polyline_length(chain) -> float:
    if len(chain.vert_cos) < 2:
        return 0.0
    return sum(
        (chain.vert_cos[index + 1] - chain.vert_cos[index]).length
        for index in range(len(chain.vert_cos) - 1)
    )


def _compute_neighbor_inherited_roles(graph):
    """For each FREE chain with a patch neighbor, check neighbor's frame role along shared boundary.

    Returns dict[ChainRef, (FrameRole, int)] mapping chain reference to
    (inherited_role, source_patch_id). Only includes FREE chains where the
    neighbor has a strong H/V chain along the same boundary.
    """

    inherited = {}
    for patch_id, node in graph.nodes.items():
        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            for chain_index, chain in enumerate(boundary_loop.chains):
                if chain.frame_role != FrameRole.FREE:
                    continue
                if chain.neighbor_patch_id < 0:
                    continue  # MESH_BORDER or SEAM_SELF — no inheritance

                neighbor_id = chain.neighbor_patch_id
                neighbor_node = graph.nodes.get(neighbor_id)
                if neighbor_node is None:
                    continue

                # Find matching chain in neighbor: points back to our patch, shares vertices
                my_verts = set(chain.vert_indices)
                for n_loop_idx, n_loop in enumerate(neighbor_node.boundary_loops):
                    for n_chain_idx, n_chain in enumerate(n_loop.chains):
                        if n_chain.neighbor_patch_id != patch_id:
                            continue
                        n_verts = set(n_chain.vert_indices)
                        if len(my_verts & n_verts) < 2:
                            continue
                        # Found matching neighbor chain
                        if n_chain.frame_role in (FrameRole.H_FRAME, FrameRole.V_FRAME):
                            ref = (patch_id, loop_index, chain_index)
                            inherited[ref] = (n_chain.frame_role, neighbor_id)
                        break  # one match per chain is enough

    return inherited


def _interpret_run_structural_roles(frame_runs_by_loop, junctions, neighbor_inherited_roles=None):
    """Assign structural roles to frame runs: spine candidates, ranks, opposing pairs.

    Spine axis = the FrameRole axis (H or V) with greater total run length in a patch.
    Spine candidates = runs on the spine axis, ranked by total_length (0=longest).
    Opposing = another spine run on the SAME effective axis in the SAME loop.

    FREE runs where all chains have the same inherited role from a neighbor are treated
    as effective H/V runs for spine detection. This enables strip detection on patches
    like arc walls where the long sides are FREE but aligned with a framed neighbor.
    """

    if neighbor_inherited_roles is None:
        neighbor_inherited_roles = {}

    # Build full run list with RunKey = (patch_id, loop_index, run_index)
    all_run_keys = []  # list of (RunKey, _FrameRun)
    for (patch_id, loop_index), runs in frame_runs_by_loop.items():
        for run_index, run in enumerate(runs):
            key = (patch_id, loop_index, run_index)
            all_run_keys.append((key, run))

    # Group by patch_id
    runs_by_patch = {}
    for key, run in all_run_keys:
        patch_id = key[0]
        runs_by_patch.setdefault(patch_id, []).append((key, run))

    result = {}

    for patch_id, patch_runs in runs_by_patch.items():
        # Determine effective role for each run (own role or inherited from neighbor)
        # A FREE run gets an inherited role only if ALL its chains share the same inherited role.
        run_effective_roles = {}  # RunKey -> (effective_role, inherited_role_or_None, inherited_from_or_None)
        for key, run in patch_runs:
            if run.dominant_role != FrameRole.FREE:
                run_effective_roles[key] = (run.dominant_role, None, None)
                continue

            # Check if all chains in this run have the same inherited role
            chain_inherited = []
            for chain_idx in run.chain_indices:
                chain_ref = (patch_id, key[1], chain_idx)  # (patch_id, loop_index, chain_index)
                inh = neighbor_inherited_roles.get(chain_ref)
                chain_inherited.append(inh)

            if chain_inherited and all(inh is not None for inh in chain_inherited):
                inherited_roles_set = set(inh[0] for inh in chain_inherited)
                if len(inherited_roles_set) == 1:
                    inh_role = chain_inherited[0][0]
                    inh_from = chain_inherited[0][1]
                    run_effective_roles[key] = (inh_role, inh_role, inh_from)
                    continue

            run_effective_roles[key] = (FrameRole.FREE, None, None)

        # Separate runs by effective role
        h_runs = [(k, r) for k, r in patch_runs if run_effective_roles[k][0] == FrameRole.H_FRAME]
        v_runs = [(k, r) for k, r in patch_runs if run_effective_roles[k][0] == FrameRole.V_FRAME]

        h_total = sum(r.total_length for _, r in h_runs)
        v_total = sum(r.total_length for _, r in v_runs)

        if h_total == 0.0 and v_total == 0.0:
            # All FREE with no inheritance — default roles
            for key, _run in patch_runs:
                result[key] = _RunStructuralRole(run_key=key)
            continue

        if h_total >= v_total and h_total > 0.0:
            spine_runs = h_runs
        else:
            spine_runs = v_runs

        spine_keys = {k for k, _r in spine_runs}
        non_spine_runs = [
            (k, r) for k, r in patch_runs
            if k not in spine_keys
        ]

        # Rank spine runs by total_length descending
        spine_runs_sorted = sorted(spine_runs, key=lambda kr: kr[1].total_length, reverse=True)

        # Build spine role assignments
        spine_roles = {}
        for rank, (key, run) in enumerate(spine_runs_sorted):
            # Find opposing run: same effective axis, same loop, closest total_length
            same_loop_candidates = [
                (k, r) for k, r in spine_runs_sorted
                if k != key and k[1] == key[1]  # same loop_index
            ]
            opposing_run_key = None
            side_pair_length_ratio = 0.0
            if same_loop_candidates:
                best = min(
                    same_loop_candidates,
                    key=lambda kr: abs(kr[1].total_length - run.total_length),
                )
                opp_key, opp_run = best
                len_a = run.total_length
                len_b = opp_run.total_length
                denom = max(len_a, len_b, 1e-8)
                opposing_run_key = opp_key
                side_pair_length_ratio = abs(len_a - len_b) / denom

            eff_role, inh_role, inh_from = run_effective_roles[key]
            spine_roles[key] = _RunStructuralRole(
                run_key=key,
                is_spine_candidate=True,
                spine_rank=rank,
                opposing_run_key=opposing_run_key,
                side_pair_length_ratio=side_pair_length_ratio,
                inherited_role=inh_role,
                inherited_from_patch_id=inh_from,
            )

        for key, _run in spine_runs_sorted:
            result[key] = spine_roles[key]

        # Non-spine runs
        for key, _run in non_spine_runs:
            eff_role, inh_role, inh_from = run_effective_roles[key]
            result[key] = _RunStructuralRole(
                run_key=key,
                inherited_role=inh_role,
                inherited_from_patch_id=inh_from,
            )

        # FREE runs without inheritance not yet in result
        for key, _run in patch_runs:
            if key not in result:
                result[key] = _RunStructuralRole(run_key=key)

    return result


def _interpret_junction_structural_roles(junctions, frame_runs_by_loop, run_structural_roles):
    """Assign structural roles to junctions in context of each patch they touch.

    Each junction vertex may appear in multiple patches. We produce one role per (vert_index, patch_id).
    """

    effective_role_by_chain_ref = {}
    for loop_key, runs in frame_runs_by_loop.items():
        patch_id, loop_index = loop_key
        for run_index, run in enumerate(runs):
            structural_role = run_structural_roles.get((patch_id, loop_index, run_index))
            effective_role = run.dominant_role
            if (
                effective_role == FrameRole.FREE
                and structural_role is not None
                and structural_role.inherited_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
            ):
                effective_role = structural_role.inherited_role
            for chain_index in run.chain_indices:
                effective_role_by_chain_ref[(patch_id, loop_index, chain_index)] = effective_role

    result = {}

    for junction in junctions:
        for patch_id in junction.patch_ids:
            # Filter chain_refs to only those for this patch
            patch_chain_refs = [
                cr for cr in junction.chain_refs if cr.patch_id == patch_id
            ]

            # Count corners in this patch
            patch_corner_refs = [c for c in junction.corner_refs if c.patch_id == patch_id]
            patch_valence = len(patch_corner_refs)

            if patch_valence == 0:
                continue

            kind = _JunctionStructuralKind.FREE
            dominant_axis = FrameRole.FREE
            supports_inherited_axis = False
            implied_turn = -1.0

            if patch_valence >= 3:
                kind = _JunctionStructuralKind.BRANCH

            elif patch_valence == 1:
                # Check for mesh border with h/v chain involvement
                has_mesh_border = any(
                    cr.neighbor_kind == ChainNeighborKind.MESH_BORDER
                    for cr in patch_chain_refs
                )
                has_hv = any(
                    cr.frame_role in (FrameRole.H_FRAME, FrameRole.V_FRAME)
                    for cr in patch_chain_refs
                )
                if has_mesh_border and has_hv:
                    kind = _JunctionStructuralKind.TERMINAL
                else:
                    # Determine from prev/next chain roles at this corner
                    corner_ref = patch_corner_refs[0]
                    prev_chain_refs = [
                        cr for cr in patch_chain_refs
                        if cr.chain_index == corner_ref.prev_chain_index
                    ]
                    next_chain_refs = [
                        cr for cr in patch_chain_refs
                        if cr.chain_index == corner_ref.next_chain_index
                    ]
                    prev_raw_role = prev_chain_refs[0].frame_role if prev_chain_refs else FrameRole.FREE
                    next_raw_role = next_chain_refs[0].frame_role if next_chain_refs else FrameRole.FREE
                    prev_role = effective_role_by_chain_ref.get(
                        (patch_id, corner_ref.loop_index, corner_ref.prev_chain_index),
                        prev_raw_role,
                    )
                    next_role = effective_role_by_chain_ref.get(
                        (patch_id, corner_ref.loop_index, corner_ref.next_chain_index),
                        next_raw_role,
                    )

                    prev_hv = prev_role in (FrameRole.H_FRAME, FrameRole.V_FRAME)
                    next_hv = next_role in (FrameRole.H_FRAME, FrameRole.V_FRAME)
                    inherited_hv = (
                        (prev_hv and prev_raw_role == FrameRole.FREE)
                        or (next_hv and next_raw_role == FrameRole.FREE)
                    )

                    if prev_hv and next_hv:
                        if prev_role == next_role:
                            kind = _JunctionStructuralKind.CONTINUATION
                            dominant_axis = prev_role
                            supports_inherited_axis = inherited_hv
                            implied_turn = 0.0
                        else:
                            kind = _JunctionStructuralKind.TURN
                            implied_turn = 90.0
                    elif prev_hv != next_hv and inherited_hv:
                        kind = _JunctionStructuralKind.BRIDGE
                        dominant_axis = prev_role if prev_hv else next_role
                    elif inherited_hv:
                        kind = _JunctionStructuralKind.AMBIGUOUS
                    else:
                        kind = _JunctionStructuralKind.FREE

            elif patch_valence == 2:
                kind = _JunctionStructuralKind.AMBIGUOUS

            # Find spine_run_key: look at run_endpoint_refs for this patch
            spine_run_key = None
            for ref in junction.run_endpoint_refs:
                if ref.patch_id != patch_id:
                    continue
                run_key = (ref.patch_id, ref.loop_index, ref.run_index)
                role = run_structural_roles.get(run_key)
                if role is not None and role.is_spine_candidate:
                    spine_run_key = run_key
                    break

            junc_key = (junction.vert_index, patch_id)
            result[junc_key] = _JunctionStructuralRole(
                vert_index=junction.vert_index,
                patch_id=patch_id,
                kind=kind,
                spine_run_key=spine_run_key,
                dominant_axis=dominant_axis,
                supports_inherited_axis=supports_inherited_axis,
                implied_turn=implied_turn,
            )

    return result


def _derive_patch_structural_summary(graph, frame_runs_by_loop, run_structural_roles, junction_structural_roles):
    """Derive per-patch structural fields: spine info, strip_confidence, straighten_eligible.

    Returns dict[patch_id, dict] with field values to merge into _PatchDerivedTopologySummary.
    """

    result = {}

    for patch_id in graph.nodes:
        node = graph.nodes[patch_id]

        # Collect spine runs for this patch
        spine_roles = [
            role for key, role in run_structural_roles.items()
            if key[0] == patch_id and role.is_spine_candidate
        ]
        spine_roles_sorted = sorted(spine_roles, key=lambda r: r.spine_rank)

        # Derive spine_run_indices in spine_rank order
        if spine_roles_sorted:
            spine_run_indices = tuple(r.run_key[2] for r in spine_roles_sorted)
            primary_role = spine_roles_sorted[0]
            # Get the actual _FrameRun for spine_axis
            primary_run_key = primary_role.run_key
            primary_loop_key = (primary_run_key[0], primary_run_key[1])
            primary_runs_tuple = frame_runs_by_loop.get(primary_loop_key, ())
            if primary_run_key[2] < len(primary_runs_tuple):
                spine_axis = primary_runs_tuple[primary_run_key[2]].dominant_role
            else:
                spine_axis = FrameRole.FREE
        else:
            spine_run_indices = ()
            spine_axis = FrameRole.FREE

        inherited_spine_count = sum(
            1 for role in spine_roles_sorted
            if role.inherited_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
        )
        inherited_axes = {
            role.inherited_role
            for role in spine_roles_sorted
            if role.inherited_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
        }
        inherited_axis_candidate = (
            next(iter(inherited_axes))
            if len(inherited_axes) == 1 else
            FrameRole.FREE
        )
        axis_candidate = (
            spine_axis
            if spine_axis in {FrameRole.H_FRAME, FrameRole.V_FRAME}
            else inherited_axis_candidate
        )
        band_axis_candidate = FrameRole.FREE

        # spine_length = sum of total_length of all spine runs
        spine_length = 0.0
        for role in spine_roles_sorted:
            rk = role.run_key
            loop_key = (rk[0], rk[1])
            runs_tuple = frame_runs_by_loop.get(loop_key, ())
            if rk[2] < len(runs_tuple):
                spine_length += runs_tuple[rk[2]].total_length

        # Collect junction structural roles for this patch
        patch_junction_roles = [
            jrole for (vi, pid), jrole in junction_structural_roles.items()
            if pid == patch_id
        ]
        terminal_count = sum(
            1 for jr in patch_junction_roles
            if jr.kind == _JunctionStructuralKind.TERMINAL
        )
        branch_count = sum(
            1 for jr in patch_junction_roles
            if jr.kind == _JunctionStructuralKind.BRANCH
        )
        junction_axes = {
            jr.dominant_axis
            for jr in patch_junction_roles
            if jr.supports_inherited_axis and jr.dominant_axis in {FrameRole.H_FRAME, FrameRole.V_FRAME}
        }
        junction_supported_axis = (
            next(iter(junction_axes))
            if len(junction_axes) == 1 else
            FrameRole.FREE
        )

        # Compute strip_confidence
        perimeter = node.perimeter if node.perimeter > 0.0 else sum(
            runs_tuple[ri].total_length
            for (pid, li), runs_tuple in frame_runs_by_loop.items()
            if pid == patch_id
            for ri in range(len(runs_tuple))
        )
        spine_ratio = spine_length / max(perimeter, 1e-8)
        spine_score = min(1.0, spine_ratio / 0.45)

        # Side confidence from spine runs with opposing pairs
        side_pair_ratios = [
            role.side_pair_length_ratio
            for role in spine_roles_sorted
            if role.opposing_run_key is not None
        ]
        if side_pair_ratios:
            side_confidence = 1.0 - (sum(side_pair_ratios) / len(side_pair_ratios))
        else:
            side_confidence = 0.0
        single_sided_inherited_support = (
            spine_axis == FrameRole.FREE
            and inherited_spine_count == 1
            and axis_candidate in {FrameRole.H_FRAME, FrameRole.V_FRAME}
            and side_confidence < 0.35
        )

        band_cap_count = 0
        band_side_candidate_count = 0
        band_opposite_cap_length_ratio = 0.0
        band_width_stability = 0.0
        band_candidate = False
        band_confirmed_for_runtime = False
        band_rejected_reason = ""
        band_requires_intervention = False
        band_intervention_reject_reason = ""

        outer_loop_index = next(
            (loop_index for loop_index, boundary_loop in enumerate(node.boundary_loops) if boundary_loop.kind == LoopKind.OUTER),
            -1,
        )
        if outer_loop_index >= 0:
            outer_loop = node.boundary_loops[outer_loop_index]
            outer_chains = tuple(outer_loop.chains)
            chain_count = len(outer_chains)
            chain_lengths = tuple(_chain_polyline_length(chain) for chain in outer_chains)
            mean_chain_length = (sum(chain_lengths) / chain_count) if chain_count > 0 else 0.0
            effective_role_by_chain_index = {}
            for chain_index, chain in enumerate(outer_chains):
                effective_role = chain.frame_role
                if effective_role == FrameRole.FREE:
                    for run_index, run in enumerate(frame_runs_by_loop.get((patch_id, outer_loop_index), ())):
                        if chain_index not in run.chain_indices:
                            continue
                        run_role = run_structural_roles.get((patch_id, outer_loop_index, run_index))
                        if run_role is not None and run_role.inherited_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
                            effective_role = run_role.inherited_role
                        break
                effective_role_by_chain_index[chain_index] = effective_role

            cap_candidate_indices = [
                chain_index
                for chain_index, chain in enumerate(outer_chains)
                if (
                    (
                        effective_role_by_chain_index.get(chain_index, FrameRole.FREE) in {FrameRole.H_FRAME, FrameRole.V_FRAME}
                        or (chain.frame_role == FrameRole.FREE and (len(chain.vert_cos) <= 2 or len(chain.edge_indices) <= 1))
                    )
                    and chain_lengths[chain_index] > 0.0
                    and mean_chain_length > 0.0
                    and chain_lengths[chain_index] <= mean_chain_length * 0.5
                )
            ]
            side_candidate_indices = [
                chain_index
                for chain_index, chain in enumerate(outer_chains)
                if (
                    chain.frame_role == FrameRole.FREE
                    and chain_lengths[chain_index] >= mean_chain_length * 1.2
                    and any(
                        ((chain_index - cap_index) % max(chain_count, 1) == 1)
                        or ((cap_index - chain_index) % max(chain_count, 1) == 1)
                        for cap_index in cap_candidate_indices
                    )
                )
            ]

            band_cap_count = len(cap_candidate_indices)
            band_side_candidate_count = len(side_candidate_indices)

            side_axes = set()
            for side_chain_index in side_candidate_indices:
                side_chain = outer_chains[side_chain_index]
                if len(side_chain.vert_cos) < 2:
                    continue
                chord = side_chain.vert_cos[-1] - side_chain.vert_cos[0]
                u_span = abs(chord.dot(node.basis_u))
                v_span = abs(chord.dot(node.basis_v))
                if max(u_span, v_span) <= 1e-8:
                    continue
                side_axes.add(FrameRole.H_FRAME if u_span >= v_span else FrameRole.V_FRAME)
            if len(side_axes) == 1:
                band_axis_candidate = next(iter(side_axes))

            supported_band_axis = (
                band_axis_candidate
                if band_axis_candidate in {FrameRole.H_FRAME, FrameRole.V_FRAME} else
                axis_candidate
                if axis_candidate in {FrameRole.H_FRAME, FrameRole.V_FRAME} else
                junction_supported_axis
                if junction_supported_axis in {FrameRole.H_FRAME, FrameRole.V_FRAME} else
                FrameRole.FREE
            )

            if axis_candidate == FrameRole.FREE and supported_band_axis in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
                axis_candidate = supported_band_axis

            if band_cap_count >= 2:
                best_cap_ratio = 0.0
                for left_index in range(len(cap_candidate_indices) - 1):
                    for right_index in range(left_index + 1, len(cap_candidate_indices)):
                        cap_a = cap_candidate_indices[left_index]
                        cap_b = cap_candidate_indices[right_index]
                        separation = min(
                            (cap_b - cap_a) % chain_count,
                            (cap_a - cap_b) % chain_count,
                        )
                        if separation < 2:
                            continue
                        len_a = chain_lengths[cap_a]
                        len_b = chain_lengths[cap_b]
                        if max(len_a, len_b) <= 1e-8:
                            continue
                        similarity = 1.0 - abs(len_a - len_b) / max(len_a, len_b)
                        best_cap_ratio = max(best_cap_ratio, _clamp01(similarity))
                band_opposite_cap_length_ratio = best_cap_ratio

            if band_side_candidate_count >= 2:
                side_lengths = sorted(
                    (chain_lengths[chain_index] for chain_index in side_candidate_indices),
                    reverse=True,
                )
                len_a, len_b = side_lengths[:2]
                if max(len_a, len_b) > 1e-8:
                    band_width_stability = _clamp01(1.0 - abs(len_a - len_b) / max(len_a, len_b))

            band_candidate = bool(
                branch_count == 0
                and band_cap_count >= 2
                and band_side_candidate_count >= 2
                and band_opposite_cap_length_ratio >= 0.7
                and band_width_stability >= 0.65
                and supported_band_axis in {FrameRole.H_FRAME, FrameRole.V_FRAME}
            )

            runtime_band_axis = supported_band_axis
            runtime_role_by_chain_index = {}
            if runtime_band_axis in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
                orthogonal_axis = FrameRole.V_FRAME if runtime_band_axis == FrameRole.H_FRAME else FrameRole.H_FRAME
                for chain_index, chain in enumerate(outer_chains):
                    runtime_role = effective_role_by_chain_index.get(chain_index, chain.frame_role)
                    if runtime_role == FrameRole.FREE:
                        if chain_index in side_candidate_indices:
                            runtime_role = runtime_band_axis
                        elif chain_index in cap_candidate_indices:
                            runtime_role = orthogonal_axis
                    runtime_role_by_chain_index[chain_index] = runtime_role

            runtime_role_sequence = tuple(
                runtime_role_by_chain_index.get(chain_index, FrameRole.FREE)
                for chain_index in range(chain_count)
            )
            runtime_side_indices = []
            runtime_cap_indices = []
            if runtime_band_axis in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
                orthogonal_axis = FrameRole.V_FRAME if runtime_band_axis == FrameRole.H_FRAME else FrameRole.H_FRAME
                runtime_side_indices = [
                    chain_index
                    for chain_index, runtime_role in enumerate(runtime_role_sequence)
                    if runtime_role == runtime_band_axis
                ]
                runtime_cap_indices = [
                    chain_index
                    for chain_index, runtime_role in enumerate(runtime_role_sequence)
                    if runtime_role == orthogonal_axis
                ]

            runtime_side_pair_similarity = 0.0
            if len(runtime_side_indices) == 2:
                len_a = chain_lengths[runtime_side_indices[0]]
                len_b = chain_lengths[runtime_side_indices[1]]
                if max(len_a, len_b) > 1e-8:
                    runtime_side_pair_similarity = _clamp01(1.0 - abs(len_a - len_b) / max(len_a, len_b))

            runtime_cap_pair_similarity = 0.0
            if len(runtime_cap_indices) == 2:
                len_a = chain_lengths[runtime_cap_indices[0]]
                len_b = chain_lengths[runtime_cap_indices[1]]
                if max(len_a, len_b) > 1e-8:
                    runtime_cap_pair_similarity = _clamp01(1.0 - abs(len_a - len_b) / max(len_a, len_b))

            raw_outer_role_sequence = tuple(chain.frame_role for chain in outer_chains)
            raw_outer_free_count = sum(1 for role in raw_outer_role_sequence if role == FrameRole.FREE)
            raw_outer_hv_resolved = all(role in {FrameRole.H_FRAME, FrameRole.V_FRAME} for role in raw_outer_role_sequence)
            mesh_border_chain_count = sum(
                1 for chain in outer_chains
                if chain.neighbor_kind == ChainNeighborKind.MESH_BORDER
            )

            runtime_role_counts_ok = (
                runtime_role_sequence.count(FrameRole.H_FRAME) == 2
                and runtime_role_sequence.count(FrameRole.V_FRAME) == 2
                and runtime_role_sequence.count(FrameRole.FREE) == 0
            )
            runtime_role_pattern_ok = (
                chain_count == 4
                and runtime_role_counts_ok
                and runtime_role_sequence[0] == runtime_role_sequence[2]
                and runtime_role_sequence[1] == runtime_role_sequence[3]
                and runtime_role_sequence[0] != runtime_role_sequence[1]
            )

            if any(boundary_loop.kind == LoopKind.HOLE for boundary_loop in node.boundary_loops):
                band_rejected_reason = "has_holes"
            elif branch_count > 0:
                band_rejected_reason = "branch_junctions"
            elif chain_count != 4:
                band_rejected_reason = "outer_chain_count_not_four"
            elif runtime_band_axis not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
                band_rejected_reason = "missing_runtime_axis"
            elif len(runtime_cap_indices) != 2:
                band_rejected_reason = (
                    "missing_caps" if len(runtime_cap_indices) < 2 else "ambiguous_caps"
                )
            elif len(runtime_side_indices) != 2:
                band_rejected_reason = (
                    "missing_sides" if len(runtime_side_indices) < 2 else "ambiguous_sides"
                )
            elif runtime_cap_pair_similarity < 0.80:
                band_rejected_reason = "weak_cap_pair"
            elif runtime_side_pair_similarity < 0.80:
                band_rejected_reason = "weak_side_pair"
            elif not runtime_role_pattern_ok:
                band_rejected_reason = "runtime_role_pattern_mismatch"
            else:
                band_confirmed_for_runtime = True
                band_rejected_reason = ""
                if raw_outer_free_count > 0 or inherited_spine_count > 0 or single_sided_inherited_support:
                    band_requires_intervention = True
                    band_intervention_reject_reason = ""
                elif raw_outer_hv_resolved and runtime_role_pattern_ok and mesh_border_chain_count >= 2:
                    band_requires_intervention = False
                    band_intervention_reject_reason = "border_anchored_hv_frame"
                elif raw_outer_hv_resolved and runtime_role_pattern_ok:
                    band_requires_intervention = False
                    band_intervention_reject_reason = "already_resolved_hv_frame"
                else:
                    band_requires_intervention = True
                    band_intervention_reject_reason = ""

        # Bbox elongation: project boundary verts onto basis_u / basis_v
        basis_u = node.basis_u
        basis_v = node.basis_v
        mesh_verts = node.mesh_verts
        if mesh_verts:
            u_coords = [v.dot(basis_u) for v in mesh_verts]
            v_coords = [v.dot(basis_v) for v in mesh_verts]
            w = max(u_coords) - min(u_coords)
            h = max(v_coords) - min(v_coords)
        else:
            w = 0.0
            h = 0.0
        long_span = max(w, h)
        short_span = min(w, h)
        elongation = _clamp01(1.0 - short_span / max(long_span, 1e-8))

        branch_penalty_factor = 1.0 if branch_count == 0 else 0.0
        raw_confidence = (
            0.35 * spine_score
            + 0.25 * elongation
            + 0.25 * side_confidence
            + 0.15 * branch_penalty_factor
            - 0.30 * branch_count
        )
        strip_confidence = _clamp01(raw_confidence)

        straighten_eligible = (
            strip_confidence > 0.6
            and branch_count == 0
            and terminal_count >= 1
        )

        result[patch_id] = {
            "spine_run_indices": spine_run_indices,
            "spine_axis": spine_axis,
            "axis_candidate": axis_candidate,
            "junction_supported_axis": junction_supported_axis,
            "spine_length": spine_length,
            "inherited_spine_count": inherited_spine_count,
            "single_sided_inherited_support": single_sided_inherited_support,
            "band_cap_count": band_cap_count,
            "band_side_candidate_count": band_side_candidate_count,
            "band_opposite_cap_length_ratio": band_opposite_cap_length_ratio,
            "band_width_stability": band_width_stability,
            "band_candidate": band_candidate,
            "band_confirmed_for_runtime": band_confirmed_for_runtime,
            "band_rejected_reason": band_rejected_reason,
            "band_requires_intervention": band_requires_intervention,
            "band_intervention_reject_reason": band_intervention_reject_reason,
            "terminal_count": terminal_count,
            "branch_count": branch_count,
            "strip_confidence": strip_confidence,
            "straighten_eligible": straighten_eligible,
        }

        # DEBUG: temporary diagnostic print
        inh_tag = f" inherited_spines={inherited_spine_count}" if inherited_spine_count > 0 else ""
        axis_tag = (
            f" axis_candidate={axis_candidate}"
            if axis_candidate in {FrameRole.H_FRAME, FrameRole.V_FRAME}
            else ""
        )
        junc_tag = (
            f" junction_axis={junction_supported_axis}"
            if junction_supported_axis in {FrameRole.H_FRAME, FrameRole.V_FRAME}
            else ""
        )
        ss_tag = " single_sided=Y" if single_sided_inherited_support else ""
        band_tag = (
            f" band:caps={band_cap_count}/sides={band_side_candidate_count}"
            f"/capr={band_opposite_cap_length_ratio:.2f}/w={band_width_stability:.2f}"
            if band_cap_count > 0 or band_side_candidate_count > 0
            else ""
        )
        band_candidate_tag = " band_candidate=Y" if band_candidate else ""
        band_runtime_tag = " band_runtime=Y" if band_confirmed_for_runtime else ""
        band_reject_tag = (
            f" band_rt_reject={band_rejected_reason}"
            if not band_confirmed_for_runtime and band_rejected_reason
            else ""
        )
        band_intervene_tag = " band_intervene=Y" if band_requires_intervention else ""
        band_intervene_reject_tag = (
            f" band_int_reject={band_intervention_reject_reason}"
            if band_confirmed_for_runtime and not band_requires_intervention and band_intervention_reject_reason
            else ""
        )
        print(
            f"[STRUCTURAL] P{patch_id}: "
            f"spine_runs={len(spine_roles_sorted)} spine_axis={spine_axis} spine_len={spine_length:.2f} "
            f"perimeter={perimeter:.2f} spine_ratio={spine_ratio:.2f} spine_score={spine_score:.2f} "
            f"elongation={elongation:.2f} side_conf={side_confidence:.2f} "
            f"T={terminal_count} B={branch_count} "
            f"strip_conf={strip_confidence:.2f} eligible={'Y' if straighten_eligible else 'N'}"
            f"{inh_tag}{axis_tag}{junc_tag}{ss_tag}{band_tag}{band_candidate_tag}{band_runtime_tag}{band_reject_tag}{band_intervene_tag}{band_intervene_reject_tag}"
        )

    return result


def _enrich_patch_summaries(patch_summaries, structural_fields):
    """Create new patch summaries with structural interpretation fields merged in.

    Returns new tuple of _PatchDerivedTopologySummary with spine/strip fields filled.
    """

    enriched = []
    for summary in patch_summaries:
        fields = structural_fields.get(summary.patch_id)
        if fields is not None:
            enriched.append(replace(summary, **fields))
        else:
            enriched.append(summary)
    return tuple(enriched)


def _build_patch_graph_derived_topology(graph, measure_chain_axis_metrics):
    """Build the canonical derived topology bundle over the final PatchGraph."""

    loop_frame_results = _build_patch_graph_loop_frame_results(graph, measure_chain_axis_metrics)
    patch_summaries, aggregate_counts = _build_patch_topology_summaries(graph, loop_frame_results)
    run_refs_by_corner = _build_junction_run_refs_by_corner(loop_frame_results)
    junctions = tuple(_build_patch_graph_junctions(graph, run_refs_by_corner=run_refs_by_corner))

    frame_runs_by_loop = {
        loop_key: build_result.runs
        for loop_key, build_result in loop_frame_results.items()
    }
    junctions_by_vert_index = {
        junction.vert_index: junction
        for junction in junctions
    }

    # --- Structural interpretation pass ---
    neighbor_inherited_roles = _compute_neighbor_inherited_roles(graph)
    run_structural_roles = _interpret_run_structural_roles(
        frame_runs_by_loop, junctions, neighbor_inherited_roles
    )
    junction_structural_roles = _interpret_junction_structural_roles(
        junctions, frame_runs_by_loop, run_structural_roles
    )
    structural_fields = _derive_patch_structural_summary(
        graph, frame_runs_by_loop, run_structural_roles, junction_structural_roles
    )
    patch_summaries = _enrich_patch_summaries(patch_summaries, structural_fields)

    patch_summaries_by_id = {
        patch_summary.patch_id: patch_summary
        for patch_summary in patch_summaries
    }
    loop_summaries_by_key = {
        (loop_summary.patch_id, loop_summary.loop_index): loop_summary
        for patch_summary in patch_summaries
        for loop_summary in patch_summary.loop_summaries
    }

    return _PatchGraphDerivedTopology(
        patch_summaries=patch_summaries,
        patch_summaries_by_id=MappingProxyType(dict(patch_summaries_by_id)),
        loop_summaries_by_key=MappingProxyType(dict(loop_summaries_by_key)),
        aggregate_counts=aggregate_counts,
        loop_frame_results=MappingProxyType(dict(loop_frame_results)),
        frame_runs_by_loop=MappingProxyType(dict(frame_runs_by_loop)),
        run_refs_by_corner=MappingProxyType(dict(run_refs_by_corner)),
        junctions=junctions,
        junctions_by_vert_index=MappingProxyType(dict(junctions_by_vert_index)),
        run_structural_roles=MappingProxyType(dict(run_structural_roles)),
        junction_structural_roles=MappingProxyType(dict(junction_structural_roles)),
        neighbor_inherited_roles=MappingProxyType(dict(neighbor_inherited_roles)),
    )
