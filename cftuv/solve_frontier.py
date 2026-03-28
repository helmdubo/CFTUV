from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
import time
from heapq import heappop, heappush
from typing import Optional

from mathutils import Vector

try:
    from .model import (
        BoundaryChain, BoundaryCorner, BoundaryLoop, ChainNeighborKind, FrameRole, LoopKind,
        PatchGraph, PatchNode, PatchType,
        ScaffoldPointKey, ScaffoldChainPlacement, ScaffoldPatchPlacement,
        ScaffoldQuiltPlacement, ScaffoldMap, ScaffoldClosureSeamReport,
        ScaffoldFrameAlignmentReport, ChainGapReport, PatchPlacementStatus, PlacementSourceKind,
        ClosureAnchorMode, FrameAxisKind, ChainRef, PatchEdgeKey, LoopChainRef, SourcePoint, AnchorAdjustment,
    )
    from .console_debug import trace_console
    from .solve_records import *
    from .solve_planning import (
        _build_solve_view,
        _iter_neighbor_chains,
        _build_quilt_tree_edges,
        _build_patch_tree_adjacency,
        _find_patch_tree_path,
        _is_allowed_quilt_edge,
        _restore_original_quilt_plan,
    )
    from .solve_diagnostics import (
        _build_chain_vert_uv_map,
        _chain_uv_axis_metrics,
        _measure_shared_closure_uv_offsets,
        _collect_quilt_closure_seam_reports,
        _collect_quilt_frame_alignment_reports,
        _print_quilt_closure_seam_reports,
        _print_quilt_frame_alignment_reports,
    )
    from .solve_pin_policy import _compute_scaffold_connected_chains
    from .solve_instrumentation import FrontierTelemetryCollector, collect_stall_snapshot
except ImportError:
    from model import (
        BoundaryChain, BoundaryCorner, BoundaryLoop, ChainNeighborKind, FrameRole, LoopKind,
        PatchGraph, PatchNode, PatchType,
        ScaffoldPointKey, ScaffoldChainPlacement, ScaffoldPatchPlacement,
        ScaffoldQuiltPlacement, ScaffoldMap, ScaffoldClosureSeamReport,
        ScaffoldFrameAlignmentReport, ChainGapReport, PatchPlacementStatus, PlacementSourceKind,
        ClosureAnchorMode, FrameAxisKind, ChainRef, PatchEdgeKey, LoopChainRef, SourcePoint, AnchorAdjustment,
    )
    from console_debug import trace_console
    from solve_records import *
    from solve_planning import (
        _build_solve_view,
        _iter_neighbor_chains,
        _build_quilt_tree_edges,
        _build_patch_tree_adjacency,
        _find_patch_tree_path,
        _is_allowed_quilt_edge,
        _restore_original_quilt_plan,
    )
    from solve_diagnostics import (
        _build_chain_vert_uv_map,
        _chain_uv_axis_metrics,
        _measure_shared_closure_uv_offsets,
        _collect_quilt_closure_seam_reports,
        _collect_quilt_frame_alignment_reports,
        _print_quilt_closure_seam_reports,
        _print_quilt_frame_alignment_reports,
    )
    from solve_pin_policy import _compute_scaffold_connected_chains
    from solve_instrumentation import FrontierTelemetryCollector, collect_stall_snapshot


def _match_non_tree_closure_chain_pairs(
    graph: PatchGraph,
    owner_patch_id: int,
    target_patch_id: int,
) -> list[ClosureChainPairMatch]:
    def _chain_polyline_length(chain: BoundaryChain) -> float:
        if len(chain.vert_cos) < 2:
            return 0.0
        return sum(
            (chain.vert_cos[index + 1] - chain.vert_cos[index]).length
            for index in range(len(chain.vert_cos) - 1)
        )

    owner_refs = _iter_neighbor_chains(graph, owner_patch_id, target_patch_id)
    target_refs = _iter_neighbor_chains(graph, target_patch_id, owner_patch_id)
    pair_candidates: list[ClosureChainPairCandidate] = []

    for owner_ref in owner_refs:
        owner_chain = owner_ref.chain
        if owner_chain.frame_role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
            continue
        owner_vert_set = set(owner_chain.vert_indices)
        if not owner_vert_set:
            continue
        for target_ref in target_refs:
            target_chain = target_ref.chain
            if target_chain.frame_role != owner_chain.frame_role:
                continue
            target_vert_set = set(target_chain.vert_indices)
            shared_vert_count = len(owner_vert_set & target_vert_set)
            if shared_vert_count <= 0:
                continue
            pair_score = (shared_vert_count * 1000) - abs(len(owner_chain.vert_indices) - len(target_chain.vert_indices))
            owner_chain_length = _chain_polyline_length(owner_chain)
            target_chain_length = _chain_polyline_length(target_chain)
            if owner_chain_length > 0.0 and target_chain_length > 0.0:
                representative_length = min(owner_chain_length, target_chain_length)
            else:
                representative_length = max(owner_chain_length, target_chain_length)
            pair_candidates.append(
                ClosureChainPairCandidate(
                    score=pair_score,
                    representative_length=representative_length,
                    match=ClosureChainPairMatch(
                        owner_ref=(owner_patch_id, owner_ref.loop_index, owner_ref.chain_index),
                        owner_chain=owner_chain,
                        target_ref=(target_patch_id, target_ref.loop_index, target_ref.chain_index),
                        target_chain=target_chain,
                        shared_vert_count=shared_vert_count,
                    ),
                )
            )

    # При равном topological match оставляем более длинный seam-carrier primary,
    # чтобы короткая пара оставалась intentional closure seam на tube/ring topology.
    pair_candidates.sort(
        key=lambda item: (item.score, item.representative_length),
        reverse=True,
    )
    matched_owner_refs = set()
    matched_target_refs = set()
    matched_pairs: list[ClosureChainPairMatch] = []
    for candidate in pair_candidates:
        match = candidate.match
        if match.owner_ref in matched_owner_refs or match.target_ref in matched_target_refs:
            continue
        matched_owner_refs.add(match.owner_ref)
        matched_target_refs.add(match.target_ref)
        matched_pairs.append(match)

    return matched_pairs


def _iter_quilt_closure_chain_pairs(
    graph: PatchGraph,
    quilt_patch_ids: set[int],
    allowed_tree_edges: set[PatchEdgeKey],
    tree_ingress_pair_map: Optional[dict[PatchEdgeKey, frozenset[ChainRef]]] = None,
):
    """Iterate closure-side chain pairs inside one quilt.

    Non-tree patch pairs contribute all matched chain pairs.
    Tree patch pairs contribute only secondary matched pairs, because the first
    matched pair acts as the primary ingress seam while the rest behave like
    closure seams (tube/cycle case with two seams between the same patches).
    """

    quilt_patch_pairs = sorted(
        edge_key
        for edge_key in graph.edges.keys()
        if edge_key[0] in quilt_patch_ids and edge_key[1] in quilt_patch_ids
    )

    for owner_patch_id, target_patch_id in quilt_patch_pairs:
        matched_pairs = _match_non_tree_closure_chain_pairs(graph, owner_patch_id, target_patch_id)
        if not matched_pairs:
            continue

        if (owner_patch_id, target_patch_id) in allowed_tree_edges:
            primary_pair_refs = None
            if tree_ingress_pair_map is not None:
                primary_pair_refs = tree_ingress_pair_map.get((owner_patch_id, target_patch_id))
            if primary_pair_refs is not None:
                matched_pairs = [
                    match
                    for match in matched_pairs
                    if frozenset((match.owner_ref, match.target_ref)) != primary_pair_refs
                ]
            else:
                matched_pairs = matched_pairs[1:]
            if not matched_pairs:
                continue

        yield owner_patch_id, target_patch_id, matched_pairs


def _build_quilt_closure_pair_map(
    graph: PatchGraph,
    quilt_plan: QuiltPlan,
    quilt_patch_ids: set[int],
    allowed_tree_edges: set[PatchEdgeKey],
) -> dict[ChainRef, ChainRef]:
    tree_ingress_pair_map: dict[PatchEdgeKey, frozenset[ChainRef]] = {}
    for edge_key in sorted(allowed_tree_edges):
        owner_patch_id, target_patch_id = edge_key
        matched_pairs = _match_non_tree_closure_chain_pairs(graph, owner_patch_id, target_patch_id)
        if not matched_pairs:
            continue
        primary_match = matched_pairs[0]
        tree_ingress_pair_map[edge_key] = frozenset((
            primary_match.owner_ref,
            primary_match.target_ref,
        ))
        trace_console(
            f"[CFTUV][Frontier] Tree pair {owner_patch_id}-{target_patch_id}: "
            f"P{primary_match.owner_ref[0]} L{primary_match.owner_ref[1]}C{primary_match.owner_ref[2]}"
            f"<->"
            f"P{primary_match.target_ref[0]} L{primary_match.target_ref[1]}C{primary_match.target_ref[2]}"
        )

    for step in quilt_plan.steps:
        candidate = step.incoming_candidate
        if candidate is None:
            continue
        edge_key = (
            min(candidate.owner_patch_id, candidate.target_patch_id),
            max(candidate.owner_patch_id, candidate.target_patch_id),
        )
        if edge_key in tree_ingress_pair_map:
            continue
        tree_ingress_pair_map[edge_key] = frozenset((
            (candidate.owner_patch_id, candidate.owner_loop_index, candidate.owner_chain_index),
            (candidate.target_patch_id, candidate.target_loop_index, candidate.target_chain_index),
        ))

    pair_map = {}
    for owner_patch_id, target_patch_id, matched_pairs in _iter_quilt_closure_chain_pairs(
        graph,
        quilt_patch_ids,
        allowed_tree_edges,
        tree_ingress_pair_map,
    ):
        for match in matched_pairs:
            pair_map[match.owner_ref] = match.target_ref
            pair_map[match.target_ref] = match.owner_ref
    return pair_map


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


def _closure_preconstraint_direction_options(
    chain_ref: ChainRef,
    chain: BoundaryChain,
    node: PatchNode,
    start_anchor: Optional[ChainAnchor],
    end_anchor: Optional[ChainAnchor],
    graph: PatchGraph,
    point_registry: PointRegistry,
) -> list[DirectionOption]:
    options = [DirectionOption(label='default', direction_override=None)]
    if chain.frame_role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        return options

    base_direction = _try_inherit_direction(
        chain, node, start_anchor, end_anchor, graph, point_registry, chain_ref=chain_ref
    )
    if base_direction is None:
        base_direction = _cf_determine_direction(chain, node)
    axis_direction = _snap_direction_to_role(base_direction, chain.frame_role)
    axis_direction = _normalize_direction(axis_direction, _default_role_direction(chain.frame_role))
    flipped_direction = Vector((-axis_direction.x, -axis_direction.y))
    if (flipped_direction - axis_direction).length > 1e-8:
        options.append(DirectionOption(label='flip', direction_override=flipped_direction))
    return options


def _closure_preconstraint_metric(
    graph: PatchGraph,
    chain_ref: ChainRef,
    chain: BoundaryChain,
    uv_points: list[Vector],
    start_anchor: Optional[ChainAnchor],
    end_anchor: Optional[ChainAnchor],
    raw_start_anchor: Optional[ChainAnchor],
    raw_end_anchor: Optional[ChainAnchor],
    partner_placement: ScaffoldChainPlacement,
) -> ClosurePreconstraintMetric:
    temporary_placement = _build_temporary_chain_placement(
        chain_ref,
        chain,
        uv_points,
        start_anchor,
        end_anchor,
    )
    shared_offsets = _measure_shared_closure_uv_offsets(graph, temporary_placement, partner_placement)
    predicted_uv_span = _chain_uv_axis_metrics(temporary_placement).span
    partner_uv_span = _chain_uv_axis_metrics(partner_placement).span
    span_mismatch = abs(predicted_uv_span - partner_uv_span)

    same_patch_gap_max = 0.0
    all_gap_max = 0.0
    all_gap_sum = 0.0
    gap_count = 0
    for raw_anchor, predicted_uv in (
        (raw_start_anchor, uv_points[0]),
        (raw_end_anchor, uv_points[-1]),
    ):
        if raw_anchor is None:
            continue
        gap = (predicted_uv - raw_anchor.uv).length
        all_gap_max = max(all_gap_max, gap)
        all_gap_sum += gap
        gap_count += 1
        if raw_anchor.source_kind == PlacementSourceKind.SAME_PATCH:
            same_patch_gap_max = max(same_patch_gap_max, gap)

    all_gap_mean = all_gap_sum / gap_count if gap_count > 0 else 0.0
    return ClosurePreconstraintMetric(
        same_patch_gap_max,
        shared_offsets.axis_phase_offset_max,
        all_gap_max,
        span_mismatch,
        shared_offsets.axis_phase_offset_mean,
        all_gap_mean,
    )


def _cf_apply_closure_preconstraint(
    chain_ref: ChainRef,
    chain: BoundaryChain,
    node: PatchNode,
    raw_start_anchor: Optional[ChainAnchor],
    raw_end_anchor: Optional[ChainAnchor],
    start_anchor: Optional[ChainAnchor],
    end_anchor: Optional[ChainAnchor],
    known: int,
    graph: PatchGraph,
    point_registry: PointRegistry,
    placed_chains_map: dict[ChainRef, ScaffoldChainPlacement],
    closure_pair_map: dict[ChainRef, ChainRef],
    final_scale: float,
) -> ClosurePreconstraintApplication:
    if known != 1 or chain.frame_role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        return ClosurePreconstraintApplication(start_anchor=start_anchor, end_anchor=end_anchor)

    partner_ref = closure_pair_map.get(chain_ref)
    if partner_ref is None:
        return ClosurePreconstraintApplication(start_anchor=start_anchor, end_anchor=end_anchor)
    partner_placement = placed_chains_map.get(partner_ref)
    if partner_placement is None or len(partner_placement.points) < 2:
        return ClosurePreconstraintApplication(start_anchor=start_anchor, end_anchor=end_anchor)

    anchor_options: list[AnchorOption] = []
    seen_anchor_keys: set[AnchorRefPair] = set()
    for anchor_label, option_start, option_end in (
        ('resolved', start_anchor, end_anchor),
        ('start_only', raw_start_anchor, None),
        ('end_only', None, raw_end_anchor),
    ):
        if _cf_anchor_count(option_start, option_end) != 1:
            continue
        anchor_key: AnchorRefPair = (
            option_start.source_ref if option_start is not None else None,
            option_end.source_ref if option_end is not None else None,
        )
        if anchor_key in seen_anchor_keys:
            continue
        seen_anchor_keys.add(anchor_key)
        anchor_options.append(AnchorOption(
            label=anchor_label,
            start_anchor=option_start,
            end_anchor=option_end,
        ))

    if not anchor_options:
        return ClosurePreconstraintApplication(start_anchor=start_anchor, end_anchor=end_anchor)

    option_results: list[ClosurePreconstraintOptionResult] = []
    for anchor_option in anchor_options:
        direction_options = _closure_preconstraint_direction_options(
            chain_ref,
            chain,
            node,
            anchor_option.start_anchor,
            anchor_option.end_anchor,
            graph,
            point_registry,
        )
        for direction_option in direction_options:
            uv_points = _cf_place_chain(
                chain,
                node,
                anchor_option.start_anchor,
                anchor_option.end_anchor,
                final_scale,
                direction_option.direction_override,
            )
            if not uv_points or len(uv_points) != len(chain.vert_cos):
                continue
            metric = _closure_preconstraint_metric(
                graph,
                chain_ref,
                chain,
                uv_points,
                anchor_option.start_anchor,
                anchor_option.end_anchor,
                raw_start_anchor,
                raw_end_anchor,
                partner_placement,
            )
            option_results.append(ClosurePreconstraintOptionResult(
                metric=metric,
                anchor_label=anchor_option.label,
                direction_label=direction_option.label,
                start_anchor=anchor_option.start_anchor,
                end_anchor=anchor_option.end_anchor,
                direction_override=direction_option.direction_override,
            ))

    if not option_results:
        return ClosurePreconstraintApplication(start_anchor=start_anchor, end_anchor=end_anchor)

    current_anchor_key: AnchorRefPair = (
        start_anchor.source_ref if start_anchor is not None else None,
        end_anchor.source_ref if end_anchor is not None else None,
    )
    current_metric = None
    best_result: Optional[ClosurePreconstraintOptionResult] = None
    for result in sorted(option_results, key=lambda item: item.metric):
        option_anchor_key: AnchorRefPair = (
            result.start_anchor.source_ref if result.start_anchor is not None else None,
            result.end_anchor.source_ref if result.end_anchor is not None else None,
        )
        if option_anchor_key == current_anchor_key and result.direction_label == 'default' and current_metric is None:
            current_metric = result.metric
        if best_result is None:
            best_result = result

    if best_result is None or current_metric is None:
        return ClosurePreconstraintApplication(start_anchor=start_anchor, end_anchor=end_anchor)

    if best_result.metric >= current_metric:
        return ClosurePreconstraintApplication(start_anchor=start_anchor, end_anchor=end_anchor)

    reason = (
        f"closure_preconstraint:{best_result.anchor_label}/{best_result.direction_label}"
        f":phase={best_result.metric.axis_phase_offset_max:.4f}"
        f":gap={best_result.metric.same_patch_gap_max:.4f}"
    )
    return ClosurePreconstraintApplication(
        start_anchor=best_result.start_anchor,
        end_anchor=best_result.end_anchor,
        direction_override=best_result.direction_override,
        reason=reason,
    )


def _cf_build_closure_follow_uvs(
    graph: PatchGraph,
    chain: BoundaryChain,
    partner_placement: ScaffoldChainPlacement,
    final_scale: float,
) -> ClosureFollowUvBuildResult:
    """Строит UV для closure-пары напрямую от уже placed partner chain."""
    partner_uv_by_vert = _build_chain_vert_uv_map(graph, partner_placement)
    if not partner_uv_by_vert:
        return ClosureFollowUvBuildResult(uv_points=None)

    shared_vert_count = len(set(chain.vert_indices) & set(partner_uv_by_vert.keys()))
    if shared_vert_count <= 0:
        return ClosureFollowUvBuildResult(uv_points=None)

    if len(chain.vert_indices) == len(chain.vert_cos):
        shared_uv_points = []
        all_shared = True
        for vert_index in chain.vert_indices:
            uv = partner_uv_by_vert.get(vert_index)
            if uv is None:
                all_shared = False
                break
            shared_uv_points.append(uv.copy())
        if all_shared and len(shared_uv_points) == len(chain.vert_cos):
            return ClosureFollowUvBuildResult(
                uv_points=shared_uv_points,
                follow_mode='shared_verts',
                shared_vert_count=shared_vert_count,
            )

    start_uv = partner_uv_by_vert.get(chain.start_vert_index)
    end_uv = partner_uv_by_vert.get(chain.end_vert_index)
    if start_uv is None or end_uv is None:
        return ClosureFollowUvBuildResult(uv_points=None, shared_vert_count=shared_vert_count)

    rebuilt_uvs = _cf_rebuild_chain_points_for_endpoints(chain, start_uv, end_uv, final_scale)
    if rebuilt_uvs is None or len(rebuilt_uvs) != len(chain.vert_cos):
        return ClosureFollowUvBuildResult(uv_points=None, shared_vert_count=shared_vert_count)
    return ClosureFollowUvBuildResult(
        uv_points=rebuilt_uvs,
        follow_mode='partner_endpoints',
        shared_vert_count=shared_vert_count,
    )


def _cf_rescue_gap_candidate_class(
    rescue_path: str,
    chain: BoundaryChain,
    main_eval: FrontierCandidateEval,
    *,
    shared_vert_count: int = 0,
) -> str:
    anchor_class = {
        0: 'no_anchor',
        1: 'single_anchor',
        2: 'dual_anchor',
    }.get(main_eval.known, f'known{main_eval.known}')
    if rescue_path == 'tree_ingress':
        role_class = 'hv' if chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME} else 'free'
        patch_phase = (
            'untouched'
            if main_eval.patch_context is not None and main_eval.patch_context.is_untouched
            else 'patch_progress'
        )
        return f"tree_ingress_{anchor_class}_{role_class}_{patch_phase}"
    if rescue_path == 'closure_follow':
        shared_class = 'shared2' if shared_vert_count >= 2 else 'shared1'
        return f"closure_follow_{anchor_class}_{shared_class}"
    return f"{rescue_path}_{anchor_class}"


def _cf_build_frontier_rescue_gap(
    rescue_path: str,
    chain_ref: ChainRef,
    chain: BoundaryChain,
    main_eval: FrontierCandidateEval,
    *,
    hv_adjacency: int = 0,
    downstream_support: int = 0,
    shared_vert_count: int = 0,
) -> FrontierRescueGap:
    main_viable = bool(
        main_eval.known > 0
        and main_eval.score >= CHAIN_FRONTIER_THRESHOLD
    )
    if main_eval.known <= 0:
        threshold_gap = CHAIN_FRONTIER_THRESHOLD
        state_label = 'no_main_anchor'
    elif main_viable:
        threshold_gap = 0.0
        state_label = 'main_viable'
    else:
        threshold_gap = max(0.0, CHAIN_FRONTIER_THRESHOLD - main_eval.score)
        state_label = 'below_threshold'

    candidate_class = _cf_rescue_gap_candidate_class(
        rescue_path,
        chain,
        main_eval,
        shared_vert_count=shared_vert_count,
    )
    summary_parts = [candidate_class, state_label]
    if main_eval.rank_breakdown is not None and main_eval.rank_breakdown.summary:
        summary_parts.append(main_eval.rank_breakdown.summary)
    elif main_eval.anchor_reason:
        summary_parts.append(main_eval.anchor_reason)
    if hv_adjacency > 0:
        summary_parts.append(f"hv:{hv_adjacency}")
    if downstream_support > 0:
        summary_parts.append(f"ds:{downstream_support}")
    if shared_vert_count > 0:
        summary_parts.append(f"sh:{shared_vert_count}")

    return FrontierRescueGap(
        candidate_class=candidate_class,
        main_known=main_eval.known,
        main_score=main_eval.score,
        threshold_gap=threshold_gap,
        main_viable=main_viable,
        hv_adjacency=hv_adjacency,
        downstream_support=downstream_support,
        shared_vert_count=shared_vert_count,
        main_rank=main_eval.rank,
        main_rank_breakdown=main_eval.rank_breakdown,
        summary='|'.join(summary_parts),
    )


def _cf_try_place_closure_follow_candidate(
    runtime_policy: FrontierRuntimePolicy,
    all_chain_pool: list[ChainPoolEntry],
    closure_pair_map: dict[ChainRef, ChainRef],
    iteration: int,
    collector: Optional[FrontierTelemetryCollector] = None,
) -> bool:
    """Когда frontier встал, пробует дозавести same-role closure partner от уже placed пары."""
    graph = runtime_policy.graph
    best_candidate: Optional[ClosureFollowPlacementCandidate] = None

    for entry in all_chain_pool:
        chain_ref = entry.chain_ref
        chain = entry.chain
        if chain_ref in runtime_policy.placed_chain_refs or chain_ref in runtime_policy.rejected_chain_refs:
            continue
        if chain.frame_role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
            continue

        partner_ref = closure_pair_map.get(chain_ref)
        if partner_ref is None:
            continue
        partner_placement = runtime_policy.placed_chains_map.get(partner_ref)
        if partner_placement is None or len(partner_placement.points) < 2:
            continue
        if partner_placement.frame_role != chain.frame_role:
            continue

        follow_result = _cf_build_closure_follow_uvs(
            graph,
            chain,
            partner_placement,
            runtime_policy.final_scale,
        )
        uv_points = follow_result.uv_points
        follow_mode = follow_result.follow_mode
        shared_vert_count = follow_result.shared_vert_count
        if not uv_points or len(uv_points) != len(chain.vert_cos):
            continue

        candidate_rank = ClosureFollowCandidateRank(
            anchor_count=partner_placement.anchor_count,
            shared_vert_count=shared_vert_count,
            length_bias=-_cf_chain_total_length(chain, runtime_policy.final_scale),
        )
        if best_candidate is None or candidate_rank > best_candidate.rank:
            best_candidate = ClosureFollowPlacementCandidate(
                rank=candidate_rank,
                chain_ref=chain_ref,
                chain=chain,
                partner_ref=partner_ref,
                uv_points=uv_points,
                follow_mode=follow_mode,
                shared_vert_count=shared_vert_count,
            )

    if best_candidate is None:
        return False

    chain_ref = best_candidate.chain_ref
    chain = best_candidate.chain
    partner_ref = best_candidate.partner_ref
    uv_points = best_candidate.uv_points
    follow_mode = best_candidate.follow_mode
    shared_vert_count = best_candidate.shared_vert_count
    node = graph.nodes.get(chain_ref[0])
    main_eval = (
        runtime_policy.evaluate_candidate(
            chain_ref,
            chain,
            node,
            apply_closure_preconstraint=True,
            compute_score=True,
        )
        if node is not None else
        FrontierCandidateEval(
            raw_start_anchor=None,
            raw_end_anchor=None,
            start_anchor=None,
            end_anchor=None,
            known=0,
            placed_in_patch=runtime_policy.placed_in_patch(chain_ref[0]),
        )
    )
    rescue_gap = _cf_build_frontier_rescue_gap(
        "closure_follow",
        chain_ref,
        chain,
        main_eval,
        shared_vert_count=shared_vert_count,
    )
    chain_placement = ScaffoldChainPlacement(
        patch_id=chain_ref[0],
        loop_index=chain_ref[1],
        chain_index=chain_ref[2],
        frame_role=chain.frame_role,
        source_kind=PlacementSourceKind.CHAIN,
        anchor_count=2,
        points=tuple(
            (ScaffoldPointKey(chain_ref[0], chain_ref[1], chain_ref[2], point_index), uv.copy())
            for point_index, uv in enumerate(uv_points)
        ),
    )

    runtime_policy.register_chain(
        chain_ref,
        chain,
        chain_placement,
        uv_points,
        tuple(
            sorted(
                patch_id
                for patch_id in (partner_ref[0],)
                if patch_id != chain_ref[0]
            )
        ),
    )

    trace_console(
        f"[CFTUV][Frontier] Step {iteration}: "
        f"P{chain_ref[0]} L{chain_ref[1]}C{chain_ref[2]} "
        f"{chain.frame_role.value} score:closure_follow ep:2 "
        f"a:CP{partner_ref[0]}/CP{partner_ref[0]} "
        f"note:{follow_mode}:shared={shared_vert_count}"
    )
    if collector is not None:
        collector.record_placement(
            iteration=iteration,
            chain_ref=chain_ref,
            chain=chain,
            placement_path="closure_follow",
            score=-1.0,
            start_anchor=None,
            end_anchor=None,
            placed_in_patch_before=runtime_policy.placed_in_patch(chain_ref[0]) - 1,
            is_closure_pair=(chain_ref in runtime_policy.closure_pair_refs),
            uv_points=uv_points,
            length_factor=main_eval.length_factor,
            downstream_count=main_eval.downstream_count,
            downstream_bonus=main_eval.downstream_bonus,
            isolation_preview=main_eval.isolation_preview,
            isolation_penalty=main_eval.isolation_penalty,
            structural_free_bonus=main_eval.structural_free_bonus,
            hv_adjacency=main_eval.hv_adjacency,
            rank=main_eval.rank,
            rank_breakdown=main_eval.rank_breakdown,
            patch_context=main_eval.patch_context,
            corner_hints=main_eval.corner_hints,
            seam_relation=main_eval.seam_relation,
            seam_bonus=main_eval.seam_bonus,
            corner_bonus=main_eval.corner_bonus,
            shape_bonus=main_eval.shape_bonus,
            rescue_gap=rescue_gap,
        )
    return True


def _cf_try_place_tree_ingress_candidate(
    runtime_policy: FrontierRuntimePolicy,
    all_chain_pool: list[ChainPoolEntry],
    iteration: int,
    collector: Optional[FrontierTelemetryCollector] = None,
) -> bool:
    """Контролируемый bootstrap в untouched tree-child patch, если обычный frontier уже встал."""
    best_candidate: Optional[TreeIngressPlacementCandidate] = None

    graph = runtime_policy.graph
    for entry in all_chain_pool:
        chain_ref = entry.chain_ref
        chain = entry.chain
        node = entry.node
        if chain_ref in runtime_policy.placed_chain_refs or chain_ref in runtime_policy.rejected_chain_refs:
            continue

        candidate_eval = runtime_policy.evaluate_candidate(chain_ref, chain, node)
        if candidate_eval.placed_in_patch > 0:
            continue
        if chain.neighbor_kind != ChainNeighborKind.PATCH:
            continue
        if chain.neighbor_patch_id not in runtime_policy.quilt_patch_ids:
            continue
        if not _is_allowed_quilt_edge(runtime_policy.allowed_tree_edges, chain_ref[0], chain.neighbor_patch_id):
            continue

        if candidate_eval.known != 1:
            continue

        resolved_anchor = candidate_eval.start_anchor if candidate_eval.start_anchor is not None else candidate_eval.end_anchor
        if resolved_anchor is None or resolved_anchor.source_kind != PlacementSourceKind.CROSS_PATCH:
            continue

        if chain.frame_role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
            continue
        if candidate_eval.hv_adjacency <= 0:
            continue

        role_priority = 4 if candidate_eval.hv_adjacency >= 2 else 3
        if chain.is_corner_split:
            role_priority -= 1

        endpoint_neighbors = graph.get_chain_endpoint_neighbors(chain_ref[0], chain_ref[1], chain_ref[2])
        downstream_side = 'end' if candidate_eval.start_anchor is not None else 'start'
        downstream_hv_count = 0
        downstream_max_length = 0.0
        for neighbor_loop_index, neighbor_chain_index in endpoint_neighbors.get(downstream_side, []):
            neighbor_ref = (chain_ref[0], neighbor_loop_index, neighbor_chain_index)
            if neighbor_ref in runtime_policy.placed_chain_refs or neighbor_ref in runtime_policy.rejected_chain_refs:
                continue
            neighbor_chain = graph.get_chain(*neighbor_ref)
            if neighbor_chain is None or neighbor_chain.frame_role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
                continue
            downstream_hv_count += 1
            downstream_max_length = max(
                downstream_max_length,
                _cf_chain_total_length(neighbor_chain, runtime_policy.final_scale),
            )

        direction_override = _try_inherit_direction(
            chain,
            node,
            candidate_eval.start_anchor,
            candidate_eval.end_anchor,
            graph,
            runtime_policy.point_registry,
            chain_ref=chain_ref,
        )
        uv_points = _cf_place_chain(
            chain,
            node,
            candidate_eval.start_anchor,
            candidate_eval.end_anchor,
            runtime_policy.final_scale,
            direction_override,
        )
        if not uv_points or len(uv_points) != len(chain.vert_cos):
            continue

        candidate_rank = TreeIngressCandidateRank(
            role_priority=role_priority,
            downstream_hv_count=downstream_hv_count,
            downstream_max_length=downstream_max_length,
            chain_length=_cf_chain_total_length(chain, runtime_policy.final_scale),
        )
        if best_candidate is None or candidate_rank > best_candidate.rank:
            best_candidate = TreeIngressPlacementCandidate(
                rank=candidate_rank,
                chain_ref=chain_ref,
                chain=chain,
                start_anchor=candidate_eval.start_anchor,
                end_anchor=candidate_eval.end_anchor,
                anchor_adjustments=tuple(candidate_eval.anchor_adjustments),
                uv_points=uv_points,
                downstream_hv_count=downstream_hv_count,
                role_priority=role_priority,
                hv_adjacency=candidate_eval.hv_adjacency,
            )

    if best_candidate is None:
        return False

    chain_ref = best_candidate.chain_ref
    chain = best_candidate.chain
    anchor_start = best_candidate.start_anchor
    anchor_end = best_candidate.end_anchor
    anchor_adjustments = best_candidate.anchor_adjustments
    uv_points = best_candidate.uv_points
    downstream_hv_count = best_candidate.downstream_hv_count
    role_priority = best_candidate.role_priority
    hv_adjacency = best_candidate.hv_adjacency
    node = runtime_policy.graph.nodes.get(chain_ref[0])
    main_eval = (
        runtime_policy.evaluate_candidate(
            chain_ref,
            chain,
            node,
            apply_closure_preconstraint=True,
            compute_score=True,
        )
        if node is not None else
        FrontierCandidateEval(
            raw_start_anchor=None,
            raw_end_anchor=None,
            start_anchor=None,
            end_anchor=None,
            known=0,
            placed_in_patch=runtime_policy.placed_in_patch(chain_ref[0]),
        )
    )
    rescue_gap = _cf_build_frontier_rescue_gap(
        "tree_ingress",
        chain_ref,
        chain,
        main_eval,
        hv_adjacency=hv_adjacency,
        downstream_support=downstream_hv_count,
    )

    if anchor_adjustments and not _cf_apply_anchor_adjustments(
        anchor_adjustments,
        graph,
        runtime_policy.placed_chains_map,
        runtime_policy.point_registry,
        runtime_policy.final_scale,
    ):
        runtime_policy.reject_chain(chain_ref)
        return False

    chain_placement = ScaffoldChainPlacement(
        patch_id=chain_ref[0],
        loop_index=chain_ref[1],
        chain_index=chain_ref[2],
        frame_role=chain.frame_role,
        source_kind=PlacementSourceKind.CHAIN,
        anchor_count=_cf_anchor_count(anchor_start, anchor_end),
        points=tuple(
            (ScaffoldPointKey(chain_ref[0], chain_ref[1], chain_ref[2], point_index), uv.copy())
            for point_index, uv in enumerate(uv_points)
        ),
    )

    runtime_policy.register_chain(
        chain_ref,
        chain,
        chain_placement,
        uv_points,
        runtime_policy.dependency_patches_from_anchors(chain_ref[0], anchor_start, anchor_end),
    )

    trace_console(
        f"[CFTUV][Frontier] Step {iteration}: "
        f"P{chain_ref[0]} L{chain_ref[1]}C{chain_ref[2]} "
        f"{chain.frame_role.value} score:tree_ingress ep:1 "
        f"a:{_cf_anchor_debug_label(anchor_start, anchor_end)} "
        f"note:priority={role_priority}:downstream_hv={downstream_hv_count}:hv_adj={hv_adjacency}"
    )
    if collector is not None:
        collector.record_placement(
            iteration=iteration,
            chain_ref=chain_ref,
            chain=chain,
            placement_path="tree_ingress",
            score=-1.0,
            start_anchor=anchor_start,
            end_anchor=anchor_end,
            placed_in_patch_before=runtime_policy.placed_in_patch(chain_ref[0]) - 1,
            is_closure_pair=(chain_ref in runtime_policy.closure_pair_refs),
            uv_points=uv_points,
            anchor_adjustment_applied=bool(anchor_adjustments),
            length_factor=main_eval.length_factor,
            downstream_count=main_eval.downstream_count,
            downstream_bonus=main_eval.downstream_bonus,
            isolation_preview=main_eval.isolation_preview,
            isolation_penalty=main_eval.isolation_penalty,
            structural_free_bonus=main_eval.structural_free_bonus,
            hv_adjacency=hv_adjacency,
            rank=main_eval.rank,
            rank_breakdown=main_eval.rank_breakdown,
            patch_context=main_eval.patch_context,
            corner_hints=main_eval.corner_hints,
            seam_relation=main_eval.seam_relation,
            seam_bonus=main_eval.seam_bonus,
            corner_bonus=main_eval.corner_bonus,
            shape_bonus=main_eval.shape_bonus,
            rescue_gap=rescue_gap,
        )
    return True


@dataclass
class FrontierRuntimePolicy:
    graph: PatchGraph
    quilt_patch_ids: set[int]
    allowed_tree_edges: set[PatchEdgeKey]
    final_scale: float
    seam_relation_by_edge: dict[PatchEdgeKey, SeamRelationProfile] = field(default_factory=dict)
    point_registry: PointRegistry = field(default_factory=dict)
    vert_to_placements: VertexPlacementMap = field(default_factory=dict)
    placed_chain_refs: set[ChainRef] = field(default_factory=set)
    placed_chains_map: dict[ChainRef, ScaffoldChainPlacement] = field(default_factory=dict)
    chain_dependency_patches: dict[ChainRef, tuple[int, ...]] = field(default_factory=dict)
    rejected_chain_refs: set[ChainRef] = field(default_factory=set)
    build_order: list[ChainRef] = field(default_factory=list)
    closure_pair_map: Optional[dict[ChainRef, ChainRef]] = None
    placed_count_by_patch: dict[int, int] = field(default_factory=dict)
    placed_h_count_by_patch: dict[int, int] = field(default_factory=dict)
    placed_v_count_by_patch: dict[int, int] = field(default_factory=dict)
    placed_free_count_by_patch: dict[int, int] = field(default_factory=dict)
    closure_pair_refs: frozenset[ChainRef] = field(init=False, default_factory=frozenset)
    _outer_chain_count_by_patch: dict[int, int] = field(init=False, default_factory=dict)
    _frame_chain_count_by_patch: dict[int, int] = field(init=False, default_factory=dict)
    _closure_pair_count_by_patch: dict[int, int] = field(init=False, default_factory=dict)
    _shape_profile_by_patch: dict[int, PatchShapeProfile] = field(init=False, default_factory=dict)
    # --- Internal incremental-frontier caches (Phase A) ---
    _cached_evals: dict[ChainRef, FrontierCandidateEval] = field(init=False, default_factory=dict)
    _dirty_refs: set[ChainRef] = field(init=False, default_factory=set)
    _vert_to_pool_refs: dict[int, list[ChainRef]] = field(init=False, default_factory=dict)
    _patch_to_pool_refs: dict[int, list[ChainRef]] = field(init=False, default_factory=dict)
    _cache_hits: int = field(init=False, default=0)

    def __post_init__(self) -> None:
        self.closure_pair_refs = frozenset(self.closure_pair_map.keys()) if self.closure_pair_map else frozenset()
        for patch_id, node in self.graph.nodes.items():
            outer_chain_count = 0
            frame_chain_count = 0
            for boundary_loop in node.boundary_loops:
                if boundary_loop.kind != LoopKind.OUTER:
                    continue
                outer_chain_count += len(boundary_loop.chains)
                frame_chain_count += sum(
                    1
                    for chain in boundary_loop.chains
                    if chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
                )
            self._outer_chain_count_by_patch[patch_id] = outer_chain_count
            self._frame_chain_count_by_patch[patch_id] = frame_chain_count
        for chain_ref in self.closure_pair_refs:
            patch_id = chain_ref[0]
            self._closure_pair_count_by_patch[patch_id] = self._closure_pair_count_by_patch.get(patch_id, 0) + 1
        for patch_id, node in self.graph.nodes.items():
            self._shape_profile_by_patch[patch_id] = _cf_build_patch_shape_profile(patch_id, node, self)

    def placed_in_patch(self, patch_id: int) -> int:
        return self.placed_count_by_patch.get(patch_id, 0)

    def placed_h_in_patch(self, patch_id: int) -> int:
        return self.placed_h_count_by_patch.get(patch_id, 0)

    def placed_v_in_patch(self, patch_id: int) -> int:
        return self.placed_v_count_by_patch.get(patch_id, 0)

    def placed_free_in_patch(self, patch_id: int) -> int:
        return self.placed_free_count_by_patch.get(patch_id, 0)

    def outer_chain_count(self, patch_id: int) -> int:
        return self._outer_chain_count_by_patch.get(patch_id, 0)

    def frame_chain_count(self, patch_id: int) -> int:
        return self._frame_chain_count_by_patch.get(patch_id, 0)

    def closure_pair_count(self, patch_id: int) -> int:
        return self._closure_pair_count_by_patch.get(patch_id, 0)

    def shape_profile(self, patch_id: int) -> Optional[PatchShapeProfile]:
        return self._shape_profile_by_patch.get(patch_id)

    def seam_relation(self, patch_a_id: int, patch_b_id: int) -> Optional[SeamRelationProfile]:
        return self.seam_relation_by_edge.get(_patch_pair_key(patch_a_id, patch_b_id))

    def total_placed(self) -> int:
        return len(self.build_order)

    def is_chain_available(self, chain_ref: ChainRef) -> bool:
        return chain_ref not in self.placed_chain_refs and chain_ref not in self.rejected_chain_refs

    def placed_patch_ids(self) -> tuple[int, ...]:
        return tuple(sorted(self.placed_count_by_patch.keys()))

    def reject_chain(self, chain_ref: ChainRef) -> None:
        self.rejected_chain_refs.add(chain_ref)
        self._cached_evals.pop(chain_ref, None)
        self._dirty_refs.discard(chain_ref)

    def dependency_patches_from_anchors(
        self,
        owner_patch_id: int,
        *anchors: Optional[ChainAnchor],
    ) -> tuple[int, ...]:
        return tuple(
            sorted({
                anchor.source_ref[0]
                for anchor in anchors
                if anchor is not None and anchor.source_ref[0] != owner_patch_id
            })
        )

    def register_chain(
        self,
        chain_ref: ChainRef,
        chain: BoundaryChain,
        chain_placement: ScaffoldChainPlacement,
        uv_points: list[Vector],
        dependency_patches: tuple[int, ...] = (),
    ) -> None:
        self.placed_chain_refs.add(chain_ref)
        self.placed_chains_map[chain_ref] = chain_placement
        self.chain_dependency_patches[chain_ref] = tuple(sorted(set(dependency_patches)))
        self.build_order.append(chain_ref)
        self.placed_count_by_patch[chain_ref[0]] = self.placed_count_by_patch.get(chain_ref[0], 0) + 1
        if chain.frame_role == FrameRole.H_FRAME:
            self.placed_h_count_by_patch[chain_ref[0]] = self.placed_h_count_by_patch.get(chain_ref[0], 0) + 1
        elif chain.frame_role == FrameRole.V_FRAME:
            self.placed_v_count_by_patch[chain_ref[0]] = self.placed_v_count_by_patch.get(chain_ref[0], 0) + 1
        else:
            self.placed_free_count_by_patch[chain_ref[0]] = self.placed_free_count_by_patch.get(chain_ref[0], 0) + 1
        _cf_register_points(chain_ref, chain, uv_points, self.point_registry, self.vert_to_placements)
        self._cached_evals.pop(chain_ref, None)
        self._dirty_refs.discard(chain_ref)
        _mark_neighbors_dirty(self, chain_ref, chain)

    def evaluate_candidate(
        self,
        chain_ref: ChainRef,
        chain: BoundaryChain,
        node: PatchNode,
        apply_closure_preconstraint: bool = False,
        compute_score: bool = False,
    ) -> FrontierCandidateEval:
        found_anchors = _cf_find_anchors(
            chain_ref,
            chain,
            self.graph,
            self.point_registry,
            self.vert_to_placements,
            self.placed_chain_refs,
            self.allowed_tree_edges,
        )
        raw_start_anchor = found_anchors.start_anchor
        raw_end_anchor = found_anchors.end_anchor

        placed_in_patch = self.placed_in_patch(chain_ref[0])
        patch_context = _cf_build_patch_scoring_context(chain_ref, self)
        seam_relation = _cf_chain_seam_relation(chain_ref, chain, self)
        resolved_anchors = _cf_resolve_candidate_anchors(
            chain,
            raw_start_anchor,
            raw_end_anchor,
            placed_in_patch,
            self.final_scale,
            self.graph,
            self.placed_chains_map,
        )
        start_anchor = resolved_anchors.start_anchor
        end_anchor = resolved_anchors.end_anchor
        known = resolved_anchors.known
        anchor_reason = resolved_anchors.reason
        anchor_adjustments = resolved_anchors.anchor_adjustments

        closure_dir_override = None
        if apply_closure_preconstraint and self.closure_pair_map is not None:
            closure_application = _cf_apply_closure_preconstraint(
                chain_ref,
                chain,
                node,
                raw_start_anchor,
                raw_end_anchor,
                start_anchor,
                end_anchor,
                known,
                self.graph,
                self.point_registry,
                self.placed_chains_map,
                self.closure_pair_map,
                self.final_scale,
            )
            start_anchor = closure_application.start_anchor
            end_anchor = closure_application.end_anchor
            closure_dir_override = closure_application.direction_override
            closure_reason = closure_application.reason
            if closure_reason:
                anchor_reason = f"{anchor_reason}|{closure_reason}" if anchor_reason else closure_reason

        score = -1.0
        topology_facts = FrontierTopologyFacts()
        score_details = FrontierLocalScoreDetails()
        rank = None
        rank_breakdown = None
        corner_hints = None
        if compute_score and known > 0:
            corner_hints = _cf_build_corner_scoring_hints(chain_ref, chain, self.graph)
            score, topology_facts, score_details = _cf_score_candidate(
                chain_ref,
                chain,
                node,
                known,
                self.graph,
                patch_context,
                self.quilt_patch_ids,
                self.allowed_tree_edges,
                self,
                corner_hints=corner_hints,
                seam_relation=seam_relation,
                closure_pair_refs=self.closure_pair_refs or None,
                start_anchor=start_anchor,
                end_anchor=end_anchor,
            )
            rank, rank_breakdown = _cf_build_frontier_rank(
                chain_ref,
                chain,
                node,
                known,
                self.graph,
                topology_facts,
                patch_context,
                self.quilt_patch_ids,
                self.allowed_tree_edges,
                self,
                score,
                seam_relation=seam_relation,
                seam_bonus=score_details.seam_bonus,
                shape_bonus=score_details.shape_bonus,
                closure_pair_refs=self.closure_pair_refs or None,
                start_anchor=start_anchor,
                end_anchor=end_anchor,
            )

        return FrontierCandidateEval(
            raw_start_anchor=raw_start_anchor,
            raw_end_anchor=raw_end_anchor,
            start_anchor=start_anchor,
            end_anchor=end_anchor,
            known=known,
            placed_in_patch=placed_in_patch,
            anchor_reason=anchor_reason,
            anchor_adjustments=anchor_adjustments,
            closure_dir_override=closure_dir_override,
            score=score,
            length_factor=score_details.length_factor,
            downstream_count=score_details.downstream_count,
            downstream_bonus=score_details.downstream_bonus,
            isolation_preview=topology_facts.would_be_connected,
            isolation_penalty=score_details.isolation_penalty,
            structural_free_bonus=score_details.structural_free_bonus,
            hv_adjacency=topology_facts.hv_adjacency,
            rank=rank,
            rank_breakdown=rank_breakdown,
            patch_context=patch_context,
            corner_hints=corner_hints,
            seam_relation=seam_relation,
            seam_bonus=score_details.seam_bonus,
            corner_bonus=score_details.corner_bonus,
            shape_bonus=score_details.shape_bonus,
        )

    def build_stop_diagnostics(
        self,
        all_chain_pool: list[ChainPoolEntry],
    ) -> FrontierStopDiagnostics:
        remaining_count = 0
        no_anchor_count = 0
        low_score_count = 0
        patches_with_no_anchor = set()
        untouched_patch_ids = set()

        for entry in all_chain_pool:
            chain_ref = entry.chain_ref
            if not self.is_chain_available(chain_ref):
                continue
            remaining_count += 1
            untouched_patch_ids.add(chain_ref[0])
            candidate_eval = self.evaluate_candidate(chain_ref, entry.chain, entry.node)
            if candidate_eval.known == 0:
                no_anchor_count += 1
                patches_with_no_anchor.add(chain_ref[0])
            else:
                low_score_count += 1

        placed_patch_ids = set(self.placed_patch_ids())
        return FrontierStopDiagnostics(
            remaining_count=remaining_count,
            no_anchor_count=no_anchor_count,
            low_score_count=low_score_count,
            rejected_count=len(self.rejected_chain_refs),
            placed_patch_ids=tuple(sorted(placed_patch_ids)),
            untouched_patch_ids=tuple(sorted(untouched_patch_ids - placed_patch_ids)),
            no_anchor_patch_ids=tuple(sorted(patches_with_no_anchor)),
        )


def _mark_neighbors_dirty(
    runtime_policy: FrontierRuntimePolicy,
    chain_ref: ChainRef,
    chain: BoundaryChain,
) -> None:
    """Помечает dirty все pool refs, затронутые только что размещённым chain.

    Два триггера:
    1. Shared vertex — anchor-lookup этого ref мог измениться.
    2. First-chain-in-patch — placed_in_patch изменился 0→1, score всех refs
       этого patch меняется из-за momentum bonus.
    """
    dirty = runtime_policy._dirty_refs
    vtp = runtime_policy._vert_to_pool_refs
    patch_id = chain_ref[0]
    # Phase 2: explicit PatchScoringContext changes after every same-patch placement.

    for vi in chain.vert_indices:
        for ref in vtp.get(vi, ()):
            dirty.add(ref)

    # Проверка == 1 (после инкремента): первый chain в patch
    if runtime_policy.placed_count_by_patch.get(patch_id, 0) == 1:
        for refs_list in vtp.values():
            for ref in refs_list:
                if ref[0] == patch_id:
                    dirty.add(ref)
    for ref in runtime_policy._patch_to_pool_refs.get(patch_id, ()):
        dirty.add(ref)


def _cf_select_best_frontier_candidate(
    runtime_policy: FrontierRuntimePolicy,
    all_chain_pool: list[ChainPoolEntry],
) -> Optional[FrontierPlacementCandidate]:
    """Cache-aware версия выбора лучшего кандидата.

    Переоценивает только dirty refs. Остальные берёт из _cached_evals.
    На первой итерации _cached_evals пуст — все цепочки проходят full eval.
    """
    best_candidate: Optional[FrontierPlacementCandidate] = None
    best_rank: Optional[FrontierRank] = None
    best_viable_candidate: Optional[FrontierPlacementCandidate] = None
    best_viable_rank: Optional[FrontierRank] = None

    for entry in all_chain_pool:
        chain_ref = entry.chain_ref
        if not runtime_policy.is_chain_available(chain_ref):
            continue

        # --- Cache path ---
        if chain_ref not in runtime_policy._dirty_refs:
            cached = runtime_policy._cached_evals.get(chain_ref)
            if cached is not None:
                runtime_policy._cache_hits += 1
                if cached.known == 0:
                    continue
                placement_candidate = _cf_make_frontier_placement_candidate(entry, cached)
                candidate_rank = cached.rank
                if candidate_rank is not None and (best_rank is None or candidate_rank > best_rank):
                    best_rank = candidate_rank
                    best_candidate = placement_candidate
                if (
                    candidate_rank is not None
                    and cached.score >= CHAIN_FRONTIER_THRESHOLD
                    and (best_viable_rank is None or candidate_rank > best_viable_rank)
                ):
                    best_viable_rank = candidate_rank
                    best_viable_candidate = placement_candidate
                continue

        # --- Full evaluation path ---
        candidate_eval = runtime_policy.evaluate_candidate(
            chain_ref,
            entry.chain,
            entry.node,
            apply_closure_preconstraint=True,
            compute_score=True,
        )
        runtime_policy._cached_evals[chain_ref] = candidate_eval
        runtime_policy._dirty_refs.discard(chain_ref)

        if candidate_eval.known == 0:
            continue

        placement_candidate = _cf_make_frontier_placement_candidate(entry, candidate_eval)
        candidate_rank = candidate_eval.rank
        if candidate_rank is not None and (best_rank is None or candidate_rank > best_rank):
            best_rank = candidate_rank
            best_candidate = placement_candidate
        if (
            candidate_rank is not None
            and candidate_eval.score >= CHAIN_FRONTIER_THRESHOLD
            and (best_viable_rank is None or candidate_rank > best_viable_rank)
        ):
            best_viable_rank = candidate_rank
            best_viable_candidate = placement_candidate

    return best_viable_candidate or best_candidate


def _cf_try_place_frontier_candidate(
    runtime_policy: FrontierRuntimePolicy,
    candidate: FrontierPlacementCandidate,
    iteration: int,
    collector: Optional[FrontierTelemetryCollector] = None,
) -> bool:
    graph = runtime_policy.graph
    chain_ref = candidate.chain_ref
    chain = candidate.chain
    _placed_before = runtime_policy.placed_in_patch(chain_ref[0])

    if candidate.anchor_adjustments:
        applied = _cf_apply_anchor_adjustments(
            candidate.anchor_adjustments,
            graph,
            runtime_policy.placed_chains_map,
            runtime_policy.point_registry,
            runtime_policy.final_scale,
        )
        if not applied:
            runtime_policy.reject_chain(chain_ref)
            trace_console(
                f"[CFTUV][Frontier] Reject {iteration}: "
                f"P{chain_ref[0]} L{chain_ref[1]}C{chain_ref[2]} "
                f"{chain.frame_role.value} reason:anchor_rectify_failed"
            )
            return False
        # Phase D: UV points уже placed chains изменились — соседи получают stale anchors
        for adjusted_ref, _, _ in candidate.anchor_adjustments:
            adjusted_chain = graph.get_chain(*adjusted_ref)
            if adjusted_chain is not None:
                _mark_neighbors_dirty(runtime_policy, adjusted_ref, adjusted_chain)

    dir_override = candidate.closure_dir_override
    _closure_dir_was_set = dir_override is not None
    if dir_override is None:
        dir_override = _try_inherit_direction(
            chain,
            candidate.node,
            candidate.start_anchor,
            candidate.end_anchor,
            graph,
            runtime_policy.point_registry,
            chain_ref=chain_ref,
        )
    _direction_inherited = dir_override is not None and not _closure_dir_was_set

    uv_points = _cf_place_chain(
        chain,
        candidate.node,
        candidate.start_anchor,
        candidate.end_anchor,
        runtime_policy.final_scale,
        dir_override,
    )
    if not uv_points or len(uv_points) != len(chain.vert_cos):
        runtime_policy.reject_chain(chain_ref)
        trace_console(
            f"[CFTUV][Frontier] Reject {iteration}: "
            f"P{chain_ref[0]} L{chain_ref[1]}C{chain_ref[2]} "
            f"{chain.frame_role.value} reason:placement_failed"
        )
        return False

    anchor_count = _cf_anchor_count(candidate.start_anchor, candidate.end_anchor)
    chain_placement = ScaffoldChainPlacement(
        patch_id=chain_ref[0],
        loop_index=chain_ref[1],
        chain_index=chain_ref[2],
        frame_role=chain.frame_role,
        source_kind=PlacementSourceKind.CHAIN,
        anchor_count=anchor_count,
        points=tuple(
            (ScaffoldPointKey(chain_ref[0], chain_ref[1], chain_ref[2], i), uv.copy())
            for i, uv in enumerate(uv_points)
        ),
    )

    runtime_policy.register_chain(
        chain_ref,
        chain,
        chain_placement,
        uv_points,
        runtime_policy.dependency_patches_from_anchors(
            chain_ref[0],
            candidate.start_anchor,
            candidate.end_anchor,
        ),
    )

    anchor_label = _cf_anchor_debug_label(candidate.start_anchor, candidate.end_anchor)
    reason_suffix = f" note:{candidate.anchor_reason}" if candidate.anchor_reason else ''
    bridge_tag = ' [BRIDGE]' if chain.frame_role == FrameRole.FREE and len(chain.vert_cos) <= 2 else ''
    hv_suffix = f" hv_adj:{candidate.hv_adjacency}" if chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME} else ''
    rank_suffix = f" rank:{_cf_frontier_rank_debug_label(candidate.rank)}" if candidate.rank is not None else ''
    seam_suffix = f" seam:{candidate.seam_bonus:+.2f}" if abs(candidate.seam_bonus) > 1e-6 else ''
    corner_suffix = f" corner:+{candidate.corner_bonus:.2f}" if candidate.corner_bonus > 0.0 else ''
    shape_suffix = f" shape:{candidate.shape_bonus:+.2f}" if abs(candidate.shape_bonus) > 1e-6 else ''
    ctx_suffix = ''
    if candidate.patch_context is not None:
        ctx_suffix = (
            f" ctx:p{candidate.patch_context.placed_chain_count}"
            f" pr:{candidate.patch_context.placed_ratio:.2f}"
            f" bb:{candidate.patch_context.same_patch_backbone_strength:.2f}"
            f" cp:{candidate.patch_context.closure_pressure:.2f}"
        )
    trace_console(
        f"[CFTUV][Frontier] Step {iteration}: "
        f"P{chain_ref[0]} L{chain_ref[1]}C{chain_ref[2]} "
        f"{chain.frame_role.value} score:{candidate.score:.2f}{rank_suffix}{ctx_suffix}{seam_suffix}{corner_suffix}{shape_suffix} "
        f"ep:{anchor_count} a:{anchor_label}{reason_suffix}{bridge_tag}{hv_suffix}"
    )
    if collector is not None:
        collector.record_placement(
            iteration=iteration,
            chain_ref=chain_ref,
            chain=chain,
            placement_path="main",
            score=candidate.score,
            start_anchor=candidate.start_anchor,
            end_anchor=candidate.end_anchor,
            placed_in_patch_before=_placed_before,
            is_closure_pair=(chain_ref in runtime_policy.closure_pair_refs),
            uv_points=uv_points,
            closure_preconstraint_applied=(candidate.closure_dir_override is not None),
            anchor_adjustment_applied=bool(candidate.anchor_adjustments),
            direction_inherited=_direction_inherited,
            length_factor=candidate.length_factor,
            downstream_count=candidate.downstream_count,
            downstream_bonus=candidate.downstream_bonus,
            isolation_preview=candidate.isolation_preview,
            isolation_penalty=candidate.isolation_penalty,
            structural_free_bonus=candidate.structural_free_bonus,
            hv_adjacency=candidate.hv_adjacency,
            rank=candidate.rank,
            rank_breakdown=candidate.rank_breakdown,
            patch_context=candidate.patch_context,
            corner_hints=candidate.corner_hints,
            seam_relation=candidate.seam_relation,
            seam_bonus=candidate.seam_bonus,
            corner_bonus=candidate.corner_bonus,
            shape_bonus=candidate.shape_bonus,
        )
    return True


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


def _cf_bootstrap_frontier_runtime(
    graph: PatchGraph,
    solve_view: SolveView,
    quilt_plan: QuiltPlan,
    root_node: PatchNode,
    quilt_patch_ids: set[int],
    allowed_tree_edges: set[PatchEdgeKey],
    closure_pair_map: dict[ChainRef, ChainRef],
    final_scale: float,
) -> FrontierBootstrapAttempt:
    seed_result = _cf_choose_seed_chain(solve_view, graph, root_node, quilt_patch_ids, allowed_tree_edges)
    if seed_result is None:
        return FrontierBootstrapAttempt(result=None, error='no_seed_chain')

    seed_loop_idx = seed_result.loop_index
    seed_chain_idx = seed_result.chain_index
    seed_chain = seed_result.chain
    seed_ref = (quilt_plan.root_patch_id, seed_loop_idx, seed_chain_idx)
    runtime_policy = FrontierRuntimePolicy(
        graph=graph,
        quilt_patch_ids=quilt_patch_ids,
        allowed_tree_edges=allowed_tree_edges,
        final_scale=final_scale,
        seam_relation_by_edge=quilt_plan.seam_relation_by_edge,
        closure_pair_map=closure_pair_map,
    )

    seed_payload = _cf_build_seed_placement(seed_ref, seed_chain, root_node, final_scale)
    if seed_payload is None:
        return FrontierBootstrapAttempt(result=None, error='seed_placement_failed')

    seed_placement = seed_payload.placement
    seed_uvs = seed_payload.uv_points
    runtime_policy.register_chain(seed_ref, seed_chain, seed_placement, seed_uvs, ())

    trace_console(
        f"[CFTUV][Frontier] Seed: P{seed_ref[0]} L{seed_ref[1]}C{seed_ref[2]} "
        f"{seed_chain.frame_role.value} "
        f"({seed_uvs[0].x:.4f},{seed_uvs[0].y:.4f})"
        f"->({seed_uvs[-1].x:.4f},{seed_uvs[-1].y:.4f})"
        f"{f' hv_adj:{_cf_count_hv_adjacent_endpoints(graph, seed_ref)}' if seed_chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME} else ''}"
    )
    if allowed_tree_edges:
        edge_labels = [f"{edge[0]}-{edge[1]}" for edge in sorted(allowed_tree_edges)]
        trace_console(f"[CFTUV][Frontier] Tree edges: {edge_labels}")

    return FrontierBootstrapAttempt(
        result=FrontierBootstrapResult(
            runtime_policy=runtime_policy,
            seed_ref=seed_ref,
            seed_chain=seed_chain,
            seed_score=seed_result.score,
        )
    )


def _cf_build_frontier_chain_pool(
    solve_view: SolveView,
    graph: PatchGraph,
    ordered_quilt_patch_ids: list[int],
    seed_ref: ChainRef,
) -> list[ChainPoolEntry]:
    all_chain_pool: list[ChainPoolEntry] = []
    for patch_id in ordered_quilt_patch_ids:
        node = graph.nodes.get(patch_id)
        if node is None:
            continue
        for loop_idx, chain_idx, _loop, chain in solve_view.iter_visible_chains(patch_id):
            chain_ref = (patch_id, loop_idx, chain_idx)
            if chain_ref != seed_ref:
                all_chain_pool.append(ChainPoolEntry(
                    chain_ref=chain_ref,
                    chain=chain,
                    node=node,
                ))
    return all_chain_pool


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


def _cf_frame_cross_axis_value(role: FrameRole, uv: Vector) -> float:
    if role == FrameRole.H_FRAME:
        return uv.y
    if role == FrameRole.V_FRAME:
        return uv.x
    return 0.0


def _cf_with_frame_cross_axis(role: FrameRole, uv: Vector, cross_axis_value: float) -> Vector:
    if role == FrameRole.H_FRAME:
        return Vector((uv.x, cross_axis_value))
    if role == FrameRole.V_FRAME:
        return Vector((cross_axis_value, uv.y))
    return uv.copy()


def _cf_preview_anchor_source_adjustment(
    graph: PatchGraph,
    placed_chains_map: dict[ChainRef, ScaffoldChainPlacement],
    chain_role: FrameRole,
    anchor: ChainAnchor,
    snapped_uv: Vector,
) -> bool:
    if anchor.source_kind != PlacementSourceKind.SAME_PATCH:
        return True

    source_placement = placed_chains_map.get(anchor.source_ref)
    if source_placement is None or not source_placement.points:
        return False

    source_point_index = anchor.source_point_index
    point_count = len(source_placement.points)
    if source_point_index < 0 or source_point_index >= point_count:
        return False
    if source_point_index not in {0, point_count - 1}:
        return False

    source_role = source_placement.frame_role
    current_uv = source_placement.points[source_point_index][1]
    delta = snapped_uv - current_uv

    if source_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        orthogonal = (
            (chain_role == FrameRole.H_FRAME and source_role == FrameRole.V_FRAME)
            or (chain_role == FrameRole.V_FRAME and source_role == FrameRole.H_FRAME)
        )
        if not orthogonal:
            return False
        if source_role == FrameRole.H_FRAME and abs(delta.y) > 1e-6:
            return False
        if source_role == FrameRole.V_FRAME and abs(delta.x) > 1e-6:
            return False
        return True

    if source_role == FrameRole.FREE and point_count <= 2:
        return True

    return False


def _cf_same_patch_anchor_is_protected(graph: PatchGraph, anchor: ChainAnchor) -> bool:
    if anchor.source_kind != PlacementSourceKind.SAME_PATCH:
        return True
    source_chain = graph.get_chain(anchor.source_ref[0], anchor.source_ref[1], anchor.source_ref[2])
    if source_chain is None:
        return True
    return source_chain.neighbor_kind == ChainNeighborKind.PATCH


def _cf_preview_frame_dual_anchor_rectification(
    chain: BoundaryChain,
    start_anchor: Optional[ChainAnchor],
    end_anchor: Optional[ChainAnchor],
    graph: PatchGraph,
    placed_chains_map: dict[ChainRef, ScaffoldChainPlacement],
) -> DualAnchorRectificationPreview:
    # Локальная per-chain rectification для H/V оказалась слишком слабой моделью:
    # она умеет выпрямлять отдельный chain, но не умеет согласованно решать
    # весь frame-граф patch/quilt. В результате появлялись ступеньки между
    # геометрически коллинеарными H/V chains. До появления patch-level
    # orthogonal solve runtime placement должен оставаться без этой
    # коррекции, чтобы не ломать stitch continuity и row consistency.
    return DualAnchorRectificationPreview(start_anchor=start_anchor, end_anchor=end_anchor)

    if chain.frame_role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        return DualAnchorRectificationPreview(start_anchor=start_anchor, end_anchor=end_anchor)
    if start_anchor is None or end_anchor is None:
        return DualAnchorRectificationPreview(start_anchor=start_anchor, end_anchor=end_anchor)
    if start_anchor.source_kind == PlacementSourceKind.CROSS_PATCH and end_anchor.source_kind == PlacementSourceKind.CROSS_PATCH:
        return DualAnchorRectificationPreview(start_anchor=start_anchor, end_anchor=end_anchor)

    start_cross = _cf_frame_cross_axis_value(chain.frame_role, start_anchor.uv)
    end_cross = _cf_frame_cross_axis_value(chain.frame_role, end_anchor.uv)
    target_cross = None
    adjust_start = False
    adjust_end = False
    reason = ''
    start_protected = _cf_same_patch_anchor_is_protected(graph, start_anchor)
    end_protected = _cf_same_patch_anchor_is_protected(graph, end_anchor)

    if start_anchor.source_kind == PlacementSourceKind.SAME_PATCH and end_anchor.source_kind == PlacementSourceKind.SAME_PATCH:
        if start_protected and end_protected:
            return DualAnchorRectificationPreview(start_anchor=start_anchor, end_anchor=end_anchor)
        if start_protected and not end_protected:
            target_cross = start_cross
            adjust_end = True
            reason = 'axis_hard:keep_protected_start'
        elif end_protected and not start_protected:
            target_cross = end_cross
            adjust_start = True
            reason = 'axis_hard:keep_protected_end'
        else:
            target_cross = 0.5 * (start_cross + end_cross)
            adjust_start = True
            adjust_end = True
            reason = 'axis_hard:average_same_patch'
    elif start_anchor.source_kind == PlacementSourceKind.SAME_PATCH:
        if start_protected:
            return DualAnchorRectificationPreview(start_anchor=start_anchor, end_anchor=end_anchor)
        target_cross = end_cross
        adjust_start = True
        reason = 'axis_hard:lock_end'
    elif end_anchor.source_kind == PlacementSourceKind.SAME_PATCH:
        if end_protected:
            return DualAnchorRectificationPreview(start_anchor=start_anchor, end_anchor=end_anchor)
        target_cross = start_cross
        adjust_end = True
        reason = 'axis_hard:lock_start'
    else:
        return DualAnchorRectificationPreview(start_anchor=start_anchor, end_anchor=end_anchor)

    snapped_start_uv = _cf_with_frame_cross_axis(chain.frame_role, start_anchor.uv, target_cross)
    snapped_end_uv = _cf_with_frame_cross_axis(chain.frame_role, end_anchor.uv, target_cross)
    adjustments = []

    if adjust_start:
        if not _cf_preview_anchor_source_adjustment(graph, placed_chains_map, chain.frame_role, start_anchor, snapped_start_uv):
            return DualAnchorRectificationPreview(start_anchor=start_anchor, end_anchor=end_anchor)
        adjustments.append((start_anchor.source_ref, start_anchor.source_point_index, snapped_start_uv.copy()))
    if adjust_end:
        if not _cf_preview_anchor_source_adjustment(graph, placed_chains_map, chain.frame_role, end_anchor, snapped_end_uv):
            return DualAnchorRectificationPreview(start_anchor=start_anchor, end_anchor=end_anchor)
        adjustments.append((end_anchor.source_ref, end_anchor.source_point_index, snapped_end_uv.copy()))

    return DualAnchorRectificationPreview(
        start_anchor=ChainAnchor(
            uv=snapped_start_uv,
            source_ref=start_anchor.source_ref,
            source_point_index=start_anchor.source_point_index,
            source_kind=start_anchor.source_kind,
        ),
        end_anchor=ChainAnchor(
            uv=snapped_end_uv,
            source_ref=end_anchor.source_ref,
            source_point_index=end_anchor.source_point_index,
            source_kind=end_anchor.source_kind,
        ),
        reason=reason,
        anchor_adjustments=tuple(adjustments),
    )


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


def _cf_choose_seed_chain(
    solve_view: SolveView,
    graph: PatchGraph,
    root_node: PatchNode,
    quilt_patch_ids: set[int],
    allowed_tree_edges: set[PatchEdgeKey],
) -> Optional[SeedChainChoice]:
    """Выбирает strongest chain root patch для seed placement.

    Семантический bonus учитывает только patches того же quilt.
    Иначе caps/соседи из других quilts искажают seed для wall patch.

    Returns: strongest typed seed-chain choice или None
    """
    best_ref: Optional[SeedChainChoice] = None
    best_rank: Optional[tuple[int, int, float]] = None

    for loop_idx, loop in solve_view.iter_visible_loops(root_node.patch_id):
        for chain_idx, chain in enumerate(loop.chains):
            chain_ref = (root_node.patch_id, loop_idx, chain_idx)
            is_hv = chain.frame_role in (FrameRole.H_FRAME, FrameRole.V_FRAME)
            hv_adjacency = _cf_count_hv_adjacent_endpoints(graph, chain_ref)
            if is_hv and hv_adjacency <= 0:
                continue

            score = 0.0

            if is_hv:
                score += 1.0
            else:
                score += 0.1

            if len(chain.vert_cos) > 1:
                chain_len = sum(
                    (chain.vert_cos[i + 1] - chain.vert_cos[i]).length
                    for i in range(len(chain.vert_cos) - 1)
                )
                score += min(chain_len * 0.1, 0.5)

            if (
                chain.neighbor_kind == ChainNeighborKind.PATCH
                and chain.neighbor_patch_id in quilt_patch_ids
                and _is_allowed_quilt_edge(allowed_tree_edges, root_node.patch_id, chain.neighbor_patch_id)
            ):
                neighbor_key = graph.get_patch_semantic_key(chain.neighbor_patch_id)
                patch_type = root_node.patch_type.value if hasattr(root_node.patch_type, 'value') else str(root_node.patch_type)
                if patch_type == 'WALL':
                    if neighbor_key == 'FLOOR.DOWN':
                        score += 0.5
                    elif neighbor_key == 'FLOOR.UP':
                        score += 0.2
                    elif neighbor_key.endswith('.SIDE'):
                        score += 0.15
                else:
                    if neighbor_key.endswith('.SIDE'):
                        score += 0.2

            rank = (
                1 if is_hv else 0,
                hv_adjacency if is_hv else -1,
                score,
            )
            if best_rank is None or rank > best_rank:
                best_rank = rank
                best_ref = SeedChainChoice(
                    loop_index=loop_idx,
                    chain_index=chain_idx,
                    chain=chain,
                    score=score,
                )

    return best_ref


def _cf_find_anchors(
    chain_ref: ChainRef,
    chain: BoundaryChain,
    graph: PatchGraph,
    point_registry: PointRegistry,
    vert_to_placements: VertexPlacementMap,
    placed_refs: set[ChainRef],
    allowed_tree_edges: set[PatchEdgeKey],
) -> FoundCandidateAnchors:
    """Ищет anchor UV для start/end вершин chain с provenance.

    same_patch anchor берётся только через corner topology.
    cross_patch anchor разрешён только через shared seam vert tree-ребра quilt.
    """
    patch_id, loop_index, chain_index = chain_ref
    node = graph.nodes.get(patch_id)
    if node is None or loop_index >= len(node.boundary_loops):
        return FoundCandidateAnchors(start_anchor=None, end_anchor=None)

    boundary_loop = node.boundary_loops[loop_index]
    start_vert = chain.start_vert_index
    end_vert = chain.end_vert_index
    seam_neighbor_patch_id = chain.neighbor_patch_id if chain.neighbor_kind == ChainNeighborKind.PATCH else None
    start_anchor = None
    end_anchor = None

    for corner in boundary_loop.corners:
        prev_ci = corner.prev_chain_index
        next_ci = corner.next_chain_index

        if next_ci == chain_index and start_anchor is None:
            prev_ref = (patch_id, loop_index, prev_ci)
            if prev_ref in placed_refs:
                prev_chain = boundary_loop.chains[prev_ci]
                prev_last = len(prev_chain.vert_indices) - 1
                if prev_last >= 0:
                    key = _point_registry_key(prev_ref, prev_last)
                    if key in point_registry:
                        start_anchor = ChainAnchor(
                            uv=point_registry[key].copy(),
                            source_ref=prev_ref,
                            source_point_index=prev_last,
                            source_kind=PlacementSourceKind.SAME_PATCH,
                        )

        if prev_ci == chain_index and end_anchor is None:
            next_ref = (patch_id, loop_index, next_ci)
            if next_ref in placed_refs:
                key = _point_registry_key(next_ref, 0)
                if key in point_registry:
                    end_anchor = ChainAnchor(
                        uv=point_registry[key].copy(),
                        source_ref=next_ref,
                        source_point_index=0,
                        source_kind=PlacementSourceKind.SAME_PATCH,
                    )

    if start_anchor is None and start_vert >= 0 and start_vert in vert_to_placements:
        for other_ref, pt_idx in vert_to_placements[start_vert]:
            if other_ref[0] == patch_id:
                continue
            
            # Anchor filter layer 2: allow junction anchors for WALL/FLOOR even if not seam neighbors
            is_junction_allowed = False
            if seam_neighbor_patch_id is not None and other_ref[0] != seam_neighbor_patch_id:
                other_node = graph.nodes.get(other_ref[0])
                if other_node is not None:
                    t1, t2 = node.patch_type, other_node.patch_type
                    if (t1 == PatchType.WALL and t2 in (PatchType.FLOOR, PatchType.SLOPE)) or \
                       (t2 == PatchType.WALL and t1 in (PatchType.FLOOR, PatchType.SLOPE)):
                        is_junction_allowed = True
            
            if seam_neighbor_patch_id is not None and other_ref[0] != seam_neighbor_patch_id and not is_junction_allowed:
                continue
            if not _is_allowed_quilt_edge(allowed_tree_edges, patch_id, other_ref[0]):
                continue
            key = _point_registry_key(other_ref, pt_idx)
            if key in point_registry:
                start_anchor = ChainAnchor(
                    uv=point_registry[key].copy(),
                    source_ref=other_ref,
                    source_point_index=pt_idx,
                    source_kind=PlacementSourceKind.CROSS_PATCH,
                )
                break

    if end_anchor is None and end_vert >= 0 and end_vert in vert_to_placements:
        for other_ref, pt_idx in vert_to_placements[end_vert]:
            if other_ref[0] == patch_id:
                continue
            
            # Anchor filter layer 2: allow junction anchors for WALL/FLOOR even if not seam neighbors
            is_junction_allowed = False
            if seam_neighbor_patch_id is not None and other_ref[0] != seam_neighbor_patch_id:
                other_node = graph.nodes.get(other_ref[0])
                if other_node is not None:
                    t1, t2 = node.patch_type, other_node.patch_type
                    if (t1 == PatchType.WALL and t2 in (PatchType.FLOOR, PatchType.SLOPE)) or \
                       (t2 == PatchType.WALL and t1 in (PatchType.FLOOR, PatchType.SLOPE)):
                        is_junction_allowed = True

            if seam_neighbor_patch_id is not None and other_ref[0] != seam_neighbor_patch_id and not is_junction_allowed:
                continue
            if not _is_allowed_quilt_edge(allowed_tree_edges, patch_id, other_ref[0]):
                continue
            key = _point_registry_key(other_ref, pt_idx)
            if key in point_registry:
                end_anchor = ChainAnchor(
                    uv=point_registry[key].copy(),
                    source_ref=other_ref,
                    source_point_index=pt_idx,
                    source_kind=PlacementSourceKind.CROSS_PATCH,
                )
                break

    return FoundCandidateAnchors(start_anchor=start_anchor, end_anchor=end_anchor)


def _cf_frame_anchor_pair_is_axis_safe(
    chain: BoundaryChain,
    start_anchor: ChainAnchor,
    end_anchor: ChainAnchor,
    final_scale: float,
) -> AnchorPairSafetyDecision:
    if chain.frame_role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        return AnchorPairSafetyDecision(is_safe=True)

    total_length = _cf_chain_total_length(chain, final_scale)
    if total_length <= 1e-8:
        return AnchorPairSafetyDecision(is_safe=True)

    delta = end_anchor.uv - start_anchor.uv
    axis_error = abs(delta.y) if chain.frame_role == FrameRole.H_FRAME else abs(delta.x)
    axis_span = abs(delta.x) if chain.frame_role == FrameRole.H_FRAME else abs(delta.y)

    axis_tolerance = max(0.02, total_length * 0.05)
    span_tolerance = max(0.05, total_length * 0.15)

    if axis_error > axis_tolerance:
        return AnchorPairSafetyDecision(is_safe=False, reason='axis_mismatch')
    if abs(axis_span - total_length) > span_tolerance:
        return AnchorPairSafetyDecision(is_safe=False, reason='span_mismatch')
    return AnchorPairSafetyDecision(is_safe=True)


def _cf_can_use_dual_anchor_closure(
    chain: BoundaryChain,
    start_anchor: ChainAnchor,
    end_anchor: ChainAnchor,
    placed_in_patch: int,
    final_scale: float,
) -> DualAnchorClosureDecision:
    axis_safety = _cf_frame_anchor_pair_is_axis_safe(chain, start_anchor, end_anchor, final_scale)
    if not axis_safety.is_safe:
        if (
            axis_safety.reason == 'span_mismatch'
            and chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
            and start_anchor.source_kind == PlacementSourceKind.SAME_PATCH
            and end_anchor.source_kind == PlacementSourceKind.SAME_PATCH
        ):
            axis_safety = AnchorPairSafetyDecision(is_safe=True)
        else:
            return DualAnchorClosureDecision(can_close=False, reason=axis_safety.reason)

    # Не позволяем patch замкнуться только по двум сужим anchors.
    # Иначе второй seam-chain "прилипает" к обоим сторонам уже размещённого patch.
    if start_anchor.source_kind == PlacementSourceKind.CROSS_PATCH and end_anchor.source_kind == PlacementSourceKind.CROSS_PATCH:
        # Короткие FREE chains (bridge, ≤2 вершины) — позиция полностью определена anchors,
        # wrap невозможен для одного ребра.
        if chain.frame_role == FrameRole.FREE and len(chain.vert_cos) <= 2:
            return DualAnchorClosureDecision(can_close=True)
        return DualAnchorClosureDecision(can_close=False, reason='prevent_patch_wrap')

    return DualAnchorClosureDecision(can_close=True)


def _cf_resolve_candidate_anchors(
    chain: BoundaryChain,
    start_anchor: Optional[ChainAnchor],
    end_anchor: Optional[ChainAnchor],
    placed_in_patch: int,
    final_scale: float,
    graph: PatchGraph,
    placed_chains_map: dict[ChainRef, ScaffoldChainPlacement],
) -> ResolvedCandidateAnchors:
    known = _cf_anchor_count(start_anchor, end_anchor)
    if known < 2:
        return ResolvedCandidateAnchors(start_anchor=start_anchor, end_anchor=end_anchor, known=known)

    rectified = _cf_preview_frame_dual_anchor_rectification(
        chain,
        start_anchor,
        end_anchor,
        graph,
        placed_chains_map,
    )
    start_anchor = rectified.start_anchor
    end_anchor = rectified.end_anchor
    rect_reason = rectified.reason
    anchor_adjustments = rectified.anchor_adjustments

    closure_decision = _cf_can_use_dual_anchor_closure(
        chain,
        start_anchor,
        end_anchor,
        placed_in_patch,
        final_scale,
    )
    if closure_decision.can_close:
        return ResolvedCandidateAnchors(
            start_anchor=start_anchor,
            end_anchor=end_anchor,
            known=2,
            reason=rect_reason,
            anchor_adjustments=anchor_adjustments,
        )
    reason = closure_decision.reason

    if (
        start_anchor is not None and end_anchor is not None
        and start_anchor.source_kind == PlacementSourceKind.CROSS_PATCH
        and end_anchor.source_kind == PlacementSourceKind.CROSS_PATCH
    ):
        reason_note = f'{rect_reason}|{reason}' if rect_reason else reason
        return ResolvedCandidateAnchors(
            start_anchor=start_anchor,
            end_anchor=None,
            known=1,
            reason=f'{reason_note}:bootstrap_from_start',
        )

    if start_anchor is not None and start_anchor.source_kind == PlacementSourceKind.SAME_PATCH and (
        end_anchor is None or end_anchor.source_kind != PlacementSourceKind.SAME_PATCH
    ):
        reason_note = f'{rect_reason}|{reason}' if rect_reason else reason
        return ResolvedCandidateAnchors(
            start_anchor=start_anchor,
            end_anchor=None,
            known=1,
            reason=f'{reason_note}:drop_end',
        )
    if end_anchor is not None and end_anchor.source_kind == PlacementSourceKind.SAME_PATCH and (
        start_anchor is None or start_anchor.source_kind != PlacementSourceKind.SAME_PATCH
    ):
        reason_note = f'{rect_reason}|{reason}' if rect_reason else reason
        return ResolvedCandidateAnchors(
            start_anchor=None,
            end_anchor=end_anchor,
            known=1,
            reason=f'{reason_note}:drop_start',
        )

    if (
        start_anchor is not None and end_anchor is not None
        and start_anchor.source_kind == PlacementSourceKind.SAME_PATCH
        and end_anchor.source_kind == PlacementSourceKind.SAME_PATCH
        and chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
    ):
        reason_note = f'{rect_reason}|{reason}' if rect_reason else reason
        return ResolvedCandidateAnchors(
            start_anchor=start_anchor,
            end_anchor=None,
            known=1,
            reason=f'{reason_note}:axis_soft_from_start',
        )

    return ResolvedCandidateAnchors(start_anchor=None, end_anchor=None, known=0, reason=reason)


def _cf_estimate_downstream_anchor_count(
    chain_ref: ChainRef,
    chain: BoundaryChain,
    graph: PatchGraph,
    runtime_policy: "FrontierRuntimePolicy",
) -> int:
    """Count unplaced chains that would gain an anchor from placing chain_ref."""
    count = 0
    # Same-patch neighbors via corners
    patch_id, loop_index, chain_index = chain_ref
    node = graph.nodes.get(patch_id)
    if node is None or loop_index >= len(node.boundary_loops):
        return 0
    boundary_loop = node.boundary_loops[loop_index]
    for corner in boundary_loop.corners:
        neighbor_idx = -1
        if corner.next_chain_index == chain_index:
            neighbor_idx = corner.prev_chain_index
        elif corner.prev_chain_index == chain_index:
            neighbor_idx = corner.next_chain_index
        if neighbor_idx >= 0:
            ref = (patch_id, loop_index, neighbor_idx)
            if runtime_policy.is_chain_available(ref):
                count += 1
    # Cross-patch neighbors via shared verts
    for vert_idx in (chain.start_vert_index, chain.end_vert_index):
        if vert_idx < 0:
            continue
        for ref in runtime_policy._vert_to_pool_refs.get(vert_idx, ()):
            if ref[0] != patch_id and runtime_policy.is_chain_available(ref):
                count += 1
    return count


def _cf_count_hv_adjacent_endpoints(
    graph: PatchGraph,
    chain_ref: ChainRef,
) -> int:
    """Count H/V-supported endpoints for one local H/V chain."""
    chain = graph.get_chain(*chain_ref)
    if chain is None or chain.frame_role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        return 0

    endpoint_neighbors = graph.get_chain_endpoint_neighbors(chain_ref[0], chain_ref[1], chain_ref[2])
    hv_adjacency = 0
    for endpoint_label in ("start", "end"):
        for neighbor_loop_index, neighbor_chain_index in endpoint_neighbors.get(endpoint_label, ()):
            neighbor_chain = graph.get_chain(chain_ref[0], neighbor_loop_index, neighbor_chain_index)
            if neighbor_chain is None:
                continue
            if neighbor_chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
                hv_adjacency += 1
                break
    return hv_adjacency


def _cf_preview_would_be_connected(
    chain_ref: ChainRef,
    chain: BoundaryChain,
    runtime_policy: "FrontierRuntimePolicy",
    graph: PatchGraph,
) -> bool:
    """Would this chain be scaffold-connected if placed now?"""
    if chain.frame_role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        return True  # Only H/V can be isolated
    
    patch_id, loop_index, chain_index = chain_ref
    # If no chains placed in this patch yet — it will be root, always connected
    if runtime_policy.placed_in_patch(patch_id) == 0:
        return True
    
    # Check if any neighbor in same loop is already placed and connected
    node = graph.nodes.get(patch_id)
    if node is None or loop_index >= len(node.boundary_loops):
        return True
    boundary_loop = node.boundary_loops[loop_index]
    total_chains = len(boundary_loop.chains)
    
    for delta in (-1, 1):
        neighbor_idx = (chain_index + delta) % total_chains
        neighbor_ref = (patch_id, loop_index, neighbor_idx)
        if neighbor_ref not in runtime_policy.placed_chain_refs:
            continue
        neighbor_chain = boundary_loop.chains[neighbor_idx]
        if neighbor_chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
            return True
    
    # Layer 3: check cross-patch H/V connections
    for vert_idx in (chain.start_vert_index, chain.end_vert_index):
        if vert_idx < 0:
            continue
        for other_ref in runtime_policy._vert_to_pool_refs.get(vert_idx, ()):
            if other_ref in runtime_policy.placed_chain_refs:
                other_chain = graph.get_chain(*other_ref)
                if other_chain and other_chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
                    return True

    return False


def _cf_role_tier(
    chain_ref: ChainRef,
    chain: BoundaryChain,
    node: PatchNode,
    graph: PatchGraph,
    quilt_patch_ids: set[int],
    allowed_tree_edges: set[PatchEdgeKey],
) -> tuple[int, str]:
    """Грубый role-tier, сохраняющий текущую scalar priority ladder."""
    if chain.frame_role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        if len(chain.vert_cos) <= 2:
            return 1, 'free_bridge'
        return 0, 'free_regular'

    if chain.is_corner_split:
        return 2, 'hv_corner_split'

    if (
        chain.neighbor_kind == ChainNeighborKind.PATCH
        and chain.neighbor_patch_id in quilt_patch_ids
        and _is_allowed_quilt_edge(allowed_tree_edges, chain_ref[0], chain.neighbor_patch_id)
    ):
        neighbor_node = graph.nodes.get(chain.neighbor_patch_id)
        if neighbor_node is not None and neighbor_node.patch_type == node.patch_type:
            return 5, 'hv_cross_patch_same_type'
        return 4, 'hv_cross_patch_mixed_type'

    return 3, 'hv_regular'


def _cf_clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def _cf_chain_polyline_length_3d(chain: BoundaryChain) -> float:
    if len(chain.vert_cos) < 2:
        return 0.0
    return sum(
        (chain.vert_cos[index + 1] - chain.vert_cos[index]).length
        for index in range(len(chain.vert_cos) - 1)
    )


def _cf_patch_shape_backbone_bias(shape_profile: Optional[PatchShapeProfile]) -> float:
    if shape_profile is None:
        return 0.0
    return _cf_clamp01(
        (0.45 * shape_profile.rectilinearity)
        + (0.35 * shape_profile.frame_dominance)
        + (0.20 * shape_profile.elongation)
    )


def _cf_patch_shape_closure_sensitivity(shape_profile: Optional[PatchShapeProfile]) -> float:
    if shape_profile is None:
        return 0.0
    return _cf_clamp01(
        (0.45 * shape_profile.hole_ratio)
        + (0.35 * shape_profile.seam_multiplicity_hint)
        + (0.20 * shape_profile.elongation)
    )


def _cf_build_patch_shape_profile(
    patch_id: int,
    node: PatchNode,
    runtime_policy: "FrontierRuntimePolicy",
) -> PatchShapeProfile:
    """Грубый shape-profile patch без превращения shape в solve-mode."""
    outer_points: list[Vector] = []
    outer_chain_count = 0
    hole_chain_count = 0
    total_outer_length = 0.0
    hv_outer_length = 0.0
    h_outer_length = 0.0
    v_outer_length = 0.0
    patch_neighbor_chain_count = 0
    patch_neighbor_ids: set[int] = set()

    for boundary_loop in node.boundary_loops:
        if boundary_loop.kind == LoopKind.OUTER:
            outer_points.extend(boundary_loop.vert_cos)
            for chain in boundary_loop.chains:
                outer_chain_count += 1
                chain_length = _cf_chain_polyline_length_3d(chain)
                total_outer_length += chain_length
                if chain.frame_role == FrameRole.H_FRAME:
                    hv_outer_length += chain_length
                    h_outer_length += chain_length
                elif chain.frame_role == FrameRole.V_FRAME:
                    hv_outer_length += chain_length
                    v_outer_length += chain_length
                if chain.neighbor_kind == ChainNeighborKind.PATCH:
                    patch_neighbor_chain_count += 1
                    if chain.neighbor_patch_id >= 0:
                        patch_neighbor_ids.add(chain.neighbor_patch_id)
        elif boundary_loop.kind == LoopKind.HOLE:
            hole_chain_count += len(boundary_loop.chains)

    if not outer_points:
        outer_points.extend(node.mesh_verts)
    if not outer_points:
        outer_points.append(node.centroid)

    origin = node.centroid
    projected_u = [(co - origin).dot(node.basis_u) for co in outer_points]
    projected_v = [(co - origin).dot(node.basis_v) for co in outer_points]
    width = (max(projected_u) - min(projected_u)) if projected_u else 0.0
    height = (max(projected_v) - min(projected_v)) if projected_v else 0.0
    long_span = max(width, height)
    short_span = min(width, height)
    elongation = 0.0 if long_span <= 1e-8 else _cf_clamp01(1.0 - (short_span / long_span))

    if total_outer_length > 1e-8:
        rectilinearity = _cf_clamp01(hv_outer_length / total_outer_length)
    elif outer_chain_count > 0:
        rectilinearity = _cf_clamp01(runtime_policy.frame_chain_count(patch_id) / outer_chain_count)
    else:
        rectilinearity = 0.0

    hv_outer_total = h_outer_length + v_outer_length
    if hv_outer_total > 1e-8:
        frame_dominance = _cf_clamp01(abs(h_outer_length - v_outer_length) / hv_outer_total)
    else:
        frame_dominance = 0.0

    total_chain_count = outer_chain_count + hole_chain_count
    hole_ratio = _cf_clamp01(hole_chain_count / total_chain_count) if total_chain_count > 0 else 0.0

    closure_density = (
        _cf_clamp01(runtime_policy.closure_pair_count(patch_id) / outer_chain_count)
        if outer_chain_count > 0 else 0.0
    )
    repeated_neighbor_density = (
        _cf_clamp01(max(0, patch_neighbor_chain_count - len(patch_neighbor_ids)) / outer_chain_count)
        if outer_chain_count > 0 else 0.0
    )
    seam_multiplicity_hint = max(closure_density, repeated_neighbor_density)

    return PatchShapeProfile(
        elongation=elongation,
        rectilinearity=rectilinearity,
        hole_ratio=hole_ratio,
        frame_dominance=frame_dominance,
        seam_multiplicity_hint=seam_multiplicity_hint,
    )


def _cf_build_patch_scoring_context(
    chain_ref: ChainRef,
    runtime_policy: "FrontierRuntimePolicy",
) -> PatchScoringContext:
    """Явный runtime snapshot состояния patch вокруг frontier-кандидата."""
    patch_id = chain_ref[0]
    placed_chain_count = runtime_policy.placed_in_patch(patch_id)
    placed_h_count = runtime_policy.placed_h_in_patch(patch_id)
    placed_v_count = runtime_policy.placed_v_in_patch(patch_id)
    placed_free_count = runtime_policy.placed_free_in_patch(patch_id)
    outer_chain_count = runtime_policy.outer_chain_count(patch_id)
    frame_chain_count = runtime_policy.frame_chain_count(patch_id)
    closure_pair_count = runtime_policy.closure_pair_count(patch_id)

    placed_ratio = (placed_chain_count / outer_chain_count) if outer_chain_count > 0 else 0.0
    frame_progress = placed_h_count + placed_v_count
    hv_coverage_ratio = (frame_progress / frame_chain_count) if frame_chain_count > 0 else 0.0
    same_patch_backbone_strength = hv_coverage_ratio if frame_chain_count > 0 else placed_ratio

    shape_profile = runtime_policy.shape_profile(patch_id)
    shape_closure_sensitivity = _cf_patch_shape_closure_sensitivity(shape_profile)
    closure_base = (closure_pair_count / outer_chain_count) if outer_chain_count > 0 else 0.0
    progress_relief = same_patch_backbone_strength if frame_chain_count > 0 else placed_ratio
    closure_pressure = _cf_clamp01(
        (closure_base * (1.0 - progress_relief))
        + (0.15 * shape_closure_sensitivity)
    )

    return PatchScoringContext(
        patch_id=patch_id,
        placed_chain_count=placed_chain_count,
        placed_h_count=placed_h_count,
        placed_v_count=placed_v_count,
        placed_free_count=placed_free_count,
        placed_ratio=placed_ratio,
        hv_coverage_ratio=hv_coverage_ratio,
        same_patch_backbone_strength=same_patch_backbone_strength,
        closure_pressure=closure_pressure,
        is_untouched=(placed_chain_count == 0),
        has_secondary_seam_pairs=(closure_pair_count > 0),
        shape_profile=shape_profile,
    )


def _cf_chain_seam_relation(
    chain_ref: ChainRef,
    chain: BoundaryChain,
    runtime_policy: "FrontierRuntimePolicy",
) -> Optional[SeamRelationProfile]:
    if chain.neighbor_kind != ChainNeighborKind.PATCH or chain.neighbor_patch_id < 0:
        return None
    return runtime_policy.seam_relation(chain_ref[0], chain.neighbor_patch_id)


def _cf_seam_relation_membership(
    seam_relation: Optional[SeamRelationProfile],
    chain_ref: ChainRef,
) -> str:
    if seam_relation is None:
        return 'none'
    if chain_ref in seam_relation.primary_pair:
        return 'primary'
    for pair in seam_relation.secondary_pairs:
        if chain_ref in pair:
            return 'secondary'
    return 'support'


def _cf_corner_turn_strength(
    corner: BoundaryCorner,
    chain_role: FrameRole,
    other_role: Optional[FrameRole],
) -> float:
    angle = abs(float(corner.turn_angle_deg))
    if angle <= 1e-6:
        return 0.0
    if (
        other_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
        and chain_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
        and _is_orthogonal_hv_pair(chain_role, other_role)
    ):
        return max(0.0, 1.0 - (min(abs(angle - 90.0), 90.0) / 90.0))
    return min(1.0, angle / 180.0)


def _cf_build_corner_scoring_hints(
    chain_ref: ChainRef,
    chain: BoundaryChain,
    graph: PatchGraph,
) -> CornerScoringHints:
    """Локальные corner-derived hints для одного chain-кандидата."""
    patch_id, loop_index, chain_index = chain_ref
    node = graph.nodes.get(patch_id)
    if node is None or loop_index < 0 or loop_index >= len(node.boundary_loops):
        return CornerScoringHints()

    boundary_loop = node.boundary_loops[loop_index]

    def _endpoint_corner(corner_index: int) -> tuple[Optional[BoundaryCorner], Optional[FrameRole]]:
        if corner_index < 0 or corner_index >= len(boundary_loop.corners):
            return None, None
        corner = boundary_loop.corners[corner_index]
        other_role: Optional[FrameRole] = None
        if corner.prev_chain_index == chain_index and corner.next_chain_index != chain_index:
            other_role = corner.next_role
        elif corner.next_chain_index == chain_index and corner.prev_chain_index != chain_index:
            other_role = corner.prev_role
        return corner, other_role

    start_corner, start_other_role = _endpoint_corner(chain.start_corner_index)
    end_corner, end_other_role = _endpoint_corner(chain.end_corner_index)

    start_turn_strength = (
        _cf_corner_turn_strength(start_corner, chain.frame_role, start_other_role)
        if start_corner is not None else 0.0
    )
    end_turn_strength = (
        _cf_corner_turn_strength(end_corner, chain.frame_role, end_other_role)
        if end_corner is not None else 0.0
    )

    orthogonal_turn_count = 0
    same_role_continuation_strength = 0.0
    has_geometric_corner = False
    has_junction_corner = False

    for corner, other_role in (
        (start_corner, start_other_role),
        (end_corner, end_other_role),
    ):
        if corner is None:
            continue
        if corner.is_geometric:
            has_geometric_corner = True
        else:
            has_junction_corner = True
        if (
            other_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
            and chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
            and _is_orthogonal_hv_pair(chain.frame_role, other_role)
        ):
            orthogonal_turn_count += 1
        if other_role == chain.frame_role:
            same_role_continuation_strength += 0.5 if not corner.is_geometric else 0.25

    return CornerScoringHints(
        start_turn_strength=start_turn_strength,
        end_turn_strength=end_turn_strength,
        orthogonal_turn_count=orthogonal_turn_count,
        same_role_continuation_strength=min(1.0, same_role_continuation_strength),
        has_geometric_corner=has_geometric_corner,
        has_junction_corner=has_junction_corner,
    )


def _cf_build_frontier_rank(
    chain_ref: ChainRef,
    chain: BoundaryChain,
    node: PatchNode,
    known: int,
    graph: PatchGraph,
    topology_facts: FrontierTopologyFacts,
    patch_context: PatchScoringContext,
    quilt_patch_ids: set[int],
    allowed_tree_edges: set[PatchEdgeKey],
    runtime_policy: "FrontierRuntimePolicy",
    score: float,
    seam_relation: Optional[SeamRelationProfile] = None,
    seam_bonus: float = 0.0,
    shape_bonus: float = 0.0,
    closure_pair_refs: Optional[frozenset[ChainRef]] = None,
    start_anchor: Optional[ChainAnchor] = None,
    end_anchor: Optional[ChainAnchor] = None,
) -> tuple[FrontierRank, FrontierRankBreakdown]:
    """Собирает structured rank из текущих scalar signal'ов без смены rescue-flow."""
    is_hv = topology_facts.is_hv
    same_patch_anchor_count = topology_facts.same_patch_anchor_count
    cross_patch_anchor_count = topology_facts.cross_patch_anchor_count

    viability_tier = 1 if score >= CHAIN_FRONTIER_THRESHOLD else 0
    viability_label = 'viable' if viability_tier else 'below_threshold'
    role_tier, role_label = _cf_role_tier(
        chain_ref,
        chain,
        node,
        graph,
        quilt_patch_ids,
        allowed_tree_edges,
    )

    if topology_facts.is_secondary_closure and same_patch_anchor_count == 0 and cross_patch_anchor_count > 0:
        if patch_context.is_untouched:
            role_tier = min(role_tier, 1)
            role_label = f"{role_label}:closure_secondary_untouched"
        elif patch_context.same_patch_backbone_strength < 0.5:
            role_tier = min(role_tier, 2)
            role_label = f"{role_label}:closure_secondary_early"

    if known >= 2:
        ingress_tier = 3
        ingress_label = 'dual_anchor'
    elif not patch_context.is_untouched:
        ingress_tier = 2
        ingress_label = 'single_anchor_patch_progress'
    elif known == 1:
        ingress_tier = 1
        ingress_label = 'single_anchor_ingress'
    else:
        ingress_tier = 0
        ingress_label = 'unanchored'

    if is_hv:
        if not topology_facts.would_be_connected:
            patch_fit_tier = 0
            patch_fit_label = 'isolated_hv'
        elif not patch_context.is_untouched:
            patch_fit_tier = 2
            patch_fit_label = f"same_patch_backbone:{patch_context.same_patch_backbone_strength:.2f}"
        else:
            patch_fit_tier = 1
            patch_fit_label = 'hv_ingress'
    else:
        patch_fit_tier = 1 if not patch_context.is_untouched else 0
        patch_fit_label = (
            f"patch_progress:{patch_context.placed_ratio:.2f}"
            if not patch_context.is_untouched else
            'untouched_free'
        )

    anchor_tier = (same_patch_anchor_count * 3) + min(topology_facts.hv_adjacency, 2)
    if same_patch_anchor_count == 0 and cross_patch_anchor_count > 0:
        anchor_tier = max(0, anchor_tier - 1)
    anchor_label = (
        f"sp:{same_patch_anchor_count}"
        f"/xp:{cross_patch_anchor_count}"
        f"/hv:{min(topology_facts.hv_adjacency, 2)}"
    )

    if topology_facts.is_secondary_closure:
        if same_patch_anchor_count == 0 and cross_patch_anchor_count > 0:
            closure_risk_tier = 0
            closure_label = f"closure_cross_patch_only:{patch_context.closure_pressure:.2f}"
        elif patch_context.is_untouched:
            closure_risk_tier = 1
            closure_label = f"closure_patch_ingress:{patch_context.closure_pressure:.2f}"
        else:
            closure_risk_tier = 2
            closure_label = f"closure_supported:{patch_context.closure_pressure:.2f}"
    else:
        closure_risk_tier = 3
        closure_label = f"regular:{patch_context.closure_pressure:.2f}"

    seam_membership = _cf_seam_relation_membership(seam_relation, chain_ref)
    if seam_relation is None:
        seam_label = 'seam:none'
    else:
        if seam_membership == 'primary' and same_patch_anchor_count == 0 and cross_patch_anchor_count > 0:
            anchor_tier += 1
        elif seam_membership == 'secondary':
            if patch_context.is_untouched:
                closure_risk_tier = min(closure_risk_tier, 0)
            elif patch_context.same_patch_backbone_strength < 0.5:
                closure_risk_tier = min(closure_risk_tier, 1)
        elif (
            seam_membership == 'support'
            and seam_relation.is_closure_like
            and same_patch_anchor_count == 0
            and cross_patch_anchor_count > 0
        ):
            anchor_tier = max(0, anchor_tier - 1)
        seam_label = (
            f"seam:{seam_membership}"
            f"/sec:{seam_relation.secondary_pair_count}"
            f"/gap:{seam_relation.pair_strength_gap:.2f}"
            f"/as:{seam_relation.support_asymmetry:.2f}"
            f"/in:{seam_relation.ingress_preference:.2f}"
            f"/b:{seam_bonus:+.2f}"
        )

    shape_profile = patch_context.shape_profile
    if shape_profile is None:
        shape_label = 'shape:none'
    else:
        shape_label = (
            f"shape:{shape_bonus:+.2f}"
            f"/r{shape_profile.rectilinearity:.2f}"
            f"/b{_cf_patch_shape_backbone_bias(shape_profile):.2f}"
            f"/c{_cf_patch_shape_closure_sensitivity(shape_profile):.2f}"
        )

    rank = FrontierRank(
        viability_tier=viability_tier,
        role_tier=role_tier,
        ingress_tier=ingress_tier,
        patch_fit_tier=patch_fit_tier,
        anchor_tier=anchor_tier,
        closure_risk_tier=closure_risk_tier,
        local_score=score,
        tie_length=_cf_chain_total_length(chain, runtime_policy.final_scale),
    )
    breakdown = FrontierRankBreakdown(
        viability_label=viability_label,
        role_label=role_label,
        ingress_label=ingress_label,
        patch_fit_label=patch_fit_label,
        anchor_label=anchor_label,
        closure_label=closure_label,
        seam_label=seam_label,
        shape_label=shape_label,
        summary=(
            f"{viability_label}>{role_label}>{ingress_label}>"
            f"{patch_fit_label}>{anchor_label}>{closure_label}>{seam_label}>{shape_label}"
        ),
    )
    return rank, breakdown


def _cf_make_frontier_placement_candidate(
    entry: ChainPoolEntry,
    candidate_eval: FrontierCandidateEval,
) -> FrontierPlacementCandidate:
    return FrontierPlacementCandidate(
        chain_ref=entry.chain_ref,
        chain=entry.chain,
        node=entry.node,
        start_anchor=candidate_eval.start_anchor,
        end_anchor=candidate_eval.end_anchor,
        anchor_reason=candidate_eval.anchor_reason,
        anchor_adjustments=candidate_eval.anchor_adjustments,
        closure_dir_override=candidate_eval.closure_dir_override,
        score=candidate_eval.score,
        length_factor=candidate_eval.length_factor,
        downstream_count=candidate_eval.downstream_count,
        downstream_bonus=candidate_eval.downstream_bonus,
        isolation_preview=candidate_eval.isolation_preview,
        isolation_penalty=candidate_eval.isolation_penalty,
        structural_free_bonus=candidate_eval.structural_free_bonus,
        hv_adjacency=candidate_eval.hv_adjacency,
        rank=candidate_eval.rank,
        rank_breakdown=candidate_eval.rank_breakdown,
        patch_context=candidate_eval.patch_context,
        corner_hints=candidate_eval.corner_hints,
        seam_relation=candidate_eval.seam_relation,
        seam_bonus=candidate_eval.seam_bonus,
        corner_bonus=candidate_eval.corner_bonus,
        shape_bonus=candidate_eval.shape_bonus,
    )


def _cf_frontier_rank_debug_label(rank: Optional[FrontierRank]) -> str:
    if rank is None:
        return '-'
    return (
        f"{rank.viability_tier}/"
        f"{rank.role_tier}/"
        f"{rank.ingress_tier}/"
        f"{rank.patch_fit_tier}/"
        f"{rank.anchor_tier}/"
        f"{rank.closure_risk_tier}"
    )


def _cf_build_frontier_topology_facts(
    chain_ref: ChainRef,
    chain: BoundaryChain,
    graph: PatchGraph,
    runtime_policy: "FrontierRuntimePolicy",
    known: int,
    start_anchor: Optional[ChainAnchor],
    end_anchor: Optional[ChainAnchor],
    closure_pair_refs: Optional[frozenset[ChainRef]] = None,
) -> FrontierTopologyFacts:
    is_bridge = chain.frame_role == FrameRole.FREE and len(chain.vert_cos) <= 2
    is_hv = chain.frame_role in (FrameRole.H_FRAME, FrameRole.V_FRAME)
    same_patch_anchor_count = sum(
        1 for anchor in (start_anchor, end_anchor)
        if anchor is not None and anchor.source_kind == PlacementSourceKind.SAME_PATCH
    )
    cross_patch_anchor_count = sum(
        1 for anchor in (start_anchor, end_anchor)
        if anchor is not None and anchor.source_kind == PlacementSourceKind.CROSS_PATCH
    )
    hv_adjacency = _cf_count_hv_adjacent_endpoints(graph, chain_ref)
    if is_hv:
        for anchor in (start_anchor, end_anchor):
            if anchor is None or anchor.source_kind != PlacementSourceKind.CROSS_PATCH:
                continue
            src_chain = graph.get_chain(*anchor.source_ref)
            if src_chain is not None and src_chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
                hv_adjacency += 1
                break

    would_be_connected = _cf_preview_would_be_connected(chain_ref, chain, runtime_policy, graph)
    return FrontierTopologyFacts(
        is_hv=is_hv,
        is_bridge=is_bridge,
        same_patch_anchor_count=same_patch_anchor_count,
        cross_patch_anchor_count=cross_patch_anchor_count,
        hv_adjacency=hv_adjacency,
        would_be_connected=would_be_connected if is_hv else True,
        is_secondary_closure=bool(known > 0 and closure_pair_refs and chain_ref in closure_pair_refs),
    )


def _cf_score_topology_baseline(
    chain_ref: ChainRef,
    chain: BoundaryChain,
    node: PatchNode,
    known: int,
    graph: PatchGraph,
    quilt_patch_ids: set[int],
    allowed_tree_edges: set[PatchEdgeKey],
    topology_facts: FrontierTopologyFacts,
) -> float:
    score = 0.0
    if topology_facts.is_hv:
        if chain.is_corner_split:
            score += 0.4
        elif (
            chain.neighbor_kind == ChainNeighborKind.PATCH
            and chain.neighbor_patch_id in quilt_patch_ids
            and _is_allowed_quilt_edge(allowed_tree_edges, chain_ref[0], chain.neighbor_patch_id)
        ):
            current_type = node.patch_type
            neighbor_node = graph.nodes.get(chain.neighbor_patch_id)
            neighbor_type = neighbor_node.patch_type if neighbor_node else None
            score += 2.0 if current_type == neighbor_type else 1.5
        else:
            score += 1.0
    elif topology_facts.is_bridge:
        score += 0.1

    if known == 2:
        score += 0.8
    elif known == 1:
        score += 0.3

    if topology_facts.is_bridge and known < 2:
        score -= 0.45
    return score


def _cf_score_patch_anchor_context(
    patch_context: PatchScoringContext,
    topology_facts: FrontierTopologyFacts,
    score_hv_adj_full_bonus: float,
    score_hv_adj_isolated_penalty: float,
    score_bridge_first_patch_penalty: float,
    score_bridge_cross_patch_penalty: float,
) -> float:
    score = 0.0
    if not patch_context.is_untouched:
        score += 0.2

    score += 0.1 * topology_facts.same_patch_anchor_count
    if topology_facts.same_patch_anchor_count == 0 and topology_facts.cross_patch_anchor_count > 0:
        score -= 0.35 if not patch_context.is_untouched else 0.25

    if topology_facts.is_hv:
        if topology_facts.hv_adjacency >= 2:
            score += score_hv_adj_full_bonus
        elif topology_facts.hv_adjacency <= 0:
            score -= score_hv_adj_isolated_penalty

    if topology_facts.is_bridge and patch_context.is_untouched:
        score -= score_bridge_first_patch_penalty
    if topology_facts.is_bridge and topology_facts.same_patch_anchor_count == 0 and topology_facts.cross_patch_anchor_count > 0:
        score -= score_bridge_cross_patch_penalty
    return score


def _cf_score_closure_guard(
    patch_context: PatchScoringContext,
    topology_facts: FrontierTopologyFacts,
) -> float:
    if not topology_facts.is_secondary_closure:
        return 0.0
    if topology_facts.same_patch_anchor_count == 0 and topology_facts.cross_patch_anchor_count > 0:
        return -(0.9 if not patch_context.is_untouched else 1.1)
    if patch_context.is_untouched:
        return -0.7
    return 0.0


def _cf_score_seam_relation_hint(
    chain_ref: ChainRef,
    patch_context: PatchScoringContext,
    topology_facts: FrontierTopologyFacts,
    seam_relation: Optional[SeamRelationProfile],
) -> float:
    seam_bonus = 0.0
    seam_membership = _cf_seam_relation_membership(seam_relation, chain_ref)
    if seam_relation is None:
        return seam_bonus
    if seam_membership == 'primary':
        if topology_facts.is_hv and topology_facts.same_patch_anchor_count == 0 and topology_facts.cross_patch_anchor_count > 0:
            seam_bonus += 0.05 * seam_relation.ingress_preference
        elif topology_facts.is_hv and patch_context.is_untouched:
            seam_bonus += 0.03 * seam_relation.ingress_preference
    elif seam_membership == 'secondary':
        early_gate = _cf_clamp01(1.0 - patch_context.same_patch_backbone_strength)
        seam_bonus -= 0.06 * early_gate * (1.0 - seam_relation.pair_strength_gap)
    elif seam_relation.is_closure_like and topology_facts.same_patch_anchor_count == 0 and topology_facts.cross_patch_anchor_count > 0:
        seam_bonus -= 0.03 * seam_relation.support_asymmetry
    return seam_bonus


def _cf_score_shape_hint(
    chain_ref: ChainRef,
    patch_context: PatchScoringContext,
    topology_facts: FrontierTopologyFacts,
    shape_profile: Optional[PatchShapeProfile],
    closure_pair_refs: Optional[frozenset[ChainRef]] = None,
) -> float:
    shape_bonus = 0.0
    shape_backbone_bias = _cf_patch_shape_backbone_bias(shape_profile)
    shape_closure_sensitivity = _cf_patch_shape_closure_sensitivity(shape_profile)
    if shape_profile is None:
        return shape_bonus

    if topology_facts.is_hv:
        if patch_context.is_untouched:
            shape_bonus += 0.08 * shape_backbone_bias
        elif patch_context.same_patch_backbone_strength < 0.5:
            shape_bonus += 0.04 * shape_backbone_bias
    elif patch_context.is_untouched:
        free_guard = 0.6 * shape_profile.rectilinearity + 0.4 * shape_profile.frame_dominance
        if topology_facts.is_bridge:
            shape_bonus -= 0.03 * _cf_clamp01(free_guard)
        else:
            shape_bonus -= 0.05 * _cf_clamp01(free_guard)

    if closure_pair_refs and chain_ref in closure_pair_refs:
        progress_gate = 1.0 - patch_context.placed_ratio
        shape_bonus -= 0.05 * shape_closure_sensitivity * _cf_clamp01(progress_gate)
    return shape_bonus


def _cf_score_corner_hint(
    topology_facts: FrontierTopologyFacts,
    corner_hints: Optional[CornerScoringHints],
) -> float:
    if corner_hints is None:
        return 0.0
    avg_turn_strength = (
        corner_hints.start_turn_strength + corner_hints.end_turn_strength
    ) * 0.5
    corner_bonus = 0.0
    if topology_facts.is_hv:
        corner_bonus += 0.04 * min(2, corner_hints.orthogonal_turn_count)
        corner_bonus += 0.04 * corner_hints.same_role_continuation_strength
    else:
        corner_bonus += 0.02 * corner_hints.same_role_continuation_strength
    corner_bonus += 0.02 * avg_turn_strength
    if corner_hints.has_junction_corner and not corner_hints.has_geometric_corner:
        corner_bonus += 0.01
    return corner_bonus


def _cf_build_frontier_local_score_details(
    chain_ref: ChainRef,
    chain: BoundaryChain,
    graph: PatchGraph,
    patch_context: PatchScoringContext,
    runtime_policy: "FrontierRuntimePolicy",
    topology_facts: FrontierTopologyFacts,
    corner_hints: Optional[CornerScoringHints] = None,
    seam_relation: Optional[SeamRelationProfile] = None,
    closure_pair_refs: Optional[frozenset[ChainRef]] = None,
    *,
    score_free_length_scale: float,
    score_free_length_cap: float,
    score_downstream_scale: float,
    score_downstream_cap: float,
    score_isolated_hv_penalty: float,
    score_free_strip_connector: float,
    score_free_frame_neighbor: float,
) -> FrontierLocalScoreDetails:
    length_factor = 0.0
    if not topology_facts.is_hv:
        chain_len = _cf_chain_total_length(chain, runtime_policy.final_scale)
        length_factor = min(score_free_length_cap, chain_len * score_free_length_scale)

    downstream_count = _cf_estimate_downstream_anchor_count(chain_ref, chain, graph, runtime_policy)
    downstream_bonus = min(score_downstream_cap, downstream_count * score_downstream_scale)

    isolation_penalty = 0.0
    if topology_facts.is_hv and not topology_facts.would_be_connected:
        isolation_penalty = score_isolated_hv_penalty

    structural_free_bonus = 0.0
    if not topology_facts.is_hv and not topology_facts.is_bridge:
        neighbors = graph.get_chain_endpoint_neighbors(chain_ref[0], chain_ref[1], chain_ref[2])
        start_has_hv = any(
            graph.get_chain(chain_ref[0], li, ci) is not None
            and graph.get_chain(chain_ref[0], li, ci).frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
            for li, ci in neighbors.get("start", [])
        )
        end_has_hv = any(
            graph.get_chain(chain_ref[0], li, ci) is not None
            and graph.get_chain(chain_ref[0], li, ci).frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
            for li, ci in neighbors.get("end", [])
        )
        if start_has_hv and end_has_hv:
            structural_free_bonus = score_free_strip_connector
        elif start_has_hv or end_has_hv:
            structural_free_bonus = score_free_frame_neighbor

    seam_bonus = _cf_score_seam_relation_hint(
        chain_ref,
        patch_context,
        topology_facts,
        seam_relation,
    )
    shape_bonus = _cf_score_shape_hint(
        chain_ref,
        patch_context,
        topology_facts,
        patch_context.shape_profile,
        closure_pair_refs=closure_pair_refs,
    )
    corner_bonus = _cf_score_corner_hint(topology_facts, corner_hints)

    return FrontierLocalScoreDetails(
        length_factor=length_factor,
        downstream_count=downstream_count,
        downstream_bonus=downstream_bonus,
        isolation_penalty=isolation_penalty,
        structural_free_bonus=structural_free_bonus,
        seam_bonus=seam_bonus,
        corner_bonus=corner_bonus,
        shape_bonus=shape_bonus,
    )


def _cf_score_candidate_layered(
    chain_ref,
    chain,
    node,
    known,
    graph,
    patch_context: PatchScoringContext,
    quilt_patch_ids,
    allowed_tree_edges,
    runtime_policy: "FrontierRuntimePolicy",
    corner_hints: Optional[CornerScoringHints] = None,
    seam_relation: Optional[SeamRelationProfile] = None,
    closure_pair_refs=None,
    start_anchor: Optional[ChainAnchor] = None,
    end_anchor: Optional[ChainAnchor] = None,
):
    try:
        from .constants import (SCORE_FREE_LENGTH_SCALE, SCORE_FREE_LENGTH_CAP, SCORE_DOWNSTREAM_SCALE, SCORE_DOWNSTREAM_CAP,
                                SCORE_ISOLATED_HV_PENALTY, SCORE_HV_ADJ_FULL_BONUS, SCORE_HV_ADJ_ISOLATED_PENALTY,
                                SCORE_BRIDGE_FIRST_PATCH_PENALTY, SCORE_BRIDGE_CROSS_PATCH_PENALTY,
                                SCORE_FREE_STRIP_CONNECTOR, SCORE_FREE_FRAME_NEIGHBOR)
    except ImportError:
        from constants import (SCORE_FREE_LENGTH_SCALE, SCORE_FREE_LENGTH_CAP, SCORE_DOWNSTREAM_SCALE, SCORE_DOWNSTREAM_CAP,
                               SCORE_ISOLATED_HV_PENALTY, SCORE_HV_ADJ_FULL_BONUS, SCORE_HV_ADJ_ISOLATED_PENALTY,
                               SCORE_BRIDGE_FIRST_PATCH_PENALTY, SCORE_BRIDGE_CROSS_PATCH_PENALTY,
                               SCORE_FREE_STRIP_CONNECTOR, SCORE_FREE_FRAME_NEIGHBOR)

    topology_facts = _cf_build_frontier_topology_facts(
        chain_ref,
        chain,
        graph,
        runtime_policy,
        known,
        start_anchor,
        end_anchor,
        closure_pair_refs=closure_pair_refs,
    )
    score = 0.0
    score += _cf_score_topology_baseline(
        chain_ref,
        chain,
        node,
        known,
        graph,
        quilt_patch_ids,
        allowed_tree_edges,
        topology_facts,
    )
    score += _cf_score_patch_anchor_context(
        patch_context,
        topology_facts,
        SCORE_HV_ADJ_FULL_BONUS,
        SCORE_HV_ADJ_ISOLATED_PENALTY,
        SCORE_BRIDGE_FIRST_PATCH_PENALTY,
        SCORE_BRIDGE_CROSS_PATCH_PENALTY,
    )
    score += _cf_score_closure_guard(
        patch_context,
        topology_facts,
    )

    score_details = _cf_build_frontier_local_score_details(
        chain_ref,
        chain,
        graph,
        patch_context,
        runtime_policy,
        topology_facts,
        corner_hints=corner_hints,
        seam_relation=seam_relation,
        closure_pair_refs=closure_pair_refs,
        score_free_length_scale=SCORE_FREE_LENGTH_SCALE,
        score_free_length_cap=SCORE_FREE_LENGTH_CAP,
        score_downstream_scale=SCORE_DOWNSTREAM_SCALE,
        score_downstream_cap=SCORE_DOWNSTREAM_CAP,
        score_isolated_hv_penalty=SCORE_ISOLATED_HV_PENALTY,
        score_free_strip_connector=SCORE_FREE_STRIP_CONNECTOR,
        score_free_frame_neighbor=SCORE_FREE_FRAME_NEIGHBOR,
    )
    score += score_details.length_factor
    score += score_details.downstream_bonus
    score -= score_details.isolation_penalty
    score += score_details.structural_free_bonus
    score += score_details.seam_bonus
    score += score_details.corner_bonus
    score += score_details.shape_bonus
    return score, topology_facts, score_details


def _cf_score_candidate_legacy(
    chain_ref,
    chain,
    node,
    known,
    graph,
    patch_context: PatchScoringContext,
    quilt_patch_ids,
    allowed_tree_edges,
    runtime_policy: "FrontierRuntimePolicy",
    corner_hints: Optional[CornerScoringHints] = None,
    seam_relation: Optional[SeamRelationProfile] = None,
    closure_pair_refs=None,
    start_anchor: Optional[ChainAnchor] = None,
    end_anchor: Optional[ChainAnchor] = None,
):
    """Chain-level score для frontier candidate."""
    try:
        from .constants import (SCORE_FREE_LENGTH_SCALE, SCORE_FREE_LENGTH_CAP, SCORE_DOWNSTREAM_SCALE, SCORE_DOWNSTREAM_CAP,
                                SCORE_ISOLATED_HV_PENALTY, SCORE_HV_ADJ_FULL_BONUS, SCORE_HV_ADJ_ISOLATED_PENALTY,
                                SCORE_BRIDGE_FIRST_PATCH_PENALTY, SCORE_BRIDGE_CROSS_PATCH_PENALTY,
                                SCORE_FREE_STRIP_CONNECTOR, SCORE_FREE_FRAME_NEIGHBOR)
    except ImportError:
        from constants import (SCORE_FREE_LENGTH_SCALE, SCORE_FREE_LENGTH_CAP, SCORE_DOWNSTREAM_SCALE, SCORE_DOWNSTREAM_CAP,
                               SCORE_ISOLATED_HV_PENALTY, SCORE_HV_ADJ_FULL_BONUS, SCORE_HV_ADJ_ISOLATED_PENALTY,
                               SCORE_BRIDGE_FIRST_PATCH_PENALTY, SCORE_BRIDGE_CROSS_PATCH_PENALTY,
                               SCORE_FREE_STRIP_CONNECTOR, SCORE_FREE_FRAME_NEIGHBOR)
    # Иерархия приоритетов:
    #   1. Cross-patch same-type H/V (seam скелет quilt):  base 2.0
    #   2. Cross-patch diff-type H/V:                       base 1.5
    #   3. Regular H/V (same patch boundary):               base 1.0
    #   4. Corner-split H/V (mesh border sub-chains):       base 0.4
    #   5. Bridge FREE (1 edge, 2 verts):                   base 0.1
    #      one-anchor bridge FREE дополнительно штрафуется, чтобы ждать second anchor
    #   6. Regular FREE:                                    base 0.0
    score = 0.0
    is_bridge = chain.frame_role == FrameRole.FREE and len(chain.vert_cos) <= 2
    is_hv = chain.frame_role in (FrameRole.H_FRAME, FrameRole.V_FRAME)

    # --- Base role score с учётом cross-patch и corner-split ---
    if is_hv:
        if chain.is_corner_split:
            # Corner-split sub-chains: низкий приоритет, после bridge
            score += 0.4
        elif (
            chain.neighbor_kind == ChainNeighborKind.PATCH
            and chain.neighbor_patch_id in quilt_patch_ids
            and _is_allowed_quilt_edge(allowed_tree_edges, chain_ref[0], chain.neighbor_patch_id)
        ):
            # Cross-patch H/V: скелет quilt — высший приоритет
            current_type = node.patch_type
            neighbor_node = graph.nodes.get(chain.neighbor_patch_id)
            neighbor_type = neighbor_node.patch_type if neighbor_node else None
            if current_type == neighbor_type:
                score += 2.0
            else:
                score += 1.5
        else:
            # Regular H/V (same patch, seam_self, mesh_border)
            score += 1.0
    elif is_bridge:
        score += 0.1
    else:
        score += 0.0

    # --- Anchor count bonus ---
    if known == 2:
        score += 0.8
    elif known == 1:
        score += 0.3

    if is_bridge and known < 2:
        score -= 0.45

    # --- Momentum: patch уже имеет placed chains ---
    if not patch_context.is_untouched:
        score += 0.2

    # --- Anchor source preference ---
    same_patch_anchor_count = sum(
        1 for anchor in (start_anchor, end_anchor)
        if anchor is not None and anchor.source_kind == PlacementSourceKind.SAME_PATCH
    )
    cross_patch_anchor_count = sum(
        1 for anchor in (start_anchor, end_anchor)
        if anchor is not None and anchor.source_kind == PlacementSourceKind.CROSS_PATCH
    )
    score += 0.1 * same_patch_anchor_count

    if same_patch_anchor_count == 0 and cross_patch_anchor_count > 0:
        score -= 0.35 if not patch_context.is_untouched else 0.25

    hv_adjacency = _cf_count_hv_adjacent_endpoints(graph, chain_ref)
    if is_hv:
        for anchor in (start_anchor, end_anchor):
            if anchor is not None and anchor.source_kind == PlacementSourceKind.CROSS_PATCH:
                src_chain = graph.get_chain(*anchor.source_ref)
                if src_chain is not None and src_chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
                    hv_adjacency += 1
                    break
    if is_hv:
        if hv_adjacency >= 2:
            score += SCORE_HV_ADJ_FULL_BONUS
        elif hv_adjacency <= 0:
            score -= SCORE_HV_ADJ_ISOLATED_PENALTY

    if is_bridge and patch_context.is_untouched:
        score -= SCORE_BRIDGE_FIRST_PATCH_PENALTY
    if is_bridge and same_patch_anchor_count == 0 and cross_patch_anchor_count > 0:
        score -= SCORE_BRIDGE_CROSS_PATCH_PENALTY

    # Secondary closure-side seam pairs should not outrun same-patch width/height
    # carriers during early patch ingress. Иначе tube/ring patch получает вторую
    # seam-chain слишком рано и H/V внутри patch схлопываются в чужой span.
    if closure_pair_refs and chain_ref in closure_pair_refs:
        if same_patch_anchor_count == 0 and cross_patch_anchor_count > 0:
            score -= 0.9 if not patch_context.is_untouched else 1.1
        elif patch_context.is_untouched:
            score -= 0.7

    # --- P5 Continuous scoring factors ---
    length_factor = 0.0
    if not is_hv:
        chain_len = _cf_chain_total_length(chain, runtime_policy.final_scale)
        length_factor = min(SCORE_FREE_LENGTH_CAP, chain_len * SCORE_FREE_LENGTH_SCALE)
        score += length_factor

    downstream = _cf_estimate_downstream_anchor_count(chain_ref, chain, graph, runtime_policy)
    downstream_bonus = min(SCORE_DOWNSTREAM_CAP, downstream * SCORE_DOWNSTREAM_SCALE)
    score += downstream_bonus

    isolation_penalty = 0.0
    would_be_connected = _cf_preview_would_be_connected(chain_ref, chain, runtime_policy, graph)
    if is_hv and not would_be_connected:
        isolation_penalty = SCORE_ISOLATED_HV_PENALTY
        score -= isolation_penalty

    structural_free_bonus = 0.0
    if not is_hv and not is_bridge:
        neighbors = graph.get_chain_endpoint_neighbors(chain_ref[0], chain_ref[1], chain_ref[2])
        start_has_hv = any(
            graph.get_chain(chain_ref[0], li, ci) is not None
            and graph.get_chain(chain_ref[0], li, ci).frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
            for li, ci in neighbors.get("start", [])
        )
        end_has_hv = any(
            graph.get_chain(chain_ref[0], li, ci) is not None
            and graph.get_chain(chain_ref[0], li, ci).frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
            for li, ci in neighbors.get("end", [])
        )
        if start_has_hv and end_has_hv:
            structural_free_bonus = SCORE_FREE_STRIP_CONNECTOR
            score += structural_free_bonus
        elif start_has_hv or end_has_hv:
            structural_free_bonus = SCORE_FREE_FRAME_NEIGHBOR
            score += structural_free_bonus

    seam_bonus = 0.0
    seam_membership = _cf_seam_relation_membership(seam_relation, chain_ref)
    if seam_relation is not None:
        if seam_membership == 'primary':
            if is_hv and same_patch_anchor_count == 0 and cross_patch_anchor_count > 0:
                seam_bonus += 0.05 * seam_relation.ingress_preference
            elif is_hv and patch_context.is_untouched:
                seam_bonus += 0.03 * seam_relation.ingress_preference
        elif seam_membership == 'secondary':
            early_gate = _cf_clamp01(1.0 - patch_context.same_patch_backbone_strength)
            seam_bonus -= 0.06 * early_gate * (1.0 - seam_relation.pair_strength_gap)
        elif seam_relation.is_closure_like and same_patch_anchor_count == 0 and cross_patch_anchor_count > 0:
            seam_bonus -= 0.03 * seam_relation.support_asymmetry
    score += seam_bonus

    shape_bonus = 0.0
    shape_profile = patch_context.shape_profile
    shape_backbone_bias = _cf_patch_shape_backbone_bias(shape_profile)
    shape_closure_sensitivity = _cf_patch_shape_closure_sensitivity(shape_profile)
    if shape_profile is not None:
        if is_hv:
            if patch_context.is_untouched:
                shape_bonus += 0.08 * shape_backbone_bias
            elif patch_context.same_patch_backbone_strength < 0.5:
                shape_bonus += 0.04 * shape_backbone_bias
        elif patch_context.is_untouched:
            free_guard = 0.6 * shape_profile.rectilinearity + 0.4 * shape_profile.frame_dominance
            if is_bridge:
                shape_bonus -= 0.03 * _cf_clamp01(free_guard)
            else:
                shape_bonus -= 0.05 * _cf_clamp01(free_guard)

        if closure_pair_refs and chain_ref in closure_pair_refs:
            progress_gate = 1.0 - patch_context.placed_ratio
            shape_bonus -= 0.05 * shape_closure_sensitivity * _cf_clamp01(progress_gate)

    score += shape_bonus

    corner_bonus = 0.0
    if corner_hints is not None:
        avg_turn_strength = (
            corner_hints.start_turn_strength + corner_hints.end_turn_strength
        ) * 0.5
        if is_hv:
            corner_bonus += 0.04 * min(2, corner_hints.orthogonal_turn_count)
            corner_bonus += 0.04 * corner_hints.same_role_continuation_strength
        else:
            corner_bonus += 0.02 * corner_hints.same_role_continuation_strength
        corner_bonus += 0.02 * avg_turn_strength
        if corner_hints.has_junction_corner and not corner_hints.has_geometric_corner:
            corner_bonus += 0.01
        score += corner_bonus

    details = (
        length_factor,
        downstream,
        downstream_bonus,
        would_be_connected,
        isolation_penalty,
        structural_free_bonus,
        hv_adjacency,
        seam_bonus,
        corner_bonus,
        shape_bonus,
    )
    return score, details


_cf_score_candidate = _cf_score_candidate_layered



def _is_orthogonal_hv_pair(role_a, role_b):
    return {role_a, role_b} == {FrameRole.H_FRAME, FrameRole.V_FRAME}


def _cf_compute_local_normal(node, corner_co_3d):
    """Находит локальную нормаль грани(треугольника), прилегающей к углу."""
    if not node.mesh_verts or not node.mesh_tris:
        return node.normal

    for i, v in enumerate(node.mesh_verts):
        if (v - corner_co_3d).length_squared < 1e-7:
            for tri in node.mesh_tris:
                if i in tri:
                    v0 = node.mesh_verts[tri[0]]
                    v1 = node.mesh_verts[tri[1]]
                    v2 = node.mesh_verts[tri[2]]
                    n = (v1 - v0).cross(v2 - v0)
                    if n.length_squared > 1e-8:
                        return n.normalized()
            break
            
    return node.normal


def _compute_corner_turn_sign(chain, src_chain, anchor, is_start_anchor, node):
    """Определяет знак поворота на corner из 3D tangent'ов.
    
    Возвращает +1 (CCW), -1 (CW), или 0 (неопределённо).
    """
    src_cos = src_chain.vert_cos
    if not src_cos or len(src_cos) < 2:
        return 0

    # Tangent of src_chain pointing AWAY from the corner
    if anchor.source_point_index == 0:
        src_tangent = src_cos[1] - src_cos[0]
        corner_co = src_cos[0]
    else:
        src_tangent = src_cos[-2] - src_cos[-1]
        corner_co = src_cos[-1]

    chain_cos = chain.vert_cos
    if not chain_cos or len(chain_cos) < 2:
        return 0

    # Tangent of the new chain pointing AWAY from the corner
    if is_start_anchor:
        chain_tangent = chain_cos[1] - chain_cos[0]
    else:
        chain_tangent = chain_cos[-2] - chain_cos[-1]

    local_normal = _cf_compute_local_normal(node, corner_co)

    # Both tangents point AWAY from the junction.
    # Входящий вектор в junction: -src_tangent. Исходящий: chain_tangent.
    cross_vec = (-src_tangent).cross(chain_tangent)
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
        if not chain.is_corner_split:
            continue
        if not _is_orthogonal_hv_pair(chain.frame_role, src_chain.frame_role):
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


def _cf_register_points(
    chain_ref: ChainRef,
    chain: BoundaryChain,
    uv_points: list[Vector],
    point_registry: PointRegistry,
    vert_to_placements: VertexPlacementMap,
) -> None:
    """Регистрирует все точки chain в обоих registry."""
    for i, uv in enumerate(uv_points):
        key = _point_registry_key(chain_ref, i)
        point_registry[key] = uv.copy()

        if i < len(chain.vert_indices):
            vert_idx = chain.vert_indices[i]
            vert_to_placements.setdefault(vert_idx, []).append((chain_ref, i))


def _compute_patch_chain_gap_reports(
    patch_placements: list[ScaffoldChainPlacement],
    total_chains: int,
) -> PatchChainGapDiagnostics:
    """Вычисляет gap между соседними placed chains в loop order."""
    if total_chains < 2 or not patch_placements:
        return PatchChainGapDiagnostics()

    placements_by_chain_index = {
        placement.chain_index: placement
        for placement in patch_placements
    }

    reports: list[ChainGapReport] = []
    max_gap = 0.0
    for chain_index in sorted(placements_by_chain_index):
        next_chain_index = (chain_index + 1) % total_chains
        current_placement = placements_by_chain_index[chain_index]
        next_placement = placements_by_chain_index.get(next_chain_index)
        if next_placement is None:
            continue
        if not current_placement.points or not next_placement.points:
            continue

        current_end = current_placement.points[-1][1]
        next_start = next_placement.points[0][1]
        gap = (current_end - next_start).length
        reports.append(ChainGapReport(
            chain_index=chain_index,
            next_chain_index=next_chain_index,
            gap=gap,
        ))
        if gap > max_gap:
            max_gap = gap

    return PatchChainGapDiagnostics(
        gap_reports=tuple(reports),
        max_chain_gap=max_gap,
    )


def _cf_build_envelopes(
    solve_view: SolveView,
    runtime_policy: FrontierRuntimePolicy,
    build_order=None,
):
    """Группирует размещённые chains в ScaffoldPatchPlacement per patch."""
    graph = runtime_policy.graph
    quilt_patch_ids = runtime_policy.quilt_patch_ids
    placed_chains_map = runtime_policy.placed_chains_map
    placed_chain_refs = runtime_policy.placed_chain_refs
    chain_dependency_patches = runtime_policy.chain_dependency_patches
    patches = {}

    for patch_id in quilt_patch_ids:
        node = graph.nodes.get(patch_id)
        if node is None:
            continue

        patch_placements = [
            placement
            for ref, placement in placed_chains_map.items()
            if ref[0] == patch_id
        ]
        patch_placements.sort(key=lambda placement: placement.chain_index)

        if not patch_placements:
            patches[patch_id] = ScaffoldPatchPlacement(
                patch_id=patch_id,
                loop_index=-1,
                root_chain_index=-1,
                notes=('no_placed_chains',),
                status=PatchPlacementStatus.EMPTY,
            )
            continue

        outer_loop_index = solve_view.primary_loop_index(patch_id)

        if outer_loop_index < 0:
            patches[patch_id] = ScaffoldPatchPlacement(
                patch_id=patch_id,
                loop_index=-1,
                root_chain_index=-1,
                notes=('no_outer_loop',),
                status=PatchPlacementStatus.UNSUPPORTED,
            )
            continue

        boundary_loop = node.boundary_loops[outer_loop_index]

        corner_positions = {}
        for corner_idx, corner in enumerate(boundary_loop.corners):
            prev_ref = (patch_id, outer_loop_index, corner.prev_chain_index)
            next_ref = (patch_id, outer_loop_index, corner.next_chain_index)
            if prev_ref in placed_chains_map:
                pts = placed_chains_map[prev_ref].points
                if pts:
                    corner_positions[corner_idx] = pts[-1][1].copy()
            elif next_ref in placed_chains_map:
                pts = placed_chains_map[next_ref].points
                if pts:
                    corner_positions[corner_idx] = pts[0][1].copy()

        all_pts = [pt for cp in patch_placements for _, pt in cp.points]
        if all_pts:
            bbox_min = Vector((min(p.x for p in all_pts), min(p.y for p in all_pts)))
            bbox_max = Vector((max(p.x for p in all_pts), max(p.y for p in all_pts)))
        else:
            bbox_min = Vector((0.0, 0.0))
            bbox_max = Vector((0.0, 0.0))

        total_chains = len(boundary_loop.chains)
        placed_count = sum(
            1 for ci in range(total_chains)
            if (patch_id, outer_loop_index, ci) in placed_chain_refs
        )

        if placed_count >= total_chains:
            status = PatchPlacementStatus.COMPLETE
        elif placed_count > 0:
            status = PatchPlacementStatus.PARTIAL
        else:
            status = PatchPlacementStatus.EMPTY

        unplaced = tuple(
            ci for ci in range(total_chains)
            if (patch_id, outer_loop_index, ci) not in placed_chain_refs
        )

        dep_set = set()
        for cp in patch_placements:
            cp_ref = (cp.patch_id, cp.loop_index, cp.chain_index)
            dep_set.update(chain_dependency_patches.get(cp_ref, ()))

        closure_error = 0.0
        closure_valid = True
        if status == PatchPlacementStatus.COMPLETE and total_chains >= 2:
            if patch_placements[-1].points and patch_placements[0].points:
                last_end = patch_placements[-1].points[-1][1]
                first_start = patch_placements[0].points[0][1]
                closure_error = (last_end - first_start).length
                closure_valid = closure_error < 0.05

        gap_diagnostics = _compute_patch_chain_gap_reports(
            patch_placements,
            total_chains,
        )

        # Scaffold connectivity: найти root chain для этого patch из build_order,
        # вычислить связные H/V chains. Изолированные H/V за FREE не пинятся.
        root_ci = -1
        if build_order:
            for bo_ref in build_order:
                if bo_ref[0] == patch_id:
                    root_ci = bo_ref[2]
                    break
        scaffold_connected = _compute_scaffold_connected_chains(
            patch_placements, total_chains, root_ci,
        )

        patches[patch_id] = ScaffoldPatchPlacement(
            patch_id=patch_id,
            loop_index=outer_loop_index,
            root_chain_index=root_ci,
            corner_positions=corner_positions,
            chain_placements=patch_placements,
            bbox_min=bbox_min,
            bbox_max=bbox_max,
            closure_error=closure_error,
            max_chain_gap=gap_diagnostics.max_chain_gap,
            gap_reports=gap_diagnostics.gap_reports,
            closure_valid=closure_valid,
            notes=(),
            status=status,
            dependency_patches=tuple(sorted(dep_set)),
            unplaced_chain_indices=unplaced,
            scaffold_connected_chains=scaffold_connected,
        )

    return patches


def _finalize_quilt_scaffold_frontier(
    graph: PatchGraph,
    solve_view: SolveView,
    quilt_plan: QuiltPlan,
    quilt_scaffold: ScaffoldQuiltPlacement,
    runtime_policy: FrontierRuntimePolicy,
    final_scale: float,
    allowed_tree_edges: set[PatchEdgeKey],
    seed_ref: ChainRef,
    total_available: int,
) -> FinalizedQuiltScaffold:
    quilt_scaffold.patches = _cf_build_envelopes(
        solve_view,
        runtime_policy,
        build_order=[seed_ref] + list(runtime_policy.build_order),
    )
    untouched_patch_ids = [
        patch_id
        for patch_id, patch_placement in quilt_scaffold.patches.items()
        if patch_placement.status == PatchPlacementStatus.EMPTY and 'no_placed_chains' in patch_placement.notes
    ]

    quilt_scaffold.build_order = list(runtime_policy.build_order)
    quilt_scaffold.closure_seam_reports = _collect_quilt_closure_seam_reports(
        graph,
        quilt_plan,
        quilt_scaffold,
        runtime_policy.placed_chains_map,
        final_scale,
        allowed_tree_edges,
    )
    quilt_scaffold.frame_alignment_reports = _collect_quilt_frame_alignment_reports(
        graph,
        quilt_plan,
        quilt_scaffold,
        final_scale,
        quilt_scaffold.closure_seam_reports,
    )
    _print_quilt_closure_seam_reports(quilt_plan.quilt_index, quilt_scaffold.closure_seam_reports)
    _print_quilt_frame_alignment_reports(quilt_plan.quilt_index, quilt_scaffold.frame_alignment_reports)

    trace_console(
        f"[CFTUV][Frontier] Quilt {quilt_plan.quilt_index}: "
        f"placed {runtime_policy.total_placed()}/{total_available} chains"
    )
    return FinalizedQuiltScaffold(
        quilt_scaffold=quilt_scaffold,
        untouched_patch_ids=untouched_patch_ids,
    )


def build_quilt_scaffold_chain_frontier(graph, quilt_plan, final_scale):
    """Chain-first strongest-frontier builder.

    Шагает от раннего замыкания patch по двум сужим anchors.
    Защита prevent_patch_wrap не позволяет замкнуть patch
    если оба anchor — cross_patch.
    """
    quilt_scaffold = ScaffoldQuiltPlacement(
        quilt_index=quilt_plan.quilt_index,
        root_patch_id=quilt_plan.root_patch_id,
    )
    quilt_patch_ids = set(quilt_plan.solved_patch_ids)
    quilt_patch_ids.add(quilt_plan.root_patch_id)
    solve_view = _build_solve_view(graph)
    allowed_tree_edges = _build_quilt_tree_edges(quilt_plan)
    closure_pair_map = _build_quilt_closure_pair_map(graph, quilt_plan, quilt_patch_ids, allowed_tree_edges)
    ordered_quilt_patch_ids = list(quilt_plan.solved_patch_ids)
    if quilt_plan.root_patch_id not in ordered_quilt_patch_ids:
        ordered_quilt_patch_ids.append(quilt_plan.root_patch_id)

    root_node = graph.nodes.get(quilt_plan.root_patch_id)
    if root_node is None:
        return quilt_scaffold

    bootstrap_attempt = _cf_bootstrap_frontier_runtime(
        graph,
        solve_view,
        quilt_plan,
        root_node,
        quilt_patch_ids,
        allowed_tree_edges,
        closure_pair_map,
        final_scale,
    )
    if bootstrap_attempt.result is None:
        quilt_scaffold.patches[quilt_plan.root_patch_id] = ScaffoldPatchPlacement(
            patch_id=quilt_plan.root_patch_id,
            loop_index=-1,
            root_chain_index=-1,
            notes=(bootstrap_attempt.error or 'frontier_bootstrap_failed',),
            status=PatchPlacementStatus.UNSUPPORTED,
        )
        return quilt_scaffold

    bootstrap_result = bootstrap_attempt.result
    runtime_policy = bootstrap_result.runtime_policy
    seed_ref = bootstrap_result.seed_ref
    seed_chain = bootstrap_result.seed_chain
    all_chain_pool = _cf_build_frontier_chain_pool(
        solve_view,
        graph,
        ordered_quilt_patch_ids,
        seed_ref,
    )

    # Статический индекс vert_index → [ChainRef] для incremental dirty marking
    vert_to_pool: dict[int, list[ChainRef]] = {}
    patch_to_pool: dict[int, list[ChainRef]] = {}
    for pool_entry in all_chain_pool:
        patch_to_pool.setdefault(pool_entry.chain_ref[0], []).append(pool_entry.chain_ref)
        for vi in pool_entry.chain.vert_indices:
            vert_to_pool.setdefault(vi, []).append(pool_entry.chain_ref)
    for vi in seed_chain.vert_indices:
        vert_to_pool.setdefault(vi, []).append(seed_ref)
    runtime_policy._vert_to_pool_refs = vert_to_pool
    runtime_policy._patch_to_pool_refs = patch_to_pool
    # Phase E: seed зарегистрирован до построения индекса — повторяем dirty marking
    _mark_neighbors_dirty(runtime_policy, seed_ref, seed_chain)

    # Инициализируем collector телеметрии для данного quilt
    _collector = FrontierTelemetryCollector(quilt_index=quilt_plan.quilt_index)
    seed_placement = runtime_policy.placed_chains_map.get(seed_ref)
    if seed_placement is not None:
        _collector.record_seed_placement(
            chain_ref=seed_ref,
            chain=seed_chain,
            score=bootstrap_result.seed_score,
            uv_points=[uv.copy() for _, uv in seed_placement.points],
            is_closure_pair=(seed_ref in runtime_policy.closure_pair_refs),
            hv_adjacency=_cf_count_hv_adjacent_endpoints(graph, seed_ref)
            if seed_chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
            else 0,
        )

    max_iter = len(all_chain_pool) + 10
    _t0 = time.perf_counter()
    for iteration in range(1, max_iter + 1):
        best_candidate = _cf_select_best_frontier_candidate(runtime_policy, all_chain_pool)

        if best_candidate is None or best_candidate.score < CHAIN_FRONTIER_THRESHOLD:
            # Записываем stall-событие до попытки rescue
            _stall_snap = collect_stall_snapshot(
                cached_evals=runtime_policy._cached_evals,
                placed_chain_refs=runtime_policy.placed_chain_refs,
                rejected_chain_refs=runtime_policy.rejected_chain_refs,
                placed_count_by_patch=runtime_policy.placed_count_by_patch,
                quilt_patch_ids=runtime_policy.quilt_patch_ids,
                all_chain_pool=all_chain_pool,
            )
            _collector.record_stall(
                iteration=iteration,
                best_rejected_score=_stall_snap[0],
                best_rejected_ref=_stall_snap[1],
                best_rejected_role=_stall_snap[2],
                best_rejected_anchor_count=_stall_snap[3],
                available_count=_stall_snap[4],
                no_anchor_count=_stall_snap[5],
                below_threshold_count=_stall_snap[6],
                patches_with_placed=_stall_snap[7],
                patches_untouched=_stall_snap[8],
            )

            if _cf_try_place_tree_ingress_candidate(
                runtime_policy,
                all_chain_pool,
                iteration,
                collector=_collector,
            ):
                _collector.update_last_stall_rescue("tree_ingress", True)
                continue
            if _cf_try_place_closure_follow_candidate(
                runtime_policy,
                all_chain_pool,
                closure_pair_map,
                iteration,
                collector=_collector,
            ):
                _collector.update_last_stall_rescue("closure_follow", True)
                continue

            _collector.update_last_stall_rescue("none", False)
            # Диагностика: почему frontier остановился
            stop_diag = runtime_policy.build_stop_diagnostics(all_chain_pool)
            trace_console(
                f"[CFTUV][Frontier] STOP: remaining={stop_diag.remaining_count} "
                f"no_anchor={stop_diag.no_anchor_count} low_score={stop_diag.low_score_count} "
                f"rejected={stop_diag.rejected_count}"
            )
            trace_console(
                f"[CFTUV][Frontier] Patches: placed_in={len(stop_diag.placed_patch_ids)} "
                f"untouched={len(stop_diag.untouched_patch_ids)} "
                f"no_anchor_patches={len(stop_diag.no_anchor_patch_ids)}"
            )
            if stop_diag.untouched_patch_ids and len(stop_diag.untouched_patch_ids) <= 20:
                trace_console(f"[CFTUV][Frontier] Untouched patches: {list(stop_diag.untouched_patch_ids)}")
            break

        if not _cf_try_place_frontier_candidate(
            runtime_policy, best_candidate, iteration, collector=_collector
        ):
            continue

    _t1 = time.perf_counter()
    trace_console(
        f"[CFTUV][Frontier] Quilt {quilt_plan.quilt_index}: "
        f"frontier {_t1 - _t0:.4f}s "
        f"iters={iteration} placed={runtime_policy.total_placed()}/{len(all_chain_pool) + 1} "
        f"cache_hits={runtime_policy._cache_hits}"
    )

    total_available = len(all_chain_pool) + 1
    finalized_scaffold = _finalize_quilt_scaffold_frontier(
        graph,
        solve_view,
        quilt_plan,
        quilt_scaffold,
        runtime_policy,
        final_scale,
        allowed_tree_edges,
        seed_ref,
        total_available,
    )
    quilt_scaffold = finalized_scaffold.quilt_scaffold
    quilt_scaffold.frontier_telemetry = _collector.finalize()
    untouched_patch_ids = finalized_scaffold.untouched_patch_ids
    if untouched_patch_ids:
        fallback_quilt_plan = _restore_original_quilt_plan(quilt_plan)
        if fallback_quilt_plan is not None:
            print(
                f"[CFTUV][Plan] Quilt {quilt_plan.quilt_index}: "
                f"revert closure cut swap, untouched patches={sorted(untouched_patch_ids)}"
            )
            return build_quilt_scaffold_chain_frontier(graph, fallback_quilt_plan, final_scale)

    return quilt_scaffold

def build_root_scaffold_map(
    graph: PatchGraph,
    solve_plan: Optional[SolvePlan] = None,
    final_scale: float = 1.0,
) -> ScaffoldMap:
    """Build ScaffoldMap using chain-first strongest-frontier algorithm."""
    scaffold_map = ScaffoldMap()
    if solve_plan is None:
        return scaffold_map

    for quilt in solve_plan.quilts:
        quilt_scaffold = build_quilt_scaffold_chain_frontier(graph, quilt, final_scale)
        scaffold_map.quilts.append(quilt_scaffold)

    return scaffold_map

