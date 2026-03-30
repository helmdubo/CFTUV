from __future__ import annotations

from dataclasses import dataclass, field, replace
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
        ClosureAnchorMode, FrameAxisKind, ChainRef, PatchEdgeKey, LoopChainRef, AnchorAdjustment,
    )
    from .console_debug import trace_console
    from .frontier_finalize import _finalize_quilt_scaffold_frontier
    from .frontier_place import (
        _build_temporary_chain_placement,
        _cf_anchor_count,
        _cf_anchor_debug_label,
        _cf_build_seed_placement,
        _cf_chain_total_length,
        _cf_determine_direction,
        _cf_place_chain,
        _cf_rebuild_chain_points_for_endpoints,
        _default_role_direction,
        _is_orthogonal_hv_pair,
        _normalize_direction,
        _snap_direction_to_role,
        _try_inherit_direction,
    )
    from .frontier_rescue import (
        _cf_try_place_closure_follow_candidate,
        _cf_try_place_tree_ingress_candidate,
    )
    from .frontier_state import FrontierRuntimePolicy, _mark_neighbors_dirty
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
        _chain_uv_axis_metrics,
        _measure_shared_closure_uv_offsets,
    )
    from .solve_instrumentation import FrontierTelemetryCollector, collect_stall_snapshot
except ImportError:
    from model import (
        BoundaryChain, BoundaryCorner, BoundaryLoop, ChainNeighborKind, FrameRole, LoopKind,
        PatchGraph, PatchNode, PatchType,
        ScaffoldPointKey, ScaffoldChainPlacement, ScaffoldPatchPlacement,
        ScaffoldQuiltPlacement, ScaffoldMap, ScaffoldClosureSeamReport,
        ScaffoldFrameAlignmentReport, ChainGapReport, PatchPlacementStatus, PlacementSourceKind,
        ClosureAnchorMode, FrameAxisKind, ChainRef, PatchEdgeKey, LoopChainRef, AnchorAdjustment,
    )
    from console_debug import trace_console
    from frontier_finalize import _finalize_quilt_scaffold_frontier
    from frontier_place import (
        _build_temporary_chain_placement,
        _cf_anchor_count,
        _cf_anchor_debug_label,
        _cf_build_seed_placement,
        _cf_chain_total_length,
        _cf_determine_direction,
        _cf_place_chain,
        _cf_rebuild_chain_points_for_endpoints,
        _default_role_direction,
        _is_orthogonal_hv_pair,
        _normalize_direction,
        _snap_direction_to_role,
        _try_inherit_direction,
    )
    from frontier_rescue import (
        _cf_try_place_closure_follow_candidate,
        _cf_try_place_tree_ingress_candidate,
    )
    from frontier_state import FrontierRuntimePolicy, _mark_neighbors_dirty
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
        _chain_uv_axis_metrics,
        _measure_shared_closure_uv_offsets,
    )
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


def _build_tree_ingress_partner_map(quilt_plan: QuiltPlan) -> dict[ChainRef, ChainRef]:
    partner_map: dict[ChainRef, ChainRef] = {}
    for step in quilt_plan.steps:
        candidate = step.incoming_candidate
        if candidate is None:
            continue
        owner_ref = (
            candidate.owner_patch_id,
            candidate.owner_loop_index,
            candidate.owner_chain_index,
        )
        target_ref = (
            candidate.target_patch_id,
            candidate.target_loop_index,
            candidate.target_chain_index,
        )
        partner_map[owner_ref] = target_ref
        partner_map[target_ref] = owner_ref
    return partner_map


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


def _cf_bootstrap_runtime_score_caches(runtime_policy: FrontierRuntimePolicy) -> None:
    """Временный bootstrap score-derived caches до выделения frontier_score.py."""
    runtime_policy._outer_chain_count_by_patch.clear()
    runtime_policy._frame_chain_count_by_patch.clear()
    runtime_policy._closure_pair_count_by_patch.clear()
    runtime_policy._shape_profile_by_patch.clear()

    for patch_id, node in runtime_policy.graph.nodes.items():
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
        runtime_policy._outer_chain_count_by_patch[patch_id] = outer_chain_count
        runtime_policy._frame_chain_count_by_patch[patch_id] = frame_chain_count

    for chain_ref in runtime_policy.closure_pair_refs:
        patch_id = chain_ref[0]
        runtime_policy._closure_pair_count_by_patch[patch_id] = runtime_policy._closure_pair_count_by_patch.get(patch_id, 0) + 1

    for patch_id, node in runtime_policy.graph.nodes.items():
        runtime_policy._shape_profile_by_patch[patch_id] = _cf_build_patch_shape_profile(patch_id, node, runtime_policy)


def _cf_evaluate_candidate_runtime_policy(
    runtime_policy: FrontierRuntimePolicy,
    chain_ref: ChainRef,
    chain: BoundaryChain,
    node: PatchNode,
    apply_closure_preconstraint: bool = False,
    compute_score: bool = False,
) -> FrontierCandidateEval:
    found_anchors = _cf_find_anchors(
        chain_ref,
        chain,
        runtime_policy.graph,
        runtime_policy.point_registry,
        runtime_policy.vert_to_placements,
        runtime_policy.placed_chain_refs,
        runtime_policy.allowed_tree_edges,
    )
    raw_start_anchor = found_anchors.start_anchor
    raw_end_anchor = found_anchors.end_anchor

    placed_in_patch = runtime_policy.placed_in_patch(chain_ref[0])
    patch_context = _cf_build_patch_scoring_context(chain_ref, runtime_policy)
    seam_relation = _cf_chain_seam_relation(chain_ref, chain, runtime_policy)
    resolved_anchors = _cf_resolve_candidate_anchors(
        chain,
        raw_start_anchor,
        raw_end_anchor,
        placed_in_patch,
        runtime_policy.final_scale,
        runtime_policy.graph,
        runtime_policy.placed_chains_map,
    )
    start_anchor = resolved_anchors.start_anchor
    end_anchor = resolved_anchors.end_anchor
    known = resolved_anchors.known
    anchor_reason = resolved_anchors.reason
    anchor_adjustments = resolved_anchors.anchor_adjustments

    closure_dir_override = None
    if apply_closure_preconstraint and runtime_policy.closure_pair_map is not None:
        closure_application = _cf_apply_closure_preconstraint(
            chain_ref,
            chain,
            node,
            raw_start_anchor,
            raw_end_anchor,
            start_anchor,
            end_anchor,
            known,
            runtime_policy.graph,
            runtime_policy.point_registry,
            runtime_policy.placed_chains_map,
            runtime_policy.closure_pair_map,
            runtime_policy.final_scale,
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
        corner_hints = _cf_build_corner_scoring_hints(chain_ref, chain, runtime_policy.graph)
        score, topology_facts, score_details = _cf_score_candidate(
            chain_ref,
            chain,
            node,
            known,
            runtime_policy.graph,
            patch_context,
            runtime_policy.quilt_patch_ids,
            runtime_policy.allowed_tree_edges,
            runtime_policy,
            corner_hints=corner_hints,
            seam_relation=seam_relation,
            closure_pair_refs=runtime_policy.closure_pair_refs or None,
            start_anchor=start_anchor,
            end_anchor=end_anchor,
        )
        rank, rank_breakdown = _cf_build_frontier_rank(
            chain_ref,
            chain,
            node,
            known,
            runtime_policy.graph,
            topology_facts,
            patch_context,
            runtime_policy.quilt_patch_ids,
            runtime_policy.allowed_tree_edges,
            runtime_policy,
            score,
            seam_relation=seam_relation,
            seam_bonus=score_details.seam_bonus,
            shape_bonus=score_details.shape_bonus,
            closure_pair_refs=runtime_policy.closure_pair_refs or None,
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


def _cf_build_stop_diagnostics_runtime_policy(
    runtime_policy: FrontierRuntimePolicy,
    all_chain_pool: list[ChainPoolEntry],
) -> FrontierStopDiagnostics:
    remaining_count = 0
    no_anchor_count = 0
    low_score_count = 0
    patches_with_no_anchor = set()
    untouched_patch_ids = set()

    for entry in all_chain_pool:
        chain_ref = entry.chain_ref
        if not runtime_policy.is_chain_available(chain_ref):
            continue
        remaining_count += 1
        untouched_patch_ids.add(chain_ref[0])
        candidate_eval = _cf_evaluate_candidate_runtime_policy(runtime_policy, chain_ref, entry.chain, entry.node)
        if candidate_eval.known == 0:
            no_anchor_count += 1
            patches_with_no_anchor.add(chain_ref[0])
        else:
            low_score_count += 1

    placed_patch_ids = set(runtime_policy.placed_patch_ids())
    return FrontierStopDiagnostics(
        remaining_count=remaining_count,
        no_anchor_count=no_anchor_count,
        low_score_count=low_score_count,
        rejected_count=len(runtime_policy.rejected_chain_refs),
        placed_patch_ids=tuple(sorted(placed_patch_ids)),
        untouched_patch_ids=tuple(sorted(untouched_patch_ids - placed_patch_ids)),
        no_anchor_patch_ids=tuple(sorted(patches_with_no_anchor)),
    )


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
        candidate_eval = _cf_evaluate_candidate_runtime_policy(
            runtime_policy,
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


def _cf_bootstrap_frontier_runtime(
    graph: PatchGraph,
    solve_view: SolveView,
    quilt_plan: QuiltPlan,
    root_node: PatchNode,
    quilt_patch_ids: set[int],
    allowed_tree_edges: set[PatchEdgeKey],
    closure_pair_map: dict[ChainRef, ChainRef],
    tree_ingress_partner_by_chain: dict[ChainRef, ChainRef],
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
        tree_ingress_partner_by_chain=tree_ingress_partner_by_chain,
        closure_pair_map=closure_pair_map,
    )
    _cf_bootstrap_runtime_score_caches(runtime_policy)

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
        f" score:{seed_result.score:.2f}"
        f" len:{_cf_chain_total_length(seed_chain, final_scale):.4f}"
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


def _cf_preview_frame_dual_anchor_rectification(
    chain: BoundaryChain,
    start_anchor: Optional[ChainAnchor],
    end_anchor: Optional[ChainAnchor],
    graph: PatchGraph,
    placed_chains_map: dict[ChainRef, ScaffoldChainPlacement],
) -> DualAnchorRectificationPreview:
    # Локальная per-chain rectification для H/V намеренно отключена.
    # Историческое тело helper-а было недостижимо ниже первого return и удалено в P1.
    # Runtime-поведение не меняется: helper остаётся pass-through, чтобы не ломать
    # stitch continuity и row consistency до отдельного patch-level решения.
    _ = chain, graph, placed_chains_map
    return DualAnchorRectificationPreview(start_anchor=start_anchor, end_anchor=end_anchor)


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
    best_rank: Optional[tuple[int, int, float, float]] = None

    for loop_idx, loop in solve_view.iter_visible_loops(root_node.patch_id):
        for chain_idx, chain in enumerate(loop.chains):
            chain_ref = (root_node.patch_id, loop_idx, chain_idx)
            is_hv = chain.frame_role in (FrameRole.H_FRAME, FrameRole.V_FRAME)
            hv_adjacency = _cf_count_hv_adjacent_endpoints(graph, chain_ref)
            if is_hv and hv_adjacency <= 0:
                continue

            score = 0.0
            chain_len = 0.0

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
                chain_len,
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
            axis_safety.reason in ('span_mismatch', 'axis_mismatch')
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
        and start_anchor.source_kind == PlacementSourceKind.SAME_PATCH
        and end_anchor.source_kind == PlacementSourceKind.SAME_PATCH
        and chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
        and chain.neighbor_kind == ChainNeighborKind.MESH_BORDER
    ):
        reason_note = f'{rect_reason}|{reason}' if rect_reason else reason
        return ResolvedCandidateAnchors(
            start_anchor=None,
            end_anchor=None,
            known=0,
            reason=f'{reason_note}:reject_same_patch_axis_mismatch',
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


# Legacy scalar scorer removed in P1.
# Active frontier scoring entrypoint remains `_cf_score_candidate_layered`.
_cf_score_candidate = _cf_score_candidate_layered


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
    tree_ingress_partner_by_chain = _build_tree_ingress_partner_map(quilt_plan)
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
        tree_ingress_partner_by_chain,
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
                evaluate_candidate=_cf_evaluate_candidate_runtime_policy,
                is_allowed_quilt_edge=_is_allowed_quilt_edge,
                frame_anchor_pair_is_axis_safe=_cf_frame_anchor_pair_is_axis_safe,
                apply_anchor_adjustments=_cf_apply_anchor_adjustments,
                collector=_collector,
            ):
                _collector.update_last_stall_rescue("tree_ingress", True)
                continue
            if _cf_try_place_closure_follow_candidate(
                runtime_policy,
                all_chain_pool,
                closure_pair_map,
                iteration,
                evaluate_candidate=_cf_evaluate_candidate_runtime_policy,
                collector=_collector,
            ):
                _collector.update_last_stall_rescue("closure_follow", True)
                continue

            _collector.update_last_stall_rescue("none", False)
            # Диагностика: почему frontier остановился
            stop_diag = _cf_build_stop_diagnostics_runtime_policy(runtime_policy, all_chain_pool)
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

