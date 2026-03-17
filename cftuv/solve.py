from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
from heapq import heappop, heappush
from typing import Optional

from mathutils import Vector

try:
    from .constants import (
        FRAME_ALIGNMENT_THRESHOLD,
        ROOT_WEIGHT_AREA, ROOT_WEIGHT_FRAME, ROOT_WEIGHT_FREE_RATIO,
        ROOT_WEIGHT_HOLES, ROOT_WEIGHT_BASE,
        ATTACH_WEIGHT_SEAM, ATTACH_WEIGHT_PAIR, ATTACH_WEIGHT_TARGET, ATTACH_WEIGHT_OWNER,
        PAIR_WEIGHT_FRAME_CONT, PAIR_WEIGHT_ENDPOINT, PAIR_WEIGHT_CORNER,
        PAIR_WEIGHT_SEMANTIC, PAIR_WEIGHT_EP_STRENGTH, PAIR_WEIGHT_LOOP,
        FRONTIER_PROPAGATE_THRESHOLD, FRONTIER_WEAK_THRESHOLD, FRONTIER_MINIMUM_SCORE,
    )
    from .model import (
        BoundaryChain, ChainNeighborKind, FrameRole, LoopKind,
        PatchGraph, PatchNode,
        ScaffoldPointKey, ScaffoldChainPlacement, ScaffoldPatchPlacement,
        ScaffoldQuiltPlacement, ScaffoldMap, ScaffoldClosureSeamReport,
        ScaffoldFrameAlignmentReport,
    )
except ImportError:
    from constants import (
        FRAME_ALIGNMENT_THRESHOLD,
        ROOT_WEIGHT_AREA, ROOT_WEIGHT_FRAME, ROOT_WEIGHT_FREE_RATIO,
        ROOT_WEIGHT_HOLES, ROOT_WEIGHT_BASE,
        ATTACH_WEIGHT_SEAM, ATTACH_WEIGHT_PAIR, ATTACH_WEIGHT_TARGET, ATTACH_WEIGHT_OWNER,
        PAIR_WEIGHT_FRAME_CONT, PAIR_WEIGHT_ENDPOINT, PAIR_WEIGHT_CORNER,
        PAIR_WEIGHT_SEMANTIC, PAIR_WEIGHT_EP_STRENGTH, PAIR_WEIGHT_LOOP,
        FRONTIER_PROPAGATE_THRESHOLD, FRONTIER_WEAK_THRESHOLD, FRONTIER_MINIMUM_SCORE,
    )
    from model import (
        BoundaryChain, ChainNeighborKind, FrameRole, LoopKind,
        PatchGraph, PatchNode,
        ScaffoldPointKey, ScaffoldChainPlacement, ScaffoldPatchPlacement,
        ScaffoldQuiltPlacement, ScaffoldMap, ScaffoldClosureSeamReport,
        ScaffoldFrameAlignmentReport,
    )


EDGE_PROPAGATE_MIN = FRONTIER_PROPAGATE_THRESHOLD
EDGE_WEAK_MIN = FRONTIER_WEAK_THRESHOLD
SCAFFOLD_CLOSURE_EPSILON = 1e-4
FRAME_ROW_GROUP_TOLERANCE = 1e-3
FRAME_COLUMN_GROUP_TOLERANCE = 1e-3


@dataclass(frozen=True)
class PatchCertainty:
    patch_id: int
    local_solvable: bool
    root_score: float
    outer_count: int
    hole_count: int
    chain_count: int
    free_count: int
    h_count: int
    v_count: int
    reasons: tuple[str, ...] = ()


@dataclass(frozen=True)
class AttachmentCandidate:
    owner_patch_id: int
    target_patch_id: int
    score: float
    seam_length: float
    seam_norm: float
    best_pair_strength: float
    frame_continuation: float
    endpoint_bridge: float
    corner_strength: float
    semantic_strength: float
    endpoint_strength: float
    owner_certainty: float
    target_certainty: float
    ambiguity_penalty: float
    owner_loop_index: int
    owner_chain_index: int
    target_loop_index: int
    target_chain_index: int
    owner_loop_kind: LoopKind
    target_loop_kind: LoopKind
    owner_role: FrameRole
    target_role: FrameRole
    owner_transition: str
    target_transition: str
    reasons: tuple[str, ...] = ()


@dataclass
class SolverGraph:
    patch_scores: dict[int, PatchCertainty] = field(default_factory=dict)
    candidates: list[AttachmentCandidate] = field(default_factory=list)
    candidates_by_owner: dict[int, list[AttachmentCandidate]] = field(default_factory=dict)
    solve_components: list[set[int]] = field(default_factory=list)
    component_by_patch: dict[int, int] = field(default_factory=dict)
    max_shared_length: float = 0.0


@dataclass(frozen=True)
class QuiltStep:
    step_index: int
    patch_id: int
    is_root: bool = False
    incoming_candidate: Optional[AttachmentCandidate] = None


@dataclass
class QuiltPlan:
    quilt_index: int
    component_index: int
    root_patch_id: int
    root_score: float
    solved_patch_ids: list[int] = field(default_factory=list)
    steps: list[QuiltStep] = field(default_factory=list)
    deferred_candidates: list[AttachmentCandidate] = field(default_factory=list)
    rejected_candidates: list[AttachmentCandidate] = field(default_factory=list)
    stop_reason: str = ""


@dataclass
class SolvePlan:
    quilts: list[QuiltPlan] = field(default_factory=list)
    skipped_patch_ids: list[int] = field(default_factory=list)
    propagate_threshold: float = EDGE_PROPAGATE_MIN
    weak_threshold: float = EDGE_WEAK_MIN


@dataclass(frozen=True)
class ClosureCutHeuristic:
    edge_key: tuple[int, int]
    candidate: AttachmentCandidate
    score: float
    support_label: str
    fixed_endpoint_count: int
    same_axis_endpoint_count: int
    free_touched_endpoint_count: int
    reasons: tuple[str, ...] = ()


@dataclass(frozen=True)
class QuiltClosureCutAnalysis:
    current_cut: ClosureCutHeuristic
    recommended_cut: ClosureCutHeuristic
    path_patch_ids: tuple[int, ...] = ()
    cycle_edges: tuple[ClosureCutHeuristic, ...] = ()



@dataclass(frozen=True)
class ScaffoldUvTarget:
    face_index: int
    vert_index: int
    loop_point_index: int


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def _iter_neighbor_chains(graph: PatchGraph, owner_patch_id: int, target_patch_id: int):
    node = graph.nodes.get(owner_patch_id)
    if node is None:
        return []

    refs = []
    for loop_index, boundary_loop in enumerate(node.boundary_loops):
        # Solve connectivity строится только по OUTER loop.
        if boundary_loop.kind != LoopKind.OUTER:
            continue
        for chain_index, chain in enumerate(boundary_loop.chains):
            if chain.neighbor_kind != ChainNeighborKind.PATCH:
                continue
            if chain.neighbor_patch_id != target_patch_id:
                continue
            refs.append((loop_index, chain_index, boundary_loop, chain))
    return refs


def _build_solve_components(graph: PatchGraph, candidates: list[AttachmentCandidate]) -> tuple[list[set[int]], dict[int, int]]:
    adjacency = {patch_id: set() for patch_id in graph.nodes}

    for candidate in candidates:
        adjacency.setdefault(candidate.owner_patch_id, set()).add(candidate.target_patch_id)
        adjacency.setdefault(candidate.target_patch_id, set()).add(candidate.owner_patch_id)

    components: list[set[int]] = []
    component_by_patch: dict[int, int] = {}

    for patch_id in graph.nodes:
        if patch_id in component_by_patch:
            continue

        component = set()
        stack = [patch_id]
        while stack:
            current_id = stack.pop()
            if current_id in component:
                continue
            component.add(current_id)
            for neighbor_id in adjacency.get(current_id, ()):
                if neighbor_id not in component:
                    stack.append(neighbor_id)

        component_index = len(components)
        components.append(component)
        for component_patch_id in component:
            component_by_patch[component_patch_id] = component_index

    return components, component_by_patch


def _patch_pair_key(patch_a_id: int, patch_b_id: int) -> tuple[int, int]:
    return (min(patch_a_id, patch_b_id), max(patch_a_id, patch_b_id))


def _build_quilt_tree_edges(quilt_plan: QuiltPlan) -> set[tuple[int, int]]:
    edges = set()
    for step in quilt_plan.steps:
        candidate = step.incoming_candidate
        if candidate is None:
            continue
        edges.add(_patch_pair_key(candidate.owner_patch_id, candidate.target_patch_id))
    return edges


def _is_allowed_quilt_edge(
    allowed_tree_edges: set[tuple[int, int]],
    patch_a_id: int,
    patch_b_id: int,
) -> bool:
    return _patch_pair_key(patch_a_id, patch_b_id) in allowed_tree_edges


def _build_patch_tree_adjacency(quilt_plan: QuiltPlan) -> dict[int, set[int]]:
    adjacency = {quilt_plan.root_patch_id: set()}
    for patch_id in quilt_plan.solved_patch_ids:
        adjacency.setdefault(patch_id, set())
    for edge_a, edge_b in _build_quilt_tree_edges(quilt_plan):
        adjacency.setdefault(edge_a, set()).add(edge_b)
        adjacency.setdefault(edge_b, set()).add(edge_a)
    return adjacency


def _find_patch_tree_path(
    patch_tree_adjacency: dict[int, set[int]],
    start_patch_id: int,
    target_patch_id: int,
) -> list[int]:
    if start_patch_id == target_patch_id:
        return [start_patch_id]
    if start_patch_id not in patch_tree_adjacency or target_patch_id not in patch_tree_adjacency:
        return []

    parents = {start_patch_id: -1}
    queue = [start_patch_id]
    queue_index = 0
    while queue_index < len(queue):
        current_patch_id = queue[queue_index]
        queue_index += 1
        for neighbor_patch_id in patch_tree_adjacency.get(current_patch_id, ()):
            if neighbor_patch_id in parents:
                continue
            parents[neighbor_patch_id] = current_patch_id
            if neighbor_patch_id == target_patch_id:
                path = [target_patch_id]
                cursor = target_patch_id
                while parents[cursor] >= 0:
                    cursor = parents[cursor]
                    path.append(cursor)
                path.reverse()
                return path
            queue.append(neighbor_patch_id)
    return []


def _chain_uv_axis_metrics(chain_placement: ScaffoldChainPlacement) -> tuple[float, float]:
    if len(chain_placement.points) < 2:
        return 0.0, 0.0

    start_uv = chain_placement.points[0][1]
    end_uv = chain_placement.points[-1][1]
    delta = end_uv - start_uv

    if chain_placement.frame_role == FrameRole.H_FRAME:
        return abs(delta.x), abs(delta.y)
    if chain_placement.frame_role == FrameRole.V_FRAME:
        return abs(delta.y), abs(delta.x)
    return delta.length, 0.0


def _split_uv_by_frame_role(frame_role: FrameRole, uv: Vector) -> tuple[float, float]:
    if frame_role == FrameRole.H_FRAME:
        return uv.x, uv.y
    if frame_role == FrameRole.V_FRAME:
        return uv.y, uv.x
    return uv.length, 0.0


def _build_chain_vert_uv_map(
    graph: PatchGraph,
    chain_placement: ScaffoldChainPlacement,
) -> dict[int, Vector]:
    node = graph.nodes.get(chain_placement.patch_id)
    if node is None:
        return {}
    if chain_placement.loop_index < 0 or chain_placement.loop_index >= len(node.boundary_loops):
        return {}

    boundary_loop = node.boundary_loops[chain_placement.loop_index]
    if chain_placement.chain_index < 0 or chain_placement.chain_index >= len(boundary_loop.chains):
        return {}

    chain = boundary_loop.chains[chain_placement.chain_index]
    vert_uv_map = {}
    for point_key, uv in chain_placement.points:
        if (
            point_key.patch_id != chain_placement.patch_id
            or point_key.loop_index != chain_placement.loop_index
            or point_key.chain_index != chain_placement.chain_index
        ):
            continue
        source_point_index = point_key.source_point_index
        if 0 <= source_point_index < len(chain.vert_indices):
            vert_uv_map[chain.vert_indices[source_point_index]] = uv

    if not vert_uv_map and len(chain.vert_indices) == len(chain_placement.points):
        for source_point_index, (_, uv) in enumerate(chain_placement.points):
            vert_uv_map[chain.vert_indices[source_point_index]] = uv

    return vert_uv_map


def _measure_shared_closure_uv_offsets(
    graph: PatchGraph,
    owner_placement: ScaffoldChainPlacement,
    target_placement: ScaffoldChainPlacement,
) -> tuple[int, float, float, float, float, float, float]:
    owner_uv_by_vert = _build_chain_vert_uv_map(graph, owner_placement)
    target_uv_by_vert = _build_chain_vert_uv_map(graph, target_placement)
    shared_vert_indices = sorted(set(owner_uv_by_vert.keys()) & set(target_uv_by_vert.keys()))
    if not shared_vert_indices:
        return 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

    uv_deltas = []
    axis_offsets = []
    cross_axis_offsets = []
    for vert_index in shared_vert_indices:
        owner_uv = owner_uv_by_vert[vert_index]
        target_uv = target_uv_by_vert[vert_index]
        owner_axis, owner_cross = _split_uv_by_frame_role(owner_placement.frame_role, owner_uv)
        target_axis, target_cross = _split_uv_by_frame_role(target_placement.frame_role, target_uv)
        uv_deltas.append((owner_uv - target_uv).length)
        axis_offsets.append(abs(owner_axis - target_axis))
        cross_axis_offsets.append(abs(owner_cross - target_cross))

    sample_count = len(shared_vert_indices)
    return (
        sample_count,
        max(uv_deltas),
        sum(uv_deltas) / sample_count,
        max(axis_offsets),
        sum(axis_offsets) / sample_count,
        max(cross_axis_offsets),
        sum(cross_axis_offsets) / sample_count,
    )


def _classify_closure_anchor_mode(owner_anchor_count: int, target_anchor_count: int) -> str:
    if owner_anchor_count >= 2 and target_anchor_count >= 2:
        return 'dual-anchor'
    if owner_anchor_count <= 1 and target_anchor_count <= 1:
        return 'one-anchor'
    return 'mixed'


def _count_free_bridges_on_patch_path(
    quilt_scaffold: ScaffoldQuiltPlacement,
    patch_path: list[int],
) -> int:
    free_bridge_count = 0
    for patch_id in patch_path:
        patch_placement = quilt_scaffold.patches.get(patch_id)
        if patch_placement is None or patch_placement.notes:
            continue
        for chain_placement in patch_placement.chain_placements:
            if chain_placement.frame_role == FrameRole.FREE and len(chain_placement.points) <= 2:
                free_bridge_count += 1
    return free_bridge_count


def _match_non_tree_closure_chain_pairs(
    graph: PatchGraph,
    owner_patch_id: int,
    target_patch_id: int,
) -> list[tuple[tuple[int, int, int], BoundaryChain, tuple[int, int, int], BoundaryChain, int]]:
    owner_refs = _iter_neighbor_chains(graph, owner_patch_id, target_patch_id)
    target_refs = _iter_neighbor_chains(graph, target_patch_id, owner_patch_id)
    pair_candidates = []

    for owner_loop_index, owner_chain_index, _, owner_chain in owner_refs:
        if owner_chain.frame_role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
            continue
        owner_vert_set = set(owner_chain.vert_indices)
        if not owner_vert_set:
            continue
        for target_loop_index, target_chain_index, _, target_chain in target_refs:
            if target_chain.frame_role != owner_chain.frame_role:
                continue
            target_vert_set = set(target_chain.vert_indices)
            shared_vert_count = len(owner_vert_set & target_vert_set)
            if shared_vert_count <= 0:
                continue
            pair_score = (shared_vert_count * 1000) - abs(len(owner_chain.vert_indices) - len(target_chain.vert_indices))
            pair_candidates.append((
                pair_score,
                shared_vert_count,
                (owner_patch_id, owner_loop_index, owner_chain_index),
                owner_chain,
                (target_patch_id, target_loop_index, target_chain_index),
                target_chain,
            ))

    pair_candidates.sort(key=lambda item: item[0], reverse=True)
    matched_owner_refs = set()
    matched_target_refs = set()
    matched_pairs = []
    for _, shared_vert_count, owner_ref, owner_chain, target_ref, target_chain in pair_candidates:
        if owner_ref in matched_owner_refs or target_ref in matched_target_refs:
            continue
        matched_owner_refs.add(owner_ref)
        matched_target_refs.add(target_ref)
        matched_pairs.append((owner_ref, owner_chain, target_ref, target_chain, shared_vert_count))

    return matched_pairs


def _build_quilt_closure_pair_map(
    graph: PatchGraph,
    quilt_patch_ids: set[int],
    allowed_tree_edges: set[tuple[int, int]],
) -> dict[tuple[int, int, int], tuple[int, int, int]]:
    pair_map = {}
    non_tree_patch_pairs = sorted(
        edge_key
        for edge_key in graph.edges.keys()
        if edge_key[0] in quilt_patch_ids
        and edge_key[1] in quilt_patch_ids
        and edge_key not in allowed_tree_edges
    )
    for owner_patch_id, target_patch_id in non_tree_patch_pairs:
        matched_pairs = _match_non_tree_closure_chain_pairs(graph, owner_patch_id, target_patch_id)
        for owner_ref, _, target_ref, _, _ in matched_pairs:
            pair_map[owner_ref] = target_ref
            pair_map[target_ref] = owner_ref
    return pair_map


def _collect_quilt_closure_seam_reports(
    graph: PatchGraph,
    quilt_plan: QuiltPlan,
    quilt_scaffold: ScaffoldQuiltPlacement,
    placed_chains_map: dict[tuple[int, int, int], ScaffoldChainPlacement],
    final_scale: float,
    allowed_tree_edges: set[tuple[int, int]],
) -> tuple[ScaffoldClosureSeamReport, ...]:
    quilt_patch_ids = set(quilt_plan.solved_patch_ids)
    quilt_patch_ids.add(quilt_plan.root_patch_id)
    patch_tree_adjacency = _build_patch_tree_adjacency(quilt_plan)
    non_tree_patch_pairs = sorted(
        edge_key
        for edge_key in graph.edges.keys()
        if edge_key[0] in quilt_patch_ids
        and edge_key[1] in quilt_patch_ids
        and edge_key not in allowed_tree_edges
    )

    reports = []
    for owner_patch_id, target_patch_id in non_tree_patch_pairs:
        matched_pairs = _match_non_tree_closure_chain_pairs(graph, owner_patch_id, target_patch_id)
        if not matched_pairs:
            continue

        patch_path = _find_patch_tree_path(patch_tree_adjacency, owner_patch_id, target_patch_id)
        tree_patch_distance = max(0, len(patch_path) - 1) if patch_path else 0
        free_bridge_count = _count_free_bridges_on_patch_path(quilt_scaffold, patch_path) if patch_path else 0

        for owner_ref, owner_chain, target_ref, target_chain, shared_vert_count in matched_pairs:
            owner_placement = placed_chains_map.get(owner_ref)
            target_placement = placed_chains_map.get(target_ref)
            if owner_placement is None or target_placement is None:
                continue
            if len(owner_placement.points) < 2 or len(target_placement.points) < 2:
                continue

            owner_uv_span, owner_axis_error = _chain_uv_axis_metrics(owner_placement)
            target_uv_span, target_axis_error = _chain_uv_axis_metrics(target_placement)
            (
                sampled_shared_vert_count,
                shared_uv_delta_max,
                shared_uv_delta_mean,
                axis_phase_offset_max,
                axis_phase_offset_mean,
                cross_axis_offset_max,
                cross_axis_offset_mean,
            ) = _measure_shared_closure_uv_offsets(graph, owner_placement, target_placement)
            canonical_3d_span = (
                _cf_chain_total_length(owner_chain, final_scale)
                + _cf_chain_total_length(target_chain, final_scale)
            ) * 0.5

            reports.append(ScaffoldClosureSeamReport(
                owner_patch_id=owner_ref[0],
                owner_loop_index=owner_ref[1],
                owner_chain_index=owner_ref[2],
                target_patch_id=target_ref[0],
                target_loop_index=target_ref[1],
                target_chain_index=target_ref[2],
                frame_role=owner_chain.frame_role,
                owner_anchor_count=owner_placement.anchor_count,
                target_anchor_count=target_placement.anchor_count,
                anchor_mode=_classify_closure_anchor_mode(owner_placement.anchor_count, target_placement.anchor_count),
                canonical_3d_span=canonical_3d_span,
                owner_uv_span=owner_uv_span,
                target_uv_span=target_uv_span,
                owner_axis_error=owner_axis_error,
                target_axis_error=target_axis_error,
                span_mismatch=abs(owner_uv_span - target_uv_span),
                sampled_shared_vert_count=sampled_shared_vert_count,
                shared_uv_delta_max=shared_uv_delta_max,
                shared_uv_delta_mean=shared_uv_delta_mean,
                axis_phase_offset_max=axis_phase_offset_max,
                axis_phase_offset_mean=axis_phase_offset_mean,
                cross_axis_offset_max=cross_axis_offset_max,
                cross_axis_offset_mean=cross_axis_offset_mean,
                tree_patch_distance=tree_patch_distance,
                free_bridge_count=free_bridge_count,
                shared_vert_count=shared_vert_count,
            ))

    reports.sort(
        key=lambda report: (
            -report.axis_phase_offset_max,
            -report.cross_axis_offset_max,
            -report.shared_uv_delta_max,
            -report.span_mismatch,
            report.owner_patch_id,
            report.target_patch_id,
            report.owner_chain_index,
            report.target_chain_index,
        )
    )
    return tuple(reports)


def _build_temporary_chain_placement(
    chain_ref: tuple[int, int, int],
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
        source_kind='chain',
        anchor_count=_cf_anchor_count(start_anchor, end_anchor),
        start_anchor_kind=start_anchor.source_kind if start_anchor is not None else '',
        end_anchor_kind=end_anchor.source_kind if end_anchor is not None else '',
        points=tuple(
            (ScaffoldPointKey(patch_id, loop_index, chain_index, point_index), uv.copy())
            for point_index, uv in enumerate(uv_points)
        ),
    )


def _closure_preconstraint_direction_options(
    chain: BoundaryChain,
    node: PatchNode,
    start_anchor: Optional[ChainAnchor],
    end_anchor: Optional[ChainAnchor],
    graph: PatchGraph,
    point_registry: dict[tuple[int, int, int, int], Vector],
) -> list[tuple[str, Optional[Vector]]]:
    options = [('default', None)]
    if chain.frame_role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        return options

    base_direction = _try_inherit_direction(chain, node, start_anchor, end_anchor, graph, point_registry)
    if base_direction is None:
        base_direction = _cf_determine_direction(chain, node)
    axis_direction = _snap_direction_to_role(base_direction, chain.frame_role)
    axis_direction = _normalize_direction(axis_direction, _default_role_direction(chain.frame_role))
    flipped_direction = Vector((-axis_direction.x, -axis_direction.y))
    if (flipped_direction - axis_direction).length > 1e-8:
        options.append(('flip', flipped_direction))
    return options


def _closure_preconstraint_metric(
    graph: PatchGraph,
    chain_ref: tuple[int, int, int],
    chain: BoundaryChain,
    uv_points: list[Vector],
    start_anchor: Optional[ChainAnchor],
    end_anchor: Optional[ChainAnchor],
    raw_start_anchor: Optional[ChainAnchor],
    raw_end_anchor: Optional[ChainAnchor],
    partner_placement: ScaffoldChainPlacement,
) -> tuple[float, float, float, float, float, float]:
    temporary_placement = _build_temporary_chain_placement(
        chain_ref,
        chain,
        uv_points,
        start_anchor,
        end_anchor,
    )
    (
        _sampled_shared_vert_count,
        _shared_uv_delta_max,
        _shared_uv_delta_mean,
        axis_phase_offset_max,
        axis_phase_offset_mean,
        _cross_axis_offset_max,
        _cross_axis_offset_mean,
    ) = _measure_shared_closure_uv_offsets(graph, temporary_placement, partner_placement)
    predicted_uv_span, _ = _chain_uv_axis_metrics(temporary_placement)
    partner_uv_span, _ = _chain_uv_axis_metrics(partner_placement)
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
        if raw_anchor.source_kind == 'same_patch':
            same_patch_gap_max = max(same_patch_gap_max, gap)

    all_gap_mean = all_gap_sum / gap_count if gap_count > 0 else 0.0
    return (
        same_patch_gap_max,
        axis_phase_offset_max,
        all_gap_max,
        span_mismatch,
        axis_phase_offset_mean,
        all_gap_mean,
    )


def _cf_apply_closure_preconstraint(
    chain_ref: tuple[int, int, int],
    chain: BoundaryChain,
    node: PatchNode,
    raw_start_anchor: Optional[ChainAnchor],
    raw_end_anchor: Optional[ChainAnchor],
    start_anchor: Optional[ChainAnchor],
    end_anchor: Optional[ChainAnchor],
    known: int,
    graph: PatchGraph,
    point_registry: dict[tuple[int, int, int, int], Vector],
    placed_chains_map: dict[tuple[int, int, int], ScaffoldChainPlacement],
    closure_pair_map: dict[tuple[int, int, int], tuple[int, int, int]],
    final_scale: float,
) -> tuple[Optional[ChainAnchor], Optional[ChainAnchor], Optional[Vector], str]:
    if known != 1 or chain.frame_role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        return start_anchor, end_anchor, None, ''

    partner_ref = closure_pair_map.get(chain_ref)
    if partner_ref is None:
        return start_anchor, end_anchor, None, ''
    partner_placement = placed_chains_map.get(partner_ref)
    if partner_placement is None or len(partner_placement.points) < 2:
        return start_anchor, end_anchor, None, ''

    anchor_options = []
    seen_anchor_keys = set()
    for anchor_label, option_start, option_end in (
        ('resolved', start_anchor, end_anchor),
        ('start_only', raw_start_anchor, None),
        ('end_only', None, raw_end_anchor),
    ):
        if _cf_anchor_count(option_start, option_end) != 1:
            continue
        anchor_key = (
            option_start.source_ref if option_start is not None else None,
            option_end.source_ref if option_end is not None else None,
        )
        if anchor_key in seen_anchor_keys:
            continue
        seen_anchor_keys.add(anchor_key)
        anchor_options.append((anchor_label, option_start, option_end))

    if not anchor_options:
        return start_anchor, end_anchor, None, ''

    option_results = []
    for anchor_label, option_start, option_end in anchor_options:
        direction_options = _closure_preconstraint_direction_options(
            chain,
            node,
            option_start,
            option_end,
            graph,
            point_registry,
        )
        for direction_label, direction_override in direction_options:
            uv_points = _cf_place_chain(
                chain,
                node,
                option_start,
                option_end,
                final_scale,
                direction_override,
            )
            if not uv_points or len(uv_points) != len(chain.vert_cos):
                continue
            metric = _closure_preconstraint_metric(
                graph,
                chain_ref,
                chain,
                uv_points,
                option_start,
                option_end,
                raw_start_anchor,
                raw_end_anchor,
                partner_placement,
            )
            option_results.append((
                metric,
                anchor_label,
                direction_label,
                option_start,
                option_end,
                direction_override,
            ))

    if not option_results:
        return start_anchor, end_anchor, None, ''

    current_anchor_key = (
        start_anchor.source_ref if start_anchor is not None else None,
        end_anchor.source_ref if end_anchor is not None else None,
    )
    current_metric = None
    best_result = None
    for result in sorted(option_results, key=lambda item: item[0]):
        metric, anchor_label, direction_label, option_start, option_end, direction_override = result
        option_anchor_key = (
            option_start.source_ref if option_start is not None else None,
            option_end.source_ref if option_end is not None else None,
        )
        if option_anchor_key == current_anchor_key and direction_label == 'default' and current_metric is None:
            current_metric = metric
        if best_result is None:
            best_result = result

    if best_result is None or current_metric is None:
        return start_anchor, end_anchor, None, ''

    best_metric, best_anchor_label, best_direction_label, best_start, best_end, best_direction_override = best_result
    if best_metric >= current_metric:
        return start_anchor, end_anchor, None, ''

    reason = (
        f"closure_preconstraint:{best_anchor_label}/{best_direction_label}"
        f":phase={best_metric[1]:.4f}:gap={best_metric[0]:.4f}"
    )
    return best_start, best_end, best_direction_override, reason


def _print_quilt_closure_seam_reports(
    quilt_index: int,
    closure_seam_reports: tuple[ScaffoldClosureSeamReport, ...],
) -> None:
    if not closure_seam_reports:
        return

    max_mismatch = max((report.span_mismatch for report in closure_seam_reports), default=0.0)
    max_axis_phase = max((report.axis_phase_offset_max for report in closure_seam_reports), default=0.0)
    print(
        f"[CFTUV][ClosureDiag] Quilt {quilt_index}: "
        f"seams={len(closure_seam_reports)} "
        f"max_span_mismatch={max_mismatch:.6f} "
        f"max_axis_phase={max_axis_phase:.6f}"
    )
    for report in closure_seam_reports:
        print(
            f"[CFTUV][ClosureDiag] Quilt {quilt_index} "
            f"P{report.owner_patch_id} L{report.owner_loop_index}C{report.owner_chain_index}"
            f"<->P{report.target_patch_id} L{report.target_loop_index}C{report.target_chain_index} "
            f"{report.frame_role.value} mode:{report.anchor_mode} "
            f"a:{report.owner_anchor_count}/{report.target_anchor_count} "
            f"span3d:{report.canonical_3d_span:.6f} "
            f"uv:{report.owner_uv_span:.6f}/{report.target_uv_span:.6f} "
            f"mismatch:{report.span_mismatch:.6f} "
            f"axis:{report.owner_axis_error:.6f}/{report.target_axis_error:.6f} "
            f"phase:{report.axis_phase_offset_max:.6f}/{report.axis_phase_offset_mean:.6f} "
            f"cross:{report.cross_axis_offset_max:.6f}/{report.cross_axis_offset_mean:.6f} "
            f"uvd:{report.shared_uv_delta_max:.6f}/{report.shared_uv_delta_mean:.6f} "
            f"path:{report.tree_patch_distance} free:{report.free_bridge_count} "
            f"shared_verts:{report.sampled_shared_vert_count}/{report.shared_vert_count}"
        )


def _frame_cross_axis_uv_value(chain_placement: ScaffoldChainPlacement) -> float:
    if not chain_placement.points:
        return 0.0
    if chain_placement.frame_role == FrameRole.H_FRAME:
        return sum(uv.y for _, uv in chain_placement.points) / float(len(chain_placement.points))
    if chain_placement.frame_role == FrameRole.V_FRAME:
        return sum(uv.x for _, uv in chain_placement.points) / float(len(chain_placement.points))
    return 0.0


def _wall_side_row_class_key(chain: BoundaryChain) -> tuple[int]:
    if not chain.vert_cos:
        return (0,)
    avg_z = sum(point.z for point in chain.vert_cos) / float(len(chain.vert_cos))
    return (int(round(avg_z / FRAME_ROW_GROUP_TOLERANCE)),)


def _wall_side_column_class_key(chain: BoundaryChain) -> tuple[int, int]:
    if not chain.vert_cos:
        return (0, 0)
    avg_x = sum(point.x for point in chain.vert_cos) / float(len(chain.vert_cos))
    avg_y = sum(point.y for point in chain.vert_cos) / float(len(chain.vert_cos))
    return (
        int(round(avg_x / FRAME_COLUMN_GROUP_TOLERANCE)),
        int(round(avg_y / FRAME_COLUMN_GROUP_TOLERANCE)),
    )


def _frame_group_display_coords(
    axis_kind: str,
    class_key: tuple[int, ...],
) -> tuple[float, float]:
    if axis_kind == 'ROW':
        return class_key[0] * FRAME_ROW_GROUP_TOLERANCE, 0.0
    if axis_kind == 'COLUMN':
        return (
            class_key[0] * FRAME_COLUMN_GROUP_TOLERANCE,
            class_key[1] * FRAME_COLUMN_GROUP_TOLERANCE,
        )
    return 0.0, 0.0


def _collect_quilt_closure_sensitive_patch_ids(
    quilt_plan: QuiltPlan,
    closure_seam_reports: tuple[ScaffoldClosureSeamReport, ...],
) -> set[int]:
    if not closure_seam_reports:
        return set()

    patch_tree_adjacency = _build_patch_tree_adjacency(quilt_plan)
    closure_sensitive_patch_ids: set[int] = set()
    for report in closure_seam_reports:
        path = _find_patch_tree_path(
            patch_tree_adjacency,
            report.owner_patch_id,
            report.target_patch_id,
        )
        if path:
            closure_sensitive_patch_ids.update(path)
        else:
            closure_sensitive_patch_ids.add(report.owner_patch_id)
            closure_sensitive_patch_ids.add(report.target_patch_id)
    return closure_sensitive_patch_ids


def _collect_quilt_frame_alignment_reports(
    graph: PatchGraph,
    quilt_plan: QuiltPlan,
    quilt_scaffold: ScaffoldQuiltPlacement,
    final_scale: float,
    closure_seam_reports: tuple[ScaffoldClosureSeamReport, ...],
) -> tuple[ScaffoldFrameAlignmentReport, ...]:
    closure_sensitive_patch_ids = _collect_quilt_closure_sensitive_patch_ids(quilt_plan, closure_seam_reports)
    grouped_members: dict[tuple[str, str, tuple[int, ...]], list[tuple[tuple[int, int, int], float, float, int]]] = {}

    for patch_id, patch_placement in quilt_scaffold.patches.items():
        if patch_placement is None or patch_placement.notes:
            continue
        node = graph.nodes.get(patch_id)
        if node is None or node.semantic_key != 'WALL.SIDE':
            continue
        if patch_placement.loop_index < 0 or patch_placement.loop_index >= len(node.boundary_loops):
            continue
        boundary_loop = node.boundary_loops[patch_placement.loop_index]

        for chain_placement in patch_placement.chain_placements:
            if chain_placement.frame_role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
                continue
            if chain_placement.chain_index < 0 or chain_placement.chain_index >= len(boundary_loop.chains):
                continue
            chain = boundary_loop.chains[chain_placement.chain_index]
            if len(chain.vert_cos) < 2:
                continue

            axis_kind = 'ROW' if chain_placement.frame_role == FrameRole.H_FRAME else 'COLUMN'
            if axis_kind == 'ROW':
                class_key = _wall_side_row_class_key(chain)
            else:
                class_key = _wall_side_column_class_key(chain)
            ref = (patch_id, patch_placement.loop_index, chain_placement.chain_index)
            cross_uv = _frame_cross_axis_uv_value(chain_placement)
            weight = max(_cf_chain_total_length(chain, final_scale), 1e-8)
            grouped_members.setdefault((axis_kind, node.semantic_key, class_key), []).append(
                (ref, cross_uv, weight, patch_id)
            )

    reports = []
    for (axis_kind, semantic_key, class_key), members in grouped_members.items():
        if len(members) < 2:
            continue

        total_weight = sum(weight for _, _, weight, _ in members)
        if total_weight <= 1e-8:
            continue
        target_cross_uv = sum(cross_uv * weight for _, cross_uv, weight, _ in members) / total_weight
        scatter_values = [abs(cross_uv - target_cross_uv) for _, cross_uv, _, _ in members]
        class_coord_a, class_coord_b = _frame_group_display_coords(axis_kind, class_key)

        reports.append(ScaffoldFrameAlignmentReport(
            axis_kind=axis_kind,
            semantic_key=semantic_key,
            frame_role=FrameRole.H_FRAME if axis_kind == 'ROW' else FrameRole.V_FRAME,
            class_coord_a=class_coord_a,
            class_coord_b=class_coord_b,
            chain_count=len(members),
            total_weight=total_weight,
            target_cross_uv=target_cross_uv,
            scatter_max=max(scatter_values),
            scatter_mean=sum(scatter_values) / float(len(scatter_values)),
            closure_sensitive=any(patch_id in closure_sensitive_patch_ids for _, _, _, patch_id in members),
            member_refs=tuple(sorted(ref for ref, _, _, _ in members)),
        ))

    reports.sort(
        key=lambda report: (
            -report.scatter_max,
            -report.scatter_mean,
            report.axis_kind,
            report.class_coord_a,
            report.class_coord_b,
        )
    )
    return tuple(reports)


def _print_quilt_frame_alignment_reports(
    quilt_index: int,
    frame_alignment_reports: tuple[ScaffoldFrameAlignmentReport, ...],
) -> None:
    if not frame_alignment_reports:
        return

    max_row_scatter = max(
        (report.scatter_max for report in frame_alignment_reports if report.axis_kind == 'ROW'),
        default=0.0,
    )
    max_column_scatter = max(
        (report.scatter_max for report in frame_alignment_reports if report.axis_kind == 'COLUMN'),
        default=0.0,
    )
    print(
        f"[CFTUV][FrameDiag] Quilt {quilt_index}: "
        f"groups={len(frame_alignment_reports)} "
        f"max_row_scatter={max_row_scatter:.6f} "
        f"max_column_scatter={max_column_scatter:.6f}"
    )
    for report in frame_alignment_reports:
        coord_label = (
            f"z:{report.class_coord_a:.6f}"
            if report.axis_kind == 'ROW'
            else f"xy:({report.class_coord_a:.6f},{report.class_coord_b:.6f})"
        )
        refs_label = ','.join(
            f"P{patch_id}L{loop_index}C{chain_index}"
            for patch_id, loop_index, chain_index in report.member_refs
        )
        closure_tag = " closure:1" if report.closure_sensitive else " closure:0"
        print(
            f"[CFTUV][FrameDiag] Quilt {quilt_index} "
            f"{report.axis_kind} {report.frame_role.value} {coord_label} "
            f"chains:{report.chain_count} target:{report.target_cross_uv:.6f} "
            f"scatter:{report.scatter_max:.6f}/{report.scatter_mean:.6f} "
            f"weight:{report.total_weight:.6f}{closure_tag} refs:[{refs_label}]"
        )


def _count_patch_roles(node: PatchNode) -> tuple[int, int, int, int, int, int]:
    outer_count = 0
    hole_count = 0
    chain_count = 0
    free_count = 0
    h_count = 0
    v_count = 0

    for boundary_loop in node.boundary_loops:
        if boundary_loop.kind == LoopKind.OUTER:
            outer_count += 1
        elif boundary_loop.kind == LoopKind.HOLE:
            hole_count += 1

        for chain in boundary_loop.chains:
            chain_count += 1
            if chain.frame_role == FrameRole.H_FRAME:
                h_count += 1
            elif chain.frame_role == FrameRole.V_FRAME:
                v_count += 1
            else:
                free_count += 1

    return outer_count, hole_count, chain_count, free_count, h_count, v_count


def _frame_presence_strength(h_count: int, v_count: int) -> float:
    if h_count > 0 and v_count > 0:
        return 1.0
    if h_count > 0 or v_count > 0:
        return 0.6
    return 0.2


def _semantic_stability_bonus(node: PatchNode) -> float:
    patch_type = node.patch_type.value if hasattr(node.patch_type, 'value') else str(node.patch_type)
    world_facing = node.world_facing.value if hasattr(node.world_facing, 'value') else str(node.world_facing)
    if patch_type == "WALL" and world_facing == "SIDE":
        return 0.06
    if patch_type == "FLOOR" and world_facing in {"UP", "DOWN"}:
        return 0.06
    if patch_type == "SLOPE":
        return 0.04
    return 0.02


def _build_patch_certainty(node: PatchNode, max_area: float) -> PatchCertainty:
    outer_count, hole_count, chain_count, free_count, h_count, v_count = _count_patch_roles(node)
    local_solvable = bool(node.boundary_loops) and outer_count == 1 and all(loop.chains for loop in node.boundary_loops)
    if not local_solvable:
        return PatchCertainty(
            patch_id=node.patch_id,
            local_solvable=False,
            root_score=0.0,
            outer_count=outer_count,
            hole_count=hole_count,
            chain_count=chain_count,
            free_count=free_count,
            h_count=h_count,
            v_count=v_count,
            reasons=("local_gate=fail",),
        )

    area_norm = 0.0 if max_area <= 0.0 else _clamp01(node.area / max_area)
    free_ratio = 1.0 if chain_count <= 0 else float(free_count) / float(chain_count)
    frame_strength = _frame_presence_strength(h_count, v_count)
    hole_factor = _clamp01(1.0 - min(hole_count, 2) * 0.35)
    root_score = (
        ROOT_WEIGHT_AREA * area_norm
        + ROOT_WEIGHT_FRAME * frame_strength
        + ROOT_WEIGHT_FREE_RATIO * _clamp01(1.0 - free_ratio)
        + ROOT_WEIGHT_HOLES * hole_factor
        + ROOT_WEIGHT_BASE * 1.0
        + _semantic_stability_bonus(node)
    )
    root_score = _clamp01(root_score)

    reasons = (
        f"area={area_norm:.2f}",
        f"frame={frame_strength:.2f}",
        f"free={_clamp01(1.0 - free_ratio):.2f}",
        f"holes={hole_factor:.2f}",
    )
    return PatchCertainty(
        patch_id=node.patch_id,
        local_solvable=True,
        root_score=root_score,
        outer_count=outer_count,
        hole_count=hole_count,
        chain_count=chain_count,
        free_count=free_count,
        h_count=h_count,
        v_count=v_count,
        reasons=reasons,
    )


def _chain_endpoint_strength(
    graph: PatchGraph,
    patch_id: int,
    loop_index: int,
    chain_index: int,
    chain: BoundaryChain,
) -> float:
    score = 0.0
    if chain.start_corner_index >= 0:
        score += 0.35
    if chain.end_corner_index >= 0:
        score += 0.35

    neighbors = graph.get_chain_endpoint_neighbors(patch_id, loop_index, chain_index)
    if neighbors.get("start"):
        score += 0.15
    if neighbors.get("end"):
        score += 0.15
    return _clamp01(score)


def _get_chain_corner(graph: PatchGraph, patch_id: int, loop_index: int, corner_index: int):
    node = graph.nodes.get(patch_id)
    if node is None or loop_index < 0 or loop_index >= len(node.boundary_loops):
        return None
    boundary_loop = node.boundary_loops[loop_index]
    if corner_index < 0 or corner_index >= len(boundary_loop.corners):
        return None
    return boundary_loop.corners[corner_index]


def _get_chain_endpoint_context(graph: PatchGraph, patch_id: int, loop_index: int, chain_index: int, endpoint_label: str):
    chain = graph.get_chain(patch_id, loop_index, chain_index)
    if chain is None:
        return None

    endpoint_neighbors = graph.get_chain_endpoint_neighbors(patch_id, loop_index, chain_index)
    if endpoint_label == 'start':
        return {
            'vert_index': chain.start_vert_index,
            'corner': _get_chain_corner(graph, patch_id, loop_index, chain.start_corner_index),
            'neighbors': endpoint_neighbors.get('start', []),
        }
    return {
        'vert_index': chain.end_vert_index,
        'corner': _get_chain_corner(graph, patch_id, loop_index, chain.end_corner_index),
        'neighbors': endpoint_neighbors.get('end', []),
    }


def _corner_similarity_strength(owner_corner, target_corner) -> float:
    if owner_corner is None or target_corner is None:
        return 0.0
    angle_delta = abs(owner_corner.turn_angle_deg - target_corner.turn_angle_deg)
    similarity = _clamp01(1.0 - (angle_delta / 90.0))
    sharp_avg = _clamp01(((owner_corner.turn_angle_deg / 90.0) + (target_corner.turn_angle_deg / 90.0)) * 0.5)
    return _clamp01(0.5 * similarity + 0.5 * sharp_avg)


def _find_endpoint_matches(owner_chain: BoundaryChain, target_chain: BoundaryChain):
    owner_endpoints = (('start', owner_chain.start_vert_index), ('end', owner_chain.end_vert_index))
    target_endpoints = (('start', target_chain.start_vert_index), ('end', target_chain.end_vert_index))
    matches = []
    for owner_label, owner_vert in owner_endpoints:
        if owner_vert < 0:
            continue
        for target_label, target_vert in target_endpoints:
            if owner_vert != target_vert or target_vert < 0:
                continue
            matches.append((owner_label, target_label))
    return matches


def _endpoint_bridge_strength(
    graph: PatchGraph,
    owner_patch_id: int,
    owner_loop_index: int,
    owner_chain_index: int,
    owner_chain: BoundaryChain,
    target_patch_id: int,
    target_loop_index: int,
    target_chain_index: int,
    target_chain: BoundaryChain,
) -> tuple[float, float]:
    endpoint_matches = _find_endpoint_matches(owner_chain, target_chain)
    if not endpoint_matches:
        return 0.0, 0.0

    bridge_scores = []
    corner_scores = []
    for owner_label, target_label in endpoint_matches:
        owner_ctx = _get_chain_endpoint_context(graph, owner_patch_id, owner_loop_index, owner_chain_index, owner_label)
        target_ctx = _get_chain_endpoint_context(graph, target_patch_id, target_loop_index, target_chain_index, target_label)
        if owner_ctx is None or target_ctx is None:
            continue

        corner_strength = _corner_similarity_strength(owner_ctx['corner'], target_ctx['corner'])
        corner_scores.append(corner_strength)

        best_neighbor_bridge = 0.0
        for owner_ref in owner_ctx['neighbors']:
            owner_neighbor_chain = graph.get_chain(owner_patch_id, owner_ref[0], owner_ref[1])
            if owner_neighbor_chain is None:
                continue
            for target_ref in target_ctx['neighbors']:
                target_neighbor_chain = graph.get_chain(target_patch_id, target_ref[0], target_ref[1])
                if target_neighbor_chain is None:
                    continue
                best_neighbor_bridge = max(
                    best_neighbor_bridge,
                    _frame_continuation_strength(owner_neighbor_chain.frame_role, target_neighbor_chain.frame_role),
                )

        bridge_scores.append(_clamp01(0.75 * best_neighbor_bridge + 0.25 * corner_strength))

    if not bridge_scores:
        return 0.0, 0.0
    return _clamp01(sum(bridge_scores) / len(bridge_scores)), _clamp01(sum(corner_scores) / len(corner_scores))


def _frame_continuation_strength(owner_role: FrameRole, target_role: FrameRole) -> float:
    owner_is_frame = owner_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
    target_is_frame = target_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}

    if owner_is_frame and target_is_frame and owner_role == target_role:
        return 1.0
    if owner_is_frame and target_is_frame:
        return 0.55
    if owner_is_frame or target_is_frame:
        return 0.05
    return 0.0


def _semantic_pair_strength(graph: PatchGraph, owner_patch_id: int, target_patch_id: int) -> float:
    owner_key = graph.get_patch_semantic_key(owner_patch_id)
    target_key = graph.get_patch_semantic_key(target_patch_id)
    if owner_key == target_key:
        return 1.0

    owner_type = owner_key.split('.', 1)[0]
    target_type = target_key.split('.', 1)[0]
    if owner_type == target_type:
        return 0.85

    pair = {owner_type, target_type}
    if pair == {'WALL', 'FLOOR'}:
        return 0.35
    if pair == {'WALL', 'SLOPE'}:
        return 0.55
    if pair == {'FLOOR', 'SLOPE'}:
        return 0.55
    return 0.40


def _patch_types_compatible(graph: PatchGraph, owner_patch_id: int, target_patch_id: int) -> bool:
    owner_node = graph.nodes.get(owner_patch_id)
    target_node = graph.nodes.get(target_patch_id)
    if owner_node is None or target_node is None:
        return False

    owner_type = owner_node.patch_type.value if hasattr(owner_node.patch_type, 'value') else str(owner_node.patch_type)
    target_type = target_node.patch_type.value if hasattr(target_node.patch_type, 'value') else str(target_node.patch_type)
    return owner_type == target_type


def _loop_pair_strength(owner_loop_kind: LoopKind, target_loop_kind: LoopKind) -> float:
    if owner_loop_kind == LoopKind.OUTER and target_loop_kind == LoopKind.OUTER:
        return 1.0
    if owner_loop_kind == target_loop_kind:
        return 0.55
    return 0.35


def _best_chain_pair(
    graph: PatchGraph,
    owner_patch_id: int,
    owner_refs,
    target_patch_id: int,
    target_refs,
):
    best_pair = None
    best_strength = -1.0
    best_endpoint = 0.0
    best_frame_continuation = 0.0
    best_endpoint_bridge = 0.0
    best_corner_strength = 0.0
    best_semantic_strength = 0.0
    semantic_strength = _semantic_pair_strength(graph, owner_patch_id, target_patch_id)

    for owner_loop_index, owner_chain_index, owner_loop, owner_chain in owner_refs:
        owner_endpoint = _chain_endpoint_strength(graph, owner_patch_id, owner_loop_index, owner_chain_index, owner_chain)
        for target_loop_index, target_chain_index, target_loop, target_chain in target_refs:
            target_endpoint = _chain_endpoint_strength(graph, target_patch_id, target_loop_index, target_chain_index, target_chain)
            endpoint_strength = (owner_endpoint + target_endpoint) * 0.5
            frame_continuation = _frame_continuation_strength(owner_chain.frame_role, target_chain.frame_role)
            endpoint_bridge, corner_strength = _endpoint_bridge_strength(
                graph,
                owner_patch_id,
                owner_loop_index,
                owner_chain_index,
                owner_chain,
                target_patch_id,
                target_loop_index,
                target_chain_index,
                target_chain,
            )
            pair_strength = (
                PAIR_WEIGHT_FRAME_CONT * frame_continuation
                + PAIR_WEIGHT_ENDPOINT * endpoint_bridge
                + PAIR_WEIGHT_CORNER * corner_strength
                + PAIR_WEIGHT_SEMANTIC * semantic_strength
                + PAIR_WEIGHT_EP_STRENGTH * endpoint_strength
                + PAIR_WEIGHT_LOOP * _loop_pair_strength(owner_loop.kind, target_loop.kind)
            )
            if pair_strength <= best_strength:
                continue
            best_strength = pair_strength
            best_endpoint = endpoint_strength
            best_frame_continuation = frame_continuation
            best_endpoint_bridge = endpoint_bridge
            best_corner_strength = corner_strength
            best_semantic_strength = semantic_strength
            best_pair = (
                owner_loop_index,
                owner_chain_index,
                owner_loop,
                owner_chain,
                target_loop_index,
                target_chain_index,
                target_loop,
                target_chain,
            )

    return (
        best_pair,
        _clamp01(best_strength),
        _clamp01(best_endpoint),
        _clamp01(best_frame_continuation),
        _clamp01(best_endpoint_bridge),
        _clamp01(best_corner_strength),
        _clamp01(best_semantic_strength),
    )


def _build_attachment_candidate(
    graph: PatchGraph,
    owner_patch_id: int,
    target_patch_id: int,
    seam,
    patch_scores: dict[int, PatchCertainty],
    max_shared_length: float,
) -> Optional[AttachmentCandidate]:
    owner_score = patch_scores[owner_patch_id]
    target_score = patch_scores[target_patch_id]
    if not _patch_types_compatible(graph, owner_patch_id, target_patch_id):
        return None
    owner_refs = _iter_neighbor_chains(graph, owner_patch_id, target_patch_id)
    target_refs = _iter_neighbor_chains(graph, target_patch_id, owner_patch_id)

    if not owner_refs or not target_refs:
        return None

    best_pair, best_pair_strength, endpoint_strength, frame_continuation, endpoint_bridge, corner_strength, semantic_strength = _best_chain_pair(
        graph,
        owner_patch_id,
        owner_refs,
        target_patch_id,
        target_refs,
    )
    if best_pair is None:
        return None

    (
        owner_loop_index,
        owner_chain_index,
        owner_loop,
        owner_chain,
        target_loop_index,
        target_chain_index,
        target_loop,
        target_chain,
    ) = best_pair

    seam_norm = 1.0 if max_shared_length <= 0.0 else _clamp01(seam.shared_length / max_shared_length)
    ambiguity_penalty = min(0.15, 0.05 * max(0, len(owner_refs) - 1) + 0.05 * max(0, len(target_refs) - 1))

    if not owner_score.local_solvable or not target_score.local_solvable:
        total_score = 0.0
    else:
        total_score = (
            ATTACH_WEIGHT_SEAM * seam_norm
            + ATTACH_WEIGHT_PAIR * best_pair_strength
            + ATTACH_WEIGHT_TARGET * target_score.root_score
            + ATTACH_WEIGHT_OWNER * owner_score.root_score
            - ambiguity_penalty
        )
        owner_is_frame = owner_chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
        target_is_frame = target_chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
        if owner_is_frame ^ target_is_frame:
            total_score -= max(0.0, 0.12 - 0.10 * endpoint_bridge)
        elif not owner_is_frame and not target_is_frame:
            total_score -= max(0.0, 0.25 - 0.20 * endpoint_bridge)
        if target_score.hole_count > 0:
            total_score -= min(0.08, target_score.hole_count * 0.04)
        total_score = _clamp01(total_score)

    reasons = (
        f"seam={seam_norm:.2f}",
        f"pair={best_pair_strength:.2f}",
        f"cont={frame_continuation:.2f}",
        f"bridge={endpoint_bridge:.2f}",
        f"corner={corner_strength:.2f}",
        f"sem={semantic_strength:.2f}",
        f"ep={endpoint_strength:.2f}",
        f"target={target_score.root_score:.2f}",
        f"amb={ambiguity_penalty:.2f}",
    )
    return AttachmentCandidate(
        owner_patch_id=owner_patch_id,
        target_patch_id=target_patch_id,
        score=total_score,
        seam_length=seam.shared_length,
        seam_norm=seam_norm,
        best_pair_strength=best_pair_strength,
        frame_continuation=frame_continuation,
        endpoint_bridge=endpoint_bridge,
        corner_strength=corner_strength,
        semantic_strength=semantic_strength,
        endpoint_strength=endpoint_strength,
        owner_certainty=owner_score.root_score,
        target_certainty=target_score.root_score,
        ambiguity_penalty=ambiguity_penalty,
        owner_loop_index=owner_loop_index,
        owner_chain_index=owner_chain_index,
        target_loop_index=target_loop_index,
        target_chain_index=target_chain_index,
        owner_loop_kind=owner_loop.kind,
        target_loop_kind=target_loop.kind,
        owner_role=owner_chain.frame_role,
        target_role=target_chain.frame_role,
        owner_transition=graph.describe_chain_transition(owner_patch_id, owner_chain),
        target_transition=graph.describe_chain_transition(target_patch_id, target_chain),
        reasons=reasons,
    )


def build_solver_graph(graph: PatchGraph) -> SolverGraph:
    solver_graph = SolverGraph()

    max_area = max((node.area for node in graph.nodes.values()), default=0.0)
    for patch_id, node in graph.nodes.items():
        solver_graph.patch_scores[patch_id] = _build_patch_certainty(node, max_area)

    solver_graph.max_shared_length = max((edge.shared_length for edge in graph.edges.values()), default=0.0)
    for seam in graph.edges.values():
        forward = _build_attachment_candidate(
            graph,
            seam.patch_a_id,
            seam.patch_b_id,
            seam,
            solver_graph.patch_scores,
            solver_graph.max_shared_length,
        )
        backward = _build_attachment_candidate(
            graph,
            seam.patch_b_id,
            seam.patch_a_id,
            seam,
            solver_graph.patch_scores,
            solver_graph.max_shared_length,
        )
        for candidate in (forward, backward):
            if candidate is None:
                continue
            solver_graph.candidates.append(candidate)
            solver_graph.candidates_by_owner.setdefault(candidate.owner_patch_id, []).append(candidate)

    for candidate_list in solver_graph.candidates_by_owner.values():
        candidate_list.sort(key=lambda candidate: candidate.score, reverse=True)
    solver_graph.candidates.sort(key=lambda candidate: (candidate.owner_patch_id, -candidate.score, candidate.target_patch_id))
    solver_graph.solve_components, solver_graph.component_by_patch = _build_solve_components(graph, solver_graph.candidates)
    return solver_graph


def choose_best_root(remaining_patch_ids: set[int], solver_graph: SolverGraph) -> Optional[int]:
    valid_patch_ids = [
        patch_id
        for patch_id in remaining_patch_ids
        if solver_graph.patch_scores.get(patch_id) and solver_graph.patch_scores[patch_id].local_solvable
    ]
    if not valid_patch_ids:
        return None
    return max(
        valid_patch_ids,
        key=lambda patch_id: (
            solver_graph.patch_scores[patch_id].root_score,
            solver_graph.patch_scores[patch_id].h_count + solver_graph.patch_scores[patch_id].v_count,
            -solver_graph.patch_scores[patch_id].free_count,
            patch_id,
        ),
    )


def plan_solve_phase1(
    graph: PatchGraph,
    solver_graph: Optional[SolverGraph] = None,
    propagate_threshold: float = EDGE_PROPAGATE_MIN,
    weak_threshold: float = EDGE_WEAK_MIN,
) -> SolvePlan:
    solver_graph = solver_graph or build_solver_graph(graph)
    remaining_patch_ids = set(graph.nodes.keys())
    quilts: list[QuiltPlan] = []
    skipped_patch_ids: list[int] = []
    queue_index = 0

    while remaining_patch_ids:
        root_patch_id = choose_best_root(remaining_patch_ids, solver_graph)
        if root_patch_id is None:
            skipped_patch_ids.extend(sorted(remaining_patch_ids))
            break

        root_score = solver_graph.patch_scores[root_patch_id].root_score
        component_index = solver_graph.component_by_patch.get(root_patch_id, -1)
        quilt = QuiltPlan(
            quilt_index=len(quilts),
            component_index=component_index,
            root_patch_id=root_patch_id,
            root_score=root_score,
        )

        solved_patch_ids: set[int] = set()
        frontier_heap = []

        def enqueue_patch_candidates(patch_id: int) -> None:
            nonlocal queue_index
            for candidate in solver_graph.candidates_by_owner.get(patch_id, []):
                if candidate.target_patch_id in solved_patch_ids:
                    continue
                if candidate.target_patch_id not in remaining_patch_ids:
                    continue
                heappush(frontier_heap, (-candidate.score, queue_index, candidate))
                queue_index += 1

        solved_patch_ids.add(root_patch_id)
        remaining_patch_ids.remove(root_patch_id)
        quilt.solved_patch_ids.append(root_patch_id)
        quilt.steps.append(QuiltStep(step_index=0, patch_id=root_patch_id, is_root=True))
        enqueue_patch_candidates(root_patch_id)

        while frontier_heap:
            best_score, _, candidate = frontier_heap[0]
            if -best_score < propagate_threshold:
                deferred = []
                rejected = []
                seen_keys = set()
                while frontier_heap:
                    _, _, pending = heappop(frontier_heap)
                    if pending.target_patch_id in solved_patch_ids:
                        continue
                    if pending.target_patch_id not in remaining_patch_ids:
                        continue
                    key = (pending.owner_patch_id, pending.target_patch_id)
                    if key in seen_keys:
                        continue
                    seen_keys.add(key)
                    if pending.score >= weak_threshold:
                        deferred.append(pending)
                    else:
                        rejected.append(pending)
                deferred.sort(key=lambda item: item.score, reverse=True)
                rejected.sort(key=lambda item: item.score, reverse=True)
                quilt.deferred_candidates = deferred
                quilt.rejected_candidates = rejected
                quilt.stop_reason = "frontier_below_threshold"
                break

            _, _, candidate = heappop(frontier_heap)
            if candidate.target_patch_id in solved_patch_ids:
                continue
            if candidate.target_patch_id not in remaining_patch_ids:
                continue

            solved_patch_ids.add(candidate.target_patch_id)
            remaining_patch_ids.remove(candidate.target_patch_id)
            quilt.solved_patch_ids.append(candidate.target_patch_id)
            quilt.steps.append(
                QuiltStep(
                    step_index=len(quilt.steps),
                    patch_id=candidate.target_patch_id,
                    is_root=False,
                    incoming_candidate=candidate,
                )
            )
            enqueue_patch_candidates(candidate.target_patch_id)

        if not quilt.stop_reason:
            quilt.stop_reason = "frontier_exhausted"
        quilts.append(quilt)

    return SolvePlan(
        quilts=quilts,
        skipped_patch_ids=skipped_patch_ids,
        propagate_threshold=propagate_threshold,
        weak_threshold=weak_threshold,
    )



def _choose_root_loop(node: PatchNode) -> int:
    best_loop_index = -1
    best_score = (-1, -1, -1)
    for loop_index, boundary_loop in enumerate(node.boundary_loops):
        frame_count = sum(1 for chain in boundary_loop.chains if chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME})
        corner_count = len(boundary_loop.corners)
        derived_bonus = 1 if frame_count <= 0 and corner_count >= 4 else 0
        if frame_count <= 0 and not derived_bonus:
            continue
        outer_bonus = 1 if boundary_loop.kind == LoopKind.OUTER else 0
        score = (outer_bonus, frame_count, derived_bonus)
        if score > best_score:
            best_score = score
            best_loop_index = loop_index
    return best_loop_index



def _patch_scaffold_is_supported(patch_placement: Optional[ScaffoldPatchPlacement]) -> bool:
    return patch_placement is not None and not patch_placement.notes and patch_placement.closure_valid


def _format_scaffold_uv_points(points: tuple[tuple[ScaffoldPointKey, Vector], ...]) -> str:
    if not points:
        return "[]"
    return '[' + ' '.join(f'({point.x:.4f},{point.y:.4f})' for _, point in points) + ']'


def _all_patch_ids(graph: PatchGraph) -> list[int]:
    return sorted(graph.nodes.keys())



def _collect_patch_face_indices(graph: PatchGraph, patch_ids: list[int]) -> set[int]:
    face_indices = set()
    for patch_id in patch_ids:
        node = graph.nodes.get(patch_id)
        if node is None:
            continue
        face_indices.update(node.face_indices)
    return face_indices



def _select_patch_faces(bm, graph: PatchGraph, patch_ids: list[int]) -> None:
    face_indices = _collect_patch_face_indices(graph, patch_ids)
    for face in bm.faces:
        face_selected = face.index in face_indices
        if hasattr(face, 'select_set'):
            face.select_set(face_selected)
        else:
            face.select = face_selected
    if hasattr(bm, 'select_flush_mode'):
        bm.select_flush_mode()



def _select_patch_uv_loops(bm, graph: PatchGraph, uv_layer, patch_ids: list[int]) -> None:
    face_indices = _collect_patch_face_indices(graph, patch_ids)
    for face in bm.faces:
        face_selected = face.index in face_indices
        for loop in face.loops:
            uv_data = loop[uv_layer]
            uv_data.select = face_selected
            uv_data.select_edge = face_selected



def _count_selected_patch_uv_loops(bm, graph: PatchGraph, uv_layer, patch_ids: list[int]) -> int:
    face_indices = _collect_patch_face_indices(graph, patch_ids)
    count = 0
    for face in bm.faces:
        if face.index not in face_indices:
            continue
        for loop in face.loops:
            if loop[uv_layer].select:
                count += 1
    return count



def _compute_patch_uv_bbox(bm, graph: PatchGraph, uv_layer, patch_ids: list[int]) -> tuple[Vector, Vector]:
    face_indices = _collect_patch_face_indices(graph, patch_ids)
    points = []
    for face in bm.faces:
        if face.index not in face_indices:
            continue
        for loop in face.loops:
            points.append(loop[uv_layer].uv.copy())
    if not points:
        return Vector((0.0, 0.0)), Vector((0.0, 0.0))
    min_x = min(point.x for point in points)
    min_y = min(point.y for point in points)
    max_x = max(point.x for point in points)
    max_y = max(point.y for point in points)
    return Vector((min_x, min_y)), Vector((max_x, max_y))



def _translate_patch_uvs(bm, graph: PatchGraph, uv_layer, patch_ids: list[int], offset: Vector) -> None:
    face_indices = _collect_patch_face_indices(graph, patch_ids)
    for face in bm.faces:
        if face.index not in face_indices:
            continue
        for loop in face.loops:
            loop[uv_layer].uv += offset



def _clear_patch_pins(bm, graph: PatchGraph, uv_layer, patch_ids: list[int]) -> None:
    face_indices = set()
    for patch_id in patch_ids:
        node = graph.nodes.get(patch_id)
        if node is None:
            continue
        face_indices.update(node.face_indices)

    for face_index in face_indices:
        face = bm.faces[face_index]
        for loop in face.loops:
            loop[uv_layer].pin_uv = False


def _pin_corner_vertices(bm, graph: PatchGraph, uv_layer, patch_ids: list[int]) -> int:
    """Пинит только corner vertices (стыки chains).
    Corners определяют прямоугольный каркас для LSCM без over-constraining
    boundary segments. Возвращает кол-во запинённых UV loops."""
    corner_vert_indices = set()
    for patch_id in patch_ids:
        node = graph.nodes.get(patch_id)
        if node is None:
            continue
        for bl in node.boundary_loops:
            for corner in bl.corners:
                if corner.vert_index >= 0:
                    corner_vert_indices.add(corner.vert_index)

    face_indices = set()
    for patch_id in patch_ids:
        node = graph.nodes.get(patch_id)
        if node is None:
            continue
        face_indices.update(node.face_indices)

    pinned = 0
    for face_index in face_indices:
        face = bm.faces[face_index]
        for loop in face.loops:
            if loop.vert.index in corner_vert_indices:
                loop[uv_layer].pin_uv = True
                pinned += 1
    return pinned



def _scaffold_key_id(point_key: ScaffoldPointKey, source_kind: str) -> tuple[int, int, int, int, str]:
    return (
        point_key.patch_id,
        point_key.loop_index,
        point_key.chain_index,
        point_key.source_point_index,
        source_kind,
    )



def _resolve_scaffold_uv_targets(bm, graph: PatchGraph, key: ScaffoldPointKey, source_kind: str) -> list[ScaffoldUvTarget]:
    node = graph.nodes.get(key.patch_id)
    if node is None or key.loop_index < 0 or key.loop_index >= len(node.boundary_loops):
        return []

    boundary_loop = node.boundary_loops[key.loop_index]
    loop_count = len(boundary_loop.vert_indices)
    if loop_count <= 0 or not boundary_loop.side_face_indices:
        return []

    chain = None
    if source_kind == 'chain':
        if key.chain_index < 0 or key.chain_index >= len(boundary_loop.chains):
            return []
        chain = boundary_loop.chains[key.chain_index]
        if key.source_point_index < 0 or key.source_point_index >= len(chain.vert_indices):
            return []
        loop_point_index = (chain.start_loop_index + key.source_point_index) % loop_count
        vert_index = chain.vert_indices[key.source_point_index]
        if loop_point_index >= len(boundary_loop.vert_indices) or boundary_loop.vert_indices[loop_point_index] != vert_index:
            return []
    else:
        if key.source_point_index < 0:
            return []
        loop_point_index = key.source_point_index % loop_count
        vert_index = boundary_loop.vert_indices[loop_point_index]

    targets = []
    seen = set()

    def _add_target(face_index: int) -> None:
        target_id = (face_index, vert_index)
        if face_index < 0 or target_id in seen:
            return
        if bm is not None:
            if face_index >= len(bm.faces):
                return
            face = bm.faces[face_index]
            if not any(loop.vert.index == vert_index for loop in face.loops):
                return
        seen.add(target_id)
        targets.append(
            ScaffoldUvTarget(
                face_index=face_index,
                vert_index=vert_index,
                loop_point_index=loop_point_index,
            )
        )

    face_candidates = []
    for side_index in ((loop_point_index - 1) % loop_count, loop_point_index):
        if side_index < 0 or side_index >= len(boundary_loop.side_face_indices):
            continue
        face_index = boundary_loop.side_face_indices[side_index]
        if face_index < 0 or face_index in face_candidates:
            continue
        face_candidates.append(face_index)

    for face_index in face_candidates:
        _add_target(face_index)

    if bm is None:
        return targets

    if boundary_loop.vert_indices.count(vert_index) != 1:
        return targets

    for face_index in node.face_indices:
        if face_index < 0 or face_index >= len(bm.faces):
            continue
        face = bm.faces[face_index]
        for loop in face.loops:
            if loop.vert.index != vert_index:
                continue
            _add_target(face_index)
            break
    return targets
def _count_patch_scaffold_points(patch_placement: ScaffoldPatchPlacement) -> int:
    scaffold_keys = set()
    for chain_placement in patch_placement.chain_placements:
        for point_key, _ in chain_placement.points:
            scaffold_keys.add(_scaffold_key_id(point_key, chain_placement.source_kind))
    return len(scaffold_keys)



def _should_pin_scaffold_point(chain_placement: ScaffoldChainPlacement, point_index: int, point_count: int, conformal_patch: bool = False, scaffold_connected: bool = True) -> bool:
    """Pin policy для scaffold points.

    conformal_patch=True (patch целиком ушёл в FREE):
      Ничего не пинить — Conformal unwrap обработает весь patch свободно.

    H_FRAME/V_FRAME: пинятся только если scaffold_connected=True
      (связаны с seed chain через непрерывную цепочку H/V chains).
      Изолированные H/V за FREE-разрывом не пинятся — они не часть
      основного scaffold каркаса и будут обработаны Conformal.

    FREE chains: endpoints + каждый 3-й intermediate point.
      Короткие FREE (≤4 points) — пинятся все.
      Это даёт Conformal достаточно constraints без полного lock-down.
    """
    if point_count <= 0:
        return False
    if conformal_patch:
        return False
    if chain_placement.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        return scaffold_connected
    # FREE chains: endpoints + every 3rd intermediate point
    if point_count <= 4:
        return True
    return point_index == 0 or point_index == (point_count - 1) or point_index % 3 == 0



def _apply_patch_scaffold_to_uv(bm, graph: PatchGraph, uv_layer, patch_placement: ScaffoldPatchPlacement, uv_offset: Vector) -> dict[str, object]:
    scaffold_point_count = _count_patch_scaffold_points(patch_placement)
    if patch_placement.notes:
        return {
            'status': 'unsupported',
            'scaffold_points': scaffold_point_count,
            'resolved_scaffold_points': 0,
            'uv_targets_resolved': 0,
            'unresolved_scaffold_points': 0,
            'missing_uv_targets': 0,
            'conflicting_uv_targets': 0,
            'pinned_uv_loops': 0,
            'invalid_scaffold_patches': 0,
            'closure_error': patch_placement.closure_error,
            'max_chain_gap': patch_placement.max_chain_gap,
            'closure_gap_count': len(patch_placement.gap_reports),
        }
    if not patch_placement.closure_valid:
        return {
            'status': 'invalid_scaffold',
            'scaffold_points': scaffold_point_count,
            'resolved_scaffold_points': 0,
            'uv_targets_resolved': 0,
            'unresolved_scaffold_points': 0,
            'missing_uv_targets': 0,
            'conflicting_uv_targets': 0,
            'pinned_uv_loops': 0,
            'invalid_scaffold_patches': 1,
            'closure_error': patch_placement.closure_error,
            'max_chain_gap': patch_placement.max_chain_gap,
            'closure_gap_count': len(patch_placement.gap_reports),
        }
    if graph.nodes.get(patch_placement.patch_id) is None:
        return {
            'status': 'missing_patch',
            'scaffold_points': 0,
            'resolved_scaffold_points': 0,
            'uv_targets_resolved': 0,
            'unresolved_scaffold_points': 0,
            'missing_uv_targets': 0,
            'conflicting_uv_targets': 0,
            'pinned_uv_loops': 0,
            'invalid_scaffold_patches': 0,
            'closure_error': patch_placement.closure_error,
            'max_chain_gap': patch_placement.max_chain_gap,
            'closure_gap_count': len(patch_placement.gap_reports),
        }

    # Patch с полностью FREE chains: не пинить, Conformal обработает
    node = graph.nodes.get(patch_placement.patch_id)
    conformal_patch = False
    if node is not None:
        conformal_patch = all(
            cp.frame_role == FrameRole.FREE for cp in patch_placement.chain_placements
        )

    target_samples: dict[tuple[int, int], list[Vector]] = {}
    pin_target_ids: set[tuple[int, int]] = set()
    scaffold_keys = set()
    unresolved_keys = set()

    scaffold_connected_set = patch_placement.scaffold_connected_chains

    for chain_placement in patch_placement.chain_placements:
        point_count = len(chain_placement.points)
        is_connected = chain_placement.chain_index in scaffold_connected_set
        for point_index, (point_key, target_uv) in enumerate(chain_placement.points):
            key_id = _scaffold_key_id(point_key, chain_placement.source_kind)
            scaffold_keys.add(key_id)
            targets = _resolve_scaffold_uv_targets(bm, graph, point_key, chain_placement.source_kind)
            if not targets:
                unresolved_keys.add(key_id)
                continue

            shifted_uv = target_uv + uv_offset
            should_pin = _should_pin_scaffold_point(chain_placement, point_index, point_count, conformal_patch, scaffold_connected=is_connected)
            for target in targets:
                target_id = (target.face_index, target.vert_index)
                target_samples.setdefault(target_id, []).append(shifted_uv.copy())
                if should_pin:
                    pin_target_ids.add(target_id)

    conflicting_uv_targets = 0
    missing_uv_targets = 0
    pinned_uv_loops = 0
    for (face_index, vert_index), samples in target_samples.items():
        base_sample = samples[0]
        if any((sample - base_sample).length > 1e-5 for sample in samples[1:]):
            conflicting_uv_targets += 1

        target_uv = sum(samples, Vector((0.0, 0.0))) / float(len(samples))
        if face_index < 0 or face_index >= len(bm.faces):
            missing_uv_targets += 1
            continue

        face = bm.faces[face_index]
        applied = False
        for loop in face.loops:
            if loop.vert.index != vert_index:
                continue
            loop[uv_layer].uv = target_uv.copy()
            loop[uv_layer].pin_uv = (face_index, vert_index) in pin_target_ids
            if loop[uv_layer].pin_uv:
                pinned_uv_loops += 1
            applied = True
            break
        if not applied:
            missing_uv_targets += 1

    return {
        'status': 'ok',
        'scaffold_points': len(scaffold_keys),
        'resolved_scaffold_points': len(scaffold_keys) - len(unresolved_keys),
        'uv_targets_resolved': len(target_samples),
        'unresolved_scaffold_points': len(unresolved_keys),
        'missing_uv_targets': missing_uv_targets,
        'conflicting_uv_targets': conflicting_uv_targets,
        'pinned_uv_loops': pinned_uv_loops,
        'invalid_scaffold_patches': 0,
        'closure_error': patch_placement.closure_error,
        'max_chain_gap': patch_placement.max_chain_gap,
        'closure_gap_count': len(patch_placement.gap_reports),
    }


CHAIN_FRONTIER_THRESHOLD = FRONTIER_MINIMUM_SCORE


@dataclass(frozen=True)
class ChainAnchor:
    uv: Vector
    source_ref: tuple[int, int, int]
    source_point_index: int
    source_kind: str = "same_patch"


def _cf_chain_source_points(chain):
    """Конвертирует chain.vert_cos в формат [(index, Vector)] для placement функций."""
    return [(i, co.copy()) for i, co in enumerate(chain.vert_cos)]


def _cf_anchor_count(start_anchor: Optional[ChainAnchor], end_anchor: Optional[ChainAnchor]) -> int:
    return (1 if start_anchor is not None else 0) + (1 if end_anchor is not None else 0)


def _cf_anchor_debug_label(start_anchor: Optional[ChainAnchor], end_anchor: Optional[ChainAnchor]) -> str:
    def _label(anchor: Optional[ChainAnchor]) -> str:
        if anchor is None:
            return '-'
        prefix = 'S' if anchor.source_kind == 'same_patch' else 'X'
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
    placed_chains_map: dict[tuple[int, int, int], ScaffoldChainPlacement],
    chain_role: FrameRole,
    anchor: ChainAnchor,
    snapped_uv: Vector,
) -> bool:
    if anchor.source_kind != 'same_patch':
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
    if anchor.source_kind != 'same_patch':
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
    placed_chains_map: dict[tuple[int, int, int], ScaffoldChainPlacement],
) -> tuple[Optional[ChainAnchor], Optional[ChainAnchor], str, tuple[tuple[tuple[int, int, int], int, Vector], ...]]:
    # Локальная per-chain rectification для H/V оказалась слишком слабой моделью:
    # она умеет выпрямлять отдельный chain, но не умеет согласованно решать
    # весь frame-граф patch/quilt. В результате появлялись ступеньки между
    # геометрически коллинеарными H/V chains. До появления patch-level
    # orthogonal solve runtime placement должен оставаться без этой
    # коррекции, чтобы не ломать stitch continuity и row consistency.
    return start_anchor, end_anchor, '', ()

    if chain.frame_role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        return start_anchor, end_anchor, '', ()
    if start_anchor is None or end_anchor is None:
        return start_anchor, end_anchor, '', ()
    if start_anchor.source_kind == 'cross_patch' and end_anchor.source_kind == 'cross_patch':
        return start_anchor, end_anchor, '', ()

    start_cross = _cf_frame_cross_axis_value(chain.frame_role, start_anchor.uv)
    end_cross = _cf_frame_cross_axis_value(chain.frame_role, end_anchor.uv)
    target_cross = None
    adjust_start = False
    adjust_end = False
    reason = ''
    start_protected = _cf_same_patch_anchor_is_protected(graph, start_anchor)
    end_protected = _cf_same_patch_anchor_is_protected(graph, end_anchor)

    if start_anchor.source_kind == 'same_patch' and end_anchor.source_kind == 'same_patch':
        if start_protected and end_protected:
            return start_anchor, end_anchor, '', ()
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
    elif start_anchor.source_kind == 'same_patch':
        if start_protected:
            return start_anchor, end_anchor, '', ()
        target_cross = end_cross
        adjust_start = True
        reason = 'axis_hard:lock_end'
    elif end_anchor.source_kind == 'same_patch':
        if end_protected:
            return start_anchor, end_anchor, '', ()
        target_cross = start_cross
        adjust_end = True
        reason = 'axis_hard:lock_start'
    else:
        return start_anchor, end_anchor, '', ()

    snapped_start_uv = _cf_with_frame_cross_axis(chain.frame_role, start_anchor.uv, target_cross)
    snapped_end_uv = _cf_with_frame_cross_axis(chain.frame_role, end_anchor.uv, target_cross)
    adjustments = []

    if adjust_start:
        if not _cf_preview_anchor_source_adjustment(graph, placed_chains_map, chain.frame_role, start_anchor, snapped_start_uv):
            return start_anchor, end_anchor, '', ()
        adjustments.append((start_anchor.source_ref, start_anchor.source_point_index, snapped_start_uv.copy()))
    if adjust_end:
        if not _cf_preview_anchor_source_adjustment(graph, placed_chains_map, chain.frame_role, end_anchor, snapped_end_uv):
            return start_anchor, end_anchor, '', ()
        adjustments.append((end_anchor.source_ref, end_anchor.source_point_index, snapped_end_uv.copy()))

    return (
        ChainAnchor(
            uv=snapped_start_uv,
            source_ref=start_anchor.source_ref,
            source_point_index=start_anchor.source_point_index,
            source_kind=start_anchor.source_kind,
        ),
        ChainAnchor(
            uv=snapped_end_uv,
            source_ref=end_anchor.source_ref,
            source_point_index=end_anchor.source_point_index,
            source_kind=end_anchor.source_kind,
        ),
        reason,
        tuple(adjustments),
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


def _chain_edge_lengths(ordered_source_points: list[tuple[int, Vector]], final_scale: float) -> list[float]:
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


def _segment_source_step_directions(node: PatchNode, ordered_source_points: list[tuple[int, Vector]]) -> list[Vector]:
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
    ordered_source_points: list[tuple[int, Vector]],
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
    ordered_source_points: list[tuple[int, Vector]],
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
    ordered_source_points: list[tuple[int, Vector]],
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
    ordered_source_points: list[tuple[int, Vector]],
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
    adjustments: tuple[tuple[tuple[int, int, int], int, Vector], ...],
    graph: PatchGraph,
    placed_chains_map: dict[tuple[int, int, int], ScaffoldChainPlacement],
    point_registry: dict[tuple[int, int, int, int], Vector],
    final_scale: float,
) -> bool:
    if not adjustments:
        return True

    grouped_targets: dict[tuple[int, int, int], dict[int, Vector]] = {}
    for chain_ref, source_point_index, target_uv in adjustments:
        grouped_targets.setdefault(chain_ref, {})[source_point_index] = target_uv.copy()

    staged_updates: dict[tuple[int, int, int], ScaffoldChainPlacement] = {}
    staged_registry: dict[tuple[int, int, int, int], Vector] = {}

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
            staged_registry[(chain_ref[0], chain_ref[1], chain_ref[2], index)] = uv.copy()

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


def _cf_choose_seed_chain(graph, root_node, quilt_patch_ids, allowed_tree_edges):
    """Выбирает strongest chain root patch для seed placement.

    Семантический bonus учитывает только patches того же quilt.
    Иначе caps/соседи из других quilts искажают seed для wall patch.

    Returns: (loop_index, chain_index, chain) или None
    """
    best_ref = None
    best_score = -1.0

    for loop_idx, loop in enumerate(root_node.boundary_loops):
        if loop.kind != LoopKind.OUTER:
            continue
        for chain_idx, chain in enumerate(loop.chains):
            score = 0.0

            if chain.frame_role in (FrameRole.H_FRAME, FrameRole.V_FRAME):
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

            if score > best_score:
                best_score = score
                best_ref = (loop_idx, chain_idx, chain)

    return best_ref


def _cf_find_anchors(
    chain_ref,
    chain,
    graph,
    point_registry,
    vert_to_placements,
    placed_refs,
    allowed_tree_edges,
):
    """Ищет anchor UV для start/end вершин chain с provenance.

    same_patch anchor берётся только через corner topology.
    cross_patch anchor разрешён только через shared seam vert tree-ребра quilt.
    """
    patch_id, loop_index, chain_index = chain_ref
    node = graph.nodes.get(patch_id)
    if node is None or loop_index >= len(node.boundary_loops):
        return None, None

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
                    key = (patch_id, loop_index, prev_ci, prev_last)
                    if key in point_registry:
                        start_anchor = ChainAnchor(
                            uv=point_registry[key].copy(),
                            source_ref=prev_ref,
                            source_point_index=prev_last,
                            source_kind='same_patch',
                        )

        if prev_ci == chain_index and end_anchor is None:
            next_ref = (patch_id, loop_index, next_ci)
            if next_ref in placed_refs:
                key = (patch_id, loop_index, next_ci, 0)
                if key in point_registry:
                    end_anchor = ChainAnchor(
                        uv=point_registry[key].copy(),
                        source_ref=next_ref,
                        source_point_index=0,
                        source_kind='same_patch',
                    )

    if start_anchor is None and start_vert >= 0 and start_vert in vert_to_placements:
        for other_ref, pt_idx in vert_to_placements[start_vert]:
            if other_ref[0] == patch_id:
                continue
            if seam_neighbor_patch_id is None or other_ref[0] != seam_neighbor_patch_id:
                continue
            if not _is_allowed_quilt_edge(allowed_tree_edges, patch_id, other_ref[0]):
                continue
            key = (other_ref[0], other_ref[1], other_ref[2], pt_idx)
            if key in point_registry:
                start_anchor = ChainAnchor(
                    uv=point_registry[key].copy(),
                    source_ref=other_ref,
                    source_point_index=pt_idx,
                    source_kind='cross_patch',
                )
                break

    if end_anchor is None and end_vert >= 0 and end_vert in vert_to_placements:
        for other_ref, pt_idx in vert_to_placements[end_vert]:
            if other_ref[0] == patch_id:
                continue
            if seam_neighbor_patch_id is None or other_ref[0] != seam_neighbor_patch_id:
                continue
            if not _is_allowed_quilt_edge(allowed_tree_edges, patch_id, other_ref[0]):
                continue
            key = (other_ref[0], other_ref[1], other_ref[2], pt_idx)
            if key in point_registry:
                end_anchor = ChainAnchor(
                    uv=point_registry[key].copy(),
                    source_ref=other_ref,
                    source_point_index=pt_idx,
                    source_kind='cross_patch',
                )
                break

    return start_anchor, end_anchor


def _cf_frame_anchor_pair_is_axis_safe(
    chain: BoundaryChain,
    start_anchor: ChainAnchor,
    end_anchor: ChainAnchor,
    final_scale: float,
) -> tuple[bool, str]:
    if chain.frame_role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        return True, ''

    total_length = _cf_chain_total_length(chain, final_scale)
    if total_length <= 1e-8:
        return True, ''

    delta = end_anchor.uv - start_anchor.uv
    axis_error = abs(delta.y) if chain.frame_role == FrameRole.H_FRAME else abs(delta.x)
    axis_span = abs(delta.x) if chain.frame_role == FrameRole.H_FRAME else abs(delta.y)

    axis_tolerance = max(0.02, total_length * 0.05)
    span_tolerance = max(0.05, total_length * 0.15)

    if axis_error > axis_tolerance:
        return False, 'axis_mismatch'
    if abs(axis_span - total_length) > span_tolerance:
        return False, 'span_mismatch'
    return True, ''


def _cf_can_use_dual_anchor_closure(
    chain: BoundaryChain,
    start_anchor: ChainAnchor,
    end_anchor: ChainAnchor,
    placed_in_patch: int,
    final_scale: float,
) -> tuple[bool, str]:
    axis_ok, axis_reason = _cf_frame_anchor_pair_is_axis_safe(chain, start_anchor, end_anchor, final_scale)
    if not axis_ok:
        return False, axis_reason

    # Не позволяем patch замкнуться только по двум сужим anchors.
    # Иначе второй seam-chain "прилипает" к обоим сторонам уже размещённого patch.
    if start_anchor.source_kind == 'cross_patch' and end_anchor.source_kind == 'cross_patch':
        # Короткие FREE chains (bridge, ≤2 вершины) — позиция полностью определена anchors,
        # wrap невозможен для одного ребра.
        if chain.frame_role == FrameRole.FREE and len(chain.vert_cos) <= 2:
            return True, ''
        return False, 'prevent_patch_wrap'

    return True, ''


def _cf_resolve_candidate_anchors(
    chain: BoundaryChain,
    start_anchor: Optional[ChainAnchor],
    end_anchor: Optional[ChainAnchor],
    placed_in_patch: int,
    final_scale: float,
    graph: PatchGraph,
    placed_chains_map: dict[tuple[int, int, int], ScaffoldChainPlacement],
) -> tuple[Optional[ChainAnchor], Optional[ChainAnchor], int, str, tuple[tuple[tuple[int, int, int], int, Vector], ...]]:
    known = _cf_anchor_count(start_anchor, end_anchor)
    if known < 2:
        return start_anchor, end_anchor, known, '', ()

    start_anchor, end_anchor, rect_reason, anchor_adjustments = _cf_preview_frame_dual_anchor_rectification(
        chain,
        start_anchor,
        end_anchor,
        graph,
        placed_chains_map,
    )

    can_close, reason = _cf_can_use_dual_anchor_closure(
        chain,
        start_anchor,
        end_anchor,
        placed_in_patch,
        final_scale,
    )
    if can_close:
        return start_anchor, end_anchor, 2, rect_reason, anchor_adjustments

    if (
        start_anchor is not None and end_anchor is not None
        and start_anchor.source_kind == 'cross_patch'
        and end_anchor.source_kind == 'cross_patch'
    ):
        reason_note = f'{rect_reason}|{reason}' if rect_reason else reason
        return start_anchor, None, 1, f'{reason_note}:bootstrap_from_start', ()

    if start_anchor is not None and start_anchor.source_kind == 'same_patch' and (
        end_anchor is None or end_anchor.source_kind != 'same_patch'
    ):
        reason_note = f'{rect_reason}|{reason}' if rect_reason else reason
        return start_anchor, None, 1, f'{reason_note}:drop_end', ()
    if end_anchor is not None and end_anchor.source_kind == 'same_patch' and (
        start_anchor is None or start_anchor.source_kind != 'same_patch'
    ):
        reason_note = f'{rect_reason}|{reason}' if rect_reason else reason
        return None, end_anchor, 1, f'{reason_note}:drop_start', ()

    if (
        start_anchor is not None and end_anchor is not None
        and start_anchor.source_kind == 'same_patch'
        and end_anchor.source_kind == 'same_patch'
        and chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
    ):
        reason_note = f'{rect_reason}|{reason}' if rect_reason else reason
        return start_anchor, None, 1, f'{reason_note}:axis_soft_from_start', ()

    return None, None, 0, reason, ()


def _cf_score_candidate(
    chain_ref,
    chain,
    node,
    known,
    graph,
    placed_in_patch,
    quilt_patch_ids,
    allowed_tree_edges,
    start_anchor: Optional[ChainAnchor] = None,
    end_anchor: Optional[ChainAnchor] = None,
):
    """Chain-level score для frontier candidate."""
    # Иерархия приоритетов:
    #   1. Cross-patch same-type H/V (seam скелет quilt):  base 2.0
    #   2. Cross-patch diff-type H/V:                       base 1.5
    #   3. Regular H/V (same patch boundary):               base 1.0
    #   4. Bridge FREE (1 edge, 2 verts):                   base 0.8
    #   5. Corner-split H/V (mesh border sub-chains):       base 0.4
    #   6. Regular FREE:                                    base 0.2
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
        score += 0.8
    else:
        score += 0.2

    # --- Anchor count bonus ---
    if known == 2:
        score += 0.8
    elif known == 1:
        score += 0.3

    # --- Momentum: patch уже имеет placed chains ---
    if placed_in_patch > 0:
        score += 0.2

    # --- Anchor source preference ---
    same_patch_anchor_count = sum(
        1 for anchor in (start_anchor, end_anchor)
        if anchor is not None and anchor.source_kind == 'same_patch'
    )
    cross_patch_anchor_count = sum(
        1 for anchor in (start_anchor, end_anchor)
        if anchor is not None and anchor.source_kind == 'cross_patch'
    )
    score += 0.1 * same_patch_anchor_count

    if same_patch_anchor_count == 0 and cross_patch_anchor_count > 0:
        score -= 0.35 if placed_in_patch > 0 else 0.25

    return score



def _try_inherit_direction(chain, node, start_anchor, end_anchor, graph, point_registry):
    """Наследует direction от соседнего same-role chain в том же patch.

    Если anchor-chain имеет тот же frame_role (H↔H, V↔V) в том же patch,
    берём его UV direction вместо вычисления из 3D. Это continuation —
    бевельный sub-chain продолжает движение основного chain.
    """
    if chain.frame_role not in (FrameRole.H_FRAME, FrameRole.V_FRAME):
        return None

    own_direction = _cf_determine_direction(chain, node)

    for anchor in (start_anchor, end_anchor):
        if anchor is None or anchor.source_kind != 'same_patch':
            continue
        src_chain = graph.get_chain(*anchor.source_ref)
        if src_chain is None or src_chain.frame_role != chain.frame_role:
            continue

        # Нашли same-role same-patch соседа — берём его UV direction
        src_ref = anchor.source_ref
        n_pts = len(src_chain.vert_cos)
        src_start_uv = point_registry.get((*src_ref, 0))
        src_end_uv = point_registry.get((*src_ref, n_pts - 1))
        if src_start_uv is None or src_end_uv is None:
            continue

        if chain.frame_role == FrameRole.H_FRAME:
            du = src_end_uv.x - src_start_uv.x
            if abs(du) > 1e-9:
                inherited = Vector((1.0 if du > 0 else -1.0, 0.0))
                if inherited.dot(own_direction) > 0.0:
                    return inherited
        else:  # V_FRAME
            dv = src_end_uv.y - src_start_uv.y
            if abs(dv) > 1e-9:
                inherited = Vector((0.0, 1.0 if dv > 0 else -1.0))
                if inherited.dot(own_direction) > 0.0:
                    return inherited

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


def _cf_register_points(chain_ref, chain, uv_points, point_registry, vert_to_placements):
    """Регистрирует все точки chain в обоих registry."""
    patch_id, loop_index, chain_index = chain_ref
    for i, uv in enumerate(uv_points):
        key = (patch_id, loop_index, chain_index, i)
        point_registry[key] = uv.copy()

        if i < len(chain.vert_indices):
            vert_idx = chain.vert_indices[i]
            vert_to_placements.setdefault(vert_idx, []).append((chain_ref, i))


def _compute_scaffold_connected_chains(patch_placements, total_chains, root_chain_index):
    """Найти H/V chains связанные с root через непрерывную цепочку H/V.

    Обход замкнутого loop: chains идут по порядку 0→1→...→N-1→0.
    Два соседних chain scaffold-connected если ОБА имеют H/V role.
    One-edge FREE chains (bridge, ≤2 вершин) прозрачны для BFS —
    пропускают связность к H/V chain за ними, но сами включаются
    в connected set (их endpoints = corners, уже запинены соседями).
    Длинные FREE chains разрывают scaffold connectivity.
    BFS от root_chain_index по H/V-adjacency.
    """
    hv_roles = {FrameRole.H_FRAME, FrameRole.V_FRAME}

    # Карта chain_index → (frame_role, point_count) для размещённых chains
    placed_info = {}
    for cp in patch_placements:
        placed_info[cp.chain_index] = (cp.frame_role, len(cp.points))

    # Root должен быть H/V, иначе нет scaffold вообще
    if root_chain_index not in placed_info or placed_info[root_chain_index][0] not in hv_roles:
        return frozenset()

    def _is_bridge(ci):
        """One-edge FREE chain (≤2 points) — прозрачный мост."""
        if ci not in placed_info:
            return False
        role, pc = placed_info[ci]
        return role == FrameRole.FREE and pc <= 2

    # BFS по H/V adjacency, one-edge FREE мосты прозрачны
    visited = {root_chain_index}
    queue = [root_chain_index]
    while queue:
        ci = queue.pop()
        for neighbor in [(ci - 1) % total_chains, (ci + 1) % total_chains]:
            if neighbor in visited:
                continue
            if neighbor not in placed_info:
                continue
            role, _ = placed_info[neighbor]
            if role in hv_roles:
                visited.add(neighbor)
                queue.append(neighbor)
            elif _is_bridge(neighbor):
                # Bridge прозрачен: включаем его и продолжаем BFS через него
                visited.add(neighbor)
                queue.append(neighbor)

    return frozenset(visited)


def _cf_build_envelopes(
    graph,
    quilt_patch_ids,
    placed_chains_map,
    placed_chain_refs,
    chain_dependency_patches,
    build_order=None,
):
    """Группирует размещённые chains в ScaffoldPatchPlacement per patch."""
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
                notes=('no_placed_chains',),
                status='EMPTY',
            )
            continue

        outer_loop_index = -1
        for loop_idx, loop in enumerate(node.boundary_loops):
            if loop.kind == LoopKind.OUTER:
                outer_loop_index = loop_idx
                break

        if outer_loop_index < 0:
            patches[patch_id] = ScaffoldPatchPlacement(
                patch_id=patch_id,
                loop_index=-1,
                notes=('no_outer_loop',),
                status='UNSUPPORTED',
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
            status = 'COMPLETE'
        elif placed_count > 0:
            status = 'PARTIAL'
        else:
            status = 'EMPTY'

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
        if status == 'COMPLETE' and total_chains >= 2:
            if patch_placements[-1].points and patch_placements[0].points:
                last_end = patch_placements[-1].points[-1][1]
                first_start = patch_placements[0].points[0][1]
                closure_error = (last_end - first_start).length
                closure_valid = closure_error < 0.05

        # Scaffold connectivity: найти root chain для этого patch из build_order,
        # вычислить связные H/V chains. Изолированные H/V за FREE не пинятся.
        root_ci = 0
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
            corner_positions=corner_positions,
            chain_placements=patch_placements,
            bbox_min=bbox_min,
            bbox_max=bbox_max,
            closure_error=closure_error,
            closure_valid=closure_valid,
            notes=(),
            status=status,
            dependency_patches=tuple(sorted(dep_set)),
            unplaced_chain_indices=unplaced,
            scaffold_connected_chains=scaffold_connected,
        )

    return patches


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
    allowed_tree_edges = _build_quilt_tree_edges(quilt_plan)
    closure_pair_map = _build_quilt_closure_pair_map(graph, quilt_patch_ids, allowed_tree_edges)
    ordered_quilt_patch_ids = list(quilt_plan.solved_patch_ids)
    if quilt_plan.root_patch_id not in ordered_quilt_patch_ids:
        ordered_quilt_patch_ids.append(quilt_plan.root_patch_id)

    root_node = graph.nodes.get(quilt_plan.root_patch_id)
    if root_node is None:
        return quilt_scaffold

    seed_result = _cf_choose_seed_chain(graph, root_node, quilt_patch_ids, allowed_tree_edges)
    if seed_result is None:
        quilt_scaffold.patches[quilt_plan.root_patch_id] = ScaffoldPatchPlacement(
            patch_id=quilt_plan.root_patch_id,
            loop_index=-1,
            notes=('no_seed_chain',),
            status='UNSUPPORTED',
        )
        return quilt_scaffold

    seed_loop_idx, seed_chain_idx, seed_chain = seed_result
    seed_ref = (quilt_plan.root_patch_id, seed_loop_idx, seed_chain_idx)

    point_registry = {}
    vert_to_placements = {}
    placed_chain_refs = set()
    placed_chains_map = {}
    chain_dependency_patches = {}
    rejected_chain_refs = set()
    build_order = []

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
        quilt_scaffold.patches[quilt_plan.root_patch_id] = ScaffoldPatchPlacement(
            patch_id=quilt_plan.root_patch_id,
            loop_index=-1,
            notes=('seed_placement_failed',),
            status='UNSUPPORTED',
        )
        return quilt_scaffold

    seed_placement = ScaffoldChainPlacement(
        patch_id=seed_ref[0],
        loop_index=seed_ref[1],
        chain_index=seed_ref[2],
        frame_role=seed_chain.frame_role,
        source_kind='chain',
        anchor_count=0,
        start_anchor_kind='',
        end_anchor_kind='',
        points=tuple(
            (ScaffoldPointKey(seed_ref[0], seed_ref[1], seed_ref[2], i), uv.copy())
            for i, uv in enumerate(seed_uvs)
        ),
    )

    placed_chain_refs.add(seed_ref)
    placed_chains_map[seed_ref] = seed_placement
    chain_dependency_patches[seed_ref] = ()
    build_order.append(seed_ref)
    _cf_register_points(seed_ref, seed_chain, seed_uvs, point_registry, vert_to_placements)

    print(
        f"[CFTUV][Frontier] Seed: P{seed_ref[0]} L{seed_ref[1]}C{seed_ref[2]} "
        f"{seed_chain.frame_role.value} "
        f"({seed_uvs[0].x:.4f},{seed_uvs[0].y:.4f})"
        f"->({seed_uvs[-1].x:.4f},{seed_uvs[-1].y:.4f})"
    )
    if allowed_tree_edges:
        edge_labels = [f"{edge[0]}-{edge[1]}" for edge in sorted(allowed_tree_edges)]
        print(f"[CFTUV][Frontier] Tree edges: {edge_labels}")

    all_chain_pool = []
    for patch_id in ordered_quilt_patch_ids:
        node = graph.nodes.get(patch_id)
        if node is None:
            continue
        for loop_idx, loop in enumerate(node.boundary_loops):
            if loop.kind != LoopKind.OUTER:
                continue
            for chain_idx, chain in enumerate(loop.chains):
                ref = (patch_id, loop_idx, chain_idx)
                if ref != seed_ref:
                    all_chain_pool.append((ref, chain, node))

    max_iter = len(all_chain_pool) + 10
    for iteration in range(1, max_iter + 1):
        best_ref = None
        best_score = -1.0
        best_data = None

        for ref, chain, node in all_chain_pool:
            if ref in placed_chain_refs or ref in rejected_chain_refs:
                continue

            raw_start_anchor, raw_end_anchor = _cf_find_anchors(
                ref,
                chain,
                graph,
                point_registry,
                vert_to_placements,
                placed_chain_refs,
                allowed_tree_edges,
            )

            placed_in_patch = sum(1 for placed_ref in placed_chain_refs if placed_ref[0] == ref[0])
            anchor_start, anchor_end, known, anchor_reason, anchor_adjustments = _cf_resolve_candidate_anchors(
                chain,
                raw_start_anchor,
                raw_end_anchor,
                placed_in_patch,
                final_scale,
                graph,
                placed_chains_map,
            )
            closure_dir_override = None
            anchor_start, anchor_end, closure_dir_override, closure_reason = _cf_apply_closure_preconstraint(
                ref,
                chain,
                node,
                raw_start_anchor,
                raw_end_anchor,
                anchor_start,
                anchor_end,
                known,
                graph,
                point_registry,
                placed_chains_map,
                closure_pair_map,
                final_scale,
            )
            if closure_reason:
                anchor_reason = f"{anchor_reason}|{closure_reason}" if anchor_reason else closure_reason
            if known == 0:
                continue

            score = _cf_score_candidate(
                ref,
                chain,
                node,
                known,
                graph,
                placed_in_patch,
                quilt_patch_ids,
                allowed_tree_edges,
                start_anchor=anchor_start,
                end_anchor=anchor_end,
            )

            if score > best_score:
                best_score = score
                best_ref = ref
                best_data = (
                    chain,
                    node,
                    anchor_start,
                    anchor_end,
                    anchor_reason,
                    anchor_adjustments,
                    closure_dir_override,
                )

        if best_ref is None or best_score < CHAIN_FRONTIER_THRESHOLD:
            # Диагностика: почему frontier остановился
            remaining = [ref for ref, _, _ in all_chain_pool if ref not in placed_chain_refs and ref not in rejected_chain_refs]
            no_anchor_count = 0
            low_score_count = 0
            patches_with_no_anchor = set()
            for ref, chain, node in all_chain_pool:
                if ref in placed_chain_refs or ref in rejected_chain_refs:
                    continue
                raw_sa, raw_ea = _cf_find_anchors(
                    ref,
                    chain,
                    graph,
                    point_registry,
                    vert_to_placements,
                    placed_chain_refs,
                    allowed_tree_edges,
                )
                pip = sum(1 for pr in placed_chain_refs if pr[0] == ref[0])
                sa, ea, k, _, _ = _cf_resolve_candidate_anchors(
                    chain,
                    raw_sa,
                    raw_ea,
                    pip,
                    final_scale,
                    graph,
                    placed_chains_map,
                )
                if k == 0:
                    no_anchor_count += 1
                    patches_with_no_anchor.add(ref[0])
                else:
                    low_score_count += 1
            patches_with_placed = set(pr[0] for pr in placed_chain_refs)
            patches_without_placed = set(ref[0] for ref, _, _ in all_chain_pool if ref not in placed_chain_refs) - patches_with_placed
            print(
                f"[CFTUV][Frontier] STOP: remaining={len(remaining)} "
                f"no_anchor={no_anchor_count} low_score={low_score_count} "
                f"rejected={len(rejected_chain_refs)}"
            )
            print(
                f"[CFTUV][Frontier] Patches: placed_in={len(patches_with_placed)} "
                f"untouched={len(patches_without_placed)} "
                f"no_anchor_patches={len(patches_with_no_anchor)}"
            )
            if patches_without_placed and len(patches_without_placed) <= 20:
                print(f"[CFTUV][Frontier] Untouched patches: {sorted(patches_without_placed)}")
            break

        chain, node, anchor_start, anchor_end, anchor_reason, anchor_adjustments, closure_dir_override = best_data
        if anchor_adjustments:
            applied = _cf_apply_anchor_adjustments(
                anchor_adjustments,
                graph,
                placed_chains_map,
                point_registry,
                final_scale,
            )
            if not applied:
                rejected_chain_refs.add(best_ref)
                print(
                    f"[CFTUV][Frontier] Reject {iteration}: "
                    f"P{best_ref[0]} L{best_ref[1]}C{best_ref[2]} "
                    f"{chain.frame_role.value} reason:anchor_rectify_failed"
                )
                continue
        dir_override = closure_dir_override
        if dir_override is None:
            dir_override = _try_inherit_direction(chain, node, anchor_start, anchor_end, graph, point_registry)
        uv_points = _cf_place_chain(chain, node, anchor_start, anchor_end, final_scale, dir_override)

        if not uv_points or len(uv_points) != len(chain.vert_cos):
            rejected_chain_refs.add(best_ref)
            print(
                f"[CFTUV][Frontier] Reject {iteration}: "
                f"P{best_ref[0]} L{best_ref[1]}C{best_ref[2]} "
                f"{chain.frame_role.value} reason:placement_failed"
            )
            continue

        ep = _cf_anchor_count(anchor_start, anchor_end)
        chain_placement = ScaffoldChainPlacement(
            patch_id=best_ref[0],
            loop_index=best_ref[1],
            chain_index=best_ref[2],
            frame_role=chain.frame_role,
            source_kind='chain',
            anchor_count=ep,
            start_anchor_kind=anchor_start.source_kind if anchor_start is not None else '',
            end_anchor_kind=anchor_end.source_kind if anchor_end is not None else '',
            points=tuple(
                (ScaffoldPointKey(best_ref[0], best_ref[1], best_ref[2], i), uv.copy())
                for i, uv in enumerate(uv_points)
            ),
        )

        placed_chain_refs.add(best_ref)
        placed_chains_map[best_ref] = chain_placement
        chain_dependency_patches[best_ref] = tuple(sorted({
            anchor.source_ref[0]
            for anchor in (anchor_start, anchor_end)
            if anchor is not None and anchor.source_ref[0] != best_ref[0]
        }))
        build_order.append(best_ref)
        _cf_register_points(best_ref, chain, uv_points, point_registry, vert_to_placements)

        anchor_label = _cf_anchor_debug_label(anchor_start, anchor_end)
        reason_suffix = f" note:{anchor_reason}" if anchor_reason else ''
        bridge_tag = ' [BRIDGE]' if chain.frame_role == FrameRole.FREE and len(chain.vert_cos) <= 2 else ''
        print(
            f"[CFTUV][Frontier] Step {iteration}: "
            f"P{best_ref[0]} L{best_ref[1]}C{best_ref[2]} "
            f"{chain.frame_role.value} score:{best_score:.2f} ep:{ep} a:{anchor_label}{reason_suffix}{bridge_tag}"
        )

    quilt_scaffold.patches = _cf_build_envelopes(
        graph,
        ordered_quilt_patch_ids,
        placed_chains_map,
        placed_chain_refs,
        chain_dependency_patches,
        build_order=[seed_ref] + list(build_order),
    )
    quilt_scaffold.build_order = list(build_order)
    quilt_scaffold.closure_seam_reports = _collect_quilt_closure_seam_reports(
        graph,
        quilt_plan,
        quilt_scaffold,
        placed_chains_map,
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

    total_placed = len(build_order)
    total_available = len(all_chain_pool) + 1
    print(
        f"[CFTUV][Frontier] Quilt {quilt_plan.quilt_index}: "
        f"placed {total_placed}/{total_available} chains"
    )

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


def format_root_scaffold_report(
    graph: PatchGraph,
    scaffold_map: ScaffoldMap,
    mesh_name: Optional[str] = None,
) -> tuple[list[str], str]:
    lines = []
    if mesh_name:
        lines.append(f"Mesh: {mesh_name}")

    total_patches = 0
    unsupported = 0
    invalid_closure = 0
    closure_seam_count = 0
    max_closure_span_mismatch = 0.0
    max_closure_axis_phase = 0.0
    frame_group_count = 0
    max_row_scatter = 0.0
    max_column_scatter = 0.0
    for quilt in scaffold_map.quilts:
        placed_patch_ids = []
        for step_patch_id in [quilt.root_patch_id] + [patch_id for patch_id in quilt.patches.keys() if patch_id != quilt.root_patch_id]:
            if step_patch_id not in quilt.patches:
                continue
            placed_patch_ids.append(step_patch_id)
        if not placed_patch_ids:
            placed_patch_ids = [quilt.root_patch_id]
        lines.append(f"Quilt {quilt.quilt_index}: root=Patch {quilt.root_patch_id} ({graph.get_patch_semantic_key(quilt.root_patch_id)}) | patches:{placed_patch_ids}")
        closure_reports = getattr(quilt, 'closure_seam_reports', ())
        frame_reports = getattr(quilt, 'frame_alignment_reports', ())
        if closure_reports:
            closure_seam_count += len(closure_reports)
            max_closure_span_mismatch = max(
                max_closure_span_mismatch,
                max((report.span_mismatch for report in closure_reports), default=0.0),
            )
            max_closure_axis_phase = max(
                max_closure_axis_phase,
                max((report.axis_phase_offset_max for report in closure_reports), default=0.0),
            )
            lines.append(
                f"  ClosureSeams: {len(closure_reports)} | "
                f"max_span_mismatch:{max((report.span_mismatch for report in closure_reports), default=0.0):.6f} | "
                f"max_axis_phase:{max((report.axis_phase_offset_max for report in closure_reports), default=0.0):.6f}"
            )
            for report in closure_reports:
                lines.append(
                    "    "
                    + f"P{report.owner_patch_id} L{report.owner_loop_index}C{report.owner_chain_index}"
                    + f"<->P{report.target_patch_id} L{report.target_loop_index}C{report.target_chain_index} "
                    + f"{report.frame_role.value} mode:{report.anchor_mode} "
                    + f"a:{report.owner_anchor_count}/{report.target_anchor_count} "
                    + f"span3d:{report.canonical_3d_span:.6f} "
                    + f"uv:{report.owner_uv_span:.6f}/{report.target_uv_span:.6f} "
                    + f"mismatch:{report.span_mismatch:.6f} "
                    + f"axis:{report.owner_axis_error:.6f}/{report.target_axis_error:.6f} "
                    + f"phase:{report.axis_phase_offset_max:.6f}/{report.axis_phase_offset_mean:.6f} "
                    + f"cross:{report.cross_axis_offset_max:.6f}/{report.cross_axis_offset_mean:.6f} "
                    + f"uvd:{report.shared_uv_delta_max:.6f}/{report.shared_uv_delta_mean:.6f} "
                    + f"path:{report.tree_patch_distance} free:{report.free_bridge_count}"
                )
        if frame_reports:
            frame_group_count += len(frame_reports)
            max_row_scatter = max(
                max_row_scatter,
                max((report.scatter_max for report in frame_reports if report.axis_kind == 'ROW'), default=0.0),
            )
            max_column_scatter = max(
                max_column_scatter,
                max((report.scatter_max for report in frame_reports if report.axis_kind == 'COLUMN'), default=0.0),
            )
            lines.append(
                f"  FrameGroups: {len(frame_reports)} | "
                f"max_row_scatter:{max((report.scatter_max for report in frame_reports if report.axis_kind == 'ROW'), default=0.0):.6f} | "
                f"max_column_scatter:{max((report.scatter_max for report in frame_reports if report.axis_kind == 'COLUMN'), default=0.0):.6f}"
            )
            for report in frame_reports:
                coord_label = (
                    f"z:{report.class_coord_a:.6f}"
                    if report.axis_kind == 'ROW'
                    else f"xy:({report.class_coord_a:.6f},{report.class_coord_b:.6f})"
                )
                refs_label = ','.join(
                    f"P{patch_id}L{loop_index}C{chain_index}"
                    for patch_id, loop_index, chain_index in report.member_refs
                )
                closure_tag = " closure:1" if report.closure_sensitive else " closure:0"
                lines.append(
                    "    "
                    + f"{report.axis_kind} {report.frame_role.value} {coord_label} "
                    + f"chains:{report.chain_count} target:{report.target_cross_uv:.6f} "
                    + f"scatter:{report.scatter_max:.6f}/{report.scatter_mean:.6f} "
                    + f"weight:{report.total_weight:.6f}{closure_tag} refs:[{refs_label}]"
                )
        for patch_id in placed_patch_ids:
            patch_placement = quilt.patches.get(patch_id)
            total_patches += 1
            signature = graph.get_patch_semantic_key(patch_id)
            if patch_placement is None:
                unsupported += 1
                lines.append(f"  Patch {patch_id} ({signature}) | scaffold:missing")
                continue
            if patch_placement.notes:
                unsupported += 1
                lines.append(f"  Patch {patch_id} ({signature}) | scaffold:unsupported | notes:{', '.join(patch_placement.notes)}")
                continue

            scaffold_status = 'ok' if patch_placement.closure_valid else 'invalid_closure'
            if not patch_placement.closure_valid:
                invalid_closure += 1
            lines.append(
                f"  Patch {patch_id} ({signature}) | loop:{patch_placement.loop_index} | start_chain:{patch_placement.root_chain_index} | "
                f"bbox:({patch_placement.bbox_min.x:.4f}, {patch_placement.bbox_min.y:.4f}) -> ({patch_placement.bbox_max.x:.4f}, {patch_placement.bbox_max.y:.4f}) | "
                f"closure:{patch_placement.closure_error:.6f} | max_gap:{patch_placement.max_chain_gap:.6f} | status:{scaffold_status}"
            )
            if patch_placement.gap_reports:
                for chain_index, next_chain_index, gap in patch_placement.gap_reports:
                    lines.append(f"    Gap {chain_index}->{next_chain_index}: {gap:.6f}")
            node = graph.nodes.get(patch_id)
            if node is None or patch_placement.loop_index < 0 or patch_placement.loop_index >= len(node.boundary_loops):
                continue
            boundary_loop = node.boundary_loops[patch_placement.loop_index]
            for corner_index in sorted(patch_placement.corner_positions.keys()):
                point = patch_placement.corner_positions[corner_index]
                turn_angle = 0.0
                prev_chain = '-'
                next_chain = '-'
                if 0 <= corner_index < len(boundary_loop.corners):
                    corner = boundary_loop.corners[corner_index]
                    turn_angle = corner.turn_angle_deg
                    prev_chain = corner.prev_chain_index
                    next_chain = corner.next_chain_index
                lines.append(
                    f"    Corner {corner_index}: ({point.x:.4f}, {point.y:.4f}) | chains:{prev_chain}->{next_chain} | turn:{turn_angle:.1f}"
                )
            sc_set = patch_placement.scaffold_connected_chains
            for chain_placement in patch_placement.chain_placements:
                if not chain_placement.points:
                    continue
                start_point = chain_placement.points[0][1]
                end_point = chain_placement.points[-1][1]
                isolated_tag = ""
                if chain_placement.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME} and chain_placement.chain_index not in sc_set:
                    isolated_tag = " [ISOLATED]"
                lines.append(
                    f"    {chain_placement.source_kind.title()} {chain_placement.chain_index}: {chain_placement.frame_role.value} | "
                    f"points:{len(chain_placement.points)} | start:({start_point.x:.4f}, {start_point.y:.4f}) | "
                    f"end:({end_point.x:.4f}, {end_point.y:.4f}) | uv:{_format_scaffold_uv_points(chain_placement.points)}{isolated_tag}"
                )

    summary = (
        f"Scaffold quilts: {len(scaffold_map.quilts)} | Patches: {total_patches} | Unsupported: {unsupported} | "
        f"Invalid closure: {invalid_closure} | Closure seams: {closure_seam_count} | "
        f"Max seam mismatch: {max_closure_span_mismatch:.6f} | "
        f"Max seam phase: {max_closure_axis_phase:.6f} | "
        f"Frame groups: {frame_group_count} | "
        f"Max row scatter: {max_row_scatter:.6f} | "
        f"Max column scatter: {max_column_scatter:.6f}"
    )
    return lines, summary


def _compute_quilt_bbox(quilt_scaffold: ScaffoldQuiltPlacement) -> tuple[Vector, Vector]:
    placements = [patch for patch in quilt_scaffold.patches.values() if _patch_scaffold_is_supported(patch)]
    if not placements:
        return Vector((0.0, 0.0)), Vector((0.0, 0.0))
    min_x = min(patch.bbox_min.x for patch in placements)
    min_y = min(patch.bbox_min.y for patch in placements)
    max_x = max(patch.bbox_max.x for patch in placements)
    max_y = max(patch.bbox_max.y for patch in placements)
    return Vector((min_x, min_y)), Vector((max_x, max_y))


def _ordered_quilt_patch_ids(quilt_scaffold: ScaffoldQuiltPlacement, quilt_plan: Optional[QuiltPlan]) -> list[int]:
    ordered_patch_ids = []
    seen = set()

    if quilt_plan is not None:
        for step in quilt_plan.steps:
            patch_placement = quilt_scaffold.patches.get(step.patch_id)
            if patch_placement is None or patch_placement.notes:
                continue
            ordered_patch_ids.append(step.patch_id)
            seen.add(step.patch_id)

    root_patch_placement = quilt_scaffold.patches.get(quilt_scaffold.root_patch_id)
    if root_patch_placement is not None and not root_patch_placement.notes and quilt_scaffold.root_patch_id not in seen:
        ordered_patch_ids.append(quilt_scaffold.root_patch_id)
        seen.add(quilt_scaffold.root_patch_id)

    for patch_id, patch_placement in quilt_scaffold.patches.items():
        if patch_id in seen or patch_placement.notes:
            continue
        ordered_patch_ids.append(patch_id)
        seen.add(patch_id)

    return ordered_patch_ids


def _print_phase1_preview_patch_report(quilt_index: int, patch_id: int, stats: dict[str, object]) -> None:
    status = str(stats.get('status', 'ok'))
    status_suffix = '' if status == 'ok' else f" status={status}"
    closure_suffix = ''
    if status != 'ok' or int(stats.get('closure_gap_count', 0)) > 0:
        closure_suffix = (
            f" closure={float(stats.get('closure_error', 0.0)):.6f}"
            f" max_gap={float(stats.get('max_chain_gap', 0.0)):.6f}"
            f" gaps={int(stats.get('closure_gap_count', 0))}"
        )
    print(
        f"[CFTUV][Phase1] Quilt {quilt_index} Patch {patch_id}: "
        f"scaffold={stats.get('scaffold_points', 0)} resolved={stats.get('resolved_scaffold_points', 0)} "
        f"uv_targets={stats.get('uv_targets_resolved', 0)} unresolved={stats.get('unresolved_scaffold_points', 0)} "
        f"missing={stats.get('missing_uv_targets', 0)} conflicts={stats.get('conflicting_uv_targets', 0)} "
        f"pinned={stats.get('pinned_uv_loops', 0)}{status_suffix}{closure_suffix}"
    )


def _print_phase1_preview_quilt_report(quilt_index: int, patch_ids: list[int], stats: dict[str, int]) -> None:
    print(
        f"[CFTUV][Phase1] Quilt {quilt_index}: patches={patch_ids} "
        f"scaffold={stats.get('scaffold_points', 0)} resolved={stats.get('resolved_scaffold_points', 0)} "
        f"uv_targets={stats.get('uv_targets_resolved', 0)} unresolved={stats.get('unresolved_scaffold_points', 0)} "
        f"missing={stats.get('missing_uv_targets', 0)} conflicts={stats.get('conflicting_uv_targets', 0)} "
        f"pinned={stats.get('pinned_uv_loops', 0)} invalid={stats.get('invalid_scaffold_patches', 0)}"
    )


def _collect_phase1_unsupported_patch_ids(scaffold_map: ScaffoldMap) -> list[int]:
    unsupported_patch_ids = set()
    for quilt_scaffold in scaffold_map.quilts:
        for patch_id, patch_placement in quilt_scaffold.patches.items():
            if not _patch_scaffold_is_supported(patch_placement):
                unsupported_patch_ids.add(patch_id)
    return sorted(unsupported_patch_ids)



def _execute_phase1_preview_impl(
    context,
    obj,
    bm,
    patch_graph: PatchGraph,
    settings,
    solve_plan: Optional[SolvePlan] = None,
    run_conformal: bool = True,
    keep_pins: bool = False,
) -> dict[str, object]:
    import bmesh
    import bpy

    patch_ids = _all_patch_ids(patch_graph)
    if not patch_ids:
        return {
            'patches': 0,
            'supported_roots': 0,
            'scaffold_points': 0,
            'resolved_scaffold_points': 0,
            'uv_targets_resolved': 0,
            'unresolved_scaffold_points': 0,
            'missing_uv_targets': 0,
            'conflicting_uv_targets': 0,
            'pinned_uv_loops': 0,
            'quilts': 0,
            'attached_children': 0,
            'invalid_scaffold_patches': 0,
            'unsupported_patch_count': 0,
            'unsupported_patch_ids': [],
            'conformal_applied': 0,
            'frame_group_count': 0,
            'frame_row_max_scatter': 0.0,
            'frame_column_max_scatter': 0.0,
        }

    bm.faces.ensure_lookup_table()
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    uv_layer = bm.loops.layers.uv.verify()
    _clear_patch_pins(bm, patch_graph, uv_layer, patch_ids)

    scaffold_map = build_root_scaffold_map(patch_graph, solve_plan, settings.final_scale)
    unsupported_patch_ids = _collect_phase1_unsupported_patch_ids(scaffold_map)
    if unsupported_patch_ids:
        print(f"[CFTUV][Phase1] Unsupported patches: {unsupported_patch_ids}")
    quilt_plan_by_index = {quilt.quilt_index: quilt for quilt in solve_plan.quilts} if solve_plan is not None else {}

    supported_roots = 0
    scaffold_points = 0
    resolved_scaffold_points = 0
    uv_targets_resolved = 0
    unresolved_scaffold_points = 0
    missing_uv_targets = 0
    conflicting_uv_targets = 0
    pinned_uv_loops = 0
    attached_children = 0
    invalid_scaffold_patches = 0
    global_supported_patch_ids: set[int] = set()
    conformal_applied = 0
    all_conformal_patch_ids: list[int] = []
    closure_seam_count = 0
    closure_seam_max_mismatch = 0.0
    closure_seam_max_phase = 0.0
    frame_group_count = 0
    frame_row_max_scatter = 0.0
    frame_column_max_scatter = 0.0

    for quilt_scaffold in scaffold_map.quilts:
        quilt_plan = quilt_plan_by_index.get(quilt_scaffold.quilt_index)
        quilt_patch_ids = _ordered_quilt_patch_ids(quilt_scaffold, quilt_plan)
        if not quilt_patch_ids:
            continue
        closure_reports = getattr(quilt_scaffold, 'closure_seam_reports', ())
        closure_seam_count += len(closure_reports)
        closure_seam_max_mismatch = max(
            closure_seam_max_mismatch,
            max((report.span_mismatch for report in closure_reports), default=0.0),
        )
        closure_seam_max_phase = max(
            closure_seam_max_phase,
            max((report.axis_phase_offset_max for report in closure_reports), default=0.0),
        )
        frame_reports = getattr(quilt_scaffold, 'frame_alignment_reports', ())
        frame_group_count += len(frame_reports)
        frame_row_max_scatter = max(
            frame_row_max_scatter,
            max((report.scatter_max for report in frame_reports if report.axis_kind == 'ROW'), default=0.0),
        )
        frame_column_max_scatter = max(
            frame_column_max_scatter,
            max((report.scatter_max for report in frame_reports if report.axis_kind == 'COLUMN'), default=0.0),
        )
        quilt_apply_patch_ids = [
            patch_id
            for patch_id in quilt_patch_ids
            if _patch_scaffold_is_supported(quilt_scaffold.patches.get(patch_id))
        ]

        bm = bmesh.from_edit_mesh(obj.data)
        bm.faces.ensure_lookup_table()
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        uv_layer = bm.loops.layers.uv.verify()
        _select_patch_faces(bm, patch_graph, quilt_apply_patch_ids)
        _clear_patch_pins(bm, patch_graph, uv_layer, quilt_apply_patch_ids)

        quilt_bbox_min, quilt_bbox_max = _compute_quilt_bbox(quilt_scaffold)
        # Каждый quilt стартует в (0,0) UV — без горизонтального стекания
        uv_offset = Vector((-quilt_bbox_min.x, -quilt_bbox_min.y))
        quilt_stats = {
            'scaffold_points': 0,
            'resolved_scaffold_points': 0,
            'uv_targets_resolved': 0,
            'unresolved_scaffold_points': 0,
            'missing_uv_targets': 0,
            'conflicting_uv_targets': 0,
            'pinned_uv_loops': 0,
            'invalid_scaffold_patches': 0,
        }

        for patch_id in quilt_patch_ids:
            patch_placement = quilt_scaffold.patches[patch_id]
            patch_stats = _apply_patch_scaffold_to_uv(
                bm,
                patch_graph,
                uv_layer,
                patch_placement,
                uv_offset,
            )
            validate_scaffold_uv_transfer(bm, patch_graph, uv_layer, patch_placement, uv_offset)
            _print_phase1_preview_patch_report(quilt_scaffold.quilt_index, patch_id, patch_stats)

            for stat_key in quilt_stats:
                quilt_stats[stat_key] += int(patch_stats.get(stat_key, 0))

            if int(patch_stats.get('resolved_scaffold_points', 0)) > 0:
                all_conformal_patch_ids.append(patch_id)

            if int(patch_stats.get('pinned_uv_loops', 0)) <= 0:
                continue
            if patch_id == quilt_scaffold.root_patch_id:
                supported_roots += 1
            else:
                attached_children += 1

        _print_phase1_preview_quilt_report(quilt_scaffold.quilt_index, quilt_patch_ids, quilt_stats)

        scaffold_points += quilt_stats['scaffold_points']
        resolved_scaffold_points += quilt_stats['resolved_scaffold_points']
        uv_targets_resolved += quilt_stats['uv_targets_resolved']
        unresolved_scaffold_points += quilt_stats['unresolved_scaffold_points']
        missing_uv_targets += quilt_stats['missing_uv_targets']
        conflicting_uv_targets += quilt_stats['conflicting_uv_targets']
        pinned_uv_loops += quilt_stats['pinned_uv_loops']
        invalid_scaffold_patches += quilt_stats['invalid_scaffold_patches']
        global_supported_patch_ids.update(quilt_apply_patch_ids)


    # Финальный Conformal: один вызов ПОСЛЕ всех quilts.
    # bmesh.update_edit_mesh вызывается один раз, потом bpy.ops.uv.unwrap
    # без промежуточных bmesh операций — исключает перезапись результатов.
    if run_conformal and all_conformal_patch_ids:
        bmesh.update_edit_mesh(obj.data)
        bm = bmesh.from_edit_mesh(obj.data)
        bm.faces.ensure_lookup_table()
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        uv_layer = bm.loops.layers.uv.verify()
        _select_patch_faces(bm, patch_graph, all_conformal_patch_ids)
        _select_patch_uv_loops(bm, patch_graph, uv_layer, all_conformal_patch_ids)
        sel_faces = sum(1 for f in bm.faces if f.select)
        pinned_count = sum(
            1 for f in bm.faces if f.select
            for lp in f.loops if lp[uv_layer].pin_uv
        )
        unpinned_count = sum(
            1 for f in bm.faces if f.select
            for lp in f.loops if not lp[uv_layer].pin_uv
        )
        # Снимок UV до Conformal: vert_index → uv для первых unpinned loops
        _pre_uv = {}
        for f in bm.faces:
            if f.select:
                for lp in f.loops:
                    if not lp[uv_layer].pin_uv:
                        key = (f.index, lp.vert.index)
                        _pre_uv[key] = lp[uv_layer].uv.copy()
        bmesh.update_edit_mesh(obj.data)
        print(
            f"[CFTUV][Phase1] Final Conformal: "
            f"patches={all_conformal_patch_ids} faces={sel_faces} "
            f"pinned={pinned_count} unpinned={unpinned_count}"
        )
        print(f"[CFTUV][Phase1] obj.mode={obj.mode} active={bpy.context.active_object == obj}")
        bpy.ops.uv.unwrap(method='CONFORMAL', fill_holes=False, margin=0.0)
        conformal_applied += 1
        # Проверка: сколько UV изменилось
        bm2 = bmesh.from_edit_mesh(obj.data)
        bm2.faces.ensure_lookup_table()
        uv2 = bm2.loops.layers.uv.verify()
        _changed = 0
        _checked = 0
        for f in bm2.faces:
            if f.select:
                for lp in f.loops:
                    if not lp[uv2].pin_uv:
                        key = (f.index, lp.vert.index)
                        pre = _pre_uv.get(key)
                        if pre is not None:
                            _checked += 1
                            if (lp[uv2].uv - pre).length > 1e-6:
                                _changed += 1
        print(f"[CFTUV][Phase1] Conformal result: checked={_checked} changed={_changed}/{len(_pre_uv)}")

    if run_conformal and unsupported_patch_ids:
        for patch_id in unsupported_patch_ids:
            bm = bmesh.from_edit_mesh(obj.data)
            bm.faces.ensure_lookup_table()
            bm.verts.ensure_lookup_table()
            bm.edges.ensure_lookup_table()
            uv_layer = bm.loops.layers.uv.verify()
            _select_patch_faces(bm, patch_graph, [patch_id])
            _select_patch_uv_loops(bm, patch_graph, uv_layer, [patch_id])
            _clear_patch_pins(bm, patch_graph, uv_layer, [patch_id])
            selected_face_count = len(_collect_patch_face_indices(patch_graph, [patch_id]))
            selected_uv_count = _count_selected_patch_uv_loops(bm, patch_graph, uv_layer, [patch_id])
            bmesh.update_edit_mesh(obj.data)
            print(
                f"[CFTUV][Phase1] Unsupported Patch {patch_id} Fallback Conformal: "
                f"faces={selected_face_count} uv_loops={selected_uv_count}"
            )
            bpy.ops.uv.unwrap(method='CONFORMAL', fill_holes=False, margin=0.0)
            conformal_applied += 1

            bm = bmesh.from_edit_mesh(obj.data)
            bm.faces.ensure_lookup_table()
            bm.verts.ensure_lookup_table()
            bm.edges.ensure_lookup_table()
            uv_layer = bm.loops.layers.uv.verify()
            bbox_min, bbox_max = _compute_patch_uv_bbox(bm, patch_graph, uv_layer, [patch_id])
            uv_offset = Vector((-bbox_min.x, -bbox_min.y))
            _translate_patch_uvs(bm, patch_graph, uv_layer, [patch_id], uv_offset)
            bmesh.update_edit_mesh(obj.data)
    if not run_conformal:
        print(f"[CFTUV][Phase1] Transfer Only: quilts={len(scaffold_map.quilts)} patches={sorted(global_supported_patch_ids)}")

    # DEBUG: пины не снимаются — для проверки pinned state после Phase1
    # if not keep_pins:
    #     bm = bmesh.from_edit_mesh(obj.data)
    #     bm.faces.ensure_lookup_table()
    #     bm.verts.ensure_lookup_table()
    #     bm.edges.ensure_lookup_table()
    #     uv_layer = bm.loops.layers.uv.verify()
    #     _clear_patch_pins(bm, patch_graph, uv_layer, patch_ids)
    #     bmesh.update_edit_mesh(obj.data)

    return {
        'patches': len(patch_ids),
        'supported_roots': supported_roots,
        'scaffold_points': scaffold_points,
        'resolved_scaffold_points': resolved_scaffold_points,
        'uv_targets_resolved': uv_targets_resolved,
        'unresolved_scaffold_points': unresolved_scaffold_points,
        'missing_uv_targets': missing_uv_targets,
        'conflicting_uv_targets': conflicting_uv_targets,
        'pinned_uv_loops': pinned_uv_loops,
        'quilts': len(scaffold_map.quilts),
        'attached_children': attached_children,
        'invalid_scaffold_patches': invalid_scaffold_patches,
        'unsupported_patch_count': len(unsupported_patch_ids),
        'unsupported_patch_ids': unsupported_patch_ids,
        'conformal_applied': conformal_applied,
        'closure_seam_count': closure_seam_count,
        'closure_seam_max_mismatch': closure_seam_max_mismatch,
        'closure_seam_max_phase': closure_seam_max_phase,
        'frame_group_count': frame_group_count,
        'frame_row_max_scatter': frame_row_max_scatter,
        'frame_column_max_scatter': frame_column_max_scatter,
    }

# ============================================================
# PHASE 2 PATCH — Validation Layer for solve.py
#
# ИНСТРУМЕНТАРИЙ
#
# Шаг 1: Ожидаемый UV
#         Найти vert_index в BMFace loops,
#         Установить UV
#         до конца chain.
#         Между ожидаемым UV и фактическим UV loop.
#
# Шаг 2: Вызвать
#             patch_stats = _apply_patch_scaffold_to_uv(bm, patch_graph, uv_layer, patch_placement, uv_offset)
#         Использовать
#             validate_scaffold_uv_transfer(bm, patch_graph, uv_layer, patch_placement, uv_offset)
#
# Готово.
# ============================================================


def validate_scaffold_uv_transfer(bm, graph, uv_layer, patch_placement, uv_offset, epsilon=1e-4):
    """Phase 2: Diagnostic — проверка scaffold→UV transfer.

    Вызывается после transfer,
    проверяет совпадение UV с ожидаемыми значениями scaffold.

    Каждый scaffold point проверяется на соответствие UV loop.
    Несовпадения собираются в mismatches.
    Также проверяется SEAM_SELF collapsed UV.
    """
    if patch_placement.notes or not patch_placement.closure_valid:
        return

    mismatches = []
    total_points = 0
    verified_ok = 0

    # --- Проверка: scaffold points записаны в правильные UV loops ---
    for chain_placement in patch_placement.chain_placements:
        for point_key, intended_uv in chain_placement.points:
            total_points += 1
            targets = _resolve_scaffold_uv_targets(bm, graph, point_key, chain_placement.source_kind)
            shifted_uv = intended_uv + uv_offset

            if not targets:
                mismatches.append(
                    f"  UNRESOLVED patch:{point_key.patch_id} "
                    f"L{point_key.loop_index}C{point_key.chain_index} "
                    f"pt:{point_key.source_point_index}"
                )
                continue

            point_ok = True
            for target in targets:
                if target.face_index < 0 or target.face_index >= len(bm.faces):
                    mismatches.append(
                        f"  MISSING_FACE patch:{point_key.patch_id} "
                        f"face:{target.face_index}"
                    )
                    point_ok = False
                    continue

                face = bm.faces[target.face_index]
                found_loop = False
                for loop in face.loops:
                    if loop.vert.index != target.vert_index:
                        continue
                    found_loop = True
                    actual_uv = loop[uv_layer].uv.copy()
                    dist = (actual_uv - shifted_uv).length
                    if dist > epsilon:
                        mismatches.append(
                            f"  MISMATCH patch:{point_key.patch_id} "
                            f"L{point_key.loop_index}C{point_key.chain_index} "
                            f"pt:{point_key.source_point_index} "
                            f"vert:{target.vert_index} face:{target.face_index} "
                            f"expected:({shifted_uv.x:.4f},{shifted_uv.y:.4f}) "
                            f"actual:({actual_uv.x:.4f},{actual_uv.y:.4f}) "
                            f"dist:{dist:.6f}"
                        )
                        point_ok = False
                    break

                if not found_loop:
                    mismatches.append(
                        f"  VERT_NOT_IN_FACE patch:{point_key.patch_id} "
                        f"vert:{target.vert_index} face:{target.face_index}"
                    )
                    point_ok = False

            if point_ok:
                verified_ok += 1

    # --- Проверка: SEAM_SELF вершины имеют разные UV на разных сторонах ---
    node = graph.nodes.get(patch_placement.patch_id)
    seam_self_collapsed = 0
    if node is not None and patch_placement.loop_index >= 0:
        if patch_placement.loop_index < len(node.boundary_loops):
            boundary_loop = node.boundary_loops[patch_placement.loop_index]
            for chain in boundary_loop.chains:
                if chain.neighbor_patch_id != -2:  # NB_SEAM_SELF
                    continue
                # Проверяем что UV не схлопнулись на SEAM_SELF
                for vert_index in chain.vert_indices:
                    uv_values = []
                    for face_index in node.face_indices:
                        if face_index >= len(bm.faces):
                            continue
                        face = bm.faces[face_index]
                        for loop in face.loops:
                            if loop.vert.index == vert_index:
                                uv_values.append(loop[uv_layer].uv.copy())
                    # SEAM_SELF vert должен иметь >= 2 разных UV
                    if len(uv_values) >= 2:
                        all_same = all(
                            (uv - uv_values[0]).length < epsilon
                            for uv in uv_values[1:]
                        )
                        if all_same:
                            seam_self_collapsed += 1

    # --- Console output ---
    if mismatches or seam_self_collapsed > 0:
        print(
            f"[CFTUV][Validate] Patch {patch_placement.patch_id}: "
            f"{len(mismatches)} mismatches, "
            f"{seam_self_collapsed} collapsed SEAM_SELF verts "
            f"({verified_ok}/{total_points} OK)"
        )
        for m in mismatches:
            print(m)
    else:
        print(
            f"[CFTUV][Validate] Patch {patch_placement.patch_id}: "
            f"OK ({verified_ok}/{total_points} points verified)"
        )


def execute_phase1_preview(context, obj, bm, patch_graph: PatchGraph, settings, solve_plan: Optional[SolvePlan] = None) -> dict[str, object]:
    return _execute_phase1_preview_impl(
        context,
        obj,
        bm,
        patch_graph,
        settings,
        solve_plan,
        run_conformal=True,
        keep_pins=False,
    )



def execute_phase1_transfer_only(context, obj, bm, patch_graph: PatchGraph, settings, solve_plan: Optional[SolvePlan] = None) -> dict[str, object]:
    return _execute_phase1_preview_impl(
        context,
        obj,
        bm,
        patch_graph,
        settings,
        solve_plan,
        run_conformal=False,
        keep_pins=True,
    )


def _select_preferred_edge_candidate(
    current: Optional[AttachmentCandidate],
    candidate: AttachmentCandidate,
) -> AttachmentCandidate:
    if current is None:
        return candidate
    current_key = (
        current.score,
        current.frame_continuation,
        current.endpoint_bridge,
        current.endpoint_strength,
        current.best_pair_strength,
        current.seam_norm,
    )
    candidate_key = (
        candidate.score,
        candidate.frame_continuation,
        candidate.endpoint_bridge,
        candidate.endpoint_strength,
        candidate.best_pair_strength,
        candidate.seam_norm,
    )
    return candidate if candidate_key > current_key else current


def _count_chain_endpoint_support(
    graph: PatchGraph,
    patch_id: int,
    loop_index: int,
    chain_index: int,
    chain_role: FrameRole,
) -> tuple[int, int, int]:
    fixed_endpoint_count = 0
    same_axis_endpoint_count = 0
    free_touched_endpoint_count = 0

    for endpoint_label in ('start', 'end'):
        endpoint_ctx = _get_chain_endpoint_context(graph, patch_id, loop_index, chain_index, endpoint_label)
        if endpoint_ctx is None:
            continue
        neighbor_roles = []
        for neighbor_loop_index, neighbor_chain_index in endpoint_ctx.get('neighbors', ()):
            neighbor_chain = graph.get_chain(patch_id, neighbor_loop_index, neighbor_chain_index)
            if neighbor_chain is None:
                continue
            neighbor_roles.append(neighbor_chain.frame_role)

        if any(role in {FrameRole.H_FRAME, FrameRole.V_FRAME} for role in neighbor_roles):
            fixed_endpoint_count += 1
        if chain_role in {FrameRole.H_FRAME, FrameRole.V_FRAME} and any(role == chain_role for role in neighbor_roles):
            same_axis_endpoint_count += 1
        if any(role == FrameRole.FREE for role in neighbor_roles):
            free_touched_endpoint_count += 1

    return fixed_endpoint_count, same_axis_endpoint_count, free_touched_endpoint_count


def _closure_cut_support_label(score: float) -> str:
    if score >= 0.80:
        return 'rigid'
    if score >= 0.62:
        return 'stable'
    if score >= 0.45:
        return 'mixed'
    return 'weak'


def _build_closure_cut_heuristic(
    graph: PatchGraph,
    candidate: AttachmentCandidate,
) -> ClosureCutHeuristic:
    owner_fixed, owner_same_axis, owner_free = _count_chain_endpoint_support(
        graph,
        candidate.owner_patch_id,
        candidate.owner_loop_index,
        candidate.owner_chain_index,
        candidate.owner_role,
    )
    target_fixed, target_same_axis, target_free = _count_chain_endpoint_support(
        graph,
        candidate.target_patch_id,
        candidate.target_loop_index,
        candidate.target_chain_index,
        candidate.target_role,
    )

    fixed_endpoint_count = owner_fixed + target_fixed
    same_axis_endpoint_count = owner_same_axis + target_same_axis
    free_touched_endpoint_count = owner_free + target_free
    fixed_ratio = fixed_endpoint_count / 4.0
    same_axis_ratio = same_axis_endpoint_count / 4.0
    free_touch_ratio = free_touched_endpoint_count / 4.0
    score = _clamp01(
        0.24 * candidate.frame_continuation
        + 0.18 * candidate.endpoint_bridge
        + 0.12 * candidate.endpoint_strength
        + 0.10 * candidate.seam_norm
        + 0.16 * fixed_ratio
        + 0.16 * same_axis_ratio
        + 0.14 * (1.0 - free_touch_ratio)
    )
    reasons = (
        f"fc={candidate.frame_continuation:.2f}",
        f"bridge={candidate.endpoint_bridge:.2f}",
        f"ep={candidate.endpoint_strength:.2f}",
        f"rigid={fixed_endpoint_count}/4",
        f"axis={same_axis_endpoint_count}/4",
        f"free={free_touched_endpoint_count}/4",
    )
    return ClosureCutHeuristic(
        edge_key=_patch_pair_key(candidate.owner_patch_id, candidate.target_patch_id),
        candidate=candidate,
        score=score,
        support_label=_closure_cut_support_label(score),
        fixed_endpoint_count=fixed_endpoint_count,
        same_axis_endpoint_count=same_axis_endpoint_count,
        free_touched_endpoint_count=free_touched_endpoint_count,
        reasons=reasons,
    )


def _build_quilt_edge_candidate_map(
    solver_graph: SolverGraph,
    quilt_patch_ids: set[int],
) -> dict[tuple[int, int], AttachmentCandidate]:
    edge_candidate_map: dict[tuple[int, int], AttachmentCandidate] = {}
    for candidate in solver_graph.candidates:
        if candidate.owner_patch_id not in quilt_patch_ids or candidate.target_patch_id not in quilt_patch_ids:
            continue
        edge_key = _patch_pair_key(candidate.owner_patch_id, candidate.target_patch_id)
        edge_candidate_map[edge_key] = _select_preferred_edge_candidate(edge_candidate_map.get(edge_key), candidate)
    return edge_candidate_map


def _analyze_quilt_closure_cuts(
    graph: PatchGraph,
    solver_graph: SolverGraph,
    quilt_plan: QuiltPlan,
) -> tuple[QuiltClosureCutAnalysis, ...]:
    quilt_patch_ids = set(quilt_plan.solved_patch_ids)
    quilt_patch_ids.add(quilt_plan.root_patch_id)
    tree_edges = _build_quilt_tree_edges(quilt_plan)
    patch_tree_adjacency = _build_patch_tree_adjacency(quilt_plan)
    edge_candidate_map = _build_quilt_edge_candidate_map(solver_graph, quilt_patch_ids)
    non_tree_edges = sorted(
        edge_key
        for edge_key in edge_candidate_map.keys()
        if edge_key not in tree_edges
    )

    analyses = []
    for edge_key in non_tree_edges:
        patch_path = _find_patch_tree_path(patch_tree_adjacency, edge_key[0], edge_key[1])
        if len(patch_path) < 2:
            continue
        cycle_edge_keys = [
            _patch_pair_key(patch_path[index], patch_path[index + 1])
            for index in range(len(patch_path) - 1)
        ]
        cycle_edge_keys.append(edge_key)

        cycle_edges = []
        for cycle_edge_key in cycle_edge_keys:
            cycle_candidate = edge_candidate_map.get(cycle_edge_key)
            if cycle_candidate is None:
                continue
            cycle_edges.append(_build_closure_cut_heuristic(graph, cycle_candidate))
        if not cycle_edges:
            continue

        current_cut = next((item for item in cycle_edges if item.edge_key == edge_key), None)
        if current_cut is None:
            continue
        recommended_cut = max(
            cycle_edges,
            key=lambda item: (
                item.score,
                item.same_axis_endpoint_count,
                item.fixed_endpoint_count,
                -item.free_touched_endpoint_count,
                item.edge_key,
            ),
        )
        analyses.append(
            QuiltClosureCutAnalysis(
                current_cut=current_cut,
                recommended_cut=recommended_cut,
                path_patch_ids=tuple(patch_path),
                cycle_edges=tuple(sorted(
                    cycle_edges,
                    key=lambda item: (
                        item.edge_key != recommended_cut.edge_key,
                        item.edge_key != current_cut.edge_key,
                        -item.score,
                        item.edge_key,
                    ),
                )),
            )
        )

    return tuple(analyses)


def _format_closure_cut_heuristic(item: ClosureCutHeuristic) -> str:
    candidate = item.candidate
    return (
        f"{item.edge_key[0]}-{item.edge_key[1]} cut:{item.score:.2f} {item.support_label} "
        f"| roles:{candidate.owner_role.value}<->{candidate.target_role.value} "
        f"| refs:L{candidate.owner_loop_index}C{candidate.owner_chain_index}->"
        f"L{candidate.target_loop_index}C{candidate.target_chain_index} "
        f"| {' '.join(item.reasons)}"
    )


def _format_patch_signature(graph: PatchGraph, patch_id: int) -> str:
    return graph.get_patch_semantic_key(patch_id)


def _format_candidate_line(candidate: AttachmentCandidate) -> str:
    return (
        f"{candidate.owner_patch_id} -> {candidate.target_patch_id} | score:{candidate.score:.2f} "
        f"| seam:{candidate.seam_length:.4f} | roles:{candidate.owner_role.value}<->{candidate.target_role.value} "
        f"| loops:{candidate.owner_loop_kind.value}<->{candidate.target_loop_kind.value} "
        f"| refs:L{candidate.owner_loop_index}C{candidate.owner_chain_index}->L{candidate.target_loop_index}C{candidate.target_chain_index}"
    )


def format_solve_plan_report(
    graph: PatchGraph,
    solver_graph: SolverGraph,
    solve_plan: SolvePlan,
    mesh_name: Optional[str] = None,
) -> tuple[list[str], str]:
    lines: list[str] = []
    if mesh_name:
        lines.append(f"Mesh: {mesh_name}")
    lines.append(
        f"Thresholds: propagate>={solve_plan.propagate_threshold:.2f} weak>={solve_plan.weak_threshold:.2f}"
    )

    components = solver_graph.solve_components or graph.connected_components()
    for component_index, component in enumerate(components):
        patch_ids = sorted(component)
        lines.append(f"Component {component_index}: patches={patch_ids}")
        lines.append("  Patch Certainty:")
        component_scores = sorted(
            (solver_graph.patch_scores[patch_id] for patch_id in patch_ids),
            key=lambda item: item.root_score,
            reverse=True,
        )
        for certainty in component_scores:
            lines.append(
                "    "
                + f"Patch {certainty.patch_id}: {_format_patch_signature(graph, certainty.patch_id)} "
                + f"| local:{'Y' if certainty.local_solvable else 'N'} | root:{certainty.root_score:.2f} "
                + f"| loops:outer={certainty.outer_count} hole={certainty.hole_count} "
                + f"| chains:{certainty.chain_count} H:{certainty.h_count} V:{certainty.v_count} Free:{certainty.free_count}"
            )
            if certainty.reasons:
                lines.append(f"      reasons: {' | '.join(certainty.reasons)}")

        component_candidates = [
            candidate
            for candidate in solver_graph.candidates
            if candidate.owner_patch_id in component and candidate.target_patch_id in component
        ]
        if component_candidates:
            lines.append("  Conductivity:")
            for candidate in sorted(component_candidates, key=lambda item: (-item.score, item.owner_patch_id, item.target_patch_id)):
                lines.append("    " + _format_candidate_line(candidate))
                lines.append(
                    "      "
                    + f"transitions:{candidate.owner_transition} | {candidate.target_transition} "
                    + f"| seam:{candidate.seam_norm:.2f} pair:{candidate.best_pair_strength:.2f} "
                    + f"cont:{candidate.frame_continuation:.2f} bridge:{candidate.endpoint_bridge:.2f} "
                    + f"corner:{candidate.corner_strength:.2f} sem:{candidate.semantic_strength:.2f} "
                    + f"ep:{candidate.endpoint_strength:.2f} amb:{candidate.ambiguity_penalty:.2f}"
                )

    closure_cut_analyses_by_quilt = {
        quilt.quilt_index: _analyze_quilt_closure_cuts(graph, solver_graph, quilt)
        for quilt in solve_plan.quilts
    }

    for quilt in solve_plan.quilts:
        tree_edges = sorted(_build_quilt_tree_edges(quilt))
        closure_cut_analyses = closure_cut_analyses_by_quilt.get(quilt.quilt_index, ())
        lines.append(
            f"Quilt {quilt.quilt_index}: component={quilt.component_index} root=Patch {quilt.root_patch_id} "
            f"({_format_patch_signature(graph, quilt.root_patch_id)}) root_score={quilt.root_score:.2f}"
        )
        if tree_edges:
            lines.append(
                "  Tree edges: " + ", ".join(f"{edge_key[0]}-{edge_key[1]}" for edge_key in tree_edges)
            )
        for step in quilt.steps:
            if step.is_root or step.incoming_candidate is None:
                lines.append(
                    f"  Step {step.step_index}: ROOT Patch {step.patch_id} "
                    f"({_format_patch_signature(graph, step.patch_id)})"
                )
                continue
            candidate = step.incoming_candidate
            lines.append(
                f"  Step {step.step_index}: Patch {candidate.owner_patch_id} -> Patch {step.patch_id} "
                f"| score:{candidate.score:.2f} | roles:{candidate.owner_role.value}<->{candidate.target_role.value}"
            )
            lines.append(
                "    "
                + f"refs:L{candidate.owner_loop_index}C{candidate.owner_chain_index}->"
                + f"L{candidate.target_loop_index}C{candidate.target_chain_index} "
                + f"| transitions:{candidate.owner_transition} | {candidate.target_transition}"
            )
        if closure_cut_analyses:
            lines.append("  Closure cuts:")
            for analysis_index, analysis in enumerate(closure_cut_analyses):
                current_edge = f"{analysis.current_cut.edge_key[0]}-{analysis.current_cut.edge_key[1]}"
                recommended_edge = f"{analysis.recommended_cut.edge_key[0]}-{analysis.recommended_cut.edge_key[1]}"
                decision = (
                    f"keep {current_edge}"
                    if analysis.current_cut.edge_key == analysis.recommended_cut.edge_key
                    else f"swap {current_edge} -> {recommended_edge}"
                )
                lines.append(
                    f"    Cycle {analysis_index}: path={list(analysis.path_patch_ids)} | decision:{decision}"
                )
                lines.append("      current: " + _format_closure_cut_heuristic(analysis.current_cut))
                lines.append("      best:    " + _format_closure_cut_heuristic(analysis.recommended_cut))
                for cycle_edge in analysis.cycle_edges:
                    marker = ""
                    if cycle_edge.edge_key == analysis.recommended_cut.edge_key:
                        marker += " [BEST]"
                    if cycle_edge.edge_key == analysis.current_cut.edge_key:
                        marker += " [CURRENT]"
                    lines.append("      edge: " + _format_closure_cut_heuristic(cycle_edge) + marker)
        lines.append(f"  Stop: {quilt.stop_reason}")
        if quilt.deferred_candidates:
            lines.append("  Deferred frontier:")
            for candidate in quilt.deferred_candidates:
                lines.append("    " + _format_candidate_line(candidate))
        if quilt.rejected_candidates:
            lines.append("  Rejected frontier:")
            for candidate in quilt.rejected_candidates:
                lines.append("    " + _format_candidate_line(candidate))

    if solve_plan.skipped_patch_ids:
        lines.append(f"Skipped patches: {sorted(solve_plan.skipped_patch_ids)}")

    total_steps = sum(len(quilt.steps) for quilt in solve_plan.quilts)
    total_deferred = sum(len(quilt.deferred_candidates) for quilt in solve_plan.quilts)
    total_rejected = sum(len(quilt.rejected_candidates) for quilt in solve_plan.quilts)
    total_closure_cycles = sum(len(items) for items in closure_cut_analyses_by_quilt.values())
    total_cut_swaps = sum(
        1
        for analyses in closure_cut_analyses_by_quilt.values()
        for analysis in analyses
        if analysis.current_cut.edge_key != analysis.recommended_cut.edge_key
    )
    summary = (
        f"Quilts: {len(solve_plan.quilts)} | Steps: {total_steps} | "
        f"Deferred: {total_deferred} | Rejected: {total_rejected} | "
        f"ClosureCycles: {total_closure_cycles} | CutSwaps: {total_cut_swaps} | "
        f"Skipped: {len(solve_plan.skipped_patch_ids)}"
    )
    return lines, summary





