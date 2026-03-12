from __future__ import annotations

from dataclasses import dataclass, field
import math
from heapq import heappop, heappush
from typing import Optional

from mathutils import Vector

try:
    from .constants import FRAME_ALIGNMENT_THRESHOLD
    from .model import BoundaryChain, ChainNeighborKind, FrameRole, LoopKind, PatchGraph, PatchNode
except ImportError:
    from constants import FRAME_ALIGNMENT_THRESHOLD
    from model import BoundaryChain, ChainNeighborKind, FrameRole, LoopKind, PatchGraph, PatchNode


EDGE_PROPAGATE_MIN = 0.45
EDGE_WEAK_MIN = 0.25


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
class ScaffoldPointKey:
    patch_id: int
    loop_index: int
    chain_index: int
    source_point_index: int


@dataclass(frozen=True)
class ScaffoldChainPlacement:
    patch_id: int
    loop_index: int
    chain_index: int
    frame_role: FrameRole
    source_kind: str = "chain"
    points: tuple[tuple[ScaffoldPointKey, Vector], ...] = ()


@dataclass
class ScaffoldPatchPlacement:
    patch_id: int
    loop_index: int
    root_chain_index: int = 0
    corner_positions: dict[int, Vector] = field(default_factory=dict)
    chain_placements: list[ScaffoldChainPlacement] = field(default_factory=list)
    bbox_min: Vector = field(default_factory=lambda: Vector((0.0, 0.0)))
    bbox_max: Vector = field(default_factory=lambda: Vector((0.0, 0.0)))
    notes: tuple[str, ...] = ()


@dataclass
class ScaffoldQuiltPlacement:
    quilt_index: int
    root_patch_id: int
    patches: dict[int, ScaffoldPatchPlacement] = field(default_factory=dict)


@dataclass
class ScaffoldMap:
    quilts: list[ScaffoldQuiltPlacement] = field(default_factory=list)


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def _iter_neighbor_chains(graph: PatchGraph, owner_patch_id: int, target_patch_id: int):
    node = graph.nodes.get(owner_patch_id)
    if node is None:
        return []

    refs = []
    for loop_index, boundary_loop in enumerate(node.boundary_loops):
        for chain_index, chain in enumerate(boundary_loop.chains):
            if chain.neighbor_kind != ChainNeighborKind.PATCH:
                continue
            if chain.neighbor_patch_id != target_patch_id:
                continue
            refs.append((loop_index, chain_index, boundary_loop, chain))
    return refs


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
        0.30 * area_norm
        + 0.30 * frame_strength
        + 0.20 * _clamp01(1.0 - free_ratio)
        + 0.10 * hole_factor
        + 0.10 * 1.0
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
                0.40 * frame_continuation
                + 0.25 * endpoint_bridge
                + 0.10 * corner_strength
                + 0.10 * semantic_strength
                + 0.10 * endpoint_strength
                + 0.05 * _loop_pair_strength(owner_loop.kind, target_loop.kind)
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
            0.25 * seam_norm
            + 0.40 * best_pair_strength
            + 0.20 * target_score.root_score
            + 0.15 * owner_score.root_score
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
    component_map = {}
    for component_index, component in enumerate(graph.connected_components()):
        for patch_id in component:
            component_map[patch_id] = component_index
    solver_graph.component_by_patch = component_map

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



def _classify_scaffold_points_role(node: PatchNode, points: list[Vector]) -> FrameRole:
    if len(points) < 2:
        return FrameRole.FREE

    us = [point.dot(node.basis_u) for point in points]
    vs = [point.dot(node.basis_v) for point in points]
    extent_u = max(us) - min(us)
    extent_v = max(vs) - min(vs)
    total_extent = max(extent_u, extent_v)
    if total_extent < 1e-8:
        return FrameRole.FREE

    ratio_v = extent_v / total_extent
    ratio_u = extent_u / total_extent
    if ratio_v < FRAME_ALIGNMENT_THRESHOLD:
        return FrameRole.H_FRAME
    if ratio_u < FRAME_ALIGNMENT_THRESHOLD:
        return FrameRole.V_FRAME
    return FrameRole.FREE



def _collect_loop_segment_points(boundary_loop, start_loop_index: int, end_loop_index: int) -> list[tuple[int, Vector]]:
    vertex_count = len(boundary_loop.vert_cos)
    if vertex_count <= 0:
        return []

    points = []
    current_index = start_loop_index % vertex_count
    safety = 0
    while safety < vertex_count + 2:
        safety += 1
        points.append((current_index, boundary_loop.vert_cos[current_index].copy()))
        if current_index == end_loop_index % vertex_count:
            break
        current_index = (current_index + 1) % vertex_count
    return points



def _derive_frame_spans_from_free_loop(node: PatchNode, boundary_loop) -> tuple[list[dict[str, object]], tuple[str, ...]]:
    if len(boundary_loop.chains) != 1:
        return [], ("not_single_chain",)
    if not boundary_loop.corners:
        return [], ("no_geometric_corners",)

    ordered_corners = sorted(enumerate(boundary_loop.corners), key=lambda item: item[1].loop_vert_index)
    if len(ordered_corners) < 4:
        return [], ("few_geometric_corners",)

    spans = []
    roles = []
    corner_count = len(ordered_corners)
    for index in range(corner_count):
        start_corner_index, start_corner = ordered_corners[index]
        end_corner_index, end_corner = ordered_corners[(index + 1) % corner_count]
        span_points = _collect_loop_segment_points(boundary_loop, start_corner.loop_vert_index, end_corner.loop_vert_index)
        point_cos = [point for _, point in span_points]
        role = _classify_scaffold_points_role(node, point_cos)
        if role == FrameRole.FREE:
            return [], ("contains_free_spans",)
        spans.append({
            'span_index': index,
            'role': role,
            'source_kind': 'span',
            'start_corner_index': start_corner_index,
            'end_corner_index': end_corner_index,
            'points': span_points,
        })
        roles.append(role)

    if any(roles[index] == roles[(index + 1) % len(roles)] for index in range(len(roles))):
        return [], ("non_alternating_spans",)
    if FrameRole.H_FRAME not in roles or FrameRole.V_FRAME not in roles:
        return [], ("missing_hv_mix",)

    return spans, ()


def _segment_source_direction(node: PatchNode, segment: dict[str, object]) -> Vector:
    role = segment.get('role', FrameRole.FREE)
    segment_points = segment.get('points', [])
    if len(segment_points) < 2:
        if role == FrameRole.H_FRAME:
            return Vector((1.0, 0.0))
        if role == FrameRole.V_FRAME:
            return Vector((0.0, 1.0))
        return Vector((0.0, 0.0))

    start_point = segment_points[0][1]
    end_point = segment_points[-1][1]
    delta = end_point - start_point
    return Vector((delta.dot(node.basis_u), delta.dot(node.basis_v)))



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


def _measure_loop_winding_sign(node: PatchNode, boundary_loop) -> float:
    if len(boundary_loop.vert_cos) < 3:
        return 1.0

    points_2d = [Vector((point.dot(node.basis_u), point.dot(node.basis_v))) for point in boundary_loop.vert_cos]
    signed_area = 0.0
    count = len(points_2d)
    for index in range(count):
        p1 = points_2d[index]
        p2 = points_2d[(index + 1) % count]
        signed_area += p1.x * p2.y - p2.x * p1.y

    if abs(signed_area) < 1e-8:
        return 1.0
    sign = 1.0 if signed_area > 0.0 else -1.0
    if boundary_loop.kind == LoopKind.HOLE:
        sign *= -1.0
    return sign




def _segment_neighbor_semantic(graph: PatchGraph, boundary_loop, segment: dict[str, object]) -> str:
    if segment.get('source_kind') != 'chain':
        return 'DERIVED'

    chain_index = int(segment.get('segment_index', -1))
    if chain_index < 0 or chain_index >= len(boundary_loop.chains):
        return 'UNKNOWN'

    chain = boundary_loop.chains[chain_index]
    if chain.neighbor_kind == ChainNeighborKind.PATCH:
        return graph.get_patch_semantic_key(chain.neighbor_patch_id)
    return chain.neighbor_kind.value



def _score_root_segment(graph: PatchGraph, node: PatchNode, boundary_loop, segment: dict[str, object]) -> float:
    role = segment.get('role', FrameRole.FREE)
    score = 0.0
    if role in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        score += 1.0
    if segment.get('source_kind') == 'chain':
        score += 0.15

    patch_type = node.patch_type.value if hasattr(node.patch_type, 'value') else str(node.patch_type)
    neighbor_semantic = _segment_neighbor_semantic(graph, boundary_loop, segment)

    if patch_type == 'WALL':
        if role == FrameRole.H_FRAME and neighbor_semantic == 'FLOOR.DOWN':
            score += 0.80
        elif role == FrameRole.H_FRAME and neighbor_semantic == 'FLOOR.UP':
            score += 0.70
        elif role == FrameRole.V_FRAME and neighbor_semantic == ChainNeighborKind.SEAM_SELF.value:
            score += 0.55
        elif role == FrameRole.V_FRAME and neighbor_semantic.endswith('.SIDE'):
            score += 0.35
    elif patch_type == 'FLOOR':
        if role == FrameRole.H_FRAME and neighbor_semantic.endswith('.SIDE'):
            score += 0.55
        elif role == FrameRole.V_FRAME and neighbor_semantic.endswith('.SIDE'):
            score += 0.25
    else:
        if neighbor_semantic != 'DERIVED':
            score += 0.25

    return score



def _choose_root_segment_index(graph: PatchGraph, node: PatchNode, boundary_loop, segments: list[dict[str, object]]) -> int:
    best_index = 0
    best_score = -1.0
    for index, segment in enumerate(segments):
        segment_score = _score_root_segment(graph, node, boundary_loop, segment)
        if segment_score > best_score:
            best_score = segment_score
            best_index = index
    return best_index



def _build_root_start_direction(graph: PatchGraph, node: PatchNode, boundary_loop, segment: dict[str, object]) -> Vector:
    role = segment.get('role', FrameRole.FREE)
    patch_type = node.patch_type.value if hasattr(node.patch_type, 'value') else str(node.patch_type)
    neighbor_semantic = _segment_neighbor_semantic(graph, boundary_loop, segment)

    if patch_type == 'WALL' and role == FrameRole.H_FRAME and neighbor_semantic in {'FLOOR.DOWN', 'FLOOR.UP'}:
        return Vector((1.0, 0.0))

    return _snap_direction_to_role(_segment_source_direction(node, segment), role)



def _transform_patch_placement_y(patch_placement: ScaffoldPatchPlacement, scale_y: float) -> ScaffoldPatchPlacement:
    corner_positions = {
        corner_index: Vector((point.x, point.y * scale_y))
        for corner_index, point in patch_placement.corner_positions.items()
    }
    chain_placements = []
    for chain_placement in patch_placement.chain_placements:
        chain_placements.append(
            ScaffoldChainPlacement(
                patch_id=chain_placement.patch_id,
                loop_index=chain_placement.loop_index,
                chain_index=chain_placement.chain_index,
                frame_role=chain_placement.frame_role,
                source_kind=chain_placement.source_kind,
                points=tuple((point_key, Vector((point.x, point.y * scale_y))) for point_key, point in chain_placement.points),
            )
        )
    bbox_min, bbox_max = _compute_bbox_from_chain_placements(chain_placements)
    return ScaffoldPatchPlacement(
        patch_id=patch_placement.patch_id,
        loop_index=patch_placement.loop_index,
        root_chain_index=patch_placement.root_chain_index,
        corner_positions=corner_positions,
        chain_placements=chain_placements,
        bbox_min=bbox_min,
        bbox_max=bbox_max,
        notes=patch_placement.notes,
    )



def _normalize_root_patch_orientation(graph: PatchGraph, node: PatchNode, boundary_loop, patch_placement: ScaffoldPatchPlacement) -> ScaffoldPatchPlacement:
    patch_type = node.patch_type.value if hasattr(node.patch_type, 'value') else str(node.patch_type)
    if patch_type != 'WALL':
        return patch_placement

    down_ys = []
    up_ys = []
    for chain_placement in patch_placement.chain_placements:
        if chain_placement.source_kind != 'chain':
            continue
        if chain_placement.chain_index < 0 or chain_placement.chain_index >= len(boundary_loop.chains):
            continue
        chain = boundary_loop.chains[chain_placement.chain_index]
        if chain.neighbor_kind != ChainNeighborKind.PATCH:
            continue
        neighbor_semantic = graph.get_patch_semantic_key(chain.neighbor_patch_id)
        avg_y = sum(point.y for _, point in chain_placement.points) / float(len(chain_placement.points))
        if neighbor_semantic == 'FLOOR.DOWN':
            down_ys.append(avg_y)
        elif neighbor_semantic == 'FLOOR.UP':
            up_ys.append(avg_y)

    should_flip = False
    if down_ys and up_ys:
        should_flip = (sum(down_ys) / len(down_ys)) > (sum(up_ys) / len(up_ys))
    elif down_ys:
        bbox_mid_y = 0.5 * (patch_placement.bbox_min.y + patch_placement.bbox_max.y)
        should_flip = (sum(down_ys) / len(down_ys)) > bbox_mid_y
    elif up_ys:
        bbox_mid_y = 0.5 * (patch_placement.bbox_min.y + patch_placement.bbox_max.y)
        should_flip = (sum(up_ys) / len(up_ys)) < bbox_mid_y

    if not should_flip:
        return patch_placement
    return _transform_patch_placement_y(patch_placement, -1.0)



def _compute_bbox_from_chain_placements(chain_placements: list[ScaffoldChainPlacement]) -> tuple[Vector, Vector]:
    all_points = [point for chain in chain_placements for _, point in chain.points]
    if not all_points:
        return Vector((0.0, 0.0)), Vector((0.0, 0.0))

    min_x = min(point.x for point in all_points)
    min_y = min(point.y for point in all_points)
    max_x = max(point.x for point in all_points)
    max_y = max(point.y for point in all_points)
    return Vector((min_x, min_y)), Vector((max_x, max_y))



def _build_root_patch_scaffold(graph: PatchGraph, node: PatchNode, final_scale: float) -> ScaffoldPatchPlacement:
    loop_index = _choose_root_loop(node)
    if loop_index < 0:
        return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=-1, notes=('no_frame_loop',))

    boundary_loop, segments, notes = _collect_patch_segments_for_loop(node, loop_index)
    if boundary_loop is None or not segments:
        return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=loop_index, notes=notes or ('contains_free',))

    start_segment_index = _choose_root_segment_index(graph, node, boundary_loop, segments)
    start_direction = _build_root_start_direction(graph, node, boundary_loop, segments[start_segment_index])
    patch_placement = _build_patch_scaffold_walk(
        node,
        loop_index,
        boundary_loop,
        segments,
        start_segment_index,
        final_scale,
        Vector((0.0, 0.0)),
        start_direction,
        walk_forward=True,
        anchor_points=None,
        notes=notes,
    )
    return _normalize_root_patch_orientation(graph, node, boundary_loop, patch_placement)


def build_root_scaffold_map(
    graph: PatchGraph,
    solve_plan: Optional[SolvePlan] = None,
    final_scale: float = 1.0,
) -> ScaffoldMap:
    scaffold_map = ScaffoldMap()
    if solve_plan is None:
        return scaffold_map

    for quilt in solve_plan.quilts:
        node = graph.nodes.get(quilt.root_patch_id)
        if node is None:
            continue
        quilt_scaffold = ScaffoldQuiltPlacement(quilt_index=quilt.quilt_index, root_patch_id=quilt.root_patch_id)
        quilt_scaffold.patches[quilt.root_patch_id] = _build_root_patch_scaffold(graph, node, final_scale)
        scaffold_map.quilts.append(quilt_scaffold)
    return scaffold_map


def _format_scaffold_uv_points(points: tuple[tuple[ScaffoldPointKey, Vector], ...]) -> str:
    if not points:
        return "[]"
    return '[' + ' '.join(f'({point.x:.4f},{point.y:.4f})' for _, point in points) + ']'



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
    for quilt in scaffold_map.quilts:
        patch_placement = quilt.patches.get(quilt.root_patch_id)
        total_patches += len(quilt.patches)
        signature = graph.get_patch_semantic_key(quilt.root_patch_id)
        if patch_placement is None:
            unsupported += 1
            lines.append(f"Quilt {quilt.quilt_index}: root=Patch {quilt.root_patch_id} ({signature}) | scaffold:missing")
            continue

        if patch_placement.notes:
            unsupported += 1
            note_text = ', '.join(patch_placement.notes)
            lines.append(
                f"Quilt {quilt.quilt_index}: root=Patch {quilt.root_patch_id} ({signature}) | scaffold:unsupported | notes:{note_text}"
            )
            continue

        lines.append(
            f"Quilt {quilt.quilt_index}: root=Patch {quilt.root_patch_id} ({signature}) | "
            f"loop:{patch_placement.loop_index} | start_chain:{patch_placement.root_chain_index} | "
            f"bbox:({patch_placement.bbox_min.x:.4f}, {patch_placement.bbox_min.y:.4f}) -> "
            f"({patch_placement.bbox_max.x:.4f}, {patch_placement.bbox_max.y:.4f})"
        )
        boundary_loop = graph.nodes[quilt.root_patch_id].boundary_loops[patch_placement.loop_index]
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
                f"  Corner {corner_index}: ({point.x:.4f}, {point.y:.4f}) | chains:{prev_chain}->{next_chain} | turn:{turn_angle:.1f}"
            )
        for chain_placement in patch_placement.chain_placements:
            if not chain_placement.points:
                continue
            start_point = chain_placement.points[0][1]
            end_point = chain_placement.points[-1][1]
            lines.append(
                f"  {chain_placement.source_kind.title()} {chain_placement.chain_index}: {chain_placement.frame_role.value} | "
                f"points:{len(chain_placement.points)} | start:({start_point.x:.4f}, {start_point.y:.4f}) | "
                f"end:({end_point.x:.4f}, {end_point.y:.4f}) | uv:{_format_scaffold_uv_points(chain_placement.points)}"
            )

    summary = f"Scaffold roots: {len(scaffold_map.quilts)} | Root patches: {total_patches} | Unsupported: {unsupported}"
    return lines, summary


def _all_patch_ids(graph: PatchGraph) -> list[int]:
    return sorted(graph.nodes.keys())



def _select_patch_faces(bm, graph: PatchGraph, patch_ids: list[int]) -> None:
    face_indices = set()
    for patch_id in patch_ids:
        node = graph.nodes.get(patch_id)
        if node is None:
            continue
        face_indices.update(node.face_indices)

    for face in bm.faces:
        face.select = face.index in face_indices



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



def _resolve_scaffold_loop_side(graph: PatchGraph, key: ScaffoldPointKey, source_kind: str) -> Optional[tuple[int, int]]:
    node = graph.nodes.get(key.patch_id)
    if node is None or key.loop_index < 0 or key.loop_index >= len(node.boundary_loops):
        return None

    boundary_loop = node.boundary_loops[key.loop_index]
    if not boundary_loop.vert_indices or not boundary_loop.side_face_indices:
        return None

    if source_kind == 'chain':
        if key.chain_index < 0 or key.chain_index >= len(boundary_loop.chains):
            return None
        chain = boundary_loop.chains[key.chain_index]
        if key.source_point_index < 0 or key.source_point_index >= len(chain.vert_indices):
            return None

        vert_index = chain.vert_indices[key.source_point_index]
        if key.source_point_index < len(chain.side_face_indices):
            face_index = chain.side_face_indices[key.source_point_index]
            return face_index, vert_index

        loop_point_index = (chain.start_loop_index + key.source_point_index) % len(boundary_loop.vert_indices)
        if loop_point_index < len(boundary_loop.side_face_indices):
            return boundary_loop.side_face_indices[loop_point_index], vert_index
        return None

    loop_point_index = key.source_point_index % len(boundary_loop.vert_indices)
    if loop_point_index >= len(boundary_loop.side_face_indices):
        return None
    return boundary_loop.side_face_indices[loop_point_index], boundary_loop.vert_indices[loop_point_index]



def _collect_patch_vertex_fan_faces(bm, patch_face_set: set[int], start_face_index: int, vert_index: int) -> list[int]:
    if start_face_index not in patch_face_set:
        return []

    visited = set()
    stack = [start_face_index]
    faces = []
    while stack:
        face_index = stack.pop()
        if face_index in visited:
            continue
        visited.add(face_index)
        face = bm.faces[face_index]
        if all(vert.index != vert_index for vert in face.verts):
            continue
        faces.append(face_index)

        for edge in face.edges:
            if edge.seam:
                continue
            if all(vert.index != vert_index for vert in edge.verts):
                continue
            for linked_face in edge.link_faces:
                if linked_face.index == face_index or linked_face.index in visited:
                    continue
                if linked_face.index not in patch_face_set:
                    continue
                if all(vert.index != vert_index for vert in linked_face.verts):
                    continue
                stack.append(linked_face.index)

    return faces



def _apply_patch_scaffold_to_uv(bm, graph: PatchGraph, uv_layer, patch_placement: ScaffoldPatchPlacement, uv_offset: Vector) -> dict[str, int]:
    node = graph.nodes.get(patch_placement.patch_id)
    if node is None:
        return {'scaffold_points': 0, 'pinned_uv_loops': 0}

    patch_face_set = set(node.face_indices)
    target_samples: dict[tuple[int, int], list[Vector]] = {}
    scaffold_keys = set()

    for chain_placement in patch_placement.chain_placements:
        for point_key, target_uv in chain_placement.points:
            resolved = _resolve_scaffold_loop_side(graph, point_key, chain_placement.source_kind)
            if resolved is None:
                continue
            face_index, vert_index = resolved
            fan_face_indices = _collect_patch_vertex_fan_faces(bm, patch_face_set, face_index, vert_index)
            if not fan_face_indices:
                fan_face_indices = [face_index]
            scaffold_keys.add((point_key.patch_id, point_key.loop_index, point_key.chain_index, point_key.source_point_index, chain_placement.source_kind))
            shifted_uv = target_uv + uv_offset
            for fan_face_index in fan_face_indices:
                target_samples.setdefault((fan_face_index, vert_index), []).append(shifted_uv.copy())

    if not target_samples:
        return {'scaffold_points': len(scaffold_keys), 'pinned_uv_loops': 0}

    pinned_uv_loops = 0
    for (face_index, vert_index), samples in target_samples.items():
        target_uv = sum(samples, Vector((0.0, 0.0))) / float(len(samples))
        face = bm.faces[face_index]
        for loop in face.loops:
            if loop.vert.index != vert_index:
                continue
            loop[uv_layer].uv = target_uv.copy()
            loop[uv_layer].pin_uv = True
            pinned_uv_loops += 1
            break

    return {'scaffold_points': len(scaffold_keys), 'pinned_uv_loops': pinned_uv_loops}



def _capture_patch_reference_uv_layouts(bm, graph: PatchGraph, uv_layer, patch_ids: list[int]) -> dict[int, dict[str, object]]:
    layouts = {}
    for patch_id in patch_ids:
        node = graph.nodes.get(patch_id)
        if node is None:
            continue

        loop_uvs = []
        all_uvs = []
        for face_index in node.face_indices:
            face = bm.faces[face_index]
            for loop in face.loops:
                uv = loop[uv_layer].uv.copy()
                loop_uvs.append((face.index, loop.vert.index, uv))
                all_uvs.append(uv)

        if not all_uvs:
            layouts[patch_id] = {
                'loop_uvs': [],
                'bbox_min': Vector((0.0, 0.0)),
                'bbox_max': Vector((0.0, 0.0)),
                'centroid': Vector((0.0, 0.0)),
            }
            continue

        min_x = min(uv.x for uv in all_uvs)
        min_y = min(uv.y for uv in all_uvs)
        max_x = max(uv.x for uv in all_uvs)
        max_y = max(uv.y for uv in all_uvs)
        centroid = sum(all_uvs, Vector((0.0, 0.0))) / float(len(all_uvs))
        layouts[patch_id] = {
            'loop_uvs': loop_uvs,
            'bbox_min': Vector((min_x, min_y)),
            'bbox_max': Vector((max_x, max_y)),
            'centroid': centroid,
        }
    return layouts



def _find_scaffold_chain_placement(patch_placement: ScaffoldPatchPlacement, loop_index: int, chain_index: int) -> Optional[ScaffoldChainPlacement]:
    for chain_placement in patch_placement.chain_placements:
        if chain_placement.loop_index == loop_index and chain_placement.chain_index == chain_index:
            return chain_placement
    return None



def _compute_owner_chain_outward(chain_placement: ScaffoldChainPlacement, patch_placement: ScaffoldPatchPlacement) -> tuple[Vector, Vector]:
    points = [point for _, point in chain_placement.points]
    if not points:
        return Vector((0.0, 0.0)), Vector((0.0, 1.0))

    midpoint = sum(points, Vector((0.0, 0.0))) / float(len(points))
    bbox_mid = 0.5 * (patch_placement.bbox_min + patch_placement.bbox_max)
    if chain_placement.frame_role == FrameRole.H_FRAME:
        outward = Vector((0.0, 1.0 if midpoint.y >= bbox_mid.y else -1.0))
    else:
        outward = Vector((1.0 if midpoint.x >= bbox_mid.x else -1.0, 0.0))
    return midpoint, outward



def _place_child_patch_reference(
    bm,
    graph: PatchGraph,
    uv_layer,
    child_patch_id: int,
    owner_chain_placement: ScaffoldChainPlacement,
    owner_patch_placement: ScaffoldPatchPlacement,
    reference_layout: dict[str, object],
    margin: float,
) -> dict[str, int]:
    node = graph.nodes.get(child_patch_id)
    if node is None:
        return {'attached_children': 0, 'pinned_uv_loops': 0}

    loop_uvs = reference_layout.get('loop_uvs', [])
    if not loop_uvs:
        return {'attached_children': 0, 'pinned_uv_loops': 0}

    bbox_min = reference_layout.get('bbox_min', Vector((0.0, 0.0)))
    bbox_max = reference_layout.get('bbox_max', Vector((0.0, 0.0)))
    centroid = reference_layout.get('centroid', Vector((0.0, 0.0)))
    midpoint, outward = _compute_owner_chain_outward(owner_chain_placement, owner_patch_placement)

    half_extent = 0.5 * (bbox_max - bbox_min)
    offset_distance = margin + (half_extent.y if abs(outward.y) > 0.5 else half_extent.x)
    target_centroid = midpoint + outward * offset_distance
    translation = target_centroid - centroid

    pinned_uv_loops = 0
    for face_index, vert_index, ref_uv in loop_uvs:
        face = bm.faces[face_index]
        target_uv = ref_uv + translation
        for loop in face.loops:
            if loop.vert.index != vert_index:
                continue
            loop[uv_layer].uv = target_uv.copy()
            loop[uv_layer].pin_uv = True
            pinned_uv_loops += 1
            break

    return {'attached_children': 1, 'pinned_uv_loops': pinned_uv_loops}


def execute_phase1_preview(context, obj, bm, patch_graph: PatchGraph, settings, solve_plan: Optional[SolvePlan] = None) -> dict[str, int]:
    import bmesh
    import bpy

    patch_ids = _all_patch_ids(patch_graph)
    if not patch_ids:
        return {'patches': 0, 'scaffold_points': 0, 'pinned_uv_loops': 0, 'quilts': 0, 'supported_roots': 0}

    bm.faces.ensure_lookup_table()
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    uv_layer = bm.loops.layers.uv.verify()

    _select_patch_faces(bm, patch_graph, patch_ids)
    bmesh.update_edit_mesh(obj.data)
    bpy.ops.uv.unwrap(method='CONFORMAL', margin=0.0)

    bm = bmesh.from_edit_mesh(obj.data)
    bm.faces.ensure_lookup_table()
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    uv_layer = bm.loops.layers.uv.verify()
    reference_layouts = _capture_patch_reference_uv_layouts(bm, patch_graph, uv_layer, patch_ids)
    _clear_patch_pins(bm, patch_graph, uv_layer, patch_ids)

    scaffold_map = build_root_scaffold_map(patch_graph, solve_plan, settings.final_scale)
    x_cursor = 0.0
    supported_roots = 0
    scaffold_points = 0
    pinned_uv_loops = 0
    attached_children = 0
    placed_root_info = {}

    for quilt in scaffold_map.quilts:
        patch_placement = quilt.patches.get(quilt.root_patch_id)
        if patch_placement is None or patch_placement.notes:
            continue

        width = max(patch_placement.bbox_max.x - patch_placement.bbox_min.x, settings.final_scale, 0.25)
        uv_offset = Vector((x_cursor - patch_placement.bbox_min.x, -patch_placement.bbox_min.y))
        stats = _apply_patch_scaffold_to_uv(bm, patch_graph, uv_layer, patch_placement, uv_offset)
        if int(stats.get('pinned_uv_loops', 0)) <= 0:
            continue

        supported_roots += 1
        scaffold_points += int(stats.get('scaffold_points', 0))
        pinned_uv_loops += int(stats.get('pinned_uv_loops', 0))
        placed_root_info[quilt.root_patch_id] = {
            'placement': patch_placement,
            'offset': uv_offset,
            'width': width,
        }
        x_cursor += width + max(width * 0.25, settings.final_scale * 2.0, 0.25)

    if solve_plan is not None:
        margin = max(settings.final_scale * 2.0, 0.25)
        for quilt in solve_plan.quilts:
            if len(quilt.steps) < 2:
                continue
            root_info = placed_root_info.get(quilt.root_patch_id)
            if root_info is None:
                continue
            first_child_step = quilt.steps[1]
            candidate = first_child_step.incoming_candidate
            if candidate is None or candidate.owner_patch_id != quilt.root_patch_id:
                continue

            owner_patch_placement = root_info['placement']
            owner_chain_placement = _find_scaffold_chain_placement(owner_patch_placement, candidate.owner_loop_index, candidate.owner_chain_index)
            if owner_chain_placement is None:
                continue

            owner_chain_placement_uv = ScaffoldChainPlacement(
                patch_id=owner_chain_placement.patch_id,
                loop_index=owner_chain_placement.loop_index,
                chain_index=owner_chain_placement.chain_index,
                frame_role=owner_chain_placement.frame_role,
                source_kind=owner_chain_placement.source_kind,
                points=tuple((point_key, point + root_info['offset']) for point_key, point in owner_chain_placement.points),
            )
            owner_patch_placement_uv = ScaffoldPatchPlacement(
                patch_id=owner_patch_placement.patch_id,
                loop_index=owner_patch_placement.loop_index,
                root_chain_index=owner_patch_placement.root_chain_index,
                corner_positions={key: point + root_info['offset'] for key, point in owner_patch_placement.corner_positions.items()},
                chain_placements=owner_patch_placement.chain_placements,
                bbox_min=owner_patch_placement.bbox_min + root_info['offset'],
                bbox_max=owner_patch_placement.bbox_max + root_info['offset'],
                notes=owner_patch_placement.notes,
            )
            child_stats = _place_child_patch_reference(
                bm,
                patch_graph,
                uv_layer,
                first_child_step.patch_id,
                owner_chain_placement_uv,
                owner_patch_placement_uv,
                reference_layouts.get(first_child_step.patch_id, {}),
                margin,
            )
            attached_children += int(child_stats.get('attached_children', 0))
            pinned_uv_loops += int(child_stats.get('pinned_uv_loops', 0))

    bmesh.update_edit_mesh(obj.data)
    if pinned_uv_loops > 0:
        bpy.ops.uv.unwrap(method='CONFORMAL', margin=0.0)

    bm = bmesh.from_edit_mesh(obj.data)
    bm.faces.ensure_lookup_table()
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    uv_layer = bm.loops.layers.uv.verify()
    _clear_patch_pins(bm, patch_graph, uv_layer, patch_ids)
    bmesh.update_edit_mesh(obj.data)

    return {
        'patches': len(patch_ids),
        'supported_roots': supported_roots,
        'scaffold_points': scaffold_points,
        'pinned_uv_loops': pinned_uv_loops,
        'quilts': len(scaffold_map.quilts),
        'attached_children': attached_children,
    }



def _collect_patch_segments_for_loop(node: PatchNode, loop_index: int) -> tuple[Optional[BoundaryLoop], list[dict[str, object]], tuple[str, ...]]:
    if loop_index < 0 or loop_index >= len(node.boundary_loops):
        return None, [], ('bad_loop_index',)

    boundary_loop = node.boundary_loops[loop_index]
    segments: list[dict[str, object]] = []
    notes: tuple[str, ...] = ()

    roles = [chain.frame_role for chain in boundary_loop.chains]
    frame_roles = [role for role in roles if role in {FrameRole.H_FRAME, FrameRole.V_FRAME}]
    if frame_roles:
        for chain_index, chain in enumerate(boundary_loop.chains):
            point_count = min(len(chain.vert_indices), len(chain.vert_cos))
            if point_count <= 0:
                continue
            point_indices = list(range(point_count))
            segments.append({
                'segment_index': chain_index,
                'role': chain.frame_role,
                'source_kind': 'chain',
                'start_corner_index': chain.start_corner_index,
                'end_corner_index': chain.end_corner_index,
                'points': [(point_index, chain.vert_cos[point_index].copy()) for point_index in point_indices],
            })
        return boundary_loop, segments, ()

    segments, notes = _derive_frame_spans_from_free_loop(node, boundary_loop)
    if not segments:
        return boundary_loop, [], notes or ('contains_free',)

    return boundary_loop, segments, notes


def _find_segment_list_index(segments: list[dict[str, object]], segment_index: int) -> int:
    for list_index, segment in enumerate(segments):
        if int(segment.get('segment_index', -1)) == segment_index:
            return list_index
    return -1


def _ordered_segment_source_points(segment: dict[str, object], walk_forward: bool) -> list[tuple[int, Vector]]:
    points = list(segment.get('points', []))
    if walk_forward:
        return points
    return list(reversed(points))


def _project_source_polyline_2d(node: PatchNode, ordered_source_points: list[tuple[int, Vector]], final_scale: float) -> list[Vector]:
    points_2d = [Vector((point.dot(node.basis_u), point.dot(node.basis_v))) for _, point in ordered_source_points]
    if not points_2d:
        return []
    origin = points_2d[0].copy()
    return [(point - origin) * final_scale for point in points_2d]



def _build_free_segment_points(node: PatchNode, ordered_source_points: list[tuple[int, Vector]], start_point: Vector, current_direction: Vector, final_scale: float) -> list[Vector]:
    relative_points = _project_source_polyline_2d(node, ordered_source_points, final_scale)
    if not relative_points:
        return [start_point.copy()]

    if len(relative_points) == 1:
        return [start_point.copy()]

    chord = relative_points[-1]
    if chord.length <= 1e-8 or current_direction.length <= 1e-8:
        return [start_point + point for point in relative_points]

    chord_angle = math.atan2(chord.y, chord.x)
    target_angle = math.atan2(current_direction.y, current_direction.x)
    rotate_deg = math.degrees(target_angle - chord_angle)
    transformed = []
    for point in relative_points:
        transformed.append(start_point + _rotate_direction(point, rotate_deg))
    return transformed


def _resample_uv_points(points: list[Vector], target_count: int) -> list[Vector]:
    if target_count <= 0:
        return []
    if not points:
        return []
    if len(points) == target_count:
        return [point.copy() for point in points]
    if len(points) == 1:
        return [points[0].copy() for _ in range(target_count)]

    distances = [0.0]
    for index in range(1, len(points)):
        distances.append(distances[-1] + (points[index] - points[index - 1]).length)
    total_length = distances[-1]
    if total_length <= 1e-8:
        return [points[0].copy() for _ in range(target_count)]

    resampled = []
    for sample_index in range(target_count):
        target_distance = total_length * (float(sample_index) / float(max(target_count - 1, 1)))
        source_index = 1
        while source_index < len(distances) and distances[source_index] < target_distance:
            source_index += 1
        if source_index >= len(distances):
            resampled.append(points[-1].copy())
            continue
        prev_distance = distances[source_index - 1]
        next_distance = distances[source_index]
        if next_distance - prev_distance <= 1e-8:
            resampled.append(points[source_index].copy())
            continue
        factor = (target_distance - prev_distance) / (next_distance - prev_distance)
        resampled.append(points[source_index - 1].lerp(points[source_index], factor))
    return resampled


def _build_patch_scaffold_walk(
    node: PatchNode,
    loop_index: int,
    boundary_loop,
    segments: list[dict[str, object]],
    start_segment_list_index: int,
    final_scale: float,
    start_point: Vector,
    start_direction: Vector,
    walk_forward: bool = True,
    anchor_points: Optional[list[Vector]] = None,
    notes: tuple[str, ...] = (),
) -> ScaffoldPatchPlacement:
    if not segments:
        return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=-1, notes=notes or ('no_segments',))

    segment_count = len(segments)
    ordered_list_indices = []
    for offset in range(segment_count):
        if walk_forward:
            ordered_list_indices.append((start_segment_list_index + offset) % segment_count)
        else:
            ordered_list_indices.append((start_segment_list_index - offset) % segment_count)

    winding_sign = _measure_loop_winding_sign(node, boundary_loop)
    corner_positions: dict[int, Vector] = {}
    chain_placements: list[ScaffoldChainPlacement] = []
    current_point = start_point.copy()
    current_direction = start_direction.normalized() if start_direction.length > 1e-8 else Vector((1.0, 0.0))
    root_chain_index = int(segments[start_segment_list_index].get('segment_index', 0))

    for ordered_index, segment_list_index in enumerate(ordered_list_indices):
        segment = segments[segment_list_index]
        ordered_source_points = _ordered_segment_source_points(segment, walk_forward)
        if not ordered_source_points:
            continue

        traversal_start_corner = int(segment.get('start_corner_index', -1)) if walk_forward else int(segment.get('end_corner_index', -1))
        traversal_end_corner = int(segment.get('end_corner_index', -1)) if walk_forward else int(segment.get('start_corner_index', -1))
        if traversal_start_corner >= 0 and traversal_start_corner not in corner_positions:
            corner_positions[traversal_start_corner] = current_point.copy()

        point_entries = []
        if ordered_index == 0 and anchor_points:
            fitted_anchor_points = _resample_uv_points(anchor_points, len(ordered_source_points))
            for (source_point_index, _), target_point in zip(ordered_source_points, fitted_anchor_points):
                point_entries.append((
                    ScaffoldPointKey(node.patch_id, loop_index, int(segment.get('segment_index', 0)), int(source_point_index)),
                    target_point.copy(),
                ))
            current_point = fitted_anchor_points[-1].copy()
            anchor_delta = fitted_anchor_points[-1] - fitted_anchor_points[0]
            if anchor_delta.length > 1e-8:
                if segment['role'] == FrameRole.FREE:
                    current_direction = anchor_delta.normalized()
                else:
                    current_direction = _snap_direction_to_role(anchor_delta, segment['role'])
            elif current_direction.length > 1e-8 and segment['role'] != FrameRole.FREE:
                current_direction = _snap_direction_to_role(current_direction, segment['role'])
        else:
            if segment['role'] == FrameRole.FREE:
                fitted_points = _build_free_segment_points(node, ordered_source_points, current_point, current_direction, final_scale)
                for (source_point_index, _), target_point in zip(ordered_source_points, fitted_points):
                    point_entries.append((
                        ScaffoldPointKey(node.patch_id, loop_index, int(segment.get('segment_index', 0)), int(source_point_index)),
                        target_point.copy(),
                    ))
                current_point = fitted_points[-1].copy()
                if len(fitted_points) >= 2:
                    tail = fitted_points[-1] - fitted_points[-2]
                    if tail.length > 1e-8:
                        current_direction = tail.normalized()
            else:
                segment_direction = _snap_direction_to_role(current_direction, segment['role'])
                point_entries.append((
                    ScaffoldPointKey(node.patch_id, loop_index, int(segment.get('segment_index', 0)), int(ordered_source_points[0][0])),
                    current_point.copy(),
                ))
                for point_index in range(1, len(ordered_source_points)):
                    prev_point = ordered_source_points[point_index - 1][1]
                    next_point = ordered_source_points[point_index][1]
                    segment_length = (next_point - prev_point).length * final_scale
                    current_point = current_point + segment_direction * segment_length
                    point_entries.append((
                        ScaffoldPointKey(node.patch_id, loop_index, int(segment.get('segment_index', 0)), int(ordered_source_points[point_index][0])),
                        current_point.copy(),
                    ))
                current_direction = segment_direction

        if traversal_end_corner >= 0:
            corner_positions[traversal_end_corner] = current_point.copy()

        chain_placements.append(
            ScaffoldChainPlacement(
                patch_id=node.patch_id,
                loop_index=loop_index,
                chain_index=int(segment.get('segment_index', 0)),
                frame_role=segment['role'],
                source_kind=str(segment.get('source_kind', 'chain')),
                points=tuple(point_entries),
            )
        )

        if ordered_index < len(ordered_list_indices) - 1:
            corner_index = traversal_end_corner
            turn_angle_deg = 90.0
            if 0 <= corner_index < len(boundary_loop.corners):
                turn_angle_deg = boundary_loop.corners[corner_index].turn_angle_deg
            turn_sign = winding_sign if walk_forward else -winding_sign
            current_direction = _rotate_direction(current_direction, turn_sign * turn_angle_deg)

    bbox_min, bbox_max = _compute_bbox_from_chain_placements(chain_placements)
    placement = ScaffoldPatchPlacement(
        patch_id=node.patch_id,
        loop_index=loop_index,
        root_chain_index=root_chain_index,
        corner_positions=corner_positions,
        chain_placements=chain_placements,
        bbox_min=bbox_min,
        bbox_max=bbox_max,
        notes=notes,
    )
    return placement


def _copy_patch_placement_with_loop_index(patch_placement: ScaffoldPatchPlacement, loop_index: int) -> ScaffoldPatchPlacement:
    chain_placements = [
        ScaffoldChainPlacement(
            patch_id=chain.patch_id,
            loop_index=loop_index,
            chain_index=chain.chain_index,
            frame_role=chain.frame_role,
            source_kind=chain.source_kind,
            points=tuple((point_key, point.copy()) for point_key, point in chain.points),
        )
        for chain in patch_placement.chain_placements
    ]
    return ScaffoldPatchPlacement(
        patch_id=patch_placement.patch_id,
        loop_index=loop_index,
        root_chain_index=patch_placement.root_chain_index,
        corner_positions={key: value.copy() for key, value in patch_placement.corner_positions.items()},
        chain_placements=chain_placements,
        bbox_min=patch_placement.bbox_min.copy(),
        bbox_max=patch_placement.bbox_max.copy(),
        notes=patch_placement.notes,
    )


def _build_target_anchor_points(
    graph: PatchGraph,
    candidate: AttachmentCandidate,
    owner_chain_placement: ScaffoldChainPlacement,
    target_segment: dict[str, object],
    walk_forward: bool,
) -> Optional[list[Vector]]:
    owner_chain = graph.get_chain(candidate.owner_patch_id, candidate.owner_loop_index, candidate.owner_chain_index)
    target_chain = graph.get_chain(candidate.target_patch_id, candidate.target_loop_index, candidate.target_chain_index)
    if owner_chain is None or target_chain is None:
        return None
    if target_segment.get('source_kind') != 'chain':
        return None

    owner_uv_by_vert: dict[int, Vector] = {}
    for point_key, point in owner_chain_placement.points:
        source_index = point_key.source_point_index
        if 0 <= source_index < len(owner_chain.vert_indices):
            owner_uv_by_vert[owner_chain.vert_indices[source_index]] = point.copy()

    ordered_source_points = _ordered_segment_source_points(target_segment, walk_forward)
    if not ordered_source_points:
        return None

    anchor_points = []
    for source_point_index, _ in ordered_source_points:
        if source_point_index < 0 or source_point_index >= len(target_chain.vert_indices):
            return None
        vert_index = target_chain.vert_indices[source_point_index]
        owner_point = owner_uv_by_vert.get(vert_index)
        if owner_point is None:
            return None
        anchor_points.append(owner_point.copy())
    return anchor_points


def _score_child_patch_side(owner_patch_placement: ScaffoldPatchPlacement, owner_chain_placement: ScaffoldChainPlacement, child_patch_placement: ScaffoldPatchPlacement) -> float:
    owner_midpoint, owner_outward = _compute_owner_chain_outward(owner_chain_placement, owner_patch_placement)
    child_midpoint = 0.5 * (child_patch_placement.bbox_min + child_patch_placement.bbox_max)
    return (child_midpoint - owner_midpoint).dot(owner_outward)


def _build_child_patch_scaffold(
    graph: PatchGraph,
    node: PatchNode,
    candidate: AttachmentCandidate,
    owner_patch_placement: ScaffoldPatchPlacement,
    owner_chain_placement: ScaffoldChainPlacement,
    final_scale: float,
) -> ScaffoldPatchPlacement:
    boundary_loop, segments, notes = _collect_patch_segments_for_loop(node, candidate.target_loop_index)
    if boundary_loop is None or not segments:
        return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=candidate.target_loop_index, notes=notes or ('no_target_segments',))

    start_segment_list_index = _find_segment_list_index(segments, candidate.target_chain_index)
    if start_segment_list_index < 0:
        return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=candidate.target_loop_index, notes=('missing_target_chain',))

    target_segment = segments[start_segment_list_index]
    if str(target_segment.get('source_kind', 'chain')) != 'chain':
        return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=candidate.target_loop_index, notes=('derived_child_unsupported',))

    best_placement = None
    best_score = float('-inf')
    for walk_forward in (True, False):
        anchor_points = _build_target_anchor_points(graph, candidate, owner_chain_placement, target_segment, walk_forward)
        if not anchor_points:
            continue
        start_direction = anchor_points[-1] - anchor_points[0]
        if start_direction.length <= 1e-8:
            if candidate.target_role == FrameRole.H_FRAME:
                start_direction = Vector((1.0, 0.0))
            elif candidate.target_role == FrameRole.V_FRAME:
                start_direction = Vector((0.0, 1.0))
            else:
                start_direction = Vector((1.0, 0.0))
        patch_placement = _build_patch_scaffold_walk(
            node,
            candidate.target_loop_index,
            boundary_loop,
            segments,
            start_segment_list_index,
            final_scale,
            anchor_points[0],
            start_direction,
            walk_forward=walk_forward,
            anchor_points=anchor_points,
            notes=notes,
        )
        patch_placement = _copy_patch_placement_with_loop_index(patch_placement, candidate.target_loop_index)
        side_score = _score_child_patch_side(owner_patch_placement, owner_chain_placement, patch_placement)
        if side_score > best_score:
            best_score = side_score
            best_placement = patch_placement

    if best_placement is None:
        return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=candidate.target_loop_index, notes=('child_anchor_failed',))
    return best_placement


def build_root_scaffold_map(
    graph: PatchGraph,
    solve_plan: Optional[SolvePlan] = None,
    final_scale: float = 1.0,
) -> ScaffoldMap:
    scaffold_map = ScaffoldMap()
    if solve_plan is None:
        return scaffold_map

    for quilt in solve_plan.quilts:
        quilt_scaffold = ScaffoldQuiltPlacement(quilt_index=quilt.quilt_index, root_patch_id=quilt.root_patch_id)
        root_node = graph.nodes.get(quilt.root_patch_id)
        if root_node is None:
            scaffold_map.quilts.append(quilt_scaffold)
            continue

        root_patch_placement = _build_root_patch_scaffold(graph, root_node, final_scale)
        quilt_scaffold.patches[quilt.root_patch_id] = root_patch_placement

        if not root_patch_placement.notes:
            for step in quilt.steps[1:]:
                candidate = step.incoming_candidate
                if candidate is None:
                    continue
                owner_patch_placement = quilt_scaffold.patches.get(candidate.owner_patch_id)
                if owner_patch_placement is None or owner_patch_placement.notes:
                    continue
                owner_chain_placement = _find_scaffold_chain_placement(owner_patch_placement, candidate.owner_loop_index, candidate.owner_chain_index)
                if owner_chain_placement is None:
                    continue
                child_node = graph.nodes.get(step.patch_id)
                if child_node is None:
                    continue
                child_patch_placement = _build_child_patch_scaffold(
                    graph,
                    child_node,
                    candidate,
                    owner_patch_placement,
                    owner_chain_placement,
                    final_scale,
                )
                quilt_scaffold.patches[step.patch_id] = child_patch_placement

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
    for quilt in scaffold_map.quilts:
        placed_patch_ids = []
        for step_patch_id in [quilt.root_patch_id] + [patch_id for patch_id in quilt.patches.keys() if patch_id != quilt.root_patch_id]:
            if step_patch_id not in quilt.patches:
                continue
            placed_patch_ids.append(step_patch_id)
        if not placed_patch_ids:
            placed_patch_ids = [quilt.root_patch_id]
        lines.append(f"Quilt {quilt.quilt_index}: root=Patch {quilt.root_patch_id} ({graph.get_patch_semantic_key(quilt.root_patch_id)}) | patches:{placed_patch_ids}")
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

            lines.append(
                f"  Patch {patch_id} ({signature}) | loop:{patch_placement.loop_index} | start_chain:{patch_placement.root_chain_index} | "
                f"bbox:({patch_placement.bbox_min.x:.4f}, {patch_placement.bbox_min.y:.4f}) -> ({patch_placement.bbox_max.x:.4f}, {patch_placement.bbox_max.y:.4f})"
            )
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
            for chain_placement in patch_placement.chain_placements:
                if not chain_placement.points:
                    continue
                start_point = chain_placement.points[0][1]
                end_point = chain_placement.points[-1][1]
                lines.append(
                    f"    {chain_placement.source_kind.title()} {chain_placement.chain_index}: {chain_placement.frame_role.value} | "
                    f"points:{len(chain_placement.points)} | start:({start_point.x:.4f}, {start_point.y:.4f}) | "
                    f"end:({end_point.x:.4f}, {end_point.y:.4f}) | uv:{_format_scaffold_uv_points(chain_placement.points)}"
                )

    summary = f"Scaffold quilts: {len(scaffold_map.quilts)} | Patches: {total_patches} | Unsupported: {unsupported}"
    return lines, summary


def _compute_quilt_bbox(quilt_scaffold: ScaffoldQuiltPlacement) -> tuple[Vector, Vector]:
    placements = [patch for patch in quilt_scaffold.patches.values() if not patch.notes]
    if not placements:
        return Vector((0.0, 0.0)), Vector((0.0, 0.0))
    min_x = min(patch.bbox_min.x for patch in placements)
    min_y = min(patch.bbox_min.y for patch in placements)
    max_x = max(patch.bbox_max.x for patch in placements)
    max_y = max(patch.bbox_max.y for patch in placements)
    return Vector((min_x, min_y)), Vector((max_x, max_y))


def execute_phase1_preview(context, obj, bm, patch_graph: PatchGraph, settings, solve_plan: Optional[SolvePlan] = None) -> dict[str, int]:
    import bmesh
    import bpy

    patch_ids = _all_patch_ids(patch_graph)
    if not patch_ids:
        return {'patches': 0, 'scaffold_points': 0, 'pinned_uv_loops': 0, 'quilts': 0, 'supported_roots': 0, 'attached_children': 0}

    bm.faces.ensure_lookup_table()
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    uv_layer = bm.loops.layers.uv.verify()
    _clear_patch_pins(bm, patch_graph, uv_layer, patch_ids)

    scaffold_map = build_root_scaffold_map(patch_graph, solve_plan, settings.final_scale)
    x_cursor = 0.0
    supported_roots = 0
    scaffold_points = 0
    pinned_uv_loops = 0
    attached_children = 0
    quilt_margin = max(settings.final_scale * 2.0, 0.25)

    for quilt in scaffold_map.quilts:
        quilt_patch_ids = sorted(
            patch_id
            for patch_id, patch_placement in quilt.patches.items()
            if not patch_placement.notes
        )
        if not quilt_patch_ids:
            continue

        bm = bmesh.from_edit_mesh(obj.data)
        bm.faces.ensure_lookup_table()
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        uv_layer = bm.loops.layers.uv.verify()
        _select_patch_faces(bm, patch_graph, quilt_patch_ids)
        _clear_patch_pins(bm, patch_graph, uv_layer, quilt_patch_ids)

        quilt_bbox_min, quilt_bbox_max = _compute_quilt_bbox(quilt)
        quilt_width = max(quilt_bbox_max.x - quilt_bbox_min.x, settings.final_scale, 0.25)
        uv_offset = Vector((x_cursor - quilt_bbox_min.x, -quilt_bbox_min.y))
        quilt_pinned_uv_loops = 0

        for patch_id in quilt_patch_ids:
            patch_placement = quilt.patches[patch_id]
            stats = _apply_patch_scaffold_to_uv(bm, patch_graph, uv_layer, patch_placement, uv_offset)
            if int(stats.get('pinned_uv_loops', 0)) <= 0:
                continue
            scaffold_points += int(stats.get('scaffold_points', 0))
            pinned_uv_loops += int(stats.get('pinned_uv_loops', 0))
            quilt_pinned_uv_loops += int(stats.get('pinned_uv_loops', 0))
            if patch_id == quilt.root_patch_id:
                supported_roots += 1
            else:
                attached_children += 1

        bmesh.update_edit_mesh(obj.data)
        if quilt_pinned_uv_loops > 0:
            bpy.ops.uv.unwrap(method='CONFORMAL', margin=0.0)
            bm = bmesh.from_edit_mesh(obj.data)
            bm.faces.ensure_lookup_table()
            bm.verts.ensure_lookup_table()
            bm.edges.ensure_lookup_table()
            uv_layer = bm.loops.layers.uv.verify()
            _clear_patch_pins(bm, patch_graph, uv_layer, quilt_patch_ids)
            bmesh.update_edit_mesh(obj.data)

        x_cursor += quilt_width + quilt_margin

    bm = bmesh.from_edit_mesh(obj.data)
    bm.faces.ensure_lookup_table()
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    uv_layer = bm.loops.layers.uv.verify()
    _clear_patch_pins(bm, patch_graph, uv_layer, patch_ids)
    bmesh.update_edit_mesh(obj.data)

    return {
        'patches': len(patch_ids),
        'supported_roots': supported_roots,
        'scaffold_points': scaffold_points,
        'pinned_uv_loops': pinned_uv_loops,
        'quilts': len(scaffold_map.quilts),
        'attached_children': attached_children,
    }

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

    components = graph.connected_components()
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

    for quilt in solve_plan.quilts:
        lines.append(
            f"Quilt {quilt.quilt_index}: component={quilt.component_index} root=Patch {quilt.root_patch_id} "
            f"({_format_patch_signature(graph, quilt.root_patch_id)}) root_score={quilt.root_score:.2f}"
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
    summary = (
        f"Quilts: {len(solve_plan.quilts)} | Steps: {total_steps} | "
        f"Deferred: {total_deferred} | Rejected: {total_rejected} | Skipped: {len(solve_plan.skipped_patch_ids)}"
    )
    return lines, summary
