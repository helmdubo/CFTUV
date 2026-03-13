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
SCAFFOLD_CLOSURE_EPSILON = 1e-4


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
    closure_error: float = 0.0
    max_chain_gap: float = 0.0
    gap_reports: tuple[tuple[int, int, float], ...] = ()
    closure_valid: bool = True
    notes: tuple[str, ...] = ()
    # Phase 3: envelope fields
    status: str = "COMPLETE"
    dependency_patches: tuple = ()
    unplaced_chain_indices: tuple = ()
    pinned: bool = False
    origin_offset: tuple = (0.0, 0.0)  # sleeping field, Phase 5

@dataclass
class ScaffoldQuiltPlacement:
    quilt_index: int
    root_patch_id: int
    patches: dict[int, ScaffoldPatchPlacement] = field(default_factory=dict)
    build_order: list = field(default_factory=list)  # Phase 3: ChainRef tuples


@dataclass
class ScaffoldMap:
    quilts: list[ScaffoldQuiltPlacement] = field(default_factory=list)


@dataclass(frozen=True)
class ScaffoldUvTarget:
    face_index: int
    vert_index: int
    loop_point_index: int


@dataclass(frozen=True)
class PatchFrontierCandidate:
    segment_list_index: int
    traversal_forward: bool
    known_endpoints: int
    start_point: Vector
    start_direction: Vector
    end_point: Optional[Vector] = None
    end_direction: Optional[Vector] = None
    score: float = 0.0


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


def _build_root_anchor_points(
    node: PatchNode,
    segment: dict[str, object],
    final_scale: float,
    start_direction: Vector,
    walk_forward: bool = True,
) -> list[Vector]:
    ordered_source_points = _ordered_segment_source_points(segment, walk_forward)
    if not ordered_source_points:
        return []

    role = segment.get('role', FrameRole.FREE)
    if role == FrameRole.FREE:
        return _build_guided_free_chain_from_one_end(
            node,
            ordered_source_points,
            Vector((0.0, 0.0)),
            start_direction,
            final_scale,
        )

    axis_direction = _snap_direction_to_role(start_direction, role)
    if axis_direction.length <= 1e-8:
        if role == FrameRole.H_FRAME:
            axis_direction = Vector((1.0, 0.0))
        elif role == FrameRole.V_FRAME:
            axis_direction = Vector((0.0, 1.0))
        else:
            axis_direction = Vector((1.0, 0.0))
    axis_direction.normalize()

    anchor_points = [Vector((0.0, 0.0))]
    total_length = 0.0
    for point_index in range(1, len(ordered_source_points)):
        prev_point = ordered_source_points[point_index - 1][1]
        next_point = ordered_source_points[point_index][1]
        total_length += (next_point - prev_point).length * final_scale
        anchor_points.append(axis_direction * total_length)
    return anchor_points


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
        closure_error=patch_placement.closure_error,
        max_chain_gap=patch_placement.max_chain_gap,
        gap_reports=patch_placement.gap_reports,
        closure_valid=patch_placement.closure_valid,
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


def _measure_patch_scaffold_closure(chain_placements: list[ScaffoldChainPlacement]) -> tuple[float, float, tuple[tuple[int, int, float], ...], bool]:
    ordered_chains = [chain for chain in chain_placements if chain.points]
    if not ordered_chains:
        return 0.0, 0.0, (), False

    gap_reports = []
    max_chain_gap = 0.0
    for chain_index, chain_placement in enumerate(ordered_chains):
        next_chain_placement = ordered_chains[(chain_index + 1) % len(ordered_chains)]
        chain_end = chain_placement.points[-1][1]
        next_start = next_chain_placement.points[0][1]
        gap = (next_start - chain_end).length
        max_chain_gap = max(max_chain_gap, gap)
        if gap > SCAFFOLD_CLOSURE_EPSILON:
            gap_reports.append((chain_placement.chain_index, next_chain_placement.chain_index, gap))

    closure_error = max_chain_gap
    return closure_error, max_chain_gap, tuple(gap_reports), len(gap_reports) == 0


def _patch_scaffold_is_supported(patch_placement: Optional[ScaffoldPatchPlacement]) -> bool:
    return patch_placement is not None and not patch_placement.notes and patch_placement.closure_valid


def _build_root_patch_scaffold(graph: PatchGraph, node: PatchNode, final_scale: float) -> ScaffoldPatchPlacement:
    loop_index = _choose_root_loop(node)
    if loop_index < 0:
        return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=-1, notes=('no_frame_loop',))

    boundary_loop, segments, notes = _collect_patch_segments_for_loop(node, loop_index)
    if boundary_loop is None or not segments:
        return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=loop_index, notes=notes or ('contains_free',))

    start_segment_index = _choose_root_segment_index(graph, node, boundary_loop, segments)
    start_segment = segments[start_segment_index]
    start_direction = _build_root_start_direction(graph, node, boundary_loop, start_segment)
    anchor_points = _build_root_anchor_points(node, start_segment, final_scale, start_direction, walk_forward=True)
    if not anchor_points:
        return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=loop_index, notes=notes or ('root_anchor_failed',))

    patch_placement = _build_patch_scaffold_frontier(
        node,
        loop_index,
        boundary_loop,
        segments,
        start_segment_index,
        final_scale,
        anchor_points,
        anchor_forward=True,
        notes=notes,
    )
    return _normalize_root_patch_orientation(graph, node, boundary_loop, patch_placement)


def _format_scaffold_uv_points(points: tuple[tuple[ScaffoldPointKey, Vector], ...]) -> str:
    if not points:
        return "[]"
    return '[' + ' '.join(f'({point.x:.4f},{point.y:.4f})' for _, point in points) + ']'


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



def _scaffold_key_id(point_key: ScaffoldPointKey, source_kind: str) -> tuple[int, int, int, int, str]:
    return (
        point_key.patch_id,
        point_key.loop_index,
        point_key.chain_index,
        point_key.source_point_index,
        source_kind,
    )


def _count_patch_scaffold_points(patch_placement: ScaffoldPatchPlacement) -> int:
    scaffold_keys = set()
    for chain_placement in patch_placement.chain_placements:
        for point_key, _ in chain_placement.points:
            scaffold_keys.add(_scaffold_key_id(point_key, chain_placement.source_kind))
    return len(scaffold_keys)


def _resolve_scaffold_uv_targets(graph: PatchGraph, key: ScaffoldPointKey, source_kind: str) -> list[ScaffoldUvTarget]:
    node = graph.nodes.get(key.patch_id)
    if node is None or key.loop_index < 0 or key.loop_index >= len(node.boundary_loops):
        return []

    boundary_loop = node.boundary_loops[key.loop_index]
    loop_count = len(boundary_loop.vert_indices)
    if loop_count <= 0 or not boundary_loop.side_face_indices:
        return []

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
    for side_index in ((loop_point_index - 1) % loop_count, loop_point_index):
        if side_index < 0 or side_index >= len(boundary_loop.side_face_indices):
            continue
        face_index = boundary_loop.side_face_indices[side_index]
        target_id = (face_index, vert_index)
        if face_index < 0 or target_id in seen:
            continue
        seen.add(target_id)
        targets.append(
            ScaffoldUvTarget(
                face_index=face_index,
                vert_index=vert_index,
                loop_point_index=loop_point_index,
            )
        )
    return targets


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

    target_samples: dict[tuple[int, int], list[Vector]] = {}
    scaffold_keys = set()
    unresolved_keys = set()

    for chain_placement in patch_placement.chain_placements:
        for point_key, target_uv in chain_placement.points:
            key_id = _scaffold_key_id(point_key, chain_placement.source_kind)
            scaffold_keys.add(key_id)
            targets = _resolve_scaffold_uv_targets(graph, point_key, chain_placement.source_kind)
            if not targets:
                unresolved_keys.add(key_id)
                continue

            shifted_uv = target_uv + uv_offset
            for target in targets:
                target_samples.setdefault((target.face_index, target.vert_index), []).append(shifted_uv.copy())

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
            loop[uv_layer].pin_uv = True
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
    total_length = sum(edge_lengths)
    target_delta = end_point - start_point
    chord_length = target_delta.length
    if total_length <= 1e-8 or chord_length <= 1e-8:
        return _interpolate_between_anchors_by_lengths(start_point, end_point, edge_lengths)
    if total_length <= chord_length * 1.02:
        return _interpolate_between_anchors_by_lengths(start_point, end_point, edge_lengths)

    source_directions = _segment_source_step_directions(node, ordered_source_points)
    start_dir = _normalize_direction(start_direction, source_directions[0] if source_directions else target_delta)
    end_fallback = source_directions[-1] if source_directions else target_delta
    end_dir = _normalize_direction(end_direction if end_direction is not None else end_fallback, end_fallback)

    handle_length = max(chord_length * 0.25, (total_length - chord_length) * 0.75)
    handle_length = max(handle_length, total_length * 0.1)
    sample_count = max(32, len(edge_lengths) * 8)
    for _ in range(8):
        guide_polyline = _sample_cubic_bezier_polyline(
            start_point,
            start_point + start_dir * handle_length,
            end_point - end_dir * handle_length,
            end_point,
            sample_count=sample_count,
        )
        resampled = _resample_polyline_by_edge_lengths(guide_polyline, edge_lengths)
        if resampled is not None:
            resampled[0] = start_point.copy()
            resampled[-1] = end_point.copy()
            return resampled
        handle_length *= 1.35

    points = _interpolate_between_anchors_by_lengths(start_point, end_point, edge_lengths)
    points[0] = start_point.copy()
    points[-1] = end_point.copy()
    return points



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




def _compute_frontier_direction_after_chain(
    boundary_loop,
    traversal_points: list[Vector],
    traversal_end_corner: int,
    walk_forward: bool,
    winding_sign: float,
    role: FrameRole,
) -> Vector:
    if len(traversal_points) >= 2:
        chain_direction = traversal_points[-1] - traversal_points[-2]
    else:
        chain_direction = _default_role_direction(role)

    if role != FrameRole.FREE:
        chain_direction = _snap_direction_to_role(chain_direction, role)
        if chain_direction.length <= 1e-8:
            chain_direction = _default_role_direction(role)
    chain_direction = _normalize_direction(chain_direction, _default_role_direction(role))

    turn_angle_deg = 90.0
    if 0 <= traversal_end_corner < len(boundary_loop.corners):
        turn_angle_deg = boundary_loop.corners[traversal_end_corner].turn_angle_deg
    turn_sign = winding_sign if walk_forward else -winding_sign
    return _normalize_direction(_rotate_direction(chain_direction, turn_sign * turn_angle_deg), chain_direction)



def _score_loop_frontier_candidate(segment: dict[str, object], known_endpoints: int, start_direction: Vector) -> float:
    role = segment.get('role', FrameRole.FREE)
    score = 0.0
    if known_endpoints >= 2:
        score += 1.5
    if role == FrameRole.H_FRAME:
        score += 1.0
    elif role == FrameRole.V_FRAME:
        score += 0.95
    else:
        score += 0.25
    if str(segment.get('source_kind', 'chain')) == 'chain':
        score += 0.05
    if role in {FrameRole.H_FRAME, FrameRole.V_FRAME} and start_direction.length > 1e-8:
        snapped = _snap_direction_to_role(start_direction, role)
        if snapped.length > 1e-8:
            score += 0.15
    return score



def _build_patch_chain_points(
    node: PatchNode,
    segment: dict[str, object],
    ordered_source_points: list[tuple[int, Vector]],
    final_scale: float,
    start_point: Vector,
    start_direction: Vector,
    end_point: Optional[Vector] = None,
    end_direction: Optional[Vector] = None,
) -> list[Vector]:
    role = segment.get('role', FrameRole.FREE)
    if role in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        if end_point is not None:
            return _build_frame_chain_between_anchors(ordered_source_points, start_point, end_point, final_scale)
        return _build_frame_chain_from_one_end(ordered_source_points, start_point, start_direction, role, final_scale)

    if end_point is not None:
        return _build_guided_free_chain_between_anchors(
            node,
            ordered_source_points,
            start_point,
            end_point,
            start_direction,
            end_direction,
            final_scale,
        )
    return _build_guided_free_chain_from_one_end(node, ordered_source_points, start_point, start_direction, final_scale)




def _build_patch_scaffold_frontier(
    node: PatchNode,
    loop_index: int,
    boundary_loop,
    segments: list[dict[str, object]],
    start_segment_list_index: int,
    final_scale: float,
    anchor_points: list[Vector],
    anchor_forward: bool = True,
    notes: tuple[str, ...] = (),
) -> ScaffoldPatchPlacement:
    if not segments:
        return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=loop_index, notes=notes or ('no_segments',))
    if start_segment_list_index < 0 or start_segment_list_index >= len(segments):
        return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=loop_index, notes=notes or ('bad_start_segment',))
    if not anchor_points:
        return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=loop_index, notes=notes or ('missing_anchor',))

    start_segment = segments[start_segment_list_index]
    anchor_source_points = _ordered_segment_source_points(start_segment, anchor_forward)
    if not anchor_source_points:
        return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=loop_index, notes=notes or ('empty_anchor_chain',))

    fitted_anchor_points = _resample_uv_points(anchor_points, len(anchor_source_points))
    if not fitted_anchor_points:
        return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=loop_index, notes=notes or ('anchor_resample_failed',))

    chain_index = int(start_segment.get('segment_index', 0))
    anchor_traversal_entries = [
        (
            ScaffoldPointKey(node.patch_id, loop_index, chain_index, int(source_point_index)),
            target_point.copy(),
        )
        for (source_point_index, _), target_point in zip(anchor_source_points, fitted_anchor_points)
    ]
    anchor_canonical_entries = anchor_traversal_entries if anchor_forward else list(reversed(anchor_traversal_entries))
    anchor_canonical_points = [point.copy() for _, point in anchor_canonical_entries]
    if not anchor_canonical_points:
        return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=loop_index, notes=notes or ('anchor_points_failed',))

    chain_placements_by_list_index = {
        start_segment_list_index: ScaffoldChainPlacement(
            patch_id=node.patch_id,
            loop_index=loop_index,
            chain_index=chain_index,
            frame_role=start_segment.get('role', FrameRole.FREE),
            source_kind=str(start_segment.get('source_kind', 'chain')),
            points=tuple(anchor_canonical_entries),
        )
    }
    corner_positions: dict[int, Vector] = {}
    start_corner_index = int(start_segment.get('start_corner_index', -1))
    end_corner_index = int(start_segment.get('end_corner_index', -1))
    if start_corner_index >= 0:
        corner_positions[start_corner_index] = anchor_canonical_points[0].copy()
    if end_corner_index >= 0:
        corner_positions[end_corner_index] = anchor_canonical_points[-1].copy()

    segment_count = len(segments)
    if segment_count == 1:
        chain_placements = list(chain_placements_by_list_index.values())
        bbox_min, bbox_max = _compute_bbox_from_chain_placements(chain_placements)
        closure_error, max_chain_gap, gap_reports, closure_valid = _measure_patch_scaffold_closure(chain_placements)
        return ScaffoldPatchPlacement(
            patch_id=node.patch_id,
            loop_index=loop_index,
            root_chain_index=chain_index,
            corner_positions=corner_positions,
            chain_placements=chain_placements,
            bbox_min=bbox_min,
            bbox_max=bbox_max,
            closure_error=closure_error,
            max_chain_gap=max_chain_gap,
            gap_reports=gap_reports,
            closure_valid=closure_valid,
            notes=notes,
        )

    winding_sign = _measure_loop_winding_sign(node, boundary_loop)
    placed_indices = {start_segment_list_index}
    left_index = (start_segment_list_index - 1) % segment_count
    right_index = (start_segment_list_index + 1) % segment_count
    left_point = anchor_canonical_points[0].copy()
    right_point = anchor_canonical_points[-1].copy()
    left_direction = _compute_frontier_direction_after_chain(
        boundary_loop,
        list(reversed(anchor_canonical_points)),
        start_corner_index,
        False,
        winding_sign,
        start_segment.get('role', FrameRole.FREE),
    )
    right_direction = _compute_frontier_direction_after_chain(
        boundary_loop,
        anchor_canonical_points,
        end_corner_index,
        True,
        winding_sign,
        start_segment.get('role', FrameRole.FREE),
    )

    while len(placed_indices) < segment_count:
        candidates: list[PatchFrontierCandidate] = []

        if right_index not in placed_indices:
            right_segment = segments[right_index]
            known_endpoints = 1
            end_point = None
            end_direction = None
            if right_index == left_index:
                known_endpoints = 2
                end_point = left_point.copy()
                end_direction = (-left_direction.copy()) if left_direction.length > 1e-8 else None
            candidates.append(
                PatchFrontierCandidate(
                    segment_list_index=right_index,
                    traversal_forward=True,
                    known_endpoints=known_endpoints,
                    start_point=right_point.copy(),
                    start_direction=right_direction.copy(),
                    end_point=end_point,
                    end_direction=end_direction,
                    score=_score_loop_frontier_candidate(right_segment, known_endpoints, right_direction),
                )
            )

        if left_index not in placed_indices and left_index != right_index:
            left_segment = segments[left_index]
            candidates.append(
                PatchFrontierCandidate(
                    segment_list_index=left_index,
                    traversal_forward=False,
                    known_endpoints=1,
                    start_point=left_point.copy(),
                    start_direction=left_direction.copy(),
                    score=_score_loop_frontier_candidate(left_segment, 1, left_direction),
                )
            )

        if not candidates:
            return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=loop_index, notes=notes or ('frontier_stalled',))

        candidate = max(candidates, key=lambda item: (item.score, item.known_endpoints, 1 if item.traversal_forward else 0))
        segment = segments[candidate.segment_list_index]
        traversal_source_points = _ordered_segment_source_points(segment, candidate.traversal_forward)
        if not traversal_source_points:
            return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=loop_index, notes=notes or ('empty_frontier_segment',))

        traversal_points = _build_patch_chain_points(
            node,
            segment,
            traversal_source_points,
            final_scale,
            candidate.start_point,
            candidate.start_direction,
            candidate.end_point,
            candidate.end_direction,
        )
        if len(traversal_points) != len(traversal_source_points):
            return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=loop_index, notes=notes or ('frontier_point_mismatch',))

        chain_index = int(segment.get('segment_index', 0))
        traversal_entries = [
            (
                ScaffoldPointKey(node.patch_id, loop_index, chain_index, int(source_point_index)),
                target_point.copy(),
            )
            for (source_point_index, _), target_point in zip(traversal_source_points, traversal_points)
        ]
        canonical_entries = traversal_entries if candidate.traversal_forward else list(reversed(traversal_entries))
        canonical_points = [point.copy() for _, point in canonical_entries]
        start_corner_index = int(segment.get('start_corner_index', -1))
        end_corner_index = int(segment.get('end_corner_index', -1))
        if start_corner_index >= 0:
            corner_positions[start_corner_index] = canonical_points[0].copy()
        if end_corner_index >= 0:
            corner_positions[end_corner_index] = canonical_points[-1].copy()

        chain_placements_by_list_index[candidate.segment_list_index] = ScaffoldChainPlacement(
            patch_id=node.patch_id,
            loop_index=loop_index,
            chain_index=chain_index,
            frame_role=segment.get('role', FrameRole.FREE),
            source_kind=str(segment.get('source_kind', 'chain')),
            points=tuple(canonical_entries),
        )
        placed_indices.add(candidate.segment_list_index)

        if candidate.traversal_forward:
            right_point = traversal_points[-1].copy()
            right_direction = _compute_frontier_direction_after_chain(boundary_loop, traversal_points, end_corner_index, True, winding_sign, segment.get('role', FrameRole.FREE))
            right_index = (candidate.segment_list_index + 1) % segment_count
        else:
            left_point = traversal_points[-1].copy()
            left_direction = _compute_frontier_direction_after_chain(boundary_loop, traversal_points, start_corner_index, False, winding_sign, segment.get('role', FrameRole.FREE))
            left_index = (candidate.segment_list_index - 1) % segment_count

    chain_placements = []
    for offset in range(segment_count):
        segment_list_index = (start_segment_list_index + offset) % segment_count
        placement = chain_placements_by_list_index.get(segment_list_index)
        if placement is not None:
            chain_placements.append(placement)

    bbox_min, bbox_max = _compute_bbox_from_chain_placements(chain_placements)
    closure_error, max_chain_gap, gap_reports, closure_valid = _measure_patch_scaffold_closure(chain_placements)
    return ScaffoldPatchPlacement(
        patch_id=node.patch_id,
        loop_index=loop_index,
        root_chain_index=int(start_segment.get('segment_index', 0)),
        corner_positions=corner_positions,
        chain_placements=chain_placements,
        bbox_min=bbox_min,
        bbox_max=bbox_max,
        closure_error=closure_error,
        max_chain_gap=max_chain_gap,
        gap_reports=gap_reports,
        closure_valid=closure_valid,
        notes=notes,
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
    for anchor_forward in (True, False):
        anchor_points = _build_target_anchor_points(graph, candidate, owner_chain_placement, target_segment, anchor_forward)
        if not anchor_points:
            continue
        patch_placement = _build_patch_scaffold_frontier(
            node,
            candidate.target_loop_index,
            boundary_loop,
            segments,
            start_segment_list_index,
            final_scale,
            anchor_points,
            anchor_forward=anchor_forward,
            notes=notes,
        )
        if patch_placement.notes:
            continue
        side_score = _score_child_patch_side(owner_patch_placement, owner_chain_placement, patch_placement)
        if side_score > best_score:
            best_score = side_score
            best_placement = patch_placement

    if best_placement is None:
        return ScaffoldPatchPlacement(patch_id=node.patch_id, loop_index=candidate.target_loop_index, notes=('child_anchor_failed',))
    return best_placement

CHAIN_FRONTIER_THRESHOLD = 0.3


def _cf_chain_source_points(chain):
    """Конвертирует chain.vert_cos в формат [(index, Vector)] для placement функций."""
    return [(i, co.copy()) for i, co in enumerate(chain.vert_cos)]


def _cf_determine_direction(chain, node):
    """UV direction для chain из базиса patch.

    H_FRAME → snap к (±1, 0)
    V_FRAME → snap к (0, ±1)
    FREE → проекция 3D direction на basis
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

    # FREE: проекция на 2D basis space
    u_comp = chain_3d.dot(node.basis_u)
    v_comp = chain_3d.dot(node.basis_v)
    d = Vector((u_comp, v_comp))
    if d.length > 1e-8:
        return d.normalized()
    return Vector((1.0, 0.0))


def _cf_choose_seed_chain(graph, root_node):
    """Выбирает strongest chain root patch для seed placement.

    Предпочитает H/V с сильным соседним контекстом.
    Для WALL: FLOOR.DOWN сосед = нижняя кромка стены = лучший seed.

    Returns: (loop_index, chain_index, chain) или None
    """
    best_ref = None
    best_score = -1.0

    for loop_idx, loop in enumerate(root_node.boundary_loops):
        if loop.kind != LoopKind.OUTER:
            continue
        for chain_idx, chain in enumerate(loop.chains):
            score = 0.0

            # Role
            if chain.frame_role in (FrameRole.H_FRAME, FrameRole.V_FRAME):
                score += 1.0
            else:
                score += 0.1

            # Chain length
            if len(chain.vert_cos) > 1:
                chain_len = sum(
                    (chain.vert_cos[i + 1] - chain.vert_cos[i]).length
                    for i in range(len(chain.vert_cos) - 1)
                )
                score += min(chain_len * 0.1, 0.5)

            # Neighbor semantic
            if chain.neighbor_kind == ChainNeighborKind.PATCH:
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


def _cf_is_adjacent(chain_a_idx, chain_b_idx, boundary_loop):
    """Два chain соседние в loop если делят corner."""
    for corner in boundary_loop.corners:
        if ((corner.prev_chain_index == chain_a_idx and corner.next_chain_index == chain_b_idx) or
                (corner.prev_chain_index == chain_b_idx and corner.next_chain_index == chain_a_idx)):
            return True
    return False


def _cf_find_anchors(chain_ref, chain, graph, point_registry, vert_to_placements, placed_refs):
    """Ищет anchor UV для start и end вершин chain.

    Same-patch: через corner adjacency (не берёт другую сторону SEAM_SELF).
    Cross-patch: через vert_index (seam connection).

    Returns: (start_uv, end_uv, known_endpoints)
    """
    patch_id, loop_index, chain_index = chain_ref
    node = graph.nodes.get(patch_id)
    if node is None or loop_index >= len(node.boundary_loops):
        return None, None, 0

    boundary_loop = node.boundary_loops[loop_index]
    start_vert = chain.start_vert_index
    end_vert = chain.end_vert_index
    start_uv = None
    end_uv = None

    # --- Same-patch: corner adjacency only ---
    for placed_ref in placed_refs:
        if placed_ref[0] != patch_id or placed_ref[1] != loop_index:
            continue
        p_chain_idx = placed_ref[2]
        if not _cf_is_adjacent(chain_index, p_chain_idx, boundary_loop):
            continue
        p_chain = boundary_loop.chains[p_chain_idx]
        p_last = len(p_chain.vert_indices) - 1

        if start_uv is None:
            if p_chain.end_vert_index == start_vert:
                key = (placed_ref[0], placed_ref[1], placed_ref[2], p_last)
                if key in point_registry:
                    start_uv = point_registry[key]
            elif p_chain.start_vert_index == start_vert:
                key = (placed_ref[0], placed_ref[1], placed_ref[2], 0)
                if key in point_registry:
                    start_uv = point_registry[key]

        if end_uv is None:
            if p_chain.end_vert_index == end_vert:
                key = (placed_ref[0], placed_ref[1], placed_ref[2], p_last)
                if key in point_registry:
                    end_uv = point_registry[key]
            elif p_chain.start_vert_index == end_vert:
                key = (placed_ref[0], placed_ref[1], placed_ref[2], 0)
                if key in point_registry:
                    end_uv = point_registry[key]

    # --- Cross-patch: vert_index match ---
    if start_uv is None and start_vert >= 0 and start_vert in vert_to_placements:
        for other_ref, pt_idx in vert_to_placements[start_vert]:
            if other_ref[0] != patch_id:
                key = (other_ref[0], other_ref[1], other_ref[2], pt_idx)
                if key in point_registry:
                    start_uv = point_registry[key]
                    break

    if end_uv is None and end_vert >= 0 and end_vert in vert_to_placements:
        for other_ref, pt_idx in vert_to_placements[end_vert]:
            if other_ref[0] != patch_id:
                key = (other_ref[0], other_ref[1], other_ref[2], pt_idx)
                if key in point_registry:
                    end_uv = point_registry[key]
                    break

    known = (1 if start_uv is not None else 0) + (1 if end_uv is not None else 0)
    return start_uv, end_uv, known


def _cf_score_candidate(chain_ref, chain, node, known, graph, placed_in_patch):
    """Chain-level score для frontier candidate.

    Scoring заморожен до стабилизации placement.
    """
    score = 0.0

    # Role (primary factor)
    if chain.frame_role in (FrameRole.H_FRAME, FrameRole.V_FRAME):
        score += 1.0
    else:
        score += 0.2

    # Known endpoints
    if known == 2:
        score += 0.8
    elif known == 1:
        score += 0.3

    # Neighbor semantic
    if chain.neighbor_kind == ChainNeighborKind.PATCH:
        neighbor_key = graph.get_patch_semantic_key(chain.neighbor_patch_id)
        if 'FLOOR' in neighbor_key:
            score += 0.3
        elif neighbor_key.endswith('.SIDE'):
            score += 0.15

    # Patch continuity (patch уже частично размещён)
    if placed_in_patch > 0:
        score += 0.2

    return score


def _cf_place_chain(chain, node, start_uv, end_uv, final_scale):
    """Размещает один chain в UV.

    Dispatch по role + known_endpoints к существующим placement функциям.
    H/V = snap к оси. FREE = guided interpolation/extension.

    Returns: list[Vector] UV позиций (по одной на каждую вершину chain).
    """
    source_pts = _cf_chain_source_points(chain)
    direction = _cf_determine_direction(chain, node)
    role = chain.frame_role

    if start_uv is not None and end_uv is not None:
        # Оба endpoints известны
        if role in (FrameRole.H_FRAME, FrameRole.V_FRAME):
            return _build_frame_chain_between_anchors(source_pts, start_uv, end_uv, final_scale)
        else:
            return _build_guided_free_chain_between_anchors(
                node, source_pts, start_uv, end_uv, direction, None, final_scale)

    elif start_uv is not None:
        # Только start известен
        if role in (FrameRole.H_FRAME, FrameRole.V_FRAME):
            return _build_frame_chain_from_one_end(source_pts, start_uv, direction, role, final_scale)
        else:
            return _build_guided_free_chain_from_one_end(
                node, source_pts, start_uv, direction, final_scale)

    elif end_uv is not None:
        # Только end известен — reverse, build from end, reverse back
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


def _cf_build_envelopes(graph, quilt_patch_ids, placed_chains_map, placed_chain_refs, point_registry):
    """Группирует размещённые chains в ScaffoldPatchPlacement per patch.

    Вычисляет corners, bbox, status, dependencies.
    """
    patches = {}

    for patch_id in quilt_patch_ids:
        node = graph.nodes.get(patch_id)
        if node is None:
            continue

        # Собираем placed chains этого patch
        patch_placements = []
        for ref, placement in placed_chains_map.items():
            if ref[0] == patch_id:
                patch_placements.append(placement)

        if not patch_placements:
            patches[patch_id] = ScaffoldPatchPlacement(
                patch_id=patch_id, loop_index=-1,
                notes=('no_placed_chains',), status="EMPTY")
            continue

        # OUTER loop index
        outer_loop_index = -1
        for loop_idx, loop in enumerate(node.boundary_loops):
            if loop.kind == LoopKind.OUTER:
                outer_loop_index = loop_idx
                break

        if outer_loop_index < 0:
            patches[patch_id] = ScaffoldPatchPlacement(
                patch_id=patch_id, loop_index=-1,
                notes=('no_outer_loop',), status="UNSUPPORTED")
            continue

        boundary_loop = node.boundary_loops[outer_loop_index]

        # Corners из endpoints размещённых chains
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

        # BBox
        all_pts = [pt for cp in patch_placements for _, pt in cp.points]
        if all_pts:
            bbox_min = Vector((min(p.x for p in all_pts), min(p.y for p in all_pts)))
            bbox_max = Vector((max(p.x for p in all_pts), max(p.y for p in all_pts)))
        else:
            bbox_min = Vector((0.0, 0.0))
            bbox_max = Vector((0.0, 0.0))

        # Status
        total_chains = len(boundary_loop.chains)
        placed_count = sum(
            1 for ci in range(total_chains)
            if (patch_id, outer_loop_index, ci) in placed_chain_refs
        )

        if placed_count >= total_chains:
            status = "COMPLETE"
        elif placed_count > 0:
            status = "PARTIAL"
        else:
            status = "EMPTY"

        # Unplaced chains
        unplaced = tuple(
            ci for ci in range(total_chains)
            if (patch_id, outer_loop_index, ci) not in placed_chain_refs
        )

        # Dependencies
        dep_set = set()
        for cp in patch_placements:
            for point_key, _ in cp.points:
                if point_key.patch_id != patch_id:
                    dep_set.add(point_key.patch_id)

        # Closure
        closure_error = 0.0
        closure_valid = True
        if status == "COMPLETE" and total_chains >= 2:
            sorted_pl = sorted(patch_placements, key=lambda cp: cp.chain_index)
            if sorted_pl[-1].points and sorted_pl[0].points:
                last_end = sorted_pl[-1].points[-1][1]
                first_start = sorted_pl[0].points[0][1]
                closure_error = (last_end - first_start).length
                closure_valid = closure_error < 0.05

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
        )

    return patches


def build_quilt_scaffold_chain_frontier(graph, quilt_plan, final_scale):
    """Chain-first strongest-frontier builder.

    Строит scaffold для одного quilt chain-by-chain через единый frontier pool.
    Не различает chains одного patch и chains через seam.
    """
    quilt_scaffold = ScaffoldQuiltPlacement(
        quilt_index=quilt_plan.quilt_index,
        root_patch_id=quilt_plan.root_patch_id,
    )

    root_node = graph.nodes.get(quilt_plan.root_patch_id)
    if root_node is None:
        return quilt_scaffold

    # --- Seed chain ---
    seed_result = _cf_choose_seed_chain(graph, root_node)
    if seed_result is None:
        quilt_scaffold.patches[quilt_plan.root_patch_id] = ScaffoldPatchPlacement(
            patch_id=quilt_plan.root_patch_id, loop_index=-1,
            notes=('no_seed_chain',), status="UNSUPPORTED")
        return quilt_scaffold

    seed_loop_idx, seed_chain_idx, seed_chain = seed_result
    seed_ref = (quilt_plan.root_patch_id, seed_loop_idx, seed_chain_idx)

    # Registries
    point_registry = {}
    vert_to_placements = {}
    placed_chain_refs = set()
    placed_chains_map = {}
    build_order = []

    # Place seed от (0, 0)
    seed_src = _cf_chain_source_points(seed_chain)
    seed_dir = _cf_determine_direction(seed_chain, root_node)

    if seed_chain.frame_role in (FrameRole.H_FRAME, FrameRole.V_FRAME):
        seed_uvs = _build_frame_chain_from_one_end(
            seed_src, Vector((0.0, 0.0)), seed_dir, seed_chain.frame_role, final_scale)
    else:
        seed_uvs = _build_guided_free_chain_from_one_end(
            root_node, seed_src, Vector((0.0, 0.0)), seed_dir, final_scale)

    if not seed_uvs:
        quilt_scaffold.patches[quilt_plan.root_patch_id] = ScaffoldPatchPlacement(
            patch_id=quilt_plan.root_patch_id, loop_index=-1,
            notes=('seed_placement_failed',), status="UNSUPPORTED")
        return quilt_scaffold

    seed_placement = ScaffoldChainPlacement(
        patch_id=seed_ref[0], loop_index=seed_ref[1], chain_index=seed_ref[2],
        frame_role=seed_chain.frame_role, source_kind='chain',
        points=tuple(
            (ScaffoldPointKey(seed_ref[0], seed_ref[1], seed_ref[2], i), uv.copy())
            for i, uv in enumerate(seed_uvs)
        ),
    )

    placed_chain_refs.add(seed_ref)
    placed_chains_map[seed_ref] = seed_placement
    build_order.append(seed_ref)
    _cf_register_points(seed_ref, seed_chain, seed_uvs, point_registry, vert_to_placements)

    print(
        f"[CFTUV][Frontier] Seed: P{seed_ref[0]} L{seed_ref[1]}C{seed_ref[2]} "
        f"{seed_chain.frame_role.value} "
        f"({seed_uvs[0].x:.4f},{seed_uvs[0].y:.4f})"
        f"->({seed_uvs[-1].x:.4f},{seed_uvs[-1].y:.4f})"
    )

    # --- All chains in quilt (OUTER loops only) ---
    all_chain_pool = []
    for patch_id in quilt_plan.solved_patch_ids:
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

    # --- Frontier loop ---
    max_iter = len(all_chain_pool) + 10
    for iteration in range(1, max_iter + 1):
        best_ref = None
        best_score = -1.0
        best_data = None

        for ref, chain, node in all_chain_pool:
            if ref in placed_chain_refs:
                continue

            anchor_start, anchor_end, known = _cf_find_anchors(
                ref, chain, graph, point_registry, vert_to_placements, placed_chain_refs)

            if known == 0:
                continue

            placed_in_patch = sum(1 for pr in placed_chain_refs if pr[0] == ref[0])
            score = _cf_score_candidate(ref, chain, node, known, graph, placed_in_patch)

            if score > best_score:
                best_score = score
                best_ref = ref
                best_data = (chain, node, anchor_start, anchor_end)

        if best_ref is None or best_score < CHAIN_FRONTIER_THRESHOLD:
            break

        chain, node, anchor_start, anchor_end = best_data
        uv_points = _cf_place_chain(chain, node, anchor_start, anchor_end, final_scale)

        if not uv_points or len(uv_points) != len(chain.vert_cos):
            placed_chain_refs.add(best_ref)
            continue

        chain_placement = ScaffoldChainPlacement(
            patch_id=best_ref[0], loop_index=best_ref[1], chain_index=best_ref[2],
            frame_role=chain.frame_role, source_kind='chain',
            points=tuple(
                (ScaffoldPointKey(best_ref[0], best_ref[1], best_ref[2], i), uv.copy())
                for i, uv in enumerate(uv_points)
            ),
        )

        placed_chain_refs.add(best_ref)
        placed_chains_map[best_ref] = chain_placement
        build_order.append(best_ref)
        _cf_register_points(best_ref, chain, uv_points, point_registry, vert_to_placements)

        ep = (1 if anchor_start else 0) + (1 if anchor_end else 0)
        print(
            f"[CFTUV][Frontier] Step {iteration}: "
            f"P{best_ref[0]} L{best_ref[1]}C{best_ref[2]} "
            f"{chain.frame_role.value} score:{best_score:.2f} ep:{ep}"
        )

    # --- Build envelopes ---
    quilt_scaffold.patches = _cf_build_envelopes(
        graph, quilt_plan.solved_patch_ids,
        placed_chains_map, placed_chain_refs, point_registry)
    quilt_scaffold.build_order = list(build_order)

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

        if _patch_scaffold_is_supported(root_patch_placement):
            for step in quilt.steps[1:]:
                candidate = step.incoming_candidate
                if candidate is None:
                    continue
                owner_patch_placement = quilt_scaffold.patches.get(candidate.owner_patch_id)
                if not _patch_scaffold_is_supported(owner_patch_placement):
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
    invalid_closure = 0
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

    summary = (
        f"Scaffold quilts: {len(scaffold_map.quilts)} | Patches: {total_patches} | Unsupported: {unsupported} | "
        f"Invalid closure: {invalid_closure}"
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

    x_cursor = 0.0
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
    quilt_margin = max(settings.final_scale * 2.0, 0.25)
    global_supported_patch_ids: set[int] = set()
    conformal_applied = 0

    for quilt_scaffold in scaffold_map.quilts:
        quilt_plan = quilt_plan_by_index.get(quilt_scaffold.quilt_index)
        quilt_patch_ids = _ordered_quilt_patch_ids(quilt_scaffold, quilt_plan)
        if not quilt_patch_ids:
            continue
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
        quilt_width = max(quilt_bbox_max.x - quilt_bbox_min.x, settings.final_scale, 0.25)
        uv_offset = Vector((x_cursor - quilt_bbox_min.x, -quilt_bbox_min.y))
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
            patch_stats = _apply_patch_scaffold_to_uv(bm, patch_graph, uv_layer, patch_placement, uv_offset)
            validate_scaffold_uv_transfer(bm, patch_graph, uv_layer, patch_placement, uv_offset)
            _print_phase1_preview_patch_report(quilt_scaffold.quilt_index, patch_id, patch_stats)

            for stat_key in quilt_stats:
                quilt_stats[stat_key] += int(patch_stats.get(stat_key, 0))

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

        bmesh.update_edit_mesh(obj.data)
        x_cursor += quilt_width + quilt_margin

    if run_conformal and pinned_uv_loops > 0 and global_supported_patch_ids:
        bm = bmesh.from_edit_mesh(obj.data)
        bm.faces.ensure_lookup_table()
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        uv_layer = bm.loops.layers.uv.verify()
        supported_patch_ids = sorted(global_supported_patch_ids)
        _select_patch_faces(bm, patch_graph, supported_patch_ids)
        bmesh.update_edit_mesh(obj.data)
        print(
            f"[CFTUV][Phase1] Global Conformal: quilts={len(scaffold_map.quilts)} "
            f"patches={supported_patch_ids}"
        )
        bpy.ops.uv.unwrap(method='CONFORMAL', margin=0.0)
        conformal_applied = 1
        if not keep_pins:
            bm = bmesh.from_edit_mesh(obj.data)
            bm.faces.ensure_lookup_table()
            bm.verts.ensure_lookup_table()
            bm.edges.ensure_lookup_table()
            uv_layer = bm.loops.layers.uv.verify()
            _clear_patch_pins(bm, patch_graph, uv_layer, supported_patch_ids)
            bmesh.update_edit_mesh(obj.data)
    elif not run_conformal:
        print(f"[CFTUV][Phase1] Transfer Only: quilts={len(scaffold_map.quilts)} patches={sorted(global_supported_patch_ids)}")

    if not keep_pins:
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
    }

# ============================================================
# PHASE 2 PATCH — Validation Layer for solve.py
#
# ИНСТРУКЦИЯ:
#
# ШАГ 1: Открой cftuv/solve.py
#         Найди строку:  def execute_phase1_preview(
#         ВСТАВЬ весь блок ниже (от "def validate_scaffold_uv_transfer"
#         до конца файла) ПЕРЕД этой строкой.
#         Между ними должна быть пустая строка.
#
# ШАГ 2: В той же функции execute_phase1_preview найди строку:
#             patch_stats = _apply_patch_scaffold_to_uv(bm, patch_graph, uv_layer, patch_placement, uv_offset)
#         ПОСЛЕ неё добавь одну строку:
#             validate_scaffold_uv_transfer(bm, patch_graph, uv_layer, patch_placement, uv_offset)
#
# Готово. Две правки, ничего больше не трогаем.
# ============================================================


def validate_scaffold_uv_transfer(bm, graph, uv_layer, patch_placement, uv_offset, epsilon=1e-4):
    """Phase 2: Diagnostic — проверяет что scaffold points корректно записаны в UV.

    Вызывается ПОСЛЕ _apply_patch_scaffold_to_uv() и ПЕРЕД Conformal unwrap.
    Печатает конкретные mismatches в System Console.

    Что проверяет:
    - scaffold point resolve -> правильный vert_index в правильном face
    - intended UV ≈ actual UV (distance < epsilon)
    - SEAM_SELF sides получили разные UV (не схлопнулись)
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
            targets = _resolve_scaffold_uv_targets(graph, point_key, chain_placement.source_kind)
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
