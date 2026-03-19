from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum
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
        BoundaryChain, BoundaryCorner, BoundaryLoop, ChainNeighborKind, FrameRole, LoopKind,
        PatchGraph, PatchNode,
        ScaffoldPointKey, ScaffoldChainPlacement, ScaffoldPatchPlacement,
        ScaffoldQuiltPlacement, ScaffoldMap, ScaffoldClosureSeamReport,
        ScaffoldFrameAlignmentReport, ChainGapReport, PatchPlacementStatus, PlacementSourceKind,
        ClosureAnchorMode, FrameAxisKind, ChainRef, PatchEdgeKey, LoopChainRef, SourcePoint, AnchorAdjustment,
        FormattedReport,
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
        BoundaryChain, BoundaryCorner, BoundaryLoop, ChainNeighborKind, FrameRole, LoopKind,
        PatchGraph, PatchNode,
        ScaffoldPointKey, ScaffoldChainPlacement, ScaffoldPatchPlacement,
        ScaffoldQuiltPlacement, ScaffoldMap, ScaffoldClosureSeamReport,
        ScaffoldFrameAlignmentReport, ChainGapReport, PatchPlacementStatus, PlacementSourceKind,
        ClosureAnchorMode, FrameAxisKind, ChainRef, PatchEdgeKey, LoopChainRef, SourcePoint, AnchorAdjustment,
        FormattedReport,
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


RowClassKey = tuple[int]
ColumnClassKey = tuple[int, int]
FrameClassKey = tuple[int, ...]


@dataclass(frozen=True)
class SolveComponentsResult:
    components: list[set[int]]
    component_by_patch: dict[int, int]


@dataclass(frozen=True)
class UvAxisMetrics:
    span: float
    axis_error: float


@dataclass(frozen=True)
class FrameUvComponents:
    axis: float
    cross: float


@dataclass(frozen=True)
class FrameGroupDisplayCoords:
    coord_a: float
    coord_b: float


@dataclass(frozen=True)
class PatchRoleCounts:
    outer_count: int
    hole_count: int
    chain_count: int
    free_count: int
    h_count: int
    v_count: int


@dataclass(frozen=True)
class UvBounds:
    bbox_min: Vector
    bbox_max: Vector


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
class SolveView:
    graph: PatchGraph
    visible_loop_indices_by_patch: dict[int, tuple[int, ...]] = field(default_factory=dict)
    primary_loop_index_by_patch: dict[int, int] = field(default_factory=dict)
    locally_solvable_patch_ids: frozenset[int] = frozenset()

    def get_visible_loop_indices(self, patch_id: int) -> tuple[int, ...]:
        return self.visible_loop_indices_by_patch.get(patch_id, ())

    def iter_visible_loops(self, patch_id: int):
        node = self.graph.nodes.get(patch_id)
        if node is None:
            return
        for loop_index in self.get_visible_loop_indices(patch_id):
            if 0 <= loop_index < len(node.boundary_loops):
                yield loop_index, node.boundary_loops[loop_index]

    def iter_visible_chains(self, patch_id: int):
        for loop_index, boundary_loop in self.iter_visible_loops(patch_id):
            for chain_index, chain in enumerate(boundary_loop.chains):
                yield loop_index, chain_index, boundary_loop, chain

    def iter_attachment_neighbor_chains(self, owner_patch_id: int, target_patch_id: int):
        refs: list[AttachmentNeighborRef] = []
        for loop_index, chain_index, boundary_loop, chain in self.iter_visible_chains(owner_patch_id):
            if chain.neighbor_kind != ChainNeighborKind.PATCH:
                continue
            if chain.neighbor_patch_id != target_patch_id:
                continue
            refs.append(AttachmentNeighborRef(
                loop_index=loop_index,
                chain_index=chain_index,
                boundary_loop=boundary_loop,
                chain=chain,
            ))
        return refs

    def primary_loop_index(self, patch_id: int) -> int:
        return self.primary_loop_index_by_patch.get(patch_id, -1)

    def patch_is_locally_solvable(self, patch_id: int) -> bool:
        return patch_id in self.locally_solvable_patch_ids

    def patch_types_compatible(self, owner_patch_id: int, target_patch_id: int) -> bool:
        owner_node = self.graph.nodes.get(owner_patch_id)
        target_node = self.graph.nodes.get(target_patch_id)
        if owner_node is None or target_node is None:
            return False

        owner_type = owner_node.patch_type.value if hasattr(owner_node.patch_type, 'value') else str(owner_node.patch_type)
        target_type = target_node.patch_type.value if hasattr(target_node.patch_type, 'value') else str(target_node.patch_type)
        return owner_type == target_type


@dataclass(frozen=True)
class QuiltStep:
    step_index: int
    patch_id: int
    is_root: bool = False
    incoming_candidate: Optional[AttachmentCandidate] = None


class QuiltStopReason(str, Enum):
    UNSET = ""
    FRONTIER_BELOW_THRESHOLD = "frontier_below_threshold"
    FRONTIER_EXHAUSTED = "frontier_exhausted"


PointRegistryKey = tuple[int, int, int, int]
VertexPlacementRef = tuple[ChainRef, int]
PointRegistry = dict[PointRegistryKey, Vector]
VertexPlacementMap = dict[int, list[VertexPlacementRef]]
TransferTargetId = tuple[int, int]
ScaffoldKeyId = tuple[int, int, int, int, PlacementSourceKind]
TargetSampleMap = dict[TransferTargetId, list[Vector]]
PinnedTargetIdSet = set[TransferTargetId]
ScaffoldKeySet = set[ScaffoldKeyId]


def _point_registry_key(chain_ref: ChainRef, source_point_index: int) -> PointRegistryKey:
    return (chain_ref[0], chain_ref[1], chain_ref[2], source_point_index)


@dataclass(frozen=True)
class ChainPoolEntry:
    chain_ref: ChainRef
    chain: BoundaryChain
    node: PatchNode


@dataclass(frozen=True)
class ClosureChainPairMatch:
    owner_ref: ChainRef
    owner_chain: BoundaryChain
    target_ref: ChainRef
    target_chain: BoundaryChain
    shared_vert_count: int


@dataclass(frozen=True)
class ClosureChainPairCandidate:
    score: int
    match: ClosureChainPairMatch


@dataclass(frozen=True)
class FrameGroupMember:
    ref: ChainRef
    cross_uv: float
    weight: float
    patch_id: int


@dataclass(frozen=True)
class ClosurePreconstraintOptionResult:
    metric: ClosurePreconstraintMetric
    anchor_label: str
    direction_label: str
    start_anchor: Optional["ChainAnchor"]
    end_anchor: Optional["ChainAnchor"]
    direction_override: Optional[Vector]


@dataclass(frozen=True, order=True)
class ClosureFollowCandidateRank:
    anchor_count: int
    shared_vert_count: int
    chain_length: float


@dataclass(frozen=True, order=True)
class FreeIngressCandidateRank:
    downstream_cross_patch: int
    downstream_count: int
    placed_in_patch: int
    downstream_max_length: float


@dataclass(frozen=True, order=True)
class TreeIngressCandidateRank:
    role_priority: int
    downstream_hv_count: int
    downstream_max_length: float
    chain_length: float


@dataclass(frozen=True, order=True)
class AttachmentCandidatePreference:
    score: float
    frame_continuation: float
    endpoint_bridge: float
    endpoint_strength: float
    best_pair_strength: float
    seam_norm: float


@dataclass(frozen=True)
class DualAnchorRectificationPreview:
    start_anchor: Optional["ChainAnchor"]
    end_anchor: Optional["ChainAnchor"]
    reason: str = ''
    anchor_adjustments: tuple[AnchorAdjustment, ...] = ()


@dataclass(frozen=True)
class ResolvedCandidateAnchors:
    start_anchor: Optional["ChainAnchor"]
    end_anchor: Optional["ChainAnchor"]
    known: int
    reason: str = ''
    anchor_adjustments: tuple[AnchorAdjustment, ...] = ()


@dataclass(frozen=True)
class FoundCandidateAnchors:
    start_anchor: Optional["ChainAnchor"]
    end_anchor: Optional["ChainAnchor"]


@dataclass(frozen=True)
class AnchorPairSafetyDecision:
    is_safe: bool
    reason: str = ''


@dataclass(frozen=True)
class DualAnchorClosureDecision:
    can_close: bool
    reason: str = ''


@dataclass(frozen=True)
class ClosurePreconstraintApplication:
    start_anchor: Optional["ChainAnchor"]
    end_anchor: Optional["ChainAnchor"]
    direction_override: Optional[Vector] = None
    reason: str = ''


@dataclass(frozen=True)
class ClosureFollowUvBuildResult:
    uv_points: Optional[list[Vector]]
    follow_mode: str = ''
    shared_vert_count: int = 0


@dataclass(frozen=True)
class PatchChainGapDiagnostics:
    gap_reports: tuple[ChainGapReport, ...] = ()
    max_chain_gap: float = 0.0


@dataclass(frozen=True)
class ClosureFollowPlacementCandidate:
    rank: ClosureFollowCandidateRank
    chain_ref: ChainRef
    chain: BoundaryChain
    partner_ref: ChainRef
    uv_points: list[Vector]
    follow_mode: str
    shared_vert_count: int


@dataclass(frozen=True)
class FreeIngressPlacementCandidate:
    rank: FreeIngressCandidateRank
    chain_ref: ChainRef
    chain: BoundaryChain
    start_anchor: Optional["ChainAnchor"]
    end_anchor: Optional["ChainAnchor"]
    anchor_adjustments: tuple[AnchorAdjustment, ...]
    uv_points: list[Vector]
    downstream_refs: tuple[ChainRef, ...]
    downstream_cross_patch: int


@dataclass(frozen=True)
class TreeIngressPlacementCandidate:
    rank: TreeIngressCandidateRank
    chain_ref: ChainRef
    chain: BoundaryChain
    start_anchor: Optional["ChainAnchor"]
    end_anchor: Optional["ChainAnchor"]
    anchor_adjustments: tuple[AnchorAdjustment, ...]
    uv_points: list[Vector]
    downstream_hv_count: int
    role_priority: int


@dataclass(frozen=True)
class SeedChainChoice:
    loop_index: int
    chain_index: int
    chain: BoundaryChain
    score: float


@dataclass(frozen=True)
class SeedPlacementResult:
    placement: ScaffoldChainPlacement
    uv_points: list[Vector]


AnchorRefPair = tuple[Optional[ChainRef], Optional[ChainRef]]


@dataclass(frozen=True)
class AttachmentNeighborRef:
    loop_index: int
    chain_index: int
    boundary_loop: BoundaryLoop
    chain: BoundaryChain


@dataclass(frozen=True)
class ChainEndpointContext:
    vert_index: int
    corner: Optional[BoundaryCorner]
    neighbors: tuple[LoopChainRef, ...]


@dataclass(frozen=True)
class EndpointMatch:
    owner_label: str
    target_label: str


@dataclass(frozen=True)
class DirectionOption:
    label: str
    direction_override: Optional[Vector]


@dataclass(frozen=True)
class AnchorOption:
    label: str
    start_anchor: Optional["ChainAnchor"]
    end_anchor: Optional["ChainAnchor"]


@dataclass(frozen=True)
class ChainPairSelection:
    owner_ref: AttachmentNeighborRef
    target_ref: AttachmentNeighborRef
    pair_strength: float
    endpoint_strength: float
    frame_continuation: float
    endpoint_bridge: float
    corner_strength: float
    semantic_strength: float


@dataclass(frozen=True)
class SharedClosureUvOffsets:
    sampled_shared_vert_count: int = 0
    shared_uv_delta_max: float = 0.0
    shared_uv_delta_mean: float = 0.0
    axis_phase_offset_max: float = 0.0
    axis_phase_offset_mean: float = 0.0
    cross_axis_offset_max: float = 0.0
    cross_axis_offset_mean: float = 0.0


@dataclass(frozen=True, order=True)
class ClosurePreconstraintMetric:
    same_patch_gap_max: float = 0.0
    axis_phase_offset_max: float = 0.0
    all_gap_max: float = 0.0
    span_mismatch: float = 0.0
    axis_phase_offset_mean: float = 0.0
    all_gap_mean: float = 0.0


@dataclass(frozen=True)
class EndpointSupportStats:
    fixed_endpoint_count: int
    same_axis_endpoint_count: int
    free_touched_endpoint_count: int


@dataclass(frozen=True)
class EndpointBridgeMetrics:
    endpoint_bridge: float
    corner_strength: float


@dataclass
class QuiltPlan:
    quilt_index: int
    component_index: int
    root_patch_id: int
    root_score: float
    solved_patch_ids: list[int] = field(default_factory=list)
    steps: list[QuiltStep] = field(default_factory=list)
    original_solved_patch_ids: list[int] = field(default_factory=list)
    original_steps: list[QuiltStep] = field(default_factory=list)
    deferred_candidates: list[AttachmentCandidate] = field(default_factory=list)
    rejected_candidates: list[AttachmentCandidate] = field(default_factory=list)
    stop_reason: QuiltStopReason = QuiltStopReason.UNSET


@dataclass
class SolvePlan:
    quilts: list[QuiltPlan] = field(default_factory=list)
    skipped_patch_ids: list[int] = field(default_factory=list)
    propagate_threshold: float = EDGE_PROPAGATE_MIN
    weak_threshold: float = EDGE_WEAK_MIN


@dataclass(frozen=True)
class ClosureCutHeuristic:
    edge_key: PatchEdgeKey
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


def _build_solve_view(graph: PatchGraph) -> SolveView:
    visible_loop_indices_by_patch: dict[int, tuple[int, ...]] = {}
    primary_loop_index_by_patch: dict[int, int] = {}
    locally_solvable_patch_ids: set[int] = set()

    for patch_id, node in graph.nodes.items():
        visible_loop_indices = tuple(
            loop_index
            for loop_index, boundary_loop in enumerate(node.boundary_loops)
            if boundary_loop.kind == LoopKind.OUTER
        )
        visible_loop_indices_by_patch[patch_id] = visible_loop_indices
        if visible_loop_indices:
            primary_loop_index_by_patch[patch_id] = visible_loop_indices[0]

        if visible_loop_indices and len(visible_loop_indices) == 1 and all(loop.chains for loop in node.boundary_loops):
            locally_solvable_patch_ids.add(patch_id)

    return SolveView(
        graph=graph,
        visible_loop_indices_by_patch=visible_loop_indices_by_patch,
        primary_loop_index_by_patch=primary_loop_index_by_patch,
        locally_solvable_patch_ids=frozenset(locally_solvable_patch_ids),
    )


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def _iter_neighbor_chains(graph: PatchGraph, owner_patch_id: int, target_patch_id: int):
    """Compatibility shim during Phase 2 rollout.

    Hot-path solve entry points use SolveView directly. Remaining stateless callers
    can still route through the same central visibility layer here.
    """
    return _build_solve_view(graph).iter_attachment_neighbor_chains(owner_patch_id, target_patch_id)


def _build_solve_components(graph: PatchGraph, candidates: list[AttachmentCandidate]) -> SolveComponentsResult:
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

    return SolveComponentsResult(components=components, component_by_patch=component_by_patch)


def _patch_pair_key(patch_a_id: int, patch_b_id: int) -> PatchEdgeKey:
    return (min(patch_a_id, patch_b_id), max(patch_a_id, patch_b_id))


def _build_quilt_tree_edges(quilt_plan: QuiltPlan) -> set[PatchEdgeKey]:
    edges = set()
    for step in quilt_plan.steps:
        candidate = step.incoming_candidate
        if candidate is None:
            continue
        edges.add(_patch_pair_key(candidate.owner_patch_id, candidate.target_patch_id))
    return edges


def _rebuild_quilt_steps_with_forbidden_edges(
    quilt_plan: QuiltPlan,
    solver_graph: SolverGraph,
    forbidden_edge_keys: set[PatchEdgeKey],
) -> Optional[QuiltPlan]:
    quilt_patch_ids = set(quilt_plan.solved_patch_ids)
    quilt_patch_ids.add(quilt_plan.root_patch_id)
    if len(quilt_patch_ids) <= 1:
        return quilt_plan

    solved_patch_ids: set[int] = {quilt_plan.root_patch_id}
    ordered_patch_ids: list[int] = [quilt_plan.root_patch_id]
    steps: list[QuiltStep] = [
        QuiltStep(step_index=0, patch_id=quilt_plan.root_patch_id, is_root=True)
    ]
    frontier_heap = []
    queue_index = 0

    def enqueue_patch_candidates(patch_id: int) -> None:
        nonlocal queue_index
        for candidate in solver_graph.candidates_by_owner.get(patch_id, ()):
            if candidate.target_patch_id not in quilt_patch_ids:
                continue
            if candidate.target_patch_id in solved_patch_ids:
                continue
            edge_key = _patch_pair_key(candidate.owner_patch_id, candidate.target_patch_id)
            if edge_key in forbidden_edge_keys:
                continue
            heappush(frontier_heap, (-candidate.score, queue_index, candidate))
            queue_index += 1

    enqueue_patch_candidates(quilt_plan.root_patch_id)

    while frontier_heap and len(solved_patch_ids) < len(quilt_patch_ids):
        _, _, candidate = heappop(frontier_heap)
        if candidate.target_patch_id in solved_patch_ids:
            continue
        if candidate.target_patch_id not in quilt_patch_ids:
            continue
        edge_key = _patch_pair_key(candidate.owner_patch_id, candidate.target_patch_id)
        if edge_key in forbidden_edge_keys:
            continue

        solved_patch_ids.add(candidate.target_patch_id)
        ordered_patch_ids.append(candidate.target_patch_id)
        steps.append(
            QuiltStep(
                step_index=len(steps),
                patch_id=candidate.target_patch_id,
                is_root=False,
                incoming_candidate=candidate,
            )
        )
        enqueue_patch_candidates(candidate.target_patch_id)

    if solved_patch_ids != quilt_patch_ids:
        return None

        rebuilt = replace(quilt_plan)
        if not rebuilt.original_steps:
            rebuilt.original_steps = list(quilt_plan.steps)
        if not rebuilt.original_solved_patch_ids:
            rebuilt.original_solved_patch_ids = list(quilt_plan.solved_patch_ids)
        rebuilt.solved_patch_ids = ordered_patch_ids
        rebuilt.steps = steps
        return rebuilt


def _is_allowed_quilt_edge(
    allowed_tree_edges: set[PatchEdgeKey],
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


def _chain_uv_axis_metrics(chain_placement: ScaffoldChainPlacement) -> UvAxisMetrics:
    if len(chain_placement.points) < 2:
        return UvAxisMetrics(span=0.0, axis_error=0.0)

    start_uv = chain_placement.points[0][1]
    end_uv = chain_placement.points[-1][1]
    delta = end_uv - start_uv

    if chain_placement.frame_role == FrameRole.H_FRAME:
        return UvAxisMetrics(span=abs(delta.x), axis_error=abs(delta.y))
    if chain_placement.frame_role == FrameRole.V_FRAME:
        return UvAxisMetrics(span=abs(delta.y), axis_error=abs(delta.x))
    return UvAxisMetrics(span=delta.length, axis_error=0.0)


def _split_uv_by_frame_role(frame_role: FrameRole, uv: Vector) -> FrameUvComponents:
    if frame_role == FrameRole.H_FRAME:
        return FrameUvComponents(axis=uv.x, cross=uv.y)
    if frame_role == FrameRole.V_FRAME:
        return FrameUvComponents(axis=uv.y, cross=uv.x)
    return FrameUvComponents(axis=uv.length, cross=0.0)


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
) -> SharedClosureUvOffsets:
    owner_uv_by_vert = _build_chain_vert_uv_map(graph, owner_placement)
    target_uv_by_vert = _build_chain_vert_uv_map(graph, target_placement)
    shared_vert_indices = sorted(set(owner_uv_by_vert.keys()) & set(target_uv_by_vert.keys()))
    if not shared_vert_indices:
        return SharedClosureUvOffsets()

    uv_deltas = []
    axis_offsets = []
    cross_axis_offsets = []
    for vert_index in shared_vert_indices:
        owner_uv = owner_uv_by_vert[vert_index]
        target_uv = target_uv_by_vert[vert_index]
        owner_components = _split_uv_by_frame_role(owner_placement.frame_role, owner_uv)
        target_components = _split_uv_by_frame_role(target_placement.frame_role, target_uv)
        uv_deltas.append((owner_uv - target_uv).length)
        axis_offsets.append(abs(owner_components.axis - target_components.axis))
        cross_axis_offsets.append(abs(owner_components.cross - target_components.cross))

    sample_count = len(shared_vert_indices)
    return SharedClosureUvOffsets(
        sampled_shared_vert_count=sample_count,
        shared_uv_delta_max=max(uv_deltas),
        shared_uv_delta_mean=sum(uv_deltas) / sample_count,
        axis_phase_offset_max=max(axis_offsets),
        axis_phase_offset_mean=sum(axis_offsets) / sample_count,
        cross_axis_offset_max=max(cross_axis_offsets),
        cross_axis_offset_mean=sum(cross_axis_offsets) / sample_count,
    )


def _classify_closure_anchor_mode(
    owner_anchor_count: int,
    target_anchor_count: int,
) -> ClosureAnchorMode:
    if owner_anchor_count >= 2 and target_anchor_count >= 2:
        return ClosureAnchorMode.DUAL_ANCHOR
    if owner_anchor_count <= 1 and target_anchor_count <= 1:
        return ClosureAnchorMode.ONE_ANCHOR
    return ClosureAnchorMode.MIXED


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
) -> list[ClosureChainPairMatch]:
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
            pair_candidates.append(
                ClosureChainPairCandidate(
                    score=pair_score,
                    match=ClosureChainPairMatch(
                        owner_ref=(owner_patch_id, owner_ref.loop_index, owner_ref.chain_index),
                        owner_chain=owner_chain,
                        target_ref=(target_patch_id, target_ref.loop_index, target_ref.chain_index),
                        target_chain=target_chain,
                        shared_vert_count=shared_vert_count,
                    ),
                )
            )

    pair_candidates.sort(key=lambda item: item.score, reverse=True)
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
            matched_pairs = matched_pairs[1:]
            if not matched_pairs:
                continue

        yield owner_patch_id, target_patch_id, matched_pairs


def _build_quilt_closure_pair_map(
    graph: PatchGraph,
    quilt_patch_ids: set[int],
    allowed_tree_edges: set[PatchEdgeKey],
) -> dict[ChainRef, ChainRef]:
    pair_map = {}
    for owner_patch_id, target_patch_id, matched_pairs in _iter_quilt_closure_chain_pairs(
        graph,
        quilt_patch_ids,
        allowed_tree_edges,
    ):
        for match in matched_pairs:
            pair_map[match.owner_ref] = match.target_ref
            pair_map[match.target_ref] = match.owner_ref
    return pair_map


def _collect_quilt_closure_seam_reports(
    graph: PatchGraph,
    quilt_plan: QuiltPlan,
    quilt_scaffold: ScaffoldQuiltPlacement,
    placed_chains_map: dict[ChainRef, ScaffoldChainPlacement],
    final_scale: float,
    allowed_tree_edges: set[PatchEdgeKey],
) -> tuple[ScaffoldClosureSeamReport, ...]:
    quilt_patch_ids = set(quilt_plan.solved_patch_ids)
    quilt_patch_ids.add(quilt_plan.root_patch_id)
    patch_tree_adjacency = _build_patch_tree_adjacency(quilt_plan)

    reports = []
    for owner_patch_id, target_patch_id, matched_pairs in _iter_quilt_closure_chain_pairs(
        graph,
        quilt_patch_ids,
        allowed_tree_edges,
    ):
        patch_path = _find_patch_tree_path(patch_tree_adjacency, owner_patch_id, target_patch_id)
        tree_patch_distance = max(0, len(patch_path) - 1) if patch_path else 0
        free_bridge_count = _count_free_bridges_on_patch_path(quilt_scaffold, patch_path) if patch_path else 0

        for match in matched_pairs:
            owner_placement = placed_chains_map.get(match.owner_ref)
            target_placement = placed_chains_map.get(match.target_ref)
            if owner_placement is None or target_placement is None:
                continue
            if len(owner_placement.points) < 2 or len(target_placement.points) < 2:
                continue

            owner_axis_metrics = _chain_uv_axis_metrics(owner_placement)
            target_axis_metrics = _chain_uv_axis_metrics(target_placement)
            shared_offsets = _measure_shared_closure_uv_offsets(graph, owner_placement, target_placement)
            canonical_3d_span = (
                _cf_chain_total_length(match.owner_chain, final_scale)
                + _cf_chain_total_length(match.target_chain, final_scale)
            ) * 0.5

            reports.append(ScaffoldClosureSeamReport(
                owner_patch_id=match.owner_ref[0],
                owner_loop_index=match.owner_ref[1],
                owner_chain_index=match.owner_ref[2],
                target_patch_id=match.target_ref[0],
                target_loop_index=match.target_ref[1],
                target_chain_index=match.target_ref[2],
                frame_role=match.owner_chain.frame_role,
                owner_anchor_count=owner_placement.anchor_count,
                target_anchor_count=target_placement.anchor_count,
                anchor_mode=_classify_closure_anchor_mode(owner_placement.anchor_count, target_placement.anchor_count),
                canonical_3d_span=canonical_3d_span,
                owner_uv_span=owner_axis_metrics.span,
                target_uv_span=target_axis_metrics.span,
                owner_axis_error=owner_axis_metrics.axis_error,
                target_axis_error=target_axis_metrics.axis_error,
                span_mismatch=abs(owner_axis_metrics.span - target_axis_metrics.span),
                sampled_shared_vert_count=shared_offsets.sampled_shared_vert_count,
                shared_uv_delta_max=shared_offsets.shared_uv_delta_max,
                shared_uv_delta_mean=shared_offsets.shared_uv_delta_mean,
                axis_phase_offset_max=shared_offsets.axis_phase_offset_max,
                axis_phase_offset_mean=shared_offsets.axis_phase_offset_mean,
                cross_axis_offset_max=shared_offsets.cross_axis_offset_max,
                cross_axis_offset_mean=shared_offsets.cross_axis_offset_mean,
                tree_patch_distance=tree_patch_distance,
                free_bridge_count=free_bridge_count,
                shared_vert_count=match.shared_vert_count,
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

    base_direction = _try_inherit_direction(chain, node, start_anchor, end_anchor, graph, point_registry)
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


def _cf_try_place_closure_follow_candidate(
    runtime_policy: FrontierRuntimePolicy,
    all_chain_pool: list[ChainPoolEntry],
    closure_pair_map: dict[ChainRef, ChainRef],
    iteration: int,
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
            chain_length=_cf_chain_total_length(chain, runtime_policy.final_scale),
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

    print(
        f"[CFTUV][Frontier] Step {iteration}: "
        f"P{chain_ref[0]} L{chain_ref[1]}C{chain_ref[2]} "
        f"{chain.frame_role.value} score:closure_follow ep:2 "
        f"a:CP{partner_ref[0]}/CP{partner_ref[0]} "
        f"note:{follow_mode}:shared={shared_vert_count}"
    )
    return True


def _cf_try_place_free_ingress_bridge(
    runtime_policy: FrontierRuntimePolicy,
    all_chain_pool: list[ChainPoolEntry],
    iteration: int,
) -> bool:
    """Локально прогрызает frontier через one-edge FREE bridge к downstream H/V."""
    graph = runtime_policy.graph
    best_candidate: Optional[FreeIngressPlacementCandidate] = None

    for entry in all_chain_pool:
        chain_ref = entry.chain_ref
        chain = entry.chain
        node = entry.node
        if chain_ref in runtime_policy.placed_chain_refs or chain_ref in runtime_policy.rejected_chain_refs:
            continue
        if chain.frame_role != FrameRole.FREE or len(chain.vert_cos) > 2:
            continue

        candidate_eval = runtime_policy.evaluate_candidate(chain_ref, chain, node)
        if candidate_eval.known != 1:
            continue

        resolved_anchor = candidate_eval.start_anchor if candidate_eval.start_anchor is not None else candidate_eval.end_anchor
        if resolved_anchor is None or resolved_anchor.source_kind != PlacementSourceKind.SAME_PATCH:
            continue

        endpoint_neighbors = graph.get_chain_endpoint_neighbors(chain_ref[0], chain_ref[1], chain_ref[2])
        downstream_side = 'end' if candidate_eval.start_anchor is not None else 'start'
        downstream_refs = []
        downstream_cross_patch = 0
        downstream_max_length = 0.0

        for neighbor_loop_index, neighbor_chain_index in endpoint_neighbors.get(downstream_side, []):
            neighbor_ref = (chain_ref[0], neighbor_loop_index, neighbor_chain_index)
            if neighbor_ref in runtime_policy.placed_chain_refs or neighbor_ref in runtime_policy.rejected_chain_refs:
                continue
            neighbor_chain = graph.get_chain(*neighbor_ref)
            if neighbor_chain is None or neighbor_chain.frame_role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
                continue
            downstream_refs.append(neighbor_ref)
            downstream_max_length = max(
                downstream_max_length,
                _cf_chain_total_length(neighbor_chain, runtime_policy.final_scale),
            )
            if (
                neighbor_chain.neighbor_kind == ChainNeighborKind.PATCH
                and neighbor_chain.neighbor_patch_id in runtime_policy.quilt_patch_ids
                and _is_allowed_quilt_edge(runtime_policy.allowed_tree_edges, chain_ref[0], neighbor_chain.neighbor_patch_id)
            ):
                downstream_cross_patch += 1

        if not downstream_refs:
            continue

        direction_override = _try_inherit_direction(
            chain,
            node,
            candidate_eval.start_anchor,
            candidate_eval.end_anchor,
            graph,
            runtime_policy.point_registry,
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

        candidate_rank = FreeIngressCandidateRank(
            downstream_cross_patch=downstream_cross_patch,
            downstream_count=len(downstream_refs),
            placed_in_patch=candidate_eval.placed_in_patch,
            downstream_max_length=downstream_max_length,
        )
        if best_candidate is None or candidate_rank > best_candidate.rank:
            best_candidate = FreeIngressPlacementCandidate(
                rank=candidate_rank,
                chain_ref=chain_ref,
                chain=chain,
                start_anchor=candidate_eval.start_anchor,
                end_anchor=candidate_eval.end_anchor,
                anchor_adjustments=tuple(candidate_eval.anchor_adjustments),
                uv_points=uv_points,
                downstream_refs=tuple(downstream_refs),
                downstream_cross_patch=downstream_cross_patch,
            )

    if best_candidate is None:
        return False

    chain_ref = best_candidate.chain_ref
    chain = best_candidate.chain
    anchor_start = best_candidate.start_anchor
    anchor_end = best_candidate.end_anchor
    anchor_adjustments = best_candidate.anchor_adjustments
    uv_points = best_candidate.uv_points
    downstream_refs = best_candidate.downstream_refs
    downstream_cross_patch = best_candidate.downstream_cross_patch

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

    runtime_policy.register_chain(chain_ref, chain, chain_placement, uv_points, ())

    print(
        f"[CFTUV][Frontier] Step {iteration}: "
        f"P{chain_ref[0]} L{chain_ref[1]}C{chain_ref[2]} "
        f"{chain.frame_role.value} score:free_ingress ep:1 "
        f"a:{_cf_anchor_debug_label(anchor_start, anchor_end)} "
        f"note:downstream_hv={len(downstream_refs)}:cross={downstream_cross_patch} [BRIDGE]"
    )
    return True


def _cf_try_place_tree_ingress_candidate(
    runtime_policy: FrontierRuntimePolicy,
    all_chain_pool: list[ChainPoolEntry],
    iteration: int,
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

        role_priority = 0
        if chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
            role_priority = 3 if not chain.is_corner_split else 2
        elif chain.frame_role == FrameRole.FREE and len(chain.vert_cos) <= 2:
            role_priority = 1
        else:
            continue

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

    print(
        f"[CFTUV][Frontier] Step {iteration}: "
        f"P{chain_ref[0]} L{chain_ref[1]}C{chain_ref[2]} "
        f"{chain.frame_role.value} score:tree_ingress ep:1 "
        f"a:{_cf_anchor_debug_label(anchor_start, anchor_end)} "
        f"note:priority={role_priority}:downstream_hv={downstream_hv_count}"
    )
    return True


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
            f"{report.frame_role.value} mode:{report.anchor_mode.value} "
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


def _wall_side_row_class_key(chain: BoundaryChain) -> RowClassKey:
    if not chain.vert_cos:
        return (0,)
    avg_z = sum(point.z for point in chain.vert_cos) / float(len(chain.vert_cos))
    return (int(round(avg_z / FRAME_ROW_GROUP_TOLERANCE)),)


def _wall_side_column_class_key(chain: BoundaryChain) -> ColumnClassKey:
    if not chain.vert_cos:
        return (0, 0)
    avg_x = sum(point.x for point in chain.vert_cos) / float(len(chain.vert_cos))
    avg_y = sum(point.y for point in chain.vert_cos) / float(len(chain.vert_cos))
    return (
        int(round(avg_x / FRAME_COLUMN_GROUP_TOLERANCE)),
        int(round(avg_y / FRAME_COLUMN_GROUP_TOLERANCE)),
    )


def _frame_group_display_coords(
    axis_kind: FrameAxisKind,
    class_key: FrameClassKey,
) -> FrameGroupDisplayCoords:
    if axis_kind == FrameAxisKind.ROW:
        return FrameGroupDisplayCoords(
            coord_a=class_key[0] * FRAME_ROW_GROUP_TOLERANCE,
            coord_b=0.0,
        )
    if axis_kind == FrameAxisKind.COLUMN:
        return FrameGroupDisplayCoords(
            coord_a=class_key[0] * FRAME_COLUMN_GROUP_TOLERANCE,
            coord_b=class_key[1] * FRAME_COLUMN_GROUP_TOLERANCE,
        )
    return FrameGroupDisplayCoords(coord_a=0.0, coord_b=0.0)


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
    grouped_members: dict[
        tuple[FrameAxisKind, str, FrameClassKey],
        list[FrameGroupMember],
    ] = {}

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

            axis_kind = (
                FrameAxisKind.ROW
                if chain_placement.frame_role == FrameRole.H_FRAME
                else FrameAxisKind.COLUMN
            )
            if axis_kind == FrameAxisKind.ROW:
                class_key = _wall_side_row_class_key(chain)
            else:
                class_key = _wall_side_column_class_key(chain)
            ref = (patch_id, patch_placement.loop_index, chain_placement.chain_index)
            cross_uv = _frame_cross_axis_uv_value(chain_placement)
            weight = max(_cf_chain_total_length(chain, final_scale), 1e-8)
            grouped_members.setdefault((axis_kind, node.semantic_key, class_key), []).append(
                FrameGroupMember(
                    ref=ref,
                    cross_uv=cross_uv,
                    weight=weight,
                    patch_id=patch_id,
                )
            )

    reports = []
    for (axis_kind, semantic_key, class_key), members in grouped_members.items():
        if len(members) < 2:
            continue

        total_weight = sum(member.weight for member in members)
        if total_weight <= 1e-8:
            continue
        target_cross_uv = sum(member.cross_uv * member.weight for member in members) / total_weight
        scatter_values = [abs(member.cross_uv - target_cross_uv) for member in members]
        display_coords = _frame_group_display_coords(axis_kind, class_key)

        reports.append(ScaffoldFrameAlignmentReport(
            axis_kind=axis_kind,
            semantic_key=semantic_key,
            frame_role=FrameRole.H_FRAME if axis_kind == FrameAxisKind.ROW else FrameRole.V_FRAME,
            class_coord_a=display_coords.coord_a,
            class_coord_b=display_coords.coord_b,
            chain_count=len(members),
            total_weight=total_weight,
            target_cross_uv=target_cross_uv,
            scatter_max=max(scatter_values),
            scatter_mean=sum(scatter_values) / float(len(scatter_values)),
            closure_sensitive=any(member.patch_id in closure_sensitive_patch_ids for member in members),
            member_refs=tuple(sorted(member.ref for member in members)),
        ))

    reports.sort(
        key=lambda report: (
            -report.scatter_max,
            -report.scatter_mean,
            report.axis_kind.value,
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
        (report.scatter_max for report in frame_alignment_reports if report.axis_kind == FrameAxisKind.ROW),
        default=0.0,
    )
    max_column_scatter = max(
        (report.scatter_max for report in frame_alignment_reports if report.axis_kind == FrameAxisKind.COLUMN),
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
            if report.axis_kind == FrameAxisKind.ROW
            else f"xy:({report.class_coord_a:.6f},{report.class_coord_b:.6f})"
        )
        refs_label = ','.join(
            f"P{patch_id}L{loop_index}C{chain_index}"
            for patch_id, loop_index, chain_index in report.member_refs
        )
        closure_tag = " closure:1" if report.closure_sensitive else " closure:0"
        print(
            f"[CFTUV][FrameDiag] Quilt {quilt_index} "
            f"{report.axis_kind.value} {report.frame_role.value} {coord_label} "
            f"chains:{report.chain_count} target:{report.target_cross_uv:.6f} "
            f"scatter:{report.scatter_max:.6f}/{report.scatter_mean:.6f} "
            f"weight:{report.total_weight:.6f}{closure_tag} refs:[{refs_label}]"
        )


def _count_patch_roles(node: PatchNode) -> PatchRoleCounts:
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

    return PatchRoleCounts(
        outer_count=outer_count,
        hole_count=hole_count,
        chain_count=chain_count,
        free_count=free_count,
        h_count=h_count,
        v_count=v_count,
    )


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


def _build_patch_certainty(node: PatchNode, solve_view: SolveView, max_area: float) -> PatchCertainty:
    role_counts = _count_patch_roles(node)
    local_solvable = solve_view.patch_is_locally_solvable(node.patch_id)
    if not local_solvable:
        return PatchCertainty(
            patch_id=node.patch_id,
            local_solvable=False,
            root_score=0.0,
            outer_count=role_counts.outer_count,
            hole_count=role_counts.hole_count,
            chain_count=role_counts.chain_count,
            free_count=role_counts.free_count,
            h_count=role_counts.h_count,
            v_count=role_counts.v_count,
            reasons=("local_gate=fail",),
        )

    area_norm = 0.0 if max_area <= 0.0 else _clamp01(node.area / max_area)
    free_ratio = 1.0 if role_counts.chain_count <= 0 else float(role_counts.free_count) / float(role_counts.chain_count)
    frame_strength = _frame_presence_strength(role_counts.h_count, role_counts.v_count)
    hole_factor = _clamp01(1.0 - min(role_counts.hole_count, 2) * 0.35)
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
        outer_count=role_counts.outer_count,
        hole_count=role_counts.hole_count,
        chain_count=role_counts.chain_count,
        free_count=role_counts.free_count,
        h_count=role_counts.h_count,
        v_count=role_counts.v_count,
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


def _get_chain_endpoint_context(
    graph: PatchGraph,
    patch_id: int,
    loop_index: int,
    chain_index: int,
    endpoint_label: str,
) -> Optional[ChainEndpointContext]:
    chain = graph.get_chain(patch_id, loop_index, chain_index)
    if chain is None:
        return None

    endpoint_neighbors = graph.get_chain_endpoint_neighbors(patch_id, loop_index, chain_index)
    if endpoint_label == 'start':
        return ChainEndpointContext(
            vert_index=chain.start_vert_index,
            corner=_get_chain_corner(graph, patch_id, loop_index, chain.start_corner_index),
            neighbors=tuple(endpoint_neighbors.get('start', ())),
        )
    return ChainEndpointContext(
        vert_index=chain.end_vert_index,
        corner=_get_chain_corner(graph, patch_id, loop_index, chain.end_corner_index),
        neighbors=tuple(endpoint_neighbors.get('end', ())),
    )


def _corner_similarity_strength(owner_corner, target_corner) -> float:
    if owner_corner is None or target_corner is None:
        return 0.0
    angle_delta = abs(owner_corner.turn_angle_deg - target_corner.turn_angle_deg)
    similarity = _clamp01(1.0 - (angle_delta / 90.0))
    sharp_avg = _clamp01(((owner_corner.turn_angle_deg / 90.0) + (target_corner.turn_angle_deg / 90.0)) * 0.5)
    return _clamp01(0.5 * similarity + 0.5 * sharp_avg)


def _find_endpoint_matches(owner_chain: BoundaryChain, target_chain: BoundaryChain) -> list[EndpointMatch]:
    owner_endpoints = (('start', owner_chain.start_vert_index), ('end', owner_chain.end_vert_index))
    target_endpoints = (('start', target_chain.start_vert_index), ('end', target_chain.end_vert_index))
    matches: list[EndpointMatch] = []
    for owner_label, owner_vert in owner_endpoints:
        if owner_vert < 0:
            continue
        for target_label, target_vert in target_endpoints:
            if owner_vert != target_vert or target_vert < 0:
                continue
            matches.append(EndpointMatch(
                owner_label=owner_label,
                target_label=target_label,
            ))
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
) -> EndpointBridgeMetrics:
    endpoint_matches = _find_endpoint_matches(owner_chain, target_chain)
    if not endpoint_matches:
        return EndpointBridgeMetrics(endpoint_bridge=0.0, corner_strength=0.0)

    bridge_scores = []
    corner_scores = []
    for endpoint_match in endpoint_matches:
        owner_ctx = _get_chain_endpoint_context(
            graph, owner_patch_id, owner_loop_index, owner_chain_index, endpoint_match.owner_label)
        target_ctx = _get_chain_endpoint_context(
            graph, target_patch_id, target_loop_index, target_chain_index, endpoint_match.target_label)
        if owner_ctx is None or target_ctx is None:
            continue

        corner_strength = _corner_similarity_strength(owner_ctx.corner, target_ctx.corner)
        corner_scores.append(corner_strength)

        best_neighbor_bridge = 0.0
        for owner_ref in owner_ctx.neighbors:
            owner_neighbor_chain = graph.get_chain(owner_patch_id, owner_ref[0], owner_ref[1])
            if owner_neighbor_chain is None:
                continue
            for target_ref in target_ctx.neighbors:
                target_neighbor_chain = graph.get_chain(target_patch_id, target_ref[0], target_ref[1])
                if target_neighbor_chain is None:
                    continue
                best_neighbor_bridge = max(
                    best_neighbor_bridge,
                    _frame_continuation_strength(owner_neighbor_chain.frame_role, target_neighbor_chain.frame_role),
                )

        bridge_scores.append(_clamp01(0.75 * best_neighbor_bridge + 0.25 * corner_strength))

    if not bridge_scores:
        return EndpointBridgeMetrics(endpoint_bridge=0.0, corner_strength=0.0)
    return EndpointBridgeMetrics(
        endpoint_bridge=_clamp01(sum(bridge_scores) / len(bridge_scores)),
        corner_strength=_clamp01(sum(corner_scores) / len(corner_scores)),
    )


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
    owner_refs: list[AttachmentNeighborRef],
    target_patch_id: int,
    target_refs: list[AttachmentNeighborRef],
) -> Optional[ChainPairSelection]:
    best_pair: Optional[ChainPairSelection] = None
    best_strength = -1.0
    semantic_strength = _semantic_pair_strength(graph, owner_patch_id, target_patch_id)

    for owner_ref in owner_refs:
        owner_endpoint = _chain_endpoint_strength(
            graph, owner_patch_id, owner_ref.loop_index, owner_ref.chain_index, owner_ref.chain)
        for target_ref in target_refs:
            target_endpoint = _chain_endpoint_strength(
                graph, target_patch_id, target_ref.loop_index, target_ref.chain_index, target_ref.chain)
            endpoint_strength = (owner_endpoint + target_endpoint) * 0.5
            frame_continuation = _frame_continuation_strength(owner_ref.chain.frame_role, target_ref.chain.frame_role)
            bridge_metrics = _endpoint_bridge_strength(
                graph,
                owner_patch_id,
                owner_ref.loop_index,
                owner_ref.chain_index,
                owner_ref.chain,
                target_patch_id,
                target_ref.loop_index,
                target_ref.chain_index,
                target_ref.chain,
            )
            endpoint_bridge = bridge_metrics.endpoint_bridge
            corner_strength = bridge_metrics.corner_strength
            pair_strength = (
                PAIR_WEIGHT_FRAME_CONT * frame_continuation
                + PAIR_WEIGHT_ENDPOINT * endpoint_bridge
                + PAIR_WEIGHT_CORNER * corner_strength
                + PAIR_WEIGHT_SEMANTIC * semantic_strength
                + PAIR_WEIGHT_EP_STRENGTH * endpoint_strength
                + PAIR_WEIGHT_LOOP * _loop_pair_strength(owner_ref.boundary_loop.kind, target_ref.boundary_loop.kind)
            )
            if pair_strength <= best_strength:
                continue
            best_strength = pair_strength
            best_pair = ChainPairSelection(
                owner_ref=owner_ref,
                target_ref=target_ref,
                pair_strength=_clamp01(pair_strength),
                endpoint_strength=_clamp01(endpoint_strength),
                frame_continuation=_clamp01(frame_continuation),
                endpoint_bridge=_clamp01(endpoint_bridge),
                corner_strength=_clamp01(corner_strength),
                semantic_strength=_clamp01(semantic_strength),
            )

    return best_pair


def _build_attachment_candidate(
    solve_view: SolveView,
    owner_patch_id: int,
    target_patch_id: int,
    seam,
    patch_scores: dict[int, PatchCertainty],
    max_shared_length: float,
) -> Optional[AttachmentCandidate]:
    graph = solve_view.graph
    owner_score = patch_scores[owner_patch_id]
    target_score = patch_scores[target_patch_id]
    if not solve_view.patch_types_compatible(owner_patch_id, target_patch_id):
        return None
    owner_refs = solve_view.iter_attachment_neighbor_chains(owner_patch_id, target_patch_id)
    target_refs = solve_view.iter_attachment_neighbor_chains(target_patch_id, owner_patch_id)

    if not owner_refs or not target_refs:
        return None

    best_pair = _best_chain_pair(
        graph,
        owner_patch_id,
        owner_refs,
        target_patch_id,
        target_refs,
    )
    if best_pair is None:
        return None

    owner_ref = best_pair.owner_ref
    target_ref = best_pair.target_ref
    owner_loop_index = owner_ref.loop_index
    owner_chain_index = owner_ref.chain_index
    owner_loop = owner_ref.boundary_loop
    owner_chain = owner_ref.chain
    target_loop_index = target_ref.loop_index
    target_chain_index = target_ref.chain_index
    target_loop = target_ref.boundary_loop
    target_chain = target_ref.chain

    seam_norm = 1.0 if max_shared_length <= 0.0 else _clamp01(seam.shared_length / max_shared_length)
    ambiguity_penalty = min(0.15, 0.05 * max(0, len(owner_refs) - 1) + 0.05 * max(0, len(target_refs) - 1))

    if not owner_score.local_solvable or not target_score.local_solvable:
        total_score = 0.0
    else:
        total_score = (
            ATTACH_WEIGHT_SEAM * seam_norm
            + ATTACH_WEIGHT_PAIR * best_pair.pair_strength
            + ATTACH_WEIGHT_TARGET * target_score.root_score
            + ATTACH_WEIGHT_OWNER * owner_score.root_score
            - ambiguity_penalty
        )
        owner_is_frame = owner_chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
        target_is_frame = target_chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}
        if owner_is_frame ^ target_is_frame:
            total_score -= max(0.0, 0.12 - 0.10 * best_pair.endpoint_bridge)
        elif not owner_is_frame and not target_is_frame:
            total_score -= max(0.0, 0.25 - 0.20 * best_pair.endpoint_bridge)
        if target_score.hole_count > 0:
            total_score -= min(0.08, target_score.hole_count * 0.04)
        total_score = _clamp01(total_score)

    reasons = (
        f"seam={seam_norm:.2f}",
        f"pair={best_pair.pair_strength:.2f}",
        f"cont={best_pair.frame_continuation:.2f}",
        f"bridge={best_pair.endpoint_bridge:.2f}",
        f"corner={best_pair.corner_strength:.2f}",
        f"sem={best_pair.semantic_strength:.2f}",
        f"ep={best_pair.endpoint_strength:.2f}",
        f"target={target_score.root_score:.2f}",
        f"amb={ambiguity_penalty:.2f}",
    )
    return AttachmentCandidate(
        owner_patch_id=owner_patch_id,
        target_patch_id=target_patch_id,
        score=total_score,
        seam_length=seam.shared_length,
        seam_norm=seam_norm,
        best_pair_strength=best_pair.pair_strength,
        frame_continuation=best_pair.frame_continuation,
        endpoint_bridge=best_pair.endpoint_bridge,
        corner_strength=best_pair.corner_strength,
        semantic_strength=best_pair.semantic_strength,
        endpoint_strength=best_pair.endpoint_strength,
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
    solve_view = _build_solve_view(graph)

    max_area = max((node.area for node in graph.nodes.values()), default=0.0)
    for patch_id, node in graph.nodes.items():
        solver_graph.patch_scores[patch_id] = _build_patch_certainty(node, solve_view, max_area)

    solver_graph.max_shared_length = max((edge.shared_length for edge in graph.edges.values()), default=0.0)
    for seam in graph.edges.values():
        forward = _build_attachment_candidate(
            solve_view,
            seam.patch_a_id,
            seam.patch_b_id,
            seam,
            solver_graph.patch_scores,
            solver_graph.max_shared_length,
        )
        backward = _build_attachment_candidate(
            solve_view,
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
    components_result = _build_solve_components(graph, solver_graph.candidates)
    solver_graph.solve_components = components_result.components
    solver_graph.component_by_patch = components_result.component_by_patch
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
                quilt.stop_reason = QuiltStopReason.FRONTIER_BELOW_THRESHOLD
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

        if quilt.stop_reason == QuiltStopReason.UNSET:
            quilt.stop_reason = QuiltStopReason.FRONTIER_EXHAUSTED
        quilt = _apply_quilt_closure_cut_recommendations(graph, solver_graph, quilt)
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



def _compute_patch_uv_bbox(bm, graph: PatchGraph, uv_layer, patch_ids: list[int]) -> UvBounds:
    face_indices = _collect_patch_face_indices(graph, patch_ids)
    points = []
    for face in bm.faces:
        if face.index not in face_indices:
            continue
        for loop in face.loops:
            points.append(loop[uv_layer].uv.copy())
    if not points:
        return UvBounds(bbox_min=Vector((0.0, 0.0)), bbox_max=Vector((0.0, 0.0)))
    min_x = min(point.x for point in points)
    min_y = min(point.y for point in points)
    max_x = max(point.x for point in points)
    max_y = max(point.y for point in points)
    return UvBounds(
        bbox_min=Vector((min_x, min_y)),
        bbox_max=Vector((max_x, max_y)),
    )



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



def _scaffold_key_id(point_key: ScaffoldPointKey, source_kind: PlacementSourceKind) -> ScaffoldKeyId:
    return (
        point_key.patch_id,
        point_key.loop_index,
        point_key.chain_index,
        point_key.source_point_index,
        source_kind,
    )



def _resolve_scaffold_uv_targets(
    bm,
    graph: PatchGraph,
    key: ScaffoldPointKey,
    source_kind: PlacementSourceKind,
) -> list[ScaffoldUvTarget]:
    node = graph.nodes.get(key.patch_id)
    if node is None or key.loop_index < 0 or key.loop_index >= len(node.boundary_loops):
        return []

    boundary_loop = node.boundary_loops[key.loop_index]
    loop_count = len(boundary_loop.vert_indices)
    if loop_count <= 0 or not boundary_loop.side_face_indices:
        return []

    chain = None
    if source_kind == PlacementSourceKind.CHAIN:
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
    seen: set[TransferTargetId] = set()

    def _add_target(face_index: int) -> None:
        target_id: TransferTargetId = (face_index, vert_index)
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
    scaffold_keys: ScaffoldKeySet = set()
    for chain_placement in patch_placement.chain_placements:
        for point_key, _ in chain_placement.points:
            scaffold_keys.add(_scaffold_key_id(point_key, chain_placement.source_kind))
    return len(scaffold_keys)


class PatchTransferStatus(str, Enum):
    OK = 'ok'
    UNSUPPORTED = 'unsupported'
    INVALID_SCAFFOLD = 'invalid_scaffold'
    MISSING_PATCH = 'missing_patch'


@dataclass(frozen=True)
class PatchTransferTargetsState:
    status: PatchTransferStatus = PatchTransferStatus.OK
    scaffold_points: int = 0
    resolved_scaffold_points: int = 0
    uv_targets_resolved: int = 0
    unresolved_scaffold_points: int = 0
    missing_uv_targets: int = 0
    conflicting_uv_targets: int = 0
    pinned_uv_targets: int = 0
    unpinned_uv_targets: int = 0
    invalid_scaffold_patches: int = 0
    closure_error: float = 0.0
    max_chain_gap: float = 0.0
    chain_gap_count: int = 0
    target_samples: TargetSampleMap = field(default_factory=dict)
    pin_target_ids: PinnedTargetIdSet = field(default_factory=set)
    scaffold_keys: ScaffoldKeySet = field(default_factory=set)
    unresolved_keys: ScaffoldKeySet = field(default_factory=set)
    conformal_patch: bool = False


@dataclass(frozen=True)
class PatchApplyStats:
    status: PatchTransferStatus = PatchTransferStatus.OK
    scaffold_points: int = 0
    resolved_scaffold_points: int = 0
    uv_targets_resolved: int = 0
    unresolved_scaffold_points: int = 0
    missing_uv_targets: int = 0
    conflicting_uv_targets: int = 0
    pinned_uv_loops: int = 0
    invalid_scaffold_patches: int = 0
    closure_error: float = 0.0
    max_chain_gap: float = 0.0
    chain_gap_count: int = 0

    def accumulate_into(self, bucket: dict[str, int]) -> None:
        bucket['scaffold_points'] += int(self.scaffold_points)
        bucket['resolved_scaffold_points'] += int(self.resolved_scaffold_points)
        bucket['uv_targets_resolved'] += int(self.uv_targets_resolved)
        bucket['unresolved_scaffold_points'] += int(self.unresolved_scaffold_points)
        bucket['missing_uv_targets'] += int(self.missing_uv_targets)
        bucket['conflicting_uv_targets'] += int(self.conflicting_uv_targets)
        bucket['pinned_uv_loops'] += int(self.pinned_uv_loops)
        bucket['invalid_scaffold_patches'] += int(self.invalid_scaffold_patches)


def _build_patch_transfer_targets(
    bm,
    graph: PatchGraph,
    patch_placement: ScaffoldPatchPlacement,
    uv_offset: Vector,
) -> PatchTransferTargetsState:
    scaffold_point_count = _count_patch_scaffold_points(patch_placement)
    base_kwargs = {
        'scaffold_points': scaffold_point_count,
        'closure_error': patch_placement.closure_error,
        'max_chain_gap': patch_placement.max_chain_gap,
        'chain_gap_count': len(patch_placement.gap_reports),
    }

    if patch_placement.notes:
        return PatchTransferTargetsState(status=PatchTransferStatus.UNSUPPORTED, **base_kwargs)
    if not patch_placement.closure_valid:
        return PatchTransferTargetsState(
            status=PatchTransferStatus.INVALID_SCAFFOLD,
            invalid_scaffold_patches=1,
            **base_kwargs,
        )

    node = graph.nodes.get(patch_placement.patch_id)
    if node is None:
        return PatchTransferTargetsState(
            status=PatchTransferStatus.MISSING_PATCH,
            scaffold_points=0,
            closure_error=patch_placement.closure_error,
            max_chain_gap=patch_placement.max_chain_gap,
            chain_gap_count=len(patch_placement.gap_reports),
        )

    conformal_patch = all(
        cp.frame_role == FrameRole.FREE for cp in patch_placement.chain_placements
    )
    target_samples: TargetSampleMap = {}
    pin_target_ids: PinnedTargetIdSet = set()
    scaffold_keys: ScaffoldKeySet = set()
    unresolved_keys: ScaffoldKeySet = set()
    scaffold_connected_set = patch_placement.scaffold_connected_chains

    for chain_placement in patch_placement.chain_placements:
        point_count = len(chain_placement.points)
        is_connected = chain_placement.chain_index in scaffold_connected_set
        boundary_loop = None
        if 0 <= chain_placement.loop_index < len(node.boundary_loops):
            boundary_loop = node.boundary_loops[chain_placement.loop_index]
        for point_index, (point_key, target_uv) in enumerate(chain_placement.points):
            key_id = _scaffold_key_id(point_key, chain_placement.source_kind)
            scaffold_keys.add(key_id)
            targets = _resolve_scaffold_uv_targets(bm, graph, point_key, chain_placement.source_kind)
            if not targets:
                unresolved_keys.add(key_id)
                continue

            shifted_uv = target_uv + uv_offset
            should_pin = _should_pin_scaffold_point(
                chain_placement,
                point_index,
                point_count,
                conformal_patch,
                scaffold_connected=is_connected,
            )
            if (
                should_pin
                and chain_placement.frame_role == FrameRole.FREE
                and boundary_loop is not None
                and not _free_endpoint_has_local_frame_anchor(
                    boundary_loop,
                    chain_placement.chain_index,
                    point_index,
                    scaffold_connected_set,
                )
            ):
                should_pin = False
            for target in targets:
                target_id: TransferTargetId = (target.face_index, target.vert_index)
                target_samples.setdefault(target_id, []).append(shifted_uv.copy())
                if should_pin:
                    pin_target_ids.add(target_id)

    conflicting_uv_targets = 0
    for samples in target_samples.values():
        base_sample = samples[0]
        if any((sample - base_sample).length > 1e-5 for sample in samples[1:]):
            conflicting_uv_targets += 1

    return PatchTransferTargetsState(
        status=PatchTransferStatus.OK,
        scaffold_points=scaffold_point_count,
        resolved_scaffold_points=len(scaffold_keys) - len(unresolved_keys),
        uv_targets_resolved=len(target_samples),
        unresolved_scaffold_points=len(unresolved_keys),
        conflicting_uv_targets=conflicting_uv_targets,
        pinned_uv_targets=len(pin_target_ids),
        unpinned_uv_targets=max(0, len(target_samples) - len(pin_target_ids)),
        closure_error=patch_placement.closure_error,
        max_chain_gap=patch_placement.max_chain_gap,
        chain_gap_count=len(patch_placement.gap_reports),
        target_samples=target_samples,
        pin_target_ids=pin_target_ids,
        scaffold_keys=scaffold_keys,
        unresolved_keys=unresolved_keys,
        conformal_patch=conformal_patch,
    )



def _should_pin_scaffold_point(chain_placement: ScaffoldChainPlacement, point_index: int, point_count: int, conformal_patch: bool = False, scaffold_connected: bool = True) -> bool:
    """Pin policy для scaffold points.

    conformal_patch=True (patch целиком ушёл в FREE):
      Ничего не пинить — Conformal unwrap обработает весь patch свободно.

    H_FRAME/V_FRAME: пинятся только если scaffold_connected=True
      (связаны с seed chain через непрерывную цепочку H/V chains).
      Изолированные H/V за FREE-разрывом не пинятся — они не часть
      основного scaffold каркаса и будут обработаны Conformal.

    FREE chains в mixed patch:
      пинятся только endpoints.
      Intermediate points должны оставаться свободными, чтобы реальный shape
      добирался финальным Conformal, а не lock-down'ился scaffold transfer.
    """
    if point_count <= 0:
        return False
    if conformal_patch:
        return False
    if chain_placement.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        return scaffold_connected
    # FREE chains: pin only endpoints so mixed H/V+FREE patches can still relax
    # their non-frame spans during the final Conformal unwrap.
    return point_index == 0 or point_index == (point_count - 1)


def _free_endpoint_has_local_frame_anchor(
    boundary_loop: BoundaryLoop,
    chain_index: int,
    point_index: int,
    scaffold_connected_chains: frozenset[int],
) -> bool:
    """Check whether a FREE endpoint touches a connected local H/V chain in the same patch."""

    if chain_index < 0 or chain_index >= len(boundary_loop.chains):
        return False

    chain = boundary_loop.chains[chain_index]
    if chain.frame_role != FrameRole.FREE:
        return False

    if point_index == 0:
        corner_index = chain.start_corner_index
    elif point_index == len(chain.vert_indices) - 1:
        corner_index = chain.end_corner_index
    else:
        return False

    if corner_index < 0 or corner_index >= len(boundary_loop.corners):
        return False

    corner = boundary_loop.corners[corner_index]
    for neighbor_chain_index in (corner.prev_chain_index, corner.next_chain_index):
        if neighbor_chain_index == chain_index:
            continue
        if neighbor_chain_index not in scaffold_connected_chains:
            continue
        if neighbor_chain_index < 0 or neighbor_chain_index >= len(boundary_loop.chains):
            continue
        neighbor_chain = boundary_loop.chains[neighbor_chain_index]
        if neighbor_chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
            return True

    return False



def _apply_patch_scaffold_to_uv(
    bm,
    graph: PatchGraph,
    uv_layer,
    patch_placement: ScaffoldPatchPlacement,
    uv_offset: Vector,
) -> PatchApplyStats:
    transfer_state = _build_patch_transfer_targets(bm, graph, patch_placement, uv_offset)
    if transfer_state.status != PatchTransferStatus.OK:
        return PatchApplyStats(
            status=transfer_state.status,
            scaffold_points=int(transfer_state.scaffold_points),
            resolved_scaffold_points=int(transfer_state.resolved_scaffold_points),
            uv_targets_resolved=int(transfer_state.uv_targets_resolved),
            unresolved_scaffold_points=int(transfer_state.unresolved_scaffold_points),
            missing_uv_targets=int(transfer_state.missing_uv_targets),
            conflicting_uv_targets=int(transfer_state.conflicting_uv_targets),
            pinned_uv_loops=0,
            invalid_scaffold_patches=int(transfer_state.invalid_scaffold_patches),
            closure_error=float(transfer_state.closure_error),
            max_chain_gap=float(transfer_state.max_chain_gap),
            chain_gap_count=int(transfer_state.chain_gap_count),
        )

    target_samples: TargetSampleMap = transfer_state.target_samples
    pin_target_ids: PinnedTargetIdSet = transfer_state.pin_target_ids
    missing_uv_targets = 0
    pinned_uv_loops = 0
    for target_id, samples in target_samples.items():
        face_index, vert_index = target_id
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
            loop[uv_layer].pin_uv = target_id in pin_target_ids
            if loop[uv_layer].pin_uv:
                pinned_uv_loops += 1
            applied = True
            break
        if not applied:
            missing_uv_targets += 1

    return PatchApplyStats(
        status=PatchTransferStatus.OK,
        scaffold_points=int(transfer_state.scaffold_points),
        resolved_scaffold_points=int(transfer_state.resolved_scaffold_points),
        uv_targets_resolved=int(transfer_state.uv_targets_resolved),
        unresolved_scaffold_points=int(transfer_state.unresolved_scaffold_points),
        missing_uv_targets=missing_uv_targets,
        conflicting_uv_targets=int(transfer_state.conflicting_uv_targets),
        pinned_uv_loops=pinned_uv_loops,
        invalid_scaffold_patches=int(transfer_state.invalid_scaffold_patches),
        closure_error=float(transfer_state.closure_error),
        max_chain_gap=float(transfer_state.max_chain_gap),
        chain_gap_count=int(transfer_state.chain_gap_count),
    )


CHAIN_FRONTIER_THRESHOLD = FRONTIER_MINIMUM_SCORE


@dataclass(frozen=True)
class ChainAnchor:
    uv: Vector
    source_ref: ChainRef
    source_point_index: int
    source_kind: PlacementSourceKind = PlacementSourceKind.SAME_PATCH


@dataclass(frozen=True)
class FrontierCandidateEval:
    raw_start_anchor: Optional[ChainAnchor]
    raw_end_anchor: Optional[ChainAnchor]
    start_anchor: Optional[ChainAnchor]
    end_anchor: Optional[ChainAnchor]
    known: int
    placed_in_patch: int
    anchor_reason: str = ''
    anchor_adjustments: tuple[AnchorAdjustment, ...] = ()
    closure_dir_override: Optional[Vector] = None
    score: float = -1.0


@dataclass(frozen=True)
class FrontierStopDiagnostics:
    remaining_count: int
    no_anchor_count: int
    low_score_count: int
    rejected_count: int
    placed_patch_ids: tuple[int, ...] = ()
    untouched_patch_ids: tuple[int, ...] = ()
    no_anchor_patch_ids: tuple[int, ...] = ()


@dataclass(frozen=True)
class FrontierPlacementCandidate:
    chain_ref: ChainRef
    chain: BoundaryChain
    node: PatchNode
    start_anchor: Optional[ChainAnchor]
    end_anchor: Optional[ChainAnchor]
    anchor_reason: str = ''
    anchor_adjustments: tuple[AnchorAdjustment, ...] = ()
    closure_dir_override: Optional[Vector] = None
    score: float = -1.0


@dataclass(frozen=True)
class FrontierBootstrapResult:
    runtime_policy: "FrontierRuntimePolicy"
    seed_ref: ChainRef
    seed_chain: BoundaryChain


@dataclass(frozen=True)
class FrontierBootstrapAttempt:
    result: Optional[FrontierBootstrapResult]
    error: str = ''


@dataclass(frozen=True)
class FinalizedQuiltScaffold:
    quilt_scaffold: ScaffoldQuiltPlacement
    untouched_patch_ids: list[int]


@dataclass
class FrontierRuntimePolicy:
    graph: PatchGraph
    quilt_patch_ids: set[int]
    allowed_tree_edges: set[PatchEdgeKey]
    final_scale: float
    point_registry: PointRegistry = field(default_factory=dict)
    vert_to_placements: VertexPlacementMap = field(default_factory=dict)
    placed_chain_refs: set[ChainRef] = field(default_factory=set)
    placed_chains_map: dict[ChainRef, ScaffoldChainPlacement] = field(default_factory=dict)
    chain_dependency_patches: dict[ChainRef, tuple[int, ...]] = field(default_factory=dict)
    rejected_chain_refs: set[ChainRef] = field(default_factory=set)
    build_order: list[ChainRef] = field(default_factory=list)
    closure_pair_map: Optional[dict[ChainRef, ChainRef]] = None
    placed_count_by_patch: dict[int, int] = field(default_factory=dict)
    closure_pair_refs: frozenset[ChainRef] = field(init=False, default_factory=frozenset)

    def __post_init__(self) -> None:
        self.closure_pair_refs = frozenset(self.closure_pair_map.keys()) if self.closure_pair_map else frozenset()

    def placed_in_patch(self, patch_id: int) -> int:
        return self.placed_count_by_patch.get(patch_id, 0)

    def total_placed(self) -> int:
        return len(self.build_order)

    def is_chain_available(self, chain_ref: ChainRef) -> bool:
        return chain_ref not in self.placed_chain_refs and chain_ref not in self.rejected_chain_refs

    def placed_patch_ids(self) -> tuple[int, ...]:
        return tuple(sorted(self.placed_count_by_patch.keys()))

    def reject_chain(self, chain_ref: ChainRef) -> None:
        self.rejected_chain_refs.add(chain_ref)

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
        _cf_register_points(chain_ref, chain, uv_points, self.point_registry, self.vert_to_placements)

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
        if compute_score and known > 0:
            score = _cf_score_candidate(
                chain_ref,
                chain,
                node,
                known,
                self.graph,
                placed_in_patch,
                self.quilt_patch_ids,
                self.allowed_tree_edges,
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


def _cf_select_best_frontier_candidate(
    runtime_policy: FrontierRuntimePolicy,
    all_chain_pool: list[ChainPoolEntry],
) -> Optional[FrontierPlacementCandidate]:
    best_candidate = None
    best_score = -1.0

    for entry in all_chain_pool:
        chain_ref = entry.chain_ref
        if not runtime_policy.is_chain_available(chain_ref):
            continue

        candidate_eval = runtime_policy.evaluate_candidate(
            chain_ref,
            entry.chain,
            entry.node,
            apply_closure_preconstraint=True,
            compute_score=True,
        )
        if candidate_eval.known == 0:
            continue

        if candidate_eval.score > best_score:
            best_score = candidate_eval.score
            best_candidate = FrontierPlacementCandidate(
                chain_ref=chain_ref,
                chain=entry.chain,
                node=entry.node,
                start_anchor=candidate_eval.start_anchor,
                end_anchor=candidate_eval.end_anchor,
                anchor_reason=candidate_eval.anchor_reason,
                anchor_adjustments=candidate_eval.anchor_adjustments,
                closure_dir_override=candidate_eval.closure_dir_override,
                score=candidate_eval.score,
            )

    return best_candidate


def _cf_try_place_frontier_candidate(
    runtime_policy: FrontierRuntimePolicy,
    candidate: FrontierPlacementCandidate,
    iteration: int,
) -> bool:
    graph = runtime_policy.graph
    chain_ref = candidate.chain_ref
    chain = candidate.chain

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
            print(
                f"[CFTUV][Frontier] Reject {iteration}: "
                f"P{chain_ref[0]} L{chain_ref[1]}C{chain_ref[2]} "
                f"{chain.frame_role.value} reason:anchor_rectify_failed"
            )
            return False

    dir_override = candidate.closure_dir_override
    if dir_override is None:
        dir_override = _try_inherit_direction(
            chain,
            candidate.node,
            candidate.start_anchor,
            candidate.end_anchor,
            graph,
            runtime_policy.point_registry,
        )

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
        print(
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
    print(
        f"[CFTUV][Frontier] Step {iteration}: "
        f"P{chain_ref[0]} L{chain_ref[1]}C{chain_ref[2]} "
        f"{chain.frame_role.value} score:{candidate.score:.2f} "
        f"ep:{anchor_count} a:{anchor_label}{reason_suffix}{bridge_tag}"
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
        closure_pair_map=closure_pair_map,
    )

    seed_payload = _cf_build_seed_placement(seed_ref, seed_chain, root_node, final_scale)
    if seed_payload is None:
        return FrontierBootstrapAttempt(result=None, error='seed_placement_failed')

    seed_placement = seed_payload.placement
    seed_uvs = seed_payload.uv_points
    runtime_policy.register_chain(seed_ref, seed_chain, seed_placement, seed_uvs, ())

    print(
        f"[CFTUV][Frontier] Seed: P{seed_ref[0]} L{seed_ref[1]}C{seed_ref[2]} "
        f"{seed_chain.frame_role.value} "
        f"({seed_uvs[0].x:.4f},{seed_uvs[0].y:.4f})"
        f"->({seed_uvs[-1].x:.4f},{seed_uvs[-1].y:.4f})"
    )
    if allowed_tree_edges:
        edge_labels = [f"{edge[0]}-{edge[1]}" for edge in sorted(allowed_tree_edges)]
        print(f"[CFTUV][Frontier] Tree edges: {edge_labels}")

    return FrontierBootstrapAttempt(
        result=FrontierBootstrapResult(
            runtime_policy=runtime_policy,
            seed_ref=seed_ref,
            seed_chain=seed_chain,
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
    best_score = -1.0

    for loop_idx, loop in solve_view.iter_visible_loops(root_node.patch_id):
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
            if seam_neighbor_patch_id is None or other_ref[0] != seam_neighbor_patch_id:
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
            if seam_neighbor_patch_id is None or other_ref[0] != seam_neighbor_patch_id:
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


def _cf_score_candidate(
    chain_ref,
    chain,
    node,
    known,
    graph,
    placed_in_patch,
    quilt_patch_ids,
    allowed_tree_edges,
    closure_pair_refs=None,
    start_anchor: Optional[ChainAnchor] = None,
    end_anchor: Optional[ChainAnchor] = None,
):
    """Chain-level score для frontier candidate."""
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
    if placed_in_patch > 0:
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
        score -= 0.35 if placed_in_patch > 0 else 0.25

    # Secondary closure-side seam pairs should not outrun same-patch width/height
    # carriers during early patch ingress. Иначе tube/ring patch получает вторую
    # seam-chain слишком рано и H/V внутри patch схлопываются в чужой span.
    if closure_pair_refs and chain_ref in closure_pair_refs:
        if same_patch_anchor_count == 0 and cross_patch_anchor_count > 0:
            score -= 0.9 if placed_in_patch > 0 else 1.1
        elif placed_in_patch == 0:
            score -= 0.7

    return score



def _try_inherit_direction(
    chain: BoundaryChain,
    node: PatchNode,
    start_anchor: Optional[ChainAnchor],
    end_anchor: Optional[ChainAnchor],
    graph: PatchGraph,
    point_registry: PointRegistry,
):
    """Наследует direction от соседнего same-role chain в том же patch.

    Если anchor-chain имеет тот же frame_role (H↔H, V↔V) в том же patch,
    берём его UV direction вместо вычисления из 3D. Это continuation —
    бевельный sub-chain продолжает движение основного chain.
    """
    if chain.frame_role not in (FrameRole.H_FRAME, FrameRole.V_FRAME):
        return None

    own_direction = _cf_determine_direction(chain, node)

    for anchor in (start_anchor, end_anchor):
        if anchor is None or anchor.source_kind != PlacementSourceKind.SAME_PATCH:
            continue
        src_chain = graph.get_chain(*anchor.source_ref)
        if src_chain is None or src_chain.frame_role != chain.frame_role:
            continue

        # Нашли same-role same-patch соседа — берём его UV direction
        src_ref = anchor.source_ref
        n_pts = len(src_chain.vert_cos)
        src_start_uv = point_registry.get(_point_registry_key(src_ref, 0))
        src_end_uv = point_registry.get(_point_registry_key(src_ref, n_pts - 1))
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

    print(
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
    closure_pair_map = _build_quilt_closure_pair_map(graph, quilt_patch_ids, allowed_tree_edges)
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
    all_chain_pool = _cf_build_frontier_chain_pool(
        solve_view,
        graph,
        ordered_quilt_patch_ids,
        seed_ref,
    )

    max_iter = len(all_chain_pool) + 10
    for iteration in range(1, max_iter + 1):
        best_candidate = _cf_select_best_frontier_candidate(runtime_policy, all_chain_pool)

        if best_candidate is None or best_candidate.score < CHAIN_FRONTIER_THRESHOLD:
            if _cf_try_place_tree_ingress_candidate(
                runtime_policy,
                all_chain_pool,
                iteration,
            ):
                continue
            if _cf_try_place_free_ingress_bridge(
                runtime_policy,
                all_chain_pool,
                iteration,
            ):
                continue
            if _cf_try_place_closure_follow_candidate(
                runtime_policy,
                all_chain_pool,
                closure_pair_map,
                iteration,
            ):
                continue
            # Диагностика: почему frontier остановился
            stop_diag = runtime_policy.build_stop_diagnostics(all_chain_pool)
            print(
                f"[CFTUV][Frontier] STOP: remaining={stop_diag.remaining_count} "
                f"no_anchor={stop_diag.no_anchor_count} low_score={stop_diag.low_score_count} "
                f"rejected={stop_diag.rejected_count}"
            )
            print(
                f"[CFTUV][Frontier] Patches: placed_in={len(stop_diag.placed_patch_ids)} "
                f"untouched={len(stop_diag.untouched_patch_ids)} "
                f"no_anchor_patches={len(stop_diag.no_anchor_patch_ids)}"
            )
            if stop_diag.untouched_patch_ids and len(stop_diag.untouched_patch_ids) <= 20:
                print(f"[CFTUV][Frontier] Untouched patches: {list(stop_diag.untouched_patch_ids)}")
            break

        if not _cf_try_place_frontier_candidate(runtime_policy, best_candidate, iteration):
            continue

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


def format_root_scaffold_report(
    graph: PatchGraph,
    scaffold_map: ScaffoldMap,
    mesh_name: Optional[str] = None,
) -> FormattedReport:
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
                    + f"{report.frame_role.value} mode:{report.anchor_mode.value} "
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
                max((report.scatter_max for report in frame_reports if report.axis_kind == FrameAxisKind.ROW), default=0.0),
            )
            max_column_scatter = max(
                max_column_scatter,
                max((report.scatter_max for report in frame_reports if report.axis_kind == FrameAxisKind.COLUMN), default=0.0),
            )
            lines.append(
                f"  FrameGroups: {len(frame_reports)} | "
                f"max_row_scatter:{max((report.scatter_max for report in frame_reports if report.axis_kind == FrameAxisKind.ROW), default=0.0):.6f} | "
                f"max_column_scatter:{max((report.scatter_max for report in frame_reports if report.axis_kind == FrameAxisKind.COLUMN), default=0.0):.6f}"
            )
            for report in frame_reports:
                coord_label = (
                    f"z:{report.class_coord_a:.6f}"
                    if report.axis_kind == FrameAxisKind.ROW
                    else f"xy:({report.class_coord_a:.6f},{report.class_coord_b:.6f})"
                )
                refs_label = ','.join(
                    f"P{patch_id}L{loop_index}C{chain_index}"
                    for patch_id, loop_index, chain_index in report.member_refs
                )
                closure_tag = " closure:1" if report.closure_sensitive else " closure:0"
                lines.append(
                    "    "
                    + f"{report.axis_kind.value} {report.frame_role.value} {coord_label} "
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
                for gap_report in patch_placement.gap_reports:
                    lines.append(
                        f"    Gap {gap_report.chain_index}->{gap_report.next_chain_index}: {gap_report.gap:.6f}"
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
                    f"    {chain_placement.source_kind.value.title()} {chain_placement.chain_index}: {chain_placement.frame_role.value} | "
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
    return FormattedReport(lines=lines, summary=summary)


def _compute_quilt_bbox(quilt_scaffold: ScaffoldQuiltPlacement) -> UvBounds:
    placements = [patch for patch in quilt_scaffold.patches.values() if _patch_scaffold_is_supported(patch)]
    if not placements:
        return UvBounds(bbox_min=Vector((0.0, 0.0)), bbox_max=Vector((0.0, 0.0)))
    min_x = min(patch.bbox_min.x for patch in placements)
    min_y = min(patch.bbox_min.y for patch in placements)
    max_x = max(patch.bbox_max.x for patch in placements)
    max_y = max(patch.bbox_max.y for patch in placements)
    return UvBounds(
        bbox_min=Vector((min_x, min_y)),
        bbox_max=Vector((max_x, max_y)),
    )


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


def format_regression_snapshot_report(
    bm,
    graph: PatchGraph,
    solve_plan: Optional[SolvePlan],
    scaffold_map: ScaffoldMap,
    mesh_name: Optional[str] = None,
) -> FormattedReport:
    lines = []
    if mesh_name:
        lines.append(f"Mesh: {mesh_name}")

    solve_plan = solve_plan or SolvePlan()
    quilt_plan_by_index = {quilt.quilt_index: quilt for quilt in solve_plan.quilts}
    unsupported_patch_ids = _collect_phase1_unsupported_patch_ids(scaffold_map)
    invalid_closure_patch_ids = sorted({
        patch_id
        for quilt_scaffold in scaffold_map.quilts
        for patch_id, patch_placement in quilt_scaffold.patches.items()
        if patch_placement is not None and not patch_placement.notes and not patch_placement.closure_valid
    })
    closure_seam_count = sum(
        len(getattr(quilt_scaffold, 'closure_seam_reports', ()))
        for quilt_scaffold in scaffold_map.quilts
    )
    max_closure_span_mismatch = max(
        (
            report.span_mismatch
            for quilt_scaffold in scaffold_map.quilts
            for report in getattr(quilt_scaffold, 'closure_seam_reports', ())
        ),
        default=0.0,
    )
    max_closure_axis_phase = max(
        (
            report.axis_phase_offset_max
            for quilt_scaffold in scaffold_map.quilts
            for report in getattr(quilt_scaffold, 'closure_seam_reports', ())
        ),
        default=0.0,
    )
    frame_group_count = sum(
        len(getattr(quilt_scaffold, 'frame_alignment_reports', ()))
        for quilt_scaffold in scaffold_map.quilts
    )
    max_row_scatter = max(
        (
            report.scatter_max
            for quilt_scaffold in scaffold_map.quilts
            for report in getattr(quilt_scaffold, 'frame_alignment_reports', ())
            if report.axis_kind == FrameAxisKind.ROW
        ),
        default=0.0,
    )
    max_column_scatter = max(
        (
            report.scatter_max
            for quilt_scaffold in scaffold_map.quilts
            for report in getattr(quilt_scaffold, 'frame_alignment_reports', ())
            if report.axis_kind == FrameAxisKind.COLUMN
        ),
        default=0.0,
    )

    lines.append(f"Patches: {len(graph.nodes)}")
    lines.append(f"Quilts: {len(scaffold_map.quilts)}")
    lines.append(f"Skipped patches: {sorted(solve_plan.skipped_patch_ids)}")
    lines.append(f"Unsupported patches: {unsupported_patch_ids}")
    lines.append(f"Invalid closure patches: {invalid_closure_patch_ids}")
    lines.append(f"Conformal fallback patches: {len(unsupported_patch_ids)}")
    lines.append(
        "Closure seams: "
        f"{closure_seam_count} | max_span_mismatch:{max_closure_span_mismatch:.6f} | "
        f"max_axis_phase:{max_closure_axis_phase:.6f}"
    )
    lines.append(
        "Frame groups: "
        f"{frame_group_count} | max_row_scatter:{max_row_scatter:.6f} | "
        f"max_column_scatter:{max_column_scatter:.6f}"
    )

    for quilt_scaffold in scaffold_map.quilts:
        quilt_plan = quilt_plan_by_index.get(quilt_scaffold.quilt_index)
        ordered_patch_ids = _ordered_quilt_patch_ids(quilt_scaffold, quilt_plan)
        if not ordered_patch_ids:
            ordered_patch_ids = sorted(quilt_scaffold.patches.keys())

        lines.append("")
        lines.append(f"## Quilt {quilt_scaffold.quilt_index}")
        lines.append(f"root_patch_id: {quilt_scaffold.root_patch_id}")
        lines.append(f"patch_ids: {ordered_patch_ids}")
        lines.append(f"stop_reason: {quilt_plan.stop_reason.value if quilt_plan is not None else ''}")
        lines.append(
            "build_order: "
            + "[" + ", ".join(
                f"P{patch_id}L{loop_index}C{chain_index}"
                for patch_id, loop_index, chain_index in quilt_scaffold.build_order
            ) + "]"
        )

        quilt_unsupported_patch_ids = sorted(
            patch_id
            for patch_id in ordered_patch_ids
            if quilt_scaffold.patches.get(patch_id) is not None
            and quilt_scaffold.patches[patch_id].notes
        )
        quilt_invalid_closure_patch_ids = sorted(
            patch_id
            for patch_id in ordered_patch_ids
            if quilt_scaffold.patches.get(patch_id) is not None
            and not quilt_scaffold.patches[patch_id].notes
            and not quilt_scaffold.patches[patch_id].closure_valid
        )
        lines.append(f"unsupported_patch_ids: {quilt_unsupported_patch_ids}")
        lines.append(f"invalid_closure_patch_ids: {quilt_invalid_closure_patch_ids}")

        closure_reports = getattr(quilt_scaffold, 'closure_seam_reports', ())
        if closure_reports:
            lines.append("closure_seams:")
            for report in closure_reports:
                lines.append(
                    "  - "
                    + f"P{report.owner_patch_id}L{report.owner_loop_index}C{report.owner_chain_index}"
                    + f"<->P{report.target_patch_id}L{report.target_loop_index}C{report.target_chain_index} "
                    + f"{report.frame_role.value} mode:{report.anchor_mode.value} "
                    + f"mismatch:{report.span_mismatch:.6f} phase:{report.axis_phase_offset_max:.6f} "
                    + f"free:{report.free_bridge_count}"
                )
        else:
            lines.append("closure_seams: []")

        frame_reports = getattr(quilt_scaffold, 'frame_alignment_reports', ())
        if frame_reports:
            lines.append("frame_groups:")
            for report in frame_reports:
                coord_label = (
                    f"z:{report.class_coord_a:.6f}"
                    if report.axis_kind == FrameAxisKind.ROW
                    else f"xy:({report.class_coord_a:.6f},{report.class_coord_b:.6f})"
                )
                lines.append(
                    "  - "
                    + f"{report.axis_kind.value} {report.frame_role.value} {coord_label} "
                    + f"chains:{report.chain_count} target:{report.target_cross_uv:.6f} "
                    + f"scatter:{report.scatter_max:.6f}/{report.scatter_mean:.6f}"
                )
        else:
            lines.append("frame_groups: []")

        lines.append("patches:")
        for patch_id in ordered_patch_ids:
            patch_placement = quilt_scaffold.patches.get(patch_id)
            if patch_placement is None:
                lines.append(f"  - P{patch_id} status:missing_patch")
                continue

            transfer_state = _build_patch_transfer_targets(
                bm,
                graph,
                patch_placement,
                Vector((0.0, 0.0)),
            )
            node = graph.nodes.get(patch_id)
            signature = graph.get_patch_semantic_key(patch_id)
            note_label = ",".join(patch_placement.notes) if patch_placement.notes else "-"
            dep_label = list(patch_placement.dependency_patches) if patch_placement.dependency_patches else []
            lines.append(
                "  - "
                + f"P{patch_id} {signature} "
                + f"status:{patch_placement.status.value} transfer:{transfer_state.status.value} "
                + f"loop:{patch_placement.loop_index} root_chain:{patch_placement.root_chain_index} "
                + f"closure_valid:{1 if patch_placement.closure_valid else 0} "
                + f"closure_error:{patch_placement.closure_error:.6f} "
                + f"placed_chains:{len(patch_placement.chain_placements)} "
                + f"unplaced:{list(patch_placement.unplaced_chain_indices)} "
                + f"deps:{dep_label} "
                + f"notes:{note_label} "
                + f"resolved:{transfer_state.resolved_scaffold_points}/{transfer_state.scaffold_points} "
                + f"uv_targets:{transfer_state.uv_targets_resolved} "
                + f"pinned:{transfer_state.pinned_uv_targets} "
                + f"unpinned:{transfer_state.unpinned_uv_targets} "
                + f"conflicts:{transfer_state.conflicting_uv_targets}"
            )
            if node is not None and 0 <= patch_placement.loop_index < len(node.boundary_loops):
                sc_count = len(patch_placement.scaffold_connected_chains)
                lines.append(
                    f"    scaffold_connected_chains:{sc_count}/{len(node.boundary_loops[patch_placement.loop_index].chains)}"
                )

    summary = (
        f"Regression snapshot | quilts:{len(scaffold_map.quilts)} | patches:{len(graph.nodes)} | "
        f"unsupported:{len(unsupported_patch_ids)} | invalid_closure:{len(invalid_closure_patch_ids)} | "
        f"conformal_fallback_patches:{len(unsupported_patch_ids)}"
    )
    return FormattedReport(lines=lines, summary=summary)


def _print_phase1_preview_patch_report(
    quilt_index: int,
    patch_id: int,
    stats: PatchApplyStats,
) -> None:
    status = stats.status.value
    status_suffix = '' if status == 'ok' else f" status={status}"
    closure_suffix = ''
    if status != 'ok' or int(stats.chain_gap_count) > 0:
        closure_suffix = (
            f" closure={float(stats.closure_error):.6f}"
            f" max_gap={float(stats.max_chain_gap):.6f}"
            f" gaps={int(stats.chain_gap_count)}"
        )
    print(
        f"[CFTUV][Phase1] Quilt {quilt_index} Patch {patch_id}: "
        f"scaffold={stats.scaffold_points} resolved={stats.resolved_scaffold_points} "
        f"uv_targets={stats.uv_targets_resolved} unresolved={stats.unresolved_scaffold_points} "
        f"missing={stats.missing_uv_targets} conflicts={stats.conflicting_uv_targets} "
        f"pinned={stats.pinned_uv_loops}{status_suffix}{closure_suffix}"
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
            max((report.scatter_max for report in frame_reports if report.axis_kind == FrameAxisKind.ROW), default=0.0),
        )
        frame_column_max_scatter = max(
            frame_column_max_scatter,
            max((report.scatter_max for report in frame_reports if report.axis_kind == FrameAxisKind.COLUMN), default=0.0),
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

        quilt_bounds = _compute_quilt_bbox(quilt_scaffold)
        # Каждый quilt стартует в (0,0) UV — без горизонтального стекания
        uv_offset = Vector((-quilt_bounds.bbox_min.x, -quilt_bounds.bbox_min.y))
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

            patch_stats.accumulate_into(quilt_stats)

            if int(patch_stats.resolved_scaffold_points) > 0:
                all_conformal_patch_ids.append(patch_id)

            if int(patch_stats.pinned_uv_loops) <= 0:
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
            patch_bounds = _compute_patch_uv_bbox(bm, patch_graph, uv_layer, [patch_id])
            uv_offset = Vector((-patch_bounds.bbox_min.x, -patch_bounds.bbox_min.y))
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


def _attachment_candidate_preference_key(candidate: AttachmentCandidate) -> AttachmentCandidatePreference:
    return AttachmentCandidatePreference(
        score=candidate.score,
        frame_continuation=candidate.frame_continuation,
        endpoint_bridge=candidate.endpoint_bridge,
        endpoint_strength=candidate.endpoint_strength,
        best_pair_strength=candidate.best_pair_strength,
        seam_norm=candidate.seam_norm,
    )


def _select_preferred_edge_candidate(
    current: Optional[AttachmentCandidate],
    candidate: AttachmentCandidate,
) -> AttachmentCandidate:
    if current is None:
        return candidate
    current_key = _attachment_candidate_preference_key(current)
    candidate_key = _attachment_candidate_preference_key(candidate)
    return candidate if candidate_key > current_key else current


def _count_chain_endpoint_support(
    graph: PatchGraph,
    patch_id: int,
    loop_index: int,
    chain_index: int,
    chain_role: FrameRole,
) -> EndpointSupportStats:
    fixed_endpoint_count = 0
    same_axis_endpoint_count = 0
    free_touched_endpoint_count = 0

    for endpoint_label in ('start', 'end'):
        endpoint_ctx = _get_chain_endpoint_context(graph, patch_id, loop_index, chain_index, endpoint_label)
        if endpoint_ctx is None:
            continue
        neighbor_roles = []
        for neighbor_loop_index, neighbor_chain_index in endpoint_ctx.neighbors:
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

    return EndpointSupportStats(
        fixed_endpoint_count=fixed_endpoint_count,
        same_axis_endpoint_count=same_axis_endpoint_count,
        free_touched_endpoint_count=free_touched_endpoint_count,
    )


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
    owner_support = _count_chain_endpoint_support(
        graph,
        candidate.owner_patch_id,
        candidate.owner_loop_index,
        candidate.owner_chain_index,
        candidate.owner_role,
    )
    target_support = _count_chain_endpoint_support(
        graph,
        candidate.target_patch_id,
        candidate.target_loop_index,
        candidate.target_chain_index,
        candidate.target_role,
    )

    fixed_endpoint_count = owner_support.fixed_endpoint_count + target_support.fixed_endpoint_count
    same_axis_endpoint_count = owner_support.same_axis_endpoint_count + target_support.same_axis_endpoint_count
    free_touched_endpoint_count = owner_support.free_touched_endpoint_count + target_support.free_touched_endpoint_count
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
) -> dict[PatchEdgeKey, AttachmentCandidate]:
    edge_candidate_map: dict[PatchEdgeKey, AttachmentCandidate] = {}
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


def _apply_quilt_closure_cut_recommendations(
    graph: PatchGraph,
    solver_graph: SolverGraph,
    quilt_plan: QuiltPlan,
) -> QuiltPlan:
    if len(quilt_plan.steps) <= 2:
        return quilt_plan

    current_quilt = quilt_plan
    seen_tree_signatures = set()

    for _ in range(max(2, len(quilt_plan.steps) + 1)):
        current_tree_edges = frozenset(_build_quilt_tree_edges(current_quilt))
        if current_tree_edges in seen_tree_signatures:
            break
        seen_tree_signatures.add(current_tree_edges)

        analyses = _analyze_quilt_closure_cuts(graph, solver_graph, current_quilt)
        swap_analyses = [
            analysis
            for analysis in analyses
            if analysis.current_cut.edge_key != analysis.recommended_cut.edge_key
        ]
        if not swap_analyses:
            break

        forbidden_edge_keys = {analysis.recommended_cut.edge_key for analysis in swap_analyses}
        rebuilt_quilt = _rebuild_quilt_steps_with_forbidden_edges(
            current_quilt,
            solver_graph,
            forbidden_edge_keys,
        )
        if rebuilt_quilt is None:
            break

        rebuilt_tree_edges = _build_quilt_tree_edges(rebuilt_quilt)
        if rebuilt_tree_edges == _build_quilt_tree_edges(current_quilt):
            break

        swap_labels = ", ".join(
            f"{analysis.current_cut.edge_key[0]}-{analysis.current_cut.edge_key[1]}"
            f"->{analysis.recommended_cut.edge_key[0]}-{analysis.recommended_cut.edge_key[1]}"
            for analysis in swap_analyses
        )
        print(
            f"[CFTUV][Plan] Quilt {quilt_plan.quilt_index}: closure cut swap {swap_labels}"
        )
        current_quilt = rebuilt_quilt

    return current_quilt


def _restore_original_quilt_plan(quilt_plan: QuiltPlan) -> Optional[QuiltPlan]:
    if not quilt_plan.original_steps:
        return None
    return replace(
        quilt_plan,
        solved_patch_ids=list(quilt_plan.original_solved_patch_ids),
        steps=list(quilt_plan.original_steps),
        original_solved_patch_ids=[],
        original_steps=[],
    )


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
) -> FormattedReport:
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
        lines.append(f"  Stop: {quilt.stop_reason.value}")
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
    return FormattedReport(lines=lines, summary=summary)





