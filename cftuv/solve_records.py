from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from mathutils import Vector

try:
    from .constants import (
        FRONTIER_PROPAGATE_THRESHOLD,
        FRONTIER_WEAK_THRESHOLD,
        FRONTIER_MINIMUM_SCORE,
    )
    from .model import (
        BoundaryChain, BoundaryCorner, BoundaryLoop, ChainNeighborKind, FrameRole, LoopKind,
        PatchGraph, PatchNode,
        ScaffoldChainPlacement, ScaffoldPatchPlacement, ScaffoldQuiltPlacement,
        ChainGapReport, PlacementSourceKind,
        ChainRef, PatchEdgeKey, LoopChainRef, AnchorAdjustment,
    )
except ImportError:
    from constants import (
        FRONTIER_PROPAGATE_THRESHOLD,
        FRONTIER_WEAK_THRESHOLD,
        FRONTIER_MINIMUM_SCORE,
    )
    from model import (
        BoundaryChain, BoundaryCorner, BoundaryLoop, ChainNeighborKind, FrameRole, LoopKind,
        PatchGraph, PatchNode,
        ScaffoldChainPlacement, ScaffoldPatchPlacement, ScaffoldQuiltPlacement,
        ChainGapReport, PlacementSourceKind,
        ChainRef, PatchEdgeKey, LoopChainRef, AnchorAdjustment,
    )


EDGE_PROPAGATE_MIN = FRONTIER_PROPAGATE_THRESHOLD
EDGE_WEAK_MIN = FRONTIER_WEAK_THRESHOLD
SCAFFOLD_CLOSURE_EPSILON = 1e-4
FRAME_ROW_GROUP_TOLERANCE = 1e-3
FRAME_COLUMN_GROUP_TOLERANCE = 1e-3
CHAIN_FRONTIER_THRESHOLD = FRONTIER_MINIMUM_SCORE


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
        if owner_type == target_type:
            return True
        # WALL может быть bridge к FLOOR/SLOPE — через единственный tree edge
        return owner_type == 'WALL' or target_type == 'WALL'


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
    representative_length: float
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
    length_bias: float


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
    hv_adjacency: int


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
    support_class: str
    fixed_endpoint_count: int
    same_axis_endpoint_count: int
    free_touched_endpoint_count: int
    representative_chain_length: float
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


def _patch_pair_key(patch_a_id: int, patch_b_id: int) -> PatchEdgeKey:
    return (min(patch_a_id, patch_b_id), max(patch_a_id, patch_b_id))


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
    pin_map: Optional["PatchPinMap"] = None


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
    length_factor: float = 0.0
    downstream_count: int = 0
    downstream_bonus: float = 0.0
    isolation_preview: bool = True
    isolation_penalty: float = 0.0
    structural_free_bonus: float = 0.0
    hv_adjacency: int = 0


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
    length_factor: float = 0.0
    downstream_count: int = 0
    downstream_bonus: float = 0.0
    isolation_preview: bool = True
    isolation_penalty: float = 0.0
    structural_free_bonus: float = 0.0
    hv_adjacency: int = 0


@dataclass(frozen=True)
class FrontierBootstrapResult:
    runtime_policy: "FrontierRuntimePolicy"
    seed_ref: ChainRef
    seed_chain: BoundaryChain
    seed_score: float


@dataclass(frozen=True)
class FrontierBootstrapAttempt:
    result: Optional[FrontierBootstrapResult]
    error: str = ''


@dataclass(frozen=True)
class FinalizedQuiltScaffold:
    quilt_scaffold: ScaffoldQuiltPlacement
    untouched_patch_ids: list[int]


@dataclass(frozen=True)
class FrontierPlacementRecord:
    """Telemetry record for one frontier placement step."""
    iteration: int
    chain_ref: ChainRef
    frame_role: FrameRole
    placement_path: str              # "main" | "tree_ingress" | "closure_follow"
    score: float                     # -1.0 для rescue-путей (без score)
    anchor_count: int                # 0, 1, 2
    start_anchor_kind: str           # "same_patch" | "cross_patch" | "none"
    end_anchor_kind: str             # "same_patch" | "cross_patch" | "none"
    placed_in_patch_before: int      # сколько chains в этом patch ДО данного placement
    is_first_in_patch: bool
    is_bridge: bool                  # FREE с ≤2 вертами
    is_corner_split: bool
    neighbor_kind: str               # "PATCH" | "MESH_BORDER" | "SEAM_SELF"
    is_closure_pair: bool            # chain_ref входит в closure_pair_refs
    closure_preconstraint_applied: bool
    anchor_adjustment_applied: bool
    direction_inherited: bool
    chain_length_uv: float           # полная UV длина после размещения

    # --- P5 Scoring telemetry ---
    length_factor: float = 0.0
    downstream_count: int = 0
    downstream_bonus: float = 0.0
    isolation_preview: bool = True     # would_be_connected
    isolation_penalty: float = 0.0
    structural_free_bonus: float = 0.0
    hv_adjacency: int = 0


@dataclass(frozen=True)
class FrontierStallRecord:
    """Telemetry record for one frontier stall event."""
    iteration: int
    best_rejected_score: float       # наибольший score среди кандидатов не прошедших threshold
    best_rejected_ref: Optional[ChainRef]
    best_rejected_role: Optional[FrameRole]
    best_rejected_anchor_count: int
    available_count: int             # chains ещё в pool
    no_anchor_count: int             # chains с known=0
    below_threshold_count: int       # chains с 0 < score < threshold
    rescue_attempted: str            # "tree_ingress" | "closure_follow" | "none"
    rescue_succeeded: bool
    patches_with_placed: int         # patches с ≥1 размещённым chain
    patches_untouched: int           # patches с 0 размещёнными chains


@dataclass(frozen=True)
class QuiltFrontierTelemetry:
    """Aggregated frontier telemetry for one quilt."""
    quilt_index: int
    total_placements: int
    main_placements: int
    tree_ingress_placements: int
    closure_follow_placements: int
    total_stalls: int
    stalls_resolved_by_rescue: int
    stalls_unresolved: int           # финальная остановка без rescue
    score_min: float                 # min score main-пути
    score_max: float
    score_mean: float
    score_p25: float                 # 25-й перцентиль
    score_p50: float                 # медиана
    score_p75: float
    best_rejected_score_max: float   # наибольший score, не прошедший threshold
    first_rescue_iteration: int      # -1 если rescue не было
    rescue_ratio: float              # rescue_placements / total_placements
    frontier_duration_sec: float
    placement_records: tuple[FrontierPlacementRecord, ...]
    stall_records: tuple[FrontierStallRecord, ...]


@dataclass(frozen=True)
class PinPolicy:
    """Immutable pin policy configuration. Passed as parameter, not global."""
    pin_connected_hv: bool = True
    pin_free_endpoints_with_hv_anchor: bool = True
    skip_conformal_patches: bool = True
    skip_isolated_hv: bool = True


@dataclass(frozen=True)
class ChainPinDecision:
    """Pin decision for one placed chain."""
    chain_index: int
    frame_role: FrameRole
    pin_all: bool            # True = every point pinned (H/V connected)
    pin_endpoints_only: bool # True = endpoints checked individually via pin_start/pin_end
    pin_start: bool          # endpoint 0 pinned (meaningful when pin_endpoints_only=True)
    pin_end: bool            # endpoint last pinned (meaningful when pin_endpoints_only=True)
    pin_nothing: bool        # True = no points pinned
    reason: str              # Human-readable reason for debug/snapshot


@dataclass(frozen=True)
class PatchPinMap:
    """Complete pin map for one patch placement."""
    patch_id: int
    loop_index: int
    conformal_patch: bool
    scaffold_connected_chains: frozenset[int]
    chain_decisions: tuple[ChainPinDecision, ...]

    def is_point_pinned(self, chain_index: int, point_index: int, point_count: int) -> bool:
        """Query: should this specific scaffold point be pinned?"""
        for dec in self.chain_decisions:
            if dec.chain_index != chain_index:
                continue
            if dec.pin_all:
                return True
            if dec.pin_nothing:
                return False
            # pin_endpoints_only: check start/end independently
            if point_index == 0:
                return dec.pin_start
            if point_count > 0 and point_index == point_count - 1:
                return dec.pin_end
            return False
        return False

    def pinned_chain_indices(self) -> frozenset[int]:
        """Which chains have any pinned points?"""
        return frozenset(
            dec.chain_index for dec in self.chain_decisions
            if dec.pin_all or dec.pin_endpoints_only
        )


__all__ = [
    'EDGE_PROPAGATE_MIN',
    'EDGE_WEAK_MIN',
    'SCAFFOLD_CLOSURE_EPSILON',
    'FRAME_ROW_GROUP_TOLERANCE',
    'FRAME_COLUMN_GROUP_TOLERANCE',
    'CHAIN_FRONTIER_THRESHOLD',
    'RowClassKey',
    'ColumnClassKey',
    'FrameClassKey',
    'PointRegistryKey',
    'VertexPlacementRef',
    'PointRegistry',
    'VertexPlacementMap',
    'TransferTargetId',
    'ScaffoldKeyId',
    'TargetSampleMap',
    'PinnedTargetIdSet',
    'ScaffoldKeySet',
    'AnchorRefPair',
    'PatchCertainty',
    'SolveComponentsResult',
    'AttachmentCandidate',
    'SolverGraph',
    'SolveView',
    'QuiltStep',
    'QuiltPlan',
    'SolvePlan',
    'QuiltStopReason',
    'AttachmentCandidatePreference',
    'ChainPairSelection',
    'AttachmentNeighborRef',
    'ChainEndpointContext',
    'EndpointMatch',
    'EndpointBridgeMetrics',
    'EndpointSupportStats',
    'ClosureCutHeuristic',
    'QuiltClosureCutAnalysis',
    'ChainAnchor',
    'ChainPoolEntry',
    'FrontierCandidateEval',
    'FrontierStopDiagnostics',
    'FrontierPlacementCandidate',
    'FrontierBootstrapResult',
    'FrontierBootstrapAttempt',
    'FinalizedQuiltScaffold',
    'SeedChainChoice',
    'SeedPlacementResult',
    'ClosureFollowPlacementCandidate',
    'FreeIngressPlacementCandidate',
    'TreeIngressPlacementCandidate',
    'ClosureFollowCandidateRank',
    'FreeIngressCandidateRank',
    'TreeIngressCandidateRank',
    'DirectionOption',
    'AnchorOption',
    'ClosurePreconstraintOptionResult',
    'ClosurePreconstraintMetric',
    'ClosurePreconstraintApplication',
    'DualAnchorRectificationPreview',
    'ResolvedCandidateAnchors',
    'FoundCandidateAnchors',
    'AnchorPairSafetyDecision',
    'DualAnchorClosureDecision',
    'ClosureFollowUvBuildResult',
    'UvAxisMetrics',
    'FrameUvComponents',
    'FrameGroupDisplayCoords',
    'SharedClosureUvOffsets',
    'ClosureChainPairMatch',
    'ClosureChainPairCandidate',
    'FrameGroupMember',
    'PatchChainGapDiagnostics',
    'ScaffoldUvTarget',
    'PatchTransferStatus',
    'PatchTransferTargetsState',
    'PatchApplyStats',
    'UvBounds',
    'PatchRoleCounts',
    '_point_registry_key',
    '_clamp01',
    '_patch_pair_key',
    'FrontierPlacementRecord',
    'FrontierStallRecord',
    'QuiltFrontierTelemetry',
    'PinPolicy',
    'ChainPinDecision',
    'PatchPinMap',
]
