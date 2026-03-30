from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from mathutils import Vector

try:
    from .model import (
        BoundaryChain,
        ChainRef,
        FrameRole,
        PatchEdgeKey,
        PatchGraph,
        ScaffoldChainPlacement,
    )
    from .solve_records import (
        ChainAnchor,
        FrontierCandidateEval,
        PatchShapeProfile,
        PointRegistry,
        SeamRelationProfile,
        VertexPlacementMap,
        _patch_pair_key,
        _point_registry_key,
    )
except ImportError:
    from model import (
        BoundaryChain,
        ChainRef,
        FrameRole,
        PatchEdgeKey,
        PatchGraph,
        ScaffoldChainPlacement,
    )
    from solve_records import (
        ChainAnchor,
        FrontierCandidateEval,
        PatchShapeProfile,
        PointRegistry,
        SeamRelationProfile,
        VertexPlacementMap,
        _patch_pair_key,
        _point_registry_key,
    )


@dataclass
class FrontierRuntimePolicy:
    graph: PatchGraph
    quilt_patch_ids: set[int]
    allowed_tree_edges: set[PatchEdgeKey]
    final_scale: float
    seam_relation_by_edge: dict[PatchEdgeKey, SeamRelationProfile] = field(default_factory=dict)
    tree_ingress_partner_by_chain: dict[ChainRef, ChainRef] = field(default_factory=dict)
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
    # Temporary compatibility storage for score-owned derived caches until P7.
    _outer_chain_count_by_patch: dict[int, int] = field(init=False, default_factory=dict)
    _frame_chain_count_by_patch: dict[int, int] = field(init=False, default_factory=dict)
    _closure_pair_count_by_patch: dict[int, int] = field(init=False, default_factory=dict)
    _shape_profile_by_patch: dict[int, PatchShapeProfile] = field(init=False, default_factory=dict)
    _cached_evals: dict[ChainRef, FrontierCandidateEval] = field(init=False, default_factory=dict)
    _dirty_refs: set[ChainRef] = field(init=False, default_factory=set)
    _vert_to_pool_refs: dict[int, list[ChainRef]] = field(init=False, default_factory=dict)
    _patch_to_pool_refs: dict[int, list[ChainRef]] = field(init=False, default_factory=dict)
    _cache_hits: int = field(init=False, default=0)

    def __post_init__(self) -> None:
        self.closure_pair_refs = frozenset(self.closure_pair_map.keys()) if self.closure_pair_map else frozenset()

    def placed_in_patch(self, patch_id: int) -> int:
        return self.placed_count_by_patch.get(patch_id, 0)

    def placed_h_in_patch(self, patch_id: int) -> int:
        return self.placed_h_count_by_patch.get(patch_id, 0)

    def placed_v_in_patch(self, patch_id: int) -> int:
        return self.placed_v_count_by_patch.get(patch_id, 0)

    def placed_free_in_patch(self, patch_id: int) -> int:
        return self.placed_free_count_by_patch.get(patch_id, 0)

    # Temporary compatibility accessors for score-owned caches until P7.
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

    for vi in chain.vert_indices:
        for ref in vtp.get(vi, ()):
            dirty.add(ref)

    if runtime_policy.placed_count_by_patch.get(patch_id, 0) == 1:
        for refs_list in vtp.values():
            for ref in refs_list:
                if ref[0] == patch_id:
                    dirty.add(ref)
    for ref in runtime_policy._patch_to_pool_refs.get(patch_id, ()):
        dirty.add(ref)


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
