# CFTUV Frontier Split Contract

Status: authoritative P0 contract  
Phase scope: contract, DAG, ownership, risk classification only  
Behavior scope: no runtime behavior change is allowed by this document

This file is the authoritative ownership contract for splitting `cftuv/solve_frontier.py`
into sibling frontier modules. It exists so later extraction phases can move code
mechanically without guessing boundaries.

The architectural invariant is unchanged:

> Preserve chain-first strongest-frontier. Patch and Chain remain the only primary
> solve units. Do not drift into patch-first, loop-sequential, or corner-first placement.

## Locked decisions

- `frontier_closure.py` owns closure-pair matching and closure-pair maps.
- Closure preconstraint is not closure-map ownership. It remains eval-owned.
- Closure-follow UV reconstruction is not closure-map ownership. It remains rescue-owned.
- `FrontierRuntimePolicy` remains the compatibility runtime container, but heavy logic is split by responsibility.
- `solve_records.py` is classified only. No physical record migration is part of this refactor plan.
- Telemetry ownership remains in `solve_instrumentation.py`. Frontier modules accept the collector by parameter.

## Import DAG contract

The orchestrator DAG is confirmed with two explicit amendments that must be treated as
part of the contract before extraction starts:

1. `frontier_eval.py` may bridge to `solve_diagnostics.py` for closure-preconstraint
   metric helpers. This keeps closure-pair ownership narrow and avoids forcing
   `frontier_closure.py` to depend on placement or diagnostics.
2. `frontier_finalize.py` may bridge to `solve_pin_policy.py` for
   `_compute_scaffold_connected_chains`, in addition to its already-allowed diagnostics bridge.

Operational DAG after P0:

```text
solve_frontier.py (orchestrator)
  |- frontier_state.py
  |    `- no frontier_* imports
  |
  |- frontier_place.py
  |    `- imports only model.py / solve_records.py / stdlib
  |
  |- frontier_score.py
  |    `- imports only model.py / solve_records.py / stdlib
  |       (runtime state arrives as data/accessors; avoid runtime back-edge)
  |
  |- frontier_closure.py
  |    `- imports model.py / solve_records.py / solve_planning.py tree helpers
  |
  |- frontier_eval.py
  |    `- imports frontier_state.py, frontier_place.py, frontier_score.py,
  |       frontier_closure.py, and solve_diagnostics.py closure metrics only
  |
  |- frontier_rescue.py
  |    `- imports frontier_state.py, frontier_eval.py, frontier_place.py,
  |       frontier_closure.py
  |
  `- frontier_finalize.py
       `- imports frontier_state.py, frontier_closure.py, solve_diagnostics.py,
          and solve_pin_policy.py where already required
```

Additional DAG lock:

- `frontier_score.py` must not import `frontier_eval.py`.
- `frontier_place.py` must stay free of runtime-state ownership.
- `frontier_closure.py` must not absorb closure preconstraint or closure-follow placement logic.
- If a helper is shared by score and place, prefer trivial inlining in one side over creating a new import cycle.

## `solve_frontier.py` symbol map

Move classes used below:

- `mechanical`: move/extract with behavior preserved.
- `mechanical but risky`: move/extract only, but ordering/cache/telemetry dependencies are fragile.
- `behavioral / later`: keep runtime behavior unchanged now; any deeper cleanup waits for a later phase.

### Keep in `solve_frontier.py` orchestrator shell

| Current symbol | Current line | Future owner | Shape | Move class | Notes |
|---|---:|---|---|---|---|
| `build_quilt_scaffold_chain_frontier` | 4357 | `solve_frontier.py` | keep public orchestrator entrypoint | mechanical but risky | Final shell must still build solve view, bootstrap runtime, run main loop, stall -> rescue order, finalize, and fallback recursion in the same order. |
| `build_root_scaffold_map` | 4545 | `solve_frontier.py` | keep public facade | mechanical | Thin outer loop over quilts. |
| `_cf_bootstrap_frontier_runtime` | 1689 | `solve_frontier.py` | keep as bootstrap coordinator calling extracted helpers | mechanical but risky | Wires closure maps, runtime container, seed choice, seed placement, and seed registration. |
| `_cf_build_frontier_chain_pool` | 1749 | `solve_frontier.py` | keep as local orchestration helper | mechanical | Static pool build belongs to pipeline bootstrap, not to mutable runtime ownership. |

### `frontier_closure.py`

| Current symbol | Current line | Future owner | Shape | Move class | Notes |
|---|---:|---|---|---|---|
| `_match_non_tree_closure_chain_pairs` | 75 | `frontier_closure.py` | standalone function | mechanical but risky | Shared closure-pair matcher; topological sort order and representative-length tie-break must remain identical. |
| `_iter_quilt_closure_chain_pairs` | 148 | `frontier_closure.py` | standalone function | mechanical but risky | Tree-edge primary-pair stripping is part of the current closure contract. |
| `_build_quilt_closure_pair_map` | 191 | `frontier_closure.py` | standalone function | mechanical but risky | Shared bootstrap artifact consumed by eval/rescue/finalize. |
| `_build_tree_ingress_partner_map` | 243 | `frontier_closure.py` | standalone function | mechanical | Shared seam-partner map from quilt plan; rescue reads it, orchestrator bootstraps it. |

Closure-specific non-moves:

- `_closure_preconstraint_direction_options`
- `_closure_preconstraint_metric`
- `_cf_apply_closure_preconstraint`
- `_cf_build_closure_follow_uvs`

These are closure-related, but not closure-map ownership. They stay out of `frontier_closure.py`.

### `frontier_rescue.py`

| Current symbol | Current line | Future owner | Shape | Move class | Notes |
|---|---:|---|---|---|---|
| `_cf_build_closure_follow_uvs` | 493 | `frontier_rescue.py` | standalone function | mechanical but risky | Uses placed partner geometry and endpoint rebuild; do not merge into main frontier. |
| `_cf_rescue_gap_candidate_class` | 539 | `frontier_rescue.py` | standalone function | mechanical | Telemetry-only rescue classification. |
| `_cf_build_frontier_rescue_gap` | 565 | `frontier_rescue.py` | standalone function | mechanical but risky | Telemetry payload shape must stay stable. |
| `_cf_try_place_closure_follow_candidate` | 622 | `frontier_rescue.py` | standalone function | mechanical but risky | Cold-path rescue only; stall order and telemetry fields are locked. |
| `_cf_build_partner_chain_anchor` | 779 | `frontier_rescue.py` | standalone function | mechanical | Tree-ingress helper, used only by rescue path. |
| `_cf_try_place_tree_ingress_candidate` | 802 | `frontier_rescue.py` | standalone function | mechanical but risky | Untouched child-patch ingress rescue must stay separate from main frontier. |

### `frontier_state.py`

| Current symbol | Current line | Future owner | Shape | Move class | Notes |
|---|---:|---|---|---|---|
| `FrontierRuntimePolicy` | 1095 | `frontier_state.py` | compatibility dataclass/container | mechanical but risky | Keep class name if possible; split heavy logic by responsibility. |
| `_mark_neighbors_dirty` | 1395 | `frontier_state.py` | standalone helper plus optional method delegator | mechanical but risky | Dirty propagation must remain bit-identical with cached full scan. |
| `_cf_register_points` | 4107 | `frontier_state.py` | standalone helper | mechanical but risky | Point registry and vertex-placement registry mutation are state ownership. |

### `frontier_eval.py`

| Current symbol | Current line | Future owner | Shape | Move class | Notes |
|---|---:|---|---|---|---|
| `_closure_preconstraint_direction_options` | 286 | `frontier_eval.py` | standalone function | mechanical but risky | Eval-owned because it mixes anchors, inherited direction, and placement preview. |
| `_closure_preconstraint_metric` | 312 | `frontier_eval.py` | standalone function | mechanical but risky | Allowed direct bridge to `solve_diagnostics.py` lives here. |
| `_cf_apply_closure_preconstraint` | 363 | `frontier_eval.py` | standalone function | mechanical but risky | Closure preconstraint is part of candidate evaluation contract. |
| `_cf_select_best_frontier_candidate` | 1426 | `frontier_eval.py` | standalone function | mechanical but risky | Cache-aware selector sits above state/eval/score layers. |
| `_cf_try_place_frontier_candidate` | 1496 | `frontier_eval.py` | standalone function | mechanical but risky | Main-path candidate application; orchestrator should call it as a shell step later. |
| `_cf_choose_seed_chain` | 2371 | `frontier_eval.py` | standalone function | mechanical | Seed choice is evaluation/bootstrap logic, not placement math. |
| `_cf_find_anchors` | 2447 | `frontier_eval.py` | standalone function | mechanical but risky | Anchor provenance semantics are locked. |
| `_cf_frame_anchor_pair_is_axis_safe` | 2565 | `frontier_eval.py` | standalone function | mechanical | Anchor-safety rule set belongs to eval/anchor resolution. |
| `_cf_can_use_dual_anchor_closure` | 2592 | `frontier_eval.py` | standalone function | mechanical but risky | Prevent-patch-wrap behavior is locked. |
| `_cf_resolve_candidate_anchors` | 2623 | `frontier_eval.py` | standalone function | mechanical but risky | Anchor dropping, same-patch softening, and failure reasons must not drift. |
| `_cf_make_frontier_placement_candidate` | 3283 | `frontier_eval.py` | standalone function | mechanical | Eval result -> placement candidate adapter. |
| `_cf_same_patch_anchor_is_protected` | 1850 | `frontier_eval.py` | standalone helper | behavioral / later | Only active through the rectification stub cluster. |
| `_cf_preview_frame_dual_anchor_rectification` | 1859 | `frontier_eval.py` | standalone helper retained as disabled stub | behavioral / later | Active runtime behavior is the early return only; body below it is dead-code triage for P1. |
| `_cf_frame_cross_axis_value` | 1790 | `frontier_eval.py` | standalone helper | behavioral / later | Remains attached to rectification cluster unless the dead body is removed. |
| `_cf_with_frame_cross_axis` | 1798 | `frontier_eval.py` | standalone helper | behavioral / later | Same as above. |
| `_cf_preview_anchor_source_adjustment` | 1806 | `frontier_eval.py` | standalone helper | behavioral / later | Same as above. |

### `frontier_place.py`

| Current symbol | Current line | Future owner | Shape | Move class | Notes |
|---|---:|---|---|---|---|
| `_build_temporary_chain_placement` | 264 | `frontier_place.py` | standalone function | mechanical | Generic temporary placement builder for metric evaluation. |
| `_cf_build_seed_placement` | 1642 | `frontier_place.py` | standalone function | mechanical | Seed placement construction belongs to placement layer. |
| `_cf_chain_source_points` | 1771 | `frontier_place.py` | standalone helper | mechanical | Placement input adapter. |
| `_cf_anchor_count` | 1776 | `frontier_place.py` | standalone helper | mechanical | Shared placement helper used when building placements. |
| `_cf_anchor_debug_label` | 1780 | `frontier_place.py` | standalone helper | mechanical | Shared logging/placement helper. |
| `_cf_chain_total_length` | 1952 | `frontier_place.py` | standalone helper | mechanical | Placement and rescue ranking utility. |
| `_snap_direction_to_role` | 1962 | `frontier_place.py` | standalone helper | mechanical | Direction normalization for H/V placement. |
| `_rotate_direction` | 1972 | `frontier_place.py` | standalone helper | mechanical | Placement geometry helper. |
| `_chain_edge_lengths` | 1982 | `frontier_place.py` | standalone helper | mechanical | Placement geometry helper. |
| `_default_role_direction` | 1991 | `frontier_place.py` | standalone helper | mechanical | Placement geometry helper. |
| `_normalize_direction` | 1997 | `frontier_place.py` | standalone helper | mechanical | Placement geometry helper. |
| `_segment_source_step_directions` | 2005 | `frontier_place.py` | standalone helper | mechanical | Placement geometry helper. |
| `_interpolate_between_anchors_by_lengths` | 2021 | `frontier_place.py` | standalone helper | mechanical | Placement geometry helper. |
| `_sample_cubic_bezier_point` | 2042 | `frontier_place.py` | standalone helper | mechanical | Placement geometry helper. |
| `_sample_cubic_bezier_polyline` | 2052 | `frontier_place.py` | standalone helper | mechanical | Placement geometry helper. |
| `_resample_polyline_by_edge_lengths` | 2066 | `frontier_place.py` | standalone helper | mechanical | Placement geometry helper. |
| `_build_guided_free_chain_from_one_end` | 2109 | `frontier_place.py` | standalone function | mechanical | Placement builder. |
| `_build_guided_free_chain_between_anchors` | 2147 | `frontier_place.py` | standalone function | mechanical | Placement builder. |
| `_build_frame_chain_from_one_end` | 2225 | `frontier_place.py` | standalone function | mechanical | Placement builder. |
| `_build_frame_chain_between_anchors` | 2249 | `frontier_place.py` | standalone function | mechanical | Placement builder. |
| `_cf_rebuild_chain_points_for_endpoints` | 2263 | `frontier_place.py` | standalone function | mechanical but risky | Shared by closure-follow and endpoint-based rebuild. |
| `_cf_apply_anchor_adjustments` | 2286 | `frontier_place.py` | standalone function | mechanical but risky | Mutates already placed UV anchors; ordering with dirty propagation must stay stable. |
| `_cf_determine_direction` | 2339 | `frontier_place.py` | standalone function | mechanical | Base direction inference. |
| `_is_orthogonal_hv_pair` | 3919 | `frontier_place.py` | standalone helper | mechanical | Primary owner is placement/direction inference; score-side consumers should inline the one-liner if needed to avoid a DAG back-edge. |
| `_cf_compute_local_normal` | 3923 | `frontier_place.py` | standalone helper | mechanical | Direction-inheritance geometry helper. |
| `_compute_corner_turn_sign` | 3943 | `frontier_place.py` | standalone helper | mechanical | Direction-inheritance geometry helper. |
| `_perpendicular_direction_for_role` | 3982 | `frontier_place.py` | standalone helper | mechanical | Direction-inheritance geometry helper. |
| `_cf_can_inherit_corner_turn_direction` | 3998 | `frontier_place.py` | standalone helper | mechanical | Direction-inheritance gate. |
| `_try_inherit_direction` | 4005 | `frontier_place.py` | standalone function | mechanical but risky | Shared by eval preconstraint, main placement, and rescue. |
| `_cf_place_chain` | 4067 | `frontier_place.py` | standalone function | mechanical but risky | Primary chain placement entrypoint. |

### `frontier_score.py`

| Current symbol | Current line | Future owner | Shape | Move class | Notes |
|---|---:|---|---|---|---|
| `_cf_estimate_downstream_anchor_count` | 2718 | `frontier_score.py` | standalone function | mechanical but risky | Reads runtime adjacency caches; no behavioral drift allowed. |
| `_cf_count_hv_adjacent_endpoints` | 2752 | `frontier_score.py` | standalone function | mechanical | Shared frontier score helper. |
| `_cf_preview_would_be_connected` | 2774 | `frontier_score.py` | standalone function | mechanical but risky | Connectivity preview must stay aligned with scaffold-connected semantics. |
| `_cf_role_tier` | 2818 | `frontier_score.py` | standalone function | mechanical | Structured rank helper. |
| `_cf_clamp01` | 2848 | `frontier_score.py` | standalone helper | mechanical | Scoring utility. |
| `_cf_chain_polyline_length_3d` | 2852 | `frontier_score.py` | standalone helper | mechanical | Shape-profile helper. |
| `_cf_patch_shape_backbone_bias` | 2861 | `frontier_score.py` | standalone helper | mechanical | Shape-profile helper. |
| `_cf_patch_shape_closure_sensitivity` | 2871 | `frontier_score.py` | standalone helper | mechanical | Shape-profile helper. |
| `_cf_build_patch_shape_profile` | 2881 | `frontier_score.py` | standalone function | mechanical but risky | Score-owned derived cache builder. |
| `_cf_build_patch_scoring_context` | 2966 | `frontier_score.py` | standalone function | mechanical but risky | Explicit patch scoring context must remain semantically identical. |
| `_cf_chain_seam_relation` | 3010 | `frontier_score.py` | standalone function | mechanical | Seam profile lookup helper. |
| `_cf_seam_relation_membership` | 3020 | `frontier_score.py` | standalone function | mechanical | Seam classification helper. |
| `_cf_corner_turn_strength` | 3034 | `frontier_score.py` | standalone helper | mechanical | Corner-hint helper. |
| `_cf_build_corner_scoring_hints` | 3051 | `frontier_score.py` | standalone function | mechanical | Corner-hint builder. |
| `_cf_build_frontier_rank` | 3121 | `frontier_score.py` | standalone function | mechanical but risky | Rank ordering semantics are locked. |
| `_cf_frontier_rank_debug_label` | 3315 | `frontier_score.py` | standalone helper | mechanical | Logging helper derived from structured rank. |
| `_cf_build_frontier_topology_facts` | 3328 | `frontier_score.py` | standalone function | mechanical but risky | Topology facts are reused by layered score and rank. |
| `_cf_score_topology_baseline` | 3370 | `frontier_score.py` | standalone function | mechanical | Layered score helper. |
| `_cf_score_patch_anchor_context` | 3408 | `frontier_score.py` | standalone function | mechanical | Layered score helper. |
| `_cf_score_closure_guard` | 3437 | `frontier_score.py` | standalone function | mechanical | Layered score helper. |
| `_cf_score_seam_relation_hint` | 3450 | `frontier_score.py` | standalone function | mechanical | Layered score helper. |
| `_cf_score_shape_hint` | 3473 | `frontier_score.py` | standalone function | mechanical | Layered score helper. |
| `_cf_score_corner_hint` | 3504 | `frontier_score.py` | standalone function | mechanical | Layered score helper. |
| `_cf_build_frontier_local_score_details` | 3525 | `frontier_score.py` | standalone function | mechanical but risky | Bundles local score sub-signals that feed telemetry too. |
| `_cf_score_candidate_layered` | 3601 | `frontier_score.py` | standalone function | mechanical but risky | Active score entrypoint. |
| `_cf_score_candidate_legacy` | 3690 | `frontier_score.py` | standalone function retained as dead/legacy until P1 | behavioral / later | Do not delete in P0, but do not reactivate. |
| `_cf_score_candidate` alias | 3915 | `frontier_score.py` | alias to active entrypoint | mechanical | Preserve alias until P1/P7 clean-up. |

### `frontier_finalize.py`

| Current symbol | Current line | Future owner | Shape | Move class | Notes |
|---|---:|---|---|---|---|
| `_compute_patch_chain_gap_reports` | 4124 | `frontier_finalize.py` | standalone function | mechanical | Finalization-local diagnostics helper. |
| `_cf_build_envelopes` | 4165 | `frontier_finalize.py` | standalone function | mechanical but risky | Patch envelope assembly plus scaffold-connected-chain derivation. |
| `_finalize_quilt_scaffold_frontier` | 4306 | `frontier_finalize.py` | standalone function | mechanical but risky | Finalization order, diagnostics collection, and untouched-patch detection are locked. |

## `FrontierRuntimePolicy` decomposition

`FrontierRuntimePolicy` must be treated as a compatibility container, not as the final
owner of every behavior currently attached to it.

### Field ownership

| Field | Semantic owner | Storage host after split | Notes |
|---|---|---|---|
| `graph` | `frontier_state.py` | `FrontierRuntimePolicy` | Core immutable runtime context. |
| `quilt_patch_ids` | `frontier_state.py` | `FrontierRuntimePolicy` | Core immutable runtime context. |
| `allowed_tree_edges` | `frontier_state.py` | `FrontierRuntimePolicy` | Core immutable runtime context used by eval/rescue/score. |
| `final_scale` | `frontier_state.py` | `FrontierRuntimePolicy` | Core immutable runtime context. |
| `point_registry` | `frontier_state.py` | `FrontierRuntimePolicy` | Mutable point registry. |
| `vert_to_placements` | `frontier_state.py` | `FrontierRuntimePolicy` | Mutable vertex -> placed-point index. |
| `placed_chain_refs` | `frontier_state.py` | `FrontierRuntimePolicy` | Mutable placement membership set. |
| `placed_chains_map` | `frontier_state.py` | `FrontierRuntimePolicy` | Mutable placement storage. |
| `chain_dependency_patches` | `frontier_state.py` | `FrontierRuntimePolicy` | Mutable dependency store used during finalize. |
| `rejected_chain_refs` | `frontier_state.py` | `FrontierRuntimePolicy` | Mutable rejection set. |
| `build_order` | `frontier_state.py` | `FrontierRuntimePolicy` | Mutable placement order. |
| `placed_count_by_patch` | `frontier_state.py` | `FrontierRuntimePolicy` | Mutable patch counters. |
| `placed_h_count_by_patch` | `frontier_state.py` | `FrontierRuntimePolicy` | Mutable patch counters. |
| `placed_v_count_by_patch` | `frontier_state.py` | `FrontierRuntimePolicy` | Mutable patch counters. |
| `placed_free_count_by_patch` | `frontier_state.py` | `FrontierRuntimePolicy` | Mutable patch counters. |
| `_cached_evals` | `frontier_state.py` | `FrontierRuntimePolicy` | Eval cache; cache contract is state-owned even though eval populates it. |
| `_dirty_refs` | `frontier_state.py` | `FrontierRuntimePolicy` | Dirty propagation state. |
| `_vert_to_pool_refs` | `frontier_state.py` | `FrontierRuntimePolicy` | Static pool index used by state/score. |
| `_patch_to_pool_refs` | `frontier_state.py` | `FrontierRuntimePolicy` | Static patch -> pool index used by dirty propagation. |
| `_cache_hits` | `frontier_state.py` | `FrontierRuntimePolicy` | Selector accounting. |
| `closure_pair_map` | `frontier_closure.py` | `FrontierRuntimePolicy` | Closure map is closure-owned, state-hosted. |
| `closure_pair_refs` | `frontier_closure.py` | `FrontierRuntimePolicy` | Derived closure set is closure-owned, state-hosted. |
| `tree_ingress_partner_by_chain` | `frontier_rescue.py` | `FrontierRuntimePolicy` | Rescue-owned seam partner map, state-hosted. |
| `seam_relation_by_edge` | `frontier_score.py` | `FrontierRuntimePolicy` | Score-owned seam context, state-hosted. |
| `_outer_chain_count_by_patch` | `frontier_score.py` | `FrontierRuntimePolicy` | Score-owned derived cache. |
| `_frame_chain_count_by_patch` | `frontier_score.py` | `FrontierRuntimePolicy` | Score-owned derived cache. |
| `_closure_pair_count_by_patch` | `frontier_score.py` | `FrontierRuntimePolicy` | Score-owned derived cache built from closure state. |
| `_shape_profile_by_patch` | `frontier_score.py` | `FrontierRuntimePolicy` | Score-owned derived cache. |

### Method ownership

#### Retained concrete state methods

These stay as real methods on `FrontierRuntimePolicy` because they are direct state access or state mutation:

- `placed_in_patch`
- `placed_h_in_patch`
- `placed_v_in_patch`
- `placed_free_in_patch`
- `outer_chain_count`
- `frame_chain_count`
- `closure_pair_count`
- `shape_profile`
- `seam_relation`
- `total_placed`
- `is_chain_available`
- `placed_patch_ids`
- `reject_chain`
- `dependency_patches_from_anchors`
- `register_chain`

Notes:

- `register_chain()` remains state-owned, but must call extracted state helpers for
  point registration and dirty propagation.
- `reject_chain()` remains state-owned and must continue to clear cache/dirty entries.

#### Thin delegator methods

These should remain as temporary compatibility methods only:

- `evaluate_candidate` -> thin delegator to `frontier_eval.evaluate_candidate(...)`
- `build_stop_diagnostics` -> thin delegator to `frontier_eval.build_stop_diagnostics(...)`

#### Split initializer responsibilities

`__post_init__` is too mixed to remain monolithic. It must be decomposed into:

- state bootstrap: initialize closure refs and empty mutable stores
- closure-derived bootstrap: closure-pair ref materialization
- score-cache bootstrap: outer/frame/closure counts plus shape profiles

Compatibility rule:

- keep `__post_init__` if the dataclass still needs it, but make it a coordinator that
  calls extracted helpers instead of owning the logic directly

## `solve_records.py` ownership classification

This section is conceptual ownership only. Records remain physically in `solve_records.py`.

### State-owned frontier aliases

- `PointRegistryKey`
- `VertexPlacementRef`
- `PointRegistry`
- `VertexPlacementMap`
- `AnchorRefPair`

### Orchestrator-owned frontier records

- `ChainPoolEntry`
- `FrontierBootstrapResult`
- `FrontierBootstrapAttempt`

### Eval-owned records

- `ChainAnchor`
- `FoundCandidateAnchors`
- `ResolvedCandidateAnchors`
- `AnchorPairSafetyDecision`
- `DualAnchorClosureDecision`
- `ClosurePreconstraintApplication`
- `DirectionOption`
- `AnchorOption`
- `ClosurePreconstraintOptionResult`
- `ClosurePreconstraintMetric`
- `FrontierCandidateEval`
- `FrontierStopDiagnostics`
- `FrontierPlacementCandidate`
- `SeedChainChoice`
- `DualAnchorRectificationPreview`

### Placement-owned records

- `SeedPlacementResult`

### Closure-owned records

- `ClosureChainPairMatch`
- `ClosureChainPairCandidate`
- `SharedClosureUvOffsets`

### Score-owned records

- `FrontierTopologyFacts`
- `FrontierRank`
- `FrontierRankBreakdown`
- `PatchShapeProfile`
- `PatchScoringContext`
- `CornerScoringHints`
- `FrontierLocalScoreDetails`
- `FrameGroupMember`

### Rescue-owned records

- `FrontierRescueGap`
- `FrontierRescueGapClassCount`
- `ClosureFollowUvBuildResult`
- `ClosureFollowPlacementCandidate`
- `FreeIngressPlacementCandidate`
- `TreeIngressPlacementCandidate`
- `ClosureFollowCandidateRank`
- `FreeIngressCandidateRank`
- `TreeIngressCandidateRank`

### Finalize-owned records

- `PatchChainGapDiagnostics`
- `FinalizedQuiltScaffold`

### Shared solve-layer / planning-owned records

- `SolveView`
- `QuiltPlan`
- `QuiltStep`
- `AttachmentCandidate`
- `SeamRelationProfile`

These are shared planning/solve inputs and should not be re-homed during this refactor.

### Shared instrumentation-facing records

- `FrontierPlacementRecord`
- `FrontierStallRecord`
- `QuiltFrontierTelemetry`

These stay solve-layer shared because instrumentation owns collection, but frontier modules emit them.

## Risk ledger for later extraction phases

### 1. Runtime cache / dirty propagation

Risk class: highest mechanical risk

Locked zone:

- `_cached_evals`
- `_dirty_refs`
- `_vert_to_pool_refs`
- `_patch_to_pool_refs`
- `_mark_neighbors_dirty`
- `register_chain`
- `reject_chain`
- `_cf_select_best_frontier_candidate`

Rules:

- Cached selection must stay bit-identical to a full scan.
- Dirty propagation after anchor adjustment and first-in-patch placement must not change order.
- Do not opportunistically redesign the selector in extraction phases.

### 2. Rescue flows

Risk class: highest mechanical risk

Locked zone:

- `_cf_try_place_tree_ingress_candidate`
- `_cf_try_place_closure_follow_candidate`
- `_cf_build_frontier_rescue_gap`
- rescue telemetry payloads

Rules:

- Main frontier stall must attempt `tree_ingress` before `closure_follow`.
- Rescue remains a cold path, not merged into main frontier ranking.
- Rescue gap telemetry semantics stay unchanged.

### 3. Anchor resolution

Risk class: high mechanical risk

Locked zone:

- `_cf_find_anchors`
- `_cf_resolve_candidate_anchors`
- `_cf_frame_anchor_pair_is_axis_safe`
- `_cf_can_use_dual_anchor_closure`
- `_cf_apply_closure_preconstraint`
- `_try_inherit_direction`

Rules:

- Same-patch vs cross-patch provenance must stay intact.
- Prevent-patch-wrap semantics are architectural.
- Closure preconstraint stays in eval, not closure-map ownership.

### 4. Scoring / rank construction

Risk class: high mechanical risk

Locked zone:

- patch shape profile caches
- patch scoring context
- topology facts
- layered score helpers
- structured rank and debug breakdown
- seam and corner hints

Rules:

- Scalar threshold gate and structured rank ordering must not drift.
- Score helpers may move, but formulas and tier order are locked in mechanical phases.

### 5. Finalization

Risk class: medium/high mechanical risk

Locked zone:

- `_cf_build_envelopes`
- `_compute_patch_chain_gap_reports`
- `_finalize_quilt_scaffold_frontier`
- untouched-patch detection
- final fallback to `_restore_original_quilt_plan(...)`

Rules:

- Finalization order and diagnostics collection stay unchanged.
- Finalize may bridge to `solve_pin_policy.py`; do not hide that dependency.

### 6. Dead or disabled code branches

Status for later triage, not for P0 behavior change:

- `_cf_preview_frame_dual_anchor_rectification`: active behavior is the early return only; body below it is dead/disabled and belongs to P1.
- `_cf_score_candidate_legacy`: retained legacy scorer; keep isolated, do not reactivate.
- `_cf_score_candidate` alias: preserve until P1/P7 cleanup to avoid accidental entrypoint drift.

## Extraction guidance by phase

- P1 should isolate dead branches before moving them.
- P2 can safely extract placement math first because its ownership is now explicit.
- P3 can move finalize without guessing whether scaffold connectivity is finalize-owned.
- P4 can move rescue without guessing whether closure-follow logic belongs to closure-map ownership.
- P5 must keep `FrontierRuntimePolicy` as the lowest frontier layer and only extract substrate/state behavior.
- P6 should convert `evaluate_candidate()` and `build_stop_diagnostics()` into thin delegators.
- P7 should own score caches previously initialized inside `FrontierRuntimePolicy.__post_init__`.

## Short handoff note for the next agent

Mapped:

- all major `solve_frontier.py` helpers into `frontier_closure.py`, `frontier_rescue.py`,
  `frontier_state.py`, `frontier_eval.py`, `frontier_place.py`, `frontier_score.py`,
  `frontier_finalize.py`, or retained orchestrator ownership
- every `FrontierRuntimePolicy` field/method by semantic owner and storage/delegation shape
- frontier-related records in `solve_records.py` by owning subsystem

Not touched:

- runtime logic in `cftuv/solve_frontier.py`
- `cftuv/solve_records.py`
- any non-doc file

Key risks:

- cache/dirty propagation must remain bit-identical
- rescue ordering and telemetry payloads are fragile
- anchor resolution and prevent-patch-wrap semantics must not drift
- finalize order and untouched-patch fallback must remain identical
