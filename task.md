# CFTUV Band Spine Parametrization — Task Checklist

## Context

BAND patches (PatchShapeClass.BAND) have 4 chains: 2 SIDE (STRAIGHTEN) + 2 CAP (FREE).
Current problem: CAP chains fall to FREE in frontier, get conformal unfold instead of scaffold pin.
Root cause chain: `ambiguous_caps` rejection blocks `band_confirmed_for_runtime` -> `_patch_allows_straighten_runtime` returns False -> continuation role promotion blocked -> CAPs stay FREE -> pinned=30/60.

Solution: spine-based pre-parametrization. Pre-compute UV for ALL 4 chains of BAND patch using a midpoint Bezier spine between two SIDE chains. Frontier uses these pre-computed UVs instead of standard axis projection.

---

## Phase 0: Bug Fix — `ambiguous_caps` Runtime Rejection

### 0.1 Diagnose `ambiguous_caps`
- [ ] Read `analysis_derived.py` lines 780-812
- [ ] `runtime_cap_indices` is built from `runtime_role_sequence` (line 808-812)
- [ ] `runtime_role_by_chain_index` (line 786-793) only overrides FREE chains
- [ ] If a chain already has inherited H/V effective_role, it stays — but may match `orthogonal_axis`
- [ ] This creates >2 chains with orthogonal role -> `len(runtime_cap_indices) != 2` -> `ambiguous_caps`
- [ ] Root cause: inherited role from cross-patch neighbor adds a 3rd "cap" to the count

### 0.2 Fix `ambiguous_caps`
- [ ] In `analysis_derived.py` line 800-812: when building `runtime_cap_indices` and `runtime_side_indices`, cross-reference with `cap_candidate_indices` and `side_candidate_indices` from structural detection
- [ ] Option: if `patch_shape_classes[patch_id] == BAND` (from structural_tokens), trust the structural SIDE/CAP assignment and skip the runtime heuristic
- [ ] OR: filter `runtime_cap_indices` to only include chains that are in `cap_candidate_indices`
- [ ] Verify: after fix, P2 and P4 should show `band_confirmed_for_runtime=True` instead of `ambiguous_caps`

---

## Phase 1: BandSpineData — Pre-computation

### 1.1 Create `BandSpineData` dataclass
- [ ] File: `cftuv/analysis_records.py` (add after `_PatchDerivedTopologySummary`)
- [ ] Fields:
  ```
  patch_id: int
  side_a_ref: ChainRef        # first SIDE chain ref
  side_b_ref: ChainRef        # second SIDE chain ref
  cap_start_ref: ChainRef     # CAP at spine start (V=0)
  cap_end_ref: ChainRef       # CAP at spine end (V=1)
  spine_points_3d: tuple[Vector, ...]   # midpoint Bezier control points
  spine_arc_length: float               # total arc length of spine
  cap_start_width: float                # 3D distance between SIDEs at spine start
  cap_end_width: float                  # 3D distance between SIDEs at spine end
  chain_uv_targets: dict[ChainRef, tuple[tuple[float, float], ...]]
      # pre-computed (U, V) for each vertex of each chain
  ```
- [ ] Frozen dataclass, immutable after construction

### 1.2 Build spine midpoint curve
- [ ] File: NEW `cftuv/band_spine.py`
- [ ] Function: `build_band_spine(graph, patch_id, side_a_ref, side_b_ref, cap_start_ref, cap_end_ref) -> BandSpineData`
- [ ] Algorithm:
  1. Get `side_a.vert_cos` and `side_b.vert_cos` (3D world coords)
  2. Ensure both SIDEs are oriented consistently (same direction along band). If anti-parallel, reverse one.
  3. Resample both SIDEs to same number of points (e.g., max(len_a, len_b) or interpolated)
  4. Compute midpoints: `spine[i] = (side_a[i] + side_b[i]) / 2`
  5. Fit Bezier through midpoints (or use midpoints directly as polyline)
  6. Compute arc-length parametrization: cumulative distances along spine, normalized to [0, 1]

### 1.3 Compute U/V for each chain vertex
- [ ] Function: `_compute_chain_uv(spine_points, spine_arc_lengths, chain_vert_cos, is_side, side_sign) -> tuple[tuple[float, float], ...]`
- [ ] For SIDE chains:
  - V = project each vertex onto spine, find nearest spine segment, get normalized arc length at projection point
  - U = signed distance from spine to vertex (positive for side_a, negative for side_b)
- [ ] For CAP chains:
  - V = 0.0 (start cap) or 1.0 (end cap) — or interpolated if CAP is not perfectly orthogonal
  - U = varies across CAP from side_a position to side_b position, normalized

### 1.4 Determine SIDE orientation consistency
- [ ] SIDEs share corners with CAPs. Use corner connectivity to determine which SIDE endpoint connects to which CAP
- [ ] side_a goes from cap_start corner to cap_end corner
- [ ] side_b goes in same direction (cap_start to cap_end)
- [ ] If side_b is reversed in boundary loop, flip its vert_cos order before midpoint computation

### 1.5 Store spine data in topology bundle
- [ ] In `analysis_derived.py` `_build_patch_graph_derived_topology()`:
  - After structural_tokens classify BAND patches (line 1055-1063)
  - For each BAND patch, call `build_band_spine()`
  - Collect into `band_spine_data: dict[int, BandSpineData]` (patch_id -> spine)
- [ ] Add field to `_PatchGraphDerivedTopology` (analysis_records.py):
  ```
  band_spine_data: dict[int, BandSpineData]   # keyed by patch_id
  ```

---

## Phase 2: Data Flow Integration

### 2.1 Pass spine data through analysis pipeline
- [ ] `analysis.py` `build_straighten_structural_support()`:
  - Currently returns 4-tuple. Add `band_spine_data` as 5th element
  - Return: `(inherited_role_map, patch_structural_summaries, patch_shape_classes, straighten_chain_refs, band_spine_data)`
- [ ] `operators.py`: unpack 5-tuple, pass `band_spine_data` to frontier
- [ ] `solve_transfer.py`: same 5-tuple unpacking

### 2.2 Pass to frontier builder
- [ ] `solve_frontier.py` `build_root_scaffold_map()`: accept `band_spine_data` parameter
- [ ] `solve_frontier.py` `build_quilt_scaffold_chain_frontier()`: accept and pass to runtime
- [ ] `frontier_state.py` `FrontierRuntimePolicy.__init__()`: accept and store `band_spine_data`

---

## Phase 3: Frontier Placement Integration

### 3.1 CAP role promotion
- [ ] In `frontier_state.py` `effective_placement_role()` (line 109):
  - After STRAIGHTEN check (line 120-121), add CAP check:
  - If `chain_ref` is a CAP of a BAND patch (check via `band_spine_data`), return the orthogonal axis role
  - CAP of H_FRAME spine -> V_FRAME role; CAP of V_FRAME spine -> H_FRAME role
  - This makes CAPs get H/V role -> they get scored, placed, and pinned properly

### 3.2 Spine UV as placement target
- [ ] In `frontier_place.py` `_cf_place_chain()` (line 1213):
  - When placing a chain that belongs to a BAND patch with spine data:
  - Instead of standard `_build_frame_chain_from_stations()` or `_build_guided_free_chain_between_anchors()`
  - Use spine-derived UV from `BandSpineData.chain_uv_targets[chain_ref]`
  - Transform spine UV to world UV: apply offset + scale from anchor position
- [ ] New function: `_build_spine_chain_placement(chain, node, spine_data, anchor_uv_offset, anchor_uv_scale) -> list[tuple[ScaffoldPointKey, Vector]]`
  - Takes pre-computed (U, V) from spine, applies anchor transform
  - Returns list of (key, uv_vector) pairs

### 3.3 Anchor drift correction
- [ ] When CAP has cross-patch anchor that differs from spine-predicted position:
  - Compute scale factor: `real_cap_width / spine_cap_width`
  - Apply scale to U-coordinates of all chains on that end of the band
  - If both CAPs have anchors with different scales, interpolate scale along V
- [ ] Function: `_apply_drift_correction(spine_data, cap_start_anchor, cap_end_anchor) -> dict[ChainRef, tuple[tuple[float, float], ...]]`
  - Returns corrected UV targets

### 3.4 CAP scoring boost
- [ ] In `frontier_score.py` `_cf_role_tier()` (line 233):
  - Add case: if chain is CAP of BAND with spine data:
  - Tier 2.5 or same tier as STRAIGHTEN but with priority label `'band_cap_spine'`
  - This ensures CAPs are placed before FREE chains but after native H/V

---

## Phase 4: Pin Policy Update

### 4.1 Pin all BAND spine chains
- [ ] In `solve_pin_policy.py` `_decide_chain_pin()` (line 95):
  - Add rule: if chain belongs to BAND with spine data -> `pin_all=True`
  - Applies to both SIDEs AND CAPs
  - Reason: `'band_spine_connected'`
- [ ] All 4 chains of BAND get fully pinned, not just endpoints

### 4.2 Scaffold connectivity for BAND
- [ ] In `solve_pin_policy.py` `_compute_scaffold_connected_chains()`:
  - BAND spine chains should be considered scaffold-connected to each other
  - Even if they don't share H/V frame axis, they share spine reference frame

---

## Phase 5: Validation & Cleanup

### 5.1 Verify fix
- [ ] Run on test mesh with BAND patches
- [ ] Check logs: `band_confirmed_for_runtime=True` (no `ambiguous_caps`)
- [ ] Check: all 4 chains placed with H/V roles (no `[BRIDGE]` on CAPs)
- [ ] Check: `pinned=60/60` for BAND patches (not 30/60)
- [ ] Check: UV shape matches 3D band geometry (no distortion)

### 5.2 Remove dead code
- [ ] Remove `band_operator` imports in `solve_frontier.py` (if any remain)
- [ ] Remove diagnostic `print()` from `structural_tokens.py` classifier
- [ ] Clean up `_band_geometric_role()` in `frontier_state.py` if superseded

### 5.3 Update documentation
- [ ] `AGENTS.md`: add spine parametrization to module layout, update P7 priority
- [ ] `docs/cftuv_reference.md`: add spine rules (S8-S10)
- [ ] `docs/cftuv_architecture.md`: add `band_spine.py` module, update pipeline diagram

---

## Phase 6: Conflicts in Quilt 0 (Secondary)

### 6.1 Investigate 2 conflicts
- [ ] Conflicts are in P3 (conflicts=1) and P5 (conflicts=1) — non-BAND patches in Q0
- [ ] From `FrameDiag`: COLUMN V_FRAME scatter=1.208116 on chains P3L0C2, P5L0C1
- [ ] Likely cause: `frame_dual_anchor_rectify:keep_start` — two anchors disagree slightly
- [ ] Phase1 max_gap=0.000021 (P3), 0.000026 (P5) — numerically insignificant
- [ ] May self-resolve after BAND spine fix improves adjacent patch anchoring
- [ ] If not: investigate dual-anchor rectification logic in `frontier_place.py`

---

## Execution Order

```
Phase 0 (bug fix)  ->  must be done first, unblocks runtime
Phase 1 (spine)    ->  core feature, can start after Phase 0
Phase 2 (plumbing) ->  data flow, depends on Phase 1 dataclass
Phase 3 (frontier) ->  placement logic, depends on Phase 2
Phase 4 (pin)      ->  pin rules, depends on Phase 3
Phase 5 (validate) ->  verify everything works
Phase 6 (conflicts)->  independent, investigate after Phase 5
```
