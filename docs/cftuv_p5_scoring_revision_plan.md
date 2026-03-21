# CFTUV P5: Scoring Revision Based on P3 Instrumentation
## Data-driven refinement of frontier candidate scoring

---

## Evidence Base (P3 Telemetry Analysis)

### Source: Cylinder.007 (11 patches, 7 quilts, 50 chains)

**Observation 1: Zero rescue firings.**
All 7 quilts: `rescue_ratio: 0.000`. Every placement through main path.
Every stall = pool exhausted (`available:0`). On this mesh class, rescue
paths are dead code.

**Observation 2: Discrete score clusters.**
Current scoring produces exactly 6 values:
```
0.60  — FREE, 1 anchor, same_patch          (base 0.0 + 0.3 + 0.2 + 0.1)
1.20  — FREE, 2 anchors, same_patch         (base 0.0 + 0.8 + 0.2 + 0.2)
1.30  — FREE bridge, 2 anchors, same_patch  (base 0.1 + 0.8 + 0.2 + 0.2)
1.60  — H/V, 1 anchor, same_patch           (base 1.0 + 0.3 + 0.2 + 0.1)
2.05  — Cross-patch H/V, 1 anchor           (base 2.0 + 0.3 - 0.25)
2.60  — Cross-patch H/V, 1 same_patch       (base 2.0 + 0.3 + 0.2 + 0.1)
```
No continuous factors. Within a cluster, tie-breaking is arbitrary.

**Observation 3: Low scaffold connectivity.**
```
P0: 3/6 connected    P1: 3/4    P2: 3/5
P3: 2/3              P9: 2/5    P10: 2/4
P6: 1/4              P5: 1/4    P4: 0/3 (conformal)
```
Cause: FREE chains break connectivity BFS. H/V chains isolated behind
FREE get `isolated_hv` pin reason = not pinned = scaffold work wasted.

**Observation 4: Column scatter on closure.**
`max_column_scatter: 0.784` on `P1L0C2<->P2L0C0` (closure-sensitive V_FRAME pair).
`cross: 1.568` offset on the closure seam. These chains were placed independently
without cross-axis coordination.

**Observation 5: FREE chain ordering is suboptimal.**
Quilt 0 placement order shows FREE chains placed last (steps 9-14, scores 0.60-1.20)
after all H/V. This is by design (H/V base >> FREE base), but means FREE chains
have no influence on scaffold shape — they just fill gaps.

### Data Limitations

Single mesh analyzed. Rescue paths may still fire on:
- Ring/cylinder meshes (closure-heavy)
- Large meshes with >50 patches (deep tree traversal)
- Meshes with HOLE loops
- Bevel-heavy geometry (many corner-split chains)

**P5 changes must be validated on the full regression mesh set.**
Any change that improves Cylinder.007 but breaks other meshes = regression.

---

## Goals

1. **Reduce score discretization.** Introduce continuous factors so chains
   within the same role cluster get meaningful differentiation.

2. **Improve scaffold connectivity ratio.** Score chains higher when their
   placement would extend the connected scaffold skeleton.

3. **Prevent isolated H/V waste.** Detect chains that will be isolated
   (behind FREE gap, no BFS path to root) and score them lower.

4. **Refine FREE chain differentiation.** Distinguish structurally
   important FREE (connects two H/V, short bridge) from filler FREE.

5. **Zero regression on existing mesh set.** Every change validated by
   regression snapshot comparison. Behavioral improvements measured by
   scaffold_connected ratios and column/row scatter.

---

## Architecture

### No new modules. Changes in existing files only.

| File | Changes |
|------|---------|
| `solve_frontier.py` | `_cf_score_candidate()` — refined scoring formula |
| `solve_frontier.py` | New helper: `_cf_estimate_connectivity_gain()` |
| `constants.py` | New scoring weight constants |
| `solve_records.py` | Extended `FrontierPlacementRecord` with new score components |

No changes to: model.py, analysis.py, solve_planning.py, solve_transfer.py,
solve_pin_policy.py, debug.py, operators.py.

### Scoring changes are additive refinements, not rewrites.

Current formula structure preserved. New factors added as continuous
modifiers within existing score tiers. Existing production meshes that
already work well should see minimal score reordering.

---

## Specific Changes

### Change 1: Chain Length Factor (continuous)

**Problem:** All FREE chains with 1 anchor score 0.60. A 2-vertex FREE
and a 20-vertex FREE are indistinguishable.

**Fix:** Add `length_factor` as continuous multiplier within the FREE tier.

```python
# In _cf_score_candidate, after base score assignment:
if not is_hv:
    chain_len = _cf_chain_total_length(chain, final_scale)  # already available
    # Normalize to [0, 0.15] — small bonus, won't override tier structure
    length_factor = min(0.15, chain_len * 0.1)
    score += length_factor
```

**New constant:** `SCORE_FREE_LENGTH_SCALE = 0.1` in constants.py
**New constant:** `SCORE_FREE_LENGTH_CAP = 0.15` in constants.py

**Impact:** FREE chains within same anchor-count cluster get differentiated
by geometric importance. Longer FREE (more shape information for Conformal)
placed earlier.

### Change 2: Downstream Connectivity Bonus

**Problem:** Placing a chain that gives cross-patch anchors to 3 unplaced
neighbors scores the same as one that gives anchors to 0 neighbors.

**Fix:** Count how many currently-unplaced chains would gain an anchor
from this placement. Add scaled bonus.

```python
def _cf_estimate_downstream_anchor_count(
    chain_ref: ChainRef,
    chain: BoundaryChain,
    graph: PatchGraph,
    runtime_policy: FrontierRuntimePolicy,
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
```

```python
# In _cf_score_candidate:
downstream = _cf_estimate_downstream_anchor_count(chain_ref, chain, graph, runtime_policy)
downstream_bonus = min(0.20, downstream * 0.05)
score += downstream_bonus
```

**New constant:** `SCORE_DOWNSTREAM_SCALE = 0.05` in constants.py
**New constant:** `SCORE_DOWNSTREAM_CAP = 0.20` in constants.py

**Impact:** Chains at junction points (touching many unplaced chains)
get priority. Cross-patch ingress chains naturally score higher because
they unlock more downstream chains.

**Performance note:** Uses `_vert_to_pool_refs` (already built for
incremental frontier). No additional data structures needed.

### Change 3: Scaffold Isolation Penalty for H/V

**Problem:** H/V chains placed after a FREE gap become `isolated_hv` —
scaffold-disconnected, not pinned, work wasted. The scorer doesn't know
this will happen.

**Fix:** Preview scaffold connectivity before scoring. If the chain would
be isolated (no BFS path through H/V+bridge to any already-placed H/V
in same patch), apply penalty.

```python
def _cf_preview_would_be_connected(
    chain_ref: ChainRef,
    chain: BoundaryChain,
    runtime_policy: FrontierRuntimePolicy,
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
        # Connected if neighbor is H/V or bridge FREE
        if neighbor_chain.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
            return True
        if neighbor_chain.frame_role == FrameRole.FREE and len(neighbor_chain.vert_cos) <= 2:
            return True  # Bridge is transparent
    
    return False
```

```python
# In _cf_score_candidate, for H/V chains:
if is_hv and not _cf_preview_would_be_connected(chain_ref, chain, runtime_policy, graph):
    score -= 0.40  # Strong penalty — prefer placing connected H/V first
```

**New constant:** `SCORE_ISOLATED_HV_PENALTY = 0.40` in constants.py

**Impact:** H/V chains that would end up isolated get deferred until
their connecting FREE chain is placed. This increases scaffold_connected
ratios.

**Risk:** On some meshes, this might delay H/V placement and cause
worse overall layout. Mitigated by keeping penalty moderate (0.40) —
an isolated H/V at score 1.60 - 0.40 = 1.20 still beats most FREE at 0.60.

### Change 4: Structural FREE Bonus

**Problem:** All non-bridge FREE chains get base 0.0. But a FREE chain
connecting two H/V chains (forming the "floating side" of a strip patch)
is structurally more important than a FREE chain between two other FREEs.

**Fix:** Check neighbor roles and add bonus for FREE chains that connect
frame chains.

```python
# In _cf_score_candidate, for non-bridge FREE:
if not is_hv and not is_bridge:
    # Check if this FREE connects two different H/V chains
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
        score += 0.10  # Strip connector bonus
    elif start_has_hv or end_has_hv:
        score += 0.05  # One-sided frame neighbor
```

**New constants:** `SCORE_FREE_STRIP_CONNECTOR = 0.10`, `SCORE_FREE_FRAME_NEIGHBOR = 0.05`

**Impact:** Strip-shaped patches (H, FREE, V, FREE) get their FREE chains
placed in better order — connectors first, fillers last.

---

## What NOT To Change in P5

- **Tier structure preserved.** Cross-patch H/V > regular H/V > FREE.
  New factors are continuous modifiers within tiers, not tier overrides.
- **Rescue paths untouched.** Even if they fire on other meshes, P5 doesn't
  change rescue logic — only main scoring.
- **Threshold unchanged.** `FRONTIER_MINIMUM_SCORE = 0.30` stays.
  `FRONTIER_PROPAGATE_THRESHOLD = 0.45` stays.
- **Closure preconstraint untouched.** Direction-flip and anchor-swap
  heuristics remain as-is.
- **Pin policy untouched** (that's P6).
- **Analysis untouched.** Chain roles, corner detection, boundary tracing
  all stay the same.

---

## Phases

### Phase A: Add constants and extend telemetry record

- [ ] Add new scoring constants to `constants.py`:
  ```
  SCORE_FREE_LENGTH_SCALE = 0.1
  SCORE_FREE_LENGTH_CAP = 0.15
  SCORE_DOWNSTREAM_SCALE = 0.05
  SCORE_DOWNSTREAM_CAP = 0.20
  SCORE_ISOLATED_HV_PENALTY = 0.40
  SCORE_FREE_STRIP_CONNECTOR = 0.10
  SCORE_FREE_FRAME_NEIGHBOR = 0.05
  ```
- [ ] Extend `FrontierPlacementRecord` in solve_records.py with new fields:
  ```
  length_factor: float = 0.0
  downstream_count: int = 0
  downstream_bonus: float = 0.0
  isolation_preview: bool = True     # would_be_connected
  isolation_penalty: float = 0.0
  structural_free_bonus: float = 0.0
  ```
- [ ] Verify: addon loads, no import errors

### Phase B: Implement helper functions

- [ ] Add `_cf_estimate_downstream_anchor_count()` to solve_frontier.py
- [ ] Add `_cf_preview_would_be_connected()` to solve_frontier.py
- [ ] Both are pure functions — no side effects, no state mutation
- [ ] Verify: functions callable, return expected values for manual test cases

### Phase C: Modify `_cf_score_candidate()`

- [ ] Add `length_factor` for FREE chains (Change 1)
- [ ] Add `downstream_bonus` (Change 2)
- [ ] Add `isolation_penalty` for H/V chains (Change 3)
- [ ] Add `structural_free_bonus` for FREE chains (Change 4)
- [ ] **Pass `runtime_policy` as parameter** to `_cf_score_candidate()` —
      currently it receives individual fields; needs access to
      `_vert_to_pool_refs`, `placed_chain_refs`, `placed_in_patch()`
- [ ] Update all call sites of `_cf_score_candidate()` (in
      `FrontierRuntimePolicy.evaluate_candidate`)

### Phase D: Wire new score components into telemetry emission

- [ ] Update `record_placement()` calls in solve_frontier.py to include
      new score component fields
- [ ] Update `FrontierTelemetryCollector` if needed to handle new fields
- [ ] Update telemetry summary/detail formatting in solve_instrumentation.py

### Phase E: Regression testing and tuning

- [ ] Run regression snapshot on Cylinder.007 — compare with baseline
- [ ] **Expected improvements on Cylinder.007:**
  - `scaffold_connected` ratios increase (fewer isolated_hv)
  - Score distribution less clustered (more unique values)
  - Build order may change (FREE connectors placed earlier)
- [ ] **Expected preserved on Cylinder.007:**
  - All patches COMPLETE
  - No unsupported/invalid patches
  - Closure seam metrics same or better
- [ ] Run on ALL regression mesh set — check for regressions
- [ ] If regression found: identify which change caused it, adjust
      constants, re-test
- [ ] **Tuning protocol:** Adjust constants in 0.05 increments. Each
      adjustment = full regression run. Max 3 tuning rounds.

### Phase F: Documentation

- [ ] Update AGENTS.md: add new scoring constants documentation
- [ ] Update cftuv_architecture.md: mark P5 status, describe revised scoring
- [ ] Update cftuv_reference.md: Scoring Weights section with new constants
- [ ] Update regression snapshot baselines for all meshes

---

## Verification Protocol

### Per-mesh checklist after Phase E:

```
Mesh: ___________
Previous baseline snapshot: ___________

1. All patches COMPLETE:                    [ ] same  [ ] better  [ ] worse
2. Unsupported patches:                     [ ] same  [ ] better  [ ] worse
3. Invalid closure:                         [ ] same  [ ] better  [ ] worse
4. scaffold_connected ratios:               [ ] same  [ ] better  [ ] worse
5. Row scatter:                             [ ] same  [ ] better  [ ] worse
6. Column scatter:                          [ ] same  [ ] better  [ ] worse
7. Closure seam mismatch:                   [ ] same  [ ] better  [ ] worse
8. Closure seam phase:                      [ ] same  [ ] better  [ ] worse
9. rescue_ratio:                            [ ] same  [ ] better  [ ] worse
10. isolated_hv count:                      [ ] same  [ ] better  [ ] worse
11. Build order:                            [ ] same  [ ] changed (explain)
12. UV visual inspection:                   [ ] same  [ ] better  [ ] worse

Decision: [ ] PASS  [ ] INVESTIGATE  [ ] REVERT change ___
```

### Regression thresholds (any of these = REVERT):

- Any patch that was COMPLETE becomes UNSUPPORTED
- scaffold_connected ratio decreases by >20% on any patch
- Column/row scatter increases by >0.1 on any quilt
- Closure seam phase increases by >0.01 on any seam
- New rescue firings appear where there were none before (may indicate
  main scoring now rejects valid candidates)

---

## Files Changed

| Phase | File | Change type |
|-------|------|-------------|
| A | constants.py | Add 7 constants |
| A | solve_records.py | Extend FrontierPlacementRecord (6 fields) |
| B | solve_frontier.py | Add 2 helper functions (~50 lines) |
| C | solve_frontier.py | Modify `_cf_score_candidate` (~30 lines) |
| C | solve_frontier.py | Update `evaluate_candidate` call site |
| D | solve_frontier.py | Update telemetry emission (~10 lines) |
| D | solve_instrumentation.py | Handle new fields in formatting |
| F | AGENTS.md, cftuv_architecture.md, cftuv_reference.md | Documentation |

**Never touched:** model.py, analysis.py, analysis_*.py, solve_planning.py,
solve_transfer.py, solve_pin_policy.py, solve_diagnostics.py, debug.py,
operators.py, console_debug.py, __init__.py

---

## Risk Assessment

**Risk: Build order changes cascade into closure seam degradation.**
Scoring changes reorder placements. Different order → different anchor
provenance → different UV coordinates → potentially worse closure alignment.
Mitigation: Regression threshold on closure metrics. Revert if exceeded.

**Risk: Downstream anchor count is expensive to compute per candidate.**
`_cf_estimate_downstream_anchor_count` walks corners + vert_to_pool_refs.
For N available chains, this is O(N × avg_neighbors). With incremental
cache, only dirty candidates are re-evaluated — so amortized cost is low.
Mitigation: Profile on largest mesh. If >10% slowdown, cap neighbor scan.

**Risk: Isolation preview causes oscillation.**
Chain A is isolated → scored low → chain B (FREE connector) placed first →
now chain A is connected → chain A scored high. This is correct behavior,
not oscillation — the incremental dirty marking ensures chain A gets
re-evaluated after chain B is placed.

**Risk: Tuning constants on one mesh overfits.**
Mitigated by regression protocol requiring ALL meshes to pass.
Constants chosen to be moderate (max bonus 0.20, max penalty 0.40)
so no single factor dominates.

---

## Data Collection for Future P5 Rounds

After P5 round 1 is deployed, collect telemetry on the full mesh set.
Compare before/after:

| Metric | Cylinder.007 before | Target after |
|--------|-------------------|-------------|
| rescue_ratio | 0.000 | 0.000 (preserve) |
| scaffold_connected avg | ~55% | >70% |
| isolated_hv count | 3 | 0-1 |
| column_scatter max | 0.784 | <0.5 (stretch) |
| score unique values | 6 | >10 |

If other meshes show rescue firings in the before data, add rescue-specific
metrics to the comparison.
