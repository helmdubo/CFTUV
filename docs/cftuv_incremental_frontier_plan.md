# CFTUV Incremental Frontier Optimization Plan
## Performance fix for O(N²) candidate evaluation in solve_frontier.py

---

## Problem Statement

`build_quilt_scaffold_chain_frontier()` main loop (line ~2720) calls
`_cf_select_best_frontier_candidate()` on every iteration. That function
scans the **entire** chain pool and runs full `evaluate_candidate()` for
each available chain — including anchor search, resolution, closure
preconstraint, and scoring.

For a quilt with N chains this gives N iterations × N evaluations = O(N²)
full evaluations. Each evaluation is non-trivial: `_cf_find_anchors` walks
corners + registries, `_cf_resolve_candidate_anchors` does safety checks,
`_cf_apply_closure_preconstraint` builds temporary placements and metrics,
`_cf_score_candidate` does graph lookups.

## Solution

Cache evaluation results. After placing chain X, mark only chains whose
anchor state **could have changed** as dirty. Re-evaluate only dirty chains;
serve all others from cache.

## Critical Invariant

**Incremental frontier must produce bit-identical output to the current
full-scan frontier.** Same build_order, same UV points, same scaffold.
If output differs — dirty marking is incomplete, which is a bug.

This invariant must be verified after every phase.

---

## Strategy

- All changes in `solve_frontier.py` only
- No changes to solve_records.py, solve_planning.py, solve_diagnostics.py,
  solve_transfer.py, solve_reporting.py, model.py, analysis.py, operators.py
- No behavior change — pure performance optimization
- Phased execution: each phase independently testable
- Rescue paths (tree_ingress, free_ingress, closure_follow) left as full-scan
  in first pass — they fire only on stall, not hot path

---

## Dirty Propagation Model

When chain X is placed, the following chains may have changed scores:

| Cause | Affected chains | Why |
|-------|----------------|-----|
| Same-patch corner neighbor | Chains adjacent to X in same loop via corners | X's endpoints become same_patch anchors for neighbors |
| Cross-patch vertex sharing | Chains in other patches that share start/end vert with X | X's endpoints become cross_patch anchors |
| Closure partner | The closure pair partner of X (if exists) | Partner's closure preconstraint now sees X as placed |
| First chain in patch | All chains of same patch if X is the first placed there | `placed_in_patch` changes from 0→1, giving +0.2 score bonus |
| Anchor adjustment | Neighbors of any chain whose UV points were adjusted | Adjusted UV propagates to anchors of adjacent chains |

Chains NOT in any of these categories retain their cached score exactly.

---

## Phase A: Add cache infrastructure to FrontierRuntimePolicy

### What to add

Two new fields in `FrontierRuntimePolicy.__init__` / field declarations:

```python
_cached_evals: dict[ChainRef, FrontierCandidateEval] = field(default_factory=dict)
_dirty_refs: set[ChainRef] = field(default_factory=set)
```

Underscore prefix: these are internal cache, not part of the public contract.

### Rules

- `_cached_evals` stores the last `FrontierCandidateEval` returned by
  `evaluate_candidate()` for each chain_ref
- `_dirty_refs` stores chain_refs that need re-evaluation on next selection
- Both start empty. Phase B will populate them.

### What NOT to change

- Do NOT modify `evaluate_candidate()` yet
- Do NOT modify `_cf_select_best_frontier_candidate()` yet
- Do NOT modify `register_chain()` yet
- Just add the two fields

### Verification

- Addon loads and runs identically (fields exist but are unused)
- Run Analyze + Phase 1 Preview on a production mesh — identical output

---

## Phase B: Implement dirty marking in register_chain()

### What to add

A new private method `_mark_neighbors_dirty()` called at the end of
`register_chain()`.

### Implementation specification

```python
def _mark_neighbors_dirty(self, chain_ref: ChainRef, chain: BoundaryChain) -> None:
    """Mark chains whose anchor state may have changed after placing chain_ref."""
    patch_id, loop_index, chain_index = chain_ref

    # --- 1. Same-patch corner neighbors ---
    node = self.graph.nodes.get(patch_id)
    if node is not None and 0 <= loop_index < len(node.boundary_loops):
        boundary_loop = node.boundary_loops[loop_index]
        for corner in boundary_loop.corners:
            if corner.next_chain_index == chain_index:
                neighbor_ref = (patch_id, loop_index, corner.prev_chain_index)
                if self.is_chain_available(neighbor_ref):
                    self._dirty_refs.add(neighbor_ref)
            if corner.prev_chain_index == chain_index:
                neighbor_ref = (patch_id, loop_index, corner.next_chain_index)
                if self.is_chain_available(neighbor_ref):
                    self._dirty_refs.add(neighbor_ref)

    # --- 2. Cross-patch vertex sharing ---
    for vert_idx in (chain.start_vert_index, chain.end_vert_index):
        if vert_idx < 0 or vert_idx not in self.vert_to_placements:
            continue
        for other_ref, _ in self.vert_to_placements[vert_idx]:
            if other_ref == chain_ref:
                continue
            if other_ref in self.placed_chain_refs:
                continue
            if self.is_chain_available(other_ref):
                self._dirty_refs.add(other_ref)

    # --- 3. Closure partner ---
    if self.closure_pair_map is not None:
        partner_ref = self.closure_pair_map.get(chain_ref)
        if partner_ref is not None and self.is_chain_available(partner_ref):
            self._dirty_refs.add(partner_ref)

    # --- 4. First chain in patch bonus ---
    # When placed_count goes from 0 to 1, all same-patch chains get +0.2.
    # Check BEFORE incrementing counter (register_chain increments it).
    # IMPORTANT: this check must use the count AFTER increment, so we
    # check == 1 (just became 1 on this call).
    if self.placed_count_by_patch.get(patch_id, 0) == 1:
        if node is not None:
            for li, boundary_loop in enumerate(node.boundary_loops):
                for ci in range(len(boundary_loop.chains)):
                    ref = (patch_id, li, ci)
                    if ref != chain_ref and self.is_chain_available(ref):
                        self._dirty_refs.add(ref)
```

### Where to call it

At the **end** of `register_chain()`, after `_cf_register_points`:

```python
def register_chain(self, chain_ref, chain, chain_placement, uv_points, dependency_patches=()):
    # ... existing code ...
    self.placed_count_by_patch[chain_ref[0]] = self.placed_count_by_patch.get(chain_ref[0], 0) + 1
    _cf_register_points(chain_ref, chain, uv_points, self.point_registry, self.vert_to_placements)
    self._mark_neighbors_dirty(chain_ref, chain)  # <-- ADD THIS
```

### IMPORTANT: first-chain-in-patch timing

`register_chain` increments `placed_count_by_patch` before calling
`_mark_neighbors_dirty`. So the check inside the method must be
`== 1` (counter just became 1), not `== 0` (was 0 before increment).

### What NOT to change

- Do NOT modify `evaluate_candidate()` yet
- Do NOT modify `_cf_select_best_frontier_candidate()` yet
- `_dirty_refs` gets populated but not consumed yet

### Verification

- Run with a `print(f"dirty after placing {chain_ref}: {len(self._dirty_refs)}")` 
  inside `_mark_neighbors_dirty` to confirm dirty sets are small (2-8 typically)
- Output still identical (dirty marking has no consumers yet)

---

## Phase C: Wire cache into _cf_select_best_frontier_candidate()

### What to change

Replace the body of `_cf_select_best_frontier_candidate()`:

```python
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

        # --- Cache path ---
        if chain_ref not in runtime_policy._dirty_refs:
            cached = runtime_policy._cached_evals.get(chain_ref)
            if cached is not None:
                if cached.known == 0:
                    continue
                if cached.score > best_score:
                    best_score = cached.score
                    best_candidate = FrontierPlacementCandidate(
                        chain_ref=chain_ref,
                        chain=entry.chain,
                        node=entry.node,
                        start_anchor=cached.start_anchor,
                        end_anchor=cached.end_anchor,
                        anchor_reason=cached.anchor_reason,
                        anchor_adjustments=cached.anchor_adjustments,
                        closure_dir_override=cached.closure_dir_override,
                        score=cached.score,
                    )
                continue

        # --- Full evaluation path ---
        candidate_eval = runtime_policy.evaluate_candidate(
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
```

### Bootstrap: first iteration must be all-dirty

After `_cf_build_frontier_chain_pool()` in `build_quilt_scaffold_chain_frontier()`,
mark all pool entries as dirty:

```python
all_chain_pool = _cf_build_frontier_chain_pool(...)

# Bootstrap: first iteration evaluates everything
for entry in all_chain_pool:
    runtime_policy._dirty_refs.add(entry.chain_ref)
```

This ensures the first iteration is a full scan (identical to current behavior),
and subsequent iterations use the cache.

### What NOT to change

- Do NOT touch rescue paths yet (tree_ingress, free_ingress, closure_follow)
- Do NOT touch `evaluate_candidate()` internals
- Do NOT change scoring logic

### Verification — THE CRITICAL TEST

Run on **every** production mesh from the regression checklist:

1. Save regression snapshot WITH the optimization
2. Compare bit-for-bit with snapshot WITHOUT the optimization
3. `build_order` must be identical
4. All UV coordinates must be identical
5. All closure/frame reports must be identical

If ANY difference — dirty marking is incomplete. Do NOT proceed to Phase D.
Instead debug which chain was served from stale cache.

### Debug aid for verification

Temporarily add a paranoid full-scan check mode:

```python
# TEMPORARY: verify incremental matches full-scan
if _PARANOID_CHECK:
    full_scan_best = _cf_select_best_frontier_candidate_full_scan(runtime_policy, all_chain_pool)
    if full_scan_best is not None and best_candidate is not None:
        if full_scan_best.chain_ref != best_candidate.chain_ref:
            print(f"[CFTUV][CACHE BUG] full={full_scan_best.chain_ref}:{full_scan_best.score:.4f} "
                  f"cached={best_candidate.chain_ref}:{best_candidate.score:.4f}")
```

Where `_cf_select_best_frontier_candidate_full_scan` is the original function
preserved under a different name. Remove after verification passes on all meshes.

---

## Phase D: Handle anchor adjustment dirty propagation

### Problem

`_cf_try_place_frontier_candidate()` (line ~1058) can call
`_cf_apply_anchor_adjustments()` which modifies UV points of **already placed
chains**. This means anchors derived from those chains are now stale.

### What to add

After successful `_cf_apply_anchor_adjustments` in `_cf_try_place_frontier_candidate`,
mark neighbors of each adjusted chain as dirty:

```python
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
        ...
        return False

    # Mark neighbors of adjusted chains dirty
    for adjusted_ref, _, _ in candidate.anchor_adjustments:
        adjusted_chain = graph.get_chain(*adjusted_ref)
        if adjusted_chain is not None:
            runtime_policy._mark_neighbors_dirty(adjusted_ref, adjusted_chain)
```

### Edge case

An adjustment can cascade: chain A adjusts chain B's endpoint, which changes
anchors for chain C (neighbor of B). The `_mark_neighbors_dirty` call on B
covers C. But if adjustment targets multiple chains, we need to mark ALL of
their neighbors. The loop above handles this.

### Verification

- Same bit-identical test as Phase C
- Specifically test on meshes with closure seams (these trigger anchor adjustments)
- Paranoid check mode should still pass

---

## Phase E: Cache cleanup for rejected and placed chains

### Problem

When a chain is rejected or placed, its cache entry becomes waste. Not a
correctness issue but memory hygiene.

### What to add

In `register_chain()`:
```python
self._cached_evals.pop(chain_ref, None)
self._dirty_refs.discard(chain_ref)
```

In `reject_chain()`:
```python
def reject_chain(self, chain_ref: ChainRef) -> None:
    self.rejected_chain_refs.add(chain_ref)
    self._cached_evals.pop(chain_ref, None)
    self._dirty_refs.discard(chain_ref)
```

### Verification

- Identical output (cleanup doesn't affect logic)
- Memory footprint of `_cached_evals` stays bounded

---

## Phase F: Performance measurement and cleanup

### Tasks

1. Add timing instrumentation around `build_quilt_scaffold_chain_frontier`:
   ```python
   import time
   t0 = time.perf_counter()
   # ... frontier loop ...
   t1 = time.perf_counter()
   trace_console(f"[CFTUV][Perf] Frontier: {t1-t0:.3f}s, {iteration} iterations, "
                 f"cache_hits/evals ratio=...")
   ```

2. Log cache hit ratio per quilt:
   ```python
   # Count inside _cf_select_best_frontier_candidate
   cache_hits = 0
   full_evals = 0
   ```

3. Run on production meshes. Expected improvement:
   - Small meshes (<50 chains): marginal improvement (first iteration dominates)
   - Medium meshes (50-200 chains): 3-5x faster
   - Large meshes (200+ chains): 5-10x faster

4. Remove paranoid check mode (`_PARANOID_CHECK` and `_full_scan` backup function)

5. Remove timing instrumentation (or hide behind `dbg_verbose_console`)

### Verification

- Final regression snapshot matches pre-optimization baseline
- Performance improvement confirmed on at least 3 production meshes

---

## Phase G: Update documentation

### What to update

In `AGENTS.md` under invariants, add:
```
11. Incremental frontier cache must produce bit-identical output to full scan
```

In `docs/cftuv_architecture.md` under Frontier Builder Rules, add:
```
13. Frontier uses incremental score cache. After placing a chain, only
    topologically affected neighbors are re-evaluated. Cache correctness
    is verified by bit-identical output against full-scan baseline.
```

### Update execution checklist in this plan

Mark all phases as done with dates and notes.

---

## Execution Checklist

### Phase A: Cache infrastructure

- [ ] Add `_cached_evals` and `_dirty_refs` fields to `FrontierRuntimePolicy`
- [ ] Verify addon loads and runs identically

### Phase B: Dirty marking

- [ ] Implement `_mark_neighbors_dirty()` method
- [ ] Wire into `register_chain()` — call after `_cf_register_points`
- [ ] Handle first-chain-in-patch timing correctly (check `== 1` after increment)
- [ ] Verify dirty sets are small (print diagnostic)
- [ ] Verify output unchanged

### Phase C: Wire cache into selection

- [ ] Preserve original `_cf_select_best_frontier_candidate` as `_full_scan` backup
- [ ] Implement cache-aware `_cf_select_best_frontier_candidate`
- [ ] Add bootstrap dirty marking after pool creation
- [ ] Add paranoid check mode (temporary)
- [ ] **CRITICAL: Verify bit-identical output on ALL regression meshes**
- [ ] If any mismatch: debug and fix before proceeding

### Phase D: Anchor adjustment propagation

- [ ] Mark neighbors dirty after successful `_cf_apply_anchor_adjustments`
- [ ] Verify on meshes with closure seams
- [ ] Paranoid check still passes

### Phase E: Cache cleanup

- [ ] Clean cache on `register_chain()`
- [ ] Clean cache on `reject_chain()`
- [ ] Verify output unchanged

### Phase F: Measurement and cleanup

- [ ] Add timing instrumentation
- [ ] Measure cache hit ratio
- [ ] Verify improvement on 3+ production meshes
- [ ] Remove paranoid check backup function
- [ ] Remove or gate timing instrumentation behind verbose flag

### Phase G: Documentation

- [ ] Update AGENTS.md invariants
- [ ] Update cftuv_architecture.md frontier rules
- [ ] Mark all checklist phases done

---

## Risk Assessment

**Risk: Stale cache serving wrong best candidate.**
Mitigation: Paranoid check mode in Phase C catches this immediately.
If full-scan and cache disagree on best candidate, it's a dirty marking bug.
Severity: HIGH but fully detectable.

**Risk: First-chain-in-patch timing off-by-one.**
The `placed_count_by_patch` is incremented in `register_chain()` BEFORE
`_mark_neighbors_dirty()` is called. The dirty method must check `== 1`
(just became 1), not `== 0` (was 0 before). Getting this wrong means
same-patch chains don't get the +0.2 momentum bonus update.
Mitigation: Paranoid check catches this. Severity: MEDIUM.

**Risk: Cross-patch vertex lookup misses newly registered vertices.**
`_cf_register_points` adds entries to `vert_to_placements` BEFORE
`_mark_neighbors_dirty` is called. So the dirty method sees the full
vertex map including the just-placed chain's endpoints.
But chains in OTHER patches that share these vertices must be found
via `vert_to_placements` — which now includes entries from the new chain.
We need to iterate `vert_to_placements[vert_idx]` and find chains in
other patches that are NOT yet placed. These might not have been in
`vert_to_placements` yet (they're not placed).
**CORRECTION:** `vert_to_placements` only contains PLACED chain points.
Unplaced chains that share a vertex are found by checking if their
`start_vert_index` or `end_vert_index` matches. But we don't have
a reverse index from vert_index → unplaced chain_refs.
**Fix:** Build a static `vert_to_pool_chains` index at pool creation time.
This maps vert_index → list of ChainRef from the pool. Use this in
`_mark_neighbors_dirty` for cross-patch lookups instead of `vert_to_placements`.

**Risk: Rescue paths bypass cache.**
tree_ingress, free_ingress, closure_follow do their own full pool scans
with their own evaluation logic. They don't use `_cf_select_best_frontier_candidate`.
This is fine — rescue fires only on stall, not in hot path.
No action needed in this plan.

**Risk: Memory growth.**
`_cached_evals` holds one `FrontierCandidateEval` per pool chain.
FrontierCandidateEval contains Vectors (anchors), strings (reason), tuples.
For 200 chains this is ~50KB — negligible.
Phase E cleanup bounds memory further.

---

## What This Plan Does NOT Do

- Does not optimize rescue paths (they are cold path)
- Does not optimize `evaluate_candidate()` internals
- Does not change scoring logic or behavior
- Does not touch any module except `solve_frontier.py`
- Does not add new dependencies
- Does not change any public API

---

## Prerequisite

Verify the static vert-to-pool index concern from Risk Assessment above.
If `vert_to_placements` does not contain unplaced chain endpoints (which it
doesn't — it only stores placed chain points), then Phase B needs an
additional data structure:

```python
# Built once at pool creation, stored in FrontierRuntimePolicy:
_vert_to_pool_refs: dict[int, list[ChainRef]] = field(default_factory=dict)
```

Populated in `build_quilt_scaffold_chain_frontier()` right after
`_cf_build_frontier_chain_pool()`:

```python
# Build static reverse index for dirty marking
for entry in all_chain_pool:
    for vert_idx in (entry.chain.start_vert_index, entry.chain.end_vert_index):
        if vert_idx >= 0:
            runtime_policy._vert_to_pool_refs.setdefault(vert_idx, []).append(entry.chain_ref)
# Also add seed chain
for vert_idx in (seed_chain.start_vert_index, seed_chain.end_vert_index):
    if vert_idx >= 0:
        runtime_policy._vert_to_pool_refs.setdefault(vert_idx, []).append(seed_ref)
```

Then `_mark_neighbors_dirty` cross-patch section uses `_vert_to_pool_refs`
instead of `vert_to_placements`:

```python
# --- 2. Cross-patch vertex sharing ---
for vert_idx in (chain.start_vert_index, chain.end_vert_index):
    if vert_idx < 0:
        continue
    for other_ref in self._vert_to_pool_refs.get(vert_idx, []):
        if other_ref == chain_ref:
            continue
        if self.is_chain_available(other_ref):
            self._dirty_refs.add(other_ref)
```

This must be added in Phase A alongside `_cached_evals` and `_dirty_refs`.
