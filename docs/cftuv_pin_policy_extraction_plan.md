# CFTUV P6: Pin Policy Extraction Plan
## Extract implicit pin logic into an explicit, queryable layer

---

## Problem Statement

Pin policy is currently scattered across two modules with implicit coupling:

1. **solve_frontier.py** computes `scaffold_connected_chains` (BFS through H/V
   from root chain) — this is a **connectivity fact** that the pin layer needs.

2. **solve_transfer.py** contains `_should_pin_scaffold_point()` and
   `_free_endpoint_has_local_frame_anchor()` — these are **pin decisions**
   interleaved with UV transfer mechanics.

3. **The circular dependency:** Scaffold builder doesn't know which points will
   be pinned. Pin policy doesn't know future placement order. Quality depends
   on placement order which depends on scoring which doesn't account for
   pinning.

Current pin rules (implicit, reconstructed from code):

| Chain Role | Condition | Pin Decision |
|-----------|-----------|-------------|
| H_FRAME / V_FRAME | scaffold_connected = True | Pin ALL points |
| H_FRAME / V_FRAME | scaffold_connected = False (isolated behind FREE gap) | Pin NOTHING |
| FREE | conformal_patch = True (entire patch is FREE) | Pin NOTHING |
| FREE | endpoint (index 0 or last) + touches connected local H/V | Pin endpoint |
| FREE | endpoint + does NOT touch connected local H/V | Pin NOTHING |
| FREE | interior point (not endpoint) | Pin NOTHING |

These rules are correct but exist as code branches inside transfer functions,
making them impossible to query from frontier, test in isolation, or extend.

---

## Goals

1. **Explicit model:** Pin decisions are computed by a dedicated function that
   takes scaffold placement data and returns a typed pin map — not scattered
   across transfer helpers.

2. **Queryable from frontier:** Scaffold builder can preview "if I place this
   chain, which of its points will be pinned?" — without running full transfer.

3. **Testable in isolation:** Pin decisions can be validated against scaffold
   data without BMesh or UV layer.

4. **Zero behavior change:** Identical pin_uv flags on every UV loop for every
   production mesh. Verified by regression snapshot comparison.

---

## Architecture

### New type: `PatchPinMap`

```python
@dataclass(frozen=True)
class ChainPinDecision:
    """Pin decision for one placed chain."""
    chain_index: int
    frame_role: FrameRole
    pin_all: bool                    # True = every point pinned
    pin_endpoints_only: bool         # True = only index 0 and last
    pin_nothing: bool                # True = no points pinned
    reason: str                      # Human-readable reason for debug/snapshot

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
        ...

    def pinned_chain_indices(self) -> frozenset[int]:
        """Which chains have any pinned points?"""
        ...
```

`PatchPinMap` is the **single source of truth** for pin decisions. Transfer
reads it. Frontier can preview it. Reporting can serialize it.

### New type: `PinPolicy`

```python
@dataclass(frozen=True)
class PinPolicy:
    """Immutable pin policy configuration. Passed as parameter, not global."""
    pin_connected_hv: bool = True
    pin_free_endpoints_with_hv_anchor: bool = True
    skip_conformal_patches: bool = True
    skip_isolated_hv: bool = True
```

This makes the pin rules explicit data. Default values reproduce current
behavior exactly. Future changes adjust policy fields, not code branches.

---

## Module Layout

### New file: `solve_pin_policy.py`

Position in dependency graph:

```
solve_records.py          ← PatchPinMap, ChainPinDecision, PinPolicy live here
    ↑
solve_pin_policy.py       ← imports solve_records, model
    ↑
solve_frontier.py         ← imports solve_pin_policy (preview queries)
    ↑
solve_transfer.py         ← imports solve_pin_policy (authoritative pin map)
```

No circular dependencies. `solve_pin_policy.py` depends only on records and
model — same level as solve_planning.py.

### What moves INTO solve_pin_policy.py

| Current location | Function | New location |
|-----------------|----------|-------------|
| solve_transfer.py | `_should_pin_scaffold_point()` | solve_pin_policy.py (rewritten as `_decide_chain_pin()`) |
| solve_transfer.py | `_free_endpoint_has_local_frame_anchor()` | solve_pin_policy.py (unchanged logic) |
| solve_frontier.py | `_compute_scaffold_connected_chains()` | solve_pin_policy.py (connectivity is a pin input) |

### What stays where it is

| Function | Stays in | Reason |
|----------|---------|--------|
| `_build_patch_transfer_targets()` | solve_transfer.py | UV target resolution is transfer, not pin policy |
| `_apply_patch_scaffold_to_uv()` | solve_transfer.py | Writing UV loops is transfer |
| `_clear_patch_pins()` | solve_transfer.py | BMesh mutation is transfer |
| `_cf_build_envelopes()` | solve_frontier.py | Envelope assembly is frontier; calls pin_policy for connected_chains |

---

## Strategy

- Mechanical extraction with zero behavior change
- Each phase independently testable
- Regression snapshot comparison after every phase
- No new abstractions beyond PatchPinMap / PinPolicy
- No changes to model.py, analysis.py, operators.py, debug.py, constants.py

---

## Phase A: Create solve_pin_policy.py with types

### What to do

1. Add `PinPolicy`, `ChainPinDecision`, `PatchPinMap` to `solve_records.py`

2. Create `cftuv/solve_pin_policy.py` with:
   - Move `_compute_scaffold_connected_chains()` from solve_frontier.py
   - Move `_free_endpoint_has_local_frame_anchor()` from solve_transfer.py
   - Move `_should_pin_scaffold_point()` from solve_transfer.py
   - Add `build_patch_pin_map()` — wraps the moved functions into PatchPinMap

3. Wire imports:
   - `solve_frontier.py`: `from .solve_pin_policy import _compute_scaffold_connected_chains`
   - `solve_transfer.py`: `from .solve_pin_policy import build_patch_pin_map`

### build_patch_pin_map specification

```python
def build_patch_pin_map(
    graph: PatchGraph,
    patch_placement: ScaffoldPatchPlacement,
    policy: PinPolicy = PinPolicy(),
) -> PatchPinMap:
    """Build complete pin map for one patch from scaffold placement data.

    Does NOT touch BMesh or UV. Pure function over PatchGraph + scaffold.
    """
```

Implementation: iterate `patch_placement.chain_placements`, for each chain
call the extracted `_decide_chain_pin()` (renamed from `_should_pin_scaffold_point`
but with same logic), collect into `ChainPinDecision` list, return `PatchPinMap`.

### What NOT to do

- Do NOT change `_build_patch_transfer_targets` yet — it still uses the old
  functions directly
- Do NOT change `_cf_build_envelopes` yet
- Do NOT remove original functions yet — keep them as pass-through wrappers
  that delegate to solve_pin_policy

### Verification

- Addon loads
- `build_patch_pin_map()` produces decisions that match current behavior
  (add a temporary assert in `_build_patch_transfer_targets` that old pin
  decision == new PatchPinMap decision for every point)

---

## Phase B: Wire PatchPinMap into solve_transfer.py

### What to do

1. In `_build_patch_transfer_targets()`:
   - Call `build_patch_pin_map()` at the top
   - Replace inline `_should_pin_scaffold_point()` + `_free_endpoint_has_local_frame_anchor()`
     calls with `pin_map.is_point_pinned(chain_index, point_index, point_count)`
   - The `conformal_patch` flag is now read from `pin_map.conformal_patch`

2. Remove the old `_should_pin_scaffold_point()` and
   `_free_endpoint_has_local_frame_anchor()` from solve_transfer.py
   (they now live in solve_pin_policy.py)

3. Store `PatchPinMap` in `PatchTransferTargetsState` for reporting access:
   ```python
   @dataclass(frozen=True)
   class PatchTransferTargetsState:
       ...
       pin_map: Optional[PatchPinMap] = None  # new field
   ```

### What NOT to do

- Do NOT change `_apply_patch_scaffold_to_uv` logic — it still reads
  `pin_target_ids` from `PatchTransferTargetsState`
- Do NOT change solve_frontier.py yet

### Verification

- **CRITICAL:** Run regression snapshot on every production mesh
- Every `pin_target_ids` set must be identical to pre-P6 baseline
- Every `pinned_uv_loops` count must be identical
- Remove temporary asserts from Phase A

---

## Phase C: Wire connected_chains through solve_pin_policy.py

### What to do

1. In `solve_frontier.py` → `_cf_build_envelopes()`:
   - Replace direct call to `_compute_scaffold_connected_chains()` with
     import from `solve_pin_policy`
   - The function signature and logic are unchanged — just the import source

2. Remove `_compute_scaffold_connected_chains()` from solve_frontier.py

3. In `solve_pin_policy.py`, `build_patch_pin_map()` now calls
   `_compute_scaffold_connected_chains()` internally instead of receiving
   `scaffold_connected_chains` as pre-computed input. This means
   `build_patch_pin_map` needs access to:
   - `patch_placement.chain_placements` (already has)
   - `patch_placement.root_chain_index` (already has)
   - total chain count from `boundary_loop` (passed via graph lookup)

### Important: _cf_build_envelopes still needs scaffold_connected_chains

`_cf_build_envelopes` stores `scaffold_connected_chains` on the
`ScaffoldPatchPlacement` dataclass. This value is consumed by:
1. Pin policy (now via solve_pin_policy.py) ✓
2. Reporting: `[ISOLATED]` tag in scaffold report ✓
3. Regression snapshot: `scaffold_connected_chains:N/M` line ✓

So `_cf_build_envelopes` must still call `_compute_scaffold_connected_chains`
and store the result. The difference: it imports from solve_pin_policy.py
instead of defining locally.

### Verification

- Identical regression snapshots
- `scaffold_connected_chains` counts unchanged in reports

---

## Phase D: Add preview query for frontier

### What to do

Add a lightweight preview function to solve_pin_policy.py:

```python
def preview_chain_pin_decision(
    chain: BoundaryChain,
    scaffold_connected: bool,
    conformal_patch: bool,
    boundary_loop: BoundaryLoop,
    chain_index: int,
    scaffold_connected_chains: frozenset[int],
    policy: PinPolicy = PinPolicy(),
) -> ChainPinDecision:
    """Preview pin decision for a single chain without full PatchPinMap.

    Usable from frontier builder to estimate pin impact before commit.
    """
```

This does NOT change frontier behavior — it adds a queryable function that
frontier CAN use in future scoring (P5). The function must return the same
decision as `build_patch_pin_map` would for the same chain.

### What NOT to do

- Do NOT modify `_cf_score_candidate` or any frontier scoring
- Do NOT add preview calls to the hot path
- This is infrastructure for P5, not a behavior change

### Verification

- Addon loads and runs identically (new function exists but is not called
  from hot path)
- Unit test: `preview_chain_pin_decision` matches `build_patch_pin_map`
  for same inputs

---

## Phase E: Add pin_map to reporting and snapshot

### What to do

1. In `solve_reporting.py` → `format_regression_snapshot_report()`:
   - After computing `transfer_state`, read `transfer_state.pin_map`
   - Add per-chain pin reason to snapshot line:
     ```
     P3 WALL.SIDE ... pin_reasons:[C0:pin_all:connected_hv, C1:pin_endpoints:free_with_hv_anchor, C2:pin_nothing:isolated_hv]
     ```

2. In `solve_reporting.py` → `format_root_scaffold_report()`:
   - Add pin summary per patch (optional, behind verbose flag)

### What NOT to do

- Do NOT change snapshot format in a way that breaks existing baselines —
  add new fields at the end of existing lines, or as new sub-lines

### Verification

- Old snapshot lines still present and identical
- New pin_reasons lines appear and match manual inspection

---

## Phase F: Documentation and cleanup

### What to do

1. Update `AGENTS.md`:
   - Module layout: add `solve_pin_policy.py` with description
   - Invariants: add rule about pin policy being explicit layer

2. Update `docs/cftuv_architecture.md`:
   - Debt section: mark P6 as resolved
   - UV Transfer and Pin Policy section: describe PatchPinMap as source of truth
   - Add solve_pin_policy.py to Module Responsibilities table

3. Update `docs/cftuv_solve_decomposition_plan.md`:
   - Add solve_pin_policy.py to cross-module dependency graph

4. Clean up any remaining pass-through wrappers from Phase A

### Verification

- Final regression snapshot matches Phase A baseline exactly
- All documentation references are consistent

---

## Cross-Module Dependency Graph (updated)

```
solve_records.py          ← PinPolicy, ChainPinDecision, PatchPinMap added here
    ↑
solve_planning.py         ← unchanged
    ↑
solve_pin_policy.py       ← NEW: imports solve_records, model only
    ↑
solve_diagnostics.py      ← unchanged
    ↑
solve_frontier.py         ← imports solve_pin_policy (_compute_scaffold_connected_chains)
    ↑
solve_transfer.py         ← imports solve_pin_policy (build_patch_pin_map)
    ↑
solve_reporting.py        ← reads pin_map from PatchTransferTargetsState
    ↑
solve.py                  ← facade, unchanged
```

No circular dependencies. DAG preserved.

---

## Files Changed Per Phase

| Phase | Create | Modify | Delete |
|-------|--------|--------|--------|
| A | solve_pin_policy.py | solve_records.py, solve_frontier.py (import), solve_transfer.py (import) | — |
| B | — | solve_transfer.py (wire PatchPinMap) | remove old functions from solve_transfer.py |
| C | — | solve_frontier.py (import source change), solve_pin_policy.py | remove _compute_scaffold_connected_chains from solve_frontier.py |
| D | — | solve_pin_policy.py (add preview function) | — |
| E | — | solve_reporting.py | — |
| F | — | AGENTS.md, cftuv_architecture.md, cftuv_solve_decomposition_plan.md | — |

**Never touched:** model.py, constants.py, analysis.py, analysis_*.py,
debug.py, operators.py, console_debug.py, __init__.py

---

## Risk Assessment

**Risk: Pin decision divergence during Phase A transition.**
Both old and new code paths exist simultaneously. A temporary assert
catches any divergence immediately.
Mitigation: Assert on every scaffold point for every patch on every mesh.
Severity: HIGH but fully detectable. Remove asserts in Phase B.

**Risk: _compute_scaffold_connected_chains has frontier-internal dependencies.**
Current code accesses `FrameRole`, `ScaffoldChainPlacement.points` count —
all available from model.py and solve_records.py. No frontier-specific
runtime state needed.
Mitigation: Function already pure — takes patch_placements list and
total_chains count. Move is mechanical.

**Risk: PatchPinMap adds overhead to transfer hot path.**
`build_patch_pin_map` iterates chain_placements once (already done in current
code). PatchPinMap lookup is O(1) dict access. Net overhead: negligible.

**Risk: Preview function in Phase D may diverge from authoritative pin map.**
Both `preview_chain_pin_decision` and `build_patch_pin_map` must call the
same internal `_decide_chain_pin`. Single implementation, two entry points.

---

## Execution Checklist

### Phase A: Create module + types
- [x] Add `PinPolicy`, `ChainPinDecision`, `PatchPinMap` to solve_records.py
- [x] Create solve_pin_policy.py
- [x] Move `_compute_scaffold_connected_chains` from solve_frontier.py
- [x] Move `_should_pin_scaffold_point` from solve_transfer.py (rename to `_decide_chain_pin`)
- [x] Move `_free_endpoint_has_local_frame_anchor` from solve_transfer.py
- [x] Implement `build_patch_pin_map()`
- [x] Wire imports in solve_frontier.py and solve_transfer.py
- [x] Add temporary divergence asserts in `_build_patch_transfer_targets`
- [x] Verify: addon loads, asserts silent on production meshes

### Phase B: Wire into transfer
- [x] Replace inline pin calls with `pin_map.is_point_pinned()` in `_build_patch_transfer_targets`
- [x] Add `pin_map` field to `PatchTransferTargetsState`
- [x] Remove old `_should_pin_scaffold_point` and `_free_endpoint_has_local_frame_anchor` from solve_transfer.py
- [x] Remove temporary asserts
- [x] **CRITICAL: Regression snapshot identical on all production meshes**

### Phase C: Move connected_chains
- [x] Change import source in solve_frontier.py
- [x] Remove `_compute_scaffold_connected_chains` from solve_frontier.py
- [x] Verify `build_patch_pin_map` computes connected_chains internally
- [x] Regression snapshot identical

### Phase D: Preview query
- [x] Add `preview_chain_pin_decision()` to solve_pin_policy.py
- [x] Verify it delegates to same `_decide_chain_pin` as build_patch_pin_map
- [x] No hot-path calls — infrastructure only

### Phase E: Reporting
- [x] Add pin_reasons to regression snapshot output
- [x] Old snapshot fields unchanged
- [x] New pin data matches manual inspection

### Phase F: Documentation
- [x] AGENTS.md updated (module layout, invariant)
- [x] cftuv_architecture.md updated (debt, pin policy section, module table)
- [x] cftuv_solve_decomposition_plan.md updated (dependency graph)
- [x] Final regression snapshot matches Phase A baseline
