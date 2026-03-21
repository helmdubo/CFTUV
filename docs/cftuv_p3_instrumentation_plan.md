# CFTUV P3: Rescue/Scoring Instrumentation Plan
## Collect structured placement data before changing scoring logic

---

## Problem Statement

The frontier builder has three rescue paths that fire when main scoring stalls:

1. **tree_ingress** — bootstrap into untouched tree-child patch
2. **free_ingress** — one-edge FREE bridge to downstream H/V
3. **closure_follow** — same-role closure partner from already-placed pair

These exist because `_cf_score_candidate()` doesn't cover all valid production
cases. But we don't know:

- **How often** each rescue fires vs main frontier placement
- **What scores** were available when main frontier stalled (gap between
  threshold and best rejected candidate)
- **What anchor combinations** cause stalls (no anchor? both cross-patch?
  closure-sensitive?)
- **Which chain roles/types** end up in rescue vs main path
- **Whether rescue results** are actually good (do they cause downstream
  closure errors?)

Without this data, any P5 scoring revision is guesswork.

---

## Goals

1. **Structured telemetry:** Every frontier placement (main + rescue) emits a
   typed record with score, anchor state, role, provenance, timing.

2. **Stall diagnostics:** When main frontier fails to find a candidate above
   threshold, capture the full rejected candidate landscape.

3. **Per-quilt summary:** Aggregated stats: main/rescue ratio, score
   distribution, stall count, rescue success rate.

4. **Serializable:** Data goes into regression snapshot for offline analysis
   and cross-mesh comparison.

5. **Zero behavior change:** Pure observation. No scoring modifications.
   Identical `build_order`, UV coordinates, pin decisions.

---

## Architecture

### New type: `FrontierPlacementRecord`

One record per placed chain (main path or rescue):

```python
@dataclass(frozen=True)
class FrontierPlacementRecord:
    """Telemetry record for one frontier placement step."""
    iteration: int
    chain_ref: ChainRef
    frame_role: FrameRole
    placement_path: str              # "main" | "tree_ingress" | "free_ingress" | "closure_follow"
    score: float                     # -1.0 for rescue (no score)
    anchor_count: int                # 0, 1, 2
    start_anchor_kind: str           # "same_patch" | "cross_patch" | "none"
    end_anchor_kind: str             # "same_patch" | "cross_patch" | "none"
    placed_in_patch_before: int      # how many chains in this patch BEFORE this placement
    is_first_in_patch: bool
    is_bridge: bool                  # FREE with ≤2 verts
    is_corner_split: bool
    neighbor_kind: str               # "PATCH" | "MESH_BORDER" | "SEAM_SELF"
    is_closure_pair: bool            # chain_ref in closure_pair_refs
    closure_preconstraint_applied: bool
    anchor_adjustment_applied: bool
    direction_inherited: bool
    chain_length_uv: float           # total UV length after placement
```

### New type: `FrontierStallRecord`

One record per stall event (when main path returns None or below threshold):

```python
@dataclass(frozen=True)
class FrontierStallRecord:
    """Telemetry record for one frontier stall event."""
    iteration: int
    best_rejected_score: float       # highest score among candidates that didn't pass
    best_rejected_ref: Optional[ChainRef]
    best_rejected_role: Optional[FrameRole]
    best_rejected_anchor_count: int
    available_count: int             # chains still in pool
    no_anchor_count: int             # chains with known=0
    below_threshold_count: int       # chains with 0 < score < threshold
    rescue_attempted: str            # "tree_ingress" | "free_ingress" | "closure_follow" | "none"
    rescue_succeeded: bool
    patches_with_placed: int         # patches that have ≥1 chain placed
    patches_untouched: int           # patches with 0 chains placed
```

### New type: `QuiltFrontierTelemetry`

Aggregated per-quilt summary:

```python
@dataclass(frozen=True)
class QuiltFrontierTelemetry:
    """Aggregated frontier telemetry for one quilt."""
    quilt_index: int
    total_placements: int
    main_placements: int
    tree_ingress_placements: int
    free_ingress_placements: int
    closure_follow_placements: int
    total_stalls: int
    stalls_resolved_by_rescue: int
    stalls_unresolved: int           # final stop
    score_min: float                 # min main-path score
    score_max: float
    score_mean: float
    score_p25: float                 # 25th percentile
    score_p50: float                 # median
    score_p75: float
    best_rejected_score_max: float   # highest score that was still below threshold
    first_rescue_iteration: int      # -1 if no rescue
    rescue_ratio: float              # rescue_placements / total_placements
    frontier_duration_sec: float
    placement_records: tuple[FrontierPlacementRecord, ...]
    stall_records: tuple[FrontierStallRecord, ...]
```

---

## Module Layout

### New file: `solve_instrumentation.py`

```
solve_records.py          ← FrontierPlacementRecord, FrontierStallRecord,
                            QuiltFrontierTelemetry added here
    ↑
solve_instrumentation.py  ← NEW: collector + aggregation + formatting
    ↑
solve_frontier.py         ← emits records into collector
    ↑
solve_reporting.py        ← reads telemetry for snapshot output
```

No circular dependencies. Instrumentation depends only on records.

### What goes into solve_instrumentation.py

```python
class FrontierTelemetryCollector:
    """Mutable collector that accumulates records during frontier build.
    
    Created once per quilt. Passed to frontier loop. Finalized into
    immutable QuiltFrontierTelemetry at end of quilt.
    """
    
    def record_placement(self, ...) -> None: ...
    def record_stall(self, ...) -> None: ...
    def finalize(self) -> QuiltFrontierTelemetry: ...

def format_quilt_telemetry_summary(telemetry: QuiltFrontierTelemetry) -> list[str]: ...
def format_quilt_telemetry_detail(telemetry: QuiltFrontierTelemetry) -> list[str]: ...
```

---

## Emission Points in solve_frontier.py

### Point 1: Main path placement

In `_cf_try_place_frontier_candidate()`, after successful `register_chain`:

```python
collector.record_placement(
    iteration=iteration,
    chain_ref=chain_ref,
    chain=chain,
    placement_path="main",
    score=candidate.score,
    start_anchor=candidate.start_anchor,
    end_anchor=candidate.end_anchor,
    placed_in_patch_before=runtime_policy.placed_in_patch(chain_ref[0]) - 1,
    closure_preconstraint_applied=candidate.closure_dir_override is not None,
    anchor_adjustment_applied=bool(candidate.anchor_adjustments),
    direction_inherited=dir_override is not None and candidate.closure_dir_override is None,
    uv_points=uv_points,
    runtime_policy=runtime_policy,
)
```

### Point 2: Rescue path placements

In each of `_cf_try_place_tree_ingress_candidate()`,
`_cf_try_place_free_ingress_bridge()`,
`_cf_try_place_closure_follow_candidate()` — after successful register_chain:

```python
collector.record_placement(
    ...
    placement_path="tree_ingress",  # or "free_ingress" or "closure_follow"
    score=-1.0,                     # rescue paths don't use main scoring
    ...
)
```

### Point 3: Stall events

In `build_quilt_scaffold_chain_frontier()` main loop, at the point where
main path returns None or below threshold, BEFORE trying rescue:

```python
# Main candidate not found or below threshold
stall_best = _collect_stall_best_rejected(runtime_policy, all_chain_pool)
collector.record_stall(
    iteration=iteration,
    best_rejected=stall_best,
    available_count=...,
    no_anchor_count=...,
    below_threshold_count=...,
)
# Then try rescue paths...
# After rescue attempt:
collector.update_last_stall_rescue(
    rescue_attempted="tree_ingress",
    rescue_succeeded=True,
)
```

### Point 4: Finalization

At end of `build_quilt_scaffold_chain_frontier()`, before return:

```python
telemetry = collector.finalize()
quilt_scaffold.frontier_telemetry = telemetry
```

This requires adding one field to `ScaffoldQuiltPlacement`:
```python
frontier_telemetry: Optional[QuiltFrontierTelemetry] = None
```

---

## Stall Analysis Helper

New function in solve_instrumentation.py:

```python
def _collect_stall_best_rejected(
    runtime_policy: FrontierRuntimePolicy,
    all_chain_pool: list[ChainPoolEntry],
) -> StallBestRejected:
    """Snapshot of the best available-but-rejected candidate at stall time.
    
    Iterates pool to find: highest-scoring candidate (even if below threshold),
    count of candidates by anchor state, count by role.
    """
```

This uses `runtime_policy._cached_evals` from the incremental frontier cache —
no re-evaluation needed, just reads existing cached scores.

---

## Reporting Integration

### Regression snapshot (solve_reporting.py)

Per-quilt telemetry summary added after existing closure/frame reports:

```
## Quilt 0
...existing lines...
frontier_telemetry:
  placements: 47 main:41 tree_ingress:3 free_ingress:2 closure_follow:1
  stalls: 6 resolved:5 unresolved:1
  scores: min:0.32 p25:0.78 p50:1.15 p75:1.62 max:2.45
  rescue_ratio: 0.128
  best_rejected_max: 0.28
  duration: 0.047s
```

### Verbose console (behind dbg_verbose_console)

Full per-placement detail:

```
[CFTUV][Telemetry] Q0 Step 14: main P3L0C2 H_FRAME score:1.42 ep:2 SP/XP bridge:N closure:N
[CFTUV][Telemetry] Q0 Stall 15: best_rejected:0.28 P5L0C1 FREE available:12 no_anchor:8
[CFTUV][Telemetry] Q0 Step 15: tree_ingress P5L0C3 V_FRAME score:-1 ep:1 XP/- bridge:N
```

---

## Strategy

- All new code in solve_instrumentation.py + type additions in solve_records.py
- Emission points are 1-2 line calls in solve_frontier.py (after existing trace_console)
- solve_reporting.py reads finalized telemetry from quilt_scaffold
- Zero behavior change — collector is append-only, never read during frontier loop
- Phased: types first, then collector, then emission, then reporting

---

## Phase A: Types in solve_records.py

- [ ] Add `FrontierPlacementRecord` dataclass
- [ ] Add `FrontierStallRecord` dataclass
- [ ] Add `QuiltFrontierTelemetry` dataclass
- [ ] Add `frontier_telemetry: Optional[QuiltFrontierTelemetry] = None` to `ScaffoldQuiltPlacement` in model.py
- [ ] Verify: addon loads, no import errors

## Phase B: Collector in solve_instrumentation.py

- [ ] Create `cftuv/solve_instrumentation.py`
- [ ] Implement `FrontierTelemetryCollector` class
- [ ] Implement `record_placement()` — builds FrontierPlacementRecord from args
- [ ] Implement `record_stall()` + `update_last_stall_rescue()` — builds FrontierStallRecord
- [ ] Implement `finalize()` — computes aggregates, returns QuiltFrontierTelemetry
- [ ] Implement `_collect_stall_best_rejected()` helper
- [ ] Implement `format_quilt_telemetry_summary()` and `format_quilt_telemetry_detail()`
- [ ] Verify: module imports cleanly, collector can be instantiated

## Phase C: Emission points in solve_frontier.py

- [ ] Create collector at start of `build_quilt_scaffold_chain_frontier()`
- [ ] Emit after main path placement in `_cf_try_place_frontier_candidate()`
- [ ] Emit after tree_ingress in `_cf_try_place_tree_ingress_candidate()`
- [ ] Emit after free_ingress in `_cf_try_place_free_ingress_bridge()`
- [ ] Emit after closure_follow in `_cf_try_place_closure_follow_candidate()`
- [ ] Emit stall record in main loop before rescue attempts
- [ ] Update stall record after rescue attempt outcome
- [ ] Finalize telemetry and store on `quilt_scaffold.frontier_telemetry`
- [ ] Pass collector to rescue functions (add parameter, default None for backward compat)
- [ ] **CRITICAL: Regression snapshot identical — build_order, UV, pins unchanged**

## Phase D: Reporting integration

- [ ] Add telemetry summary to `format_regression_snapshot_report()` in solve_reporting.py
- [ ] Add telemetry detail to `format_root_scaffold_report()` (verbose only)
- [ ] Add verbose console output via `trace_console` in solve_instrumentation.py
- [ ] Verify: old snapshot lines unchanged, new telemetry lines appear

## Phase E: Documentation

- [ ] Update AGENTS.md: add solve_instrumentation.py to module layout
- [ ] Update cftuv_architecture.md: mark P3 as resolved, describe telemetry layer
- [ ] Update cftuv_solve_decomposition_plan.md: add to dependency graph

---

## Files Changed Per Phase

| Phase | Create | Modify | Delete |
|-------|--------|--------|--------|
| A | — | solve_records.py, model.py (1 field) | — |
| B | solve_instrumentation.py | — | — |
| C | — | solve_frontier.py (emission calls) | — |
| D | — | solve_reporting.py | — |
| E | — | AGENTS.md, cftuv_architecture.md, cftuv_solve_decomposition_plan.md | — |

**Never touched:** constants.py, analysis.py, analysis_*.py, debug.py,
operators.py, console_debug.py, __init__.py, solve_planning.py,
solve_diagnostics.py, solve_transfer.py, solve_pin_policy.py

---

## Cross-Module Dependency Graph (updated)

```
solve_records.py              ← telemetry types added here
    ↑
solve_instrumentation.py      ← NEW: imports solve_records only
    ↑
solve_frontier.py             ← imports solve_instrumentation (collector)
    ↑
solve_reporting.py            ← reads telemetry from ScaffoldQuiltPlacement
```

---

## What This Plan Does NOT Do

- Does not change scoring logic (that's P5)
- Does not change rescue path behavior
- Does not change pin policy (that's P6)
- Does not add UI elements or operator buttons
- Does not require production mesh assets in repo
- Does not add dependencies
- Does not affect frontier cache or incremental optimization

---

## Success Criteria for P5 Readiness

After P3 is complete, running regression snapshot on any production mesh
must answer these questions from the snapshot file alone:

1. What % of placements used rescue paths?
2. Which rescue path fires most often?
3. What's the score gap between threshold and best rejected candidate at stall?
4. Do rescue placements correlate with downstream closure errors?
5. Are there patches where main scoring consistently fails (always needs rescue)?
6. What's the score distribution — is there a bimodal pattern suggesting
   two different "populations" of chains?
