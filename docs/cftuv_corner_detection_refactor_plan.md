# CFTUV — Corner Detection Refactor: Chord-Based Multi-Scale Stabilization

> Status: Plan ready for execution.
> Owner: Alexander (Ideas Owner / Orchestrator)
> Execution: AI agents, phase by phase with verification gates.

---

## Problem Statement

Current corner detection in CFTUV boundary analysis is **not invariant to mesh tessellation density**. Two geometrically identical shapes with different vertex counts produce different corner sets, leading to different chain splits, different FrameRole assignments, and ultimately different (sometimes broken) UV layouts.

### Root Cause

The turn angle is measured between three consecutive vertices `[i-1, i, i+1]` — a single vertex-hop in each direction. On a dense mesh, every turn is small (below the 30° threshold) and real macro-corners get missed. On a sparse mesh, tessellation noise creates false corners.

### Symptoms

- Same architectural shape with different bevel/subdivision density → different chain topology
- "Staircase" artifacts: a diagonal FREE connector gets split into short sub-chains that receive false H/V roles
- Unstable regression snapshots: vertex density changes propagate into chain count / role distribution

### Affected Code

Primary:
- `cftuv/analysis_corners.py` — all corner detection functions
- `cftuv/analysis_boundary_loops.py` — consumes corner results for chain split

Secondary (consume chain topology downstream):
- `cftuv/analysis_frame_runs.py` — frame run diagnostics over chains
- `cftuv/analysis_derived.py` — derived topology summaries
- `cftuv/analysis_junctions.py` — junction aggregation over corners
- `cftuv/solve_frontier.py` — frontier builder consumes chain/corner topology

---

## Architectural Decision

**Replace vertex-hop angle measurement with chord-based sliding window angle measurement.**

Instead of:
```
angle(polyline[i-1], polyline[i], polyline[i+1])
```

Use:
```
incoming_chord = polyline[i] - polyline_at_arc(arc_position[i] - window)
outgoing_chord = polyline_at_arc(arc_position[i] + window) - polyline[i]
angle(incoming_chord, outgoing_chord)
```

Where `window` is a fixed fraction of total arc length, and `polyline_at_arc()` returns an interpolated point at a given arc-length position.

This makes corner detection a function of **shape geometry**, not **vertex count**.

---

## Constraints

These constraints are mandatory for all phases. Violation = bug.

1. **No solve behavior change until Phase 5.** Phases 1–4 build and validate the new detector in parallel. The old detector stays active.
2. **No new dependencies.** Pure Python + `mathutils.Vector` only.
3. **No changes to `model.py` data structures.** `BoundaryCorner`, `BoundaryChain`, `BoundaryLoop` stay as-is. The refactor changes **how** corners are detected, not **what** a corner is.
4. **Deterministic.** Same input polyline → same output corners. No randomness, no floating-point comparison without epsilon.
5. **`AGENTS.md` invariants preserved.** Especially: "Sharp does NOT participate in patch split — only Seam", "HOLE loops do NOT participate in scaffold placement pool", all P1–P6 module boundaries.
6. **Regression snapshots must not regress.** Verification on production mesh set required before swap.
7. **Comments in Russian** (studio convention). Docstrings Russian or English.

---

## Glossary (for agents)

| Term | Meaning |
|------|---------|
| **polyline** | Ordered list of `Vector` (3D vertex positions) representing a boundary contour |
| **arc length** | Cumulative distance along polyline from start vertex to current vertex |
| **chord** | Straight-line vector between two points on the polyline |
| **window** | Arc-length distance used for chord sampling on each side of a candidate vertex |
| **macro-corner** | A real geometric turn of the shape (wall corner, step edge), not tessellation noise |
| **vertex-hop** | Using immediately adjacent vertices — the current (broken) approach |
| **NMS** | Non-maximum suppression — keeping only the strongest candidate in a local neighborhood |

---

## Phase 0: Foundation — Arc-Length Polyline Utilities

### Goal

Build reusable arc-length infrastructure that all subsequent phases depend on.

### Scope

Create a new file `cftuv/analysis_arc_utils.py` with pure geometric utilities. No corner detection logic here — only polyline measurement and interpolation.

### Deliverables

#### 0.1 `_build_cumulative_arc_lengths(vert_cos: list[Vector]) -> list[float]`

- Input: ordered list of 3D positions
- Output: list of same length where `result[i]` = cumulative distance from `vert_cos[0]` to `vert_cos[i]`
- `result[0]` is always `0.0`
- `result[-1]` is total polyline length

#### 0.2 `_interpolate_point_at_arc(vert_cos: list[Vector], cumulative_arcs: list[float], target_arc: float) -> Vector`

- Returns the interpolated 3D point at `target_arc` distance along the polyline
- If `target_arc <= 0`, return `vert_cos[0]`
- If `target_arc >= total_length`, return `vert_cos[-1]`
- Otherwise, find the segment containing `target_arc` and linearly interpolate

#### 0.3 `_interpolate_point_at_arc_wrapped(vert_cos: list[Vector], cumulative_arcs: list[float], total_length: float, target_arc: float) -> Vector`

- Same as 0.2 but for closed loops: `target_arc` wraps around modulo `total_length`
- Handles both positive and negative wrap

#### 0.4 `_arc_distance_between(cumulative_arcs: list[float], index_a: int, index_b: int, total_length: float, wrapped: bool) -> float`

- Shortest arc-length distance between two vertex indices
- For wrapped=True (closed loop): considers both directions around the loop

### File Location

`cftuv/analysis_arc_utils.py`

### Import Convention

```python
try:
    from .analysis_arc_utils import (
        _build_cumulative_arc_lengths,
        _interpolate_point_at_arc,
        _interpolate_point_at_arc_wrapped,
        _arc_distance_between,
    )
except ImportError:
    from analysis_arc_utils import (...)
```

### Verification

- Unit-level: hardcoded polyline examples with known arc lengths
- Edge cases: zero-length segments, single vertex, two vertices, collinear points
- Closed-loop wrap: verify wrapping arithmetic with known simple polygons (square, triangle)
- Run: `python -c "from cftuv.analysis_arc_utils import *; print('OK')"` outside Blender

### Stop Gate

Do not proceed to Phase 1 until all arc-length utilities pass edge-case verification.

---

## Phase 1: Chord-Based Turn Angle Measurement

### Goal

Implement the core chord-based angle measurement that replaces vertex-hop measurement.

### Scope

Add new functions to `cftuv/analysis_corners.py`. Do NOT modify existing functions yet.

### Deliverables

#### 1.1 `_measure_chord_turn_angle(vert_cos, cumulative_arcs, vertex_index, window_arc, basis_u, basis_v, *, wrapped=False, total_length=0.0) -> float`

Core function. For vertex at `vertex_index`:

1. Compute `incoming_point = interpolate_at_arc(arc_position[vertex_index] - window_arc)`
2. Compute `outgoing_point = interpolate_at_arc(arc_position[vertex_index] + window_arc)`
3. Project `(incoming_point → vertex)` and `(vertex → outgoing_point)` into 2D via `basis_u`, `basis_v`
4. Return angle in degrees between these two 2D vectors

If `wrapped=True`, use wrapped interpolation (for closed loops).

Edge case: if `window_arc` extends beyond polyline ends (open chain), clamp to polyline endpoints.

#### 1.2 `_select_chord_window_arc(total_arc_length: float, vert_count: int, *, min_fraction: float = 0.03, max_fraction: float = 0.12, min_absolute: float = 0.02, max_absolute: float = 0.5) -> float`

Heuristic to choose window size:

- Base: `total_arc_length * target_fraction` where `target_fraction` is in `[min_fraction, max_fraction]`
- Clamped to `[min_absolute, max_absolute]`
- Scale `target_fraction` inversely with vertex density: denser mesh → larger fraction (because each vertex covers less arc)
- Default `target_fraction = lerp(min_fraction, max_fraction, clamp01(avg_edge_length / reference_edge_length))`
- `reference_edge_length` ≈ median edge length on typical production mesh (~0.05–0.15 world units); expose as constant in `constants.py` if needed

The exact formula can be tuned later. The key invariant: **two meshes of the same shape with different density should produce approximately the same window_arc in world units.**

#### 1.3 `_measure_chord_turn_angle_3d(vert_cos, cumulative_arcs, vertex_index, window_arc, *, wrapped=False, total_length=0.0) -> float`

Same as 1.1 but without basis projection — full 3D angle. Needed for bevel filter compatibility.

### Constants (add to `constants.py`)

```python
# Chord-based corner detection
CHORD_WINDOW_MIN_FRACTION = 0.03
CHORD_WINDOW_MAX_FRACTION = 0.12
CHORD_WINDOW_MIN_ABSOLUTE = 0.02
CHORD_WINDOW_MAX_ABSOLUTE = 0.50
```

### Verification

- Compare chord-based angles vs vertex-hop angles on a simple axis-aligned L-shape (4 vertices at 90° corner): both should return ~90°
- Compare on same L-shape with 20 intermediate vertices on each arm: vertex-hop gives ~5° per vertex, chord-based should still give ~90° at the actual corner vertex
- Compare on a smooth arc (semicircle, 32 vertices): vertex-hop gives many false positives, chord-based should give 0 corners (no sharp turns)

### Stop Gate

Do not proceed to Phase 2 until chord-based angle on the L-shape test gives stable results regardless of vertex density.

---

## Phase 2: Chord-Based Candidate Collection

### Goal

Build a complete chord-based corner candidate pipeline that mirrors the existing `_collect_corner_turn_candidates_shared` but uses chord angles.

### Scope

New functions in `cftuv/analysis_corners.py`, parallel to existing functions. Existing functions remain untouched.

### Deliverables

#### 2.1 `_collect_chord_corner_candidates(polyline_cos, basis_u, basis_v, policy, vert_indices=None, bm=None) -> tuple[list[_CornerTurnCandidate], list[_CornerTurnCandidate]]`

Parallel to `_collect_corner_turn_candidates_shared`, but:

- Pre-computes `cumulative_arcs` and `window_arc` once
- Uses `_measure_chord_turn_angle` instead of `_measure_corner_turn_angle`
- Uses `_measure_chord_turn_angle_3d` instead of `_measure_corner_turn_angle_3d` for bevel filter
- Returns `(raw_candidates, filtered_candidates)` with same `_CornerTurnCandidate` type

Reuses the existing `_CornerDetectionPolicy` and threshold infrastructure. The only change is the angle measurement source.

#### 2.2 `_filter_chord_candidates_by_arc_spacing(candidates, cumulative_arcs, min_arc_spacing, *, wrapped=False, total_length=0.0) -> list[_CornerTurnCandidate]`

Replacement for both `_filter_corner_candidates_by_index_spacing` (open) and `_filter_closed_corner_candidates_by_loop_spacing` (closed).

**Unified logic**: for both open and closed polylines, spacing is measured in arc-length, not vertex count. This eliminates the semantic gap between the two paths (Problem #2 in analysis).

NMS by arc-length neighborhood: if two candidates are within `min_arc_spacing`, keep only the one with larger turn angle.

For closed loops, spacing wraps around.

#### 2.3 `_detect_chord_closed_loop_corners(loop_vert_cos, basis_u, basis_v, loop_vert_indices=None, bm=None) -> list[int]`

Parallel to `_detect_closed_loop_corner_indices`. Uses chord-based candidates + arc-spacing filter.

Same contract: returns list of vertex indices within the loop where corners are detected. Same `min_corner_count >= 4` rule for closed-loop geometric split.

#### 2.4 `_detect_chord_open_border_corners(vert_cos, basis_u, basis_v, min_spacing=2, vert_indices=None, bm=None) -> list[int]`

Parallel to `_detect_open_border_corner_indices`. Uses chord-based candidates + arc-spacing filter.

Same contract: returns list of vertex indices where corners are detected.

### Verification — Side-by-Side Comparison

Add a diagnostic function (not for production):

```python
def _compare_corner_detection_methods(polyline_cos, basis_u, basis_v, *, closed=False, vert_indices=None, bm=None):
    """Diagnostic: run both old and new detectors, print comparison."""
```

This should be callable from a temporary operator or console debug path.

Run on at least 3 production meshes:
- Simple rectangular wall (should give same 4 corners both ways)
- Bevel-heavy wall (old method produces unstable corners; new should be stable)
- Curved wall strip (old method produces false positives; new should not)

Report format:
```
Patch N Loop M: old_corners=[...] new_corners=[...] match=Y/N
  old_roles=[H,V,H,V,...] new_roles=[H,FREE,V,H,...]
```

### Stop Gate

Do not proceed to Phase 3 until side-by-side comparison shows:
1. New method produces same or fewer corners on production meshes
2. New method produces same corners on simple rectangular meshes
3. New method is stable across tessellation density variants of the same shape

---

## Phase 3: Support Validation Adaptation

### Goal

Adapt the existing support validation logic to work with chord-based candidates. Support validation is already the right idea — it checks whether a corner candidate has stable geometry on both sides. The problem is that it currently validates candidates that were poorly selected. With better candidates, support validation becomes more reliable.

### Scope

New parallel functions in `cftuv/analysis_corners.py`.

### Deliverables

#### 3.1 `_chord_open_corner_has_support(vert_cos, cumulative_arcs, corner_index, basis_u, basis_v, window_arc) -> Optional[dict]`

Parallel to `_open_corner_has_support`. Same semantics, but:

- `min_support_length` is now derived from `window_arc` (at least `window_arc * 1.5`) instead of absolute constants
- This ties support requirement to the same scale as corner detection itself

#### 3.2 `_chord_open_corner_pair_has_support(vert_cos, cumulative_arcs, left_corner_index, right_corner_index, basis_u, basis_v, window_arc) -> Optional[dict]`

Parallel to `_open_corner_pair_has_support`. Same connector-pair logic, but `connector_length_limit` is derived from `window_arc` instead of absolute constants.

#### 3.3 `_filter_chord_open_corner_indices_with_support(raw_chain, cumulative_arcs, corner_indices, basis_u, basis_v, window_arc) -> list[int]`

Parallel to `_filter_open_chain_corner_indices_with_support`. Same micro-gap elimination logic, adapted to use arc-length metrics.

#### 3.4 `_detect_chord_open_border_corners_full(raw_chain, basis_u, basis_v, bm=None, min_spacing=1) -> _OpenBorderCornerDetectionResult`

Parallel to `_detect_open_border_corners`. Full pipeline: chord candidates → support filter → result.

### Verification

Side-by-side comparison on open MESH_BORDER chains:
- L-shaped border (should produce 1 corner, both methods)
- U-shaped border (should produce 2 corners, both methods)
- Diagonal connector between two H/V segments (old method may produce false corners; new should not)

### Stop Gate

Do not proceed to Phase 4 until support-filtered chord corners are equal or better than old support-filtered corners on production meshes.

---

## Phase 4: Integration Wrappers — Parallel Path

### Goal

Create integration-ready wrappers that can be swapped into `analysis_boundary_loops.py` and `analysis_corners.py` entry points with a single flag.

### Scope

Modify `cftuv/analysis_corners.py` to add a routing layer. Do NOT yet change the default path.

### Deliverables

#### 4.1 Feature flag in `constants.py`

```python
# Corner detection method selection
# False = legacy vertex-hop (current default)
# True  = chord-based (new, must pass regression before enabling)
USE_CHORD_CORNER_DETECTION = False
```

#### 4.2 Routing wrappers

Modify the existing public-facing functions to delegate:

```python
def _collect_geometric_split_indices(loop_vert_cos, basis_u, basis_v, loop_vert_indices=None, bm=None):
    if USE_CHORD_CORNER_DETECTION:
        return _detect_chord_closed_loop_corners(loop_vert_cos, basis_u, basis_v, loop_vert_indices, bm)
    return _detect_closed_loop_corner_indices(loop_vert_cos, basis_u, basis_v, loop_vert_indices, bm)

def _find_open_chain_corners(vert_cos, basis_u, basis_v, min_spacing=2, vert_indices=None, bm=None):
    if USE_CHORD_CORNER_DETECTION:
        return _detect_chord_open_border_corners(vert_cos, basis_u, basis_v, min_spacing, vert_indices, bm)
    return _detect_open_border_corner_indices(vert_cos, basis_u, basis_v, min_spacing, vert_indices, bm)
```

And similarly for the `_full` support-filtered variant.

#### 4.3 Corner measurement routing in `_measure_corner_turn_angle` callsites

The existing `_measure_corner_turn_angle` is used in two places:
1. Corner detection (being replaced)
2. Corner record creation in `_derive_loop_corners_from_final_chains` — the `turn_angle_deg` field on `BoundaryCorner`

For (2), when `USE_CHORD_CORNER_DETECTION=True`, use chord-based measurement for `turn_angle_deg` so the recorded angle reflects the stabilized measurement. This requires passing `cumulative_arcs` and `window_arc` through to corner building — add them as optional parameters with fallback to vertex-hop if not provided.

### Verification

1. With `USE_CHORD_CORNER_DETECTION = False`: full regression suite passes, zero diff in snapshots
2. With `USE_CHORD_CORNER_DETECTION = True`: run on full production mesh set, compare snapshots
3. Document every diff in a comparison report

### Stop Gate

Do not proceed to Phase 5 until:
- Flag=False produces identical output to current codebase
- Flag=True produces a documented comparison report
- Alexander reviews the comparison and approves the swap

---

## Phase 5: Swap Default and Cleanup

### Goal

Enable chord-based detection as default. Remove old code paths after validation period.

### Scope

This phase has two sub-steps with a validation gap between them.

### 5.1 Enable by default

```python
USE_CHORD_CORNER_DETECTION = True
```

Re-run full regression suite. Save new baseline snapshots.

### 5.2 Cleanup (after validation period)

Only after Alexander confirms the new baselines are accepted:

- Remove old vertex-hop corner detection functions
- Remove the feature flag routing
- Remove the diagnostic comparison function
- Update `AGENTS.md` and `docs/cftuv_reference.md` with new corner detection description
- Clean up any constants that are no longer used

### Verification

- Full regression snapshot comparison
- Grease Pencil debug visualization matches expected topology
- Frontier builder produces expected quilt/patch/chain counts
- No new unsupported patches or closure failures

### Stop Gate

Phase 5.2 (cleanup) requires explicit approval from Alexander after a validation period with real production meshes.

---

## File Change Summary

| Phase | Files Created | Files Modified |
|-------|--------------|----------------|
| 0 | `analysis_arc_utils.py` | — |
| 1 | — | `analysis_corners.py` (add functions), `constants.py` (add constants) |
| 2 | — | `analysis_corners.py` (add functions) |
| 3 | — | `analysis_corners.py` (add functions) |
| 4 | — | `analysis_corners.py` (add routing), `constants.py` (add flag) |
| 5.1 | — | `constants.py` (flip flag) |
| 5.2 | — | `analysis_corners.py` (remove old), `constants.py` (remove flag), `AGENTS.md`, `docs/cftuv_reference.md` |

No changes to: `model.py`, `solve_*.py`, `operators.py`, `debug.py`, `analysis_boundary_loops.py` (it calls the same public functions which now route internally).

---

## Agent Instructions Per Phase

### Common Rules

- Read `AGENTS.md` before starting any phase.
- Comments in Russian, docstrings Russian or English.
- All functions prefixed with `_` (private).
- Use `try/except ImportError` pattern for relative/absolute import compatibility.
- Use `from mathutils import Vector` — no other external dependencies.
- Do NOT modify any function signature that is called from outside `analysis_corners.py` unless explicitly specified in the deliverables.
- After each phase, report:
  1. Files changed (with line counts)
  2. Functions added/modified
  3. Verification results
  4. Any deviations from plan and rationale
  5. Open risks

### Phase-Specific Agent Briefs

**Phase 0 agent:**
You are building geometric utilities. No domain knowledge needed beyond "polyline = list of 3D points". Focus on correctness and edge cases. The functions you build will be called thousands of times per mesh — avoid unnecessary allocations.

**Phase 1 agent:**
You are replacing the angle measurement core. You need to understand: the old `_measure_corner_turn_angle` takes 3 points. Your new function takes a polyline + index + window. The output semantics are the same (angle in degrees), but the input neighborhood is arc-length-based instead of index-based. Read `_measure_corner_turn_angle` and `_measure_corner_turn_angle_3d` to understand the 2D projection and 3D variants.

**Phase 2 agent:**
You are building the chord-based candidate pipeline. Study `_collect_corner_turn_candidates_shared` carefully — your function has the same contract (same policy type, same output type). The key difference: spacing filter uses arc-length, not vertex indices. Study both `_filter_corner_candidates_by_index_spacing` and `_filter_closed_corner_candidates_by_loop_spacing` — your unified function replaces both.

**Phase 3 agent:**
You are adapting support validation. Study `_open_corner_has_support`, `_open_corner_pair_has_support`, and `_filter_open_chain_corner_indices_with_support`. Your versions use the same logic but derive thresholds from `window_arc` instead of absolute constants. The support concept is sound — you are stabilizing its inputs, not redesigning it.

**Phase 4 agent:**
You are wiring the new path in parallel. The key constraint: with flag=False, output must be **bit-identical** to current codebase. With flag=True, the new path activates. Study all callsites of `_collect_geometric_split_indices`, `_find_open_chain_corners`, `_find_open_chain_corners_filtered`, and `_detect_open_border_corners` to understand the integration surface. Your routing must be transparent to callers.

**Phase 5 agent:**
You are flipping the switch and (after approval) cleaning up. Study the full pipeline from `_trace_boundary_loops` through `_build_boundary_loops` to understand what downstream code consumes corner results. Your cleanup must not leave orphan imports or dead references.

---

## Risk Register

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Chord window too small → behaves like vertex-hop | Medium | High | Phase 1 verification explicitly tests density variants |
| Chord window too large → merges adjacent corners | Medium | High | `max_absolute` constant + Phase 2 side-by-side comparison |
| Bevel filter interacts differently with chord angles | Medium | Medium | Phase 1.3 provides chord-based 3D angle; Phase 2 reuses existing bevel policy |
| Support validation thresholds need retuning | High | Low | Phase 3 derives from window_arc, not absolute constants; tune on production meshes |
| Downstream frame role changes break solve | Low | High | Phase 4 flag=False guarantees zero diff; Phase 5 requires full regression |
| Performance regression on dense meshes | Low | Medium | Phase 0 pre-computes cumulative arcs once; chord lookup is O(log N) with bisect |

---

## Success Criteria

After Phase 5.1:

1. **Tessellation invariance**: same architectural shape with 1x and 4x vertex density produces identical corner sets (within ±1 corner tolerance for extreme density ratios)
2. **No false staircase**: diagonal connectors between H/V segments remain as single FREE chains, not split into false H-V-H sequences
3. **Stable regression**: production mesh snapshots show equal or fewer anomalies than before
4. **No solve regression**: quilt count, patch count, unsupported count, closure seam metrics are equal or better
5. **Performance**: analysis time on production meshes within 1.5x of current (acceptable trade for stability)

---

## Appendix A: Conceptual Pseudocode for Chord Angle

```python
def chord_turn_angle(polyline, index, window_arc, basis_u, basis_v, wrapped):
    arcs = cumulative_arc_lengths(polyline)
    total = arcs[-1]
    current_arc = arcs[index]

    if wrapped:
        back_point = interpolate_wrapped(polyline, arcs, total, current_arc - window_arc)
        forward_point = interpolate_wrapped(polyline, arcs, total, current_arc + window_arc)
    else:
        back_point = interpolate_clamped(polyline, arcs, current_arc - window_arc)
        forward_point = interpolate_clamped(polyline, arcs, current_arc + window_arc)

    incoming = project_2d(polyline[index] - back_point, basis_u, basis_v)
    outgoing = project_2d(forward_point - polyline[index], basis_u, basis_v)

    return angle_between(incoming, outgoing)  # degrees
```

## Appendix B: Unified Arc-Spacing NMS Pseudocode

```python
def filter_by_arc_spacing(candidates, arcs, min_arc, wrapped, total_length):
    if not candidates:
        return []
    result = [candidates[0]]
    for candidate in candidates[1:]:
        prev = result[-1]
        spacing = arc_distance(arcs, prev.index, candidate.index, total_length, wrapped)
        if spacing < min_arc:
            if candidate.turn_angle > prev.turn_angle:
                result[-1] = candidate
        else:
            result.append(candidate)
    # For wrapped: also check spacing between last and first
    if wrapped and len(result) >= 2:
        wrap_spacing = arc_distance(arcs, result[-1].index, result[0].index, total_length, True)
        if wrap_spacing < min_arc:
            # keep stronger of the two
            ...
    return result
```
