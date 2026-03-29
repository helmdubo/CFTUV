# CFTUV — Per-Edge Atomization of MESH_BORDER Chains

> Status: Ready for execution.
> Replaces: `cftuv_corner_detection_refactor_plan.md` (chord-based approach — abandoned).

---

## Problem

MESH_BORDER boundary chains (edges without patch neighbor) enter the pipeline as
long FREE chains. To create H/V scaffold structure, a corner-detection algorithm
splits them at detected angle breaks. This corner detection is fragile — it depends
on mesh tessellation density and produces unstable results on identical shapes
with different vertex counts.

## Solution

**Do not detect corners. Atomize MESH_BORDER chains into single-edge chains,
classify each edge by direction, let the existing merge algorithm reassemble.**

Flow:
```
MESH_BORDER raw chain (N edges)
  → atomize into N single-edge raw chains (2 verts, 1 edge each)
  → _build_boundary_chain_objects: classify each → H / V / FREE
  → _merge_same_role_border_chains: adjacent same-role → merged chains
  → corners arise naturally at role transitions
```

Single-edge classification is exact — one edge direction vs patch basis, no
ambiguity, no density dependency.

## What NOT to do

- Do NOT use chord-based / sliding-window angle measurement (previous plan)
- Do NOT use vertex-hop corner detection for MESH_BORDER splitting
- Do NOT change `model.py`, `solve_*.py`, `operators.py`, or `debug.py`
- Do NOT change how PATCH-neighbor chains are split (neighbor-change split works)
- Do NOT change `_classify_chain_frame_role` logic
- Do NOT change `_merge_same_role_border_chains` logic
- Do NOT assign roles during atomization — only during classification step

---

## Scope of Change

### Files modified

1. **`analysis_boundary_loops.py`** — add atomization function, modify `_refine_boundary_loop_raw_chains`
2. **`analysis_corners.py`** — no functional changes, but remove chord-based code if present from previous plan

### Files NOT modified

Everything else. The downstream pipeline (`_build_boundary_chain_objects` → `_downgrade_same_role_point_contact_chains` → `_merge_same_role_border_chains` → `_derive_loop_corners_from_final_chains`) stays untouched.

---

## Step 1: Add atomization function

**File:** `analysis_boundary_loops.py`

Add one new function:

```python
def _atomize_raw_chain_to_edges(raw_chain):
    """Разбивает один raw chain на per-edge chains (2 вершины, 1 ребро каждый).

    Для MESH_BORDER chains: каждое ребро станет отдельным chain,
    затем _build_boundary_chain_objects классифицирует его по направлению,
    затем _merge_same_role_border_chains соберёт смежные одинаковые роли обратно.
    """
```

**Contract:**

- Input: one `_RawBoundaryChain` with N vertices, N-1 edges (open) or N edges (closed)
- Output: list of `_RawBoundaryChain`, each with exactly 2 vertices, 1 edge
- For closed chain: N edges → N single-edge chains (last edge connects last→first vertex)
- For open chain: N-1 edges → N-1 single-edge chains
- Each output chain inherits `neighbor` from input
- Each output chain has `is_closed = False`
- Each output chain has correct `start_loop_index` and `end_loop_index` relative to parent loop
- Each output chain has `is_corner_split = False`
- If input has fewer than 2 vertices: return `[raw_chain]` unchanged

**Implementation detail for closed chains:**

```
Input:  verts=[A,B,C,D], edges=[e0,e1,e2,e3], closed=True
Output: [
    (verts=[A,B], edges=[e0]),
    (verts=[B,C], edges=[e1]),
    (verts=[C,D], edges=[e2]),
    (verts=[D,A], edges=[e3]),  ← wrap edge
]
```

`side_face_indices` per output chain: each edge's chain gets the corresponding
`side_face_indices` entries from the input. For a 2-vert chain: `[side_face_indices[edge_pos], side_face_indices[next_vert_pos]]`.

Match the same vertex/edge/side_face slicing pattern that `_split_open_chain_at_corners`
already uses — study that function as reference for index arithmetic.

---

## Step 2: Modify `_refine_boundary_loop_raw_chains`

**File:** `analysis_boundary_loops.py`

Current code:
```python
def _refine_boundary_loop_raw_chains(state, basis_u, basis_v, bm):
    state.raw_chains = _merge_bevel_wrap_chains(state.raw_chains, bm)
    state.raw_chains = _try_geometric_outer_loop_split(...)
    state.raw_chains = _split_border_chains_by_corners(...)
```

New code:
```python
def _refine_boundary_loop_raw_chains(state, basis_u, basis_v, bm):
    state.raw_chains = _merge_bevel_wrap_chains(state.raw_chains, bm)
    state.raw_chains = _atomize_mesh_border_chains(state.raw_loop, state.raw_chains)
```

Where `_atomize_mesh_border_chains` is a new router function:

```python
def _atomize_mesh_border_chains(raw_loop, raw_chains):
    """Атомизирует MESH_BORDER chains до single-edge уровня.

    PATCH-neighbor и SEAM_SELF chains остаются без изменений.
    Closed single-chain OUTER loop с MESH_BORDER neighbor: атомизируется.
    Open MESH_BORDER chains: атомизируются.
    Closed single-chain OUTER loop с PATCH neighbor: оставляем как есть
    (downstream geometric corner fallback через _build_geometric_loop_corners
    создаст corners если нужно).
    """
```

**Logic:**

```
result = []
for raw_chain in raw_chains:
    if raw_chain.neighbor == NB_MESH_BORDER:
        result.extend(_atomize_raw_chain_to_edges(raw_chain))
    else:
        result.append(raw_chain)
return result
```

That's it. Simple routing: MESH_BORDER → atomize, everything else → pass through.

---

## Step 3: Handle closed single-PATCH-neighbor loops

When the entire OUTER loop has one PATCH neighbor (single closed chain facing one
patch), it currently goes through `_try_geometric_outer_loop_split`.

After Step 2, this chain will NOT be atomized (it's PATCH, not MESH_BORDER).
It will reach `_build_boundary_chain_objects` as one closed chain → classified as
one role → `_build_geometric_loop_corners` creates geometric corners from turns.

This is the existing fallback path that already works. No change needed.

**Verify:** confirm `_build_geometric_loop_corners` still gets called for
single-chain loops (it's in `_derive_loop_corners_from_final_chains`, path
`chain_count < 2`).

---

## Step 4: Remove dead code

After Steps 1-2, these functions in `analysis_corners.py` are no longer called
from the boundary loop pipeline for MESH_BORDER splitting:

- `_split_border_chains_by_corners`
- `_detect_open_border_corners`
- `_detect_open_border_corner_indices`
- `_find_open_chain_corners`
- `_find_open_chain_corners_filtered`
- `_filter_open_chain_corner_indices_with_support`
- `_open_corner_has_support`
- `_open_corner_pair_has_support`
- `_measure_open_corner_side_support`
- `_open_corner_pair_has_support`
- `_split_open_chain_at_corners`
- `_measure_polyline_length` (if no other callers)

And also no longer called:
- `_try_geometric_outer_loop_split`
- `_collect_geometric_split_indices`
- `_detect_closed_loop_corner_indices`
- `_split_closed_loop_by_corner_indices`

And any chord-based functions added by the previous (abandoned) refactor plan.

**Do NOT delete yet.** Mark with a comment:
```python
# DEAD CODE — candidate for removal after per-edge atomization validation
```

Delete only after Alexander confirms regression results.

---

## Step 5: Remove chord-based code from previous plan

If `analysis_arc_utils.py` was created — mark as dead or delete.

If `USE_CHORD_CORNER_DETECTION` flag was added to `constants.py` — remove.

If chord-based functions were added to `analysis_corners.py` — mark as dead or delete.

The per-edge atomization replaces both the old vertex-hop corner detection AND
the chord-based detection. Neither is needed for MESH_BORDER splitting.

---

## What existing code handles automatically

After atomization, the existing pipeline does all the work:

| Step | Function | What it does |
|------|----------|-------------|
| Classify | `_build_boundary_chain_objects` | Each 2-vert chain gets H/V/FREE from `_classify_chain_frame_role` |
| Conflict | `_downgrade_same_role_point_contact_chains` | Adjacent same-role at point contact: weaker → FREE |
| Merge | `_merge_same_role_border_chains` | Adjacent MESH_BORDER + same role → one chain (handles wrap) |
| Corners | `_derive_loop_corners_from_final_chains` | Corners at role transitions between merged chains |
| Endpoint | `_assign_loop_chain_endpoint_topology` | Corner↔chain linkage |
| Validate | `_validate_boundary_loop_topology` | Invariant checks |

No changes to any of these.

---

## Verification

### V1: Invariant — identical output for non-MESH_BORDER topologies

Any mesh where all boundary chains have PATCH or SEAM_SELF neighbors must produce
**identical** PatchGraph output before and after the change.

### V2: Simple rectangular wall (4 MESH_BORDER edges)

Expected: 4 edges → 4 single-edge chains → H,V,H,V → merge → 4 chains (H,V,H,V)
with 4 corners. Same as current result.

### V3: L-shaped MESH_BORDER boundary

Expected: straight horizontal segment → one H chain. Straight vertical segment →
one V chain. Corner between them. Regardless of vertex density.

### V4: U-shaped MESH_BORDER boundary

Expected: two vertical segments → two V chains. One horizontal connector → H or
FREE depending on direction. No false V-H-V staircase.

### V5: Curved/diagonal MESH_BORDER boundary

Expected: edges at varying angles → some H, some V, some FREE → isolated H/V
without H/V neighbors → not scaffold-connected → go to conformal. Curve becomes
FREE. Correct.

### V6: Bevel edges on MESH_BORDER

Expected: bevel edges at ~45° → classify as FREE → merge with adjacent FREE →
adjacent H/V chains merge across bevel gap. Same result regardless of bevel
vertex count.

### V7: Full regression suite

Run `Save Regression Snapshot` on production mesh set. Compare with previous
baseline. Report diffs.

---

## Constraints

1. Comments in Russian. Docstrings Russian or English.
2. All new functions prefixed `_`.
3. `try/except ImportError` pattern for imports.
4. No changes to `model.py`.
5. No changes to `solve_*.py`.
6. No new dependencies.
7. Grease Pencil debug must work after the change.

---

## Estimated change size

- New code: ~40–60 lines (`_atomize_raw_chain_to_edges` + `_atomize_mesh_border_chains`)
- Modified code: ~5 lines (`_refine_boundary_loop_raw_chains`)
- Dead code marked: ~200–300 lines (corner detection functions, chord functions)

This is a net code reduction.
