# CFTUV Structural Interpretation Layer — Plan

Status: approved design, pre-implementation
Phase scope: analysis layer only — zero solve/frontier changes
Goal: enable Straighten Patch by enriching existing derived topology with structural interpretation

---

## Strategic Context

This layer is not only about Straighten Patch. It is the **shared semantic foundation** for a family of future unwrap strategies. Today the system has no structural vocabulary between raw topology (chains, corners, runs) and placement decisions. Every new shape case (tube/cable unwrap, arc trim-shell, T-patch layout) would have to be solved as an isolated special case reading raw chain/corner data.

The structural interpretation layer provides a **common language**: spine, terminal, branch, continuation, turn, side pair, strip confidence. Future unwrap strategies — cylindrical unwrap, arc-aware layout, branch-aware T-placement, and others not yet designed — will all consume this same vocabulary instead of re-deriving structural meaning from scratch. Straighten Patch is the first consumer, not the only one.

---

## Motivation

The system needs a Straighten Patch capability: for strip-like patches (long curved walls, narrow floor bands), UV placement should produce a clean straight rectangle instead of following 3D curvature chain-by-chain.

Current state:
- `PatchShapeProfile` (in `solve_records.py`) provides weak numeric priors (elongation, rectilinearity, frame_dominance) used only for minor scoring bonuses (+/-0.03..0.08). It does not affect placement strategy.
- `_Junction` (in `analysis_records.py`, built by `analysis_junctions.py`) is a purely diagnostic/analytical object. It stores `role_signature`, `valence`, `chain_refs` but is never consumed by solve or placement.
- `_FrameRun` (in `analysis_records.py`, built by `analysis_frame_runs.py`) captures continuous aligned chain sequences but has no structural role annotation.
- There is no shape-driven placement strategy. All patches follow the same chain-by-chain frontier logic.

---

## Design Principles

### P1. Not a new system — enrichment of existing derived topology

All new data lives inside `_PatchGraphDerivedTopology`. No parallel graph, no separate shape subsystem, no new pipeline phase. The existing IR (`PatchGraph`) remains the single source of truth.

### P2. Fact vs Interpretation separation

`_FrameRun` and `_Junction` store **topological facts** (what exists). New companion records store **structural interpretation** (what it means for straightening). We never add interpretation fields to fact-level dataclasses.

### P3. Run-first, Junction-supported

`_FrameRun` is the primary carrier for spine/side detection. `_Junction` provides support: what happens at run endpoints (terminal, continuation, turn, branch). Junction is not the center of shape reasoning.

### P4. Descriptor-first, Label-second

No `PatchShape` enum on v1. Continuous signals (`strip_confidence`, `straighten_eligible`) drive decisions. A convenience label (STRIP_OPEN, RECTLIKE, etc.) may be added later as presentation layer, not as a branch-point for logic.

### P5. Solve untouched on this phase

Phase A+B are pure analysis. Phase C (actual straighten placement) is a separate future step, gated on descriptor stability over regression meshes.

---

## Architecture: Three-Layer Model

```
Layer 1: Topology Facts (existing, unchanged)
    loops → chains → corners → frame runs → junctions

Layer 2: Structural Interpretation (NEW, this plan)
    run structural roles → junction structural roles → enriched patch summaries

Layer 3: Consumers (future, NOT this plan)
    reporting, debug, straighten placement
```

---

## What We Add

### New type aliases (in `analysis_records.py`)

```python
RunKey = tuple[int, int, int]          # (patch_id, loop_index, run_index)
JunctionPatchKey = tuple[int, int]     # (vert_index, patch_id)
```

Place next to existing aliases (`ChainRef`, `PatchEdgeKey`, etc.) for discoverability.

### New enum (in `analysis_records.py`)

```python
class _JunctionStructuralKind(str, Enum):
    """Structural role of a junction vertex in context of one patch."""
    TERMINAL = "TERMINAL"         # run endpoint at mesh border
    CONTINUATION = "CONTINUATION" # two runs of same axis meet — spine continues
    TURN = "TURN"                 # two runs of different axes — 90-degree corner
    BRANCH = "BRANCH"             # valence >= 3 — T/Y junction
    FREE = "FREE"                 # unresolved
```

### New annotation record: `_RunStructuralRole`

```python
@dataclass(frozen=True)
class _RunStructuralRole:
    """Structural interpretation of one FrameRun within its patch.
    Companion to _FrameRun — fact vs interpretation separation."""

    run_key: RunKey
    is_spine_candidate: bool = False
    spine_rank: int = -1                      # 0=primary, 1=secondary, -1=not spine
    opposing_run_key: Optional[RunKey] = None  # paired side run on perpendicular axis
    side_pair_length_ratio: float = 0.0        # |len_a - len_b| / max(len_a, len_b)
    inherited_role: Optional[FrameRole] = None       # frame role inherited from neighbor
    inherited_from_patch_id: Optional[int] = None    # which neighbor provided the role
```

Key semantics:
- `is_spine_candidate` = True for runs belonging to the longest run family along dominant axis
- `spine_rank` 0 = primary spine (longest), 1 = secondary
- `opposing_run_key` links spine run to its paired side run (if exists)
- `side_pair_length_ratio` near 0.0 = good pair, near 1.0 = poor pair
- `inherited_role` = frame role inherited from a strong neighbor (for FREE runs only)
- `inherited_from_patch_id` = which neighbor provided the inherited role

### Neighbor-Inherited Frame Roles

A critical insight: many real-world patches have FREE chains that are structurally aligned but
not classified as H/V by the frame classifier. When such a FREE chain borders a neighbor patch
with a strong H/V chain along the same boundary, the FREE chain inherits the neighbor's frame
role for structural interpretation purposes.

Example: an arc mesh where a WALL patch has chains `[FREE, H, FREE, H]`. The first FREE chain
borders a FLOOR patch that has V_FRAME along the shared boundary. The FREE chain inherits
V_FRAME, making the patch a strip with spine axis V.

The second FREE chain (MESH_BORDER, no neighbor) gets no inheritance. This is fine — Conformal
mapping will resolve it based on the other chains' placement. The system degrades gracefully:
neighbor info enriches interpretation when available, but its absence doesn't block anything.

Implementation: `_compute_neighbor_inherited_roles(graph)` returns a dict mapping
`ChainRef -> (inherited_role, source_patch_id)`. This feeds into `_interpret_run_structural_roles`
so FREE runs with inherited roles participate in spine detection alongside own H/V runs.

### New annotation record: `_JunctionStructuralRole`

```python
@dataclass(frozen=True)
class _JunctionStructuralRole:
    """Structural role of a junction vertex in context of one patch.
    Derived view over existing _Junction + _FrameRun data."""

    vert_index: int
    patch_id: int
    kind: _JunctionStructuralKind = _JunctionStructuralKind.FREE
    spine_run_key: Optional[RunKey] = None  # spine run ending at this junction
    implied_turn: float = -1.0               # 0.0=straight, 90.0=corner, -1=unknown
```

Key semantics:
- One junction vertex may have **different roles in different patches** (hence `patch_id` in key)
- `kind` is derived from `_Junction.role_signature` + run endpoint analysis
- `spine_run_key` links to the spine run that terminates here (if any)

### Extended fields on `_PatchDerivedTopologySummary`

```python
# Add to existing dataclass:
    spine_run_indices: tuple[int, ...] = ()
    spine_axis: FrameRole = FrameRole.FREE
    spine_length: float = 0.0
    terminal_count: int = 0
    branch_count: int = 0
    strip_confidence: float = 0.0
    straighten_eligible: bool = False
```

These fields are filled by the structural interpretation pass and consumed by reporting / future straighten.

### Extended fields on `_PatchGraphDerivedTopology`

```python
# Add to existing dataclass:
    run_structural_roles: Mapping[RunKey, _RunStructuralRole] = field(default_factory=dict)
    junction_structural_roles: Mapping[JunctionPatchKey, _JunctionStructuralRole] = field(default_factory=dict)
```

---

## What We Do NOT Add

| Excluded | Reason |
|----------|--------|
| `PatchShapeDescriptor` class | Fields go into existing `_PatchDerivedTopologySummary` |
| `analysis_shape.py` file | Logic goes into existing `analysis_derived.py` |
| `shape_label` / `PatchShape` enum | Not needed on v1; descriptor-first |
| Any changes to solve/frontier | Phase C, separate plan |
| Modification of `_FrameRun` fields | Companion record instead; fact vs interpretation |
| Mutable dict defaults | Use `field(default_factory=dict)` everywhere |
| String literals for junction kind | `_JunctionStructuralKind` enum |

---

## New Functions (in `analysis_derived.py`)

### `_interpret_run_structural_roles()`

```
Input:  frame_runs_by_loop, junctions
Output: dict[RunKey, _RunStructuralRole]
```

Algorithm:
1. Pre-compute neighbor-inherited roles: `_compute_neighbor_inherited_roles(graph)` → dict[ChainRef, (FrameRole, int)]
2. Group all runs by `patch_id`
3. For each patch, separate runs by `dominant_role` (H vs V)
4. For FREE runs: check if ALL chains in the run have the SAME inherited role → treat as effective H/V for spine detection
5. Compute axis totals: own H/V length + inherited H/V length
6. Identify spine axis: the axis family with the greater total run length
7. Rank spine runs by `total_length` (rank 0 = longest = primary spine); includes inherited-role runs
8. For each spine run, find opposing run (same effective axis, same loop, closest length)
9. Return run_key -> _RunStructuralRole mapping (with inherited_role/inherited_from_patch_id for FREE runs)

### `_interpret_junction_structural_roles()`

```
Input:  junctions, frame_runs_by_loop, run_structural_roles
Output: dict[JunctionPatchKey, _JunctionStructuralRole]
```

Algorithm per junction, per patch_id in junction.patch_ids:
1. If `junction.valence == 1` and `junction.is_open` -> TERMINAL
2. If `junction.valence >= 3` -> BRANCH
3. If `junction.valence == 2`:
   - Extract `role_signature` pair for this patch
   - If both runs on same axis (H->H or V->V) -> CONTINUATION
   - If runs on different axes (H->V or V->H) -> TURN (implied_turn=90.0)
   - Otherwise -> FREE
4. Link `spine_run_key` if any incoming run is a spine candidate

### `_derive_patch_structural_summary()`

```
Input:  frame_runs_by_loop, run_structural_roles, junction_structural_roles
Output: dict[int, dict]  # patch_id -> field values to merge into summary
```

Algorithm per patch:
1. Collect spine runs from `run_structural_roles` where `is_spine_candidate == True`
2. `spine_axis` = dominant_role of primary spine run
3. `spine_length` = sum of total_length of all spine runs
4. `terminal_count` = count of TERMINAL junctions for this patch
5. `branch_count` = count of BRANCH junctions for this patch
6. Compute `strip_confidence`:
   ```
   spine_ratio = spine_length / patch_perimeter  (good strip: ~0.4-0.5)
   spine_score = clamp01(spine_ratio / 0.45)

   side_confidence = average side_pair_confidence across spine runs

   strip_confidence = clamp01(
       0.35 * spine_score
       + 0.25 * elongation        (from PatchShapeProfile or bbox)
       + 0.25 * side_confidence
       + 0.15 * (1.0 if branch_count == 0 else 0.0)
       - 0.30 * branch_count      (hard penalty)
   )
   ```
7. `straighten_eligible` = `strip_confidence > threshold AND branch_count == 0 AND terminal_count >= 1`

Note: `strip_confidence` formula is a bootstrap heuristic. Expect tuning after regression testing. The formula intentionally separates concerns (spine coverage, elongation, side pairing, branch penalty) so individual terms can be debugged and adjusted independently.

### `_enrich_patch_summaries()`

```
Input:  existing patch_summaries, structural_fields dict
Output: updated patch_summaries with spine/strip fields merged
```

Purely mechanical: for each patch_id, create a new `_PatchDerivedTopologySummary` with existing fields + new structural fields.

---

## Wiring Into Existing Pipeline

In `analysis_derived.py`, function `_build_patch_graph_derived_topology()`:

```python
def _build_patch_graph_derived_topology(graph, ...):
    # ... existing code: build frame_runs, junctions, summaries ...

    # --- NEW: structural interpretation pass ---
    run_roles = _interpret_run_structural_roles(frame_runs_by_loop, junctions)
    junction_roles = _interpret_junction_structural_roles(
        junctions, frame_runs_by_loop, run_roles
    )
    structural_fields = _derive_patch_structural_summary(
        frame_runs_by_loop, run_roles, junction_roles
    )
    patch_summaries = _enrich_patch_summaries(patch_summaries, structural_fields)

    return _PatchGraphDerivedTopology(
        # ... all existing fields ...
        run_structural_roles=run_roles,
        junction_structural_roles=junction_roles,
    )
```

The new code runs **after** all existing analysis is complete. It is a pure consumer of existing facts. It cannot break existing behavior because it only writes to new fields.

---

## Implementation Order

| Step | What | Where | Dependencies |
|------|------|-------|-------------|
| 1 | Type aliases `RunKey`, `JunctionPatchKey` | `analysis_records.py` | none |
| 2 | Enum `_JunctionStructuralKind` | `analysis_records.py` | none |
| 3 | Dataclass `_RunStructuralRole` | `analysis_records.py` | step 1 |
| 4 | Dataclass `_JunctionStructuralRole` | `analysis_records.py` | steps 1, 2 |
| 5 | New fields on `_PatchDerivedTopologySummary` | `analysis_records.py` | step 1 |
| 6 | New fields on `_PatchGraphDerivedTopology` | `analysis_records.py` | steps 3, 4 |
| 7 | `_interpret_run_structural_roles()` | `analysis_derived.py` | steps 1, 3 |
| 8 | `_interpret_junction_structural_roles()` | `analysis_derived.py` | steps 1, 2, 4, 7 |
| 9 | `_derive_patch_structural_summary()` | `analysis_derived.py` | steps 5, 7, 8 |
| 10 | `_enrich_patch_summaries()` | `analysis_derived.py` | steps 5, 9 |
| 11 | Wire into `_build_patch_graph_derived_topology()` | `analysis_derived.py` | steps 7-10 |

Steps 1-6 are data structure definitions (safe, no behavior change).
Steps 7-10 are pure functions (testable in isolation).
Step 11 is the integration point (single wiring change).

---

## Future Phase C: Straighten Placement (NOT in scope)

For reference only. Phase C will be a separate plan after structural interpretation proves stable.

Straighten placement means: for patches where `straighten_eligible == True`, bypass chain-by-chain frontier and place the entire patch as one coherent rectangle.

Conceptual difference from current placement:

```
Current (chain-by-chain):
    Each chain placed individually via frontier scoring.
    Curved wall → each chain slightly rotated → accumulated angle drift.

    ┌──────┐
    │      │╲
    │      │ ╲
    │      │  ╲
    └──────┘   ╲

Straighten (whole-patch):
    All spine chains on one UV line. Sides parallel. Caps perpendicular.
    Curvature absorbed into edge lengths, not angles.

    ┌──────────────────┐
    │                  │
    │                  │
    └──────────────────┘
```

Phase C will consume:
- `spine_run_indices` + `spine_axis` (from patch summary) → UV spine direction
- `cap_chain_refs` (to be added in Phase C) → perpendicular end segments
- `terminal_count` → open vs closed strip handling
- Junction structural roles → endpoint pinning

---

## Validation Criteria

Before Phase C, the structural interpretation must demonstrate:

1. **Stability**: spine detection and strip_confidence must be deterministic across runs
2. **Coverage**: on test meshes, known strip-like patches must have `strip_confidence > 0.7`
3. **Selectivity**: non-strip patches (L-shapes, blobs, branched) must have `strip_confidence < 0.3`
4. **Zero regression**: existing solve behavior must be unchanged (new fields are analysis-only)

---

## Key Terms Reference

| Term | Definition |
|------|-----------|
| **Spine** | Longest continuous FrameRun family along dominant axis of a patch |
| **Side pair** | Two runs on perpendicular axis that flank the spine |
| **Cap** | Short chain at the endpoint of a spine (strip terminal) |
| **Terminal** | Junction at mesh border where a spine run ends |
| **Continuation** | Junction where two runs of same axis meet — spine unbroken |
| **Branch** | Junction with valence >= 3 — disqualifies strip eligibility |
| **Strip confidence** | 0..1 score combining spine coverage, elongation, side pairing, branch penalty |
| **Straighten eligible** | Boolean gate: high strip_confidence AND no branches AND at least one terminal |
