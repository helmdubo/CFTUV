# CFTUV Structural Token System — Execution Plan v3

Orchestrator brief for executing AI agents.

Revision note: v3 strengthens v2 in a few critical areas:
- `effective_frame_role` is explicitly downgraded to a **supporting signal only**
- `band_operator()` is constrained to be a **narrow v1 operator**, not a parallel mini-solver
- BAND path gets a **mandatory safe fallback** to generic operator on any prerequisite failure
- old band heuristics are retained for **comparison/debug only**, not as admission logic
- spine projection stays **operator-private** for v1, not a shared generic frontier primitive
- MIX path is required to preserve **baseline parity**, not just “tests pass”

---

## Context

The project needs:
1. An **explicit admission gate** (shape classification) before solve,
   replacing scattered band heuristics across runtime.
2. A **dedicated band operator** with spine-projection stationing,
   replacing the current approach of mutating the generic frontier solver via flags.

Future (not now): a decal producer that reads the same structural tokens.
Phases 2/3 describe this at design level to ensure Phase 1 doesn't block them.

### System name

**Structural Token System with Rule-Based Consumers.**

Not a grammar system. No production rules over geometry, no recursive derivation.
- A shared token layer (ChainToken, CornerToken, LoopSignature)
- Consumer A (now): rule-based shape classifier
- Consumer B (future): pattern-action decal producer

### Core principles

1. **Tokens are thin views, not a new canonical model.** They reformat existing
   BoundaryChain / BoundaryCorner / PatchNode data. They do not replace model.py types.
2. **One chain = one ChainToken.** Future consumers (junctions, decals) reference
   the same token, never copy its fields.
3. **Shape classifier starts strict.** Under-classify (MIX) is always safer than
   false-positive BAND. Relax rules later based on test evidence.
4. **Band operator is an explicit code boundary.** Not `straighten_enabled=True`
   on the generic solver, but a separate `band_operator()` entry point that may
   reuse frontier internals.
5. **Shape identity is not FrameRole identity.** `effective_frame_role` is only a
   supporting signal for shape classification, never the primary determinant of shape.
6. **Generic path stability is sacred.** MIX behavior must remain baseline-equivalent.
   New BAND logic must not change generic frontier results.

---

## Phase 0 — Minimal Cleanup

**Goal:** Fix the one type issue that directly affects Phase 1 integration.
Other cleanup is optional and can be done anytime (in parallel or after Phase 1).

### Task 0.1 — Fix type erasure on patch_structural_summaries (required)

In `cftuv/frontier_state.py`, class `FrontierRuntimePolicy`:

```python
# BEFORE:
patch_structural_summaries: dict[int, object] = field(default_factory=dict)

# AFTER:
patch_structural_summaries: dict[int, _PatchDerivedTopologySummary] = field(default_factory=dict)
```

Add the necessary import. Replace `_patch_summary_attr()` getattr() calls
with direct typed attribute access where possible.

This is the only cleanup task that is a **prerequisite** for Phase 1.

### Task 0.2 — Optional cleanup (non-blocking, can be done anytime)

These are safe to do in parallel with Phase 1 or after it:

- Delete dead types: `_BoundaryLoopDerivedTopology`, `_BoundaryLoopBuildState`
  from analysis_records.py (zero usages, never instantiated).
- Eliminate `LoopChainRef` (7 usages, redundant with ChainRef).
- Relocate `_AnalysisUvClassificationState` → analysis_classification.py.
- Relocate `_PatchNeighborChainRef` → analysis_topology.py.

### Phase 0 acceptance criteria

- `patch_structural_summaries` has correct type annotation
- All existing tests pass unchanged

---

## Phase 1 — Shape Classification + Band Operator

**Goal:** Every patch gets a `PatchShapeClass` before solve.
BAND patches go through an explicit `band_operator()`.
Everything else goes through the existing generic frontier unchanged.

This is the **only phase that needs to ship now**.

### Task 1.1 — Token dataclasses

Create `cftuv/structural_tokens.py` (new file).

```python
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from cftuv.model import ChainRef, FrameRole


class ChainRoleClass(str, Enum):
    """Structural role of a chain in its loop. Independent of FrameRole."""
    SIDE = "SIDE"
    CAP = "CAP"
    BORDER = "BORDER"
    FREE = "FREE"


class PatchShapeClass(str, Enum):
    """Shape classification of a patch. Determines operator choice."""
    MIX = "MIX"
    BAND = "BAND"


@dataclass(frozen=True)
class ChainToken:
    """Thin structural view of one boundary chain.
    Contains only what the shape classifier needs.
    Future fields (spin, point_count, etc.) are added when consumers need them.
    """
    chain_ref: ChainRef
    role_class: ChainRoleClass
    effective_frame_role: FrameRole     # SUPPORTING SIGNAL ONLY, not shape identity
    length: float                       # 3D arc length (sum of edge lengths)
    neighbor_patch_id: int | None       # None if mesh border
    is_border: bool                     # True if mesh boundary
    opposite_ref: ChainRef | None       # paired opposite chain if detected


@dataclass(frozen=True)
class CornerToken:
    """Thin structural view of one boundary corner."""
    vert_index: int
    turn_angle_deg: float               # from BoundaryCorner.turn_angle_deg
    prev_chain_ref: ChainRef
    next_chain_ref: ChainRef


@dataclass(frozen=True)
class LoopSignature:
    """Ordered structural view of one boundary loop.
    Chains and corners alternate around the loop.
    """
    patch_id: int
    loop_index: int
    chain_tokens: tuple[ChainToken, ...]
    corner_tokens: tuple[CornerToken, ...]
    # Derived (computed at construction, cached):
    chain_count: int
    has_opposite_pairs: bool
    side_count: int                     # chains with role_class == SIDE
    cap_count: int                      # chains with role_class == CAP
```

Design notes for executing agent:
- **Minimal fields only.** Do not add fields the classifier does not read.
  Future consumers (decals) will add fields (spin, point_count) when they arrive.
- `opposite_ref` detection: reuse logic from `frontier_state.py:_paired_candidate_ref()`.
  Extract the core matching (same loop, same role, non-adjacent, longest match)
  into a pure function in structural_tokens.py. Do NOT duplicate.
- `effective_frame_role` is allowed here only as a **supporting signal**.
  Shape must still be determined primarily by loop structure, side/cap arrangement,
  opposite pairing, lengths, and corners.
- `ChainRoleClass` determination (simple v1):
  if `is_border` → BORDER.
  If `opposite_ref` exists and chain is among the longest pair → SIDE.
  If shorter than sides → CAP.
  Else → FREE.
- `length` = sum of 3D edge lengths along chain. Already computed in several places
  in the analysis layer — find and reuse, do not recompute.
- `CornerToken.turn_angle_deg` comes directly from `BoundaryCorner.turn_angle_deg`.

### Task 1.2 — LoopSignature builder

In `cftuv/structural_tokens.py`:

```python
def build_loop_signature(
    patch_id: int,
    loop_index: int,
    loop: BoundaryLoop,
    node: PatchNode,
) -> LoopSignature:
    """Build a LoopSignature from existing BoundaryLoop data.
    All input data comes from model.py types — no new analysis.
    """
    ...
```

Steps:
1. Build ChainTokens from `loop.chains` (compute role_class, opposite_ref, length).
2. Build CornerTokens from `loop.corners` (extract angle, vert_index, refs).
3. Compute derived counts (side_count, cap_count, has_opposite_pairs).
4. Return frozen LoopSignature.

Note: `opposite_ref` detection requires seeing all chains in the loop first,
then pairing. Build all ChainTokens in a first pass with `opposite_ref=None`,
then fill in opposites in a second pass, then freeze.

### Task 1.3 — Shape classifier

In `cftuv/structural_tokens.py`:

```python
def classify_patch_shape(signatures: list[LoopSignature]) -> PatchShapeClass:
    """Classify patch shape from its loop signatures.
    Uses only the primary (outer) loop.
    """
    ...
```

BAND rules (all must hold for primary loop):
1. `chain_count == 4`
2. `side_count == 2`
3. `cap_count == 2`
4. Both SIDE chains have same `effective_frame_role` (both H or both V)
5. Both SIDE chains have `opposite_ref` pointing to each other
6. Side/cap length ratio > threshold (start with 1.5, tune later)

Everything else → MIX.

**Important:**
- `effective_frame_role` is only rule #4, not the primary basis of classification.
- The old `band_candidate` / `band_confirmed_for_runtime` fields remain alive
  **for comparison/debug only**. Add comment `# DEPRECATED: see PatchShapeClass`.
- Do NOT let old band heuristics participate in runtime admission once
  `PatchShapeClass` is wired.

### Task 1.4 — Integration into analysis pipeline

In `cftuv/analysis_derived.py` (or `cftuv/analysis.py`):

1. After building `_PatchDerivedTopologySummary` for each patch,
   also call `build_loop_signature()` for each patch's loops.
2. Call `classify_patch_shape()` for each patch.
3. Add to `_PatchGraphDerivedTopology`:

```python
patch_shape_classes: Mapping[int, PatchShapeClass] = {}
loop_signatures: Mapping[int, list[LoopSignature]] = {}
```

### Task 1.5 — Operator dispatch with explicit band_operator

**Critical design decision:** the band operator is a **separate function**,
not a flag on the generic solver.

In `cftuv/solve_frontier.py` (or a new `cftuv/band_operator.py` if cleaner):

```python
def band_operator(
    patch_id: int,
    graph: PatchGraph,
    runtime_policy: FrontierRuntimePolicy,
    loop_signature: LoopSignature,
    final_scale: float,
) -> ScaffoldPatchPlacement:
    """Dedicated operator for BAND-shaped patches.

    Uses spine-projection stationing along the resolved band axis.
    May reuse frontier internals (anchor resolution, frame segment,
    point registry) but is NOT a parameterized call to the generic solver.
    """
    ...
```

The band operator v1 is deliberately narrow:
1. Identifies the two SIDE chains and two CAP chains from LoopSignature.
2. Resolves the band axis (from SIDE effective_frame_role).
3. Resolves frame segment (start_uv, end_uv, span) — can reuse
   `_resolve_frame_segment()` from frontier_place.py.
4. For each SIDE chain: computes spine-projection stations (Task 1.6).
5. Places SIDE chains using `_build_frame_chain_from_stations()`.
6. Places CAP chains using guided free chain logic or similar.
7. Returns ScaffoldPatchPlacement.

**Guardrail:** do not let `band_operator()` become a parallel mini-solver.
It should be a small operator-specific branch over a confirmed band patch,
not a full alternative frontier system.

Dispatch logic in the main solve loop:

```python
shape = runtime_policy.patch_shape_classes.get(patch_id, PatchShapeClass.MIX)
if shape == PatchShapeClass.BAND:
    placement = band_operator(patch_id, graph, runtime_policy, ...)
else:
    placement = generic_frontier_operator(patch_id, graph, runtime_policy, ...)
```

**Mandatory fallback rule:**
If `band_operator()` cannot satisfy any required prerequisite
(missing paired sides, unresolved frame segment, invalid projection stations,
internal consistency failure, etc.), it must immediately fall back to
`generic_frontier_operator(...)`.

BAND classification is an admission ticket, not an obligation to force special handling.

**Fallback protocol:**
- Fallback is **total, not partial**. If band_operator fails at any point,
  discard all its partial results and pass the patch to generic_frontier_operator
  as if it were MIX.
- Log the fallback with patch_id and failure reason (for diagnostics).
- The patch retains its PatchShapeClass.BAND in classification data
  (for debugging), but its runtime placement is produced by the generic path.

**Note for executing agent — dispatch integration point:**
The current solver (`build_quilt_scaffold_chain_frontier`) iterates by **chain
within a quilt**, not by patch. The dispatch point needs careful placement —
either pre-solve per-patch routing or intra-solve patch-level branching.
Study `build_quilt_scaffold_chain_frontier()` control flow before implementing
dispatch. Do not assume a simple per-patch loop exists.

**Note on CornerToken in Phase 1:**
v1 classifier does not read CornerToken fields. CornerToken is built and stored
for future use (Phase 2/3 consumers). The cost is trivial (one float + two refs
per corner). If this feels wasteful during implementation, it can be deferred —
but building it now avoids a second pass over loop data later.

### Task 1.6 — Spine-projection station function

For v1, keep this helper **operator-private** if possible.
Preferred location: `band_operator.py`.
Fallback location: `frontier_place.py` only if reuse is unavoidable.

```python
def _project_chain_stations_onto_segment(
    chain: BoundaryChain,
    node: PatchNode,
    segment: _ResolvedFrameSegment,
    final_scale: float,
) -> list[float]:
    """Project chain 3D vertices onto resolved frame segment axis.
    Returns normalized station values [0.0 .. 1.0] in resolved frame domain.
    """
    ...
```

Implementation:
1. For each `v_3d` in `chain.vert_cos`:
   - Project to patch-local UV: `u = (v_3d - origin).dot(node.basis_u) * final_scale`,
     `v = (v_3d - origin).dot(node.basis_v) * final_scale`
   - Project UV onto segment axis: `t = (v_uv - segment.start_uv).dot(axis_norm) / axis_len`
   - Clamp to [0.0, 1.0]
2. Enforce monotonicity.
3. Return station list.

Both SIDE chains project onto the **same** resolved axis → shared s-domain
without explicit averaging or copied-phase.

### Phase 1 acceptance criteria

- New file `cftuv/structural_tokens.py` exists with minimal token types + classifier.
- Every patch in every quilt has PatchShapeClass assigned before solve.
- BAND patches go through `band_operator()`, not generic frontier.
- MIX patches behave **identically to baseline behavior** (baseline parity, not just passing tests).
- Old band_candidate fields still exist (deprecated, comparison/debug only).
- `band_operator` uses spine-projection stations for SIDE chains.
- `band_operator` has safe fallback to generic operator on failure.
- All existing tests pass.

---

## Phase 2 — Junction Enrichment (design only, do not implement yet)

**Goal:** Enrich `_Junction` with geometric data for future cross-patch decal matching.
Implement only after Phase 1 is stable and validated.

### Design

Add to `cftuv/structural_tokens.py` when needed:

```python
@dataclass(frozen=True)
class JunctionArmView:
    chain_token: ChainToken                         # reference, not copy
    patch_id: int
    direction_from_junction: tuple[float, float, float]
    is_chain_start: bool

@dataclass(frozen=True)
class JunctionView:
    vert_index: int
    vert_co: tuple[float, float, float]
    arms: tuple[JunctionArmView, ...]
    valence: int
    patch_ids: tuple[int, ...]
    arm_angles: tuple[tuple[int, int, float], ...] | None
```

Key design constraint: `JunctionArmView.chain_token` must be the **same object**
as in LoopSignature (identity, not equality). This prevents field duplication.

At this point `ChainToken` gains the `spin` field:

```python
# Added to ChainToken when Phase 2 starts:
    spin: int                           # +1 or -1 for H/V, 0 for FREE
    spin_vector: tuple[float, float]    # normalized 2D direction in patch basis
```

`spin` is deferred to Phase 2 because:
- Shape classifier (Phase 1) does not use it.
- Band operator (Phase 1) does not use it.
- Decal orientation (Phase 3) requires it via JunctionArmView.
- Adding it in Phase 2 alongside junction enrichment is natural.

### Builder

```python
def build_junction_views(
    junctions: Iterable[_Junction],
    loop_signatures: Mapping[int, list[LoopSignature]],
    graph: PatchGraph,
) -> list[JunctionView]:
    ...
```

Builds from existing `_Junction` records. Does not replace them.

---

## Phase 3 — Decal Producer (design only, do not implement yet)

**Goal:** Pattern-action rule system reading tokens + junction views, emitting DecalPlacement[].
Implement only after Phase 2 is stable.

### Vocabulary

```
TRIM_H, TRIM_V      — along SIDE chains
CORNER_90            — L-shaped, 4 orientations via spin
T_JUNCTION           — at valence-3 junctions
CAP_END              — at CAP chain endpoints
```

### Rule types

**Chain-local** (per ChainToken):
- SIDE + H_FRAME → TRIM_H
- SIDE + V_FRAME → TRIM_V
- CAP → CAP_END

**Junction-centric** (per JunctionView, cross-patch):
- valence 2, angle ≈ 90° → CORNER_90 (orientation from spin)
- valence 3, one straight pair + perpendicular → T_JUNCTION
- valence 2, angle ≈ 180° → no corner decal (continuation)

### Cross-patch corner decals

Corner decal at junction spanning 2-3 patches:
- Reads all JunctionView.arms
- Uses arm_angles for shape selection
- Uses ChainToken.spin for orientation
- DecalPlacement.patch_ids lists all affected patches

---

## Phase 4 — Deprecation Cleanup (after Phase 1 validated)

Remove old parallel systems only after PatchShapeClass is validated
on all test meshes and proven equivalent-or-better:

1. Remove `band_candidate`, `band_confirmed_for_runtime`, `straighten_eligible`
   from `_PatchDerivedTopologySummary`.
2. Remove `_patch_allows_straighten_runtime()` from frontier_state.py.
3. Evaluate whether `_FrameRun` / `RunKey` / `_RunStructuralRole` are still needed.
   If spine detection is fully replaced by LoopSignature, run system may be dead code.
4. Simplify junction ref types (`_JunctionRunEndpointRef`, `_JunctionCornerRef`)
   if JunctionView covers their use cases.

Do NOT remove anything preemptively. Only after verified non-usage.

---

## File map

| File | Phase | Action |
|------|-------|--------|
| `cftuv/frontier_state.py` | 0 | Fix type annotation |
| `cftuv/structural_tokens.py` | 1 | **NEW** — tokens + classifier + builder |
| `cftuv/analysis_derived.py` | 1 | Call builders, store in topology |
| `cftuv/analysis.py` | 1 | Export shape classes to solve layer |
| `cftuv/frontier_place.py` | 1 | Only if operator-private helper placement is impossible |
| `cftuv/solve_frontier.py` | 1 | Operator dispatch: BAND → band_operator |
| `cftuv/band_operator.py` | 1 | **NEW** — dedicated narrow band operator |
| `cftuv/solve_planning.py` | 1 | Pass shape classes to runtime policy |
| `cftuv/structural_tokens.py` | 2* | Add JunctionArmView, JunctionView, spin |
| `cftuv/analysis_junctions.py` | 2* | Build JunctionViews |

*Phase 2 files listed for design reference only. Do not implement until Phase 1 stable.

## Dependency graph

```text
Phase 0 ──→ Phase 1 ──→ Phase 2* ──→ Phase 3*
                │                       │
                └──→ Phase 4 ◄──────────┘
                     (after validation)

* = design only, implement later
```

Phase 0 is lightweight prerequisite (one type fix).
Phase 1 is the deliverable. Ship this.
Phase 2/3 are designed so Phase 1 tokens support them without refactoring.
Phase 4 is cleanup after validation.
