# AGENTS.md — Project Context for AI Agents

## What is this project

CFTUV (Constraint-First Trim UV) — Blender addon for semi-procedural UV unwrapping
of architectural hard-surface assets under trim sheet / tile workflows.

Target: AA-AAA game environment art. Blender 4.1+, Python 3.10+.
Single developer, in-house studio tool. No third-party deps beyond Blender built-in.

---

## Core Principle

**Chain-first strongest-frontier.**

Scaffold builds chain by chain, picking the strongest available chain from a
global frontier pool across the entire quilt. The pool does not distinguish
"chain of the same patch" from "chain of another patch across a seam" — they
are equal candidates.

Quilt grows organically from root chain outward, crossing seam boundaries,
until frontier is exhausted or falls below threshold. Everything frontier
didn't place becomes free vertices for final Conformal unwrap.

This principle must be preserved in any change. Drift toward patch-first,
loop-sequential, or corner-based placement is an architectural regression.

---

## Module Layout

```text
cftuv/
├── __init__.py
├── constants.py        # Thresholds, sentinels, scoring weights
├── model.py            # Enums, topology IR (PatchGraph), solve IR (ScaffoldMap)
├── analysis.py         # BMesh → PatchGraph (facade over analysis_* submodules)
├── analysis_*.py       # submodules: topology, boundary, corners, classification, etc.
├── solve.py            # Planning, frontier builder, UV transfer, validation (facade)
├── solve_records.py    # Pure data types for solve layer (PinPolicy, FrontierRank, PatchPinMap, etc.)
├── solve_planning.py   # Quilt planning, SolveView, attachment candidates
├── solve_frontier.py   # Chain-first frontier builder, scaffold assembly
├── solve_pin_policy.py # Pin policy: PatchPinMap, build_patch_pin_map, preview_chain_pin_decision
├── solve_instrumentation.py # Frontier telemetry: FrontierTelemetryCollector, QuiltFrontierTelemetry
├── solve_transfer.py   # UV transfer: scaffold → UV layer, conformal fallback
├── solve_diagnostics.py# UV axis metrics, closure seam diagnostics
├── solve_reporting.py  # Regression snapshots, scaffold reports, human-readable output
├── structural_tokens.py# Shape classifier: ChainToken, LoopSignature, PatchShapeClass
├── band_spine.py      # BAND spine pre-parametrization: midpoint spine + 4-chain UV targets
├── band_operator.py    # (legacy utility) Spine projection helpers, not imported
├── debug.py            # Grease Pencil visualization
├── operators.py        # Blender UI wrappers (max 5 lines math)
└── console_debug.py    # Verbose console toggle
```

Data flow: `analysis.py → solve.py → debug.py`, orchestrated by `operators.py`.

Central IR: **PatchGraph** (topology facts, indices only, no BMFace/BMEdge refs).
Solve IR: **ScaffoldMap** (persistent 2D placement result, can be cached/edited).

---

## Glossary

**Patch** — connected face group after flood fill by seam. Minimum topology unit.
Types: `WALL`, `FLOOR`, `SLOPE` (by normal angle to WORLD_UP).

**BoundaryLoop** — closed boundary contour of one patch. Types: `OUTER` (one per
patch) or `HOLE` (internal openings).

**BoundaryChain** — continuous segment of boundary loop with one neighbor.
Split point = vertex where neighbor changes.
**Chain is the primary placement unit in solve.**

**BoundaryCorner** — vertex at junction of two chains within one loop.
Corner has no own position — it emerges from chain placement.
Also stores local wedge-orientation facts (`wedge_face_indices`, `wedge_normal`)
derived in analysis from owner-patch faces around the corner. These are runtime
inputs for local turn-sign decisions; do not reconstruct them in solve.

**FrameRole** — chain alignment in local patch basis: `H_FRAME` (horizontal),
`V_FRAME` (vertical), `STRAIGHTEN` (strong but axis-flexible, resolved to H/V
at placement time), `FREE` (diagonal/undefined).

**PatchShapeClass** — shape classification of a patch from structural tokens:
`MIX` (default), `BAND` (rectangular strip with two parallel FREE sides and
two similar-length caps). Determines whether SIDE chains receive STRAIGHTEN role.

**ChainRoleClass** — structural role of a chain within its loop: `SIDE` (parallel
pair in BAND), `CAP` (connecting pair), `BORDER` (mesh boundary), `FREE` (no pair).

**DihedralConvexity** — geometric property of a PATCH-neighbor chain.
-1.0 = concave (inner corner), +1.0 = convex (outer corner), 0.0 = neutral.
Computed in analysis post-pass from patch normals and chain chord direction.
Used by closure cut heuristic to prefer cutting at inner corners.

**ChainNeighborKind** — `PATCH` (another patch), `MESH_BORDER` (mesh edge),
`SEAM_SELF` (seam within same patch).

**Quilt** — independent group of patches built from one root by solve.

**ScaffoldMap** — persistent 2D placement result. Not a process.

**FrameRun** — analysis-only derived view over neighboring chains. Diagnostic, not
a solve unit. Never participates in placement.

**Junction** — global derived view at mesh vertex where corners from different
patches meet. Diagnostic/research entity, not solve runtime.

---

## Invariants — violation = bug

1. `model.py` does NOT import `bpy`, `bmesh` (only `mathutils`)
2. `analysis.py` does NOT write UV (except `_classify_loops_outer_hole` — marked)
3. `solve.py` does NOT flood fill and does NOT classify patches
4. `debug.py` does NOT read BMesh directly
5. Operators contain NO geometry logic (max 5 lines math)
6. No global mutable variables — UVSettings passed as parameter
7. PatchGraph stores indices (int), NOT BMFace/BMEdge references
8. Sharp does NOT participate in patch split — only Seam
9. Scaffold grows chain-first strongest-frontier, NOT patch-by-patch
10. HOLE loops do NOT participate in scaffold placement pool
11. Frontier candidate cache (`_cached_evals`) must produce bit-identical output to full scan — if output differs, dirty marking is incomplete (bug)
12. Pin decisions live ONLY in `solve_pin_policy.py` — `PatchPinMap` is the single source of truth; do NOT inline pin logic in transfer or frontier
13. `dihedral_convexity` is a contextual derived field — computed AFTER full PatchGraph assembly, never during chain build
14. Analyze debug geometry must be generated independently of layer visibility toggles; panel / eye toggles only control GP layer visibility, not whether patch data is built
15. BAND SIDE chains must BOTH be FREE (H/V chains can never be SIDE in a BAND)
16. STRAIGHTEN is a frontier-level role — structural tokens classify, frontier places. No separate pre/post pass operator.
17. `band_operator.py` is NOT imported — kept only as utility reference for spine projection

---

## Code Conventions

- Comments: Russian (internal studio tool)
- Docstrings: Russian or English
- Naming: `snake_case` functions, `PascalCase` classes, `UPPER_CASE` constants
- Enum values: `UPPER_CASE` strings (`PatchType.WALL`, `FrameRole.H_FRAME`)
- Private functions: prefix `_` (not `__`)
- Types: dataclass for data, regular classes for operators
- No third-party dependencies beyond Blender built-in

---

## What NOT To Do

- Do NOT use `Hotspot_UV_v2_5_26.py` — dead legacy monolith
- Do NOT return to patch-first or loop-sequential placement
- Do NOT add multi-axis semantic profiles (confidence scores beyond ChainRoleClass)
- Do NOT create a separate Boundary Graph — chains are substructure of PatchNode
- Do NOT create constraint classes — stitching rules are code in solve
- Do NOT use globals for settings — pass UVSettings as parameter
- Do NOT store BMFace/BMEdge refs in model — indices only
- Do NOT return local per-chain rectification for H/V — already caused regression
- Do NOT do quilt-wide snap without diagnostics data first

---

## Current State and Priorities

Active chain-first frontier path works. HOLE solve drift fixed. Same-type quilt
separation active. Ring/cylinder cycle bug closed via tree-edge-only sewing.

Current priority order (agreed):

1. **P1: Decompose solve.py** into sibling modules ✓
2. **P2: Rewrite AGENTS.md** as self-contained entry point ✓
3. **P3: Rescue/scoring instrumentation** ✓
4. **P4: Minimal trim abstraction** in model.py
5. **P5: Scoring revision** based on instrumentation data ✓
6. **P6: Pin policy extraction** into explicit layer ✓
7. **P7: Structural Token System** — shape classifier + STRAIGHTEN role ✓ (Phase 1)
   - `structural_tokens.py`: ChainToken, LoopSignature, PatchShapeClass
   - BAND patches: FREE SIDE chains → STRAIGHTEN, frontier handles natively
   - `band_spine.py`: midpoint-spine parametrization for SIDE + CAP runtime placement
   - Future phases: junction enrichment (Phase 2), decal producer (Phase 3)

Current frontier selection is in **Phase 7 structured-rank + layered scoring + structural tokens (STRAIGHTEN)** mode:
`viable → role → ingress → patch_fit → anchor → closure_risk → local_score → tie_length`.
Scalar `score` is still kept as threshold gate and local refinement. STRAIGHTEN chains
from BAND patches get tier 2 (`straighten_band_side`) in role scoring — between native
H/V (tier 3–5) and FREE (tier 0). At placement time, STRAIGHTEN resolves to H/V via
geometry (dominant axis of chain start→end vector projected onto patch basis).
Structural tokens (`structural_tokens.py`) classify patches before solve:
`PatchShapeClass.BAND` → FREE SIDE chains become STRAIGHTEN → frontier treats them as
strong chains with authority resolution (axis, span, station, parameter).
Toggle-gated: `straighten_chain_refs` only passed to frontier when straighten is ON.
When straighten is ON, BAND patches may also carry `band_spine_data`: pre-computed
midpoint-spine UV targets for both SIDEs and both CAPs, consumed directly by frontier
placement and pin policy.
Phase 8 alignment / drift work is a separate roadmap in `docs/cftuv_alignment_drift_roadmap.md`.

---

## Scoring Refactor Contract

Current score refactor plan lives in **`docs/cftuv_score_refactor_plan.md`**. Read it before changing frontier scoring, patch context, seam scoring, or related telemetry.

Mandatory rules for all agents working on scoring:

1. Preserve **chain-first strongest-frontier**.
2. **Patch** and **Chain** remain the only primary solve units.
3. **Corner** may contribute local chain features, but must NOT become an independent placement unit.
4. **Junction** remains a derived/global entity and must NOT become a primary runtime scoring entity in this phase.
5. **Row / Column** drift handling belongs to a separate alignment layer, not to main frontier score.
6. Refactor direction is:
   - scalar score → structured frontier rank
   - structured frontier rank → explicit patch scoring context
   - then corner / shape / seam enrichment
   - then rescue-gap telemetry
7. Do NOT absorb rescue paths into main frontier early.
8. Do NOT mix future manual operations into the current score refactor.

Short handoff paragraph:

> Preserve chain-first strongest-frontier. Patch and Chain remain the only primary solve units. Corner may contribute local chain features, but must not become an independent placement unit. Junction remains derived/global and must not become a primary runtime scoring entity in this phase. Row/Column drift handling belongs to a separate alignment layer, not to the main frontier score. Refactor path is: scalar score → structured frontier rank → explicit patch scoring context → corner/shape/seam enrichment → rescue-gap telemetry. Manual operations are a future layer on top of patch/chain context and are out of scope for the current score refactor.

---

## Testing Approach

No formal tests. Verification through:
1. Debug visualization (Grease Pencil) — must work after every change
2. Console output (patch stats, chain info, frame roles)
3. Validation layer (scaffold vs UV mismatch report)
4. Manual UV inspection on production meshes
5. Regression snapshots (`Save Regression Snapshot` button)

If debug visualization breaks — the change is wrong.

---

## When To Read Companion Docs

**`docs/cftuv_architecture.md`** — read when your task requires understanding:
- how the pipeline works end-to-end
- IR layer design (PatchGraph vs ScaffoldMap)
- entity model (primary vs composite vs derived)
- two different connectivities (topology vs solve)
- current architectural debt

**`docs/cftuv_reference.md`** — look up specific sections when you need:
- full topology invariant tables (30+ rules with status)
- runtime heuristics and thresholds
- regression checklist and mesh set
- scoring weight documentation

**`docs/cftuv_alignment_drift_roadmap.md`** — read when your task touches:
- row / column scatter
- closure drift after scaffold build
- future alignment pass design
- pre-transfer / post-frontier correction ideas

For most tasks, this file alone is sufficient.
