# AGENTS.md — Project Context for AI Agents

## What is this project

CFTUV (Constraint-First Trim UV) — Blender addon for semi-procedural UV unwrapping
of architectural hard-surface assets under trim sheet / tile workflows.

Target: AA-AAA game environment art. Blender 3.0+, Python 3.10+.
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
├── analysis_*.py       # 9 submodules: topology, boundary, corners, classification, etc.
├── solve.py            # Planning, frontier builder, UV transfer, validation (facade)
├── solve_*.py          # Target: 6 submodules after P1 decomposition
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

**FrameRole** — chain alignment in local patch basis: `H_FRAME` (horizontal),
`V_FRAME` (vertical), `FREE` (diagonal/undefined).

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
- Do NOT add multi-axis semantic profiles (confidence scores, role_class)
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

1. **P1: Decompose solve.py** into sibling modules (see `cftuv_solve_decomposition_plan.md`)
2. **P2: Rewrite AGENTS.md** as self-contained entry point (this file)
3. **P3: Rescue/scoring instrumentation** — collect data before changing logic
4. **P4: Minimal trim abstraction** in model.py
5. **P5: Scoring revision** based on instrumentation data
6. **P6: Pin policy extraction** into explicit layer

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

For most tasks, this file alone is sufficient.
