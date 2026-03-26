# CFTUV Architecture
## Current implementation state and design decisions

---

## Purpose

This is the detailed architecture document. Read `AGENTS.md` first.
Open this when your task requires understanding pipeline internals,
IR design, entity model, or current architectural debt.

---

## Module Responsibilities

| Module | Reads | Writes | Role |
|--------|-------|--------|------|
| `model.py` | — | — | Enums, topology IR, solve IR, UVSettings |
| `constants.py` | — | — | Thresholds, sentinels, scoring weights |
| `analysis.py` | BMesh | PatchGraph | Patch split, basis, loops, chains, corners, seams |
| `solve_records.py` | — | — | Pure data types: PinPolicy, PatchPinMap, frontier records |
| `solve_planning.py` | PatchGraph | SolveView, SolvePlan | Quilt planning, attachment candidates |
| `solve_frontier.py` | PatchGraph, SolvePlan | ScaffoldMap | Chain-first frontier builder |
| `solve_pin_policy.py` | PatchGraph, ScaffoldPatchPlacement | PatchPinMap | Pin decisions — single source of truth |
| `solve_instrumentation.py` | solve_records | QuiltFrontierTelemetry | Frontier telemetry collector and formatting |
| `solve_transfer.py` | ScaffoldMap, BMesh | UV layer | Scaffold → UV, conformal fallback |
| `solve_diagnostics.py` | ScaffoldMap, BMesh UV | reports | Closure/alignment diagnostics |
| `solve_reporting.py` | ScaffoldMap, PatchPinMap | text | Snapshots, human-readable reports |
| `solve.py` | all above | — | Facade — orchestrates solve pipeline |
| `debug.py` | PatchGraph | Grease Pencil | Visualization |
| `operators.py` | all | — | Blender UI, orchestration, preflight |

Legacy monolith `Hotspot_UV_v2_5_xx.py` is dead. Do not use.

---

## IR Layers

### 1. Topology IR: PatchGraph

Lives in `model.py`. Central inter-module contract.

Stores topology-only mesh representation:
- `PatchNode` — patch geometry, basis, classification
- `BoundaryLoop` — closed boundary contour (OUTER or HOLE)
- `BoundaryChain` — continuous boundary segment with one neighbor
- `BoundaryCorner` — junction between two chains
- `SeamEdge` — shared seam relation between patches

Key properties:
- Stores indices, not BMFace/BMEdge references
- Describes patch topology, not solve decisions
- Contains both OUTER and HOLE loops
- Stores seam adjacency at patch level

### 2. Persistent Solve IR: ScaffoldMap

Also lives in `model.py`. Working runtime IR:
- `ScaffoldPointKey` — unique source point reference
- `ScaffoldChainPlacement` — placed chain with UV coordinates
- `ScaffoldPatchPlacement` — per-patch envelope with status/diagnostics
- `ScaffoldQuiltPlacement` — per-quilt collection with build order
- `ScaffoldMap` — root container of all quilts

### 3. Transient Planning IR

Lives in `solve.py` (target: `solve_planning.py`):
- `PatchCertainty` — per-patch solvability + root score
- `AttachmentCandidate` — scored seam relation between two patches
- `SolverGraph` — all candidates + components
- `QuiltPlan` / `SolvePlan` — ordered solve plan

---

## Entity Model

### Primary Topology Entities

**Patch** and **Chain** — the two fundamental units.

Patch properties by layer:
- Intrinsic: `face_indices`, `normal`, `area`, `perimeter`, `basis`
- Contextual: `patch_type`, `world_facing`
- Derived: quilt membership, solve diagnostics

Chain properties by layer:
- Intrinsic: `vert_indices`, `vert_cos`, `edge_indices`, `is_closed`
- Contextual: `neighbor_kind`, `neighbor_patch_id`, `frame_role`
- Derived: scaffold placement, anchor provenance

### Composite Topology Entities

**BoundaryLoop** — ordered container over chains. Needed for boundary traversal
order, OUTER/HOLE classification, corner ordering.

**BoundaryCorner** — intra-patch junction record. Fixes vertex identity,
chain stitch point, turn angle. Not a solve unit.

### Analysis-Derived Views

**FrameRun** — local-derived continuity view over neighboring chains of one
loop. Diagnostic only. Does not change topology. Does not participate in
placement. Answers: "Do these adjacent chains behave as one logical side?"

**Junction** — global-derived view at mesh vertex, aggregating corners from
different patches. Diagnostic/research entity. Not in solve runtime yet.

### Solve Entities

**Quilt** and **ScaffoldMap** — solve layer. Scaffold is built only from chains.
FrameRun and Junction do not participate.

---

## Two Different Connectivities

This is critically important.

**Topology connectivity** — `PatchGraph.edges`, `PatchGraph.connected_components()`.
Answers: "Which patches are seam-connected at all?"

**Solve connectivity** — valid `AttachmentCandidate`, `SolverGraph.solve_components`,
tree edges from `QuiltPlan`.
Answers: "Which seam relations are allowed for solve propagation?"

These do NOT coincide:
- Patch may be topology-connected only through HOLE but solve-ineligible
- Patch may be in one topology component but non-tree seam remains UV cut
- Ring/cylinder topology does NOT mean quilt may close as UV cycle

---

## Pipeline

```
operators.py
  → validate_solver_input_mesh()
  → build_patch_graph()           # analysis.py
  → build_solver_graph()          # solve.py: scoring, components
  → plan_solve_phase1()           # solve.py: quilt plans, closure cuts
  → build_root_scaffold_map()     # solve.py: chain frontier builder
  → transfer scaffold to UV       # solve.py: pin + write UV loops
  → bpy.ops.uv.unwrap(CONFORMAL)  # Blender: relax with pinned scaffold
```

### Analysis Pipeline (analysis.py)

1. Flood fill faces into patches by seam
2. Classify patch as WALL / FLOOR / SLOPE
3. Build local basis (basis_u, basis_v)
4. Build explicit patch assembly state
5. Trace raw boundary loops
6. Classify loops as OUTER / HOLE via temporary UV unwrap
7. Validate raw patch boundary topology
8. Split loop into raw chains by neighbor change
9. Geometric outer fallback split for isolated OUTER loops (≥4 corners)
10. Split border chains by geometric corners
11. Build BoundaryChain objects
12. Downgrade weaker same-role point-contact chains to FREE
13. Merge adjacent same-role MESH_BORDER chains
14. Build corners and endpoint topology
15. Build seam edges between patches
16. Validate patch-neighbor chains against seam graph

Important: `_classify_loops_outer_hole()` is the only analysis step that
temporarily mutates UV state. This is intentional for wrapped/cylindrical
geometry and must remain isolated.

### Frontier Builder Rules

1. Pool contains only OUTER chains. HOLE loops excluded.
2. Seed chain chosen within root patch, quilt-context only.
3. Anchor provenance: `same_patch` (via corner) or `cross_patch` (via shared
   seam vertex on tree edge).
4. Dual-anchor closure via two cross_patch anchors forbidden (prevents patch wrap).
5. Non-tree seams are intentional UV cuts, not re-sewn.
6. When frontier stalls, only narrow recovery in this order:
   - `tree_ingress`: one ingress chain into untouched tree-child patch
   - `closure_follow`: same-role non-tree closure partner if its pair already placed

### UV Transfer and Pin Policy

1. Conformal all-FREE patch => pin nothing.
2. Connected H/V => pin all.
3. Isolated H/V => pin nothing.
4. FREE => only endpoints that touch connected local H/V.
5. Unsupported patches get individual fallback Conformal.

### Reporting / Telemetry Contract

Reporting is now policy-driven and must not be treated as raw runtime trace dump.

- Stable addresses are mandatory in reports: `ADDR Q#/P#`, `ADDR Q#/P#/L#/C#`, `ADDR Q#/S#`
- Default reports are anomaly-first and compact; healthy routine entities are compressed
- `summary` / `diagnostic` / `forensic` are presentation modes only, never solve modes
- Live telemetry trace is separate from post-hoc report detail
- Suspicious H/V chains must retain directional diagnostics: role, start/end UV, delta, axis error, inherited flag, anchor kinds, status code
- Stall lifecycle is explicit: `open` / `close`; terminal exhausted stop is not the same as actionable unresolved stall
- Regression snapshot uses telemetry summary, not step-by-step replay
- `quilt_index` is a stable quilt id from planning and may contain gaps after orphan-quilt merge; report formatter must preserve it verbatim

---

## Current Architectural Debt

### 1. solve.py is a 6500-line monolith

Contains planning, frontier, transfer, diagnostics, reporting, and ~60 dataclasses.
**P1 (decompose) is the blocking priority.** See `cftuv_solve_decomposition_plan.md`.

### 2. No trim sheet abstraction

CFTUV does constrained UV layout but doesn't know about trim atlas positions.
H_FRAME chain gets UV direction but not target atlas row.
**P4 addresses this with minimal data layer in model.py.**

### 3. Two rescue paths are a symptom

`tree_ingress`, `closure_follow` exist because main frontier
scoring doesn't cover all valid production cases.
**P3 collects instrumentation data; P5 revises scoring based on evidence.**

### 4. Pin policy — RESOLVED (P6)

`PatchPinMap` is now the single source of truth for pin decisions.
`solve_pin_policy.py` owns `build_patch_pin_map()` and `preview_chain_pin_decision()`.
`solve_transfer.py` reads the map; frontier can preview decisions before commit.
Circular dependency reduced: scoring can now query pin impact via `preview_chain_pin_decision`.

### 5. Temporary UV side-effect in analysis

`_classify_loops_outer_hole()` uses temporary UV unwrap for OUTER/HOLE
classification of multi-loop patches. Intentionally retained for
wrapped/cylindrical geometry. Must remain localized.

### 6. No closure/row-column correction pass

Tree-only quilt solve is correct but can accumulate:
- Span drift on non-tree closure seams
- Cross-axis scatter in geometrically collinear H/V chains

These are two related but distinct problems, both needing diagnostics-first
approach. Not solved by returning to UV cycle sewing.

---

## Priorities (Agreed)

| # | Task | Status | Blocks |
|---|------|--------|--------|
| P1 | Decompose solve.py → sibling modules | ✓ Done | Everything |
| P2 | AGENTS.md self-contained rewrite | ✓ Done | Agent onboarding |
| P3 | Rescue/scoring instrumentation | ✓ Done | P5 |
| P4 | Minimal trim abstraction in model.py | Pending | Future solve direction |
| P5 | Scoring revision based on data | ✓ Done | — |
| P6 | Pin policy extraction | ✓ Done | — |

Not doing now: lattice alignment pass, automated test framework,
UV classification refactor, strategy pattern for placement.

---

## Entry Points For New Agents

1. Read `AGENTS.md` (mandatory)
2. Read this file (if task needs pipeline/IR understanding)
3. Look at code:
   - `analysis.py → build_patch_graph()`
   - `solve.py → build_solver_graph()`
   - `solve.py → plan_solve_phase1()`
   - `solve.py → build_root_scaffold_map()`
   - `solve.py → execute_phase1_preview()`
   - `operators.py → _prepare_patch_graph()`
4. If decomposition done, look at `solve_frontier.py` instead of `solve.py`
