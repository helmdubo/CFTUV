# AGENTS.md — Project Context for AI Agents

## Envelope v1 documentation authority

Machine-readable document status: `CURRENT_ONBOARDING_AUTHORITY`.

For every Envelope v1 task, resolve live control metadata before reading
implementation context:

1. read `docs/architecture_status.json` from
   `codex/base-00-canonical-integration`;
2. read `docs/agent_execution/envelope_v1/01_GLOBAL_CANON.md`;
3. read `docs/agent_execution/envelope_v1/02_AGENT_PROTOCOL.md`;
4. read the exact `current_card_path` named by the status manifest;
5. read only accepted dependency handoffs and paths allowlisted by that card.

The immutable implementation base comes from `accepted_integration_sha` in the
live status manifest. An older copy of that manifest embedded in the immutable
commit must not override the canonical control ref.

Current architecture map:

```text
host facts/request
→ Blender-free Envelope compiler/evaluator in kernel/
→ RawCoverage / approved interactions / ownership
→ semantic arrangement and GeometryBatch
→ optional host preview/materialization
```

The existing `cftuv/decals.py` and `cftuv/decal_voronoi.py` paths, including
their `pyvoronoi` backend, are `SCOPED_LEGACY_RUNTIME`. They are context for the
existing decal producer, not implementation authority for the new Envelope
kernel. A kernel agent must not read or modify them unless its exact card
assigns the Legacy Evidence Curator or Host Adapter role.

Start at `docs/envelope_engine_start_here.md` for the human-readable authority
map. Historical session manifests and semantic rebaseline documents retain
evidence value but cannot dispatch work.

---

## What is this project

CFTUV (Constraint-First Trim UV) — Blender addon for semi-procedural UV unwrapping
of architectural hard-surface assets under trim sheet / tile workflows.

Target: AA-AAA game environment art. Blender 4.1+, Python 3.10+.
Single developer, in-house studio tool. Core analysis/solve code uses only
Blender built-ins. The existing decal producer may use its explicitly
documented `pyvoronoi` backend; that scoped legacy runtime dependency is not a
dependency or semantic authority of the new Blender-free Envelope kernel.

**Architectural debt.** This codebase contains `ARCHITECTURAL_DEBT: <ID>`
markers at sites with known suboptimal design. Before substantially modifying
any file containing such a marker, read `docs/architectural_debt.md` and check
whether your change should trigger debt pay-down. If your change repeats the
same pattern in a new place, either consolidate now or install a new marker.

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

## Envelope Kernel Contract

Machine-readable section status: `ACCEPTED_SEMANTIC_CONTRACT`.

This contract applies to the new Blender-free decal envelope kernel. It does
not change the chain-first strongest-frontier rule of the existing UV solve.

Bootstrap every active task through the authority order at the top of this
file. `docs/envelope_backend_semantics.md` remains
`ACCEPTED_SCOPED_SEMANTIC_EVIDENCE`. The earlier roadmap and pivot
instructions are `SUPERSEDED_FOR_ACTIVE_DISPATCH`; they are retained only as
historical context:

1. `docs/decal_envelope_roadmap_compromise.md`
2. `docs/envelope_kernel_pivot_instructions.md`

The accepted scoped semantic reference is:

1. `docs/envelope_backend_semantics.md`

Non-negotiable model:

- `PatchDomain` is the propagation field. A pChain is a front source, never a
  private field or independent solve domain.
- `PhysicalChain` stores physical edge/vertex identity. `ChainUse` stores one
  directed patch-side use with owner Patch, loop, orientation, side, and roles.
- A seam between two Patches produces two ChainUses in different domains.
  `SEAM_SELF` produces two distinct ChainUses in one domain. Do not merge them.
- All active sources of one `DecalRequestId` and one PatchDomain are evaluated
  together. Execution key is `(DecalRequestId, PatchDomainId)`. The kernel
  produces one patch-level single-cover union, then resolves interactions,
  then ownership. Per-pChain materialization followed by post-hoc stitching is
  an architectural regression.
- Corner/Junction remain derived analysis relations, not primary topology or
  solve units. Their compile-static seeds are atomic envelope inputs.
- Case 16 policy is B: same-decal, same-Patch wing contributions clip at the
  mutual equality locus after mutual arrival. Cross-decal and cross-Patch
  collision are forbidden. Self-collision uses the same contract.
- A front may continue, reach a named event, or produce a named failure. Silent
  disappearance is forbidden.
- Envelope v1 uses `BOUNDARY_LIMITED_PROPAGATION`. An ordinary directed
  `ChainUse` has exactly one owner-interior `FrontComponent`. Additional
  sectors/components are legal only when analysis explicitly proves distinct
  Patch sectors; never synthesize default `left/right` sectors for one use.
- Boundary facts distinguish `TOPOLOGICAL_BOUNDARY_USE`,
  `PHYSICAL_DOMAIN_BARRIER`, and `SOURCE_LAUNCH_BOUNDARY`. A launch support
  never blocks its originating seed. Topological seam-cut identity alone is
  not a physical barrier. Contact clips/shrinks a component; it does not stop
  other sources or the opposite use of a physical chain.
- Endpoint contact may slide/shrink on the same boundary component. Interior
  contact that would split an active interval emits `BARRIER_SPLIT_REQUIRED`
  at the exact contact alpha; no ambiguous "slide or split later" state.
- A v1 FrontComponent may not increase its active branch count. If obstacle
  bypass would require split/path choice/merge, emit capacity reason
  `BARRIER_SPLIT_REQUIRED`, preserve `requested_alpha`, clamp only that
  component to proven `effective_alpha`, and report boundary capacity reached.
- First/last front vertex criteria, boundary rollback, material teleportation
  behind an obstacle, and global pChain/Patch stop are forbidden. Full obstacle
  bypass belongs to EC8+.

Required pipeline:

```text
Patch/PhysicalChain/ChainUse facts
→ PatchDomain + sectors/holes/barriers
→ seeds
→ FrontComponents and local contributions grouped by (DecalRequestId, PatchDomainId)
→ boundary-limited resolution
→ patch union
→ intra-Patch interactions
→ ownership
→ SemanticArrangement
→ GeometryBatch
```

Contract ownership is one-way:

```text
AnalysisSnapshotV1 (host facts only)
+ DecalRequestV1 (user/request policy)
→ CompiledPatchEvaluationPlan (kernel-owned seeds/fronts/envelopes/events)
→ GeometryBatch
```

`AnalysisSnapshotV1` must never contain seeds, FrontComponents, envelopes,
active intervals, effective alpha, capacity state, or kernel events.

Cross-Patch collision remains forbidden, but cross-Patch topology coordination
is required: one global `JunctionRelation` may produce per-Patch projections
sharing one semantic anchor and provenance.

Project artifact policy: do not create presentation visuals for requirements,
roadmaps, reviews, handoffs, or acceptance. This includes SVG/PNG semantic
sheets, contact sheets, slide-like diagrams, decks, and interactive HTML
presentations. Prefer prose, code, JSON, schemas, validator code, and validator
output. Screenshots captured from Blender viewport, UV Editor, or Blender debug
overlays are allowed as diagnostic/runtime evidence; they are not semantic
authority by themselves. Existing Grease Pencil/debug visualization remains a
runtime diagnostic facility, not authorization to create presentation assets.

Role separation remains mandatory:

- Kernel implementer does not read `cftuv/decal_voronoi.py` or legacy parts of
  `cftuv/decals.py`.
- Legacy Evidence Curator reads legacy evidence but does not write kernel code.
- Host Adapter Author maps contracts and does not repair geometry.
- These roles run in separate sessions/contexts.

The historical EC1 gate required the canonical coordinate-free JSON corpus and
prose to define PatchDomain, PhysicalChain, ChainUse, request identity, seeds,
patch union, interactions, ownership, UV/station flow, boundary contact
topology, cross-Patch junction coordination, and mixed-alpha shared envelopes.
That gate is accepted evidence, not current dispatch authority.

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
├── solve_skeleton.py      # Post-frontier junction-based skeleton solve + canonical write-back
├── solve_transfer.py   # UV transfer: scaffold → UV layer, conformal fallback
├── solve_diagnostics.py# UV axis metrics, closure seam diagnostics
├── solve_reporting.py  # Regression snapshots, scaffold reports, human-readable output
├── structural_tokens.py# Generic loop/chain structural fingerprints
├── shape_types.py      # Shape enums + loop interpretation contracts
├── shape_classify.py   # Shape policy: BAND/MIX classification, FREE→STRAIGHTEN interpretation
├── band_spine.py      # BAND runtime parametrization: section-based spine + UV targets
├── debug.py            # Grease Pencil visualization + GPENCIL/GREASEPENCIL v3 compatibility
├── decals.py           # scoped legacy decal runtime; not Envelope-kernel authority
├── decal_charts.py     # Immutable IR/input boundary for intrinsic strip charts
├── decal_voronoi.py    # scoped legacy pyvoronoi backend
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

**Raw Chain Role** — early local `frame_role` stored in PatchGraph.
Topology / local-basis fact only. Must not be overwritten by downstream
inheritance or runtime shape logic.

**Effective Structural Role** — downstream structural chain role derived from
raw role plus inherited seam/junction facts in `analysis_derived.py`.
This is the canonical non-local chain truth for reporting, structural runtime
interpretation, and solve-facing analysis output.

**Runtime Placement Role** — solve-context role used by frontier/runtime
placement. Built from effective structural role plus shape-owned runtime
promotions such as BAND `STRAIGHTEN` / CAP authority.

**PatchShapeClass** — shape classification of a patch from structural fingerprints:
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
15. Grease Pencil compatibility is owned by `debug.py` helpers; do NOT hardcode Blender-version-specific GP API (`GPENCIL`/`GREASEPENCIL`, `layer.info/name`, `layer.clear()`, `stroke.line_width`, etc.) outside that layer
16. BAND SIDE chains must BOTH be FREE (H/V chains can never be SIDE in a BAND)
17. STRAIGHTEN is a frontier-level role — shape policy interprets structural fingerprints, frontier places. No separate pre/post pass operator.
18. Do NOT reintroduce a separate BAND operator path — BAND support must stay inside the analysis/frontier shape pipeline
19. Raw `BoundaryChain.frame_role` is an early topology fact. Do NOT overwrite it with inherited or runtime promotions.
20. Frontier / scaffold placement must consume effective structural or runtime placement roles, not raw role alone.
21. Patch shape identity (`PatchShapeClass`) must be derived from raw structural fingerprints, not from downstream runtime placement role.
22. Analyze / debug output must distinguish raw chain role from effective structural role whenever they differ.

---

## Code Conventions

- Comments: Russian (internal studio tool)
- Docstrings: Russian or English
- Naming: `snake_case` functions, `PascalCase` classes, `UPPER_CASE` constants
- Enum values: `UPPER_CASE` strings (`PatchType.WALL`, `FrameRole.H_FRAME`)
- Private functions: prefix `_` (not `__`)
- Types: dataclass for data, regular classes for operators
- Core analysis/solve modules do not add third-party dependencies. Decal-only
  geometry dependencies must be documented, isolated behind their backend,
  and have an explicit named unavailable/unsupported outcome. Geometry
  fallback is prohibited unless separately approved by the user.

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
7. **P7: Structural Token System** — structural fingerprints + shape classifier + STRAIGHTEN role ✓ (Phase 1)
   - `structural_tokens.py`: ChainToken, LoopSignature
   - `shape_types.py` / `shape_classify.py`: `PatchShapeClass`, BAND/MIX policy, FREE → STRAIGHTEN interpretation
   - BAND patches: FREE SIDE chains → STRAIGHTEN, frontier handles natively
   - `band_spine.py`: section-based BAND parametrization for SIDE + CAP runtime placement
   - Future phases: junction enrichment (Phase 2), decal producer (Phase 3)

Current frontier selection is in **Phase 7 structured-rank + layered scoring + structural tokens (STRAIGHTEN)** mode:
`viable → role → ingress → patch_fit → anchor → closure_risk → local_score → tie_length`.
Scalar `score` is still kept as threshold gate and local refinement. STRAIGHTEN chains
from BAND patches get tier 2 (`straighten_band_side`) in role scoring — between native
H/V (tier 3–5) and FREE (tier 0). At placement time, STRAIGHTEN resolves to H/V via
geometry (dominant axis of chain start→end vector projected onto patch basis).
Shape support (`structural_tokens.py` + `shape_classify.py` + `analysis_shape_support.py`) classifies patches before solve:
`PatchShapeClass.BAND` → FREE SIDE chains become STRAIGHTEN → frontier treats them as
strong chains with authority resolution (axis, span, station, parameter).
Toggle-gated: `straighten_chain_refs` only passed to frontier when straighten is ON.
When straighten is ON, BAND patches may also carry `band_spine_data`: pre-computed
section-based UV targets for both SIDEs and both CAPs, consumed directly by frontier
placement and pin policy.
Current BAND support is intentionally no longer patch-neighbor dependent:
- 4-chain BORDER loops may classify as BAND.
- Geometric closed-loop split for isolated OUTER border loops must survive through final loop topology.
- `band_spine_data` is sufficient to enable runtime straighten authorities even when legacy `band_mode` summary is absent.
- Future `CABLE` / `CYLINDER` work must extend this shape-support path rather than reintroducing neighbor-only admission rules.

Role-layer contract:
- `raw_chain_role` lives in PatchGraph and remains the identity input for shape classification.
- `effective_structural_role` is produced in `analysis_derived.py` from raw role + inherited seam/junction context.
- `runtime_placement_role` is resolved in frontier/runtime from effective structural role + shape-owned runtime promotions.
- Frontier, scaffold build, and chain placement authority must use effective/runtime roles.
- Shape identity must not depend on runtime role, otherwise BAND/CABLE/CYLINDER admission becomes cyclic.
- Reporting/debug should show effective structural role as primary and raw role as secondary when they diverge.
`Solve Phase 1 Preview` clears final UV pins by default; Add-on Preferences may keep
them for debug inspection. `Transfer Only` keeps pins.
The former Phase 8 alignment/drift roadmap is absent from this baseline and is
not current authority. Any new alignment slice must be selected by the live
status manifest and its exact card.

---

## Scoring Refactor Contract

The former standalone score-refactor plan is absent from this baseline. The
contract below is retained project context; before changing frontier scoring,
patch context, seam scoring or telemetry, require an exact current task and
read the relevant sections of `docs/cftuv_architecture.md` and
`docs/cftuv_reference.md`.

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
Compatibility target includes legacy Blender 4.1 GPENCIL and modern Blender 4.5.x GREASEPENCIL v3.

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

Alignment/drift work has no standalone roadmap file in this baseline. Do not
infer such work from historical references in companion docs; follow the live
status and exact assigned card.

For most tasks, this file alone is sufficient.
