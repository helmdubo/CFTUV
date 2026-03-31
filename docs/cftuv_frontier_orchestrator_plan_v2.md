# CFTUV Frontier Refactor — Orchestrator Plan v2

Status: active orchestration plan  
Owner: orchestrator  
Primary scope: `cftuv/solve_frontier.py` and new sibling frontier modules under `cftuv/`

---

## Purpose

Turn `solve_frontier.py` from a large mixed-responsibility module into a thin orchestrator/facade, while preserving the current chain-first strongest-frontier behavior.

This document is written for AI agents. Each agent takes exactly one phase, updates this file, and leaves a short handoff note.

This v2 plan explicitly incorporates review feedback on:

- extraction order
- import DAG
- `FrontierRuntimePolicy` ownership
- closure-pair ownership
- dead-code candidate list
- `solve_records.py` classification
- telemetry coupling rules

---

## Hard invariants

The following must remain true after every phase:

- Preserve **chain-first strongest-frontier**. Do not drift into patch-first, loop-sequential, or corner-first placement.
- Patch and Chain remain the only primary solve units.
- Do not inline pin logic into frontier/transfer.
- Do not move geometry logic into operators.
- Do not change public API unless the phase explicitly allows it.
- If a phase is marked **mechanical**, output must remain behaviorally identical.
- Frontier cache / dirty-marking behavior must remain bit-identical where refactor is declared mechanical.
- `FrontierTelemetryCollector` remains owned by `solve_instrumentation.py` and is passed through parameters; do not re-home telemetry ownership during this refactor.
- `solve_records.py` is not to be physically split in this refactor unless a later dedicated plan says so. In this plan it is **classified**, not migrated.

Authoritative project context: `AGENTS.md`.

---

## Fixed ownership decisions before any extraction

These decisions are locked before P0 starts.

### 1. `FrontierRuntimePolicy` is not a bulk-move candidate

It will be **disassembled by responsibility**, not moved whole.

Planned split:

- mutable state fields / cache fields / registration state → `frontier_state.py`
- candidate evaluation logic → `frontier_eval.py`
- scoring-derived caches / shape-profile helpers → `frontier_score.py`
- stop-diagnostic ownership decided in P0 contract and then extracted by responsibility

Operational rule:

- during structural refactor, keep the class name if possible for compatibility
- but convert heavy methods into standalone functions where that reduces coupling
- thin delegator methods are allowed temporarily

### 2. Closure-pair matching gets explicit ownership

Closure-pair logic is shared across frontier bootstrap, evaluation context, rescue, and finalization.

Therefore introduce explicit shared utility ownership:

- `cftuv/frontier_closure.py` — closure-pair matching and closure-pair maps

Do not leave closure-pair logic half in orchestrator and half elsewhere after P0 contract is established.

### 3. `solve_records.py` is classified, not moved

P0 must classify records into:

- frontier-state-owned
- frontier-eval-owned
- frontier-score-owned
- closure-owned
- rescue-owned
- finalize-owned
- genuinely shared solve-layer records

But P0 must **not** physically move records yet.

### 4. Telemetry coupling remains parameter-based

`FrontierTelemetryCollector` stays in `solve_instrumentation.py`.

Frontier modules may accept the collector as an argument, but they must not absorb telemetry ownership.

---

## Expected import DAG

This DAG is part of the contract. Agents must preserve its direction.

```text
solve_frontier.py (orchestrator)
  ├── frontier_state.py
  │     └── no frontier_* imports
  │
  ├── frontier_place.py
  │     └── imports only model.py / solve_records.py / stdlib (no frontier_state dependency unless type-only later)
  │
  ├── frontier_score.py
  │     └── imports only model.py / solve_records.py / stdlib
  │        (if state types are needed later, use TYPE_CHECKING-only imports where possible)
  │
  ├── frontier_closure.py
  │     └── imports model.py / solve_records.py / solve_planning.py tree helpers only
  │        (closure-pair matching/maps only; no closure-preconstraint ownership)
  │
  ├── frontier_eval.py
  │     └── imports frontier_state.py, frontier_place.py, frontier_score.py, frontier_closure.py
  │        and may bridge to solve_diagnostics.py for closure-preconstraint metrics
  │
  ├── frontier_rescue.py
  │     └── imports frontier_state.py, frontier_eval.py, frontier_place.py, frontier_closure.py
  │
  └── frontier_finalize.py
        └── imports frontier_state.py, frontier_closure.py
           and may bridge to solve_diagnostics.py / solve_pin_policy.py where already required
```

Operational reading of the DAG:

- **state is the lowest frontier layer**
- **eval sits above state/place/score/closure**
- **score must not depend on eval**
- **rescue depends on already-stable state/eval/place/closure layers**
- **finalize depends on state + closure context, not the other way around**

---

## Global execution rules for every agent

1. One agent = one phase = one PR.
2. No opportunistic rewrites outside the phase scope.
3. If you notice adjacent debt, record it in the handoff note, but do not fix it unless the phase explicitly allows it.
4. If a phase says **mechanical extraction only**, do not change formulas, ranking order, threshold logic, rescue flow, cache invalidation order, or telemetry payload shape.
5. If imports become awkward, stop and update the contract instead of improvising a cycle.
6. After finishing a phase, update the checkbox in this file and append one entry to the execution log.

---

## Required handoff format for every phase

Append to the execution log:

- Date
- Phase ID
- Files touched
- What moved / changed
- What was deliberately not touched
- Risks remaining

Also include a short handoff paragraph in the PR body or summary.

---

## Phase board

- [x] P0 — Contract + DAG + ownership classification
- [x] P1 — Debt triage / dead code isolation
- [x] P2 — Extract placement subsystem
- [x] P3 — Extract finalize subsystem
- [x] P4 — Extract rescue subsystem
- [x] P5 — Extract runtime state / cache substrate
- [x] P6 — Extract evaluation / anchors subsystem
- [x] P7 — Extract scoring subsystem
- [x] P8 — Compress `solve_frontier.py` into orchestration shell
- [x] P9 — Post-refactor perf research plan (design only, no behavior change in mainline)

---

# P0 — Contract + DAG + ownership classification

Status: completed

P0 result:

- authoritative split contract added at `docs/cftuv_frontier_split_contract.md`
- DAG amended before code motion:
  - `frontier_eval.py` may bridge to `solve_diagnostics.py` for closure-preconstraint metrics
  - `frontier_finalize.py` may bridge to `solve_pin_policy.py` for scaffold connectivity
- runtime logic intentionally untouched in this phase

## Goal

Produce the authoritative split contract before code motion begins.

## Allowed files

- `docs/cftuv_frontier_orchestrator_plan_v2.md`
- optional: `docs/cftuv_frontier_split_contract.md`

## Tasks

- Map every significant function/class in `solve_frontier.py` to its future target module.
- Map closure-pair helpers into `frontier_closure.py`.
- Classify every `FrontierRuntimePolicy` field/method by destination ownership.
- Explicitly state which methods become standalone functions versus thin delegators.
- Classify frontier-related dataclasses in `solve_records.py` by owning subsystem.
- Mark risky zones:
  - runtime cache / dirty propagation
  - rescue flows
  - anchor resolution
  - scoring/rank construction
  - finalization
  - dead or disabled code branches
- Mark each future move as either:
  - `mechanical`
  - `mechanical but risky`
  - `behavioral / later`
- Confirm the import DAG or amend this plan before code moves start.

## Done when

- Another agent can start extraction without guessing ownership boundaries.
- Import direction is explicit.
- `FrontierRuntimePolicy` decomposition is explicit.
- `solve_records.py` ownership classification exists.

---

# P1 — Debt triage / dead code isolation

Status: completed

P1 result:

- removed the unused legacy scorer `_cf_score_candidate_legacy`
- collapsed `_cf_preview_frame_dual_anchor_rectification()` to its active pass-through behavior and deleted the unreachable body below the first `return`
- removed disabled unused planning helper `_merge_orphan_quilts_into_wall_quilts`
- scoring formulas, placement behavior, rescue order, cache behavior, and public API intentionally unchanged

## Goal

Reduce frontier cognitive noise before extraction.

## Allowed files

- `cftuv/solve_frontier.py`
- `cftuv/solve_planning.py` **only** for the explicitly listed dead helper below
- optional archival note under `docs/`

## Required candidate list

The agent must inspect these first:

- `_cf_score_candidate_legacy`
- `_cf_preview_frame_dual_anchor_rectification` — the body below the first early `return`
- `_merge_orphan_quilts_into_wall_quilts` in `solve_planning.py` if still present as disabled legacy helper

## Tasks

- Identify and isolate dead / disabled / legacy branches.
- Collapse obviously unreachable code paths.
- Keep comments clear about intentionally disabled behavior.
- Do **not** reactivate old logic.

## Must not change

- scoring formulas
- placement behavior
- rescue order
- cache behavior
- public API

## Done when

- `solve_frontier.py` contains less fossilized code and fewer misleading branches.
- explicitly listed dead candidates are resolved or archived
- runtime output remains unchanged.

---

# P2 — Extract placement subsystem

Status: completed

P2 result:

- created `cftuv/frontier_place.py` and moved placement math / direction inference helpers out of `solve_frontier.py`
- moved the seed-placement builder, temporary placement builder, pure endpoint rebuild helpers, and main `_cf_place_chain()` entrypoint into the placement module
- kept `_point_registry_key` in `solve_records.py` as shared registry contract and left the stateful anchor-adjustment apply wrapper in `solve_frontier.py`
- updated `solve_frontier.py` to import and use the extracted placement helpers without changing frontier orchestration order
- scoring/rank behavior, runtime state/cache behavior, rescue flow, and finalization were intentionally left unchanged

## Goal

Move placement math and direction logic into `cftuv/frontier_place.py`.

## Allowed files

- `cftuv/solve_frontier.py`
- `cftuv/frontier_place.py`

## Move candidates

- chain length helpers
- direction normalization / snapping / rotation helpers
- source step direction helpers
- interpolation / resample helpers
- guided free chain builders
- frame chain builders
- endpoint rebuild helpers
- anchor adjustment application
- direction inference helpers
- chain placement function

## Must not change

- score/rank behavior
- runtime state/cache behavior
- rescue flow
- finalization

## Done when

- placement math lives outside `solve_frontier.py`
- frontier orchestrator imports and uses the moved functions
- behavior remains identical

---

# P3 — Extract finalize subsystem

Status: completed

P3 result:

- created `cftuv/frontier_finalize.py` and moved patch gap diagnostics, envelope building, and quilt finalization into it
- moved finalize-only diagnostics / pin-policy imports out of `solve_frontier.py` and into the finalize module
- updated `solve_frontier.py` to call imported finalization helpers without changing finalize order or untouched-patch fallback behavior
- diagnostics semantics, closure/frame report collection flow, and public orchestration entrypoints were intentionally left unchanged

## Goal

Move patch envelope assembly and quilt finalization into `cftuv/frontier_finalize.py`.

## Allowed files

- `cftuv/solve_frontier.py`
- `cftuv/frontier_finalize.py`

## Move candidates

- patch chain gap diagnostics
- envelope building
- scaffold finalization for a quilt
- stop-diagnostics ownership only if P0 assigned it here

## Must not change

- diagnostics semantics
- untouched patch fallback logic
- closure/frame report collection flow

## Done when

- finalize path is isolated
- `solve_frontier.py` reads more like a pipeline

---

# P4 — Extract rescue subsystem

Status: completed

P4 result:

- created `cftuv/frontier_rescue.py` and moved closure-follow UV reconstruction, rescue-gap builders, closure-follow placement flow, partner anchor helper, and tree-ingress placement flow into it
- kept rescue as a cold path called only from stall handling; stall -> `tree_ingress` -> `closure_follow` order remains unchanged
- used temporary dependency injection for the still-orchestrator-owned axis-safety check and anchor-adjustment apply wrapper, to avoid creating a cycle before P5/P6
- rescue telemetry payloads and placement-path labels were intentionally left unchanged

## Goal

Move cold-path rescue logic into `cftuv/frontier_rescue.py`.

## Allowed files

- `cftuv/solve_frontier.py`
- `cftuv/frontier_rescue.py`

## Move candidates

- closure-follow UV reconstruction
- rescue-gap builders
- closure-follow placement flow
- partner anchor helper
- tree-ingress placement flow

## Must not change

- rescue must remain separate from main frontier
- stall -> rescue order must remain the same
- rescue telemetry fields must stay intact

## Done when

- rescue logic is isolated as a cold path subsystem

---

# P5 — Extract runtime state / cache substrate

Status: completed

P5 result:

- created `cftuv/frontier_state.py` and moved `FrontierRuntimePolicy`, point registration, dirty propagation, and register/reject/dependency state helpers into it
- left `evaluate_candidate()` and `build_stop_diagnostics()` outside `frontier_state.py` as free functions in `solve_frontier.py`, so the state layer does not depend upward on the orchestrator before P6
- left score-cache bootstrap as an explicit orchestrator-side helper for now, so score-owned cache initialization did not get absorbed into state ownership
- score-owned compatibility fields/accessors on `FrontierRuntimePolicy` remain temporary until P7; this phase did not reclassify them as final state ownership
- cache invalidation order, dirty-marking rules, and placed-count semantics were intentionally left unchanged

## Goal

Extract the mutable runtime substrate into `cftuv/frontier_state.py`.

## Allowed files

- `cftuv/solve_frontier.py`
- `cftuv/frontier_state.py`

## Move candidates

- mutable runtime fields currently owned by `FrontierRuntimePolicy`
- cache storage fields
- dirty propagation helper
- register/reject helpers
- dependency patch derivation
- point registration, if P0 assigns it here
- a **thin** runtime container class if compatibility requires keeping one

## Explicit non-goal

Do **not** bulk-move all `FrontierRuntimePolicy` methods unchanged.
This phase extracts the state substrate only.

## Must not change

- cache invalidation order
- dirty-marking rules
- bit-identical frontier candidate selection behavior
- placed-count semantics

## Done when

- mutable runtime state/cache ownership is isolated
- no evaluation/scoring logic is left masquerading as state ownership

---

# P6 — Extract evaluation / anchors subsystem

Status: completed

P6 result:

- created `cftuv/frontier_eval.py` and moved closure-preconstraint helpers, seed choice, anchor discovery/resolution, candidate evaluation, selector, stop-diagnostics, and main-path frontier placement into it
- kept score-building ownership in `solve_frontier.py` by wiring `frontier_eval.py` through explicit score callback entrypoints instead of importing upward into the orchestrator
- left `FrontierRuntimePolicy` as a state container without adding an upward edge; `solve_frontier.py` now keeps only thin delegators/wrappers for eval-owned logic
- runtime cache contract, rescue order, threshold gate, anchor provenance semantics, and public frontier API were intentionally left unchanged

## Goal

Move anchor finding/resolution and candidate evaluation into `cftuv/frontier_eval.py`.

## Allowed files

- `cftuv/solve_frontier.py`
- `cftuv/frontier_eval.py`

## Move candidates

- anchor counting/debug labels
- anchor discovery
- frame axis helper functions
- anchor safety checks
- dual-anchor closure rules
- resolved anchor logic
- closure preconstraint helpers
- candidate evaluation body
- `evaluate_candidate()` as standalone helper plus optional thin delegator method

## Must not change

- runtime cache contract
- anchor provenance semantics
- evaluation ordering

## Done when

- `FrontierRuntimePolicy.evaluate_candidate()` is either a thin delegator or fully replaced by clearly owned `frontier_eval.py` logic
- eval imports only from already-stable lower layers per DAG

---

# P7 — Extract scoring subsystem

Status: completed

P7 result:

- created `cftuv/frontier_score.py` and moved score-cache bootstrap, topology/patch/seam/corner helpers, structured-rank construction, layered score helpers, and the active score entrypoint alias into it
- kept `frontier_eval.py` wiring stable by preserving callback-based score entrypoints; `solve_frontier.py` now rebinds score-owned compatibility symbols to `frontier_score.py`
- left formulas, threshold gating, rank ordering, rescue boundary, and public frontier API unchanged
- intentionally deferred dead local score helper removal in `solve_frontier.py` to P8 shell compression, so this phase stays structural

## Goal

Move scoring / rank construction into `cftuv/frontier_score.py`.

## Allowed files

- `cftuv/solve_frontier.py`
- `cftuv/frontier_score.py`

## Move candidates

- role tier helpers
- shape profile builders
- patch scoring context
- seam relation helpers
- corner hint builders
- topology fact builders
- layered score helpers
- rank builder
- active score entrypoint alias
- shape-profile caching ownership if P0 assigned it here

## Must not change

- formulas
- threshold gate behavior
- rank ordering semantics
- rescue integration boundary

## Done when

- scoring becomes a coherent subsystem outside the frontier orchestrator
- outputs remain identical
- scoring depends only on already-stable lower layers per DAG

---

# P8 — Compress `solve_frontier.py` into orchestration shell

Status: completed

P8 result:

- created `cftuv/frontier_closure.py` and moved closure-pair matching / closure bootstrap helpers out of `solve_frontier.py`
- moved the remaining anchor-adjustment body into `cftuv/frontier_place.py`, so the shell no longer owns placement-side endpoint rebuild mutation
- rewrote `solve_frontier.py` as a top-level coordinator with only bootstrap, pool indexing/build, main-loop orchestration, finalize handoff, public entrypoints, and thin eval stitching wrappers
- preserved rescue order, bootstrap/main-loop/finalize order, threshold behavior, telemetry ownership, and compatibility re-exports needed by diagnostics

## Goal

Make `solve_frontier.py` read as a top-level frontier pipeline.

## Allowed files

- `cftuv/solve_frontier.py`
- small import fixes in sibling frontier modules if required

## Tasks

- Keep only orchestration-level logic in `solve_frontier.py`
- Preserve public entrypoints:
  - `build_quilt_scaffold_chain_frontier`
  - `build_root_scaffold_map`
- Keep bootstrap / pool-build / main loop structure easy to read
- Leave closure-pair ownership outside the shell
- Leave telemetry ownership outside the shell

## Done when

- A reviewer can read `solve_frontier.py` top-to-bottom as a pipeline coordinator
- module ownership boundaries are obvious

---

# P9 — Post-refactor perf research plan

Status: completed

P9 result:

- added `docs/cftuv_frontier_perf_phase2_plan.md` as the concrete post-refactor optimization roadmap
- documented selector-queue shadow mode, versioned patch-context caches, hot/cold path separation, and telemetry counters as separate workstreams with migration order
- made correctness gates explicit: bit-identical selector output against the current scan+cache control before any future cutover
- left frontier runtime, selector behavior, scoring, rescue order, and public API unchanged in this phase

## Goal

Design the next optimization stage after structural split. This is a design/doc phase, not a mainline behavior change.

## Allowed files

- `docs/cftuv_frontier_perf_phase2_plan.md`

## Research topics

- heap/lazy frontier queue instead of scan+cache selection
- patch-context cache / versioning
- hot/cold path separation
- telemetry impact of alternative selector structures

## Must not do

- do not land selector redesign in this phase
- do not alter mainline frontier behavior

## Done when

- there is a concrete phase-2 optimization doc with migration steps and risks

---

## Review gates after phase groups

### Gate A — after P1–P4

- addon loads
- no import cycles
- frontier output unchanged
- `solve_frontier.py` is already visibly smaller
- closure ownership is no longer ambiguous

### Gate B — after P5–P7

- build order unchanged
- UV output unchanged
- diagnostics unchanged
- cache-driven frontier still behaves identically
- no circular imports in lower frontier layers

### Gate C — after P8

- `solve_frontier.py` is a thin orchestration shell
- ownership boundaries are obvious to future agents
- `FrontierRuntimePolicy` no longer acts as a mixed-responsibility god object

---

## First message template from orchestrator to an AI agent

Use this template when assigning a phase:

```text
You are implementing phase <PHASE_ID> from `docs/cftuv_frontier_orchestrator_plan_v2.md`.

Read first:
1. `AGENTS.md`
2. `docs/cftuv_frontier_orchestrator_plan_v2.md`
3. Any files directly named in your phase scope

Hard rules:
- Preserve chain-first strongest-frontier.
- Do not drift into patch-first, loop-sequential, or corner-first placement.
- Respect the import DAG in the plan.
- If the phase is mechanical, do not change behavior.
- Do not touch files outside the allowed phase scope unless strictly required for imports.
- Leave a short execution log entry in the plan file when done.

Your task for this run:
<PASTE PHASE GOAL + ALLOWED FILES + MUST NOT CHANGE + DONE WHEN>

Deliverables:
- code changes for this phase only
- updated checkbox in the plan file
- one execution log entry
- short handoff note: what changed, what was not touched, remaining risks
```

---

## Suggested first assignment order

Start with **P0**, then **P1**, then **P2**.

Reason:

- P0 removes ownership ambiguity and locks the DAG.
- P1 reduces frontier noise before extraction.
- P2 yields the fastest safe reduction in `solve_frontier.py` size.

Then continue with:

- P3
- P4
- P5
- P6
- P7
- P8
- P9

This order is intentional: **state → eval → score** for the sensitive core.

---

## Execution log

| Date | Phase | Result | Files | Notes |
|------|------|--------|-------|-------|
| 2026-03-31 | P0 | completed | `docs/cftuv_frontier_orchestrator_plan_v2.md`, `docs/cftuv_frontier_split_contract.md` | Locked ownership map for `solve_frontier.py`, classified `FrontierRuntimePolicy` + frontier records, and amended DAG for eval diagnostics bridge / finalize pin-policy bridge. |
| 2026-03-31 | P1 | completed | `cftuv/solve_frontier.py`, `cftuv/solve_planning.py`, `docs/cftuv_frontier_orchestrator_plan_v2.md` | Removed dead legacy scorer and disabled planning helper, collapsed the rectification helper to its active pass-through path, and left runtime behavior unchanged. |
| 2026-03-31 | P2 | completed | `cftuv/frontier_place.py`, `cftuv/solve_frontier.py`, `docs/cftuv_frontier_orchestrator_plan_v2.md` | Extracted pure placement math / direction helpers into `frontier_place.py`, kept `_point_registry_key` in `solve_records.py`, left the stateful anchor-adjustment apply wrapper in the orchestrator, and kept scoring, rescue, finalize, and cache behavior unchanged. |
| 2026-03-31 | P3 | completed | `cftuv/frontier_finalize.py`, `cftuv/solve_frontier.py`, `docs/cftuv_frontier_orchestrator_plan_v2.md` | Extracted patch gap diagnostics, envelope assembly, and quilt finalization into `frontier_finalize.py`, moved finalize-only diagnostics / pin-policy imports there, and preserved finalize order plus untouched-patch fallback behavior. |
| 2026-03-31 | P4 | completed | `cftuv/frontier_rescue.py`, `cftuv/solve_frontier.py`, `docs/cftuv_frontier_orchestrator_plan_v2.md` | Extracted closure-follow and tree-ingress rescue flows into `frontier_rescue.py`, preserved stall rescue order and telemetry fields, and used injected seams for still-orchestrator-owned axis-safety / anchor-adjustment helpers to avoid premature state/eval coupling. |
| 2026-03-31 | P5 | completed | `cftuv/frontier_state.py`, `cftuv/solve_frontier.py`, `docs/cftuv_frontier_orchestrator_plan_v2.md` | Extracted the runtime container, point registration, dirty propagation, and register/reject substrate into `frontier_state.py`, kept eval/stop-diagnostics as free functions in `solve_frontier.py` to avoid an upward DAG edge from state, and left score-cache bootstrap explicitly in the orchestrator with score-owned compatibility caches marked temporary until P7. |
| 2026-03-31 | P6 | completed | `cftuv/frontier_eval.py`, `cftuv/solve_frontier.py`, `docs/cftuv_frontier_orchestrator_plan_v2.md` | Extracted anchor discovery/resolution, closure preconstraint, candidate evaluation, selector, stop-diagnostics, and main-path frontier placement into `frontier_eval.py`, kept score ownership in the orchestrator via injected score hooks, and preserved cache/threshold/rescue behavior with thin delegators only in `solve_frontier.py`. |
| 2026-03-31 | P7 | completed | `cftuv/frontier_score.py`, `cftuv/solve_frontier.py`, `docs/cftuv_frontier_orchestrator_plan_v2.md` | Extracted score-cache bootstrap and the scoring/rank helper cluster into `frontier_score.py`, rebound the compatibility score symbols in `solve_frontier.py` to the new module, and kept formulas, threshold gating, rank ordering, and rescue boundaries unchanged. |
| 2026-03-31 | P8 | completed | `cftuv/solve_frontier.py`, `cftuv/frontier_closure.py`, `cftuv/frontier_place.py`, `docs/cftuv_frontier_orchestrator_plan_v2.md` | Removed unreachable legacy bodies and alias ballast from `solve_frontier.py`, moved closure-pair ownership into `frontier_closure.py` plus anchor-adjustment ownership into `frontier_place.py`, kept only orchestration helpers and thin eval stitching wrappers in the shell, and left bootstrap/main-loop/finalize/rescue order plus threshold/telemetry behavior unchanged. |
| 2026-03-31 | P9 | completed | `docs/cftuv_frontier_perf_phase2_plan.md`, `docs/cftuv_frontier_orchestrator_plan_v2.md` | Wrote the Phase 2 optimization roadmap covering queue shadow mode, patch-context versioning, hot/cold path split, and telemetry gates, defined correctness/cutover criteria, and left mainline frontier behavior unchanged. |
