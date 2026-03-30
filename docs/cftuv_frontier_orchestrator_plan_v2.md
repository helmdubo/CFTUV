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
- [ ] P2 — Extract placement subsystem
- [ ] P3 — Extract finalize subsystem
- [ ] P4 — Extract rescue subsystem
- [ ] P5 — Extract runtime state / cache substrate
- [ ] P6 — Extract evaluation / anchors subsystem
- [ ] P7 — Extract scoring subsystem
- [ ] P8 — Compress `solve_frontier.py` into orchestration shell
- [ ] P9 — Post-refactor perf research plan (design only, no behavior change in mainline)

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

Status: not started

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

Status: not started

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

Status: not started

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

Status: not started

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

Status: not started

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

Status: not started

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

Status: not started

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

Status: not started

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
