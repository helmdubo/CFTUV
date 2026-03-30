# CFTUV Frontier Refactor — Orchestrator Plan for AI Agents

Status: active orchestration plan  
Owner: orchestrator  
Scope: `cftuv/solve_frontier.py` and new sibling modules under `cftuv/`

---

## Purpose

Turn `solve_frontier.py` from a large mixed-responsibility module into a thin orchestrator/facade, while preserving the current chain-first strongest-frontier behavior.

This plan is written for AI agents. Each agent takes one phase, updates this file, and leaves a short handoff note.

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

Authoritative project context: `AGENTS.md`.

---

## Target module shape

Planned frontier split:

- `cftuv/solve_frontier.py` — thin orchestrator / public frontier facade
- `cftuv/frontier_state.py` — mutable runtime state, cache, dirty propagation, registration
- `cftuv/frontier_eval.py` — anchor finding/resolution, candidate evaluation, closure preconstraint
- `cftuv/frontier_score.py` — topology facts, patch context, seam/shape/corner scoring, rank building
- `cftuv/frontier_place.py` — placement math, direction inheritance, endpoint rebuild, anchor adjustment apply
- `cftuv/frontier_rescue.py` — `tree_ingress`, `closure_follow`, rescue ranking
- `cftuv/frontier_finalize.py` — envelopes, chain gaps, quilt finalization

---

## Global execution rules for every agent

1. One agent = one phase = one PR.
2. No opportunistic rewrites outside the phase scope.
3. If you notice adjacent debt, record it in the handoff note, but do not fix it unless the phase explicitly allows it.
4. If a phase says **mechanical extraction only**, do not change formulas, ranking order, threshold logic, rescue flow, cache invalidation order, or telemetry payload shape.
5. After finishing a phase, update the checkbox in this file and append one entry to the execution log.

---

## Required handoff format for every phase

Append to the execution log:

- Date
- Phase ID
- Files touched
- What moved / changed
- What was deliberately not touched
- Risks remaining

---

## Phase board

- [ ] P0 — Contract + symbol map
- [ ] P1 — Debt triage / dead code isolation
- [ ] P2 — Extract placement subsystem
- [ ] P3 — Extract finalize subsystem
- [ ] P4 — Extract rescue subsystem
- [ ] P5 — Extract scoring subsystem
- [ ] P6 — Extract evaluation / anchors subsystem
- [ ] P7 — Extract runtime state / cache subsystem
- [ ] P8 — Compress `solve_frontier.py` into orchestration shell
- [ ] P9 — Post-refactor perf research plan (design only, no behavior change in mainline)

---

# P0 — Contract + symbol map

Status: not started

## Goal

Produce the authoritative split contract before code motion begins.

## Allowed files

- `docs/cftuv_frontier_orchestrator_plan.md`
- optional: new doc `docs/cftuv_frontier_split_contract.md`

## Tasks

- Map every significant function/class in `solve_frontier.py` to its future target module.
- Mark risky zones:
  - runtime cache / dirty propagation
  - rescue flows
  - anchor resolution
  - scoring/rank construction
  - finalization
  - dead or disabled code branches
- Define import DAG and avoid cycles.
- Mark each future move as either:
  - `mechanical`
  - `mechanical but risky`
  - `behavioral / later`

## Done when

- Another agent can start extraction without guessing ownership boundaries.
- Import direction is explicit.

---

# P1 — Debt triage / dead code isolation

Status: not started

## Goal

Reduce frontier cognitive noise before extraction.

## Allowed files

- `cftuv/solve_frontier.py`
- optional archival note under `docs/`

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
- Runtime output remains unchanged.

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

# P5 — Extract scoring subsystem

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

## Must not change

- formulas
- threshold gate behavior
- rank ordering semantics
- rescue integration boundary

## Done when

- scoring becomes a coherent subsystem outside the frontier orchestrator
- outputs remain identical

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

## Must not change

- runtime cache contract
- anchor provenance semantics
- evaluation ordering

## Done when

- `FrontierRuntimePolicy.evaluate_candidate()` is either a thin delegator or the logic is otherwise clearly owned by `frontier_eval.py`

---

# P7 — Extract runtime state / cache subsystem

Status: not started

## Goal

Move mutable runtime owner logic into `cftuv/frontier_state.py`.

## Allowed files

- `cftuv/solve_frontier.py`
- `cftuv/frontier_state.py`

## Move candidates

- `FrontierRuntimePolicy`
- cache storage fields
- dirty propagation helper
- register/reject helpers
- dependency patch derivation
- point registration, if needed

## Must not change

- cache invalidation order
- dirty-marking rules
- bit-identical frontier candidate selection behavior
- placed-count semantics

## Done when

- runtime state/cache is isolated and the main frontier file no longer owns this mutable engine directly

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

## Done when

- A reviewer can read `solve_frontier.py` top-to-bottom as a pipeline coordinator

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

## Review gates after phases

### Gate A — after P1–P4

- addon loads
- no import cycles
- frontier output unchanged
- `solve_frontier.py` is already visibly smaller

### Gate B — after P5–P7

- build order unchanged
- UV output unchanged
- diagnostics unchanged
- cache-driven frontier still behaves identically

### Gate C — after P8

- `solve_frontier.py` is a thin orchestration shell
- ownership boundaries are obvious to future agents

---

## First message template from orchestrator to an AI agent

Use this template when assigning a phase:

```text
You are implementing phase <PHASE_ID> from `docs/cftuv_frontier_orchestrator_plan.md`.

Read first:
1. `AGENTS.md`
2. `docs/cftuv_frontier_orchestrator_plan.md`
3. Any files directly named in your phase scope

Hard rules:
- Preserve chain-first strongest-frontier.
- Do not drift into patch-first, loop-sequential, or corner-first placement.
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

## Suggested first assignment

Start with **P0**, then **P1**, then **P2**.

Reason:
- P0 removes ambiguity.
- P1 reduces frontier noise before extraction.
- P2 yields the fastest safe reduction in `solve_frontier.py` size.

---

## Execution log

| Date | Phase | Result | Files | Notes |
|------|------|--------|-------|-------|
| TBD | P0 | not started | — | awaiting first agent |
