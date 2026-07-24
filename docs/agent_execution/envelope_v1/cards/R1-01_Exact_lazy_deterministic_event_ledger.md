# R1-01 — Exact lazy deterministic event ledger

Status: **BLOCKED**  
Phase: `Runtime`  
Dependencies: `R0-01, H0-01`  
Primary role: `Runtime algorithm implementer`  
Parallel group: `none`  
Relative size: `XL`

## Copy-paste start prompt

```text
You are the implementation agent for `R1-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

Ordinary alpha drag must query a compiled evolution rather than full-recompute every frame. Initial reflex fan subdivision does not cover later kinetic events.

## Objective

Implement an exact incremental event ledger with lazy successor scheduling and backward checkpoint/replay, matching the reference evaluator.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted StaticPatchProgram/EvaluationState contracts
- accepted cell-complex pipeline
- kernel/src/cftuv_envelope/contracts/events.py
- Session A drag topology fixtures
- CGAL event taxonomy as design reference

## Allowed paths

- kernel/src/cftuv_envelope/runtime/**
- kernel/src/cftuv_envelope/contracts/events.py
- kernel/tests/test_event_ledger.py
- kernel/fixtures/session_g_events_v1/**

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- Do not conflate hidden-support fan with split events.
- Do not precompute an unbounded future schedule.
- Do not publish partial semantic states.
- No arbitrary pairwise ordering for same-alpha events.

## Required design

- Typed events: edge collapse, reflex split, reflex-vertex contact, boundary contact, component exhaustion, point-only contact, atomic same-alpha batch.
- Schedule only immediate candidates; invalidate stale candidates by topology version.
- Advance analytically inside topology intervals.
- Backward drag uses deterministic checkpoint/replay or an accepted reversible ledger.

## Implementation sequence

- Define exact event records and transitions.
- Implement current-state event queue.
- Implement forward advance and checkpointing.
- Compare every state with full reference before/at/after events.
- Integrate generation tokens for host publication.

## Acceptance criteria

- Runtime state equals reference semantic records/digest around every event.
- Same-alpha processing is permutation invariant.
- Backward replay returns the same state.
- No full static recompile on alpha drag.

## Required tests and evidence

- Event triplets alpha-epsilon/alpha/alpha+epsilon.
- Forward/backward sweeps.
- Point contact, split, boundary capacity and interaction events.
- Target-scale event density benchmark.

## Deliverables

- Exact event ledger.
- Event corpus.
- Differential and performance receipt.

## Stop conditions

- Stop on an unapproved event type or multiway policy; return a named unsupported state.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
