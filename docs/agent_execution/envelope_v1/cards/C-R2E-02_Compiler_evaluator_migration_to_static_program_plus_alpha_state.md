# C-R2E-02 — Compiler/evaluator migration to static program plus alpha state

Status: **BLOCKED**  
Phase: `C-R2E`  
Dependencies: `C-R2E-01`  
Primary role: `Kernel implementation agent`  
Parallel group: `none`  
Relative size: `XL`

## Copy-paste start prompt

```text
You are the implementation agent for `C-R2E-02` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

Contracts are separated; the reference compile/evaluate path must now obey them without changing geometry.

## Objective

Compile once per source revision/request/domain and evaluate arbitrary alpha values into independent states.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted C-R2E-01 contracts
- kernel/src/cftuv_envelope/reference/compile.py
- kernel/src/cftuv_envelope/reference/raw_coverage.py
- kernel/src/cftuv_envelope/interactions/resolved_coverage.py

## Allowed paths

- kernel/src/cftuv_envelope/reference/compile.py
- kernel/src/cftuv_envelope/reference/raw_coverage.py
- kernel/src/cftuv_envelope/interactions/** as adapters
- kernel/tests/test_static_patch_program.py
- kernel/tests/test_alpha_evaluation_state.py

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- Do not cache mutable alpha state inside the static program.
- Do not change full-recompute reference results.

## Required design

- Compile source supports, hidden profile selection and topology once.
- Instantiate moving supports and capacities per evaluation.
- Use stable IDs from static entities plus alpha-independent construction laws; alpha-specific instance IDs must be explicit and deterministic.

## Implementation sequence

- Implement `compile_static_patch_program`.
- Implement `evaluate_reference_state(program, alpha)`.
- Adapt old entrypoints.
- Evaluate an alpha sweep with one program.
- Compare all states against the old compile-per-alpha path.

## Acceptance criteria

- One compile serves at least 100 alpha queries.
- Reference Raw/Resolved records match accepted output.
- Static objects are immutable and not mutated by evaluation.
- Parallel evaluations of two alpha values do not interfere.

## Required tests and evidence

- Alpha sweep differential.
- Thread/process isolation test where practical.
- Boundary capacity and interaction cases.
- Full kernel suite.

## Deliverables

- New compiler/evaluator APIs.
- Compatibility wrappers.
- Migration receipt.

## Stop conditions

- Stop if an identity currently depends on requested alpha and no accepted replacement exists; document the identity decision.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
