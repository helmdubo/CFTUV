# R4-01 — Native hot paths, GPU drag overlay and target-scale gate

Status: **BLOCKED**  
Phase: `Runtime`  
Dependencies: `R2-01, R3-01 if activated`  
Primary role: `Native/runtime integration agent`  
Parallel group: `none`  
Relative size: `XL`

## Copy-paste start prompt

```text
You are the implementation agent for `R4-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

After algorithms and profiles are stable, measured hot paths may move to native code. The final gate is target-scale interactive drag.

## Objective

Port only measured hot paths, preserve Python contracts/reference tests, and meet the selected p95/p99 drag SLA.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- R0 benchmark harness
- R1/R2 handoffs
- R3 handoff if activated
- kernel pyproject/wheel CI

## Allowed paths

- native extension package under kernel/
- Python bindings
- .github workflows for wheel matrix
- GPU overlay integration
- benchmark artifacts

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- No semantic redesign during native port.
- No approximate geometry published as final.
- No GPU-owned geometry algorithm.

## Required design

- Likely candidates: predicates, intersection splitting, DCEL/cell operations, broadphase, event queue, SoA runtime state, triangulation.
- Python remains contract/orchestration layer.
- GPU overlay reads runtime GeometryBatch/deltas.

## Implementation sequence

- Select hot paths from profiles.
- Port with bit/record differential tests.
- Build hermetic wheels for supported platforms.
- Integrate incremental GPU buffer updates.
- Run target-scale p50/p95/p99 gate.

## Acceptance criteria

- Native and Python reference results are semantically identical.
- Wheel/extraction CI passes.
- 3000-edge/50-patch workload meets the product-selected SLA.
- Event frames and backward drag are included.
- No source mesh mutation on cancellation/failure.

## Required tests and evidence

- Cross-platform wheel tests.
- Python/native differential.
- Target benchmark matrix.
- Long drag/cancellation soak.

## Deliverables

- Native runtime.
- GPU drag integration.
- Production performance receipt.

## Stop conditions

- Stop if the selected SLA is not formally recorded.
- Stop if a native discrepancy cannot be reduced to a reproducible exact fixture.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
