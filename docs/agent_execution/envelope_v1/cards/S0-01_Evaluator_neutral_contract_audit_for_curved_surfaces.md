# S0-01 — Evaluator-neutral contract audit for curved surfaces

Status: **BLOCKED**  
Phase: `Curved research`  
Dependencies: `F2-01`  
Primary role: `Geometry architecture reviewer`  
Parallel group: `none`  
Relative size: `M`

## Copy-paste start prompt

```text
You are the implementation agent for `S0-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

Curved implementation is deferred, but planar public contracts must not accidentally require globally linear supports or polygon-only identity.

## Objective

Audit and minimally generalize evaluator boundaries without implementing a curved solver.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted GeometryBatch and ownership contracts
- docs/decal_envelope_linear_axis_rebaseline.md curved sections
- kernel/src/cftuv_envelope/contracts/surface.py
- kernel evaluator interfaces

## Allowed paths

- docs/session_s0_curved_contract_gate.md
- kernel evaluator protocols/interfaces
- contract tests only where needed

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- No FMM/Heat/SurfaceVoronoi implementation.
- Do not weaken planar exact guarantees.
- Do not create a second user-facing Decal Engine.

## Required design

- Same semantic Envelope variants and ownership model.
- Evaluator strategy owns metric/construction certificates.
- Boundary occurrences, cells, equality loci and GeometryBatch are evaluator-neutral.
- Approximation/error certificates are explicit for non-planar strategies.

## Implementation sequence

- Audit planar-specific types.
- Define Planar/Developable/Surface evaluator protocol.
- List required contract version changes.
- Obtain architecture acceptance.

## Acceptance criteria

- No public semantic entity requires a global planar coordinate unless scoped to the planar evaluator.
- Planar fixtures remain unchanged.
- Curved error/metric authority is explicit.

## Required tests and evidence

- Protocol/contract tests.
- Independent architecture review.

## Deliverables

- Curved evaluator ADR and interface gate.

## Stop conditions

- Stop before choosing a general-surface algorithm.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
