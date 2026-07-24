# S1-01 — Developable surface evaluator by certified unfolding

Status: **BLOCKED**  
Phase: `Curved research`  
Dependencies: `S0-01, R4-01 recommended`  
Primary role: `Developable geometry implementer`  
Parallel group: `none`  
Relative size: `XL`

## Copy-paste start prompt

```text
You are the implementation agent for `S1-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

Ribbons, cylinders, cones and other proven developable patches can reuse planar semantics through an isometric atlas.

## Objective

Implement a certified developable evaluator that unfolds, invokes the planar evaluator and lifts results through explicit chart transitions.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted S0 protocol
- planar evaluator APIs
- surface metric contracts

## Allowed paths

- kernel/src/cftuv_envelope/developable/**
- developable metric contracts/schemas
- kernel/tests/test_developable_evaluator.py
- fixtures for ribbon/cylinder/cone

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- Do not accept visually flat surfaces without a certificate.
- No hidden projection tolerance.
- Do not change planar Envelope semantics.

## Required design

- Admit only a named developable regime.
- Build an intrinsic isometric chart/atlas.
- Reuse planar Envelope/cell/ownership pipeline.
- Record chart transitions and lift certificates.

## Implementation sequence

- Define admission certificate.
- Implement unfolding and transitions.
- Evaluate/lift fixtures.
- Compare intrinsic output to planar unfolded oracle.

## Acceptance criteria

- Intrinsic coverage/ownership matches unfolded planar result.
- Chart boundaries preserve occurrences and UV interfaces.
- Non-developable input fails closed.

## Required tests and evidence

- Bent ribbon invariance.
- Cylinder/cone fixtures.
- Mesh refinement/retriangulation.

## Deliverables

- Developable evaluator.
- Admission/lift certificates.
- Fixture corpus.

## Stop conditions

- Stop on significant Gaussian curvature or ambiguous holonomy; defer to general surface.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
