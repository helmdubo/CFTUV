# S3-01 — General SurfaceEnvelopeEvaluator

Status: **BLOCKED**  
Phase: `Curved implementation`  
Dependencies: `S2-01, N0-01, R4-01 recommended`  
Primary role: `Surface geometry implementation team`  
Parallel group: `none`  
Relative size: `XXL`

## Copy-paste start prompt

```text
You are the implementation agent for `S3-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

The research and metric policies have been accepted. The production evaluator must emit the same semantic cell/ownership/GeometryBatch model as planar.

## Objective

Implement multi-source intrinsic arrival for curve generators, vector/tangent transport, multi-arrival equality loci and cut-locus events under explicit error contracts.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted SurfaceArrivalComplex proposal
- accepted metric policies
- accepted evaluator-neutral contracts
- planar/developable reference behavior

## Allowed paths

- kernel/src/cftuv_envelope/surface/**
- surface contracts/schemas
- kernel/tests/test_surface_evaluator.py
- surface fixture corpora
- native/GPU modules only after reference correctness

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- No rail-as-silhouette authority.
- No nearest-source ownership repair.
- No single-winner tie-break at cut loci.
- No unreported approximation.

## Required design

- Intrinsic StripEnvelope from curve generators and owner-side co-normal.
- Intrinsic AngularEnvelope from tangent-cone reflex subdivision and transported directions.
- Multiple arrival candidates per cell/triangle.
- Cut locus and equal-arrival sets become typed events/interfaces.
- Reference surface evaluator first; accelerated runtime later.

## Implementation sequence

- Implement reference arrival complex.
- Project to common Coverage/Ownership contracts.
- Add cut-locus/event handling.
- Add downstream surface tessellation/lift.
- Build filtered/native runtime only after differential acceptance.

## Acceptance criteria

- Hemisphere, ribbon and noise fixtures satisfy accepted error/topology contracts.
- Ownership is deterministic and no ID tie-break occurs.
- Mesh refinement is stable within the error contract.
- Target-scale runtime is measured separately.

## Required tests and evidence

- Rotational invariance.
- Multiple geodesic arrivals.
- Curve source/barrier cases.
- Refinement and policy differential.

## Deliverables

- SurfaceEnvelopeEvaluator.
- Surface fixture corpus.
- Correctness/error/performance receipts.

## Stop conditions

- Stop on unrepresented cut-locus topology or missing owner candidate; do not collapse to an arbitrary winner.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
