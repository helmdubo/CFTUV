# F2-01 — Downstream tessellation and GeometryBatch

Status: **BLOCKED**  
Phase: `Session F`  
Dependencies: `F1-01`  
Primary role: `Tessellation implementer`  
Parallel group: `none`  
Relative size: `XL`

## Copy-paste start prompt

```text
You are the implementation agent for `F2-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

The semantic arrangement fully defines geometry and attributes. Tessellation only encodes it as mesh faces.

## Objective

Produce a deterministic GeometryBatch with occurrence-aware vertex keys and complete semantic provenance.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted F1 handoff
- kernel/src/cftuv_envelope/contracts/tessellation.py
- kernel/src/cftuv_envelope/contracts/geometry_batch.py
- kernel/tests/test_geometry_batch.py

## Allowed paths

- kernel/src/cftuv_envelope/tessellation/**
- kernel/src/cftuv_envelope/contracts/tessellation.py
- kernel/src/cftuv_envelope/contracts/geometry_batch.py
- kernel/tests/test_geometry_batch.py
- kernel/tests/test_reference_tessellation.py

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- No silhouette repair.
- No owner inference.
- No coordinate-based VertexKey.
- No hidden-edge/profile simplification.

## Required design

- VertexKey includes semantic/topological occurrence identity, not just position.
- Different valid triangulations share one semantic geometry digest.
- Representation-only diagonals are typed separately.
- Every face references one ownership/station/UV/provenance record.

## Implementation sequence

- Define final VertexKey contract.
- Implement reference tessellation for supported polygons/holes.
- Emit vertices, faces, boundaries and attributes.
- Validate manifoldness by semantic occurrences rather than coordinate welding.

## Acceptance criteria

- One correct GeometryBatch is produced for the reference corpus.
- Silhouette/ownership/UV digests match input semantic arrangement.
- Coincident occurrences remain distinct vertices where required.
- Every batch element has traceable provenance.

## Required tests and evidence

- Multiple tessellation strategy differential on semantic digest.
- Holes, reflex fans, equality interfaces and point contacts.
- Deterministic serialization.

## Deliverables

- GeometryBatch evaluator.
- Schemas and fixtures.
- Tessellation receipt.

## Stop conditions

- Stop and return a named tessellation unsupported outcome rather than altering semantic geometry.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
