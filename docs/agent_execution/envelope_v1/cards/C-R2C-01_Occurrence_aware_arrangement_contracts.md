# C-R2C-01 — Occurrence-aware arrangement contracts

Status: **BLOCKED**  
Phase: `C-R2C`  
Dependencies: `BASE-00, FIX-00`  
Primary role: `Kernel contract author`  
Parallel group: `none`  
Relative size: `M`

## Copy-paste start prompt

```text
You are the implementation agent for `C-R2C-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

The arrangement currently stores one vertex per exact coordinate. At point contacts this loses the distinction between geometric point and boundary occurrence.

## Objective

Introduce versioned contracts for geometric points, topological occurrences, directed boundary halfedges and point contacts without yet rewriting traversal.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- kernel/src/cftuv_envelope/reference/contracts.py
- kernel/src/cftuv_envelope/reference/arrangement_protocol.py
- kernel/src/cftuv_envelope/reference/arrangement.py
- kernel/src/cftuv_envelope/schema.py
- kernel/tests/test_public_contracts.py

## Allowed paths

- kernel/src/cftuv_envelope/reference/contracts.py
- kernel/src/cftuv_envelope/reference/arrangement_protocol.py
- kernel/src/cftuv_envelope/ids.py
- kernel/src/cftuv_envelope/schema.py
- kernel/schema/**
- kernel/tests/test_public_contracts.py
- kernel/tests/test_generated_schemas.py
- docs/session_c_r2c_*

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- Do not change Envelope laws, metric, Policy B, ownership or tessellation.
- Do not choose topology by ID order.

## Required design

- Recommended concepts: `ArrangementPointV1`, `BoundaryVertexOccurrenceV1`, `DirectedBoundaryHalfEdgeV1`, `PointContactRecordV1`.
- A point stores geometry/construction history; an occurrence stores one boundary-sector role at that point.
- Edges reference start/end occurrences and their shared arrangement points.
- Point contact records describe coincident occurrences without merging area components.
- Public versioning must make old fixture migration explicit.

## Implementation sequence

- Write contract invariants and examples first.
- Add IDs based on construction history and semantic incidence, not rounded coordinates.
- Add codecs/schemas.
- Add negative validation cases for occurrence reuse, missing point refs and coordinate-only welding.
- Leave a migration adapter if current tests require V1 output during implementation.

## Acceptance criteria

- One ArrangementPoint can own at least three distinct occurrences.
- Two loops can share a point and remain distinct regions.
- Every directed halfedge references exactly one start and end occurrence.
- Serialization and canonical ordering are deterministic.

## Required tests and evidence

- Public contract round-trip.
- Generated schema tests.
- Negative contract tests.
- Canonical digest stability under set ordering.

## Deliverables

- Versioned contracts and schemas.
- Prose invariants.
- Migration note for arrangement implementation agent.

## Stop conditions

- Stop if point identity cannot be expressed without a new exact-equality registry; record the required internal service but do not redesign ExactScalar in this card.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
