# C-R2C-01 — Occurrence-aware arrangement contracts

Status: **SUPERSEDED_BY_ACCEPTED_C_R2C_GATE — DO NOT EXECUTE**\
Phase: `C-R2C`  
Dependencies: `BASE-00, FIX-00`  
Primary role: `Kernel contract author`  
Parallel group: `none`  
Relative size: `M`

Satisfied by the consolidated C-R2C implementation at
`43e69d3889d273ed19daee9239ae0e311a1b213d`, handoff
`979870f17d7890d96f008609966bdeb24d8b0b58`, and selected baseline
`c2622d07020338e5231b81f41655fe6c74cdca72`.

This decomposition is retained as historical planning evidence. It is not an
active task, and differences in type names do not authorize a new
implementation branch. Any remaining contract delta requires a new card and
explicit acceptance scope.

## Non-execution notice

```text
Do not execute C-R2C-01. Read the accepted consolidated C-R2C handoff:
docs/session_c_r2c_regularized_boundary_rotation_handoff.md
```

## Context

At plan-drafting time, the arrangement stored one vertex per exact coordinate
and lost boundary occurrences at point contacts. The selected baseline now
separates `RawCoverageVertexV1`, `BoundaryVertexOccurrenceV1`, and
`PointContactRecordV1`.

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
