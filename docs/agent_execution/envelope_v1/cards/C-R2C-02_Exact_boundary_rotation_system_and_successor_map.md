# C-R2C-02 — Exact boundary rotation system and successor map

Status: **SUPERSEDED_BY_ACCEPTED_C_R2C_GATE — DO NOT EXECUTE**\
Phase: `C-R2C`  
Dependencies: `C-R2C-01`  
Primary role: `Kernel geometry implementer`  
Parallel group: `none`  
Relative size: `L`

Satisfied by the consolidated C-R2C implementation at
`43e69d3889d273ed19daee9239ae0e311a1b213d`, handoff
`979870f17d7890d96f008609966bdeb24d8b0b58`, and selected baseline
`c2622d07020338e5231b81f41655fe6c74cdca72`.

This decomposition is retained as historical planning evidence. It is not an
active task. Any new rotation-system delta requires a new card.

## Non-execution notice

```text
Do not execute C-R2C-02. Read the accepted consolidated C-R2C handoff:
docs/session_c_r2c_regularized_boundary_rotation_handoff.md
```

## Context

At plan-drafting time, the loop walker required one outgoing edge from every
merged vertex. The selected baseline now builds exact circular ray order and a
covered-left successor per incoming halfedge.

## Objective

Implement exact circular ray ordering and a deterministic face-successor map over boundary occurrences.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted C-R2C-01 contracts/handoff
- kernel/src/cftuv_envelope/reference/arrangement.py
- kernel/src/cftuv_envelope/reference/planar_types.py
- kernel/tests/test_reference_arrangement_history.py

## Allowed paths

- kernel/src/cftuv_envelope/reference/arrangement.py
- kernel/src/cftuv_envelope/reference/planar_types.py only for reusable exact predicates
- new kernel/src/cftuv_envelope/reference/rotation_system.py
- kernel/tests/test_reference_rotation_system.py
- kernel/tests/test_reference_arrangement_history.py

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- No atan2(float), angle epsilon, coordinate perturbation or ID tie-break.
- Do not group holes/regions yet beyond what is needed to test successor topology.

## Required design

- Classify rays by exact half-plane/quadrant.
- Order within a half-plane by exact cross sign and resolve collinearity by exact dot/direction semantics.
- At each ArrangementPoint, pair every incoming boundary halfedge with the outgoing halfedge bounding the same covered-left sector.
- Geometrically coincident rays require a typed coincident-ray relation; ID may only stabilize serialization after geometric equivalence is proved.

## Implementation sequence

- Extract oriented output halfedges before public vertices are emitted.
- Build an exact point registry that groups by proven point equality, not unchecked expression-string equality.
- Build incident ray lists and rotation order.
- Construct successor map keyed by incoming halfedge occurrence.
- Validate bijection and covered-left invariant.

## Acceptance criteria

- Every emitted boundary halfedge has exactly one successor and one predecessor.
- Point-touch fixtures with degree 4/6 succeed without merging occurrences.
- Successor map is invariant under input segment order, rigid rotation and retriangulation.
- Ambiguous coincident sectors return a named exact outcome, not an arbitrary pairing.

## Required tests and evidence

- Synthetic degree-2, degree-4 and degree-6 point contacts.
- Rotation/reversal metamorphic tests.
- Broadphase and exhaustive arrangement inputs yield identical successor maps.
- Exact collinear-ray cases.

## Deliverables

- Rotation-system module.
- Successor-map tests.
- Counters for points, occurrences, incident rays and ambiguous sectors.

## Stop conditions

- Stop if the covered side of an atomic edge is not provable.
- Stop if a new multiway interaction semantic is needed; arrangement only represents topology.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
