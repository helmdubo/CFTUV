# C-R2C-03 — Face extraction, loops and containment without vertex probes

Status: **BLOCKED**  
Phase: `C-R2C`  
Dependencies: `C-R2C-02`  
Primary role: `Kernel geometry implementer`  
Parallel group: `none`  
Relative size: `L`

## Copy-paste start prompt

```text
You are the implementation agent for `C-R2C-03` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

After successor construction, the arrangement must extract loops and area components without using a boundary vertex as an inside probe.

## Objective

Replace vertex-outdegree traversal and boundary-vertex containment probes with occurrence-based halfedge cycles, face adjacency and certified interior witnesses.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted C-R2C-02 handoff
- kernel/src/cftuv_envelope/reference/arrangement.py
- kernel/src/cftuv_envelope/reference/raw_coverage.py
- kernel/tests/test_reference_boundary_union.py

## Allowed paths

- kernel/src/cftuv_envelope/reference/arrangement.py
- new face/cell helper modules under kernel/src/cftuv_envelope/reference/
- kernel/tests/test_reference_boundary_union.py
- kernel/tests/test_reference_point_contacts.py

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- Do not infer containment from the first loop vertex.
- Do not collapse point-connected regions into one component.
- Do not alter interaction or owner semantics.

## Required design

- Traverse each directed halfedge occurrence exactly once.
- Construct boundary cycles from the successor map.
- Represent face/sector adjacency explicitly.
- Classify positive-area connected components through shared edges/faces, not shared points.
- Assign holes by face adjacency or an exact interior witness guaranteed not to lie on a boundary.

## Implementation sequence

- Remove `outdegree == 1` and single-candidate traversal assumptions.
- Create cycles and signed areas from occurrence points.
- Build outer/hole relations without boundary probes.
- Create PointContactRecords for shared ArrangementPoints across distinct cycles.
- Project to versioned RawCoverage records.

## Acceptance criteria

- Every boundary halfedge occurrence is consumed exactly once.
- No loop is open or repeated.
- Point-only contact does not merge area components.
- Holes are assigned correctly when a hole vertex touches another boundary point.
- Exact total area is preserved.

## Required tests and evidence

- Bow-tie point contact with two separate regions.
- Multiple U-shaped contributions meeting at one point.
- Hole touching another loop at one point.
- Nested holes and multiple outers.
- Input-order and rigid-transform metamorphic tests.

## Deliverables

- Face/loop extraction implementation.
- Point-contact records in RawCoverage.
- Removal report for boundary-vertex probes.

## Stop conditions

- Stop if a positive-area overlap creates a new interaction question; capture a fixture and defer to Session D.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
