# C-R2D-01 — Internal labeled coverage cell-complex IR

Status: **BLOCKED**  
Phase: `C-R2D`  
Dependencies: `C-R2C-04`  
Primary role: `Kernel data-model author`  
Parallel group: `none`  
Relative size: `L`

## Copy-paste start prompt

```text
You are the implementation agent for `C-R2D-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

Public RawCoverage DTOs are currently used as an internal computational IR and then converted back to PlanarRegions. The replacement needs a persistent, labeled cell complex.

## Objective

Define an internal IR for arrangement points, occurrences, halfedges, faces, labels and provenance handles, with explicit Raw/Resolved/Ownership stage views.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted C-R2C handoff
- kernel/src/cftuv_envelope/reference/arrangement.py
- kernel/src/cftuv_envelope/reference/raw_coverage.py
- kernel/src/cftuv_envelope/interactions/policy_b.py

## Allowed paths

- new kernel/src/cftuv_envelope/cell_complex/**
- kernel/src/cftuv_envelope/reference/arrangement_protocol.py
- kernel/tests/test_cell_complex_contracts.py
- docs/session_c_r2d_cell_complex.md

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- Do not create one monolithic type containing ownership, UV and tessellation algorithms.
- Do not remove current DTO outputs yet.
- Do not redesign ExactScalar.

## Required design

- Core entities: Point, Occurrence, DirectedHalfEdge, Face, connected component, label sets, construction/provenance handles.
- Stage views: RawCoverageComplex, ResolvedCoverageComplex, OwnershipComplex.
- Derived stages may add typed 1D/0D entities and labels while sharing immutable geometry where valid.
- Separate static PatchDomain topology from alpha-dependent coverage state in the lifecycle design, even though C-R2E changes public compilation later.

## Implementation sequence

- Write invariants and mutation/derivation model.
- Define internal IDs and interning boundaries.
- Add validation and deterministic iteration.
- Add adapters from the current arrangement build state for test-only use.

## Acceptance criteria

- Cell complex represents multiple occurrences at one point.
- Faces carry domain/contributor labels.
- Stage derivation cannot mutate predecessor records.
- DTO projection is possible without recomputing geometry.

## Required tests and evidence

- Construction/validation tests.
- Deterministic serialization of debug projection, if provided.
- Point-contact and hole complex fixtures.

## Deliverables

- Internal IR.
- Design ADR.
- Validation and test fixtures.

## Stop conditions

- Stop if public ownership semantics are needed; store generic typed labels and defer E0 decisions.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
