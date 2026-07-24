# C-R2D-03 — RawCoverage DTO projection and oracle retirement gate

Status: **BLOCKED**  
Phase: `C-R2D`  
Dependencies: `C-R2D-02`  
Primary role: `Independent verifier`  
Parallel group: `none`  
Relative size: `M`

## Copy-paste start prompt

```text
You are the implementation agent for `C-R2D-03` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

The labeled complex must become the compute authority while preserving stable public stage records and permanent differential evidence.

## Objective

Project the cell complex to versioned RawCoverage DTOs, prove equality, and remove public-DTO-to-PlanarRegion round-trips from the active path.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted C-R2D-02 handoff
- kernel/src/cftuv_envelope/reference/contracts.py
- kernel/src/cftuv_envelope/reference/digest.py
- kernel/src/cftuv_envelope/reference/raw_coverage.py

## Allowed paths

- RawCoverage DTO adapter modules
- kernel/src/cftuv_envelope/reference/digest.py
- kernel/tests/test_reference_boundary_union.py
- kernel/tests/test_labeled_raw_overlay.py
- artifacts/envelope_c_r2d/**

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- Do not hide record differences by excluding fields from the semantic digest without an ADR.
- Do not remove the exhaustive arrangement oracle.

## Required design

- DTOs are stage-boundary projections.
- Occurrence/point-contact records are included in the versioned semantic projection.
- Diagnostics and timings are not geometry identity.

## Implementation sequence

- Implement deterministic projection.
- Split geometry/content/audit digest responsibilities where needed, or defer the full three-digest migration to E0 with an explicit bridge.
- Run old-vs-new complete record differential.
- Remove active `_arrangement_regions` round-trip from RawCoverage.

## Acceptance criteria

- Complete record equality for unchanged schema fields.
- Explicit versioned differences for new occurrence records.
- No active N+1 path in normal evaluation.
- Permanent fixture-only oracle remains callable.

## Required tests and evidence

- Corpus differential.
- Digest mutation tests.
- Wheel/extraction CI.
- Field fixture timing and correctness.

## Deliverables

- C-R2D accepted handoff.
- Old/new differential report.
- Updated schemas.

## Stop conditions

- Stop if digest compatibility cannot be made explicit; version the contract rather than silently changing identity.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
