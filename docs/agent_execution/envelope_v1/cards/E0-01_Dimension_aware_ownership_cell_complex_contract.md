# E0-01 — Dimension-aware ownership cell-complex contract

Status: **BLOCKED**  
Phase: `E0`  
Dependencies: `C-R2F-01`  
Primary role: `Ownership contract author`  
Parallel group: `none`  
Relative size: `L`

## Copy-paste start prompt

```text
You are the implementation agent for `E0-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

The current ownership contract says total/disjoint while equality boundaries are ownerless. Ownership must be stated by dimension before a solver is written.

## Objective

Define ownership semantics for open 2-cells, 1D interfaces and 0D contacts/junctions, plus separate geometry/content/audit digests.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- kernel/src/cftuv_envelope/contracts/ownership.py
- accepted ResolvedCoverageComplex handoff
- kernel/src/cftuv_envelope/contracts/geometry_batch.py
- kernel/src/cftuv_envelope/canonical.py

## Allowed paths

- kernel/src/cftuv_envelope/contracts/ownership.py
- kernel/src/cftuv_envelope/contracts/geometry_batch.py
- kernel/src/cftuv_envelope/canonical.py
- kernel/src/cftuv_envelope/schema.py
- kernel/schema/**
- kernel/tests/test_public_contracts.py
- docs/session_e0_ownership_semantics.md

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- No ownership solver.
- No first-wins/lowest-ID claimant.
- No silhouette edits.

## Required design

- 2-cells: open material regions, exactly one owner where supported.
- 1-cells: equality/ownership/UV interfaces, may be ownerless and have left/right incident claims.
- 0-cells: point contacts, junctions, event anchors, distinct semantic entities.
- Closures of owned 2-cells plus interfaces cover ResolvedCoverage.
- Define `semantic_geometry_digest`, `record_content_digest`, `audit_receipt_digest`.

## Implementation sequence

- Write formal obligations and examples.
- Version contracts and schemas.
- Define occurrence-aware future VertexKey requirements.
- Add negative cases for shared-coordinate welding and diagnostics changing geometry digest.
- Obtain explicit user acceptance.

## Acceptance criteria

- Totality is unambiguous on open 2D interior.
- Ownerless interfaces do not violate the contract.
- Point-only contacts remain separate occurrences.
- Changing diagnostics/timings does not change semantic geometry digest.

## Required tests and evidence

- Contract validators.
- Digest mutation tests.
- Example complexes with equality interface and point contact.

## Deliverables

- Accepted E0 ADR/contracts.
- Generated schemas.
- Digest migration guide.

## Stop conditions

- Stop if claimant priority is not specified by accepted source/arrival facts.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
