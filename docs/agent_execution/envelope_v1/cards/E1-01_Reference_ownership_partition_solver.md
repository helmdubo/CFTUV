# E1-01 — Reference ownership partition solver

Status: **BLOCKED**  
Phase: `Session E`  
Dependencies: `E0-01`  
Primary role: `Ownership geometry implementer`  
Parallel group: `none`  
Relative size: `XL`

## Copy-paste start prompt

```text
You are the implementation agent for `E1-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

ResolvedCoverage now has a dimension-aware target contract. The reference solver must partition existing matter without changing its silhouette.

## Objective

Build exact ownership claims over 2-cells and explicit interfaces from source/FrontReading/arrival facts.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted E0 contract
- kernel/src/cftuv_envelope/interactions/contracts.py
- kernel/src/cftuv_envelope/contracts/ownership.py
- kernel/src/cftuv_envelope/cell_complex/**

## Allowed paths

- new kernel/src/cftuv_envelope/ownership/**
- kernel/src/cftuv_envelope/cell_complex/**
- kernel/tests/test_reference_ownership.py
- kernel/fixtures/session_e_ownership_v1/**

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- No nearest-source recovery.
- No priority by record order or ID.
- No geometry creation/deletion.

## Required design

- Claims derive from accepted FrontReading/Envelope contributor and arrival laws.
- Equality loci become explicit interfaces.
- Ambiguous claims return a named ownership outcome.
- Ownership attaches to existing faces and may split faces only along certified semantic interfaces.

## Implementation sequence

- Compile claimant candidates per face.
- Insert required certified interfaces.
- Assign one owner to every supported open face.
- Validate totality/disjointness/closure.
- Compute ownership semantic digest.

## Acceptance criteria

- Silhouette geometry digest is unchanged.
- Every open 2-cell has exactly one owner or the entire result is a named unsupported outcome.
- Interfaces are ownerless where required.
- Provenance is complete.

## Required tests and evidence

- Straight strip/cap/angular ownership.
- Same-request Policy B equality interface.
- Self-contact.
- Point-contact and multiple occurrence cases.
- Permutation/metamorphic tests.

## Deliverables

- OwnershipComplex and public partition result.
- Reference fixtures.
- Proof/validation report.

## Stop conditions

- Stop on an unapproved owner tournament or junction claimant ambiguity.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
