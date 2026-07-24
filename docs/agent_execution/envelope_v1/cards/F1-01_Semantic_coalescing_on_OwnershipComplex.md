# F1-01 — Semantic coalescing on OwnershipComplex

Status: **BLOCKED**  
Phase: `Session F`  
Dependencies: `E2-01`  
Primary role: `Semantic arrangement agent`  
Parallel group: `none`  
Relative size: `L`

## Copy-paste start prompt

```text
You are the implementation agent for `F1-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

Before tessellation, adjacent cells may be merged only when all semantic records agree.

## Objective

Create a semantic coalescing stage that reduces representation complexity without erasing silhouette, ownership, UV, station, provenance or event interfaces.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted E2 handoff
- kernel/src/cftuv_envelope/contracts/tessellation.py
- kernel/src/cftuv_envelope/cell_complex/**

## Allowed paths

- new kernel/src/cftuv_envelope/semantic_arrangement/**
- kernel/tests/test_semantic_coalescing.py
- kernel/fixtures/session_f_semantic_v1/**

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- Do not dissolve hidden-support silhouette edges.
- Do not merge across owner/UV/station/provenance interfaces.
- Do not use face count as semantic authority.

## Required design

- Define a complete semantic equivalence key.
- Representation-only edges may be removed if both incident cells have equal semantic keys and topology remains valid.
- Semantic boundary types are preserved explicitly.

## Implementation sequence

- Define equivalence key and edge-removal rules.
- Coalesce faces/cycles.
- Preserve occurrence and interface mappings.
- Generate before/after semantic digest proof.

## Acceptance criteria

- Geometry and semantic digests are unchanged.
- Only representation complexity decreases.
- Point-contact occurrences are not welded.
- Coalescing is deterministic and idempotent.

## Required tests and evidence

- Idempotence.
- Permutation invariance.
- Reflex fan, equality interface, UV seam and point contact cases.

## Deliverables

- SemanticArrangement result.
- Coalescing proof report.

## Stop conditions

- Stop if an edge classification is missing; add a typed boundary role rather than guessing.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
