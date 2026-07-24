# D-R3-01 — ResolvedCoverage as a derived cell complex

Status: **BLOCKED**  
Phase: `Session D hardening`  
Dependencies: `C-R2D-03, D-R2-00`\
Primary role: `Interaction geometry implementer`  
Parallel group: `none`  
Relative size: `XL`

## Copy-paste start prompt

```text
You are the implementation agent for `D-R3-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

Policy B currently clips PlanarRegions and rebuilds unions. Ownership should not start on a polygon round-trip representation.

## Objective

Apply approved equality loci and interaction labels to derive a ResolvedCoverageComplex while preserving Policy B semantics.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted C-R2D handoff
- kernel/src/cftuv_envelope/interactions/resolved_coverage.py
- kernel/src/cftuv_envelope/interactions/policy_b.py
- docs/envelope_ec2_5_interaction_resolver.md

## Allowed paths

- kernel/src/cftuv_envelope/interactions/**
- kernel/src/cftuv_envelope/cell_complex/**
- kernel/tests/test_session_d_*
- artifacts/envelope_d_r3/**

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- Do not change mutual-arrival or Policy B product semantics.
- Do not introduce ownership decisions.
- Do not sequence multiway events arbitrarily.

## Required design

- Insert equality/event loci as typed 1-cells.
- Split affected faces and assign resolved contributor labels.
- Preserve RawCoverage union in approved cases.
- Expose ResolvedCoverage DTO as a projection of the derived complex.

## Implementation sequence

- Map Policy B inputs to cell labels.
- Apply interaction applications atomically.
- Replace active `_arrangement_regions` and union rebuild where the complex can be derived.
- Keep previous policy implementation as differential oracle until accepted.

## Acceptance criteria

- Before/at/after interaction fixtures are exact.
- Resolved union equals RawCoverage in approved v1 cases.
- Equality loci are explicit ownerless candidates.
- No polygon DTO round-trip in active path.

## Required tests and evidence

- Session D corpus.
- Self-contact fixtures.
- Permutation tests for any supported multiway case.
- Digest and provenance equality.

## Deliverables

- ResolvedCoverageComplex.
- DTO adapter.
- Differential receipt.

## Stop conditions

- Stop on unproven multiway semantics and return the accepted named outcome.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
