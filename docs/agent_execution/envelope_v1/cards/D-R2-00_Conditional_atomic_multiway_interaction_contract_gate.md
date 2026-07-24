# D-R2-00 — Conditional atomic multiway interaction contract gate

Status: **CONDITIONAL**  
Phase: `Session D`  
Dependencies: `C-R2C-04`  
Primary role: `Interaction contract author`  
Parallel group: `none`  
Relative size: `M`

## Copy-paste start prompt

```text
You are the implementation agent for `D-R2-00` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

After point-contact topology is fixed, three or more InteractionComponents may meet at one exact alpha/point/locus. Pairwise sequential resolution would be order-dependent.

## Objective

If and only if the C-R2C field gate exposes a multiway failure, define the atomic same-alpha contract or preserve a named unsupported outcome.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- kernel/src/cftuv_envelope/interactions/**
- docs/envelope_ec2_5_interaction_resolver.md
- C-R2C field receipt

## Allowed paths

- docs/session_d_r2_multiway_contract.md
- kernel/src/cftuv_envelope/interactions/contracts.py
- kernel/fixtures/session_d_interactions_v1/**
- kernel/tests/test_session_d_interaction_edges.py

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- Do not implement A-vs-B then winner-vs-C.
- Do not change Envelope geometry or ownership.
- Do not activate this card without a reproducible multiway fixture.

## Required design

- Same-alpha participants form one atomic component/hypergraph.
- The contract states whether the v1 case is supported and how union preservation/equality loci work.
- Unsupported cases retain `MULTIWAY_INTERACTION_POLICY_UNPROVEN`.

## Implementation sequence

- Minimize the field failure.
- Write before/at/after semantic cases.
- Prove permutation invariance.
- Obtain product-owner acceptance before implementation.

## Acceptance criteria

- All participant permutations produce the same result or the same named unsupported outcome.
- No new matter or gap is introduced.
- RawCoverage union preservation is explicit.

## Required tests and evidence

- Permutation matrix.
- Before/at/after exact fixtures.
- Contract validator.

## Deliverables

- Accepted multiway ADR and fixtures, or a formal decision to remain unsupported.

## Stop conditions

- Stop if ownership is required to define interaction geometry; keep stages separate.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
