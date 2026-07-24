# D-R2-00 — Conditional atomic multiway interaction contract gate

Status: **READY_AFTER_DOC_AND_FIX**\
Phase: `Session D`  
Dependencies: `DOC-00, FIX-00`\
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

The selected baseline already fixes point-contact topology. Its accepted
`building.002` field receipt reaches RawCoverage and then returns
`MULTIWAY_INTERACTION_POLICY_UNPROVEN`. Three or more InteractionComponents may
meet at one exact alpha/point/locus; pairwise sequential resolution would be
order-dependent.

## Objective

Define the atomic same-alpha contract for the reproduced field case, or
formally preserve the named unsupported outcome. The activation condition is
already met by the accepted C-R2C field receipt.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- BASELINE_RECONCILIATION.md from this execution pack
- accepted handoff from every dependency card
- docs/session_c_r2c_regularized_boundary_rotation_handoff.md
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
- Do not start implementation until `FIX-00` provides the portable multiway
  fixture and `DOC-00` is accepted.

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
