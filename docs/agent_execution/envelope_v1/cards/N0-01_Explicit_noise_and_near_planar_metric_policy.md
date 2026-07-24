# N0-01 — Explicit noise and near-planar metric policy

Status: **BLOCKED**  
Phase: `Curved research`  
Dependencies: `S2-01`  
Primary role: `Product-policy and numeric contract author`  
Parallel group: `none`  
Relative size: `M`

## Copy-paste start prompt

```text
You are the implementation agent for `N0-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

Noise can be intended geometry or unwanted high-frequency detail. Near-planar projection is also currently a named unimplemented policy.

## Objective

Define explicit, versioned metric policies; never let a solver smooth or project silently.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- S2 research report
- ReferenceMetricV2 and planarity admission
- surface contracts

## Allowed paths

- docs/session_n0_metric_policy.md
- kernel metric policy contracts/schemas
- policy fixtures and validators

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- No hidden tolerance.
- No automatic smoothing selected by solver failure.
- No implementation of full surface evaluator in this card.

## Required design

- Policies may include exact source plane, certified near-planar projection, raw intrinsic metric, or named smoothed intrinsic metric.
- Every non-exact policy records scale, method, residual/error budget and source revision.

## Implementation sequence

- Write product choices and examples.
- Define certificates and named outcomes.
- Add policy validation fixtures.
- Obtain explicit owner acceptance.

## Acceptance criteria

- Same input/policy is deterministic.
- Different policies have different semantic identity.
- Out-of-budget inputs fail closed.

## Required tests and evidence

- Planar/near-planar/noise matrices.
- Certificate validation.

## Deliverables

- Accepted metric policy ADR/contracts.

## Stop conditions

- Stop before selecting smoothing scales without product approval.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
