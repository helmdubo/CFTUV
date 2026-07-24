# R2-01 — Filtered topology predicates with exact fallback

Status: **BLOCKED**  
Phase: `Runtime`  
Dependencies: `R1-01`  
Primary role: `Numeric runtime implementer`  
Parallel group: `none`  
Relative size: `XL`

## Copy-paste start prompt

```text
You are the implementation agent for `R2-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

The current RuntimeMetric path is mainly a preflight and still calls the exact reference arrangement. A real runtime must filter the predicates that decide topology.

## Objective

Implement certified binary64 filters for hot topology predicates with authoritative exact fallback and no float-derived semantic IDs.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- kernel/src/cftuv_envelope/runtime_metric.py
- kernel/src/cftuv_envelope/planar_metric.py
- accepted event ledger
- R0 benchmark report

## Allowed paths

- kernel/src/cftuv_envelope/runtime_metric.py
- kernel/src/cftuv_envelope/runtime/**
- kernel/tests/test_runtime_predicate_differential.py
- artifacts/envelope_runtime_r2/**

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- No rationalization of a rounded runtime frame as exact.
- No uncertain equality resolved by epsilon.
- No semantic construction record emitted from float-only data.

## Required design

- Filter actual hot predicates: orientation, segment intersection, support-side classification, circular order, point/edge ordering, event-alpha comparison, equality-locus clipping and face classification.
- Outward error bounds certify only strict signs; uncertainty falls back to exact metric facts.

## Implementation sequence

- Profile exact predicate call sites.
- Implement one predicate family at a time with differential tests.
- Add fallback reason/counter telemetry.
- Integrate with event ledger and cell complex.

## Acceptance criteria

- Complete semantic equality to reference.
- Every uncertain result uses exact fallback.
- Fallback fraction and time are reported by predicate family.
- No new named failure caused by float authority.

## Required tests and evidence

- Adversarial near-zero predicates.
- Scale/rotation/retriangulation metamorphic tests.
- Event-order differential.
- Target-scale benchmarks.

## Deliverables

- Filtered predicate service.
- Differential corpus.
- Fallback telemetry.

## Stop conditions

- Stop if a construction cannot be made exact from accepted source facts.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
