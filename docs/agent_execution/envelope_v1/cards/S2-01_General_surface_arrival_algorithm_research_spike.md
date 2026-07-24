# S2-01 — General-surface arrival algorithm research spike

Status: **BLOCKED**  
Phase: `Curved research`  
Dependencies: `S0-01`  
Primary role: `Research agent`  
Parallel group: `none`  
Relative size: `XL`

## Copy-paste start prompt

```text
You are the implementation agent for `S2-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

Hemispheres, noisy and non-developable surfaces require intrinsic arrival and cut-locus handling. The reference articles do not supply a complete product algorithm.

## Objective

Compare candidate methods and produce an accepted SurfaceArrivalComplex contract and implementation recommendation, not production geometry.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted S0 gate
- research references in references/RESEARCH_MAPPING.md
- target-scale benchmark requirements

## Allowed paths

- research/surface_arrival/**
- docs/session_s2_surface_arrival_research.md
- prototype benchmarks outside production path
- fixtures for hemisphere/noise/ribbon

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- Do not merge prototype code into production evaluator.
- Do not hide approximation error.
- Do not use one scalar winner field when ownership needs multiple arrival candidates.

## Required design

- Compare multi-source FMM, Heat Method candidate fields, Surface Voronoi reconstruction and vector transport.
- Sources are curves/ChainUses; barriers are intrinsic constraints.
- Store best and additional arrival candidates sufficient for equality loci/cut locus.
- Define error, refinement and determinism contracts.

## Implementation sequence

- Build common fixture/metric harness.
- Prototype candidate methods.
- Measure accuracy, topology stability, source labels, memory and update cost.
- Propose one reference and one runtime strategy.
- Obtain product/architecture acceptance.

## Acceptance criteria

- Recommendation is supported by reproducible fixtures and numbers.
- Cut-locus/multi-arrival cases are represented explicitly.
- No silent smoothing or rail-constrained front.

## Required tests and evidence

- Hemisphere rotational invariance.
- Non-developable ribbon.
- Noise with raw and candidate regularized metrics.
- Mesh refinement.

## Deliverables

- Research report.
- Prototype code and data.
- Accepted SurfaceArrivalComplex proposal.

## Stop conditions

- Stop if no candidate meets semantic requirements; report gaps rather than choosing the fastest.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
