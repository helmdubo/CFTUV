# R0-01 — Runtime benchmark corpus and SLA harness

Status: **BLOCKED**  
Phase: `Runtime`  
Dependencies: `F2-01`  
Primary role: `Performance engineer`  
Parallel group: `none`  
Relative size: `M`

## Copy-paste start prompt

```text
You are the implementation agent for `R0-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

Optimization must be driven by target-scale measurements, not microbenchmarks. The product target is approximately 2500–3000 seam edges and up to 50 patches.

## Objective

Create deterministic cold/warm/drag benchmarks and report both 16.7 ms and 33.3 ms frame budgets until a product SLA is selected.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- M-R1 field timing artifacts
- accepted StaticPatchProgram/EvaluationState APIs
- accepted GeometryBatch pipeline

## Allowed paths

- kernel/benchmarks/**
- tools/benchmark_envelope_*.py
- artifacts/envelope_runtime_benchmarks/**
- CI benchmark workflow in non-blocking/report mode initially

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- Do not change geometry while building benchmarks.
- Do not report only averages.
- Do not omit event-crossing or backward drag frames.

## Required design

- Scales: 100/500/1000/3000 edges and 1/10/50 patches.
- Phases: cold analysis, static compile, first state, between-event drag, event frame, backward drag, selection change, GPU upload, BMesh transaction.
- Metrics: p50/p95/p99, peak memory, rebuilt patches, exact fallback fraction, cell/halfedge/event counts.

## Implementation sequence

- Build synthetic deterministic corpus plus portable field fixture.
- Add benchmark runner and JSON schema.
- Capture reference baseline.
- Define regression thresholds separately from final product SLA.

## Acceptance criteria

- One command produces machine-readable and human-readable reports.
- Reports are reproducible and include hardware/runtime metadata.
- All target scales are represented.

## Required tests and evidence

- Benchmark schema validation.
- Repeated-run variance report.
- No semantic digest differences under instrumentation.

## Deliverables

- Benchmark harness.
- Baseline receipts.
- SLA decision input.

## Stop conditions

- Stop before declaring real-time success without p95/p99 target-scale evidence.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
