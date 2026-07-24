# R3-01 — Exact-value, provenance and digest interning optimization

Status: **CONDITIONAL**  
Phase: `Runtime`  
Dependencies: `R2-01`  
Primary role: `Performance/data-structure engineer`  
Parallel group: `none`  
Relative size: `XL`

## Copy-paste start prompt

```text
You are the implementation agent for `R3-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

ExactScalar string parse/normalize cycles and repeated provenance sets may become hot, but a large rewrite is justified only by profiling.

## Objective

If R0/R2 profiling proves these costs material, introduce internal exact/provenance handles while preserving public serialization and semantic equality.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- R0/R2 profiling receipts
- kernel/src/cftuv_envelope/reference/planar_types.py
- kernel/src/cftuv_envelope/reference/provenance.py
- kernel/src/cftuv_envelope/canonical.py

## Allowed paths

- new internal exact arena/provenance pool modules
- canonical serialization adapters
- kernel/tests/test_exact_value_arena.py
- kernel/tests/test_provenance_interning.py

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- Do not activate without a measured bottleneck.
- Do not change public exact expression semantics silently.
- Do not use expression-string identity as mathematical equality.

## Required design

- Internal hash-consed exact values with cached sign/equality where valid.
- Interned provenance DAG/handles.
- Structural/Merkle hashes for runtime; versioned canonical strings at serialization boundary.

## Implementation sequence

- Write a profiling-based ADR.
- Build adapters behind current public records.
- Migrate one hot path at a time.
- Run complete record/digest differential.

## Acceptance criteria

- Reference records and public digests are unchanged or explicitly versioned.
- Measured target workload improves materially.
- Memory and parse/normalization counts decrease.

## Required tests and evidence

- Mathematically equal expressions with different forms.
- Serialization compatibility.
- Full corpus differential.
- Performance report.

## Deliverables

- Optimization implementation.
- ADR with before/after numbers.

## Stop conditions

- Stop if optimization requires changing semantic contracts; return to the responsible earlier gate.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
