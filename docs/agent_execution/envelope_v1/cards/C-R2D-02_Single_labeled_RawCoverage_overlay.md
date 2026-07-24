# C-R2D-02 — Single labeled RawCoverage overlay

Status: **BLOCKED**  
Phase: `C-R2D`  
Dependencies: `C-R2D-01`  
Primary role: `Kernel geometry implementer`  
Parallel group: `none`  
Relative size: `XL`

## Copy-paste start prompt

```text
You are the implementation agent for `C-R2D-02` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

Current RawCoverage clips every Envelope with an exact arrangement and then performs another union. This repeats splitting, classification, DTO emission and parsing.

## Objective

Build one labeled overlay from PatchDomain boundaries and all Envelope boundaries, classify faces once, and derive both silhouette and per-contributor regions.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted C-R2D-01 IR
- kernel/src/cftuv_envelope/reference/raw_coverage.py
- kernel/src/cftuv_envelope/reference/arrangement.py
- kernel/src/cftuv_envelope/reference/boundary.py

## Allowed paths

- kernel/src/cftuv_envelope/cell_complex/**
- kernel/src/cftuv_envelope/reference/raw_coverage.py
- kernel/src/cftuv_envelope/reference/arrangement.py
- kernel/tests/test_labeled_raw_overlay.py
- kernel/tools/benchmark_*coverage*.py

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- Do not drop per-Envelope boundary-capacity semantics.
- Do not classify topology by float.
- Do not delete the old exact path until differential gates pass.

## Required design

- Inputs carry DOMAIN/CONTRIBUTION and contributor labels.
- Atomic edges are built once.
- Faces are classified for domain inclusion and contributor coverage.
- Per-contributor clipping, exposed fronts and reachability are read from labels.
- Angular boundary-contact and barrier-bypass outcomes remain exactly as specified.

## Implementation sequence

- Add all boundaries to one arrangement build.
- Classify faces and boundary halfedges.
- Derive clipped contribution views without DTO round-trip.
- Derive RawCoverage silhouette and regions.
- Retain the existing N+1 path as a test oracle during migration.

## Acceptance criteria

- Supported fixtures produce equal area, topology, provenance and semantic digest.
- Point occurrences are preserved.
- No separate exact arrangement is invoked per Envelope on the new path.
- Telemetry reports one Raw overlay build.

## Required tests and evidence

- Full Session C corpus differential.
- Broadphase vs exhaustive differential.
- Boundary-capacity cases.
- Performance counters versus baseline.

## Deliverables

- Labeled Raw overlay implementation.
- Old-path oracle switch for tests.
- Benchmark receipt.

## Stop conditions

- Stop if an existing boundary behavior cannot be derived from labels; capture a semantic gap instead of emulating it with post-hoc polygon repair.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
