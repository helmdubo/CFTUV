# E2-01 — Station and UV semantic facts

Status: **BLOCKED**  
Phase: `Session E`  
Dependencies: `E1-01`  
Primary role: `UV/station contract and implementation agent`  
Parallel group: `none`  
Relative size: `L`

## Copy-paste start prompt

```text
You are the implementation agent for `E2-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

Owners are known, but mesh emission needs stable longitudinal/transverse coordinates and explicit seam/interface behavior.

## Objective

Attach source station, transverse coordinate and UV facts to ownership cells and occurrences without coordinate welding or provenance reconstruction.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted E1 handoff
- kernel/src/cftuv_envelope/contracts/plan.py StationFlowV1
- kernel/src/cftuv_envelope/contracts/ownership.py
- kernel/src/cftuv_envelope/contracts/geometry_batch.py

## Allowed paths

- new kernel/src/cftuv_envelope/station_uv/**
- kernel contracts/schemas for station and UV
- kernel/tests/test_station_uv.py
- kernel/fixtures/session_e_uv_v1/**

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- No nearest projection to a source edge after Boolean.
- No UV-driven silhouette edits.
- No welding of distinct occurrences sharing coordinates.

## Required design

- Longitudinal `s` follows semantic ChainUse orientation and physical support intervals.
- Transverse `r` follows the accepted Envelope/owner law.
- Interfaces may duplicate topological vertices and UV values.
- Cross-Patch continuity is declared but coordinated later by L0.

## Implementation sequence

- Define station/UV records and IDs.
- Propagate exact/parametric facts through cells and occurrences.
- Define seam/interface discontinuity records.
- Validate orientation reversal invariance.

## Acceptance criteria

- Every owned output occurrence has complete station/UV facts or a named unsupported outcome.
- Distinct occurrences at one coordinate may carry different UVs.
- Storage reversal does not change semantic flow.
- Silhouette digest remains unchanged.

## Required tests and evidence

- Open/closed chains.
- Caps and reflex fans.
- Point contact and equality interface.
- Orientation/reversal metamorphic tests.

## Deliverables

- Station/UV contracts and implementation.
- Fixture corpus.
- Continuity/discontinuity report.

## Stop conditions

- Stop if a cross-Patch lift decision is required; emit a coordination requirement for L0.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
