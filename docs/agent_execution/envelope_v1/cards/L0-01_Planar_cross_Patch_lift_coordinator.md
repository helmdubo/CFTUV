# L0-01 — Planar cross-Patch lift coordinator

Status: **BLOCKED**  
Phase: `Cross-Patch lift`  
Dependencies: `F2-01, H0-01`  
Primary role: `Cross-Patch geometry contract author/implementer`  
Parallel group: `none`  
Relative size: `L`

## Copy-paste start prompt

```text
You are the implementation agent for `L0-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

Propagation is per PatchDomain, but two uses of one physical seam require coordinated extrinsic lifted geometry before final materialization.

## Objective

Define and implement planar cross-Patch lift with shared semantic anchors and named ambiguous cases.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted GeometryBatch handoff
- docs/envelope_external_agent_session_manifest.md cross-Patch lift section
- AnalysisSnapshot ChainUse/PhysicalChain contracts

## Allowed paths

- new kernel/src/cftuv_envelope/cross_patch_lift/**
- kernel contracts/schemas for lift
- host lift adapter
- kernel/tests/test_cross_patch_lift.py
- tests/blender/test_cross_patch_lift.py

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- Do not introduce cross-Patch propagation collision.
- Do not average normals by default.
- Do not alter per-Patch coverage.

## Required design

- Two uses of one physical seam share one lift relation and semantic anchor.
- For two planar patches, coordinate compatible lifted lines from the relevant offset-plane intersection under an explicit law.
- Ambiguous/parallel/degenerate cases are named outcomes.

## Implementation sequence

- Define lift inputs/outputs and identities.
- Implement supported two-plane case.
- Coordinate per-Patch GeometryBatch boundary occurrences.
- Add junction anchor handling only for explicitly supported cases.

## Acceptance criteria

- Shared seam occurrences agree in 3D where contract says shared.
- Per-Patch ownership/UV identities remain distinct as needed.
- No cracks in supported planar fixtures.
- Ambiguous cases fail closed.

## Required tests and evidence

- Two planar patches at several dihedral angles.
- Parallel/degenerate offset planes.
- Cross-Patch junction fixture.

## Deliverables

- Lift contracts and coordinator.
- Kernel/Blender tests.
- Named-failure catalog.

## Stop conditions

- Stop before inventing a general curved offset-surface intersection law.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
