# C-R2C-04 — Occurrence propagation, field gate and independent verification

Status: **BLOCKED**  
Phase: `C-R2C`  
Dependencies: `C-R2C-03`  
Primary role: `Independent verifier plus host adapter author`  
Parallel group: `none`  
Relative size: `M`

## Copy-paste start prompt

```text
You are the implementation agent for `C-R2C-04` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

Kernel topology is not complete until occurrence identities survive debug projection and the portable/field fixture passes.

## Objective

Validate the full C-R2C slice, propagate occurrence IDs to debug output, and close the `building.002` RawCoverage blocker without touching interactions.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted C-R2C-03 handoff
- portable FIX-00 fixture
- cftuv/envelope_debug_renderer.py
- cftuv/envelope_topology_debug.py
- tests/test_envelope_debug_renderer_contract.py

## Allowed paths

- kernel tests and fixture files
- kernel/src/cftuv_envelope/debug_scene.py
- cftuv/envelope_debug_renderer.py
- cftuv/envelope_topology_debug.py
- tests/test_envelope_debug_renderer_contract.py
- artifacts/envelope_c_r2c/**

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- No geometry repair in the renderer.
- No interaction-policy edits.
- No coordinate welding in debug projection.

## Required design

- Debug records may display coincident points, but must retain distinct occurrence IDs.
- The field gate ends at RawCoverage. A subsequent `MULTIWAY_INTERACTION_POLICY_UNPROVEN` is a valid separate finding.

## Implementation sequence

- Run the portable fixture.
- Run the actual Blender fixture if available.
- Compare broadphase and exhaustive outputs including occurrences and point contacts.
- Check source mesh fingerprint.
- Audit all consumers of `RawCoverageVertexV1` for coordinate-based welding.

## Acceptance criteria

- `building.002`, selected physical edges 2/3/7: Patch 0 reaches `RAW_READY`.
- `REFERENCE_ARRANGEMENT_NON_MANIFOLD` is absent for the point-contact case.
- Exact area and contributor provenance match the accepted expectation.
- Every halfedge occurrence is consumed once.
- Broadphase equals exhaustive oracle.
- Source mesh is unchanged.

## Required tests and evidence

- Full kernel wheel/extraction suite.
- Host debug bridge tests.
- Portable fixture gate.
- Blender field receipt if environment exists.

## Deliverables

- C-R2C handoff JSON/Markdown.
- Updated field receipt.
- Consumer audit list.

## Stop conditions

- Stop if interaction fails after RawCoverage; open D-R2-00 rather than patching arrangement.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
