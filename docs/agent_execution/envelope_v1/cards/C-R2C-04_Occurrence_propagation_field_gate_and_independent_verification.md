# C-R2C-04 — Occurrence propagation, field gate and independent verification

Status: **SUPERSEDED_BY_ACCEPTED_C_R2C_GATE — DO NOT EXECUTE**\
Phase: `C-R2C`  
Dependencies: `C-R2C-03`  
Primary role: `Independent verifier plus host adapter author`  
Parallel group: `none`  
Relative size: `M`

Satisfied by the consolidated C-R2C implementation at
`43e69d3889d273ed19daee9239ae0e311a1b213d`, handoff
`979870f17d7890d96f008609966bdeb24d8b0b58`, host reflex export at
`c2622d07020338e5231b81f41655fe6c74cdca72`, and CI run
`30095478731`.

This decomposition is retained as historical planning evidence. It is not an
active task. Any new host/field delta requires a new card.

## Non-execution notice

```text
Do not execute C-R2C-04. Read the accepted consolidated C-R2C handoff:
docs/session_c_r2c_regularized_boundary_rotation_handoff.md
```

## Context

The selected baseline propagates occurrence/contact records to debug projection
and passes the accepted `building.002` field gate through RawCoverage. The
portable fixture remains a separate `FIX-00` deliverable.

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
