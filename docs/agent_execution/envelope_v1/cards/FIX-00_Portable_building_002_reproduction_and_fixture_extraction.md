# FIX-00 — Portable building.002 reproduction and fixture extraction

Status: **READY_AFTER_BASE**  
Phase: `Baseline`  
Dependencies: `BASE-00`  
Primary role: `Host fixture curator`  
Parallel group: `baseline-parallel`  
Relative size: `M`

## Copy-paste start prompt

```text
You are the implementation agent for `FIX-00` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

The historical point-contact blocker was captured from a developer-local
Blender scene. The selected baseline `c2622d0...` already reaches RawCoverage;
kernel agents still need a hermetic reproduction that does not depend on
`E:\testscene.blend` and can be replayed against both immutable historical
states.

## Objective

Create a portable serialized `AnalysisSnapshotV1 + DecalRequestV1` reproducer and a minimal Blender-side fixture or scene generator for the selected edges 2, 3 and 7.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- artifacts/envelope_runtime_r1/building_002.json
- artifacts/envelope_runtime_r1/patch0_diagnostic.json
- tools/run_envelope_mr1_building_gate.py
- cftuv/envelope_topology_export.py
- cftuv/envelope_request_export.py

## Allowed paths

- artifacts/envelope_c_r2c_fixture/**
- kernel/fixtures/building_002_point_contact_v1/**
- kernel/tests/test_building_002_point_contact_fixture.py
- tools/export_*fixture*.py
- tests/blender/** for the minimal fixture/generator

## Forbidden work

- Do not simplify the geometry until the same construction point, contributors and failure are proven.
- Do not patch arrangement behavior.
- Do not include proprietary/unnecessary scene content.
- Do not read legacy geometry beyond accepted factual export code.

## Required design

- The kernel fixture must reproduce the exact pre-fix outcome at
  `df587ed...`, the accepted post-fix outcome at `c2622d0...`, and preserve
  stable source IDs.
- The Blender fixture/generator is evidence for host extraction, not semantic authority.
- Store source mesh fingerprints and selected ChainUse/PhysicalChain IDs.

## Implementation sequence

- Export the smallest accepted snapshot/request pair from the field evidence.
- Minimize while repeatedly checking the same failure and construction histories.
- Add fixture schema/hash manifest.
- Add one command that runs the kernel reproduction.
- Add a Blender test or scene generator that recreates the host facts.

## Acceptance criteria

- At historical baseline `df587ed...`, the portable kernel fixture returns
  `REFERENCE_ARRANGEMENT_NON_MANIFOLD` at the same point-contact topology.
- At selected baseline `c2622d0...`, the same immutable fixture reaches
  RawCoverage V2 with the accepted occurrence/contact facts.
- Fixture hash and source mesh fingerprint are stable.

## Required tests and evidence

- Kernel fixture test before and after fix.
- Snapshot/request validation.
- Host export equality or documented normalized equality.
- Source mesh unchanged check.

## Deliverables

- Portable kernel fixture.
- Minimal Blender fixture or deterministic scene generator.
- Fixture manifest and reproduction command.

## Stop conditions

- Stop if minimization changes the event from point-only contact to positive-area overlap.
- Stop if source IDs must be regenerated nondeterministically.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
