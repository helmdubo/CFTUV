# HOST-ANG-00 — Exact angular certificate export for non-rational pi ratios

Status: **AUTHORIZED_BY_PRODUCT_OWNER**\
Phase: `Host Adapter`\
Dependencies: `FIX-00`\
Primary role: `Host Adapter Author`\
Parallel group: `none`\
Relative size: `M`

## Authorization

The product owner explicitly authorized this separate card on 2026-07-24
after FIX-00 recorded
`ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE` from the current host
exporter. This is a Host Adapter correction, not a Kernel or Session D
semantic amendment.

## Context

The current adapter correctly derives reflex angles from exact owner support
directions. It then rejects any exact angle whose ratio to pi is not rational,
although the public contract requires certified decimal bounds rather than a
proof that the ratio itself is rational. The accepted `building.002` field
case exposes this accidental restriction.

## Objective

Export exact-predicate-certified decimal intervals for support-derived angular
ratios, including non-rational ratios, while preserving the named fail-closed
outcome whenever the certificate cannot be proved.

## Required reading

- AGENTS.md
- docs/agent_execution/envelope_v1/01_GLOBAL_CANON.md
- docs/agent_execution/envelope_v1/02_AGENT_PROTOCOL.md
- docs/agent_execution/envelope_v1/cards/FIX-00_Portable_building_002_reproduction_and_fixture_extraction.md
- accepted FIX-00 handoff
- docs/envelope_v0_debug_bridge.md
- cftuv/envelope_request_export.py
- tests/test_envelope_host_adapter.py
- tools/export_building_002_point_contact_fixture.py

## Allowed paths

- docs/agent_execution/envelope_v1/README.md
- docs/agent_execution/envelope_v1/CHANGELOG.md
- docs/agent_execution/envelope_v1/task_manifest.json
- docs/agent_execution/envelope_v1/cards/HOST-ANG-00_Exact_angular_certificate_export.md
- docs/envelope_v0_debug_bridge.md
- cftuv/envelope_request_export.py
- tests/test_envelope_host_adapter.py
- tests/blender/test_building_002_point_contact_fixture.py
- artifacts/envelope_host_angular_cert_00/**

## Forbidden work

- Do not read or invoke `cftuv/decal_voronoi.py`.
- Do not read legacy geometry sections of `cftuv/decals.py`.
- Do not modify Kernel contracts or implementations.
- Do not use the approximate host `turn_angle_deg` as authority.
- No tolerance, snapping, rounded-coordinate identity or silent repair.
- Do not modify the accepted portable FIX-00 fixture to fit the exporter.
- Do not start C-R2D, D-R3 or later cards.

## Required design

- The owner support directions remain the exact angular authority.
- A finite decimal interval is emitted only after both outward bounds are
  proved against exact support geometry.
- The reflex-excess interval is certified first; the `phi/pi` interval is its
  exact translation by one.
- The rational-ratio fast path remains valid.
- An undecidable exact comparison returns
  `ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE`.

## Acceptance criteria

- A non-rational exact angular-ratio regression is certified without fallback.
- Existing K0/K1/K2 angular host tests remain exact and pass.
- The current `building.002` exporter passes the angular certificate stage.
  Its request and all non-angular snapshot records reproduce FIX-00; newly
  admitted exact angular records and their downstream result are recorded as
  an explicit difference from the older approximate-angle exporter.
- Source mesh and accepted FIX-00 fixture bytes are unchanged.

## Required tests and evidence

- Focused host adapter tests.
- Full host bridge test suite.
- Current Blender exporter run against the deterministic `building.002`
  fixture.
- Exact comparison evidence for both interval bounds.
- Differential evidence separating new angular records from unchanged
  snapshot/request records.
- Source mesh and fixture digest comparison.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and
handoff SHAs; the field command and Blender version; full test results; exact
bound-certificate evidence; named unsupported outcomes; source mesh mutation
check; changed paths; and the next-agent allowlist.
