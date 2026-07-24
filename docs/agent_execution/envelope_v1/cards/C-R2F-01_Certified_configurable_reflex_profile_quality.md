# C-R2F-01 — Certified configurable reflex profile quality

Status: **BLOCKED**  
Phase: `C-R2F`  
Dependencies: `C-R2E-03`  
Primary role: `Kernel contract and geometry agent`  
Parallel group: `none`  
Relative size: `L`

## Copy-paste start prompt

```text
You are the implementation agent for `C-R2F-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

The accepted default 60° max-subturn is sound but the current reference compiler effectively hardcodes only k=0..2. Product requirements call for adjustable reflex density that changes with angle size.

## Objective

Generalize hidden-support count from an exact request policy while separating semantic profile quality from render tessellation density.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- docs/decal_envelope_linear_axis_rebaseline.md
- kernel/src/cftuv_envelope/reference/compile.py
- kernel/src/cftuv_envelope/contracts/envelopes.py
- kernel/src/cftuv_envelope/contracts/request.py
- Session A reflex fixtures

## Allowed paths

- kernel/src/cftuv_envelope/contracts/request.py
- kernel/src/cftuv_envelope/contracts/envelopes.py
- kernel/src/cftuv_envelope/reference/compile.py
- kernel/tests/test_reference_envelopes.py
- kernel/fixtures/session_a_v5/**
- docs/session_c_r2f_reflex_quality.md

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- Do not expose raw hidden-edge count as the primary user semantic.
- Do not change k during ordinary alpha drag or camera zoom.
- Do not silently clamp profile complexity.

## Required design

- Primary parameter: exact `max_reflex_subturn` preset/value.
- Formula: `k=max(0, ceil((phi-pi)/Delta_max)-1)`.
- Optional derived certificate records maximum speed ratio.
- Default pi/3 produces existing output.
- A separate render/tessellation setting never changes Envelope boundaries.
- A product complexity limit returns a named outcome and requires owner acceptance.

## Implementation sequence

- Replace hardcoded `range(3)`/third thresholds with exact generic selection.
- Version request and profile-selection certificate.
- Add presets such as 60°, 45°, 30° only if accepted as exact ratios of pi.
- Add threshold and speed-bound certificates.
- Add alpha-stability tests.

## Acceptance criteria

- Default fixtures are bit/record compatible.
- Selected k changes monotonically with reflex excess and selected quality.
- Before/at/after threshold cases are exact.
- Semantic digest changes when quality changes and does not change with render density.

## Required tests and evidence

- Angle sweep from pi+ε to 2pi-ε.
- Preset matrix.
- Rigid-transform/retriangulation metamorphic tests.
- Complexity-limit named outcome.

## Deliverables

- Configurable profile contract and implementation.
- Product-facing preset documentation.
- Regression and certificate report.

## Stop conditions

- Stop before choosing an unapproved maximum k.
- Stop if angle input is not certified under the accepted metric.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
