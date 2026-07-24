# C-R2E-01 — StaticPatchProgram and EvaluationState contracts

Status: **BLOCKED**  
Phase: `C-R2E`  
Dependencies: `D-R3-01`  
Primary role: `Kernel contract author`  
Parallel group: `none`  
Relative size: `L`

## Copy-paste start prompt

```text
You are the implementation agent for `C-R2E-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

Current compilation embeds requested/effective alpha in FrontComponent and is not reusable for drag.

## Objective

Define alpha-independent static program contracts and alpha-specific evaluation-state contracts with stable semantic identities.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- kernel/src/cftuv_envelope/contracts/plan.py
- kernel/src/cftuv_envelope/reference/contracts.py
- kernel/src/cftuv_envelope/reference/compile.py
- docs/envelope_external_agent_session_manifest.md

## Allowed paths

- kernel/src/cftuv_envelope/contracts/plan.py
- kernel/src/cftuv_envelope/reference/contracts.py
- kernel/src/cftuv_envelope/contracts/events.py
- kernel/src/cftuv_envelope/schema.py
- kernel/schema/**
- kernel/tests/test_public_contracts.py

## Forbidden work

- Do not read or invoke cftuv/decal_voronoi.py.
- Do not read legacy geometry sections of cftuv/decals.py.
- No snapping, epsilon topology, rounded-coordinate identity or nearest-source repair.
- Do not broaden product semantics beyond this card.
- Do not implement event optimization yet.
- Do not change Envelope laws or interaction semantics.

## Required design

- `StaticPatchProgramV1`: source revision, plan key, domain complex ref, seeds, stable FrontComponents, supports, EnvelopeSpecs, profile certificates, interaction declarations and event-predicate declarations.
- `EvaluationStateV1`: requested/effective alpha, EnvelopeInstances, component states, capacity, current topology interval/event cursor placeholders, Raw/Resolved complex refs and diagnostics.
- Static digest must be identical for all alpha values of the same request/selection/source revision.

## Implementation sequence

- Write contracts and migration matrix.
- Remove alpha fields from the new static FrontComponent record.
- Create component-state records.
- Version schemas and codecs.
- Provide temporary compatibility views for existing callers.

## Acceptance criteria

- Static identities do not change across alpha.
- No EvaluationState fact appears in AnalysisSnapshot.
- Serialization and validation are deterministic.
- Old APIs have an explicit deprecation path.

## Required tests and evidence

- Round-trip and schema tests.
- Two-alpha static-digest equality.
- Negative cross-source-revision tests.

## Deliverables

- Accepted contracts/ADR.
- Generated schemas.
- Migration guide.

## Stop conditions

- Stop if a product event semantic is required; use placeholders/declarations only.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
