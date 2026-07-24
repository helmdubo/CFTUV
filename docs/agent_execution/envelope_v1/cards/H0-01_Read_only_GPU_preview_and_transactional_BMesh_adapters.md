# H0-01 — Read-only GPU preview and transactional BMesh adapters

Status: **BLOCKED**  
Phase: `Host integration`  
Dependencies: `F2-01`  
Primary role: `Host adapter author`  
Parallel group: `none`  
Relative size: `L`

## Copy-paste start prompt

```text
You are the implementation agent for `H0-01` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

GPU and BMesh must consume the same GeometryBatch and perform no independent geometry computation.

## Objective

Implement or migrate host adapters to read GeometryBatch, preserve occurrence keys, and guarantee transactional mesh updates.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted F2 handoff
- cftuv/envelope_debug_renderer.py
- cftuv/envelope_host_adapter.py
- tests/blender/test_envelope_debug_bridge.py

## Allowed paths

- cftuv/envelope_host_adapter.py
- new cftuv/envelope_geometry_batch_adapter.py
- GPU/debug adapter files
- transactional BMesh materializer files
- tests/test_envelope_*adapter*.py
- tests/blender/**

## Forbidden work

- No geometry/owner/UV repair.
- No legacy fallback.
- No coordinate welding of distinct VertexKeys.

## Required design

- One batch, two read-only projections.
- GPU may use simplified display buffers only if they reference the same semantic elements.
- BMesh mutation is staged and committed only after complete validation.

## Implementation sequence

- Implement batch-to-GPU buffers.
- Implement batch-to-BMesh transaction.
- Add generation token/cancellation handling.
- Compare preview and materialized topology/attributes.

## Acceptance criteria

- GPU and BMesh consume the same batch digest.
- Failure leaves source mesh unchanged.
- Coincident distinct occurrences remain distinct where semantic.
- No host geometry algorithm exists.

## Required tests and evidence

- Blender-free adapter tests.
- Headless Blender tests.
- Source mesh fingerprint before/after failure.
- Preview/materialization equality.

## Deliverables

- Host adapters.
- Transactional tests.
- Field screenshot/receipt as diagnostic evidence.

## Stop conditions

- Stop if the batch lacks required semantics; return to the responsible kernel card.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
