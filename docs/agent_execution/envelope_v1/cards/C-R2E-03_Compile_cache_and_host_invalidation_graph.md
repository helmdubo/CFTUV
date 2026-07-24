# C-R2E-03 — Compile cache and host invalidation graph

Status: **BLOCKED**  
Phase: `C-R2E`  
Dependencies: `C-R2E-02`  
Primary role: `Host adapter author`  
Parallel group: `none`  
Relative size: `L`

## Copy-paste start prompt

```text
You are the implementation agent for `C-R2E-03` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

The host cache was disabled because compilation carried alpha. After the split, alpha-only updates must not rebuild analysis or static compilation.

## Objective

Re-enable static program caching and install dependency-aware invalidation with telemetry.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- accepted C-R2E-02 handoff
- cftuv/envelope_debug_session.py
- cftuv/envelope_host_adapter.py
- cftuv/envelope_request_export.py
- tests/test_envelope_host_adapter.py

## Allowed paths

- cftuv/envelope_debug_session.py
- cftuv/envelope_host_adapter.py
- cftuv/envelope_request_export.py
- cftuv/envelope_debug_profile.py
- tests/test_envelope_host_adapter.py
- tests/test_envelope_debug_renderer_contract.py

## Forbidden work

- No host-side geometry decisions.
- No geometry repair on cache miss.
- Do not use viewport visibility as a geometry invalidator.

## Required design

- Dependency graph: source revision → indexed analysis → metric/domain complex → static program → alpha state → projection.
- Alpha/selection visibility/viewport changes have different invalidation scopes.
- Coordinates/topology/seams/object data invalidate dependent caches.

## Implementation sequence

- Define cache keys and generation tokens.
- Re-enable static compile cache.
- Add hit/miss/rebuild counters.
- Test alpha-only updates and source edits.
- Prevent stale result publication.

## Acceptance criteria

- Changing only alpha performs zero analysis rebuilds and zero static recompiles.
- Changing selection rebuilds only affected programs.
- Changing mesh revision invalidates all dependent states.
- No stale state is published after a newer request.

## Required tests and evidence

- Host adapter unit tests.
- Cache invalidation matrix.
- Blender-free bridge tests.
- Field telemetry receipt.

## Deliverables

- Working compile cache.
- Invalidation ADR.
- Telemetry report.

## Stop conditions

- Stop if host code must compute exact geometric laws; move the law into kernel and keep host facts only.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
