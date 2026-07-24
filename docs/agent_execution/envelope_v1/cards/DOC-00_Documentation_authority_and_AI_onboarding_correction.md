# DOC-00 — Documentation authority and AI onboarding correction

Status: **READY_AFTER_BASE**  
Phase: `Baseline`  
Dependencies: `BASE-00`  
Primary role: `Documentation authority curator`  
Parallel group: `baseline-parallel`  
Relative size: `S`

## Copy-paste start prompt

```text
You are the implementation agent for `DOC-00` in `helmdubo/CFTUV`.
Start only from the accepted integration SHA recorded in `docs/architecture_status.json`.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

Current root guidance still contains legacy/pyvoronoi statements close to the new kernel instructions. External agents can follow the wrong backend before reaching the newer handoffs.

## Objective

Make the accepted Envelope architecture, active blocker and session authority unambiguous without deleting historical evidence.

## Required reading

- AGENTS.md at the accepted integration SHA
- 01_GLOBAL_CANON.md from this execution pack
- accepted handoff from every dependency card
- docs/envelope_engine_start_here.md
- docs/envelope_external_agent_session_manifest.md
- docs/decal_envelope_linear_axis_rebaseline.md
- docs/envelope_backend_semantics.md

## Allowed paths

- AGENTS.md
- README.md
- docs/envelope_engine_start_here.md
- docs/envelope_external_agent_session_manifest.md
- docs/architecture_status.json
- docs/adr_index.md

## Forbidden work

- Do not edit kernel or host implementation.
- Do not erase legacy documentation; mark it superseded or scoped.
- Do not claim a slice is complete without an accepted handoff.

## Required design

- Define one authority order.
- Mark documents accepted/superseded/experimental.
- Point every external agent to the status manifest and current card.
- Clarify that pyvoronoi is legacy decal context, not the new Envelope kernel authority.

## Implementation sequence

- Audit all top-level onboarding references.
- Add a short current architecture map.
- Add active blocker and next-card references.
- Add CI that checks referenced authority files exist and the active slice is known.

## Acceptance criteria

- A new agent reading only README/AGENTS/start_here reaches the Envelope canon and current card.
- No current document instructs a kernel agent to implement the new engine in legacy decal modules.
- Status labels are machine-readable.

## Required tests and evidence

- Documentation link validator.
- Status-manifest schema test.
- Review by an agent with no prior conversation context.

## Deliverables

- Updated authority docs.
- ADR/status index.
- Documentation audit report.

## Stop conditions

- Stop if two accepted documents conflict on geometry; escalate as an ADR rather than choosing wording silently.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
