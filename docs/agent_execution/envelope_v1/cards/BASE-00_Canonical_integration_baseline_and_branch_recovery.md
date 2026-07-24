# BASE-00 — Canonical integration baseline and branch recovery

Status: **READY**  
Phase: `Baseline`  
Dependencies: `None`  
Primary role: `Release integrator`  
Parallel group: `none`  
Relative size: `S`

## Copy-paste start prompt

```text
You are the release-integration agent for `BASE-00` in `helmdubo/CFTUV`.
Start from a coordination branch based directly on reviewed commit `df587ed166cfb0e0b615148f08c583b4477c5ac4`.
`docs/architecture_status.json` does not exist yet; creating it is a deliverable of this card.
Read the card and its allowlist. Do not read legacy decal geometry unless this card explicitly assigns that role.
Implement only this slice, preserve all global invariants, run the listed gates, and leave the required handoff artifacts.
Do not merge main-side commits or modify geometry without an explicit impact decision.
If a new product semantic decision is required, stop with a named issue instead of inventing behavior.
```

## Context

The reviewed M-R1 handoff is at df587ed166cfb0e0b615148f08c583b4477c5ac4; its implementation and CI were at e38d1406b591d1189bf98bb850c8cab5d233f1c8. The original branch ref is not currently a reliable onboarding target, and the history diverges from main.

## Objective

Create one canonical integration ref, prove its contents and CI, and make every later card resolve its baseline from a repository status manifest.

## Required reading

- GitHub commit df587ed166cfb0e0b615148f08c583b4477c5ac4
- artifacts/envelope_runtime_r1/session_m_r1_handoff.json
- docs/session_m_r1_rational_affine_filtered_runtime_handoff.md
- AGENTS.md

## Allowed paths

- repository refs/branches
- docs/architecture_status.json (new)
- docs/adr_index.md (new or updated)
- artifacts/envelope_baseline/**
- .github/workflows/envelope-kernel.yml only if needed to restore reproducibility

## Forbidden work

- Do not change geometry, contracts or tests except baseline metadata/CI repair.
- Do not blindly rebase 49 reviewed commits onto main.
- Do not merge the three main-side commits without an explicit file/semantic impact report.

## Required design

- Create a new protected or clearly named integration branch from the immutable reviewed commit.
- Compare the reviewed commit with main and record divergence.
- Record integration SHA, predecessor SHA, package version and CI run.
- Add a machine-readable architecture status manifest used by later cards.

## Implementation sequence

- Fetch the immutable commit and verify the M-R1 handoff.
- Create the integration ref.
- Run kernel wheel, extraction-readiness and host bridge tests.
- Generate `docs/architecture_status.json` with `accepted_integration_sha`, `active_slice`, `last_accepted_gate`, and known blockers.
- Write a baseline handoff.

## Acceptance criteria

- A fresh checkout can resolve the integration ref.
- The status manifest points to an immutable SHA.
- 205 kernel tests and 26 host bridge tests pass, or updated exact counts are recorded with reasons.
- No geometry file differs from the reviewed baseline unless explicitly explained.

## Required tests and evidence

- Full kernel wheel tests outside checkout.
- Extraction-readiness and forbidden-import scan.
- Blender-free host bridge tests.
- Compare-tree report against df587ed and main.

## Deliverables

- Canonical integration ref.
- docs/architecture_status.json.
- artifacts/envelope_baseline/session_base_00_handoff.json.
- Markdown handoff with compare report.

## Stop conditions

- Stop if the immutable commit cannot be fetched.
- Stop if main-side commits change Envelope semantics; request an explicit merge decision.
- Stop if CI cannot be reproduced without modifying geometry.

## Mandatory handoff

Use `templates/HANDOFF_TEMPLATE.md`. Record exact base, implementation and CI SHAs; contract versions; full test results; semantic digest comparison; named unsupported outcomes; source mesh mutation check; and next-agent allowlist.
