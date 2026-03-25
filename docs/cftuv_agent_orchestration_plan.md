# CFTUV — Agent Orchestration Plan for Logging / Telemetry Optimization

> Status: Completed on 2026-03-26.
>
> Phases 1-7 were implemented as a reporting / telemetry refactor without solve-behavior changes.
> This file is retained as a historical execution plan. Current output contract lives in
> `docs/cftuv_architecture.md` and `docs/cftuv_reference.md`.

## Objective

Refactor CFTUV reporting and telemetry output to reduce token-heavy log spam while preserving fast fault localization for the tester.

This work must **not change solve behavior**. It is a reporting / instrumentation policy refactor, not an algorithm rewrite.

Core goals:

1. Reduce repetitive and low-signal console/report output.
2. Preserve and improve diagnostics for:
   - H/V direction bugs
   - closure seam mismatches
   - frame scatter issues
   - scaffold transfer conflicts
   - unresolved / unsupported patches
   - frontier stall / rescue failures
3. Introduce stable problem addresses so the tester can locate faults quickly.
4. Make regression snapshots shorter, more stable, and more AI-reviewable.

---

## Architectural Constraints

Respect current project architecture:

- Keep `PatchGraph`, `ScaffoldMap`, and telemetry records as the source of truth.
- Do not change solve semantics.
- Do not remove structured records from `solve_records.py`.
- Do not replace telemetry records with raw strings.
- Prefer pure helper functions for aggregation / formatting.
- Default logs should become shorter, but directional diagnostics for suspicious H/V chains must remain visible.

Relevant current files:
- `cftuv/solve_reporting.py`
- `cftuv/solve_instrumentation.py`
- `cftuv/solve_records.py`

---

## Expected Outcomes

After this refactor:

- Default logs should highlight **anomalies first**, not dump all geometry.
- Every serious problem should include a stable address like:
  - `ADDR Q1/P14`
  - `ADDR Q1/P14/L0/C3`
  - `ADDR Q1/P14<->P21`
- Healthy entities should not emit large raw coordinate floods by default.
- Direction-suspicious H/V chains must still emit compact directional diagnostics.
- Regression snapshots should be compact and stable for diff / AI review.
- Telemetry should remain structured and queryable.

---

## Work Plan by Phase

### Phase 1 — Introduce Reporting Modes and Address Helpers

#### Goal
Create the foundation for layered output.

#### Tasks
1. Add stable address format helpers in reporting layer:
   - patch address
   - chain address
   - corner address
   - patch-pair / seam address
2. Introduce reporting mode concept:
   - `summary`
   - `diagnostic`
   - `forensic`
3. Add a small configuration structure or parameters to formatting functions.

#### Deliverables
- Address helper functions
- Reporting mode plumbing
- No behavioral changes in solve

#### Verification
- Unit-style smoke verification by running existing report formatters
- Confirm all anomaly lines can include a stable address
- Confirm no solve output changed

#### Stop gate
Do not continue until address formatting is stable and reused consistently.

---

### Phase 2 — Shared Metrics Collector

#### Goal
Remove duplicated aggregate calculations and create one source of truth.

#### Tasks
1. Extract duplicated aggregate logic from:
   - `format_root_scaffold_report`
   - `format_regression_snapshot_report`
2. Create a pure summary collector, e.g.:
   - `collect_scaffold_metrics(...) -> ScaffoldMetricsSummary`
3. Include aggregates for:
   - quilts count
   - patch totals
   - unsupported / invalid closure patches
   - closure seam counts
   - max seam mismatch / phase
   - frame group counts
   - max row / column scatter
   - unresolved / missing / conflicting transfer counts
   - invalid scaffold patch counts
   - unresolved stall counts if available

#### Deliverables
- Shared metrics collector
- Both reports consume the same aggregate source

#### Verification
- Compare old and new summary values on same asset
- Ensure numbers match previous semantics
- Ensure no duplicated aggregate logic remains in both reports

#### Stop gate
Do not continue until summary metrics are centralized.

---

### Phase 3 — Anomaly-First Reporting

#### Goal
Make logs immediately useful to the tester.

#### Tasks
1. Add `Anomalies:` section near the top of default reports.
2. Emit only problem-bearing items in this section.
3. Introduce concise anomaly codes such as:
   - `closure_invalid`
   - `closure_seam_mismatch`
   - `frame_scatter`
   - `transfer_conflict`
   - `missing_uv_targets`
   - `unresolved_scaffold`
   - `stall_unresolved`
   - `direction_conflict`
   - `axis_drift`
   - `direction_flip`
   - `hv_adjacency_conflict`
   - `frame_role_mismatch`
4. Each anomaly line must include:
   - stable address
   - anomaly code
   - 1–3 key metrics

#### Deliverables
- Anomaly-first section in default report
- Threshold-based anomaly filtering

#### Verification
- On problematic meshes, first 30–50 lines reveal problem class and address
- On healthy meshes, anomaly section is empty or tiny
- Tester can identify target patch/chain without reading entire dump

#### Stop gate
Do not continue until anomaly-first output is readable and compact.

---

### Phase 4 — Compact Default Scaffold Report

#### Goal
Reduce coordinate spam without losing critical debugging power.

#### Tasks
1. Suppress default raw dumps for healthy entities:
   - per-corner coordinate flood
   - full chain UV point list flood
   - full gap line flood
   - long refs lists
2. Keep compact per-chain diagnostics only where useful:
   - address
   - role
   - point count
   - start/end UV
   - UV delta
   - axis error (if relevant)
   - direction inherited flag
   - anchor kinds
3. Keep full raw data only for:
   - forensic mode
   - anomaly-triggered entities

#### Important directional rule
Do **not** blindly remove chain endpoint data from default diagnostics.
Direction-related bugs have historically been found from these logs.

Healthy routine entities should be compressed.
Direction-suspicious H/V chains should still surface compact directional diagnostics automatically.

#### Deliverables
- Much shorter default root scaffold report
- Preserved directional signal for suspicious H/V chains

#### Verification
- Compare line count before/after on representative asset
- Confirm healthy patches no longer dump all corners/chains
- Confirm direction-related anomalies still show enough data to debug

#### Stop gate
Do not continue until token reduction is significant without losing direction debugging value.

---

### Phase 5 — Separate Live Trace from Post-Hoc Detail

#### Goal
Avoid double text rendering of the same telemetry event.

#### Tasks
1. Keep structured telemetry records as-is.
2. Make runtime `trace_console` policy-driven:
   - `off`
   - `compact`
   - `full`
3. Ensure post-hoc detail rendering is separate from runtime trace.
4. Default runtime trace should be compact if enabled.

#### Example compact live trace
- `Q1 S12 main P14L0C3 H s:0.82 a:2`
- `Q1 Stall12 rej:0.74 rescue:tree_ingress=Y`

#### Deliverables
- Runtime trace policy
- No forced verbose duplication between collector and formatter

#### Verification
- Confirm same event is not emitted twice in nearly identical long form
- Confirm structured records still contain required detail
- Confirm compact live trace is readable

#### Stop gate
Do not continue until live trace duplication is reduced.

---

### Phase 6 — Stabilize Stall Lifecycle

#### Goal
Make stall telemetry state transitions explicit and reliable.

#### Tasks
1. Refactor pending stall lifecycle into explicit open/close semantics.
2. Avoid silent or ambiguous finalization where possible.
3. Optionally add close metadata such as:
   - close reason
   - rescue error flag
   - rescue exception type

#### Deliverables
- Clear stall state transition flow
- Less ambiguous unresolved stall reporting

#### Verification
- Simulate or inspect code paths:
  - stall + rescue success
  - stall + rescue fail
  - stall + finalize with unresolved close
- Confirm no pending stall is lost silently

#### Stop gate
Do not continue until stall lifecycle semantics are explicit.

---

### Phase 7 — Compact Regression Snapshot

#### Goal
Make snapshots stable, compact, and AI-reviewable.

#### Tasks
1. Rebuild snapshot around:
   - shared metrics summary
   - anomaly list
   - compact per-quilt status
   - compact per-patch transfer / pin / scaffold status
   - telemetry summary, not verbose telemetry detail
2. Remove default snapshot flood:
   - full UV point lists
   - raw corner floods
   - long repeated refs unless anomaly-triggered

#### Deliverables
- Stable compact markdown snapshot
- Better diff behavior

#### Verification
- Compare snapshot size before/after
- Confirm key problem signals remain visible
- Confirm repeated runs on same mesh stay stable enough for diff

#### Stop gate
Finish only after snapshot readability improves meaningfully.

---

## Directional Diagnostics Requirements

These are mandatory. They are not optional nice-to-have items.

For suspicious H/V chains, default diagnostic mode should still include compact directional data.

### Trigger conditions
A chain should be marked direction-suspicious if one or more apply:
- H chain looks too vertical in UV
- V chain looks too horizontal in UV
- large axis error
- sign / direction flip relative to expectation
- suspicious inherited direction
- conflicting adjacent H/V continuity
- closure / rescue path introduces visible orientation drift

### Required compact fields
For direction-suspicious chains, include:
- stable address
- frame role
- start/end UV
- UV delta
- axis error
- direction inherited flag
- anchor kinds
- concise status code

### Optional anomaly codes
- `direction_conflict`
- `axis_drift`
- `direction_flip`
- `hv_adjacency_conflict`
- `frame_role_mismatch`
- `inherited_direction_suspect`

---

## Verification Cycle

The agent must work in small phases and stop after each one with a verification report.

### Required cycle per phase
1. Implement one phase only.
2. Summarize exact files changed.
3. Explain what behavior is now different.
4. Run minimal validation / self-check.
5. Report:
   - what was completed
   - what was intentionally not changed
   - any open risks
6. Wait for approval before next phase.

### Required report template after each phase
- Phase completed:
- Files changed:
- New functions / structures:
- What output changed:
- What output intentionally did not change:
- Verification performed:
- Risks / follow-ups:

---

## Acceptance Criteria

### AC1
Default logs for healthy assets no longer dump large raw coordinate floods.

### AC2
Problematic assets show issue class and address within the first 30–50 lines.

### AC3
Direction-related H/V bugs remain diagnosable from logs without needing full raw UV dumps for everything.

### AC4
Shared metrics collector eliminates duplicated aggregate logic across major reports.

### AC5
Live trace and post-hoc detail are no longer redundant by default.

### AC6
Regression snapshots are shorter and more stable for diff / AI analysis.

### AC7
Solve behavior does not change.

---

## Out of Scope

Do not do these in this task:
- solve algorithm redesign
- frontier scoring changes
- topology / patch graph semantic changes
- pin policy redesign
- UI redesign beyond minimal debug/report toggles if absolutely needed

---

## Original Execution Order (Completed)

The agent should proceed in this exact order:

1. Phase 1 — reporting modes + address helpers
2. Phase 2 — shared metrics collector
3. Phase 3 — anomaly-first reporting
4. Phase 4 — compact default scaffold report
5. Phase 5 — separate live trace from post-hoc detail
6. Phase 6 — stall lifecycle cleanup
7. Phase 7 — compact regression snapshot

No phase skipping.
No batching multiple phases into one uncontrolled rewrite.

---

## Final Note to the Agent

Optimize for **tester speed of diagnosis** first, then token reduction.
Token reduction matters, but not at the cost of hiding directional bugs.

The correct strategy is:
- compress routine healthy output,
- preserve structured data,
- elevate anomalies,
- keep compact directional diagnostics for suspicious H/V chains,
- reserve raw floods for forensic mode only.
