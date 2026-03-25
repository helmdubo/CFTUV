# CFTUV — Cleanup Task: Remove Dormant `free_ingress / Bootstrap Bridge`

## Objective

Remove the dormant `free_ingress / Bootstrap Bridge` path from the codebase, documentation, telemetry vocabulary, and reporting surfaces **without changing active frontier behavior**.

Current confirmed state:
- `_cf_try_place_free_ingress_bridge(...)` still exists in `cftuv/solve_frontier.py`
- It is **not called** from the active stall-rescue loop
- Runtime currently attempts only:
  - `tree_ingress`
  - `closure_follow`
- Residual `free_ingress` vocabulary still exists in:
  - code
  - telemetry record schema
  - reporting
  - documentation

This task is a cleanup task, not a rescue/scoring redesign.

---

## Why this task exists

The project owner explicitly rejected Bootstrap Bridge / `free_ingress` as an active solve path.

That means the repository should no longer present `free_ingress` as:
- a supported rescue mode
- an expected telemetry path
- a live architecture concept
- a documented recovery mechanism

Residual dead code and stale vocabulary increase confusion and can mislead future AI agents.

---

## Hard Constraints

The agent must follow these constraints:

1. **Do not change active frontier behavior**
2. **Do not modify scoring semantics**
3. **Do not redesign rescue order**
4. **Do not reintroduce Bootstrap Bridge through fallback logic**
5. **Do not touch unrelated logging / telemetry work beyond what is necessary for this cleanup**
6. **Do not change `tree_ingress` or `closure_follow` behavior except for reference cleanup if needed**
7. **Respect AGENTS.md invariants and architecture**

---

## Confirmed Current Facts

The agent should first confirm these facts in code before editing:

1. `_cf_try_place_free_ingress_bridge(...)` exists in `cftuv/solve_frontier.py`
2. Active stall loop does **not** call that function
3. Active stall loop currently tries:
   - `tree_ingress`
   - `closure_follow`
4. `solve_records.py` still contains `free_ingress` in telemetry structures / comments
5. `docs/cftuv_architecture.md` still references `free_ingress` as a rescue path
6. Reporting / telemetry formatting may still expose `free_ingress`

Do not proceed until these facts are verified in the current branch.

---

## Work Plan by Phase

### Phase 1 — Audit and Dependency Map

#### Goal
Build a precise inventory of all remaining `free_ingress` / bridge-related references.

#### Tasks
Search for all occurrences of:
- `free_ingress`
- `bridge`
- `Bootstrap Bridge`
- `_cf_try_place_free_ingress_bridge`
- `free_ingress_placements`
- comments/docstrings describing one-edge FREE bridge rescue

#### Output
Produce an audit grouped into:
- active code paths
- dead code
- telemetry schema
- reporting
- docs/comments

#### Verification
The audit must clearly distinguish:
- dead dormant path
- active runtime path
- stale documentation

#### Stop gate
Do not edit code until the audit is complete.

---

### Phase 2 — Remove Dormant Runtime Function

#### Goal
Delete the dead bridge rescue implementation from `solve_frontier.py`.

#### Tasks
1. Remove `_cf_try_place_free_ingress_bridge(...)`
2. Remove any now-unused imports / types / helper references that existed only for this path
3. Remove any bridge-specific trace labels such as `[BRIDGE]` that belong only to dead bootstrap-bridge behavior
4. Keep active frontier loop unchanged except for cleanup of dead references

#### Verification
- `solve_frontier.py` still compiles conceptually
- active rescue order remains:
  - `tree_ingress`
  - `closure_follow`
- no residual dead callsites remain

#### Stop gate
Do not continue until dormant runtime function is fully removed.

---

### Phase 3 — Remove Telemetry Vocabulary

#### Goal
Remove `free_ingress` from telemetry data structures and formatting surfaces.

#### Tasks
1. Clean `solve_records.py`:
   - remove `free_ingress` from comments / documented path enums
   - remove `free_ingress_placements` field from telemetry aggregates if present
2. Update `solve_instrumentation.py`:
   - remove counting / formatting logic for `free_ingress`
   - preserve telemetry behavior for:
     - `main`
     - `tree_ingress`
     - `closure_follow`
3. Update any affected reporting that still expects `free_ingress`

#### Important
Do not accidentally break deserialization / formatting assumptions in reports.

#### Verification
- telemetry summary no longer mentions `free_ingress`
- telemetry detail no longer assumes this path exists
- no stale counters remain

#### Stop gate
Do not continue until telemetry schema and formatting are internally consistent again.

---

### Phase 4 — Remove Documentation Drift

#### Goal
Bring docs and comments in line with real runtime.

#### Tasks
Update stale references in:
- `docs/cftuv_architecture.md`
- any comments/docstrings in:
  - `solve_frontier.py`
  - `solve_records.py`
  - `solve_instrumentation.py`
  - `solve_reporting.py`
  - other touched files

Specifically:
- remove `free_ingress` from lists of active rescue paths
- remove wording that implies Bootstrap Bridge is supported
- preserve accurate description of current rescue behavior:
  - `tree_ingress`
  - `closure_follow`

#### Verification
- architecture docs match actual code
- no doc claims an inactive rescue mode is still active

#### Stop gate
Do not continue until documentation matches runtime behavior.

---

### Phase 5 — Consistency Pass and Safe Verification

#### Goal
Confirm the cleanup did not disturb active scaffold build semantics.

#### Tasks
Perform a consistency pass across touched files:
- naming
- imports
- counters
- summaries
- docs
- comments

Then run lightweight verification logic:
- inspect active frontier stall loop
- inspect telemetry summary paths
- inspect reporting text
- inspect doc alignment

#### Verification checklist
Confirm:
1. Active rescue paths are exactly:
   - `tree_ingress`
   - `closure_follow`
2. No `free_ingress` fields remain in telemetry aggregates
3. No dead helper for Bootstrap Bridge remains
4. No doc still presents Bootstrap Bridge as active
5. No runtime behavior was intentionally changed

---

## Deliverables

At the end of the task, the branch should contain:

1. Dead `free_ingress` bridge code removed
2. Telemetry / reporting vocabulary cleaned
3. Documentation aligned with actual runtime
4. No stale bridge references in active surfaces
5. A short completion report explaining:
   - what was removed
   - what was intentionally preserved
   - why runtime behavior should remain unchanged

---

## Required Verification Cycle

The agent must work phase-by-phase and stop after each phase.

### Required report format after each phase

- Phase completed:
- Files changed:
- References removed:
- What behavior changed:
- What behavior intentionally did not change:
- Verification performed:
- Remaining cleanup items:
- Risks / notes:

The agent must **wait for approval before starting the next phase**.

---

## Acceptance Criteria

### AC1
`_cf_try_place_free_ingress_bridge(...)` no longer exists.

### AC2
Active stall rescue loop still uses only:
- `tree_ingress`
- `closure_follow`

### AC3
Telemetry no longer exposes `free_ingress` as a valid active path.

### AC4
Architecture docs no longer describe Bootstrap Bridge / `free_ingress` as active runtime behavior.

### AC5
No unrelated solve behavior was changed.

---

## Out of Scope

Do not do any of the following in this task:
- redesign rescue heuristics
- modify scoring thresholds
- alter seed selection
- change closure-cut behavior
- change pin policy
- expand reporting refactor beyond required cleanup
- introduce new rescue paths

---

## Final Note to the Agent

This is a **cleanup and alignment** task.

The project owner explicitly rejected Bootstrap Bridge / `free_ingress`.
Your responsibility is to remove dormant remnants and documentation drift without destabilizing the active chain-first frontier path.
