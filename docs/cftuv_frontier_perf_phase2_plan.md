# CFTUV Frontier Perf Phase 2 Plan

Status: design only
Owner: orchestrator
Scope: post-refactor frontier optimization research
Behavior scope: no mainline behavior change in this phase

---

## Purpose

Define the next optimization stage after the frontier split is complete.

This document is not permission to land selector redesign or cache rewrites
directly into mainline. It defines what to measure, what to prototype, what to
compare against, and what evidence is required before any runtime cutover.

The architectural invariant is unchanged:

> Preserve chain-first strongest-frontier. Patch and Chain remain the only
> primary solve units. Do not drift into patch-first, loop-sequential, or
> corner-first placement.

---

## Current baseline

Current frontier runtime is intentionally conservative after P8:

- `solve_frontier.py` is now only an orchestration shell.
- Main frontier selection still performs a scan over `all_chain_pool` on each
  iteration, with `_cached_evals` and `_dirty_refs` used to avoid recomputing
  every candidate.
- Dirty propagation remains correctness-first:
  shared-vertex updates mark affected pool refs dirty, and first placement in a
  patch marks patch-local refs dirty.
- Rescue paths are already split from the main frontier path and remain cold.
- Telemetry is rich enough to compare rejected/below-threshold behavior and
  rescue-gap outcomes without changing solve behavior.

This baseline is the control implementation. All future perf work must compare
 against it first.

---

## Optimization targets

Primary hot spots to address in the next phase:

1. Frontier selector scan cost.
   The current selector walks the entire pool each iteration even when only a
   small subset is dirty.

2. Patch-context recomputation cost.
   Patch-scoring context and related derived data are structurally cacheable,
   but the current runtime does not track fine-grained versions for them.

3. Main-path coupling to cold-path diagnostics.
   Stall diagnostics and telemetry are already separated conceptually, but some
   expensive aggregation still happens near the hot loop boundary.

4. Telemetry cost visibility.
   The system records rich telemetry, but Phase 2 needs explicit counters for
   selector structure overhead, stale queue churn, and cache invalidation fanout.

---

## Hard guardrails

Any prototype or later implementation derived from this plan must preserve:

- chain-first strongest-frontier selection semantics
- current threshold gate semantics
- current structured-rank ordering
- current rescue boundary
- current stall -> `tree_ingress` -> `closure_follow` order
- current finalize order and untouched-patch fallback behavior
- bit-identical output against the scan+cache selector before any cutover

Explicit non-goals for Phase 2:

- no patch-first frontier
- no rescue absorption into main ranking
- no score/rank formula rewrite
- no telemetry ownership move out of `solve_instrumentation.py`
- no hidden behavior change under "perf cleanup"

---

## Workstream A - Baseline and acceptance harness

Before changing selector structure, add a repeatable comparison harness.

Requirements:

- choose a stable mesh set that includes:
  - simple wall/floor quilts
  - same-type quilts
  - cycle or tube closure cases
  - rescue-triggering cases
- capture per-quilt:
  - wall-clock frontier time
  - iteration count
  - cache hit count
  - number of evaluated dirty refs
  - stall count
  - rescue count by kind
- preserve a bit-identical acceptance oracle:
  - placement order
  - selected chain ref per iteration
  - final scaffold placements
  - finalized telemetry shape

Recommendation:

- keep the current selector as the oracle implementation
- add experimental comparators in shadow mode only
- fail experiments immediately on any selection mismatch

---

## Workstream B - Lazy frontier queue research

### Motivation

The current selector is `O(pool)` per iteration even when few refs changed.
The most likely Phase 2 speedup is a lazy max-heap or equivalent priority queue
fed by the same ranking data already produced by evaluation.

### Proposed shape

Introduce an experimental selector adapter with:

- heap items keyed by:
  - structured rank
  - scalar score
  - chain ref
  - generation id
- per-chain generation counters
- stale-pop detection
- dirty ref enqueue on invalidation

### Shadow-mode contract

The queue prototype must first run in shadow mode:

- mainline selection still comes from current scan+cache selector
- experimental queue computes its own top candidate
- compare:
  - selected ref
  - rank
  - threshold viability
- record mismatch telemetry
- never place from the queue while mismatches exist

### Key risks

- stale queue entries hiding dirty-marking bugs
- tie-break drift from rank serialization mistakes
- accidental threshold drift if viability is checked at a different point
- memory growth from duplicate enqueues

### Exit criteria before cutover discussion

- zero selection mismatches across the baseline mesh set
- measured reduction in selector CPU time on large quilts
- acceptable stale-pop overhead

---

## Workstream C - Patch-context cache versioning

### Motivation

Patch scoring context is derived mostly from patch-local state and a small set
of runtime counters. It should not require fully fresh recomputation on every
dirty candidate if none of its inputs changed.

### Proposed shape

Add explicit lightweight versions:

- `patch_state_version_by_patch`
  increments when patch-local placed counts change
- `patch_closure_version_by_patch`
  increments only if closure-owned facts for that patch change in future work
- optional `shape_profile_version`
  remains static unless shape-profile ownership becomes dynamic later

Derived cache candidates:

- patch scoring context
- patch-local placed ratios and coverage values
- seam-membership summaries if later shown hot in profiling

### Invalidation rules

- placing a chain in patch `P` increments only `P`'s patch-state version
- first-placement special cases must remain explicit
- cross-patch anchor lookups do not automatically invalidate unrelated patch
  scoring contexts unless they truly change patch-owned state

### Risks

- version bugs that preserve old patch context across first-placement boundary
- over-broad invalidation eliminating the benefit
- under-invalidation causing selector mismatch

### Acceptance rule

Versioned patch-context caching is only eligible for mainline if selector output
remains bit-identical against the control implementation.

---

## Workstream D - Hot/cold path separation

### Goal

Make hot-path placement and selection cheaper without reducing diagnostics.

### Candidate separations

- keep rescue ranking/builders cold and callable only after stall
- avoid building heavy stop diagnostics until frontier actually stalls
- avoid expensive formatting on the hot path when live trace is disabled
- isolate experimental selector counters from default-mode telemetry cost

### Important limit

Do not remove useful instrumentation blindly. First prove that a telemetry
component is materially visible in profiling, then gate or defer it.

---

## Workstream E - Telemetry additions for perf research

Phase 2 needs a small set of new counters specifically for perf experiments.

Recommended counters:

- selector scans performed
- dirty candidates evaluated
- queue pushes
- queue stale pops
- queue shadow mismatches
- patch-context cache hits/misses
- patch-context invalidations by reason

Recommended derived reports:

- selector cost per placed chain
- stale-pop ratio
- average dirty-set size per iteration
- average patch-context reuse ratio
- unresolved stall rate before and after prototype

Rule:

- keep these counters optional or low-cost in default mode
- do not expand placement record payloads unless the data is needed for analysis

---

## Suggested rollout order

1. Baseline harness and profiling pass.
   No selector changes. Measure current costs on representative quilts.

2. Patch-context versioning prototype.
   Lower-risk optimization with direct correctness oracle.

3. Queue shadow mode.
   Compare against the scan+cache selector without affecting solve output.

4. Queue cutover decision.
   Only if shadow mode shows zero mismatches and real speedup.

5. Optional cleanup pass.
   Remove temporary dual-path experiment scaffolding after cutover is proven.

This order is intentional:

- versioned patch-context caching can reduce evaluation cost immediately
- queue redesign is the highest-risk change and should come after better
  visibility into invalidation behavior

---

## Cutover checklist for any future implementation phase

Before a perf prototype becomes mainline:

- selector output is bit-identical to control on the baseline mesh set
- no import-cycle regression is introduced
- rescue order is unchanged
- finalize output is unchanged
- telemetry still explains stalls and rescue outcomes clearly
- measured win is large enough to justify added complexity

Minimum evidence threshold:

- at least one large quilt case with meaningful frontier time reduction
- no correctness mismatches on representative production meshes
- no new unresolved stalls attributable to selector/cache changes

---

## Deferred ideas intentionally out of scope

These may be interesting later, but they are not part of Phase 2:

- manual frontier steering
- heuristic score redesign
- adaptive thresholding
- patch batching
- GPU/vectorized evaluation
- collapse of rescue into main frontier selection

---

## Deliverable of this phase

Phase 9 is complete when:

- this document exists and is specific enough to guide later implementation
- the migration sequence is explicit
- correctness gates are explicit
- risks and stop conditions are explicit
- no mainline behavior changed while producing the plan
