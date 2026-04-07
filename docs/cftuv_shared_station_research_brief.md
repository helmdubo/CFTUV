# CFTUV Shared Station Research Brief

Purpose: stop pushing full runtime integration blindly and instead build an isolated research prototype for the remaining straighten-band defect.

This is **not** a full pipeline refactor task.
This is a **research / prototype task**.

---

## Why this brief exists

The project already made several valid steps:
- copied-phase cleanup
- axis authority split
- span/frame authority split
- band candidate detection improvements

But the visual bug still remains on the problematic S-case.

That strongly suggests the remaining issue is not another small runtime heuristic.
It is now a **modeling / parameterization problem**.

The most likely remaining defect is:
- paired long-side chains of a band patch still do not share a robust common station domain
- even after axis and span are already resolved

So the next step should not be:
- more full-pipeline tweaks
- more rescue/closure heuristics
- more metadata-only authority labels

The next step should be:
- build and evaluate an **isolated prototype** for shared station authority on a single patch

---

# Research objective

For one selected problematic band patch, answer this question:

> Can we build a visually better strip parameterization by using
> - existing chain / neighbor / corner / junction data,
> - resolved band axis/frame,
> - and a **shared consensual station map** for the paired long-side chains,
> without relying on face-type-specific logic or copying phase from one side to the other?

---

# Scope of this task

## In scope

- isolate one problematic band patch from the S-case
- identify its:
  - two long-side candidate chains
  - cap/start-end chains
  - resolved axis
  - current target span/frame assumptions
- compute local normalized cumulative edge-length fractions for each long-side chain
- build one or more candidate **shared station maps** for the pair
- place only the boundary chains of that patch in a temporary / prototype strip frame
- inspect whether checker / station correspondence would be better than the current runtime result

## Out of scope

- no full Phase 1 pipeline rewrite
- no rescue/closure overhaul
- no conformal solver integration in main flow yet
- no face-type-specific branch like “if triangles do X” as the main approach
- no massive refactor of frontier runtime in this task

---

# Key hypothesis to test

The current remaining artifact is likely caused by this:
- long-side A uses its own local `cumlen / total_len`
- long-side B uses its own local `cumlen / total_len`
- the two sides therefore occupy the same strip frame with different internal stations
- this creates cross-strip correspondence drift and distorted checker appearance

The prototype should test whether the artifact improves when both long sides instead use:
- the same resolved axis
- the same resolved span/frame
- the same **shared station domain**

while still **not** copying one side directly onto the other.

---

# Research questions

The prototype should help answer these specific questions:

1. Is the remaining bug really a shared-station problem?
2. Is a pair-consensus station map enough to improve the visual result?
3. Which station-map construction works best for chain-based data?
4. Can this be done using only chains / neighbors / corners / junctions, without depending on face layout?
5. Does the result look robust enough to justify integration into runtime?

---

# Prototype requirements

## Step 1. Choose one concrete patch

Pick one exact problematic S-case patch where:
- band detection is already correct
- axis is already correct
- span/frame seems mostly correct
- visual checker still looks wrong because interior correspondence appears shifted

Use this one patch as the research target.

## Step 2. Explicitly log its band structure

For the chosen patch, print / expose:
- patch id
- loop id
- the two long-side chain refs
- the cap/start-end chain refs
- resolved axis candidate
- current span authority / target span if already available
- local total lengths for both long-side chains
- normalized cumulative fractions for both long-side chains

The goal is to make the problem observable on one patch in isolation.

## Step 3. Build a temporary strip frame for this patch only

Do not run the whole full-flow pipeline to judge the stationing idea.

Create a temporary strip frame using:
- resolved axis direction
- target span
- start/end cap positions or synthetic start/end frame positions if needed

This temporary frame is only for prototype evaluation.

## Step 4. Build and compare candidate shared station maps

At minimum, test these station map strategies:

### Strategy A — current baseline
Each long side uses only its own local normalized cumulative fractions.

### Strategy B — merged pair consensus
Take the normalized cumulative fractions from both long sides, merge them into one sorted station set in `[0,1]`, cluster near-duplicates if needed, and use that as the shared station domain.

### Strategy C — averaged pair consensus
If the two sides have comparable sampling, build a consensual map by averaging or blending the station positions rather than taking them from only one side.

Optional:
- simple smoothing / regularization of the shared station list if needed

Important:
- do not make one side the owner and copy it directly to the other
- the station map must be pair-owned, not side-owned

## Step 5. Evaluate only boundary placement first

For the prototype, it is enough to evaluate:
- long-side A boundary positions
- long-side B boundary positions
- cap positions / frame endpoints
- checker appearance or station alignment diagnostics on the isolated patch

Do not try to solve the whole mesh in this step.

---

# Preferred prototype output

The prototype should produce something concrete and easy to compare, for example:

- a small debug report for the chosen patch
- before/after station vectors
- before/after boundary UV coordinates for the two long sides
- a simple metric like:
  - station mismatch
  - cross-strip correspondence drift
  - checker stretch proxy
- and ideally one visual debug output / screenshot / exported debug data that helps compare the strategies

---

# Important design constraints

## Constraint 1 — chain-based only

The prototype should use existing conceptual data sources:
- chains
- neighbors
- corners
- junctions
- patch summary / band candidate info
- resolved axis/frame/span assumptions

Do not build the research prototype primarily around interior face categories.

## Constraint 2 — no copied-phase rollback

Do not reintroduce the old wrong idea under another name:
- no “take the opposite side UV points as template”
- no “copy partner fractions verbatim if point counts match”
- no one-sided phase ownership

## Constraint 3 — keep local chain geometry relevant

The prototype should still respect that each chain has its own geometry and sampling.

The question is not:
- can side B imitate side A perfectly?

The question is:
- can both sides share one stable station domain while still being driven by their own chain data?

## Constraint 4 — do not over-integrate yet

This is a research sandbox task.
Do not wire the prototype deeply into every runtime path before the experiment proves useful.

---

# Suggested success criteria

The prototype is considered successful if it shows that, on the chosen isolated patch:

- axis and span stay reasonable
- the two long sides no longer drift relative to each other as strongly
- checker appearance / correspondence proxy is visibly or numerically better
- the improvement comes from pair-consensus stationing, not from copied phase

If the prototype fails, that is still useful:
- it means the next missing layer is not station authority alone
- and then we can reassess before more runtime integration

---

# Deliverable expected from the agent

Please return:

1. A short note describing the chosen patch and why it is a good research target.
2. The exact prototype strategies tested.
3. The station vectors / diagnostics for each strategy.
4. A conclusion:
   - whether shared station authority looks promising
   - which strategy looks best
   - whether it should be integrated into runtime next
5. A minimal implementation summary of the prototype files/helpers added, keeping it isolated and reviewable.

---

# Final guidance

Do not treat this task as “fix the whole straighten system now”.
Treat it as:

> build the smallest isolated experiment that can tell us whether
> shared station authority is the missing conceptual layer.

That answer is more valuable right now than another broad runtime patch.