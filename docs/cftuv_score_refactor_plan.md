# CFTUV Score Refactor Plan

AI agent, act as a refactor orchestrator: preserve **chain-first strongest-frontier**, keep **Patch** and **Chain** as the only primary solve units, and execute this plan phase by phase without mixing future manual operations into the current scoring refactor.

---

## 1. Purpose

This document defines the agreed plan for refactoring the CFTUV scoring model.

Current problem:
- frontier choice is effectively driven by a flat scalar score plus many embedded rules;
- patch context, seam context, and local topology hints are mixed together inside runtime scoring;
- rescue paths (`tree_ingress`, `closure_follow`) compensate for cases the main frontier score does not model cleanly;
- row/column drift concerns are real, but they belong to a different alignment layer, not to the main frontier score.

Target state:
- candidate comparison is based on a **structured frontier rank**, not a single float;
- patch runtime context becomes explicit;
- corner-derived facts and patch shape become controlled feature sources;
- seam relations carry richer structure than `best_pair + scalar`;
- rescue paths remain separate until telemetry proves the main rank can absorb them safely.

---

## 2. Architectural invariants

These invariants are mandatory.

1. **Chain-first strongest-frontier** remains the core solve principle.
2. **Patch** and **Chain** remain the only primary solve units.
3. **BoundaryCorner** may contribute local features, but must not become an independent placement unit.
4. **Junction** remains a derived/global entity at this stage, not a primary runtime scoring entity.
5. **HOLE** loops do not enter the scaffold placement frontier.
6. Do not regress toward patch-first, loop-sequential, or corner-driven placement.
7. Do not fold row/column drift control into the main frontier score.
8. Do not merge future manual operations into this refactor.
9. Rescue control-flow stays separate during the first scoring refactor phases.
10. Debug, telemetry, and regression snapshots must remain usable after every phase.

---

## 3. Scope

### In scope
- Replace scalar-only frontier comparison with a structured rank.
- Introduce explicit `PatchScoringContext`.
- Use `Corner` only as a feature source for chain ranking.
- Introduce `PatchShape` / `PatchShapeProfile` as a coarse prior.
- Enrich patch-to-patch seam relation data.
- Expand telemetry and documentation to explain the new scoring model.

### Out of scope for this document
- Manual operations on selected patch/chains.
- Full `Junction` runtime layer.
- Row/column drift redistribution pass.
- Quilt-wide snap or correction pass.
- Large rewrite of root scoring / `PatchCertainty`.
- Absorbing rescue paths into main frontier before telemetry says it is safe.

---

## 4. Entity roles in scoring

## 4.1. Patch

Patch participates in:
- planning/root/seed selection;
- runtime patch context;
- coarse semantic priors such as shape and closure sensitivity.

Patch does **not** become the step unit of solve.

## 4.2. Chain

Chain remains the main runtime placement unit.
Every frontier decision is still a decision about **which chain to place next**.

## 4.3. Corner

Corner is a **feature source**, not a candidate type.
Use it for:
- turn quality;
- orthogonal continuation confidence;
- same-role continuity strength;
- geometric-vs-junction distinction.

Do **not** give corner its own competing frontier score.

## 4.4. Junction

Junction is a **derived/global topology view**.
Use it only as context for future seam enrichment and later tooling.
Do not make it a first-order runtime scoring primitive in this phase.

## 4.5. Patch Shape

Patch shape is a **coarse prior**.
It may influence:
- root/seed preference;
- ingress confidence;
- closure sensitivity;
- patch-fit heuristics.

It must not become a hard solve mode controller.

## 4.6. Row / Column

Row/Column belongs to a separate **alignment layer**.
It is useful for:
- diagnostics;
- future rigid-structure handling;
- future drift redistribution.

It must not be added as a normal bonus/penalty in main frontier rank.

---

## 5. Target scoring architecture

Replace:

```text
candidate -> one scalar float
```

with:

```text
candidate -> structured rank + explicit context + debug breakdown
```

## 5.1. Frontier rank

Introduce a rank object that is compared lexicographically.

Suggested shape:

```python
@dataclass(frozen=True, order=True)
class FrontierRank:
    role_tier: int
    ingress_tier: int
    patch_fit_tier: int
    anchor_tier: int
    closure_risk_tier: int
    local_score: float
    tie_length: float
```

This exact field list may evolve, but the design principle must stay the same:
- upper layers are **discrete priority tiers**;
- lower layers are **refinement / tie-breakers**.

## 5.2. What belongs in the rank

Allowed in rank:
- role tier: H/V, FREE, bridge, corner-split;
- ingress quality;
- patch fit;
- anchor quality and provenance;
- closure risk;
- local refinement score.

Not allowed in rank:
- global row/column scatter;
- quilt-wide alignment drift;
- raw Junction score;
- correction-pass logic;
- post-hoc diagnostic aggregates.

---

## 6. New data objects

## 6.1. `PatchScoringContext`

Purpose: explicit runtime summary of the patch state around a chain candidate.

Minimum fields:
- `patch_id`
- `placed_chain_count`
- `placed_h_count`
- `placed_v_count`
- `placed_free_count`
- `placed_ratio`
- `hv_coverage_ratio`
- `same_patch_backbone_strength`
- `closure_pressure`
- `is_untouched`
- `has_secondary_seam_pairs`
- `shape_profile`

Important rule:
- use **continuous metrics** for scoring;
- derived discrete labels are allowed only for reporting/debug.

## 6.2. `CornerScoringHints`

Purpose: package corner-derived local facts for one candidate chain.

Suggested fields:
- `start_turn_strength`
- `end_turn_strength`
- `orthogonal_turn_count`
- `same_role_continuation_strength`
- `has_geometric_corner`
- `has_junction_corner`

## 6.3. `PatchShapeProfile`

Purpose: represent coarse patch shape as a weak semantic prior.

Start simple.
Suggested numeric fields:
- `elongation`
- `rectilinearity`
- `hole_ratio`
- `frame_dominance`
- `seam_multiplicity_hint`

Optional derived labels later:
- `STRIP`
- `RING`
- `CAP`
- `BOXY`
- `IRREGULAR`

## 6.4. `SeamRelationProfile`

Purpose: enrich attachment/seam semantics beyond a scalar.

Suggested fields:
- `edge_key`
- `primary_pair`
- `secondary_pair_count`
- `pair_strength_gap`
- `is_closure_like`
- `support_asymmetry`
- `ingress_preference`

This object exists because runtime frontier currently has to reconstruct seam structure that planning does not preserve explicitly enough.

## 6.5. Optional debug-only `PatchFrontierState`

Allowed only as a derived reporting label.

Possible values:
- `UNTOUCHED`
- `INGRESSING`
- `BACKBONE_FORMING`
- `BACKBONE_STABLE`
- `FREE_FILL`
- `CLOSURE_SENSITIVE`
- `SATURATED`

Do not use this as the main source of truth for runtime ranking.

---

## 7. File-by-file implementation plan

## 7.1. `cftuv/solve_records.py`

Add the core scoring records here:
- `FrontierRank`
- `PatchScoringContext`
- `CornerScoringHints`
- `SeamRelationProfile`
- optional `FrontierRankBreakdown`
- expanded `FrontierCandidateEval` fields for rank/context debugging

This file should become the canonical home of scoring-layer dataclasses.

## 7.2. `cftuv/solve_frontier.py`

This is the main runtime refactor site.

Refactor goals:
- split `_cf_score_candidate()` into smaller helpers;
- add helpers such as:
  - `build_patch_scoring_context(...)`
  - `build_corner_scoring_hints(...)`
  - `build_frontier_rank(...)`
  - `build_frontier_debug_breakdown(...)`
- make `_cf_select_best_frontier_candidate()` compare `FrontierRank`, not one float;
- keep rescue path control-flow separate for now.

## 7.3. `cftuv/solve_planning.py`

Extend the seam/attachment layer.

Goals:
- produce `SeamRelationProfile`;
- preserve richer seam facts for frontier use;
- avoid turning planning into runtime frontier policy.

Do **not** do a large rewrite of `PatchCertainty` in early phases.

## 7.4. `cftuv/model.py`

Only place stable, lightweight data here.

Allowed:
- shape enum/record if it truly belongs in model IR.

Not allowed:
- runtime-heavy scoring logic.

## 7.5. `cftuv/solve_instrumentation.py`

Expand telemetry so that the refactor is inspectable.

Must expose:
- frontier rank tiers;
- patch scoring context snapshot;
- corner hints summary;
- seam relation hints summary;
- reason why a candidate won.

## 7.6. Documentation

Update:
- `AGENTS.md`
- `docs/cftuv_architecture.md`
- `docs/cftuv_reference.md`

These docs must explain:
- structured-rank frontier;
- roles of Patch / Chain / Corner / Junction / Row-Column;
- why alignment logic is intentionally separate;
- that manual operations are a future layer.

---

## 8. Phased rollout

## Phase 1 — Introduce structured rank with near-equivalent behavior

Goal:
- migrate comparison style without changing solve behavior significantly.

Tasks:
1. Add `FrontierRank`.
2. Build rank from the **existing** signals.
3. Keep the old scalar score as `local_score` or subordinate refinement.
4. Switch frontier candidate selection to lexicographic rank comparison.
5. Add telemetry for the new rank breakdown.

Acceptance criteria:
- ordering stays close to existing behavior;
- regression meshes do not show large unexplained changes;
- telemetry exposes the new comparison logic.

## Phase 2 — Extract `PatchScoringContext`

Goal:
- remove implicit patch-lifecycle reasoning from `_cf_score_candidate()`.

Tasks:
1. Add `PatchScoringContext`.
2. Move patch-state derivation into dedicated helpers.
3. Replace ad-hoc `if placed_in_patch...` style reasoning with explicit context usage.

Acceptance criteria:
- patch-aware logic becomes readable;
- scoring code shrinks and becomes easier to inspect.

## Phase 3 — Add corner-derived hints

Goal:
- improve local chain ranking using corner structure without elevating corners to solve units.

Tasks:
1. Add `CornerScoringHints`.
2. Feed them only into local chain refinement.
3. Keep comparison unit = chain.

Acceptance criteria:
- better local continuity modeling;
- no corner-driven solve regressions.

## Phase 4 — Add patch shape as weak prior

Goal:
- use shape to improve ingress/backbone/root confidence without hard-coding solve modes.

Tasks:
1. Add `PatchShapeProfile`.
2. Use it only as a weak prior in rank/context.
3. Add debug output explaining shape-derived influence.

Acceptance criteria:
- shape improves tie-breaks and early placement preference;
- shape does not dominate frontier decisions.

## Phase 5 — Enrich seam relation layer

Goal:
- reduce runtime seam guesswork by preserving more seam structure from planning.

Tasks:
1. Add `SeamRelationProfile`.
2. Persist primary/secondary seam facts.
3. Pass seam relation data into frontier ranking.

Acceptance criteria:
- frontier has explicit seam multiplicity and closure-likeness data;
- fewer ad-hoc seam-specific branches are needed later.

## Phase 6 — Turn monolithic score into layered helpers

Goal:
- scoring becomes a readable pipeline.

Required layers:
- raw topology facts
- patch scoring context
- corner hints
- seam relation hints
- structured rank
- debug explanation

Acceptance criteria:
- `_cf_score_candidate()` is either removed or reduced to a thin orchestrator;
- each scoring layer is inspectable in isolation.

## Phase 7 — Measure main frontier vs rescue gap

Goal:
- understand how much of rescue behavior is still missing from the main rank.

Tasks:
1. Measure stall frequency.
2. Measure when `tree_ingress` is needed.
3. Measure when `closure_follow` is needed.
4. Record which candidate classes main frontier still undervalues.

Acceptance criteria:
- telemetry tells us whether rescue integration is safe to consider next.

## Phase 8 — Separate alignment/drift roadmap

Goal:
- move row/column and accumulated drift handling into a dedicated layer.

Future topics here:
- rigid row/column constructs;
- drift redistribution;
- lattice/alignment pass;
- pre-transfer or post-frontier correction logic.

This phase is explicitly outside the current score refactor.

---

## 9. What must not happen during implementation

Do not:
- add corner score as a competing candidate type;
- add raw Junction score into main frontier rank;
- turn row/column diagnostics into normal frontier bonuses/penalties;
- replace one scalar score with an even longer scalar score;
- absorb rescue paths into main frontier too early;
- mix manual operations into this scoring refactor;
- perform large root/plan rewrites before frontier rank migration stabilizes.

---

## 10. Minimal first deliverable

The safest first PR should contain only:
1. `FrontierRank` in `solve_records.py`.
2. A helper that builds the rank from current signals.
3. Frontier selection switched to rank comparison.
4. Rank breakdown added to telemetry.
5. Documentation updated to say frontier is now structured-rank-based.
6. No rescue control-flow rewrite.
7. No row/column integration into frontier score.

If this first deliverable is stable, continue with later phases.

---

## 11. Acceptance criteria for the whole plan

The score refactor is considered successful when:
- frontier choice is rank-based rather than scalar-only;
- patch runtime context is explicit;
- corner facts are used as local features only;
- seam relation structure is richer and explicit;
- rescue paths still work and remain measurable;
- docs and telemetry explain why a candidate wins;
- row/column remains a separate alignment concern;
- manual operations remain a future extension layer, not part of the current core refactor.

---

## 12. Future compatibility with manual operations

Manual operations are the **next stage**, not part of this refactor.

This refactor should only prepare the ground for them by making patch/chain context queryable.

The scoring layer should expose enough stable context that a future manual system can ask:
- what is the current patch shape/profile?
- what is the current patch frontier context?
- what corner hints exist around this chain?
- what seam relation exists between these patches?
- what alignment diagnostics exist around this quilt?

Manual overrides must remain a separate layer over the scoring model, not a rewrite of its core assumptions.

---

## 13. Short handoff paragraph

Use this paragraph when handing the task to another AI agent:

> Preserve chain-first strongest-frontier. Patch and Chain remain the only primary solve units. Corner may contribute local chain features, but must not become an independent placement unit. Junction remains derived/global and must not become a primary runtime scoring entity in this phase. Row/Column drift handling belongs to a separate alignment layer, not to the main frontier score. Refactor path is: scalar score → structured frontier rank → explicit patch scoring context → corner/shape/seam enrichment → rescue-gap telemetry. Manual operations are a future layer on top of patch/chain context and are out of scope for the current score refactor.
