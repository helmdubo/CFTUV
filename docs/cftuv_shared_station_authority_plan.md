# CFTUV Shared Station Authority Follow-up Plan

Purpose: continue after commit `1ff56c599dba8beec260e49fc25085e2555f8dea`.

Current state:
- axis authority exists
- span/frame authority exists
- copied opposite-side phase was removed from the main path
- straightened H/V chains now place into a resolved scaffold segment using local self-arclength stationing

Remaining problem:
- paired long-side chains of a band patch still use **independent local stationing**
- even when they already share the same resolved axis and span
- this causes cross-strip correspondence drift when the two long sides have different local edge-length patterns

Conclusion:
- the next missing layer is **shared station authority**
- not another axis tweak
- not another span tweak
- not a return to copied-phase from one side

---

## Core design goal

For a confirmed or near-confirmed band patch, the two paired long-side candidates must use:
- the same band axis
- the same target span/frame
- and now also the same **station domain**

But this station domain must belong to the **pair / patch consensus**, not to one side copied onto the other.

Critical rule:
- do **not** reintroduce “left side owns phase” or “right side owns phase”
- do **not** copy fractions from one side’s UV points directly
- instead build a **shared consensual station map** for the pair, then let both sides place into it

---

# Part 1 — Add explicit station authority layer

## Goal

Extend the current authority stack from:
- axis authority
- span authority
- parameter authority

to also include:
- **station authority**

## Step 1. Add `StationAuthorityKind`

Introduce a new enum, for example:
- `NONE`
- `PAIRED_BAND_CONSENSUS`
- `PATCH_SELF_CONSENSUS`
- `SELF_ONLY`

Suggested interpretation:
- `PAIRED_BAND_CONSENSUS` = both long-side candidates share one consensual station map
- `PATCH_SELF_CONSENSUS` = patch-level fallback if pair data is partial
- `SELF_ONLY` = no shared stationing is available, fall back to local chain stationing

Important:
- `SELF_ONLY` is the fallback, not the ideal mode for paired band sides

## Step 2. Store station authority in placement metadata

Add `station_authority_kind` to `ScaffoldChainPlacement`.

This is useful for:
- debugging
- telemetry
- verifying when a band side is truly using shared stationing vs fallback self stationing

But unlike earlier iterations, this must not stop at metadata.

---

# Part 2 — Define the shared station map

## Goal

Create a pair-owned / patch-owned station domain for long-side candidates.

## Step 3. Identify the paired long-side candidates

Reuse existing band / loop / corner / paired-candidate logic.

For a band-like patch, identify the two long-side candidates that should correspond across the strip.

Important:
- this is not about copying one side to the other
- this is about identifying the **pair that should share stations**

## Step 4. Build a consensual station map for the pair

For v1, build the station map from pair consensus rather than from either side individually.

Recommended v1 approach:
- compute normalized cumulative edge-length fractions for side A
- compute normalized cumulative edge-length fractions for side B
- resample / merge them into one shared sorted station vector in `[0, 1]`
- use the merged / consensual stations as the strip station domain

Examples of acceptable v1 strategies:
- union of both normalized station sets, then simplify if needed
- average-based consensual stations when point counts are compatible
- robust merged station set with small epsilon clustering

Important constraints:
- the shared station map must not simply equal side A's fractions
- the shared station map must not simply equal side B's fractions
- it must be built from both sides as a consensus

## Step 5. Keep the shared station map independent from face layout

Do not derive the station map from internal face types.
Do not add quad-only logic.
Do not inspect interior triangles/quads as the main rule.

Use only:
- chain-local edge-length parameterizations
- band pairing
- existing corner/junction/loop semantics

This keeps the model universal and chain-based.

---

# Part 3 — Use shared station authority in placement

## Goal

Make station authority actually drive geometry for paired long-side band chains.

## Step 6. Add runtime helper to resolve shared station authority

In runtime policy or placement helpers, add something like:
- `resolve_station_authority_kind(...)`
- `resolve_shared_station_map(...)`

Behavior:
- if the chain is a paired long-side band candidate and enough pair data exists -> `PAIRED_BAND_CONSENSUS`
- if only patch-level fallback exists -> `PATCH_SELF_CONSENSUS`
- otherwise -> `SELF_ONLY`

## Step 7. Use shared station map instead of local self-arclength for paired band sides

This is the critical behavior change.

For a straightened long-side candidate with shared station authority:
- do not place intermediate points with only its own `cumlen / total_len`
- instead place points according to the resolved shared station map on the already-resolved frame/span segment

Interpretation:
- axis/frame comes from existing axis/span authority
- station positions come from shared station authority

## Step 8. Keep self-arclength as fallback only

If shared station authority cannot be resolved robustly:
- keep the current fallback path using local self-arclength

This preserves robustness for incomplete or weak cases.

But for proper paired band sides, `SELF_ARCLENGTH` should no longer be the only mode.

---

# Part 4 — Do not regress into copied-phase

## Goal

Make sure the new station layer is truly consensual, not a disguised rollback.

## Step 9. Explicitly forbid one-sided station copying in the main path

Audit any new implementation carefully.

Do **not** allow the main path to become:
- “use partner UV points as template”
- “copy partner fractions if point counts match”
- “take whichever side is already placed and reuse its stationing verbatim”

If a side is used as seed/reference, it may only contribute to the pair consensus, not become the sole authority.

## Step 10. Preserve the good parts of the previous commits

Keep:
- copied-phase cleanup from `d28d33d`
- axis authority split
- span/frame authority from `1ff56c5`
- anchor rectification and resolved frame segment logic

Do not rewrite those layers.

This step must sit **on top of** them.

---

# Part 5 — Implementation details and integration points

## Files to inspect

- `cftuv/model.py`
- `cftuv/frontier_state.py`
- `cftuv/frontier_place.py`
- `cftuv/frontier_eval.py`
- `cftuv/frontier_rescue.py`
- `cftuv/solve_frontier.py`

## Step 11. Add station-authority helpers in runtime policy

Suggested helpers:
- `_paired_band_station_sources(...)`
- `resolve_station_authority_kind(...)`
- `resolve_shared_station_map(...)`

These helpers should use existing paired-candidate / band summary / loop ordering data.

## Step 12. Add a new straight-chain build path that accepts explicit station values

You already have frame placement from resolved frame segment.

Add a path like:
- build straight chain from resolved segment + explicit normalized station list

This should be separate from:
- pure self-arclength interpolation

The new builder should:
- take `start_uv`, `end_uv`, `role`, `stations`
- place points by those normalized stations

## Step 13. Wire station authority into main/rescue placement

When a straightened H/V chain is a paired band side:
- resolve frame segment as today
- then choose stations from shared station authority if available
- otherwise fall back to self-arclength

Make sure rescue / closure paths use the same station resolution policy.

---

# Part 6 — Validation criteria

## Step 14. Validate on the problematic S-case

Expected result:
- band detection stays correct
- axis remains correct
- span authority remains correct
- paired long-side chains now share one consensual station map
- internal cross-strip correspondence improves because the two sides are no longer independently stationing themselves

This should be the first test where the visual result has a real chance to improve in the exact place still failing today.

## Step 15. Re-test older cases

Confirm no regression on:
- Arc cases
- J cases
- earlier acceptable band-like cases

Important:
- old good cases should still work under the new general station model
- not because a hidden copied-phase shortcut came back

---

# Guardrails for Codex

- Do not add another round of axis/span heuristics instead of station logic.
- Do not reintroduce one-sided copied-phase under another name.
- Do not make station authority depend on face types.
- Do not stop at metadata or debug labels.
- This step must make shared station authority drive actual geometry placement for paired band sides.

---

# Final design summary

After this change, the intended model should be:

- **Axis authority** decides orientation.
- **Span authority** decides the resolved frame segment.
- **Station authority** decides the shared station map for paired long-side chains.
- **Parameter fallback** remains local self-arclength when no shared station authority is available.

This is the next concrete step after `1ff56c599dba8beec260e49fc25085e2555f8dea`.