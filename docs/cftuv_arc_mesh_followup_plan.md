# CFTUV Arc Mesh Follow-up Plan

Purpose: fix the two issues found on the Arc Mesh test after Phase C (`straighten_strips`).

Scope:
- Issue A: `P2` becomes its own quilt even though structural analysis says the strip is strong.
- Issue B: `P1C1 / P1C3` can mirror, which makes `P1C2` pin almost diagonally.

Non-goals for this task:
- Do **not** implement whole-patch straighten.
- Do **not** build cross-quilt runtime stitching.
- Do **not** redesign the whole solve pipeline.

---

## Summary of the intended solution

There are **two separate fixes**:

1. **Planning fix**: make quilt formation structural-aware, so inherited strong FREE chains can participate in attachment planning and the correct patches land in the same quilt from the start.
2. **Placement-direction fix**: make direction/sign resolution use `effective_placement_role`, so inherited FREE chains do not straighten with mirrored axis direction.

Implement them in this order:

1. Fix mirror / diagonal pin bug first.
2. Then fix quilt planning.

---

# Part 1 — Fix mirrored inherited strip direction

## Goal

If a chain is placed as effective `H_FRAME` / `V_FRAME`, then **direction selection must also respect that effective role**.

Today the geometry mode is frame-like, but direction/sign can still come from raw FREE-chain logic. That can flip the axis sign and create mirrored placement.

## Files to inspect

- `cftuv/frontier_place.py`
- `cftuv/frontier_eval.py`
- `cftuv/frontier_rescue.py`
- `cftuv/frontier_state.py`

## Step 1. Add focused debug output for the failing case

Before changing logic, add temporary debug prints for chains placed with inherited role.

For every chain where:
- `chain.frame_role == FREE`
- `effective_placement_role(...) != FREE`

print:
- `chain_ref`
- raw role
- effective role
- inherited source patch id (if available from `inherited_role_map`)
- start/end anchor kinds and refs
- base direction from `_cf_determine_direction(...)`
- final snapped axis direction actually used for placement
- whether direction came from anchor inheritance / override / default axis

Keep these prints temporary and easy to remove.

## Step 2. Introduce effective-role-aware direction helper

Add one small helper in `frontier_place.py`, for example:

- `_cf_determine_direction_for_role(chain, node, role)`

Behavior:
- If `role == H_FRAME`: derive the sign using patch basis and return `(±1, 0)`.
- If `role == V_FRAME`: derive the sign using patch basis and return `(0, ±1)`.
- If `role == FREE`: keep current FREE behavior.

Important:
- This helper must use the **passed role**, not `chain.frame_role`.
- Do not silently fall back to raw FREE logic when `effective_role` is H/V.

## Step 3. Use that helper everywhere placement direction is seeded

Update these code paths so they pass the effective role into direction resolution:

- `_cf_build_seed_placement(...)`
- `_cf_place_chain(...)`
- any direction inheritance / direction override path used by frontier candidate placement
- tree-ingress rescue path if it computes direction separately

Rule:
- if chain is placed with `effective_role != FREE`, direction/sign logic must also behave as that role.

## Step 4. Make anchor-direction inheritance role-aware

Review any helper that inherits direction from neighbor anchors / same-patch chains.

If that logic still compares only raw `chain.frame_role`, update it so inherited FREE chains can participate as their effective H/V role.

Do **not** rewrite the full system. Only make it consistent with `effective_placement_role`.

## Step 5. Fix placement metadata consistency if still needed

Check that all runtime metadata use the same effective role when geometry is placed with effective role.

Already fixed once for counters, but verify these too:
- `ScaffoldChainPlacement.frame_role`
- telemetry/debug rows that report placement role
- any downstream code that reads `placement.frame_role` to classify frame/free state

If geometry is placed as frame-like, metadata must not still claim it is FREE.

## Step 6. Validate on Arc Mesh

Expected result for Part 1:
- `P1C1 / P1C3` no longer mirror
- `P1C2` no longer gets diagonal pinning caused by mirrored side
- inherited straightened chains keep correct axis sign on repeated runs

---

# Part 2 — Make quilt planning structural-aware

## Goal

The correct fix is **not** runtime cross-quilt continuity.

The correct fix is:
- quilts should already be formed correctly during planning,
- using structural information from analysis,
- so strong inherited FREE chains can contribute to attachment candidate validity.

## Files to inspect

- `cftuv/solve_planning.py`
- `cftuv/analysis.py`
- optionally `cftuv/analysis_records.py` if a small read-only payload is needed

## Step 7. Add a planning-level semantic bridge

Expose a small structural payload for planning.

Minimal acceptable first version:
- reuse `build_neighbor_inherited_roles(graph)` from analysis
- build a lightweight planning helper that can answer:
  - raw role for chain
  - effective planning role for chain (`H_FRAME`, `V_FRAME`, or `FREE`)

Important:
- planning must **read** structural facts from analysis
- planning must **not** reimplement structural interpretation independently

## Step 8. Add planning helper for effective seam role

In `solve_planning.py`, add one helper like:

- `_effective_planning_role(chain_ref, chain, inherited_role_map)`

Behavior:
- raw H/V stays raw H/V
- FREE with inherited H/V becomes effective H/V
- other FREE stays FREE

This is the planning analogue of `effective_placement_role`, but for solve planning only.

## Step 9. Make attachment pair scoring use effective planning role

Update the pair/attachment scoring path so it no longer relies only on raw roles.

Review and adapt at least these pieces:
- `_frame_continuation_strength(...)`
- `_rank_chain_pairs(...)`
- `_best_chain_pair(...)`
- `_build_attachment_candidate(...)`

Key rule:
- if a seam pair is raw FREE+FREE but structurally acts like H/V continuation, it must not be treated as zero-support by default.

## Step 10. Relax the raw FREE+FREE hard reject gate

Today `_build_attachment_candidate(...)` rejects the pair when:
- `best_pair.frame_continuation <= 0.0`

This gate is currently too raw-role-dependent.

Change it so:
- raw FREE+FREE with **no** structural support can still be rejected
- but structurally supported inherited H/V continuation is allowed through

Keep the score weaker than true H/V continuation.
Do not give full parity on day one.

## Step 11. Keep the preference conservative

Inherited planning support should be:
- stronger than unsupported FREE seams
- weaker than true raw H/V seams

So in ranking/score terms:
- true H/V continuation = strongest
- inherited H/V continuation = medium/high
- raw FREE-only continuation = weak or rejected

Do not make inherited seams dominate everything.

## Step 12. Ensure `P0-P1-P2` can land in one quilt when structural support exists

Re-run planning on the Arc Mesh case.

Expected result:
- `P2` should no longer be forced into a separate quilt if its only blocker was the raw FREE+FREE seam interpretation.
- quilt formation should become structural-aware enough that the correct neighboring patches can enter the same quilt.

---

# Validation checklist

## After Part 1

- [ ] inherited FREE chains use effective role for geometry mode
- [ ] inherited FREE chains use effective role for direction/sign
- [ ] no mirror on `P1C1 / P1C3`
- [ ] `P1C2` no longer pins diagonally because of mirrored side

## After Part 2

- [ ] solve planning reads structural facts from analysis, not duplicated logic
- [ ] raw FREE+FREE seams with inherited structural support can produce attachment candidates
- [ ] `P2` can join the same quilt when structure supports it
- [ ] no runtime cross-quilt stitching hacks were added

---

# Implementation notes for Codex

- Make **small, reviewable commits**.
- Part 1 and Part 2 should be separate commits.
- Keep temporary debug prints only while diagnosing; remove or gate them before final commit.
- Prefer small helpers over broad rewrites.
- If a choice appears between:
  - patching frontier to connect across quilts
  - or improving quilt planning
  choose **improving quilt planning**.

---

# Deliverable expectation

At the end of this task, the system should:
- place inherited straightened strip chains without mirrored direction bugs,
- and plan quilts using structural support strongly enough that the Arc Mesh no longer fragments into unnecessary isolated quilts.
