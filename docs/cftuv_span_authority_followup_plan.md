# CFTUV Placement Frame / Span Authority Follow-up Plan

Purpose: continue after commit `d28d33d`.

Current state:
- opposite-side copied-phase logic was correctly removed from the main band path
- axis authority vs parameter authority was correctly separated
- visual result is still not fixed

Conclusion:
- the next missing layer is **placement frame / span authority**
- not another round of copied-phase cleanup
- not a return to quad-like heuristics

---

## Short diagnosis

The runtime now knows:
- who likely owns the axis
- that local stationing should come from `SELF_ARCLENGTH`

But it still does not fully know:
- what the actual target scaffold segment is for the candidate chain
- who owns the target span along the inherited axis
- how to rectify start/end anchors into that resolved scaffold frame before placing the chain

So the remaining bug is now:
- **good parameter authority, incomplete placement frame authority**

---

# Part 1 — Add explicit span/frame authority

## Goal

Extend the current authority split from:
- axis authority
- parameter authority

to also include:
- **span/frame authority**

## Step 1. Add a minimal runtime notion of span authority

For straightened H/V candidates, resolve a third decision:
- `span_authority_kind`

Suggested values:
- `DIRECT_STRONG_NEIGHBOR`
- `PAIRED_CANDIDATE`
- `PATCH_SELF_CONSENSUS`
- `NONE`

This may reuse the same source as axis authority, but it must be an explicit runtime decision.

## Step 2. Resolve a target scaffold segment for the candidate chain

Before building UV points for a straightened H/V chain, compute:
- resolved axis direction
- resolved start anchor in scaffold frame
- resolved end anchor in scaffold frame
- resolved target span along the axis

This should become the actual geometric controller for placement.

Important:
- `SELF_ARCLENGTH` alone is not enough
- local `t_i` only works after the target segment is correctly resolved

---

# Part 2 — Make strong authority geometric, not only semantic

## Goal

Turn authority metadata into actual placement behavior.

## Step 3. Direct strong-neighbor span authority

If a chain has `DIRECT_STRONG_NEIGHBOR` authority:
- use the strong neighbor / strong anchor relation to resolve the candidate's target span
- do not just classify the authority kind
- actually derive the final straight interval the candidate should occupy

What authority may provide:
- signed axis
- start/end alignment
- target span estimate
- width/frame reference if needed

But still not point-by-point phase.

## Step 4. Paired-candidate span authority

If direct strong-neighbor authority is absent, but the paired side candidate is already placed/resolved:
- use it as a secondary source for target span reference
- not as a point-distribution template

What it may provide:
- span reference
- axis confirmation
- width/frame consistency

What it must not provide:
- copied stationing / copied phase fractions

## Step 5. Patch self-consensus span authority

If neither side has direct strong authority, but the patch is still a valid band candidate:
- resolve target span from the patch itself
- use the side-candidate pair and cap constraints

Suggested v1:
- robust mean / average of side candidate total lengths
- cap agreement as a stabilizer
- no copied opposite-side phase

---

# Part 3 — Anchor rectification in the resolved frame

## Goal

Make sure straightened candidates are built inside the correct scaffold frame.

## Step 6. Rectify anchors into the resolved frame before point build

For a straightened candidate chain:
- do not directly trust raw start/end anchors as final segment endpoints
- first rectify them into the resolved scaffold frame from span authority

This means:
- if the chain is H, endpoints should be rectified onto the resolved H segment
- if the chain is V, endpoints should be rectified onto the resolved V segment
- if authority says a larger/smaller target span is needed, honor that before distributing points

## Step 7. Only then apply SELF_ARCLENGTH stationing

After the resolved scaffold segment is known:
- take the candidate chain's own cumulative edge-length fractions
- map those fractions onto the resolved segment

This is the intended universal behavior:
- scaffold frame inherited
- local stationing owned by the candidate chain

---

# Part 4 — Where to implement

## Files to inspect

- `cftuv/frontier_state.py`
- `cftuv/frontier_eval.py`
- `cftuv/frontier_place.py`
- `cftuv/frontier_rescue.py`
- `cftuv/solve_frontier.py`
- `cftuv/model.py`

## Step 8. Do not just add metadata fields

Avoid another iteration that only stores:
- authority kinds in `ScaffoldChainPlacement`
- debug labels
- telemetry values

This time the new authority must directly change:
- segment resolution
- anchor rectification
- final straight H/V interval

---

# Part 5 — Validation target

## Step 9. Validate on the problematic S-case

Expected result:
- band detection stays correct
- copied-phase behavior remains removed
- candidate chains still use `SELF_ARCLENGTH`
- but now their target straight segment is resolved correctly from actual frame/span authority
- visual placement should improve because stationing is now mapped into the right segment instead of the wrong or under-resolved one

## Step 10. Re-test older cases

Confirm no regression on:
- older Arc cases
- J cases
- previously acceptable quad-like cases

Important:
- if old quad-like cases still work, they should work because the new authority model is general enough
- not because copied-phase logic came back implicitly

---

# Guardrails

- Do not reintroduce opposite-side phase copying.
- Do not add new quad-only shortcuts.
- Do not solve this with another layer of local gating on top of old assumptions.
- Do not stop at metadata / telemetry.
- This step must make authority actually drive geometry.

---

# Deliverable expectation for Codex

At the end of this iteration, the runtime should not merely know:
- who owns axis authority
- who owns parameter authority

It should also know and use:
- **who owns the resolved placement frame/span for the chain**
- and build the candidate from:
  - inherited frame/span
  - local self-arclength stationing

That is the next concrete step after `d28d33d`.