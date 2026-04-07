# CFTUV Axis-Authority vs Parameter-Authority Plan

Purpose: replace the current fragile / quad-like straighten-band behavior with a more universal chain-based model.

This plan is specifically written for the current state of the repo **after** the recent straighten / band / continuation / junction iterations.

The main requirement:
- keep thinking in terms of **chains, neighbors, junctions, corners, scaffold authority**
- do **not** fall back to a face-layout-specific solution
- do **not** build a quad-only solution
- do **not** depend on internal face types (quads / tris / ngons)

---

## Problem statement

The current system is already much better at:
- detecting band-like patches
- resolving patch axis / candidate axis
- using strong neighbors and frontier-first authority

But the remaining bug is deeper:

The system still mixes up:
1. **Who owns the axis/scaffold authority** for a straighten candidate chain
2. **Who owns the parameterization authority** along that chain

Right now, some of the recent logic still behaves too much like this:
- strong neighbor or opposite side gives axis
- then the candidate chain also inherits the other side's phase / stationing / point distribution

That is what creates the fragile quad-like behavior and breaks on triangulated S-cases.

---

# Core design change

## Principle 1 — Separate two authorities

For every straighten candidate chain, resolve **two separate authorities**:

### A. Axis / scaffold authority
This decides:
- orientation (`H_FRAME` / `V_FRAME`)
- signed direction
- start/end alignment in scaffold
- target span along the axis
- possibly width reference

### B. Parameter authority
This decides:
- how intermediate points are distributed between the start and end
- local stationing / normalized cumulative length along the chain

Critical rule:
- **Axis authority may come from neighbors / scaffold / paired candidate**
- **Parameter authority must stay local to the candidate chain by default**

Do **not** copy phase / point factors from opposite side or neighbor just because the patch is band-like.

---

## Principle 2 — Recent commits are not waste, but some logic must be cleaned up

The recent straighten/band iterations were useful because they:
- cleaned telemetry
- improved axis / band detection
- added junction-aware signals
- separated ingress and continuation more than before

Keep and reuse:
- telemetry fixes
- structural band detection signals
- junction-aware axis signals
- ingress vs continuation separation

But remove / disable / stop relying on logic that assumes band parameterization can be copied from the opposite side.

In particular, review and likely retire any behavior whose essence is:
- opposite placed chain provides axis factors / phase template
- candidate chain copies those fractions directly

That behavior is the quad-like part that should not remain the basis of the system.

---

# Part 1 — Introduce explicit authority model

## Goal

Make runtime placement reason in terms of explicit authority sources.

## Files to inspect

- `cftuv/frontier_state.py`
- `cftuv/frontier_eval.py`
- `cftuv/frontier_place.py`
- `cftuv/analysis_derived.py`
- `cftuv/analysis_records.py`

## Step 1. Add explicit chain authority summary for straighten candidates

Add a lightweight runtime/derived concept for straighten candidate chains, for example:
- `axis_authority_kind`
- `parameter_authority_kind`

Suggested values:

### Axis authority
- `DIRECT_STRONG_NEIGHBOR`
- `PAIRED_CANDIDATE`
- `PATCH_SELF_CONSENSUS`
- `NONE`

### Parameter authority
- `SELF_ARCLENGTH`
- `PAIRED_CANDIDATE` (not for v1 default)
- `NONE`

Important v1 rule:
- parameter authority should almost always be `SELF_ARCLENGTH`
- do not add a new copied-phase mechanism as default

## Step 2. Resolve axis authority in priority order

For each straighten side candidate chain, resolve axis authority with this priority:

### Level A — Direct strong-neighbor authority
If the candidate chain directly borders a strong H/V neighbor chain:
- use that neighbor as axis authority

This authority can provide:
- axis role
- signed direction
- target span estimate
- start/end scaffold alignment

### Level B — Paired-candidate authority
If the candidate chain itself has no direct strong neighbor, but its paired side candidate already has strong resolved authority:
- use that paired candidate as secondary axis authority

This provides:
- axis role
- signed direction
- target span reference
- maybe width reference

But not point-by-point phase copy.

### Level C — Patch self-consensus authority
If neither side has strong neighbor authority:
- build axis authority from the patch itself
- use patch summary / band grammar / candidate pair agreement

This can provide:
- axis role from patch band interpretation
- target span estimate from the pair of side candidates
- width estimate from caps / side agreement

---

# Part 2 — Clean out quad-like phase inheritance

## Goal

Remove the part of the runtime that still behaves like “opposite side gives my phase”.

## Files to inspect

- `cftuv/frontier_place.py`
- any helper that copies axis factors / fractions / point correspondence from opposite placed chain

## Step 3. Audit and remove opposite-side factor copying as a default band behavior

Find helpers that effectively do this:
- read UV points from opposite placed chain
- derive fractions / axis factors / stationing from that chain
- rebuild the candidate chain using those copied factors

That logic was useful for narrow quad-like cases, but it is the wrong default for a universal algorithm.

Action:
- remove it entirely, or
- gate it behind a disabled / legacy path not used by the new straighten-band flow

The new flow must not depend on opposite-side phase copy.

## Step 4. Keep only scaffold inheritance, not phase inheritance

After cleanup, strong neighbors / paired chains may still provide:
- axis
- sign
- span
- width / frame reference

But the candidate chain must keep its own local parameterization.

---

# Part 3 — Candidate chain local parameterization

## Goal

Make parameterization universal and chain-local.

## Step 5. Use candidate chain's own cumulative edge lengths as parameter authority

For every straighten candidate chain:
- compute normalized cumulative edge-length fractions from the chain itself
- use those fractions as the only default stationing for intermediate points

This must work identically regardless of whether the patch interior is:
- quads
- triangles
- mixed topology

Critical rule:
- the chain's own edge sequence defines its local parameterization
- not the opposite side's point distribution
- not the neighbor patch's internal vertex pattern

## Step 6. Map the local parameterization into the scaffold frame

Once axis authority resolves:
- axis direction
- target span
- start/end alignment

then place the candidate's points by mapping its own normalized `t_i` onto that span.

This is the key universal behavior.

Meaning:
- the chain is **follower in frame**,
- but **owner of its local parameterization**.

---

# Part 4 — Band patch self-consensus

## Goal

Handle the case where neither side candidate has a strong neighbor.

## Files to inspect

- `cftuv/analysis_derived.py`
- patch summary / band candidate logic
- runtime placement authority helpers

## Step 7. Use the paired side candidates as a self-consensus scaffold source

If both side candidates are weak / FREE / border / no strong neighbor authority, but the patch is still a valid band candidate:
- resolve axis from patch band interpretation
- estimate target span from the side pair itself

Suggested v1 estimate:
- robust mean / average of the two side-candidate total lengths
- not a copied template from one side to the other

## Step 8. Use caps only as band constraints, not phase templates

Cap-like start/end chains should provide:
- endpoints of the strip frame
- width constraints
- start/end alignment

But not point-by-point distribution for side chains.

This keeps the model chain-based and universal.

---

# Part 5 — Neighbor / junction / corner semantics

## Goal

Use the existing graph semantics, not face-layout assumptions.

## Step 9. Filter neighbor authority through junction/corner semantics

Do not let “first neighbor found” become authority automatically.

A neighbor may be authoritative for a chain only if the local semantics support it:
- shared edge relation is relevant
- corner/junction role supports the transition
- the chain is actually one of the band sides or caps

The existing corner/junction work should remain in service here.

## Step 10. Continue to use corner orthogonality for role propagation

Keep the already-correct direction of previous work:
- entry axis can come from a strong seam neighbor
- next chain role is resolved through corner orthogonality

But now combine that with the new authority split:
- axis may propagate through corner logic
- parameterization still stays local to the chain

---

# Part 6 — Runtime placement behavior

## Goal

Make the actual geometry placement reflect the new authority model.

## Step 11. For straighten side candidates, place by axis-authority + self-parameterization

When a candidate is promoted to straighten-band placement:
- do not build it from copied opposite-side phase
- build it from:
  - axis authority
  - target span
  - chain's own normalized cumulative lengths

In other words:
- scaffold frame is inherited
- internal stationing is local

## Step 12. Keep ingress special-case local

Ingress bridge chains can still inherit directly from strong neighbors.

But once the system proceeds into the actual band sides:
- parameter authority must switch back to the side chain itself
- not stay tied to ingress chain UV fractions

## Step 13. Align scoring / ranking with the same authority model

If a candidate has strong axis authority but only self parameter authority:
- ranking/scoring should still consider it a good straighten candidate
- but no component of scoring should assume copied opposite-side phase exists

That means scoring should reason about:
- strength of axis authority
- confidence of band candidate
- availability of paired side candidate / caps

Not about copied phase support.

---

# Part 7 — What to clean out from earlier attempts

## Goal

Prevent the agent from layering more heuristics on top of obsolete assumptions.

## Step 14. Explicit cleanup checklist

Before or during implementation, audit the recent straighten-band code and remove / retire behavior that embodies the old wrong assumption.

Things to clean out if present:
- opposite-side UV factor copying as the default band method
- any hidden “copy partner phase if point counts match” behavior in the main path
- logic that treats a paired chain as a point-distribution template rather than scaffold authority
- quad-like shortcuts that are now incompatible with the universal authority split

Do **not** remove useful work such as:
- telemetry role fixes
- band candidate detection
- corner-aware continuation logic
- junction-aware axis signals
- ingress vs continuation role separation

---

# Part 8 — Validation criteria

## Step 15. Validate on the current S-case

Expected outcome:
- `P2/P4` remain `band_candidate=Y`
- axis remains correct
- boundary chains still straighten coherently
- internal placement no longer depends on copied opposite-side phase
- triangulated band patches no longer collapse because of opposite-side phase mismatch

## Step 16. Re-test older working cases

Make sure the new model does not regress:
- quad-like cases that already looked good
- arc cases
- J-cases

Important:
- old good quad-like cases should still work, but only because the universal authority model also works there
- not because the runtime silently fell back to old copied-phase heuristics

---

# Implementation order

1. Audit and identify all remaining opposite-side / copied-phase logic in the straighten-band path
2. Introduce explicit `axis authority` vs `parameter authority`
3. Resolve axis authority from:
   - direct strong neighbor
   - paired candidate
   - patch self-consensus
4. Make parameter authority default to `SELF_ARCLENGTH`
5. Rebuild straighten candidate placement from:
   - inherited scaffold frame
   - local chain cumulative lengths
6. Remove / retire opposite-side phase copy from the main path
7. Validate on S-case, then regress older meshes

---

# Guardrails for Codex

- Do not add more one-off gates around old phase-copy logic.
- Do not add new face-type checks (`if triangulated`, `if quad-safe`) as the primary solution.
- Do not reintroduce opposite-side phase copying under a different name.
- Do not rewrite the whole frontier system.
- Reuse the recent useful work, but be willing to delete the specific quad-like parameterization shortcut if it is still in the main path.

---

# Final design summary for the agent

The intended universal model is:

- **Strong neighbors own the scaffold axis**.
- **Corners/junctions decide role transitions**.
- **Each candidate chain owns its own local parameterization**.
- **Paired candidate or patch self-consensus may provide span / frame fallback**.
- **No default opposite-side phase copy**.

That is the elegant chain-based solution we want.