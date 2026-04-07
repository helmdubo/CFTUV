# CFTUV Junction-Aware Follow-up Plan

Purpose: address the new J-case failure mode after commits `4030e37` and `439e6a7`.

Problem summary:
- The recent fixes improved effective-role direction and structural-aware quilt planning.
- However, J-case logs show that `P2` / `P4` still behave like partially straightened patches even though their patch-level structural state is still unresolved:
  - `spine_axis = FREE`
  - `inherited_spines = 1`
  - `side_conf = 0.00`
  - `strip_conf = 0.54`
- This means the system currently trusts inherited chain-level support earlier than it should trust patch-level straighten behavior.

Main design idea:
- Keep **run / patch summary** as the primary long-range shape layer.
- Use **junction interpretation** as a local semantic arbiter:
  - continuation
  - turn
  - bridge
  - branch
  - ambiguous
- Junction should not replace run interpretation.
- Junction should help decide **whether inherited support is strong enough to allow aggressive straighten-like placement**.

Non-goals:
- Do not redesign the whole topology model.
- Do not replace run-based structural interpretation with junction-only logic.
- Do not add runtime cross-quilt stitching hacks.
- Do not implement whole-patch straighten in this task.

---

## Architectural intent

### Existing layers

1. **Original topology facts**
   - chains
   - runs
   - corners
   - junctions

2. **Current interpretation**
   - run structural roles
   - patch structural summary
   - inherited chain roles for planning / placement

### Missing layer for the J-case

The system still lacks a local semantic decision at patch boundaries:

- does the axis really continue through this vertex?
- is this only a turn?
- is this only a bridge through a tiny mediator patch?
- is the node too ambiguous to justify axis-level straighten behavior?

This is the job of **junction interpretation**.

---

# Part 1 — Add junction interpretation facts

## Goal

Extend the current diagnostic `_Junction` layer with a small, derived interpretation layer that can answer:
- continuation?
- turn?
- bridge?
- branch?
- ambiguous?

This layer must stay **derived / semantic**, not a new source-of-truth topology model.

## Files to inspect

- `cftuv/analysis_junctions.py`
- `cftuv/analysis_records.py`
- `cftuv/analysis_derived.py`
- any existing structural-summary code that already consumes runs/junctions

## Step 1. Add a companion interpretation record for junctions

Do **not** overload `_Junction` with decision fields if that makes the topology record impure.

Preferred approach:
- add a companion derived record, for example `_JunctionStructuralRole` or `_JunctionInterpretation`

Suggested minimal fields:
- `vert_index`
- `patch_id`
- `kind` = `CONTINUATION | TURN | BRIDGE | BRANCH | AMBIGUOUS | FREE`
- `dominant_axis` = `H_FRAME | V_FRAME | FREE`
- `supports_inherited_axis: bool`
- `confidence: float`
- `continuation_chain_refs: tuple[...]`
- `turn_chain_refs: tuple[...]`

Important:
- this is **per patch in junction context**, not just global per vertex
- the same junction vertex can mean different things for different patches

## Step 2. Derive continuation vs turn vs bridge from existing junction facts

Use what already exists in `_Junction`:
- `role_signature`
- `chain_refs`
- `run_endpoint_refs`
- `patch_ids`
- `valence`
- H/V/FREE counts

Suggested rules for v1:

### CONTINUATION
When the patch contributes a chain/run that aligns with a same-axis frame path through the junction.

### TURN
When the patch participates through an orthogonal H↔V corner relation, not same-axis continuation.

### BRIDGE
When the patch is only connected via a tiny mediator / bridge relation and should support connectivity but not become a strong axis authority.

### BRANCH
When the junction fans into multiple competing continuations for the same patch / axis.

### AMBIGUOUS
When continuation cannot be resolved confidently from the junction facts.

Keep v1 simple and conservative.

## Step 3. Store junction interpretations in derived topology

Add the new mapping to `_PatchGraphDerivedTopology`, for example:
- `junction_structural_roles`

If a similar field already exists but is too weak, extend it rather than adding a second parallel structure.

The key principle remains:
- original `_Junction` = fact
- interpreted junction role = semantic meaning

---

# Part 2 — Use junction interpretation as a gate, not as a replacement

## Goal

Use junction interpretation to decide when inherited chain support is strong enough for:
- planning continuity
- aggressive frame-like placement

Do **not** let a single inherited chain automatically promote a whole patch into a fully axis-resolved strip.

## Step 4. Distinguish chain-level inherited role from patch-level axis confidence

Keep these concepts separate:

- `effective_chain_role`
- `effective_patch_axis`
- `junction-supported continuation`

Current J-case problem:
- chain-level inherited support exists
- patch-level axis is unresolved (`spine_axis = FREE`)
- placement still behaves too aggressively

Add a small patch-level gate, for example:
- `axis_confidence`
- `has_resolved_patch_axis`
- `single_sided_inherited_support`

At minimum, detect this unsafe state:
- `spine_axis == FREE`
- `inherited_spines == 1`
- `side_conf` is low
- continuation is not confirmed by junction interpretation

In that state:
- planning may still use inherited support
- placement must stay conservative

## Step 5. Add a placement gate using junction interpretation

Before treating an inherited FREE chain like a strong H/V placement carrier, require more than raw inherited role.

V1 policy:

Allow aggressive frame-like placement only if at least one of these is true:
- patch summary already resolved a patch axis (`spine_axis != FREE`)
- the patch has multiple inherited spines with same axis
- junction interpretation says this chain participates in `CONTINUATION` with sufficient confidence
- anchors confirm same-axis support on both ends

Otherwise:
- keep the inherited role for planning / connectivity / mild scoring
- do not let it fully control axis-level straighten behavior

## Step 6. Use junction interpretation in planning only as a confidence modifier

Planning should remain mostly seam/run based.

Junction should help answer:
- is this inherited seam support truly a continuation?
- or only a local turn / bridge?

So in `solve_planning.py`:
- do not replace `_effective_planning_role(...)`
- add junction-aware confidence modifiers around raw inherited support

Guideline:
- true raw H/V continuation = strongest
- inherited H/V + junction CONTINUATION = medium/high
- inherited H/V + junction TURN/BRIDGE = weak / connectivity only
- inherited H/V + junction AMBIGUOUS = conservative

---

# Part 3 — Patch summary follow-up

## Goal

Patch summary must better express unresolved vs partially inherited axis state.

Today the logs show:
- `inherited_spines = 1`
- but `spine_axis = FREE`

That is useful but not expressive enough for downstream behavior.

## Step 7. Add patch-level unresolved-axis signals

Without introducing a full new taxonomy, add one or two summary facts such as:
- `inherited_axis_candidate`
- `axis_confidence`
- `has_single_sided_inherited_support`
- `junction_supported_axis`

This allows downstream code to distinguish:
- fully resolved strip axis
- partially inherited support
- unresolved / ambiguous patch

## Step 8. Do not over-promote low-confidence patches

For patches like `P2/P4`, where current summary looks like:
- `spine_axis = FREE`
- `inherited_spines = 1`
- `side_conf = 0.00`
- `strip_conf = 0.54`

make sure the system does **not** treat them like fully resolved strip patches.

These patches can still:
- join the correct quilt
- inherit connectivity support
- receive anchors from neighbors

But they should not automatically receive full axis-straighten behavior until junction + patch summary agree.

---

# Part 4 — Validation strategy

## Step 9. Validate on the J-case mesh

Expected outcome:
- `P2` / `P4` can still participate in the correct quilt structure
- `P5` can still act as a mediator / anchor provider
- but `P2` / `P4` should no longer be over-promoted into a fully resolved vertical strip behavior when the patch summary is still unresolved

## Step 10. Re-test earlier Arc-case regressions

Make sure the new junction gate does **not** break the earlier Arc-case wins:
- no mirrored inherited strip placement
- no diagonal pin regression
- no re-fragmentation into unnecessary separate quilts

---

# Implementation order

1. Add junction interpretation companion record
2. Derive basic `CONTINUATION / TURN / BRIDGE / AMBIGUOUS` semantics
3. Store that in derived topology
4. Add patch-level unresolved-axis signal(s)
5. Gate aggressive inherited placement using patch summary + junction semantics
6. Use junction semantics as a conservative modifier in planning
7. Validate on J-case, then re-run Arc-case

---

# Guardrails for Codex

- Prefer **small, reviewable commits**.
- First commit: analysis-layer junction interpretation only.
- Second commit: placement gate using new junction signals.
- Third commit if needed: planning confidence adjustment.
- Keep planning and placement responsibilities distinct.
- Junction is a **local semantic arbiter**, not a replacement for run-based structural interpretation.

---

# Done criteria

- Junction interpretation exists as a derived semantic layer
- The system can distinguish continuation vs turn vs bridge at shared patch vertices
- `P2/P4` no longer behave like fully axis-resolved strips when only single inherited support exists
- Quilt connectivity remains good
- Arc-case improvements remain intact
