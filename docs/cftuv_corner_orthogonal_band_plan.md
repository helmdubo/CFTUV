# CFTUV Corner-Orthogonal Band Plan

Purpose: fix the current J-mesh failure mode where a strong inherited ingress chain gets the correct seam axis, but that axis then propagates incorrectly through the rest of the unresolved patch.

Observed problem on J-mesh:
- `P2` and `P4` still build as vertical on UV after entering from `P5`.
- The shared seam with `P5` is genuinely `V_FRAME`, so the ingress chain can inherit `V` correctly.
- The failure happens after ingress: adjacent FREE chains continue as if they are same-axis continuation, even though in 3D they are orthogonal to the ingress seam.
- Current analysis already reports the unresolved state clearly:
  - `spine_axis = FREE`
  - `inherited_spines = 1`
  - `axis_candidate = V_FRAME`
  - `junction_axis = V_FRAME`
  - `single_sided = Y`
- In the concrete J-mesh logs, `P2/P4` each have one large FREE run `[C1 C2 C3]`, while `C2` is the short bridge to `P5` and `P5` itself is a strict `2V + 2H` wall patch. This means the system is currently over-propagating the bridge axis to the whole FREE run instead of using corner orthogonality to determine the next chain role.

Key conclusion:
- The problem is no longer quilt planning.
- The problem is no longer telemetry.
- The problem is no longer just "gate inherited same-patch continuation".
- The missing piece is **corner-aware role propagation** inside a potential band-like patch.

Non-goals:
- Do not redesign the topology model.
- Do not replace existing run / junction interpretation with a new parallel system.
- Do not add runtime cross-quilt stitching.
- Do not implement general whole-patch straighten for every shape.

---

## Core idea

A strong H/V chain should not color the whole FREE run with the same axis.

Instead:
1. A strong chain defines an **entry band axis** only for the contact seam.
2. Propagation to the next chain inside the patch must go **through the corner**.
3. If the corner is a normal 90° band corner, the adjacent FREE chain should become **orthogonal** to the ingress axis.
4. Full straighten / trim-band promotion should happen only after validating the opposite band and side consistency.

In short:
- seam inheritance gives the axis of the **entry chain**,
- corner semantics determine the axis of the **next chain**,
- patch-level band validation decides whether the whole patch is a true straighten-band candidate.

---

# Part 1 — Introduce corner-aware chain propagation

## Goal

Stop treating inherited support as run-wide axis truth.

Instead, determine candidate H/V roles chain-by-chain using:
- ingress seam axis
- corner orthogonality
- local patch cycle order

## Files to inspect

- `cftuv/frontier_state.py`
- `cftuv/frontier_eval.py`
- `cftuv/frontier_place.py`
- `cftuv/analysis_derived.py`
- `cftuv/analysis_records.py`

## Step 1. Add a new runtime concept: continuation placement role

Keep the existing concepts:
- effective planning / inherited role
- candidate placement role

Add one more explicit concept for same-patch propagation, for example:
- `continuation_placement_role(...)`

Meaning:
- the role to use when a new candidate is being inferred from an already placed chain inside the same patch
- this role may differ from the ingress role

Important:
- ingress role may stay equal to the inherited seam axis
- continuation role must be corner-aware and may become orthogonal or FREE

## Step 2. Use local corner topology to derive orthogonal continuation

For a candidate FREE chain adjacent to a placed chain in the same patch:
- inspect the shared patch corner between them
- if the corner is a 90° corner and the placed chain has a strong H/V placement role,
  then the adjacent FREE chain should receive the **orthogonal** role, not the same role

Rule for v1:
- placed `V` through a normal 90° band corner -> next FREE candidate becomes `H`
- placed `H` through a normal 90° band corner -> next FREE candidate becomes `V`
- if the corner semantics are ambiguous, leave the next chain as `FREE`

Do not propagate same-axis H/V blindly across corner boundaries.

## Step 3. Restrict this to target cases that look like band-like patches

Do not apply this rule globally to every FREE chain.

Only enable corner-orthogonal continuation when the patch already looks like a straighten-band candidate or near-candidate:
- unresolved `spine_axis == FREE`
- single-sided inherited support exists
- patch has one strong entry chain and corner-driven side candidates
- no strong evidence of a branch / blob shape

This should be a conservative heuristic, not a universal rewrite.

---

# Part 2 — Add a band-like patch grammar

## Goal

Recognize the pattern the user described:
- strong start/end H/V chains (caps / band ends)
- two side FREE chains between them
- corner transitions should be orthogonal
- side width should be reasonably stable

## Files to inspect

- `cftuv/analysis_records.py`
- `cftuv/analysis_derived.py`
- existing patch summary / structural summary consumers

## Step 4. Add minimal band grammar signals to patch summary

Do not replace the current summary. Extend it minimally.

Suggested new fields:
- `band_cap_count`
- `band_side_candidate_count`
- `band_opposite_cap_length_ratio`
- `band_width_stability`
- `band_candidate: bool`

Definitions for v1:
- **cap** = short H/V chain likely serving as start/end band chain
- **side candidate** = FREE chain adjacent to cap through orthogonal corner
- **opposite cap ratio** = how similar the two cap lengths are
- **width stability** = how consistent the two side-chain distances / spans are

## Step 5. Detect the simplest useful band pattern first

First support the simple 4-chain cycle shape:
- one H/V cap
- one FREE side
- one H/V cap
- one FREE side

For the current J-case, also handle the practical variant where a FREE run is currently split semantically into:
- side FREE
- bridge FREE
- side FREE

Do not force this into a full generalized grammar yet.

V1 target:
- detect whether the patch has two approximately opposing cap-like H/V chains
- detect whether the chains between them are orthogonal side candidates rather than same-axis continuation

## Step 6. Validate start/end agreement before promoting straighten behavior

A patch becomes a true band candidate only if:
- start/end cap chains are approximately similar in length
- the path between them is corner-orthogonal
- side width does not vary too much
- no branch-like junction pattern invalidates the interpretation

If width is clearly unstable or caps disagree strongly:
- do not promote to trim-like straighten
- keep conformal / mixed behavior

---

# Part 3 — Separate ingress axis from patch axis

## Goal

Prevent the ingress seam axis from becoming patch backbone truth too early.

## Step 7. Keep ingress axis local

For unresolved band-like patches:
- ingress bridge chain may inherit `H` or `V` from the neighbor seam
- but that inherited seam axis must stay local to the ingress chain until corner propagation confirms the next chains

In other words:
- `entry axis != patch backbone axis` by default

## Step 8. Promote patch axis only after corner and cap checks succeed

Only after the patch passes the band grammar checks should the system allow:
- whole-patch strong H/V continuation behavior
- backbone accounting as resolved band structure
- stronger straighten placement behavior for later chains

Before that point:
- planning connectivity can still use inherited support
- ingress can still use inherited seam role
- same-patch continuation must remain corner-aware and conservative

---

# Part 4 — Runtime behavior changes

## Goal

Translate the new grammar into actual placement behavior.

## Step 9. Make same-patch candidate scoring use continuation role, not raw inherited role

Current issue:
- even after recent fixes, scoring / ranking still appears to prefer the inherited seam axis too strongly

Update candidate evaluation so that same-patch FREE candidates are ranked using:
- corner-derived continuation role
- not just inherited seam role from the original ingress contact

Important:
- keep ingress candidate behavior unchanged where it is already correct
- only alter continuation behavior for same-patch propagation

## Step 10. Use continuation role in geometry placement

When placing a same-patch continuation candidate:
- use the corner-derived continuation role for direction / line-building
- not the raw inherited seam axis

This is the actual geometric fix that should stop `P2/P4` from simply extending downward as vertical strips in UV.

## Step 11. Keep rescue aligned

Any rescue path (`tree_ingress`, `closure_follow`, other recovery logic) must use the same continuation-role logic.

Do not allow rescue to silently reintroduce same-axis inherited placement where main frontier would now choose orthogonal continuation or FREE fallback.

---

# Part 5 — Validation on current J-case

## Step 12. Validate the exact intended behavior

On the current J-mesh:
- `P5` may still provide a valid `V` ingress axis to the bridge chain (`C2`-like contact)
- but adjacent FREE chains (`C1/C3`-like) should no longer automatically keep the same vertical role
- through the patch corners, they should either:
  - become orthogonal side candidates,
  - or remain FREE if the band grammar is not sufficiently confirmed

Expected effect:
- `P2/P4` should stop building as if the whole unresolved patch is vertically resolved
- UV flow should stop blindly dropping downward from the bridge seam

## Step 13. Re-test Arc-case

Make sure this new corner-aware propagation does not regress the earlier Arc-case fixes:
- no mirror reintroduction
- no diagonal pin regression
- no unnecessary quilt fragmentation

---

# Implementation order

1. Add `continuation_placement_role(...)` for same-patch propagation
2. Drive that role from patch-corner orthogonality, not just inherited seam axis
3. Add minimal band grammar signals to patch summary
4. Use those signals to gate promotion from local ingress axis to full patch straighten behavior
5. Update scoring / ranking to use continuation role for same-patch propagation
6. Update geometry placement to use continuation role
7. Align rescue paths
8. Validate on J-case and Arc-case

---

# Guardrails for Codex

- Keep commits small and reviewable.
- First commit: corner-aware continuation role only.
- Second commit: minimal band grammar signals.
- Third commit: scoring / placement wiring.
- Do not delete or rewrite the recent structural / telemetry work; reuse it.
- Do not revert the useful analysis-layer changes from the recent commits.

---

# Were the recent commits useful or waste?

They were useful, but incomplete.

What they already solved:
- telemetry now reflects effective role correctly
- unresolved single-sided inherited state is now visible in analysis (`axis_candidate`, `junction_supported_axis`, `single_sided_inherited_support`)
- runtime now distinguishes ingress role from continuation role more than before
- rescue is more aligned with main frontier behavior

Why they did not visibly fix the patch:
- they mostly gated or weakened propagation,
- but they still did not teach the system how to derive the **orthogonal next-chain role through the corner**.

So:
- **not waste**,
- but **insufficient by themselves**,
- because the missing logic is now clearly corner-aware role propagation plus band grammar validation.
