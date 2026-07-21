# R2.1 hotfix gate — RF29b fixed, proposed RF30 diagnosis rejected by materialized oracle

Base: `e921fef`

Candidate implementation: `d98c58f`

Field snapshot: `cftuv_r21_user_field.blend`, SHA-256

`7db56e359deab0cca39426cabee66e16620f6de31aba68cdca3c19eed30c480f`

Blender: `4.3.2`; source `.blend` was never saved.

## Result

- **RF29b fixed.** `sagging_wall`, selected edges `6,7,8`, widths
  `7.8,12,18,24,30,36`: preview and confirm are both `OK` and have
  identical full GeometryBatch digests at every width. The terminal bridge
  clamps to the last constructible station and reports
  `TERMINAL_ROUTE_STATION_CLAMPED` (`1` at width `7.8`, `2` thereafter).
- **The arrows on `walls.001` are not an RC5b failure.** The labeled
  materialized-geometry oracle finds that every suspect curved edge of the
  representative repeated unit (`SEGMENT e94`) has exactly two actual face
  owners: `SEGMENT e94` plus `BEVEL e45` or `BEVEL e36`. The only one-use
  edge is the true outer/source contour. Therefore both waves have arrived;
  straightening these curves would erase a legal material boundary.
- No RC5b production change is included. An experimental generic
  second-owner implementation was rejected before commit: it added 655
  compile atoms on `walls.001`, changed no final geometry, and raised field
  evaluation time from roughly 3 s to 8 s.

## RF29b structural cause and fix

The saturation resolver previously entered `CapacityPolicy` only when
`guide.saturated` meant that the compiled route itself was exhausted.
Bridge constructibility was validated later, while materializing the
terminal polygon. On the field case, route reach was `14.581...`, requested
alpha was `3.9`, but the last constructible station was `3.2330793554`;
the following station already produced `TERMINAL_BRIDGE_CUT_INVALID`.

The shared pre-materialization resolver now:

1. validates the requested terminal guide regardless of route exhaustion;
2. on `SATURATE_PROVEN`, walks immutable station prefixes backwards;
3. selects the last constructible prefix and records the clamp;
4. retains the named invalid failure when no prefix is constructible;
5. retains `REJECT_UNPROVEN` and `CONTROLLED_RECOMPILE` semantics.

The wider RF29b sweep exposed a separate latent error: a concave
CornerModel contour was silently replaced by its convex hull before
competition. It is now deterministically decomposed into convex ears,
preserving exact semantic area, owner, anchors, and provenance. The enhanced
error includes corner/sector/key/point/contour when the semantic invariant is
violated.

## I6 recurrence explanation

The pure evaluator at base `e921fef` fails identically for both
`preview=True` and `preview=False`; there is no third geometry validator.
The user's apparent confirm-only failure was the former UI last-valid preview
masking the error until Done. S2b `PreviewFailurePolicy=CLEAR` removes that
presentation asymmetry.

What survived the two earlier directives was instead a validation-phase
split: route saturation was checked in the CapacityPolicy resolver, while
bridge polygon constructibility was checked later during materialization.
The fix moves constructibility into that same shared resolver, so both
preview and confirm consume the already validated/clamped guide.

## Evidence

- `decal_r21_hotfix_before.json`: base `e921fef`; RF29b fails at every width;
  preview and confirm report the same named error. RF30 field oracle reports
  `MUTUAL_ARRIVAL_CONFIRMED`.
- `decal_r21_hotfix_after.json`: `green: true`; RF29b is `OK` and
  preview-identical-to-confirm at all six widths; the same RF30 conclusion.
- `decal_r21_rf29b_before.png`: exact width-7.8 field view with the named
  error and no decal result.
- `decal_r21_rf29b_after.png`: the same camera/selection with materialized
  geometry and counted station clamp.
- `verify_decal_r21_hotfix.py`: reproducible read-only field harness.
- `render_decal_r21_rf29b.py`: reproducible read-only wireframe harness.
- Unit/full suite: `494 passed, 3 skipped, 4 deselected`.
- `python -m compileall -q cftuv artifacts/verify_decal_r21_hotfix.py
  artifacts/render_decal_r21_rf29b.py`: pass.

## Risks

1. The RF30 conclusion is deliberately narrow: it disproves the supplied
   `walls.001` arrows as a mutual-arrival counterexample. A different fixture
   with a genuinely one-sided materialized competition edge would still be a
   valid RF30 failure and should arrive with its labeled witness pair.
2. Station clamping can now occur before route exhaustion. This is intended
   bridge-capacity semantics, and the counted policy makes the early clamp
   observable.
3. Concave semantic contours now produce several convex arrangement pieces.
   Their union is exact, but downstream code must continue treating
   `semantic_owner_id` rather than piece count as corner identity.

## Особое мнение

I object to changing geometry for problem A on the supplied evidence. The
arrowed curves are shared edges between two materialized faces, not a
freeze-locus acting in empty space. Both the base and candidate code produce
the same GeometryBatch, and the labeled adjacency witnesses explicitly name
the two owners. Calling a no-op or forced straightening an RC5b fix would be
less honest than returning the contradictory oracle to the gate owner.
