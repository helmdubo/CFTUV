# CFTUV Arc Mesh Follow-up Review Checklist

Use this as a pre-coding / pre-review checklist for the Arc Mesh follow-up task.

## Part 1 — Mirror / diagonal pin fix

- [ ] Add temporary debug prints only for chains where `raw role == FREE` but `effective role != FREE`
- [ ] Print: `chain_ref`, raw role, effective role, inherited source patch, anchors, base direction, final axis direction
- [ ] Add one helper that resolves direction from an explicit role, not from `chain.frame_role`
- [ ] Use that helper in seed placement
- [ ] Use that helper in regular frontier placement
- [ ] Use that helper in rescue / ingress placement paths too
- [ ] Make anchor-direction inheritance aware of effective role
- [ ] Verify placement metadata matches geometry mode
- [ ] Verify runtime counters still match effective role
- [ ] Re-test Arc Mesh: no mirror on `P1C1 / P1C3`
- [ ] Re-test Arc Mesh: `P1C2` no longer pins diagonally because of mirrored side

## Part 2 — Structural-aware quilt planning

- [ ] Do **not** add runtime cross-quilt stitching hacks
- [ ] Read structural facts from analysis layer, do not reimplement structural interpretation in planning
- [ ] Add a small planning helper for effective seam role (`raw` vs `inherited H/V`)
- [ ] Use effective planning role in seam pair scoring
- [ ] Update attachment candidate ranking to see structurally strong inherited FREE chains
- [ ] Relax the hard reject for raw `FREE+FREE` seam pairs when structural support exists
- [ ] Keep inherited seam support weaker than true raw H/V continuation
- [ ] Re-test planning: `P2` should not split into an unnecessary separate quilt if structural support exists

## Guardrails

- [ ] No whole-patch straighten in this task
- [ ] No big pipeline rewrite
- [ ] Small commits: Part 1 and Part 2 separately
- [ ] Remove or gate temporary debug prints before final commit

## Done criteria

- [ ] Straightened inherited chains no longer mirror
- [ ] Arc Mesh scaffold is structurally continuous for the intended neighboring patches
- [ ] Quilt fragmentation is reduced by planning, not hacked at frontier runtime
