# BASE-00 control-plane reconciliation

Pack version: `1.1.1`

## Reason

The original v1.1 execution pack was drafted against reviewed baseline
`df587ed166cfb0e0b615148f08c583b4477c5ac4`, where point-contact topology was
still the immediate blocker. BASE-00 later selected
`c2622d07020338e5231b81f41655fe6c74cdca72` as the code baseline.

That selected baseline already contains the consolidated occurrence-aware
C-R2C gate. Leaving the historical four-card decomposition runnable would
direct a later agent to reimplement an accepted semantic layer.

## Accepted C-R2C authority

- implementation:
  `43e69d3889d273ed19daee9239ae0e311a1b213d`;
- handoff commit:
  `979870f17d7890d96f008609966bdeb24d8b0b58`;
- selected code baseline plus host reflex-corner export:
  `c2622d07020338e5231b81f41655fe6c74cdca72`;
- handoff:
  `docs/session_c_r2c_regularized_boundary_rotation_handoff.md`;
- CI:
  GitHub Actions run `30095478731`.

The accepted gate provides:

- `RawCoverageResultV2`;
- multiple `BoundaryVertexOccurrenceV1` values at one exact
  `RawCoverageVertexV1`;
- `PointContactRecordV1`;
- exact circular ray ordering and covered-left successor construction;
- exact face walks and containment witnesses without boundary-vertex probes;
- occurrence/contact debug projection;
- a `building.002` field receipt that reaches RawCoverage V2 without
  `REFERENCE_ARRANGEMENT_NON_MANIFOLD`.

## Card reconciliation

`C-R2C-01` through `C-R2C-04` are
`SUPERSEDED_BY_ACCEPTED_C_R2C_GATE`. They remain in the pack as historical
planning evidence and are not runnable cards.

The consolidated implementation used some different internal/public type
boundaries than the draft decomposition. That difference does not authorize
new work. Any required delta must be stated as a new versioned card with an
explicit accepted contract gap.

## Current sequence

```text
BASE-00 human acceptance
 ├─ DOC-00
 └─ FIX-00
       ↓ both accepted
D-R2-00
       ↓
C-R2D-01
```

The activation condition for `D-R2-00` is already met: the accepted field
receipt returns `MULTIWAY_INTERACTION_POLICY_UNPROVEN` after RawCoverage.
`FIX-00` must make that case portable, and `DOC-00` must finish top-level
onboarding authority before the new interaction contract starts.

## Scope

This amendment changes only the repository-resident execution control plane.
It does not change kernel geometry, host code, public contracts, tests,
workflows, fixtures, or source meshes.
