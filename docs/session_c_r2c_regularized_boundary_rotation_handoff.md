# C-R2C Regularized Exact Boundary Rotation System — handoff

Session: `SESSION_C_R2C_REGULARIZED_BOUNDARY_ROTATION_SYSTEM`

Branch: `codex/c-r2c-regularized-boundary-rotation`

Accepted available base: `df587ed166cfb0e0b615148f08c583b4477c5ac4`

Implementation SHA:
`43e69d3889d273ed19daee9239ae0e311a1b213d`

CI-tested SHA:
`43e69d3889d273ed19daee9239ae0e311a1b213d`

Status: `C_R2C_READY_FOR_REVIEW`

`D-R2 Multiway Interaction Policy`, ownership, tessellation, and Session E
were not opened.

## Implemented boundary topology

The Blender-free package is now `cftuv-envelope-core 0.7.0`.

RawCoverage has a versioned V2 result with two explicit identity levels:

- `RawCoverageVertexV1` remains the one exact geometric arrangement point;
- `BoundaryVertexOccurrenceV1` records one incoming/outgoing boundary
  half-edge traversal through that point;
- `PointContactRecordV1` preserves a lower-dimensional contact as an exact
  semantic/diagnostic fact without joining area components.

Boundary continuation now comes from an exact half-edge rotation system.
Outgoing rays are ordered with exact affine half-plane, cross-sign, and
collinear-direction predicates. The covered-left face walk selects the
clockwise predecessor of the reverse incoming ray. Provenance and semantic
IDs do not select topology.

Every incoming boundary half-edge has one proven successor, every outgoing
half-edge has one predecessor, and each oriented output edge is consumed once
by a closed face walk. Point-touch outer components therefore remain separate
RawCoverage regions while sharing one exact arrangement point.

Construction certificates now include bounded generator segment identities.
This distinguishes different intersections of the same infinite support sets
without using coordinates or float-derived IDs.

Boundary-vertex containment probes were removed. Hole grouping and
contributor transfer use exact witnesses and exact winding. Touching
outer/hole or hole/hole topology remains an explicit unsupported product
case:

```text
REFERENCE_TOUCHING_HOLE_TOPOLOGY_UNPROVEN
```

Unproved rotation and collinear branch cases are separately named:

```text
REFERENCE_ARRANGEMENT_ROTATION_SYSTEM_UNPROVEN
REFERENCE_ARRANGEMENT_COLLINEAR_BRANCH_UNPROVEN
```

Broadphase and exhaustive backends share the same rotation/face-walk layer.
The accepted exact-AABB candidate generation and exact narrowphase were not
changed.

Normative semantics:
`docs/envelope_boundary_rotation_semantics.md`.

## Executable corpus

`kernel/fixtures/session_c_r2c_boundary_rotation_v1/manifest.json` maps all 20
required cases to executable pytest node IDs, including:

- two, three, and four component point contacts;
- exact before/at/after contact;
- tangential contact;
- concave contact;
- coincident generators and named collinear ambiguity;
- touching-hole named outcomes;
- input permutation, storage reversal, exact rotation, and scale;
- broadphase/exhaustive equality;
- the U-shaped three-chain asset case;
- `building.002`, edges `2, 3, 7`.

Differential records include vertices, atomic edges, loops, regions,
occurrences, contacts, exact area, provenance, and semantic digest.

## building.002 field gate

Source: `E:\testscene.blend`, object `building.002`, selected physical edges
`2, 3, 7`, requested alpha `0.25`.

Patch 0 now reaches RawCoverage:

| Fact | Result |
|---|---:|
| Metric | ready |
| Compile | ready |
| Raw | ready, schema V2 |
| Shared arrangement points | 2 |
| Boundary occurrences | 12 |
| Point contacts | 2 |
| Loops / regions | 3 / 3 |
| Exact area | `Rational(122766786560, 373821260323)` |
| Max incident degree | 4 |
| Rotation comparisons | 22 |
| Half-edges consumed exactly once | true |
| Provenance complete | true |
| Broadphase/exhaustive equal | true |

`REFERENCE_ARRANGEMENT_NON_MANIFOLD` is absent.

The next stage honestly returns:

```text
MULTIWAY_INTERACTION_POLICY_UNPROVEN
Patch 0: INTERACTION_REJECTED
```

This is the expected independent D-stage carryover and was not repaired in
the arrangement layer.

The real Blender operator returned `FINISHED` with:

```text
Topology: 4/4 | Metric: 4/4 | Raw: 4/4 | Resolved: 3/4
```

The GP object contains 243 strokes. New immutable projections are populated:

```text
ENV_53_POINT_CONTACTS          4 strokes
ENV_54_BOUNDARY_OCCURRENCES    4 strokes
```

The source mesh fingerprints before and after both the background field
verifier and the interactive operator are equal.

Evidence:

- `artifacts/envelope_r2c/building_002_point_touch_report.json`;
- `artifacts/envelope_r2c/blender_debug_operator_report.json`.

## Validation

- focused boundary/renderer matrix: `51 passed`;
- broadphase stress and exhaustive differential: `5 passed`;
- host adapter/artifact/renderer matrix: `33 passed`;
- hermetic source suite outside checkout: `224 passed`;
- installed wheel-only suite outside checkout: `224 passed`;
- extracted-kernel installed-wheel suite: `224 passed`;
- Blender 4.3.2 background bridge: `ENVELOPE_DEBUG_BLENDER_SMOKE_OK`;
- generated schemas: OK;
- forbidden import scan: OK;
- Session A fixture projection: OK;
- `git diff --check`: OK.

The wheel and extracted package were independently built as
`cftuv_envelope_core-0.7.0-py3-none-any.whl`.

## Preserved boundaries and carryovers

- No snapping, epsilon, rounded angles, coordinate welding, or approximate
  fallback was introduced.
- Envelope laws, metric contracts, Policy B, interaction semantics,
  ownership, tessellation, production mesh, and broadphase admission were not
  changed.
- The Blender renderer only reads occurrence/contact records and never
  chooses a pairing.
- Legacy geometry was neither read nor invoked.
- Touching-hole product semantics remains named unsupported.
- Multiway interaction at Patch 0 is the next independent policy question;
  C-R2C does not authorize D-R2 automatically.
- The available ancestry contained the accepted M-R1 handoff as its head; no
  separate accepted V0-R2 implementation commit was present or synthesized.
