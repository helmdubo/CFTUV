# Exact Boundary Rotation Semantics

Status: normative contract for
`SESSION_C_R2C_REGULARIZED_BOUNDARY_ROTATION_SYSTEM`.

This document changes only RawCoverage boundary-topology extraction. It does
not change `EnvelopeSpec`, `EnvelopeInstance`, `PatchDomain`,
`ReferenceMetricV2`, `RuntimeMetricV1`, Policy B, exact segment
intersection, broadphase admission, or covered-left/covered-right
classification.

## Regularized area semantics

`RawCoverage` represents two-dimensional area matter.

- Positive-area overlap joins material through exact union.
- Contact at one exact point does not create area connectivity and does not
  weld material components.
- A point contact remains an exact geometric event fact. It is not deleted,
  displaced, snapped, or widened.
- Two outer boundary loops that touch only at one point remain two
  `RawCoverageRegionV1` records.

At an exact contact event there is one geometric arrangement point and one or
more topological boundary occurrences:

```text
ArrangementPoint
  +-- BoundaryVertexOccurrence for loop A
  +-- BoundaryVertexOccurrence for loop B
  `-- BoundaryVertexOccurrence for loop C
```

The exact point owns coordinate equality and aggregate construction history.
Each occurrence owns one incoming/outgoing boundary half-edge pairing and one
covered sector. Future materialization must therefore be able to produce
distinct topological vertex keys with identical coordinates.

`PointContactRecordV1` records the participating loops, Envelope instances,
front components, construction certificates, and provenance. It is a
candidate fact for a later interaction policy and is not itself an
interaction decision.

## Exact rotation system

Every output atomic boundary edge is already oriented with covered material
on its left. For every geometric arrangement point the boundary extractor:

1. collects all incident exact rays;
2. uses the reverse of each incoming ray;
3. sorts outgoing rays by exact affine-chart cyclic order;
4. selects the clockwise predecessor of the reverse incoming ray;
5. records that incoming/outgoing pair as one covered-sector occurrence.

The cyclic comparator uses only:

- exact upper/lower half-plane classification;
- exact cross-product sign;
- exact affine-coordinate sign for collinear same/opposite direction.

It does not use `atan2`, float angles, epsilon, distance, rounded direction,
provenance, or semantic IDs. IDs are used only to produce a stable order after
the geometric pairing is proven.

Every incoming half-edge must have exactly one successor, every outgoing
half-edge must have exactly one predecessor, every oriented boundary edge
must be consumed exactly once, and every face walk must close. Failure to
prove these invariants returns:

```text
REFERENCE_ARRANGEMENT_ROTATION_SYSTEM_UNPROVEN
```

If distinct atomic edges still occupy one exact outgoing ray after exact
splitting, the more specific result is:

```text
REFERENCE_ARRANGEMENT_COLLINEAR_BRANCH_UNPROVEN
```

Equivalent generators of one geometric atomic edge merge construction
history and provenance on that edge; they do not create competing rays.

## Construction and contributor identity

An intersection certificate includes the exact bounded generator segment
identities in addition to support and boundary identities. This prevents two
different intersections of the same infinite supports from collapsing to one
semantic arrangement-point identity. No coordinate-derived or float-derived
ID participates.

Region contributors are transferred with an exact strict-interior witness
against source contribution regions. Hole/outer grouping uses exact
edge-interior witnesses and exact winding. A boundary vertex is never used as
an interior probe and contributor provenance is never reconstructed from the
nearest output edge.

Touching outer/hole or hole/hole topology has no accepted product semantics in
this version and returns:

```text
REFERENCE_TOUCHING_HOLE_TOPOLOGY_UNPROVEN
```

Ordinary point-touch between outer area components is supported.

## Backend and differential authority

The deterministic exact-AABB broadphase and
`ExhaustiveExactArrangementBackend` share the same atomic-edge,
rotation-system, and face-walk implementation. Differential fixtures compare:

- arrangement points;
- boundary occurrences;
- point contacts;
- atomic edges;
- loops and regions;
- exact area;
- construction history and provenance;
- semantic digest.

The added rotation cost is
`sum_v O(d_v log d_v)` for exact angular sorting. Candidate segment
generation remains the accepted exact AABB sweep.

## Blender projection

The Blender renderer only projects immutable kernel records:

```text
ENV_53_POINT_CONTACTS
ENV_54_BOUNDARY_OCCURRENCES
```

It neither chooses boundary pairings nor repairs topology. A point marker
shows geometric equality; short occurrence ticks show the distinct
incoming/outgoing loop occurrences.

## Explicit non-scope

This contract does not define multiway interaction, touching-hole product
semantics, ownership, tessellation, production mesh welding, snapping,
tolerance, near-planar projection, or modal scheduling.
