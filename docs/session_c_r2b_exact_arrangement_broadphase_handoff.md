# C-R2B Deterministic Exact Arrangement Broadphase — handoff

Session: `SESSION_C_R2B_EXACT_ARRANGEMENT_BROADPHASE`

Branch: `codex/c-r2b-arrangement-broadphase`

Accepted C-R2A base: `fe091b4`

## Closed gate

The production reference arrangement now uses a deterministic exact
min-x sweep to select candidate segment pairs. The existing exact
`segment_intersections()` narrowphase, atomic splitting, curve history,
coverage classification, provenance and union traversal remain shared with the
exhaustive oracle.

Every admitted candidate satisfies an inclusive exact AABB overlap test.
`ExactSegmentAabbV1` stores `min_x`, `max_x`, `min_y` and `max_y` as
`ExactScalar` expressions. No float AABB, tolerance, approximate predicate or
fallback exists.

Pair tests are sorted by semantic segment identity. Storage/input region order
does not select pair order.

## Permanent differential oracle

`ExhaustiveExactArrangementBackend` remains in
`reference/arrangement.py`, but production paths instantiate only
`ExactSegmentArrangementBackend`. The exhaustive backend is used by
differential tests and the offline Blender asset verifier.

The differential matrix covers:

- tiled grid;
- many collinear boundaries;
- partial coincident boundaries;
- a high-intersection algebraic star;
- one sparse domain with many Envelope regions.

For every matrix case, broadphase and exhaustive results are equal by exact
vertices, atomic boundary edges, loops, regions, exact area, full provenance
records and canonical semantic digest.

The 2000-disjoint-segment stress receipt is:

| Counter | Value |
| --- | ---: |
| input segments | 2000 |
| all possible pairs | 1,999,000 |
| broadphase candidate pairs | 0 |
| narrowphase tests | 0 |
| actual intersections | 0 |
| atomic edges | 2000 |

An independent 80-segment sample proves the exhaustive oracle still executes
all 3,160 possible pair tests while the production sweep executes zero.

## Runtime counters

`ArrangementUnionV1` now carries:

- `input_segment_count`;
- `all_possible_pair_count`;
- `broadphase_candidate_pair_count`;
- `narrowphase_test_count`;
- `intersection_count`;
- `atomic_edge_count`.

Host telemetry publishes:

- `ARRANGEMENT_INPUT_SEGMENTS`;
- `ARRANGEMENT_ALL_POSSIBLE_PAIRS`;
- `ARRANGEMENT_BROADPHASE_CANDIDATE_PAIRS`;
- `ARRANGEMENT_NARROWPHASE_TESTS`;
- `ARRANGEMENT_INTERSECTIONS`;
- `ARRANGEMENT_ATOMIC_SEGMENTS`.

`ARRANGEMENT_PAIR_TESTS` remains a compatibility alias, but now reports actual
narrowphase tests rather than the theoretical all-pairs count.

## building.002 receipt

Source: `E:\testscene.blend`, object `building.002`, selected physical edges
`2, 3, 7`, requested alpha `0.3`.

Three exact-resolved PatchDomains each reported:

- 8 arrangement input segments;
- 28 possible pairs;
- 17 candidate/narrowphase pairs;
- 11 rejected pairs;
- identical production and exhaustive RawCoverage semantic digests.

Resolved-domain totals:

| Counter | Value |
| --- | ---: |
| input segments | 24 |
| all possible pairs | 84 |
| broadphase candidates | 51 |
| narrowphase tests | 51 |
| rejected pairs | 33 |
| actual intersection points | 60 |
| atomic edges | 21 |

The exact reduction is `33/84 = 11/28` of all possible pairs. Patch 0 remains
honestly reported as `RAW_REJECTED /
REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE`; it is not counted as a successful
differential domain. The three resolved domains all match the exhaustive
oracle. Source mesh fingerprints before and after evaluation are identical.

Machine-readable evidence:
`artifacts/envelope_r2b/building_002_arrangement_broadphase_report.json`.

## Validation

- Blender-free kernel suite: `187 passed`.
- Host adapter telemetry suite: `23 passed`.
- Focused broadphase/R2A arrangement suite: `16 passed`.
- Headless Blender 4.3.2 asset receipt: passed.
- Source mesh unchanged: passed.
- Legacy geometry read or invoked: no.

PatchDomain remains the execution field; PhysicalChain/ChainUse provenance is
unchanged. No Envelope, interaction, metric or
`CanonicalGeometryDigest` semantic contract changed.

## Static domain index

No persistent static-domain cache was added in this slice. R2B.5 permits but
does not require it, and V0-R1B already owns session cache lifecycle. The exact
sweep indexes the complete immutable input set per evaluation; adding a
SourceRevision/PatchDomain static index later can reuse the same AABB and
semantic-key records without changing the narrowphase contract.

## Risks

- Exact symbolic ordering can intentionally fail closed with
  `CertifiedPredicateUndecidable`; there is no numeric fallback.
- The sweep eliminates narrowphase calls, but its active set is a linear exact
  interval collection. Pathological x-overlap can still make candidate
  admission work quadratic even when y rejects most pairs. This does not
  change exact results and is outside the requested narrowphase gate.

## Implementer opinion

Keep `ExhaustiveExactArrangementBackend` permanently small and mechanically
shared with production splitting/union code. If a future interval tree or
static domain index replaces the sweep active set, the exhaustive differential
matrix should remain the blocking acceptance oracle.
