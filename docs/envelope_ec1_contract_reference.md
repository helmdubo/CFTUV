# Envelope kernel EC1 contract reference

Status: `SESSION_B_R1_CANDIDATE_READY_FOR_CI`.

This reference describes the Blender-free public boundary implemented by
Session B. It does not open Session C and contains no geometry evaluator.

## Package boundary

- distribution: `cftuv-envelope-core`;
- import package: `cftuv_envelope`;
- Python: 3.10+;
- runtime dependencies: none;
- accepted input authority: Session A v5 at
  `1fb0394a634c089ed590f544e4d524be28f52123`.

The independent package lives under `kernel/`. Its wheel has no import edge to
`bpy`, `mathutils` or `cftuv`. CI tests the installed wheel from a clean
directory and repeats build/tests after copying only `kernel/` into an empty
Git repository.

## Public aggregate boundaries

`AnalysisSnapshotV1` contains host-observed facts only: source revision,
capabilities, Patch/PatchDomain identities, directed ChainUses, fixed boundary
constraints, PatchSurfaceIR, authoritative metric descriptors, oriented owner
sectors, reflex-angle certificates and Corner/Junction/Terminal relations.
Seeds, alpha, events, capacity, coverage,
ownership and output geometry are absent by construction.

Every `FULL_HOST_SURFACE` PatchDomain has one authoritative metric descriptor.
For a planar domain this is `PlanarPatchFrameV1`: origin, oriented U/V basis,
normal, handedness, an exact zero-distance planarity certificate and the total
mapping from source vertices to domain coordinates. Session C reads this frame;
it must not select one from face order, triangulation or world axes.

`PatchSurfaceIRV1` has an authoritative `SourceEdgeV1` table. A physical chain
stores both its ordered source vertices and ordered physical edges plus its
open/closed state. Structural validation enforces `E = V - 1` for open chains,
`E = V` for closed chains, and endpoint agreement with the source-edge table.
Boundary constraints use a tagged target: BoundaryLoop, ChainUse or ordered
physical-edge sequence. A production snapshot cannot substitute an unavailable
target.

`DecalRequestV1` is independent of the snapshot. V1 fixes the accepted policy
references:

```text
LINEAR_REFLEX_EQUAL_V1
MIN_K_FOR_MAX_SUBTURN_V1
LINEAR_REFLEX_MAX_SUBTURN_V1
LINEAR_REFLEX_MAX_SUBTURN_60_DEGREES_V1 = PI_OVER_3
BOUNDARY_LIMITED_PROPAGATION
INTRAPATCH_POLICY_B_V1
TOTAL_DISJOINT_RESOLVED_COVERAGE_V1
```

Widths use an explicit `SOURCE_LOCAL_INTRINSIC` metric. The host's world-to-
local conversion is outside this package.

`CompiledPatchEvaluationPlanV1` is keyed by
`(DecalRequestId, PatchDomainId)`. Every contained record carries the same key.
The aggregate expresses compile seeds, component lifecycle declarations,
angle-selection proofs, Envelope specs/instances, initial-front references,
event predicates/transitions, lazy-ledger policy, distinct Raw/Resolved
coverage stages, interaction declarations, ownership obligations,
`TessellationPlanV1` and future GeometryBatch provenance.

The `EnvelopeSpec` union has exactly four variants:

```text
StripEnvelopeSpec
AngularEnvelopeSpec
JunctionEnvelopeSpec
CapEnvelopeSpec
```

`ReflexAngleCertificateV1` carries a normalized-to-symbolic-PI certified
interval for both phi and reflex excess, including endpoint openness, error
bound and turn orientation. Its owner sector carries authoritative planar
incoming/outgoing support directions. `AngularProfileSelectionCertificateV1`
then records the accepted proof:

```text
k < delta / delta_max <= k + 1
subturn_count = k + 1
support_count = segment_count = k + 2
```

Hidden supports remain local immutable records of one Angular spec. Their
explicit ordinal and exact rational turn fraction make storage permutation
irrelevant without turning them into PhysicalChains or ChainUses.

The cross-contract validator compares the certified excess interval directly
against exact `PI_OVER_3`. It accepts `k` only when the strict lower and closed
upper inequalities are proven for the entire interval. A threshold-crossing
interval returns `ANGULAR_PROFILE_SELECTION_UNCERTAIN`; there is no epsilon,
rounding or silent choice.

Junction route topology is tagged as T, X, Y or declared route pairs. Each
actual route pair has a `RoutePairingId` and station-continuity law, and
`JunctionEnvelopeSpec` references those IDs. A flat ChainUse tuple is not a
route-pairing contract.

`GeometryBatchV1` is the only future output adapter boundary. Vertices use
semantic `VertexKey` identity; positions never define topology. Faces retain
UV, region, ownership, material, source provenance and per-semantic-vertex
station/transverse facts. GPU and transactional BMesh adapters are future
readers of the same record.

## Serialization and digests

The four codecs are independent:

- `AnalysisSnapshotCodecV1`;
- `DecalRequestCodecV1`;
- `CompiledPlanCodecV1`;
- `GeometryBatchCodecV1`.

Canonical JSON is UTF-8, rejects NaN/infinity and uses stable separators,
field names, enum values, schema versions and typed ID tags. Tuples retain
semantic order; only `frozenset` collections are canonical-sorted. Decimal
format normalization does not round values.

Three digest boundaries prevent accidental equivalence:

- `SnapshotDigest` covers the exact serialized host snapshot;
- `SemanticPlanDigest` covers compile semantics;
- `GeometryBatchSemanticDigest` excludes face order, internal fill diagonals,
  valid internal triangulation and runtime face count, while retaining exposed
  boundaries, interfaces, regions, provenance, shared semantic vertices and
  normalized semantic UV/station projections.

The four checked-in JSON Schemas in `kernel/schema/` are generated from the
same authoritative public Python types; a test rejects drift.
The package root has a checked-in explicit `__all__`; a public-API snapshot test
rejects accidental exports or removals.

## Structural validation scope

Pure validators check versions, capabilities, ID uniqueness, reference
integrity, request/domain consistency, SourceRevision alignment, ordered owner
supports, angle/selection linkage, `k + 1`/`k + 2`, seed/Envelope variant
compatibility, event participant typing, raw/resolved stage separation,
ownership declarations, tessellation authority and GeometryBatch keys.

The deep validator additionally proves Snapshot-to-plan linkage for every
selected ChainUse, Front/Corner/Junction/Cap seed, relation, owner sector,
angle/selection certificate, fixed boundary constraint, event participant,
EnvelopeSpec and GeometryBatch provenance reference. Every selected ChainUse
must compile exactly once in exactly one request/domain plan.

They do not compute line intersections, hidden-support directions, Envelope
instances, Boolean/arrangement topology, ownership cells, event times or mesh
tessellation.

## Accepted fixture projection

All 23 accepted cases project to the separate production records (26 patch
plans because P01 and P05 span multiple domains). The copied corpus is
hash-verified against the exact Session A source files and commit. Its test-only
adapter is not shipped in the runtime wheel.

Session A cases intentionally omit source coordinates, authoritative planar
frames, physical-edge paths, certified numeric angles/support directions,
typed terminal facts and some route topology.
The adapter represents that absence with the explicit
`EC0_COORDINATE_FREE_FIXTURE_V5` capability and tagged unavailable records; it
does not invent geometry. Full host snapshots reject those unavailable states
and require complete local positions, metric authority, face/edge cycles,
loop-triangle provenance and R1 capabilities.

## Named outcomes

V1 declares:

```text
DECAL_ANALYSIS_SCHEMA_UNSUPPORTED
BARRIER_SPLIT_REQUIRED
BARRIER_BYPASS_UNSUPPORTED
SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN
OWNERSHIP_PARTITION_UNPROVEN
PENDING_EXACT_EVALUATION
APPROXIMATE_MATERIALIZATION_PENDING
JUNCTION_ROUTE_PAIRING_REQUIRED
ANGULAR_PROFILE_SELECTION_UNCERTAIN
```

## Explicit non-scope

There is no evaluator, supporting-line math, Boolean engine, arrangement/DCEL,
ownership solver, event queue, triangulator, Blender/GPU adapter or native
extension. Session C is not opened by this reference.
