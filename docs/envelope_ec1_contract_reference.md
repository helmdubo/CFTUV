# Envelope kernel EC1 contract reference

Status: `SESSION_B_CANDIDATE_READY_FOR_REVIEW`.

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
constraints, PatchSurfaceIR, oriented owner sectors, reflex-angle certificates
and Corner/Junction relations. Seeds, alpha, events, capacity, coverage,
ownership and output geometry are absent by construction.

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

`AngularProfileSelectionCertificateV1` represents, but does not numerically
derive, the accepted proof:

```text
k < delta / delta_max <= k + 1
subturn_count = k + 1
support_count = segment_count = k + 2
```

Hidden supports remain local immutable records of one Angular spec. Their
explicit ordinal and exact rational turn fraction make storage permutation
irrelevant without turning them into PhysicalChains or ChainUses.

`GeometryBatchV1` is the only future output adapter boundary. Vertices use
semantic `VertexKey` identity; positions never define topology. Faces retain
UV, region, ownership, material and source provenance. GPU and transactional
BMesh adapters are future readers of the same record.

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
  boundaries, interfaces, regions, provenance and shared semantic vertices.

The four checked-in JSON Schemas in `kernel/schema/` are generated from the
same authoritative public Python types; a test rejects drift.

## Structural validation scope

Pure validators check versions, capabilities, ID uniqueness, reference
integrity, request/domain consistency, SourceRevision alignment, ordered owner
supports, angle/selection linkage, `k + 1`/`k + 2`, seed/Envelope variant
compatibility, event participant typing, raw/resolved stage separation,
ownership declarations, tessellation authority and GeometryBatch keys.

They do not compute line intersections, hidden-support directions, Envelope
instances, Boolean/arrangement topology, ownership cells, event times or mesh
tessellation.

## Accepted fixture projection

All 23 accepted cases project to the separate production records (26 patch
plans because P01 and P05 span multiple domains). The copied corpus is
hash-verified against the exact Session A source files and commit. Its test-only
adapter is not shipped in the runtime wheel.

Session A cases intentionally omit source coordinates and face tessellation.
The adapter represents that absence with the explicit
`EC0_COORDINATE_FREE_FIXTURE_V5` capability and tagged unavailable-position
record; it does not invent geometry. Full host snapshots require complete
local positions, face cycles and loop-triangle provenance.

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
```

## Explicit non-scope

There is no evaluator, supporting-line math, Boolean engine, arrangement/DCEL,
ownership solver, event queue, triangulator, Blender/GPU adapter or native
extension. Session C remains closed until independent review accepts this EC1
candidate.

