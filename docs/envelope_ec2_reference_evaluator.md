# EC2 planar reference envelope evaluator

Status: `CANDIDATE_READY_FOR_SESSION_C_REVIEW`

Accepted input baseline: Session B-R1 closure commit
`a18bd486ba198d6a14b81b703184c2402681576e`.

## Purpose and boundary

The `kernel/src/cftuv_envelope/reference/` package is the permanent,
Blender-free, full-recompute semantic reference path for one
`(DecalRequestId, PatchDomainId)` evaluation:

```text
AnalysisSnapshotV1 + DecalRequestV1
-> ReferenceEnvelopeCompilationV1
-> EnvelopeInstances(alpha)
-> boundary-limited component resolution
-> exact segment arrangement and provenance-preserving patch union
-> RawCoverageResultV1
```

It is deliberately not a modal production runtime. It contains no
interaction resolution, Case 16 policy B, ownership, station/UV partition,
tessellation, `GeometryBatch`, event scheduler, Blender adapter, or native
code. Session D is not opened by this work.

## Admission and exact arithmetic

Geometry evaluation admits only `FULL_HOST_SURFACE`, `PLANAR` PatchDomains
with exactly one exact `PlanarPatchFrameV1` and total SourceEdge /
PhysicalChain data. EC0 coordinate-free records remain valid compilation and
negative-validation inputs, but geometry evaluation returns
`REFERENCE_GEOMETRY_PAYLOAD_REQUIRED`; it never invents coordinates.

Finite host decimals are interpreted as exact decimal rationals. SymPy
1.13-1.x supplies exact rational/algebraic expression manipulation and
certified sign queries. The package itself owns the segment arrangement,
intersection history, clipping, and Boolean union. If a predicate cannot be
proved, evaluation returns `REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE`.
There is no tolerance-selected, float, raster, SDF, Marching Squares, GEOS,
or silent approximate fallback.

Backend identity is
`CFTUV_EXACT_SEGMENT_ARRANGEMENT_WITH_HISTORY`, version `1.0.0`.
The standalone distribution version is `cftuv-envelope-core==0.2.0` with
`sympy>=1.13,<2` as its only runtime dependency.

## Compilation and envelope variants

`compile_reference_envelopes` compiles exactly one request/domain plan and
keeps the accepted EC1 authority direction: host facts and request policy are
inputs; seeds, FrontComponents, envelope declarations, effective alpha, and
events are kernel-owned outputs.

- `StripEnvelope` evaluates every explicit straight physical support with an
  owner-interior unit-speed moving line. Collinear data segmentation is
  treated as one continuous active interval. A non-linear route must have an
  explicit segment-corner support relation; it is never replaced by a chord.
- `AngularEnvelope` implements only `LINEAR_REFLEX_EQUAL_V1`. One
  angle-driven algorithm consumes the accepted selection certificate for
  K=0, K=1, or K=2. Hidden directions are at `j/(K+1)` of the certified
  oriented turn and all support normal speeds are one.
- `CapEnvelope` is emitted only for physical endpoints / accepted
  `TerminalRelationV1` records. A data-record end does not create a cap, and a
  closed chain has none.
- `JunctionEnvelope` implements only declared fixture semantics for T, X with
  two explicit route pairs, Y with explicit upstream/downstream routes, and
  per-Patch projections of a cross-Patch relation. Missing route topology
  returns `JUNCTION_ROUTE_PAIRING_REQUIRED`.

## Boundary-limited resolution

The evaluator derives the physical PatchDomain boundary from owner source
faces and distinguishes outer boundary, hole boundary, explicit physical
barriers, and non-blocking source launch loci. Exact swept-front contact can
clip, shrink, or slide an endpoint on the same component. An interior contact
that would increase branch count produces `BARRIER_SPLIT_REQUIRED` at the
exact contact alpha and clamps only that FrontComponent. Other components
continue.

No crossing-and-rollback, obstacle bypass, material teleport, first/last
vertex heuristic, or Patch-wide stop is used. A disconnected clipped result
that would require bypass returns `BARRIER_BYPASS_UNSUPPORTED`.

## Exact arrangement and RawCoverage

`PlanarArrangementBackend` separates the reference API from its exact segment
implementation. The implementation intersects input boundaries exactly,
splits them into atomic segments, keeps the complete source history on each
atom, classifies both sides, and emits only atoms separating covered and
uncovered space. Coincident histories are merged before output.

Every output vertex has one or more construction certificates from this
closed set: `SOURCE_VERTEX`, `SUPPORT_SUPPORT_INTERSECTION`,
`SUPPORT_BOUNDARY_INTERSECTION`, `BOUNDARY_BOUNDARY_INTERSECTION`, or
`EVENT_ANCHOR`. Every silhouette edge preserves EnvelopeSpec,
EnvelopeInstance, support, physical-edge/boundary, ChainUse, PatchDomain, and
lineage claims supplied by arrangement history. Every region preserves its
complete contributor set, including an Angular contribution that becomes
fully internal. There is no post-Boolean nearest-source reconstruction.

`RawCoverageResultV1` schema is
`cftuv.envelope.raw_coverage_result.v1`. It records requested and effective
component alphas, immutable EnvelopeInstances, boundary-resolution
certificates, exact vertices/edges/loops/regions, exact area expression,
diagnostics, and a canonical semantic digest. It remains pre-interaction and
pre-ownership by construction.

## Permanent evidence

`kernel/fixtures/session_c_planar_v1/` contains 23 coordinate-bearing
declarations and its own LF-normalized SHA-256 manifest. It is independent of
the accepted Session A v5 coordinate-free corpus.

Permanent tests cover:

- K=0/1/2 cardinality, `j/(K+1)` supports, unit speed, and exposed Angular
  lineage;
- physical caps, closed-chain no-cap behavior, exact endpoint contact,
  explicit barriers, holes, concave boundaries, local saturation, and no
  bypass;
- single-cover union, preserved holes, internalized Angular contributor
  claims, complete construction/provenance records, and physical seam A/B
  domain separation;
- T/X/Y and cross-Patch declared junction semantics;
- record/source order, semantic storage reversal, coherent rigid and uniform
  transforms, physical route split/merge, retriangulation, and hidden-support
  permutation;
- semantic changes to angle selection, Delta-max validity, deleted profile
  support, and request identity;
- an independent exact orthogonal-grid oracle for area, topology, silhouette
  edge count, and contributor lineage.

The package wheel is tested outside the checkout, and the `kernel/` subtree is
also copied into an empty repository, built, installed, tested, import-scanned,
schema-checked, and fixture-checked there.

The final local and isolated-wheel suites each report 82 passing tests. The
extracted-subtree suite also reports 82 passing tests, followed by successful
forbidden-import, generated-schema, and accepted-fixture checks.

Reference performance was measured on the Session C Windows host with Python
3.13.1 and SymPy 1.14.0. Values below are medians of three full compilations
and three independent per-alpha recomputations; no incremental cache or event
queue was used:

| fixture | compile | full recompute | Raw V/E/L/R |
|---|---:|---:|---:|
| straight strip | 0.186 ms | 78.441 ms | 4/4/1/1 |
| Angular K=2 | 0.381 ms | 618.152 ms | 10/10/1/1 |
| X junction | 0.408 ms | 4529.893 ms | 16/16/1/1 |
| hole split | 0.278 ms | 1654.756 ms | 16/16/2/2 |

These measurements describe the permanent oracle path and are not a modal
latency claim.

## Named outcomes and known limits

The evaluator fails closed with named outcomes for missing/coordinate-free
geometry, non-planar or ambiguous frames, incomplete physical-edge lineage,
non-linear chain support without declared corner relations, uncertain Angular
selection, missing Junction route topology, exact barrier split, unsupported
bypass, unproven mixed effective alpha, undecidable exact predicates, and
non-manifold arrangements.

The deliberate v1 limitation exposed by the closed-chain fixture is that a
piecewise-linear closed route requires explicit segment-corner relations for
every bend. The evaluator emits no synthetic cap and returns
`PLANAR_CHAIN_SUPPORT_NOT_LINEAR` when those relations are absent. Full
obstacle bypass remains EC8+.

## Next-session allowlist

Session C review may inspect the new reference contracts, exact evaluator,
coordinate corpus, tests, workflow, and this handoff. Session D may begin only
after explicit user acceptance of this candidate. No Session D work is
included or implied here.
