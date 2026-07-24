# Session V0 Envelope Debug Bridge

## V0-R1A staged boundary

The Blender UI exposes two distinct actions. `Build Topology Debug` projects
immutable `EnvelopeTopologyDebugSceneV1` records directly from
`AnalysisBundle`. `Build Exact Reference Envelope Debug` keeps that topology
projection and evaluates exact reference stages independently per selected
PatchDomain.

Topology records contain source-local 3D coordinates and host identities for
Patch outer loops, holes, PhysicalChains, directed ChainUses, selected
sources, physical PATCH pairs, and `SEAM_SELF` pairs. This projection does not
load `cftuv-envelope-core`, import SymPy, construct `PlanarPatchFrameV1`, or
evaluate an Envelope. It is diagnostic host evidence, not kernel geometry
authority.

Each selected domain ends with exactly one receipt:
`TOPOLOGY_READY`, `METRIC_REJECTED`, `COMPILE_REJECTED`, `RAW_REJECTED`,
`INTERACTION_REJECTED`, or `RESOLVED`. The UI reports stage numerators rather
than a generic Built state. A late failure clears prior Raw/Resolved strokes
and publishes the new topology plus the current named failure.

## V0-A boundary

V0-A is the static, Blender-free diagnostic boundary consumed by V0-B.

The accepted flow for this gate is:

```text
AnalysisBundle
+ exact whole-PhysicalChain selection
+ requested alpha
→ AnalysisSnapshotV1 / DecalRequestV1
→ ReferenceEnvelopeCompilationV1
→ RawCoverageResultV1
→ InteractionResolutionResultV1
→ immutable EnvelopeDebugSceneV1
```

`cftuv/envelope_host_adapter.py` is the only host adapter. It maps original
source vertices, physical edges, polygon cycles, Blender triangles, Patch
facts, boundary chains, directed uses, owner sectors, terminals, and exact
planar frames. It does not read or import legacy decal geometry.

`kernel/src/cftuv_envelope/debug_scene.py` only projects already accepted
kernel records. It does not solve, clip, reconstruct arrivals, tessellate, or
repair geometry.

## Exact admission

V0 accepts only `PLANAR_EXACT` domains:

- all PatchDomain source vertices define one exact plane;
- the adapter derives an exactly orthonormal U/V/N frame from that plane;
- approximate host U/V/N values select orientation only and are not treated as
  a planarity or orthonormality certificate;
- `cross(U, V)` exactly proves the derived handedness and normal;
- every source vertex is exactly coplanar;
- every projected coordinate has an exact round-trip through the public
  float-valued frame contract;
- Angular relations have exact owner-side support directions and certified
  decimal intervals for `phi/pi` and `(phi-pi)/pi`. The angle ratio itself
  need not be a rational multiple of pi: each emitted decimal bound is proved
  against the exact support cosine, and `phi/pi` is the exact translation of
  the certified reflex-excess interval by one. V1 uses at most 27 fractional
  digits so that this translation remains exact under its 28-digit Decimal
  validation context.

An unproved frame returns
`ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE`. An unproved Angular relation
returns `ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE`. There is no
basis rounding, tolerance, SDF/raster/Voronoi substitute, or legacy fallback.

A geometrically exact plane can still be outside V0 when its exact unit normal
requires irrational components. `LocalVector3V1` is float-valued, so such a
normal cannot satisfy exact round-trip validation. Supporting that case
requires a separately accepted algebraic-frame contract; the Blender adapter
does not invent one for UI convenience.

The PatchNode v1 schema has no shape-class field. Its declared V0 mapping is
`PatchShapeClass.MIX`; an explicitly present but unknown shape class fails.

## Selection and topology

Selection is a set of complete physical edge routes. Selecting only part of a
PhysicalChain returns
`ENVELOPE_DEBUG_PARTIAL_CHAIN_SELECTION_UNSUPPORTED`.

A physical seam may be segmented differently by its two host-side
BoundaryChain lists. If both sides cover exactly the same non-repeated
physical-edge set, the adapter computes the common refinement using only
their declared endpoint vertices. Each resulting interval must have exactly
two patch-side uses. Missing coverage, repeated edges, or an unproved
partition still fails named; no coordinate or support heuristic participates.

A physical seam maps to two ChainUses in two PatchDomains. `SEAM_SELF` maps to
two distinct ChainUses in one PatchDomain. V0 never infers a
`SelfContactPairDeclarationV1`; real host assets therefore compile no
self-contact pair unless a later, explicit debug-fixture contract provides
one.

Only PatchDomains reached by the request are projected into the DebugScene.
Whole-chain selection is resolved before exact-frame admission, so unrelated
unselected PatchDomains cannot reject the request. A failing selected domain
has its own named diagnostic. The staged facade publishes topology for every
selected domain before metric admission. Exact scenes are request-scoped per
admitted PatchDomain; a rejected domain cannot erase topology or suppress an
admitted neighbor. Cross-Patch competition is never introduced.

## V0-B Grease Pencil projection

The immutable scene is rendered into an independent object named
`CFTUV_DEBUG_Envelope_<source object>`. The renderer creates these layers:

`ENV_00_PATCH_DOMAIN`, `ENV_01_HOLES`, `ENV_02_BARRIERS`,
`ENV_10_PHYSICAL_CHAINS`, `ENV_11_CHAIN_USES`,
`ENV_12_SELECTED_SOURCES`, `ENV_13_PATCH_PAIRS`,
`ENV_14_SEAM_SELF_PAIRS`,
`ENV_20_SOURCE_SUPPORTS`, `ENV_21_MOVING_FRONTS`,
`ENV_22_HIDDEN_SUPPORTS`, `ENV_30_ENVELOPE_INSTANCES`,
`ENV_40_RAW_COVERAGE`, `ENV_50_INTERACTION_COMPONENTS`,
`ENV_51_FRONT_READINGS`, `ENV_52_EQUALITY_LOCI`,
`ENV_60_RESOLVED_COVERAGE`, `ENV_61_RETAINED_REGIONS`,
`ENV_62_REMOVED_REGIONS`, `ENV_70_EVENT_ANCHORS`, and
`ENV_80_DIAGNOSTICS`.

Every point stores canonical exact expression strings. Semantic IDs,
PatchDomain IDs, source lineage, stage, kind, label, and style key remain
typed data. `cftuv/envelope_debug_renderer.py` converts exact expressions to
display floats only after the DebugScene is complete. The lift is
`origin + x*U + y*V` plus a display-only normal offset. Display values never
return to kernel semantics.

The GP object copies the source object's `matrix_world` and stores kernel
version, source revision, request IDs, PatchDomain IDs, alpha, Raw/Resolved
digests, and stage counts as custom properties. The Text datablock
`CFTUV_EnvelopeDebug_<source object>.json` stores the semantic ID to
layer/stroke sidecar. A second datablock,
`CFTUV_EnvelopeProfile_<source object>.json`, stores per-domain receipts,
stage timings, counters, and the dominant stage.

All Blender-version-specific GP access remains in `cftuv/debug.py`.
The renderer and operator use only `GreasePencilDebugWriter`.

## Blender operator

The `Hotspot UV` sidebar has a separate
`Envelope Debug (Staged)` box before the legacy Decals controls.

1. Select the source mesh and enter Edit Mode.
2. Enable Edge Select.
3. Select every edge of one or more complete PhysicalChains.
4. Press `Build Topology Debug`, or set Alpha and press
   `Build Exact Reference Envelope Debug`.

Both builds always clear the prior Envelope object and sidecars before
evaluation.
An empty or invalid selection therefore cannot leave stale Raw/Resolved
diagnostics. Clear removes only the Envelope debug object and sidecar; the
existing Analyze and decal-rail GP objects remain independent.

Ten visibility controls update existing GP layer visibility without
recompiling geometry: Domains, Chains, Supports, Envelopes, Raw, Readings,
Equality, Resolved, Diagnostics, and Labels.

## Failure and staging law

`evaluate_envelope_debug_staged()` first publishes host topology, then
recomputes exact reference stages per domain. It never retains a previous
scene. If a stage fails, downstream records for that domain are absent and its
named receipt is returned. V0-B clears previous Envelope strokes before
displaying the new staged result.

The runtime profile separately records `ANALYSIS_BUNDLE`, `SELECTION_SCOPE`,
`HOST_CHAIN_COLLECTION`, `SEAM_PARTITION_NORMALIZATION`, `BUNDLE_SLICE`,
`TOPOLOGY_SCENE`, `SNAPSHOT_EXPORT`, per-domain `FRAME_ADMISSION`,
`ANGULAR_RELATIONS`, `SNAPSHOT_VALIDATION`, `COMPILE`, `DOMAIN_BUILD`,
`ENVELOPE_INSTANCE_BUILD`, `DOMAIN_CLIP`, `RAW_UNION`, `INTERACTION`,
`DEBUG_SCENE`, `GP_RENDER`, and `SIDECAR_JSON`. Counters cover host mesh and
selection size, per-domain topology, Envelope specs, arrangement work, and GP
stroke/point output. A compact timing table is also printed to the console.

## V0-R1B host export and session lifecycle

`cftuv/envelope_host_adapter.py` is now a thin compatibility facade. Runtime
ownership is split into:

- `envelope_topology_export.py`: immutable SourceRevision-scoped host-chain
  collection, seam common refinement, whole-chain selection views, and
  topology debug projection;
- `envelope_metric_export.py`: selected PatchDomain exact metric and immutable
  domain snapshot export;
- `envelope_request_export.py`: `DecalRequestV1` construction and reference
  C/D evaluation;
- `envelope_debug_session.py`: explicit analysis/export cache ownership and
  invalidation.

The only legacy evidence used by this slice was the factual Session L0 report
at commit
`3e3e10864b093d63a56336494dd752efe18b2d6a:docs/envelope_legacy_planarity_chainuse_evidence.md`.
No legacy geometry source was opened or called.

Blender stores the controller only as the runtime attribute
`context.window_manager._cftuv_envelope_debug_session`. There is no
module-global controller or cache.

The cache layers are:

- `AnalysisBundleCache`: source object plus SourceRevision;
- `TopologyExportCache`: SourceRevision;
- `PatchMetricCache`: SourceRevision plus PatchDomainId;
- `DomainGeometryCache`: SourceRevision plus PatchDomainId;
- `CompiledEnvelopeCache`: declared with a SourceRevision, selected ChainUse,
  and policy key, but disabled while the current reference compile contract
  contains requested alpha.

Request-scoped evaluation uses `AnalysisBundleIdView`, an immutable shallow
ID filter over the full bundle. It does not construct a copied PatchGraph or
PatchSurfaceIR. Selection and alpha rebuild request/evaluation state but keep
analysis, normalized PhysicalChains, exact metric, and domain geometry.
Visibility and view/camera changes do not enter the session pipeline.

The SourceRevision fingerprint includes vertex coordinates, edge endpoints,
seam flags, face cycles, and analysis scope. A changed revision or replaced
object data invalidates all dependent layers before reuse. Profile JSON and
the console report expose per-layer cache hits, misses, cumulative build
counts, and invalidation count.

## V0-B validation

`tests/blender/test_envelope_debug_bridge.py` is the Blender 4.3 background
smoke. It verifies:

- axis-aligned exact-planar staged diagnostics;
- request-scoped frame admission with an unselected tilted patch;
- a two-Patch physical seam reaching ResolvedCoverage in two domains;
- K0/K1/K2 Angular profile rendering with visible hidden supports;
- exact before/at/after mutual-arrival equality-locus rendering;
- hole and concave `BARRIER_SPLIT_REQUIRED` diagnostics;
- named unsupported Junction diagnostics;
- 22 named GP layers and semantic sidecar data;
- source mesh coordinates remain unchanged;
- visibility callbacks;
- stale-result clearing on empty selection;
- explicit Clear cleanup;
- no production decal object is created.
- alpha `0.2 -> 0.3 -> 0.4` reuses one AnalysisBundle, topology export,
  Patch metric, and domain geometry export;
- vertex-coordinate and seam-flag changes invalidate every dependent cache.

The viewport evidence fixture and capture live under
`artifacts/envelope_v0/session_v0b_viewport.*`.

Ownership, station/UV, tessellation, GeometryBatch production, production
decal meshes, modal scheduling, native code, and Session E remain out of
scope.
