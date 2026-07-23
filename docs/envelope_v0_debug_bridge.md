# Session V0 Envelope Debug Bridge

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

- the host U, V, and N basis is exactly orthonormal under the decimal values
  stored by Blender;
- `cross(U, V)` exactly proves the declared handedness and normal;
- every source vertex is exactly coplanar;
- every projected coordinate has an exact round-trip through the public
  float-valued frame contract;
- Angular relations have exact owner-side support directions and a certified
  rational interval for `phi/pi` and `(phi-pi)/pi`.

An unproved frame returns
`ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE`. An unproved Angular relation
returns `ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE`. There is no
basis rounding, tolerance, SDF/raster/Voronoi substitute, or legacy fallback.

The PatchNode v1 schema has no shape-class field. Its declared V0 mapping is
`PatchShapeClass.MIX`; an explicitly present but unknown shape class fails.

## Selection and topology

Selection is a set of complete physical edge routes. Selecting only part of a
PhysicalChain returns
`ENVELOPE_DEBUG_PARTIAL_CHAIN_SELECTION_UNSUPPORTED`.

A physical seam maps to two ChainUses in two PatchDomains. `SEAM_SELF` maps to
two distinct ChainUses in one PatchDomain. V0 never infers a
`SelfContactPairDeclarationV1`; real host assets therefore compile no
self-contact pair unless a later, explicit debug-fixture contract provides
one.

Only PatchDomains reached by the request are projected into the DebugScene.
Whole-chain selection is resolved before exact-frame admission, so unrelated
unselected PatchDomains cannot reject the request. A failing selected domain
has its own named diagnostic. The current facade publishes no scene when any
selected domain fails; it does not retain an older or partially successful
scene. Cross-Patch competition is never introduced.

## V0-B Grease Pencil projection

The immutable scene is rendered into an independent object named
`CFTUV_DEBUG_Envelope_<source object>`. The renderer creates these layers:

`ENV_00_PATCH_DOMAIN`, `ENV_01_HOLES`, `ENV_02_BARRIERS`,
`ENV_10_PHYSICAL_CHAINS`, `ENV_11_CHAIN_USES`,
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
layer/stroke sidecar.

All Blender-version-specific GP access remains in `cftuv/debug.py`.
The renderer and operator use only `GreasePencilDebugWriter`.

## Blender operator

The `Hotspot UV` sidebar has a separate
`Envelope Debug (Exact Planar)` box before the legacy Decals controls.

1. Select the source mesh and enter Edit Mode.
2. Enable Edge Select.
3. Select every edge of one or more complete PhysicalChains.
4. Set Alpha and press Build.

Build always clears the prior Envelope object and sidecar before evaluation.
An empty or invalid selection therefore cannot leave stale Raw/Resolved
diagnostics. Clear removes only the Envelope debug object and sidecar; the
existing Analyze and decal-rail GP objects remain independent.

Ten visibility controls update existing GP layer visibility without
recompiling geometry: Domains, Chains, Supports, Envelopes, Raw, Readings,
Equality, Resolved, Diagnostics, and Labels.

## Failure and staging law

`evaluate_envelope_debug()` recomputes from AnalysisBundle and publishes one
new scene. It never retains a previous scene. If a stage fails, downstream
records for that domain are absent and its named diagnostic is returned.
V0-B must clear previous Envelope strokes before displaying the new staged
result.

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
- 19 named GP layers and semantic sidecar data;
- source mesh coordinates remain unchanged;
- visibility callbacks;
- stale-result clearing on empty selection;
- explicit Clear cleanup;
- no production decal object is created.

The viewport evidence fixture and capture live under
`artifacts/envelope_v0/session_v0b_viewport.*`.

Ownership, station/UV, tessellation, GeometryBatch production, production
decal meshes, modal scheduling, native code, and Session E remain out of
scope.
