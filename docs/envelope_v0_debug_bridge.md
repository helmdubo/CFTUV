# Session V0 Envelope Debug Bridge

## V0-A gate

V0-A is a static, Blender-free diagnostic boundary. It does not register a
Blender operator and does not write Grease Pencil data.

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
A failing selected domain has its own named diagnostic, while successful
selected domains may retain stage-complete layers. Cross-Patch competition is
never introduced.

## DebugScene layers

The immutable scene reserves these style keys for V0-B:

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
typed data. A renderer may convert these expressions to display floats, but
display values must never return to kernel semantics.

## Failure and staging law

`evaluate_envelope_debug()` recomputes from AnalysisBundle and publishes one
new scene. It never retains a previous scene. If a stage fails, downstream
records for that domain are absent and its named diagnostic is returned.
V0-B must clear previous Envelope strokes before displaying the new staged
result.

## V0-B remains closed at this gate

Grease Pencil compatibility wrappers, Blender operator/panel registration,
viewport fixtures, background Blender smoke, and screenshot evidence belong
to V0-B. They must consume `EnvelopeDebugSceneV1` without modifying Snapshot
mapping or kernel geometry.

Ownership, station/UV, tessellation, GeometryBatch production, production
decal meshes, modal scheduling, native code, and Session E remain out of
scope.
