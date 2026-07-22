# Envelope Engine — START HERE

Status: **NORMATIVE ENTRYPOINT FOR AM11 WORK**.

This file is the first document for every new Envelope-kernel session. It does
not replace the detailed contracts; it selects their order and prevents an
agent from starting an implementation slice against superseded EC0 semantics.

## 1. Read order and precedence

Read in this order:

1. `docs/decal_envelope_linear_axis_rebaseline.md` — AM11 geometry and engine
   north star; highest authority for the new angular model.
2. `docs/envelope_external_agent_session_manifest.md` — restricted-context
   execution slices and handoff rules.
3. `docs/envelope_ec0_correction_log.md` — audit trail and current gate state.
4. `docs/envelope_backend_semantics.md` — retained EC0 semantics that do not
   conflict with AM11.
5. `docs/decal_envelope_roadmap_compromise.md` and
   `docs/envelope_kernel_pivot_instructions.md` — broader roadmap and AM7–AM10
   context.

On conflict, AM11 wins. The current v3 corpus remains useful evidence for
PatchDomain, ChainUse, request/domain identity, boundary-limited propagation,
cross-Patch coordination and mixed-alpha behavior, but the old core
`MITER/BEVEL/ROUND` records are not accepted AngularEnvelope semantics.

## 2. North star

```text
sparse source skeleton
+ expressive analytic Envelope specifications
+ exact resolved coverage
+ separate ownership
+ lazy deterministic event ledger
+ downstream tessellation
```

A full generalized straight-skeleton solver is optional. It is introduced only
if analytic Envelope laws, exact arrangement/union and the deterministic event
ledger cannot reproduce the accepted semantics.

## 3. Terms that must not be mixed

- `CornerRelation` / `JunctionRelation` / `TerminalRelation` are analysis facts.
- Seeds are request-specific compile products of those facts.
- `EnvelopeSpec` is an analytic material law over alpha.
- `EnvelopeInstance(alpha)` is one concrete contribution.
- `RawCoverage` is the exact union of boundary-resolved contributions.
- `ResolvedCoverage` is RawCoverage after explicitly approved interactions.
- Ownership/UV/station/provenance partition ResolvedCoverage and cannot alter
  its silhouette.
- Wavefront is an active labelled boundary state derived from Envelope specs;
  it is neither the source skeleton nor Coverage.

Required future tagged union:

```text
StripEnvelopeSpec
AngularEnvelopeSpec
JunctionEnvelopeSpec
CapEnvelopeSpec
```

`AngularEnvelopeSpec` replaces separate core MITER/BEVEL/ROUND algorithms with
one declared linear-reflex profile family. Zero-length hidden edges are local
support features inside that spec. A finite profile-controlled fan is accepted
semantic geometry; an uncontrolled sampled Voronoi fan is not.

## 4. Downstream tessellation — exact boundary

Downstream tessellation means converting an already authoritative semantic
arrangement into mesh faces. It begins only here:

```text
EnvelopeSpecs
→ EnvelopeInstances(alpha)
→ boundary-limited resolution
→ exact union
→ approved interactions
→ ownership / UV / station arrangement
→ semantic coalescing
→ DOWNSTREAM TESSELLATION
→ GeometryBatch faces and shared vertex identities
→ GPU preview or transactional Blender BMesh materialization
```

It may:

- triangulate or polygonize an already-defined semantic region;
- add representation vertices required by a named curved-surface error
  contract;
- preserve semantic boundaries, UV interfaces, provenance and shared keys;
- remove or replace internal diagonals only when all semantic records agree.

It may not:

- choose an angular profile or hidden-edge count;
- generate, smooth, repair or simplify the silhouette;
- infer owner or provenance after Boolean/arrangement;
- dissolve a profile-controlled fan edge that belongs to the semantic
  boundary;
- use face count, triangulation or mesh iteration order as semantic authority.

A polygonal chain produced by hidden supports is part of the Envelope boundary.
The triangles used later to fill its region are downstream tessellation.
Different valid tessellations must have the same canonical semantic digest.

## 5. Current gate and next allowed task

EC1 implementation is closed. The next allowed slice is **Session A — EC0
Linear-Reflex corpus rebaseline**.

Session A may modify only EC0 prose, JSON, schema, matrices and validator. It
must:

- replace core join enums with explicit angular profile references;
- migrate C02/C03/C04/C13/P06/P07 and dependent matrices;
- add variant-specific schema for all four EnvelopeSpec types;
- encode hidden-edge count, subdivision policy and support lineage;
- encode Strip support laws and Cap closure laws;
- make ownership claims explicit enough to prove total/disjoint partition;
- encode downstream-tessellation invariants;
- keep EC1 closed until validator/CI is green and the user accepts the migrated
  semantics.

No kernel evaluator, legacy geometry reuse or Blender integration belongs in
Session A.

## 6. Mandatory fresh-session boundaries

Start a new restricted-context session at each boundary:

1. EC0 semantics/corpus → EC1 public contracts and hermetic package.
2. Contracts → EC2 full-recompute reference Envelope evaluator.
3. RawCoverage → EC2.5 approved interactions.
4. ResolvedCoverage → EC3 ownership.
5. Ownership arrangement → EC4 coalescing/downstream tessellation.
6. Reference evaluator → EC5 lazy deterministic event ledger.
7. Profiled Python implementation → native/C++ acceleration.
8. Blender-free kernel → host shadow integration.
9. Planar production → curved/developable research.
10. Stable coverage contracts → separate cross-Patch lift work.

Do not keep one long-running agent across these boundaries merely because it
still has context. Each new session receives only this file, AM11, the session
manifest, the accepted artifact from the immediately preceding gate, the exact
slice task and its owned fixtures/tests.

## 7. Handoff minimum

Every session must leave a repository handoff containing:

```text
Active slice:
Input contract/version:
Accepted previous gate:
Changed paths:
Commit SHA:
Tests and exact result:
Canonical digest tested:
Named unsupported outcomes:
Assumptions not proven:
Risks:
Special opinion:
Legacy paths read: yes/no + reason:
Next-session allowlist:
```

A handoff that says only “tests green” is rejected.

## 8. Forbidden shortcuts

- treating Envelope as an alias for legacy emitted faces;
- independent per-pChain materialization followed by stitching;
- ownership or UV dividers changing the silhouette;
- post-mesh silhouette repair;
- fixture-tuned hidden-edge counts or tolerance-selected topology;
- eager complete future event scheduling as a startup requirement;
- small-step wavefront simulation;
- publishing a partial mesh while lazy event evaluation is pending;
- forcing smooth curved fronts to follow mesh edges solely because legacy rails
  did so;
- averaging normals by default for cross-Patch lift.
