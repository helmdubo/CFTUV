# Envelope Engine — external-agent session manifest

Status: **NORMATIVE OPERATIONAL COMPANION TO AM11**.

This document turns
`docs/decal_envelope_linear_axis_rebaseline.md` into restricted-context work
slices. It does not redefine geometry. If this document conflicts with AM11,
AM11 wins.

The goal is to prevent a single long-running agent from carrying legacy
implementation habits, provisional decisions, and unrelated host details across
semantic boundaries.

---

## 1. When a new session is mandatory

Start a fresh session whenever one of the following boundaries is crossed:

1. user-visible semantics become an accepted contract;
2. a public IR/schema becomes frozen;
3. coverage work hands off to interaction work;
4. interaction work hands off to ownership work;
5. semantic arrangement hands off to downstream tessellation;
6. the reference evaluator hands off to the lazy event ledger;
7. measured Python hot paths hand off to native acceleration;
8. the Blender-free kernel hands off to host integration;
9. planar production work hands off to curved-surface research.

Do not continue the same implementation session across these boundaries merely
because the agent still has context. The context separation is part of the
architecture.

---

## 2. Common entry packet for every kernel session

Every kernel session receives only:

- `AGENTS.md`;
- `docs/decal_envelope_linear_axis_rebaseline.md`;
- this session manifest;
- the accepted artifact/contracts produced by the immediately preceding gate;
- the exact task statement for the active slice;
- the tests and fixtures owned by that slice.

Additional files are allowlisted per session below.

Conversation transcripts are not authority. The previous session must leave a
factual repository handoff containing:

- accepted input contract/version;
- changed paths and commit SHA;
- tests run and exact result;
- unresolved named failures;
- assumptions not yet proven;
- risks and special opinion;
- explicit statement that forbidden legacy paths were or were not read.

---

## 3. Common forbidden context

Unless a session is explicitly the Legacy Evidence Curator or Host Adapter
Author, it must not read:

- `cftuv/decal_voronoi.py`;
- legacy geometry paths in `cftuv/decals.py`;
- archived decal generations;
- previous agent scratch files outside the accepted handoff;
- UI/operator code;
- Blender runtime code.

A kernel session works in a sparse checkout/worktree containing only the
allowlisted paths. This is stronger than a verbal instruction not to import
legacy modules.

---

## 4. Session A — EC0 Linear-Reflex corpus rebaseline

Status: **PASSED / CLOSED**. The accepted v5 corpus uses
`LINEAR_REFLEX_EQUAL_V1`, angle-driven `MIN_K_FOR_MAX_SUBTURN_V1`, and the
user-selected exact default `LINEAR_REFLEX_MAX_SUBTURN_V1 = pi/3 = 60 degrees`.
The accepted handoff opens Session B only in a fresh restricted context.

### Purpose

Migrate the current EC0 corpus from `MITER/BEVEL/ROUND` core branching to the
AM11 `AngularEnvelopeSpec` / `LinearReflexProfile` model.

### Allowed paths

- `docs/decal_envelope_linear_axis_rebaseline.md`;
- `docs/envelope_backend_semantics.md`;
- `docs/envelope_ec0_correction_log.md`;
- `docs/envelope_ec0_acceptance_guide.md`;
- `artifacts/envelope_ec0/corpus/**`;
- `tools/validate_envelope_ec0.py`;
- EC0 workflow files.

### Deliverables

- variant-specific tagged-union schema for
  `StripEnvelopeSpec | AngularEnvelopeSpec | JunctionEnvelopeSpec | CapEnvelopeSpec`;
- one explicit `AngularProfileId`, angle-driven hidden-edge selection
  certificate, oriented owner-sector, ordered supports, exact 60-degree
  max-subturn default, subdivision policy and hidden support lineage;
- removal of core join enums from the canonical request/plan model;
- migrated C02/C03/C04/C13/P06/P07 and dependent matrices;
- explicit support law for StripEnvelope and closure law for CapEnvelope;
- explicit downstream-tessellation contract;
- explicit ownership claims sufficient to prove total/disjoint partition;
- validator regressions for all of the above.

### Stop conditions

Stop and report instead of inventing semantics if:

- reflex-sector identity is not derivable from accepted analysis facts;
- hidden-edge count requires fixture-specific tuning;
- a case cannot be expressed without restoring a separate join algorithm;
- the requested fan would become sampling/tolerance authority;
- a 2*pi terminal is being forced into a normal reflex profile.

### Gate

**PASSED.** Validator and external CI are green, and explicit user acceptance
of the migrated verbal/JSON semantics is recorded as
`SESSION_A_FINAL_ACCEPTANCE`. EC1 may start only as Session B in a new
restricted context; continuing this Session A context is forbidden.

---

## 5. Session B — EC1 contracts and hermetic package

### Purpose

Create the Blender-free package and typed contracts only.

### Allowed inputs

Accepted Session A corpus/schema plus surface/GeometryBatch boundary contracts.

### Deliverables

- `AnalysisSnapshotV1` host-fact types;
- `DecalRequestV1` including angular, cap, boundary, interaction and ownership
  policy references;
- `CompiledPatchEvaluationPlan` types;
- typed EnvelopeSpec tagged union;
- `InitialFrontSpec`, event-predicate types and downstream `TessellationPlan`;
- deterministic serialization and canonical digest;
- hermetic wheel-only CI and extraction-readiness.

### Forbidden work

No geometry evaluator, Boolean union, ownership resolver, Blender adapter or
native extension.

### Gate

Package builds and tests in an environment where `bpy`, `mathutils` and `cftuv`
are physically absent. Repeated serialization is deterministic.

---

## 6. Session C — EC2 reference Envelope evaluator

### Purpose

Implement the permanent, full-recompute reference path for arbitrary alpha.

### Scope

- analytic StripEnvelope instances;
- Linear-Reflex AngularEnvelope instances with declared hidden supports;
- CapEnvelope and minimal JunctionEnvelope instances;
- boundary-limited reachability;
- exact patch-level union and complete contribution provenance.

### Forbidden work

No lazy event optimization, ownership tournament, GPU/BMesh output or
materializer repair.

### Gate

Reference RawCoverage matches accepted EC0 semantics for all fixtures and every
boundary segment/region retains source Envelope lineage.

---

## 7. Session D — EC2.5 approved interactions

### Purpose

Transform RawCoverage claims into ResolvedCoverage using only approved product
interactions.

### Scope

- same-request/same-Patch mutual arrival;
- policy-B coverage clip;
- freeze and capacity;
- self-collision;
- before/at/after event semantics.

### Forbidden work

Do not change Envelope definitions, hidden-edge profiles or ownership rules.
Cross-request and cross-Patch collision remain forbidden.

### Gate

No effect before mutual arrival, no overlap or new matter after the event, and
reference differentials are exact around every event.

---

## 8. Session E — EC3 ownership

### Purpose

Partition ResolvedCoverage into source/station/UV/provenance claims.

### Required invariant

Ownership cannot create, delete, move or repair the coverage silhouette.

### Forbidden work

No changes to Envelope geometry, interaction loci or boundary reachability.
No first-wins, smallest-id, tolerance-selected owner or reconstructed
provenance.

### Gate

Ownership is total and disjoint on open regions; equality boundaries are
ownerless; silhouette digest is unchanged.

---

## 9. Session F — EC4 arrangement, coalescing and downstream tessellation

### Purpose

Create the exact semantic arrangement, merge only semantically equivalent
regions, then encode the accepted arrangement as mesh faces.

### Downstream tessellation boundary

Tessellation begins only after:

```text
EnvelopeInstances
→ boundary resolution
→ exact union
→ approved interactions
→ ownership/UV/station arrangement
→ semantic coalescing
```

It ends by producing `GeometryBatch` faces and shared vertex identities for GPU
or BMesh adapters.

It may:

- triangulate or polygonize an already-defined semantic region;
- insert representation vertices required by curved-surface approximation under
  a named error contract;
- preserve semantic boundary chains, UV interfaces and shared keys;
- optimize internal diagonals when all semantic records are identical.

It may not:

- select an angular profile or hidden-edge count;
- create or repair silhouette geometry;
- infer owner/provenance after the fact;
- dissolve a profile-controlled fan edge that is part of the silhouette;
- use mesh face count as semantic authority.

### Gate

Different valid tessellations have the same canonical semantic digest and all
GeometryBatch elements trace to semantic regions/boundaries.

---

## 10. Session G — EC5 lazy deterministic event ledger

### Purpose

Accelerate drag without changing reference semantics.

### Startup work

Compile immutable Envelope laws, current exact state, event predicates,
transition types, spatial index and immediately reachable candidates only.

### Runtime work

- analytically advance within a cached topology interval;
- process same-alpha events as one atomic batch;
- lazily schedule successor candidates;
- use checkpoints/replay or a deterministic undo ledger for backward drag;
- publish only complete exact states;
- return `PENDING_EXACT_EVALUATION` rather than a partial mesh when the target is
  beyond the computed horizon.

### Forbidden work

Runtime may instantiate known predicates but may not invent new routes, event
types, hidden profiles or semantic rules.

### Gate

`Compiled(alpha) == Reference(alpha)` before, at and after every event; modal
latency meets the agreed budget.

---

## 11. Session H — native acceleration

Start only after profiling Sessions C–G. Move measured hot paths to C++ while
preserving the Python contract and permanent reference tests.

Typical candidates:

- exact predicates and intersections;
- arrangement/DCEL operations;
- swept broadphase;
- event queue and structure-of-arrays runtime state;
- downstream triangulation.

No redesign is permitted during the native port. Any required contract change
returns to the responsible earlier session/gate.

---

## 12. Session I — host shadow integration

A fresh Host Adapter Author session may read both host and kernel public APIs.
It performs only:

```text
AnalysisBundle → AnalysisSnapshotV1
user settings/selection → DecalRequestV1
GeometryBatch → GPU preview / transactional BMesh materialization
```

It cannot repair geometry, change owner, generate missing faces, reinterpret
Envelope profiles or fall back to legacy geometry.

Gate: shadow overlay and final materialization consume the same GeometryBatch
and match on the field fixture set.

---

## 13. Session J — curved/developable research

Start after planar cutover with a fresh research context.

The session keeps the same semantic identities and investigates evaluator
strategies:

- `PlanarEnvelopeEvaluator`;
- `DevelopableEnvelopeEvaluator` for proven isometric charts;
- `SurfaceEnvelopeEvaluator` for general curved patches.

Rails may be evaluated only under explicit roles:

```text
BARRIER | FOLD_GUIDE | SURFACE_ROUTE | CHARACTERISTIC_HINT
```

They are not silhouette authority. Smooth fronts may cross triangle interiors.
The deliverable is a feasibility/contract gate before implementation, not a
second unrelated user-facing Decal Engine.

---

## 14. Separate cross-Patch lift slice

Cross-Patch width propagation remains per PatchDomain. Extrinsic decal lift is
a later coordinated stage and deserves a fresh session after the relevant
coverage contracts are stable.

The session owns:

- shared lifted edge identity for two uses of one physical seam;
- global junction/lift anchors and per-Patch projections;
- named failures for ambiguous offset-surface intersections;
- prohibition of default normal averaging.

It does not reopen propagation collision across Patch boundaries.

---

## 15. Handoff template

Every session ends with a repository file or report containing:

```text
Active slice:
Input contract/version:
Accepted gate from previous slice:
Changed paths:
Commit SHA:
Tests and exact result:
Named unsupported outcomes:
Assumptions not proven:
Risks:
Special opinion:
Legacy paths read: yes/no + reason:
Next session allowlist:
```

A handoff is rejected if it only says “tests green” without identifying the
contract and semantic digest that were tested.
