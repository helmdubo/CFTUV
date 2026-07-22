# Decal Envelope Engine — Linear-Axis rebaseline (AM11)

Status: **NORMATIVE REBASELINE ACCEPTED — SESSION A CLOSED**.
This document supersedes the old MITER/BEVEL/ROUND interpretation of
`CornerEnvelope` for all new kernel work. EC1 implementation remains closed
The EC0 JSON corpus, schema and validator are migrated to this model; external
CI is green; and the user has accepted the resulting semantics, including
`LINEAR_REFLEX_MAX_SUBTURN_V1 = pi/3 = 60 degrees`. Session A is closed. EC1
may start only as Session B in a new restricted context.

The architectural north star is:

```text
sparse source skeleton
+ expressive analytic Envelope specifications
+ exact resolved coverage
+ separate ownership
+ lazy deterministic event ledger
+ downstream tessellation
```

A full generalized straight-skeleton solver is not a prerequisite. It is only a
fallback if analytic Envelopes, exact arrangement and a deterministic event
ledger cannot reproduce the approved semantics.

---

## 1. The central material law

For one `(DecalRequestId, PatchDomainId)`, every active source contributes an
analytic, provenance-bearing Envelope family `E_i(alpha)`.

```text
RawCoverage(alpha)
    = PatchDomain ∩ exact_union(E_1(alpha), ..., E_n(alpha))

ResolvedCoverage(alpha)
    = approved_interactions(RawCoverage(alpha), contribution claims)

Ownership(alpha)
    = a total, disjoint partition of ResolvedCoverage(alpha)
```

Only an explicitly approved interaction may change `RawCoverage` into
`ResolvedCoverage`. Ownership, UV, station and provenance dividers operate
*inside* `ResolvedCoverage`; they must never modify its silhouette.

This is the generalized form of the field-proven RC5b/mutual-arrival lesson:
inside one material contribution/component, internal attribution boundaries do
not create or remove matter. The selected same-request/same-Patch policy B is a
separate interaction stage and may clip contributions at a declared equality
locus before ownership.

---

## 2. Vocabulary and authority levels

The following terms are not interchangeable.

### 2.1 Analysis relation

`CornerRelation`, `JunctionRelation` and `TerminalRelation` are observed or
derived topology facts. They contain source identity, incident `ChainUse`s,
Patch sectors, ordering and provenance. An angular relation additionally
references an analysis-proven `owner_sector_id`, its ordered incoming/outgoing
supports, a turn orientation and an exact or certified owner-sector angle.
They do not generate geometry.

### 2.2 Seed

A `CornerSeed`, `JunctionSeed`, `TerminalSeed` or `FrontSeed` is the
request-specific compile product of an analysis relation. It selects the
participating sectors and profiles. It is not emitted geometry.

### 2.3 EnvelopeSpec

An `EnvelopeSpec` is an immutable analytic law describing a family of material
contributions over `alpha`.

Required tagged union:

```text
StripEnvelopeSpec
AngularEnvelopeSpec
JunctionEnvelopeSpec
CapEnvelopeSpec
```

`CornerEnvelope` is renamed conceptually to `AngularEnvelopeSpec` in the new
kernel contract to avoid confusion with `CornerRelation`. Historic names may
remain only in migration notes and UI presets.

### 2.4 EnvelopeInstance

`EnvelopeInstance(alpha)` is the concrete boundary/region produced by one spec
at one effective alpha vector.

### 2.5 Wavefront

A wavefront is the active, labelled boundary state derived from Envelope specs.
There are two different views:

- contribution front — boundary features of an individual Envelope;
- coverage front — the external silhouette of exact resolved coverage.

An individual contribution front can become internal after union while still
remaining relevant to event and ownership provenance.

### 2.6 Coverage

Coverage is an area/region, not a generator. `Envelope` and `Coverage` are not
synonyms.

### 2.7 Downstream tessellation

Downstream tessellation is the final representational step:

```text
EnvelopeSpecs
→ EnvelopeInstances(alpha)
→ boundary resolution
→ exact union
→ approved interactions
→ ownership / UV / station arrangement
→ semantic coalescing
→ downstream tessellation
→ GeometryBatch triangles/n-gons
→ GPU or Blender BMesh adapter
```

It chooses how to encode an already-authoritative semantic arrangement as mesh
faces. It must not choose joins, repair silhouettes, infer ownership, recreate
provenance or change topology. Runtime face count and triangulation are not part
of Envelope semantics. Different valid tessellations must have the same
canonical semantic digest.

A controlled fan that is part of an `AngularEnvelopeSpec` boundary is *not*
downstream tessellation noise. It is semantic geometry. Triangles later used to
fill that polygonal region are downstream tessellation.

---

## 3. StripEnvelope semantics

One ordinary directed owner-side `ChainUse` produces one owner-interior
`FrontComponent` and one `StripEnvelopeSpec`.

For a straight planar source segment, the longitudinal support and moving front
support are parallel. The moving support follows a linear normal-offset law.

```text
source support: n · x = c
moving support: n · x = c + alpha
```

The two *longitudinal* strip boundaries are parallel support features. The
terminal interfaces where the strip meets an AngularEnvelope or CapEnvelope are
not required to be perpendicular. They are supplied by the adjacent semantic
Envelope and may be oblique.

Required fields for the future tagged schema:

```text
source ChainUse identity
owner PatchDomain and sector
support-line law / surface metric law
station model s
transverse coordinate r
terminal interface references
boundary and source provenance
```

A StripEnvelope is never materialized independently and stitched afterward.

---

## 4. Unified AngularEnvelope semantics

The kernel must not implement MITER, BEVEL and ROUND as three unrelated corner
algorithms. They are replaced by one linear-reflex profile family.

### 4.1 Reflex profile

For a planar reflex sector with internal angle `phi`, where

```text
pi < phi < 2*pi
```

define reflex excess:

```text
delta = phi - pi
```

A profile with `k` zero-length hidden edges splits this excess into `k + 1`
sub-turns. The default profile family uses equal subdivision:

```text
subturn = delta / (k + 1)
```

At `alpha = 0`, hidden edges have zero length and belong only to the local
`AngularEnvelopeSpec`. They are not `PhysicalChain`, `ChainUse`, global rails or
new source topology. At `alpha > 0`, their supporting lines generate a finite,
controlled polygonal chain.

All ordinary and hidden wavefront edges move with the same unit normal speed.
Their intersection vertices generally move at another linear speed determined
by the incident support directions. Increasing `k` reduces extreme reflex
vertex motion and yields a smoother polygonal approximation of the uniform
offset.

### 4.2 No core join enum

The new core contract removes semantic branching on:

```text
MITER | BEVEL | ROUND
```

and replaces it with:

```text
AngularProfileId
hidden_edge_count
subdivision_policy
hidden_support_directions
profile provenance
```

Historic UI names may compile to named profile presets, but they are not kernel
algorithms and cannot introduce separate code paths.

### 4.3 Controlled fan is legal

A polygonal fan/chain produced by declared hidden supports is accepted semantic
geometry because it is:

- finite and profile-controlled;
- deterministic;
- local to one angular relation/sector;
- source-provenanced;
- independent of runtime sampling tolerance;
- part of the intended silhouette where exposed.

This is categorically different from the rejected Voronoi fan, whose topology
was an uncontrolled consequence of mixed finite point/segment bisectors and
sampling.

### 4.4 Profile selection

Session A defines one `AngularProfileId`:

```text
LINEAR_REFLEX_EQUAL_V1
```

It is parameterized by an angle-driven selection certificate; `K0` and `K1`
are not separate product profiles. The selected policy is:

```text
MIN_K_FOR_MAX_SUBTURN_V1
delta_max = LINEAR_REFLEX_MAX_SUBTURN_V1
k = max(0, ceil(delta / delta_max) - 1)
```

Equivalently, `k` is the smallest non-negative integer for which
`delta / (k + 1) <= delta_max`. The compiler derives it from the certified
oriented owner-sector angle and the named request policy. A coordinate-free
selection certificate may encode the exact/certified interval
`k < delta / delta_max <= k + 1`; the validator derives `k` from that interval
instead of trusting a separate case mapping. It must not be
selected manually per corner, by fixture identity, sampling density,
tessellation, face count, Boolean output or hidden epsilon fitting.

The user-selected v1 product default is exact:

```text
LINEAR_REFLEX_MAX_SUBTURN_V1 = pi / 3 = 60 degrees = 1/6 turn
```

This is a product semantic value, not an implementation tolerance. The user
may revise it after runtime tests, but that requires an explicit corpus update,
revalidation and renewed acceptance; an implementation must not silently tune
or hide a different numeric constant.

For resolved `k`, hidden support ordinal `j`, where `1 <= j <= k`, is placed at
turn fraction `j / (k + 1)` from the ordered incoming support toward the
ordered outgoing support inside the oriented owner sector. The local profile
has `k + 2` active supports and `k + 2` local AngularEnvelope profile segments
before clipping, union and interaction. This is local profile cardinality, not
a promise that all `k + 2` segments remain exposed on the final resolved
silhouette.

`LINEAR_REFLEX_K0_EQUAL_FIXTURE_V1` and
`LINEAR_REFLEX_K1_EQUAL_FIXTURE_V1` are regression-result labels for cases in
which the certificate resolves respectively to `k = 0` and `k = 1`. They are
not `AngularProfileId`s, request choices or a permanent case-to-profile table.

The local angle-only policy is a product definition, not a claim that the full
Tanase–Veltkamp global epsilon-equivalence proof has been implemented. If global
conflicting-site refinement is later required, it is a separate capability.

### 4.5 Degenerate 360-degree cases

An exact `2*pi` sector is not treated as a normal reflex vertex. Open
`SEAM_SELF` endpoints and poles are represented by `TerminalRelation` or
`JunctionRelation`, then compiled into Cap/Junction Envelopes. The kernel must
not invent an `AngularEnvelope(phi = 2*pi)`.

---

## 5. Initial wavefront topology

The initial wavefront topology is a derived `InitialFrontSpec`, not the
PatchCoverage and not the source skeleton itself.

It is assembled from all active Envelope specs in one
`(DecalRequestId, PatchDomainId)` evaluation:

- Strip support features;
- zero-length hidden support features from AngularEnvelope specs;
- JunctionEnvelope support features;
- CapEnvelope closure features;
- fixed PatchDomain constraints.

Exact union remains the shape authority. The front graph is the kinetic state
used to predict and process topology changes.

---

## 6. Propagation and event strategy

Motion is analytic between events and discrete at events.

For a planar moving support:

```text
n_i · x = c_i + v_i * alpha
```

with fixed normal and speed in one topology interval. Intersections of active
supports have affine trajectories in alpha. The engine must evaluate those
trajectories directly; it must not advance by small simulation steps.

### 6.1 Event classes

Minimum named groups:

```text
local envelope:
    HIDDEN_EDGE_ACTIVATION
    FRONT_EDGE_COLLAPSE
    CAP_COLLAPSE

domain:
    BOUNDARY_CONTACT
    ENDPOINT_SLIDE_START
    BARRIER_SPLIT_REQUIRED
    FRONT_EXHAUSTED

arrangement / coverage:
    EDGE_EDGE_CONTACT
    VERTEX_EDGE_CONTACT
    COVERAGE_COMPONENT_MERGE
    COVERAGE_COMPONENT_SPLIT
    SILHOUETTE_CHANGE

approved interaction:
    MUTUAL_ARRIVAL
    FREEZE
    CAPACITY

ownership-only:
    DIVIDER_BIRTH
    DIVIDER_DEATH
    UV_INTERFACE_CHANGE
```

Ownership-only events cannot change the silhouette.

### 6.2 Lazy deterministic event ledger

The engine must not eagerly calculate the complete future event schedule before
the first preview frame.

At tool start it compiles:

- immutable Envelope specs and support laws;
- initial exact state at the requested alpha;
- event predicates and transition rules;
- an initial spatial index;
- only immediately reachable candidate events.

During forward drag:

```text
AdvanceTo(target_alpha)
    consume cached topology intervals
    process an atomic batch of exact events up to target
    lazily schedule successor candidates
    publish only a complete exact state
```

During backward drag, use a deterministic event undo log or checkpoint plus
replay. Background prefetch may expand a bounded alpha horizon after publishing
the current frame.

If the user requests alpha beyond the computed horizon, retain the last complete
exact frame and report `PENDING_EXACT_EVALUATION`; never expose a partial mesh.
Confirm uses the same `AdvanceTo` path.

The invariant is not “all event instances are known before drag.” It is:

> all source facts, geometric laws, event predicates and transition types are
> compiled before they are used; runtime may instantiate and schedule successor
> events lazily but may not invent new semantic rules or routes.

The full reference evaluator remains permanent and must match the compiled path
before, at and after every event.

---

## 7. Data granularity

Use several deliberate levels rather than one Python object per runtime point or
one monolithic graph.

### Sparse immutable semantic records

```text
PatchDomain
PhysicalChain
ChainUse
Corner/Junction/Terminal relations
EnvelopeSpec
DecalRequest
```

### Local support features

Hidden edges are compact arrays/records inside an AngularEnvelope spec and
expand to support features only for evaluation.

### Dense runtime state

Use integer handles and contiguous structure-of-arrays storage for front edges,
vertices, adjacency, birth/death event ids and source provenance. Avoid Python
object graphs for thousands of moving features.

### Event arena and queue

Events carry exact/certified time, kind, participant handles, generation and
predicate provenance. Simultaneous events are processed as one deterministic
batch.

### Arrangement

DCEL/half-edge regions retain boundary lineage, covering Envelope ids,
interaction state and ownership claims.

### Output

`GeometryBatch` is created only after semantic coalescing and downstream
tessellation.

The logical evaluation unit remains `(DecalRequestId, PatchDomainId)`. Proven
non-interacting `InteractionIsland`s may be used for parallel computation, but
they are not separate semantic domains.

---

## 8. Curved and developable roadmap

There is one Envelope semantic contract with multiple metric/evaluator
strategies, not multiple unrelated user-facing Decal Engines.

### Planar exact

`PlanarEnvelopeEvaluator` uses linear supports, angular hidden supports, exact
planar arrangement and the lazy kinetic ledger.

### Developable / ribbon-like

A `DevelopableEnvelopeEvaluator` may evaluate the same Envelope laws in a proven
isometric chart and lift them back. Approximate flattening is not silently
accepted as exact.

### General curved

`SurfaceEnvelopeEvaluator` retains Strip/Angular/Junction/Cap identities, but
interprets them intrinsically:

- planar parallel supports become equidistant surface fronts;
- hidden support directions become initial tangent characteristics;
- alpha is geodesic distance;
- planar arrangement becomes triangle/surface-local arrangement;
- collisions become characteristic/cut-locus events.

The curved evaluator is not planar Boolean stretched through a distorted chart.

### Role of legacy rails

Rails are not the silhouette authority. Future roles are explicit and
orthogonal:

```text
BARRIER
FOLD_GUIDE
SURFACE_ROUTE
CHARACTERISTIC_HINT
```

A hard crease rail may be an exact characteristic. On smooth curvature, fronts
may cross triangle interiors; they are not forced to mesh-edge routes.

---

## 9. Cross-Patch surface lift

Intrinsic width propagation is evaluated separately inside each PatchDomain.
Extrinsic decal offset/lift is a later coordinated stage.

```text
ResolvedPatchCoverage
→ CrossPatchLiftCoordinator
→ semantic coalescing
→ downstream tessellation
→ GeometryBatch
```

Two patch-side uses of one physical seam must share lifted edge identity. For
planar incident patches, the shared lifted line is determined from the
intersection of the two offset planes when a unique supported solution exists.
At multi-Patch junctions, a global junction/lift relation provides a shared
anchor and per-Patch projections.

Default averaging of normals is forbidden. Ambiguous or unsupported lift
produces a named failure. Cross-Patch topology coordination is required even
though cross-Patch propagation collision remains forbidden.

---

## 10. Required EC0/EC1 migration

The current v3 EC0 corpus remains useful evidence for PatchDomain, ChainUse,
request identity, boundary propagation, cross-Patch coordination and mixed
alpha. It is not accepted as final AngularEnvelope semantics.

Before EC1 implementation:

1. Replace core `join_policy = MITER/BEVEL/ROUND` with
   `LINEAR_REFLEX_EQUAL_V1` plus an angle-driven selection certificate.
2. Migrate C02/C03/C04/C13/P06/P07 and any dependent matrices.
3. Add `AngularEnvelopeSpec` fields for oriented owner-sector identity,
   ordered incident supports, turn orientation, reflex-angle certificate,
   computed hidden-edge count, subdivision policy and hidden support lineage.
4. Add variant-specific tagged-union schema for all four Envelope specs.
5. State support laws for StripEnvelope and closure laws for CapEnvelope.
6. Make ownership claims explicit enough to prove total/disjoint partition.
7. Add lazy-ledger semantics and forbid eager-full-schedule as a startup
   requirement.
8. Add downstream-tessellation invariants and semantic-digest equivalence.
9. Reserve `SurfaceEvaluatorStrategy`, rail roles and `CrossPatchLiftRelation`
   in contracts without implementing EC8.
10. Record the user-selected exact default
    `LINEAR_REFLEX_MAX_SUBTURN_V1 = pi/3 = 60 degrees`; any later test-driven
    revision is an explicit semantic change, not hidden tuning.
11. Re-run corpus validation and external CI, then request explicit user
    acceptance.

No kernel implementation task may reinterpret an Envelope as an alias for
legacy emitted faces.

---

## 11. Session boundaries for external agents

Use new, restricted-context sessions at semantic and ownership boundaries. Do
not let one agent carry implementation history across all stages.

### Session A — EC0 Linear-Axis corpus migration

Reads only this document, EC0 semantics/corpus/schema/validator and the
compromise roadmap. Writes JSON/schema/validator/docs only. Does not read legacy
geometry code and does not create kernel code.

Gate: **PASSED** — the user accepted the migrated semantic corpus with the
exact 60-degree max-subturn default. Session B must use a new restricted
context; this Session A context must not implement EC1.

### Session B — EC1 contracts and hermetic package

Reads accepted EC0 artifacts and contract docs only. Creates the Blender-free
package, typed records, serialization and hermetic CI. No geometry evaluator.

Gate: extraction-readiness and deterministic snapshot hashes.

### Session C — EC2 reference Envelope evaluator

Reads accepted contracts and Envelope semantic fixtures. Implements analytic
EnvelopeInstance generation and full exact reference union per alpha. No event
optimization and no ownership tournament.

Gate: reference coverage matches EC0 and preserves provenance.

### Session D — EC2.5 interactions

Reads reference coverage API and interaction cases only. Implements mutual
arrival/freeze/capacity as transformations from RawCoverage claims to
ResolvedCoverage. It cannot alter Envelope definitions.

Gate: before/at/after-event differentials.

### Session E — EC3 ownership

Reads ResolvedCoverage and ownership fixtures only. Implements total/disjoint
partition, station and UV claims. It cannot change coverage.

Gate: exact ownership and silhouette invariance.

### Session F — EC4 arrangement/coalescing/tessellation

Reads resolved semantic arrangement contracts. Implements ownership overlay,
semantic coalescing and downstream tessellation. It cannot add shape repair.

Gate: multiple tessellations share one semantic digest and produce valid
GeometryBatch.

### Session G — EC5 lazy event ledger

Reads reference evaluator and accepted event semantics. Implements interval
motion, lazy scheduling, checkpoints/replay and compiled/reference differential.
It cannot change Envelope or interaction semantics.

Gate: modal performance and exact differential around events.

### Session H — native acceleration

Starts only after profiling EC2–EC5. Moves measured hot paths to C++ while
preserving the Python contract and reference tests. No redesign in the native
port.

### Session I — host shadow integration

A separate host-adapter agent may read both CFTUV host and kernel API. It only
maps AnalysisBundle to Snapshot/Request and GeometryBatch to preview/BMesh. No
geometry repair or fallback.

### Session J — curved research

A fresh research session after planar cutover. Reads accepted Envelope contracts
and curved fixtures, not planar implementation internals unless necessary for
API compatibility. Produces a strategy/feasibility gate before implementation.

Each new session receives a small manifest of permitted documents and paths.
The previous agent writes a factual handoff artifact; conversational history is
not the authority.
