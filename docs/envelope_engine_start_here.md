# Envelope Engine — START HERE

Document status: `CURRENT_ENVELOPE_ONBOARDING_AUTHORITY`.

This file is the first document for every new Envelope-kernel session. It does
not replace detailed contracts. It defines their authority order and prevents
an agent from starting a slice from historical session sequencing.

## 1. Read order and precedence

Resolve dispatch in this order:

1. `docs/architecture_status.json` from
   `codex/base-00-canonical-integration` — live control state, immutable base,
   active blocker and `current_card_path`.
2. `AGENTS.md` — repository invariants, role separation and forbidden context.
3. `docs/agent_execution/envelope_v1/01_GLOBAL_CANON.md` — accepted Envelope v1
   architecture and dependency order.
4. `docs/agent_execution/envelope_v1/02_AGENT_PROTOCOL.md` — branch, allowlist,
   gate and handoff rules.
5. The exact card at `current_card_path` — sole authority for the active slice.
6. Accepted dependency handoffs named by the live status, then only the card's
   allowlisted files.

The immutable base may contain an older copy of `docs/architecture_status.json`;
never let it downgrade the canonical live control state.

Document classifications:

- `docs/decal_envelope_linear_axis_rebaseline.md`:
  `ACCEPTED_SCOPED_SEMANTIC_EVIDENCE`;
- `docs/envelope_backend_semantics.md`:
  `ACCEPTED_SCOPED_SEMANTIC_EVIDENCE`;
- `docs/envelope_external_agent_session_manifest.md`:
  `SUPERSEDED_FOR_ACTIVE_DISPATCH`;
- older v3 `MITER/BEVEL/ROUND` corpus:
  `HISTORICAL_AUDIT_EVIDENCE`;
- `cftuv/decals.py`, `cftuv/decal_voronoi.py` and `pyvoronoi`:
  `SCOPED_LEGACY_RUNTIME`.

These labels preserve historical evidence without allowing it to select the
current branch, card or backend.

Current architecture map:

```text
AnalysisSnapshotV1 + DecalRequestV1
→ kernel-owned CompiledPatchEvaluationPlan
→ exact RawCoverage
→ approved interactions and ownership
→ SemanticArrangement
→ GeometryBatch
→ optional host preview/materialization
```

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

`BASE-00` is accepted at immutable content SHA
`e4db68371cab83a6a26368bf9a95eda74ae8d02e`, including its docs-only
control-plane amendment.

The active slice on this branch is `DOC-00`. Its exact card is
`docs/agent_execution/envelope_v1/cards/DOC-00_Documentation_authority_and_AI_onboarding_correction.md`.
`FIX-00` is a separate parallel card and is not part of this session.
`D-R2-00` remains blocked until both `DOC-00` and `FIX-00` are accepted. The
active correctness blocker carried to D-R2 is
`MULTIWAY_INTERACTION_POLICY_UNPROVEN`.

`C-R2C-01` through `C-R2C-04` are
`SUPERSEDED_BY_ACCEPTED_C_R2C_GATE` and must not be dispatched.

The live status manifest remains authoritative if this branch text later
becomes stale.

## 6. Historical accepted gates and stage boundaries

Session A — EC0 Linear-Reflex corpus rebaseline is accepted and closed. Its
accepted result:

- replaced core join enums with explicit angular profile references;
- migrated C02/C03/C04/C13/P06/P07 and dependent matrices;
- added variant-specific schema for all four EnvelopeSpec types;
- encoded angle-driven hidden-edge selection, subdivision and support lineage;
- encoded Strip support laws and Cap closure laws;
- made ownership claims explicit enough to prove total/disjoint partition;
- encoded downstream-tessellation invariants;
- uses `LINEAR_REFLEX_EQUAL_V1` with angle-driven
  `MIN_K_FOR_MAX_SUBTURN_V1` and exact max-subturn `pi/3 = 60 degrees`;
- passed local validator and external CI;
- records `SESSION_A_FINAL_ACCEPTANCE`.

The historical stage model required a new restricted-context session at each
boundary:

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

These boundaries remain architectural evidence, but they do not dispatch a
current task. Every new session receives the live status, AGENTS, global canon,
protocol, exact active card, accepted predecessor handoffs, and only its
owned fixtures/tests.

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
