# EC0 external-review correction log

Review target: branch `codex/ec0-envelope-semantics`, commit `b16be81`.
Current disposition: v5 A1/A2 correction candidate with user-selected exact
max-subturn `π/3 = 60°`; refreshed external CI and explicit final user
acceptance are still pending. History retained.

## Applied corrections

1. **Front cardinality (P0).** Removed automatic left/right sectors and two
   FrontComponents from ordinary ChainUse. Canon: one owner-interior sector and
   component; extras require analysis provenance.
2. **Single source of truth (P0).** Replaced duplicate
   skeleton/envelope/region/pivot descriptions with one authoritative JSON ID
   graph per case.
3. **C12 identity (P0).** Claim B is exactly one CornerEnvelope from a confirmed
   two-incident CornerRelation; it is not duplicated as StripEnvelope or a
   second PhysicalChain claim.
4. **C13 incident set (P0).** BEVEL CornerSeed/CornerEnvelope contains only its
   incident bevel uses. `other_wing_use` is interaction-only.
5. **Contract separation (P0).** AnalysisSnapshotV1 now contains facts/relations
   only. Request intent moved to DecalRequestV1; runtime state moved to
   CompiledPatchEvaluationPlan.
6. **Cross-Patch junction (P1).** Added a global JunctionRelation/shared anchor
   with per-Patch projections. Topology coordination is required; cross-Patch
   collision remains forbidden.
7. **Hole contact (P1).** Endpoint contact may slide/shrink. Interior contact
   that needs interval split yields `BARRIER_SPLIT_REQUIRED` at exact contact
   alpha.
8. **Launch exception (P1).** Distinguished `TOPOLOGICAL_BOUNDARY_USE`,
   `PHYSICAL_DOMAIN_BARRIER`, and `SOURCE_LAUNCH_BOUNDARY`; a source does not
   block its own seed.
9. **Request identity (P1).** Plan key is `(DecalRequestId, PatchDomainId)`;
   request/domain provenance is preserved through contributions, interactions,
   digest, PatchCoverage, and GeometryBatch.
10. **Mixed effective alpha (P1).** Corner/Junction shared envelopes consume an
    incident effective-alpha vector and never stretch a frozen side. Ambiguity
    returns `SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN`; neighboring fronts continue.
11. **C11 seed type (P1).** Endpoint claims use EndpointClaimSeed rather than a
    one-incident CornerSeed.
12. **Executable validation (P1).** Added `tools/validate_envelope_ec0.py` and a
    dedicated GitHub Actions workflow.
13. **Old corpus lifecycle (P1).** v1/v2 YAML and presentation visuals are
    removed from the active workspace. Commit `b16be81` remains the audit copy.
14. **Fallback wording (P1).** AGENTS contract now requires an explicit named
    unavailable/unsupported outcome; geometry fallback is forbidden unless a
    later gate separately approves it.

## AM11 Linear-Axis / sparse-envelope rebaseline

The subsequent design review identified a higher-level semantic correction,
recorded normatively in:

`docs/decal_envelope_linear_axis_rebaseline.md`

Restricted-context execution and handoff boundaries are recorded in:

`docs/envelope_external_agent_session_manifest.md`

The session manifest is an operational companion to AM11. It does not redefine
geometry; AM11 wins on any conflict. Kernel work must start a fresh session at
semantic, contract, coverage/interaction/ownership, tessellation, event-ledger,
native-port, host-integration and curved-research boundaries.

This is not a rollback of the PatchDomain/ChainUse correction. It preserves the
validated architecture:

- PatchDomain is the field;
- pChain/ChainUse is a source;
- Envelope contributions are resolved by exact union;
- approved interaction precedes ownership;
- ownership/UV/station dividers cannot alter the silhouette;
- downstream tessellation is representational only.

It changes the angular primitive model:

- `CornerRelation` and generated angular material are explicitly different
  authority levels;
- the future kernel type is `AngularEnvelopeSpec`, not an alias for
  `CornerRelation` or emitted faces;
- independent core algorithms `MITER | BEVEL | ROUND` are superseded by one
  linear-reflex profile family with declared zero-length hidden edges;
- a finite profile-controlled fan is legal semantic boundary geometry;
- hidden edges remain local support features and never become PhysicalChain,
  ChainUse or global rails;
- propagation is analytic between topology events;
- the production event ledger is deterministic and may be expanded lazily
  during drag rather than fully scheduled before the first frame;
- a full generalized straight-skeleton solver remains optional.

The v3 JSON corpus at commit `4c0cb57` therefore remains valid evidence for
PatchDomain, request/domain identity, boundary-limited propagation,
cross-Patch coordination and mixed-alpha behavior, but is **not accepted as the
final AngularEnvelope semantic corpus**. In particular, old `join_policy =
MITER/BEVEL/ROUND` records and dependent cases must be migrated before EC1.

Required migration scope is defined in section 10 of the AM11 document. Until
that migration, schema/validator update and explicit user acceptance, no kernel
implementation may begin.

## User artifact-policy correction

Presentation visual artifacts are forbidden for this project: no SVG/PNG
semantic sheets, diagrams, contact sheets, slides, or interactive HTML.
Preferred artifacts are prose, code, JSON, schemas, and validator output.

Blender viewport, UV Editor, and debug-overlay screenshots remain allowed as
diagnostic runtime evidence. They are not SemanticAuthority.

## Gate effect

The corrections do not open EC1. The user has approved
`LINEAR_REFLEX_MAX_SUBTURN_V1 = π/3 = 60°`; the corpus remains a correction
candidate until the refreshed local validator and real external GitHub workflow
are green and the user explicitly accepts the updated verbal semantics. There
are no current `BLOCKED_PENDING_USER_DECISION` items. Case 16 remains
user-selected policy B for wings of one DecalRequest in one Patch.

AM11 re-closes the EC0 semantic gate for angular cases until the canonical JSON
corpus is migrated from join enums to explicit angular profiles. Existing
validator success does not override this semantic gate.

## Operational note

The branch diverged from `main` during the EC0 work. No rebase or merge is
performed as part of this semantic rebaseline; reconcile with current main only
after preserving the validated EC0/AM11 workspace and before PR/merge.

## Superseded Session A receipt — fixed K0/K1 proposal

The following subsection is retained only as an audit record of the rejected
fixed K0/K1 proposal. Its v4 catalog, case mapping, uncommitted SHA text and CI
status are not current authority. They are superseded by the A1/A2 correction
receipt appended below.

Active slice: `SESSION_A_EC0_LINEAR_REFLEX_REBASELINE`.

Input contract/version: AM11,
`cftuv.envelope.ec0.case.v4`,
`cftuv.envelope.ec0.corpus.v4` and
`cftuv.envelope.ec0.envelope_spec_contracts.v1`.

Accepted gate from previous slice: PatchDomain/ChainUse/request-domain,
boundary-limited propagation, cross-Patch coordination and case 16 policy B
facts were preserved. No earlier acceptance is treated as acceptance of the
new K0/K1 catalog or mapping.

Changed paths are limited to the Session A allowlist:

- `docs/envelope_backend_semantics.md`;
- `docs/envelope_ec0_acceptance_guide.md`;
- this correction log;
- `artifacts/envelope_ec0/corpus/**`;
- `tools/validate_envelope_ec0.py`.

The corpus now uses a strict typed union of `StripEnvelopeSpec`,
`AngularEnvelopeSpec`, `JunctionEnvelopeSpec` and `CapEnvelopeSpec`, with a
separate alpha-bound `EnvelopeInstance`. The named Angular profiles are:

- `LINEAR_REFLEX_K0_EQUAL_V1`: zero hidden edges;
- `LINEAR_REFLEX_K1_EQUAL_V1`: one hidden edge/support at reflex-excess turn
  fraction `1/2`.

Both profiles use equal subdivision of `Δ = φ - π` into `k + 1` subturns.
Hidden-edge count is fixed by profile ID rather than threshold, tolerance,
tessellation or face count. C02/C04/C12/P06 use K0; C03/C13 use K1; P07 remains
a JunctionEnvelopeSpec.

Every plan now explicitly references the Strip support law, physical Cap
closure law, complete ownership partition, lazy deterministic event ledger and
downstream-tessellation invariants. C15 no longer requires an eager complete
future event schedule.

Commit SHA: Session A changes are currently uncommitted on base `95d57ee`.
The machine-readable receipt is
`artifacts/envelope_ec0/corpus/session_a_handoff.json` and must be updated with
the result SHA if the work is committed later.

Tests and exact result:

```text
python tools/validate_envelope_ec0.py
EC0 AM11 corpus validation OK
canonical JSON cases: 23 (main=16, pivot=7)
strict JSON Schema conformance: OK
typed union/profile/law/ownership/event/tessellation regressions: OK
EC1 gate: CLOSED_PENDING_EXPLICIT_USER_ACCEPTANCE
```

GitHub CI has not run for these uncommitted Session A changes. The existing
`.github/workflows/envelope-ec0-corpus.yml` executes the same validator under
Python 3.10 and will be required after commit/push before the gate can close.

Named unsupported outcomes remain explicit:
`BARRIER_SPLIT_REQUIRED`, `BARRIER_BYPASS_UNSUPPORTED`,
`SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN`,
`OWNERSHIP_PARTITION_UNPROVEN`, `PENDING_EXACT_EVALUATION` and
`APPROXIMATE_MATERIALIZATION_PENDING`.

Assumptions not proven:

- the user has not yet accepted the K0/K1 catalog and case mapping;
- no production Boolean/arrangement backend has been selected or proven;
- the future host adapter has not proven stable ChainUse/sector/relation facts;
- no geometry evaluator or Blender runtime fixture was built in Session A.

Risks:

- any future profile requires a new named ID and gate rather than retuning K0
  or K1;
- exact curved ownership/Junction behavior will require backend provenance
  support;
- runtime UI must distinguish requested, effective and pending alpha states.

Special opinion: keep the first profile catalog minimal and explicit. Do not
derive hidden-edge count from angle, sampling density or downstream faces.

Legacy paths read: **no**. Neither `cftuv/decal_voronoi.py` nor geometry parts
of `cftuv/decals.py` were read.

Next session allowlist: Session B may receive only the accepted Session A
corpus/schema plus surface and GeometryBatch boundary contracts, and may start
only after validator/CI and explicit user acceptance. EC1 remains closed now.

## Current Session A A1/A2 correction receipt

Before these edits the worktree was fetched and verified clean at remote commit
`522bd4cfae8f5aaa7d5392e20ebe53101c2eef0c`, with zero local/remote divergence.
The already-pushed Session A diff was not applied a second time. Commit
`522bd4cf` used the misleading message “Implement coordinate-free decal
envelope kernel”, although it contained only Session A semantic artifacts; it
was later replaced by the factual docs/EC0 history recorded below.

A1 changes AM11 authority and all dependent active documents:

- the only product profile family is `LINEAR_REFLEX_EQUAL_V1`;
- `MIN_K_FOR_MAX_SUBTURN_V1` computes
  `k = max(0, ceil(Δ / Δ_MAX) - 1)` from an exact/certified oriented reflex
  angle;
- at this A1 correction point `LINEAR_REFLEX_MAX_SUBTURN_V1` deliberately had
  no agent-selected default; the later user selection of exact 60° is recorded
  below and supersedes that temporary blocker;
- K0/K1 names survive only as C02/C03 regression-result fixtures;
- no permanent product case-to-k table remains.

A2 makes angular orientation and cardinality explicit:

- each Angular relation references `owner_sector_id`, ordered incoming/outgoing
  supports, owner-patch turn orientation and an angle certificate;
- hidden support ordinal `j` lies at `j / (k + 1)` inside that oriented sector;
- `k + 2` counts local AngularEnvelope profile supports/segments before
  clip/union/interaction, not guaranteed exposed silhouette segments.

Current machine-readable versions are:

- `cftuv.envelope.ec0.case.v5`;
- `cftuv.envelope.ec0.corpus.v5`;
- `cftuv.envelope.ec0.envelope_spec_contracts.v2`;
- `artifacts/envelope_ec0/corpus/session_a_handoff.json` v2.

No geometry evaluator, Blender integration, production Boolean selection or
EC1 artifact was added. Legacy geometry paths were not read.

The misleading commit was amended to factual semantic commit
`d5f3aae34f55f7cb1fd8a4017d1db8ea25d4d578` with message
`docs(ec0): rebaseline LinearReflex semantic corpus`. A separate receipt commit
records this SHA; a commit cannot embed its own final SHA, so the receipt commit
is identified by the enclosing Git history. At the time this receipt was
written, the lease-protected push and external workflow were still pending.
The branch was then published by exact `force-with-lease` from old remote
`522bd4cf` to receipt commit
`7fbd32a4a990a89273ad8b8ddb7c66d03e05b473`. External GitHub Actions run
`29949730053` tested that commit and concluded `SUCCESS`; this external record,
not the corpus or local validator, is the CI authority.

Current blockers:

- final local validator on the committed corpus;
- actual external GitHub workflow result on the published commit;
- repeat explicit user acceptance of the resulting A1/A2 semantics.

EC1 remains closed.

## User selection — LinearReflex max subturn

The user selected the v1 production default:

```text
LINEAR_REFLEX_MAX_SUBTURN_V1 = pi / 3 = 60 degrees = 1/6 turn
```

This closes `A1_MAX_SUBTURN_DEFAULT`. It does not create K0/K1 product modes:
`k` remains computed by `MIN_K_FOR_MAX_SUBTURN_V1` from each oriented-angle
certificate. The user explicitly allows later adjustment after runtime tests;
such an adjustment remains an explicit semantic change requiring corpus
revalidation, external CI and renewed acceptance, never silent tuning.

The selected value and all dependent authority artifacts were committed as
`10445ec8bcd51fc3ad9226e3ed823e769c76c14e` with message
`docs(ec0): select 60-degree LinearReflex max subturn`. At this receipt point,
external CI for that selection commit remains `PENDING`.
