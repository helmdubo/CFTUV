# EC0 external-review correction log

Review target: branch `codex/ec0-envelope-semantics`, commit `b16be81`.
Current disposition: old corpus rejected as canonical; history retained.

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

The corrections do not open EC1. The new corpus is a correction candidate until
the validator/CI is green and the user explicitly accepts the updated verbal
semantics. There are no current `BLOCKED_PENDING_USER_DECISION` cases; case 16
remains user-selected policy B for wings of one DecalRequest in one Patch.

AM11 re-closes the EC0 semantic gate for angular cases until the canonical JSON
corpus is migrated from join enums to explicit angular profiles. Existing
validator success does not override this semantic gate.

## Operational note

The branch diverged from `main` during the EC0 work. No rebase or merge is
performed as part of this semantic rebaseline; reconcile with current main only
after preserving the validated EC0/AM11 workspace and before PR/merge.
