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

## Operational note

At this correction snapshot the branch is still 2 commits ahead and 3 commits
behind the locally known `origin/main`, matching the review warning. No rebase
or merge was performed as part of semantic correction; reconcile with current
main immediately before PR/merge, after preserving this validated workspace.
