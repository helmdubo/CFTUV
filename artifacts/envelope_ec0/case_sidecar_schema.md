# EC0 canonical case schema

Normative machine schema:
`artifacts/envelope_ec0/corpus/schema/case.schema.json`.

JSON cases are semantic graphs, not sampled geometry fixtures. Coordinates,
vectors, tolerances, tessellation, face count, presentation visuals, and
runtime mesh order are forbidden.

## Required top-level records

- `schema`, `case_id`, `slug`, `title`, `status`;
- `semantic_contracts`, each tagged with `SemanticAuthority`;
- `analysis_snapshot` — facts and relations only;
- `decal_request` — request id, selected uses, alpha/width and policies;
- `expected_compiled_plan` — kernel-created seeds, components, contributions,
  events, coverage, ownership, effective alpha/capacity and GeometryBatch
  provenance;
- `acceptance` — human-readable assertions derived from the same graph;
- `canonical_digest_projection` — fields included in semantic digest.

Allowed status values: `DEFINED`, `UNSUPPORTED_NAMED_FAILURE`, and
`BLOCKED_PENDING_USER_DECISION`.

Allowed authority values: `USER_REQUIRED`, `FIELD_PROVEN`,
`MATHEMATICALLY_REQUIRED`, `LEGACY_COMPATIBILITY`, `IMPLEMENTATION_ACCIDENT`,
and `OPEN_RESEARCH`. The last three belong to explicit review lists and cannot
become canonical by presence alone.

## AnalysisSnapshotV1

Contains PatchDomains, PhysicalChains, directed ChainUses, BoundaryLoops,
analysis-proven sectors, holes/barriers, SourceVertices, CornerRelations,
JunctionRelations, and lineage. It must not contain seeds, fronts, runtime
events, alpha, capacity, envelopes, or material state.

An ordinary ChainUse has one owner-interior sector. Extra sectors require
analysis provenance; automatic abstract left/right sectors are invalid.

## DecalRequestV1

Contains `decal_request_id`, selected ChainUse ids, requested alpha/width, and
explicit policies. Different request ids never collide or share a plan merely
because they reference the same Patch.

## CompiledPatchEvaluationPlan

Key: `(DecalRequestId, PatchDomainId)`. Contains all active sources of that
request/domain together. Every component, contribution, interaction,
PatchCoverage, ownership claim, digest record, and GeometryBatch provenance
must preserve both ids.

Required semantic invariants include:

- one ordinary ChainUse -> one owner-interior FrontComponent;
- a physical seam A/B -> two ChainUses in two domains;
- `SEAM_SELF` -> two distinct ChainUses in one domain;
- `SOURCE_LAUNCH_BOUNDARY` does not block its own FrontSeed;
- endpoint boundary contact may slide/shrink;
- interior contact needing branch birth yields `BARRIER_SPLIT_REQUIRED` at the
  same exact alpha;
- CornerSeed/JunctionSeed references a confirmed relation;
- EndpointClaimSeed is used for endpoint claims without a CornerRelation;
- C13 foreign wing use is interaction-only, never incident to BEVEL seed;
- cross-Patch junctions share relation/anchor but never collide;
- mixed-alpha shared envelopes use incident effective-alpha vectors or return
  `SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN`;
- ownership partitions resolved coverage and never creates matter;
- case 16 B applies only to same-request, same-Patch wings.

Metamorphic verdicts live in `corpus/matrices/` and are `INVARIANT`,
`SEMANTIC_CHANGE`, or preconditioned `CONDITIONAL`.

Validation is executable:

```text
python tools/validate_envelope_ec0.py
```
