# EC0 case sidecar schema

The YAML files are normative semantic descriptions, not geometry fixtures.
They intentionally contain no points, vectors, coordinates, tolerances, or
tessellation. Geometry in EC0a is reviewed in the paired visual sheet.

## v1 evidence schema

Required top-level fields in the existing v1 corpus:

- `schema`, `case_id`, `slug`, `title`, `status`;
- `semantic_contracts`: each transferable rule with a `SemanticAuthority`;
- `skeleton`, `envelopes`, `region_graph`, `boundary_lineage`, `owners`,
  `adjacency`, `s_direction`, `alpha_evolution`, `topology_events`;
- `forbidden`: named forbidden outcomes for this case;
- `metamorphic_expectations`: AM3 transformation verdicts;
- `canonical_digest_projection`: the semantic fields included in
  `CanonicalGeometryDigest`;
- `visuals`: relative paths to the review sheets.

Allowed `status` values are `DEFINED`, `UNSUPPORTED_NAMED_FAILURE`, and
`BLOCKED_PENDING_USER_DECISION`.

Allowed `SemanticAuthority` values are `USER_REQUIRED`, `FIELD_PROVEN`,
`MATHEMATICALLY_REQUIRED`, `LEGACY_COMPATIBILITY`,
`IMPLEMENTATION_ACCIDENT`, and `OPEN_RESEARCH`. The last three never make a
rule canonical by themselves; they belong in the review list.

Metamorphic verdicts are `INVARIANT` or `SEMANTIC_CHANGE`. `INVARIANT` means
that the semantic graph and digest are equal after applying the declared
normalization/inverse transform; it does not mean that world coordinates stay
numerically unchanged under scale, translation, or a permitted perturbation.

## v2 pivot schema — required before EC1

EC0-P must migrate every case to `cftuv.envelope.ec0.case.v2`. In addition to
the v1 semantic result, every v2 sidecar requires these top-level records:

- `patch_domains`: domain identity, surface regime, boundary-loop references,
  holes, barriers and sectors, without coordinates;
- `physical_chains`: physical source identity and source-vertex/edge lineage;
- `chain_uses`: directed patch-side uses with physical-chain reference,
  owner Patch, boundary loop, orientation, side and role set;
- `seeds`: `FrontSeed`, `CornerSeed`, `JunctionSeed` and `CapSeed` records with
  source relation and owner Patch;
- `front_components`: one record per `(ChainUse, owner Patch, sector)` with
  active-interval topology, branch count and lifecycle state;
- `evaluation_groups`: exactly one group per active owner Patch containing all
  sources evaluated together;
- `coverage_contributions`: source/seed provenance plus boundary-resolution
  state before patch union;
- `boundary_events`: exact contact keys, boundary provenance, atomic batches
  and active-interval transitions;
- `capacity_states`: requested/effective alpha, `CapacityReason` and terminal
  outcome per FrontComponent;
- `patch_coverage`: one domain-clipped single-cover union per evaluation group;
- `interactions`: declared front encounters, event gates and resolved-coverage
  effects before ownership.

v2 invariants:

- no pChain, ChainUse, seed or envelope owns a private domain;
- a physical seam between two Patch records has two ChainUses;
- `SEAM_SELF` preserves two distinct ChainUses in one PatchDomain;
- every seed/envelope resolves through ChainUse to PhysicalChain and Patch;
- all active sources of one Patch appear in one evaluation group;
- outer boundary, holes and explicit `BARRIER` roles are non-owner constraints;
- branch count of a v1 FrontComponent never increases;
- `BARRIER_SPLIT_REQUIRED` clamps only the affected FrontComponent;
- completion is based on active intervals, never first/last runtime vertex;
- no region behind a hole/barrier exists without proven source reachability;
- ownership partitions resolved patch coverage and does not create matter;
- ordering of physical chains, uses, seeds and contributions is metamorphically
  invariant;
- no coordinate, tolerance, tessellation or runtime face count enters v2.

Existing v1 files remain decision evidence; they are not silently interpreted
as v2. Migration must be explicit and followed by user acceptance of the
corresponding EC0-P visual sheet.
