# EC2.5 exact resolved-coverage interaction resolver

## Status and boundary

Session D implements the Blender-free full-recompute stage:

```text
ReferenceEnvelopeCompilationV1
+ BoundaryResolvedEnvelopeV1[]
+ RawCoverageResultV1
-> ResolvedCoverageResultV1
```

The stage consumes accepted Session C-R1 topology, exact planar envelope
instances, boundary-resolved contributions, and raw single-cover coverage. It
does not infer sources from emitted polygons.

The implementation contains no ownership assignment, station/UV flow,
tessellation, GeometryBatch production, event scheduler, Blender integration,
native runtime, Voronoi/SDF/raster construction, floating tolerance, or
approximate fallback.

Session D-R1 hardening adds nominal interaction IDs, explicit front-reading
and self-contact-pair compile declarations, fail-fast Cap reading selection,
equivalent-locus provenance aggregation, and candidate-scoped unsupported
Junction diagnostics.

Session D-R1 is accepted. The current gates are:

- `current_gate = ACCEPTED`;
- `session_v0 = OPEN` for the separate static Blender envelope debug bridge;
- `session_e = NOT_OPEN`.

This acceptance does not authorize ownership/UV work or production mesh decal
materialization.

## Compile declarations for readings and self-contact

Every exact Strip source support that can be evaluated is compiled into one
`FrontReadingDeclarationV1`. A declaration binds the reading to:

- one concrete `FrontComponentId`;
- one `SourceSupportId` and physical edge interval;
- one `ChainUseId` and `OwnerSectorId`;
- one immutable arrival-law identity.

The compiled `FrontComponentV1.front_reading_ids` must equal the declarations
for that component. No resolver path invents reading IDs from an
`InteractionComponent` or from emitted polygons.

Self-contact is admitted only through `SelfContactPairDeclarationV1`.
`declare_reference_self_contacts()` validates an explicit pair against the
compiled reading records and requires an identical non-empty source lineage.
Candidate generation consumes exactly that pair, suppresses the ordinary
distinct-component interpretation for the declared pair, and never expands
the declaration to every arrival model in an interaction component.

The V0 Blender adapter must not infer self-contact pairs from proximity or from
the presence of multiple readings. A real asset either exports no self-contact
pair or uses an explicitly selected debug-fixture pair.

## Interaction components

`InteractionComponentV1` is compiled from declared topology only:

- shared `FrontComponent` incidence;
- a Strip and its declared terminal interfaces;
- a Cap and its incident Strip;
- Angular or Junction incidence declared by their specs.

Spatial overlap is not a component relation. Inside one component,
Strip/Angular/Junction/Cap envelopes remain the exact union already produced
by Session C.

Candidate competition is admitted only for:

1. distinct components with the same `DecalRequestId` and `PatchDomainId`; or
2. an explicit self-contact with two distinct front-reading identities.

Different requests and different PatchDomains never form collision
candidates. Cross-Patch junction projections therefore share topology
provenance but do not collide.

## Exact front-arrival models

For an oriented support with inward normal `n`, source constant `c`, and
positive normal speed `v`, an arrival reading is:

```text
T(x) = (n dot x - c) / v
```

All coefficients and predicates use SymPy exact algebraic expressions.
`effective_alpha` and the Session C reachability certificate limit the active
reading. Boundary-resolved exact regions and active physical segments limit
where the reading is valid; their emitted polygon boundary is not used to
reconstruct `T`.

- Strip uses its source launch support.
- Cap inherits the incident Strip reading and restricts contact to the Cap's
  physical active segment, so endpoint-only equality remains an exact event
  anchor. The incident support must have exactly one matching Strip reading;
  absence returns `INTERACTION_MISSING_FRONT_READING`, and more than one exact
  match returns `INTERACTION_CAP_READING_AMBIGUOUS`. There is no first-item
  fallback.
- An exposed Angular profile is piecewise:

  ```text
  T_profile(x) = max(T_0(x), ..., T_k(x))
  ```

  Every selected support law is retained in the arrival model. A profile
  support participates only where it is dominant on the exposed component
  boundary.
- Junction geometry has no invented v1 arrival law. The unsupported model is
  inert unless a generated candidate actually requires that reading. Such a
  candidate produces `INTERACTION_JUNCTION_ARRIVAL_UNPROVEN`; an unrelated
  Junction in the same PatchDomain is not a stage-global failure.

## Mutual-arrival certificate

Overlap by itself is not an event. A certificate exists only when both
readings reach the same exact active-domain locus at the same non-negative
alpha and both are reachable at their proven effective alpha.

The equality equation is solved directly from the two arrival laws:

```text
T_left(x) = T_right(x)
```

The resulting line is clipped exactly against both active domains, their
holes, Angular dominance half-planes, and Cap active segments. The first exact
arrival alpha, immutable front readings, active-domain certificates,
Session C reachability certificates, and a same-alpha batch identity are
recorded in `MutualArrivalCertificateV1`.

Proofs on one canonical equality line are never selected by
`arrival_model_id`. Endpoint Strip readings that are exactly the same exposed
Angular support law are aggregated into one certificate with all readings,
arrival laws, active-domain certificates, reachability, and locus provenance.
If that semantic equivalence cannot be proved, resolution fails closed with
`INTERACTION_EQUIVALENT_LOCUS_PROVENANCE_UNPROVEN`.

Before the first mutual arrival no interaction application or equality locus
is emitted. At and after it, the certificate stays frozen at that first exact
locus.

Independent same-alpha pairs are allowed in one atomic batch. A same-alpha
meeting in which one component has degree greater than one has no
order-independent v1 rule and fails closed with
`MULTIWAY_INTERACTION_POLICY_UNPROVEN`. Coincident arrival laws fail closed
with `INTERACTION_COINCIDENT_ARRIVAL_LAWS_UNPROVEN`; no first-wins order is
introduced.

## INTRAPATCH_POLICY_B_V1

Policy B partitions only matter that already exists in
`BoundaryResolvedEnvelopeV1`:

- the side with the earlier exact reading retains the corresponding region;
- the equality locus is ownerless (`EqualityLocusOwner.NONE`);
- the opposing retained interiors are disjoint;
- their exact union equals the pre-interaction participant matter;
- the exact global union equals `RawCoverageResultV1`;
- no application sets `creates_new_matter`.

The implementation clips convex cells directly by exact half-planes and uses
the existing exact arrangement machinery for final canonical union records.
It does not derive arrival from those records.

Explicit self-contact applies the same partition rule while preserving the
two front-reading identities. A boundary contact may restrict one reading
without stopping unrelated sources or the opposite use.

## C13 exposed K-profile rule

The C13 third Strip is a distinct component. It is not inserted into the
Angular seed or `incident_front_component_ids`. Competition is evaluated
between that Strip's exact law and the actual exposed Angular
`max(T_0, ..., T_k)` boundary.

The executable corpus covers K0, K1, and K2 profiles. It proves an exact first
arrival at alpha 5 and verifies exact raw-area preservation after the event.

## Result and digest

`ResolvedCoverageResultV1` contains:

- the immutable raw-coverage semantic digest;
- resolved per-envelope contributions with retained/removed exact regions;
- canonical exact global vertices, edges, loops, and regions;
- interaction applications;
- mutual-arrival certificates and ownerless equality loci;
- named diagnostics;
- a distinct canonical semantic digest.

The raw digest is a hard input gate. Supplied
`BoundaryResolvedEnvelopeV1` values must exactly match the immutable set
carried by `RawCoverageResultV1`.

All Session D identities are nominal public value types:
`InteractionComponentId`, `ArrivalModelId`, `FrontReadingId`,
`InteractionCandidateId`, `MutualArrivalCertificateId`, `EqualityLocusId`,
`InteractionApplicationId`, `ResolvedContributionId`, and
`SameAlphaInteractionBatchId`. They remain distinct from
`EnvelopeSpecId`, `EnvelopeInstanceId`, and `FrontComponentId` even when their
text payloads are equal.

## Verification

`kernel/fixtures/session_d_interactions_v1` is the single declarative Session D
corpus and contains exactly 23 executable cases. It covers:

- exact before/at/after opposing fronts and unequal rational arrival;
- explicit self-contact;
- request/domain isolation;
- C13 exposed K0/K1/K2 profiles;
- saturated one-sided arrival;
- equality-locus termination at the outer boundary and a hole;
- independent simultaneous contacts and named multiway failure;
- Cap endpoint contact;
- internalized Angular supports;
- coincident laws;
- reversed storage order;
- rigid translation and uniform scale;
- cross-Patch junction projections.

Additional D-R1 cases are focused Python regression tests; they are not counted
as canonical Session D JSON corpus cases.

The repository workflow builds a wheel, tests it outside the checkout, copies
the kernel into an empty repository, rebuilds and retests it there, and runs
the forbidden-import scan.
