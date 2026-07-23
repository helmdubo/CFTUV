# CFTUV Envelope Core

Standalone, Blender-free public contracts and exact planar reference evaluator
for the CFTUV Envelope kernel.

The EC1 layer contains immutable data records, structural validators,
canonical JSON codecs, and semantic digests. The EC2 `reference` package adds
a permanent full-recompute path from `AnalysisSnapshotV1 + DecalRequestV1` to
provenance-preserving `RawCoverageResultV1`. The EC2.5 `interactions` package
adds exact front-arrival models, mutual-arrival certificates, and
`INTRAPATCH_POLICY_B_V1` partitioning of existing contributions into
`ResolvedCoverageResultV1`. It deliberately contains no ownership solver,
event scheduler, tessellator, GeometryBatch materializer, Blender adapter, or
native extension.

SymPy 1.14.0 is pinned as the exact algebraic-number and certified-sign
dependency because canonical algebra serialization participates in stable
reference IDs and digests. Polygon
arrangement, segment history, clipping, union, and provenance propagation are
implemented by this package; SymPy is not used as a Boolean backend. There is
no float, raster, SDF, Marching Squares, GEOS, or approximate fallback.

Public boundaries:

- `AnalysisSnapshotV1` — versioned host facts, `PatchSurfaceIRV1`, directed
  `ChainUseV1` records, owner sectors, angle certificates and relations;
- `RationalAffinePlanarMetricV2` — deterministic exact source-plane chart with
  non-normalized basis vectors, exact Gram/inverse-Gram matrices and exact
  reconstruction of every source vertex;
- `RuntimePlanarMetricV1` — non-authoritative binary64 view whose uncertain
  predicates fall back to the referenced `RationalAffinePlanarMetricV2`;
  construction identity never comes from float coordinates;
- `DecalRequestV1` — one request with explicit local metric and the accepted
  Linear-Reflex/boundary/interaction/ownership policies;
- `CompiledPatchEvaluationPlanV1` — request/domain-scoped seeds, declarations,
  four-way `EnvelopeSpec` union, front/event records, distinct Raw/Resolved
  coverage references, ownership and downstream `TessellationPlanV1`;
- `GeometryBatchV1` — immutable adapter boundary keyed by semantic
  `VertexKey`, not coordinates.
- `ReferenceEnvelopeCompilationV1` — request/domain compilation containing
  Strip, angle-selected Linear-Reflex Angular, physical Cap, and minimally
  declared Junction envelopes; Junction geometry remains named-unproven;
- `RawCoverageResultV1` — exact single-cover patch union with construction
  certificates, reachability, contributor sets, and complete segment-history
  provenance.
- `ResolvedCoverageResultV1` — the same exact global matter after
  request/domain-scoped component competition, with mutual-arrival
  certificates, ownerless equality loci, retained contribution regions, and a
  distinct semantic digest.

Each top-level boundary has its own strict canonical JSON codec. JSON Schemas
under `schema/` are generated from the same public types. Snapshot, semantic
plan and GeometryBatch semantic digests are deliberately separate.

The accepted Session A v5 corpus is mirrored under `fixtures/session_a_v5/`
with a SHA-256 manifest. The projection adapter exists only in `tests/`; the
runtime package neither imports fixtures nor depends on the parent CFTUV repo.
The separate coordinate-bearing Session C corpus is under
`fixtures/session_c_planar_v1/`; it never supplies invented geometry to the
accepted coordinate-free EC0 corpus.
The Session D EC2.5 interaction corpus is under
`fixtures/session_d_interactions_v1/`; its 23 declarations execute exact
before/at/after, exposed Angular K-profile, boundary/hole, self-contact, and
named fail-closed interaction cases.

Useful checks from this directory:

```text
python -m pytest -q
python tools/check_forbidden_imports.py src/cftuv_envelope
python tools/generate_contract_schemas.py --output schema --check
python tools/sync_session_a_fixtures.py --check --fixture-root fixtures/session_a_v5
```
