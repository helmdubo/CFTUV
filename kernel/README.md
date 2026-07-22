# CFTUV Envelope Core

Standalone, Blender-free public contracts for the CFTUV Envelope kernel.

This EC1 package contains immutable data records, structural validators,
canonical JSON codecs, and semantic digests. It deliberately contains no
geometry evaluator, Boolean/arrangement implementation, ownership solver,
event scheduler, triangulator, Blender adapter, or native extension.

The package has no runtime dependencies outside the Python standard library.

Public boundaries:

- `AnalysisSnapshotV1` — versioned host facts, `PatchSurfaceIRV1`, directed
  `ChainUseV1` records, owner sectors, angle certificates and relations;
- `DecalRequestV1` — one request with explicit local metric and the accepted
  Linear-Reflex/boundary/interaction/ownership policies;
- `CompiledPatchEvaluationPlanV1` — request/domain-scoped seeds, declarations,
  four-way `EnvelopeSpec` union, front/event records, distinct Raw/Resolved
  coverage references, ownership and downstream `TessellationPlanV1`;
- `GeometryBatchV1` — immutable adapter boundary keyed by semantic
  `VertexKey`, not coordinates.

Each top-level boundary has its own strict canonical JSON codec. JSON Schemas
under `schema/` are generated from the same public types. Snapshot, semantic
plan and GeometryBatch semantic digests are deliberately separate.

The accepted Session A v5 corpus is mirrored under `fixtures/session_a_v5/`
with a SHA-256 manifest. The projection adapter exists only in `tests/`; the
runtime package neither imports fixtures nor depends on the parent CFTUV repo.

Useful checks from this directory:

```text
python -m pytest -q
python tools/check_forbidden_imports.py src/cftuv_envelope
python tools/generate_contract_schemas.py --output schema --check
python tools/sync_session_a_fixtures.py --check --fixture-root fixtures/session_a_v5
```
