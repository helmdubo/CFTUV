# Envelope runtime R0 — acceptance boundary

Status: `M_R1_OPEN`

Session: `SESSION_M_R1_RATIONAL_AFFINE_REFERENCE_AND_FILTERED_RUNTIME`

Branch: `codex/m-r1-runtime-metric-exact-fallback`

Accepted base: `51996a4a574a290042014c14ba29d1a034a605db`

## Accepted ancestry

The M-R1 slice starts after six accepted factual or implementation slices:

| Slice | Accepted SHA | Result carried into M-R1 |
|---|---|---|
| L0 Legacy Planarity and ChainUse Evidence | `3e3e10864b093d63a56336494dd752efe18b2d6a` | Factual evidence only; no legacy geometry authority. |
| V0-R1A Staged Topology Debug and Telemetry | `5fb22ea6a974b38dade2251489a9f75713a9ba6d` | Topology remains visible after later exact-stage failure. |
| V0-R1B Host Adapter Slimming and Cache | `9132b1df8303847a4f75bb68cc28dcb79721a9cb` | WindowManager-owned analysis/topology/metric/domain caches. |
| C-R2A Sparse PatchDomain Boundary | `fe091b4f21cf15cd1888eaa6bab47a697c0b596a` | Runtime domain is sparse semantic boundary plus explicit barriers. |
| C-R2B Deterministic Exact Arrangement Broadphase | `c7bfed158802b767f7e94f6672cfb6b0181dae16` | Exact sweep broadphase; exhaustive backend remains the differential oracle. |
| M-R0 Planar Metric Contract Decision | `5c4d91b8e1c9ed235f203c2859f031bb5b9804f5` | ReferenceMetricV2 and RuntimeMetricV1 architecture accepted. |

The five L0 evidence files were copied into the linear M-R0 ancestry by
`51996a4a574a290042014c14ba29d1a034a605db`. Their content is byte-identical
to accepted L0 SHA `3e3e10864b093d63a56336494dd752efe18b2d6a`.

Historical handoff files containing `implementation_sha: null` remain
unchanged. This document records reviewer acceptance without rewriting
historical receipts.

## Open slice

M-R1 implements two separate numeric authorities:

```text
ReferenceMetricV2
  exact rational affine chart
  exact Gram metric
  exact construction identities

RuntimeMetricV1
  derived binary64 candidate view
  certified predicates
  exact fallback to the same ReferenceMetricV2
```

Only `EXACT_SOURCE_PLANE_V1` is admitted. Near-planar projection remains a
named unsupported product-policy gap:

```text
RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED
```

M-R1 does not open production Blender runtime work:

```text
M_R1_OPEN
V0_R2_CLOSED
SESSION_E_CLOSED
```

## Blocking gates

M-R1 is not complete until all of the following are factual:

1. `RationalAffinePlanarMetricV2` and `RuntimePlanarMetricV1` public
   contracts, codecs, schemas and validators pass.
2. Deterministic frame selection is invariant to record permutation, loop
   cyclic shift, coherent reverse, retriangulation, rigid rotation and
   uniform scale.
3. The two M-R0 rotation divergences have exact semantic equality with the
   reference metric.
4. Float coordinates do not participate in semantic construction identities.
5. Uncertain runtime predicates fall back to the authoritative reference
   metric and never rationalize a rounded runtime frame.
6. `building.002` Patch 3 reaches RawCoverage under arbitrary exact planar
   orientation.
7. `building.002` Patch 0 no longer ends as
   `REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE`.
8. Field scopes for 1, 3, 10 and all seam chains report every selected
   PatchDomain with compile/Raw outcome and timings; no silent omission.
9. Full Session B/C/D/R2 source, wheel and extracted-kernel gates pass.
10. Blender source mesh is unchanged.

Any semantic divergence from the exact reference is a blocking failure. It is
not authorization to retune fixtures, add tolerance or snap topology.

## Accepted carryovers

### V0-R1B

`CompiledEnvelopeCache` remains disabled because current compilation carries
requested alpha. The future V0-R2 slice must make topology/profile compilation
alpha-independent. M-R1 does not enable this cache.

The host facade is small, but `cftuv/envelope_request_export.py` remains large.
M-R1 must not add numeric contracts or algorithms there; new metric logic
belongs to the Blender-free kernel.

### C-R2B

The exact broadphase is accepted. Patch 0
`REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE` is not a broadphase defect, but it
is a blocking M-R1 field gate.

### M-R0

The following research candidates are explicitly not product policies:

- `RUNTIME_PLANAR_RESIDUAL_BUDGET_CANDIDATE_V1`;
- `RUNTIME_FILTERED_PREDICATE_BOUND_CANDIDATE_V1`.

The first controls planarity admission and remains unaccepted. The second may
only become an implementation certificate after a conservative proof and
full exact differential equality.

## Scope boundary

M-R1 does not change Envelope laws, interaction policy B, ownership,
station/UV flow, tessellation, GeometryBatch, production decal mesh, modal
drag, GPU rendering, Grease Pencil or curved-surface behavior.

Legacy geometry source is prohibited. Only the accepted L0 factual evidence is
available to this slice.

