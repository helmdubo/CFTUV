# M-R1 Rational-Affine Reference Metric and Filtered Runtime — handoff

Session: `SESSION_M_R1_RATIONAL_AFFINE_REFERENCE_AND_FILTERED_RUNTIME`

Branch: `codex/m-r1-runtime-metric-exact-fallback`

Accepted base: `51996a4a574a290042014c14ba29d1a034a605db`

Implementation SHA:
`e38d1406b591d1189bf98bb850c8cab5d233f1c8`

CI-tested SHA:
`e38d1406b591d1189bf98bb850c8cab5d233f1c8`

Status: `M_R1_READY_FOR_REVIEW`

`V0_R2_CLOSED` remains in force. This session did not start practical
production runtime work.

## Implemented metric contracts

The Blender-free package is now `cftuv-envelope-core 0.6.0`.

`RationalAffinePlanarMetricV2` contains:

- deterministic exact origin and non-normalized basis A/B selected from stable
  `SourceVertexId` order;
- exact Gram and inverse-Gram matrices;
- exact rational source coordinates and reconstruction certificate;
- owner-Patch chart orientation;
- `EXACT_SOURCE_PLANE_V1` planarity certificate;
- source revision, PatchDomain identity and source lineage.

Blender binary64 source coordinates are interpreted as their exact IEEE-754
rational values. No plane fit, residual budget, tolerance or snapping is
used. An exact non-coplanar source receives:

```text
RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED
```

`RuntimePlanarMetricV1` is a derived binary64 view referencing one
authoritative `RationalAffinePlanarMetricV2`. Its predicate API returns only:

```text
CERTIFIED_NEGATIVE
CERTIFIED_ZERO
CERTIFIED_POSITIVE
EXACT_FALLBACK_REQUIRED
```

The outward interval fast path certifies only a strict positive or negative
sign. An uncertain result falls back to exact source coordinates owned by the
referenced V2 metric, where an exact zero can be certified. The runtime path
never constructs exact facts by
rationalizing a rounded runtime frame. Semantic vertices, edges, events,
provenance and digests are emitted only by exact construction records.

The exact evaluator now requests Euclidean operations through the Gram metric:

- `dot_G`;
- `length_G`;
- `unit_G`;
- `owner_normal_G`;
- `distance_to_support_G`;
- `offset_support_G`;
- `angle_G`.

Arrangement topology and exact narrowphase remain unchanged.

## Differential gate

Runtime output equals the authoritative reference output as complete immutable
records, covering vertices, atomic edges, loops, regions, provenance, event
identity, failure identity and semantic digest.

The blocking corpus covers:

- axis-aligned plane;
- arbitrary rotated exact plane;
- 45-degree rotation;
- irrational binary64 unit-normal representation;
- micro scale;
- huge coordinates;
- same plane after rigid rotation;
- source-surface retriangulation;
- near-planar source outside the admitted policy.

The two M-R0 rotation divergences are closed. Dedicated metamorphic tests prove
full exact RawCoverage equality after rigid rotation and unchanged RawCoverage
digest after retriangulation.

Exact fallback evidence is explicit: a two-predicate fixture certifies one
orientation on the binary64 fast path and resolves one exact-zero case through
the V2 authority (`fallback_fraction = 1/2`).

## building.002 field gate

Source: `E:\testscene.blend`, object `building.002`.

Selected physical edge 12 evaluates both domains through RawCoverage:

| Patch | Result |
|---:|---|
| 0 | `RAW_READY / EXACT` |
| 3 | `RAW_READY / EXACT` |

Patch 3 therefore no longer depends on a rational orthonormal admission frame.

All required L0 scopes record every selected PatchDomain, compile/Raw outcome
and timing:

| Scope | Selected edges | Recorded domains | Stage time |
|---|---:|---:|---:|
| `one_chain_l0` | 1 | 2 | 4.708 s |
| `three_chains_l0` | 3 | 4 | 12.497 s baseline / 9.643 s precise rerun |
| `ten_chains_l0` | 10 | 11 | 225.533 s |
| `all_seam_chains_l0` | 36 | 13 | 870.226 s |

No selected domain is silently omitted.

The original full-scope capture predates the final exception refinement and
therefore preserves the previous generic Patch 0 label. A subsequent
three-chain diagnostic proves the exact failure:

```text
RAW_REJECTED
REFERENCE_ARRANGEMENT_NON_MANIFOLD
exact union boundary is non-manifold at a construction vertex
```

This is a proven exact union topology outcome, not an undecidable sign
predicate. A permanent regression test maps this exact failure class to
`REFERENCE_ARRANGEMENT_NON_MANIFOLD`; Patch 0 no longer returns
`REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE` from this path. No float result was
used to bypass it.

Source mesh fingerprints before and after the complete field run are equal:

```text
3205834b7ead22cf1eb8985902144ca1c7295a88b8015556b364be17d8ac9021
```

Evidence:

- `artifacts/envelope_runtime_r1/building_002.json`;
- `artifacts/envelope_runtime_r1/patch0_diagnostic.json`;
- `artifacts/envelope_runtime_r1/differential_results.json`.

## Validation

Local:

- metric-focused contracts/differential tests: `17 passed`;
- host adapter and renderer: `26 passed`;
- wheel outside checkout: `205 passed`;
- extracted kernel: `205 passed`;
- generated schemas: OK;
- forbidden import scan: OK;
- Session A fixture projection: OK;
- L0 evidence validator: `4 cases, 36 physical chains, 60 chain uses`.

GitHub Actions run:
`https://github.com/helmdubo/CFTUV/actions/runs/30053100289`

| Job | Result |
|---|---|
| `kernel-hermetic` | success, `205 passed` |
| `kernel-extraction-readiness` | success, `205 passed`, import/fixture checks OK |
| `blender-free-host-bridge-v0b` | success, `26 passed`, legacy-import scan OK |

The CI run tested implementation SHA
`e38d1406b591d1189bf98bb850c8cab5d233f1c8`.

## Preserved boundaries and carryovers

- Envelope laws, Policy B interactions, ownership, tessellation and
  GeometryBatch semantics were not changed.
- Legacy geometry was neither read nor invoked; only accepted L0 factual
  evidence was used.
- Metric contracts and algorithms live in the Blender-free kernel. The host
  request exporter only delegates metric construction and maps host facts.
- `CompiledEnvelopeCache` remains disabled because current compilation carries
  requested alpha. Alpha-independent topology/profile compilation remains a
  V0-R2 debt.
- `envelope_request_export.py` remains large. M-R1 did not move new metric
  algorithms into it.
- The filtered runtime is a correctness implementation, not a performance SLA;
  exact fallback frequency may be high.
- Near-planar projection remains a named future product-policy decision.
- Patch 0's exact non-manifold union remains an honest unsupported Raw outcome;
  it is no longer mislabeled as a certified-predicate ambiguity.
