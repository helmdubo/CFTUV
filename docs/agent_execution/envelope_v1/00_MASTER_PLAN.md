# CFTUV Envelope Decal Engine — final execution plan

Version: `1.1.1`\
Date: `2026-07-24`  
Reviewed baseline: `df587ed166cfb0e0b615148f08c583b4477c5ac4`

Operation: keep this pack unpacked under `docs/agent_execution/envelope_v1/`; launch one fresh session per active card using `OWNER_OPERATING_GUIDE.md`.

## Executive verdict on the received feedback

The feedback is correct and materially improves the sequencing. It does not reject the earlier architectural review; it applies the most important correction:

> Fix the topology and computational data model before ownership, but do not wait for a complete production runtime before obtaining the first correct end-to-end `GeometryBatch`.

The final program therefore preserves the semantic design and changes the implementation order.

## Program objective

Deliver a decal engine that:

- handles reflex corners through a certified, configurable linear-reflex profile;
- grows/shrinks coverage inside fixed PatchDomains while preserving source ownership;
- produces one exact semantic result consumed by GPU and BMesh;
- supports interactive alpha drag for the target scale of roughly 2500–3000 seam edges and up to 50 patches;
- later supports developable and general curved surfaces through evaluator strategies sharing the same semantic contracts.

## Release gates

| Gate | Result required |
|---|---|
| B0 | Canonical integration SHA, reproducible CI, current authority docs |
| C-R2C | **Accepted in baseline `c2622d0...`**: point contacts and branch coordinates represented without false non-manifold failure |
| C-R2D | One labeled RawCoverage overlay and cell-complex DTO projection |
| D-R3 | ResolvedCoverage represented as a derived cell complex without polygon round-trip |
| C-R2E | StaticPatchProgram and EvaluationState(alpha) separated; alpha cache works |
| C-R2F | Configurable certified reflex quality; semantic and render density separated |
| E0 | Dimension-aware ownership contract accepted |
| E1/E2 | Reference ownership, station and UV completed without silhouette change |
| F | Semantic coalescing, tessellation and GeometryBatch completed |
| H/L | Read-only host adapters and explicit cross-Patch lift |
| R | Incremental filtered runtime meets agreed drag SLA and matches reference |
| S | Developable/general-surface evaluator accepted under explicit error contracts |

## Dependency graph

```text
BASE-00
 ├─ DOC-00
 └─ FIX-00
       ↓
C-R2C consolidated gate [accepted in c2622d0]
       ↓ observed field carryover
D-R2-00
       ↓
C-R2D-01 → C-R2D-02 → C-R2D-03 → D-R3-01
       ↓
C-R2E-01 → C-R2E-02 → C-R2E-03
       ↓
C-R2F-01
       ↓
E0-01 → E1-01 → E2-01
       ↓
F1-01 → F2-01 → H0-01
                    └→ L0-01
       ↓
R0-01 → R1-01 → R2-01 → [R3-01 if profiling proves need] → R4-01
       ↓
S0-01 → S1-01
      └→ S2-01 → N0-01 → S3-01
```

## Immediate queue

Only these cards are in the current acceptance sequence:

1. `BASE-00` — complete its human review.
2. `DOC-00` — authority/onboarding correction, parallel after accepted baseline.
3. `FIX-00` — portable `building.002` reproduction, parallel after accepted baseline.
4. `D-R2-00` — after both baseline cards are accepted, resolve or formally
   preserve the observed atomic multiway interaction outcome.

`C-R2C-01` through `C-R2C-04` are superseded by the consolidated accepted gate
at `c2622d0...`; do not execute them. Do not start ownership, runtime, native or
curved implementation in parallel with the active baseline/interaction gate.

## Architectural staging

### Stage 1 — Correct topological identity (accepted in baseline)

Replace:

```text
one exact coordinate
→ one boundary vertex
→ one outgoing edge
```

with:

```text
ArrangementPoint
→ multiple BoundaryOccurrences
→ directed halfedges
→ exact rotation system
→ face-sector successor
```

The selected baseline implements this stage through `RawCoverageResultV2`,
`RawCoverageVertexV1`, `BoundaryVertexOccurrenceV1`,
`PointContactRecordV1`, exact covered-left successor construction, and
occurrence-aware debug projection. Differences in internal naming from the
historical four-card decomposition do not reopen the gate.

### Stage 2 — Correct computational data flow

Replace:

```text
N per-Envelope clips
→ public DTO
→ internal PlanarRegion
→ final union
→ more clips/unions
```

with:

```text
PatchDomain boundaries
+ all Envelope boundaries
+ contributor/domain labels
→ one RawCoverage cell complex
→ derived ResolvedCoverage complex
→ derived Ownership complex
```

Public DTOs are projections at stage boundaries, not the internal compute representation.

### Stage 3 — Stable program versus alpha state

Static identities and laws are compiled once. Alpha creates an evaluation state. Ordinary drag does not recreate `EnvelopeSpec`, `FrontComponent`, source support, profile selection or domain topology.

### Stage 4 — Reference end-to-end semantics

Define dimension-aware ownership, station/UV and tessellation on the cell complex. Produce one `GeometryBatch`.

### Stage 5 — Runtime acceleration

Build a lazy exact event ledger, then filtered predicates and exact fallback, then native hot paths only after profiling.

### Stage 6 — Curved surfaces

First prove that public contracts are evaluator-neutral. Reuse planar semantics through unfolding for developable surfaces. For general surfaces, define a multi-source arrival complex with curve generators, multiple arrival candidates, vector transport and cut-locus events.

## Product decisions that remain explicit

The following must not be silently selected by an agent:

- maximum accepted reflex profile complexity;
- exact drag SLA: report both 16.7 ms and 33.3 ms budgets until product owner chooses;
- near-planar projection policy;
- noise-as-geometry versus smoothed intrinsic metric;
- general-surface approximation/error budget;
- unsupported multiway interaction policy;
- ambiguous cross-Patch lift.

## Program-wide performance corpus

The benchmark harness must cover:

```text
100 / 500 / 1000 / 3000 selected seam edges
1 / 10 / 50 PatchDomains
cold compile
first alpha query
ordinary between-event drag
event-crossing drag
backward drag
selection change
GPU upload
BMesh transaction
```

Report p50, p95 and p99; memory peak; exact fallback fraction; number of rebuilt patches; event count; cell/halfedge counts. Never hide a slow exact state behind approximate geometry.

## Completion definition

The planar program is production-ready only when:

- field fixtures are reproducible without a developer-local path;
- exact reference emits one correct `GeometryBatch`;
- GPU and BMesh consume the same batch;
- source mesh is unchanged on any failure;
- point contacts preserve separate occurrences;
- ownership is total on open 2-cells and interfaces are explicit;
- runtime equals reference before/at/after all event types;
- target-scale drag meets the chosen SLA;
- no legacy geometry fallback exists.
