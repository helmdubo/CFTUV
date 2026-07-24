# CFTUV Envelope Decal Engine — global canon for external agents

Version: `1.1.1`\
Prepared: `2026-07-24`  
Repository: `helmdubo/CFTUV`  
Reviewed immutable baseline: `df587ed166cfb0e0b615148f08c583b4477c5ac4`  
M-R1 implementation/CI SHA: `e38d1406b591d1189bf98bb850c8cab5d233f1c8`

## 1. What is being built

The new decal engine is a Blender-free geometric kernel plus thin Blender adapters.

```text
sparse source skeleton
→ analytic EnvelopeSpecs
→ EnvelopeInstances(alpha)
→ boundary resolution inside one PatchDomain
→ exact/labeled RawCoverage
→ approved interactions
→ ResolvedCoverage
→ dimension-aware ownership / station / UV
→ semantic coalescing
→ downstream tessellation
→ one GeometryBatch
→ read-only GPU and transactional BMesh adapters
```

The old `cftuv/decal_voronoi.py` and legacy geometry in `cftuv/decals.py` are not the implementation foundation.

## 2. Non-negotiable semantic rules

1. **Envelope plus coverage union is the only silhouette authority.**
2. Ownership, UV, station, provenance, tessellation, GPU and BMesh may not create, delete, move or repair silhouette geometry.
3. `PatchDomain` is the propagation field. `PhysicalChain` is physical identity. `ChainUse` is one directed patch-side use.
4. One geometric coordinate may have several topological boundary occurrences and several future mesh vertex keys.
5. Point-only contact does not merge positive-area components.
6. Provenance flows forward through construction records. It is never reconstructed by nearest-source lookup.
7. IDs are not selected from rounded coordinates, epsilon welding, candidate order or smallest ID.
8. Ambiguity produces a named unsupported outcome. No first-wins and no silent legacy fallback.
9. The full exact reference evaluator remains a permanent differential oracle.
10. Initial reflex fan subdivision and later kinetic split events are different mechanisms.
11. Cross-Patch propagation collision is forbidden. Cross-Patch lift/topology coordination is a later explicit stage.
12. Public contracts must permit planar, developable and general-surface evaluator strategies without changing user-facing Envelope semantics.

## 3. Current diagnosis

The semantic architecture is strong; the computational architecture is still reference-grade.

The selected code baseline `c2622d0...` already contains the consolidated
occurrence-aware C-R2C gate:

- one exact geometric `RawCoverageVertexV1` may have multiple
  `BoundaryVertexOccurrenceV1` records;
- `PointContactRecordV1` preserves lower-dimensional contact without joining
  positive-area components;
- exact circular ordering and a covered-left successor map replace
  one-outgoing-edge traversal;
- containment uses exact witnesses/winding rather than boundary-vertex probes;
- `building.002`, selected edges 2/3/7, reaches RawCoverage V2 without
  `REFERENCE_ARRANGEMENT_NON_MANIFOLD`.

The active correctness carryover is
`MULTIWAY_INTERACTION_POLICY_UNPROVEN` in the interaction stage. The largest
structural performance debt remains repeated arrangement construction:
Envelopes are clipped separately, public arrangement DTOs are converted back
to internal `PlanarRegion` values, and a final union is rebuilt. Static
identity is also still mixed with alpha-specific state in
`FrontComponentV1`.

## 4. Reflex profile canon

The accepted profile family is `LINEAR_REFLEX_EQUAL_V1`.

For reflex interior angle `phi`:

```text
Delta = phi - pi
k = max(0, ceil(Delta / Delta_max) - 1)
```

Hidden supports are local to `AngularEnvelope`; they are not `PhysicalChain`, `ChainUse`, owner, rail or global skeleton entities.

Default `Delta_max = pi/3 = 60°` remains unchanged unless the request explicitly chooses another accepted quality preset. Semantic reflex quality is separate from render/tessellation density and must not change with camera distance or ordinary alpha drag.

## 5. Evaluator separation

```text
ReferenceEvaluator
    full exact rebuild
    simple and auditable
    permanent oracle

RuntimeEvaluator
    cached static program
    incremental alpha state
    filtered predicates
    exact fallback
    same semantic result
```

The project must first obtain a correct end-to-end reference `GeometryBatch`. A production runtime does not block reference ownership and tessellation, but the data model required by them must be corrected first.

## 6. Baseline warning

The historical reviewed commit remains addressable, but later work must resolve
the candidate/accepted integration SHA and active cards from
`docs/architecture_status.json`. The canonical ref is
`codex/base-00-canonical-integration`. Never onboard from the old reviewed
branch name or from default `main` alone.

## 7. Common forbidden paths for kernel agents

Unless the card explicitly assigns the Legacy Evidence Curator or Host Adapter role, do not read:

```text
cftuv/decal_voronoi.py
legacy geometry sections of cftuv/decals.py
archived decal generators
unaccepted scratch notes from prior agents
```

Kernel agents may read accepted docs, contracts, fixtures, handoffs and the files allowlisted by their task card.

## 8. Common definition of done

A slice is not accepted merely because tests are green. It must leave:

- immutable base SHA and implementation SHA;
- changed path list;
- public contract/schema changes;
- exact tests and CI results;
- semantic digest comparison;
- named unsupported outcomes;
- assumptions not proven;
- performance counters where relevant;
- statement of whether forbidden legacy paths were read;
- next-agent allowlist.
