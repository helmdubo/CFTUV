# CFTUV Envelope Decal Engine — global canon for external agents

Version: `1.0`  
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

Current exact arrangement:

- merges all equal coordinates into one `RawCoverageVertexV1`;
- requires one outgoing boundary edge per merged vertex;
- uses boundary vertices as containment probes;
- clips each Envelope separately and then unions all clipped results again;
- converts public arrangement DTOs back into internal `PlanarRegion` values;
- compiles `requested_alpha` and `effective_alpha` into `FrontComponentV1`.

The immediate correctness blocker is the false non-manifold failure at point contacts. The largest structural performance debt is repeated arrangement construction.

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

The reviewed commit remains addressable, but the original branch ref was not returned by GitHub during plan preparation. The reviewed history is also diverged from `main`. Never tell an agent only “start from the old branch”. The first gate creates a canonical integration ref and records its immutable SHA.

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
