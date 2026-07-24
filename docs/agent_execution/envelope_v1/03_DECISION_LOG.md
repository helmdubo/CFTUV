# Final decision log after review feedback

## Accepted without change

- Preserve the Envelope semantic model.
- C-R2C occurrence-aware topology was the immediate blocker at plan-drafting
  time.
- Repeated RawCoverage arrangements are a major structural cost.
- Compile-static data and alpha state must be separated.
- Ownership must be dimension-aware.
- Reference and runtime evaluators remain separate.
- Initial reflex fan is not a kinetic split event.
- Straight skeleton of the Patch boundary is not the same problem as source-driven decal growth.
- Curved evaluators share one semantic contract and are not the next implementation slice.

## Accepted with a roadmap correction

The earlier proposal placed a true filtered/event runtime before ownership and tessellation. The final order is:

```text
topology/data-model correction
→ labeled reference cell complexes
→ static/alpha contracts
→ dimension-aware ownership contract
→ reference ownership/UV/tessellation/GeometryBatch
→ optimized filtered/event runtime
```

The data model must be fixed before ownership. Full production optimization does not block the first correct end-to-end mesh.

## Additional refinements added to the feedback

1. A mandatory baseline card creates a canonical integration ref because the reviewed branch ref is not currently reliable.
2. Documentation authority is moved to the beginning so agents do not follow stale `pyvoronoi`/legacy guidance.
3. A dedicated `C-R2F` slice generalizes reflex profile quality; current code hardcodes the default 60° behavior.
4. Performance telemetry and benchmark cases are installed early, even though native optimization is deferred.
5. “One arrangement” is defined as a persistent/derived cell-complex pipeline, not one monolithic stage mixing coverage, interactions, ownership and tessellation.
6. A curved-evaluator contract audit occurs before curved implementation so planar contracts do not become an accidental dead end.

## BASE-00 baseline reconciliation

BASE-00 selected `c2622d07020338e5231b81f41655fe6c74cdca72`,
which already contains the consolidated occurrence-aware C-R2C implementation,
field verification, and host reflex-corner export. Therefore:

- `C-R2C-01` through `C-R2C-04` are superseded planning decompositions, not
  runnable cards;
- the point-contact false non-manifold failure is no longer the active
  correctness blocker;
- the accepted field result reaches RawCoverage and then exposes
  `MULTIWAY_INTERACTION_POLICY_UNPROVEN`;
- after `DOC-00` and `FIX-00`, the next contract gate is `D-R2-00`;
- any new C-R2C delta requires a new amendment/card and may not be inferred
  merely because the consolidated implementation used different internal type
  names than the historical decomposition.
