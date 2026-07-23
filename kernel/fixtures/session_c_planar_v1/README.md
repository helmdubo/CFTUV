# Session C coordinate-bearing planar corpus v1

This corpus is independent from, and does not modify, the accepted Session A
v5 coordinate-free corpus.  Each declaration names an executable
`AnalysisSnapshotV1`/`DecalRequestV1` builder in
`tests/reference_factories.py`, records the authoritative planar coordinates
or transform, and states the permanent RawCoverage assertions exercised by the
reference test modules.

The compact declarations are expanded to public immutable contracts by tests;
they are not a second host schema and are not shipped as runtime input.  No
coordinate is invented for Session A fixtures.

Every declaration records profile K (or `null`), expected RawCoverage
topology, silhouette lineage obligations, hole count, contributor cardinality,
and named outcomes.  `fixture_hash_manifest.json` locks the declarations with
the same UTF-8/LF SHA-256 law used by the accepted corpus.
