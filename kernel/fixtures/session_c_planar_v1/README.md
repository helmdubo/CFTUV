# Session C coordinate-bearing planar corpus v1

This corpus is independent from, and does not modify, the accepted Session A
v5 coordinate-free corpus. Each declaration names an executable
`AnalysisSnapshotV1`/`DecalRequestV1` registry builder, records the
authoritative planar coordinates or transform, and states the permanent
RawCoverage assertions.

`tests/test_session_c_fixture_corpus.py` parametrizes directly over every JSON
record and dispatches through `tests/session_c_case_registry.py`. All 23
records therefore compile and evaluate their declared fixture; the test fails
if a builder or expected observation is missing. The compact declarations are
not a second host schema and are not shipped as runtime input. No coordinate
is invented for Session A fixtures.

Every declaration records profile K (or `null`), expected RawCoverage
topology, silhouette lineage obligations, hole count, contributor cardinality,
and named outcomes.  `fixture_hash_manifest.json` locks the declarations with
the same UTF-8/LF SHA-256 law used by the accepted corpus.
