# Session handoff

Active slice:

`FIX-00 — Portable building.002 reproduction and fixture extraction`

Base integration SHA:

`e4db68371cab83a6a26368bf9a95eda74ae8d02e`

Implementation SHA:

`f870cba6b48b81d95ec390e7d46129c2550d1728`

CI-tested SHA:

`f870cba6b48b81d95ec390e7d46129c2550d1728`

Input contract/schema versions:

- `cftuv.envelope.analysis_snapshot.v1`
- `cftuv.envelope.decal_request.v1`
- `cftuv.envelope.host_mesh_fixture.v1`
- selected output: `cftuv.envelope.raw_coverage_result.v2`
- historical/selected kernel packages: `0.6.0` / `0.7.0`

Accepted predecessor gate:

BASE-00 is `BASE_00_ACCEPTED / ACCEPTED_BY_RELEASE_OWNER`. Canonical control
commit: `688c6cd6c9359dbc9e08db4cd399ae7d81c3b14c`; immutable content SHA:
`e4db68371cab83a6a26368bf9a95eda74ae8d02e`.

Changed paths:

- `artifacts/envelope_c_r2c_fixture/**`
- `kernel/fixtures/building_002_point_contact_v1/**`
- `kernel/tests/test_building_002_point_contact_fixture.py`
- `tools/export_building_002_point_contact_fixture.py`
- `tests/blender/generate_building_002_point_contact_fixture.py`
- `tests/blender/test_building_002_point_contact_fixture.py`
- this JSON/Markdown handoff pair under `artifacts/envelope_fix_00/`

Public API/schema changes:

None.

Algorithms implemented:

- deterministic canonical snapshot/request export for patch 0, physical edges
  `2, 3, 7`, alpha `0.25`;
- deterministic one-object Blender host-mesh generator using binary64 hex
  coordinates;
- read-only historical exception-frame capture for exact rejected construction
  points and provenance. Arrangement behavior is not patched.

Differential/oracle comparison:

The same immutable fixture was replayed at both required revisions. The two
historical non-manifold vertices and the two selected V2 point contacts have
identical exact rational coordinates and identical ChainUse contributor sets.
Both contacts remain point-only; no positive-area overlap was introduced.
Receipt:
`artifacts/envelope_c_r2c_fixture/point_contact_topology_comparison.json`.

Focused tests and exact result:

- FIX-00 plus accepted artifact tests: `5 passed in 1.81s`.
- Python compile check for exporter, generator, host test, and kernel test:
  `PASS`.
- repeated accepted-host export: all four fixture files byte-identical.

Full wheel/extraction tests and exact result:

- built `cftuv_envelope_core-0.7.0-py3-none-any.whl`, installed it into a
  clean venv outside the checkout, copied only tests/fixtures/evidence, then
  ran the full suite: `227 passed in 331.44s`.
- refreshed FIX-00 test/evidence in that external bundle: `3 passed in 1.87s`.
- source and installed-wheel forbidden-import scans:
  `forbidden-import-scan: OK`.
- generated schema check: `generated-contract-schemas: OK`.

One earlier diagnostic run launched the suite from the monorepo root and
therefore made the sibling `cftuv` package visible to the isolation test:
`226 passed, 1 failed`. The intended external-wheel run above is clean; the
isolation test also passed independently from `kernel/`: `3 passed in 0.19s`.

Host tests and exact result, if applicable:

- Blender `4.3.2` deterministic generator: `PASS` for 24 vertices, 38 edges,
  15 faces.
- accepted exporter `43e69d3889d273ed19daee9239ae0e311a1b213d`:
  source mesh unchanged; snapshot and request byte-identical.
- Blender may reverse storage order of undirected edge endpoints, so host mesh
  comparison normalizes endpoint orientation. SourceRevision already applies
  this normalization.

Field/portable fixture result:

- fixture hash:
  `efbae17746688b517ce40fa71efa85c2ff7b8ba59573ada93fb3758663a70515`;
- `df587ed166cfb0e0b615148f08c583b4477c5ac4`:
  `RAW_REJECTED / REFERENCE_ARRANGEMENT_NON_MANIFOLD`;
- `c2622d07020338e5231b81f41655fe6c74cdca72`:
  `RAW_READY / EXACT`, RawCoverage V2, 12 boundary occurrences, 2 point
  contacts, 3 loops, 3 regions.

Semantic digest result:

Selected RawCoverage digest:
`622e1f6eec09e64bc1294c37643af19f630086005f9af21f10ac4cd6ed0e987a`.
Exact area: `Rational(122766786560, 373821260323)`.

Performance counters:

Not a performance card. Telemetry does not affect semantics.

Named unsupported outcomes:

- expected historical reproduction:
  `REFERENCE_ARRANGEMENT_NON_MANIFOLD`;
- current host-export limitation outside FIX-00:
  `ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE`.

Assumptions not proven:

- The current `c2622d0` host exporter cannot serialize patch 0 from the
  developer-local scene; the accepted `43e69d3` exporter is the extraction
  authority.
- No claim is made that the 24-vertex host mesh can be further reduced without
  changing construction history.

Risks and special opinion:

The current host exact-angular rejection is a genuine inconsistency worth a
separate scoped card. FIX-00 deliberately does not modify host or kernel
implementation.

Forbidden legacy paths read:

- no

Source mesh mutation check:

Before and after:
`3205834b7ead22cf1eb8985902144ca1c7295a88b8015556b364be17d8ac9021`;
unchanged: `true`.

Next session allowlist:

Start a fresh session and resolve the exact write allowlist from
`docs/agent_execution/envelope_v1/cards/D-R2-00_Conditional_atomic_multiway_interaction_contract_gate.md`.
FIX-00 did not read or start that card.

Suggested next card:

`D-R2-00`

Rollback notes:

Revert implementation commit `f870cba6b48b81d95ec390e7d46129c2550d1728`
and the subsequent handoff commit. No kernel or host implementation behavior
was changed.
