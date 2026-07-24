# Session handoff

Active slice:

`HOST-ANG-00 — Exact angular certificate export for non-rational pi ratios`

Base integration SHA:

`361102a6539ffbbe6fa8957a04bf12a9bae42bd8`

Implementation and CI-tested SHA:

`b943ccb6cdaa0d970b3bbcd4dbeccecf75f93b7e`

Input contract/schema versions:

- `cftuv.envelope.analysis_snapshot.v1`
- `cftuv.envelope.decal_request.v1`
- `cftuv.envelope.raw_coverage_result.v2`
- kernel package `0.7.0`

Accepted predecessor gate:

FIX-00 implementation
`f870cba6b48b81d95ec390e7d46129c2550d1728`.

Changed paths:

- `cftuv/envelope_request_export.py`
- `tests/test_envelope_host_adapter.py`
- `docs/envelope_v0_debug_bridge.md`
- execution-pack v1.1.2 registration and `HOST-ANG-00` card
- `artifacts/envelope_host_angular_cert_00/**`

Public API/schema changes:

None.

Algorithms implemented:

- exact support directions remain angular authority;
- a high-precision value only seeds outward decimal candidates;
- exact cosine predicates prove each lower and upper bound;
- `phi/pi` is the exact translation of the certified reflex-excess interval;
- rational ratios retain the existing fast path;
- undecidable predicates retain
  `ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE`.

Differential/oracle comparison:

The FIX-00 request is byte-identical and every non-angular snapshot record is
equal. The complete snapshot differs because the current exact exporter emits
four proved Angular records while the older accepted exporter emitted none.
The old result had 3 regions and 2 point contacts; the current exact result
has 1 region and 0 point contacts. The complete evidence is in
`artifacts/envelope_host_angular_cert_00/field_receipt.json`.

Focused tests and exact result:

- host adapter: `25 passed in 10.87s`;
- full Envelope host/debug group: `33 passed in 11.54s`;
- Python compile: `PASS`;
- execution packet build: `PASS`;
- `git diff --check`: `PASS`.

Full wheel/extraction tests and exact result:

Kernel source and contracts were not modified, so the 230-test D-R2 external
wheel gate was not repeated. The current exported contracts were decoded,
validated and evaluated with the kernel package outside Blender:
`RAW_READY / EXACT`, with zero validation issues.

Host tests and exact result:

Blender 4.3.2 generated the 24-vertex/38-edge/15-face fixture. Two current
exports were byte-identical. The source mesh fingerprint remained
`6eb41c1d243367a2424edc3f664a8f9c61d185e0866be827fa28bf8c44d55f3a`.

Field/portable fixture result:

The current exporter emits 4 Angular sectors, 4 relations and 4 certificates.
Two non-rational reflex-excess intervals have width `5e-27`; both bounds are
exactly certified. Contract validation has zero issues.

Semantic digest result:

`1ae4404ece65704f092fc87e9b28d11941995d1c0b7607a8e1324f4ff42cfee8`

Exact area:

`Mul(Rational(32768, 373821260323), Add(Mul(Integer(524288), Pow(Integer(2), Rational(1, 2))), Integer(3222257)))`

Performance counters:

Not a performance card. The candidate guard schedule is bounded and telemetry
does not affect behavior.

Named unsupported outcomes:

- `ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE` is resolved for the
  reproduced non-rational `building.002` ratios;
- the same outcome remains mandatory when exact bound comparison is
  undecidable.

Assumptions not proven:

- not every future SymPy transcendental comparison is assumed decidable;
- V1 uses 27 fractional digits because its exact `phi - 1` relation is
  validated in a 28-digit Decimal context;
- FIX-00 remains a valid portable kernel interaction fixture, but not the
  current exact host-export oracle.

Risks and special opinion:

Do not silently regenerate or replace the accepted FIX-00 fixture. Its
point-contact topology is useful kernel evidence, while the current exact host
path has additional Angular semantics. Conflating those authorities would
hide a real semantic difference.

Forbidden legacy paths read:

- no.

Source mesh mutation check:

Unchanged: `true`.

Next session allowlist:

HOST-ANG-00 needs independent review/integration only. Any later card must
resolve its own allowlist and must read this differential receipt before using
the old fixture as current host evidence.

Suggested next card:

`C-R2D-01` only after independent acceptance of D-R2-00. This Host Adapter
card does not authorize that work.

Rollback notes:

Revert implementation commit
`b943ccb6cdaa0d970b3bbcd4dbeccecf75f93b7e` and the handoff commit. Kernel
files and accepted FIX-00 fixture bytes are unchanged.
