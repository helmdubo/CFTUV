# DOC-00 and FIX-00 dependency acceptance

Status: `DOC_00_AND_FIX_00_ACCEPTED_AND_INTEGRATED`

The release owner accepted DOC-00 by approving progression to FIX-00 and then
explicitly accepted FIX-00 with `принимаю`.

The two published dependency branches were integrated into the canonical
ancestry without changing kernel or host implementation:

- previous accepted integration SHA:
  `e4db68371cab83a6a26368bf9a95eda74ae8d02e`;
- new immutable accepted content SHA:
  `361102a6539ffbbe6fa8957a04bf12a9bae42bd8`;
- content tree:
  `070448c32cdde0bb49b93478fdd598edbfec8439`.

Integrated dependency implementations:

- DOC-00: `779371b2cb62bce7295c522a51c05a968c8f653b`;
- FIX-00: `f870cba6b48b81d95ec390e7d46129c2550d1728`.

Validation:

- dependency content equivalence: `PASS`;
- control-plane JSON and accepted-handoff cross-references: `PASS`;
- FIX-00 focused tests on the integrated content: `5 passed in 1.94s`;
- `git diff --check`: `PASS`.

The live control ref now dispatches `D-R2-00` from the immutable content SHA
above. D-R2-00 is at its required product-owner contract gate; semantic
implementation has not started.

The separate current-host issue remains open:
`ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE`. It does not block
D-R2-00 because FIX-00 provides the portable immutable snapshot/request pair.
It must not be repaired inside the D-R2-00 interaction-contract allowlist.
