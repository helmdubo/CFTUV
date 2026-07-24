# BASE-00 docs-only control-plane amendment

Status: `ACCEPTED`

Human acceptance: `ACCEPTED_BY_RELEASE_OWNER` on `2026-07-24`

The selected baseline `c2622d0…` already includes the accepted consolidated
C-R2C gate. The imported v1.1 execution pack nevertheless described
point-contact handling as the current blocker and scheduled four C-R2C cards
again.

Execution-pack v1.1.1 fixes that control-plane mismatch:

- `C-R2C-01` through `C-R2C-04` are superseded and non-dispatchable;
- `MULTIWAY_INTERACTION_POLICY_UNPROVEN` is the active correctness carryover;
- `DOC-00` and `FIX-00` are next after BASE-00 acceptance;
- `D-R2-00` follows both;
- packet bootstrap separates the canonical live control ref from the immutable
  content SHA.

Immutable amended content:
`e4db68371cab83a6a26368bf9a95eda74ae8d02e`

Content tree:
`70579a29715aa846d88e409c85c53144b341386b`

Amendment commits:

- `49cfa218d7d6704f30ae1de6788cd350e0e52f83`;
- `729e33025409f51a7dae816b300bd1ed618acf5e`;
- `e4db68371cab83a6a26368bf9a95eda74ae8d02e`.

The amendment changes documentation/control-plane files only. Kernel, `cftuv`,
tests and workflows retain their exact selected-baseline tree OIDs. Existing
CI evidence remains GitHub Actions run `30095478731`; amendment-specific
manifest, packet-builder, syntax and diff gates pass.

The release owner explicitly accepted BASE-00 with this docs-only amendment.
`DOC-00` and `FIX-00` may be dispatched as separate baseline-parallel cards.
