# BASE-00 canonical integration baseline — amended handoff

Active slice:
`BASE-00_CANONICAL_INTEGRATION_BASELINE_AND_BRANCH_RECOVERY`

Reviewed M-R1 baseline:
`df587ed166cfb0e0b615148f08c583b4477c5ac4`

Selected code baseline:
`c2622d07020338e5231b81f41655fe6c74cdca72` from
`codex/fix-host-reflex-corner-envelope`, by explicit integration decision.

Immutable amended content SHA:
`e4db68371cab83a6a26368bf9a95eda74ae8d02e`

CI-tested SHA:
`c2622d07020338e5231b81f41655fe6c74cdca72`

Human acceptance:
`HUMAN_ACCEPTANCE_PENDING`

## Amendment

The original v1.1 execution pack imported at `955213c…` did not describe its
selected code baseline accurately. That baseline already contains the accepted
consolidated C-R2C implementation (`43e69d…`), handoff (`979870f…`), exact
boundary rotation, occurrence-aware records, point-contact records and the
`building.002` RawCoverage gate.

Execution-pack v1.1.1 resolves the mismatch:

- `C-R2C-01` through `C-R2C-04` are
  `SUPERSEDED_BY_ACCEPTED_C_R2C_GATE` and cannot be dispatched;
- the active correctness carryover is
  `MULTIWAY_INTERACTION_POLICY_UNPROVEN`;
- `DOC-00` and `FIX-00` remain the first cards after BASE-00 acceptance;
- `D-R2-00` becomes ready after both cards;
- the packet builder reads live status and handoffs from
  `codex/base-00-canonical-integration`, while branches are based on the
  immutable content SHA recorded there.

The amendment is recorded by content commits `49cfa21…`, `729e330…` and
`e4db683…`. The final content tree is
`70579a29715aa846d88e409c85c53144b341386b`.

## Scope

Changed paths through the immutable content commit:

- 47 files under `docs/agent_execution/envelope_v1/**`;
- the four original BASE-00 status/handoff files.

This following metadata commit also updates those records and adds:

- `artifacts/envelope_baseline/session_base_00_control_plane_amendment.json`;
- `artifacts/envelope_baseline/session_base_00_control_plane_amendment.md`.

No kernel, `cftuv`, test, workflow, geometry or contract path changed.
Kernel tree `51485c55…`, `cftuv` tree `c9df6334…`, test tree `c1e12923…` and
workflow tree `f37881cb…` remain byte-identical to `c2622d0`.

## Verification

The code-bearing predecessor remains covered by GitHub Actions run
`30095478731`:

- installed wheel outside checkout: `224 passed`;
- extracted kernel: `224 passed`;
- forbidden-import and Session A fixture scans: `OK`;
- Blender-free host bridge: `26 passed`;
- host legacy-import scan: `OK`.

Those expensive suites were not rerun because their exact Git tree OIDs did
not change. Amendment-specific gates passed:

- execution-pack JSON and dependency cross-references;
- active `DOC-00` packet generation;
- rejection of superseded C-R2C packet generation;
- Python syntax parsing for the packet builder;
- `git diff --check`.

Relative to `c2622d0`, amended content `e4db683…` changes 51 documentation
files with 6013 insertions and no deletions. Relative to `main@3021146…`, it
changes 24 docs/control-plane files with 864 insertions and 117 deletions.
The semantic subtrees remain identical.

## Authority and next cards

The immutable content commit cannot contain its own hash. Agents must read
`docs/architecture_status.json`, this handoff and
`BASELINE_RECONCILIATION.md` from the canonical control ref, then create a
card branch from the `accepted_integration_sha` stored in that status.

After human acceptance, run `DOC-00` and `FIX-00` in separate fresh branches.
`D-R2-00` follows both. Do not dispatch `C-R2C-01` through `C-R2C-04`.

No legacy decal geometry was opened or invoked, and no source mesh was opened
or mutated by BASE-00.
