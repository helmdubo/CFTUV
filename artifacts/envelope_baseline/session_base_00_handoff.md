# BASE-00 canonical integration baseline — handoff

Active slice:
`BASE-00_CANONICAL_INTEGRATION_BASELINE_AND_BRANCH_RECOVERY`

Base integration SHA:
`df587ed166cfb0e0b615148f08c583b4477c5ac4` was the reviewed M-R1 baseline.
By explicit integration decision, the latest code baseline is
`c2622d07020338e5231b81f41655fe6c74cdca72` from
`codex/fix-host-reflex-corner-envelope`.

Implementation SHA:
`955213c64b62f96df8d76de568671a31dd3310c5`

CI-tested SHA:
`c2622d07020338e5231b81f41655fe6c74cdca72`

Input contract/schema versions:

- package `cftuv-envelope-core 0.7.0`;
- `AnalysisSnapshotV1`;
- `DecalRequestV1`;
- `CompiledPatchEvaluationPlanV1`;
- `RationalAffinePlanarMetricV2`;
- `RuntimePlanarMetricV1`;
- `RawCoverageResultV2`;
- `cftuv.envelope.raw_coverage_result.v2`.

Accepted predecessor gate:
M-R1 plus the exact C-R2C boundary rotation gate and the host reflex-corner
export at `c2622d0`. GitHub Actions run
`30095478731` passed all three required jobs.

Changed paths:

- 46 files under `docs/agent_execution/envelope_v1/**`, replayed from the
  single documentation-only source commit `3021146…`;
- `docs/architecture_status.json`;
- `docs/adr_index.md`;
- `artifacts/envelope_baseline/session_base_00_handoff.json`;
- `artifacts/envelope_baseline/session_base_00_handoff.md`.

Public API/schema changes:
None in BASE-00.

Algorithms implemented:
None. This slice performs branch recovery, immutable baseline selection and
control-plane metadata only.

Differential/oracle comparison:
The accepted integration content at `955213c…` and `origin/main` at
`3021146…` have the identical full tree
`ff75de519f0410193808ea8c474b82effe9ae1c1`. Their histories differ, but their
files do not. Relative to `c2622d0`, `955213c…` adds only the execution pack.
Kernel, `cftuv`, tests and workflow tree OIDs are byte-identical to
`c2622d0`.

Focused tests and exact result:
The Blender-free host matrix passed `26 passed in 12.54s`; the legacy-import
scan returned `OK`.

Full wheel/extraction tests and exact result:

- independently built installed wheel outside checkout: `224 passed in
  301.34s`;
- independently copied and built extracted kernel: `224 passed in 299.27s`;
- forbidden-import scan: `OK`;
- Session A fixture projection: `OK`;
- extracted repository after gates: clean.

The card's historical expectation of `205` kernel tests is superseded by the
19 accepted C-R2C tests, giving the exact current count of `224`.

`git diff --check HEAD^..HEAD` passes for the BASE-00 metadata commit. The
complete `c2622d0..955213c` range reports the two-space Markdown hard breaks
already present in source commit `3021146`. They are preserved deliberately so
the imported execution pack remains byte-identical to `main`.

Host tests and exact result, if applicable:
`26 passed`; the host legacy-import scan returned `OK`.

Field/portable fixture result:
BASE-00 did not rerun Blender field geometry because it changes no code.
The selected predecessor's accepted `building.002` gate records exact
RawCoverage V2, source fingerprint
`3205834b7ead22cf1eb8985902144ca1c7295a88b8015556b364be17d8ac9021`, and no
source-mesh mutation.

Semantic digest result:
`BYTE_IDENTICAL_GIT_TREE_OID_V1: IDENTICAL_TO_SELECTED_CODE_BASELINE`.
Kernel tree `51485c55…`, `cftuv` tree `c9df6334…`, test tree `c1e12923…` and
workflow tree `f37881cb…` are unchanged.

Performance counters:
Not applicable to this metadata-only slice. Gate durations are recorded above.

Named unsupported outcomes:

- `MULTIWAY_INTERACTION_POLICY_UNPROVEN`;
- `REFERENCE_TOUCHING_HOLE_TOPOLOGY_UNPROVEN`;
- `REFERENCE_ARRANGEMENT_ROTATION_SYSTEM_UNPROVEN`;
- `REFERENCE_ARRANGEMENT_COLLINEAR_BRANCH_UNPROVEN`;
- `RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED`.

Assumptions not proven:

- GitHub branch protection was not configured; the branch is clearly named and
  all authority is by immutable SHA.
- Human acceptance of BASE-00 remains pending.

Risks and special opinion:
`accepted_integration_sha` points to the immutable content commit
`955213c…`, while the status and handoff live in the following control-metadata
commit. This is deliberate and avoids an impossible self-referential commit
hash. Later agents must read the manifest from the canonical ref and create
their card branch from the SHA stored in that manifest.

Forbidden legacy paths read:

- no;
- legacy decal geometry was neither opened nor invoked.

Source mesh mutation check:
No source mesh was opened in BASE-00, no tracked mesh path differs from
`c2622d0`, and no geometry source differs from the selected baseline.

## Compare-tree report

### Reviewed M-R1 baseline → selected code baseline

`df587ed… → c2622d0…`: three commits, 27 files changed, 2274 insertions and
132 deletions. These are the accepted exact boundary rotation system, its
handoff and the host reflex-corner export. Package version changes from
`0.6.0` to `0.7.0`.

### Selected code baseline → accepted integration content

`c2622d0… → 955213c…`: 46 added files and 5266 insertions, exclusively
`docs/agent_execution/envelope_v1/**`. No code, geometry, contract, test or
workflow path differs.

### Accepted integration content → main

`955213c…` and `origin/main` at `3021146…` have zero tree differences and the
same tree OID `ff75de51…`. BASE-00 therefore preserves the latest code and the
latest roadmap without merging unrelated history.

Next session allowlist:

- `docs/architecture_status.json`;
- `docs/agent_execution/envelope_v1/01_GLOBAL_CANON.md`;
- `docs/agent_execution/envelope_v1/02_AGENT_PROTOCOL.md`;
- exactly one selected card and its accepted predecessor handoffs;
- only the paths allowlisted by that card.

Suggested next card:
After human acceptance, run `DOC-00` and `FIX-00` in separate branches and
fresh sessions.

Rollback notes:
The branch can be abandoned without touching `main` or
`codex/fix-host-reflex-corner-envelope`. The immutable code predecessor is
`c2622d07020338e5231b81f41655fe6c74cdca72`.
