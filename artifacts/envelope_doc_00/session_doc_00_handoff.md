# DOC-00 session handoff

Status: `DOC_00_READY_FOR_HUMAN_REVIEW`

Active slice:
`DOC-00_DOCUMENTATION_AUTHORITY_AND_AI_ONBOARDING_CORRECTION`

Branch:
`codex/doc-00-documentation-authority`

Published branch URL:
`https://github.com/helmdubo/CFTUV/tree/codex/doc-00-documentation-authority`

Base integration SHA:
`e4db68371cab83a6a26368bf9a95eda74ae8d02e`

Implementation SHA:
`779371b2cb62bce7295c522a51c05a968c8f653b`

CI-tested SHA:
`779371b2cb62bce7295c522a51c05a968c8f653b`

The CI-tested SHA was tested by the DOC-00 local authority-link, status-schema,
allowlist, diff and semantic-subtree gates. No remote workflow was run because
DOC-00 is docs-only and `.github/workflows/**` is outside the card allowlist.

Input contract/schema versions:

- status manifest: `cftuv.architecture_status.v1`;
- analysis snapshot: `AnalysisSnapshotV1`;
- decal request: `DecalRequestV1`;
- compiled plan: `CompiledPatchEvaluationPlanV1`;
- reference metric: `RationalAffinePlanarMetricV2`;
- runtime metric: `RuntimePlanarMetricV1`;
- RawCoverage: `RawCoverageResultV2`;
- RawCoverage schema: `cftuv.envelope.raw_coverage_result.v2`.

Accepted predecessor gate:
`BASE-00` accepted by the release owner and reviewer with the docs-only
control-plane amendment at
`e4db68371cab83a6a26368bf9a95eda74ae8d02e`.

Changed paths:

- `AGENTS.md`;
- `README.md`;
- `docs/envelope_engine_start_here.md`;
- `docs/envelope_external_agent_session_manifest.md`;
- `docs/architecture_status.json`;
- `docs/adr_index.md`;
- this Markdown handoff and its machine-readable JSON receipt, required by the
  global protocol.

Public API/schema changes:
No kernel, host, Python API, contract or fixture schema changed.
`docs/architecture_status.json` retains schema ID
`cftuv.architecture_status.v1` and adds backward-compatible control metadata:
one ranked authority order, document labels, current-card routing, accepted
BASE-00 state and structured dispatch.

Algorithms implemented:
No geometry or runtime algorithm. DOC-00 implements documentation-authority
routing and classification only.

Differential/oracle comparison:

- baseline audit reproduced the stale embedded BASE-00 status, old
  `955213c…` SHA, missing authority order/labels, and Session A/B dispatch;
- current authority validator resolved `DOC-00` in the execution-pack manifest
  as `READY_AFTER_BASE`;
- no accepted geometry documents conflicted, so no ADR-level semantic choice
  was required.

Focused tests and exact result:

- status-manifest schema: `PASS`;
- active-slice lookup: `DOC-00:READY_AFTER_BASE`;
- documentation-link validator: `PASS`;
- validator scope: `AUTHORITY_LINK_SCOPE_V1`;
- authority/local paths checked: `21`;
- canonical-only BASE-00 receipt paths checked: `2`;
- validator errors: `0`;
- DOC-00 six-file allowlist gate: `PASS`;
- `git diff --check`: `PASS`;
- independent no-prior-conversation review: three findings corrected, final
  result `PASS`.

Full wheel/extraction tests and exact result:
Not rerun because DOC-00 changed documentation/control metadata only. The
implementation commit retains the exact accepted subtree OIDs:

- kernel: `51485c555b6739326fa46b5a507fdc795da8aab9`;
- `cftuv`: `c9df63347ff5fe216f4b3e4dd15d9d257fbef814`;
- tests: `c1e129238931054bbb6fa7468c41e1e17493547a`;
- workflows: `f37881cbf99f372aa4d22d508f869b9e447ce2ad`.

The accepted predecessor remains covered by GitHub Actions run
`30095478731`: wheel/extraction `224 passed` and Blender-free host bridge
`26 passed`.

Host tests and exact result, if applicable:
Not applicable and not rerun; no host path changed.

Field/portable fixture result:
Not applicable; no fixture or source mesh was opened, invoked or changed.

Semantic digest result:
`BYTE_IDENTICAL_SEMANTIC_SUBTREES_TO_E4DB683`. Kernel, addon, tests and
workflow tree OIDs match the immutable base exactly.

Performance counters:
Not applicable to documentation-only work.

Named unsupported outcomes:

- `MULTIWAY_INTERACTION_POLICY_UNPROVEN` — active correctness blocker owned by
  D-R2;
- `REFERENCE_TOUCHING_HOLE_TOPOLOGY_UNPROVEN`;
- `RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED`;
- `COMPILED_ENVELOPE_CACHE_ALPHA_DEPENDENCE`;
- `HOST_REQUEST_EXPORT_COMPLEXITY`;
- `DOC_00_CI_HOOK_OUTSIDE_ALLOWLIST` — the requested workflow hook cannot be
  added without violating DOC-00's write allowlist.

Assumptions not proven:

- publication of this branch does not itself update the canonical control ref;
- DOC-00 remains human-acceptance pending after publication;
- a remote CI hook still requires an explicitly allowlisted control-plane
  maintenance slice.

Risks and special opinion:
The live status must be read from
`codex/base-00-canonical-integration`; immutable card bases can contain older
embedded status. Historical documents remain useful evidence but cannot
dispatch tasks. Installing a workflow by violating the allowlist would be a
larger control-plane error than recording the named CI-hook limitation.

Forbidden legacy paths read:

- no;
- neither `cftuv/decal_voronoi.py` nor legacy geometry in `cftuv/decals.py` was
  opened or invoked.

Source mesh mutation check:
No source mesh was opened or mutated. No tracked mesh path differs from the
immutable base.

Next session allowlist:

- for DOC-00 review: the six changed allowlisted authority files and these two
  handoff receipts only;
- after DOC-00 acceptance: update the canonical control ref through a
  separately authorized control-plane action;
- `FIX-00` remains a separate branch/session whose allowlist must be taken from
  its exact card;
- do not start `D-R2-00` until both DOC-00 and FIX-00 are accepted;
- do not dispatch superseded `C-R2C-01` through `C-R2C-04`.

Suggested next card:
`FIX-00` if it has not already run in the parallel baseline group. Then
`D-R2-00` only after both required acceptances. Neither card was read or started
by DOC-00.

Rollback notes:
Revert the DOC-00 implementation commit and the subsequent handoff-receipt
commit. No kernel, host, test, workflow or geometry rollback is required.
