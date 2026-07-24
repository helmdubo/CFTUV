# Session handoff

Active slice:

`D-R2-00 — Conditional atomic multiway interaction contract gate`

Status:

`D_R2_CONTRACT_READY_FOR_HUMAN_REVIEW`

Branch:

`codex/d-r2-00-conditional-atomic-multiway`

Published branch URL:

`https://github.com/helmdubo/CFTUV/tree/codex/d-r2-00-conditional-atomic-multiway`

Base integration SHA:

`361102a6539ffbbe6fa8957a04bf12a9bae42bd8`

Implementation SHA:

`e81e1186caf906d5ba0cc549330ce4a02d14f22e`

CI-tested SHA:

`e81e1186caf906d5ba0cc549330ce4a02d14f22e`

The SHA was tested locally as source, as an installed wheel outside the
checkout, and as an extracted archive. Remote CI has not yet run.

Input contract/schema versions:

- `AnalysisSnapshotV1`;
- `DecalRequestV1`;
- `CompiledPatchEvaluationPlanV1`;
- `RawCoverageResultV2`;
- `ResolvedCoverageResultV1`;
- D-R2 contract:
  `ATOMIC_OWNERLESS_POINT_CONTACT_V1`;
- additive fixture schema:
  `cftuv.envelope.session_d_r2_atomic_point_contact_contract.v1`.

Accepted predecessor gate:

DOC-00 and FIX-00 were accepted by the release owner and integrated into
immutable content SHA
`361102a6539ffbbe6fa8957a04bf12a9bae42bd8`.
The release owner authorized the proposed D-R2 contract and implementation on
2026-07-24.

Changed paths:

- `docs/session_d_r2_multiway_contract.md`;
- `kernel/src/cftuv_envelope/interactions/contracts.py`;
- `kernel/fixtures/session_d_interactions_v1/README.md`;
- `kernel/fixtures/session_d_interactions_v1/manifest.json`;
- `kernel/fixtures/session_d_interactions_v1/fixture_hash_manifest.json`;
- `kernel/fixtures/session_d_interactions_v1/d_r2_atomic_point_contact_cases.json`;
- `kernel/tests/test_session_d_interaction_edges.py`;
- this Markdown handoff and its JSON receipt, required by the global protocol.

Public API/schema changes:

- added contract ID `ATOMIC_OWNERLESS_POINT_CONTACT_V1` in the interaction
  contract namespace;
- added
  `InteractionCoverageEffect.ATOMIC_OWNERLESS_POINT_CONTACT_IDENTITY`;
- no existing record field or schema ID changed;
- generated checked-in schemas remain current:
  `generated-contract-schemas: OK`.

Algorithms implemented:

This is a contract gate, not a resolver implementation card. The executable
contract validator admits only connected same-alpha hypergraphs whose loci are
exact point anchors, whose participant positive-area interiors are pairwise
disjoint, whose PointContact/provenance facts are complete, and which require
no ownership decision. The result is identity preservation of all 2D matter
plus ownerless point equality anchors.

Pairwise Policy B sequencing is explicitly forbidden. The current resolver
continues to fail closed until a later implementation card consumes this
contract.

Differential/oracle comparison:

- accepted portable field case: three components, three models, three
  candidates, two exact proofs;
- both proofs are at `Integer(0)` in one same-alpha batch;
- each proof contains one exact anchor and zero equality segments;
- all three pairwise positive-area intersections are exactly zero;
- 216 permutations of component/model/candidate order produce one current
  fail-closed projection;
- bypassing the guard and applying current pair Policy B in either order
  produces area `Integer(0)`,
  `INTERACTION_POLICY_B_PARTITION_UNPROVEN`, and order-dependent contribution
  digests. That path is prohibited by the accepted contract.

Focused tests and exact result:

- pre-edit baseline:
  `17 passed in 4.23s`;
- D-R2 interaction, original Session D corpus, and portable field fixture:
  `45 passed in 78.10s`;
- `py_compile`: `PASS`;
- `git diff --check`: `PASS`;
- source forbidden-import scan:
  `forbidden-import-scan: OK`;
- schema check:
  `generated-contract-schemas: OK`.

Full wheel/extraction tests and exact result:

- built `cftuv_envelope_core-0.7.0-py3-none-any.whl` from implementation SHA;
- installed it into a clean external venv and ran the complete suite from a
  correctly isolated external monorepo layout:
  `230 passed in 296.12s`;
- an earlier diagnostic launch used the checkout as cwd and an incorrect
  external directory layout:
  `226 passed, 4 failed`; three failures were missing copied fixture/evidence
  paths and one was the expected isolation check seeing sibling `cftuv`;
- `git archive` extraction of the exact implementation SHA built another
  wheel successfully;
- extracted source forbidden-import scan:
  `forbidden-import-scan: OK`;
- extracted schema check:
  `generated-contract-schemas: OK`.

Host tests and exact result, if applicable:

Not applicable. D-R2 changes no host path. The separate host regression
`ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE` remains assigned to an
independent Host Adapter card.

Field/portable fixture result:

- fixture hash:
  `efbae17746688b517ce40fa71efa85c2ff7b8ba59573ada93fb3758663a70515`;
- exact RawCoverage area:
  `Rational(122766786560, 373821260323)`;
- exact anchors:
  `Rational(232403250106, 373821260323),
  Rational(240403797009, 373821260323)` and
  `Rational(2651330823508, 373821260323),
  Rational(-452766086928, 373821260323)`;
- contractual result:
  exact identity preservation with two ownerless point loci;
- current runtime result until a later implementation card:
  `MULTIWAY_INTERACTION_POLICY_UNPROVEN`.

Semantic digest result:

The admitted field contract preserves the RawCoverage semantic digest exactly:
`622e1f6eec09e64bc1294c37643af19f630086005f9af21f10ac4cd6ed0e987a`.
The new LF-normalized contract fixture hash is
`89d27d8d308eecfed74df54c5d29ad2e3e1f38cd5ced80671de6ff05c6eaea44`.
The original 23 Session D `cases.json` hash remains
`2b54fc9061612b2c1fff85b1fda0a542f7442e1aa2991b4fde68c3af042b015c`.

Performance counters:

Not a performance card. The contract validator and evidence do not influence
runtime semantics.

Named unsupported outcomes:

- `MULTIWAY_INTERACTION_POLICY_UNPROVEN` remains mandatory for
  positive-length loci, positive-area overlap, incomplete provenance,
  coincident laws without one unique locus, ownership-dependent geometry, and
  undecidable exact admission;
- the existing three-front segment-locus fixture remains unsupported;
- `ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE` remains a separate
  Host Adapter issue.

Assumptions not proven:

- no general multiway Policy B partition is defined;
- no claim is made that point-only identity batches have been wired into the
  runtime resolver;
- acceptance of this branch does not by itself update the canonical control
  ref.

Risks and special opinion:

Removing the current multiway guard before a dedicated implementation consumes
the accepted contract would expose the demonstrated zero-area/order-dependent
pairwise failure. Keep the guard until the atomic identity path is implemented
from immutable pre-batch inputs.

Forbidden legacy paths read:

- no;
- neither `cftuv/decal_voronoi.py` nor legacy geometry in
  `cftuv/decals.py` was opened or invoked.

Source mesh mutation check:

No source mesh was opened or mutated. The portable JSON fixture was read only.

Next session allowlist:

- D-R2 review: only the seven implementation paths and these two handoff
  receipts;
- a later resolver implementation must receive its own exact card and may not
  infer permission from this contract gate;
- the separate Host Adapter regression card must use a fresh worktree and its
  own explicit allowlist.

Suggested next card:

The release owner requested a separate Host Adapter card for
`ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE`. After D-R2 acceptance,
the roadmap successor remains `C-R2D-01`.

Rollback notes:

Revert implementation commit
`e81e1186caf906d5ba0cc549330ce4a02d14f22e` and the subsequent handoff commit.
No runtime resolver, host implementation, ownership, or geometry rollback is
required.
