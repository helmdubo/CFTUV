# Architecture decision index

## Envelope v1 integration

| Decision | Status | Immutable content SHA | Record |
|---|---|---|---|
| BASE-00: recover the canonical integration baseline from `codex/fix-host-reflex-corner-envelope`, import the Envelope v1 execution pack, and reconcile it with the selected consolidated C-R2C gate | Amendment ready for human review | `e4db68371cab83a6a26368bf9a95eda74ae8d02e` | `artifacts/envelope_baseline/session_base_00_control_plane_amendment.md` |

The accepted content SHA deliberately precedes the BASE-00 control-metadata
commit. This avoids a self-referential manifest hash: agents read
`docs/architecture_status.json` from the canonical integration ref, then
create the next card branch from its immutable `accepted_integration_sha`.

The original BASE-00 content SHA `955213c…` remains an auditable predecessor.
Execution-pack v1.1.1 supersedes its stale C-R2C scheduling assumptions without
changing kernel, host, tests, workflow, contracts, or geometry.
