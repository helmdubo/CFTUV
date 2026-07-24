# Architecture decision index

## Envelope v1 integration

| Decision | Status | Immutable content SHA | Record |
|---|---|---|---|
| BASE-00: recover the canonical integration baseline from `codex/fix-host-reflex-corner-envelope` and import only the final Envelope v1 execution-pack commit from `main` | Ready for human review | `955213c64b62f96df8d76de568671a31dd3310c5` | `artifacts/envelope_baseline/session_base_00_handoff.md` |

The accepted content SHA deliberately precedes the BASE-00 control-metadata
commit. This avoids a self-referential manifest hash: agents read
`docs/architecture_status.json` from the canonical integration ref, then
create the next card branch from its immutable `accepted_integration_sha`.
