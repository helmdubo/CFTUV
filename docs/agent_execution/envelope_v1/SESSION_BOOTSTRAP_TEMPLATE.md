# AI session bootstrap templates

## Repo-connected implementation session

```text
Work in repository `helmdubo/CFTUV`.

Active task: `<CARD_ID>`
Card path: `docs/agent_execution/envelope_v1/cards/<CARD_FILE>.md`
Accepted base SHA: `<SHA>`
Working branch/worktree: `<BRANCH_OR_PATH>`
Accepted dependency handoffs:
- `<HANDOFF_PATH>`

Read in order:
1. `AGENTS.md`
2. `docs/architecture_status.json` (not applicable to BASE-00)
3. `docs/agent_execution/envelope_v1/01_GLOBAL_CANON.md`
4. `docs/agent_execution/envelope_v1/02_AGENT_PROTOCOL.md`
5. the exact active task card
6. accepted dependency handoffs
7. only code/tests in the card allowlist

Implement only this card. Do not read legacy decal geometry unless this card explicitly assigns that role. Do not start downstream tasks. If a new product semantic choice is needed, stop with a named issue. Commit the card scope and leave the mandatory repository handoff and exact evidence.
```

## BASE-00 exception

```text
Work in repository `helmdubo/CFTUV` on `BASE-00`.
The reviewed baseline is `df587ed166cfb0e0b615148f08c583b4477c5ac4`.
Use a coordination branch based directly on that commit.
`docs/architecture_status.json` does not exist yet; creating it is a deliverable of this card.
Do not merge main-side commits or modify geometry without an explicit impact decision.
Read `AGENTS.md`, the BASE-00 card, the M-R1 handoff and only its allowlist.
```

## Independent verification session

```text
Act as the independent verifier for `<CARD_ID>`.
Accepted base: `<BASE_SHA>`
Implementation SHA: `<IMPLEMENTATION_SHA>`
PR/diff: `<PR_OR_DIFF>`
Card: `<CARD_PATH>`
Implementation handoff: `<HANDOFF_PATH>`
Evidence: `<RECEIPT_PATHS>`

Do not continue implementation unless a minimal verifier-owned test is explicitly permitted by the card. Search for invariant violations, missing counterexamples, scope drift, hidden fallback, non-determinism and mismatch with the acceptance gate. Return ACCEPT, REJECT or ACCEPT_WITH_NAMED_CARRYOVERS with exact evidence.
```
