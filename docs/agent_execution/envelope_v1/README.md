# CFTUV AI Agent Execution Pack v1.1.1

This directory is a **repository-resident execution control plane** for external AI agents.

## Project owner: start here

Read `OWNER_OPERATING_GUIDE.md` first.

The essential rule is:

```text
Store this pack unpacked in the repository.
Do not upload all 33 cards into every new chat.
One fresh session receives one active card, one immutable base SHA,
and accepted handoffs from that card's dependencies.
```

Recommended repository path:

```text
docs/agent_execution/envelope_v1/
```

For a repo-connected agent, generate the short bootstrap prompt:

```bash
python docs/agent_execution/envelope_v1/tools/build_agent_packet.py \
  --repo-root . \
  --card DOC-00 \
  --mode repo \
  --handoff <accepted-handoff-path>
```

For a chat without repository access, generate one minimal attachment:

```bash
python docs/agent_execution/envelope_v1/tools/build_agent_packet.py \
  --repo-root . \
  --card DOC-00 \
  --mode offline \
  --handoff <accepted-handoff-path> \
  --output /tmp/DOC-00_AGENT_PACKET.md
```

## Agent reading order

1. `AGENTS.md` in the repository
2. `docs/architecture_status.json` after BASE-00
3. `01_GLOBAL_CANON.md`
4. `02_AGENT_PROTOCOL.md`
5. the exact assigned card
6. accepted dependency handoffs
7. only allowlisted code/tests
8. `templates/HANDOFF_TEMPLATE.md`

## Program documents

1. `OWNER_OPERATING_GUIDE.md`
2. `00_MASTER_PLAN.md`
3. `01_GLOBAL_CANON.md`
4. `02_AGENT_PROTOCOL.md`
5. `03_DECISION_LOG.md`
6. `BASELINE_RECONCILIATION.md`
7. `task_manifest.json`
8. `SESSION_BOOTSTRAP_TEMPLATE.md`

## Immediate cards

```text
BASE-00
DOC-00 and FIX-00 after BASE-00
D-R2-00 after DOC-00 and FIX-00
```

The consolidated occurrence-aware C-R2C gate is already present in the
selected code baseline `c2622d0...`. Its four decomposition cards are retained
as superseded historical planning records and must not be executed.

Later cards are deliberately blocked. Do not parallelize ownership, runtime or
curved implementation with the active baseline/interaction-contract sequence.

Reviewed baseline: `df587ed166cfb0e0b615148f08c583b4477c5ac4`  
M-R1 implementation/CI SHA: `e38d1406b591d1189bf98bb850c8cab5d233f1c8`

The package contains 33 task cards.
