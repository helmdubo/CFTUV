# Architecture decision and documentation-status index

Machine-readable index status: `CURRENT_AUTHORITY_INDEX`.

## Envelope v1 decisions

| Decision | Status | Immutable content SHA | Record |
|---|---|---|---|
| BASE-00: recover the canonical integration baseline from `codex/fix-host-reflex-corner-envelope`, import the Envelope v1 execution pack, and reconcile it with the selected consolidated C-R2C gate | Accepted by release owner with docs-only amendment | `e4db68371cab83a6a26368bf9a95eda74ae8d02e` | `artifacts/envelope_baseline/session_base_00_control_plane_amendment.md` |
| DOC-00: canonical control status selects one authority order; historical session documents cannot dispatch work | Accepted by release owner | `779371b2cb62bce7295c522a51c05a968c8f653b` | `artifacts/envelope_doc_00/session_doc_00_handoff.md` |

The immutable content SHA deliberately precedes live control-metadata updates.
Agents read `docs/architecture_status.json` from
`codex/base-00-canonical-integration`, then create a card branch from its
`accepted_integration_sha`.

## Documentation status index

| Path or context | Machine-readable label | Authority scope |
|---|---|---|
| `README.md` | `CURRENT_ONBOARDING_AUTHORITY` | Repository entry and Envelope bootstrap |
| `AGENTS.md` | `CURRENT_ONBOARDING_AUTHORITY` | Repository invariants, role and context guards |
| `docs/envelope_engine_start_here.md` | `CURRENT_ENVELOPE_ONBOARDING_AUTHORITY` | Human-readable authority order and architecture map |
| `docs/architecture_status.json` on the canonical control ref | `LIVE_CONTROL_AUTHORITY` | Immutable base, active card, blockers, handoffs and dispatch |
| `docs/agent_execution/envelope_v1/01_GLOBAL_CANON.md` | `ACCEPTED_GLOBAL_CANON` | Accepted Envelope architecture |
| `docs/agent_execution/envelope_v1/02_AGENT_PROTOCOL.md` | `CURRENT_EXECUTION_PROTOCOL` | Branch, allowlist, gates and handoff |
| `docs/envelope_external_agent_session_manifest.md` | `SUPERSEDED_FOR_ACTIVE_DISPATCH` | Retained historical A–J stage evidence only |
| `docs/decal_envelope_linear_axis_rebaseline.md` | `ACCEPTED_SCOPED_SEMANTIC_EVIDENCE` | Accepted AM11 semantic evidence; no task dispatch |
| `docs/envelope_backend_semantics.md` | `ACCEPTED_SCOPED_SEMANTIC_EVIDENCE` | Accepted EC0/backend semantic evidence; no task dispatch |
| `docs/decal_envelope_roadmap_compromise.md`, `docs/envelope_kernel_pivot_instructions.md` | `SUPERSEDED_FOR_ACTIVE_DISPATCH` | Retained pivot history; no task dispatch |
| `docs/envelope_ec0_correction_log.md` | `HISTORICAL_AUDIT_EVIDENCE` | Historical gate audit only |
| `cftuv/decals.py`, `cftuv/decal_voronoi.py`, `pyvoronoi` | `SCOPED_LEGACY_RUNTIME` | Existing decal producer only; not new-kernel authority |

Any future experimental document must use
`EXPERIMENTAL_NON_AUTHORITY`; it cannot override current, accepted or
active-card authority. No experimental document is present in the current
authority order.

## DOC-00 documentation audit

Baseline audit against
`e4db68371cab83a6a26368bf9a95eda74ae8d02e` found:

- the embedded status still named BASE-00 and pre-amendment SHA `955213c…`;
- README presented `pyvoronoi` before any new-kernel authority route;
- AGENTS placed legacy decal-backend language next to new-kernel instructions;
- `start_here` and the external session manifest still dispatched historical
  Session B/EC1 rather than the Envelope v1 card pack;
- status labels and a single machine-readable authority order were absent.

No accepted documents conflicted on geometry. The conflict was authority and
dispatch metadata, so DOC-00 did not create or alter product semantics.

DOC-00 corrects all six allowlisted authority files, preserves historical
evidence with explicit scope labels, names
`MULTIWAY_INTERACTION_POLICY_UNPROVEN` as the active correctness blocker, and
routes every new agent to the live status and exact current card.

The card also requests a CI hook, but `.github/workflows/**` is outside its
write allowlist. DOC-00 therefore records
`DOC_00_CI_HOOK_OUTSIDE_ALLOWLIST`, runs the equivalent documentation-link and
status-schema gates locally, and leaves workflow installation to an explicitly
allowlisted control-plane maintenance slice.

The documentation-link gate uses `AUTHORITY_LINK_SCOPE_V1`: it validates every
path in the machine-readable authority order, document-status index, accepted
handoff list and current-card route, plus canonical-ref-only BASE-00 receipts.
It does not claim that every historical Markdown code span anywhere in the
repository is a current authority link.
