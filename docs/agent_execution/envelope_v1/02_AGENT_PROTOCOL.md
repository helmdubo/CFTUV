# AI-agent operating protocol

## One semantic boundary, one fresh session

Start a fresh agent context whenever work crosses any of these boundaries:

- topology contracts → arrangement implementation;
- RawCoverage → interactions;
- interactions → ownership;
- ownership → tessellation;
- reference path → runtime;
- Python runtime → native acceleration;
- kernel → Blender host;
- planar production → curved-surface research.

Context separation is part of the architecture.

## Branch and commit policy

1. Resolve the current accepted integration SHA from `docs/architecture_status.json`.
2. Create one branch per card.
3. Do not rebase or merge unrelated work silently.
4. Commit only the card scope.
5. Leave a repository handoff file and machine-readable receipt.
6. Open a draft PR only after local gates pass.
7. An agent may not approve its own semantic contract change.

Recommended branch names:

```text
agent/<card-id-lowercase>-<short-slug>
```

`docs/architecture_status.json` is control metadata on the canonical
integration ref. Because it records an immutable content SHA, the content
commit may contain an older copy of the self-referential status file. Resolve
the SHA once from `codex/base-00-canonical-integration` (or from the offline
packet produced there), create the card branch from that SHA, and do not
downgrade it by rereading the older embedded copy. The bootstrap tool prints
both the immutable content SHA and the canonical control status ref.
Repository handoff receipts listed by the bootstrap are read from the same
canonical control ref when the immutable content commit contains an older
metadata copy.

## Change discipline

Before editing:

1. Read `01_GLOBAL_CANON.md`.
2. Read the exact card.
3. Read only the card allowlist plus accepted predecessor handoff.
4. Record baseline tests.
5. Identify public schema impact before implementation.

During implementation:

- add or update tests before deleting old behavior;
- keep a differential oracle until the replacement is proven;
- emit named outcomes for unsupported geometry;
- add telemetry without allowing telemetry to affect semantics;
- avoid unrelated formatting/refactors.

Before handoff:

- run focused tests;
- run full kernel wheel tests outside checkout;
- run extraction-readiness/forbidden-import checks;
- run host bridge tests if the card touches host code;
- capture exact field/fixture evidence;
- update schemas and version IDs when public records change;
- write `artifacts/<slice>/session_<slice>_handoff.json` and a Markdown handoff.

## Review model

Every semantic slice has three roles, preferably separate agents:

```text
Contract Author
Implementation Agent
Independent Verifier
```

The verifier checks invariants and counterexamples, not only the happy path.

## Stop conditions common to all cards

Stop and report a named issue instead of broadening scope when:

- a new product semantic choice is required;
- a card would need to alter an upstream accepted contract;
- a geometry ambiguity can only be resolved by epsilon, snapping or ID order;
- a hidden legacy fallback appears necessary;
- the field fixture cannot be reproduced;
- a performance optimization changes semantic records;
- an exact predicate cannot decide a required fact under the accepted numeric domain.
