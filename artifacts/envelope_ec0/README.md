# EC0 envelope semantics artifacts

This directory is the review corpus for `docs/envelope_backend_semantics.md`.
It contains no implementation fixtures and no sampled geometry authority.

**Pivot status: v1 audit evidence.** These sheets and sidecars preserve the
original 16 semantic outcomes and the user's case-16 choice B, but predate
AM7. The completed EC0-P candidate is in `v2/` and is ready for explicit user
review. EC1 remains closed until that acceptance.

- `cases/` — one coordinate-free YAML semantic sidecar per canonical case.
- `sheets/` — review sheets in SVG and PNG. Every sheet contains skeleton,
  coverage, ownership, UV/station flow, and an alpha evolution strip.
- `sheets/ec0-overview.png` — contact sheet for all 18 review variants.
- `metamorphic_matrix.yaml` — AM3 transformation verdicts for all cases.

Case 16 has three visual variants (`A`, `B`, `C`). The user selected **B**:
coverage of colliding pChain wings is clipped at the mutual equality locus.
All participants are wings of one decal inside one owner Patch; cross-decal and
cross-Patch collision is forbidden. Variant A and C sheets remain only as
rejected decision evidence.

The EC0-P rebaseline does not reopen that choice. It redraws B as an
interaction of source contributions inside one shared PatchDomain and adds a
separate self-collision view. That rebaseline is now present in `v2/`; the v1
files remain non-canonical evidence rather than being silently upgraded.

AM8 adds `BOUNDARY_LIMITED_PROPAGATION` for v1. Formal policy, its ten
coordinate-free scenarios, metamorphic verdicts and the consolidated review
sheet live under `pivot/`. Full obstacle bypass is explicitly deferred to
EC8+; current artifacts must never imply split/merge around a hole.

Color key used by every sheet:

- blue line — semantic skeleton/spine;
- pale blue area — coverage of component or primitive A;
- pale green area — coverage of component or primitive B;
- amber line/area — ownership divider or junction matter;
- magenta line/area — BEVEL or rejected interaction matter;
- dashed gray — domain or pre-interaction ghost boundary.
