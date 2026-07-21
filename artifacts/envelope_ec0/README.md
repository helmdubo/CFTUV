# EC0 envelope semantics artifacts

This directory is the review corpus for `docs/envelope_backend_semantics.md`.
It contains no implementation fixtures and no sampled geometry authority.

**Pivot status: v1 evidence, EC0-P revision required.** These sheets and
sidecars preserve the 16 semantic outcomes and the user's case-16 choice B,
but predate AM7. They do not consistently expose PatchDomain,
PhysicalChain/ChainUse and compile-static seeds, so they do not open EC1.
The replacement corpus must follow
`docs/envelope_kernel_pivot_instructions.md` and sidecar schema v2 below.

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

The EC0-P rebaseline does not reopen that choice. It must redraw B as an
interaction of source contributions inside one shared PatchDomain and add a
separate self-collision view.

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
