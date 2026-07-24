# Changelog

## v1.1.1 — 2026-07-24

- Reconciled the control plane with selected code baseline `c2622d0...`.
- Marked `C-R2C-01` through `C-R2C-04` as superseded by the accepted
  consolidated C-R2C gate instead of scheduling duplicate implementation.
- Recorded the observed `MULTIWAY_INTERACTION_POLICY_UNPROVEN` field carryover
  and activated `D-R2-00` after `DOC-00` and `FIX-00`.
- Clarified that `FIX-00` compares the historical pre-fix baseline with the
  selected post-C-R2C baseline using one immutable portable fixture.
- Made the packet builder reject superseded card IDs and print the canonical
  control-status ref separately from the immutable content SHA.

## v1.1 — 2026-07-24

- Added `OWNER_OPERATING_GUIDE.md` with repository installation and session lifecycle instructions.
- Added `SESSION_BOOTSTRAP_TEMPLATE.md`.
- Added `tools/build_agent_packet.py` for repo-connected and offline session packets.
- Fixed the circular BASE-00 prompt: BASE-00 now starts from reviewed commit `df587ed...` and creates `docs/architecture_status.json`.
- Updated README and task manifest with the repository-resident control-plane model.

## v1.0 — 2026-07-24

- Initial master plan, canon, protocol, decision log and 33 task cards.
