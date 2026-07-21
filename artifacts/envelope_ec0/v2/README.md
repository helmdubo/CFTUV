# EC0-P v2 review corpus

Status: **READY_FOR_USER_REVIEW**. This corpus is complete on the
kernel-implementer side, but it does not become accepted until the user
explicitly approves it.

This directory is the AM7/AM8 replacement for the v1 decision evidence in the
parent directory. The v1 files remain as an audit trail; EC1 must consume v2.

Contents:

- `cases/` — 16 coordinate-free `cftuv.envelope.ec0.case.v2` sidecars;
- `sheets/` — one SVG and PNG per canonical case, plus `ec0p-overview`;
- `metamorphic_matrix.yaml` — AM3 plus AM7/AM8 verdicts for all 16 cases;
- `pivot_topology_cases.yaml` — physical seam A/B, `SEAM_SELF`, multiple
  sources in one PatchDomain, and one-wing self-collision;
- `pivot_topology_metamorphic_matrix.yaml` — transformation verdicts for the
  four AM7 pivot cases;
- `pivot-topology-cases.svg/png` — consolidated AM7 visual review sheet.

Every case sheet exposes seven layers separately:

1. `PatchDomain`;
2. `PhysicalChain` and directed `ChainUse`;
3. compile-static seeds and `FrontComponent` records;
4. boundary-limited resolution;
5. patch-level union and front interactions;
6. ownership and UV/station flow;
7. alpha evolution and named events.

Case index:

| Case | Review sheet | Sidecar |
|---|---|---|
| 01 Straight | [PNG](sheets/ec0-c01-straight-open-strip.png) / [SVG](sheets/ec0-c01-straight-open-strip.svg) | [YAML](cases/ec0-c01-straight-open-strip.yaml) |
| 02 MITER | [PNG](sheets/ec0-c02-convex-miter-corner.png) / [SVG](sheets/ec0-c02-convex-miter-corner.svg) | [YAML](cases/ec0-c02-convex-miter-corner.yaml) |
| 03 BEVEL | [PNG](sheets/ec0-c03-convex-bevel-corner.png) / [SVG](sheets/ec0-c03-convex-bevel-corner.svg) | [YAML](cases/ec0-c03-convex-bevel-corner.yaml) |
| 04 Reflex | [PNG](sheets/ec0-c04-reflex-corner.png) / [SVG](sheets/ec0-c04-reflex-corner.svg) | [YAML](cases/ec0-c04-reflex-corner.yaml) |
| 05 Physical end | [PNG](sheets/ec0-c05-physical-s-endpoint.png) / [SVG](sheets/ec0-c05-physical-s-endpoint.svg) | [YAML](cases/ec0-c05-physical-s-endpoint.yaml) |
| 06 Domain boundary | [PNG](sheets/ec0-c06-domain-boundary-contour.png) / [SVG](sheets/ec0-c06-domain-boundary-contour.svg) | [YAML](cases/ec0-c06-domain-boundary-contour.yaml) |
| 07 Data segmentation | [PNG](sheets/ec0-c07-data-chain-segmentation.png) / [SVG](sheets/ec0-c07-data-chain-segmentation.svg) | [YAML](cases/ec0-c07-data-chain-segmentation.yaml) |
| 08 T-junction | [PNG](sheets/ec0-c08-t-junction.png) / [SVG](sheets/ec0-c08-t-junction.svg) | [YAML](cases/ec0-c08-t-junction.yaml) |
| 09 X-junction | [PNG](sheets/ec0-c09-x-junction.png) / [SVG](sheets/ec0-c09-x-junction.svg) | [YAML](cases/ec0-c09-x-junction.yaml) |
| 10 Y merge/split | [PNG](sheets/ec0-c10-merge-split-junction.png) / [SVG](sheets/ec0-c10-merge-split-junction.svg) | [YAML](cases/ec0-c10-merge-split-junction.yaml) |
| 11 Short segment owner | [PNG](sheets/ec0-c11-short-segment-corner-ownership.png) / [SVG](sheets/ec0-c11-short-segment-corner-ownership.svg) | [YAML](cases/ec0-c11-short-segment-corner-ownership.yaml) |
| 12 Curved divider | [PNG](sheets/ec0-c12-curved-internal-divider.png) / [SVG](sheets/ec0-c12-curved-internal-divider.svg) | [YAML](cases/ec0-c12-curved-internal-divider.yaml) |
| 13 BEVEL collision | [PNG](sheets/ec0-c13-bevel-foreign-collision.png) / [SVG](sheets/ec0-c13-bevel-foreign-collision.svg) | [YAML](cases/ec0-c13-bevel-foreign-collision.yaml) |
| 14 Saturation | [PNG](sheets/ec0-c14-proven-saturation.png) / [SVG](sheets/ec0-c14-proven-saturation.svg) | [YAML](cases/ec0-c14-proven-saturation.yaml) |
| 15 Drag events | [PNG](sheets/ec0-c15-drag-topology-events.png) / [SVG](sheets/ec0-c15-drag-topology-events.svg) | [YAML](cases/ec0-c15-drag-topology-events.yaml) |
| 16 Wing freeze B | [PNG](sheets/ec0-c16-intrapatch-wing-freeze.png) / [SVG](sheets/ec0-c16-intrapatch-wing-freeze.svg) | [YAML](cases/ec0-c16-intrapatch-wing-freeze.yaml) |

The ten AM8 boundary scenarios remain in `../pivot/`; they are part of the
same EC0-P acceptance package. The user-facing, prose-first review route is
`../../../docs/envelope_ec0_acceptance_guide.md`.

Case 16 contains only the selected **B — coverage clip** result. Rejected A/C
drawings remain in the v1 directory solely as decision history. All collision
participants are wings/readings of one decal inside one owner Patch; different
decals and different Patches never share an interaction system.
