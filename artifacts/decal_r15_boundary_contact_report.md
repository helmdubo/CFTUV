# R1.5 / RF26 — boundary-contact provenance

Field object: `sagging_wall`; selected spine edge: `e3`; compiled backend:
`RAIL_PLANAR`; compile budget: `alpha=4.0`. The `.blend` file was not saved.

## Terminal routing

| Patch side | Endpoint | Plan choice | Backend |
|---|---:|---|---|
| patch 0, faces 14/16 | v3/e3 | `PERP` / `IN_PLANE_EMPTY` | `RAIL_PLANAR` |
| patch 0, faces 0/15/16 | v4/e3 | `PERP` / `IN_PLANE_EMPTY` | `RAIL_PLANAR` |
| patch 1, face 3 | v3/e3 | `FOLD e18` / `ROUTE` | `RAIL_PLANAR` |
| patch 1, face 3 | v4/e3 | `FOLD e22` / `ROUTE` | `RAIL_PLANAR` |

The marked field defect is therefore `BODY/SOURCE_EDGE`, not a terminal,
CAP, or KITE ownership failure.

## Root cause

The in-plane channel has no corner partition. Its two compiled boundary paths
have independent structural reach:

| Path | Termination | Reach |
|---|---|---:|
| `IN_PLANE_PATH(v3,e3,-1)` | `REGION_BORDER` | 1.447880990 |
| `IN_PLANE_PATH(v4,e3,-1)` | `REGION_BORDER` | 2.729303861 |

`RailChannel.alpha_limit=1.447880990` is the last alpha at which both paths
are still open. The old evaluator used it as a limit for the whole channel,
therefore the long bank was frozen at the short bank's end. At every requested
width above 2.895761980 the two contacts collapsed near the same `e1` station:
0.724323 and 0.724680. This was the forbidden coupling of the two skirts.

## RR8b result

The short bank remains on its proven route end. The other contact continues as
the canonical `rail-source-edge-station` of physical mesh-boundary edge `e1`:

| Width | Alpha | Moving contact key | Moving position | Short route end |
|---:|---:|---|---|---|
| 2.4 | 1.2 | `rail-frontier(...,1.2)` | (3.032671, -25.847111, 37.343487) | still open |
| 2.8 | 1.4 | `rail-frontier(...,1.4)` | (3.013293, -25.718298, 37.191723) | still open |
| 3.2 | 1.6 | `rail-source-edge-station(e1,0.650663)` | (3.003823, -25.687395, 36.955597) | `e1@0.724680` |
| 4.0 | 2.0 | `rail-source-edge-station(e1,0.456972)` | (2.947186, -25.687395, 36.435692) | `e1@0.724680` |

No `rail-frontier-cell` second representation remains after the route end.
The moving and stopped sides are not numerically linked.

Wireframe evidence:
`artifacts/decal_r15_boundary_contact_wireframe.png`.

## Verification

- Focused rail tiers: `56 passed, 3 skipped`.
- Tier-2, exactly once after the code change:
  `516 passed, 3 skipped, 4 deselected` with
  `-m "not atlas_frozen"` in 81.29 s.
- The four heavy atlas cases are explicitly marked `atlas_frozen`; no atlas
  test leaked into tier-2.
- Installed Blender 4.3 addon synchronized byte-for-byte (56 Python files),
  embedded code identity `e98c5ba9`. Live installed-addon evaluation at width
  4.0 reproduced independent `e1` stations 0.456972 and 0.724680.
