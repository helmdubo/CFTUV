# Corner Turn Sign Fix — Agent Implementation Plan

## Goal

Replace the local-normal-based turn sign computation in `_compute_corner_turn_sign()` with a 2D cross product in the patch's `(basis_u, basis_v)` space. This eliminates dependence on `mesh_tris` fan-triangulation normals and makes the algorithm universal for both flat concave patches and wrapped (U-shaped) patches.

## Context

File: `cftuv/frontier_place.py`

### Current broken flow (lines 582–618)

`_compute_corner_turn_sign()` computes CW/CCW turn direction at a corner where two chains meet. It currently:
1. Gets 3D tangents of both chains pointing AWAY from the corner.
2. Computes `cross(-src_tangent, chain_tangent)`.
3. Dots the cross product with a **local face normal** from `_cf_compute_local_normal()`.
4. Sign of dot → turn sign.

**Bug**: `_cf_compute_local_normal()` (lines 562–579) looks up a triangle from `node.mesh_tris`, which are fan-triangulated. For concave ngons the fan-tri normal can be flipped. Using real face normals instead fixes flat patches but breaks wrapped patches (different face normals at corner).

### Target algorithm

Compute the turn sign as a **2D pseudo cross product** in the patch's `(basis_u, basis_v)` coordinate system:

```python
def _compute_corner_turn_sign(chain, src_chain, anchor, is_start_anchor, node):
    # 1. src tangent pointing AWAY from corner
    src_cos = src_chain.vert_cos
    if not src_cos or len(src_cos) < 2:
        return 0
    if anchor.source_point_index == 0:
        src_tangent = src_cos[1] - src_cos[0]
    else:
        src_tangent = src_cos[-2] - src_cos[-1]

    # 2. target chain tangent pointing AWAY from corner
    chain_cos = chain.vert_cos
    if not chain_cos or len(chain_cos) < 2:
        return 0
    if is_start_anchor:
        chain_tangent = chain_cos[1] - chain_cos[0]
    else:
        chain_tangent = chain_cos[-2] - chain_cos[-1]

    # 3. incoming = -src_tangent (points INTO corner)
    incoming = -src_tangent
    outgoing = chain_tangent  # points AWAY from corner

    # 4. Project both into patch 2D basis
    in_u = incoming.dot(node.basis_u)
    in_v = incoming.dot(node.basis_v)
    out_u = outgoing.dot(node.basis_u)
    out_v = outgoing.dot(node.basis_v)

    # 5. 2D pseudo cross product (z-component of cross in UV plane)
    cross_2d = in_u * out_v - in_v * out_u

    # 6. Sign
    if abs(cross_2d) < 1e-8:
        return 0
    return 1 if cross_2d > 0 else -1
```

## Changes Required

### 1. Rewrite `_compute_corner_turn_sign()` (lines 582–618)

Replace the body with the 2D basis cross algorithm above. The function signature stays **exactly the same**:
```python
def _compute_corner_turn_sign(chain, src_chain, anchor, is_start_anchor, node):
```

The `node` parameter is still needed — for `node.basis_u` and `node.basis_v` (instead of `node.mesh_tris`).

### 2. Remove `_cf_compute_local_normal()` (lines 562–579)

This function is no longer called by anything in `frontier_place.py` after the rewrite. Delete it entirely.

### 3. Verify no other callers

Grep the entire `cftuv/` directory for `_cf_compute_local_normal` to confirm there are no other call sites. If there are other callers, do NOT delete the function — only remove the call from `_compute_corner_turn_sign`.

## What NOT to change

- Do NOT change the function signature of `_compute_corner_turn_sign`.
- Do NOT change `_try_inherit_direction` or `_perpendicular_direction_for_role` — they consume `turn_sign` and are correct.
- Do NOT change `_cf_determine_direction` — it already uses `basis_u / basis_v` correctly.
- Do NOT touch any other file.
- Do NOT add new parameters, classes, or imports.

## Verification

After making the change, search the file for any remaining references to `_cf_compute_local_normal` and confirm none exist (unless other callers were found in step 3).

The function interface is unchanged, so all existing callers (`_try_inherit_direction` at line 690) continue to work without modification.
