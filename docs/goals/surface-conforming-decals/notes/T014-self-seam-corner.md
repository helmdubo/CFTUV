# T014 - SEAM_SELF corner recovery

## Root cause

A tube surface cut by one seam remains one wrapped WALL patch. Boundary
analysis correctly emits two `SEAM_SELF` chain uses for the same physical mesh
edge, one for each local owner face. The decal collector accepted only
`PATCH` neighbors, so automatic CORNERS had no candidate.

## Change

- Pair chain uses by owner patch id plus sorted source edge indices.
- Use each side's analysis-owned `side_face_normals`, not the wrapped patch
  average normal.
- Compute dihedral convexity for the paired self seam.
- Route every paired `SEAM_SELF` to CORNERS regardless of the coplanar
  threshold, so behavior does not change with tube segment density.
- Reuse the same pair in manual edge mode; unpaired mesh borders retain the
  one-wing fallback.

## Proof

- Python suite: 29 passed.
- Blender 4.3.2 real BMesh fixtures at 16 and 64 cylinder segments each
  produced one WALL patch, two `SEAM_SELF` uses, one corner chain, zero seam
  chains, and two generated corner faces.
- The 64-segment fixture has local-normal dot `0.995185`, proving the result is
  not accidentally dependent on `DECAL_COPLANAR_DOT = 0.99`.
