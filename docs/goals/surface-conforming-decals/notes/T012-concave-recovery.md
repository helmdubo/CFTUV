# T012 concave TOP/BOTTOM recovery

## User evidence

- Blender 4.3 screenshot: `codex-clipboard-eb4701f2-1027-4527-8f35-e12628367d44.png`
- Exact autosave fixture: `C:/Users/helmd/AppData/Local/Temp/buildings2_9156_autosave.blend`
- Source object: `Cube` (24 vertices, 22 faces)
- Saved failing object: `Decal_Top_Cube` (12 faces)

## Root cause

All vertical faces belong to one wrapped WALL patch because CFTUV splits
patches only by Seam. Its average normal is approximately
`(0.8576, -0.5143, 0)`. TOP/BOTTOM classified every boundary segment against
that average, so the saved TOP object contained six strips at `z ~= +1` and
six strips at `z ~= -1`.

## Recovery contract

Analysis serializes one owner-face normal per final BoundaryChain segment in
`BoundaryChain.side_face_normals`, aligned with `edge_indices`. Decal trim
classification and segment frames consume that local normal and a local
WORLD_UP projection, with PatchNode normal/basis only as compatibility fallback.

## Proof

Blender 4.3 loaded the exact autosave with `--factory-startup` and confirmed
the repo module path. After the recovery, every TOP run point is above `z=0.9`,
every BOTTOM run point is below `z=-0.9`, and regenerated TOP geometry has
12 faces with z bounds approximately `0.643..0.998`.
