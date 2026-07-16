# T018 - Manual Decal Seams recovery

## Exact-scene failure

The user selected physical seam edges `35`, `5`, and `14` on `Cube.001`:
one vertical wrapped WALL self seam and two upper WALL-to-top junction edges.
The old manual path expanded each selected edge to every containing canonical
chain. Because WALL-to-top was one closed 12-edge chain while top-to-WALL was
split into short chains, the generated object contained 32 faces and duplicate
or unrelated geometry.

Manual `Decal Seams` also ignored its mode: it used Corner Width `0.682` and the
corner builder settings instead of Seam Width `0.15`.

## Systemic change

- Preserve selected physical edge indices in the immutable operator state.
- Treat each selected physical edge as atomic manual scope.
- Find its one or two owner-side uses inside PatchGraph chains and pair those
  uses by source mesh edge index.
- Derive points, local normals, and convexity per selected segment, independent
  of chain segmentation on the other patch.
- Use Seam Width and seam UV for manual SEAMS; non-coplanar seams receive one
  half-wing per owner surface, while coplanar seams use one centered strip.
- Preserve the user's edge selection after generation.

## Proof

- Python suite: 31 passed.
- Exact Blender 4.3.2 MCP/UI operator selection `[35, 5, 14]`:
  - before: 32 faces, 56 vertices;
  - after: 6 faces, 18 vertices;
  - all six transverse half-widths: `0.075` within floating-point tolerance;
  - selected edges after generation: `[5, 14, 35]`.
- The fixed `Decal_Seams_Cube.001` remains visible in the current scene.
