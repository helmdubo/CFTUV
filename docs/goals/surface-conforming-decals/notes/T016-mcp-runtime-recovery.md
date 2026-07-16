# T016 - MCP runtime recovery

## Exact-scene reproduction

Blender MCP connected to the user's open Blender 4.3.2 scene with source mesh
`Cube.001`. Before runtime synchronization, automatic TOP produced 12 faces:
six were based on upper edges and six were based on lower edges. Automatic
CORNERS produced only one face because its two wings had the same fallback
normal.

## Root cause

The installed addon was a mixed version. Installed `decals.py` and
`analysis_topology.py` matched the branch, but installed
`analysis_boundary_loops.py` did not contain
`_assign_boundary_chain_side_face_normals`. Every final chain therefore had an
empty `side_face_normals` list and decal generation fell back to the wrapped
patch average normal.

## Recovery and proof

- Backed up the installed addon to
  `cftuv_backup_20260711_mcp_recovery` beside the Blender 4.3 addon directory.
- Synchronized all four mismatched package files from the repository.
- Verified every repository package file against the installed package:
  mismatch count `0`.
- Reloaded boundary, topology, analysis, and decal modules through MCP.
- All 24 exact-scene chains now contain local side normals.
- TOP now has 12 faces whose base Z range is `0.993162..0.998217`; lower base
  count is `0`.
- CORNERS now has two faces and two distinct wings.

The regenerated objects remain visible in the current unsaved scene for direct
user inspection.
