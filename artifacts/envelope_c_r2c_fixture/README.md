# FIX-00 portable building.002 point-contact fixture

This directory records the evidence for the immutable fixture in
`kernel/fixtures/building_002_point_contact_v1`. The fixture contains only:

- one canonical `AnalysisSnapshotV1`;
- one canonical `DecalRequestV1` for physical edges `2, 3, 7` at alpha `0.25`;
- the 24-vertex, 38-edge, 15-face host mesh facts needed by the deterministic
  Blender generator.

No `.blend` file, unrelated object, material, UV data, or source path is
included.

## One-command kernel reproduction

With the kernel dependencies installed, run from the repository root:

```powershell
python tools/export_building_002_point_contact_fixture.py verify --source-root . --fixture-dir kernel/fixtures/building_002_point_contact_v1
```

The same command can use a detached worktree at
`df587ed166cfb0e0b615148f08c583b4477c5ac4` as `--source-root`. The committed
receipts are:

- `historical_df587ed_result.json`: `REFERENCE_ARRANGEMENT_NON_MANIFOLD`;
- `selected_c262_result.json`: exact RawCoverage V2, digest
  `622e1f6eec09e64bc1294c37643af19f630086005f9af21f10ac4cd6ed0e987a`;
- `point_contact_topology_comparison.json`: the two rejected historical
  construction points and contributor sets are exactly the two selected
  point-contact records.

## Blender regeneration and equality

Generate the minimal object in a blank Blender process:

```powershell
blender --background --python tests/blender/generate_building_002_point_contact_fixture.py -- --fixture-dir kernel/fixtures/building_002_point_contact_v1
```

Re-export through the accepted C-R2C host implementation and compare canonical
bytes:

```powershell
blender --background --python tests/blender/test_building_002_point_contact_fixture.py -- --fixture-dir kernel/fixtures/building_002_point_contact_v1 --source-root <worktree-at-43e69d3889d273ed19daee9239ae0e311a1b213d>
```

Blender may reverse the endpoint order stored on an undirected mesh edge.
`host_export_equality.json` therefore records normalized mesh equality and
byte-identical snapshot/request output. SourceRevision is unaffected because
its edge endpoints are canonically sorted.

## Named limitation outside FIX-00

The current host exporter at `c2622d0...` rejects patch 0 before serialization
with `ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE`; the exact message
is recorded in `current_host_export_limitation.json`. FIX-00 does not patch
host or kernel behavior. The fixture was extracted with the accepted C-R2C
exporter at `43e69d3...` and is then replayed unchanged against both required
kernel revisions.
