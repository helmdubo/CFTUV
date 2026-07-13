# T031 — Continuous selected-edge corner runs

The screenshot showed alternating gaps because `_collect_manual_edge_decals()` returned one two-point item per selected physical edge. `_build_corner_strip()` therefore computed endpoint tangents and wing vertices independently for every edge. Neighboring strips met only at the spine and left unmatched outer wing endpoints.

The recovery introduces `_OrientedCornerRun` as a decal-only derived run:

- selected physical edges remain the exact atomic manual scope;
- endpoint-valence `2` stitches consecutive segments into one run;
- point contacts and valence `3+` stop the run;
- both owner-surface normals and source edge indices remain segment-local;
- surface side A/B is aligned by normal continuity when neighboring chains use a different patch ordering;
- internal stations blend adjacent frames and allocate one shared spine/A-wing/B-wing vertex triplet.

This remains a decal producer view over canonical PatchGraph chains. It does not make Junction a solve entity and does not alter chain-first scaffold behavior.

The Blender 4.3.2 curved-seam oracle selected edges `5, 14, 20` across changing side normals. It produced one three-segment run, 12 vertices, 6 faces, and minimum face area `0.16558867`. Independent edge strips would have allocated 18 pre-weld vertices. Blender 4.1.1 produced the same 1/12/6 result.
