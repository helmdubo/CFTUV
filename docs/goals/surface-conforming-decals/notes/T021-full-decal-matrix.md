# T021 — Full decal verification matrix

Blender 4.3.2 artifact: `artifacts/cftuv_decal_verification.blend`.

The `CFTUV_Decal_Verification` collection contains six labeled source/decal pairs:

- automatic TOP: 12 faces;
- automatic BOTTOM: 12 faces;
- automatic CORNERS: 2 faces;
- automatic SEAMS: 1 face;
- manual CORNERS: 6 faces;
- manual SEAMS: 6 faces.

The saved file was reopened headlessly in Blender 4.3.2. All decal objects are non-empty. The full geometry/UV oracle additionally confirmed positive polygon areas, non-zero UV transverse signs, consistent orientation inside each mode, upper-only TOP bases, lower-only BOTTOM bases, and expected mode widths.
