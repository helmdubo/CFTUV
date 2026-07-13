# T028 — Interactive Decal Seams width

`SEAMS` now uses the same horizontal modal controller as TOP, BOTTOM, and CORNERS:

- right increases `Seam Width` / `DecalSettings.width_seam`;
- left decreases it to the shared `0.001` minimum;
- Shift keeps precise sensitivity;
- X-only bbox anchoring is unchanged;
- confirm reports `Seam Width`;
- cancel restores the original scene property and regenerates the base geometry.

Unlike the other manual decal modes, selected-edge `SEAMS` also enters modal. `_prepare_decal_generation()` captures `chain_refs` and `selected_edge_indices` once before the first generation, and every drag update reuses that immutable state. The selected seam scope therefore cannot expand during adjustment.

Verification passed with 46 pytest tests. Blender 4.1.1 and 4.3.2 both load the interactive SEAMS invoke path and expose a `decal_width_seam` RNA minimum of `0.001`. The installed Blender 4.3 source hash matches the repository after explicit factory-startup verification.
