# T024 — BBox-centered decal drag anchor

Automatic TOP, BOTTOM, and CORNERS no longer use the N-panel button click as the modal origin.

At invoke time CFTUV now:

1. finds the `WINDOW` region of the current 3D View;
2. transforms the source object's local bounding-box center to world space;
3. projects that point into the viewport;
4. clamps the anchor to a 24-pixel viewport inset;
5. warps the cursor to the anchor and uses its X coordinate as the zero point.

If the object center is off-screen, behind the view, or cannot be projected, the viewport center is used. If cursor warping itself is unavailable, the old click position is retained as a safe fallback.

The screen-space logic lives in `cftuv/decal_modal.py`; `operators.py` remains a thin Blender UI wrapper.

Verification passed with 40 pytest tests and Blender 4.1.1/4.3.2 runtime imports. Both Blender versions expose `Window.cursor_warp`, and both resolve the synthetic projected bbox center to the expected viewport coordinate `(500, 350)`.
