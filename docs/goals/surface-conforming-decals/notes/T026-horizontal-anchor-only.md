# T026 — Horizontal-only bbox anchor

User validation showed that the full 2D warp from T024 changed the feel of the otherwise-correct horizontal sizing from `0cc1406`.

The sizing formula was already based only on `event.mouse_x`. The recovery therefore leaves direction, sensitivity, Shift precision, and minimum unchanged, while narrowing the anchor adjustment to one axis:

- X is warped to the projected source-object bbox center or viewport-center fallback;
- Y remains exactly `event.mouse_y` from the button click;
- a `MOUSEMOVE` that changes only Y returns immediately without regenerating the decal.

Blender's `WM_cursor_warp` writes the requested X/Y into the window event state. Preserving the original Y avoids introducing an unrelated vertical relocation before modal input starts.

Verification passed with 41 pytest tests. Blender 4.1.1 and 4.3.2 both converted the synthetic click `(321, 654)` into the expected X-only anchor `(500, 654)`.
