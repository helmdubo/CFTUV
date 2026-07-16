# T022 — Universal horizontal decal sizing

The automatic TOP, BOTTOM, and CORNERS modal paths now use the same mouse axis and sign:

- drag right to increase the active size;
- drag left to decrease it;
- hold Shift for one-tenth sensitivity;
- clamp at `0.001` world units instead of producing zero-width geometry.

TOP and BOTTOM continue to edit `Trim Height`; CORNERS continues to edit `Corner Width`. Confirm and cancel semantics are unchanged.

Verification:

- 38 pytest tests passed, including modal simulations with contradictory `mouse_y` values to prove only horizontal delta is consumed;
- Blender 4.3.2 loaded the synchronized installed addon and reported RNA minima `0.001` for both properties;
- Blender 4.1.1 loaded the repository addon with the same minima and drag results;
- installed Blender 4.3 Python files match the repository package hashes.
