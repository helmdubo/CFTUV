# CFTUV

CFTUV (Constraint-First Trim UV) is a Blender addon for semi-procedural UV
unwrapping of architectural hard-surface meshes for trim sheet / tile workflows.

The core solve rule is **chain-first strongest-frontier**: scaffold grows chain
by chain from a global frontier pool across the whole quilt.

## Target

- Blender 4.1+ (legacy GPENCIL) through 4.5.x (GREASEPENCIL v3)
- Python 3.10+
- Hard-surface environment production meshes
- Trim sheet / tile UV workflows

## Decal dependency

The patch-bounded Decal Seams backend uses `pyvoronoi` 1.2.8+ (Boost segment
Voronoi bindings). Install it into Blender's Python environment before using
interactive seam decals:

```powershell
& "<path-to-Blender>\4.3\python\bin\python.exe" -m pip install "pyvoronoi>=1.2.8"
```

Core PatchGraph analysis and UV solve modules remain dependency-free. If the
wheel is unavailable or a selected component is unsupported, Decal Seams
fails with a named reason. There is no legacy geometry fallback.

This backend is **frozen**: only field-crash fixes, no new capability. New decal
work goes into the Blender-free envelope kernel (`kernel/`). See `ROADMAP.md`.

## Layout

```text
cftuv/     Blender addon: analysis -> solve -> UV, plus the frozen decal backends
kernel/    Blender-free exact envelope kernel, published as `cftuv-envelope-core`
tests/     Host test suite (needs pyvoronoi for decal tests)
tools/     Corpus validators, field gates, benchmarks
```

## Tests

```bash
python3 -m pytest tests                                # host addon
PYTHONPATH=kernel/src python3 -m pytest kernel/tests   # Blender-free kernel
```

A clean clone must be green. `tests/blender/` runs only inside Blender.
`tests/test_architecture.py` holds the project's structural rules in executable
form — it is the reason `AGENTS.md` can stay short.

## Documentation

| File | When to read |
|------|-------------|
| `AGENTS.md` | Always. The only mandatory read, ~145 lines |
| `ROADMAP.md` | Phase plan for collapsing to a single decal engine |
| `ACCEPTANCE.md` | What the owner checks in Blender before legacy is deleted |
| `DECISIONS.md` | Why things are the way they are, one line per decision |

Everything under `docs/` is reference material for a specific task, not required
reading. How the code works is answered by the code.

## Validation

- Automated: `pytest` (host + kernel), including metamorphic and differential
  kernel tests
- In Blender: Grease Pencil debug layers (Analyze toggle), console diagnostics
  (Verbose Console toggle), regression snapshots, UV Editor inspection on the
  field mesh set listed in `ACCEPTANCE.md`
