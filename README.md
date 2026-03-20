# CFTUV

CFTUV (Constraint-First Trim UV) is a Blender addon for semi-procedural UV
unwrapping of architectural hard-surface meshes for trim sheet / tile workflows.

The core solve rule is **chain-first strongest-frontier**: scaffold grows chain
by chain from a global frontier pool across the whole quilt.

## Target

- Blender 4.1+ (targeting 4.5)
- Python 3.11+
- Hard-surface environment production meshes
- Trim sheet / tile UV workflows

## Repo Layout

```text
cftuv/
├── __init__.py
├── constants.py
├── model.py
├── analysis.py          # facade over analysis_* submodules
├── analysis_*.py        # topology, boundary, corners, classification, etc.
├── solve.py             # facade (target: split into solve_* submodules)
├── debug.py
├── operators.py
└── console_debug.py

docs/
├── cftuv_architecture.md
└── cftuv_reference.md
```

## Documentation

| Document | When to read |
|----------|-------------|
| `AGENTS.md` | Always. Self-contained project context for any contributor or AI agent |
| `docs/cftuv_architecture.md` | When task requires pipeline, IR, or entity model understanding |
| `docs/cftuv_reference.md` | Lookup: topology invariants, runtime heuristics, regression checklist |

Start with `AGENTS.md`. For most tasks, it is sufficient on its own.

## Validation

Manual and debug-driven:

- Grease Pencil debug layers (Analyze toggle)
- Console diagnostics (Verbose Console toggle)
- Regression snapshots (`Save Regression Snapshot`)
- Scaffold vs UV validation output
- UV Editor inspection on production meshes
