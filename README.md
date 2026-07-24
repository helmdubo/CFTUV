# CFTUV

CFTUV (Constraint-First Trim UV) is a Blender addon for semi-procedural UV
unwrapping of architectural hard-surface meshes for trim sheet / tile workflows.

The core solve rule is **chain-first strongest-frontier**: scaffold grows chain
by chain from a global frontier pool across the whole quilt.

## Envelope v1 contributors

Machine-readable document status: `CURRENT_ONBOARDING_AUTHORITY`.

Do not infer the current Envelope task from this checkout alone. Read, in order:

1. `docs/architecture_status.json` from
   `codex/base-00-canonical-integration`;
2. `AGENTS.md`;
3. `docs/envelope_engine_start_here.md`;
4. the `current_card_path` declared by the live status manifest.

The status manifest supplies the immutable `accepted_integration_sha`, active
blocker, accepted handoffs and current card. The current engine path is:

```text
host facts/request
→ Blender-free kernel/
→ exact coverage and approved interactions
→ ownership / semantic arrangement
→ GeometryBatch
→ optional host materialization
```

The top-level `cftuv` addon and the Blender-free Envelope kernel coexist, but
their geometry backends are not interchangeable.

## Target

- Blender 3.0+
- Python 3.10+
- Hard-surface environment production meshes
- Trim sheet / tile UV workflows

## Existing decal-runtime dependency

Machine-readable section status: `SCOPED_LEGACY_RUNTIME`.

The existing patch-bounded Decal Seams backend uses `pyvoronoi` 1.2.8+ (Boost
segment Voronoi bindings). This applies only to the legacy/in-place Blender
decal producer. It is not a dependency, semantic oracle or implementation
target for the new Envelope kernel.

Install it into Blender's Python environment only when using that existing
interactive seam-decal runtime:

```powershell
& "<path-to-Blender>\4.3\python\bin\python.exe" -m pip install "pyvoronoi>=1.2.8"
```

Core PatchGraph analysis, UV solve and the new Envelope kernel remain free of
this dependency. If the wheel is unavailable or a selected legacy decal
component is unsupported, Decal Seams fails with a named reason.

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
├── decals.py            # scoped legacy decal runtime
├── decal_charts.py      # immutable IR для intrinsic strip charts
├── decal_voronoi.py     # scoped legacy `pyvoronoi` backend
├── operators.py
└── console_debug.py

docs/
├── cftuv_architecture.md
└── cftuv_reference.md
```

## Documentation

| Document | When to read |
|----------|-------------|
| `docs/architecture_status.json` | Live machine-readable Envelope control state; read from the canonical control ref |
| `AGENTS.md` | Current contributor constraints and Envelope authority bootstrap |
| `docs/envelope_engine_start_here.md` | Current human-readable Envelope authority and architecture map |
| `docs/agent_execution/envelope_v1/01_GLOBAL_CANON.md` | Accepted Envelope v1 global canon |
| `docs/agent_execution/envelope_v1/02_AGENT_PROTOCOL.md` | Branch, allowlist, gate and handoff protocol |
| `docs/cftuv_architecture.md` | When task requires pipeline, IR, or entity model understanding |
| `docs/cftuv_reference.md` | Lookup: topology invariants, runtime heuristics, regression checklist |

Non-Envelope tasks start with `AGENTS.md`. Envelope tasks must use the
four-step live-control order above.

## Validation

Manual and debug-driven:

- Grease Pencil debug layers (Analyze toggle)
- Console diagnostics (Verbose Console toggle)
- Regression snapshots (`Save Regression Snapshot`)
- Scaffold vs UV validation output
- UV Editor inspection on production meshes
