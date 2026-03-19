# CFTUV

CFTUV (Constraint-First Trim UV) is a Blender addon for semi-procedural UV
unwrapping of architectural hard-surface meshes for trim sheet / tile workflows.

The core solve rule is:

**chain-first strongest-frontier**

The scaffold grows chain by chain from a global frontier pool across the whole
quilt. It must not drift into patch-first or loop-sequential placement.

## Target

- Blender 3.0+
- Python 3.10+
- hard-surface environment production meshes
- trim sheet / tile UV workflows

## Repo Layout

```text
cftuv/
|- __init__.py
|- constants.py
|- model.py
|- analysis.py
|- solve.py
|- debug.py
`- operators.py

docs/
|- README.md
|- cftuv_regression_checklist.md
|- cftuv_architecture_v2.0.md
|- cftuv_entity_model_and_control_plan.md
|- cftuv_refactor_roadmap_for_agents.md
`- cftuv_runtime_notes.md
```

Module roles:

- `model.py`: enums, topology IR, solve IR, settings dataclasses
- `analysis.py`: BMesh -> PatchGraph
- `solve.py`: planning, frontier scaffold build, UV transfer, validation
- `debug.py`: Grease Pencil visualization and replay tooling
- `operators.py`: Blender UI wrappers

## Read Order

If you need to understand or modify the project, read in this order:

1. [docs/cftuv_architecture_v2.0.md](/D:/_mimirhead/website/CFTUV/docs/cftuv_architecture_v2.0.md)
2. [docs/cftuv_entity_model_and_control_plan.md](/D:/_mimirhead/website/CFTUV/docs/cftuv_entity_model_and_control_plan.md)
   current control document for entity layering, `FrameRun` / `Junction`
   boundaries, and the small cleanup plan
3. [docs/cftuv_refactor_roadmap_for_agents.md](/D:/_mimirhead/website/CFTUV/docs/cftuv_refactor_roadmap_for_agents.md)
4. [docs/cftuv_runtime_notes.md](/D:/_mimirhead/website/CFTUV/docs/cftuv_runtime_notes.md)
   only for active runtime stabilization or lattice research tasks
5. [docs/cftuv_regression_checklist.md](/D:/_mimirhead/website/CFTUV/docs/cftuv_regression_checklist.md)
   for Phase 0 regression baselines and manual snapshot review

For contributor and agent guardrails, see
[AGENTS.md](/D:/_mimirhead/website/CFTUV/AGENTS.md).

## Current Focus

The current runtime track is intentionally narrow:

- stabilize chain-first frontier behavior on production meshes
- improve frame alignment and closure behavior without redesigning the solver
- keep unreachable tree-connected patches from falling into conformal fallback
- develop diagnostics-first research for the pre-frontier `aligned frame lattice`

## Validation

The project currently relies on manual and debug-driven verification:

- Analyze / Grease Pencil debug layers
- console diagnostics
- regression snapshots via `Save Regression Snapshot`
- scaffold vs UV validation output
- UV Editor inspection on production meshes
