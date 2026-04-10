# CFTUV Cleanup Decisions

## Applied
- Split `solve_records.py` into themed siblings:
  - `cftuv/solve_records_common.py`
  - `cftuv/solve_records_domain.py`
  - `cftuv/solve_records_frontier.py`
  - `cftuv/solve_records_transfer.py`
  - `cftuv/solve_records_telemetry.py`
- Kept `cftuv/solve_records.py` as a compatibility facade so existing imports continue to work.
- Added `cftuv/analysis_shape_support.py` and moved shape orchestration there:
  - fingerprint detection
  - shape classification dispatch
  - BAND support assembly
  - reserved no-op hooks for `CYLINDER` and `CABLE`
- Added `cftuv/frontier_bootstrap.py` and moved frontier launch/bootstrap responsibilities there.
- Refactored `cftuv/frontier_state.py` around:
  - immutable `FrontierLaunchContext`
  - mutable `FrontierRuntimeState`
  - compatibility wrapper `FrontierRuntimePolicy`
- Added `cftuv/operators_pipeline.py` for preflight / solve-state / report helpers.
- Added `cftuv/operators_session.py` for replay / debug lifecycle orchestration.
- Physically removed the extracted helper block from `cftuv/operators.py`; the file now keeps Blender classes, panel drawing, and registration only.
- Removed disabled legacy addon UI paths:
  - `unwrap_faces`
  - `manual_dock`
  - `select_similar`
  - `stack_similar`
- Routed business-logic console noise away from raw `print()` into `trace_console()` across analysis / planning / frontier / transfer modules.
- Marked `cftuv/band_operator.py` as explicit reference-only code.

## Core Kept As-Is
- `PatchGraph` / `ScaffoldMap` boundary in `cftuv/model.py`
- Chain-first strongest-frontier runtime principle
- `solve_pin_policy.py` as the single source of truth for pin decisions
- `debug.py` as the only Grease Pencil compatibility layer
- Diagnostics / telemetry / reporting modules as explicit observability boundaries

## Consciously Deferred
- No trim abstraction work
- No alignment pass work
- No junction enrichment work
- No manual/reverse operator implementation
- No semantic widening of public `PatchShapeClass`
- No solver redesign or patch-first fallback path

## Reference-Only / Archived
- `cftuv/band_operator.py` is retained only as implementation reference.
- `Hotspot_UV_v2_5_19.py` remains legacy-only and outside active architecture.

## Residual Cleanup Debt
- `cftuv/analysis_derived.py`, `cftuv/solve_frontier.py`, `cftuv/solve_transfer.py`, and `cftuv/frontier_state.py` remain the main refactor hotspots for future structure-only passes.
