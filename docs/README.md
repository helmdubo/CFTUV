# Docs Overview

Only current project documents should live in `docs/`.

## Read Order

1. `docs/cftuv_architecture_v2.0.md`
   Main project baseline. Describes the current addon structure, IR, solve
   pipeline, architectural invariants, and active system boundaries.

2. `docs/cftuv_entity_model_and_control_plan.md`
   Current control document. Fixes the entity model, layering
   (`intrinsic/contextual/derived`), and the allowed scope of `FrameRun` /
   `Junction` work.

3. `docs/cftuv_refactor_roadmap_for_agents.md`
   Practical companion document. Describes structural findings, safe execution
   order, runtime track priorities, and research branches.

4. `docs/cftuv_runtime_notes.md`
   Read only when the task is inside the active runtime stabilization track or
   in diagnostics / research work around the `aligned frame lattice`.

5. `docs/cftuv_regression_checklist.md`
   Phase 0 manual checklist for agreed regression meshes and snapshot review.

## What Each Doc Answers

- `cftuv_architecture_v2.0.md`
  What exists right now? What are the real module boundaries, IRs, and runtime
  invariants?

- `cftuv_entity_model_and_control_plan.md`
  What is the current entity hierarchy? What is primary vs composite vs
  analysis-derived? What small refactor is allowed now?

- `cftuv_refactor_roadmap_for_agents.md`
  In what order is it safe to refactor? Which structural debts are broader than
  the current cleanup?

- `cftuv_runtime_notes.md`
  What are the current runtime heuristics, production-case lessons, and active
  stabilization boundaries?

- `cftuv_regression_checklist.md`
  Which manual regression cases and snapshots should be checked after changes?

## Notes

- Start with architecture, then control plan, then roadmap.
- Runtime notes are not the design baseline. They store sprint-level runtime
  facts, production-case lessons, and active heuristics.
- `README.md` is the repo entry point.
- `docs/README.md` is the docs map.
- `AGENTS.md` continues to treat `docs/cftuv_architecture_v2.0.md` as the main
  baseline document, with the control plan as the next mandatory companion.
