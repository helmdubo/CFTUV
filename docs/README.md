# Docs Overview

Only current project documents should live in `docs/`.

## Read Order

1. `docs/cftuv_architecture_v2.0.md`
   Main project baseline. Describes the current addon structure, IR, solve
   pipeline, architectural invariants, and active system boundaries.

2. `docs/cftuv_refactor_roadmap_for_agents.md`
   Practical companion document. Describes structural findings, safe execution
   order, runtime track priorities, and research branches.

3. `docs/cftuv_runtime_notes.md`
   Read only when the task is inside the active runtime stabilization track or
   in diagnostics / research work around the `aligned frame lattice`.

## Notes

- Start with architecture, then roadmap.
- Runtime notes are not the design baseline. They store sprint-level runtime
  facts, production-case lessons, and active heuristics.
- `AGENTS.md` and `CLAUDE.md` continue to treat
  `docs/cftuv_architecture_v2.0.md` as the main project document.
