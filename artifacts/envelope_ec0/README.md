# EC0 envelope semantics corpus

Active status: `CORRECTION_CANDIDATE_READY_FOR_EXTERNAL_REVIEW`.

The active EC0 corpus is `corpus/`. It is text/JSON-only and contains:

- `manifest.json` — inventory and global contracts;
- `decision_record.json` — accepted/rejected user decisions;
- `cases/` — 16 canonical cases plus 7 pivot/correction cases;
- `schema/case.schema.json` — structural JSON Schema;
- `policies/boundary_limited_propagation.json` — AM8 capability policy;
- `matrices/` — metamorphic verdicts;
- `tools/validate_envelope_ec0.py` — reproducible semantic validator.

Each case has one authoritative graph with three separate records:
`analysis_snapshot`, `decal_request`, and `expected_compiled_plan`. Hand-maintained
duplicate skeleton/envelope/region graphs are forbidden.

## Presentation artifact policy

Do not create SVG/PNG semantic sheets, diagrams, contact sheets, slides, or
interactive HTML for EC0 review or acceptance. Prefer prose, code, JSON,
schemas, and validator output.

Blender viewport, UV Editor, and debug-overlay screenshots are allowed as
diagnostic runtime evidence. They are not SemanticAuthority and do not replace
JSON/prose assertions.

The former v1/v2 YAML and visual corpus was rejected by external review at
commit `b16be81` and removed from the active workspace. Git history preserves
it as audit evidence. It must not be copied forward or silently interpreted as
the current schema.

Run from the repository root:

```text
python tools/validate_envelope_ec0.py
```

EC1 remains closed until the validator/CI is green and the user explicitly
accepts the corrected prose/JSON semantics.
