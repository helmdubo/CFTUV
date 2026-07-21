# S2b gate report — DecalSessionController ownership cut

Date: 2026-07-21  
Baseline: `261f23800e540b04392add76c3501cb1198eeb96`  
Implementation: `42fa2d9`, `2f10d97`  
Test-boundary follow-up: `888be31`

## Result

S2b is implemented as an ownership-only cut. `DecalSessionController` is now
the sole owner of the captured request, captured `MetricContext`, compiled
plan, current and last-valid evaluations, controlled-recompile boundary,
confirmability, `PreviewFailurePolicy.CLEAR`, and terminal cleanup. The modal
operator retains Blender event translation and display resources only.

The decal engine geometry semantics are unchanged by this slice.

## Ownership boundary

- `CapturedDecalRequest` freezes source/mode/analysis/selection/settings and
  the `MetricContext` captured before analysis.
- Drag updates convert world settings with that captured metric; they do not
  reread `matrix_world`.
- `DecalSessionController.begin/preview/confirm/cancel` owns the state
  transitions, last-valid result, pending controlled recompile, and cleanup.
- Mouse movement evaluates the compiled plan only. Compilation is permitted
  at session start and at the explicit confirm-time controlled-recompile
  boundary.
- Both GPU and mesh preview adapters receive the same CLEAR decision.
- Confirm and cancel close the session and release preview/GPU/header/rail
  resources idempotently.
- World-space public engine entry points remain compatibility wrappers;
  production modal routing uses typed local-settings entry points.

## Strict no-op differential

The first attempted pair was discarded because the live `E:\testscene.blend`
was saved between the two runs. The accepted pair ran the baseline and S2b
code against one frozen copy with SHA-256
`11a4fca3442189e6e927d99dc608845247ce5bd6f51531b7300181e02abd3a44`.

| Check | Result |
|---|---:|
| Field objects | 6 |
| Width evaluations (0.4 and 0.8) | 12 |
| Plan digests / routing / counters | identical |
| GeometryBatch serialization / topology / policy counters | identical |
| `semantic_equal` | `true` |
| `all_preview_confirm_equal` | `true` |
| Blend saved by harness | `false` |

Receipts:

- `artifacts/decal_s2b_before_field.json`
- `artifacts/decal_s2b_after_field.json`
- `artifacts/decal_s2b_comparison_field.json`

The comparison includes the complete semantic receipt while ignoring only
provenance fields (`phase`, `source_commit`, and input path).

## Verification

- `python -m compileall -q cftuv tests` — PASS.
- `python -m pytest -q -m "not atlas_frozen"` —
  `492 passed, 3 skipped, 4 deselected`.
- `git diff --check` — PASS.
- Blender 4.3.2 preview-object lifecycle smoke — `[CFTUV][A7] PASS`.
- GPU operator tests cover session-local evaluation, empty/CLEAR, adapter
  fallback, evaluator failure propagation, confirm without GPU, and
  idempotent stop.

The old `artifacts/cftuv_decal_verification.blend` is not an S2b gate input:
it already fails on the accepted baseline with
`BEVEL_JOIN_SIDE_AMBIGUOUS`, so it cannot distinguish this ownership cut.

## Risks

1. `CapturedDecalRequest` intentionally retains a Blender object reference,
   not a deep copy of mesh data. The modal operator blocks normal edits while
   active; out-of-band deletion or mutation still fails at the Blender port.
2. Compatibility world-space wrappers remain callable by saved scripts. The
   production operator no longer uses them, but removing them would be a
   separate API migration rather than a no-op ownership cut.
3. The differential proves engine output equivalence, not interactive feel.
   The user's independent MITER/BEVEL field check remains useful but is not an
   S2b blocker by instruction.
4. The stale verification blend should be regenerated before it is reused as
   a future gate; its baseline failure is preserved rather than masked.

## Special opinion

The next step should remain the short intrinsic-width backend design gate
before R3. S-WF0 data justify evaluating FMM as the curvature-width candidate
and excluding Heat as authority; neither backend choice nor R3 production
routing belongs in S2b.

`DOMAIN_BUDGET_EXCEEDED` is still recognized at the Blender boundary through
a named reason. Replacing string recognition with a typed capacity outcome is
a worthwhile later cleanup, but doing it here would broaden a proven no-op
ownership slice.
