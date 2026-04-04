# CFTUV Alignment / Drift Roadmap

This document is the dedicated roadmap for row/column consistency and accumulated drift handling.

It exists to keep these concerns out of the main frontier score refactor.

Chain-first strongest-frontier remains the core solve path.
Patch and Chain remain the only primary solve units.
Row/column alignment is a separate layer that may consume diagnostics after scaffold build,
but it must not become a hidden bonus/penalty inside frontier rank.

---

## 1. Scope

This roadmap covers:

- row / column cross-axis scatter between solved `H_FRAME` / `V_FRAME` chains;
- accumulated closure drift across non-tree closure seams;
- future conservative correction logic after frontier placement;
- future research branch for a pre-frontier lattice / landmark guide.

This roadmap does not cover:

- main frontier rank changes;
- rescue control-flow changes;
- patch-first or loop-sequential traversal;
- manual operations / artist overrides;
- replacing diagnostics with silent UV mutation.

---

## 2. Current Runtime State

The project already has explicit diagnostics for this track.

### Data already produced today

- `ScaffoldClosureSeamReport` in `cftuv/model.py`
- `ScaffoldFrameAlignmentReport` in `cftuv/model.py`
- `ScaffoldQuiltPlacement.frame_alignment_reports` in `cftuv/model.py`
- row / column grouping + scatter measurement in `cftuv/solve_diagnostics.py`
- reporting / anomaly surfacing in `cftuv/solve_reporting.py`, `cftuv/solve_report_metrics.py`, `cftuv/solve_report_anomalies.py`
- transfer-stage summary fields `frame_row_max_scatter` / `frame_column_max_scatter` in `cftuv/solve_transfer.py`

### Current pipeline boundary

Today the pipeline is:

1. frontier builds scaffold chain placements;
2. closure seam diagnostics are collected;
3. row / column alignment diagnostics are collected;
4. reports and transfer consume those diagnostics.

This means the project is already diagnostics-first for alignment.
What is missing is a dedicated correction layer, not more frontier score inputs.

---

## 3. Architectural Rules

The following rules are mandatory for any future implementation on this track.

1. Do not add row / column scatter as a normal frontier bonus or penalty.
2. Do not absorb closure drift logic into `FrontierRank`.
3. Do not create fake placed chains to simulate alignment.
4. Do not snap `FREE`-dominated regions as if they were stable frame landmarks.
5. Do not hide alignment provenance inside ordinary chain placement state.
6. Do not regress toward UV cycle sewing or patch-first solve.
7. Any correction pass must be diagnostics-driven and explicitly measurable.

---

## 4. Problem Split

Two related but distinct issues live here.

### 4.1. Closure drift

Symptoms:

- non-tree closure seams accumulate span mismatch;
- axis phase offset grows with tree distance;
- `FREE` bridges can amplify mismatch on the way to closure.

Primary evidence source:

- `ScaffoldClosureSeamReport`

### 4.2. Row / column scatter

Symptoms:

- geometrically collinear `H_FRAME` chains drift across UV rows;
- geometrically collinear `V_FRAME` chains drift across UV columns;
- closure-sensitive quilts may show both closure mismatch and class scatter at once.

Primary evidence source:

- `ScaffoldFrameAlignmentReport`

These problems may interact, but they must not be collapsed into one hidden scalar.

---

## 5. Preferred Future Runtime Track

The preferred implementation order is conservative and staged.

### Stage A. Diagnostics only

Already active.

Requirements:

- keep collecting closure seam reports;
- keep collecting row / column class scatter;
- keep surfacing anomaly metrics in snapshots and reports.

### Stage B. Read-only proposal layer

First implementation step for this roadmap should be read-only.

Suggested output:

- candidate alignment actions per quilt;
- eligibility flags;
- expected scatter reduction;
- risk notes for closure-sensitive groups.

This stage must not mutate UVs yet.

### Stage C. Conservative post-frontier correction

If Stage B is stable on production meshes, add a narrow correction pass.

Preferred placement in pipeline:

- after frontier scaffold finalization;
- after closure / frame diagnostics are available;
- before final transfer writes UVs.

This keeps correction separate from chain selection and anchor resolution.

### Stage D. Optional re-measure

After any correction pass:

- recompute closure seam reports;
- recompute frame alignment reports;
- compare before/after metrics in the same quilt.

If metrics do not improve or regress elsewhere, the correction is rejected.

---

## 6. Eligibility Rules For Future Correction

Future alignment correction should only target chains that satisfy all of the following.

- chain role is `H_FRAME` or `V_FRAME`;
- row / column class has at least two strong members;
- measured scatter is above tolerance and not just noise;
- correction does not cross an intentional closure cut blindly;
- patch is not unsupported conformal fallback;
- expected benefit is larger than the induced closure / seam risk.

By default, `FREE` chains stay outside the guarantee.
They may only move indirectly if a future correction system has an explicit provenance for that.

---

## 7. Research Branch: Pre-Frontier Lattice

This branch is explicitly secondary to the conservative post-frontier track.

Use terminology:

- `pre-frontier landmark alignment pass`
- `aligned frame lattice`

Rules for that branch:

- the lattice is temporary solve-time guidance, not a new persistent IR;
- lattice coordinates must keep separate provenance such as `lattice_anchor`;
- metric is role-aligned accumulated span, not raw endpoint projection;
- guarantees apply only to solved frame components, not `FREE`-dominated zones.

This branch should not be attempted until the post-frontier track has real production evidence.

---

## 8. Practical Entry Points

When implementation of this roadmap begins, start here.

### Diagnostics sources

- `_collect_quilt_closure_seam_reports()` in `cftuv/solve_diagnostics.py`
- `_collect_quilt_frame_alignment_reports()` in `cftuv/solve_diagnostics.py`

### Pipeline handoff area

- frontier finalization in `cftuv/solve_frontier.py`
- transfer/application in `cftuv/solve_transfer.py`

### Reporting feedback loop

- `cftuv/solve_reporting.py`
- `cftuv/solve_report_metrics.py`
- `cftuv/solve_report_anomalies.py`

Suggested first code module when this track becomes active:

- `cftuv/solve_alignment.py`

Its first responsibility should be proposal building and re-measure orchestration, not direct mutation.

---

## 9. Success Criteria

This roadmap is successful when future implementation can demonstrate:

- row / column groups are diagnosed explicitly and remain outside frontier rank;
- closure-sensitive quilts reduce measurable mismatch without patch-first regressions;
- collinear frame groups stop drifting across the cross-axis beyond tolerance;
- intentional closure cuts remain intentional;
- diagnostics and snapshots can explain both the problem and the correction result.

Canonical frame guarantee for any future alignment layer:

- `H_FRAME` chains in one solved row-class share the same UV row coordinate within epsilon;
- `V_FRAME` chains in one solved column-class share the same UV column coordinate within epsilon;
- `FREE`-dominated zones remain outside this guarantee by design.

---

## 10. Verification Checklist For Future Work

Before accepting any alignment implementation:

- [ ] Grease Pencil / debug visualization still works
- [ ] closure seam mismatch is not worse
- [ ] max row scatter is not worse
- [ ] max column scatter is not worse
- [ ] rescue telemetry does not regress unexpectedly
- [ ] unsupported / conformal fallback count does not grow unexpectedly
- [ ] regression snapshot explains the before/after delta

If any of these fail, the correction layer is wrong or under-instrumented.

---

## 11. Cross-Axis Consensus Fix (implemented)

### Problem

`_build_frame_chain_from_one_end()` copies full UV from XP anchor including cross-axis.
Direction is snapped to pure axis but start_point keeps the neighbor's cross-axis error.
On T-shaped wall niches this injects ~36% span drift into clean chains from distorted neighbors.

### Root cause

`_cf_find_anchors()` creates `ChainAnchor.uv` from `point_registry` without cross-axis filtering.
For one-end CROSS_PATCH frame anchors the cross-axis is blind inheritance from neighbor patch.

### Fix

Frontier-level override in `_cf_place_chain()` (`cftuv/frontier_place.py`):
for one-end XP H/V placement, cross-axis component is replaced with **trusted frame group consensus**
(weighted average of already-placed same-group chains). Anchor UV unchanged if no consensus exists.

Trusted = `anchor_count >= 2` or `primary_anchor_kind != CROSS_PATCH`.
Weight = chain UV length (scaled 3D edge sum), not point count.

### Key files

- `cftuv/frontier_place.py` — `_frame_group_cross_axis_consensus()`, consensus override in `_cf_place_chain()`
- `cftuv/model.py` — `ScaffoldChainPlacement.primary_anchor_kind` field
- `cftuv/solve_records.py` — `frame_row_class_key()`, `frame_column_class_key()` (shared helpers)
- `cftuv/frontier_eval.py`, `cftuv/frontier_rescue.py` — propagate `primary_anchor_kind`, pass `placed_chains_map`/`graph`

### Known limitations

- If the problem chain enters its frame group first, no consensus exists yet — anchor UV used as-is.
- Post-hoc correction (Variant A: frame snap, Variant B: subtree translation) was prototyped
  in `drift_correction.py` / `frontier_finalize.py` but removed as insufficient for closure-sensitive cases.
  The approach is documented below for reference if needed in the future.

### Rejected approaches

- **Post-hoc frame snap** (Variant A): shift cross-axis after placement. All groups were closure_sensitive on target mesh — zero proposals generated.
- **Subtree rigid translation** (Variant B): move downstream patch subtree. Redistributes error between closure seam and tree edge rather than eliminating it.
- **3D-based cross-axis override**: compute cross-axis from 3D geometry. Breaks anchor seam connectivity contract.
