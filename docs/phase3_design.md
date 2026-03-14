# Phase 3 Design
## Current Status and Handoff, March 2026

---

## Purpose

Этот документ больше не описывает "что надо когда-нибудь сделать".
Он фиксирует:

- что в Phase 3 уже реально внедрено;
- какие защитные правки были добавлены поверх frontier;
- какие долги и открытые вопросы остались для следующей сессии.

Главное:

**активный solve path уже chain-first strongest-frontier.**

---

## What Phase 3 Replaced

Phase 3 заменил активный patch-first scaffold builder.

Текущий construction entry point:

- `build_root_scaffold_map()`
  - вызывает `build_quilt_scaffold_chain_frontier()`

То есть новый runtime path уже не строит quilt patch-by-patch как основной режим.

---

## Implemented Core

### 1. Chain-first frontier builder

Реализовано:

- seed chain selection внутри root patch;
- единый frontier pool на весь quilt;
- step-by-step chain scoring;
- placement по anchors;
- `build_order` на уровне quilt.

### 2. Quilt-local scoring

Исправлено влияние чужих patch вне текущего quilt:

- caps или другие внешние patch не должны портить seed/scoring текущего wall quilt;
- semantic bonus применяется только к patch внутри `quilt_plan.solved_patch_ids`.

### 3. Anchor provenance

Frontier различает:

- `same_patch`
- `cross_patch`

Это используется и в scoring, и в closure guards.

### 4. Early closure guards

Добавлены защиты от ложного замыкания patch:

- dual-anchor closure через два `cross_patch` anchors запрещен;
- для H/V chain введены `axis_mismatch` и `span_mismatch` проверки;
- при сомнительном dual-anchor candidate frontier падает обратно в bootstrap от одного конца.

Это было нужно для tube / cylinder / pseudo-cube кейсов.

### 5. Patch-type separation

Planning не строит propagation между разными `PatchType`.

Практический эффект:

- `WALL` quilts не должны тащить к себе `FLOOR` caps;
- caps теперь решаются отдельными quilts, если не принадлежат тому же patch type.

### 6. Mesh preflight gate

Перед solve path теперь есть блокирующая mesh validation.

Она останавливает pipeline на:

- duplicate faces;
- non-manifold edges;
- degenerate faces.

Это убрало класс ложных "UV solver bug", когда источник проблемы был в самой mesh topology.

### 7. Outer geometric fallback in analysis

Для isolated `OUTER` loop без meaningful neighbor split реализован fallback по
геометрическим corners:

- loop режется по corner turns;
- derived spans повторно классифицируются в `H_FRAME` / `V_FRAME` / `FREE`;
- fallback принимается только если split получился осмысленным.

`HOLE` loops в этот fallback специально не включаются.

### 8. Better frame-role classification

Классификация chain роли больше не основана только на extents:

- учитывается waviness;
- chord deviation;
- polyline vs chord;
- turn budget.

Это уменьшает ложные `H_FRAME` / `V_FRAME` на кривых chains.

### 9. UV transfer stabilization

Сделаны важные fixes в target resolution:

- coverage расширен на patch faces для уникальных vertices;
- `SEAM_SELF` обрабатывается отдельно;
- исчезли ложные конфликты и распад patch на острова из-за transfer layer.

### 10. Conformal stage is no longer global-only

Conformal теперь запускается не одним глобальным вызовом на все quilts.

Сейчас:

- supported patches unwrap-ятся per-patch;
- unsupported patches получают отдельный fallback conformal;
- islands раскладываются по cursor offset.

---

## Current Runtime Model

### Planning

Planning layer уже стабильно работает как upstream solve stage:

- `build_solver_graph()`
- `plan_solve_phase1()`

Он решает:

- patch certainty;
- root selection;
- attachment candidates;
- quilt grouping;
- propagation threshold logic.

### Construction

Construction layer делает:

1. seed placement;
2. frontier expansion;
3. envelope grouping обратно в per-patch placements;
4. UV transfer;
5. validation;
6. conformal completion.

---

## Current Scaffold Model

Phase 3 envelope model реализован частично.

### What already exists

- `ScaffoldQuiltPlacement.build_order`
- `ScaffoldPatchPlacement.status`
- `ScaffoldPatchPlacement.dependency_patches`
- `ScaffoldPatchPlacement.unplaced_chain_indices`
- bbox / closure data / notes

### What is still missing

- persistent `anchor_registry` внутри `ScaffoldMap`
- полноценный rebuild graph на его основе
- rich replay/debug по build order

Иными словами:

Phase 3 уже вышла из стадии "идея", но еще не дошла до fully rebuildable map platform.

---

## Known Transitional Debt

Это не случайные баги, а известные unfinished parts:

1. Solve dataclasses все еще живут в `solve.py`, не в `model.py`.
2. `ScaffoldSegment` все еще присутствует в `model.py` как legacy artifact.
3. Explicit `ContinuationEdge` пока нет.
4. `format_root_scaffold_report()` еще не отражает весь frontier state.
5. Patch envelope еще не является самостоятельным editable persistent объектом.

---

## Rules Added During Stabilization

### Cross-patch closure rule

Нельзя позволять новому patch входить в quilt сразу через оба конца прошлого patch,
если оба anchors пришли только извне.

Практическая формулировка:

- dual-anchor `cross_patch + cross_patch` closure запрещен;
- candidate должен сначала набрать локальную continuity.

### Patch-type rule

Propagation между разными `PatchType` запрещен на planning stage.

### Outer-only fallback rule

Geometric fallback split применим только к `OUTER`.

### Preflight rule

Bitая topology не доходит до solve.

---

## What Worked Best In Practice

На текущем состоянии кода стабильно помогли именно эти решения:

- quilt-local scoring;
- anchor provenance;
- prevent patch wrap;
- axis/span guard;
- per-patch conformal;
- mesh preflight;
- outer geometric fallback.

Если новый агент будет чинить локальный баг, эти вещи не надо откатывать.

---

## Open Problems

Ниже не "всё сломано", а текущие честные остаточные проблемы.

### 1. Isolated single-patch conformal overconstraint

Изолированный patch с outer frame и сложным interior может визуально выглядеть так,
будто `Conformal` "не сработал", хотя оператор реально вызывается.

Основной текущий suspect:

- слишком жесткий pinning для full `H_FRAME` / `V_FRAME`.

Это особенно заметно на одиночной стене с оконными проемами.

### 2. Ornamental / curved walls

Есть класс wall patch, где часть внешнего контура геометрически кривая, но topology
все равно похожа на frame boundary.

Там простого `H/V/FREE` иногда уже недостаточно.

Следующий правильный уровень решения:

- либо profile-preserving frame placement;
- либо derived sub-spans внутри формально frame-like chain.

### 3. Debug depth

Сейчас лог frontier хороший, но persistent debug model еще беднее, чем хотелось бы:

- нет replay-ready anchor registry;
- нет explicit continuation visualization;
- build history читается из console, а не из полноценного map state.

---

## Recommended Next Work

Следующему агенту лучше идти в таком порядке.

### Priority 1

Разобраться с isolated single-patch case:

- pin policy для `H_FRAME` / `V_FRAME`;
- когда frame надо пинить целиком, а когда только anchors/corners;
- как не ломать multi-patch seam continuity.

### Priority 2

Улучшить debug/reporting:

- richer build-order report;
- provenance and closure notes в scaffold report;
- явное указание unsupported reasons.

### Priority 3

Довести envelope model:

- persistent anchor registry;
- local rebuild contract;
- dependency-driven partial re-solve.

### Priority 4

Только после стабилизации выше:

- думать про explicit `ContinuationEdge` и Phase 4 continuity layer.

---

## What Should Not Be Reintroduced

Не нужно возвращать:

- patch-first build как активный path;
- global conformal на все patches сразу;
- смешивание `WALL` и `FLOOR` в один propagation quilt;
- разрешение dual cross-patch closure без guard;
- reliance на legacy monolith как runtime source.

Legacy `Hotspot_UV_v2_5_19.py` полезен только как reference для isolated outer-loop
segmentation по basis и corner turns.

---

## Short Status Summary

Phase 3 сейчас — это **рабочий frontier solve path с рядом production fixes, но еще
не финальная rebuildable platform**.

Главный актив:

- frontier уже настоящий;
- tube / pseudo-cube / cap separation / preflight / UV target coverage уже стабилизированы.

Главный незакрытый участок:

- isolated patch behavior и pinning strategy после geometric outer split.

