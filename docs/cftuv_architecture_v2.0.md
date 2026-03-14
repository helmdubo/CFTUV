# CFTUV Architecture v2.0
## Actual Implementation State, March 2026

---

## Purpose

Этот документ описывает не идеальную целевую архитектуру, а фактическое состояние
рефакторинга после серии правок Phase 3 и стабилизации `analysis.py` / `solve.py`.

Главная идея по-прежнему одна:

**не patch-first и не loop-first, а chain-first strongest-frontier.**

Scaffold растет chain за chain из общего frontier-пула quilt. Все, что frontier
не взял, добирается поздним `Conformal`.

---

## Modules

```text
cftuv/
├── __init__.py
├── constants.py
├── model.py
├── analysis.py
├── solve.py
├── debug.py
└── operators.py
```

Роли модулей:

- `model.py` — PatchGraph IR, enums, topology dataclasses, `UVSettings`, mesh preflight report.
- `analysis.py` — BMesh -> PatchGraph, basis, loops, chains, corners, seam edges.
- `solve.py` — planning (`SolverGraph`, `SolvePlan`), chain-frontier scaffold builder, UV transfer, validation, conformal stage.
- `debug.py` — Grease Pencil и console visualization из PatchGraph.
- `operators.py` — Blender UI wrappers, preflight gate, operator orchestration.
- `constants.py` — thresholds, sentinels, shared config.

Legacy scripts вида `Hotspot_UV_v2_5_xx.py` не являются runtime-частью аддона.
Их можно читать только как reference для старых идей.

---

## Core Principle

### Chain-first frontier

Scaffold строится из `BoundaryChain`, а не из patch целиком.

Алгоритм:

1. Planning layer выбирает quilts и root patches.
2. Construction layer выбирает seed chain root patch.
3. Дальше каждый шаг берет strongest доступный chain из общего quilt frontier.
4. Chain считается доступным, если у него уже есть хотя бы один anchor.
5. После frontier оставшиеся вершины решаются `Conformal`.

### Quilt-local solve

Scoring и propagation теперь должны быть quilt-local:

- соседние patch вне текущего quilt не должны влиять на seed/scoring;
- propagation между разными `PatchType` не строится;
- `WALL`, `FLOOR`, `SLOPE` по умолчанию оказываются в разных solve-группах.

---

## Data Flow

```text
operators.py
  -> validate_solver_input_mesh()
  -> build_patch_graph()
  -> build_solver_graph()
  -> plan_solve_phase1()
  -> build_root_scaffold_map()
  -> _apply_patch_scaffold_to_uv()
  -> bpy.ops.uv.unwrap(method='CONFORMAL')
```

Параллельный debug path:

```text
operators.py -> build_patch_graph() -> debug.py -> Grease Pencil / Console
```

---

## Main IR Layers

## PatchGraph

Центральный topology IR.

Содержит:

- `PatchNode`
- `BoundaryLoop`
- `BoundaryChain`
- `BoundaryCorner`
- `SeamEdge`

Ключевой принцип: `PatchGraph` хранит только индексы и сериализованную геометрию,
но не держит `BMFace` / `BMEdge` ссылки.

## Solve IR

Текущее solve-представление все еще частично transitional.

Сейчас в коде:

- `ScaffoldMap`
- `ScaffoldQuiltPlacement`
- `ScaffoldPatchPlacement`
- `ScaffoldChainPlacement`
- `ScaffoldPointKey`

живут в `solve.py`, а не в `model.py`.

Это технический долг, но это и есть реальное состояние проекта на текущий момент.
Новый агент не должен бездумно переносить эти dataclasses в `model.py` в рамках
локального багафикса: это отдельный рефакторинг.

---

## Domain Model

### PatchNode

Один patch после flood fill.

Ключевые поля:

- `patch_id`
- `face_indices`
- `normal`
- `patch_type`
- `world_facing`
- `basis_u`
- `basis_v`
- `boundary_loops`

### BoundaryLoop

Замкнутый boundary контур patch.

Типы:

- `OUTER`
- `HOLE`

### BoundaryChain

Единица solve placement.

Ключевые поля:

- `vert_indices`
- `vert_cos`
- `edge_indices`
- `side_face_indices`
- `neighbor_patch_id`
- `frame_role`
- `start_loop_index`
- `end_loop_index`
- `start_corner_index`
- `end_corner_index`

### BoundaryCorner

Стык двух chains внутри loop.

Ключевые поля:

- `loop_vert_index`
- `vert_index`
- `turn_angle_deg`
- `prev_chain_index`
- `next_chain_index`
- `prev_role`
- `next_role`

### FrameRole

Локальная роль chain в basis patch:

- `H_FRAME`
- `V_FRAME`
- `FREE`

### ChainNeighborKind

Тип соседа chain:

- `PATCH`
- `MESH_BORDER`
- `SEAM_SELF`

---

## analysis.py

`analysis.py` остается единственным topology-builder модулем.

### Pipeline

1. Flood fill faces в patches по seam.
2. Классификация patch в `WALL` / `FLOOR` / `SLOPE`.
3. Построение локального basis.
4. Трассировка boundary loops.
5. OUTER/HOLE classification через временный UV unwrap.
6. Split loop на chains по смене соседа.
7. Fallback geometric split для special case isolated `OUTER`.
8. Classify chain role (`H_FRAME`, `V_FRAME`, `FREE`).
9. Build corners.
10. Build seam edges.

### Important recent behavior

#### 1. Outer-loop geometric fallback

Если `OUTER` loop после neighbor-split схлопывается в один `FREE` chain,
`analysis.py` пытается геометрически нарезать его по corner turns.

Это нужно для isolated wall patches без соседства:

- внешний прямоугольный контур перестает быть одним большим `FREE`;
- появляются derived `H_FRAME` / `V_FRAME` chains;
- `HOLE` loops в этот fallback не участвуют.

#### 2. Frame classification is not bbox-only anymore

`_classify_chain_frame_role()` теперь учитывает не только extents в basis,
но и форму chain:

- waviness по ортогональной оси;
- polyline vs chord;
- chord deviation;
- turn budget.

Это защищает от ложного `H_FRAME` / `V_FRAME` для волнистых или изогнутых chains.

#### 3. Corners are already available for FREE fallback

Corner turn angle считается в `analysis.py` и служит основой geometric split.

---

## solve.py

`solve.py` сейчас делится на два слоя.

### Planning layer

Сущности:

- `PatchCertainty`
- `AttachmentCandidate`
- `SolverGraph`
- `QuiltPlan`
- `SolvePlan`

Planning делает:

- per-patch solvability gate;
- root scoring;
- attachment scoring;
- solve grouping по connected components;
- patch-type compatibility gate.

### Construction layer

Основной entry point:

- `build_root_scaffold_map()`

Он использует только:

- `build_quilt_scaffold_chain_frontier()`

Старый patch-first path больше не является активным construction path.

### Chain frontier

Ключевые свойства текущей реализации:

- seed chain выбирается внутри root patch;
- scoring учитывает роль chain, anchors и quilt-local semantic context;
- anchor provenance различается:
  - `same_patch`
  - `cross_patch`
- раннее замыкание patch через два `cross_patch` anchors запрещается;
- для H/V dual-anchor placement есть axis/span safety check;
- rejected candidate не должен маскироваться под placed chain.

### Patch wrap guard

Отдельное правило frontier:

если оба anchors пришли только cross-patch, chain не должен замыкать новый patch
на обоих концах сразу. Это введено для tube/cylinder/pseudo-cube кейсов.

### Envelope state

После frontier chains группируются обратно в `ScaffoldPatchPlacement`.

Текущие поля envelope-слоя:

- `status`
- `dependency_patches`
- `unplaced_chain_indices`
- `closure_error`
- `closure_valid`
- `bbox_min`
- `bbox_max`

Важно:

- `build_order` уже есть на уровне `ScaffoldQuiltPlacement`;
- полноценного persistent `anchor_registry` внутри `ScaffoldMap` пока нет;
- dependency tracking реализован через provenance собранных chains.

Это значит, что архитектурная идея envelope уже частично реализована, но еще не
доведена до fully rebuildable persistent model.

---

## UV Transfer and Conformal

### UV transfer

Scaffold сначала пишется в UV layer, потом запускается `Conformal`.

Ключевые детали текущей реализации:

- `_resolve_scaffold_uv_targets()` работает side-aware;
- для уникальных patch vertices coverage расширяется на все loops patch faces;
- `SEAM_SELF` обрабатывается отдельно, чтобы не схлопывать разные UV стороны;
- `validate_scaffold_uv_transfer()` сравнивает scaffold intent с реальными UV loops.

### Pin policy

На текущем коде:

- `H_FRAME` / `V_FRAME` pinятся целиком;
- `FREE` chains pinят только endpoints;
- после `Conformal`, если `keep_pins=False`, pins снимаются.

### Conformal strategy

Текущая реализация больше не делает один глобальный unwrap на все quilts.

Сейчас:

- supported patches unwrap-ятся per-patch внутри quilt;
- unsupported patches получают отдельный fallback conformal pass;
- quilt islands раскладываются по `x_cursor`, чтобы не лежать друг на друге.

### Important implication

Если в логах есть строка `Patch X Conformal`, это означает, что оператор реально
вызвал `bpy.ops.uv.unwrap()` для этого patch. Визуальная проблема после этого
обычно означает уже не "unwrap skipped", а либо слишком жесткий pinning,
либо недостаточно хороший scaffold.

---

## operators.py

`operators.py` остается thin wrapper слоем.

Но в нем теперь есть важный gate:

### Solve preflight

Перед solve path запускается `validate_solver_input_mesh()`.

Оператор должен остановиться до UV write, если найдены:

- `DUPLICATE_FACE`
- `NON_MANIFOLD_EDGE`
- `DEGENERATE_FACE`

При провале preflight:

- печатается console report;
- mode/selection откатываются;
- solve pipeline не запускается.

Это защита от ложных UV-багов, вызванных битой геометрией.

---

## Invariants

Нарушение любого пункта считается багом.

1. `model.py` не импортирует `bpy` и `bmesh`.
2. `analysis.py` не пишет рабочий UV, кроме временного unwrap для outer/hole classification.
3. `solve.py` не делает flood fill и не классифицирует patches.
4. `debug.py` не читает BMesh напрямую.
5. `operators.py` не содержит геометрической логики.
6. PatchGraph хранит индексы, а не BMesh references.
7. Active scaffold builder остается chain-first strongest-frontier.
8. Cross-patch closure не должна замыкать новый patch через два чужих anchors.
9. `HOLE` loops не участвуют в outer geometric fallback.
10. Mesh preflight должен блокировать solve на битой топологии.

---

## Transitional Debt

Это уже известно и не является "случайной поломкой":

1. Solve dataclasses все еще живут в `solve.py`, а не в `model.py`.
2. `ScaffoldSegment` legacy artifact еще присутствует в `model.py`, но не является
   основой активного frontier path.
3. Полноценного persistent `anchor_registry` внутри `ScaffoldMap` пока нет.
4. Explicit `ContinuationEdge` как самостоятельная solve-сущность еще не введен.
5. `format_root_scaffold_report()` еще не показывает всю глубину frontier history.

Это кандидаты на следующую фазу рефакторинга, а не срочные багфиксы.

---

## Current Phase State

### Phase 0

Сделана.

### Phase 1

Операторы и UI вынесены из монолита.

### Phase 2

Validation layer работает:

- scaffold vs UV
- missing targets
- conflicts
- SEAM_SELF diagnostics

### Phase 3

Сделана частично, но это уже рабочий production path:

- chain-first frontier активен;
- patch-type quilt gating активен;
- cross-patch wrap guards активны;
- outer geometric fallback активен;
- per-patch conformal активен;
- preflight mesh validation активен.

### Phase 4

Еще не сделана.

Остается:

- явная continuity model;
- richer debug для continuation routing;
- clearer frontier/build-order introspection.

### Phase 5

Еще не сделана.

---

## Recommended Reading Order For New AI Session

1. Этот документ.
2. `docs/phase3_design.md`
3. `docs/PHASE3_INSTRUCTIONS.md`
4. `cftuv/analysis.py`
5. `cftuv/solve.py`

Сначала нужно понимать текущие компромиссы и долги, а не только исходный дизайн.

