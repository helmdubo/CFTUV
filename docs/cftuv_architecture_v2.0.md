# CFTUV Architecture v2.0
## Actual implementation state, March 2026

---

## Purpose

Это главный документ проекта.

Он описывает не "идеальную будущую" архитектуру, а текущее рабочее состояние addon'а:

- какие модули существуют;
- какие IR реально используются;
- как сейчас проходит `analysis -> planning -> frontier -> UV transfer -> Conformal`;
- какие правила solve считаются обязательными;
- какие architectural debts ещё остались.

Если другой AI агент должен понять проект с нуля, начинать нужно с этого файла.

---

## Companion Docs

В `docs/` после очистки остаются только два основных документа:

1. `docs/cftuv_architecture_v2.0.md`
   Главный baseline: текущее устройство проекта и актуальные правила runtime.

2. `docs/cftuv_refactor_roadmap_for_agents.md`
   Критический review и безопасный порядок следующих рефакторингов.

Исторические handoff-документы по ранней стабилизации Phase 3 удалены.
Их содержимое либо устарело, либо уже встроено в этот файл и в roadmap.

---

## Project Summary

CFTUV (Constraint-First Trim UV) — Blender addon для полупроцедурной UV развёртки
архитектурных hard-surface ассетов под trim/tile workflow.

Целевая среда:

- Blender 3.0+
- Python 3.10+
- production meshes для environment art

Главный принцип solve:

**не patch-first и не loop-first, а chain-first strongest-frontier.**

Scaffold строится chain за chain из общего frontier-пула quilt.
Всё, что scaffold не смог стабильно разместить, добирается поздним `Conformal`.

---

## Module Layout

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

- `model.py`
  Общие enums, topology IR, solve IR, `UVSettings`, preflight dataclasses.

- `analysis.py`
  BMesh -> `PatchGraph`: patch split, basis, loops, chains, corners, seam edges.

- `solve.py`
  Planning (`SolverGraph`, `SolvePlan`), chain-frontier scaffold builder,
  UV transfer, validation, conformal stage.

- `debug.py`
  Grease Pencil и console visualization из `PatchGraph` и scaffold state.

- `operators.py`
  Blender UI wrappers, orchestration, preflight gate, report printing.

- `constants.py`
  Thresholds, sentinels, scoring weights, shared constants.

Legacy monolith `Hotspot_UV_v2_5_xx.py` не является runtime-частью проекта
и не должен использоваться как источник логики.

---

## Core IR Layers

### 1. Topology IR: `PatchGraph`

Живёт в `model.py`.

Это главный межмодульный контракт.
Он хранит topology-only представление меша:

- `PatchNode`
- `BoundaryLoop`
- `BoundaryChain`
- `BoundaryCorner`
- `SeamEdge`

Ключевые свойства:

- хранит индексы, а не `BMFace`/`BMEdge` ссылки;
- описывает patch topology, а не solve decision;
- содержит и `OUTER`, и `HOLE` loops;
- хранит seam adjacency на уровне patches.

### 2. Persistent Solve IR: `ScaffoldMap`

Тоже живёт в `model.py`.

Текущий persistent solve result:

- `ScaffoldPointKey`
- `ScaffoldChainPlacement`
- `ScaffoldPatchPlacement`
- `ScaffoldQuiltPlacement`
- `ScaffoldMap`

Это уже не временная идея, а рабочий runtime IR.

### 3. Transient Planning IR

Живёт в `solve.py`.

Planning stage использует:

- `PatchCertainty`
- `AttachmentCandidate`
- `SolverGraph`
- `QuiltPlan`
- `SolvePlan`

Это solve-time structures, не persistent map.

---

## Key Domain Terms

### Patch

Набор faces после flood fill по seam.
Минимальная topology unit.

### BoundaryLoop

Замкнутый boundary контур patch.

Типы:

- `OUTER`
- `HOLE`

### BoundaryChain

Непрерывный участок boundary loop с одним соседом.
Это первичная solve unit.

### BoundaryCorner

Junction между двумя соседними chains внутри loop.
Corner не размещается отдельно, а возникает как junction placed chains.

### Quilt

Solve-time группа patches с общим planning context и общим scaffold solve.

### ScaffoldMap

Persistent результат solve в виртуальном 2D пространстве.

---

## Two Different Connectivities

Это критически важно.

### Topology connectivity

Источник:

- `PatchGraph.edges`
- `PatchGraph.connected_components()`

Она отвечает на вопрос:
"Какие patches вообще связаны seam-топологией?"

### Solve connectivity

Источник:

- valid `AttachmentCandidate`
- `SolverGraph.solve_components`
- tree edges из `QuiltPlan`

Она отвечает на вопрос:
"Какие seam relations реально разрешены для solve propagation и scaffold sewing?"

Эти связности не совпадают.

Примеры:

- patch может быть topology-connected только через `HOLE`, но solve-ineligible;
- patch может быть в одном topology component, но non-tree seam должен оставаться UV cut;
- ring/cylinder topology не означает, что quilt имеет право замыкаться в UV cycle.

---

## Current Pipeline

```text
operators.py
  -> validate_solver_input_mesh()
  -> build_patch_graph()
  -> build_solver_graph()
  -> plan_solve_phase1()
  -> build_root_scaffold_map()
  -> transfer scaffold to UV
  -> bpy.ops.uv.unwrap(method='CONFORMAL')
```

Debug path:

```text
operators.py
  -> build_patch_graph()
  -> build_solver_graph()
  -> build_root_scaffold_map()
  -> debug.py visualization / reports
```

---

## `analysis.py` Current Behavior

### Analysis pipeline

Текущий фактический порядок:

1. flood fill faces в patches по seam;
2. classify patch в `WALL` / `FLOOR` / `SLOPE`;
3. build local basis;
4. trace raw boundary loops;
5. classify loops как `OUTER` / `HOLE` через временный UV unwrap;
6. split loop into raw chains по смене соседа;
7. merge bevel-wrap chains;
8. geometric outer fallback split для isolated `OUTER`;
9. split border chains по corners;
10. build `BoundaryChain`;
11. downgrade weaker adjacent `same-role` point-contact chains (`shared_vert_count == 1`) в `FREE`;
12. merge remaining adjacent same-role `MESH_BORDER` chains;
13. build corners и endpoint topology;
14. build seam edges между patches.

### Important current rules

1. `HOLE` loops существуют в topology IR, но не участвуют в outer geometric fallback.
2. `FLOOR` и `SLOPE` больше не насильно переводятся в `FREE`.
   Они сохраняют ту chain classification, которую реально дал `analysis`.
3. `_classify_chain_frame_role()` больше не bbox-only:
   учитывает waviness, chord deviation, polyline vs chord, turn budget.
4. `MESH_BORDER` chains классифицируются мягче, чем patch/seam chains.

### Important current debt

В analysis ещё остаётся raw-dict intermediate layer (`raw_loop`, `raw_chain`),
а corner generation ещё нужно сделать полностью согласованным с финальным chain representation.
Это известная structural зона риска и следующая кандидатура на рефакторинг.

Кроме этого, `_classify_loops_outer_hole()` остаётся единственным analysis-шагом,
который временно использует UV-dependent side effects.
Сейчас это допустимое исключение, но оно должно оставаться локализованным и не должно
расползаться на другие части topology builder.

---

## `solve.py` Current Behavior

`solve.py` делится на два слоя:

- planning layer;
- construction + transfer layer.

### Planning layer

Planning делает:

- per-patch solvability gate;
- root scoring;
- seam attachment scoring;
- solve grouping;
- root selection по remaining patches.

### Current planning rules

1. Attachment candidate строится только если patch types совместимы.
   По умолчанию propagation идёт только между одинаковыми `PatchType`.

2. Attachment candidate строится только по solve-visible chain pairs.
   Сейчас это означает:
   - только `OUTER` loops;
   - только chains с patch neighbor;
   - только допустимые seam relations.

3. `SolverGraph.solve_components` считаются не из raw topology adjacency,
   а из валидных solve attachments.

Это важно для кейсов:

- `HOLE`-connected patches;
- isolated same-type groups;
- planning/debug без ложной solve-связности.

---

## Chain Frontier Builder

Основной construction entry point:

- `build_root_scaffold_map()`
  - вызывает `build_quilt_scaffold_chain_frontier()`

### Current frontier rules

1. Frontier pool содержит только `OUTER` chains.

2. `HOLE` loops не участвуют в scaffold placement pool.
   Они могут существовать в topology, но не должны напрямую строить scaffold.

3. Seed chain выбирается внутри root patch и смотрит только на текущий quilt context.

4. Anchor provenance разделяется на:
   - `same_patch`
   - `cross_patch`

5. `same_patch` anchor разрешён только через loop corner topology.

6. `cross_patch` anchor разрешён только:
   - через shared seam vertex;
   - от фактического patch neighbor текущего chain;
   - по tree edge текущего `QuiltPlan`.

7. Non-tree seams внутри того же quilt не должны повторно зашивать UV island.
   Они считаются intentional cuts.

8. Dual-anchor closure через два `cross_patch` anchors запрещён
   для обычного chain closure.
   Это защищает от ложного patch wrap.

9. `FREE` chains остаются `endpoint-hard`: при двух anchors они могут
   растягиваться между двумя UV точками.

10. `H/V` chains архитектурно должны трактоваться не как обычная dual-anchor interpolation.
    Целевая runtime-semantics для них — `axis-hard, span-soft`:
    - `H_FRAME` обязан оставаться горизонтальным;
    - `V_FRAME` обязан оставаться вертикальным;
    - углы frame-каркаса считаются жёсткими;
    - расхождение длин компенсируется вдоль рабочей оси, а не через наклон chain.
    Важно: локальная per-chain rectification для этого оказалась недостаточной и
    временно отключена в runtime до появления patch/quilt-level orthogonal solve.

11. Tree-only scaffold solve пока не делает отдельный correction pass
    для non-tree closure seams.
    Поэтому ring/cylinder quilts могут сохранять intentional cut seam,
    но всё равно накапливать span drift между двумя сторонами closure.
    Этот drift особенно заметен, если tree path к closure seam проходит
    через one-edge `FREE` bridge.

### Practical implication

Ring / cylinder / closed-house topology может быть seam-connected по кругу,
но quilt scaffold всё равно должен оставаться tree, а не UV cycle.

Важно:
tree-only sewing решает проблему ложного замыкания, но само по себе
не решает accumulated closure error на intentional cut seam.

---

## Envelope Layer

После frontier placed chains группируются обратно в per-patch envelope:

- `ScaffoldPatchPlacement`
- `ScaffoldQuiltPlacement`

Текущие реально используемые поля envelope:

- `status`
- `dependency_patches`
- `unplaced_chain_indices`
- `closure_error`
- `closure_valid`
- `bbox_min`
- `bbox_max`
- `scaffold_connected_chains`

`build_order` уже существует на уровне quilt.

Важно:
часть полей `ScaffoldPatchPlacement` остаётся transitional и требует cleanup.
Документ ниже в разделе debt перечисляет это явно.

---

## UV Transfer And Conformal

### UV transfer

Scaffold сначала переносится в UV layer, затем запускается `Conformal`.

Ключевые детали:

- `_resolve_scaffold_uv_targets()` работает side-aware;
- coverage расширяется на все patch face loops для unique patch vertices;
- `SEAM_SELF` обрабатывается отдельно;
- validation сравнивает scaffold intent с реально записанными UV loops.

### Current pin policy

Текущая политика не совпадает со старой "pin all H/V, pin only FREE endpoints".

Сейчас:

1. Если patch целиком ушёл в `FREE`, он считается conformal patch:
   scaffold points не пинятся, весь patch отдаётся `Conformal`.

2. `H_FRAME` / `V_FRAME` pinятся только если chain scaffold-connected
   с root chain через непрерывную H/V path
   (bridge FREE chains длиной <= 2 прозрачны для этой BFS).

3. Изолированные H/V chains за FREE-разрывом не пинятся.

4. `FREE` chains:
   - если points <= 4, пинятся все;
   - иначе пинятся endpoints и каждый третий intermediate point.

### Conformal strategy

Conformal больше не запускается одним глобальным unwrap на всё сразу.

Сейчас:

- supported patches получают scaffold transfer и затем conformal completion;
- unsupported patches получают отдельный fallback conformal;
- quilts раскладываются по cursor offset.

Если в логах есть `Patch X Conformal`, это означает реальный вызов unwrap для этого patch.

---

## Debug And Validation

Debug path обязателен.
Если рефакторинг ломает debug explanation слоя, он считается плохим.

Сейчас debug/report layer должен уметь показать:

- patch graph;
- solve plan;
- scaffold quilt placement;
- unsupported patches;
- closure status;
- placed chain order;
- chain roles и corner layout.

---

## Current Architectural Debt

Это не "всё плохо", а честный список того, что ещё осталось.

### 1. Solve policy ещё не централизована полностью

После последних bugfixes правила solve стали лучше,
но всё ещё размазаны по planning, anchor search, scoring и frontier filtering.

Следующий правильный шаг:
собрать solve-visible rules в один solve-time policy/view слой.

### 2. `analysis.py` ещё использует raw dict payloads

Это делает intermediate contracts менее прозрачными
и усложняет безопасный refactor chain/corner logic.

Отдельно важно:
corner detection сейчас идёт по двум путям с не полностью одинаковой семантикой
для closed `OUTER` loops и open `MESH_BORDER` chains.
Это не объявлено багом само по себе, но должно быть явно проверено в Phase 1B,
чтобы intentional differences не смешивались со случайным drift.

### 3. `analysis.py` не полностью чистый из-за временного UV-dependent loop classification

`_classify_loops_outer_hole()` временно мутирует UV state ради `OUTER/HOLE` classification.
Даже при корректном rollback это остаётся special-case boundary внутри analysis
и требует явной изоляции.

### 4. Нет удобного serializable scaffold snapshot baseline

Сейчас regression verification в основном держится на console logs и ручном осмотре.
Следующий полезный шаг - не тяжёлый automated test framework,
а стабильный scaffold snapshot/report, который можно сравнивать до/после рефактора.

### 5. Frontier runtime state раздроблен

Сейчас frontier builder держит несколько параллельных registry:

- point registry;
- vertex-to-placement lookup;
- placed refs;
- placed placements;
- dependency map;
- build order.

Это рабочее решение, но не лучший long-term state model.

### 6. Нет отдельной runtime-стадии для frame-row alignment и closure stabilization

Сейчас quilt может быть корректным как tree solve и всё равно иметь два разных runtime-артефакта:

- drift на non-tree closure seam;
- scatter внутри геометрически коллинеарных `H_FRAME` / `V_FRAME`, когда chains одной 3D-линии получают разные UV offsets.

Это две связанные, но не одинаковые проблемы:

- closure seam может требовать точечной pre-constraint / correction логики;
- row / column inconsistency требует отдельной диагностики и, возможно, ограниченного post-pass выравнивания.

Для этой зоны нужен отдельный runtime track:

- diagnostics для closure seams и row/column classes;
- сначала точечный closure pre-constraint для closure-sensitive paths;
- затем повторный замер;
- и только потом, если scatter всё ещё значим, консервативный row/column snap post-pass.

### 7. Часть `ScaffoldPatchPlacement` полей transitional

Например:

- `root_chain_index`
- `max_chain_gap`
- `gap_reports`
- `pinned`
- `origin_offset`

Эти поля нужно либо честно заполнять,
либо убрать/заморозить до отдельной фазы.

Кроме этого, текущая pin policy вычисляется уже после frontier placement,
через scaffold connectivity на этапе transfer/envelope logic.
Для текущей грубой pin policy это допустимо, но более умная pin strategy
потребует отдельного архитектурного решения, а не локального tweak.

### 8. Формальных автоматических тестов нет

Верификация идёт через:

- debug visualization;
- console reports;
- manual UV inspection;
- повторяемые regression meshes.

Практически следующая полезная ступень -
не unit tests на всё подряд,
а стабильный regression harness с golden logs, scaffold snapshots и known meshes.

---

## Current Phase

Проект находится в состоянии:

- chain-first frontier path активен;
- `HOLE` solve drift исправлен;
- same-type quilt separation активна;
- `FLOOR` / `SLOPE` больше не насильно деградируют в all-`FREE`;
- ring/cylinder cycle bug закрыт tree-edge-only cross-patch sewing.

Следующий архитектурный приоритет:

1. typed raw payload в `analysis.py`;
2. topology consistency в `analysis.py`;
3. isolation boundary для UV-dependent analysis step;
4. central solve policy layer.

Параллельный runtime research track:

- closure seam diagnostics;
- row/column diagnostics;
- точечный closure pre-constraint;
- затем только при необходимости консервативный row/column snap post-pass.

Текущее состояние этого track:

- diagnostics для non-tree same-role closure seams уже добавлены в `solve.py`;
- они измеряют span mismatch, positional drift по shared seam vertices (`phase` вдоль рабочей оси и `cross-axis` offset), а также общий shared UV delta;
- они пишутся в logs и сохраняются в `ScaffoldQuiltPlacement.closure_seam_reports`;
- row/column diagnostics уже добавлены как отдельный `FrameDiag` report в `solve.py`;
- текущая practical реализация row/column diagnostics намеренно ограничена `WALL.SIDE` cases;
- row/column diagnostics сохраняются в `ScaffoldQuiltPlacement.frame_alignment_reports` и показывают scatter для collinear `H_FRAME` / `V_FRAME` групп;
- frame classification thresholds теперь разделены: `H_FRAME` uses `0.02`, `V_FRAME` keeps `0.04`; это намеренное asymmetric tightening, чтобы пограничные горизонтали раньше уходили в `FREE`, не пережимая вертикальный scaffold тем же порогом;
- текущая целевая семантика classifier асимметрична: `H_FRAME` = близость к локальной плоскости `N-U`, `V_FRAME` = близость к локальной оси `basis_v`; это не симметричный `extent_u / extent_v` test;
- adjacent `same-role` point-contact (`shared_vert_count == 1`) больше не считается жёстким continuation case: weaker chain должен деградировать в `FREE` ещё в `analysis.py`, а жёсткий same-role continuity допустим только при shared edge;
- weaker/stronger heuristic для такого point-contact guard должна предпочитать dominant span и chain length, а не "идеальную осевость" tiny sliver-chain;
- adjacent `MESH_BORDER + MESH_BORDER` same-role pieces должны merge-иться как один border carrier-chain и не должны деградировать в `FREE` из-за point-contact guard;
- one-edge `FREE` bridges должны иметь более низкий frontier priority, чем любые `H/V`, и с одним anchor должны ждать second anchor, чтобы rigid-frame собирался раньше мягких мостиков;
- точечный `closure pre-constraint` уже реализован в frontier placement path и работает только для `closure-sensitive` `one-anchor` `H_FRAME` / `V_FRAME` chains;
- текущая practical версия `closure pre-constraint` не меняет quilt-wide scoring и не делает post-pass; она только выбирает лучший one-anchor вариант по existing closure pair и local anchor gaps;
- следующий шаг для этого runtime track — повторный замер на production meshes и только потом решение, нужен ли консервативный `row / column snap post-pass`;
- локальная per-chain rectification для `H/V` уже проверялась и была отключена как structural regression, потому что она не держит согласованный ряд/колонку на уровне quilt.

Подробный порядок описан в:

- `docs/cftuv_refactor_roadmap_for_agents.md`

---

## Entry Points For New Agents

Если другой агент начинает работу:

1. сначала прочитать этот файл;
2. затем открыть `docs/cftuv_refactor_roadmap_for_agents.md`;
3. затем смотреть в коде:
   - `analysis.py -> build_patch_graph()`
   - `solve.py -> build_solver_graph()`
   - `solve.py -> plan_solve_phase1()`
   - `solve.py -> build_root_scaffold_map()`
   - `solve.py -> execute_phase1_preview()`
   - `operators.py -> _prepare_patch_graph()`

---
