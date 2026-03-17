# CLAUDE.md — Project Context for AI Agents

## What is this project

CFTUV (Constraint-First Trim UV) — Blender addon для полупроцедурной UV развёртки
архитектурных ассетов (hard-surface environment geometry). Пользователь размечает
mesh в 3D Viewer (seams, face selection; sharp edges только для shading), система строит граф 3D patches
и автоматически генерирует UV layout по правилам стыковки.

Target: AA-AAA game production, trim sheet / tile workflow.
Blender: 3.0+, Python 3.10+.
Single developer, in-house studio tool.

## Core Principle — Chain-First Strongest-Frontier

**Не patch-first, не loop-first, а chain-first strongest-frontier.**

Scaffold строится chain за chain, выбирая на каждом шаге самый сильный доступный
chain из общего пула всего quilt. Пул не различает "chain того же patch" и
"chain другого patch через seam" — это одинаковые кандидаты.

Quilt растёт органически от root chain наружу, пересекая seam boundaries,
пока frontier не исчерпан или не упал ниже порога.
Всё, что frontier не взял — free vertices для финального Conformal unwrap.

Этот принцип должен сохраняться при любых изменениях solve логики.
Дрейф в сторону patch-first, loop-sequential или corner-based placement
считается архитектурной регрессией.

## Architecture

Прочитай `docs/cftuv_architecture_v2.0.md` перед любой работой. Это главный документ.
Затем обязательно прочитай `docs/cftuv_refactor_roadmap_for_agents.md`.
Для нового агента это не optional companion, а документ с текущим practical plan и границами scope.

Система разделена на 7 модулей:

| Модуль | Роль | Читает | Пишет |
|--------|------|--------|-------|
| model.py | Структуры данных | — | — |
| constants.py | Конфигурация | — | — |
| analysis.py | BMesh → PatchGraph | BMesh | PatchGraph |
| solve.py | ScaffoldMap builder + UV | PatchGraph + BMesh UV | ScaffoldMap, UV layer |
| debug.py | Визуализация | PatchGraph | Grease Pencil |
| operators.py | Blender UI обёртки | всё | — |
| ~~Hotspot_UV_v2_5_26.py~~ | **УДАЛЁН.** Legacy монолит. Не использовать. | — | — |

**Центральный IR — PatchGraph.** Все данные между модулями передаются через него.
**Solve IR — ScaffoldMap.** Виртуальная 2D карта, которая хранит результат placement.

## Glossary — Canonical Definitions

Каждый термин имеет одно определение. Если термин используется в другом смысле — это баг.

### Topology Layer (analysis.py → PatchGraph)

- **Patch** — набор faces после flood fill, ограниченный seam edges и mesh boundary.
  Минимальная единица topology. Один patch = один PatchNode в PatchGraph.

- **PatchType** — dispatch key для UV стратегии: `WALL`, `FLOOR`, `SLOPE`.
  Определяется по углу нормали к WORLD_UP. Не semantic description, а routing key.

- **WorldFacing** — грубая ориентация patch относительно мирового Z: `UP`, `DOWN`, `SIDE`.

- **Boundary Loop** — замкнутый контур boundary edges одного patch.
  Типы: `OUTER` (один на patch, внешний контур) или `HOLE` (внутренние отверстия).

- **Boundary Chain** — непрерывный участок boundary loop с одним соседом.
  Split point = вершина, где сосед меняется.
  **Chain — первичная единица placement в solve.**

- **Boundary Corner** — вершина на стыке двух chains внутри одного loop.
  Corner не имеет собственных координат — его позиция возникает как результат
  размещения chains. Corner используется для validation и как anchor point
  для endpoints FREE chains.

- **FrameRole** — роль chain в локальном базисе patch: `H_FRAME`, `V_FRAME`, `FREE`.
  H_FRAME = горизонтальный в 3D, V_FRAME = вертикальный, FREE = диагональный/неопределённый.

- **ChainNeighborKind** — тип соседа chain: `PATCH` (другой patch), `MESH_BORDER`
  (край меша), `SEAM_SELF` (seam внутри того же patch).

- **SeamEdge** — связь между двумя соседними patches через общий seam boundary.

- **Basis** — локальная система координат patch.
  `basis_u` = горизонталь (U/tangent), `basis_v` = вертикаль (V/bitangent).
  Primary source для определения направления chain при placement.

- **Semantic Key** — производный label вида `WALL.SIDE`, `FLOOR.UP`, `FLOOR.DOWN`.
  Используется для scoring и debug, не для routing.

### Solve Layer (solve.py → ScaffoldMap)

- **Quilt** — независимая группа patches, собранная solve из одного root.
  Каждый quilt строится от начала координат (0,0).
  Quilts независимы друг от друга.

- **ScaffoldMap** — виртуальная 2D карта размещения всех chains/corners/patches.
  Persistent result, не процесс. Может быть кэширован, отредактирован вручную,
  частично пересчитан.

- **ScaffoldSegment** — формализованная единица placement внутри ScaffoldMap.
  Замена анонимным dict-ам. Хранит role, source_kind, points, corner refs.

- **Chain Frontier** — общий пул доступных для размещения chains всего quilt.
  Не различает chains одного patch и chains через seam.
  Strongest chain берётся из пула на каждом шаге.

- **Anchor Point** — уже размещённая вершина (shared corner или shared seam vertex),
  от которой может стартовать размещение нового chain.
  **Chain без anchor point не размещается.**

- **Free Vertices** — все вершины, которые не были размещены scaffold builder.
  Включает: не размещённые chains, внутренние вершины patches.
  Обрабатываются финальным Conformal unwrap внутри уже построенного каркаса.

- **SolverGraph** — solve-time filtered view поверх PatchGraph.
  Содержит scoring, candidates, component mapping.

- **ContinuationEdge** — явная связь "chain A продолжается в chain B через seam".
  Solve-level сущность (не topology). Типы: SAME_ROLE, CROSS_ROLE, CORNER_BRIDGE.

### Legacy Terms — НЕ ИСПОЛЬЗОВАТЬ

- **Island / IslandInfo** — legacy понятие из монолита. Заменено на Patch/PatchNode.
- **Segment (runtime)** — удалён из runtime модели. Заменён на ScaffoldSegment в solve.
- **Full faces / Core faces** — legacy разделение из Two-Pass Unwrap. Не используется
  в chain-first pipeline.

## Invariants — нарушение любого = баг

1. `model.py` НЕ импортирует `bpy`, `bmesh` (только `mathutils`)
2. `analysis.py` НЕ пишет UV (кроме `_classify_loops_outer_hole` — помечен явно)
3. `solve.py` НЕ делает flood fill и НЕ классифицирует патчи
4. `debug.py` НЕ читает BMesh напрямую
5. Операторы НЕ содержат геометрической логики (max 5 строк math)
6. Нет глобальных мутабельных переменных — UVSettings передаётся параметром
7. PatchGraph хранит индексы (int), НЕ ссылки на BMFace/BMEdge
8. Sharp НЕ участвует в patch split — только Seam
9. Corner вычисляется в analysis, НЕ в solve
10. ScaffoldMap — persistent result, НЕ процесс. Может быть кэширован и отредактирован.
11. **Scaffold растёт chain-first strongest-frontier, НЕ patch-by-patch, НЕ loop-sequential.**

## Code conventions

- Язык комментариев: русский (это внутренний инструмент студии)
- Docstrings: русский или английский, без разницы
- Naming: snake_case для функций, PascalCase для классов, UPPER_CASE для констант
- Enum values: UPPER_CASE строки (`PatchType.WALL`, `FrameRole.H_FRAME`)
- Private functions: prefix `_` (не `__`)
- Типы: dataclass для данных, обычные классы для операторов
- Нет third-party зависимостей кроме Blender built-in (bpy, bmesh, mathutils)

## What NOT to do

- НЕ добавлять multi-axis semantic profiles (confidence scores, role_class, etc.)
- НЕ создавать отдельный Boundary Graph — chains это подструктура PatchNode
- НЕ создавать constraint classes — правила стыковки это код в solve.py
- НЕ разбивать solve.py на подмодули (пока < 800 строк)
- НЕ делать debug geometry optional/lazy — debug всегда включён
- НЕ использовать globals для настроек — передавать UVSettings параметром
- НЕ хранить BMFace/BMEdge ссылки в model — только индексы
- **НЕ использовать Hotspot_UV_v2_5_26.py как источник логики — это мёртвый legacy**
- **НЕ возвращаться к patch-first или loop-sequential placement**
- **НЕ уходить в большой orthogonal frame graph solve / editable constraint platform / dirty-scope system на текущем runtime этапе**
- **НЕ возвращать локальную per-chain rectification для `H/V` — она уже дала structural regression**
- **НЕ делать quilt-wide snap "на всякий случай" без diagnostics**

## Testing approach

Формальных тестов нет. Верификация через:
1. Debug visualization (Grease Pencil) — включается кнопкой Analyze в панели
2. Console output (patch stats, chain info, frame roles)
3. Validation layer (scaffold vs UV mismatch report)
4. Ручная проверка UV в UV Editor на production meshes

При рефакторинге: debug visualization должна работать на КАЖДОЙ фазе.
Если debug сломался — рефакторинг неправильный.

## Refactoring state

Текущая фаза: см. `docs/cftuv_architecture_v2.0.md`.

Важно: кроме основных refactor phases сейчас есть отдельный active runtime track.
Он не должен уводить агента в redesign всего solve layer.
Sprint-level runtime heuristics, thresholds и production-case notes вынесены в
`docs/cftuv_runtime_notes.md`. Это operational notes, а не архитектура.

### Current active runtime task

Текущая practical задача не в том, чтобы переписать solve, а в том, чтобы
стабилизировать frame alignment / closure behavior без потери chain-first architecture.

Текущий порядок работ:

1. `row / column diagnostics` уже добавлены рядом с `closure seam diagnostics`;
2. точечный `closure pre-constraint` уже встроен только для closure-sensitive paths;
3. blocking runtime bugs на production meshes приоритетнее широкой integration-идеи;
4. diagnostics-only lattice branch можно вести параллельно, пока она не меняет placement path;
5. после runtime fixes повторно мерить diagnostics на production meshes;
6. только потом решать, нужен ли консервативный `row / column snap post-pass`.

### Runtime guardrails

Для этого runtime track действуют жёсткие ограничения:

- frontier остаётся chain-first strongest-frontier;
- tree-only quilt sewing не ломать;
- `FREE` не трогать без необходимости;
- rescue-path после stall допустим только как узкий reachability step, а не как возврат к patch-first solve;
- diagnostics-only lattice branch можно вести параллельно runtime fixes, но её integration не должна маскироваться под новый solve layer;
- если lattice данные появляются в placement path, они обязаны приходить как `source_kind='lattice_anchor'`, а не как fake scaffold placement;
- нельзя заранее записывать coarse lattice точки в runtime как будто это уже обычные placed chains того же provenance;
- не вводить новые persistent override поля в IR до реального manual-use case;
- не делать patch/quilt-wide "умный solve" вместо точечных шагов diagnostics -> pre-constraint -> remeasure -> snap.

### Already known runtime facts

- detailed current heuristics живут в `docs/cftuv_runtime_notes.md`;
- runtime track всё ещё разделён на две зоны:
  1. closure-sensitive seams;
  2. row / column inconsistency внутри геометрически коллинеарных `H_FRAME` / `V_FRAME`;
- diagnostics-first и reachability-rescue уже встроены в solve;
- lattice branch пока допускается только как diagnostics / coarse-guide track, не как новый solve layer.

Фазы рефакторинга:
- Phase 0: Glossary & Docs ✓
- Phase 1: Extract operators.py, remove monolith dependency
- Phase 2: Validation layer (scaffold vs UV diagnostic)
- Phase 3: Strongest-First Chain Frontier Builder
- Phase 4: Continuity + debug visualization
- Phase 5: Rebuild operators + manual override hooks
