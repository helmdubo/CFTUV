# CFTUV Architecture - Practical Refactoring Plan
## v1.3 - Hole Placement / Two-Pass Solve Update

## Принцип

Не переписываем инструмент с нуля. Выделяем из монолита устойчивые модули с ясными границами,
сохраняя рабочее поведение на каждом шаге. Центральная цель рефакторинга - **PatchGraph** как
единый IR, через который общаются analysis, solve и debug.

**Debug - не опция, а часть системы.** Анализ считается корректным только если его можно
проверить через System Console и Grease Pencil visualization.

---

## Текущее состояние

На март 2026 года проект находится в промежуточном состоянии:

- `cftuv/model.py` существует и содержит текущую IR-модель.
- `cftuv/constants.py` существует и хранит общие пороги и debug constants.
- `cftuv/analysis.py` существует и строит реальный `PatchGraph` из `BMesh`.
- `cftuv/debug.py` существует и читает `PatchGraph` напрямую.
- `cftuv/__init__.py` уже работает как package entrypoint Blender addon.
- `cftuv/Hotspot_UV_v2_5_26.py` пока остаётся рабочей точкой входа, UI-слоем и временным host-модулем.
- `solve.py` и `operators.py` ещё не выделены в отдельные файлы.

То есть архитектурная модель уже формализована, но runtime всё ещё частично живёт в монолите.

---

## Целевая файловая структура

```text
cftuv/
|-- __init__.py          # bl_info + register/unregister + wiring
|-- model.py             # PatchGraph, PatchNode, BoundaryLoop, BoundaryChain, BoundaryCorner, SeamEdge
|-- analysis.py          # BMesh -> PatchGraph
|-- solve.py             # PatchGraph + BMesh UV -> UV operations
|-- debug.py             # PatchGraph -> Grease Pencil + console report
|-- operators.py         # Blender operators / panel / settings wiring
`-- constants.py         # thresholds, neighbor sentinels, debug prefixes
```

### Практический статус

Сейчас фактическая структура такая:

```text
cftuv/
|-- __init__.py
|-- model.py
|-- constants.py
|-- analysis.py
|-- debug.py
`-- Hotspot_UV_v2_5_26.py
```

`solve.py` и `operators.py` остаются следующей крупной фазой.

---

## Поток данных

```text
UI / Operators -> analysis.py -> PatchGraph -> solve.py
                                 PatchGraph -> debug.py
```

Правило границ:

- `analysis.py` читает `BMesh` и пишет `PatchGraph`
- `solve.py` читает `PatchGraph` и пишет UV
- `debug.py` читает `PatchGraph` и рисует GP / печатает console report
- `model.py` не знает про `bpy` и `bmesh`

Единственное формализованное исключение в analysis:

- `_classify_loops_outer_hole()` временно использует UV unwrap как side effect helper,
  чтобы различать `OUTER` и `HOLE`

---

## Инварианты

Нарушение любого пункта считается багом архитектуры:

1. `model.py` не импортирует `bpy` и `bmesh`
2. `analysis.py` не пишет UV, кроме изолированного `_classify_loops_outer_hole()`
3. `solve.py` не делает flood fill и не классифицирует patch-и
4. `debug.py` не читает `BMesh` напрямую
5. операторы не содержат геометрической логики
6. нет глобальных mutable UV settings, используется `UVSettings`
7. `PatchGraph` хранит индексы и сериализованные данные, но не `BMFace/BMEdge`
8. `Sharp` не участвует в flood fill, boundary loop detection и seam graph construction
9. `Corner` - часть анализа, а не solve
10. `segments` не являются runtime-сущностью модели

---

## Domain Model

### Enums

```python
class PatchType(str, Enum):
    WALL = "WALL"
    FLOOR = "FLOOR"
    SLOPE = "SLOPE"

class WorldFacing(str, Enum):
    UP = "UP"
    DOWN = "DOWN"
    SIDE = "SIDE"

class LoopKind(str, Enum):
    OUTER = "OUTER"
    HOLE = "HOLE"

class FrameRole(str, Enum):
    H_FRAME = "H_FRAME"
    V_FRAME = "V_FRAME"
    FREE = "FREE"

class ChainNeighborKind(str, Enum):
    PATCH = "PATCH"
    MESH_BORDER = "MESH_BORDER"
    SEAM_SELF = "SEAM_SELF"
```

`PatchType` - dispatch key для UV стратегии.
`WorldFacing` - coarse semantic признак patch-а относительно мирового Z (`UP / DOWN / SIDE`).
`LoopKind` - тип замкнутого boundary loop.
`FrameRole` - роль boundary chain в локальном basis patch-а.
`ChainNeighborKind` - topology class соседа boundary chain.

### BoundaryChain

`BoundaryChain` - непрерывный участок одного boundary loop с одним соседом.
Split point = смена соседа.

Хранит:

- `vert_cos`
- `edge_indices`
- `neighbor_patch_id`
- `frame_role`
- `is_closed`

`neighbor_patch_id` - source of truth:

- `>= 0` -> соседний patch
- `-1` -> `MESH_BORDER`
- `-2` -> `SEAM_SELF`

`neighbor_kind` - производное query API поверх `neighbor_patch_id`.

`transition context` - производный semantic слой для boundary chain:

- owner patch -> `PatchType.WorldFacing`
- patch neighbor -> `NeighborPatchType.NeighborWorldFacing`
- `MESH_BORDER` остаётся `... -> MESH_BORDER`, а `SEAM_SELF` семантически маппится в owner -> owner

Примеры:

- `WALL.SIDE -> FLOOR.UP`
- `WALL.SIDE -> FLOOR.DOWN`
- `WALL.SIDE -> WALL.SIDE` (для `SEAM_SELF`)

### BoundaryCorner

`BoundaryCorner` - производная сущность внутри `BoundaryLoop`, а не отдельный граф.

Текущая версия (`Corner v1`) строится **по junction соседних chains**.
Она хранит:

- `loop_vert_index`
- `vert_co`
- `prev_chain_index`
- `next_chain_index`
- `turn_angle_deg`
- `prev_role`
- `next_role`

`corner_type` выводится как переход ролей, например:

- `V_FRAME_TO_H_FRAME`
- `FREE_TO_V_FRAME`
- `H_FRAME_TO_H_FRAME`

Важно:

- `Corner` живёт внутри `BoundaryLoop`
- `Corner` вычисляется в `analysis.py`
- `Corner` пока не хранит `pin_candidate`
- loop с одним chain даёт `0` corners по дизайну текущей версии

### BoundaryLoop

`BoundaryLoop` - замкнутый boundary contour одного patch.

Хранит:

- `vert_cos`
- `edge_indices`
- `kind: OUTER / HOLE`
- `depth`
- `chains`
- `corners`

Правила:

- у одного `PatchNode` может быть только один `OUTER` loop
- остальные loops этого patch - только `HOLE`
- `OUTER` задаёт главный contour scaffold patch-а
- `HOLE` loops являются внутренними контурами и не могут размещаться в UV независимо от `OUTER`

`segments` удалены из runtime-модели. Они не являются частью актуальной архитектуры.

### PatchNode

`PatchNode` - один 3D patch после flood fill.

Хранит:

- `patch_id`
- `face_indices`
- `centroid`
- `normal`
- `area`
- `perimeter`
- `patch_type`
- `world_facing`
- `semantic_key` - производный label вида `WALL.SIDE` или `FLOOR.UP`
- `basis_u`, `basis_v`
- `boundary_loops`
- `mesh_verts`, `mesh_tris`

### SeamEdge

`SeamEdge` - связь между двумя patch-ами через общий seam boundary.

Хранит:

- `patch_a_id`, `patch_b_id`
- `shared_length`
- `shared_vert_indices`
- `longest_edge_verts`
- `longest_edge_length`

### UVSettings

`UVSettings` - immutable snapshot пользовательских UV настроек.

Хранит:

- `texel_density`
- `texture_size`
- `uv_scale`
- `uv_range_limit`
- производное `final_scale`

### PatchGraph

`PatchGraph` - центральный IR.

Хранит:

- `nodes`
- `edges`
- `face_to_patch`
- `_adjacency`

PatchGraph обязан переживать mode switch и не содержать live BMesh references.

---

## analysis.py

`analysis.py` - единственный модуль, который читает `BMesh` для patch topology.

### Публичный API

```python
build_patch_graph(bm, face_indices, obj=None) -> PatchGraph
get_expanded_islands(bm, initial_faces) -> list[dict]
format_patch_graph_report(graph, mesh_name=None) -> tuple[list[str], str]
```

### Основной pipeline анализа

1. Flood fill faces в patch-и по seam / boundary rules
2. Классификация patch: `WALL / FLOOR / SLOPE`
3. Построение локального basis patch-а
4. Tracing boundary loops по boundary-side topology
5. Классификация loops как `OUTER / HOLE`
6. Split loop в chains по смене соседа
7. Классификация `frame_role` для каждого chain
8. Построение `BoundaryCorner` по junction соседних chains
9. Построение `SeamEdge` между patch-ами
10. Сериализация mesh geometry для debug

### Важные принципы анализа

- Boundary topology строится по face-side tracing, а не по уникальным `BMEdge`
- `SEAM_SELF` не должен схлопываться в один edge
- `FrameRole` классифицируется на уровне chain, не segment
- `Corner` использует chain context, но остаётся частью analysis
- analysis не должен принимать решения solve-уровня, например `pin_candidate`

---

## debug.py

`debug.py` читает `PatchGraph` напрямую.

### Console debug

Текстовый debug report отражает базовую иерархию:

```text
Mesh -> Patch -> Loop -> Chain -> Corner
```

Минимальный смысловой набор отчёта:

- mesh name
- patch type
- loop kind
- chain role + neighbor kind
- corner type + turn angle

### Visual debug

Grease Pencil debug работает через semantic filters, а не через legacy dict-слои.

Текущая semantic hierarchy для UX:

- `Patch Types`
- `Loop Kinds`
- `Chain Roles`
- `Overlay`

Это означает:

- если пользователь отключает `Patch Wall`, исчезают и patch fill, и loops, и chains wall patch-а
- если пользователь отключает `Hole`, исчезают hole loops и связанные chains
- `Overlay` живёт отдельно от иерархии topology

### UX-поведение debug

- `Analyze` строит `PatchGraph`, печатает report в System Console и создаёт GP debug
- исходный mesh скрывается на время debug session
- при `OFF`, `Force Clear` или переходе на другой mesh исходный mesh раскрывается
- если пользователь запускает `Analyze` на другом mesh при активном debug, старый debug принудительно очищается и запускается новый
- links visualization временно удалена из UX как неактуальная для текущей стадии

---

## solve.py

`solve.py` ещё не выделен, но его контракт уже зафиксирован.

Он должен:

- читать `PatchGraph`
- читать/писать UV через `BMesh`
- использовать `PatchType`, `WorldFacing`, `basis`, `loops`, `chains`, `corners`, `seam edges`
- использовать chain transition context для trim decals / hotspot textures / seam-based semantic rules
- не строить topology заново

### Что solve не делает

- не flood fill faces
- не классифицирует patch-и
- не строит loops / chains / corners
- не хранит debug-only состояние

### Corner и solve

`solve.py` будет **использовать** corners для pin-кандидатов и каркасных вершин,
но не должен их вычислять как новую topology.

То есть:

- `analysis` строит карту
- `solve` использует карту

---

### Патчи с HOLE: Two-Pass Solve

Если у patch нет `HOLE`, для него достаточно одного unwrap/relax pass.

Если у patch есть хотя бы один `HOLE`, solve обязан работать в два прохода:

1. Построить целевой `OUTER` contour scaffold
2. Pin только `OUTER` и запустить предварительный `Conformal` unwrap для внутреннего поля patch-а
3. Считать из этого coarse pass относительное размещение каждого `HOLE` внутри `OUTER`
4. Перестроить `HOLE` loops по правилам chains / corners
5. Вернуть `HOLE` loops не в абсолютные UV coarse pass, а в координаты относительно `OUTER` contour
6. Pin `OUTER + HOLE` и запустить финальный `Conformal` unwrap для остальных вершин

Важно:

- первый pass является измерительным placement-pass, а не финальным layout
- `HOLE` должен следовать за перестройкой `OUTER`, поэтому его placement хранится относительно `OUTER`, а не в абсолютных UV координатах первого pass
- для placement `HOLE` предпочтительнее использовать centroid / local frame contour-а, а не только bbox center
- финальный relax не определяет положение `HOLE` сам по себе, а только распределяет внутренние вершины между уже размещёнными boundary constraints

---

### Graph-Level Docking Order

Docking patch-ей в UV не должен быть случайным. `solve.py` обязан работать по graph-level иерархии:

1. Построить connected components по patch adjacency / seam links
2. Внутри каждой component выбрать root patch
3. Разместить root patch первым
4. Дальше размещать patch-и по strongest frontier к уже размещённому множеству
5. После базового tree placement выполнить отдельный closure pass для циклов и замыканий

Правила:

- root patch по умолчанию выбирается как patch с наибольшей площадью внутри component
- manual/operator override root policy допустим, но это solve policy, а не analysis data
- strongest frontier определяется не фактом соседства, а весом контакта; базовый вес - shared seam length
- weak или невалидные контакты должны отбрасываться до docking pass, а не исправляться постфактум
- placement в циклических graph-ах сначала строится как spanning tree, и только потом закрывается closure pass
- closure pass не выбирает новый root и не перестраивает компоненту заново; он только распределяет accumulated error на замыканиях
- порядок docking должен быть общим для auto workflow и для manual dock, различаться может только root policy и уровень пользовательского контроля

Цель этого порядка - сделать docking воспроизводимым, устойчивым к сложным компонентам и совместимым с дальнейшим pin / align workflow.

---

## operators / монолитный bridge

Пока `operators.py` не выделен, роль временного host-модуля выполняет
`Hotspot_UV_v2_5_26.py`.

Допустимый временный bridge:

- panel
- operator classes
- debug state wiring
- вызовы `analysis.py` / `debug.py`
- legacy runtime functions, которые ещё не переехали в `solve.py`

Недопустимо наращивать в монолите новую архитектурную логику, если для неё уже есть целевой модуль.

---

## constants.py

Общие constants:

```python
WORLD_UP = Vector((0.0, 0.0, 1.0))
FLOOR_THRESHOLD = 0.9
WALL_THRESHOLD = 0.3
NB_MESH_BORDER = -1
NB_SEAM_SELF = -2
FRAME_ALIGNMENT_THRESHOLD = 0.08
CORNER_ANGLE_THRESHOLD_DEG = 30.0
GP_DEBUG_PREFIX = "CFTUV_Debug_"
```

`CORNER_ANGLE_THRESHOLD_DEG` сейчас используется только для debug/report interpretation,
не для solve decision making.

---

## План рефакторинга

### Phase 1 - выполнено

- вынесены `model.py`, `constants.py`, `UVSettings`
- addon переведён на package entrypoint
- часть монолита использует общие enums / constants

### Phase 2 - выполнено

- debug вынесен в `debug.py`
- debug перешёл на `PatchGraph`
- globals UV settings убраны из рабочего pipeline

### Phase 3 - выполнено

- `analysis.py` строит реальный `PatchGraph`
- debug читает analysis result напрямую
- boundary topology стабилизирована через side tracing
- chain model формализована
- semantic console report введён
- semantic GP filters введены
- `BoundaryCorner` добавлен как часть analysis

### Phase 4 - следующая

- выделить `solve.py`
- перенести unwrap / dock / align logic
- перевести операторный слой в тонкие wrappers
- начать использовать `corners` в pin workflow

### Phase 5 - cleanup / stabilization

- удалить остатки legacy code из монолита
- добить кодировку комментариев и docstring в старом коде
- убрать дубли функций
- усилить non-manifold / invalid topology diagnostics

---

## Что не нужно делать

- не вводить multi-axis semantic profiles поверх `PatchType`
- не возвращать `segments` как отдельную runtime-сущность
- не строить отдельный boundary graph
- не делать constraint classes раньше времени
- не переносить `Corner` в `solve.py`
- не хранить `BMFace/BMEdge/BMLoop` в `PatchGraph`
- не добавлять новую геометрическую логику в монолит, если она относится к `analysis.py` или `solve.py`

---

## Validation Checklist

После каждой фазы должно быть истинно следующее:

- [ ] `model.py` не импортирует `bpy` и `bmesh`
- [ ] `analysis.py` не пишет UV, кроме `_classify_loops_outer_hole()`
- [ ] `debug.py` не читает `BMesh` напрямую
- [ ] `PatchGraph` остаётся единственным IR между analysis / solve / debug
- [ ] `BoundaryLoop` не содержит legacy `segments`
- [ ] `BoundaryCorner` строится в analysis, а не в solve
- [ ] console debug отражает иерархию `Mesh -> Patch -> Loop -> Chain -> Corner`
- [ ] visual debug работает на каждой фазе рефакторинга
- [ ] source mesh hide/show в debug session работает стабильно
- [ ] новые UV настройки передаются через `UVSettings`, без globals
