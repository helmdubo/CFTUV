# CFTUV Architecture - Practical Refactoring Plan
## v1.4 - ScaffoldMap / Flow-Driven Runtime Solve Update

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
- `cftuv/solve.py` already exists as a planner/debug + runtime preview layer; `operators.py` is still not extracted.

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
|-- solve.py
`-- Hotspot_UV_v2_5_26.py
```
`operators.py` remains the next major extraction, while `solve.py` moves into ScaffoldMap-driven runtime solve.


---

## Поток данных

```text
UI / Operators -> analysis.py -> PatchGraph -> solve.py -> ScaffoldMap -> UV
                                 PatchGraph -> debug.py
                                 PatchGraph -> solve.py -> SolverGraph -> ScaffoldMap
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


Endpoint topology addendum:

- `BoundaryChain` also stores `vert_indices`
- it stores `start_loop_index` / `end_loop_index`
- it stores `start_corner_index` / `end_corner_index`
- endpoint adjacency is part of the analysis map, not solve output
- continuation is NOT stored as a bool flag in the model
- continuation is derived later in `solve.py` from endpoint adjacency + `frame_role` + tangent / angle compatibility

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


Corner addendum:

- `BoundaryCorner` also stores `vert_index`
- corners are the stable bridge between loop order and chain endpoint topology

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


PatchGraph solve-facing query API addendum:

- `get_chain(...)`
- `find_chains_touching_vertex(...)`
- `get_chain_endpoint_neighbors(...)`
- `get_neighbors(...)`
- `get_seam(...)`

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

`solve.py` is already extracted, and its next step is to move from preview runtime to full ScaffoldMap-driven solve.

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


### Solve Phase 1 - Certainty Field / Forest of Quilts

`solve.py` should not iterate patches in arbitrary order. The first runtime policy of solve is a
**certainty field**: assemble UV quilts from the most deterministic data first, and postpone weak or
ambiguous relations until later.

Core idea:

- solve works from **high certainty** to **low certainty**
- one topological component can split into several independent **quilts**
- weak contacts do not break solve; they produce a **new root** and a new quilt cycle
- final conformal relax happens only after the local scaffold is already defined

#### PatchGraph vs SolverGraph

`PatchGraph` remains the source topology map.

`SolverGraph` is a solve-time filtered view over `PatchGraph`:

- every node is still a patch
- every edge has a solve weight / certainty weight
- not every seam edge is allowed to propagate frame / docking decisions
- weak edges may still be used later for docking polish, but not for primary propagation

This means:

- topological connectivity != solve connectivity
- one connected mesh component may become several quilts in UV solve space

#### Patch certainty score

Each patch receives a solve-time certainty score. The exact formula can evolve, but the priority rules are:

High certainty:

- patch has both `H_FRAME` and `V_FRAME`
- outer contour is readable and rigid
- there are few `FREE` chains
- there is no `HOLE`, or holes are simple
- patch area is large enough to be a stable root

Low certainty:

- patch is dominated by `FREE`
- patch has weak seam context
- patch requires conformal to define most of its shape
- patch contains complex hole placement from the start

Default root rule for Solve Phase 1:

- choose the highest-certainty patch inside one solve component
- if certainty is tied, choose the largest-area patch

#### Edge certainty / conductivity

Each seam relation also receives a solve-time propagation weight.

High conductivity:

- long shared seam
- clear chain-role context on the contact
- endpoint topology gives readable continuation / corner context
- transition semantics is structurally meaningful

Low conductivity:

- very short seam
- `FREE -> FREE`
- `FREE -> H_FRAME` or `FREE -> V_FRAME` with weak endpoint evidence
- ambiguous or weak boundary match

Solve Phase 1 rule:

- strong edges propagate quilt frame and placement
- weak edges do not stop the overall solve, but they stop the current propagation path

#### Forest of quilts

The global solve result is a **forest of quilts**, not necessarily one global quilt.

Algorithmic interpretation:

1. Start from the remaining unsolved patch with the best root score.
2. Create a new quilt root.
3. Propagate through the strongest frontier edges first.
4. Stop propagation when the next edge is below the certainty threshold.
5. Leave unresolved patches in the candidate pool.
6. Restart from the best remaining candidate as a new root.
7. Repeat until every patch belongs to some quilt.

This is the intended runtime behavior for cases where the electrical chain is broken.
A broken or weak chain is not an error; it is the signal to start another quilt.

#### Local solve ladder inside one patch

Inside each placed patch, solve also runs from deterministic structure to uncertain structure:

1. establish patch frame / basis orientation
2. build outer scaffold from `H_FRAME`, `V_FRAME`, corners
3. place `HOLE` loops with the dedicated two-pass workflow when needed
4. bind `FREE` chain endpoints and tangent context from neighboring fixed chains
5. let conformal solve unresolved interior vertices and unresolved free-chain interior vertices

Important:

- conformal is a late-stage relax / completion tool
- conformal does not define the primary scaffold
- the scaffold must exist before conformal is allowed to distribute the remaining uncertainty

#### Frontier queue policy

Propagation inside one quilt uses a frontier queue ordered by certainty:

- pop the strongest not-yet-solved frontier relation
- solve / dock that patch against the already solved quilt subset
- push newly exposed frontier edges back into the queue
- if no frontier edge is above threshold, terminate this quilt and start a new root

This is intentionally closer to weighted electrical flow than to plain BFS.

#### Closure and late weak-pass

After all quilts are assembled:

- run closure pass on cyclic relations inside each quilt
- optionally run a late weak-pass for weak contacts that were not strong enough for primary propagation
- pack quilts relative to each other only after local solve is stable

Weak contacts therefore belong to a late stabilization stage, not to the first deterministic scaffold pass.

### Solve scoring model

Solve Phase 1 uses two different scoring layers.

`patch_root_score(patch)`:

- measures whether a patch is a good quilt root
- rewards strong frame evidence, readable outer scaffold, stable area, and low ambiguity
- penalizes `FREE` dominance and complex hole setup

`edge_propagation_score(candidate)`:

- measures whether solve should continue from the already solved quilt into a specific unsolved neighbor
- the frontier unit is not just a patch and not just a chain
- the frontier unit is an **attachment candidate** built from seam relation + owner chain context + target chain context

A practical first-pass interpretation is `0.0 .. 1.0`:

- `1.0` = ideal propagation / ideal root
- `0.0` = do not propagate through this candidate in the primary pass

#### What increases patch root certainty

- patch has both `H_FRAME` and `V_FRAME`
- outer loop is simple and readable
- patch area is large enough to anchor a quilt
- `WorldFacing` gives a stable basis interpretation
- patch has low `FREE` ratio

#### What decreases patch root certainty

- most chains are `FREE`
- patch needs two-pass hole placement immediately
- patch depends mostly on conformal to reveal its shape
- patch seam context is weak or short

#### What increases edge propagation certainty

- long shared seam
- strong role match across the seam
- endpoint topology gives readable continuation or readable corner context
- seam touches strong corners instead of ambiguous free-only regions
- owner patch is already strongly solved

#### What decreases edge propagation certainty

- `FREE -> FREE`
- short seam
- weak endpoint evidence
- target patch has no reliable continuation from this attachment
- attachment would rely almost entirely on unconstrained conformal behavior

### Role of corners, patch types, and loops in scoring

Corners do not create inter-patch propagation by themselves, but they increase certainty:

- `H_FRAME <-> V_FRAME` sharp corners are strong scaffold evidence
- `FRAME <-> FREE` corners are medium evidence
- `FREE <-> FREE` corners are weak evidence
- corners also define start/end tangent context for `FREE` chains

`PatchType` and `WorldFacing` influence certainty as semantic hints, not as hard truth tables:

- `WALL.SIDE` with both `H_FRAME` and `V_FRAME` is usually a strong root
- `FLOOR.UP` / `FLOOR.DOWN` often provides a stable planar root
- `SLOPE` is usually more ambiguous than `WALL` or `FLOOR`, unless frame evidence is good

`OUTER` / `HOLE` loops affect certainty through complexity:

- one clear `OUTER` increases readability
- `HOLE` does not invalidate a patch, but lowers immediate certainty because it requires two-pass placement

### Solve Phase 1 loop pseudocode

```python
unsolved = set(all_patch_ids)
quilts = []

while unsolved:
    root = choose_best_root(unsolved, patch_root_score)
    if root is None:
        break

    quilt = Quilt(root_patch_id=root)
    solve_patch_as_root(root)
    quilt.solved.add(root)
    unsolved.remove(root)

    frontier = PriorityQueue(max_first=True)
    push_frontier_candidates(frontier, quilt.solved, unsolved)

    while not frontier.empty():
        candidate = frontier.pop_max()

        if candidate.target_patch_id not in unsolved:
            continue
        if candidate.score < EDGE_PROPAGATE_MIN:
            break
        if not patch_is_locally_solvable(candidate.target_patch_id):
            continue
        if not candidate_is_attach_ready(candidate):
            continue

        solve_patch_as_child(
            target_patch_id=candidate.target_patch_id,
            anchor_candidate=candidate,
            solved_quilt=quilt,
        )

        quilt.solved.add(candidate.target_patch_id)
        unsolved.remove(candidate.target_patch_id)
        push_frontier_candidates(frontier, quilt.solved, unsolved)

    finalize_quilt(quilt)
    quilts.append(quilt)

late_weak_pass(quilts)
closure_pass(quilts)
pack_quilts(quilts)
```

Operational meaning:

- solve does not rollback to a previous patch manually
- solve always chooses the strongest candidate from the full current frontier
- if the frontier loses enough certainty, the current quilt stops
- remaining patches stay in the pool and compete to become the next root

Continuation logic addendum:

- analysis provides endpoint adjacency and corner context
- solve decides whether chains continue into each other
- this keeps topology facts in analysis and placement rules in solve

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

### Phase 4 - next

- formalize `ScaffoldMap` as solve-level IR above `PatchGraph`
- rewrite runtime solve from local patch preview to flow-driven quilt builder
- keep `SolverGraph` as planning layer and add `ScaffoldMap` as construction layer
- migrate unwrap / dock / align logic onto ScaffoldMap-driven runtime
- implement root-at-origin, strongest-frontier propagation, and new-root restart logic in runtime
- convert the operator layer into thin wrappers
- start using `corners`, endpoint topology, and side-aware point keys in pin / continuation workflow

### Phase 5 - cleanup / stabilization

- remove legacy runtime leftovers from the monolith
- clean up comment/docstring encoding in old files
- remove duplicate functions
- improve non-manifold / invalid topology diagnostics

---

## What Not To Do

- do not introduce multi-axis semantic profiles on top of `PatchType`
- do not bring back `segments` as a runtime entity
- do not build a separate boundary graph
- do not introduce constraint classes too early
- do not move `Corner` into `solve.py`
- do not store `BMFace/BMEdge/BMLoop` inside `PatchGraph`
- do not add new geometry logic to the monolith if it belongs in `analysis.py` or `solve.py`

---

## Validation Checklist

After each phase, the following must remain true:

- [ ] `model.py` does not import `bpy` or `bmesh`
- [ ] `analysis.py` does not write UV, except `_classify_loops_outer_hole()`
- [ ] `debug.py` does not read `BMesh` directly
- [ ] `PatchGraph` remains the single IR between analysis / solve / debug
- [ ] `BoundaryLoop` does not contain legacy `segments`
- [ ] `BoundaryCorner` is built in analysis, not in solve
- [ ] endpoint topology chains is available through `PatchGraph` query API
- [ ] console debug reflects the hierarchy `Mesh -> Patch -> Loop -> Chain -> Corner`
- [ ] visual debug works in every refactoring phase
- [ ] source mesh hide/show in debug session remains stable
- [ ] UV settings are passed through `UVSettings`, without globals


---

## Solve Phase 1 API addendum

The extracted `solve.py` layer now has two tiers:

- planner/debug tier
- runtime preview tier

The planner tier reads `PatchGraph` and builds solve-time derived structures.
The runtime preview tier is allowed to modify UVs, but only as an experimental scaffold pass.

### Planner API

- `build_solver_graph(graph) -> SolverGraph`
- `choose_best_root(remaining_patch_ids, solver_graph) -> patch_id | None`
- `plan_solve_phase1(graph, solver_graph=None, propagate_threshold=..., weak_threshold=...) -> SolvePlan`
- `format_solve_plan_report(graph, solver_graph, solve_plan, mesh_name=None) -> (lines, summary)`

### Runtime preview API

- `execute_phase1_preview(context, obj, bm, patch_graph, settings, solve_plan=None) -> dict`

Current preview behavior:

1. build ordered patch list from `SolvePlan`
2. run a coarse `CONFORMAL` unwrap to capture per-patch reference UV centroids
3. build local frame scaffold only for `H_FRAME` and `V_FRAME` chains
4. pin scaffold vertices generated from frame chains
5. run a final `CONFORMAL` unwrap for unresolved vertices
6. clear pins

Current limitations:

- `FREE` chains are not rebuilt explicitly yet
- `FREE` chain interior is currently solved by the final `CONFORMAL` pass
- `HOLE` loops still use the same preview pass and do not yet run the full dedicated two-pass hole workflow from this document
- graph-level docking is not part of the preview pass yet

### Solve planner data

`SolverGraph` is a solve-time derived structure over `PatchGraph` and currently contains:

- `patch_scores`: per-patch local solvability and root certainty
- `candidates`: directional attachment candidates between neighboring patches
- `candidates_by_owner`: frontier-ready candidate lookup
- `component_by_patch`: topological component lookup

Each attachment candidate may include solve-time derived signals such as:

- direct frame continuation (`H->H`, `V->V`)
- endpoint bridge continuation through neighboring chains at shared seam endpoints
- corner similarity / corner sharpness context for endpoint continuation

`SolvePlan` contains:

- quilt order
- root selection order
- step-by-step frontier expansion order
- deferred candidates when conductivity falls below threshold
- skipped patches when no valid local root exists

### Flow Debug

A separate `Flow Debug` operator is allowed to call the planner API and print:

- patch certainty table
- conductivity map between patch pairs
- quilt decomposition
- strongest-frontier solve order
- deferred / weak frontier candidates

This debug path is console-first and does not replace the topology GP debug.
It is a planning debugger for the `certainty field / forest of quilts` policy.

### Solve Phase 1 Preview operator

A separate `Solve Phase 1 Preview` operator is allowed to call the runtime preview tier.
It is intentionally parallel to the legacy unwrap operator and does not replace it yet.


---

## ScaffoldMap addendum

`ScaffoldMap` is the new solve-level IR that sits above `PatchGraph`.

Rule of responsibility:

- `PatchGraph` stores topology facts
- `SolverGraph` stores planning / conductivity facts
- `ScaffoldMap` stores constructed 2D placement facts

The critical architectural rule is:

- UV coordinates must be born from already placed `chain` / `corner` anchors
- UV placement must not be derived from per-patch centroid projection once runtime solve starts

### Why ScaffoldMap is needed

The current preview proved that local patch projection is not enough.
The cylinder `SEAM_SELF` case showed the real blocker:

- one `BMVert` can correspond to two different UV-side positions
- therefore solve cannot use plain `vert_index` as the only placement key
- solve needs a side-aware 2D map before writing anything to BMesh UV

### ScaffoldMap minimum content

Minimal `ScaffoldMap v1` should contain:

- `quilts`
- `patch placements`
- `chain placements`
- `corner placements`
- local patch bbox / extents
- quilt-global transform (can start as identity)

A point in ScaffoldMap must be addressed by a solve-side key, not by raw mesh vertex id alone.

Minimum acceptable point key for `v1`:

- `patch_id`
- `loop_index`
- `chain_index`
- `source_point_index`

Equivalent side-aware keys are allowed, but plain `vert_index` is not sufficient.

Boundary serialization must also preserve side-aware face ownership:

- `BoundaryLoop.side_face_indices`
- `BoundaryChain.side_face_indices`

These indices are required to transfer ScaffoldMap points into UV without collapsing `SEAM_SELF`.

### Runtime build rule

Runtime solve must follow `SolvePlan` as a build order, not just as a sorted patch list.

That means:

1. choose quilt root
2. place root at UV origin `(0, 0)` in virtual 2D space
3. build root frame scaffold chain-by-chain and corner-by-corner
4. for each next `SolvePlan` step, attach target patch to the already placed owner chain
5. extend the virtual scaffold map
6. only after scaffold exists, transfer it into BMesh UV and run `Conformal` for unresolved vertices

### Root / child runtime policy

Root patch:

- starts from its first chosen frame chain
- first point may be `(0, 0)`
- next points are built step-by-step from chain length and role
- loop corners become explicit placed points in ScaffoldMap

Child patch:

- is not projected independently
- is attached through `owner_chain -> target_chain` from `SolvePlan`
- inherits its first concrete UV anchors from the already placed owner chain
- extends the quilt scaffold from existing placed geometry

Root scaffold walk rule:

- root loop does not start from an arbitrary first chain
- builder chooses a start chain by local chain score
- then walks the loop by continuity from current chain to next chain
- next segment direction is agreed through the corner turn between sequential chains
- chain / corner / patch order must follow the same score-driven hierarchy used by solve planning

### Conformal usage under ScaffoldMap

`Conformal` is a late-stage completion tool.

It may solve:

- interior patch vertices
- unresolved `FREE` chain interior samples
- later, the internal field around `HOLE` loops

It may not decide primary frame placement.

### Observable implementation roadmap

The first ScaffoldMap iterations must give visible output either in logs or in UV Editor.

#### Iteration 1 - Root Scaffold Log

Goal:

- no UV write required
- print the virtual 2D coordinates of the root patch scaffold

Current status:

- implemented as `Scaffold Debug` operator
- prints root quilt scaffold coordinates into System Console
- now supports `Corner v2` / geometric corners inside one `FREE` loop
- can derive root `Span` segments from those geometric corners for simple frame-shaped loops

Output:

- quilt root id
- placed corner coordinates
- placed frame chain sample coordinates
- patch bbox / extents

This verifies that runtime construction already exists independently of BMesh UV.

#### Iteration 2 - Root UV Preview

Goal:

- take the root scaffold from ScaffoldMap
- write only root frame points into UV
- pin them
- run `Conformal` for the rest of the same patch

Current status:

- implemented as root-only `Solve Phase 1 Preview` runtime pass
- uses `ScaffoldMap` root placements instead of centroid projection
- writes side-aware scaffold points into UV, pins them, then runs `Conformal` for unresolved vertices

Output:

- visible root patch scaffold in UV Editor
- log with supported root count, scaffold point count and pinned UV loop count

#### Iteration 3 - Owner-Chain Anchored Child Scaffold

Goal:

- attach child patches through `owner_chain -> target_chain`
- build quilt growth from already placed scaffold data
- transfer root + supported child scaffolds into UV from one shared `ScaffoldMap`

Current status:

- runtime preview no longer uses coarse reference placement for child patches
- child scaffold is now built from the already placed owner chain of the current quilt
- target patch uses `target_chain` as entry anchor and continues by loop continuity and corner turns
- unsupported patches still remain outside the placed scaffold and fall back to later iterations

Output:

- visible root + supported child quilt relation in UV Editor
- log showing supported roots, attached child count and total scaffold point count

#### Iteration 4 - Full H/V Quilt Runtime

Goal:

- build the entire quilt for patches whose scaffold is dominated by `H_FRAME / V_FRAME`
- do not rely on centroid placement

Output:

- full multi-patch quilt visible in UV
- log of quilt-local bboxes and final quilt bbox

#### Iteration 5 - FREE and HOLE integration

Goal:

- bind `FREE` endpoints from already placed scaffold
- let `Conformal` solve unresolved `FREE` interiors
- bring back dedicated two-pass placement for `HOLE` loops

Output:

- mixed H/V/FREE patches become viable
- patches with `HOLE` no longer depend on local projection

### Future value of ScaffoldMap

`ScaffoldMap` is not only a runtime helper. It is the natural base for future operators:

- patch-local transform tools
- chain reclassification (`FREE -> H/V`)
- manual re-docking
- patch bbox-aware placement tools
- cached re-solve after local edits

This means ScaffoldMap should be treated as a first-class architectural layer, not as a temporary preview cache.
