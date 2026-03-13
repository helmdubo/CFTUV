# CFTUV Architecture v2.0
## Chain-First Strongest-Frontier

---

## Принцип

**Не patch-first, не loop-first, а chain-first strongest-frontier.**

Scaffold строится chain за chain. На каждом шаге из общего пула доступных chains
берётся самый сильный. Пул не различает "chain того же patch" и "chain через seam" —
это одинаковые кандидаты. Quilt растёт органически.

Всё, что frontier не взял — free vertices для Conformal.

---

## Файловая структура

```text
cftuv/
├── __init__.py          # bl_info + register/unregister
├── model.py             # PatchGraph, ScaffoldMap, все dataclasses
├── constants.py         # пороги, sentinel values, debug prefixes
├── analysis.py          # BMesh → PatchGraph
├── solve.py             # PatchGraph → ScaffoldMap → UV
├── debug.py             # PatchGraph → Grease Pencil + console
└── operators.py         # Blender operators / panel / settings
```

`Hotspot_UV_v2_5_26.py` удалён из рабочего пакета. Не использовать как источник логики.

---

## Поток данных

```text
Operators → analysis.py → PatchGraph → solve.py → ScaffoldMap → UV
                           PatchGraph → debug.py → GP / Console
                           PatchGraph → solve.py → SolverGraph (scoring)
```

Границы:
- `analysis.py` читает BMesh, пишет PatchGraph
- `solve.py` читает PatchGraph, строит ScaffoldMap, пишет UV
- `debug.py` читает PatchGraph, рисует GP
- `model.py` не знает про bpy и bmesh
- `operators.py` — тонкие обёртки, без геометрической логики

Единственное исключение: `_classify_loops_outer_hole()` в analysis использует
временный UV unwrap как side effect для OUTER/HOLE классификации.

---

## Инварианты

Нарушение любого — баг:

1. `model.py` не импортирует `bpy`, `bmesh`
2. `analysis.py` не пишет UV (кроме `_classify_loops_outer_hole`)
3. `solve.py` не делает flood fill и не классифицирует patches
4. `debug.py` не читает BMesh напрямую
5. Операторы не содержат геометрической логики
6. UVSettings передаётся параметром, нет глобальных mutable settings
7. PatchGraph хранит индексы, не BMFace/BMEdge ссылки
8. Sharp не участвует в patch split — только Seam
9. Corner вычисляется в analysis, не в solve
10. ScaffoldMap — persistent result, может быть кэширован и отредактирован
11. **Scaffold строится chain-first strongest-frontier**

---

## Domain Model

### Enums

```python
class PatchType(str, Enum):     # WALL, FLOOR, SLOPE
class WorldFacing(str, Enum):   # UP, DOWN, SIDE
class LoopKind(str, Enum):      # OUTER, HOLE
class FrameRole(str, Enum):     # H_FRAME, V_FRAME, FREE
class ChainNeighborKind(str, Enum):  # PATCH, MESH_BORDER, SEAM_SELF
```

### BoundaryChain

Непрерывный участок boundary loop с одним соседом.
**Chain — первичная единица placement.**

Хранит: `vert_indices`, `vert_cos`, `edge_indices`, `side_face_indices`,
`neighbor_patch_id`, `frame_role`, `is_closed`,
`start_loop_index`, `end_loop_index`,
`start_corner_index`, `end_corner_index`.

`neighbor_patch_id` encoding: `>=0` = patch id, `-1` = MESH_BORDER, `-2` = SEAM_SELF.
`neighbor_kind` — derived property.

### BoundaryCorner

Вершина на стыке двух chains внутри одного loop.
**Corner не имеет собственных координат** — его позиция возникает как результат
размещения chains. Corner используется для:
- Validation (стык chains замкнулся?)
- Anchor point для endpoints FREE chains
- Debug visualization

Хранит: `loop_vert_index`, `vert_index`, `vert_co`,
`prev_chain_index`, `next_chain_index`, `turn_angle_deg`,
`prev_role`, `next_role`.

### BoundaryLoop

Замкнутый boundary contour одного patch.
Один patch = один OUTER loop + N HOLE loops.

Хранит: `vert_indices`, `vert_cos`, `edge_indices`, `side_face_indices`,
`kind`, `depth`, `chains`, `corners`.

### PatchNode

Один 3D patch. Хранит topology + geometry + basis.

Ключевые поля: `patch_id`, `face_indices`, `centroid`, `normal`,
`patch_type`, `world_facing`, `basis_u`, `basis_v`,
`boundary_loops`, `mesh_verts`, `mesh_tris`.

`basis_u`/`basis_v` — primary source для определения направления chain при placement.

### SeamEdge

Связь между двумя patches через общий seam boundary.

### PatchGraph

Центральный IR. Хранит: `nodes`, `edges`, `face_to_patch`, `_adjacency`.

Query API: `get_chain()`, `find_chains_touching_vertex()`,
`get_chain_endpoint_neighbors()`, `get_neighbors()`, `get_seam()`,
`connected_components()`, `traverse_bfs()`, `find_root()`.

### UVSettings

Immutable snapshot: `texel_density`, `texture_size`, `uv_scale`, `uv_range_limit`.
Derived: `final_scale`.

---

## Solve Architecture

### Два слоя solve

1. **Planning layer** — оценка и порядок (SolverGraph → SolvePlan)
2. **Construction layer** — размещение (ScaffoldMap → UV)

### SolverGraph (planning)

Solve-time filtered view поверх PatchGraph:
- `patch_scores` — per-patch certainty и root score
- `candidates` — directional attachment candidates между patch парами
- `continuations` — explicit continuation edges между chains через seam
- `component_by_patch` — topological component lookup

### SolvePlan (planning result)

Порядок размещения: quilts → steps → patches.
Не пространственная структура — абстрактный порядок.

### ScaffoldMap (construction result)

Виртуальная 2D карта. **Persistent result, не процесс.**

Структура:
```
ScaffoldMap
  └── ScaffoldQuiltPlacement (per quilt, starts at origin)
        └── ScaffoldPatchPlacement (per patch)
              └── ScaffoldChainPlacement (per chain)
                    └── (ScaffoldPointKey, Vector) pairs
```

ScaffoldPointKey — side-aware address: `patch_id`, `loop_index`, `chain_index`,
`source_point_index`. Plain `vert_index` недостаточен из-за SEAM_SELF.

---

## Chain-First Strongest-Frontier Algorithm

### Placement Model

```
1. Выбрать root patch quilt. Выбрать strongest chain root patch.
   Разместить от (0,0) строго по оси (direction из basis).

2. Chain frontier pool = все chains, у которых есть shared vertex
   с уже размещённым chain (anchor point).
   Pool включает chains ВСЕХ patches quilt, не только текущего.

3. Оценить pool → взять strongest → разместить:
   - H_FRAME: snap строго горизонтально
   - V_FRAME: snap строго вертикально
   - FREE: interpolate между anchor endpoints (если оба известны)
           или guided от одного anchor (если один)

4. Новые chains стали доступны (shared vertex с только что размещённым)?
   → добавить в pool.

5. Повторять пока pool не пуст или ниже threshold.

6. Всё что не размещено → free vertices для Conformal.
```

### Chain Scoring

Chain-level score (не patch-level):

Высокий score:
- H_FRAME или V_FRAME
- Есть уже размещённый сосед с anchor point
- Сосед по seam с сильным семантическим контекстом (FLOOR.DOWN, etc.)
- Оба endpoints уже известны (shared corners с размещёнными chains)

Низкий score:
- FREE без anchor points
- Нет размещённых соседей
- Слабый seam контекст

Правило: **chain без anchor point не размещается.** Без known position — нет старта.

### Direction Resolution

Направление H/V chain при placement:

1. **Primary: basis patch.** H_FRAME лежит вдоль `basis_u`, V_FRAME вдоль `basis_v`.
   Sign (±) определяется проекцией 3D direction chain на basis vector.

2. **Secondary: neighbor context.** Если basis ambiguous (edge case),
   сосед по seam подсказывает — например, H chain граничащий с FLOOR.DOWN = нижняя кромка.

3. **Inherited: уже размещённый сосед.** Если chain стыкуется с размещённым chain
   через shared vertex, direction унаследован от anchor.

### H/V Snap

H_FRAME chain размещается строго горизонтально: `direction = (±1, 0)`.
V_FRAME chain размещается строго вертикально: `direction = (0, ±1)`.

Никакого floating point angle. Snap к оси. Это убирает accumulation error полностью
для H/V dominated meshes.

Corner между H и V — точка пересечения горизонтали и вертикали. Ровно 90°.

### FREE Chain Handling

FREE chain размещается, только если:
- Оба endpoints уже известны (corners/shared verts с размещёнными H/V chains)
  → interpolation между endpoints (Bezier/polyline)
- Или один endpoint известен
  → guided extension от anchor, с direction из 3D shape

FREE chains без anchor points → не размещаются scaffold builder → free vertices.

### Conformal Role

Conformal — late-stage completion tool. Работает внутри каркаса из размещённых chains.

Что решает Conformal:
- Внутренние вершины patches (не на boundary)
- Free vertices от не размещённых chains
- Interior field вокруг HOLE loops

Что НЕ решает Conformal:
- Primary frame placement (это scaffold builder)
- Chain direction и length (это snap + 3D geometry)

---

## analysis.py

Единственный модуль, читающий BMesh для topology.

Public API:
```python
build_patch_graph(bm, face_indices, obj=None) -> PatchGraph
format_patch_graph_report(graph, mesh_name=None) -> tuple[list[str], str]
```

Pipeline:
1. Flood fill → patches
2. Classify patch type
3. Build basis
4. Trace boundary loops (side-based)
5. Classify OUTER/HOLE
6. Split loops → chains by neighbor
7. Classify FrameRole per chain
8. Build BoundaryCorner at chain junctions
9. Build SeamEdge between patches
10. Serialize geometry for debug

---

## debug.py

Читает PatchGraph напрямую.

Console: `Mesh → Patch → Loop → Chain → Corner`
Visual: GP layers по semantic groups (Patches, Frame, Loops, Overlay)

---

## operators.py

Тонкие обёртки. Не содержат геометрической логики.

Операторы:
- `Analyze` — toggle debug mode (PatchGraph + GP)
- `Flow Debug` — print SolverGraph/SolvePlan to console
- `Scaffold Debug` — print ScaffoldMap to console
- `Solve Phase 1 Preview` — write scaffold to UV

Legacy операторы (disabled, будут пересобраны на ScaffoldMap):
- `UV Unwrap Faces`
- `Manual Dock`
- `Select Similar`
- `Stack Similar`

---

## ScaffoldMap как Future Platform

ScaffoldMap — не только runtime helper. Это база для:
- Кэширование результата (не пересчитывать если mesh не изменился)
- Manual override отдельных patches/chains
- Переориентация (повернуть patch на 90°)
- Переклассификация chain (FREE → H_FRAME)
- Manual dock (привязать patch к другому seam)
- Выпрямление chain (straighten FREE в H/V)

Для этого ScaffoldChainPlacement должен хранить override флаги:
```python
role_override: Optional[FrameRole] = None
direction_override: Optional[Vector] = None
pinned: bool = False  # don't recalculate
```

---

## Plan

### Phase 0 — Glossary & Docs ✓

### Phase 1 — Extract operators.py
- Перенести UI, операторы, settings из монолита
- Обновить __init__.py
- Монолит перестаёт быть runtime-хостом

### Phase 2 — Validation Layer
- `validate_scaffold_uv_transfer()` — диагностика scaffold vs UV
- Конкретные mismatches в console

### Phase 3 — Chain-First Frontier Builder
- Chain-level scoring
- Single frontier pool для всего quilt
- H/V snap, FREE interpolation
- ScaffoldMap как persistent result

### Phase 4 — Continuity + Debug
- ContinuationEdge как явная сущность
- Debug visualization continuations

### Phase 5 — Rebuild Operators
- UV Unwrap на ScaffoldMap pipeline
- Manual override hooks

---

## What NOT To Do

- Не использовать Hotspot_UV_v2_5_26.py как источник логики
- Не возвращаться к patch-first/loop-sequential placement
- Не строить отдельный boundary graph
- Не вводить constraint classes раньше времени
- Не перемещать Corner в solve.py
- Не хранить BMFace/BMEdge в PatchGraph
- Не наращивать scoring complexity до стабилизации placement
