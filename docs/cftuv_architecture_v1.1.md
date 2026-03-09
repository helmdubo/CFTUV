# CFTUV Architecture — Practical Refactoring Plan
## v1.1 — Updated with review feedback

## Принцип

Не переписываем с нуля. Выделяем из текущих 3200 строк **чёткие модули с ясными границами**,
сохраняя работающую логику. Главная цель — появление **PatchGraph** как центральной
структуры данных, через которую общаются все части системы.

**Debug visualization — полноправная часть системы, не опциональный слой.**
Сериализованная геометрия (mesh_verts, mesh_tris, vert_cos в loops/chains) хранится
в PatchGraph всегда, потому что визуальная верификация — основной инструмент
разработки и отладки на каждом этапе.

---

## Файловая структура

```
cftuv/
├── __init__.py          # bl_info + register/unregister (только wiring)
├── model.py             # Данные: PatchGraph, PatchNode, SeamEdge, BoundaryLoop, Chain, Enums
├── analysis.py          # BMesh → PatchGraph (чтение геометрии, классификация)
├── solve.py             # PatchGraph → UV операции (unwrap, orient, scale, dock)
├── debug.py             # PatchGraph → Grease Pencil визуализация
├── operators.py         # Blender операторы + панель + settings
└── constants.py         # WORLD_UP, пороги, GP_DEBUG_PREFIX
```

6 файлов. Каждый < 600 строк. Один человек может держать в голове.

---

## Поток данных

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│   operators  │────>│   analysis   │────>│  PatchGraph  │
│  (user input)│     │ (BMesh read) │     │ (центральный │
└──────────────┘     └──────────────┘     │    IR)       │
                                          └──────┬───────┘
                                                 │
                                    ┌────────────┼────────────┐
                                    ▼            ▼            ▼
                              ┌──────────┐ ┌──────────┐ ┌──────────┐
                              │  solve   │ │  debug   │ │ operators│
                              │(UV write)│ │(GP draw) │ │(report)  │
                              └──────────┘ └──────────┘ └──────────┘
```

Правило: **analysis.py только читает BMesh и пишет в PatchGraph**.
**solve.py только читает PatchGraph и пишет UV**. Они никогда не вызывают друг друга.

**Исключение (формализованное):** analysis.py содержит одну изолированную функцию
`_classify_loops_outer_hole()`, которая делает временный conformal unwrap для определения
OUTER/HOLE через UV nesting. Это единственный side effect в analysis, он инкапсулирован
в одном helper-е и явно помечен. Долгосрочно — исследовать замену на 3D winding order +
normal direction, чтобы устранить side effect полностью.

---

## model.py — Центральная структура данных

Чистые dataclass-ы. Никаких ссылок на BMFace/BMEdge (не переживают mode switch).
Только индексы, координаты, скаляры.

### Enum-ы (dispatch keys)

```python
from enum import Enum

class PatchType(str, Enum):
    """Dispatch key для выбора UV стратегии.
    Новый тип добавляется ТОЛЬКО когда появляется новая стратегия unwrap."""
    WALL  = 'WALL'      # V=WORLD_UP projection
    FLOOR = 'FLOOR'     # normal projection, orient by dominant edges
    SLOPE = 'SLOPE'     # hybrid
    # Будущие кандидаты (добавлять при реальной потребности):
    # CYLINDER = 'CYLINDER'  # cylindrical unwrap
    # STRIP    = 'STRIP'     # straighten along long axis
    # CAP      = 'CAP'       # unique unwrap, no tiling

class LoopKind(str, Enum):
    OUTER = 'OUTER'
    HOLE  = 'HOLE'

class FrameRole(str, Enum):
    H_FRAME = 'H_FRAME'   # горизонтальная линия (low V variance) → straighten по V
    V_FRAME = 'V_FRAME'   # вертикальная линия (low U variance) → straighten по U
    FREE    = 'FREE'       # диагональ / irregular
```

`str, Enum` — для обратной совместимости со строковыми сравнениями при поэтапной миграции.

### Структуры данных

```python
from dataclasses import dataclass, field
from mathutils import Vector
from typing import Optional

# ─── Boundary Chain ───────────────────────────────────────────

@dataclass
class BoundaryChain:
    """Сегмент boundary loop с одним соседом."""
    vert_cos: list[Vector]          # координаты вершин (3D, local space)
    edge_indices: list[int]         # индексы BMEdge
    neighbor_patch_id: int          # id соседнего патча, или спец-значение:
                                    #   NB_MESH_BORDER (-1), NB_SEAM_SELF (-2)
    is_closed: bool = False
    frame_role: FrameRole = FrameRole.FREE

# ─── Boundary Loop ────────────────────────────────────────────

@dataclass
class BoundaryLoop:
    """Замкнутый контур boundary edges одного патча."""
    vert_cos: list[Vector]          # координаты вершин по порядку обхода
    edge_indices: list[int]
    kind: LoopKind = LoopKind.OUTER
    depth: int = 0                  # nesting depth (0 = outermost)
    chains: list[BoundaryChain] = field(default_factory=list)

    # Сегменты для frame classification
    segments: list[dict] = field(default_factory=list)
    # Каждый segment: {'vert_cos': [...], 'frame_role': FrameRole, 'loop_kind': LoopKind}

# ─── Patch Node ───────────────────────────────────────────────

@dataclass
class PatchNode:
    """Один 3D patch — flood fill faces внутри seam/sharp контура."""
    patch_id: int
    face_indices: list[int]         # индексы BMFace

    # Геометрия
    centroid: Vector = field(default_factory=lambda: Vector((0, 0, 0)))
    normal: Vector = field(default_factory=lambda: Vector((0, 0, 1)))
    area: float = 0.0
    perimeter: float = 0.0

    # Классификация (dispatch key для UV стратегии)
    patch_type: PatchType = PatchType.WALL

    # Локальный базис (для UV проекции и frame classification)
    basis_u: Vector = field(default_factory=lambda: Vector((1, 0, 0)))  # tangent / U
    basis_v: Vector = field(default_factory=lambda: Vector((0, 0, 1)))  # bitangent / V

    # Топология границ
    boundary_loops: list[BoundaryLoop] = field(default_factory=list)

    # Сериализованная геометрия (для debug visualization — всегда заполняется)
    mesh_verts: list[Vector] = field(default_factory=list)
    mesh_tris: list[tuple[int, int, int]] = field(default_factory=list)

# ─── Seam Edge (связь между патчами) ─────────────────────────

@dataclass
class SeamEdge:
    """Связь между двумя соседними патчами через общий шов."""
    patch_a_id: int
    patch_b_id: int
    shared_length: float = 0.0
    shared_vert_indices: list[int] = field(default_factory=list)
    longest_edge_verts: tuple[int, int] = (0, 0)
    longest_edge_length: float = 0.0

# ─── UV Settings (замена глобальных переменных) ───────────────

@dataclass
class UVSettings:
    """Иммутабельный snapshot настроек. Передаётся параметром, не через globals."""
    texel_density: int = 512
    texture_size: int = 2048
    uv_scale: float = 1.0
    uv_range_limit: float = 16.0

    @property
    def final_scale(self) -> float:
        return (self.texel_density / self.texture_size) * self.uv_scale

    @staticmethod
    def from_blender_settings(settings) -> 'UVSettings':
        """Создаёт UVSettings из HOTSPOTUV_Settings PropertyGroup."""
        return UVSettings(
            texel_density=int(settings.target_texel_density),
            texture_size=int(settings.texture_size),
            uv_scale=float(settings.uv_scale),
            uv_range_limit=float(settings.uv_range_limit),
        )

# ─── PatchGraph ───────────────────────────────────────────────

@dataclass
class PatchGraph:
    """Центральная структура: граф патчей + их связей.

    Это единственный объект, который передаётся между analysis → solve → debug.
    Не содержит ссылок на BMesh. Переживает mode switch.
    """
    nodes: dict[int, PatchNode] = field(default_factory=dict)
    edges: dict[tuple[int, int], SeamEdge] = field(default_factory=dict)

    # Быстрый lookup
    face_to_patch: dict[int, int] = field(default_factory=dict)
    _adjacency: dict[int, set[int]] = field(default_factory=dict)

    # ─── Mutation (только при построении) ──────────────────

    def add_node(self, node: PatchNode):
        self.nodes[node.patch_id] = node
        if node.patch_id not in self._adjacency:
            self._adjacency[node.patch_id] = set()
        for fi in node.face_indices:
            self.face_to_patch[fi] = node.patch_id

    def add_edge(self, seam: SeamEdge):
        key = (min(seam.patch_a_id, seam.patch_b_id),
               max(seam.patch_a_id, seam.patch_b_id))
        self.edges[key] = seam
        self._adjacency.setdefault(seam.patch_a_id, set()).add(seam.patch_b_id)
        self._adjacency.setdefault(seam.patch_b_id, set()).add(seam.patch_a_id)

    # ─── Query API ─────────────────────────────────────────

    def get_neighbors(self, patch_id: int) -> set[int]:
        """O(1) lookup соседей через adjacency list."""
        return self._adjacency.get(patch_id, set())

    def get_seam(self, patch_a: int, patch_b: int) -> Optional[SeamEdge]:
        key = (min(patch_a, patch_b), max(patch_a, patch_b))
        return self.edges.get(key)

    def traverse_bfs(self, root_id: int) -> list[list[int]]:
        """BFS от root. Возвращает уровни: [[root], [level1], [level2], ...]."""
        visited = {root_id}
        levels = [[root_id]]
        current = [root_id]
        while current:
            next_level = []
            for pid in current:
                for nb in self.get_neighbors(pid):
                    if nb not in visited:
                        visited.add(nb)
                        next_level.append(nb)
            if next_level:
                levels.append(next_level)
            current = next_level
        return levels

    def find_root(self, patch_ids: set[int] = None, strategy: str = 'MAX_AREA') -> int:
        candidates = patch_ids or set(self.nodes.keys())
        if strategy == 'MAX_AREA':
            return max(candidates, key=lambda pid: self.nodes[pid].area)
        else:
            return min(candidates, key=lambda pid: self.nodes[pid].area)

    def connected_components(self) -> list[set[int]]:
        visited = set()
        components = []
        for pid in self.nodes:
            if pid in visited:
                continue
            comp = set()
            stack = [pid]
            while stack:
                curr = stack.pop()
                if curr in visited:
                    continue
                visited.add(curr)
                comp.add(curr)
                stack.extend(nb for nb in self.get_neighbors(curr)
                            if nb not in visited)
            components.append(comp)
        return components
```

### Почему так

**Индексы вместо ссылок.** `face_indices: list[int]` вместо `faces: list[BMFace]`.
BMFace/BMEdge инвалидируются при `bmesh.update_edit_mesh()` + `bmesh.from_edit_mesh()`.
Индексы стабильны (unwrap не меняет топологию).

**Enum-ы для dispatch keys.** `PatchType`, `LoopKind`, `FrameRole` — защита от
опечаток, автокомплит в IDE, и `str, Enum` для обратной совместимости.

**UVSettings вместо globals.** Иммутабельный snapshot, передаётся параметром.
Нет глобального состояния, можно тестировать с разными настройками.

**Adjacency list.** `_adjacency: dict[int, set[int]]` заполняется при `add_edge()`.
`get_neighbors()` — O(1) вместо O(|edges|). BFS/traversal — центральная операция solve.

**Debug geometry — часть модели, не optional.** `mesh_verts`, `mesh_tris`, `vert_cos`
в loops/chains заполняются всегда. Debug visualization — основной инструмент
верификации, он нужен на каждом этапе разработки.

---

## analysis.py — Построение PatchGraph из BMesh

Единственный модуль, который **читает BMesh**. Всё остальное работает с PatchGraph.

```
analysis.py содержит:

build_patch_graph(bm, face_indices, obj=None) -> PatchGraph
    Главная точка входа. Единственная публичная функция.

_flood_fill_patches(bm, face_indices) -> list[list[int]]
    Текущий find_seam_patches(), возвращает индексы.

_classify_patch(bm, face_indices) -> (PatchType, Vector, float, float)
    Текущий analyze_island_properties() → (type, normal, area, perimeter).

_build_patch_basis(bm, face_indices, patch_type, normal) -> (Vector, Vector)
    Текущий build_patch_basis() + find_island_up() + calc_surface_basis().

_find_boundary_loops(bm, face_indices) -> list[BoundaryLoop]
    Текущие find_patch_boundary_edges() + build_ordered_boundary_loops().

_classify_loops_outer_hole(bm, patch_faces, obj) -> None
    ⚠ ЕДИНСТВЕННАЯ ФУНКЦИЯ С SIDE EFFECTS В ANALYSIS.
    Делает временный conformal unwrap для OUTER/HOLE detection.
    Мутирует loop.kind и loop.depth.
    Изолирована в отдельный helper. Временный UV layer создаётся и удаляется внутри.
    TODO: исследовать замену на 3D winding order + normal direction.

_split_loops_into_chains(loops, face_to_patch, patch_id) -> None
    Текущий split_loop_into_chains_by_neighbor().

_classify_frame_roles(loops, basis_u, basis_v) -> None
    Текущие find_loop_corners() + split_loop_into_segments() +
    classify_segment_frame_role().

_build_seam_edges(nodes, face_to_patch, bm) -> dict
    Текущий build_edge_based_links().

_serialize_patch_geometry(bm, face_indices) -> (list[Vector], list[tuple])
    Сериализация вершин + треугольников для debug visualization.

_compute_centroid(bm, face_indices) -> Vector

get_expanded_islands(bm, initial_faces) -> list[dict]
    Текущая функция, используется solve.py для two-pass unwrap (core/full logic).
    Публичная — вызывается из solve и operators (SelectSimilar, StackSimilar).
```

### Ключевой контракт

```python
def build_patch_graph(bm, face_indices: list[int], obj=None) -> PatchGraph:
    """Строит полный PatchGraph из faces.

    Args:
        bm: BMesh в edit mode
        face_indices: индексы faces для анализа
        obj: Blender object (нужен для _classify_loops_outer_hole)

    Returns:
        PatchGraph со всеми nodes, edges, boundary loops, chains, frame roles.
    """
    graph = PatchGraph()

    # 1. Flood fill → patches
    patches_raw = _flood_fill_patches(bm, face_indices)

    # 2. Per-patch analysis
    for pid, pf_indices in enumerate(patches_raw):
        ptype, normal, area, perimeter = _classify_patch(bm, pf_indices)
        basis_u, basis_v = _build_patch_basis(bm, pf_indices, ptype, normal)
        loops = _find_boundary_loops(bm, pf_indices)
        centroid = _compute_centroid(bm, pf_indices)
        mesh_verts, mesh_tris = _serialize_patch_geometry(bm, pf_indices)

        node = PatchNode(
            patch_id=pid, face_indices=pf_indices,
            centroid=centroid, normal=normal, area=area, perimeter=perimeter,
            patch_type=ptype, basis_u=basis_u, basis_v=basis_v,
            boundary_loops=loops,
            mesh_verts=mesh_verts, mesh_tris=mesh_tris,
        )
        graph.add_node(node)

    # 3. OUTER/HOLE classification (⚠ side effect — isolated)
    _classify_loops_outer_hole(bm, graph, obj)

    # 4. Chains (нужен face_to_patch)
    for node in graph.nodes.values():
        _split_loops_into_chains(node.boundary_loops, graph.face_to_patch, node.patch_id)
        _classify_frame_roles(node.boundary_loops, node.basis_u, node.basis_v)

    # 5. Seam edges между патчами
    for seam in _build_seam_edges(graph.nodes, graph.face_to_patch, bm):
        graph.add_edge(seam)

    return graph
```

---

## solve.py — UV операции по PatchGraph

Единственный модуль, который **пишет UV**. Читает PatchGraph + BMesh для UV access.

```
solve.py содержит:

unwrap_faces(bm, graph, core_face_indices, settings: UVSettings, context) -> str
    Главный pipeline для UnwrapFaces.
    Two-Pass: unwrap cores → orient/scale/pin → unwrap full → dock → align.
    core_face_indices — faces выделенные пользователем (ядро).
    full faces вычисляются через get_expanded_islands().

dock_islands(bm, selected_edge_indices, settings: UVSettings, context) -> str
    Pipeline для ManualDock.
    BFS layered docking по connected components.

stack_similar(bm, face_indices, settings: UVSettings) -> str
    Pipeline для StackSimilar.

# ─── Внутренние функции ───

_orient_scale_island(uv_layer, face_indices, bm, node: PatchNode, settings)
_compute_rigid_transform(anchor_uvs, target_uvs) -> (angle, anchor_c, target_c)
_dock_island(uv_layer, target_faces, anchor_uvs, target_uvs, ...)
_align_connected(islands_data, links, uv_layer)
_align_split_seams_within(uv_layer, face_indices, bm)
_align_split_seams_between(islands_data, links, uv_layer)
_normalize_uvs(bm, uv_layer, settings)
_weld_island_uvs(uv_layer, face_indices, bm, distance)
```

### Two-pass pipeline (core/full)

Core/full разделение — свойство конкретного solve pipeline, не PatchGraph.
`core_face_indices` передаётся как параметр, `full_faces` вычисляется из
`get_expanded_islands()` внутри solve.

```python
def unwrap_faces(bm, graph: PatchGraph, core_face_indices: list[int],
                 settings: UVSettings, context) -> str:
    """Two-Pass Unwrap.

    Args:
        bm: BMesh в edit mode
        graph: PatchGraph (для basis, type, topology info)
        core_face_indices: faces выделенные пользователем
        settings: UV настройки (не globals)
        context: Blender context (для bpy.ops.uv.unwrap)
    """
    # 1. Expand core → full (через get_expanded_islands)
    # 2. Unwrap cores (conformal)
    # 3. Orient / scale / pin cores (используя graph.nodes[pid].basis_u/v)
    # 4. Unwrap full (conformal with pinned cores)
    # 5. Clean pins, restore seams
    # 6. Dock + align connected islands
    # 7. Align split seams
    # 8. Normalize UVs
    ...
```

---

## debug.py — Визуализация PatchGraph

Принимает PatchGraph, рисует Grease Pencil. Не знает про BMesh.

```
debug.py содержит:

create_visualization(graph: PatchGraph, source_obj, settings_dict) -> bpy.types.Object
    Главная точка входа.

clear_visualization(source_obj)
    Удаляет GP + debug mesh.

_get_or_create_gp_object(source_obj)
_ensure_gp_layer(gp_data, layer_name, color)
_add_gp_stroke(frame, points, mat_idx, line_width)
_create_patch_mesh(graph, source_obj)
_apply_layer_visibility(gp_data, settings)
```

### Debug читает PatchGraph напрямую

```python
def create_visualization(graph: PatchGraph, source_obj, settings_dict=None):
    gp_obj = _get_or_create_gp_object(source_obj)
    gp_data = gp_obj.data

    for pid, node in graph.nodes.items():
        _draw_patch_fill(gp_data, node)
        _draw_basis_axes(gp_data, node)
        for loop in node.boundary_loops:
            _draw_frame_segments(gp_data, loop, node)
            _draw_chains(gp_data, loop)

    if settings_dict:
        _apply_layer_visibility(gp_data, settings_dict)

    return gp_obj
```

---

## operators.py — Тонкая обёртка

Операторы только: валидируют контекст → вызывают analysis/solve/debug → репортят.

```python
class HOTSPOTUV_OT_UnwrapFaces(bpy.types.Operator):
    # ...
    def execute(self, context):
        settings = UVSettings.from_blender_settings(context.scene.hotspotuv_settings)

        bm = bmesh.from_edit_mesh(context.edit_object.data)
        bm.faces.ensure_lookup_table()
        sel = [f.index for f in bm.faces if f.select]

        report = solve.unwrap_faces(bm, sel, context, settings)

        bmesh.update_edit_mesh(context.edit_object.data)
        self.report({"INFO"}, report)
        return {"FINISHED"}
```

Оператор — 10 строк. Вся логика в `solve.unwrap_faces()`.

---

## constants.py — Конфигурация

```python
from mathutils import Vector

WORLD_UP = Vector((0, 0, 1))

# Patch classification thresholds
FLOOR_THRESHOLD = 0.9     # abs(dot(normal, WORLD_UP)) > this → FLOOR
WALL_THRESHOLD  = 0.3     # abs(dot(normal, WORLD_UP)) < this → WALL
                          # between → SLOPE

# Boundary chain neighbor types
NB_MESH_BORDER = -1
NB_SEAM_SELF   = -2

# Frame classification
FRAME_ALIGNMENT_THRESHOLD = 0.08

# Corner detection
CORNER_ANGLE_THRESHOLD_DEG = 30.0

# Debug
GP_DEBUG_PREFIX = "CFTUV_Debug_"
```

---

## План рефакторинга — порядок действий

### Предварительно: cleanup (до любых фаз)

- [ ] Удалить мёртвый `align_connected_islands` (cluster-merging версию, строки 2341-2404).
      Zero risk, 70 строк мёртвого кода.
- [ ] Перенести проект в git (отдельная папка `cftuv/` с `__init__.py`).
      Каждая фаза = отдельный коммит/PR.

### Фаза 1: model.py + constants.py (не ломает ничего)

1. Создать `cftuv/model.py` — Enum-ы, dataclass-ы, PatchGraph с adjacency list.
2. Создать `cftuv/constants.py` — вынести все magic numbers.
3. Создать `cftuv/model.py: UVSettings` — замена глобальных переменных.
4. Текущий код продолжает работать. Новые модули пока не подключены.

**Результат:** Формализованная модель данных + enum-ы + settings object существуют.

### Фаза 2: debug.py + UVSettings wiring (минимальный risk)

1. Перенести все GP-функции в `cftuv/debug.py`.
2. Переписать `create_debug_visualization()` чтобы принимала PatchGraph.
3. Написать адаптер `_patch_results_to_graph()` — конвертирует текущие dict-результаты
   в PatchGraph (временный мост).
4. Заменить `_apply_settings_to_globals()` → `UVSettings.from_blender_settings()`.
   Передавать settings параметром в solve/debug.
5. Убрать глобальные `TARGET_TEXEL_DENSITY`, `TEXTURE_SIZE` и т.д.

**Результат:** Debug отделён, работает через PatchGraph. Globals убраны.

### Фаза 3: analysis.py (основной рефакторинг)

1. Перенести: `find_seam_patches`, `analyze_island_properties`, `build_patch_basis`,
   `find_island_up`, `calc_surface_basis`, `find_patch_boundary_edges`,
   `build_ordered_boundary_loops`, `classify_loops_via_uv`,
   `split_loop_into_chains_by_neighbor`, `find_loop_corners`,
   `split_loop_into_segments`, `classify_segment_frame_role`,
   `build_edge_based_links`, `get_expanded_islands`.
2. Обернуть в `build_patch_graph()` — единственная публичная функция
   (плюс `get_expanded_islands()` для solve).
3. Изолировать `_classify_loops_outer_hole()` как явный side-effect helper.
4. Адаптер из Фазы 2 больше не нужен — debug читает PatchGraph напрямую.

**Результат:** Вся аналитика в одном месте. PatchGraph — единственный выход.

### Фаза 4: solve.py + operators.py (финальный шаг)

1. Перенести: `orient_scale_and_position_island`, `compute_best_fit_transform`,
   `dock_island_to_anchor`, `align_connected_islands` (root-anchored),
   `align_split_seams_*`, `normalize_uvs_to_origin`, `weld_island_uvs`,
   `dock_chain_bfs_layered`, `build_island_graph`, `dock_all_chains`.
2. `unwrap_faces()` и `dock_islands()` — публичные entry points.
3. Операторы → тонкие обёртки в `operators.py`.

**Результат:** Полное разделение. Операторы — обёртки, логика в solve.

### Фаза 5: Очистка

1. Заменить safety counters на proper loop termination с валидацией.
2. Оптимизация: кэширование в analysis, убрать лишние BMesh rebuild в solve.
3. Non-manifold detection и user warning.

---

## Что НЕ нужно делать

- **Не добавлять multi-axis semantic profile.** WALL/FLOOR/SLOPE — dispatch keys.
  Новый тип добавляется когда появляется новая UV стратегия. Не раньше.

- **Не строить отдельный Boundary Graph.** Chains — подструктура PatchNode.

- **Не делать constraint classes.** Правила стыковки — код в solve.py.
  Формализовать когда правил станет > 20 и они начнут конфликтовать.

- **Не разбивать solve.py на подмодули.** Пока помещается в 600 строк — один файл.

- **Не делать debug geometry optional/lazy.** Debug — полноправная часть системы.

---

## Валидация архитектуры: чеклист

После рефакторинга каждое утверждение должно быть истинным:

- [ ] `model.py` не импортирует `bpy` и `bmesh` (только `mathutils`)
- [ ] `analysis.py` не пишет UV (кроме изолированной `_classify_loops_outer_hole`)
- [ ] `solve.py` не делает flood fill и не классифицирует патчи
- [ ] `debug.py` не читает BMesh напрямую
- [ ] Операторы не содержат геометрической логики (> 5 строк math)
- [ ] PatchGraph — единственный способ передать данные из analysis в solve/debug
- [ ] Нет глобальных мутабельных переменных (UVSettings передаётся параметром)
- [ ] Нет дублирования функций
- [ ] Все строковые типы заменены на Enum-ы
- [ ] Debug visualization работает на каждой фазе рефакторинга
