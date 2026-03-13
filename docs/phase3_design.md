# Phase 3 Design — Chain-First Strongest-Frontier Builder
## v1.0

---

## Зачем этот документ

Phase 3 заменяет ядро solve.py — способ построения ScaffoldMap.
Текущий builder работает patch-by-patch, loop-sequential.
Новый builder работает chain-by-chain через единый frontier pool.

Это главная фаза рефакторинга. Ошибка здесь = переделка всего solve.

---

## Архитектурные требования

### R1: Chain-first strongest-frontier
Scaffold растёт chain за chain. На каждом шаге берётся strongest доступный
chain из общего пула всего quilt. Пул не различает chains одного patch
и chains через seam.

### R2: Patch-level addressability
ScaffoldMap хранит результат так, что каждый patch можно:
- прочитать отдельно (bbox, chains, corners)
- заменить отдельно (manual override)
- пересчитать отдельно (local re-solve)
- удалить и пересчитать зависимые

### R3: Incremental rebuild
Изменение одного patch не требует пересчёта всего quilt.
Зависимости между patches — через shared anchor points на seam.
Если anchor points не изменились — зависимые patches не пересчитываются.

### R4: ScaffoldMap = persistent snapshot
ScaffoldMap — не процесс, а результат. После построения он self-contained.
Для чтения ScaffoldMap не нужен ни BMesh, ни PatchGraph.

### R5: Validation-compatible
validate_scaffold_uv_transfer() из Phase 2 должна работать без изменений.

---

## Ключевое изменение: ScaffoldMap структура

### Текущая структура (patch-centric)

```
ScaffoldMap
  └── ScaffoldQuiltPlacement
        └── ScaffoldPatchPlacement      ← patch целиком
              └── ScaffoldChainPlacement ← chains внутри patch
```

Patch placement строится целиком одной функцией. Нет информации о том,
какие chains были anchor (от соседа) и какие были built (самостоятельно).

### Новая структура (chain-centric с patch envelope)

```
ScaffoldMap
  └── ScaffoldQuiltPlacement
        ├── build_order: list[ChainRef]           ← порядок frontier
        ├── anchor_registry: dict[vert_index → UV] ← shared points
        │
        └── patches: dict[patch_id → ScaffoldPatchEnvelope]
              ├── patch_id, loop_index
              ├── status: COMPLETE | PARTIAL | UNSUPPORTED
              ├── bbox_min, bbox_max
              ├── chains: dict[chain_index → ScaffoldChainPlacement]
              ├── corners: dict[corner_index → Vector]
              ├── pinned: bool                     ← manual lock
              └── dependency_patches: set[int]     ← кто дал anchors
```

### Что это даёт

**Patch envelope** — оболочка вокруг chains одного patch. Она не строит chains —
она только *группирует* уже построенные chains по принадлежности к patch.

**build_order** — полная запись порядка размещения chains. Позволяет:
- воспроизвести построение
- найти "откуда взялся этот chain"
- debug visualization (показать порядок стрелками)

**anchor_registry** — общий реестр уже размещённых вершин всего quilt.
Ключ: `vert_index`. Значение: `Vector(u, v)`.
Когда chain размещается, его endpoints регистрируются здесь.
Следующий chain ищет свои anchor points здесь.

**dependency_patches** — для каждого patch: множество patch_id, от которых
получены anchor points. Если parent patch пересчитывается — зависимый
помечается как dirty.

---

## Chain Frontier — детальный алгоритм

### Терминология

```
ChainRef = (patch_id, loop_index, chain_index)

AnchorPoint = уже размещённая вершина в anchor_registry
  - может прийти от chain того же patch (corner)
  - может прийти от chain другого patch (shared seam vertex)

FrontierCandidate = chain, у которого хотя бы одна вершина
  уже есть в anchor_registry
```

### Инициализация quilt

```python
def build_quilt_scaffold(graph, quilt_plan, final_scale):

    anchor_registry = {}      # vert_index → Vector(u, v)
    placed_chains = set()     # set of ChainRef
    patch_envelopes = {}      # patch_id → ScaffoldPatchEnvelope
    build_order = []          # list of ChainRef

    # 1. Выбираем seed chain — strongest chain root patch
    root_patch = graph.nodes[quilt_plan.root_patch_id]
    seed_chain_ref = choose_seed_chain(graph, root_patch)

    # 2. Размещаем seed chain от (0, 0)
    place_seed_chain(seed_chain_ref, anchor_registry, final_scale)
    placed_chains.add(seed_chain_ref)
    build_order.append(seed_chain_ref)

    # 3. Frontier loop
    while True:
        candidates = collect_frontier(graph, anchor_registry, placed_chains)
        if not candidates:
            break

        best = max(candidates, key=lambda c: c.score)
        if best.score < FRONTIER_THRESHOLD:
            break

        place_chain(best, anchor_registry, final_scale)
        placed_chains.add(best.chain_ref)
        build_order.append(best.chain_ref)

    # 4. Собираем patch envelopes из placed chains
    for chain_ref in build_order:
        patch_id = chain_ref[0]
        envelope = patch_envelopes.setdefault(patch_id, new_envelope(patch_id))
        envelope.chains[chain_ref[2]] = ... # chain placement data
        update_envelope_bbox(envelope)
        update_envelope_corners(envelope, graph)
        update_envelope_dependencies(envelope, anchor_registry)

    return ScaffoldQuiltPlacement(...)
```

### choose_seed_chain

Seed chain — первый chain, размещаемый от (0, 0).

Критерии (по приоритету):
1. H_FRAME или V_FRAME (не FREE)
2. Сосед с сильным семантическим контекстом (FLOOR.DOWN для стены)
3. Принадлежит OUTER loop (не HOLE)
4. Максимальная длина

Direction seed chain:
- Из basis patch (primary)
- H_FRAME → direction = sign(3D_direction · basis_u) * (1, 0)
- V_FRAME → direction = sign(3D_direction · basis_v) * (0, 1)

Start point: (0, 0).
End point: start + direction * chain_3d_length * final_scale.

Все вершины seed chain регистрируются в anchor_registry.

### collect_frontier

Для каждого ещё НЕ размещённого chain в quilt:
1. Проверить: есть ли у него вершины в anchor_registry?
2. Если нет → пропустить (chain без anchor не размещается)
3. Если да → создать FrontierCandidate

```python
@dataclass
class FrontierCandidate:
    chain_ref: ChainRef               # (patch_id, loop_index, chain_index)
    chain: BoundaryChain              # ссылка на chain в PatchGraph
    role: FrameRole
    anchor_start: Optional[Vector]    # UV из registry для start_vert
    anchor_end: Optional[Vector]      # UV из registry для end_vert
    known_endpoints: int              # 0, 1, или 2
    score: float
```

`known_endpoints`:
- 2 = оба конца chain уже в registry → максимальная уверенность
- 1 = один конец → chain строится от известного конца
- 0 = невозможно → не попадает в frontier

### Chain scoring

```python
def score_frontier_candidate(candidate, graph, anchor_registry):
    score = 0.0

    # Роль chain (primary factor)
    if candidate.role == FrameRole.H_FRAME:
        score += 1.0
    elif candidate.role == FrameRole.V_FRAME:
        score += 1.0
    elif candidate.role == FrameRole.FREE:
        score += 0.2

    # Количество известных endpoints
    if candidate.known_endpoints == 2:
        score += 0.8   # оба конца зафиксированы — максимальная точность
    elif candidate.known_endpoints == 1:
        score += 0.3

    # Сосед по seam с семантическим контекстом
    neighbor_semantic = get_neighbor_semantic(graph, candidate.chain)
    if neighbor_semantic in ('FLOOR.DOWN', 'FLOOR.UP'):
        score += 0.3
    elif neighbor_semantic.endswith('.SIDE'):
        score += 0.15

    # Chain того же patch, где уже есть размещённые chains
    # (patch "заполняется" постепенно)
    patch_placed_count = count_placed_chains_in_patch(candidate.chain_ref[0])
    if patch_placed_count > 0:
        score += 0.2  # continuity bonus

    return score
```

Score диапазон примерно 0.0 — 2.5. Threshold можно начать с 0.5.

Scoring **заморожен** до стабилизации placement. Менять веса —
только после визуальной валидации на production meshes.

### place_chain

Размещение chain зависит от role и known_endpoints:

```
H_FRAME + 2 endpoints:
    Интерполяция по горизонтали между anchor_start и anchor_end
    Все промежуточные точки: y = anchor_start.y (snap)

H_FRAME + 1 endpoint (start known):
    direction = (+1, 0) или (-1, 0) из basis
    Каждая следующая точка: prev + direction * edge_length * scale

V_FRAME + 2 endpoints:
    Интерполяция по вертикали между anchor_start и anchor_end
    Все промежуточные точки: x = anchor_start.x (snap)

V_FRAME + 1 endpoint:
    direction = (0, +1) или (0, -1) из basis
    Каждая следующая точка: prev + direction * edge_length * scale

FREE + 2 endpoints:
    Bezier/polyline guided interpolation между anchors

FREE + 1 endpoint:
    Guided extension от anchor, direction из 3D shape
    (текущая реализация _build_guided_free_chain_from_one_end)
```

После размещения: все вершины chain → anchor_registry.

---

## Patch Envelope — детали

### ScaffoldPatchEnvelope

```python
@dataclass
class ScaffoldPatchEnvelope:
    patch_id: int
    loop_index: int = -1

    # Status
    status: str = "EMPTY"
    # EMPTY    — ни один chain patch не размещён
    # PARTIAL  — часть chains размещена, часть нет
    # COMPLETE — все chains OUTER loop размещены
    # UNSUPPORTED — patch не может быть размещён (no_frame_loop, etc.)

    # Placed chains (subset of patch boundary chains)
    chains: dict[int, ScaffoldChainPlacement] = field(default_factory=dict)
    # key = chain_index

    # Corners — вычисляются из endpoints соседних chains
    corners: dict[int, Vector] = field(default_factory=dict)
    # key = corner_index, value = UV position

    # Bbox
    bbox_min: Vector = field(default_factory=lambda: Vector((0.0, 0.0)))
    bbox_max: Vector = field(default_factory=lambda: Vector((0.0, 0.0)))

    # Dependencies
    dependency_patches: set[int] = field(default_factory=set)
    # patch_ids, от которых получены anchor points

    # Manual override
    pinned: bool = False
    # True = не пересчитывать при rebuild

    # Free vertices (chains, не размещённые frontier)
    unplaced_chain_indices: list[int] = field(default_factory=list)
```

### Как envelope собирается

Envelope НЕ строит chains. Envelope **группирует** уже построенные chains.

```python
def build_envelope(patch_id, placed_chains_for_patch, graph, anchor_registry):
    envelope = ScaffoldPatchEnvelope(patch_id=patch_id)
    node = graph.nodes[patch_id]

    # Определяем loop_index (OUTER loop)
    for loop_index, loop in enumerate(node.boundary_loops):
        if loop.kind == LoopKind.OUTER:
            envelope.loop_index = loop_index
            break

    # Заполняем chains
    all_chain_indices = set(range(len(node.boundary_loops[envelope.loop_index].chains)))
    placed_indices = set()

    for chain_ref, chain_placement in placed_chains_for_patch:
        chain_index = chain_ref[2]
        envelope.chains[chain_index] = chain_placement
        placed_indices.add(chain_index)

    envelope.unplaced_chain_indices = sorted(all_chain_indices - placed_indices)

    # Status
    if not placed_indices:
        envelope.status = "EMPTY"
    elif placed_indices == all_chain_indices:
        envelope.status = "COMPLETE"
    else:
        envelope.status = "PARTIAL"

    # Corners — вычисляем из endpoints соседних chains
    boundary_loop = node.boundary_loops[envelope.loop_index]
    for corner_index, corner in enumerate(boundary_loop.corners):
        prev_chain = corner.prev_chain_index
        next_chain = corner.next_chain_index
        # Corner position = end of prev chain = start of next chain
        if prev_chain in envelope.chains:
            chain_points = envelope.chains[prev_chain].points
            if chain_points:
                envelope.corners[corner_index] = chain_points[-1][1].copy()
        elif next_chain in envelope.chains:
            chain_points = envelope.chains[next_chain].points
            if chain_points:
                envelope.corners[corner_index] = chain_points[0][1].copy()

    # Bbox
    update_bbox(envelope)

    # Dependencies
    for chain_placement in envelope.chains.values():
        for point_key, _ in chain_placement.points:
            if point_key.patch_id != patch_id:
                envelope.dependency_patches.add(point_key.patch_id)

    return envelope
```

### Patch-level операции (future Phase 5)

С envelope можно:

**Прочитать patch:**
```python
envelope = scaffold_map.quilts[0].patches[patch_id]
print(envelope.bbox_min, envelope.bbox_max)
print(envelope.status)  # COMPLETE / PARTIAL
```

**Manual reposition:**
```python
# Сдвинуть patch на (du, dv)
offset = Vector((0.1, 0.0))
for chain_placement in envelope.chains.values():
    for i, (key, point) in enumerate(chain_placement.points):
        chain_placement.points[i] = (key, point + offset)
envelope.pinned = True  # не пересчитывать
```

**Manual reclassify chain:**
```python
# Переклассифицировать FREE → H_FRAME
chain = envelope.chains[2]
chain.frame_role = FrameRole.H_FRAME
# Trigger local rebuild для этого chain
```

**Пересчитать один patch:**
```python
# Удалить все chains patch, кроме anchor (от соседей)
# Перезапустить frontier только для этого patch
# Anchor points от соседей = фиксированные входы
rebuild_patch_in_quilt(scaffold_map, quilt_index, patch_id, graph, final_scale)
```

**Пересчитать зависимые:**
```python
# Patch 0 изменился. Кто от него зависит?
dirty = set()
for pid, env in quilt.patches.items():
    if patch_id in env.dependency_patches and not env.pinned:
        dirty.add(pid)
# Пересчитать dirty patches (в порядке build_order)
```

---

## Anchor Registry — детали

### Проблема: vert_index не unique для SEAM_SELF

Один vert_index может иметь два разных UV на SEAM_SELF.
Текущий ScaffoldPointKey решает это через side-aware addressing.

Для anchor_registry нужна более простая модель:

```python
# Ключ = (patch_id, loop_index, loop_vert_index)
# Это однозначно определяет "сторону" вершины
AnchorKey = tuple[int, int, int]  # (patch_id, loop_index, loop_vert_index)

anchor_registry: dict[AnchorKey, Vector]
```

Но для поиска "есть ли anchor для этой вершины chain" нужен
обратный индекс по vert_index:

```python
# Быстрый поиск: "какие anchor keys содержат vert_index V?"
vert_to_anchors: dict[int, list[AnchorKey]]
```

Когда chain ищет свои anchors:
1. Взять start_vert_index chain
2. Найти все AnchorKey с этим vert_index
3. Если chain того же patch → использовать anchor с тем же patch_id
4. Если chain другого patch → использовать anchor с другим patch_id
   (это seam connection — UV наследуется от соседа)

### Регистрация anchors при place_chain

```python
def register_chain_anchors(chain_ref, chain_placement, boundary_loop, anchor_registry):
    patch_id, loop_index, chain_index = chain_ref
    chain = boundary_loop.chains[chain_index]

    for i, (point_key, uv_point) in enumerate(chain_placement.points):
        # Абсолютный loop vert index
        loop_vert_idx = (chain.start_loop_index + i) % len(boundary_loop.vert_indices)
        vert_index = boundary_loop.vert_indices[loop_vert_idx]

        anchor_key = (patch_id, loop_index, loop_vert_idx)
        anchor_registry[anchor_key] = uv_point.copy()

        # Также регистрируем для shared seam verts (другие patches)
        # Это позволяет chains других patches найти anchor
        register_seam_shared_anchor(vert_index, uv_point, anchor_registry)
```

---

## Что происходит с Conformal

После frontier builder:

1. Все размещённые chain points → pin в UV
2. Все не размещённые chains → free (не пинятся)
3. Все внутренние вершины patches → free
4. Один вызов Conformal unwrap per quilt
5. Conformal решает free vertices внутри каркаса из pinned
6. Unpin всё

```
Размещённые chains (pinned) = жёсткий каркас
Free chains + interior         = мягкая заливка Conformal
```

---

## Что удаляется из текущего solve.py

Текущие функции, которые заменяются:

```
УДАЛИТЬ:
- _build_root_patch_scaffold()
- _build_child_patch_scaffold()
- _build_patch_scaffold_frontier()    ← заменяется chain frontier
- _choose_root_loop()
- _choose_root_segment_index()
- _build_root_start_direction()
- _build_root_anchor_points()
- _score_root_segment()
- _score_child_patch_side()
- _build_target_anchor_points()
- _normalize_root_patch_orientation()
- _collect_patch_segments_for_loop()
- _derive_frame_spans_from_free_loop()
- _find_segment_list_index()
- _ordered_segment_source_points()
- _score_loop_frontier_candidate()
- PatchFrontierCandidate
- ScaffoldSegment (из model.py — заменяется FrontierCandidate)

ОСТАВИТЬ:
- SolverGraph + все scoring функции
- SolvePlan + plan_solve_phase1()
- ScaffoldPointKey, ScaffoldChainPlacement, ScaffoldMap
- _apply_patch_scaffold_to_uv()
- _resolve_scaffold_uv_targets()
- validate_scaffold_uv_transfer()
- execute_phase1_preview()
- _build_frame_chain_between_anchors()
- _build_frame_chain_from_one_end()
- _build_guided_free_chain_between_anchors()
- _build_guided_free_chain_from_one_end()
- все Bezier/interpolation helpers
- format_*_report() functions
```

---

## Порядок имплементации внутри Phase 3

### Step 3a: model.py additions
- Добавить `ScaffoldPatchEnvelope` dataclass
- Добавить `FrontierCandidate` dataclass
- Обновить `ScaffoldQuiltPlacement` — добавить `build_order`, `anchor_registry`
- Удалить `ScaffoldSegment` (заменён FrontierCandidate + envelope)

### Step 3b: solve.py — chain frontier core
- Написать `build_quilt_scaffold_chain_frontier()`
- Написать `choose_seed_chain()`
- Написать `collect_frontier()`
- Написать `score_frontier_candidate()`
- Написать `place_chain()` (dispatch по role + known_endpoints)

### Step 3c: solve.py — envelope builder
- Написать `build_envelope()`
- Написать `update_envelope_bbox()`
- Написать `update_envelope_corners()`
- Написать `update_envelope_dependencies()`

### Step 3d: solve.py — integration
- Обновить `build_root_scaffold_map()` — вызывать новый builder
- Обновить `execute_phase1_preview()` — если нужны изменения
- Удалить старые функции (список выше)
- Прогнать validation на Cylinder.007

### Step 3e: debug/reporting
- Обновить `format_root_scaffold_report()` — показывать build_order, envelope status
- Добавить frontier log — каждый шаг frontier с score

---

## Validation criteria для Phase 3

Phase 3 считается завершённой когда:

1. `validate_scaffold_uv_transfer()` показывает 0 mismatches на Cylinder.007
2. `Scaffold Debug` показывает build_order и envelope status
3. Каждый patch envelope имеет status COMPLETE или PARTIAL (не EMPTY для
   patches, участвующих в quilt)
4. UV в Blender визуально соответствует scaffold (ручная проверка)
5. Старые patch-sequential функции удалены
6. Не сломан: Analyze, Flow Debug

---

## Будущее использование (Phase 5+)

С этой архитектурой manual операторы становятся:

**Manual Dock** = "возьми patch X, найди его envelope в quilt,
удали его chains, пересчитай через другой seam, обнови зависимые"

**Straighten Chain** = "возьми chain Y в envelope, переклассифицируй
FREE → H_FRAME, пересчитай этот chain, обнови corners"

**Rotate Patch** = "возьми envelope, поверни все chain points на 90°,
пометь pinned, обнови anchor_registry"

**Re-root** = "возьми quilt, выбери новый seed chain,
перестрой весь quilt от нового начала"

Всё это — операции над ScaffoldMap, не над BMesh.
BMesh трогается только в финальном transfer step.
